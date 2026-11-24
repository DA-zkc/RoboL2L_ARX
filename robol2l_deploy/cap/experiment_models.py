"""Persistent, local experiment model profiles. Secrets never enter run artifacts."""
import copy
import json
import os
import re
import tempfile
from pathlib import Path
from urllib.parse import urlparse
from uuid import uuid4


class ModelConfigurationError(RuntimeError):
    """Safe configuration code, never a provider body or credential value."""


class ModelRequestError(RuntimeError):
    """Allowlisted diagnostics only: never store arbitrary provider text."""
    def __init__(self, status, category, request_bytes):
        super().__init__('model_request_failed_http_' + str(status))
        self.diagnostics = dict(http_status=status, category=category,
                                request_bytes=request_bytes,
                                retryable=status in (408, 429, 500, 502, 503, 504))


def response_category(response):
    # Inspect the response in memory, but persist only a known classification.
    try:
        error = response.json().get('error', {})
        text = json.dumps(error).lower() if isinstance(error, (dict, str)) else ''
    except (ValueError, AttributeError, TypeError):
        text = ''
    if response.status_code == 413 or any(s in text for s in (
            'context_length_exceeded', 'maximum context length', 'prompt is too long',
            'too many tokens', 'request too large', 'request_too_large', 'input is too long')):
        return 'context_limit'
    if response.status_code == 429: return 'rate_limit'
    if response.status_code >= 500: return 'provider_unavailable'
    return 'invalid_request' if response.status_code == 400 else 'http_error'


def validate_api_key(key):
    if not key:
        raise ModelConfigurationError('model_configuration_error_key_missing')
    if not isinstance(key, str) or not key.isascii() or any(ord(c) <= 32 or ord(c) == 127 for c in key):
        raise ModelConfigurationError('model_configuration_error_key_format')


class ModelProfiles:
    def __init__(self, path, defaults=None):
        self.path = Path(path)
        self.defaults = defaults or (lambda: {})

    def read(self):
        if self.path.exists():
            value = json.loads(self.path.read_text())
        else:
            cfg = self.defaults()
            value = {'mode': 'codex', 'codex_model': '', 'reasoning': 'medium',
                'provider': 'openai', 'profiles': {
                    'openai': {'base_url': cfg.get('base_url') or 'https://api.openai.com/v1',
                               'model': cfg.get('model', ''), 'api_key': '',
                               'api_key_env': cfg.get('api_key_env', 'ROBOT_MODEL_API_KEY')},
                    'claude': {'base_url': 'https://api.anthropic.com/v1', 'model': '',
                               'api_key': '', 'api_key_env': 'ANTHROPIC_API_KEY'}}}
        # Migrate the original two-provider form without renaming IDs or losing keys.
        for key, profile in value['profiles'].items():
            profile.setdefault('provider', key if key in ('openai', 'claude') else 'openai')
            profile.setdefault('name', 'OpenAI 默认 / Default' if key == 'openai' else 'Claude 默认 / Default')
            profile.setdefault('reasoning', value['reasoning'])
        value.setdefault('profile_id', value.get('provider', 'openai'))
        value['schema_version'] = 2
        return value

    def public(self):
        value = self.read()
        value['api_key_env_editable'] = True
        for profile in value['profiles'].values():
            profile['key_saved'] = bool(profile.pop('api_key', ''))
            profile['environment_key_loaded'] = bool(os.environ.get(profile.get('api_key_env', '')))
        return value

    def save(self, body):
        value = self.read()
        if set(body) - {'mode', 'provider', 'codex_model', 'reasoning', 'profile', 'clear_key',
                        'profile_id', 'profile_name', 'create_profile', 'profile_reasoning'}:
            raise ValueError('invalid_model_settings')
        for name in ('mode', 'codex_model', 'reasoning'):
            if name in body:
                value[name] = body[name]
        if value['mode'] not in ('codex', 'api', 'fixture'):
            raise ValueError('invalid_model_provider')
        if value['reasoning'] not in ('none', 'low', 'medium', 'high', 'xhigh') or not isinstance(value['codex_model'], str):
            raise ValueError('invalid_model_settings')
        if body.get('create_profile') is True:
            provider = body.get('provider', 'openai')
            key = uuid4().hex
            value['profiles'][key] = dict(name='', provider=provider,
                base_url='https://api.anthropic.com/v1' if provider == 'claude' else 'https://api.openai.com/v1',
                model='', api_key='', api_key_env='', reasoning=value['reasoning'])
        else:
            # Compatibility with the old provider selector; named UI always supplies an ID.
            key = body.get('profile_id', body.get('provider', value['profile_id']))
            if key not in value['profiles']: raise ValueError('model_profile_not_found')
        profile = value['profiles'][key]
        provider = body.get('provider', profile['provider'])
        if provider not in ('openai', 'claude'): raise ValueError('invalid_model_provider')
        if profile['provider'] != provider and (profile.get('api_key') or profile.get('api_key_env')):
            raise ValueError('create_new_profile_to_change_provider_with_saved_credentials')
        profile['provider'] = provider
        name = body.get('profile_name', profile['name'])
        if not isinstance(name, str) or not name.strip() or len(name.strip()) > 100:
            raise ValueError('profile_name_required_max_100')
        if any(p['name'].casefold() == name.strip().casefold() for k, p in value['profiles'].items() if k != key):
            raise ValueError('profile_name_already_exists')
        profile['name'] = name.strip()
        # `profile_reasoning` sets only this API profile; `reasoning` alone keeps its
        # original meaning (the Codex/global value, also copied to the profile).
        if 'profile_reasoning' in body:
            if body['profile_reasoning'] not in ('none', 'low', 'medium', 'high', 'xhigh'):
                raise ValueError('invalid_model_settings')
            profile['reasoning'] = body['profile_reasoning']
        elif 'reasoning' in body: profile['reasoning'] = body['reasoning']
        value.update(profile_id=key, provider=provider)
        supplied = body.get('profile', {})
        if not isinstance(supplied, dict) or set(supplied) - {'base_url', 'model', 'api_key', 'api_key_env'}:
            raise ValueError('invalid_model_profile')
        for field, item in supplied.items():  # not `key`: that still names the profile
            if not isinstance(item, str):
                raise ValueError('invalid_model_profile')
            if field != 'api_key' or item:
                profile[field] = item.strip()
                if field == 'api_key' and body.get('clear_key') is not True:
                    validate_api_key(profile[field])
        key_env = profile.get('api_key_env', '')
        if key_env and not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', key_env):
            raise ValueError('invalid_api_key_env')
        if body.get('clear_key') is True:
            profile['api_key'] = ''
        url = urlparse(profile['base_url'])
        if url.scheme not in ('https', 'http') or not url.hostname or url.username or url.password or url.query or url.fragment:
            raise ValueError('invalid_model_endpoint')
        return self._write(value)

    def _write(self, value):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary = tempfile.mkstemp(dir=self.path.parent, prefix='.experiment-model-')
        try:
            with os.fdopen(fd, 'w') as stream:
                json.dump(value, stream, ensure_ascii=False, indent=2)
                stream.flush(); os.fsync(stream.fileno())
            os.replace(temporary, self.path)
        finally:
            if os.path.exists(temporary): os.unlink(temporary)
        return self.public()

    def delete(self, body):
        """Remove one named profile. The active profile must be reassigned explicitly."""
        if set(body) - {'delete_profile_id', 'profile_id', 'confirmed'} or body.get('confirmed') is not True:
            raise ValueError('explicit_profile_delete_confirmation_required')
        value = self.read()
        target = body.get('delete_profile_id')
        if target not in value['profiles']: raise ValueError('model_profile_not_found')
        if len(value['profiles']) < 2: raise ValueError('cannot_delete_last_model_profile')
        # Deleting the active profile requires naming the one that replaces it.
        selected = body.get('profile_id', value['profile_id'])
        if selected == target: raise ValueError('select_another_profile_before_deleting_the_active_one')
        if selected not in value['profiles']: raise ValueError('model_profile_not_found')
        del value['profiles'][target]
        value.update(profile_id=selected, provider=value['profiles'][selected]['provider'])
        return self._write(value)

    def model(self, synthetic):
        cfg = self.read()
        if cfg['mode'] == 'fixture':
            if not synthetic: raise ValueError('fixture_requires_mock_runtime')
            from .model import FeedbackFixture
            return FeedbackFixture()
        if cfg['mode'] == 'codex':
            from .codex_model import CodexModel, configuration
            from .experiment import DECISION_FIELDS
            settings = configuration()
            if cfg['codex_model']: settings['model'] = cfg['codex_model']
            settings['reasoning'] = cfg['reasoning']
            # Declare the experiment record fields, or the output schema drops them.
            return CodexModel(**settings, extra_fields=DECISION_FIELDS)
        return ExperimentAPIModel(cfg)


class ExperimentAPIModel:
    label = 'experiment_api'

    def __init__(self, cfg):
        key = cfg.get('profile_id', cfg['provider'])
        self.profile = copy.deepcopy(cfg['profiles'][key])
        self.provider = self.profile.get('provider', cfg['provider'])
        self.key = self.profile.pop('api_key', '') or os.environ.get(self.profile.get('api_key_env', ''), '')
        validate_api_key(self.key)
        if not self.profile['model']: raise ModelConfigurationError('model_configuration_error_model_missing')
        self.reasoning = self.profile.get('reasoning', cfg['reasoning'])
        self.timeout_s = request_timeout()
        self.settings = dict(provider=self.provider, model=self.profile['model'],
                             profile_id=key, profile_name=self.profile.get('name', key),
                             endpoint_host=urlparse(self.profile['base_url']).hostname)

    def decide(self, messages):
        import requests
        base = self.profile['base_url'].rstrip('/')
        headers = {'Content-Type': 'application/json'}
        if self.provider == 'openai':
            headers['Authorization'] = 'Bearer ' + self.key
            body = dict(model=self.profile['model'], messages=messages, response_format={'type': 'json_object'})
            if self.reasoning != 'none': body['reasoning_effort'] = self.reasoning
            endpoint = base + '/chat/completions'
        else:
            headers.update({'x-api-key': self.key, 'anthropic-version': '2023-06-01'})
            converted = []
            for message in messages:
                if message['role'] == 'system': continue
                blocks = []
                for block in message['content']:
                    if block['type'] == 'text': blocks.append(block)
                    elif block['type'] == 'image_url':
                        meta, data = block['image_url']['url'].split(',', 1)
                        blocks.append(dict(type='image', source=dict(type='base64',
                            media_type=meta[5:].split(';')[0], data=data)))
                converted.append(dict(role=message['role'], content=blocks))
            body = dict(model=self.profile['model'], max_tokens=8192,
                        system='\n'.join(m['content'] for m in messages if m['role'] == 'system'), messages=converted)
            endpoint = base + '/messages'
        try:
            response = requests.post(endpoint, headers=headers, json=body, timeout=self.timeout_s, allow_redirects=False)
        except (UnicodeEncodeError, requests.exceptions.InvalidHeader):
            raise ModelConfigurationError('model_configuration_error_request_header') from None
        except requests.Timeout:
            raise RuntimeError('model_request_timeout') from None
        except requests.RequestException:
            raise RuntimeError('model_transport_failed') from None
        if response.status_code in (401, 403):
            raise ModelConfigurationError('model_configuration_error_http_' + str(response.status_code))
        if not response.ok or response.is_redirect:
            raise ModelRequestError(response.status_code, response_category(response),
                                    len(json.dumps(body, ensure_ascii=False).encode('utf-8')))
        try:
            raw = response.json()
            text = (raw['choices'][0]['message']['content'] if self.provider == 'openai' else
                    '\n'.join(b['text'] for b in raw['content'] if b['type'] == 'text'))
            return decision_json(text)
        except (ValueError, KeyError, IndexError, TypeError):
            raise ValueError('invalid_model_json') from None


def request_timeout():
    """Per-request budget for API models; same default and bounds as Codex."""
    try:
        value = int(os.environ.get('ROBOL2L_MODEL_TIMEOUT_S') or 600)
    except ValueError:
        value = 600
    return min(max(value, 30), 3600)


def decision_json(text):
    """Parse the decision object even when a model wraps it in fences or prose.

    Only the outermost {...} span is used; anything that is not a JSON object
    still fails, and the experiment returns it to the model without spending an
    attempt.
    """
    text = text.strip()
    try:
        value = json.loads(text)
    except ValueError:
        start, end = text.find('{'), text.rfind('}')
        if start < 0 or end <= start: raise
        value = json.loads(text[start:end + 1])
    if not isinstance(value, dict): raise ValueError('decision_not_object')
    return value

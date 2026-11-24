"""Versioned experiment ZIP import. Archives are validated and stored, never extracted or executed."""
import hashlib
import io
import json
from pathlib import PurePosixPath
import stat
import zipfile

MAX_ARCHIVE = 20 * 1024 * 1024
MAX_TOTAL = 50 * 1024 * 1024
SCHEMA = 'franka-experiment-bundle/v1'


def parse_bundle(data):
    if not 0 < len(data) <= MAX_ARCHIVE:
        raise ValueError('bundle_size_limit')
    try:
        with zipfile.ZipFile(io.BytesIO(data)) as archive:
            infos = archive.infolist()
            if not 1 <= len(infos) <= 128 or sum(i.file_size for i in infos) > MAX_TOTAL:
                raise ValueError('bundle_size_limit')
            files = {}
            for info in infos:
                path = PurePosixPath(info.filename)
                if (not info.filename or '\\' in info.filename or ':' in info.filename
                    or path.is_absolute() or any(p in ('..', '.') for p in info.filename.split('/'))
                    or info.filename in files or info.is_dir() or '\x00' in info.filename
                    or len(info.filename) > 200
                    or stat.S_ISLNK(info.external_attr >> 16) or info.flag_bits & 1):
                    raise ValueError('bundle_unsafe_path')
                if info.file_size > 10 * 1024 * 1024:
                    raise ValueError('bundle_size_limit')
                # Limit decompression even if a ZIP header lies about its size.
                with archive.open(info) as stream:
                    body = stream.read(10 * 1024 * 1024 + 1)
                if len(body) != info.file_size:
                    raise ValueError('bundle_size_limit')
                files[info.filename] = body
        manifest = json.loads(files['manifest.json'])
        if manifest.get('schema') != SCHEMA or manifest.get('experiment') != 'experiment.json':
            raise ValueError('bundle_schema_invalid')
        hashes = manifest['files']
        if not isinstance(hashes, dict) or set(hashes) != set(files) - {'manifest.json'}:
            raise ValueError('bundle_manifest_mismatch')
        for name, body in files.items():
            if name != 'manifest.json' and hashes[name] != hashlib.sha256(body).hexdigest():
                raise ValueError('bundle_hash_mismatch')
        payload = json.loads(files['experiment.json'])
        allowed = {'name', 'goal', 'success_criteria', 'validation_criteria', 'mode', 'guidance',
                   'pipeline', 'execution_policy', 'demo_outcome'}
        if not isinstance(payload, dict) or set(payload) - allowed:
            raise ValueError('bundle_experiment_fields_invalid')
        payload['guidance'] = list(payload.get('guidance', []))
        guidance_files = manifest.get('guidance_files', [])
        if not isinstance(guidance_files, list) or len(guidance_files) > 20:
            raise ValueError('bundle_guidance_invalid')
        for item in guidance_files:
            kind, name = item['kind'], item['path']
            if kind not in ('AC', 'C') or name not in files or len(files[name]) > 100000:
                raise ValueError('bundle_guidance_invalid')
            text = files[name].decode('utf-8')
            if text.strip():
                payload['guidance'].append({'kind': kind, 'name': name, 'text': text})
        names = manifest.get('context_files', [])
        if not isinstance(names, list) or len(names) > 10 or any(n not in files for n in names):
            raise ValueError('bundle_context_invalid')
        if sum(len(files[n]) for n in names) > 150000:
            raise ValueError('bundle_context_invalid')
        context = [{'path': name, 'text': files[name].decode('utf-8'), 'sha256': hashes[name]} for name in names]
        metadata = {'schema': SCHEMA, 'sha256': hashlib.sha256(data).hexdigest(),
                    'files': [{'path': name, 'size': len(body), 'sha256': hashlib.sha256(body).hexdigest()}
                              for name, body in files.items()],
                    'calibration_policy': 'reference_only_not_installed'}
        return payload, metadata, context
    except ValueError as error:
        if str(error).startswith('bundle_'):
            raise
        raise ValueError('bundle_invalid') from None
    except (KeyError, TypeError, AttributeError, UnicodeError, zipfile.BadZipFile, NotImplementedError, RuntimeError):
        raise ValueError('bundle_invalid') from None

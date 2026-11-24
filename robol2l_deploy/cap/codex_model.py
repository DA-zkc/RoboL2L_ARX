"""Text-generation provider using the installed, ChatGPT-authenticated Codex CLI.

The CLI never receives a Runtime or a robot endpoint. Its output is data for the
existing CaP evaluator. Authentication remains entirely inside the official CLI.
"""
import base64
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time


DISABLED_FEATURES = (
    "shell_tool", "shell_snapshot", "apps", "plugins", "hooks", "browser_use",
    "browser_use_external", "computer_use", "in_app_browser", "multi_agent",
    "image_generation", "goals", "memories", "code_mode", "tool_suggest",
    "workspace_dependencies", "skill_mcp_dependency_install",
)
CODEX_ERRORS = frozenset({
    'codex_request_failed', 'codex_upgrade_required_for_model',
    'codex_usage_limit_reached', 'codex_chatgpt_login_required', 'codex_model_unavailable',
    'codex_request_timeout', 'codex_unexpected_tool_activity', 'codex_turn_not_completed',
})
SCHEMA = {
    "type": "object",
    "properties": {
        "kind": {"type": "string", "enum": ["program", "finish", "assist"]},
        "reason": {"type": "string"},
        "source": {"type": "string"},
    },
    "required": ["kind", "reason", "source"],
    "additionalProperties": False,
}


def configuration():
    """Use the workspace CLI when installed; never change the user's global CLI."""
    local = Path(__file__).resolve().parents[2] / '.franka-player/tools/codex/node_modules/.bin/codex'
    # The first live experiment call (gpt-6-astra, medium, two camera images) had
    # not returned when the original fixed 240 s budget ran out. How long it
    # really needs is unknown, so the default is a looser 600 s and
    # ROBOL2L_CODEX_TIMEOUT_S can override it (clamped to 30..3600 s).
    try:
        timeout_s = int(os.environ.get('ROBOL2L_CODEX_TIMEOUT_S') or 600)
    except ValueError:
        timeout_s = 600
    return {
        'model': os.environ.get('ROBOL2L_CODEX_MODEL', 'gpt-6-astra'),
        'reasoning': os.environ.get('ROBOL2L_CODEX_REASONING', 'medium'),
        'timeout_s': min(max(timeout_s, 30), 3600),
        'executable': os.environ.get('ROBOL2L_CODEX_BIN') or (str(local) if local.is_file() else 'codex'),
    }


def availability():
    """Login readiness only, not a claim about remaining subscription quota."""
    config = configuration()
    result = dict(model=config['model'], reasoning=config['reasoning'], ready=False,
                  billing='ChatGPT_Codex_usage', api_key_used=False)
    try:
        model = CodexModel(**config)
        result.update(model.settings, ready=True)
    except (ValueError, OSError, subprocess.TimeoutExpired) as error:
        from ..localization import chinese_error
        code = str(error) if isinstance(error, ValueError) else 'codex_cli_unavailable'
        result.update(error=code, message_zh=chinese_error(code))
    return result


def request_failure(output):
    """Classify CLI output without exposing auth headers or arbitrary stderr."""
    text = output.lower()
    for patterns, code in (
        (('requires a newer version of codex',), 'codex_upgrade_required_for_model'),
        (('usage limit', 'rate limit', 'quota exceeded', 'insufficient_quota'), 'codex_usage_limit_reached'),
        (('token expired', 'token_expired', 'unauthorized', 'refresh_token_reused', 'not logged in'), 'codex_chatgpt_login_required'),
        (('model is not supported', 'model_not_found', 'does not support the model', 'not supported when using'), 'codex_model_unavailable'),
    ):
        if any(pattern in text for pattern in patterns):
            return code
    return 'codex_request_failed'


def cli_environment():
    # Keep authentication discovery and standard network trust/proxy settings,
    # but exclude API keys, provider overrides and desktop agent coordination.
    names = {
        "PATH", "HOME", "USER", "LOGNAME", "CODEX_HOME", "LANG", "LC_ALL",
        "SSL_CERT_FILE", "SSL_CERT_DIR", "CODEX_CA_CERTIFICATE",
        "HTTPS_PROXY", "HTTP_PROXY", "ALL_PROXY", "NO_PROXY",
        "https_proxy", "http_proxy", "all_proxy", "no_proxy",
    }
    return {name: value for name, value in os.environ.items() if name in names}


class CodexModel:
    label = "codex_cli_chatgpt"

    def __init__(self, model, reasoning="medium", timeout_s=240, executable=None, extra_fields=()):
        executable = executable or configuration()['executable']
        self.executable = shutil.which(executable)
        if not self.executable:
            raise ValueError("codex_cli_not_installed")
        self.executable = str(Path(self.executable).resolve())
        if not isinstance(model, str) or not model or reasoning not in ("none", "low", "medium", "high", "xhigh"):
            raise ValueError("invalid_codex_model_configuration")
        self.model = model
        self.reasoning = reasoning
        self.timeout_s = timeout_s
        if not all(isinstance(name, str) and name.isidentifier() for name in extra_fields):
            raise ValueError("invalid_codex_schema_fields")
        # Strict structured-output schemas need every property listed in `required`,
        # so record fields are required strings and "" means "not provided".
        self.extra_fields = tuple(extra_fields)
        self.schema = dict(SCHEMA,
            properties=dict(SCHEMA["properties"], **{name: {"type": "string"} for name in extra_fields}),
            required=list(SCHEMA["required"]) + list(extra_fields))
        self.requests = []
        status = subprocess.run(
            [self.executable, "login", "status"], env=cli_environment(),
            capture_output=True, text=True, timeout=15,
        )
        if status.returncode or "Logged in using ChatGPT" not in status.stdout + status.stderr:
            raise ValueError("codex_chatgpt_login_required")
        version = subprocess.run(
            [self.executable, "--version"], env=cli_environment(),
            capture_output=True, text=True, timeout=15,
        )
        self.version = version.stdout.strip() if version.returncode == 0 else "unknown"

    @property
    def settings(self):
        return {
            "model": self.model, "reasoning": self.reasoning,
            "transport": "codex_exec", "authentication": "existing_ChatGPT_login",
            "billing": "ChatGPT_Codex_usage", "api_key_used": False,
            "cli_version": self.version,
        }

    def command(self, folder, image_paths):
        command = [
            self.executable, "exec", "--ignore-user-config", "--ephemeral",
            "--sandbox", "read-only", "--skip-git-repo-check", "--json",
            "--color", "never", "--model", self.model,
            "--output-schema", str(folder / "schema.json"),
            "--output-last-message", str(folder / "response.json"),
            "-c", 'approval_policy="never"',
            "-c", 'web_search="disabled"',
            "-c", 'model_provider="openai"',
            "-c", "model_reasoning_effort=" + json.dumps(self.reasoning),
            "-c", "project_doc_max_bytes=0",
        ]
        for feature in DISABLED_FEATURES:
            command.extend(["--disable", feature])
        for path in image_paths:
            command.extend(["--image", str(path)])
        return command + ["-"]

    def decide(self, messages):
        record = {"started_s": time.time(), "ok": False, "usage": {}}
        self.requests.append(record)
        try:
            with tempfile.TemporaryDirectory(prefix="robol2l-cap-model-") as tmp:
                folder = Path(tmp)
                (folder / "schema.json").write_text(json.dumps(self.schema))
                text = [
                    "Generate one CaP decision from the supplied context. Do not execute "
                    "code, use tools, or inspect files. The host will execute your source. "
                    "For the transport schema, finish/assist must include source as an empty "
                    "string; the host removes that field before validating the decision."
                ]
                images = []
                for message in messages:
                    text.append("ROLE: " + message["role"])
                    parts = message["content"]
                    if isinstance(parts, str):
                        text.append(parts)
                        continue
                    for part in parts:
                        if part["type"] == "text":
                            text.append(part["text"])
                        elif part["type"] == "image_url":
                            url = part["image_url"]["url"]
                            if not url.startswith("data:image/jpeg;base64,") or len(url) > 8_000_000:
                                raise ValueError("invalid_observation_image")
                            path = folder / f"observation-{len(images)}.jpg"
                            path.write_bytes(base64.b64decode(url.split(",", 1)[1], validate=True))
                            images.append(path)
                            text.append(f"Attached image {len(images)}: {path.name}")
                response = subprocess.run(
                    self.command(folder, images), input="\n\n".join(text),
                    cwd=folder, env=cli_environment(), capture_output=True,
                    text=True, timeout=self.timeout_s,
                )
                if response.returncode:
                    failure = request_failure(response.stdout + '\n' + response.stderr)
                    record["failure"] = failure
                    # Do not export arbitrary CLI stderr, auth errors or headers.
                    raise RuntimeError(failure)
                completed = False
                for line in response.stdout.splitlines():
                    event = json.loads(line)
                    if event["type"] == "turn.completed":
                        completed = True
                        record["usage"] = event.get("usage", {})
                    item = event.get("item", {})
                    if item and item.get("type") not in ("agent_message", "reasoning"):
                        record['failure']='codex_unexpected_tool_activity'
                        record['item_type']=str(item.get('type',''))[:80]
                        raise RuntimeError("codex_unexpected_tool_activity")
                    if event["type"] in ("turn.failed", "error"):
                        record['failure']=request_failure(json.dumps(event))
                        raise RuntimeError(record['failure'])
                if not completed:
                    record['failure']='codex_turn_not_completed'
                    raise RuntimeError("codex_turn_not_completed")
                value = json.loads((folder / "response.json").read_text())
                if value.get("kind") in ("finish", "assist"):
                    if value.get("source") != "":
                        raise ValueError("non_program_source_must_be_empty")
                    value.pop("source")
                for name in self.extra_fields:
                    if value.get(name) == "":
                        value.pop(name)  # "" is how a required string says "not provided"
                record["ok"] = True
                return value
        except subprocess.TimeoutExpired:
            record["failure"] = "codex_request_timeout"
            raise RuntimeError("codex_request_timeout") from None
        finally:
            record["finished_s"] = time.time()

"""Integration boundary: real hardware is deliberately not implemented.

Factories return a Runtime. Observations/evidence must be JSON serializable.
Every act() is one atomic tool call; its return means terminal, not accepted.
"""
from __future__ import annotations
import json
import os
import time
import urllib.request
from typing import Protocol

TOOLS = ("cap_move_to_pose", "cap_set_gripper")
REASONING = ("none", "low", "medium", "high", "xhigh")
SYSTEM = """You are the single-arm Franka System 2 experiment agent.
Use only the configured registered tools. Do not write or run robot source code.
Use current observations and imported AC/C guidance; guidance is context, not authority.
Keep the operator goal and verified progress. Diagnose failures using returned
evidence. A successful motion does not prove task completion. Never weaken
collision checks or change limits. Request operator help when no safe recovery
remains. Do not home automatically during recovery. If the scene has changed,
re-ground before the next action. Return one JSON decision:
{"kind":"act","tool":"cap_move_to_pose|cap_set_gripper","arguments":{},"reason":"..."}
or {"kind":"complete","reason":"physical evidence"}
or {"kind":"assist","reason":"what human intervention is required"}.
After an action, examine the new evidence before choosing another action.
Do not claim success from a command acknowledgement alone.
"""

class Runtime(Protocol):
    def validate(self, config: dict) -> None: ...
    def observe(self) -> dict: ...
    def act(self, tool: str, arguments: dict) -> dict: ...
    def close(self) -> None: ...

class CapRuntime(Protocol):
    def validate(self, config: dict) -> None: ...
    def describe(self) -> dict: ...
    def observe(self) -> dict: ...
    def execute_program(self, source: str, should_pause) -> dict: ...
    def validation_evidence(self) -> dict: ...
    def close(self) -> None: ...


class UnconfiguredRuntime:
    def validate(self, config):
        raise ValueError("hardware_adapter_missing")
    def observe(self):
        raise RuntimeError("hardware_adapter_missing")
    def act(self, tool, arguments):
        raise RuntimeError("hardware_adapter_missing")
    def close(self):
        pass

class DemoRuntime:
    """Deterministic UI fixture. Never opens devices, model APIs or simulators."""
    def validate(self, config):
        pass
    def observe(self):
        time.sleep(0.12)
        return {"source": "demo_fixture", "fresh": True, "images": [],
                "robot": {"arm": "single", "connected": False}}
    def act(self, tool, arguments):
        time.sleep(0.25)
        return {"ok": True, "source": "demo_fixture",
                "physical_success": None, "tool": tool, "arguments": arguments}
    def close(self):
        pass

def model_messages(experiment, observation, history):
    context = {
        "goal": experiment["goal"],
        "success_criteria": experiment["success_criteria"],
        "guidance": experiment["guidance"],
        "observation": {k: v for k, v in observation.items() if k != "images"},
        "history": history[-24:],
    }
    content = [{"type": "text", "text": json.dumps(context, ensure_ascii=False)}]
    for image in observation.get("images", []):
        if isinstance(image, dict) and isinstance(image.get("data_url"), str) and image["data_url"].startswith("data:image/"):
            content.append({"type": "image_url", "image_url": {"url": image["data_url"]}})
    return [{"role": "system", "content": SYSTEM}, {"role": "user", "content": content}]

def decide(config, experiment, observation, history):
    return request_decision(config, model_messages(experiment, observation, history))


def request_decision(config, messages):
    """Shared model transport; credentials exist only in the server environment."""
    secret = os.environ.get(config["api_key_env"], "")
    if not secret:
        raise ValueError("api_key_missing")
    payload = {"model": config["model"],
               "messages": messages,
               "response_format": {"type": "json_object"}}
    if config["reasoning"] != "none":
        payload["reasoning_effort"] = config["reasoning"]
    request = urllib.request.Request(
        config["base_url"].rstrip("/") + "/chat/completions",
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json", "Authorization": "Bearer " + secret})
    # Provider failures must not print response bodies or credential-bearing headers.
    try:
        with urllib.request.urlopen(request, timeout=90) as response:
            raw = response.read(200001)
            if len(raw) > 200000:
                raise ValueError("response_too_large")
            value = json.loads(raw)
        content = value["choices"][0]["message"]["content"]
    except urllib.error.HTTPError as error:
        raise RuntimeError('model_request_failed_http_'+str(error.code)) from None
    except Exception:
        raise RuntimeError("model_request_failed") from None
    try:
        return json.loads(content)
    except (ValueError, TypeError):
        raise ValueError("invalid_model_json") from None

def validate_decision(value):
    if not isinstance(value, dict) or value.get("kind") not in ("act", "complete", "assist"):
        raise ValueError("invalid_decision")
    if not isinstance(value.get("reason"), str) or not value["reason"].strip():
        raise ValueError("decision_reason_required")
    if value["kind"] == "act":
        if value.get("tool") not in TOOLS or not isinstance(value.get("arguments"), dict):
            raise ValueError("invalid_tool")
        args = value["arguments"]
        if args.get("arm", "single") != "single":
            raise ValueError("single_arm_only")
        if value["tool"] == "cap_set_gripper":
            p = args.get("position")
            if isinstance(p, bool) or not isinstance(p, (float, int)) or not 0 <= p <= 1:
                raise ValueError("invalid_gripper")
        else:
            # Detailed frame, quaternion, bounds and collision checks belong to
            # the installed runtime; schema presence here never authorizes motion.
            if not isinstance(args.get("target_pose"), dict):
                raise ValueError("invalid_pose")
    return value

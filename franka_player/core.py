from __future__ import annotations
import copy
import hashlib
import json
import os
from pathlib import Path
import re
import threading
import time
from urllib.parse import urlparse
from uuid import uuid4
from .runtime import DemoRuntime, UnconfiguredRuntime, REASONING, decide, validate_decision
from .cap_pipeline import decide_cap, validate_cap_decision, compact

DEFAULT_CONFIG = {"base_url": "", "model": "", "reasoning": "high",
                  "api_key_env": "ROBOT_MODEL_API_KEY", "station": "",
                  "max_actions": 20}
MODEL_FIELDS = ("base_url", "model", "reasoning", "api_key_env")

ACTIVE = {"observing", "reasoning", "acting", "verifying", "pausing"}

class Console:
    def __init__(self, root, factory=None, *, model_settings_path=None):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.path = self.root / "state.json"
        self.model_settings_path = Path(model_settings_path) if model_settings_path else None
        self.lock = threading.RLock()
        self.factory = factory
        self.worker = None
        self.pause_requested = False
        self.runtime = None
        self.state = {"schema": 1, "experiments": [], "selected": None,
                      "config": copy.deepcopy(DEFAULT_CONFIG), "events": []}
        if self.path.exists():
            self.state = json.loads(self.path.read_text())
            # Never resume motion automatically after process restart.
            for exp in self.state["experiments"]:
                if exp["status"] in ACTIVE or exp["status"] in ("paused", "waiting_operator"):
                    exp["status"] = "interrupted"
                    exp["detail"] = "process_restarted"
        if self.model_settings_path:
            # Migrate the launcher's saved nonsecret API settings once. Device
            # sessions have their own experiment state, but share model settings.
            if not self.model_settings_path.exists():
                previous=self.model_settings_path.parent/'launcher-console'/'state.json'
                if previous.exists():
                    values=json.loads(previous.read_text()).get('config',{})
                    self.state['config'].update({k:values[k] for k in MODEL_FIELDS if k in values})
            self._load_model_settings()
        for name,variable in (('base_url','ROBOL2L_MODEL_BASE_URL'),('model','ROBOL2L_MODEL'),('reasoning','ROBOL2L_MODEL_REASONING')):
            if os.environ.get(variable):self.state['config'][name]=os.environ[variable]
        if self.model_settings_path:self._save_model_settings()
        self._save()

    def _load_model_settings(self):
        if self.model_settings_path and self.model_settings_path.exists():
            values=json.loads(self.model_settings_path.read_text())
            self.state['config'].update({k:values[k] for k in MODEL_FIELDS if k in values})

    def _save_model_settings(self):
        self.model_settings_path.parent.mkdir(parents=True,exist_ok=True)
        temp=self.model_settings_path.with_name(self.model_settings_path.name+'.'+uuid4().hex+'.tmp')
        temp.write_text(json.dumps({k:self.state['config'][k] for k in MODEL_FIELDS},ensure_ascii=False,indent=2)+'\n')
        os.chmod(temp,0o600)
        temp.replace(self.model_settings_path)

    def _save(self):
        temp = self.path.with_suffix(".tmp")
        temp.write_text(json.dumps(self.state, ensure_ascii=False, indent=2))
        os.chmod(temp, 0o600)
        temp.replace(self.path)

    def _event(self, code, detail="", exp=None):
        self.state["events"].append({"id": uuid4().hex, "time": time.time(),
                                    "code": code, "detail": detail,
                                    "experiment": exp["id"] if exp else self.state["selected"]})
        self._save()

    def snapshot(self):
        with self.lock:
            self._load_model_settings()
            result = copy.deepcopy(self.state)
            result["busy"] = self._busy()
            result["hardware_adapter"] = self.factory is not None
            return result

    def _busy(self):
        return self.worker is not None and self.worker.is_alive()

    def _selected(self):
        for exp in self.state["experiments"]:
            if exp["id"] == self.state["selected"]:
                return exp
        raise ValueError("no_experiment")

    def _idle(self):
        if self._busy():
            raise ValueError("operation_in_flight")

    def _switchable(self):
        self._idle()
        if self.state["selected"] and self._selected()["status"] in ("paused", "waiting_operator"):
            raise ValueError("finish_current_first")

    def create(self, payload, *, bundle=None, bundle_context=None):
        with self.lock:
            self._switchable()
            name, goal = payload.get("name", "").strip(), payload.get("goal", "").strip()
            if not name or not goal or len(goal) > 20000 or len(name) > 150:
                raise ValueError("name_goal_required")
            mode = payload.get("mode", "demo")
            if mode not in ("demo", "live"):
                raise ValueError("invalid_mode")
            pipeline = payload.get("pipeline", "cap" if mode == "live" else "demo")
            if pipeline != ("cap" if mode == "live" else "demo"):
                raise ValueError("invalid_pipeline")
            policy = payload.get("execution_policy", "read_perceive_plan")
            if policy not in ("read_perceive_plan", "guarded_motion") or (policy=="guarded_motion" and mode!="live"):
                raise ValueError("motion_not_commissioned")
            validation_criteria = payload.get("validation_criteria", "验证设备读取、双相机 RGB-D、颜色感知、表面定位和局部离线规划；不执行运动。")
            if not isinstance(validation_criteria, str) or len(validation_criteria) > 20000:
                raise ValueError("invalid_validation_criteria")
            criteria = payload.get("success_criteria", "").strip()
            if not criteria or len(criteria) > 20000:
                raise ValueError("criteria_required")
            incoming_guidance = payload.get("guidance", [])
            if not isinstance(incoming_guidance, list) or len(incoming_guidance) > 20:
                raise ValueError("invalid_guidance")
            if sum(len(str(item.get("text", ""))) for item in incoming_guidance if isinstance(item, dict)) > 200000:
                raise ValueError("invalid_guidance")
            guidance = []
            for item in incoming_guidance:
                kind, text = item.get("kind"), item.get("text")
                if kind not in ("AC", "C") or not isinstance(text, str) or len(text) > 100000:
                    raise ValueError("invalid_guidance")
                guidance.append({"kind": kind, "name": str(item.get("name", kind))[:150],
                                 "text": text, "sha256": hashlib.sha256(text.encode()).hexdigest()})
            exp = {"id": uuid4().hex[:12], "name": name, "goal": goal,
                   "success_criteria": criteria, "mode": mode, "guidance": guidance,
                   "pipeline": pipeline, "execution_policy": policy, "validation_criteria": validation_criteria,
                   "bundle": copy.deepcopy(bundle), "bundle_context": copy.deepcopy(bundle_context or []),
                   "status": "ready", "detail": "", "actions": 0, "model_turns": 0, "attempt": 0,
                   "history": [], "trials": [], "config_history": [], "created": time.time(),
                   "demo_outcome": payload.get("demo_outcome", "complete")}
            self.state["experiments"].append(exp)
            self.state["selected"] = exp["id"]
            self._event("experiment_loaded", exp=exp)
            return exp["id"]

    def select(self, experiment_id=None, next_item=False):
        with self.lock:
            self._switchable()
            items = self.state["experiments"]
            if not items:
                raise ValueError("no_experiment")
            if next_item:
                index = next((i for i,e in enumerate(items) if e["id"] == self.state["selected"]), -1)
                if index + 1 >= len(items):
                    raise ValueError("no_next_experiment")
                experiment_id = items[index + 1]["id"]
            if not any(e["id"] == experiment_id for e in items):
                raise ValueError("not_found")
            self.state["selected"] = experiment_id
            self._event("experiment_selected")

    def configure(self, payload):
        with self.lock:
            self._idle()
            self._load_model_settings()
            if set(payload) - set(DEFAULT_CONFIG):
                raise ValueError("unknown_config")
            config = {**self.state["config"], **payload}
            if not all(isinstance(config[k], str) for k in ("base_url","model","reasoning","api_key_env","station")):
                raise ValueError("invalid_config")
            if config["reasoning"] not in REASONING:
                raise ValueError("invalid_reasoning")
            if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", config["api_key_env"]):
                raise ValueError("invalid_env_name")
            if config["base_url"]:
                url = urlparse(config["base_url"])
                if url.scheme not in ("http", "https") or not url.hostname or url.username or url.password or url.query or url.fragment:
                    raise ValueError("invalid_url")
            limit = config["max_actions"]
            if isinstance(limit, bool) or not isinstance(limit, int) or not 1 <= limit <= 200:
                raise ValueError("invalid_budget")
            if self.runtime is not None and config["station"] != self.state["config"]["station"]:
                raise ValueError("station_change_requires_end")
            self.state["config"] = config
            if self.model_settings_path and set(payload).intersection(MODEL_FIELDS):self._save_model_settings()
            self._event("config_saved")

    def start(self, resume=False, operator_message=""):
        with self.lock:
            self._idle()
            exp = self._selected()
            if resume:
                if exp["status"] not in ("paused", "waiting_operator"):
                    raise ValueError("not_resumable")
                if exp["status"] == "waiting_operator" and not operator_message.strip():
                    raise ValueError("operator_message_required")
            elif exp["status"] not in ("ready", "completed", "validated", "stopped", "error", "interrupted"):
                raise ValueError("not_startable")
            config = copy.deepcopy(self.state["config"])
            runtime = self.runtime if resume else None
            if exp["mode"] == "live":
                if self.factory is None:
                    raise ValueError("hardware_adapter_missing")
                if not config["base_url"] or not config["model"] or not config["station"]:
                    raise ValueError("live_config_missing")
                if not os.environ.get(config["api_key_env"]):
                    raise ValueError("api_key_missing")
            config["_execution_policy"] = exp.get("execution_policy", "read_perceive_plan")
            config["_task_goal_sha256"] = hashlib.sha256(exp["goal"].encode()).hexdigest()
            newly_created = runtime is None
            if newly_created:
                runtime = DemoRuntime() if exp["mode"] == "demo" else self.factory(copy.deepcopy(config))
            try:
                runtime.validate(config)
            except Exception:
                if newly_created:
                    runtime.close()
                raise ValueError("runtime_validation_failed") from None
            if not resume:
                if exp["attempt"]:
                    exp["trials"].append({k: copy.deepcopy(exp.get(k)) for k in
                                          ("attempt","status","actions","model_turns","history","config_history","validation") })
                exp.update(attempt=exp["attempt"] + 1, actions=0, model_turns=0, history=[], config_history=[], validation=None)
            self.runtime = runtime
            exp["config_history"].append({"time": time.time(), "config": config})
            if operator_message:
                exp["history"].append({"kind": "operator", "text": operator_message[:20000]})
                self._event("operator_confirmed", operator_message[:20000], exp)
            self.pause_requested = False
            exp["status"] = "observing"
            self._event("resumed" if resume else "started", exp=exp)
            self.worker = threading.Thread(target=self._run, args=(exp, config, runtime), daemon=True)
            self.worker.start()

    def pause(self):
        with self.lock:
            if not self._busy():
                raise ValueError("not_running")
            self.pause_requested = True
            self._selected()["status"] = "pausing"
            self._event("pause_requested")

    def stop(self):
        with self.lock:
            self._idle()
            exp = self._selected()
            exp["status"] = "stopped"
            self._event("stopped")
            if self.runtime:
                self.runtime.close()
                self.runtime = None

    def _boundary(self, exp, phase):
        with self.lock:
            if self.pause_requested:
                exp["status"] = "paused"
                self._event("paused", exp=exp)
                return False
            exp["status"] = phase
            self._save()
            return True

    def _run(self, exp, config, runtime):
        try:
            if exp["mode"] == "live":
                self._run_cap(exp, config, runtime)
                return
            while True:
                if not self._boundary(exp, "observing"):
                    return
                observation = runtime.observe()
                if observation.get("stale") or observation.get("fresh") is False:
                    with self.lock:
                        exp["status"] = "waiting_operator"
                        exp["detail"] = "observation_unavailable"
                        self._event("waiting_operator", exp["detail"], exp)
                    return
                with self.lock:
                    exp["history"].append({"kind": "observation", "value": observation})
                    self._event("observed", exp=exp)
                if not self._boundary(exp, "reasoning"):
                    return
                if exp["mode"] == "demo":
                    time.sleep(0.35)
                    decision = ({"kind": ("complete" if any(h["kind"] == "operator" for h in exp["history"]) else exp["demo_outcome"]), "reason": "demo_result"}
                                if exp["actions"] >= 2 else
                                {"kind": "act", "tool": "cap_set_gripper",
                                 "arguments": {"arm": "single", "position": 1 if exp["actions"] == 0 else 0},
                                 "reason": "demo_action"})
                else:
                    decision = decide(config, exp, observation, exp["history"])
                decision = validate_decision(decision)
                with self.lock:
                    exp["history"].append({"kind": "decision", "value": decision})
                    self._event("decision", decision["reason"], exp)
                if not self._boundary(exp, "verifying"):
                    return
                if decision["kind"] != "act":
                    with self.lock:
                        exp["status"] = "completed" if decision["kind"] == "complete" else "waiting_operator"
                        exp["detail"] = decision["reason"]
                        self._event(exp["status"], decision["reason"], exp)
                    return
                if exp["actions"] >= config["max_actions"]:
                    with self.lock:
                        exp["status"] = "waiting_operator"
                        exp["detail"] = "action_budget_reached"
                        self._event("waiting_operator", "action_budget_reached", exp)
                    return
                if not self._boundary(exp, "acting"):
                    return
                result = runtime.act(decision["tool"], decision["arguments"])
                with self.lock:
                    exp["actions"] += 1
                    exp["history"].append({"kind": "tool_result", "tool": decision["tool"], "value": result})
                    self._event("action_returned", exp=exp)
                # False/failed result is feedback, not a reset or automatic Home.
                if not self._boundary(exp, "verifying"):
                    return
                time.sleep(0.1)
        except Exception as exc:
            with self.lock:
                exp["status"] = "error"
                # Adapter errors may contain credentials; store only a category.
                exp["detail"] = str(exc) if str(exc) in (
                    "invalid_decision","decision_reason_required","invalid_tool",
                    "single_arm_only","invalid_gripper","invalid_pose",
                    "model_request_failed","api_key_missing") else "runtime_error"
                self._event("error", exp["detail"], exp)
        finally:
            with self.lock:
                if exp["status"] in ("completed", "validated", "error"):
                    try:
                        runtime.close()
                    except Exception:
                        pass
                    self.runtime = None

    def import_bundle(self, data):
        from .bundle import parse_bundle
        payload, metadata, context = parse_bundle(data)
        with self.lock:
            self._switchable()
            directory = self.root / "bundles"
            directory.mkdir(exist_ok=True)
            path = directory / (metadata["sha256"] + ".zip")
            if not path.exists():
                temp = path.with_suffix(".tmp")
                temp.write_bytes(data)
                temp.replace(path)
            metadata["archive_path"] = str(path.resolve())
            return self.create(payload, bundle=metadata, bundle_context=context)

    def _cap_feedback(self, exp, value):
        with self.lock:
            exp["history"].append({"kind": "program_result", "value": value})
            self._event("program_returned", value.get("status", "feedback"), exp)

    def _run_cap(self, exp, config, runtime):
        while True:
            if exp.get("model_turns", 0) >= config["max_actions"]:
                with self.lock:
                    exp.update(status="waiting_operator", detail="cap_round_budget_reached")
                    self._event("waiting_operator", exp["detail"], exp)
                return
            if not self._boundary(exp, "observing"):
                return
            observation = runtime.observe()
            if observation.get("stale") or observation.get("fresh") is False:
                with self.lock:
                    exp.update(status="waiting_operator", detail="observation_unavailable")
                    self._event("waiting_operator", exp["detail"], exp)
                return
            with self.lock:
                # Store image file references, not repeated base64 blobs in state/history.
                exp["history"].append({"kind": "observation", "value": compact(observation)})
                self._event("observed", exp=exp)
            if not self._boundary(exp, "reasoning"):
                return
            with self.lock:
                exp["model_turns"] = exp.get("model_turns", 0) + 1
                self._save()
            try:
                decision = validate_cap_decision(decide_cap(config, exp, observation, exp["history"],
                                                            runtime.describe(), runtime.validation_evidence()))
            except ValueError as error:
                self._cap_feedback(exp, {"ok": False, "status": "invalid_model_response",
                                         "error": str(error), "motion_sent": False})
                continue
            with self.lock:
                exp["history"].append({"kind": "program_decision", "value": decision})
                self._event("code_generated" if decision["kind"] == "program" else "decision", decision["reason"], exp)
            if not self._boundary(exp, "verifying"):
                return
            if decision["kind"] == "assist":
                with self.lock:
                    exp.update(status="waiting_operator", detail=decision["reason"])
                    self._event("waiting_operator", decision["reason"], exp)
                return
            if decision["kind"] == "finish":
                evidence = runtime.validation_evidence()
                if not evidence["passed"]:
                    self._cap_feedback(exp, {"ok": False, "status": "incomplete_evidence",
                                             "error": "Host evidence requirements are not met", "evidence": evidence})
                    continue
                with self.lock:
                    terminal="completed" if exp.get("execution_policy")=="guarded_motion" else "validated"
                    exp.update(status=terminal, detail=decision["reason"], validation=evidence)
                    self._event(terminal, decision["reason"], exp)
                return
            if not self._boundary(exp, "acting"):
                return
            result = runtime.execute_program(decision["source"], should_pause=lambda: self.pause_requested)
            with self.lock:
                exp["actions"] += 1
            self._cap_feedback(exp, result)
            if result.get("fatal_motion_fault"):
                with self.lock:
                    exp.update(status="waiting_operator",detail="physical_execution_fault_no_retry")
                    self._event("waiting_operator",exp["detail"],exp)
                return
            if not self._boundary(exp, "verifying"):
                return

    def shutdown(self):
        with self.lock:
            self.pause_requested = True
        if self.worker:
            self.worker.join(timeout=100)
        if self.runtime and not self._busy():
            self.runtime.close()

"""Two independent software cases: fresh model generation and seeded repair."""
from pathlib import Path
from ..adapters.mock import MockAdapter
from ..runtime import StationRuntime
from .session import CapSession
from .storage import write_json
from .tasks import MockReachTask


class ValidationModel:
    def __init__(self, model, repair_probe=False):
        self.model = model
        self.label = model.label
        self.repair_probe = repair_probe
        self.origins = []

    @property
    def settings(self):
        return getattr(self.model, "settings", {})

    def decide(self, messages):
        if self.repair_probe and not self.origins:
            self.origins.append("injected_test_program_not_model_generated")
            return {
                "kind": "program",
                "reason": "Validation harness injects a legacy dictionary-access error; the real model must repair its execution feedback.",
                "source": 'state = get_robot_state()\npose = state["end_effector_poses"]["primary"]\nresult = pose',
            }
        self.origins.append("model_request")
        return self.model.decide(messages)


def run_validation(model, directory, max_turns=6):
    root = Path(directory)
    root.mkdir(parents=True, exist_ok=False)
    report = {"synthetic": True, "physical_task_success": False, "cases": {}}
    for name, probe in (("generation", False), ("repair", True)):
        provider = ValidationModel(model, repair_probe=probe)
        runtime = StationRuntime(MockAdapter(), root / name / "runtime", allow_motion=True)
        try:
            session = CapSession(runtime, MockReachTask(), provider, root / name / "session", max_turns=max_turns)
            state = session.run()
            versions = state["versions"]
            repair_verified = not probe or (
                len(versions) >= 2 and not versions[0]["program_ok"]
                and any(version["program_ok"] for version in versions[1:])
            )
            report["cases"][name] = {
                "passed": state["status"] == "completed" and repair_verified,
                "status": state["status"], "reason": state.get("reason"),
                "decision_origins": provider.origins, "versions": versions,
                "verification": state["verification"],
            }
            write_json(root / "validation.json", report)
            if state["status"] == "error":
                break  # Do not repeat a failed transport/auth request in another case.
        finally:
            runtime.close()
    report["passed"] = len(report["cases"]) == 2 and all(case["passed"] for case in report["cases"].values())
    report["model_settings"] = getattr(model, "settings", {})
    report["model_requests"] = getattr(model, "requests", [])
    write_json(root / "validation.json", report)
    return report

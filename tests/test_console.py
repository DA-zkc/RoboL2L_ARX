import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from franka_player.core import Console
from franka_player.runtime import model_messages, validate_decision

class ConsoleTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.console = Console(self.tmp.name)
    def tearDown(self):
        self.console.shutdown()
        self.tmp.cleanup()
    def create(self, **extra):
        return self.console.create({"name":"test","goal":"lift","success_criteria":"object lifted",**extra})
    def wait(self):
        self.console.worker.join(5)
        self.assertFalse(self.console.worker.is_alive())
    def test_demo_has_no_model_or_hardware_calls(self):
        self.create(guidance=[{"kind":"AC","name":"a.md","text":"inspect contact"}])
        with patch("franka_player.core.decide", side_effect=AssertionError("model called")):
            self.console.start()
            self.wait()
        exp=self.console._selected()
        self.assertEqual(exp["status"],"completed")
        self.assertEqual(exp["actions"],2)
        self.assertEqual(len(exp["guidance"][0]["sha256"]),64)
        self.console.start()
        self.wait()
        self.assertEqual(len(exp["trials"]),1)
        self.assertEqual(exp["attempt"],2)
    def test_live_blocked_before_model(self):
        self.create(mode="live")
        with self.assertRaisesRegex(ValueError,"hardware_adapter_missing"):
            self.console.start()
        self.assertEqual(self.console._selected()["status"],"ready")
    def test_assistance_requires_message_and_reobserves(self):
        self.create(demo_outcome="assist")
        self.console.start()
        self.wait()
        self.assertEqual(self.console._selected()["status"],"waiting_operator")
        with self.assertRaisesRegex(ValueError,"operator_message_required"):
            self.console.start(resume=True)
        self.console.start(resume=True,operator_message="scene restored")
        self.wait()
        exp=self.console._selected()
        self.assertEqual(exp["status"],"completed")
        pos=next(i for i,h in enumerate(exp["history"]) if h["kind"]=="operator")
        self.assertEqual(exp["history"][pos+1]["kind"],"observation")
    def test_pause_during_action_waits_and_prevents_dispatch(self):
        entered, release=threading.Event(),threading.Event()
        def blocking(runtime,tool,args):
            entered.set()
            release.wait(3)
            return {"ok":False,"error":"tracking_error"}
        self.create()
        with patch("franka_player.runtime.DemoRuntime.act",blocking):
            self.console.start()
            self.assertTrue(entered.wait(3))
            self.console.pause()
            with self.assertRaisesRegex(ValueError,"operation_in_flight"):
                self.console.configure({"model":"new"})
            with self.assertRaises(ValueError):
                self.console.select(next_item=True)
            release.set()
            self.wait()
        exp=self.console._selected()
        self.assertEqual(exp["status"],"paused")
        self.assertEqual(exp["actions"],1)
        self.console.configure({"model":"new"})
        with self.assertRaisesRegex(ValueError,"station_change_requires_end"):
            self.console.configure({"station":"another"})
        with self.assertRaisesRegex(ValueError,"finish_current_first"):
            self.create()
        self.console.stop()
        self.create()
    def test_restart_never_autoresumes(self):
        self.create()
        self.console._selected()["status"]="acting"
        self.console._save()
        restored=Console(self.tmp.name)
        self.assertEqual(restored._selected()["status"],"interrupted")
        self.assertIsNone(restored.worker)
    def test_config_validation_and_no_secret_field(self):
        with self.assertRaises(ValueError):
            self.console.configure({"api_key":"secret"})
        with self.assertRaises(ValueError):
            self.console.configure({"base_url":"https://user:secret@example.com"})
        with self.assertRaises(ValueError):
            self.console.configure({"max_actions":True})
        self.console.configure({"model":"example","base_url":"http://localhost:8000/v1"})
        self.assertEqual(Console(self.tmp.name).state["config"]["model"],"example")
    def test_guidance_is_context_and_snapshot(self):
        guidance=[{"kind":"C","name":"c.md","text":"use fresh images"}]
        self.create(guidance=guidance)
        guidance[0]["text"]="mutated"
        exp=self.console._selected()
        messages=model_messages(exp,{},[])
        self.assertIn("use fresh images",messages[1]["content"][0]["text"])
        self.assertNotIn("mutated",messages[1]["content"])
    def test_stale_observation_blocks_decisions(self):
        self.create()
        with patch("franka_player.runtime.DemoRuntime.observe",return_value={"fresh":False}):
            self.console.start()
            self.wait()
        self.assertEqual(self.console._selected()["status"],"waiting_operator")
        self.assertEqual(self.console._selected()["actions"],0)
    def test_tool_allowlist(self):
        for decision in [
            {"kind":"act","tool":"shell","arguments":{},"reason":"bad"},
            {"kind":"act","tool":"cap_set_gripper","arguments":{"position":2},"reason":"bad"},
            {"kind":"act","tool":"cap_move_to_pose","arguments":{"arm":"left","target_pose":{}},"reason":"bad"}]:
            with self.assertRaises(ValueError):
                validate_decision(decision)

if __name__=="__main__":
    unittest.main()

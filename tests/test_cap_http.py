import json
from pathlib import Path
import tempfile
import threading
import unittest
import urllib.request
import urllib.error
from unittest.mock import MagicMock
from franka_player.server import Server
from franka_player.core import Console
from franka_cap.web import cap_handler

class CapHttpTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.sdk=MagicMock();self.sdk.run_dir=Path(self.tmp.name)
        self.sdk.observations={};self.sdk.describe.return_value={'motion_enabled':False}
        self.sdk.get_robot_state.return_value={'source':'test','q':[0]*7}
        self.console=Console(Path(self.tmp.name)/'console')
        self.server=Server(('127.0.0.1',0),cap_handler(self.console,self.sdk))
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.url='http://127.0.0.1:'+str(self.server.server_port)
        self.client=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.thread.join();self.console.shutdown();self.tmp.cleanup()
    def request(self,path,body=None,headers=None):
        req=urllib.request.Request(self.url+path,data=None if body is None else json.dumps(body).encode(),headers=headers or {})
        return self.client.open(req,timeout=5)
    def test_cap_contract_and_state(self):
        with self.request('/api/cap/capabilities') as r:self.assertFalse(json.load(r)['motion_enabled'])
        with self.request('/api/cap/state',{}, {'X-Console-Request':'1'}) as r:
            result=json.load(r);self.assertFalse(result['motion_sent']);self.assertEqual(result['result']['source'],'test')
    def test_csrf_and_arbitrary_paths_rejected(self):
        for headers in ({},{'X-Console-Request':'1','Origin':'https://example.org'}):
            with self.assertRaises(urllib.error.HTTPError) as e:self.request('/api/cap/state',{},headers)
            self.assertEqual(e.exception.code,403)
        self.sdk.get_robot_state.assert_not_called()
        with self.assertRaises(urllib.error.HTTPError) as e:self.request('/api/cap/image?observation=../../etc/passwd&camera=wrist')
        self.assertEqual(e.exception.code,404)
    def test_program_cannot_bypass_sdk(self):
        body={'source':'def run(api):\n api.get_robot_state()\n import os'}
        with self.assertRaises(urllib.error.HTTPError) as e:self.request('/api/cap/program',body,{'X-Console-Request':'1'})
        self.assertEqual(e.exception.code,409);self.sdk.get_robot_state.assert_not_called()
    def test_parallel_hardware_requests_rejected(self):
        entered=threading.Event();release=threading.Event();errors=[]
        def blocking():entered.set();release.wait(5);return {'source':'test'}
        self.sdk.get_robot_state.side_effect=blocking
        def first():
            try:
                with self.request('/api/cap/state',{}, {'X-Console-Request':'1'}) as r:r.read()
            except Exception as e:errors.append(e)
        thread=threading.Thread(target=first);thread.start()
        try:
            self.assertTrue(entered.wait(3))
            with self.assertRaises(urllib.error.HTTPError) as e:self.request('/api/cap/state',{}, {'X-Console-Request':'1'})
            self.assertEqual(e.exception.code,409)
        finally:release.set();thread.join()
        self.assertEqual(errors,[])

if __name__=='__main__':unittest.main()

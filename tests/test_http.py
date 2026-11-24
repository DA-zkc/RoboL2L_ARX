import json
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from franka_player.core import Console
from franka_player.server import Server, handler

class HttpTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.console=Console(self.tmp.name)
        self.server=Server(("127.0.0.1",0),handler(self.console))
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True)
        self.thread.start()
        self.url="http://127.0.0.1:"+str(self.server.server_port)
    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.console.shutdown()
        self.tmp.cleanup()
    def request(self,path,body=None,headers=None):
        req=urllib.request.Request(self.url+path,
             data=None if body is None else json.dumps(body).encode(),
             headers=headers or {})
        return urllib.request.urlopen(req)
    def test_load_config_and_export(self):
        with self.request("/") as response:
            self.assertEqual(response.status,200)
            self.assertIn("frame-ancestors",response.headers["Content-Security-Policy"])
        with self.request("/api/experiments",{"name":"test","goal":"lift","success_criteria":"clear"},
                          {"X-Console-Request":"1"}) as response:
            state=json.load(response)
        self.assertEqual(len(state["experiments"]),1)
        with self.request("/api/export") as response:
            self.assertEqual(json.load(response)["selected"],state["selected"])
    def test_cross_origin_and_missing_csrf_header_rejected(self):
        for headers in ({},{"X-Console-Request":"1","Origin":"https://attacker.example"}):
            with self.assertRaises(urllib.error.HTTPError) as caught:
                self.request("/api/pause",{},headers)
            self.assertEqual(caught.exception.code,403)
    def test_unknown_fields_cannot_store_keys(self):
        with self.assertRaises(urllib.error.HTTPError) as caught:
            self.request("/api/config",{"api_key":"do-not-persist"},{"X-Console-Request":"1"})
        self.assertEqual(caught.exception.code,409)
        self.assertNotIn("do-not-persist",self.console.path.read_text())

if __name__=="__main__":
    unittest.main()

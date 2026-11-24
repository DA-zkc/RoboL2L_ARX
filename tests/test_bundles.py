import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
import zipfile
from franka_player.bundle import parse_bundle,SCHEMA
from franka_player.core import Console


def bundle(extra=None,manifest_extra=None):
    task={'name':'green cube','goal':'put in plate','success_criteria':'lift, place, release','mode':'live','pipeline':'cap','execution_policy':'read_perceive_plan','guidance':[]}
    files={'experiment.json':json.dumps(task).encode(),'guidance/AC.md':b'','guidance/C.md':b'','README.md':b'read only'}
    files.update(extra or {})
    manifest={'schema':SCHEMA,'experiment':'experiment.json','files':{k:hashlib.sha256(v).hexdigest() for k,v in files.items()},'guidance_files':[{'kind':'AC','path':'guidance/AC.md'},{'kind':'C','path':'guidance/C.md'}],'context_files':['README.md']}
    manifest.update(manifest_extra or {})
    files['manifest.json']=json.dumps(manifest).encode();stream=io.BytesIO()
    with zipfile.ZipFile(stream,'w',zipfile.ZIP_DEFLATED) as z:
        for name,data in files.items():z.writestr(name,data)
    return stream.getvalue()


class BundleTests(unittest.TestCase):
    def test_import_preserves_archive_and_empty_guidance_without_start(self):
        data=bundle({'reference/calibration.json':b'{"reference_only":true}'})
        with tempfile.TemporaryDirectory() as tmp:
            console=Console(tmp);console.import_bundle(data);exp=console._selected()
            self.assertEqual(exp['guidance'],[]);self.assertEqual(exp['attempt'],0);self.assertEqual(exp['status'],'ready')
            self.assertEqual(Path(exp['bundle']['archive_path']).read_bytes(),data)
            self.assertEqual(exp['bundle_context'][0]['text'],'read only')
            self.assertIsNone(console.worker)
    def test_bad_hash_traversal_and_unknown_schema(self):
        for data in [bundle({'../outside.txt':b'bad'}),bundle(manifest_extra={'schema':'unknown'}),bundle(manifest_extra={'files':{}})]:
            with self.assertRaises(ValueError):parse_bundle(data)
    def test_nonempty_guidance_frozen_and_no_config_installed(self):
        with tempfile.TemporaryDirectory() as tmp:
            console=Console(tmp);console.import_bundle(bundle({'guidance/AC.md':b'look before planning','config/station.json':b'{"motion_enabled":true}'}))
            self.assertEqual(console._selected()['guidance'][0]['sha256'],hashlib.sha256(b'look before planning').hexdigest())
            self.assertFalse((Path(tmp)/'config/station.json').exists());self.assertEqual(console.state['config']['station'],'')
    def test_duplicate_and_symlink_rejected(self):
        for symlink in (False,True):
            stream=io.BytesIO()
            with zipfile.ZipFile(stream,'w') as z:
                z.writestr('a',b'one')
                info=zipfile.ZipInfo('link' if symlink else 'a')
                if symlink:info.create_system=3;info.external_attr=0o120777<<16
                z.writestr(info,b'two')
            with self.assertRaisesRegex(ValueError,'bundle_unsafe_path'):parse_bundle(stream.getvalue())
    def test_import_cannot_enable_motion(self):
        task={'name':'x','goal':'x','success_criteria':'x','mode':'live','execution_policy':'motion'}
        with tempfile.TemporaryDirectory() as tmp:
            console=Console(tmp)
            with self.assertRaisesRegex(ValueError,'motion_not_commissioned'):
                console.import_bundle(bundle({'experiment.json':json.dumps(task).encode()}))
            self.assertEqual(console.snapshot()['experiments'],[])

if __name__=='__main__':unittest.main()

class BundleHttpTests(unittest.TestCase):
    def test_raw_zip_upload_and_same_origin_boundary(self):
        import threading
        import urllib.request
        import urllib.error
        from franka_player.server import Server,handler
        with tempfile.TemporaryDirectory() as tmp:
            console=Console(tmp);server=Server(('127.0.0.1',0),handler(console))
            worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
            client=urllib.request.build_opener(urllib.request.ProxyHandler({}))
            url='http://127.0.0.1:'+str(server.server_port)+'/api/experiments/import'
            try:
                for headers in ({},{'X-Console-Request':'1','Origin':'https://other.example'}):
                    with self.assertRaises(urllib.error.HTTPError) as error:
                        client.open(urllib.request.Request(url,data=bundle(),headers=headers),timeout=5)
                    self.assertEqual(error.exception.code,403)
                self.assertEqual(console.snapshot()['experiments'],[])
                request=urllib.request.Request(url,data=bundle(),headers={'X-Console-Request':'1','Content-Type':'application/zip'})
                with client.open(request,timeout=5) as response:state=json.load(response)
                exp=state['experiments'][0]
                self.assertEqual(exp['status'],'ready');self.assertEqual(exp['guidance'],[])
                self.assertEqual(exp['pipeline'],'cap');self.assertFalse(state['busy'])
            finally:server.shutdown();server.server_close();worker.join();console.shutdown()

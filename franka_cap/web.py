"""Same-origin console routes; no arbitrary filesystem or command execution."""
import json
from pathlib import Path
import threading
from uuid import uuid4
from urllib.parse import parse_qs,urlparse
from .program import Program
from .common import write_json
from franka_player.server import handler

def cap_handler(console,sdk):
    base=handler(console);lock=threading.Lock()
    class Handler(base):
        def do_GET(self):
            parsed=urlparse(self.path)
            if not parsed.path.startswith('/api/cap/'):
                return super().do_GET()
            if not self.trusted():return self.send(403,{'error':'origin_rejected'})
            try:
                if parsed.path=='/api/cap/capabilities':return self.send(200,sdk.describe())
                if parsed.path=='/api/cap/image':
                    query=parse_qs(parsed.query)
                    obs=sdk.observations[query['observation'][0]]
                    cam=obs['cameras'][query['camera'][0]]
                    path=Path(cam['rgb_path'])
                    return self.send(200,path.read_bytes(),'image/png' if path.suffix=='.png' else 'image/jpeg')
                return self.send(404,{'error':'not_found'})
            except (KeyError,ValueError,OSError):return self.send(404,{'error':'artifact_not_found'})

        def do_POST(self):
            route=urlparse(self.path).path
            if not route.startswith('/api/cap/'):return super().do_POST()
            if not self.trusted() or self.headers.get('X-Console-Request')!='1':
                return self.send(403,{'error':'origin_rejected'})
            if not lock.acquire(blocking=False):return self.send(409,{'error':'CAP request in progress'})
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0<length<=32000:raise ValueError('invalid_body_size')
                body=json.loads(self.rfile.read(length))
                if not isinstance(body,dict):raise ValueError('invalid_body')
                if route=='/api/cap/state' and not body:result=sdk.get_robot_state()
                elif route=='/api/cap/observe' and set(body)<= {'camera_names'}:result=sdk.get_observation(body.get('camera_names'))
                elif route=='/api/cap/plan' and set(body)=={'target_pose'}:result=sdk.plan_motion(body['target_pose'])
                elif route=='/api/cap/program' and set(body)=={'source'}:
                    program=Program(sdk);program.validate(body['source'])
                    directory=sdk.run_dir/('program-'+uuid4().hex[:12]);directory.mkdir()
                    (directory/'source.py').write_text(body['source'])
                    try:
                        result=program.run(body['source'])
                        write_json(directory/'result.json',{'status':'passed','result':result,'motion_sent':False})
                    except Exception as error:
                        write_json(directory/'result.json',{'status':'failed','error':str(error),'motion_sent':False})
                        raise
                else:raise ValueError('Unsupported CAP request or fields')
                return self.send(200,{'result':result,'motion_sent':False,'run_directory':str(sdk.run_dir)})
            except Exception as error:
                # SDK operations never carry model secrets. Explicit device failures are useful here.
                return self.send(409,{'error':str(error),'motion_sent':False})
            finally:lock.release()
    return Handler

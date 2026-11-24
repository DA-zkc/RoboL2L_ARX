"""Request bounded same-frame camera traces from an already running owner.

Never starts a device, reloads a service, changes camera properties, or sends
robot commands. Refuses legacy owners without the loaded trace capability.
"""
import argparse
import json
from pathlib import Path
import time
import uuid

import requests

ROOT = Path(__file__).resolve().parents[1]
BASE = 'http://127.0.0.1:8771/api/hardware/'


def capture(camera, count):
    client = requests.Session(); client.trust_env = False
    state = client.get(BASE+'session', timeout=10).json()
    if state.get('busy') or state.get('cap_running') or state.get('closed'):
        raise RuntimeError('camera trace requires an idle console; no pause or stop will be sent')
    session_id = state['session_id']
    client.headers.update({'Origin':'http://127.0.0.1:8771','X-Console-Request':'1','X-Session-Id':session_id})
    job_id = 'depth-trace-observe-'+uuid.uuid4().hex
    response = client.post(BASE+'jobs', json=dict(request_id=job_id, operation='get_observation',
                           arguments={'camera_names':[camera]}), timeout=10)
    response.raise_for_status()
    deadline = time.monotonic()+25
    while True:
        status = client.get(BASE+'state', timeout=10).json()
        if status['session_id'] != session_id:
            raise RuntimeError('session changed; no trace request written')
        job = status['jobs'][job_id]
        if job['status'] == 'completed':
            break
        if job['status'] not in ('accepted','running') or time.monotonic() >= deadline:
            raise RuntimeError(f'observation failed: {job}')
        time.sleep(.2)
    # Observation arrays are intentionally not in the HTTP artifact allowlist.
    # This workstation CLI reads the exact freshly acknowledged local archive.
    archives = list((ROOT/'.franka-player/panel/sessions').glob(
        f"*/hardware/observation-{job['observation_id']}/metadata.json"))
    if len(archives) != 1:
        raise RuntimeError('fresh observation archive is not uniquely available locally')
    measurement = json.loads(archives[0].read_text())['measurement_diagnostics'][camera]
    capability = measurement.get('depth_capture', {})
    if capability.get('supported') is not True:
        raise RuntimeError('running camera owner has no raw-depth capture hook; no restart, injection, or trace request performed')
    request_file = Path(capability['request_file']).resolve()
    if not request_file.is_relative_to((ROOT/'.franka-player').resolve()) or request_file.name != 'request.json':
        raise RuntimeError('unexpected trace path')
    request_id = uuid.uuid4().hex
    request = dict(id=request_id, serial=measurement['serial'], frames=count)
    request_file.parent.mkdir(parents=True, exist_ok=True)
    if request_file.exists():
        raise RuntimeError('another capture request exists; refusing to replace it')
    # Exclusive creation: bounded request consumed by the existing camera thread.
    with request_file.open('x') as stream:
        json.dump(request, stream)
    output = request_file.parent/request_id
    try:
        deadline = time.monotonic()+20
        while time.monotonic() < deadline:
            current = client.get(BASE+'session', timeout=10).json()
            if current['session_id'] != session_id:
                raise RuntimeError('session changed during capture')
            if len(list(output.glob('*/metadata.json'))) == count:
                return dict(session_id=session_id, camera=camera, frames=count, directory=str(output),
                            observation_id=job['observation_id'], restart=False, motion_commands=0)
            error_file=request_file.parent/'error.json'
            if error_file.exists():
                error=json.loads(error_file.read_text())
                if error.get('request',{}).get('id')==request_id:
                    raise RuntimeError(str(error))
            time.sleep(.2)
        raise RuntimeError(f'trace capture timed out; partial evidence at {output}')
    finally:
        if request_file.exists() and json.loads(request_file.read_text()).get('id') == request_id:
            request_file.unlink()


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera',choices=('global','left_wrist','right_wrist'),required=True)
    parser.add_argument('--frames',type=int,choices=range(1,9),default=4)
    args=parser.parse_args()
    try:
        print(json.dumps(capture(args.camera,args.frames),ensure_ascii=False,indent=2))
    except (RuntimeError,requests.RequestException) as error:
        parser.exit(2,str(error)+'\n')

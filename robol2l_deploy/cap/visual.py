"""Autonomous, read-only CaP preparation from real RGB-D evidence."""
import json
import time
from pathlib import Path
import requests
from franka_cap.common import config
from ..adapters.replay import ReplayAdapter, matching_raw_state
from ..observations import load_observation
from ..runtime import StationRuntime
from .session import CapSession
from .tasks import VisualPreparationTask
from .storage import write_json


def capture_existing_station(base_url, station_directory):
    """Ask the already-running device owner for one observation, never motion."""
    from uuid import uuid4
    client=requests.Session();client.trust_env=False
    body={'request_id':uuid4().hex,'operation':'get_observation','arguments':{}}
    response=client.post(base_url.rstrip('/')+'/api/hardware/jobs',json=body,
        headers={'X-Console-Request':'1'},timeout=20);response.raise_for_status()
    job=response.json();deadline=time.monotonic()+40
    while job['status'] in ('accepted','running'):
        if time.monotonic()>deadline:raise TimeoutError('observation_owner_timeout')
        time.sleep(.2)
        state=client.get(base_url.rstrip('/')+'/api/hardware/state',timeout=10);state.raise_for_status()
        job=state.json()['jobs'][body['request_id']]
    if job['status']!='completed' or 'observation_id' not in job:
        raise RuntimeError('observation_capture_failed')
    return Path(station_directory)/('observation-'+job['observation_id'])


def run_visual(model, output, *, observation_directory=None, station_config=None,
               live_url=None, station_directory=None, max_turns=8):
    if live_url:
        if not station_directory:raise ValueError('live capture requires the existing station artifact directory')
        observation_directory=capture_existing_station(live_url,station_directory)
    if not observation_directory:raise ValueError('provide recorded observation or live owner URL')
    observation=load_observation(observation_directory)
    metadata=json.loads((Path(observation_directory)/'metadata.json').read_text())
    raw=metadata.get('raw_robot_state') or matching_raw_state(observation,Path(__file__).resolve().parents[2]/'runs/cap')
    output=Path(output)
    adapter=ReplayAdapter(observation_directory,raw,config(station_config),output/'adapter')
    runtime=StationRuntime(adapter,output/'runtime',allow_motion=False)
    try:
        state=CapSession(runtime,VisualPreparationTask(),model,output/'session',max_turns=max_turns).run()
        report=dict(status=state['status'],verification=state['verification'],versions=state['versions'],
            source_observation=str(Path(observation_directory).resolve()),
            calibration_provenance=adapter.calibration_provenance,
            evidence_kind='fresh_physical_capture' if live_url else 'recorded_physical_capture',
            program_author='model',manual_regions=False,physical_motion_executed=False,physical_task_success=False,
            model_settings=getattr(model,'settings',{}),model_requests=getattr(model,'requests',[]))
        write_json(output/'validation.json',report)
        return report
    finally:runtime.close()

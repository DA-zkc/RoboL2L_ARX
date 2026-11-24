"""Same-origin typed jobs and camera streams for the station workbench."""
import json
import time
import os
import threading
from pathlib import Path
from urllib.parse import urlparse,parse_qs
import cv2
import numpy as np
from .runtime import public

ASSETS=Path(__file__).with_name('web')

def hardware_handler(base,runtime,model_config=None,cap_controller=None,experiment_root=None):
    from .cap.session import CapController
    cap = cap_controller or CapController(runtime)
    from .operator import OperatorConsole
    operator = OperatorConsole(runtime, cap)
    control_lock = threading.RLock()
    from .cap.experiment import ExperimentController
    from .cap.experiment_models import ModelProfiles
    from .cap.skills import SkillLibrary
    from .cap.task_presets import TaskPresets
    experiment_root = Path(experiment_root) if experiment_root else runtime.directory
    profiles = ModelProfiles(experiment_root/'experiment-model-settings.json', model_config)
    skills = SkillLibrary(experiment_root/'skills')
    presets = TaskPresets(experiment_root/'task-presets.json')
    experiment = ExperimentController(runtime, lambda: profiles.model(runtime.adapter.synthetic),
                                      experiment_root/'experiments', skills=skills)
    model_choice_path = runtime.directory / 'cap-model.json'

    from .arx_calibration import CalibrationSession, DICTIONARIES
    from .arx_teach import TeachKey
    calibration = CalibrationSession(runtime)
    teach = TeachKey(runtime)
    from .arx_gripper_calibration import GripperCalibration
    hands = GripperCalibration(runtime)
    calibration_assets = {'/hardware/calibration': ('calibration.html', 'text/html'),
                          '/manual-positioning.js': ('manual-positioning.js', 'text/javascript'),
                          '/arx-calibration.js': ('calibration.js', 'text/javascript'),
                          '/arx-calibration.css': ('calibration.css', 'text/css')}

    def require_arx():
        if runtime.adapter.describe().get('embodiment') != 'arx_x5_2023_dual':
            raise ValueError('请选择双臂 ARX Runtime / Dual ARX station required')

    def model_choice():
        if model_choice_path.exists():
            return json.loads(model_choice_path.read_text())['mode']
        return 'fixture' if runtime.adapter.synthetic else 'configured'

    def make_model(mode):
        from .cap.model import FeedbackFixture,ConfiguredModel
        if mode=='fixture':return FeedbackFixture()
        if mode=='configured' and model_config is not None:
            model=ConfiguredModel(model_config())
            if not os.environ.get(model.config['api_key_env']):raise ValueError('api_key_missing')
            return model
        if mode=='codex':
            from .cap.codex_model import CodexModel,configuration
            return CodexModel(**configuration())
        raise ValueError('model_not_configured')

    def run_cap(model,task,*,authorization=None,probe_distance_m=.004,continuation=None,message=''):
        from .physical import authorize,revoke
        with control_lock:
            if experiment.active():raise ValueError('experiment_active_end_before_starting_cap')
            operator.idle()
            if runtime.describe()['busy']:raise ValueError('station_busy')
            if continuation:
                from .cap.continuation import RESUMABLE
                previous=cap.stored_session(continuation)
                if previous['status'] not in RESUMABLE:raise ValueError('cap_not_continuable')
                if runtime.faulted:raise ValueError('runtime_fault_latched')
                if runtime.paused:runtime.pause(False)
            if runtime.adapter.describe().get('robot_backend') == 'arx_x5_2023_dual' and task.context.family != 'dual-arm-joints':
                raise ValueError('use_dual_joint_task_or_experiment_panel')
            if authorization:
                with runtime.program_lease():
                    runtime.get_observation()
                    authorize(runtime,task.context.task_name,authorization,
                        probe_distance_m=probe_distance_m,require_probe=False,continuing=bool(continuation))
            try:
                accepted=cap.continue_session(model,task,continuation,message) if continuation else cap.start(model,task)
            except Exception:
                if authorization:revoke(runtime)
                raise
            if authorization:
                active_thread,policy=cap.thread,runtime.physical_session
                def finish():
                    active_thread.join()
                    with control_lock:
                        if runtime.physical_session is policy:revoke(runtime)
                threading.Thread(target=finish,daemon=True).start()
            return accepted

    def artifact_paths():
        return sorted([path for group in ('perception','operator')
            for path in (runtime.directory/group).glob('*/*')
            if path.is_file() and path.suffix in ('.png','.npz','.json')],
            key=lambda p:p.stat().st_mtime,reverse=True)[:120]
    class Handler(base):
        def do_GET(self):
            parsed=urlparse(self.path);route=parsed.path
            if route not in calibration_assets and route not in ('/hardware','/hardware.js','/hardware.css','/cap-chat.js','/experiment.js') and not route.startswith('/api/hardware/'):
                return super().do_GET()
            if not self.trusted():return self.send(403,{'error':'origin_rejected'})
            try:
                if route in calibration_assets:
                    name, kind = calibration_assets[route]
                    return self.send(200, (ASSETS / name).read_bytes(), kind)
                if route == '/api/hardware/calibration/status':
                    require_arx()
                    return self.send(200, dict(session=calibration.status(), teach=teach.status(),
                                               dictionaries=DICTIONARIES))
                if route == '/api/hardware/manual-positioning/status':
                    require_arx()
                    reason = ('experiment_busy' if experiment.busy() else
                              'cap_session_busy' if cap.thread and cap.thread.is_alive() else
                              'station_faulted' if runtime.faulted else
                              'station_paused' if runtime.paused else
                              'station_busy' if runtime.describe()['busy'] and not teach.token else None)
                    owner = getattr(runtime.adapter, 'owner_client', None)
                    if not reason and owner and getattr(owner, 'manual_positioning_version', 0) < 1:
                        reason = 'manual_positioning_requires_owner_maintenance_update'
                    return self.send(200, dict(teach=teach.status(), blocked_reason=reason))
                if route == '/api/hardware/calibration/report':
                    require_arx()
                    state = calibration.checked(parse_qs(parsed.query)['id'][0])
                    if not state['report']:
                        raise ValueError('report_unavailable')
                    return self.send(200, state['report'])
                if route in ('/hardware','/hardware.js','/hardware.css','/cap-chat.js','/experiment.js'):
                    name,kind={'/hardware':('index.html','text/html'),'/hardware.js':('app.js','text/javascript'),'/hardware.css':('style.css','text/css'),'/cap-chat.js':('cap-chat.js','text/javascript'),'/experiment.js':('experiment.js','text/javascript')}[route]
                    return self.send(200,(ASSETS/name).read_bytes(),kind)
                if route=='/api/hardware/experiment/state':return self.send(200,experiment.snapshot(summary=True))
                if route=='/api/hardware/experiment/settings':return self.send(200,profiles.public())
                if route=='/api/hardware/experiment/skills':return self.send(200,skills.describe())
                if route=='/api/hardware/experiment/presets':return self.send(200,presets.public())
                if route=='/api/hardware/experiment/records':return self.send(200,{'records':experiment.records()})
                if route=='/api/hardware/experiment/record':
                    return self.send(200,experiment.record(parse_qs(parsed.query)['id'][0]))
                if route=='/api/hardware/experiment/video':
                    query=parse_qs(parsed.query)
                    session=query['session'][0];name=query['file'][0]
                    for part in (session,name):
                        if Path(part).name!=part:raise ValueError('invalid_experiment_video')
                    if not name.endswith('.mp4'):raise ValueError('invalid_experiment_video')
                    path=experiment.directory/session/name
                    if not path.is_file():raise ValueError('experiment_video_not_found')
                    return self.send(200,path.read_bytes(),'video/mp4')
                if route=='/api/hardware/experiment/export':
                    name=parse_qs(parsed.query)['id'][0]
                    return self.send(200,experiment.archive(name),'application/zip')
                if route=='/api/hardware/experiment/image':
                    query=parse_qs(parsed.query)
                    session=query['session'][0];name=query['file'][0]
                    current=experiment.snapshot()
                    if session!=current.get('session_id') or Path(name).name!=name or not name.endswith('.jpg'):
                        raise ValueError('invalid_experiment_image')
                    return self.send(200,(experiment.directory/session/name).read_bytes(),'image/jpeg')
                if route=='/api/hardware/state':return self.send(200,dict(runtime.snapshot(),session_id=os.environ.get('ROBOL2L_MANAGED_SESSION')))
                if route=='/api/hardware/cap/state':return self.send(200,public(cap.snapshot()))
                if route=='/api/hardware/cap/turn':
                    query=parse_qs(parsed.query)
                    return self.send(200,public(cap.turn_detail(query['session'][0],int(query['turn'][0]))))
                if route=='/api/hardware/operator':return self.send(200,operator.describe())
                if route=='/api/hardware/services':
                    import requests
                    from .providers.perception import configuration
                    config=configuration();services={}
                    client=requests.Session();client.trust_env=False
                    for name,key in (('SAM3','sam3_url'),('Contact-GraspNet','graspnet_url')):
                        try:
                            response=client.get(config[key].rstrip('/')+'/healthz',timeout=2)
                            value=response.json();services[name]=dict(ready=response.ok and value.get('weights_loaded') is True,details=value)
                        except Exception:services[name]=dict(ready=False,message_zh='服务未启动或模型未加载')
                    cfg=model_config() if model_config else {}
                    from .cap.codex_model import availability
                    return self.send(200,dict(perception=services,selected_model=model_choice(),codex=availability(),model=dict(model=cfg.get('model'),base_url=cfg.get('base_url'),reasoning=cfg.get('reasoning'),key_loaded=bool(os.environ.get(cfg.get('api_key_env','ROBOT_MODEL_API_KEY'))))))
                if route=='/api/hardware/session':return self.send(200,dict(
                    session_id=os.environ.get('ROBOL2L_MANAGED_SESSION'),
                    launcher_url=os.environ.get('ROBOL2L_LAUNCHER_URL','http://127.0.0.1:8770'),
                    backend=runtime.adapter.describe().get('robot_backend','mock'),
                    busy=runtime.describe()['busy'],closed=runtime.closed,
                    hardware_lifecycle=(runtime.adapter.hardware_lifecycle() if hasattr(runtime.adapter,'hardware_lifecycle') else None),
                    cap_running=bool((cap.thread and cap.thread.is_alive()) or experiment.active())))
                if route=='/api/hardware/evidence':
                    path=runtime.directory/'events.jsonl'
                    events=[json.loads(line) for line in path.read_text().splitlines()[-100:]] if path.exists() else []
                    return self.send(200,dict(events=events,artifacts=[str(p.relative_to(runtime.directory)) for p in artifact_paths()]))
                query=parse_qs(parsed.query)
                previews = getattr(runtime.adapter, 'previews', None)
                if previews and route == '/api/hardware/previews':
                    return self.send(200, previews.describe())
                if previews and route == '/api/hardware/previews/frame':
                    data = previews.jpeg(query['role'][0], query['source'][0])
                    return self.send(200, data, 'image/jpeg')
                if route=='/api/hardware/artifact':
                    path=(runtime.directory/query['path'][0]).resolve()
                    if path not in [p.resolve() for p in artifact_paths()]:raise ValueError('artifact_not_registered')
                    return self.send(200,path.read_bytes(),{'.png':'image/png','.npz':'application/octet-stream','.json':'text/plain'}[path.suffix])
                if route in ('/api/hardware/stream','/api/hardware/image'):
                    name=query['camera'][0];depth=query.get('depth',['0'])[0]=='1'
                    def encode():
                        if route.endswith('/image'):
                            stamp,obs,binding=runtime.observations[query['observation'][0]]
                            cam=obs.cameras[name];rgb,d=cam.rgb,cam.depth_m
                        else:
                            f=runtime.adapter.cameras[name].latest();rgb,d=f.rgb,f.depth_m
                        bgr=cv2.applyColorMap(np.uint8(np.clip(d/2,0,1)*255),cv2.COLORMAP_TURBO) if depth else cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
                        ok,data=cv2.imencode('.jpg',bgr)
                        if not ok:raise ValueError('image_encode_failed')
                        return data.tobytes()
                    first=encode()
                    if route.endswith('/image'):return self.send(200,first,'image/jpeg')
                    self.send_response(200);self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame');self.send_header('Cache-Control','no-store');self.end_headers()
                    try:
                        data=first
                        while not runtime.closed:
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(data)).encode()+b'\r\n\r\n'+data+b'\r\n');self.wfile.flush()
                            time.sleep(.1);data=encode()
                    except (BrokenPipeError,ConnectionError,TimeoutError,RuntimeError):pass
                    return
                return self.send(404,{'error':'not_found'})
            except Exception as error:
                from .localization import chinese_error
                return self.send(409,{'error':str(error),'message_zh':chinese_error(error)})
        def do_POST(self):
            route=urlparse(self.path).path
            if not route.startswith('/api/hardware/'):return super().do_POST()
            if not self.trusted() or self.headers.get('X-Console-Request')!='1':return self.send(403,{'error':'origin_rejected'})
            session_id=os.environ.get('ROBOL2L_MANAGED_SESSION')
            if session_id and self.headers.get('X-Session-Id')!=session_id:
                return self.send(409,{'error':'session_changed; reopen the control panel'})
            control_lock.acquire()
            try:
                length=int(self.headers.get('Content-Length','0'))
                if length<=0:raise ValueError('invalid_body_size')
                body=json.loads(self.rfile.read(length))
                if route.startswith('/api/hardware/manual-positioning/'):
                    require_arx()
                    action = route.rsplit('/', 1)[-1]
                    if action == 'hold' and body == {}:
                        value = teach.hold()
                    else:
                        operator.idle()
                        if experiment.busy() or teach.token:
                            raise ValueError('experiment_or_teaching_active')
                        if action == 'start' and set(body) == {'side'}:
                            with runtime.lease():
                                if runtime.faulted or runtime.paused:
                                    raise ValueError('station_faulted_or_paused')
                            value = teach.prepare(body['side'], delayed=True)
                        elif (action in ('recheck-hold', 'acknowledge-parked')
                              and set(body) == {'side', 'confirmed'} and body['confirmed'] is True):
                            value = teach.acknowledge_parked(body['side'], recheck_hold=action=='recheck-hold')
                        else:
                            raise ValueError('invalid_manual_positioning_request')
                    experiment.record_manual_positioning(action, body)
                    return self.send(200, value)
                if route.startswith('/api/hardware/gripper-calibration/'):
                    require_arx()
                    operator.idle()
                    if experiment.active() or teach.token:
                        raise ValueError('experiment_or_teaching_active')
                    action = route.rsplit('/', 1)[-1]
                    if action == 'jog' and set(body) in ({'side','delta_rad','request_id'}, {'side','target_rad','request_id'}):
                        args = {key: value for key, value in body.items() if key != 'request_id'}
                        return self.send(202, runtime._submit(body['request_id'], 'calibrate_gripper_jog', args,
                            invoke=lambda inputs: hands.jog(**inputs)))
                    with runtime.lease():
                        if action == 'acknowledge' and set(body) == {'side','confirmed'} and body['confirmed'] is True:
                            return self.send(200, hands.acknowledge(body['side']))
                        if action == 'read' and set(body) == {'side'}:
                            _, measured = hands.parked(body['side'])
                            return self.send(200, dict(measured_rad=measured, samples=hands.samples))
                        if action == 'record' and set(body) == {'side','endpoint'}:
                            value = hands.record(**body)
                            if value['calibrated']:
                                operator.tools = operator.api.tools()
                                operator.api.environment.invalidate()
                                operator.references.clear()
                            return self.send(200, value)
                        if action == 'discard' and set(body) == {'side'}:
                            hands.check(body['side'])
                            hands.samples.pop(body['side'], None)
                            runtime.event('gripper_draft_discarded', side=body['side'])
                            return self.send(200, dict(side=body['side'], discarded=True))
                    raise ValueError('invalid_gripper_calibration_request')
                if route.startswith('/api/hardware/calibration/'):
                    require_arx()
                    action = route.rsplit('/', 1)[-1]
                    if action in ('heartbeat', 'release') and set(body) == {'token'}:
                        teach.signal(body['token'], action == 'release')
                        return self.send(200, teach.status())
                    operator.idle()
                    if experiment.active() or teach.token:
                        raise ValueError('experiment_or_teaching_active')
                    with runtime.lease():
                        if action == 'start' and set(body) == {'board'}:
                            value = calibration.start(body['board'])
                        elif action == 'capture' and set(body) == {'side', 'id'}:
                            value = calibration.capture(body['side'], body['id'])
                        elif action == 'solve' and set(body) == {'id'}:
                            value = calibration.calculate(body['id'])
                        elif action in ('acknowledge-parked','recheck-hold') and set(body) == {'id','side','confirmed'} and body['confirmed'] is True:
                            calibration.checked(body['id'])
                            value = teach.acknowledge_parked(body['side'], recheck_hold=action=='recheck-hold')
                        elif action == 'prepare' and set(body) == {'side', 'id', 'confirmed'} and body['confirmed'] is True:
                            calibration.checked(body['id'])
                            if runtime.faulted or runtime.paused:
                                raise ValueError('station_faulted_or_paused')
                            # Start the key worker only after releasing this preflight lease.
                            value = None
                        else:
                            raise ValueError('invalid_calibration_request')
                    if action == 'prepare':
                        value = teach.prepare(body['side'])
                    return self.send(200, value)
                if teach.token and route != '/api/hardware/pause':
                    raise ValueError('手动摆位中，请先锁定当前位置 / Finish manual positioning first')
                # Keep human/manual dispatch and the two agent runners mutually exclusive.
                if experiment.busy() and route not in ('/api/hardware/experiment/pause','/api/hardware/pause'):
                    raise ValueError('experiment_busy')
                if route.startswith('/api/hardware/experiment/'):
                    action=route.rsplit('/',1)[-1]
                    with control_lock:
                        operator.idle()
                        if action=='settings':
                            if experiment.active():raise ValueError('end_experiment_before_changing_model')
                            settings=profiles.save(body)
                            runtime.event('experiment_model_settings_saved',settings=settings)
                            return self.send(200,settings)
                        if action=='delete-profile':
                            if experiment.active():raise ValueError('end_experiment_before_changing_model')
                            settings=profiles.delete(body)
                            runtime.event('experiment_model_profile_deleted',settings=settings)
                            return self.send(200,settings)
                        if action=='skills' and set(body)=={'name','archive_base64'}:
                            if experiment.active():raise ValueError('end_experiment_before_changing_skills')
                            import base64,binascii
                            try:data=base64.b64decode(body['archive_base64'],validate=True)
                            except (binascii.Error,ValueError):raise ValueError('invalid_skill_library_upload')
                            value=skills.install(body['name'],data)
                            runtime.event('experiment_skill_library_installed',
                                          name=value['name'],file_count=value['file_count'],bytes=value['bytes'])
                            return self.send(200,value)
                        if action=='skills-remove' and set(body)<={'confirmed','name'} and body.get('confirmed') is True:
                            if experiment.active():raise ValueError('end_experiment_before_changing_skills')
                            value=skills.remove(body.get('name'))
                            runtime.event('experiment_skill_library_removed',name=body.get('name'))
                            return self.send(200,value)
                        if action=='skills-select' and set(body)=={'name'}:
                            if experiment.active():raise ValueError('end_experiment_before_changing_skills')
                            value=skills.select(body['name'])
                            runtime.event('experiment_skill_library_selected',name=value['active'])
                            return self.send(200,value)
                        if action=='records-delete' and set(body)=={'id','confirmed'}:
                            value=experiment.delete_record(body['id'],body['confirmed'])
                            runtime.event('experiment_record_deleted',experiment_id=body['id'])
                            return self.send(200,value)
                        if action=='records-delete-many' and set(body)=={'ids','confirmed'}:
                            value=experiment.delete_records(body['ids'],body['confirmed'])
                            runtime.event('experiment_records_deleted',count=len(body['ids']))
                            return self.send(200,value)
                        if action=='records-tag' and set(body)<={'ids','tags','confirmed'}:
                            value=experiment.tag_records(body['ids'],body.get('tags',[]),body.get('confirmed',True))
                            runtime.event('experiment_records_tagged',count=len(body['ids']),tags=body.get('tags',[]))
                            return self.send(200,value)
                        if action=='presets':
                            return self.send(200,presets.save(body))
                        if action=='presets-use' and set(body)=={'id'}:
                            return self.send(200,presets.touch(body['id']))
                        if action=='presets-delete':
                            return self.send(200,presets.delete(body))
                        if action=='preview' and set(body)<={'phase','goal','criteria'}:
                            return self.send(200,experiment.preview(**body))
                        if action=='start' and set(body)<={'goal','criteria','confirmed','speed_factor','record_cameras'} and {'goal','criteria','confirmed'}<=set(body):
                            if runtime.describe()['busy']:raise ValueError('station_busy')
                            return self.send(202,experiment.start(**body))
                        if action=='authorize-reset' and set(body)=={'session_id','confirmed','note','expected_attempt'}:
                            if runtime.describe()['busy']:raise ValueError('station_busy')
                            return self.send(202,experiment.authorize_reset(**body))
                        if action=='retry-review' and set(body)=={'session_id','expected_attempt'}:
                            if runtime.describe()['busy']:raise ValueError('station_busy')
                            return self.send(202,experiment.retry_review(**body))
                        if action=='confirm-reset' and set(body)=={'session_id','confirmed','note','expected_attempt'}:
                            if runtime.describe()['busy']:raise ValueError('station_busy')
                            return self.send(202,experiment.confirm_reset(**body))
                        if action=='return' and set(body)=={'session_id','confirmed'}:
                            if runtime.describe()['busy']:raise ValueError('station_busy')
                            return self.send(202,experiment.return_to_initial(**body))
                        if action=='pause' and set(body)=={'session_id'}:
                            experiment.pause(**body);return self.send(200,experiment.snapshot())
                        if action=='end' and set(body)=={'session_id'}:
                            experiment.end(**body);return self.send(200,experiment.snapshot())
                        raise ValueError('invalid_experiment_request')
                if route=='/api/hardware/cap/model' and set(body)=={'mode'}:
                    if body['mode'] not in ('codex','configured','fixture'):raise ValueError('model_not_configured')
                    from .cap.storage import write_json
                    with control_lock:
                        write_json(model_choice_path,body)
                        runtime.event('cap_model_selected',mode=body['mode'])
                    return self.send(200,body)
                if route == '/api/hardware/previews/bind' and set(body) == {'role', 'source'}:
                    previews = getattr(runtime.adapter, 'previews', None)
                    if previews is None:
                        raise ValueError('preview_unavailable')
                    result = previews.bind(body['role'], body['source'])
                    runtime.event('preview_binding', **body)
                    return self.send(200, result)
                if route=='/api/hardware/manual/action' and set(body)=={'request_id','action','arguments'}:
                    with control_lock:return self.send(202,operator.manual_action(**body))
                if route=='/api/hardware/operator/call' and set(body)=={'request_id','operation','arguments'}:
                    with control_lock:return self.send(202,operator.submit(**body))
                if route=='/api/hardware/operator/authorize' and set(body)=={'reason','probe_distance_m','confirmed'} and body['confirmed'] is True:
                    with control_lock:return self.send(200,operator.authorize(body['reason'],body['probe_distance_m']))
                if route=='/api/hardware/operator/revoke' and body=={}:
                    return self.send(200,operator.revoke())
                if (route=='/api/hardware/session/end' and body.get('confirmed') is True
                        and set(body) <= {'confirmed','hardware_supported'}):
                    with control_lock:
                        if experiment.active():raise ValueError('end_experiment_before_disconnecting')
                        operator.idle()
                        with runtime.lease():
                            supported=body.get('hardware_supported') is True
                            # Block before changing session state when a legacy direct owner is still live.
                            lifecycle=getattr(runtime.adapter,'hardware_lifecycle',lambda: {})()
                            if lifecycle.get('disconnect_requires_support_confirmation') and not supported:
                                raise ValueError('hardware_support_confirmation_required')
                            operator.revoke()
                            if hasattr(runtime.adapter,'prepare_disconnect'):
                                if supported:runtime.event('operator_confirmed_hardware_supported_for_shutdown')
                                runtime.adapter.prepare_disconnect(hardware_supported=supported)
                            runtime.closed=True
                            runtime.event('operator_end_session')
                        self.send(200,{'status':'closing'})
                        threading.Thread(target=self.server.shutdown,daemon=True).start()
                        return
                if route=='/api/hardware/cap/start' and body.get('task') in ('probe','pick-place','joint'):
                    expected = {'model_mode','task','authorization','probe_distance_m','confirmed'}
                    if body['task'] == 'joint':
                        expected.add('goal')
                        if runtime.adapter.describe().get('robot_backend') != 'arx_x5_2023_dual':
                            raise ValueError('dual_arx_station_required')
                    if set(body)!=expected or body['confirmed'] is not True:
                        raise ValueError('explicit_physical_task_authorization_required')
                    if runtime.adapter.synthetic or body['model_mode']=='fixture':raise ValueError('physical_task_requires_live_adapter_and_real_model')
                    from .cap.physical_task import DualJointTask,PhysicalProbeTask,PhysicalPickPlaceTask
                    task=DualJointTask(body['goal']) if body['task']=='joint' else PhysicalProbeTask(body['probe_distance_m']) if body['task']=='probe' else PhysicalPickPlaceTask()
                    model=make_model(body['model_mode'])
                    return self.send(202,run_cap(model,task,authorization=body['authorization'],
                        probe_distance_m=body['probe_distance_m'] if body['task']=='probe' else .004))
                if route=='/api/hardware/cap/start' and set(body) in ({'model_mode'},{'model_mode','task'}):
                    from .cap.model import FeedbackFixture,ConfiguredModel
                    from .cap.tasks import MockReachTask,VisualPreparationTask
                    task_name=body.get('task','mock')
                    if task_name not in ('mock','visual'):raise ValueError('unknown_cap_task')
                    if task_name=='visual' and (body['model_mode']=='fixture' or runtime.adapter.synthetic):
                        raise ValueError('visual_preparation_requires_real_model_and_physical_observation')
                    model=make_model(body['model_mode'])
                    return self.send(202,run_cap(model,VisualPreparationTask() if task_name=='visual' else MockReachTask()))
                if route=='/api/hardware/cap/continue' and set(body)=={'session','model_mode','message'}:
                    from .cap.continuation import task_from_state
                    previous=cap.stored_session(body['session'])
                    if previous['synthetic']!=runtime.adapter.synthetic:raise ValueError('cap_task_mismatch')
                    if not isinstance(body['message'],str):raise ValueError('invalid_operator_message')
                    task=task_from_state(previous)
                    physical=not runtime.adapter.synthetic and not getattr(task,'read_only',False)
                    if not runtime.adapter.synthetic and body['model_mode']=='fixture':raise ValueError('physical_task_requires_live_adapter_and_real_model')
                    return self.send(202,run_cap(make_model(body['model_mode']),task,
                        authorization='Human clicked Continue current task in the panel' if physical else None,
                        probe_distance_m=task.context.metadata.get('distance_m',.004),
                        continuation=body['session'],message=body['message']))
                if route=='/api/hardware/cap/pause' and body=={}:
                    cap.pause();return self.send(200,cap.snapshot())
                if route=='/api/hardware/cap/prepare' and body=={}:
                    from .cap.tasks import prepare_pick_place
                    from .cap.storage import write_json
                    prepared=prepare_pick_place()
                    write_json(runtime.directory/'prepared-pick-place.json',prepared)
                    return self.send(200,prepared)
                if route=='/api/hardware/jobs' and set(body)=={'request_id','operation','arguments'}:
                    with control_lock:
                        operator.idle()
                        return self.send(202,runtime.submit(**body))
                if route=='/api/hardware/pause' and set(body)=={'paused'} and type(body['paused']) is bool:
                    runtime.pause(body['paused']);return self.send(200,runtime.snapshot())
                if route=='/api/hardware/unlock' and body=={}:
                    with control_lock:
                        receipt=runtime.unlock();experiment.record_unlock(receipt)
                        return self.send(200,public(receipt))
                if route=='/api/hardware/localize' and set(body)=={'observation_id','camera_name','pixel'}:
                    runtime.event('operator_pixel_measurement',**body,autonomy_evidence=False)
                    return self.send(200,public(runtime.localize_pixel(**body)))
                if route=='/api/hardware/measure' and set(body)=={'observation_id','camera_name','box'}:
                    runtime.event('operator_region_measurement',**body,autonomy_evidence=False)
                    return self.send(200,public(runtime.measure_region(**body)))
                raise ValueError('invalid_request')
            except Exception as error:
                from .localization import chinese_error
                return self.send(409,{'error':str(error),'message_zh':chinese_error(error)})
            finally:
                control_lock.release()
    return Handler

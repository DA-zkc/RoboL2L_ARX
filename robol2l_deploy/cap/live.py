"""One explicitly authorized physical experiment; no automatic restart/resume."""
import argparse
from pathlib import Path
import threading
from uuid import uuid4
from franka_player.core import Console
from franka_player.server import Server,handler
from ..adapters import STATION_BACKENDS, make_station_adapter
from ..runtime import StationRuntime
from ..web import hardware_handler
from ..physical import authorize,revoke
from .codex_model import CodexModel
from .session import CapController
from .physical_task import DualJointTask,PhysicalPickPlaceTask,PhysicalProbeTask


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--authorize-physical-task',action='store_true',required=True)
    parser.add_argument('--authorization',required=True)
    parser.add_argument('--station',default='config/station.json')
    parser.add_argument('--robot-backend',choices=STATION_BACKENDS,default='panda_py')
    parser.add_argument('--codex-model',required=True)
    parser.add_argument('--codex-bin',default='codex')
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--output',default='.franka-player/physical-cap')
    parser.add_argument('--task',choices=('pick-place','probe','joint'),default='pick-place')
    parser.add_argument('--probe-distance-m',type=float,choices=(.004,.02),default=.004)
    parser.add_argument('--goal', help='Natural-language goal for the dual-arm joint task')
    args=parser.parse_args()
    if args.robot_backend == 'arx_x5_2023_dual' and args.task != 'joint':
        parser.error('the encoder-only dual station supports --task joint --goal TEXT')
    if args.task == 'joint' and (args.robot_backend != 'arx_x5_2023_dual' or not (args.goal or '').strip()):
        parser.error('--task joint requires the dual ARX backend and a nonempty --goal')
    if not args.authorization.strip():
        parser.error('--authorization requires a nonempty description')
    adapter = make_station_adapter(args.station, args.robot_backend, allow_connection=True)
    if args.robot_backend == 'arx_x5_2023':
        if not adapter.cameras:
            parser.error('the ARX live CaP entry requires calibrated RGB-D cameras in the station file')
        if args.task == 'pick-place' and adapter.gripper is None:
            parser.error('ARX pick-place requires calibrated gripper endpoints in the station file')
    root=Path(args.output)/uuid4().hex;root.mkdir(parents=True)
    runtime=StationRuntime(adapter,root/'hardware')
    console=Console(root/'console');cap=CapController(runtime)
    server=Server(('127.0.0.1',args.port),hardware_handler(handler(console),runtime,cap_controller=cap))
    try:
        # Acquire the installed station observation before enabling the session.
        runtime.get_observation()
        model=CodexModel(args.codex_model,executable=args.codex_bin)
        if args.task == 'joint':
            task = DualJointTask(args.goal)
        else:
            task = PhysicalProbeTask(args.probe_distance_m) if args.task == 'probe' else PhysicalPickPlaceTask()
        authorize(runtime,task.context.task_name,args.authorization,
            probe_distance_m=args.probe_distance_m if args.task=='probe' else .004,
            require_probe=False)
        cap.start(model,task)
        def disable_after_session():
            cap.thread.join();revoke(runtime)
        threading.Thread(target=disable_after_session,daemon=True).start()
        print('Physical CaP session ('+args.robot_backend+'): '+str(root.resolve()),flush=True)
        print('Workbench: http://127.0.0.1:'+str(args.port)+'/hardware',flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        cap.pause()
        if cap.thread:cap.thread.join()
    finally:
        revoke(runtime);server.server_close();runtime.close();console.shutdown()


if __name__=='__main__':main()

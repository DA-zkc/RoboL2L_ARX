import argparse
import json
import contextlib
import ctypes
import os
import sys
from pathlib import Path
from .sdk import CapSDK
from .program import Program
from .common import write_json

@contextlib.contextmanager
def diagnostics_to_stderr():
    # Native model loaders may write directly to fd 1; keep CLI stdout valid JSON.
    sys.stdout.flush();saved=os.dup(1)
    try:
        os.dup2(2,1)
        yield
    finally:
        sys.stdout.flush();ctypes.CDLL(None).fflush(None)
        os.dup2(saved,1);os.close(saved)

def main():
    parser=argparse.ArgumentParser(description='Independent Franka CAP SDK: physical observation and offline planning; no motion')
    parser.add_argument('--station-config')
    commands=parser.add_subparsers(dest='command',required=True)
    for name in ('describe','state','observe'):commands.add_parser(name)
    run=commands.add_parser('run');run.add_argument('program',type=Path)
    check=commands.add_parser('check');check.add_argument('program',type=Path)
    args=parser.parse_args()
    if args.command=='check':
        Program(None).validate(args.program.read_text());print('Program syntax/allowlist passed; no hardware accessed');return
    with CapSDK(args.station_config) as api:
        try:
            with diagnostics_to_stderr():
                if args.command=='describe':result=api.describe()
                elif args.command=='state':result=api.get_robot_state()
                elif args.command=='observe':result=api.get_observation()
                else:
                    source=args.program.read_text();(api.run_dir/'program.py').write_text(source)
                    result=Program(api).run(source)
            report={'status':'passed','result':result,'run_directory':str(api.run_dir),'motion_sent':False}
            write_json(api.run_dir/'result.json',report)
            print(json.dumps(report,ensure_ascii=False,indent=2))
        except Exception as error:
            write_json(api.run_dir/'result.json',{'status':'failed','error':str(error),'motion_sent':False})
            raise SystemExit(f'{error}\nReport: {api.run_dir / "result.json"}') from None

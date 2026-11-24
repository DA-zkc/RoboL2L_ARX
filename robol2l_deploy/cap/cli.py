"""Mock CaP validation and preparation-only physical task export."""
import argparse
import getpass
import json
import os
import sys
from pathlib import Path
from uuid import uuid4
from ..runtime import StationRuntime
from ..adapters.mock import MockAdapter
from .model import FeedbackFixture,ConfiguredModel
from .session import CapSession
from .tasks import MockReachTask,prepare_pick_place
from .storage import write_json


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='command',required=True)
    mock=commands.add_parser('mock')
    visual=commands.add_parser('visual',help='Real-model read-only preparation from RGB-D; no motion')
    for command in (mock,visual):
        command.add_argument('--output',default='.franka-player/cap-mock')
        command.add_argument('--max-turns',type=int,default=6)
        modes=command.add_mutually_exclusive_group()
        modes.add_argument('--model-config',help='JSON containing base_url/model/reasoning/api_key_env; no secret')
        modes.add_argument('--openai-model',help='Use this model through https://api.openai.com/v1 only')
        modes.add_argument('--codex-model',help='Use installed Codex CLI with its existing ChatGPT login; no API key')
        command.add_argument('--codex-bin',help='Official Codex CLI executable; defaults to the workspace installation, then PATH')
        command.add_argument('--validate',action='store_true',help='Run independent model-generation and seeded-repair cases')
        command.add_argument('--prompt-key',action='store_true',help='Hidden terminal input into process environment only; for --openai-model')
        command.add_argument('--reasoning',default='medium',choices=['none','low','medium','high','xhigh'])
    visual.set_defaults(output='.franka-player/cap-visual')
    visual.add_argument('--observation')
    visual.add_argument('--station',default='config/station.json')
    visual.add_argument('--live-url')
    visual.add_argument('--station-directory')
    prep=commands.add_parser('prepare-pick-place');prep.add_argument('--output',default='.franka-player/prepared-pick-place.json')
    args=parser.parse_args(argv)
    if args.command=='prepare-pick-place':
        path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True)
        write_json(path,prepare_pick_place());print(str(path.resolve()));return 0
    if args.prompt_key and not args.openai_model:parser.error('--prompt-key requires --openai-model')
    if args.codex_model:
        from .codex_model import CodexModel
        model=CodexModel(args.codex_model,reasoning=args.reasoning,executable=args.codex_bin)
    elif args.openai_model:
        if args.prompt_key:
            if not sys.stdin.isatty():parser.error('--prompt-key requires an interactive terminal')
            # Never put the key in a command argument, file, UI form or transcript.
            os.environ['OPENAI_API_KEY']=getpass.getpass('OpenAI API key (hidden, process memory only): ')
        model=ConfiguredModel(dict(base_url='https://api.openai.com/v1',model=args.openai_model,
                                   reasoning=args.reasoning,api_key_env='OPENAI_API_KEY'))
    elif args.model_config:
        config=json.loads(Path(args.model_config).read_text())
        if set(config)!={'base_url','model','reasoning','api_key_env'}:parser.error('model config must contain only the four nonsecret fields')
        model=ConfiguredModel(config)
    else:
        if args.command=='visual':parser.error('visual requires an explicit real model; fixtures are not accepted')
        model=FeedbackFixture()
    if isinstance(model,ConfiguredModel) and not os.environ.get(model.config['api_key_env']):parser.error('model API key environment variable is missing')
    folder=Path(args.output)/uuid4().hex
    if args.command=='visual':
        from .visual import run_visual
        report=run_visual(model,folder,observation_directory=args.observation,station_config=args.station,
            live_url=args.live_url,station_directory=args.station_directory,max_turns=args.max_turns)
        print(json.dumps(dict(report,evidence=str(folder.resolve())),ensure_ascii=False,indent=2))
        return 0 if report['status']=='completed' else 1
    if args.validate:
        if not (args.codex_model or args.openai_model or args.model_config):
            parser.error('--validate requires an explicit real-model provider')
        from .validation import run_validation
        try:
            report=run_validation(model,folder,max_turns=args.max_turns)
            print(json.dumps(dict(report,evidence=str(folder.resolve())),ensure_ascii=False,indent=2))
            return 0 if report['passed'] else 1
        finally:
            if args.prompt_key:os.environ.pop('OPENAI_API_KEY',None)
    runtime=StationRuntime(MockAdapter(),folder/'runtime',allow_motion=True)
    try:
        session=CapSession(runtime,MockReachTask(),model,folder/'session',max_turns=args.max_turns)
        state=session.run()
        print(json.dumps(dict(status=state['status'],model=state['model'],synthetic=True,
                             physical_task_success=False,versions=state['versions'],verification=state['verification'],
                             evidence=str(folder.resolve())),ensure_ascii=False,indent=2))
        return 0 if state['status']=='completed' else 1
    finally:
        runtime.close()
        if args.prompt_key:os.environ.pop('OPENAI_API_KEY',None)


if __name__=='__main__':raise SystemExit(main())

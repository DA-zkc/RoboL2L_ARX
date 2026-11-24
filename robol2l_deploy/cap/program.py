"""Source/version evidence and the only bridge from interpreted code to Runtime."""
from dataclasses import fields,is_dataclass
from collections.abc import Mapping
from enum import Enum
from pathlib import Path
import hashlib
import time
from uuid import uuid4
import numpy as np
from ..runtime import OutcomeUnknown, public
from .api import PublicAPI, CONSTRUCTORS, MOTION



def compact(value, depth=0):
    """Bounded JSON feedback, preserving public errors without dumping image arrays."""
    from ..errors import ApiError
    if isinstance(value,ApiError):
        from ..localization import chinese_error
        return dict(code=value.code.value,message=value.message,message_zh=chinese_error(value.message,value.details),recoverable=value.recoverable,details=compact(value.details,depth+1))
    if isinstance(value,Enum):return value.value
    if isinstance(value,np.ndarray):
        if value.size<=24:return value.tolist()
        return {'shape':list(value.shape),'dtype':str(value.dtype),'omitted':'array stored in runtime artifacts'}
    if isinstance(value,np.generic):return value.item()
    if is_dataclass(value) and not isinstance(value,type):return {f.name:compact(getattr(value,f.name),depth+1) for f in fields(value)}
    if isinstance(value,Mapping):return {str(k):compact(v,depth+1) for k,v in value.items()}
    if isinstance(value,(tuple,list,range)):return [compact(v,depth+1) for v in value]
    if isinstance(value,float) and not np.isfinite(value):return '<non-finite>'
    if value is None or isinstance(value,(bool,int,float)):return value
    if isinstance(value,str):return value
    return '<unsupported public value>'


class ProgramExecutor:
    """Run the upstream executor in an isolated process; dispatch in the owner."""
    def __init__(self,runtime,task_context,*,max_calls=None,max_steps=None,timeout_s=None):
        self.runtime=runtime;self.api=PublicAPI(runtime,task_context)
        self.max_calls=max_calls;self.max_steps=max_steps;self.timeout_s=timeout_s

    def execute(self,source,directory,*,should_stop=lambda:False):
        import json, os, queue, subprocess, sys, threading
        from .wire import encode, decode
        from .storage import write_json
        directory=Path(directory);directory.mkdir(parents=True,exist_ok=False)
        if isinstance(source,str):(directory/'source.py').write_text(source)
        calls=[];tools=self.api.tools();synthetic=self.runtime.adapter.synthetic
        report={'ok':False,'result':None,'stdout':'','calls':calls,'error':None,
            'source_sha256':hashlib.sha256(source.encode()).hexdigest() if isinstance(source,str) else None,
            'synthetic':synthetic,'physical_task_success':False,'started_s':time.time(),
            'observation_id':next(reversed(self.runtime.observations),None), 'executor':'ASPIRE.ProgramExecutor'}
        process=None
        try:
            if not isinstance(source,str):raise ValueError('invalid_program_source')
            with self.runtime.program_lease(should_stop):
                environment={k:v for k,v in os.environ.items() if k in ('PATH','LANG','LC_ALL')}
                environment.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
                process=subprocess.Popen([sys.executable,'-m','robol2l_deploy.cap.worker'],
                    stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,env=environment,
                    cwd=Path(__file__).resolve().parents[2])
                def send(value):
                    process.stdin.write(json.dumps(value,allow_nan=False)+'\n');process.stdin.flush()
                send({'source':source,'tools':list(tools),'max_steps':self.max_steps})
                incoming=queue.Queue()
                def read():
                    while True:
                        line=process.stdout.readline()
                        incoming.put(line)
                        if not line:break
                reader=threading.Thread(target=read,daemon=True);reader.start()
                waiting_since=time.monotonic()
                last_state_read = None
                while True:
                    try:line=incoming.get(timeout=.2)
                    except queue.Empty:
                        if should_stop() or self.runtime.paused:raise RuntimeError('paused_before_dispatch')
                        if self.timeout_s is not None and time.monotonic()-waiting_since>self.timeout_s:raise TimeoutError('program_python_timeout')
                        continue
                    if not line:raise RuntimeError('worker_exited_without_result')
                    message=json.loads(line)
                    if message['kind']=='done':
                        report.update(ok=message['ok'],result=compact(decode(message['result'])),stdout=message['stdout'],error=message['error'])
                        break
                    name=message['name'];trace={'id':uuid4().hex,'tool':name,'line':message.get('line'),
                        'started_s':time.time(),'ok':False,'observation_id':self.api.environment.observation_id or report['observation_id']}
                    stop=None
                    if should_stop() or self.runtime.paused:stop='paused_before_dispatch'
                    elif self.max_calls is not None and len(calls)>=self.max_calls:stop='api_call_budget_exceeded'
                    elif self.runtime.faulted:stop='runtime_fault_latched'
                    elif name in MOTION and not synthetic and not (self.runtime.physical_session and self.runtime.physical_session.permits(self.api.task_context.task_name)):
                        stop='physical_motion_not_enabled_for_cap'
                    if stop:
                        send({'kind':'stop','message':stop});continue
                    if name not in tools:raise ValueError('unregistered_tool')
                    args,kwargs=decode(message['args']),decode(message['kwargs'])
                    trace['inputs']=compact({'args':args,'kwargs':kwargs});calls.append(trace)
                    try:
                        value=tools[name](*args,**kwargs)
                        # The wire contains only public types; execution receipts keep
                        # unknown outcomes distinct without revealing private diagnostics.
                        if hasattr(value,'public_view'):value=value.public_view()
                        trace.update(ok=bool(getattr(value,'ok',True)),output=compact(value),observation_id=self.api.environment.observation_id or report['observation_id'])
                        send({'kind':'value','value':encode(value)})
                    except Exception as error:
                        trace['error']={'type':type(error).__name__,'message':str(error)[:2000]}
                        send({'kind':'error',**trace['error']})
                    finally:
                        trace['finished_s']=time.time();self.runtime.event('program_call',source_sha256=report['source_sha256'],call=trace)
                        waiting_since=time.monotonic()
        except Exception as error:
            report['error']={'type':type(error).__name__,'message':str(error)[:1000],
                'code':'host_stopped' if isinstance(error,(TimeoutError,RuntimeError)) else 'program_error'}
        finally:
            if process:
                # No host tool is running here: a controller is never killed for a
                # Python timeout. The owner completes/records its receipt first.
                if process.poll() is None:process.terminate()
                try:process.wait(timeout=3)
                except subprocess.TimeoutExpired:process.kill();process.wait()
                for stream in (process.stdin,process.stdout,process.stderr):stream.close()
        report.update(finished_s=time.time())
        write_json(directory/'result.json',report)
        return report

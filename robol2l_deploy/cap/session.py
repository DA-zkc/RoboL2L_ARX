"""Observe → model decision → versioned program → feedback, with user-controlled stopping."""
from pathlib import Path
import json
import threading
import time
from uuid import uuid4
from .api import PublicAPI
from .program import ProgramExecutor,compact
from .model import messages
from .storage import write_json
from .continuation import RESUMABLE, load_history


def decision(value,extra=()):
    """Validate a model decision. `extra` names optional keys a caller also accepts.

    With no `extra` the accepted shape is exactly the original one, so callers that
    do not opt in keep rejecting every unexpected key.
    """
    if not isinstance(value,dict) or value.get('kind') not in ('program','finish','assist'):raise ValueError('invalid_decision_kind')
    expected={'kind','reason','source'} if value['kind']=='program' else {'kind','reason'}
    if not expected<=set(value)<=expected|set(extra) or not isinstance(value['reason'],str) or not value['reason'].strip():
        raise ValueError('invalid_decision_fields')
    if value['kind']=='program' and (not isinstance(value['source'],str)):raise ValueError('invalid_program_source')
    for name in extra:
        if name in value and not isinstance(value[name],str):raise ValueError('invalid_experiment_record')
    return value


class CapSession:
    def __init__(self,runtime,task,model,directory,*,max_turns=None):
        if max_turns is not None and (type(max_turns) is not int or max_turns<1):raise ValueError('invalid_turn_budget')
        self.runtime=runtime;self.task=task;self.model=model;self.directory=Path(directory)
        self.directory.mkdir(parents=True,exist_ok=False)
        self.executor=ProgramExecutor(runtime,task.context);self.stop=threading.Event()
        self.max_turns=max_turns;self.history=[]
        self.state={'status':'ready','model':model.label,'synthetic':runtime.adapter.synthetic,
                    'physical_task_success':False,'task':compact(task.context),'turns':0,'versions':[],'verification':{}}
        if hasattr(model,'settings'):
            self.state['model_settings']=model.settings
        self.save()

    @classmethod
    def restore(cls,runtime,task,model,directory,message=''):
        directory=Path(directory)
        state=json.loads((directory/'session.json').read_text())
        if state['status'] not in RESUMABLE:raise ValueError('cap_not_continuable')
        if state['synthetic']!=runtime.adapter.synthetic or state['task']['task_name']!=task.context.task_name:
            raise ValueError('cap_task_mismatch')
        if runtime.faulted:raise ValueError('runtime_fault_latched')
        if not isinstance(message,str):raise ValueError('invalid_operator_message')
        self=cls.__new__(cls)
        self.runtime=runtime;self.task=task;self.model=model;self.directory=directory
        self.executor=ProgramExecutor(runtime,task.context);self.stop=threading.Event()
        self.max_turns=None;self.history=load_history(directory,state);self.state=state
        intervention=dict(time=time.time(),before_turn=state['turns'],
            message=message.strip() or '请根据新观测继续当前任务，保留已有对话和执行记录。',
            previous_status=state['status'],previous_reason=state.get('reason'),
            previous_model=state.get('model_settings',{}),model=getattr(model,'settings',{}))
        state.setdefault('continuations',[]).append(intervention)
        self.history.append({'human_intervention':intervention})
        state.update(status='ready',model=model.label,model_settings=getattr(model,'settings',{}),task=compact(task.context))
        for key in ('reason','message_zh','error_type','finished_s'):state.pop(key,None)
        self.save()
        runtime.event('cap_operator_continue',session=directory.name,**intervention)
        return self

    def save(self):
        write_json(self.directory/'history.json',self.history)
        write_json(self.directory/'session.json',self.state)

    def run(self):
        if self.state['status']!='ready':raise ValueError('session_is_not_resumable')
        self.state['status']='running';self.save()
        try:
            import itertools
            for offset,turn in enumerate(itertools.count(self.state['turns'])):
                if self.max_turns is not None and offset>=self.max_turns:
                    self.state.update(status='waiting_operator',reason='configured_turn_budget_exhausted');break
                if self.stop.is_set() or self.runtime.paused:
                    self.state['status']='paused';break
                if self.runtime.faulted:
                    self.state['status']='waiting_operator';self.state['reason']='runtime_fault_latched';break
                obs=self.executor.api.get_observation()
                verification=self.task.verify(self.runtime)
                self.state.update(turns=turn+1,verification=verification);self.save()
                request=messages(self.executor.api,obs,self.history,verification)
                # Persist textual context only; images already have runtime artifact IDs.
                write_json(self.directory/f'turn-{turn:02d}-context.json',{'system':request[0]['content'],'context':request[1]['content'][0]['text']})
                try:
                    value=decision(self.model.decide(request))
                except ValueError as error:
                    feedback={'error':{'type':'InvalidModelResponse','message':str(error)}}
                    self.history.append({'turn':turn,'feedback':feedback});write_json(self.directory/f'turn-{turn:02d}-feedback.json',feedback)
                    continue
                write_json(self.directory/f'turn-{turn:02d}-decision.json',value)
                if self.stop.is_set() or self.runtime.paused:
                    self.history.append({'turn':turn,'decision':value,'executed':False,'note':'Paused before execution; do not replay.'})
                    self.state['status']='paused';break
                if value['kind']=='assist':
                    self.history.append({'turn':turn,'decision':value})
                    self.state.update(status='waiting_operator',reason=value['reason']);break
                if value['kind']=='finish':
                    # Re-read independent evidence after the model request, not its claim.
                    verification=self.task.verify(self.runtime);self.state['verification']=verification
                    if verification.get('passed') or verification.get('required') is False:
                        self.state.update(status='completed',reason=value['reason'],physical_task_success=bool(verification.get('physical_task_success')));break
                    self.history.append({'turn':turn,'feedback':{'ok':False,'error':{'message':'当前观测尚不支持任务完成，请继续观察或修订程序。 / Task completion is not supported by observations.'}}})
                    self.save();continue
                # Preserve partial program artifacts left by an interrupted process.
                version=f'v{max((int(p.name[1:]) for p in self.directory.glob("v*") if p.is_dir() and p.name[1:].isdigit()),default=-1)+1:02d}'
                feedback=self.executor.execute(value['source'],self.directory/version,should_stop=self.stop.is_set)
                self.state['versions'].append({'version':version,'turn':turn,'source_sha256':feedback['source_sha256'],'program_ok':feedback['ok'],
                                               'feedback_path':f'{version}/result.json'})
                self.history.append({'version':version,'reason':value['reason'],'source':value['source'],'feedback':feedback})
                self.state['verification']=self.task.verify(self.runtime) if not self.runtime.faulted else {'passed':False,'reason':'runtime_fault_latched'}
                if feedback.get('error',{} ) and feedback['error'].get('code') in ('host_stopped','outcome_unknown'):
                    self.state.update(status='paused' if self.stop.is_set() or self.runtime.paused else 'waiting_operator',reason=feedback['error']['message']);break
                self.save()
        except Exception as error:
            # Provider error bodies/headers can contain secrets; do not store them.
            from ..localization import chinese_error
            reason=str(error) if str(error).startswith(('model_request_failed','api_key_missing','invalid_model_json')) else 'observation_verification_or_model_transport_failed'
            from .codex_model import CODEX_ERRORS
            if str(error) in CODEX_ERRORS:reason=str(error)
            self.state.update(status='error',reason=reason,message_zh=chinese_error(reason),error_type=type(error).__name__)
        finally:
            self.state['finished_s']=time.time();self.save()
        return self.state


class CapController:
    """Web lifecycle; one session at a time; loading old status never executes it."""
    def __init__(self,runtime):
        self.runtime=runtime;self.directory=runtime.directory/'cap';self.directory.mkdir(exist_ok=True)
        self.lock=threading.Lock();self.session=None;self.thread=None
        import json
        for path in self.directory.glob('*/session.json'):
            state=json.loads(path.read_text())
            if state['status'] in ('ready','running'):
                state.update(status='interrupted',reason='process_restarted_no_replay')
                write_json(path,state)
    def start(self,model,task):
        with self.lock:
            if self.thread and self.thread.is_alive():raise ValueError('cap_session_busy')
            if not self.runtime.adapter.synthetic and not getattr(task,'read_only',False) and not (self.runtime.physical_session and self.runtime.physical_session.permits(task.context.task_name)):
                raise ValueError('live_cap_not_enabled_use_preparation_only')
            self.session=CapSession(self.runtime,task,model,self.directory/uuid4().hex)
            self.thread=threading.Thread(target=self.session.run,daemon=True);self.thread.start()
            return {'status':'accepted','session':self.session.directory.name}
    def stored_session(self,session_id):
        if not isinstance(session_id,str) or len(session_id)!=32 or any(c not in '0123456789abcdef' for c in session_id):
            raise ValueError('cap_session_changed')
        current=self.snapshot()
        if current.get('session_id')!=session_id:raise ValueError('cap_session_changed')
        return current
    def continue_session(self,model,task,session_id,message=''):
        self.stored_session(session_id)
        with self.lock:
            if self.thread and self.thread.is_alive():raise ValueError('cap_session_busy')
            if not self.runtime.adapter.synthetic and not getattr(task,'read_only',False) and not (self.runtime.physical_session and self.runtime.physical_session.permits(task.context.task_name)):
                raise ValueError('live_cap_not_enabled_use_preparation_only')
            self.session=CapSession.restore(self.runtime,task,model,self.directory/session_id,message)
            self.thread=threading.Thread(target=self.session.run,daemon=True);self.thread.start()
            return {'status':'accepted','session':session_id,'continued':True}
    def snapshot(self):
        with self.lock:
            directory=self.session.directory if self.session else None
            if directory is None:
                archives=list(self.directory.glob('*/session.json'))
                if not archives:return {'status':'idle','physical_task_success':False}
                directory=max(archives,key=lambda path:path.stat().st_mtime).parent
            # Read the atomic persisted view rather than concurrently traversing mutable state.
            import json
            state=json.loads((directory/'session.json').read_text())
            state['session_directory']=str(directory)
            state['session_id']=directory.name
            state['recent_feedback']=[]
            for version in state['versions'][-2:]:
                folder=directory/version['version']
                state['recent_feedback'].append({'version':version['version'],
                    'source':(folder/'source.py').read_text(),
                    'feedback':json.loads((folder/'result.json').read_text())})
            from .presentation import turns, summary
            state['conversation']=turns(directory,state)
            state['presentation']=summary(state,state['conversation'],self.runtime)
            return state
    def turn_detail(self,session,turn):
        if len(session)!=32 or any(c not in '0123456789abcdef' for c in session):
            raise ValueError('invalid_cap_session')
        from .presentation import read,turn_detail
        directory=self.directory/session
        state=read(directory/'session.json')
        if not state:raise ValueError('cap_session_not_found')
        return turn_detail(directory,state,turn)
    def pause(self):
        with self.lock:
            if self.session:self.session.stop.set()
        self.runtime.event('cap_operator_pause')

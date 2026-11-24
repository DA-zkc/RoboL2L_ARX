"""Isolated ASPIRE ProgramExecutor; all tools are typed IPC proxies."""
import ctypes
import errno
import json
import math
import os
import resource
import sys
import time
import traceback
import numpy as np
from cap_harness.runtime import ProgramExecutor
from cap_harness.registry import ToolRegistry
from .wire import encode, decode


class HostStop(BaseException):
    pass


def restrict_process():
    # The worker has already imported its complete runtime. Deny opening files,
    # network sockets, subprocesses and process inspection at the syscall layer.
    # Existing stdin/stdout pipes are the only host gateway.
    lib=ctypes.CDLL('libseccomp.so.2', use_errno=True)
    lib.seccomp_init.argtypes=[ctypes.c_uint32];lib.seccomp_init.restype=ctypes.c_void_p
    lib.seccomp_syscall_resolve_name.argtypes=[ctypes.c_char_p];lib.seccomp_syscall_resolve_name.restype=ctypes.c_int
    lib.seccomp_rule_add.argtypes=[ctypes.c_void_p,ctypes.c_uint32,ctypes.c_int,ctypes.c_uint]
    lib.seccomp_load.argtypes=[ctypes.c_void_p];lib.seccomp_release.argtypes=[ctypes.c_void_p]
    context=lib.seccomp_init(0x7fff0000)
    if not context:raise RuntimeError('seccomp unavailable')
    blocked='open openat openat2 creat socket socketpair connect bind listen accept accept4 execve execveat fork vfork clone clone3 ptrace process_vm_readv process_vm_writev kill tkill tgkill unlink unlinkat rename renameat renameat2 mkdir mkdirat rmdir link linkat symlink symlinkat mount umount2 chmod fchmodat chown truncate'.split()
    for name in blocked:
        number=lib.seccomp_syscall_resolve_name(name.encode())
        if number>=0 and lib.seccomp_rule_add(context,0x00050000|errno.EPERM,number,0)!=0:
            raise RuntimeError('seccomp rule failed')
    if lib.seccomp_load(context)!=0:raise RuntimeError('seccomp load failed')
    lib.seccomp_release(context)


def main():
    source_in, protocol_out=sys.stdin,sys.stdout
    request=json.loads(source_in.readline())
    registry=ToolRegistry()
    last_error={};steps=0
    def emit(message):
        protocol_out.write(json.dumps(message,allow_nan=False)+'\n');protocol_out.flush()
    def proxy(name):
        def invoke(*args,**kwargs):
            frame=sys._getframe(1)
            while frame and frame.f_code.co_filename!='<generated-program>':frame=frame.f_back
            emit({'kind':'call','name':name,'line':frame.f_lineno if frame else None,'args':encode(args),'kwargs':encode(kwargs)})
            response=json.loads(source_in.readline())
            if response['kind']=='stop':raise HostStop(response['message'])
            if response['kind']=='error':
                cls={'ValueError':ValueError,'TypeError':TypeError}.get(response['type'],RuntimeError)
                raise cls(response['message'])
            return decode(response['value'])
        return invoke
    for name in request['tools']:
        registry.register(proxy(name),name=name,layer='shared_atomic',capability='gateway',public=True)
    executor=ProgramExecutor(registry)
    def trace(frame,event,arg):
        nonlocal steps
        if frame.f_code.co_filename=='<generated-program>':
            if event=='line':
                steps+=1
                if request['max_steps'] is not None and steps>request['max_steps']:raise HostStop('program_step_budget_exceeded')
            if event=='exception':
                last_error.update(type=arg[0].__name__,message=str(arg[1])[:2000],line=frame.f_lineno)
        return trace
    resource.setrlimit(resource.RLIMIT_FSIZE,(0,0))
    # NumPy lazily imports its numeric formatting routines on first use.
    # Initialize these while the worker is still loading its runtime.
    str(np.array([0.0]))
    repr(np.array([0.0]))
    restrict_process()
    sys.settrace(trace)
    try:
        outcome=executor.execute_program(request['source'])
        sys.settrace(None)
        error=None
        if outcome.error:
            error=dict(last_error or {'type':outcome.error.details.get('exception_type','ValueError'),'message':outcome.error.message,'line':None},code=outcome.error.code.value)
            if error['type'] in ('MemoryError','RecursionError'):error['code']='host_stopped'
        emit({'kind':'done','ok':outcome.ok,'result':encode(outcome.result),'stdout':outcome.stdout,'error':error})
    except HostStop as error:
        sys.settrace(None);emit({'kind':'done','ok':False,'result':None,'stdout':'','error':{'type':'HostStop','code':'host_stopped','message':str(error)}})
    except BaseException as error:
        sys.settrace(None);emit({'kind':'done','ok':False,'result':None,'stdout':'','error':{'type':type(error).__name__,'code':'host_stopped','message':str(error)[:500]}})


if __name__=='__main__':main()

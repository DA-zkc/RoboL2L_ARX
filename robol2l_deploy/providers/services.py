"""Start the two installed local inference services, without inheriting model keys."""
import fcntl
import os
from pathlib import Path
import socket
import subprocess
import sys
from urllib.parse import urlparse
from .perception import configuration


def ensure_local_services(*, include_grasps=True):
    config=configuration()
    if not config.get('enabled'):return
    root=Path(__file__).resolve().parents[2]
    folder=root/'.franka-player/perception';folder.mkdir(parents=True,exist_ok=True)
    for service,key in (('sam3','sam3_url'),('contact_graspnet','graspnet_url')):
        if service == 'contact_graspnet' and not include_grasps:
            continue
        url=urlparse(config.get(key,''))
        if url.hostname!='127.0.0.1' or not url.port:continue
        try:
            with socket.create_connection((url.hostname,url.port),timeout=.3):continue
        except OSError:pass
        env={key:value for key,value in os.environ.items() if key in ('PATH','HOME','LANG','LD_LIBRARY_PATH','CUDA_VISIBLE_DEVICES')}
        with (folder/(service+'.lock')).open('a') as lock, (folder/(service+'.log')).open('ab') as log:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                continue
            subprocess.Popen([sys.executable,str(root/'scripts/perception-service.py'),service],cwd=root,
                env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,
                pass_fds=(lock.fileno(),))

#!/usr/bin/env python3
"""Launch a pinned inference service only after verifying its local checkpoint."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('service',choices=['sam3','contact_graspnet'])
    parser.add_argument('--weights-manifest',type=Path,default=ROOT/'.franka-player/perception/weights.json')
    args=parser.parse_args()
    lock=json.loads((ROOT/'config/perception-lock.json').read_text())
    spec=lock[args.service]
    weights=json.loads(args.weights_manifest.read_text())[args.service]
    checkpoint=Path(weights['path']).resolve(strict=True)
    with checkpoint.open('rb') as stream:
        digest=hashlib.file_digest(stream,'sha256').hexdigest()
    if digest!=spec['weights_sha256'] or digest!=weights['sha256']:
        raise ValueError('checkpoint_hash_mismatch')
    vendor=ROOT/'vendor'/spec['vendor']
    commit=subprocess.check_output(['git','-C',str(vendor),'rev-parse','HEAD'],text=True).strip()
    if commit!=spec['commit']:raise ValueError('model_source_version_mismatch')
    environment=dict(os.environ,CAP_HARNESS_VENDOR_ROOT=str(ROOT/'vendor'),CAP_HARNESS_SERVICE_CONCURRENCY='1')
    if args.service=='sam3':environment['SAM3_CHECKPOINT']=str(checkpoint)
    elif checkpoint!=vendor/'checkpoints/contact_graspnet/checkpoints/model.pt':
        raise ValueError('graspnet_checkpoint_must_match_vendor_checkpoint_location')
    python=ROOT/'.franka-player/venv-perception/bin/python'
    sam3_python=ROOT/'.franka-player/venv-sam3/bin/python'
    if args.service == 'sam3' and sam3_python.is_file():
        python=sam3_python
    os.chdir(ROOT)
    service_args=[]
    if args.service=='sam3':
        config_path=Path(environment.get('ROBOL2L_PERCEPTION_CONFIG',ROOT/'config/perception.json'))
        config=json.loads(config_path.read_text()) if config_path.exists() else {}
        threshold=config.get('sam3_text_confidence_threshold',0.5)
        service_args=['--text-confidence-threshold',str(threshold)]
    print(json.dumps({'service':args.service,'commit':commit,'weights_sha256':digest,'motion':False}),flush=True)
    os.execve(python,[str(python),'-u','-m',spec['module'],'--device','cuda','--host','127.0.0.1','--port',str(spec['port']),*service_args],environment)

if __name__=='__main__':main()

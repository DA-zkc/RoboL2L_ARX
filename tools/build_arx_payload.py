"""Rebuild inertial-only URDFs OFFLINE after reviewing payload configuration.

Run from RoboL2L: PYTHONPATH=. .venv/bin/python tools/build_arx_payload.py
Does not reload a Runtime, update hand-eye calibration, or access hardware.
"""
import argparse
import json
from pathlib import Path

from robol2l_deploy.adapters.arx_payload import compose_urdf, file_sha256


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config', nargs='?', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'config/arx-x5-2023-dual-payload.json')
    args = parser.parse_args()
    path = args.config.resolve()
    cfg = json.loads(path.read_text())
    base = path.parent / cfg['base_urdf']['path']
    if file_sha256(base) != cfg['base_urdf']['sha256']:
        raise ValueError('payload_base_urdf_changed')
    generated = []
    for side, spec in cfg['sides'].items():
        data, report = compose_urdf(base.read_bytes(), spec, cfg['camera'])
        generated.append((side, path.parent / spec['urdf_path'], data, report))
    for side, model, data, report in generated:
        model.write_bytes(data)
        cfg['sides'][side]['sha256'] = file_sha256(model)
        print(json.dumps(dict(side=side, **report)))
    path.write_text(json.dumps(cfg, indent=2, ensure_ascii=False) + '\n')


if __name__ == '__main__':
    main()

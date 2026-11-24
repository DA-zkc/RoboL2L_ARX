"""Offline replay of saved SAM3 responses through current client and localization.

Never imports a hardware adapter, loads weights or contacts an HTTP service.
Keeps original RGB-D, intrinsics/extrinsics, units and saved masks unchanged.
"""
import argparse
import base64
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from cap_harness.api import CapApi
from cap_harness.providers.sam3.client import Sam3Provider
from robol2l_deploy.observations import load_observation

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_AUDIT = ROOT/'.franka-player/diagnostics/20261124-sam3-system-audit'
DEFAULT_OBSERVATION = ROOT/'.franka-player/panel/sessions/7ef324daa5da4e47aa705542efc0d19d/hardware/observation-e5d7b04eae61423caef2c529c3814e7e'


def encode(array):
    return base64.b64encode(np.ascontiguousarray(array).tobytes()).decode('ascii')


def check(audit, directory):
    observation = load_observation(directory)
    adapter = SimpleNamespace(embodiment=observation.robot_state.embodiment,
        get_robot_state=lambda: observation.robot_state, get_observation=lambda: observation)
    rows = json.loads((audit/'inference-analysis.json').read_text())
    results = []
    for row in rows:
        name, prompt = row['camera'], row['prompt']
        camera = observation.cameras[name]
        assert hashlib.sha256(camera.rgb.tobytes()).hexdigest() == row['image_sha256']
        with np.load(audit/f"case-{row['case']:02d}-masks.npz") as archive:
            masks = [archive[f'mask_{i}'] for i in range(len(row['candidates']))]
        scores = [c['score'] for c in row['candidates']]
        client = Sam3Provider(max_retries=0)
        client._http.post_json = Mock()
        api = CapApi(adapter, segmentation_provider=client, default_camera=name)
        if isinstance(prompt, str):
            client._http.post_json.return_value = {'results': [dict(mask_base64=encode(mask.astype(np.uint8)),
                shape=list(mask.shape), box=candidate['box'], score=candidate['score'], label=prompt)
                for mask, candidate in zip(masks, row['candidates'])]}
            result = api.localize_object(prompt, camera_name=name)
            assert result.ok == any(score > .5 for score in scores)
            if not result.ok:
                assert result.error.code.value == 'no_segmentation' and result.error.recoverable
                assert result.point_cloud is None and result.geometry is None
                assert result.diagnostics['raw_scores'] == scores
        else:
            array = np.stack(masks)
            client._http.post_json.return_value = dict(scores=scores, masks_base64=encode(array),
                masks_shape=list(array.shape), masks_dtype=str(array.dtype))
            points = api.segment_points(name, [prompt])
            assert points.ok
            selected = points.segmentations[0]
            result = api.localize_object('caller selected visible target', camera_name=name, segmentation=selected)
            assert result.ok, result.error
            assert client._http.post_json.call_count == 1  # Point call only, never re-segment text.
            direct = api.mask_to_point_cloud(selected, name, observation.robot_state.base_frame)
            np.testing.assert_array_equal(result.point_cloud.points, direct.points)
        if result.ok:
            np.testing.assert_allclose(result.geometry.extents, row['candidates'][0]['extents_m'], atol=1e-10)
            np.testing.assert_array_equal(result.segmentation.mask, masks[0])
        results.append(dict(case=row['case'], camera=name, prompt=prompt, ok=result.ok,
            error=None if result.ok else result.error.code.value,
            original_top_score=scores[0] if scores else None,
            text_threshold=.5 if isinstance(prompt, str) else None,
            max_extent_m=float(max(result.geometry.extents)) if result.ok else None,
            image_sha256=row['image_sha256'], http_calls=0,
            reused_point_mask=not isinstance(prompt, str)))
    return dict(mode='offline_saved_response_replay', cases=results,
        passed=len(results), new_inference=False, hardware_actions=False,
        limits='Tests saved outputs, not fresh model inference or physical localization accuracy.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit', type=Path, default=DEFAULT_AUDIT)
    parser.add_argument('--observation', type=Path, default=DEFAULT_OBSERVATION)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = check(args.audit, args.observation)
    payload = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload+'\n')
    print(payload)


if __name__ == '__main__':
    main()

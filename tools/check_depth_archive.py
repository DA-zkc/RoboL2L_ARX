"""Replay archived RGB-D through explicit depth selection. No HTTP or hardware."""
import argparse
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
from cap_harness.contracts import Segmentation
from cap_harness.depth import DepthSelectionError
from cap_harness.geometry import mask_to_point_cloud, estimate_oriented_bounding_box
from robol2l_deploy.observations import load_observation

ROOT = Path(__file__).resolve().parents[1]
HARDWARE = ROOT/'.franka-player/panel/sessions/7ef324daa5da4e47aa705542efc0d19d/hardware'
AUDIT = ROOT/'.franka-player/diagnostics/20261124-sam3-system-audit'


def check():
    observation = load_observation(HARDWARE/'observation-e5d7b04eae61423caef2c529c3814e7e')
    camera = observation.cameras['right_wrist']
    with np.load(HARDWARE/'perception/7ba92aed246546cdaead683a7c014e5e/masks.npz') as archive:
        bad = archive['mask_0']
    with np.load(AUDIT/'case-05-masks.npz') as archive:
        good = archive['mask_0']
    records = []

    def project(name, mask, *, source=camera, **options):
        segmentation = Segmentation(mask, name, 1., 'right_wrist', source.frame)
        cloud = mask_to_point_cloud(source, segmentation, **options)
        # Project back into the image: no selected pixel may come from outside the mask.
        xyz = cloud.points
        x = np.rint(xyz[:, 0]/xyz[:, 2]*source.intrinsics[0, 0]+source.intrinsics[0, 2]).astype(int)
        y = np.rint(xyz[:, 1]/xyz[:, 2]*source.intrinsics[1, 1]+source.intrinsics[1, 2]).astype(int)
        assert np.all(mask[y, x])
        extent = float(max(estimate_oriented_bounding_box(cloud).extents))
        records.append(dict(case=name, extent_m=extent, projection=dict(cloud.diagnostics['projection']),
                            added_mask_exterior_pixels=0))
        return cloud, extent

    _, raw_extent = project('original_wrong_table_mask', bad)
    assert abs(raw_extent-5.973597738162635) < 1e-9
    _, limited_extent = project('wrong_table_mask_explicit_range', bad, depth_range_m=[0, 1])
    assert limited_extent > 1.  # Depth cleanup is not semantic correction.
    try:
        project('wrong_table_mask_fruit_seed', bad, seed_px=[325, 200], max_depth_jump_m=.03)
    except DepthSelectionError as exc:
        assert exc.diagnostics['rejection_reason'] == 'seed_has_no_selected_depth'
        records.append(dict(case='wrong_table_mask_fruit_seed', rejected=True, diagnostics=exc.diagnostics))
    else:
        raise AssertionError('must not expand a table mask to include the fruit seed')
    good_raw, good_extent = project('selected_fruit_original_depth', good)
    good_filtered, _ = project('selected_fruit_explicit_range', good, depth_range_m=[.3, .5])
    np.testing.assert_array_equal(good_raw.points, good_filtered.points)
    _, seed_extent = project('selected_fruit_seed_continuity', good,
        depth_range_m=[.3, .5], seed_px=[325, 200], max_depth_jump_m=.03)
    assert seed_extent < .12
    # Fault injection is explicitly synthetic, not a claim about the saved fruit data.
    corrupted = camera.depth_m.copy()
    corrupted[200, 325] = 4.5
    injected = replace(camera, depth_m=corrupted)
    _, corrupted_extent = project('synthetic_far_depth_inside_correct_mask', good, source=injected)
    repaired, repaired_extent = project('synthetic_far_depth_explicit_range', good,
        source=injected, depth_range_m=[.3, .5])
    assert corrupted_extent > 4 and repaired_extent < .12
    assert len(repaired.points) == len(good_raw.points)-1
    return dict(mode='offline_archives_and_labeled_fault_injection', cases=records,
                original_fruit_extent_m=good_extent, hardware_actions=False, new_inference=False,
                limits='Explicit filter regression only; not sensor root-cause or physical accuracy validation.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = check()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'cases_passed': len(report['cases']), 'report': str(args.output)}))

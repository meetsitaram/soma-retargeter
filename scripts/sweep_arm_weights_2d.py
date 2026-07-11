"""Sweep (hand_t_weight, forearm_t_weight) jointly and report shoulder twist
+ elbow bend + wrist pitch. Goal: find a config with moderate elbow bend
(\u2248 -15\u00b0 to -25\u00b0) AND small shoulder_yaw twist (\u2264 \u00b135\u00b0) AND wrist_pitch near 0.
"""

from __future__ import annotations

import argparse
import csv as _csv
import json
import sys
import time
from pathlib import Path

import warp as wp

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import soma_retargeter.assets.bvh as bvh_utils  # noqa: E402
import soma_retargeter.assets.csv as csv_utils  # noqa: E402
import soma_retargeter.utils.io_utils as io_utils  # noqa: E402
from soma_retargeter.pipelines.newton_pipeline import NewtonPipeline  # noqa: E402
from soma_retargeter.utils.space_conversion_utils import (  # noqa: E402
    SpaceConverter,
    get_facing_direction_type_from_str,
)
from wrist_saturation_report import analyze as analyze_joints  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bvh", required=True)
    parser.add_argument("--out-dir", default="scratch/sweep_arm_2d")
    args = parser.parse_args()

    base_ret = io_utils.load_json(io_utils.get_config_file(
        "agibot_x2_ultra", "soma_to_x2_ultra_retargeter_config.json"))

    # Note: scaler config (LeftHand/RightHand quat offsets) is whatever is
    # currently on disk -- we already reverted to the v1 offsets in v3.
    combos = [
        (1.5, 1.0),
        (1.5, 1.5),
        (1.2, 1.0),
        (1.2, 1.5),
        (1.0, 1.0),
        (1.0, 1.5),
        (1.0, 2.0),
        (0.7, 1.0),
        (0.7, 1.5),
        (0.7, 2.0),
        (0.5, 1.5),
        (0.5, 2.0),
    ]

    out_dir = Path(args.out_dir).resolve()
    (out_dir / "csv").mkdir(parents=True, exist_ok=True)

    importer = bvh_utils.BVHImporter()
    bvh_path = Path(args.bvh).resolve()
    skel, _ = importer.create_skeleton(bvh_path)
    _, animation = bvh_utils.load_bvh(bvh_path, skel)
    conv = SpaceConverter(get_facing_direction_type_from_str("Mujoco"))
    bvh_tx = conv.transform(wp.transform_identity())

    target_joints = [
        "left_shoulder_yaw_joint", "right_shoulder_yaw_joint",
        "left_elbow_joint", "right_elbow_joint",
        "left_wrist_pitch_joint", "right_wrist_pitch_joint",
        "left_wrist_yaw_joint", "right_wrist_yaw_joint",
    ]

    results = []
    t0 = time.time()
    for hand_w, forearm_w in combos:
        ret = json.loads(json.dumps(base_ret))
        ret["ik_map"]["LeftHand"]["t_weight"] = hand_w
        ret["ik_map"]["RightHand"]["t_weight"] = hand_w
        ret["ik_map"]["LeftForeArm"]["t_weight"] = forearm_w
        ret["ik_map"]["RightForeArm"]["t_weight"] = forearm_w

        csv_path = out_dir / "csv" / f"h{hand_w:0.1f}_f{forearm_w:0.1f}.csv"
        pipe = NewtonPipeline(skel, "soma", "agibot_x2_ultra", retarget_config=ret)
        pipe.add_input_motions([animation], [bvh_tx], scale_animation=True)
        out = pipe.execute()
        csv_utils.save_csv(str(csv_path), out[0], csv_config=csv_utils.AgibotX2Ultra31DOF_CSVConfig())

        res = analyze_joints(csv_path, target_joints, near_deg=5.0)
        lsy = res["left_shoulder_yaw_joint"]["mean_deg"]
        rsy = res["right_shoulder_yaw_joint"]["mean_deg"]
        lelb = res["left_elbow_joint"]["mean_deg"]
        relb = res["right_elbow_joint"]["mean_deg"]
        lwp = res["left_wrist_pitch_joint"]["mean_deg"]
        rwp = res["right_wrist_pitch_joint"]["mean_deg"]
        lwy = res["left_wrist_yaw_joint"]["mean_deg"]
        rwy = res["right_wrist_yaw_joint"]["mean_deg"]
        # Score: want |shoulder_yaw| small AND elbow in [-25, -15] AND |wrist_pitch| small
        elbow_pen = max(0, -10 - lelb) + max(0, lelb + 30) + max(0, -10 - relb) + max(0, relb + 30)
        score = (abs(lsy) + abs(rsy)) * 0.5 + (abs(lwp) + abs(rwp)) * 1.5 + elbow_pen * 1.0
        elapsed = time.time() - t0
        print(f"hand={hand_w:0.2f} forearm={forearm_w:0.2f}  "
              f"score={score:6.2f}  sYaw={lsy:+6.2f}/{rsy:+6.2f}  "
              f"elb={lelb:+6.2f}/{relb:+6.2f}  wPit={lwp:+6.2f}/{rwp:+6.2f}  "
              f"wYaw={lwy:+6.2f}/{rwy:+6.2f}  ({elapsed:.0f}s)")
        results.append(dict(hand=hand_w, forearm=forearm_w, score=score,
                            lsy=lsy, rsy=rsy, lelb=lelb, relb=relb,
                            lwp=lwp, rwp=rwp, lwy=lwy, rwy=rwy))

    print("\nTop 5 by score:")
    for r in sorted(results, key=lambda r: r["score"])[:5]:
        print(f"  hand={r['hand']:0.2f} forearm={r['forearm']:0.2f}  "
              f"score={r['score']:6.2f}  sY={r['lsy']:+6.1f}/{r['rsy']:+6.1f}  "
              f"elb={r['lelb']:+6.1f}/{r['relb']:+6.1f}  wPit={r['lwp']:+6.1f}/{r['rwp']:+6.1f}")

    with open(out_dir / "results.csv", "w", newline="") as f:
        wcsv = _csv.DictWriter(f, fieldnames=results[0].keys())
        wcsv.writeheader()
        wcsv.writerows(results)


if __name__ == "__main__":
    main()

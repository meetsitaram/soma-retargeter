"""Sweep the IK position weight on LeftHand / RightHand and report how the
shoulder_yaw + elbow + wrist behave. Hypothesis: a high hand_t_weight
combined with the X2's tight shoulder_roll limits forces the IK to twist
the shoulder_yaw to track the human hand exactly, dragging the forearm
across the body.

Lowering hand_t_weight should let the IK ease off shoulder_yaw and let the
arm hang more naturally, even at the cost of small wrist position error.
"""

from __future__ import annotations

import argparse
import csv as _csv
import json
import sys
import time
from pathlib import Path

import numpy as np
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
    parser.add_argument("--weights", type=str,
                        default="2.0,1.5,1.2,1.0,0.7,0.5,0.3")
    parser.add_argument("--out-dir", default="scratch/sweep_hand_w")
    args = parser.parse_args()

    base_ret = io_utils.load_json(io_utils.get_config_file(
        "agibot_x2_ultra", "soma_to_x2_ultra_retargeter_config.json"))

    weights = [float(w) for w in args.weights.split(",")]

    out_dir = Path(args.out_dir).resolve()
    (out_dir / "csv").mkdir(parents=True, exist_ok=True)

    importer = bvh_utils.BVHImporter()
    bvh_path = Path(args.bvh).resolve()
    skel, _ = importer.create_skeleton(bvh_path)
    _, animation = bvh_utils.load_bvh(bvh_path, skel)
    converter = SpaceConverter(get_facing_direction_type_from_str("Mujoco"))
    bvh_tx = converter.transform(wp.transform_identity())

    target_joints = [
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint", "left_elbow_joint",
        "left_wrist_yaw_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint",
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint", "right_elbow_joint",
        "right_wrist_yaw_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint",
        "waist_pitch_joint", "waist_yaw_joint",
    ]

    results = []
    t0 = time.time()
    for w in weights:
        ret_cfg = json.loads(json.dumps(base_ret))
        ret_cfg["ik_map"]["LeftHand"]["t_weight"] = w
        ret_cfg["ik_map"]["RightHand"]["t_weight"] = w

        csv_path = out_dir / "csv" / f"hand_tw_{w:0.2f}.csv"
        pipe = NewtonPipeline(skel, "soma", "agibot_x2_ultra", retarget_config=ret_cfg)
        pipe.add_input_motions([animation], [bvh_tx], scale_animation=True)
        bufs = pipe.execute()
        csv_utils.save_csv(str(csv_path), bufs[0], csv_config=csv_utils.AgibotX2Ultra31DOF_CSVConfig())

        res = analyze_joints(csv_path, target_joints, near_deg=5.0)
        elapsed = time.time() - t0
        row = dict(
            tw=w,
            lYawMean=res["left_shoulder_yaw_joint"]["mean_deg"],
            rYawMean=res["right_shoulder_yaw_joint"]["mean_deg"],
            lYawStd=res["left_shoulder_yaw_joint"]["std_deg"],
            lElbowMean=res["left_elbow_joint"]["mean_deg"],
            rElbowMean=res["right_elbow_joint"]["mean_deg"],
            lWristPit=res["left_wrist_pitch_joint"]["mean_deg"],
            rWristPit=res["right_wrist_pitch_joint"]["mean_deg"],
            lWristYaw=res["left_wrist_yaw_joint"]["mean_deg"],
            rWristYaw=res["right_wrist_yaw_joint"]["mean_deg"],
            waistPit=res["waist_pitch_joint"]["mean_deg"],
        )
        print(f"tw={w:4.2f}  lShoulderYaw={row['lYawMean']:+6.2f}\u00b1{row['lYawStd']:.1f}  "
              f"rSY={row['rYawMean']:+6.2f}  lElb={row['lElbowMean']:+6.2f}  "
              f"rElb={row['rElbowMean']:+6.2f}  lWPit={row['lWristPit']:+6.2f}  "
              f"rWPit={row['rWristPit']:+6.2f}  waistP={row['waistPit']:+6.2f}  ({elapsed:.0f}s)")
        results.append(row)

    with open(out_dir / "results.csv", "w", newline="") as f:
        wcsv = _csv.DictWriter(f, fieldnames=results[0].keys())
        wcsv.writeheader()
        wcsv.writerows(results)


if __name__ == "__main__":
    main()

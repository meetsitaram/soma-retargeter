"""Sweep a mirrored \u0394Z (yaw-around-forearm) correction on top of the current
LeftHand / RightHand offset quaternions to steer the palm direction by
rotation around the forearm axis.

Mirrored on right side: left gets \u0394Z, right gets -\u0394Z. This is the same
mirror used in the original wrist sweep, which empirically maps cleanly
onto X2's left_wrist_yaw_joint angle.

Goal: pick a \u0394Z whose retargeted clip has the palm facing inward toward
the body's centerline (i.e. wrist-yaw rotated symmetrically).
"""

from __future__ import annotations

import argparse
import csv as _csv
import json
import sys
import time
from pathlib import Path

import warp as wp
from scipy.spatial.transform import Rotation as R

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


def _compose_mirror_z(quat_xyzw, dz_deg, mirror):
    base = R.from_quat(quat_xyzw)
    sign = -1.0 if mirror else 1.0
    correction = R.from_euler("z", sign * dz_deg, degrees=True)
    return [float(v) for v in (base * correction).as_quat()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bvh", required=True)
    parser.add_argument("--dz-min", type=float, default=-90.0)
    parser.add_argument("--dz-max", type=float, default=90.0)
    parser.add_argument("--dz-step", type=float, default=30.0)
    parser.add_argument("--out-dir", default="scratch/sweep_palm_yaw")
    args = parser.parse_args()

    base_ret = io_utils.load_json(io_utils.get_config_file(
        "agibot_x2_ultra", "soma_to_x2_ultra_retargeter_config.json"))
    base_scaler = io_utils.load_json(io_utils.get_config_file(
        "agibot_x2_ultra", "soma_to_x2_ultra_scaler_config.json"))

    left_old = list(base_scaler["joint_offsets"]["LeftHand"][1])
    right_old = list(base_scaler["joint_offsets"]["RightHand"][1])
    print(f"Base LeftHand : {left_old}")
    print(f"Base RightHand: {right_old}")

    out_dir = Path(args.out_dir).resolve()
    (out_dir / "csv").mkdir(parents=True, exist_ok=True)
    (out_dir / "cfg").mkdir(parents=True, exist_ok=True)

    importer = bvh_utils.BVHImporter()
    bvh_path = Path(args.bvh).resolve()
    skel, _ = importer.create_skeleton(bvh_path)
    _, animation = bvh_utils.load_bvh(bvh_path, skel)
    conv = SpaceConverter(get_facing_direction_type_from_str("Mujoco"))
    bvh_tx = conv.transform(wp.transform_identity())

    target_joints = [
        "left_wrist_yaw_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint",
        "right_wrist_yaw_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint",
    ]

    results = []
    t0 = time.time()
    dz_values = []
    cur = args.dz_min
    while cur <= args.dz_max + 0.01:
        dz_values.append(cur)
        cur += args.dz_step

    for dz in dz_values:
        scaler = json.loads(json.dumps(base_scaler))
        scaler["joint_offsets"]["LeftHand"][1] = _compose_mirror_z(left_old, float(dz), mirror=False)
        scaler["joint_offsets"]["RightHand"][1] = _compose_mirror_z(right_old, float(dz), mirror=True)
        tag = f"dz{dz:+06.1f}".replace("+", "p").replace("-", "m").replace(".", "_")
        cfg_path = out_dir / "cfg" / f"scaler_{tag}.json"
        cfg_path.write_text(json.dumps(scaler, indent=2))

        ret = dict(base_ret)
        ret["human_robot_scaler_config"] = str(cfg_path)
        csv_path = out_dir / "csv" / f"{tag}.csv"

        pipe = NewtonPipeline(skel, "soma", "agibot_x2_ultra", retarget_config=ret)
        pipe.add_input_motions([animation], [bvh_tx], scale_animation=True)
        bufs = pipe.execute()
        csv_utils.save_csv(str(csv_path), bufs[0], csv_config=csv_utils.AgibotX2Ultra31DOF_CSVConfig())

        res = analyze_joints(csv_path, target_joints, near_deg=5.0)
        lyaw = res["left_wrist_yaw_joint"]["mean_deg"]
        ryaw = res["right_wrist_yaw_joint"]["mean_deg"]
        lpit = res["left_wrist_pitch_joint"]["mean_deg"]
        rpit = res["right_wrist_pitch_joint"]["mean_deg"]
        lrol = res["left_wrist_roll_joint"]["mean_deg"]
        rrol = res["right_wrist_roll_joint"]["mean_deg"]
        near = sum(res[j]["pct_near_either"] for j in target_joints)
        elapsed = time.time() - t0
        print(f"\u0394Z={dz:+6.1f}  lYaw={lyaw:+6.2f}  rYaw={ryaw:+6.2f}  "
              f"lPit={lpit:+6.2f}  rPit={rpit:+6.2f}  "
              f"lRol={lrol:+6.2f}  rRol={rrol:+6.2f}  near={near:5.1f}  ({elapsed:.0f}s)  "
              f"-> {csv_path.name}")
        results.append(dict(dz=float(dz), lyaw=lyaw, ryaw=ryaw,
                            lpit=lpit, rpit=rpit, lrol=lrol, rrol=rrol, near=near,
                            csv=str(csv_path)))

    with open(out_dir / "results.csv", "w", newline="") as f:
        w = _csv.DictWriter(f, fieldnames=results[0].keys())
        w.writeheader()
        w.writerows(results)


if __name__ == "__main__":
    main()

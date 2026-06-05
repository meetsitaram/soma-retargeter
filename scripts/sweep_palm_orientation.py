"""Sweep a mirrored \u0394X (roll) correction on top of the current LeftHand /
RightHand offset quaternions to steer the palm direction.

X-rotation in the offset frame maps to robot wrist_roll, which is the
palm-twist axis around the forearm. Positive \u0394X on left is mirrored to
negative \u0394X on right so the palms rotate symmetrically.

Outputs per cell: wrist_roll mean (left/right), wrist_pitch mean (so we
don't lose the forearm-alignment we just dialed in), wrist_yaw mean.
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


def _compose_mirror_x(quat_xyzw, dx_deg, mirror):
    base = R.from_quat(quat_xyzw)
    sign = -1.0 if mirror else 1.0
    correction = R.from_euler("x", sign * dx_deg, degrees=True)
    return [float(v) for v in (base * correction).as_quat()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bvh", required=True)
    parser.add_argument("--dx-min", type=float, default=-90.0)
    parser.add_argument("--dx-max", type=float, default=90.0)
    parser.add_argument("--dx-step", type=float, default=15.0)
    parser.add_argument("--out-dir", default="scratch/sweep_palm")
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
    dx_values = []
    cur = args.dx_min
    while cur <= args.dx_max + 0.01:
        dx_values.append(cur)
        cur += args.dx_step

    for dx in dx_values:
        scaler = json.loads(json.dumps(base_scaler))
        scaler["joint_offsets"]["LeftHand"][1] = _compose_mirror_x(left_old, float(dx), mirror=False)
        scaler["joint_offsets"]["RightHand"][1] = _compose_mirror_x(right_old, float(dx), mirror=True)
        tag = f"dx{dx:+06.1f}".replace("+", "p").replace("-", "m").replace(".", "_")
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
        lpit = res["left_wrist_pitch_joint"]["mean_deg"]
        rpit = res["right_wrist_pitch_joint"]["mean_deg"]
        lrol = res["left_wrist_roll_joint"]["mean_deg"]
        rrol = res["right_wrist_roll_joint"]["mean_deg"]
        lyaw = res["left_wrist_yaw_joint"]["mean_deg"]
        ryaw = res["right_wrist_yaw_joint"]["mean_deg"]
        near = sum(res[j]["pct_near_either"] for j in target_joints)
        elapsed = time.time() - t0
        print(f"\u0394X={dx:+6.1f}  lRol={lrol:+6.2f}  rRol={rrol:+6.2f}  "
              f"lPit={lpit:+6.2f}  rPit={rpit:+6.2f}  "
              f"lYaw={lyaw:+6.2f}  rYaw={ryaw:+6.2f}  near={near:5.1f}  ({elapsed:.0f}s)  "
              f"-> {csv_path.name}")
        results.append(dict(dx=float(dx), lrol=lrol, rrol=rrol,
                            lpit=lpit, rpit=rpit, lyaw=lyaw, ryaw=ryaw, near=near,
                            csv=str(csv_path)))

    with open(out_dir / "results.csv", "w", newline="") as f:
        w = _csv.DictWriter(f, fieldnames=results[0].keys())
        w.writeheader()
        w.writerows(results)


if __name__ == "__main__":
    main()

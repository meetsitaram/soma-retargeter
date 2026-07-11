"""Sweep a no-mirror \u0394Y correction layered on top of the *current* LeftHand
and RightHand offset quaternions, aimed at driving wrist_pitch toward 0.

Unlike sweep_wrist_offsets.py, this sweep applies the same R_y(\u0394Y) on the
left AND right offsets (no sign flip) because empirically the wrist_pitch
joints respond in the same direction to a same-sign Y rotation.

Reads the current LeftHand/RightHand offset quaternions from
``soma_to_x2_ultra_scaler_config.json`` on disk (i.e. the already-applied
ones), then sweeps \u0394Y. Scoring optimizes for |wrist_pitch_mean| being
small, with a secondary penalty for limit saturation across all 6 wrist
joints.
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
from wrist_saturation_report import analyze as analyze_wrists  # noqa: E402


def _compose_same_y(quat_xyzw, dy_deg):
    base = R.from_quat(quat_xyzw)
    correction = R.from_euler("y", dy_deg, degrees=True)
    return [float(v) for v in (base * correction).as_quat()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bvh", required=True)
    parser.add_argument("--model-height", type=float, default=1.40)
    parser.add_argument("--dy-min", type=float, default=-45.0)
    parser.add_argument("--dy-max", type=float, default=15.0)
    parser.add_argument("--dy-step", type=float, default=5.0)
    parser.add_argument("--out-dir", default="scratch/sweep_pitch")
    args = parser.parse_args()

    base_ret = io_utils.load_json(io_utils.get_config_file(
        "agibot_x2_ultra", "soma_to_x2_ultra_retargeter_config.json"))
    base_ret["model_height"] = args.model_height
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
    converter = SpaceConverter(get_facing_direction_type_from_str("Mujoco"))
    bvh_tx = converter.transform(wp.transform_identity())

    results = []
    t0 = time.time()
    for dy in np.arange(args.dy_min, args.dy_max + 0.1, args.dy_step):
        scaler = json.loads(json.dumps(base_scaler))
        scaler["joint_offsets"]["LeftHand"][1] = _compose_same_y(left_old, float(dy))
        scaler["joint_offsets"]["RightHand"][1] = _compose_same_y(right_old, float(dy))
        cfg_path = out_dir / "cfg" / f"scaler_dy{dy:+05.1f}.json".replace("+", "p").replace("-", "m")
        cfg_path.write_text(json.dumps(scaler, indent=2))

        ret_cfg = dict(base_ret)
        ret_cfg["human_robot_scaler_config"] = str(cfg_path)
        csv_path = out_dir / "csv" / f"dy{dy:+05.1f}.csv".replace("+", "p").replace("-", "m")

        pipe = NewtonPipeline(skel, "soma", "agibot_x2_ultra", retarget_config=ret_cfg)
        pipe.add_input_motions([animation], [bvh_tx], scale_animation=True)
        bufs = pipe.execute()
        csv_utils.save_csv(str(csv_path), bufs[0], csv_config=csv_utils.AgibotX2Ultra31DOF_CSVConfig())

        wrist_joints = ["left_wrist_yaw_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint",
                        "right_wrist_yaw_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint"]
        res = analyze_wrists(csv_path, wrist_joints, near_deg=5.0)
        lpit = res["left_wrist_pitch_joint"]["mean_deg"]
        rpit = res["right_wrist_pitch_joint"]["mean_deg"]
        lyaw = res["left_wrist_yaw_joint"]["mean_deg"]
        ryaw = res["right_wrist_yaw_joint"]["mean_deg"]
        lrol = res["left_wrist_roll_joint"]["mean_deg"]
        rrol = res["right_wrist_roll_joint"]["mean_deg"]
        near = sum(res[j]["pct_near_either"] for j in wrist_joints)
        # primary score = |pitch_left| + |pitch_right|, secondary = saturation
        score = abs(lpit) + abs(rpit) + 0.5 * near
        elapsed = time.time() - t0
        print(f"\u0394Y={dy:+6.1f}  score={score:6.2f}  lPit={lpit:+6.2f}  rPit={rpit:+6.2f}  "
              f"lYaw={lyaw:+6.2f}  rYaw={ryaw:+6.2f}  "
              f"lRol={lrol:+6.2f}  rRol={rrol:+6.2f}  near={near:5.1f}  ({elapsed:.0f}s)")
        results.append(dict(dy=float(dy), score=score, lpit=lpit, rpit=rpit,
                            lyaw=lyaw, ryaw=ryaw, lrol=lrol, rrol=rrol, near=near))

    print("\nSorted by |pitch| sum:")
    for r in sorted(results, key=lambda r: r["score"])[:5]:
        print(f"  \u0394Y={r['dy']:+6.1f}  score={r['score']:6.2f}  "
              f"lPit={r['lpit']:+6.2f} rPit={r['rpit']:+6.2f} "
              f"lYaw={r['lyaw']:+6.2f} rYaw={r['ryaw']:+6.2f} near={r['near']:5.1f}")

    with open(out_dir / "results.csv", "w", newline="") as f:
        w = _csv.DictWriter(f, fieldnames=results[0].keys())
        w.writeheader()
        w.writerows(results)


if __name__ == "__main__":
    main()

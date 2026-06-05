"""Grid-sweep (\u0394Y, \u0394X) corrections to LeftHand/RightHand offset quaternions.

For each candidate (delta_y_deg, delta_x_deg) we:
1. Compose new LeftHand / RightHand offset quaternions by right-multiplying
   the existing config quaternions by Ry(delta_y) * Rx(delta_x). Right-side
   uses the mirrored sign on delta_y so the correction is symmetric.
2. Materialize the modified scaler config to a temp JSON file.
3. Retarget the reference BVH headlessly via the same NewtonPipeline used
   by `bvh_to_csv_converter.py --viewer null`.
4. Score the resulting CSV by:
     - sum of |mean wrist-joint angle - midrange| across all 6 wrist joints
     - large penalty for any wrist joint with >5% near-limit frames

Score interpretation: LOWER IS BETTER. The midrange the scoring uses is
the geometric center of the MJCF joint range (so for the asymmetric
left_wrist_roll [-90, +41.5] the target is -24.3 deg, not 0).

Output: a CSV at scratch/sweep_wrist_results.csv with one row per cell.
"""

from __future__ import annotations

import argparse
import csv as _csv
import json
import math
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

import warp as wp  # noqa: E402

import soma_retargeter.assets.bvh as bvh_utils  # noqa: E402
import soma_retargeter.assets.csv as csv_utils  # noqa: E402
import soma_retargeter.utils.io_utils as io_utils  # noqa: E402
from soma_retargeter.pipelines.newton_pipeline import NewtonPipeline  # noqa: E402
from soma_retargeter.utils.space_conversion_utils import (  # noqa: E402
    SpaceConverter,
    get_facing_direction_type_from_str,
)

sys.path.insert(0, str(Path(__file__).resolve().parent))
from wrist_saturation_report import analyze as analyze_wrists  # noqa: E402


def _compose(
    quat_xyzw: list[float],
    delta_z_deg: float,
    delta_y_deg: float,
    delta_x_deg: float,
    mirror: bool,
) -> list[float]:
    """Right-multiply the existing offset by Rz(\u0394Z) * Ry(\u0394Y) * Rx(\u0394X).

    Args:
        quat_xyzw: existing offset quaternion as [x, y, z, w]
        delta_z_deg, delta_y_deg, delta_x_deg: corrections in degrees
        mirror: if True, mirror the Z/Y signs (right side is anti-symmetric in Z, Y)

    Returns:
        new quaternion in [x, y, z, w]
    """
    base = R.from_quat(quat_xyzw)  # scipy expects [x, y, z, w]
    dz = -delta_z_deg if mirror else delta_z_deg
    dy = -delta_y_deg if mirror else delta_y_deg
    dx = delta_x_deg
    correction = R.from_euler("zyx", [dz, dy, dx], degrees=True)
    composed = base * correction
    q = composed.as_quat()  # [x, y, z, w]
    return [float(v) for v in q]


def _make_modified_scaler(base_scaler: dict, dz: float, dy: float, dx: float) -> dict:
    cfg = json.loads(json.dumps(base_scaler))
    left_old = cfg["joint_offsets"]["LeftHand"][1]
    right_old = cfg["joint_offsets"]["RightHand"][1]
    cfg["joint_offsets"]["LeftHand"][1] = _compose(left_old, dz, dy, dx, mirror=False)
    cfg["joint_offsets"]["RightHand"][1] = _compose(right_old, dz, dy, dx, mirror=True)
    return cfg


def _retarget(bvh_path: Path, retarget_cfg: dict, out_csv: Path, animation_cache: dict) -> None:
    if "skeleton" not in animation_cache:
        importer = bvh_utils.BVHImporter()
        animation_cache["skeleton"], _ = importer.create_skeleton(bvh_path)
        _, animation_cache["animation"] = bvh_utils.load_bvh(bvh_path, animation_cache["skeleton"])
    converter = SpaceConverter(get_facing_direction_type_from_str("Mujoco"))
    bvh_tx = converter.transform(wp.transform_identity())

    pipeline = NewtonPipeline(
        animation_cache["skeleton"],
        source_type="soma",
        robot_type="agibot_x2_ultra",
        retarget_config=retarget_cfg,
    )
    pipeline.add_input_motions([animation_cache["animation"]], [bvh_tx], scale_animation=True)
    csv_buffers = pipeline.execute()
    csv_utils.save_csv(str(out_csv), csv_buffers[0], csv_config=csv_utils.AgibotX2Ultra31DOF_CSVConfig())


def _score(csv_path: Path) -> dict:
    wrist_joints = [
        "left_wrist_yaw_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint",
        "right_wrist_yaw_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint",
    ]
    res = analyze_wrists(csv_path, wrist_joints, near_deg=5.0)
    sum_abs_offset = 0.0
    near_penalty = 0.0
    per_joint = {}
    for j in wrist_joints:
        r = res[j]
        # absolute distance of the mean angle from the geometric midrange
        offset = abs(r["mean_offset_from_mid_deg"])
        sum_abs_offset += offset
        # heavy penalty for any joint that has frames clamping at a limit
        if r["pct_near_either"] > 1.0:
            near_penalty += r["pct_near_either"]
        per_joint[j] = dict(
            mean=r["mean_deg"],
            off_mid=r["mean_offset_from_mid_deg"],
            near=r["pct_near_either"],
        )
    return dict(
        sum_abs_offset=sum_abs_offset,
        near_penalty=near_penalty,
        score=sum_abs_offset + 5.0 * near_penalty,
        per_joint=per_joint,
    )


def sweep(args):
    base_retargeter_cfg = io_utils.load_json(
        io_utils.get_config_file("agibot_x2_ultra", "soma_to_x2_ultra_retargeter_config.json")
    )
    base_retargeter_cfg["model_height"] = args.model_height

    base_scaler_cfg = io_utils.load_json(
        io_utils.get_config_file("agibot_x2_ultra", "soma_to_x2_ultra_scaler_config.json")
    )

    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    scaler_tmp_dir = out_dir / "scaler_cfgs"
    scaler_tmp_dir.mkdir(exist_ok=True)
    csv_dir = out_dir / "csv"
    csv_dir.mkdir(exist_ok=True)

    results_csv = out_dir / "sweep_results.csv"
    results = []

    bvh_path = Path(args.bvh).resolve()
    animation_cache: dict = {}

    grid_z = np.arange(args.dz_min, args.dz_max + 0.1, args.dz_step)
    grid_y = np.arange(args.dy_min, args.dy_max + 0.1, args.dy_step)
    grid_x = np.arange(args.dx_min, args.dx_max + 0.1, args.dx_step)
    n_cells = len(grid_z) * len(grid_y) * len(grid_x)
    print(f"Sweeping \u0394Z x \u0394Y x \u0394X = {len(grid_z)} x {len(grid_y)} x {len(grid_x)} = {n_cells} cells")
    print(f"\u0394Z range: {grid_z[0]}..{grid_z[-1]} step {args.dz_step}")
    print(f"\u0394Y range: {grid_y[0]}..{grid_y[-1]} step {args.dy_step}")
    print(f"\u0394X range: {grid_x[0]}..{grid_x[-1]} step {args.dx_step}")

    t0 = time.time()
    for dz in grid_z:
        for dy in grid_y:
            for dx in grid_x:
                cell_id = (f"dz{dz:+05.1f}_dy{dy:+05.1f}_dx{dx:+05.1f}"
                           .replace("+", "p").replace("-", "m"))
                scaler_cfg = _make_modified_scaler(base_scaler_cfg, float(dz), float(dy), float(dx))
                scaler_path = scaler_tmp_dir / f"scaler_{cell_id}.json"
                scaler_path.write_text(json.dumps(scaler_cfg, indent=2))

                ret_cfg = dict(base_retargeter_cfg)
                ret_cfg["human_robot_scaler_config"] = str(scaler_path)

                csv_path = csv_dir / f"{cell_id}.csv"
                try:
                    _retarget(bvh_path, ret_cfg, csv_path, animation_cache)
                    s = _score(csv_path)
                except Exception as e:
                    print(f"[WARN] cell {cell_id} failed: {e}")
                    continue

                elapsed = time.time() - t0
                print(f"\u0394Z={dz:+6.1f}  \u0394Y={dy:+6.1f}  \u0394X={dx:+6.1f}  "
                      f"score={s['score']:7.2f}  "
                      f"sum_off={s['sum_abs_offset']:6.1f}  near={s['near_penalty']:5.2f}  "
                      f"({elapsed:.0f}s)")
                results.append(dict(
                    dz=float(dz),
                    dy=float(dy),
                    dx=float(dx),
                    score=s["score"],
                    sum_abs_offset=s["sum_abs_offset"],
                    near_penalty=s["near_penalty"],
                    **{f"{j}_mean": s["per_joint"][j]["mean"] for j in s["per_joint"]},
                    **{f"{j}_off_mid": s["per_joint"][j]["off_mid"] for j in s["per_joint"]},
                    **{f"{j}_near": s["per_joint"][j]["near"] for j in s["per_joint"]},
                ))

                with open(results_csv, "w", newline="") as f:
                    w = _csv.DictWriter(f, fieldnames=results[0].keys())
                    w.writeheader()
                    w.writerows(results)

    if results:
        results.sort(key=lambda r: r["score"])
        print("\nTop 5 cells (lower score is better):")
        for r in results[:5]:
            print(f"  \u0394Z={r['dz']:+6.1f}  \u0394Y={r['dy']:+6.1f}  \u0394X={r['dx']:+6.1f}  "
                  f"score={r['score']:7.2f}  sum_off={r['sum_abs_offset']:6.1f}  "
                  f"near={r['near_penalty']:5.2f}")
    print(f"\nResults saved to {results_csv}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bvh", required=True, type=str)
    parser.add_argument("--out-dir", type=str, default="scratch/sweep")
    parser.add_argument("--model-height", type=float, default=1.40)
    parser.add_argument("--dz-min", type=float, default=0.0)
    parser.add_argument("--dz-max", type=float, default=0.0)
    parser.add_argument("--dz-step", type=float, default=30.0)
    parser.add_argument("--dy-min", type=float, default=0.0)
    parser.add_argument("--dy-max", type=float, default=0.0)
    parser.add_argument("--dy-step", type=float, default=30.0)
    parser.add_argument("--dx-min", type=float, default=0.0)
    parser.add_argument("--dx-max", type=float, default=0.0)
    parser.add_argument("--dx-step", type=float, default=30.0)
    args = parser.parse_args()
    sweep(args)


if __name__ == "__main__":
    main()

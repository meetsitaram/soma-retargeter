"""Quantify how often retargeted X2 joints sit near (or against) joint limits.

Loads a retargeted CSV produced by the SOMA -> X2 pipeline and reports, per
joint:
- mean / std / min / max of the joint angle (deg)
- midrange (center of the joint's MJCF range), in deg
- mean signed offset from midrange (deg)
- % of frames within ``--near-deg`` (default 5 deg) of either limit
- separate near-lower / near-upper percentages (asymmetric limits matter)
- max excursion in deg vs the JointLimitClamper boundary

The MJCF joint limits are hard-coded from `x2_ultra.xml` so we don't depend
on a MuJoCo install. CSV column order matches `AgibotX2Ultra31DOF_CSVConfig`
in `soma_retargeter/assets/csv.py`.

Usage:
    python scripts/wrist_saturation_report.py scratch/csv/A021__h1.70_baseline.csv
    python scripts/wrist_saturation_report.py scratch/csv/A021__h1.40.csv --joints wrist
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np


# Limits transcribed from soma_retargeter/robot_assets/agibot_x2_ultra/x2_ultra.xml
# (joint range="lower upper" in radians).
JOINT_LIMITS_RAD = {
    "left_hip_pitch_joint":      (-2.704,  2.556),
    "left_hip_roll_joint":       (-0.235,  2.906),
    "left_hip_yaw_joint":        (-1.684,  3.430),
    "left_knee_joint":           ( 0.000,  2.4073),
    "left_ankle_pitch_joint":    (-0.803,  0.453),
    "left_ankle_roll_joint":     (-0.262,  0.262),
    "right_hip_pitch_joint":     (-2.704,  2.556),
    "right_hip_roll_joint":      (-2.906,  0.235),
    "right_hip_yaw_joint":       (-3.430,  1.684),
    "right_knee_joint":          ( 0.000,  2.4073),
    "right_ankle_pitch_joint":   (-0.803,  0.453),
    "right_ankle_roll_joint":    (-0.2625, 0.2625),
    "waist_yaw_joint":           (-1.5708, 1.5708),
    "waist_pitch_joint":         (-0.785,  0.785),
    "waist_roll_joint":          (-0.785,  0.785),
    "left_shoulder_pitch_joint": (-3.08,   2.04),
    "left_shoulder_roll_joint":  (-0.061,  2.993),
    "left_shoulder_yaw_joint":   (-2.556,  2.556),
    "left_elbow_joint":          (-2.3556, 0.0),
    "left_wrist_yaw_joint":      (-2.556,  2.556),
    "left_wrist_pitch_joint":    (-0.558,  0.558),
    "left_wrist_roll_joint":     (-1.571,  0.724),
    "right_shoulder_pitch_joint":(-3.08,   2.04),
    "right_shoulder_roll_joint": (-2.993,  0.061),
    "right_shoulder_yaw_joint":  (-2.556,  2.556),
    "right_elbow_joint":         (-2.3556, 0.0),
    "right_wrist_yaw_joint":     (-2.556,  2.556),
    "right_wrist_pitch_joint":   (-0.558,  0.558),
    "right_wrist_roll_joint":    (-0.724,  1.571),
    "head_yaw_joint":            (-2.0944, 2.0944),
    "head_pitch_joint":          (-0.523,  0.785),
}

# CSV column order from AgibotX2Ultra31DOF_CSVConfig.csv_header
CSV_HEADER = [
    "Frame",
    "root_translateX", "root_translateY", "root_translateZ",
    "root_rotateX", "root_rotateY", "root_rotateZ",
    "left_hip_pitch_joint_dof", "left_hip_roll_joint_dof", "left_hip_yaw_joint_dof",
    "left_knee_joint_dof", "left_ankle_pitch_joint_dof", "left_ankle_roll_joint_dof",
    "right_hip_pitch_joint_dof", "right_hip_roll_joint_dof", "right_hip_yaw_joint_dof",
    "right_knee_joint_dof", "right_ankle_pitch_joint_dof", "right_ankle_roll_joint_dof",
    "waist_yaw_joint_dof", "waist_pitch_joint_dof", "waist_roll_joint_dof",
    "left_shoulder_pitch_joint_dof", "left_shoulder_roll_joint_dof",
    "left_shoulder_yaw_joint_dof", "left_elbow_joint_dof",
    "left_wrist_yaw_joint_dof", "left_wrist_pitch_joint_dof", "left_wrist_roll_joint_dof",
    "right_shoulder_pitch_joint_dof", "right_shoulder_roll_joint_dof",
    "right_shoulder_yaw_joint_dof", "right_elbow_joint_dof",
    "right_wrist_yaw_joint_dof", "right_wrist_pitch_joint_dof", "right_wrist_roll_joint_dof",
    "head_yaw_joint_dof", "head_pitch_joint_dof",
]


def _select_joints(filter_substr: str | None) -> list[str]:
    if not filter_substr:
        return list(JOINT_LIMITS_RAD.keys())
    s = filter_substr.lower()
    return [j for j in JOINT_LIMITS_RAD if s in j]


def analyze(csv_path: Path, joints: list[str], near_deg: float) -> dict:
    data = np.loadtxt(csv_path, delimiter=",", skiprows=1, dtype=np.float64)
    # CSV stores joint dof angles in degrees from index 7 onward.
    name_to_col = {name.replace("_dof", ""): i for i, name in enumerate(CSV_HEADER)}

    results = {}
    for joint in joints:
        col_name = joint  # matches the CSV header without the trailing "_dof"
        col_idx = name_to_col.get(col_name)
        if col_idx is None:
            continue

        vals_deg = data[:, col_idx]
        lower_rad, upper_rad = JOINT_LIMITS_RAD[joint]
        lower_deg = math.degrees(lower_rad)
        upper_deg = math.degrees(upper_rad)
        midrange_deg = 0.5 * (lower_deg + upper_deg)
        half_range_deg = 0.5 * (upper_deg - lower_deg)

        offset_from_mid = vals_deg - midrange_deg
        near_lower = float((vals_deg <= lower_deg + near_deg).mean()) * 100.0
        near_upper = float((vals_deg >= upper_deg - near_deg).mean()) * 100.0
        max_overshoot_lo = float(max(0.0, lower_deg - vals_deg.min()))
        max_overshoot_hi = float(max(0.0, vals_deg.max() - upper_deg))

        results[joint] = dict(
            mean_deg=float(vals_deg.mean()),
            std_deg=float(vals_deg.std()),
            min_deg=float(vals_deg.min()),
            max_deg=float(vals_deg.max()),
            lower_deg=lower_deg,
            upper_deg=upper_deg,
            midrange_deg=midrange_deg,
            half_range_deg=half_range_deg,
            mean_offset_from_mid_deg=float(offset_from_mid.mean()),
            mean_abs_offset_from_mid_deg=float(np.abs(offset_from_mid).mean()),
            pct_near_lower=near_lower,
            pct_near_upper=near_upper,
            pct_near_either=near_lower + near_upper,
            overshoot_lower_deg=max_overshoot_lo,
            overshoot_upper_deg=max_overshoot_hi,
        )
    return results


def _format_row(name: str, r: dict) -> str:
    return (
        f"{name:<32}  "
        f"range=[{r['lower_deg']:+7.1f},{r['upper_deg']:+7.1f}]  "
        f"mid={r['midrange_deg']:+6.1f}  "
        f"mean={r['mean_deg']:+7.2f}  "
        f"off_mid={r['mean_offset_from_mid_deg']:+6.2f}  "
        f"std={r['std_deg']:5.2f}  "
        f"min/max={r['min_deg']:+7.2f}/{r['max_deg']:+7.2f}  "
        f"near_lo={r['pct_near_lower']:5.1f}%  "
        f"near_hi={r['pct_near_upper']:5.1f}%  "
        f"near={r['pct_near_either']:5.1f}%"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--joints", type=str, default=None,
                        help="Filter substring (e.g. 'wrist', 'hip', 'shoulder')")
    parser.add_argument("--near-deg", type=float, default=5.0,
                        help="How close to a limit counts as 'near' (deg, default 5)")
    parser.add_argument("--score-only", action="store_true",
                        help="Print one summary number per side: wrist near%% L/R sum")
    args = parser.parse_args()

    csv_path = args.csv.expanduser().resolve()
    joints = _select_joints(args.joints)
    res = analyze(csv_path, joints, args.near_deg)

    if args.score_only:
        wrist_keys = [j for j in res if "wrist" in j]
        score = sum(res[j]["pct_near_either"] for j in wrist_keys)
        print(f"{csv_path.name}\tscore_wrist_near%={score:.2f}")
        return

    print(f"# CSV : {csv_path}")
    print(f"# near threshold: {args.near_deg} deg")
    print(f"# joints       : {len(res)}")
    for name in joints:
        if name in res:
            print(_format_row(name, res[name]))


if __name__ == "__main__":
    main()

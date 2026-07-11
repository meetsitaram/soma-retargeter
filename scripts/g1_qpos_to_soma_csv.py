#!/usr/bin/env python3
"""Convert an external Unitree G1 qpos CSV into the soma-retargeter G1 CSV format.

Many G1 motion sources (LAFAN-style exports, sim/policy captures) ship a flat,
headerless qpos CSV: per row, `root_pos[m](3) + root_quat(4) + 29 joints[rad]`,
with joints in the standard G1 29-DOF order (left leg 6, right leg 6, waist 3,
left arm 7, right arm 7). This rewrites it into the format that
`app/g1_csv_to_x2_csv.py` (and `bvh_to_csv_converter`) consume:

    Frame, root_translate{X,Y,Z}[cm], root_rotate{X,Y,Z}[euler xyz deg],
    <29 named *_dof joints [deg]>

Then run:  app/g1_csv_to_x2_csv.py --g1-dir <out dir> --out-dir <x2 dir> [--acrobatic]

Quaternion order is auto-detected (identity-ish first frame) but can be forced
with --quat-order {xyzw,wxyz}.

IMPORTANT for fast full-rotation motion (backflips, cartwheels): use the ORIGINAL
capture, NOT a frame-rate-resampled export. Resampling (e.g. 30->50 fps) linearly
interpolates the root quaternion/height through the 360 deg flip and smears it,
which mangles the import. Prefer the source-fps CSV.
"""
import argparse
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

G1_JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
HEADER = "Frame,root_translateX,root_translateY,root_translateZ,root_rotateX,root_rotateY,root_rotateZ," \
         + ",".join(j + "_dof" for j in G1_JOINTS)


def _to_xyzw(quat: np.ndarray, order: str) -> np.ndarray:
    if order == "wxyz":
        return quat[:, [1, 2, 3, 0]]
    if order == "xyzw":
        return quat
    # auto: the near-1 component of the (near-identity) first frame is w
    w_idx = int(np.argmax(np.abs(quat[0])))
    return quat if w_idx == 3 else quat[:, [1, 2, 3, 0]]


def convert(src: str, dst: str, quat_order: str = "auto", has_header: bool = False) -> int:
    mat = np.loadtxt(src, delimiter=",", skiprows=1 if has_header else 0, dtype=np.float64)
    if mat.ndim == 1:
        mat = mat[None, :]
    if mat.shape[1] != 36:
        raise SystemExit(f"expected 36 cols (pos3+quat4+29joints), got {mat.shape[1]}")
    pos, quat, joints = mat[:, 0:3], mat[:, 3:7], mat[:, 7:36]
    euler_deg = R.from_quat(_to_xyzw(quat, quat_order)).as_euler("xyz", degrees=True)

    out = np.zeros((len(mat), 36))
    out[:, 0] = np.arange(len(mat))
    out[:, 1:4] = pos * 100.0            # m -> cm
    out[:, 4:7] = euler_deg             # euler xyz deg
    out[:, 7:36] = np.rad2deg(joints)   # rad -> deg

    Path(dst).parent.mkdir(parents=True, exist_ok=True)
    with open(dst, "w") as f:
        f.write(HEADER + "\n")
        np.savetxt(f, out, delimiter=",", fmt="%.6f")
    return len(mat)


def main():
    ap = argparse.ArgumentParser(description="External G1 qpos CSV -> soma-retargeter G1 CSV")
    ap.add_argument("--src", required=True, help="input qpos CSV (pos[m]+quat+29 joints[rad])")
    ap.add_argument("--dst", required=True, help="output G1 CSV (soma format)")
    ap.add_argument("--quat-order", choices=["auto", "xyzw", "wxyz"], default="auto")
    ap.add_argument("--has-header", action="store_true", help="skip a header row in --src")
    args = ap.parse_args()
    n = convert(args.src, args.dst, args.quat_order, args.has_header)
    print(f"wrote {args.dst}: {n} frames")


if __name__ == "__main__":
    main()

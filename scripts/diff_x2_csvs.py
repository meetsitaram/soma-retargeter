#!/usr/bin/env python3  # noqa: EXE001
# ruff: noqa: T201
"""Tier-1 reconstruction-error harness: diff two X2-Ultra retarget CSV sets.

Compares a *test* X2 retarget against a *ground-truth* X2 retarget (e.g. a
future G1->X2 output vs. `bones-seed/retargeted/x2_uniform_h14/`) and reports:

  - per-DOF joint-angle error (deg), aggregated per joint group  [cheap, no FK]
  - per-effector world-space error via X2 MuJoCo FK:
        position error (m) and orientation error (deg, sign-invariant geodesic)

The X2 CSV schema (38 cols) and FK conventions mirror the soma-retargeter bench
tooling (`scripts/bench/kinematics.py`, `joint_limits.py`): cm->m, euler-xyz-deg
->quat, deg->rad. Reference-free plausibility metrics (foot-floor, smoothness)
already live in that bench harness; this script adds the missing test-vs-ref diff.

Usage:
    .venv/bin/python scripts/diff_x2_csvs.py \
        --test <csv_or_dir> --ref <csv_or_dir> [--limit N] [--clips sub,str] [--json out.json]
"""

import argparse
import json
import os

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as R

# --- X2 Ultra 31-DOF CSV schema (cols 7:38), MJCF joint order ----------------
JOINT_NAMES = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_yaw_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_yaw_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint",
    "head_yaw_joint", "head_pitch_joint",
]
GROUPS = ["hip", "knee", "ankle", "waist", "shoulder", "elbow", "wrist", "head"]

# The 14 X2 IK-map effector bodies (same semantic set as the G1 tracked bodies).
EFFECTOR_BODIES = [
    "pelvis", "torso_link",
    "left_hip_roll_link", "left_knee_link", "left_ankle_roll_link",
    "right_hip_roll_link", "right_knee_link", "right_ankle_roll_link",
    "left_shoulder_roll_link", "left_elbow_link", "left_wrist_roll_link",
    "right_shoulder_roll_link", "right_elbow_link", "right_wrist_roll_link",
]

DEFAULT_MJCF = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "soma_retargeter/robot_assets/agibot_x2_ultra/x2_ultra.xml")


def load_model(mjcf):
    model = mujoco.MjModel.from_xml_path(mjcf)
    data = mujoco.MjData(model)
    # CSV joint col (7+k) -> qpos index; skip joints absent from the MJCF.
    j2q = []
    for k, jn in enumerate(JOINT_NAMES):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if jid >= 0:
            j2q.append((7 + k, int(model.jnt_qposadr[jid])))
    eff_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b) for b in EFFECTOR_BODIES]
    return model, data, j2q, eff_ids


def fk_effectors(model, data, mat, j2q, eff_ids):
    """FK every frame; return (T, n_eff, 7) world transforms [x,y,z, qx,qy,qz,qw]."""
    T = mat.shape[0]
    out = np.zeros((T, len(eff_ids), 7), dtype=np.float64)
    for t in range(T):
        row = mat[t]
        data.qpos[0:3] = row[1:4] * 0.01  # cm -> m
        q_xyzw = R.from_euler("xyz", np.deg2rad(row[4:7])).as_quat()
        data.qpos[3:7] = [q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]  # wxyz
        for csv_idx, qpos_idx in j2q:
            data.qpos[qpos_idx] = np.deg2rad(row[csv_idx])
        mujoco.mj_forward(model, data)
        for e, bid in enumerate(eff_ids):
            out[t, e, 0:3] = data.xpos[bid]
            w, x, y, z = data.xquat[bid]
            out[t, e, 3:7] = [x, y, z, w]  # xyzw
    return out


def geodesic_deg(qa, qb):
    """Sign-invariant angle (deg) between quaternion arrays (..., 4 xyzw)."""
    dot = np.abs(np.sum(qa * qb, axis=-1)).clip(0.0, 1.0)
    return np.degrees(2.0 * np.arccos(dot))


def diff_clip(test_csv, ref_csv, model, data, j2q, eff_ids):
    a = np.loadtxt(test_csv, delimiter=",", skiprows=1, dtype=np.float64)
    b = np.loadtxt(ref_csv, delimiter=",", skiprows=1, dtype=np.float64)
    if a.ndim == 1:
        a = a[None, :]
    if b.ndim == 1:
        b = b[None, :]
    T = min(len(a), len(b))
    a, b = a[:T], b[:T]

    # per-DOF joint-angle error (deg), no FK
    jerr = np.abs(a[:, 7:7 + len(JOINT_NAMES)] - b[:, 7:7 + len(JOINT_NAMES)])
    per_joint_mean = jerr.mean(axis=0)
    per_group = {
        g: float(np.mean([per_joint_mean[i] for i, jn in enumerate(JOINT_NAMES) if g in jn]))
        for g in GROUPS
    }

    # per-effector FK world error
    fa = fk_effectors(model, data, a, j2q, eff_ids)
    fb = fk_effectors(model, data, b, j2q, eff_ids)
    pos_err = np.linalg.norm(fa[:, :, 0:3] - fb[:, :, 0:3], axis=2)  # (T, n_eff) meters
    rot_err = geodesic_deg(fa[:, :, 3:7], fb[:, :, 3:7])            # (T, n_eff) deg
    per_body = {
        EFFECTOR_BODIES[e]: {
            "pos_m_mean": float(pos_err[:, e].mean()),
            "pos_m_max": float(pos_err[:, e].max()),
            "rot_deg_mean": float(rot_err[:, e].mean()),
        }
        for e in range(len(eff_ids))
    }
    return {
        "frames": T,
        "joint_deg_mean": float(per_joint_mean.mean()),
        "joint_deg_max": float(jerr.max()),
        "per_group_deg": per_group,
        "eff_pos_m_mean": float(pos_err.mean()),
        "eff_pos_m_p95": float(np.percentile(pos_err, 95)),
        "eff_rot_deg_mean": float(rot_err.mean()),
        "per_body": per_body,
    }


def index_csvs(path):
    """Return {clip_stem: csv_path}. Accepts a single CSV or a dir (recursive)."""
    if os.path.isfile(path):
        return {os.path.splitext(os.path.basename(path))[0]: path}
    out = {}
    for root, _, files in os.walk(path):
        for f in files:
            if f.endswith(".csv"):
                out.setdefault(os.path.splitext(f)[0], os.path.join(root, f))
    return out


def main():
    ap = argparse.ArgumentParser(description="Diff two X2 retarget CSV sets (Tier-1 metrics)")
    ap.add_argument("--test", required=True, help="test CSV or dir")
    ap.add_argument("--ref", required=True, help="ground-truth CSV or dir")
    ap.add_argument("--mjcf", default=DEFAULT_MJCF)
    ap.add_argument("--clips", default=None, help="comma-separated substrings to filter clip stems")
    ap.add_argument("--limit", type=int, default=None, help="max clips to diff")
    ap.add_argument("--json", default=None, help="write full per-clip results to this JSON")
    args = ap.parse_args()

    test_idx, ref_idx = index_csvs(args.test), index_csvs(args.ref)
    stems = sorted(set(test_idx) & set(ref_idx))
    if args.clips:
        subs = [s.strip() for s in args.clips.split(",")]
        stems = [s for s in stems if any(sub in s for sub in subs)]
    if not stems:
        raise SystemExit("No common clip stems between --test and --ref")
    if args.limit:
        stems = stems[: args.limit]

    model, data, j2q, eff_ids = load_model(args.mjcf)
    print(f"Diffing {len(stems)} clip(s)  |  test={args.test}  ref={args.ref}")

    results = {}
    agg_joint, agg_pos, agg_rot = [], [], []
    body_pos_acc = {b: [] for b in EFFECTOR_BODIES}
    for s in stems:
        r = diff_clip(test_idx[s], ref_idx[s], model, data, j2q, eff_ids)
        results[s] = r
        agg_joint.append(r["joint_deg_mean"])
        agg_pos.append(r["eff_pos_m_mean"])
        agg_rot.append(r["eff_rot_deg_mean"])
        for b in EFFECTOR_BODIES:
            body_pos_acc[b].append(r["per_body"][b]["pos_m_mean"])

    print("\n=== AGGREGATE (mean over clips) ===")
    print(f"  joint-angle err : {np.mean(agg_joint):7.3f} deg")
    print(f"  effector pos err: {np.mean(agg_pos):7.4f} m")
    print(f"  effector rot err: {np.mean(agg_rot):7.3f} deg")
    print("\n  worst effector bodies by mean pos err (m):")
    for b in sorted(EFFECTOR_BODIES, key=lambda b: -np.mean(body_pos_acc[b]))[:6]:
        print(f"    {b:26s} {np.mean(body_pos_acc[b]):.4f}")

    if args.json:
        with open(args.json, "w") as f:
            json.dump({"clips": results}, f, indent=2)
        print(f"\nWrote per-clip results -> {args.json}")


if __name__ == "__main__":
    main()

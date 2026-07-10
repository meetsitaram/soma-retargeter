#!/usr/bin/env python3  # noqa: EXE001
# ruff: noqa: T201
"""Side-by-side MuJoCo viewer: G1 source | X2-from-G1 | X2 ground-truth, in sync.

Merges the G1 and two X2 MJCFs into one scene (mjSpec attach) and drives each
robot from its retarget CSV, so you can visually judge whether the direct
G1->X2 keypoint-injection retarget is faithful. Robots are placed in parallel
lanes (Y offset) and each clip's initial root XY is zeroed so they stay framed.

Run (needs a display; opens an interactive window you can orbit/pause):
    .venv/bin/python scripts/view_g1_x2.py --clip arc_walk_left_loop_001__A029
Controls: Space pause, mouse orbit/zoom. Loops the clip continuously.
"""

import argparse
import os
import time

import mujoco
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation as R

import newton  # noqa: E402
_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
G1_MJCF = str(newton.utils.download_asset("unitree_g1") / "mjcf" / "g1_29dof_rev_1_0.xml")
X2_MJCF = os.path.join(_REPO, "soma_retargeter/robot_assets/agibot_x2_ultra/x2_ultra.xml")
G1_DIR = ""            # pass via --g1-dir
X2_FROM_G1_DIR = ""    # pass via --x2-dir
X2_GT_DIR = ""         # optional via --x2-gt-dir

LANES = [("G1 (source)", 0.0), ("X2-from-G1", 1.4), ("X2 ground-truth", 2.8)]


def build_scene(prefixes):
    spec = mujoco.MjSpec.from_file(G1_MJCF)
    for prefix in prefixes:
        child = mujoco.MjSpec.from_file(X2_MJCF)
        frame = spec.worldbody.add_frame()
        frame.attach_body(child.worldbody.first_body(), prefix, "")
    model = spec.compile()
    return model, mujoco.MjData(model)


def robot_maps(model, header_cols, prefix):
    """(free_qpos_adr, [(csv_col, qpos_adr)]) for a robot given its CSV joint headers."""
    free_adr = None
    for j in range(model.njnt):
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) or ""
            if (prefix and name.startswith(prefix)) or (not prefix and not name.startswith(("x2a_", "x2b_"))):
                free_adr = int(model.jnt_qposadr[j])
    pairs = []
    for k, col_name in enumerate(header_cols[7:]):  # cols 7.. are joint DOFs
        jn = prefix + col_name.replace("_dof", "")
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if jid >= 0:
            pairs.append((7 + k, int(model.jnt_qposadr[jid])))
    return free_adr, pairs


def load_csv(path):
    header = open(path).readline().strip().split(",")
    mat = np.loadtxt(path, delimiter=",", skiprows=1, dtype=np.float64)
    if mat.ndim == 1:
        mat = mat[None, :]
    return header, mat


def find_gt(gt_dir, stem):
    for root, _, files in os.walk(gt_dir):
        if f"{stem}.csv" in files:
            return os.path.join(root, f"{stem}.csv")
    return None


def set_robot(data, mat, frame, free_adr, pairs, y_off, root0):
    row = mat[min(frame, len(mat) - 1)]
    data.qpos[free_adr + 0] = row[1] * 0.01 - root0[0]
    data.qpos[free_adr + 1] = row[2] * 0.01 - root0[1] + y_off
    data.qpos[free_adr + 2] = row[3] * 0.01
    q = R.from_euler("xyz", np.deg2rad(row[4:7])).as_quat()  # xyzw
    data.qpos[free_adr + 3:free_adr + 7] = [q[3], q[0], q[1], q[2]]  # wxyz
    for csv_col, qadr in pairs:
        data.qpos[qadr] = np.deg2rad(row[csv_col])


def main():
    ap = argparse.ArgumentParser(description="G1 | X2-from-G1 | X2-GT side-by-side viewer")
    ap.add_argument("--clip", default="arc_walk_left_loop_001__A029")
    ap.add_argument("--g1-dir", default=G1_DIR, help="dir of G1 retarget CSVs (lane 0)")
    ap.add_argument("--x2-dir", default=X2_FROM_G1_DIR, help="dir of X2-from-G1 CSVs (lane 1)")
    ap.add_argument("--x2-gt-dir", default=X2_GT_DIR, help="dir of X2 ground-truth CSVs (lane 2; optional)")
    ap.add_argument("--speed", type=float, default=0.5, help="playback speed multiplier")
    args = ap.parse_args()

    g1_csv = f"{args.g1_dir}/{args.clip}.csv"
    x2a_csv = f"{args.x2_dir}/{args.clip}.csv"
    for label, p in [("G1", g1_csv), ("X2-from-G1", x2a_csv)]:
        if not os.path.exists(p):
            raise SystemExit(f"missing {label} CSV for clip '{args.clip}': {p}")
    x2b_csv = find_gt(args.x2_gt_dir, args.clip)

    prefixes = ["x2a_"] + (["x2b_"] if x2b_csv else [])
    model, data = build_scene(prefixes)
    gh, gm = load_csv(g1_csv)
    g_free, g_pairs = robot_maps(model, gh, "")
    ah, am = load_csv(x2a_csv)
    a_free, a_pairs = robot_maps(model, ah, "x2a_")
    lanes = [(gm, g_free, g_pairs, LANES[0][1], [gm[0, 1] * 0.01, gm[0, 2] * 0.01]),
             (am, a_free, a_pairs, LANES[1][1], [am[0, 1] * 0.01, am[0, 2] * 0.01])]
    labels = ["G1", "X2-from-G1"]
    if x2b_csv:
        bh, bm = load_csv(x2b_csv)
        b_free, b_pairs = robot_maps(model, bh, "x2b_")
        lanes.append((bm, b_free, b_pairs, LANES[2][1], [bm[0, 1] * 0.01, bm[0, 2] * 0.01]))
        labels.append("X2-GT")
    else:
        print(f"NOTE: no X2 ground-truth for '{args.clip}' in {args.x2_gt_dir} — showing 2 lanes")

    T = min(len(lane[0]) for lane in lanes)
    dt = 1.0 / (120.0 * args.speed)
    print(f"Viewing '{args.clip}': {T} frames, lanes: {labels}")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            for f in range(T):
                if not viewer.is_running():
                    break
                for mat, free, pairs, yoff, r0 in lanes:
                    set_robot(data, mat, f, free, pairs, yoff, r0)
                mujoco.mj_forward(model, data)
                viewer.sync()
                time.sleep(dt)


if __name__ == "__main__":
    main()

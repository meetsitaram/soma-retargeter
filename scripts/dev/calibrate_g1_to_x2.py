#!/usr/bin/env python3
"""Re-derive the G1 -> X2 calibration config (reproducibility / maintenance tool).

Regenerates soma_retargeter/configs/agibot_x2_ultra/g1_to_x2_ultra_calibration.json,
which is otherwise a frozen artifact. Run this only when the robot models or the
paired reference data change. The G1->X2 method itself does NOT need this script.

It derives all three pieces of the calibration:
  * position_scale     - ratio of X2/G1 rest pelvis heights (deterministic).
  * shoulder_elbow_fit - per-joint linear fit  X2 = a*G1 + b  from paired
                         (G1 CSV, X2-GT CSV) clips (shoulders/elbows correlate 0.8-0.96).
  * wrist_remap        - FUNCTIONAL joint correspondence + sign, derived from joint-axis
                         geometry: the pronation DOF (axis parallel to the forearm) is
                         G1 `wrist_roll` but X2 `wrist_yaw` (the vendor names differ);
                         its sign comes from the signed axis-vs-forearm alignment. Flexion
                         (pitch<->pitch) and deviation (the remaining pair) keep sign +1.

Run in the retargeter venv from the repo root:
  .venv/bin/python scripts/dev/calibrate_g1_to_x2.py \
      --fit-pairs g1a.csv:x2gta.csv g1b.csv:x2gtb.csv ... [--out <path>] [--print-only]
"""

import argparse
import json
import sys
from pathlib import Path

import mujoco
import newton
import numpy as np
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # repo root
from soma_retargeter.pipelines import g1_to_x2_pipeline as g1x2  # noqa: E402

X2_MJCF = g1x2._X2_MJCF
DEFAULT_OUT = g1x2._CFG_DIR / "g1_to_x2_ultra_calibration.json"

SHOULDER_ELBOW = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
]
WRIST_DOF = ["wrist_roll_joint", "wrist_pitch_joint", "wrist_yaw_joint"]  # names differ per robot
# distal hand body per robot (end of the wrist chain) for the forearm direction
G1_HAND, X2_HAND = "wrist_yaw_link", "wrist_roll_link"


def _colmap(csv):
    h = open(csv).readline().strip().split(",")
    return {n.replace("_dof", ""): i for i, n in enumerate(h)}


def _position_scale():
    """X2/G1 retarget model-height ratio (the height each robot was retargeted to)."""
    x2_cfg = json.load(open(g1x2._CFG_DIR / "soma_to_x2_ultra_retargeter_config.json"))
    g1_cfg = json.load(open(g1x2._PKG / "configs" / "unitree_g1" / "soma_to_g1_retargeter_config.json"))
    return round(float(x2_cfg["model_height"]) / float(g1_cfg["model_height"]), 4)


def _signed_along_forearm(mjcf, hand_body):
    """For left arm at rest: signed (joint world axis . forearm dir) per wrist DOF."""
    m = mujoco.MjModel.from_xml_path(mjcf)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    bid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
    forearm = d.xpos[bid(f"left_{hand_body}")] - d.xpos[bid("left_elbow_link")]
    forearm /= np.linalg.norm(forearm)
    out = {}
    for j in WRIST_DOF:
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"left_{j}")
        ax = d.xmat[m.jnt_bodyid[jid]].reshape(3, 3) @ m.jnt_axis[jid]
        out[j] = float((ax / np.linalg.norm(ax)) @ forearm)
    return out


def fit_shoulder_elbow(pairs):
    fit = {}
    for j in SHOULDER_ELBOW:
        G, X = [], []
        for g, x in pairs:
            gm = np.loadtxt(g, delimiter=",", skiprows=1)
            xm = np.loadtxt(x, delimiter=",", skiprows=1)
            T = min(len(gm), len(xm))
            G.append(gm[:T, _colmap(g)[j]])
            X.append(xm[:T, _colmap(x)[j]])
        a, b = np.polyfit(np.concatenate(G), np.concatenate(X), 1)
        fit[j] = [round(float(a), 5), round(float(b), 4)]
    return fit


def derive_wrist_remap(g1_mjcf):
    """Return {x2_joint: [g1_joint, sign]} for both arms, derived from axis geometry."""
    g_along = _signed_along_forearm(g1_mjcf, G1_HAND)   # G1 wrist DOF vs forearm
    x_along = _signed_along_forearm(X2_MJCF, X2_HAND)   # X2 wrist DOF vs forearm
    g_pron = max(g_along, key=lambda j: abs(g_along[j]))  # axis most parallel to forearm
    x_pron = max(x_along, key=lambda j: abs(x_along[j]))
    pron_sign = float(np.sign(g_along[g_pron]) * np.sign(x_along[x_pron]))  # X2 = sign * G1
    print(f"  pronation (axis ~parallel to forearm):  G1 {g_pron}  <->  X2 {x_pron}  sign {pron_sign:+.0f}")
    print(f"  signed axis.forearm  G1: {{{', '.join(f'{k}:{v:+.2f}' for k,v in g_along.items())}}}")
    print(f"                       X2: {{{', '.join(f'{k}:{v:+.2f}' for k,v in x_along.items())}}}")
    # remaining two DOF: flexion (pitch<->pitch) and deviation (the leftover pair), sign +1
    g_rest = [j for j in WRIST_DOF if j != g_pron]
    x_rest = [j for j in WRIST_DOF if j != x_pron]
    g_pitch = "wrist_pitch_joint"
    x_pitch = "wrist_pitch_joint"
    g_dev = [j for j in g_rest if j != g_pitch][0]
    x_dev = [j for j in x_rest if j != x_pitch][0]
    base = {x_pron: [g_pron, pron_sign], x_pitch: [g_pitch, 1.0], x_dev: [g_dev, 1.0]}
    remap = {}
    for side in ("left", "right"):
        for xj, (gj, s) in base.items():
            remap[f"{side}_{xj}"] = [f"{side}_{gj}", s]
    return remap


def main():
    ap = argparse.ArgumentParser(description="Re-derive the G1->X2 calibration config")
    ap.add_argument("--fit-pairs", nargs="+", required=True, help="g1csv:x2gtcsv pairs for shoulder/elbow fit")
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    ap.add_argument("--g1-mjcf", default=None)
    ap.add_argument("--print-only", action="store_true", help="print the derived config; do not write")
    args = ap.parse_args()

    g1_mjcf = args.g1_mjcf or str(newton.utils.download_asset("unitree_g1") / "mjcf" / "g1_29dof_rev_1_0.xml")

    print("position_scale (X2/G1 retarget model-height ratio):")
    scale = _position_scale()
    print(f"  {scale}")
    print("wrist_remap (functional, from joint-axis geometry):")
    remap = derive_wrist_remap(g1_mjcf)
    print("shoulder_elbow_fit (linear fit from paired clips):")
    fit = fit_shoulder_elbow([tuple(p.split(":")) for p in args.fit_pairs])
    for j, (a, b) in fit.items():
        print(f"  {j:28s} a={a:6.2f} b={b:7.1f}")

    calib = {
        "_note": "Self-contained G1->X2 calibration. Regenerate with scripts/dev/calibrate_g1_to_x2.py.",
        "position_scale": scale,
        "position_scale_center": "clip_start_floor",
        "shoulder_elbow_fit": fit,
        "wrist_remap": remap,
        "wrist_remap_note": "Pronation joint differs by vendor name: G1 wrist_roll == X2 wrist_yaw (axis along forearm). Values clamped to X2 joint limits at apply time.",
    }
    if args.print_only:
        print("\n" + json.dumps(calib, indent=2))
    else:
        json.dump(calib, open(args.out, "w"), indent=2)
        print(f"\nWrote {args.out}")


if __name__ == "__main__":
    main()

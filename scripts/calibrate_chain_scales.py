"""Derive per-chain ``joint_scales`` from robot MJCF + SOMA BVH rest poses.

The default X2 scaler config (`soma_to_x2_ultra_scaler_config.json`) was
copied from G1 and hand-tuned by eye. That's why we see ~31 % floating
(`x2_uniform_h170_tuned`, h=1.70+wrist_smooth) or 100 % penetration
(`x2_uniform_h140`, h=1.40) on the walk anchor — the human's chain
lengths don't match X2's anatomy.

This script computes anatomically-grounded `joint_scales` by measuring each
chain's rest-pose length on both sides:

    joint_scale_j = ||robot_link_j  − robot_pelvis|| /
                    ||human_joint_j − human_hips||

For the root (Hips) we use vertical height instead of magnitude, since the
root's "scale" governs how high the pelvis is lifted above the floor.

The result is emitted as a new scaler config (``..._chain_matched_config.json``).
When the matching retargeter config sets ``model_height = human_height_assumption``
(both 1.8 by default), the scaler's ``ratio`` factor becomes 1.0 and the
per-chain ratios go straight to the IK targets — i.e. the resized BVH
skeleton lines up with the robot's anatomy.

Usage:

    uv run python scripts/calibrate_chain_scales.py \
        --mjcf soma_retargeter/robot_assets/agibot_x2_ultra/x2_ultra.xml \
        --bvh  soma_retargeter/configs/soma/soma_zero_frame0.bvh \
        --src-scaler-config soma_retargeter/configs/agibot_x2_ultra/soma_to_x2_ultra_scaler_config.json \
        --out soma_retargeter/configs/agibot_x2_ultra/soma_to_x2_ultra_chain_matched_config.json

The mapping between BVH joints and MJCF bodies for X2 Ultra is hard-coded
below (`_DEFAULT_MAPPING_X2`). Add a new mapping dict to support a new
robot — the rest of the pipeline is anatomy-agnostic.
"""

from __future__ import annotations

import argparse
import json
import sys
from copy import deepcopy
from pathlib import Path
from typing import Dict

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

import soma_retargeter.assets.bvh as bvh_utils  # noqa: E402
import soma_retargeter.utils.io_utils as io_utils  # noqa: E402
import soma_retargeter.utils.pose_utils as pose_utils  # noqa: E402


# ---------------------------------------------------------------------------
# Mapping BVH joints -> MJCF body chains (X2 Ultra)
# ---------------------------------------------------------------------------
# Each entry maps a BVH joint name (used by `joint_scales`) to the ordered
# list of MJCF bodies whose offsets, when summed, equal the chain length
# from the root (pelvis) to that joint. This mirrors the BVH hierarchy of
# parent-relative offsets — we use bone-length sums (not bind-pose
# positions) because the SOMA Uniform bind pose has limbs extending along
# world +X and ends up asymmetric when computed from positions alone.
#
# Bone lengths are pose-invariant and symmetric: LeftFoot chain length =
# RightFoot chain length, etc.
#
# The X2 doesn't have toe bodies, so `LeftToe`/`LeftToeBase` reuse the
# ankle. Adding a new robot: copy this dict and adjust the right-hand side.
_DEFAULT_MAPPING_X2: Dict[str, list[str]] = {
    "Hips":         [],  # handled specially: vertical height to floor
    "Chest":        ["waist_yaw_link", "waist_pitch_link", "torso_link"],
    "Neck1":        ["waist_yaw_link", "waist_pitch_link", "torso_link", "head_yaw_link"],
    "LeftLeg":      ["left_hip_pitch_link"],
    "LeftShin":     ["left_hip_pitch_link", "left_hip_roll_link", "left_hip_yaw_link", "left_knee_link"],
    "LeftFoot":     ["left_hip_pitch_link", "left_hip_roll_link", "left_hip_yaw_link",
                     "left_knee_link", "left_ankle_pitch_link", "left_ankle_roll_link"],
    "LeftToe":      ["left_hip_pitch_link", "left_hip_roll_link", "left_hip_yaw_link",
                     "left_knee_link", "left_ankle_pitch_link", "left_ankle_roll_link"],
    "LeftToeBase":  ["left_hip_pitch_link", "left_hip_roll_link", "left_hip_yaw_link",
                     "left_knee_link", "left_ankle_pitch_link", "left_ankle_roll_link"],
    "RightLeg":     ["right_hip_pitch_link"],
    "RightShin":    ["right_hip_pitch_link", "right_hip_roll_link", "right_hip_yaw_link", "right_knee_link"],
    "RightFoot":    ["right_hip_pitch_link", "right_hip_roll_link", "right_hip_yaw_link",
                     "right_knee_link", "right_ankle_pitch_link", "right_ankle_roll_link"],
    "RightToe":     ["right_hip_pitch_link", "right_hip_roll_link", "right_hip_yaw_link",
                     "right_knee_link", "right_ankle_pitch_link", "right_ankle_roll_link"],
    "RightToeBase": ["right_hip_pitch_link", "right_hip_roll_link", "right_hip_yaw_link",
                     "right_knee_link", "right_ankle_pitch_link", "right_ankle_roll_link"],
    "LeftArm":      ["waist_yaw_link", "waist_pitch_link", "torso_link",
                     "left_shoulder_pitch_link"],
    "LeftForeArm":  ["waist_yaw_link", "waist_pitch_link", "torso_link",
                     "left_shoulder_pitch_link", "left_shoulder_roll_link", "left_shoulder_yaw_link",
                     "left_elbow_link"],
    "LeftHand":     ["waist_yaw_link", "waist_pitch_link", "torso_link",
                     "left_shoulder_pitch_link", "left_shoulder_roll_link", "left_shoulder_yaw_link",
                     "left_elbow_link", "left_wrist_yaw_link", "left_wrist_pitch_link", "left_wrist_roll_link"],
    "RightArm":     ["waist_yaw_link", "waist_pitch_link", "torso_link",
                     "right_shoulder_pitch_link"],
    "RightForeArm": ["waist_yaw_link", "waist_pitch_link", "torso_link",
                     "right_shoulder_pitch_link", "right_shoulder_roll_link", "right_shoulder_yaw_link",
                     "right_elbow_link"],
    "RightHand":    ["waist_yaw_link", "waist_pitch_link", "torso_link",
                     "right_shoulder_pitch_link", "right_shoulder_roll_link", "right_shoulder_yaw_link",
                     "right_elbow_link", "right_wrist_yaw_link", "right_wrist_pitch_link", "right_wrist_roll_link"],
}


# ---------------------------------------------------------------------------
# Robot chain-length extraction (MJCF parent-relative offsets)
# ---------------------------------------------------------------------------

def get_robot_local_offsets(mjcf_path: Path) -> Dict[str, np.ndarray]:
    """Return {body_name: parent-relative offset[3]} for every body in MJCF.

    Reads `body_pos` from the model directly, which is what the MJCF
    `<body pos="..."/>` attribute encodes (parent-relative translation).
    Bone length = ``np.linalg.norm(body_pos[i])``.
    """
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    out: Dict[str, np.ndarray] = {}
    for i in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        if name is None:
            continue
        out[name] = np.array(model.body_pos[i], dtype=np.float64)
    return out


def robot_chain_length(
    offsets: Dict[str, np.ndarray], chain_bodies: list[str],
) -> float:
    """Sum of |body_pos| over each body in ``chain_bodies``.

    Equivalent to "how far does this chain reach if all hinge joints are
    rotated to align the chain in a single direction" — i.e. the chain's
    max physical extent from the root.
    """
    total = 0.0
    for name in chain_bodies:
        if name not in offsets:
            raise KeyError(f"[ERROR]: body not found in MJCF: {name}")
        total += float(np.linalg.norm(offsets[name]))
    return total


def get_robot_world_positions(mjcf_path: Path, body_names: list[str]) -> Dict[str, np.ndarray]:
    """Return {body_name: world_pos[3]} after ``mj_forward`` at qpos=0.

    Used for the Hips-scale floor calculation (where we want the world
    height of the pelvis, not chain length) and the foot-floor offset
    calculation (where we want the world height of the ankle).
    """
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    out: Dict[str, np.ndarray] = {}
    for name in set(body_names) | {"pelvis", "left_ankle_roll_link", "right_ankle_roll_link"}:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid < 0:
            raise KeyError(f"[ERROR]: body not found in MJCF: {name}")
        out[name] = np.array(data.xpos[bid], dtype=np.float64)
    return out


# ---------------------------------------------------------------------------
# BVH chain-length extraction
# ---------------------------------------------------------------------------

def get_bvh_local_offsets(bvh_path: Path) -> tuple[Dict[str, np.ndarray], list[str], np.ndarray]:
    """Return ({joint_name: parent-relative offset[3]}, joint_names, parent_indices).

    The BVH bind pose's per-joint TRANSLATION (first 3 components of
    ``reference_local_transforms``) is the parent-relative offset in
    centimetres-converted-to-meters by the importer. Bone lengths are
    pose-invariant — they don't depend on the bind-pose rotations, which
    is what makes them suitable for chain calibration.
    """
    importer = bvh_utils.BVHImporter()
    skel, _ = importer.create_skeleton(bvh_path)
    ref = skel.reference_local_transforms  # (N, 7) -> [tx, ty, tz, qx, qy, qz, qw]
    out: Dict[str, np.ndarray] = {}
    for jn in skel.joint_names:
        idx = skel.joint_index(jn)
        if idx < 0:
            continue
        out[jn] = np.array(ref[idx, :3], dtype=np.float64)
    return out, list(skel.joint_names), np.asarray(skel.parent_indices)


def bvh_chain_to(joint_name: str, joint_names: list[str], parent_indices: np.ndarray,
                 offsets: Dict[str, np.ndarray], stop_at: str = "Hips") -> float:
    """Sum of bone lengths from ``stop_at`` (exclusive) to ``joint_name`` (inclusive).

    Walks parents from ``joint_name`` upward until reaching ``stop_at``.
    """
    if joint_name not in offsets:
        raise KeyError(f"[ERROR]: BVH joint not found: {joint_name}")
    total = 0.0
    idx = joint_names.index(joint_name)
    stop_idx = joint_names.index(stop_at) if stop_at in joint_names else -1
    while idx != stop_idx and idx >= 0:
        total += float(np.linalg.norm(offsets[joint_names[idx]]))
        idx = int(parent_indices[idx])
        if idx < 0:
            break
    return total


# ---------------------------------------------------------------------------
# Chain scale calibration
# ---------------------------------------------------------------------------

def compute_chain_scales(
    mjcf_path: Path,
    bvh_path: Path,
    mapping: Dict[str, list[str]],
) -> tuple[Dict[str, float], Dict[str, list[float]], list[dict]]:
    """For each BVH joint, compute scale = robot_chain_len / bvh_chain_len.

    Both chain lengths are *sums of parent-relative bone offsets along the
    kinematic chain from the root* — the BVH bind pose's pose-invariant
    structural measure. This gives symmetric results (left vs right) and
    avoids the asymmetry caused by the SOMA Uniform's bind pose having
    limbs extended along world +X.

    ``Hips`` is handled separately: its scale is the floor-to-pelvis
    vertical-height ratio (robot vs canonical human). This governs the
    world height of the resized pelvis.
    """
    robot_offsets = get_robot_local_offsets(mjcf_path)
    robot_world = get_robot_world_positions(mjcf_path, ["pelvis"])
    bvh_offsets, bvh_joint_names, bvh_parents = get_bvh_local_offsets(bvh_path)

    scales: Dict[str, float] = {}
    audit: list[dict] = []

    # ---- Hips: floor-to-pelvis height ratio -----------------------------
    # Robot: pelvis Z above ground at MJCF rest pose.
    robot_pelvis_height = float(robot_world["pelvis"][2])
    # Human: bind-pose Hips Y coordinate in BVH frame. Standard SOMA BVH
    # convention is "floor at origin, +Y up", and after the runtime
    # SpaceConverter rotates (Rx 90°), BVH-Y becomes MuJoCo-Z. So the
    # bind-pose Hips Y is exactly the world-frame pelvis height that the
    # scaler will scale at runtime.
    #
    # Using SUM of leg bones overestimates this — that sum is what the
    # human's pelvis WOULD be at if the leg were perfectly straight, but
    # the BVH bind pose has a small offset (~12 cm) folded into per-joint
    # translations that the chain sum doesn't see.
    bvh_hips_pos = bvh_offsets.get("Hips", np.zeros(3))
    bvh_hips_height_Y = float(bvh_hips_pos[1])  # +Y up in raw BVH
    if bvh_hips_height_Y <= 0:
        # Sanity fallback: some skeletons may use +Z up. Try Z.
        bvh_hips_height_Y = float(abs(bvh_hips_pos[2]))
    if bvh_hips_height_Y <= 0:
        raise RuntimeError("[ERROR]: BVH Hips height is 0 — check coordinate convention")
    hips_scale = float(robot_pelvis_height / bvh_hips_height_Y)
    scales["Hips"] = hips_scale
    audit.append(dict(
        joint="Hips",
        robot_chain=["pelvis (world Z)"],
        bvh_chain="Hips bind-pose world Y",
        robot_len=robot_pelvis_height,
        bvh_len=bvh_hips_height_Y,
        scale=hips_scale,
        note="vertical height: robot_pelvis_Z / BVH_Hips_world_Y_bind",
    ))

    # ---- All other chains: bone-length sum ratio ------------------------
    for bvh_joint, robot_chain in mapping.items():
        if bvh_joint == "Hips":
            continue
        if bvh_joint not in bvh_offsets:
            print(f"[WARN] skip {bvh_joint}: not in BVH skeleton")
            continue
        r_len = robot_chain_length(robot_offsets, robot_chain)
        h_len = bvh_chain_to(bvh_joint, bvh_joint_names, bvh_parents, bvh_offsets, stop_at="Hips")
        if h_len <= 0:
            print(f"[WARN] skip {bvh_joint}: empty BVH chain")
            continue
        s = float(r_len / h_len)
        scales[bvh_joint] = s
        audit.append(dict(
            joint=bvh_joint,
            robot_chain=robot_chain,
            bvh_chain=f"Hips→{bvh_joint}",
            robot_len=r_len,
            bvh_len=h_len,
            scale=s,
            note="sum-of-bones ratio",
        ))

    # ---- World-space foot-floor offset ---------------------------------
    # The scaler computes a foot IK target whose world Z is:
    #   t.z = (bvh_foot.z - bvh_hips.z) * Fs + bvh_hips.z * Hs + rotated_offset.z
    # At stance (bvh_foot.z == 0) and bind hips height H:
    #   t.z = H * (Hs - Fs) + rotated_offset.z
    # We want t.z to equal the robot's natural ankle rest Z so the foot
    # mesh (which hangs below the ankle joint) rests exactly on the
    # floor. The fix is a constant *world-space* upward offset:
    #   world_offset_z = robot_rest_ankle_Z - H * (Hs - Fs)
    # applied only to LeftFoot / RightFoot (the IK target bodies are the
    # ankle-roll links). The leg chain joints (LeftLeg, LeftShin) get
    # the same lift so the IK doesn't see a vertically broken leg.
    world_offsets: Dict[str, list[float]] = {}
    if "LeftFoot" in scales and "RightFoot" in scales:
        l_ankle_z = float(robot_world["left_ankle_roll_link"][2])
        r_ankle_z = float(robot_world["right_ankle_roll_link"][2])
        H = bvh_hips_height_Y
        Hs = scales["Hips"]
        for side in ("Left", "Right"):
            ankle_z = l_ankle_z if side == "Left" else r_ankle_z
            Fs = scales[f"{side}Foot"]
            off_z = float(ankle_z - H * (Hs - Fs))
            # Apply the lift ONLY to the ankle/toe IK targets. Lifting
            # the hip / knee targets too would push them above their
            # geometric parents (e.g. hip above pelvis), which is
            # impossible and corrupts the IK solve. The foot target has
            # the dominant IK weight (30×), so once the ankle lands at
            # rest sole-Z, the unweighted leg chain naturally posses
            # itself around it.
            for j in (f"{side}Foot", f"{side}ToeBase", f"{side}Toe"):
                if j in scales:
                    world_offsets[j] = [0.0, 0.0, off_z]
            audit.append(dict(
                joint=f"{side}_foot_floor_offset",
                robot_chain=[f"{side.lower()}_ankle_roll_link.world_Z"],
                bvh_chain=f"H*(Hs-Fs); H={H:.3f}, Hs={Hs:.4f}, Fs={Fs:.4f}",
                robot_len=ankle_z,
                bvh_len=H * (Hs - Fs),
                scale=off_z,
                note="world_offsets[*Foot/Toe*] z-lift",
            ))

    return scales, world_offsets, audit


# ---------------------------------------------------------------------------
# Emit scaler config
# ---------------------------------------------------------------------------

def build_scaler_config(
    src_cfg_path: Path,
    new_scales: Dict[str, float],
    new_world_offsets: Dict[str, list[float]],
    human_height_assumption: float = 1.8,
) -> dict:
    """Take an existing scaler config and replace its `joint_scales` block.

    We deliberately keep `joint_offsets`, `joint_parents`, `robot_type`,
    and the original joint set untouched — those are pose offsets that
    were tuned for the robot's IK targets and aren't height-dependent.
    Only the chain-length scales change, plus an optional
    `world_offsets` dict for the foot-floor lift.
    """
    src = json.loads(src_cfg_path.read_text())
    out = deepcopy(src)
    out["human_height_assumption"] = float(human_height_assumption)

    merged = dict(out.get("joint_scales", {}))
    for k, v in new_scales.items():
        merged[k] = float(v)
    out["joint_scales"] = merged

    merged_world = dict(out.get("world_offsets", {}))
    for k, v in new_world_offsets.items():
        merged_world[k] = [float(x) for x in v]
    if merged_world:
        out["world_offsets"] = merged_world
    return out


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mjcf", type=Path, required=True,
                    help="Robot MJCF file (e.g. soma_retargeter/robot_assets/.../x2_ultra.xml)")
    ap.add_argument("--bvh", type=Path, required=True,
                    help="SOMA zero-pose BVH used as the human anatomy reference")
    ap.add_argument("--src-scaler-config", type=Path, required=True,
                    help="Existing scaler config to use as a template (joint_offsets, etc.)")
    ap.add_argument("--out", type=Path, required=True,
                    help="Output scaler config JSON path")
    ap.add_argument("--human-height-assumption", type=float, default=1.8,
                    help="Value written into the new config. Pair with a retargeter "
                         "config that sets `model_height` to the same value so that "
                         "the scaler's `ratio` factor is 1.0.")
    ap.add_argument("--mapping", type=Path, default=None,
                    help="Optional JSON file mapping bvh_joint -> mjcf_body. "
                         "Defaults to the built-in X2 Ultra mapping.")
    args = ap.parse_args()

    if args.mapping is not None:
        mapping = json.loads(args.mapping.read_text())
    else:
        mapping = _DEFAULT_MAPPING_X2

    print(f"[INFO] MJCF:  {args.mjcf}")
    print(f"[INFO] BVH:   {args.bvh}")
    print(f"[INFO] src:   {args.src_scaler_config}")
    print(f"[INFO] out:   {args.out}")

    scales, world_offsets, audit = compute_chain_scales(args.mjcf, args.bvh, mapping)

    print("\n[INFO] per-chain calibration audit (sorted by joint):")
    print(f"  {'joint':>26}  {'robot_len(m)':>12}  {'bvh_len(m)':>12}  {'scale':>8}")
    for row in sorted(audit, key=lambda r: r["joint"]):
        print(f"  {row['joint']:>26}  {row['robot_len']:>12.4f}  {row['bvh_len']:>12.4f}  {row['scale']:>8.4f}")

    if world_offsets:
        print("\n[INFO] world_offsets emitted (foot-floor lift):")
        for k, v in sorted(world_offsets.items()):
            print(f"  {k:>14}: [{v[0]:+.4f}, {v[1]:+.4f}, {v[2]:+.4f}] m")

    new_cfg = build_scaler_config(
        args.src_scaler_config,
        scales,
        world_offsets,
        human_height_assumption=args.human_height_assumption,
    )

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(new_cfg, indent=4))
    audit_path = args.out.with_suffix(".calibration_audit.json")
    audit_path.write_text(json.dumps(
        {
            "mjcf":  str(args.mjcf),
            "bvh":   str(args.bvh),
            "src_scaler_config": str(args.src_scaler_config),
            "human_height_assumption": args.human_height_assumption,
            "mapping": mapping,
            "audit": audit,
        },
        indent=2,
    ))
    print(f"\n[OK] wrote scaler config -> {args.out}")
    print(f"[OK] wrote calibration audit -> {audit_path}")


if __name__ == "__main__":
    main()

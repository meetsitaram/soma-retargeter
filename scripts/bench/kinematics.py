"""Shared kinematics helpers for metrics + IK-target renderer.

Two key primitives:

1. `compute_targets(bvh_path, retarget_config_path)` ->
   {effector_name: (N,7) np.ndarray of [tx,ty,tz,qx,qy,qz,qw]}
   Reproduces what NewtonPipeline feeds into the IK solver, frame by frame.

2. `csv_to_mj_qpos(csv_data)` and `apply_csv_frame(model, data, csv_row)` to
   drive a MuJoCo `mj_data` object from a row of the 31-DOF X2 Ultra CSV.

Plus convenience body-name <-> body-id lookup for the X2 Ultra MJCF.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict

import numpy as np
import warp as wp

import soma_retargeter.assets.bvh as bvh_utils
import soma_retargeter.utils.io_utils as io_utils
from soma_retargeter.robotics.human_to_robot_scaler import HumanToRobotScaler
from soma_retargeter.utils.space_conversion_utils import (
    SpaceConverter,
    get_facing_direction_type_from_str,
)

from scripts.bench.retarget import load_retarget_config


# ---------------------------------------------------------------------------
# SOMA IK targets (reproduce NewtonPipeline.add_input_motions for one buffer)
# ---------------------------------------------------------------------------

# Module-level cache: (bvh_abs, config_abs, max_frames, facing) -> (targets, eff_order, cfg)
# Caches the per-clip targets so the per-pair render loop doesn't reload the
# BVH N times. Bounded by len(corpus * configs) entries which stays small.
_TARGETS_CACHE: dict = {}


def clear_targets_cache() -> None:
    _TARGETS_CACHE.clear()


def compute_targets(
    bvh_path: Path,
    retarget_config_path: Path,
    max_frames: int | None = None,
    facing: str = "Mujoco",
) -> tuple[Dict[str, np.ndarray], list[str], dict]:
    """Compute per-frame SOMA IK targets from a BVH file.

    Returns
    -------
    targets : dict[effector_name] -> np.ndarray of shape (N, 7)
        Each row is [tx,ty,tz,qx,qy,qz,qw] in world space (meters/quat).
    effector_order : list[str]
        Names in the order returned by `HumanToRobotScaler.effector_names()`.
    cfg : dict
        The fully-resolved retargeter config dictionary (useful for ik_map).
    """
    cache_key = (str(Path(bvh_path).resolve()), str(Path(retarget_config_path).resolve()), max_frames, facing)
    if cache_key in _TARGETS_CACHE:
        return _TARGETS_CACHE[cache_key]

    cfg = load_retarget_config(retarget_config_path)

    importer = bvh_utils.BVHImporter()
    bvh_skeleton, _ = importer.create_skeleton(bvh_path)

    converter = SpaceConverter(get_facing_direction_type_from_str(facing))
    bvh_tx = converter.transform(wp.transform_identity())

    _, animation = bvh_utils.load_bvh(bvh_path, bvh_skeleton)
    if max_frames is not None and animation.num_frames > max_frames:
        from soma_retargeter.animation.animation_buffer import AnimationBuffer
        animation = AnimationBuffer(
            animation.skeleton,
            max_frames,
            animation.sample_rate,
            animation.local_transforms[:max_frames],
        )

    # Resolve scaler config path the same way the pipeline does.
    scaler_cfg_path = cfg["human_robot_scaler_config"]
    if not Path(scaler_cfg_path).is_absolute():
        scaler_cfg_path = io_utils.get_config_file(scaler_cfg_path)

    scaler = HumanToRobotScaler(
        bvh_skeleton,
        cfg["model_height"],
        scaler_cfg_path,
    )

    # Effectors: shape (num_frames, num_mapped_joints, 7)
    effectors_np = scaler.compute_effectors_from_buffer(
        animation, scale_animation=True, xform=bvh_tx,
    )

    effector_order = scaler.effector_names()
    targets: Dict[str, np.ndarray] = {}
    for i, name in enumerate(effector_order):
        # Each row is a wp.transform: position (3) + quat (4)
        # The np view is dtype=object/(7,) per element. Let's stack to (N,7).
        rows = effectors_np[:, i]
        flat = np.array([[float(v) for v in r] for r in rows], dtype=np.float32)
        targets[name] = flat

    out = (targets, effector_order, cfg)
    _TARGETS_CACHE[cache_key] = out
    return out


# ---------------------------------------------------------------------------
# Drive a MuJoCo data from a CSV row
# ---------------------------------------------------------------------------

# CSV column order is the X2 Ultra 31-DOF layout. Mapping CSV column index ->
# MuJoCo joint name. Columns 1..6 are the floating base (tx,ty,tz,rx,ry,rz),
# columns 7..37 are joint DOFs in this order:
from scripts.bench.joint_limits import JOINT_NAMES as _JOINT_NAMES

_FLOATING_BASE_LEN = 7  # qpos slots: x, y, z, qw, qx, qy, qz


def load_x2_mj_model(offwidth: int = 1280, offheight: int = 960):
    """Load the X2 Ultra MJCF as a (model, data) pair via mujoco-py-free API.

    Bumps the offscreen framebuffer size so PNG rendering at moderately large
    resolutions doesn't trigger MuJoCo's 'image > framebuffer' error.
    """
    import mujoco
    mjcf_path = Path(__file__).resolve().parents[2] / "soma_retargeter" / "robot_assets" / "agibot_x2_ultra" / "x2_ultra.xml"
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    model.vis.global_.offwidth = int(offwidth)
    model.vis.global_.offheight = int(offheight)
    data = mujoco.MjData(model)
    return model, data


def body_name_to_id(model, name: str) -> int:
    import mujoco
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid < 0:
        raise KeyError(f"body not found in mjcf: {name}")
    return bid


def joint_name_to_qpos_idx(model, name: str) -> int:
    """Return the qpos start index for the given joint name."""
    import mujoco
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if jid < 0:
        raise KeyError(f"joint not found in mjcf: {name}")
    return int(model.jnt_qposadr[jid])


def build_csv_to_qpos_map(model) -> list[tuple[int, int]]:
    """Return a list of (csv_col_idx, qpos_idx) for each non-base joint.

    CSV joint columns start at column index 7 in CSV_HEADER and follow
    JOINT_NAMES order. qpos for hinge joints is one slot each.
    """
    pairs: list[tuple[int, int]] = []
    for k, jn in enumerate(_JOINT_NAMES):
        csv_idx = 7 + k
        try:
            qpos_idx = joint_name_to_qpos_idx(model, jn)
        except KeyError:
            continue
        pairs.append((csv_idx, qpos_idx))
    return pairs


def apply_csv_row(model, data, csv_row: np.ndarray, csv_to_qpos: list[tuple[int, int]]) -> None:
    """Set qpos from one CSV row and call mj_forward.

    csv_row layout: [frame, tx_cm, ty_cm, tz_cm, rx_deg, ry_deg, rz_deg, dof_deg...]
    """
    import mujoco
    from scipy.spatial.transform import Rotation as R

    # Floating base: meters + quaternion (mujoco free joint = [x y z qw qx qy qz])
    tx_m = float(csv_row[1]) * 0.01
    ty_m = float(csv_row[2]) * 0.01
    tz_m = float(csv_row[3]) * 0.01
    rx_rad, ry_rad, rz_rad = np.deg2rad(csv_row[4:7])
    quat_xyzw = R.from_euler("xyz", [rx_rad, ry_rad, rz_rad]).as_quat()
    qw, qx, qy, qz = quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]

    data.qpos[0] = tx_m
    data.qpos[1] = ty_m
    data.qpos[2] = tz_m
    data.qpos[3] = qw
    data.qpos[4] = qx
    data.qpos[5] = qy
    data.qpos[6] = qz

    for csv_idx, qpos_idx in csv_to_qpos:
        data.qpos[qpos_idx] = np.deg2rad(float(csv_row[csv_idx]))

    mujoco.mj_forward(model, data)


def body_world_xform(data, body_id: int) -> tuple[np.ndarray, np.ndarray]:
    """Return (pos[3], quat_xyzw[4]) for a body after mj_forward."""
    pos = np.array(data.xpos[body_id], dtype=np.float64)
    # mujoco xquat is [w, x, y, z]
    qwxyz = np.array(data.xquat[body_id], dtype=np.float64)
    quat_xyzw = np.array([qwxyz[1], qwxyz[2], qwxyz[3], qwxyz[0]], dtype=np.float64)
    return pos, quat_xyzw


def load_csv_matrix(csv_path: Path) -> np.ndarray:
    """Load an X2 Ultra 31-DOF retargeted CSV into an (N, 38) float matrix."""
    return np.loadtxt(csv_path, delimiter=",", skiprows=1, dtype=np.float64)

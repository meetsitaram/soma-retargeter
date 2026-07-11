"""Render robot at a CSV frame with overlaid IK targets and residual lines.

Uses MuJoCo's offscreen renderer (no display server required). For each
effector specified in the retargeter config's `ik_map`, we draw:

  - a small axis tripod at the SOMA target pose (red=X, green=Y, blue=Z),
  - a thin yellow line from the FK-achieved body position to the target
    position (so eye-balling the residual is easy).

Public entry points:
  - render_frame(...) -> np.ndarray RGB image
  - render_strip(...)  -> single PNG with multiple frames stacked
  - interactive(...)   -> mujoco.viewer-based stepping view
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterable

import numpy as np

from scripts.bench import kinematics


# ---------------------------------------------------------------------------
# Scene decoration helpers
# ---------------------------------------------------------------------------

def _add_user_geom(scn, geom_type, size, pos, mat, rgba) -> None:
    """Add a single user geom to a MuJoCo scene.

    `size`, `pos`, `mat`, `rgba` are flat float arrays. `mat` is a 9-float row-
    major rotation matrix.
    """
    import mujoco
    if scn.ngeom >= scn.maxgeom:
        return  # silently drop overflow; renderer caps user geoms
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(
        g,
        type=int(geom_type),
        size=np.array(size, dtype=np.float64),
        pos=np.array(pos, dtype=np.float64),
        mat=np.array(mat, dtype=np.float64).flatten(),
        rgba=np.array(rgba, dtype=np.float32),
    )
    g.category = mujoco.mjtCatBit.mjCAT_DECOR
    scn.ngeom += 1


def _quat_xyzw_to_mat(q_xyzw: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix from a XYZW quaternion (right-handed)."""
    x, y, z, w = q_xyzw
    n = x*x + y*y + z*z + w*w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1.0 - s*(y*y + z*z),     s*(x*y - z*w),         s*(x*z + y*w)],
        [    s*(x*y + z*w), 1.0 - s*(x*x + z*z),         s*(y*z - x*w)],
        [    s*(x*z - y*w),     s*(y*z + x*w),     1.0 - s*(x*x + y*y)],
    ])


def _draw_tripod(scn, pos, quat_xyzw, axis_len: float = 0.06, axis_thickness: float = 0.004) -> None:
    """Three axis-aligned arrows at a pose. Axis colors: X=red, Y=green, Z=blue."""
    import mujoco
    mat = _quat_xyzw_to_mat(quat_xyzw)
    for axis, color in [
        (0, (1.0, 0.1, 0.1, 0.85)),
        (1, (0.1, 1.0, 0.1, 0.85)),
        (2, (0.1, 0.3, 1.0, 0.85)),
    ]:
        axis_dir = mat[:, axis]
        # Arrow: from pos to pos + axis_dir * axis_len. Use capsule.
        end = pos + axis_dir * axis_len
        midpoint = 0.5 * (pos + end)
        # Build a rotation that aligns +Z with axis_dir (capsule's long axis is Z).
        z_axis = axis_dir / max(np.linalg.norm(axis_dir), 1e-9)
        # Pick a reference vector not colinear with z_axis
        ref = np.array([1.0, 0.0, 0.0]) if abs(z_axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        x_axis = np.cross(ref, z_axis); x_axis /= max(np.linalg.norm(x_axis), 1e-9)
        y_axis = np.cross(z_axis, x_axis)
        cap_mat = np.column_stack([x_axis, y_axis, z_axis])
        _add_user_geom(
            scn,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            size=(axis_thickness, axis_len * 0.5, 0.0),
            pos=midpoint,
            mat=cap_mat,
            rgba=color,
        )


def _draw_line(scn, a: np.ndarray, b: np.ndarray, rgba=(1.0, 1.0, 0.0, 0.8), thickness: float = 0.003) -> None:
    """Draw a capsule from point a to point b."""
    import mujoco
    delta = b - a
    L = float(np.linalg.norm(delta))
    if L < 1e-6:
        return
    mid = 0.5 * (a + b)
    z = delta / L
    ref = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x = np.cross(ref, z); x /= max(np.linalg.norm(x), 1e-9)
    y = np.cross(z, x)
    mat = np.column_stack([x, y, z])
    _add_user_geom(
        scn,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        size=(thickness, L * 0.5, 0.0),
        pos=mid,
        mat=mat,
        rgba=rgba,
    )


# ---------------------------------------------------------------------------
# Top-level renderer
# ---------------------------------------------------------------------------

def _setup_camera(model, data, focus_pos, distance=2.4, azim=110.0, elev=-12.0):
    import mujoco
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, cam)
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[0] = float(focus_pos[0])
    cam.lookat[1] = float(focus_pos[1])
    cam.lookat[2] = float(focus_pos[2])
    cam.distance = float(distance)
    cam.azimuth = float(azim)
    cam.elevation = float(elev)
    return cam


def render_frame(
    csv_path: Path,
    bvh_path: Path,
    retarget_config_path: Path,
    frame_idx: int,
    out_png: Path | None = None,
    width: int = 1024,
    height: int = 768,
    distance: float = 2.4,
    azim: float = 110.0,
    elev: float = -12.0,
    max_frames: int | None = None,
    show_targets: bool = True,
    show_residuals: bool = True,
) -> np.ndarray:
    """Render a single frame. Returns an HxWx3 uint8 array; writes PNG if requested."""
    import mujoco

    model, data = kinematics.load_x2_mj_model()
    csv_to_qpos = kinematics.build_csv_to_qpos_map(model)

    csv_data = kinematics.load_csv_matrix(csv_path)
    if frame_idx < 0 or frame_idx >= csv_data.shape[0]:
        raise IndexError(f"frame {frame_idx} out of bounds [0, {csv_data.shape[0]})")
    kinematics.apply_csv_row(model, data, csv_data[frame_idx], csv_to_qpos)

    targets = None
    eff_order: list[str] = []
    cfg: dict = {}
    if show_targets or show_residuals:
        targets, eff_order, cfg = kinematics.compute_targets(
            bvh_path, retarget_config_path, max_frames=max_frames,
        )

    pelvis_id = kinematics.body_name_to_id(model, "pelvis")
    pelvis_pos = np.array(data.xpos[pelvis_id], dtype=np.float64)

    renderer = mujoco.Renderer(model, height=height, width=width)
    cam = _setup_camera(model, data, focus_pos=pelvis_pos, distance=distance, azim=azim, elev=elev)
    renderer.update_scene(data, camera=cam)
    scn = renderer.scene

    if targets is not None:
        ik_map = cfg.get("ik_map", {})
        for name in eff_order:
            if name not in ik_map:
                continue
            if frame_idx >= targets[name].shape[0]:
                continue
            tx = targets[name][frame_idx]
            tpos = tx[:3].astype(np.float64)
            tquat = tx[3:7].astype(np.float64)  # already xyzw
            if show_targets:
                _draw_tripod(scn, tpos, tquat, axis_len=0.06, axis_thickness=0.004)
            if show_residuals:
                t_body = ik_map[name].get("t_body")
                if t_body:
                    try:
                        bid = kinematics.body_name_to_id(model, t_body)
                        bpos = np.array(data.xpos[bid], dtype=np.float64)
                        _draw_line(scn, bpos, tpos, rgba=(1.0, 1.0, 0.0, 0.8), thickness=0.003)
                    except KeyError:
                        pass

    img = renderer.render()

    if out_png is not None:
        out_png.parent.mkdir(parents=True, exist_ok=True)
        try:
            import imageio.v2 as iio
            iio.imwrite(str(out_png), img)
        except ImportError:
            import PIL.Image
            PIL.Image.fromarray(img).save(str(out_png))

    renderer.close()
    return img


def render_strip(
    csv_path: Path,
    bvh_path: Path,
    retarget_config_path: Path,
    frames: Iterable[int],
    out_png: Path,
    width: int = 720,
    height: int = 540,
    **kwargs,
) -> None:
    """Render multiple frames horizontally concatenated into one PNG."""
    images = []
    for f in frames:
        img = render_frame(
            csv_path, bvh_path, retarget_config_path, f,
            out_png=None, width=width, height=height, **kwargs,
        )
        images.append(img)
    if not images:
        return
    strip = np.concatenate(images, axis=1)
    out_png.parent.mkdir(parents=True, exist_ok=True)
    try:
        import imageio.v2 as iio
        iio.imwrite(str(out_png), strip)
    except ImportError:
        import PIL.Image
        PIL.Image.fromarray(strip).save(str(out_png))


def interactive(
    csv_path: Path,
    bvh_path: Path,
    retarget_config_path: Path,
    start_frame: int = 0,
    fps: float = 30.0,
    max_frames: int | None = None,
) -> None:
    """Open a passive MuJoCo viewer and step through the clip showing targets.

    Press SPACE in the viewer to pause/resume (mujoco-viewer default).
    """
    import time
    import mujoco
    import mujoco.viewer as mjv

    model, data = kinematics.load_x2_mj_model()
    csv_to_qpos = kinematics.build_csv_to_qpos_map(model)
    csv_data = kinematics.load_csv_matrix(csv_path)

    targets, eff_order, cfg = kinematics.compute_targets(
        bvh_path, retarget_config_path, max_frames=max_frames,
    )
    ik_map = cfg.get("ik_map", {})

    pelvis_id = kinematics.body_name_to_id(model, "pelvis")
    kinematics.apply_csv_row(model, data, csv_data[start_frame], csv_to_qpos)

    with mjv.launch_passive(model, data) as viewer:
        f = start_frame
        last = time.time()
        while viewer.is_running() and f < csv_data.shape[0]:
            kinematics.apply_csv_row(model, data, csv_data[f], csv_to_qpos)

            # Decorate user_scn each frame.
            viewer.user_scn.ngeom = 0
            for name in eff_order:
                if name not in ik_map:
                    continue
                if f >= targets[name].shape[0]:
                    continue
                tx = targets[name][f]
                tpos = tx[:3].astype(np.float64)
                tquat = tx[3:7].astype(np.float64)
                _draw_tripod(viewer.user_scn, tpos, tquat)
                t_body = ik_map[name].get("t_body")
                if t_body:
                    try:
                        bid = kinematics.body_name_to_id(model, t_body)
                        bpos = np.array(data.xpos[bid], dtype=np.float64)
                        _draw_line(viewer.user_scn, bpos, tpos)
                    except KeyError:
                        pass

            viewer.sync()

            dt = max(0.0, 1.0 / fps - (time.time() - last))
            time.sleep(dt)
            last = time.time()
            f += 1

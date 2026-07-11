"""Side-by-side comparison renderer for slam/pin events.

For each picked event (clip, joint, peak_f), render a 2-row image strip:

    row 0:   config A  @  [peak-3, peak, peak+3]
    row 1:   config B  @  [peak-3, peak, peak+3]

A short label at the bottom of each row identifies the config. The event ID
goes into the filename so the markdown report can link to it.

We reuse `scripts.bench.render.render_frame` for the actual MuJoCo offscreen
render, which already overlays the IK target tripod + FK residual line; the
side-by-side view is just a thin orchestration layer on top.

Heavy reuse:
- `kinematics._TARGETS_CACHE`  -> BVH targets memoized per (bvh, retarget_cfg)
- Each cell calls `render_frame(..., width=W, height=H, ...)` which is the
  cheapest path that already handles the tripod overlay.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable

import numpy as np

from scripts.bench import render as bench_render
from scripts.bench import kinematics


# ---------------------------------------------------------------------------
# BVH skeleton rendering (the "human reference" row)
# ---------------------------------------------------------------------------

# Minimal MJCF: floor + lighting, no robot. We render the BVH skeleton as
# user_geoms on top.
_MINIMAL_MJCF = """
<mujoco>
  <visual>
    <global offwidth='1280' offheight='960'/>
    <headlight ambient='0.5 0.5 0.5' diffuse='0.8 0.8 0.8'/>
  </visual>
  <asset>
    <texture name='grid' type='2d' builtin='checker' rgb1='0.20 0.27 0.33' rgb2='0.13 0.18 0.22' width='300' height='300' mark='edge' markrgb='0.20 0.27 0.33'/>
    <material name='grid' texture='grid' texrepeat='8 8' reflectance='0.15'/>
  </asset>
  <worldbody>
    <light dir='0 0 -1' pos='0 0 5' directional='true' diffuse='0.8 0.8 0.8'/>
    <geom name='floor' type='plane' size='10 10 0.1' material='grid'/>
  </worldbody>
</mujoco>
"""


_bvh_anim_cache: dict[str, tuple] = {}
_bvh_mesh_cache: dict[str, "object"] = {}


def _load_bvh_animation(bvh_path: Path, facing: str = "Mujoco"):
    """Load (skeleton, animation, root_tx) for a BVH; cached by absolute path."""
    key = str(Path(bvh_path).resolve())
    if key in _bvh_anim_cache:
        return _bvh_anim_cache[key]
    import warp as wp
    import soma_retargeter.assets.bvh as bvh_utils
    from soma_retargeter.utils.space_conversion_utils import (
        SpaceConverter, get_facing_direction_type_from_str,
    )
    importer = bvh_utils.BVHImporter()
    skeleton, _ = importer.create_skeleton(bvh_path)
    _, animation = bvh_utils.load_bvh(bvh_path, skeleton)
    converter = SpaceConverter(get_facing_direction_type_from_str(facing))
    root_tx = converter.transform(wp.transform_identity())
    _bvh_anim_cache[key] = (skeleton, animation, root_tx)
    return _bvh_anim_cache[key]


def _load_bvh_skinned_mesh(bvh_path: Path):
    """Load the SOMA skinned body mesh bound to this BVH's skeleton (cached)."""
    key = str(Path(bvh_path).resolve())
    if key in _bvh_mesh_cache:
        return _bvh_mesh_cache[key]
    import soma_retargeter.pipelines.utils as pipeline_utils
    skeleton, _, _ = _load_bvh_animation(bvh_path)
    sm = pipeline_utils.get_source_model_mesh(pipeline_utils.SourceType.SOMA, skeleton)
    _bvh_mesh_cache[key] = sm
    return sm


def _skin_bvh_frame(bvh_path: Path, frame_idx: int) -> tuple[np.ndarray, np.ndarray]:
    """Compute posed (vertices, faces) of the SOMA body for a BVH frame.

    Vertices are in MuJoCo (Z-up, meters) world space — same convention as
    the robot renders — because the SkeletonInstance is given the Mujoco
    root transform as its character transform.
    """
    import warp as wp
    from soma_retargeter.animation.skeleton import SkeletonInstance
    from soma_retargeter.renderers.mesh_renderer import (
        skinning_kernel, update_skinned_transform_kernel,
    )
    skel, anim, root_tx = _load_bvh_animation(bvh_path)
    sm = _load_bvh_skinned_mesh(bvh_path)

    inst = SkeletonInstance(skel, (1.0, 1.0, 0.0), root_tx)
    f = max(0, min(anim.num_frames - 1, int(frame_idx)))
    inst.set_local_transforms(anim.local_transforms[f])

    num_joints = skel.num_joints
    skinned_transforms = wp.zeros((1, num_joints), dtype=wp.transform)
    local_transforms = inst.get_local_transforms()
    wp.launch(
        update_skinned_transform_kernel,
        dim=(1,),
        inputs=[
            num_joints,
            wp.array(local_transforms, dtype=wp.transform),
            wp.array(inst.parent_indices, dtype=wp.int32),
            sm.bind_transforms,
            inst.xform,
        ],
        outputs=[skinned_transforms],
    )

    sk_mesh = sm.skinned_meshes[0]
    out = wp.zeros(sk_mesh.num_points, dtype=wp.vec3)
    wp.launch(
        skinning_kernel,
        dim=sk_mesh.num_points,
        inputs=[
            sk_mesh.points,
            sk_mesh.joint_indices,
            sk_mesh.joint_weights,
            int(sk_mesh.num_influences),
            wp.array(skinned_transforms[0], dtype=wp.transform),
        ],
        outputs=[out],
    )
    vertices = out.numpy().astype(np.float32)
    faces = sk_mesh.indices.numpy().astype(np.int32).reshape(-1, 3)
    return vertices, faces


def _write_stl_binary(path: str, verts: np.ndarray, faces: np.ndarray) -> None:
    """Write a triangle mesh as binary STL (fast, no external deps)."""
    import struct
    tris = verts[faces]                                         # (N,3,3)
    e1 = tris[:, 1] - tris[:, 0]
    e2 = tris[:, 2] - tris[:, 0]
    n = np.cross(e1, e2).astype(np.float32)
    nlen = np.linalg.norm(n, axis=1, keepdims=True)
    nlen[nlen == 0] = 1.0
    n = (n / nlen).astype(np.float32)
    with open(path, "wb") as f:
        f.write(b"\0" * 80)
        f.write(struct.pack("<I", len(tris)))
        body = np.zeros((len(tris), 12), dtype=np.float32)
        body[:, 0:3] = n
        body[:, 3:6] = tris[:, 0]
        body[:, 6:9] = tris[:, 1]
        body[:, 9:12] = tris[:, 2]
        attr = np.zeros(len(tris), dtype=np.uint16)
        raw = body.tobytes()
        attrs = attr.tobytes()
        # Interleave 50 bytes per face: 48 float32 + 2 uint16
        out = bytearray()
        for i in range(len(tris)):
            out += raw[i * 48:(i + 1) * 48]
            out += attrs[i * 2:(i + 1) * 2]
        f.write(bytes(out))


def _bvh_joint_positions(bvh_path: Path, frame_idx: int) -> tuple[np.ndarray, list[int]]:
    """Return (positions[J,3], parent_indices[J]) for a single BVH frame.

    Positions are in the same Mujoco-facing space the retargeter uses, so
    they line up with the robot renders (meters, Z up).
    """
    from soma_retargeter.utils.pose_utils import compute_global_pose
    skeleton, animation, root_tx = _load_bvh_animation(bvh_path)
    f = max(0, min(animation.num_frames - 1, frame_idx))
    local = animation.local_transforms[f]
    glob = compute_global_pose(skeleton, local, root_tx)
    positions = np.array([[t[0], t[1], t[2]] for t in glob], dtype=np.float64)
    parents = list(skeleton.parent_indices)
    return positions, parents


def _resolve_bvh_joint_indices(bvh_path: Path) -> dict[str, int]:
    """Find indices of common BVH joints (Head, Hips, Spine, etc.) by name."""
    skel, _, _ = _load_bvh_animation(bvh_path)
    names_to_try = ["Head", "Hips", "Spine", "Spine1", "Spine2", "Neck", "LeftHand", "RightHand"]
    out: dict[str, int] = {}
    for n in names_to_try:
        try:
            out[n] = skel.joint_index(n)
        except Exception:
            pass
    return out


# MJCF template for the human body row. The mesh file is written per-frame
# under /tmp and loaded here. We render in Z-up because the SkeletonInstance
# already converts BVH Y-up to MuJoCo Z-up via the SOMA SpaceConverter.
_HUMAN_MJCF_TEMPLATE = """
<mujoco>
  <visual>
    <global offwidth='{w}' offheight='{h}'/>
    <headlight ambient='0.45 0.45 0.45' diffuse='0.70 0.70 0.70' specular='0.05 0.05 0.05'/>
    <map znear='0.01' zfar='50'/>
  </visual>
  <asset>
    <mesh name='human' file='{stl}'/>
    <texture name='grid' type='2d' builtin='checker' rgb1='0.20 0.27 0.33' rgb2='0.13 0.18 0.22' width='300' height='300' mark='edge' markrgb='0.20 0.27 0.33'/>
    <material name='grid' texture='grid' texrepeat='8 8' reflectance='0.15'/>
  </asset>
  <worldbody>
    <light dir='-0.3 -0.2 -1' pos='1 1 5' directional='true' diffuse='0.7 0.7 0.7'/>
    <geom name='floor' type='plane' size='10 10 0.1' material='grid'/>
    <geom name='human' type='mesh' mesh='human' rgba='0.96 0.92 0.45 1'/>
  </worldbody>
</mujoco>
"""


def render_bvh_frame(
    bvh_path: Path,
    frame_idx: int,
    *,
    width: int = 480,
    height: int = 360,
    distance: float = 2.4,
    azim: float = 110.0,
    elev: float = -12.0,
) -> np.ndarray:
    """Render the SOMA body mesh (linear blend skinning) at one BVH frame.

    Falls back to a thick-capsule mannequin if the SOMA USD mesh cannot be
    loaded (e.g. running outside the soma-retargeter env).
    """
    import mujoco
    from scripts.bench.render import _setup_camera

    # 1. Bake the posed body mesh
    try:
        vertices, faces = _skin_bvh_frame(bvh_path, frame_idx)
    except Exception as exc:
        # Fallback: render thick-capsule figure if skinning data is missing.
        print(f"[WARN] skinned body unavailable ({exc}); falling back to capsule figure")
        return _render_bvh_frame_capsules(
            bvh_path, frame_idx,
            width=width, height=height,
            distance=distance, azim=azim, elev=elev,
        )

    # 2. Compute camera focus from the bounding box (head + feet visible)
    bbox_min = vertices.min(axis=0)
    bbox_max = vertices.max(axis=0)
    focus = 0.5 * (bbox_min + bbox_max)
    bbox_diag = float(np.linalg.norm(bbox_max - bbox_min))
    eff_distance = max(distance, bbox_diag * 1.15)

    # 3. Write STL with a unique filename and build a one-shot MuJoCo scene.
    #    A unique path is required because MuJoCo caches the *parsed* mesh data
    #    by asset filename — reusing one path would freeze the body to the
    #    first baked pose across the whole batch of renders.
    import os, shutil, tempfile
    tmpdir = tempfile.mkdtemp(prefix="soma_human_")
    stl_path = os.path.join(tmpdir, f"frame_{int(frame_idx):06d}.stl")
    try:
        _write_stl_binary(stl_path, vertices, faces)
        xml = _HUMAN_MJCF_TEMPLATE.format(w=int(width), h=int(height), stl=stl_path)
        model = mujoco.MjModel.from_xml_string(xml)
        model.vis.global_.offwidth = int(width)
        model.vis.global_.offheight = int(height)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        renderer = mujoco.Renderer(model, height=height, width=width)
        cam = _setup_camera(
            model, data,
            focus_pos=focus, distance=eff_distance,
            azim=azim, elev=elev,
        )
        renderer.update_scene(data, camera=cam)
        img = renderer.render()
        renderer.close()
    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)
    return img


def _render_bvh_frame_capsules(
    bvh_path: Path,
    frame_idx: int,
    *,
    width: int = 480,
    height: int = 360,
    distance: float = 2.4,
    azim: float = 110.0,
    elev: float = -12.0,
    joint_radius: float = 0.018,
    bone_radius: float = 0.045,
    head_radius: float = 0.10,
) -> np.ndarray:
    """Fallback renderer: thick capsules + head sphere from BVH joint chain.

    Used when the SOMA skinned mesh USD is unavailable.
    """
    import mujoco
    from scripts.bench.render import _add_user_geom, _setup_camera, _draw_line

    model = mujoco.MjModel.from_xml_string(_MINIMAL_MJCF)
    model.vis.global_.offwidth = int(width)
    model.vis.global_.offheight = int(height)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    positions, parents = _bvh_joint_positions(bvh_path, frame_idx)
    head_idx = _resolve_bvh_joint_indices(bvh_path).get("Head", -1)

    bone_thickness = np.full(len(positions), bone_radius, dtype=np.float64)
    for i, parent in enumerate(parents):
        if parent is not None and parent >= 0:
            d = float(np.linalg.norm(positions[i] - positions[parent]))
            if d < 0.04:
                bone_thickness[i] = 0.012
            elif d < 0.10:
                bone_thickness[i] = 0.020

    if len(positions) > 0:
        bbox_min = positions.min(axis=0)
        bbox_max = positions.max(axis=0)
        focus = 0.5 * (bbox_min + bbox_max)
        bbox_diag = float(np.linalg.norm(bbox_max - bbox_min))
        eff_distance = max(distance, bbox_diag * 1.3)
    else:
        focus = np.array([0.0, 0.0, 1.0])
        eff_distance = distance

    renderer = mujoco.Renderer(model, height=height, width=width)
    cam = _setup_camera(model, data, focus_pos=focus, distance=eff_distance, azim=azim, elev=elev)
    renderer.update_scene(data, camera=cam)
    scn = renderer.scene

    body_rgba = (0.94, 0.86, 0.74, 1.0)
    joint_rgba = (0.85, 0.75, 0.62, 1.0)
    eye3 = np.eye(3)
    for i, parent in enumerate(parents):
        p = positions[i]
        if joint_radius > 0:
            _add_user_geom(
                scn, mujoco.mjtGeom.mjGEOM_SPHERE,
                size=(joint_radius, joint_radius, joint_radius),
                pos=p, mat=eye3, rgba=joint_rgba,
            )
        if parent is not None and parent >= 0:
            _draw_line(scn, positions[parent], p, rgba=body_rgba, thickness=bone_thickness[i])

    if head_idx >= 0 and head_idx < len(positions):
        _add_user_geom(
            scn, mujoco.mjtGeom.mjGEOM_SPHERE,
            size=(head_radius, head_radius, head_radius),
            pos=positions[head_idx], mat=eye3, rgba=(0.94, 0.86, 0.74, 1.0),
        )

    img = renderer.render()
    renderer.close()
    return img


# ---------------------------------------------------------------------------
# Label drawing
# ---------------------------------------------------------------------------

def _label_row(
    images: list[np.ndarray],
    label: str,
    bar_height: int = 28,
    column_labels: list[str] | None = None,
) -> np.ndarray:
    """Concatenate `images` horizontally and prepend a label bar.

    If ``column_labels`` is given, also paints a thin tag in the top-left of
    each cell (e.g. "approach", "peak", "recovery").
    """
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        return np.concatenate(images, axis=1)

    # Paint a small column tag on each cell if requested.
    if column_labels and len(column_labels) == len(images):
        annotated: list[np.ndarray] = []
        for cell, tag in zip(images, column_labels):
            im = Image.fromarray(cell).convert("RGB")
            draw = ImageDraw.Draw(im)
            try:
                f_small = ImageFont.truetype("DejaVuSans-Bold.ttf", 14)
            except OSError:
                f_small = ImageFont.load_default()
            tw = draw.textlength(tag, font=f_small)
            pad = 4
            draw.rectangle([(0, 0), (int(tw) + 2 * pad, 22)], fill=(0, 0, 0))
            draw.text((pad, 3), tag, fill=(255, 215, 90), font=f_small)
            annotated.append(np.array(im))
        images = annotated

    row = np.concatenate(images, axis=1)
    h, w, _ = row.shape
    bar = Image.new("RGB", (w, bar_height), (24, 24, 24))
    draw = ImageDraw.Draw(bar)
    try:
        font = ImageFont.truetype("DejaVuSans-Bold.ttf", 16)
    except OSError:
        font = ImageFont.load_default()
    draw.text((8, 4), label, fill=(240, 240, 240), font=font)
    bar_arr = np.array(bar)
    return np.concatenate([bar_arr, row], axis=0)


def _caption_strip(text_lines: list[str], width: int, line_height: int = 18) -> np.ndarray:
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        return np.zeros((line_height * max(1, len(text_lines)), width, 3), dtype=np.uint8)
    h = line_height * max(1, len(text_lines)) + 6
    img = Image.new("RGB", (width, h), (40, 40, 40))
    draw = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 13)
    except OSError:
        font = ImageFont.load_default()
    for i, t in enumerate(text_lines):
        draw.text((10, 3 + i * line_height), t, fill=(220, 220, 220), font=font)
    return np.array(img)


# ---------------------------------------------------------------------------
# Event picking
# ---------------------------------------------------------------------------

def pick_top_events(
    data: dict,
    *,
    per_config: int = 12,
    category: str = "slam",
    joint_groups: list[str] | None = None,
) -> dict[str, list[dict]]:
    """Pick top-N events per config within `category`.

    Slams are ranked by |peak_vel_dps| (severity of basin hop). Pins are
    ranked by duration_frames (how long the joint was stuck). If
    `joint_groups` is given, only events on joints in those groups qualify.

    Returns {config_name: [event_dict, ...]}.
    """
    from scripts.bench.joint_limits import JOINT_GROUPS
    allowed_joints: set[str] | None = None
    if joint_groups:
        allowed_joints = set()
        for g in joint_groups:
            allowed_joints.update(JOINT_GROUPS.get(g, []))

    if category == "slam":
        sort_key = lambda e: -abs(e["peak_vel_dps"])
    elif category == "twist":
        # peak |angle| first, longest run as tiebreaker
        sort_key = lambda e: (
            -float(e.get("peak_abs_deg", abs(e.get("peak_angle_deg", 0.0)))),
            -int(e["duration_frames"]),
        )
    else:  # pin (default)
        sort_key = lambda e: -e["duration_frames"]

    out: dict[str, list[dict]] = {}
    for cfg in data["configs"]:
        evs = []
        for clip, by_cfg in data["clips"].items():
            for e in by_cfg.get(cfg, {}).get("events", []):
                if e["category"] != category:
                    continue
                if allowed_joints is not None and e["joint"] not in allowed_joints:
                    continue
                evs.append(e)
        evs.sort(key=sort_key)
        out[cfg] = evs[:per_config]
    return out


# ---------------------------------------------------------------------------
# Render one side-by-side strip for a single event
# ---------------------------------------------------------------------------

def _pick_event_frames(
    event: dict,
    *,
    fps: float = 120.0,
    slam_window_ms: float = 250.0,
) -> tuple[list[int], list[str]]:
    """Choose three diagnostic frames for an event.

    Returns ``(frame_indices, column_labels)`` where the labels describe what
    each frame represents (e.g. "approach", "peak", "exit").

    - slam: peak-W / peak / peak+W   (W = ~slam_window_ms apart from peak)
    - pin : start_f / peak_f / end_f (entry / middle / exit of the pin span)
    """
    cat = event.get("category", "")
    peak_f = int(event.get("peak_f", 0))
    start_f = int(event.get("start_f", peak_f))
    end_f = int(event.get("end_f", peak_f))

    if cat == "slam":
        w = max(1, int(round((slam_window_ms / 1000.0) * fps)))   # frames
        a = peak_f - w
        c = peak_f + w
        return ([a, peak_f, c],
                [f"approach (peak-{w})", f"peak (f{peak_f})", f"recovery (peak+{w})"])

    # pin (or default): entry / mid / exit
    if cat == "pin":
        dur = max(1, end_f - start_f + 1)
        mid = start_f + dur // 2
        return ([start_f, mid, end_f],
                [f"pin start (f{start_f})", f"pin mid (f{mid})", f"pin end (f{end_f})"])

    # twist: entry / peak / exit — same layout as pin, but a long sustained run
    # so we want viewers to see the entry, the peak, and the exit clearly.
    if cat == "twist":
        return ([start_f, peak_f, end_f],
                [f"twist start (f{start_f})", f"peak |twist| (f{peak_f})", f"twist end (f{end_f})"])

    # fallback for unknown category
    w = max(1, int(round((slam_window_ms / 1000.0) * fps)))
    return ([peak_f - w, peak_f, peak_f + w],
            [f"peak-{w}", f"peak (f{peak_f})", f"peak+{w}"])


def render_event_strip(
    event: dict,
    bench_dir: Path,
    bvh_dir: Path,
    cfg_paths: dict[str, Path],
    *,
    cell_width: int = 560,
    cell_height: int = 440,
    fps: float = 120.0,
    slam_window_ms: float = 250.0,
    distance: float = 2.7,
    out_png: Path | None = None,
    include_human_row: bool = True,
    display_names: dict[str, str] | None = None,
) -> Path:
    """Render one event into a 3-row strip (human + 2 configs) and save as PNG.

    `event` must include keys: clip, joint, side, category, peak_f, start_f,
    end_f, peak_vel_dps, peak_angle_deg, limit_deg, duration_frames.

    Frame selection is event-aware so the three columns convey motion through
    the event, not three nearly-identical frames:
      - slam events: approach / peak / recovery, spaced ``slam_window_ms`` apart
      - pin events : entry / middle / exit of the pin span (using start_f/end_f)

    `display_names` maps internal config keys to human-friendly labels for the
    row caption.
    """
    clip = event["clip"]
    peak_f = int(event["peak_f"])

    # Locate the BVH file by stem under bvh_dir.
    bvh_path = _resolve_bvh(bvh_dir, clip)
    if bvh_path is None:
        raise FileNotFoundError(f"BVH not found for clip stem '{clip}' under {bvh_dir}")

    frame_indices, column_labels = _pick_event_frames(
        event, fps=fps, slam_window_ms=slam_window_ms,
    )

    csv_dir = bench_dir / "csvs"
    configs_in_order = list(cfg_paths.keys())
    display_names = display_names or {}

    rows: list[np.ndarray] = []

    # ----- Human reference row (top) -----
    if include_human_row:
        skel, anim, _ = _load_bvh_animation(bvh_path)
        n_bvh = anim.num_frames
        human_cells: list[np.ndarray] = []
        for f_raw in frame_indices:
            f = max(0, min(n_bvh - 1, int(f_raw)))
            human_cells.append(render_bvh_frame(
                bvh_path, f, width=cell_width, height=cell_height, distance=distance,
            ))
        # Per-cell column tags burned over each panel
        rows.append(_label_row(
            human_cells,
            f"human (BVH)   |   {clip}.bvh",
            column_labels=column_labels,
        ))

    # ----- Robot rows -----
    for cfg in configs_in_order:
        csv_path = csv_dir / f"{clip}__{cfg}.csv"
        if not csv_path.is_file():
            rows.append(np.zeros((cell_height, cell_width * len(frame_indices), 3), dtype=np.uint8))
            continue

        n_frames = _csv_num_frames(csv_path)
        cells: list[np.ndarray] = []
        for f_raw in frame_indices:
            f = max(0, min(n_frames - 1, int(f_raw)))
            img = bench_render.render_frame(
                csv_path=csv_path,
                bvh_path=bvh_path,
                retarget_config_path=cfg_paths[cfg],
                frame_idx=f,
                out_png=None,
                width=cell_width,
                height=cell_height,
                distance=distance,
            )
            cells.append(img)

        joint_summary = _joint_value_at_frame(csv_path, peak_f, event["joint"])
        label = display_names.get(cfg, cfg)
        row_label = f"{label}   |   {event['joint']}@f{peak_f}={joint_summary:+.1f} deg   |   limit {event['side']}={event['limit_deg']:+.1f} deg"
        rows.append(_label_row(cells, row_label, column_labels=column_labels))

    # Caption header
    width = rows[0].shape[1]
    rows_str = "human + " + " + ".join(display_names.get(c, c) for c in configs_in_order) if include_human_row \
        else " + ".join(display_names.get(c, c) for c in configs_in_order)
    columns_str = " | ".join(column_labels)
    cap_lines = [
        f"{clip}.bvh   joint={event['joint']}   side={event['side']}   cat={event['category']}",
        f"peak_f={peak_f}   duration={event['duration_frames']}f   |peak vel|={abs(event['peak_vel_dps']):.0f} deg/s   peak angle={event['peak_angle_deg']:+.1f} deg",
        f"columns: {columns_str}   (rows: {rows_str})",
    ]
    cap = _caption_strip(cap_lines, width)

    strip = np.concatenate([cap] + rows, axis=0)

    if out_png is None:
        eid = _event_id(clip, event)
        out_png = bench_dir / "frames" / "side_by_side" / f"{eid}.png"
    out_png.parent.mkdir(parents=True, exist_ok=True)
    try:
        import imageio.v2 as iio
        iio.imwrite(str(out_png), strip)
    except ImportError:
        import PIL.Image
        PIL.Image.fromarray(strip).save(str(out_png))
    return out_png


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _event_id(clip: str, event: dict) -> str:
    return f"{clip}__{event['joint']}__{event['category']}__f{event['peak_f']}"


def _csv_num_frames(csv_path: Path) -> int:
    """Count lines in CSV minus header. Cheap; CSV is small."""
    with open(csv_path) as f:
        n = sum(1 for _ in f)
    return max(n - 1, 1)


_bvh_cache: dict[str, Path] = {}


def _resolve_bvh(bvh_dir_or_corpus_json: Path, clip_stem: str) -> Path | None:
    """Find a BVH whose stem matches clip_stem.

    `bvh_dir_or_corpus_json` may be either:
      - the path to a `corpus.json` (preferred — direct lookup), or
      - a directory containing BVH files (fallback — recursive search).
    """
    cache = _bvh_cache
    if clip_stem in cache:
        return cache[clip_stem]

    p = bvh_dir_or_corpus_json
    if p.is_file() and p.suffix == ".json":
        try:
            entries = json.loads(p.read_text())
            for e in entries:
                stem = Path(e["name"]).stem
                cache[stem] = Path(e["path"])
        except Exception:
            pass
        if clip_stem in cache:
            return cache[clip_stem]

    if p.is_dir():
        for found in p.rglob(f"{clip_stem}.bvh"):
            cache[clip_stem] = found
            return found
    return None


def _joint_value_at_frame(csv_path: Path, frame_idx: int, joint_name: str) -> float:
    """Return the joint angle (deg) for that frame from the CSV. Cached load."""
    from scripts.bench.joint_limits import csv_col
    arr = _csv_array_cache_get(csv_path)
    f = max(0, min(arr.shape[0] - 1, frame_idx))
    return float(arr[f, csv_col(joint_name)])


_csv_cache: dict[Path, np.ndarray] = {}


def _csv_array_cache_get(csv_path: Path) -> np.ndarray:
    p = csv_path.resolve()
    if p not in _csv_cache:
        _csv_cache[p] = np.loadtxt(p, delimiter=",", skiprows=1, dtype=np.float64)
    return _csv_cache[p]


# ---------------------------------------------------------------------------
# Build many side-by-sides at once
# ---------------------------------------------------------------------------

# Anatomical zones used by `build_anatomical_fill` and the doc writer.
# Each entry: (zone_key, joint_names, categories_to_render).
ANATOMICAL_ZONES: list[tuple[str, list[str], list[str]]] = [
    ("wrist_yaw",
        ["left_wrist_yaw_joint", "right_wrist_yaw_joint"],
        ["twist"]),
    ("wrist_pitch",
        ["left_wrist_pitch_joint", "right_wrist_pitch_joint"],
        ["slam", "pin"]),
    ("wrist_roll",
        ["left_wrist_roll_joint", "right_wrist_roll_joint"],
        ["slam", "pin"]),
    ("elbow",
        ["left_elbow_joint", "right_elbow_joint"],
        ["slam", "pin"]),
    ("shoulder",
        ["left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
         "left_shoulder_roll_joint",  "right_shoulder_roll_joint",
         "left_shoulder_yaw_joint",   "right_shoulder_yaw_joint"],
        ["slam", "pin"]),
]


def _rank_key_for_category(category: str):
    if category == "slam":
        return lambda e: -abs(e["peak_vel_dps"])
    if category == "twist":
        return lambda e: (
            -float(e.get("peak_abs_deg", abs(e.get("peak_angle_deg", 0.0)))),
            -int(e["duration_frames"]),
        )
    # pin
    return lambda e: -e["duration_frames"]


def build_anatomical_fill(
    bench_dir: Path,
    bvh_dir: Path,
    cfg_paths: dict[str, Path],
    *,
    per_zone: int = 6,
    cell_width: int = 560,
    cell_height: int = 440,
    slam_window_ms: float = 250.0,
    include_human_row: bool = True,
    display_names: dict[str, str] | None = None,
    verbose: bool = True,
) -> dict[str, Path]:
    """Render top-N events per (anatomical-zone, category) combo, skipping any
    PNG that already exists on disk.

    The standard per-config top-N pass is biased toward the dominant joint in
    each category (e.g. elbow slams shadow wrist_roll slams in the global
    slam top-N). This pass guarantees each anatomical section in the doc has
    rendered examples even if its events don't dominate the corpus.
    """
    data = json.loads((bench_dir / "limit_events.json").read_text())
    fps = float(data.get("params", {}).get("fps", 120.0))

    # Collect candidate events per zone, dedup across configs.
    candidates: dict[str, dict] = {}     # event_id -> event dict
    for zone_key, joints, cats in ANATOMICAL_ZONES:
        joint_set = set(joints)
        for cat in cats:
            flat: list[dict] = []
            for clip, by_cfg in data["clips"].items():
                for cfg in data["configs"]:
                    for e in by_cfg.get(cfg, {}).get("events", []):
                        if e["category"] != cat:
                            continue
                        if e["joint"] not in joint_set:
                            continue
                        flat.append(e)
            # dedup by event_id; same (clip, joint, peak_f) may appear in both configs
            uniq: dict[str, dict] = {}
            for e in flat:
                eid = _event_id(e["clip"], e)
                if eid not in uniq:
                    uniq[eid] = e
            ranked = sorted(uniq.values(), key=_rank_key_for_category(cat))[:per_zone]
            for e in ranked:
                eid = _event_id(e["clip"], e)
                if eid not in candidates:
                    candidates[eid] = e

    sxs_dir = bench_dir / "frames" / "side_by_side"
    out: dict[str, Path] = {}
    missing = [(eid, e) for eid, e in candidates.items()
               if not (sxs_dir / f"{eid}.png").exists()]

    if verbose:
        present = len(candidates) - len(missing)
        print(f"[INFO] anatomical fill: {len(candidates)} candidates, "
              f"{present} already rendered, {len(missing)} new renders queued.")

    for i, (eid, e) in enumerate(missing):
        try:
            png = render_event_strip(
                e, bench_dir=bench_dir, bvh_dir=bvh_dir,
                cfg_paths=cfg_paths,
                fps=fps,
                slam_window_ms=slam_window_ms,
                cell_width=cell_width, cell_height=cell_height,
                include_human_row=include_human_row,
                display_names=display_names,
            )
            out[eid] = png
            if verbose:
                print(f"[OK] anatomical-fill ({i+1}/{len(missing)}) {png.name}")
        except Exception as exc:
            if verbose:
                print(f"[WARN] anatomical-fill render failed for {eid}: {exc}")

    # Also include any events that already had renders — caller wants the full
    # set to know which event_ids map to which PNGs.
    for eid in candidates:
        png_path = sxs_dir / f"{eid}.png"
        if eid not in out and png_path.exists():
            out[eid] = png_path

    return out


def build_all(
    bench_dir: Path,
    bvh_dir: Path,
    cfg_paths: dict[str, Path],
    *,
    per_config: int = 12,
    category: str = "slam",
    joint_groups: list[str] | None = None,
    cell_width: int = 560,
    cell_height: int = 440,
    slam_window_ms: float = 250.0,
    include_human_row: bool = True,
    display_names: dict[str, str] | None = None,
    verbose: bool = True,
) -> dict[str, Path]:
    """Pick top events per config and render side-by-side strips for each.

    Returns a dict mapping event_id -> PNG path. Deduplicates across configs:
    if both configs pick the same (clip, joint, peak_f) event, we render only
    one strip and share it.

    Frame selection per event is automatic — see ``_pick_event_frames`` — so
    callers no longer need to pass a fixed ``offset_frames`` tuple.
    """
    data = json.loads((bench_dir / "limit_events.json").read_text())
    fps = float(data.get("params", {}).get("fps", 120.0))
    picks = pick_top_events(
        data, per_config=per_config, category=category, joint_groups=joint_groups,
    )

    # Dedup events by (clip, joint, peak_f); union across configs.
    seen: dict[tuple[str, str, int], dict] = {}
    for cfg, evs in picks.items():
        for e in evs:
            key = (e["clip"], e["joint"], int(e["peak_f"]))
            if key not in seen:
                seen[key] = e

    sxs_dir = bench_dir / "frames" / "side_by_side"
    out: dict[str, Path] = {}
    queue: list[tuple[tuple[str, str, int], dict]] = []
    for key, e in seen.items():
        eid = _event_id(e["clip"], e)
        existing = sxs_dir / f"{eid}.png"
        if existing.exists():
            out[eid] = existing
        else:
            queue.append((key, e))

    if verbose:
        print(f"[INFO] {category}: {len(seen)} picked, "
              f"{len(seen) - len(queue)} already on disk, {len(queue)} new renders queued.")

    for i, (key, e) in enumerate(queue):
        try:
            png = render_event_strip(
                e, bench_dir=bench_dir, bvh_dir=bvh_dir,
                cfg_paths=cfg_paths,
                fps=fps,
                slam_window_ms=slam_window_ms,
                cell_width=cell_width, cell_height=cell_height,
                include_human_row=include_human_row,
                display_names=display_names,
            )
            out[_event_id(e["clip"], e)] = png
            if verbose:
                print(f"[OK] ({i+1}/{len(queue)}) {png.name}")
        except Exception as exc:
            if verbose:
                print(f"[WARN] failed to render event {key}: {exc}")

    return out

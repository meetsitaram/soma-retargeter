# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Play 2-3 retargeted X2 CSVs side-by-side with the source BVH human mesh in the Newton SOMA viewer.

This is a stripped-down "playback only" variant of ``bvh_to_csv_converter.py``:
no IK, no UI buttons, no batched export — just the motions playing back in
the same Newton/Vulkan viewer that the original SOMA pipeline uses, with the
proper skinned SOMA human mesh instead of a capsule stick figure.

Two-robot usage:
    uv run python app/play_csvs_with_human.py \\
        --csv-a scratch/.../clip__x2_chain_matched.csv \\
        --csv-b scratch/.../clip__x2_uniform_h170_tuned.csv \\
        --label-a chain_matched --label-b h1.7_tuned \\
        --bvh auto                        # auto-resolve from <bench>/corpus.json

Three-robot usage (just add --csv-c / --label-c):
    uv run python app/play_csvs_with_human.py \\
        --csv-a .../clip__x2_chain_matched.csv         --label-a chain_matched \\
        --csv-b .../clip__x2_uniform_h170_tuned.csv   --label-b h1.7_tuned \\
        --csv-c .../clip__x2_shoulder_fix.csv         --label-c shoulder_fix \\
        --bvh auto

Default layout is ``--layout row`` — the SOMA human and all robots stand in
a single line along the viewer's Y axis (all at the same X depth), with the
human at the camera-LEFT end and the robots filling in from there. Newton's
camera looks from +X toward the origin, so +Y is camera-right.

  ``--layout row`` (default, 3 robots, row_spacing=0.85):

    SOMA human   robot A   robot B   robot C
       y=-1.275  y=-0.425  y=+0.425  y=+1.275       (all x=0)

  ``--layout human-back`` (legacy):

  Two robots:                                    x       y
    SOMA human            (back, centered)      -1.5     0.00
    robot A               (front, camera-left)   0.0    -0.75
    robot B               (front, camera-right)  0.0    +0.75

  Three robots:                                  x       y
    SOMA human            (back, centered)      -1.5     0.00
    robot A               (front, camera-left)   0.0    -0.75
    robot B               (front, center)        0.0     0.00
    robot C               (front, camera-right)  0.0    +0.75
"""

import argparse
import json
import pathlib
import time

import newton
import warp as wp

import soma_retargeter.assets.bvh as bvh_utils
import soma_retargeter.assets.csv as csv_utils
import soma_retargeter.utils.io_utils as io_utils
import soma_retargeter.utils.math_utils as math_utils
import soma_retargeter.pipelines.utils as pipeline_utils

from soma_retargeter.renderers.mesh_renderer import SkeletalMeshRenderer
from soma_retargeter.animation.skeleton import SkeletonInstance
from soma_retargeter.utils.space_conversion_utils import (
    SpaceConverter, get_facing_direction_type_from_str,
)


_DEFAULT_HUMAN_COLOR = (235.0 / 255.0, 245.0 / 255.0, 112.0 / 255.0)


def _resolve_bvh_auto(csv_a: pathlib.Path) -> pathlib.Path:
    """Resolve the source BVH for ``csv_a`` from a sibling ``corpus.json``."""
    csv_a = csv_a.resolve()
    bench_dir = csv_a.parent.parent   # .../<bench>/csvs/<file>.csv -> <bench>
    corpus_json = bench_dir / "corpus.json"
    if not corpus_json.exists():
        raise SystemExit(
            f"--bvh auto: {corpus_json} not found; pass an explicit BVH path instead."
        )
    csv_stem = csv_a.stem
    if "__" not in csv_stem:
        raise SystemExit(
            f"--bvh auto: cannot derive clip stem from {csv_stem}"
        )
    clip_stem = csv_stem.rsplit("__", 1)[0]
    target = clip_stem + ".bvh"
    for entry in json.loads(corpus_json.read_text()):
        if entry.get("name") == target:
            return pathlib.Path(entry["path"])
    raise SystemExit(f"--bvh auto: '{target}' not found in {corpus_json}")


class PlaybackApp:
    """Playback-only Viewer with N (2 or 3) X2 robots + skinned SOMA human mesh."""

    def __init__(
        self,
        viewer,
        config: dict,
        csv_paths: list,
        labels: list,
        bvh_path: pathlib.Path,
        layout: str,
        human_x: float,
        human_y: float,
        robots_x: float,
        side_by_side_y: float,
        row_spacing: float,
        fps: float = 60.0,
    ):
        if len(csv_paths) != len(labels):
            raise ValueError("csv_paths and labels must have the same length")
        if not (2 <= len(csv_paths) <= 3):
            raise ValueError("PlaybackApp supports 2 or 3 robot CSVs")
        if layout not in ("row", "human-back"):
            raise ValueError(f"unknown --layout {layout!r}; use 'row' or 'human-back'")

        self.viewer = viewer
        self.viewer.vsync = True
        self.config = config
        self.converter = SpaceConverter(
            get_facing_direction_type_from_str(
                self.config.get("retarget_source_facing_direction", "Mujoco")
            )
        )
        self.fps = fps
        self.frame_dt = 1.0 / self.fps
        self.time = 0.0
        self.is_playing = True
        self.playback_time = 0.0
        self.playback_speed = 1.0
        self.playback_loop = True

        self.show_skeleton_mesh = True
        self.show_gizmos = True
        self.labels = list(labels)
        self.layout = layout

        # ----- world / robot + human placement ---------------------------------
        self.num_robots = len(csv_paths)

        if layout == "row":
            # Human + N robots all at the same X, evenly spaced along Y, with
            # the human at the camera-LEFT end (most-negative Y) and robots
            # filling in to the right. Adjacent entities are `row_spacing` m
            # apart and the whole row is centred on y=0.
            num_entities = 1 + self.num_robots
            half = (num_entities - 1) / 2.0
            entity_ys = [(i - half) * row_spacing for i in range(num_entities)]
            human_pos = (robots_x, entity_ys[0])
            robot_ys = entity_ys[1:]
            self._slot_names = (
                ["robot-right"] if self.num_robots == 1
                else ["robot-left", "robot-right"] if self.num_robots == 2
                else ["robot-left", "robot-mid", "robot-right"]
            )
        else:  # "human-back" — legacy layout
            # Human placed 1.5 m behind the robots, robots in a row.
            if self.num_robots == 2:
                robot_ys = [-side_by_side_y / 2.0, +side_by_side_y / 2.0]
                self._slot_names = ["camera-left", "camera-right"]
            else:  # 3 robots
                robot_ys = [-side_by_side_y / 2.0, 0.0, +side_by_side_y / 2.0]
                self._slot_names = ["camera-left", "center", "camera-right"]
            human_pos = (human_x, human_y)

        # Stash the actual placed human position so the rest of the app
        # (UI panel, print summary, animation_offsets default) sees the
        # layout-derived value instead of the user's --human-x/--human-y
        # defaults, which are only meaningful in 'human-back' mode.
        self._human_pos = human_pos

        # Robot offsets are *editable via gizmos*: each frame the Newton viewer
        # mutates these transforms in place if the user drags the gizmo, then
        # we clamp Z to floor and rotation to yaw-only before applying.
        self.robot_offsets = [
            wp.transform(wp.vec3(robots_x, y, 0.0), wp.quat_identity())
            for y in robot_ys
        ]

        x2_mjcf = (
            pathlib.Path(__file__).resolve().parent.parent
            / "soma_retargeter"
            / "robot_assets"
            / "agibot_x2_ultra"
            / "x2_ultra.xml"
        )
        robot_builder = newton.ModelBuilder()
        robot_builder.add_mjcf(str(x2_mjcf))

        builder = newton.ModelBuilder()
        builder.add_ground_plane()
        for _ in range(self.num_robots):
            builder.add_builder(robot_builder, wp.transform_identity())
        self.model = builder.finalize()

        self.viewer.set_model(self.model)
        self.viewer.set_world_offsets([0, 0, 0])
        self.state = self.model.state()

        self.robot_num_joint_q = (
            self.model.joint_coord_count // self.model.articulation_count
        )
        self.robot_joint_q_offsets = [
            int(i * self.robot_num_joint_q) for i in range(self.model.articulation_count)
        ]
        self.robot_default_joint_q_values = self.model.joint_q.numpy()

        # ----- CSV motion buffers ----------------------------------------------
        csv_config = csv_utils.AgibotX2Ultra31DOF_CSVConfig()
        self.robot_csv_animation_buffers = [
            csv_utils.load_csv(str(p), csv_config=csv_config) for p in csv_paths
        ]

        # ----- BVH human (mesh) -------------------------------------------------
        self.skeleton, animation = bvh_utils.load_bvh(str(bvh_path))
        # The skeleton instance's xform is the static SOMA->Mujoco facing
        # transform. The user-controllable position of the human lives in
        # animation_offsets[0] (so it can be dragged via the gizmo).
        base_xform = self.converter.transform(wp.transform_identity())
        self.skeleton_instances = [
            SkeletonInstance(self.skeleton, _DEFAULT_HUMAN_COLOR, base_xform)
        ]
        self.animation_offsets = [
            wp.transform(
                wp.vec3(self._human_pos[0], self._human_pos[1], 0.0),
                wp.quat_identity(),
            )
        ]
        self.animation_buffers = [animation]
        self.human_x = self._human_pos[0]
        self.human_y = self._human_pos[1]

        try:
            self.skeletal_mesh = pipeline_utils.get_source_model_mesh(
                pipeline_utils.SourceType.SOMA, self.skeleton
            )
            self.skeletal_mesh_renderer = SkeletalMeshRenderer(self.skeletal_mesh)
        except Exception as exc:
            print(
                f"[WARN] Unable to load SOMA skinned mesh ({exc}); the human will "
                "not be rendered. Falling back: only the two X2 robots will be shown."
            )
            self.skeletal_mesh = None
            self.skeletal_mesh_renderer = None

        # ----- timing -----------------------------------------------------------
        bvh_max = animation.num_frames * (1.0 / animation.sample_rate)
        csv_durations = [
            b.num_frames * (1.0 / b.sample_rate)
            for b in self.robot_csv_animation_buffers
        ]
        csv_max = max(csv_durations)
        csv_min = min(csv_durations)
        # Cap playback at the SHORTEST motion (human BVH or any robot CSV) so
        # the human and all robots stay in sync. Otherwise the bench's
        # --max-frames cap (default 3000 frames ≈ 25 s) truncates the CSV
        # while the BVH plays in full, and the affected robots appear
        # "frozen" at their last pose while the human keeps moving —
        # nonsensical past that point.
        self.playback_total_time = min(bvh_max, csv_min)
        if bvh_max - csv_min > 0.5:
            print(
                f"[WARN] BVH ({bvh_max:.1f}s) is longer than the shortest CSV "
                f"({csv_min:.1f}s) — clamping playback to {self.playback_total_time:.1f}s "
                "so robots and human stay synced. Re-retarget with a higher "
                "--max-frames if you need the full clip."
            )
        if csv_max - csv_min > 0.1:
            spans = ", ".join(
                f"{lab}={d:.1f}s"
                for lab, d in zip(self.labels, csv_durations)
            )
            print(
                f"[WARN] CSV lengths differ: {spans}. Playback capped to the shortest."
            )

        if hasattr(self.viewer, "renderer") and self.viewer.renderer is not None:
            try:
                title = "SOMA -> X2 playback: human | " + " | ".join(self.labels)
                self.viewer.renderer.set_title(title)
            except Exception:
                pass

        print(
            f"[playback] BVH={bvh_path.name}: "
            f"{animation.num_frames} frames @ {animation.sample_rate:.1f} Hz"
        )
        for i, (label, buf) in enumerate(
            zip(self.labels, self.robot_csv_animation_buffers)
        ):
            pos = self.robot_offsets[i].p
            print(
                f"[playback] CSV {chr(ord('A') + i)} ({label}, {self._slot_names[i]}) "
                f"@ (x={pos[0]:+.2f}, y={pos[1]:+.2f}): "
                f"{buf.num_frames} frames @ {buf.sample_rate:.1f} Hz"
            )
        human_slot = "leftmost" if layout == "row" else "back, centered"
        print(
            f"[playback] Human ({human_slot}) @ "
            f"(x={self._human_pos[0]:+.2f}, y={self._human_pos[1]:+.2f})"
        )
        print(
            f"[playback] Layout: {layout}"
            + (f"  row_spacing={row_spacing:.2f} m" if layout == "row" else "")
        )
        print(
            f"[playback] Total playback time: {self.playback_total_time:.2f} s "
            f"(viewer FPS: {self.fps:.0f})"
        )

        # Register a small UI panel for scene toggles (gizmos / mesh / playback).
        if not isinstance(self.viewer, newton.viewer.ViewerNull):
            try:
                self.viewer.register_ui_callback(
                    lambda ui: self._draw_ui_panel(ui), position="free"
                )
            except Exception:
                pass

    @staticmethod
    def _clamp_gizmo_transform(tx: wp.transform) -> wp.transform:
        """Snap a gizmo-edited transform to (x, y, z=0) + yaw-only rotation.

        The Newton viewer's gizmo lets the user translate AND rotate freely
        in world space. For floor-plane motion comparisons we want the
        instance to stay on the ground (Z=0) and only swing around the
        vertical axis, matching what the original ``bvh_to_csv_converter``
        app enforces.
        """
        return wp.transform(
            wp.vec3(tx.p[0], tx.p[1], 0.0),
            math_utils.quat_twist(wp.vec3(0.0, 0.0, 1.0), tx.q),
        )

    def _draw_ui_panel(self, ui):
        """Imgui panel with the toggles the user typically wants for playback."""
        try:
            viewport = ui.get_main_viewport()
            # Make the panel a bit taller when there's a third robot row.
            panel_h = 200 + (20 if self.num_robots == 3 else 0)
            panel_size = ui.ImVec2(300, panel_h)
            ui.set_next_window_pos(
                ui.ImVec2(
                    viewport.size.x - 10 - panel_size.x,
                    viewport.size.y - 10 - panel_size.y,
                )
            )
            ui.set_next_window_size(panel_size)
            ui.set_next_window_bg_alpha(0.9)
            flags = ui.WindowFlags_.no_collapse | ui.WindowFlags_.no_resize
            ui.begin("Playback", flags=flags)

            for i, label in enumerate(self.labels):
                p = self.robot_offsets[i].p
                ui.text(
                    f"{chr(ord('A') + i)}  "
                    f"(x={p[0]:+.2f}, y={p[1]:+.2f})  "
                    f"{self._slot_names[i]:<11}: {label}"
                )
            hp = self.animation_offsets[0].p
            human_slot = "leftmost" if self.layout == "row" else "back"
            ui.text(
                f"H  (x={hp[0]:+.2f}, y={hp[1]:+.2f})  {human_slot:<11}: SOMA human"
            )
            ui.separator()
            _, self.show_skeleton_mesh = ui.checkbox(
                "Show human mesh", self.show_skeleton_mesh
            )
            _, self.show_gizmos = ui.checkbox(
                "Show move/rotate gizmos", self.show_gizmos
            )
            _, self.is_playing = ui.checkbox("Playing", self.is_playing)
            ui.text(
                f"Time: {self.playback_time:6.2f} / {self.playback_total_time:6.2f} s"
            )
            ui.end()
        except Exception:
            # Some viewers (null, usd) lack imgui — silently no-op.
            pass

    def update_robot_states(self):
        for i in range(self.num_robots):
            robot_offset = self.robot_offsets[i]
            joint_q_offset = self.robot_joint_q_offsets[i]
            buffer = self.robot_csv_animation_buffers[i]
            prev_xform = wp.transform(buffer.xform)
            buffer.xform = robot_offset
            data = buffer.sample(self.playback_time)
            wp.copy(
                self.model.joint_q,
                wp.array(data, dtype=wp.float32),
                joint_q_offset,
                0,
                self.robot_num_joint_q,
            )
            buffer.xform = prev_xform
        newton.eval_fk(
            self.model,
            self.model.joint_q,
            self.model.joint_qd,
            self.state,
            None,
        )

    def step(self):
        self.time += self.frame_dt
        if self.is_playing:
            self.playback_time += self.frame_dt * self.playback_speed
            if self.playback_loop and self.playback_total_time > 0.0:
                self.playback_time %= self.playback_total_time
            else:
                self.playback_time = max(
                    0.0, min(self.playback_time, self.playback_total_time)
                )

        # The gizmos in the viewer mutated these transforms in-place last
        # render (if the user dragged them). Clamp the result to floor-Z and
        # yaw-only rotation before applying for this frame's playback.
        for i in range(len(self.robot_offsets)):
            self.robot_offsets[i] = self._clamp_gizmo_transform(
                self.robot_offsets[i]
            )
        for i in range(len(self.animation_offsets)):
            self.animation_offsets[i] = self._clamp_gizmo_transform(
                self.animation_offsets[i]
            )

        for i in range(len(self.animation_buffers)):
            self.skeleton_instances[i].set_local_transforms(
                self.animation_buffers[i].sample(self.playback_time)
            )

        self.update_robot_states()

    def render(self):
        self.viewer.begin_frame(self.time)
        if (
            self.skeletal_mesh_renderer is not None
            and len(self.animation_buffers) > 0
        ):
            for i in range(len(self.skeleton_instances)):
                inst = self.skeleton_instances[i]
                prev_xform = wp.transform(inst.xform)
                inst.xform = wp.mul(self.animation_offsets[i], inst.xform)
                if self.show_skeleton_mesh:
                    self.skeletal_mesh_renderer.draw(
                        self.viewer, inst, inst.color, i
                    )
                inst.xform = prev_xform

        if self.show_gizmos:
            for i, offset in enumerate(self.robot_offsets):
                self.viewer.log_gizmo(f"robot_offset_{i}", offset)
            for i, offset in enumerate(self.animation_offsets):
                self.viewer.log_gizmo(f"human_offset_{i}", offset)

        self.viewer.log_state(self.state)
        self.viewer.end_frame()

    def run(self):
        print()
        print("Playback controls (Newton viewer):")
        print("  mouse drag        = orbit camera")
        print("  scroll            = zoom")
        print("  right-drag        = pan")
        print("  click gizmo handle and drag = move that robot / human around")
        print("    (Z and tilt are auto-clamped; only ground-plane XY + yaw)")
        print("  bottom-right panel: toggle 'human mesh', 'gizmos', 'Playing'")
        print()
        while self.viewer.is_running():
            self.step()
            self.render()
        self.viewer.close()


def main():
    import newton.examples

    parser = newton.examples.create_parser()
    parser.add_argument("--csv-a", required=True, type=str,
                        help="First retargeted CSV (camera-left).")
    parser.add_argument("--csv-b", required=True, type=str,
                        help="Second retargeted CSV (camera-right in 2-robot "
                             "mode, center in 3-robot mode).")
    parser.add_argument("--csv-c", default=None, type=str,
                        help="Optional third retargeted CSV. When provided, "
                             "three robots are rendered: A=left, B=center, "
                             "C=right.")
    parser.add_argument("--label-a", default="A", type=str)
    parser.add_argument("--label-b", default="B", type=str)
    parser.add_argument("--label-c", default="C", type=str)
    parser.add_argument(
        "--bvh",
        type=str,
        default="auto",
        help="Source BVH path, or 'auto' to resolve from <bench_dir>/corpus.json.",
    )
    parser.add_argument(
        "--config",
        type=str,
        default=str(
            pathlib.Path(__file__).resolve().parent.parent
            / "assets"
            / "default_bvh_to_csv_converter_config.json"
        ),
        help="JSON config (only 'retarget_source_facing_direction' is used).",
    )
    parser.add_argument(
        "--layout",
        choices=["row", "human-back"],
        default="row",
        help="Spatial layout. 'row' (default): SOMA human + N robots all at "
             "the same X depth, in a single horizontal line along Y, with "
             "the human at the camera-LEFT end. 'human-back': legacy layout "
             "with the human placed 1.5 m behind the row of robots.",
    )
    parser.add_argument(
        "--row-spacing",
        type=float,
        default=0.85,
        help="In --layout row, distance (m) between adjacent entities along "
             "Y. Default 0.85 m fits an X2 + neighbour without overlap.",
    )
    parser.add_argument("--side-by-side-y", type=float, default=1.5,
                        help="In --layout human-back, Y separation (m) between "
                             "the outermost robots. Ignored when --layout=row.")
    parser.add_argument("--robots-x", type=float, default=0.0,
                        help="X position of every robot AND (in --layout row) "
                             "the human as well. Newton's camera looks from "
                             "+X toward origin, so increasing this moves the "
                             "whole row toward the camera.")
    parser.add_argument("--human-x", type=float, default=-1.5,
                        help="X position of the human mesh in --layout "
                             "human-back. Ignored in --layout row "
                             "(human X comes from --robots-x).")
    parser.add_argument("--human-y", type=float, default=0.0,
                        help="Y position of the human mesh in --layout "
                             "human-back. Ignored in --layout row.")
    parser.add_argument("--fps", type=float, default=60.0,
                        help="Viewer / playback FPS.")
    viewer, args = newton.examples.init(parser)

    csv_paths = []
    labels = []
    for letter, raw_csv, label in (
        ("a", args.csv_a, args.label_a),
        ("b", args.csv_b, args.label_b),
        ("c", args.csv_c, args.label_c),
    ):
        if raw_csv is None:
            continue
        p = pathlib.Path(raw_csv).expanduser().resolve()
        if not p.is_file():
            raise SystemExit(f"--csv-{letter} file not found: {p}")
        csv_paths.append(p)
        labels.append(label)

    if args.bvh == "auto":
        bvh_path = _resolve_bvh_auto(csv_paths[0])
    else:
        bvh_path = pathlib.Path(args.bvh).expanduser().resolve()
    if not bvh_path.is_file():
        raise SystemExit(f"BVH file not found: {bvh_path}")

    if not pathlib.Path(args.config).exists():
        raise SystemExit(f"--config json not found: {args.config}")
    config = io_utils.load_json(args.config)

    with wp.ScopedDevice(args.device):
        app = PlaybackApp(
            viewer=viewer,
            config=config,
            csv_paths=csv_paths,
            labels=labels,
            bvh_path=bvh_path,
            layout=args.layout,
            human_x=args.human_x,
            human_y=args.human_y,
            robots_x=args.robots_x,
            side_by_side_y=args.side_by_side_y,
            row_spacing=args.row_spacing,
            fps=args.fps,
        )
        app.run()


if __name__ == "__main__":
    main()

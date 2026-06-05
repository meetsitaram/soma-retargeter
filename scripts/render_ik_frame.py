"""Render a robot frame with overlaid IK targets and residuals.

Embodiment-agnostic in spirit, but currently wired to the X2 Ultra MJCF
(the only target our SOMA pipeline supports right now). To support another
embodiment, swap `scripts/bench/kinematics.load_x2_mj_model` for that
embodiment's loader.

Three operating modes:

    --mode png         Single frame -> single PNG
    --mode strip       Multiple frames concatenated horizontally -> one PNG
    --mode interactive Passive MuJoCo viewer that scrubs through the clip

Examples
--------
Single frame snapshot (good for embedding in reports):

    python scripts/render_ik_frame.py \\
        --csv scratch/csv/A021__FINAL_v5.csv \\
        --bvh ../bones-seed/extracted/locowalk/walk_forward_loop_001__A021.bvh \\
        --config soma_retargeter/configs/agibot_x2_ultra/soma_to_x2_ultra_retargeter_config.json \\
        --frame 320 --out scratch/dbg/A021_v5_f320.png

Strip of several frames around an IK failure peak:

    python scripts/render_ik_frame.py \\
        --csv ... --bvh ... --config ... \\
        --mode strip --frames 300,310,320,330 --out scratch/dbg/strip.png

Interactive scrub (requires X11/Wayland):

    python scripts/render_ik_frame.py \\
        --csv ... --bvh ... --config ... \\
        --mode interactive --frame 0
"""

from __future__ import annotations

import argparse
from pathlib import Path

import sys
REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from scripts.bench import render  # noqa: E402


def _parse_frames(spec: str) -> list[int]:
    out: list[int] = []
    for token in spec.split(","):
        token = token.strip()
        if not token:
            continue
        if "-" in token:
            a, b = token.split("-", 1)
            out.extend(range(int(a), int(b) + 1))
        else:
            out.append(int(token))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", type=Path, required=True, help="Retargeted CSV (X2 Ultra 31-DOF)")
    ap.add_argument("--bvh", type=Path, required=True, help="Source SOMA BVH")
    ap.add_argument("--config", type=Path, required=True, help="Retargeter config JSON (must point at the same scaler that produced the CSV)")
    ap.add_argument("--mode", choices=("png", "strip", "interactive"), default="png")
    ap.add_argument("--frame", type=int, default=0, help="Frame index for --mode png/interactive")
    ap.add_argument("--frames", type=str, default=None,
                    help="Frame spec for --mode strip, e.g. '300,310,320' or '300-310'")
    ap.add_argument("--out", type=Path, default=None, help="Output PNG (png/strip modes)")
    ap.add_argument("--width", type=int, default=1024)
    ap.add_argument("--height", type=int, default=768)
    ap.add_argument("--distance", type=float, default=2.4, help="Camera distance from pelvis (m)")
    ap.add_argument("--azim", type=float, default=110.0, help="Camera azimuth (deg)")
    ap.add_argument("--elev", type=float, default=-12.0, help="Camera elevation (deg)")
    ap.add_argument("--no-targets", action="store_true", help="Skip target tripods")
    ap.add_argument("--no-residuals", action="store_true", help="Skip yellow residual lines")
    ap.add_argument("--max-frames", type=int, default=None,
                    help="Cap on BVH frames used when computing IK targets (matches the cap used during retargeting)")
    ap.add_argument("--fps", type=float, default=30.0, help="Playback FPS (interactive)")
    args = ap.parse_args()

    show_targets = not args.no_targets
    show_residuals = not args.no_residuals

    if args.mode == "png":
        if args.out is None:
            raise SystemExit("--out is required for --mode png")
        render.render_frame(
            csv_path=args.csv.expanduser().resolve(),
            bvh_path=args.bvh.expanduser().resolve(),
            retarget_config_path=args.config.expanduser().resolve(),
            frame_idx=args.frame,
            out_png=args.out.expanduser().resolve(),
            width=args.width,
            height=args.height,
            distance=args.distance,
            azim=args.azim,
            elev=args.elev,
            max_frames=args.max_frames,
            show_targets=show_targets,
            show_residuals=show_residuals,
        )
        print(f"[OK] wrote {args.out}")

    elif args.mode == "strip":
        if args.frames is None or args.out is None:
            raise SystemExit("--frames and --out are required for --mode strip")
        frames = _parse_frames(args.frames)
        render.render_strip(
            csv_path=args.csv.expanduser().resolve(),
            bvh_path=args.bvh.expanduser().resolve(),
            retarget_config_path=args.config.expanduser().resolve(),
            frames=frames,
            out_png=args.out.expanduser().resolve(),
            width=args.width,
            height=args.height,
            distance=args.distance,
            azim=args.azim,
            elev=args.elev,
            max_frames=args.max_frames,
            show_targets=show_targets,
            show_residuals=show_residuals,
        )
        print(f"[OK] wrote strip of {len(frames)} frames to {args.out}")

    elif args.mode == "interactive":
        render.interactive(
            csv_path=args.csv.expanduser().resolve(),
            bvh_path=args.bvh.expanduser().resolve(),
            retarget_config_path=args.config.expanduser().resolve(),
            start_frame=args.frame,
            fps=args.fps,
            max_frames=args.max_frames,
        )

    else:
        raise SystemExit(f"unknown --mode {args.mode}")


if __name__ == "__main__":
    main()

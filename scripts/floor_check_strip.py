"""Render floor-visible side-by-side strips for the chain_matched verdict.

For each (clip_stem, frame_idx) pair, build a single PNG that shows N configs
side-by-side at the same animation frame, with the camera lowered + tilted so
the floor plane and both feet are always visible. This is the visual proof
for the foot-floor metrics in `summary.md`.

Usage:

    uv run python scripts/floor_check_strip.py \
        --bench-dir scratch/bench_20260604_211307 \
        --configs x2_uniform_h140 x2_uniform_h170_tuned hybrid x2_chain_matched \
        --display-name x2_uniform_h140='h=1.40' \
        --display-name x2_uniform_h170_tuned='h=1.70+wrist_smooth' \
        --display-name hybrid='h=1.40+wrist_smooth' \
        --display-name x2_chain_matched='chain_matched' \
        --clip walk_forward_loop_001__A021=1500 \
        --clip dance_basic_turn_v1_360_R_loop_fast_004__A322=500 \
        --clip neutral_idle_turn_360_002__A077_M=300 \
        --clip big_light_two_hands_right_side_high_to_behind_high_R_001__A525=400 \
        --out frames/floor_check
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from scripts.bench import render as bench_render  # noqa: E402
from scripts.bench import kinematics as bench_kinematics  # noqa: E402


# Camera tuned to fit pelvis + feet + ~3 m of floor in front of the robot.
# Default render.render_frame uses elev=-12 and looks at pelvis_pos. That
# clips the floor below the robot. Here we drop the lookat to ~0.35 m
# (knee height) and steepen the elevation to -22° so the floor plane and
# both feet are inside the frame.
FLOOR_CAM = dict(distance=2.6, azim=110.0, elev=-22.0)
FLOOR_LOOKAT_Z = 0.35


def _label_strip(img: np.ndarray, text: str, color: tuple[int, int, int] = (240, 240, 240)) -> np.ndarray:
    """Draw a caption strip at the bottom of an image. Uses PIL for text."""
    from PIL import Image, ImageDraw, ImageFont
    pil = Image.fromarray(img)
    h_strip = 28
    out = Image.new("RGB", (pil.width, pil.height + h_strip), (16, 18, 22))
    out.paste(pil, (0, 0))
    draw = ImageDraw.Draw(out)
    try:
        font = ImageFont.truetype("DejaVuSans-Bold.ttf", 16)
    except OSError:
        font = ImageFont.load_default()
    draw.text((10, pil.height + 4), text, fill=color, font=font)
    return np.array(out)


def render_floor_cell(
    csv_path: Path, bvh_path: Path, retarget_config_path: Path,
    frame_idx: int, width: int, height: int,
) -> np.ndarray:
    """Single floor-friendly render of one (config, frame) cell.

    Reuses ``bench_render.render_frame`` but with the wider distance / steeper
    elevation defined in ``FLOOR_CAM``. We can't override the lookat directly
    through the existing API (it always uses ``pelvis_pos``), so we monkey-patch
    `_setup_camera` for the duration of this call.
    """
    import mujoco

    # Stash + replace the camera helper so we get a knee-height lookat.
    orig_setup = bench_render._setup_camera

    def _floor_setup(model, data, focus_pos, distance, azim, elev):
        # Override only the Z component so we still track left/right
        # motion through the clip.
        focus = np.array(focus_pos, dtype=np.float64)
        focus[2] = FLOOR_LOOKAT_Z
        return orig_setup(model, data, focus, distance=distance, azim=azim, elev=elev)

    bench_render._setup_camera = _floor_setup
    try:
        img = bench_render.render_frame(
            csv_path=csv_path,
            bvh_path=bvh_path,
            retarget_config_path=retarget_config_path,
            frame_idx=frame_idx,
            out_png=None,
            width=width, height=height,
            distance=FLOOR_CAM["distance"],
            azim=FLOOR_CAM["azim"],
            elev=FLOOR_CAM["elev"],
            show_targets=False,   # cleaner image for the floor read
            show_residuals=False,
        )
    finally:
        bench_render._setup_camera = orig_setup
    return img


def render_strip(
    bench_dir: Path, clip_stem: str, frame_idx: int,
    configs: list[str], cfg_paths: dict[str, Path],
    display_names: dict[str, str],
    cell_width: int, cell_height: int,
) -> np.ndarray:
    """Concatenate one cell per config into a horizontal strip with a caption."""
    csv_dir = bench_dir / "csvs"
    bvh_path = _resolve_bvh(clip_stem)
    if bvh_path is None:
        raise FileNotFoundError(f"BVH not found for clip {clip_stem}")

    cells: list[np.ndarray] = []
    for cfg in configs:
        csv = csv_dir / f"{clip_stem}__{cfg}.csv"
        if not csv.is_file():
            cells.append(np.zeros((cell_height, cell_width, 3), dtype=np.uint8))
            continue
        img = render_floor_cell(
            csv, bvh_path, cfg_paths[cfg],
            frame_idx, cell_width, cell_height,
        )
        label = display_names.get(cfg, cfg)
        img = _label_strip(img, f"{label}  |  f={frame_idx}", color=(255, 220, 100))
        cells.append(img)

    # Concatenate horizontally.
    max_h = max(c.shape[0] for c in cells)
    cells_padded = []
    for c in cells:
        if c.shape[0] != max_h:
            pad = np.zeros((max_h - c.shape[0], c.shape[1], 3), dtype=np.uint8)
            c = np.concatenate([c, pad], axis=0)
        cells_padded.append(c)
    strip = np.concatenate(cells_padded, axis=1)

    # Header caption above the strip
    from PIL import Image, ImageDraw, ImageFont
    pil = Image.fromarray(strip)
    head_h = 38
    out = Image.new("RGB", (pil.width, pil.height + head_h), (16, 18, 22))
    out.paste(pil, (0, head_h))
    draw = ImageDraw.Draw(out)
    try:
        font = ImageFont.truetype("DejaVuSans-Bold.ttf", 18)
    except OSError:
        font = ImageFont.load_default()
    draw.text((12, 10), f"{clip_stem}  @  frame {frame_idx}   |   floor-visible camera",
              fill=(240, 240, 240), font=font)
    return np.array(out)


def _resolve_bvh(clip_stem: str) -> Path | None:
    """Look up the BVH file by stem under the bones-seed/extracted hierarchy."""
    bones_root = REPO_ROOT.parent / "bones-seed" / "extracted"
    if not bones_root.is_dir():
        return None
    matches = list(bones_root.rglob(f"{clip_stem}.bvh"))
    return matches[0] if matches else None


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bench-dir", type=Path, required=True)
    ap.add_argument("--configs", nargs="+", required=True,
                    help="Config labels (in display order). Each must have a configs/<label>.json under bench-dir.")
    ap.add_argument("--display-name", action="append", default=None,
                    help="Map config label to display label. Repeatable.")
    ap.add_argument("--clip", action="append", required=True,
                    help="Clip frame to render in CLIP_STEM=FRAME_IDX format. Repeatable.")
    ap.add_argument("--out", type=Path, default=None,
                    help="Output directory (default: {bench-dir}/frames/floor_check)")
    ap.add_argument("--cell-width", type=int, default=560)
    ap.add_argument("--cell-height", type=int, default=420)
    args = ap.parse_args()

    bench_dir = args.bench_dir.resolve()
    if not bench_dir.is_dir():
        raise SystemExit(f"bench dir not found: {bench_dir}")
    out_dir = (args.out or bench_dir / "frames" / "floor_check").resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    cfg_paths = {c: bench_dir / "configs" / f"{c}.json" for c in args.configs}
    for c, p in cfg_paths.items():
        if not p.is_file():
            raise SystemExit(f"config missing: {p}")

    display_names: dict[str, str] = {}
    for pair in (args.display_name or []):
        if "=" in pair:
            k, v = pair.split("=", 1)
            display_names[k.strip()] = v.strip()

    out_index = []
    for spec in args.clip:
        if "=" not in spec:
            raise SystemExit(f"--clip expects CLIP_STEM=FRAME_IDX, got {spec!r}")
        clip_stem, fstr = spec.split("=", 1)
        frame_idx = int(fstr)
        print(f"[INFO] rendering {clip_stem} @ frame {frame_idx}")
        strip = render_strip(
            bench_dir, clip_stem, frame_idx,
            configs=args.configs, cfg_paths=cfg_paths,
            display_names=display_names,
            cell_width=args.cell_width, cell_height=args.cell_height,
        )
        out_png = out_dir / f"floor_check__{clip_stem}__f{frame_idx}.png"
        from PIL import Image
        Image.fromarray(strip).save(out_png)
        print(f"  -> {out_png}")
        out_index.append({"clip": clip_stem, "frame": frame_idx, "path": str(out_png)})
        bench_kinematics.clear_targets_cache()

    (out_dir / "_index.json").write_text(json.dumps(out_index, indent=2))
    print(f"\n[OK] wrote {len(out_index)} strips to {out_dir}")


if __name__ == "__main__":
    main()

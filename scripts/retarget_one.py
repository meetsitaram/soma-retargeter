"""Retarget a single SOMA BVH to an X2 Ultra CSV with overridable knobs.

Diagnostic helper used while re-tuning the SOMA -> X2 retargeter on the
walk_forward_loop_001__A021 reference clip. The retargeter normally loads
its config from disk; this script lets us override `model_height` (and
optionally point to an alternate scaler config) in memory so we can
parameter-sweep without thrashing the committed JSON files.

For chain-matched (and other non-default) retargets that need IK weight /
mask changes on top of a swapped scaler, pass the full retargeter config
via ``--retargeter-config`` instead of overriding piecemeal.

Usage examples:

    python scripts/retarget_one.py \
        --bvh assets/motions/x2-diag-bvh/walk_forward_loop_001__A021.bvh \
        --out scratch/csv/A021__h1.70.csv

    python scripts/retarget_one.py \
        --bvh assets/motions/x2-diag-bvh/walk_forward_loop_001__A021.bvh \
        --model-height 1.40 \
        --out scratch/csv/A021__h1.40.csv

    python scripts/retarget_one.py \
        --bvh assets/motions/x2-diag-bvh/walk_forward_loop_001__A021.bvh \
        --model-height 1.40 \
        --scaler-config scratch/configs/sweep_cell_42.json \
        --out scratch/csv/sweep_cell_42.csv

    python scripts/retarget_one.py \
        --bvh assets/motions/x2-diag-bvh/walk_forward_loop_001__A021.bvh \
        --retargeter-config \
            soma_retargeter/configs/agibot_x2_ultra/soma_to_x2_ultra_chain_matched_retargeter_config.json \
        --out scratch/csv/A021__chain_matched.csv
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import tempfile
from pathlib import Path

import warp as wp

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

import soma_retargeter.assets.bvh as bvh_utils  # noqa: E402
import soma_retargeter.assets.csv as csv_utils  # noqa: E402
import soma_retargeter.pipelines.utils as pipeline_utils  # noqa: E402
import soma_retargeter.utils.io_utils as io_utils  # noqa: E402
from soma_retargeter.pipelines.newton_pipeline import NewtonPipeline  # noqa: E402
from soma_retargeter.utils.space_conversion_utils import (  # noqa: E402
    SpaceConverter,
    get_facing_direction_type_from_str,
)


def _load_retarget_config(
    scaler_override: Path | None,
    model_height_override: float | None,
    retargeter_override: Path | None = None,
) -> dict:
    if retargeter_override is not None:
        cfg = io_utils.load_json(retargeter_override.resolve())
    else:
        cfg_path = io_utils.get_config_file("agibot_x2_ultra", "soma_to_x2_ultra_retargeter_config.json")
        cfg = io_utils.load_json(cfg_path)
    if model_height_override is not None:
        cfg["model_height"] = float(model_height_override)
    if scaler_override is not None:
        # Stash an absolute path. `io_utils.get_config_file()` joinpaths with
        # an absolute Path-like input -> returns the absolute path unchanged
        # (POSIX semantics), so passing an absolute string here works.
        cfg["human_robot_scaler_config"] = str(scaler_override.resolve())
    return cfg


def retarget(bvh_path: Path, out_csv: Path, retarget_config: dict) -> None:
    out_csv.parent.mkdir(parents=True, exist_ok=True)

    importer = bvh_utils.BVHImporter()
    bvh_skeleton, _ = importer.create_skeleton(bvh_path)

    converter = SpaceConverter(get_facing_direction_type_from_str("Mujoco"))
    bvh_tx = converter.transform(wp.transform_identity())

    _, animation = bvh_utils.load_bvh(bvh_path, bvh_skeleton)

    pipeline = NewtonPipeline(
        bvh_skeleton,
        source_type="soma",
        robot_type="agibot_x2_ultra",
        retarget_config=retarget_config,
    )
    pipeline.add_input_motions([animation], [bvh_tx], scale_animation=True)
    csv_buffers = pipeline.execute()

    if not csv_buffers:
        raise RuntimeError("Retargeting produced no CSV buffer")

    csv_utils.save_csv(str(out_csv), csv_buffers[0], csv_config=csv_utils.AgibotX2Ultra31DOF_CSVConfig())
    print(f"[INFO]: Saved {out_csv} ({csv_buffers[0].num_frames} frames)")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bvh", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--model-height", type=float, default=None,
                        help="Override `model_height` in the retargeter config (e.g. 1.40)")
    parser.add_argument("--scaler-config", type=Path, default=None,
                        help="Override scaler config JSON path (relative to package root or absolute)")
    parser.add_argument("--retargeter-config", type=Path, default=None,
                        help="Use a full retargeter config JSON instead of the default base + overrides "
                             "(e.g. soma_retargeter/configs/agibot_x2_ultra/"
                             "soma_to_x2_ultra_chain_matched_retargeter_config.json). "
                             "`--scaler-config` / `--model-height` still apply on top if also given.")
    parser.add_argument("--device", default=None, help="Optional warp device (e.g. 'cuda:0')")
    args = parser.parse_args()

    bvh_path = args.bvh.expanduser().resolve()
    if not bvh_path.is_file():
        raise SystemExit(f"BVH not found: {bvh_path}")

    scaler_override = None
    if args.scaler_config is not None:
        scaler_override = args.scaler_config.expanduser().resolve()
        if not scaler_override.is_file():
            raise SystemExit(f"Scaler config not found: {scaler_override}")

    retargeter_override = None
    if args.retargeter_config is not None:
        retargeter_override = args.retargeter_config.expanduser().resolve()
        if not retargeter_override.is_file():
            raise SystemExit(f"Retargeter config not found: {retargeter_override}")

    cfg = _load_retarget_config(scaler_override, args.model_height, retargeter_override)
    print(f"[INFO]: model_height = {cfg['model_height']}")
    print(f"[INFO]: scaler_config = {cfg['human_robot_scaler_config']}")
    print(f"[INFO]: BVH = {bvh_path.name}")

    if args.device:
        with wp.ScopedDevice(args.device):
            retarget(bvh_path, args.out.expanduser().resolve(), cfg)
    else:
        retarget(bvh_path, args.out.expanduser().resolve(), cfg)


if __name__ == "__main__":
    main()

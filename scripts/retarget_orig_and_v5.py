"""Retarget a single BVH with both:
  - ORIG: the pre-2026-re-tuning state (model_height=1.70, Hand t_weight=2.0,
          original LeftHand/RightHand quaternion offsets from the committed
          version of the scaler config).
  - V5  : whatever is currently on disk (the in-progress tuning).

Used for visual A/B comparison across motions while the working tree is
dirty. Reads the ORIG values from
``scratch/configs/orig_scaler.json`` (a snapshot of the pre-change scaler
config produced with ``git show HEAD:...``).

Usage:
    python scripts/retarget_orig_and_v5.py \
        --bvh assets/motions/x2-dance-bvh/dance_hiphop_shuffle_square_R_fast_002__A318.bvh
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import warp as wp

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

import soma_retargeter.assets.bvh as bvh_utils  # noqa: E402
import soma_retargeter.assets.csv as csv_utils  # noqa: E402
import soma_retargeter.utils.io_utils as io_utils  # noqa: E402
from soma_retargeter.pipelines.newton_pipeline import NewtonPipeline  # noqa: E402
from soma_retargeter.utils.space_conversion_utils import (  # noqa: E402
    SpaceConverter,
    get_facing_direction_type_from_str,
)


def run(bvh_path: Path, ret_cfg: dict, out_csv: Path) -> None:
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    importer = bvh_utils.BVHImporter()
    skel, _ = importer.create_skeleton(bvh_path)
    _, animation = bvh_utils.load_bvh(bvh_path, skel)
    conv = SpaceConverter(get_facing_direction_type_from_str("Mujoco"))
    bvh_tx = conv.transform(wp.transform_identity())

    pipe = NewtonPipeline(skel, "soma", "agibot_x2_ultra", retarget_config=ret_cfg)
    pipe.add_input_motions([animation], [bvh_tx], scale_animation=True)
    out = pipe.execute()
    csv_utils.save_csv(str(out_csv), out[0], csv_config=csv_utils.AgibotX2Ultra31DOF_CSVConfig())
    print(f"[INFO]: Saved {out_csv} ({out[0].num_frames} frames)")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bvh", required=True, type=Path)
    parser.add_argument("--out-dir", default="scratch/csv", type=Path)
    parser.add_argument("--orig-scaler-config",
                        default=str(REPO_ROOT / "scratch/configs/orig_scaler.json"),
                        type=str)
    args = parser.parse_args()

    bvh_path = args.bvh.expanduser().resolve()
    if not bvh_path.is_file():
        raise SystemExit(f"BVH not found: {bvh_path}")

    orig_scaler_path = Path(args.orig_scaler_config).expanduser().resolve()
    if not orig_scaler_path.is_file():
        raise SystemExit(
            f"ORIG scaler config not found: {orig_scaler_path}. "
            "Regenerate with: git show HEAD:soma_retargeter/configs/agibot_x2_ultra/"
            "soma_to_x2_ultra_scaler_config.json > scratch/configs/orig_scaler.json"
        )

    # --- ORIG config (pre-2026 re-tuning) ---
    orig_ret = io_utils.load_json(
        io_utils.get_config_file("agibot_x2_ultra", "soma_to_x2_ultra_retargeter_config.json"))
    orig_ret["model_height"] = 1.70
    orig_ret["human_robot_scaler_config"] = str(orig_scaler_path)
    orig_ret["ik_map"]["LeftHand"]["t_weight"] = 2.0
    orig_ret["ik_map"]["RightHand"]["t_weight"] = 2.0
    orig_ret["ik_map"]["LeftForeArm"]["t_weight"] = 1.0
    orig_ret["ik_map"]["RightForeArm"]["t_weight"] = 1.0

    # --- v5 config (whatever is on disk right now) ---
    v5_ret = io_utils.load_json(
        io_utils.get_config_file("agibot_x2_ultra", "soma_to_x2_ultra_retargeter_config.json"))

    stem = bvh_path.stem
    orig_csv = args.out_dir / f"{stem}__ORIG.csv"
    v5_csv = args.out_dir / f"{stem}__v5.csv"

    print(f"[INFO]: ORIG : model_height={orig_ret['model_height']} hand_t_weight=2.0")
    run(bvh_path, orig_ret, orig_csv)

    print(f"[INFO]: V5   : model_height={v5_ret['model_height']} "
          f"hand_t_weight={v5_ret['ik_map']['LeftHand']['t_weight']}")
    run(bvh_path, v5_ret, v5_csv)

    print()
    print("Compare visually with:")
    print(f"  python scripts/compare_two_csvs.py \\")
    print(f"    --csv-a {orig_csv} \\")
    print(f"    --csv-b {v5_csv} \\")
    print(f"    --label-a ORIG --label-b v5")


if __name__ == "__main__":
    main()

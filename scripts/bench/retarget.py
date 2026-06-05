"""Invoke the SOMA -> X2 Ultra retargeting pipeline with a config override.

This is a thin wrapper around scripts/retarget_one.py's logic. It supports:
- pointing at any retargeter config JSON (absolute path)
- truncating very long BVHs to a sane frame cap so the bench finishes in
  reasonable time without changing metric semantics (rates/means rescale OK).
"""

from __future__ import annotations

import json
from pathlib import Path

import warp as wp

import soma_retargeter.assets.bvh as bvh_utils
import soma_retargeter.assets.csv as csv_utils
import soma_retargeter.utils.io_utils as io_utils
from soma_retargeter.animation.animation_buffer import AnimationBuffer
from soma_retargeter.pipelines.newton_pipeline import NewtonPipeline
from soma_retargeter.utils.space_conversion_utils import (
    SpaceConverter,
    get_facing_direction_type_from_str,
)


def load_retarget_config(retarget_config_path: Path) -> dict:
    """Load a retargeter config JSON from an absolute path.

    Resolves any relative scaler/feet/initialization-pose paths sitting inside
    the JSON against the directory containing the config itself, so the bench
    can point at copies dropped anywhere on disk.
    """
    cfg = json.loads(retarget_config_path.read_text())
    cfg_dir = retarget_config_path.parent.resolve()

    for rel_key in (
        "human_robot_scaler_config",
        "feet_stabilizer_config",
        "initialization_pose",
    ):
        val = cfg.get(rel_key)
        if not val:
            continue
        # Already-absolute paths are honored as-is. Otherwise we try the
        # config-relative location first; if that doesn't exist, fall back to
        # the package config resolver so configs that ship with the package
        # still resolve via the normal io_utils path lookup.
        if Path(val).is_absolute():
            continue
        candidate = (cfg_dir / val).resolve()
        if candidate.is_file():
            cfg[rel_key] = str(candidate)

    return cfg


def _truncate_animation(animation, max_frames: int | None):
    if max_frames is None or animation.num_frames <= max_frames:
        return animation

    truncated_data = animation.local_transforms[:max_frames]
    new_anim = AnimationBuffer(
        animation.skeleton,
        max_frames,
        animation.sample_rate,
        truncated_data,
    )
    return new_anim


def retarget_clip(
    bvh_path: Path,
    out_csv: Path,
    retarget_config_path: Path,
    max_frames: int | None = None,
    facing: str = "Mujoco",
) -> int:
    """Retarget one BVH to one CSV using the provided config JSON.

    Returns the number of retargeted frames written.
    """
    cfg = load_retarget_config(retarget_config_path)
    out_csv.parent.mkdir(parents=True, exist_ok=True)

    importer = bvh_utils.BVHImporter()
    bvh_skeleton, _ = importer.create_skeleton(bvh_path)

    converter = SpaceConverter(get_facing_direction_type_from_str(facing))
    bvh_tx = converter.transform(wp.transform_identity())

    _, animation = bvh_utils.load_bvh(bvh_path, bvh_skeleton)
    animation = _truncate_animation(animation, max_frames)

    pipeline = NewtonPipeline(
        bvh_skeleton,
        source_type="soma",
        robot_type="agibot_x2_ultra",
        retarget_config=cfg,
    )
    pipeline.add_input_motions([animation], [bvh_tx], scale_animation=True)
    csv_buffers = pipeline.execute()

    if not csv_buffers:
        raise RuntimeError("Retargeting produced no CSV buffer")

    csv_utils.save_csv(
        str(out_csv),
        csv_buffers[0],
        csv_config=csv_utils.AgibotX2Ultra31DOF_CSVConfig(),
    )
    return int(csv_buffers[0].num_frames)

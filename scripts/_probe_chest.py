"""Probe waist_pitch dependence on the Chest IK objective.

Runs three configurations on the A021 BVH and reports waist_pitch stats:
1. Baseline: Chest r_weight=0.7, current offset
2. r_weight=0:  zero out chest orientation tracking
3. delta_Y on Chest offset: apply -18 deg Y correction
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import warp as wp
from scipy.spatial.transform import Rotation as R

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import soma_retargeter.assets.bvh as bvh_utils  # noqa: E402
import soma_retargeter.assets.csv as csv_utils  # noqa: E402
import soma_retargeter.utils.io_utils as io_utils  # noqa: E402
from soma_retargeter.pipelines.newton_pipeline import NewtonPipeline  # noqa: E402
from soma_retargeter.utils.space_conversion_utils import (  # noqa: E402
    SpaceConverter,
    get_facing_direction_type_from_str,
)


def run(label, ret_cfg_overrides, scaler_overrides):
    bvh_path = REPO_ROOT / "assets/motions/x2-diag-bvh/walk_forward_loop_001__A021.bvh"
    out_csv = REPO_ROOT / f"scratch/csv/A021__probe_{label}.csv"

    ret_cfg = io_utils.load_json(io_utils.get_config_file(
        "agibot_x2_ultra", "soma_to_x2_ultra_retargeter_config.json"))
    for k, v in ret_cfg_overrides.items():
        if isinstance(v, dict):
            ret_cfg[k] = {**ret_cfg.get(k, {}), **v}
        else:
            ret_cfg[k] = v

    if scaler_overrides is not None:
        scaler_cfg = io_utils.load_json(io_utils.get_config_file(
            "agibot_x2_ultra", "soma_to_x2_ultra_scaler_config.json"))
        for path_, value in scaler_overrides.items():
            d = scaler_cfg
            keys = path_.split(".")
            for k in keys[:-1]:
                d = d[k]
            d[keys[-1]] = value
        tmp = REPO_ROOT / f"scratch/configs/probe_scaler_{label}.json"
        tmp.parent.mkdir(parents=True, exist_ok=True)
        tmp.write_text(json.dumps(scaler_cfg, indent=2))
        ret_cfg["human_robot_scaler_config"] = str(tmp)

    importer = bvh_utils.BVHImporter()
    skel, _ = importer.create_skeleton(bvh_path)
    _, animation = bvh_utils.load_bvh(bvh_path, skel)
    converter = SpaceConverter(get_facing_direction_type_from_str("Mujoco"))
    bvh_tx = converter.transform(wp.transform_identity())

    pipe = NewtonPipeline(skel, "soma", "agibot_x2_ultra", retarget_config=ret_cfg)
    pipe.add_input_motions([animation], [bvh_tx], scale_animation=True)
    bufs = pipe.execute()
    csv_utils.save_csv(str(out_csv), bufs[0], csv_config=csv_utils.AgibotX2Ultra31DOF_CSVConfig())
    return out_csv


def main():
    # 1) baseline (just rebuild same final config)
    run("baseline", {}, None)

    # 2) zero out chest rotation weight
    run("chest_r0", {"ik_map": {"Chest": {"t_body": "torso_link", "r_body": "torso_link",
                                          "t_weight": 0.7, "r_weight": 0.0}}}, None)

    # 3) apply delta_Y = -18 deg correction to Chest offset
    chest_q_old = [0.478, 0.521, 0.521, 0.478]
    base = R.from_quat(chest_q_old)
    new = base * R.from_euler("y", -18, degrees=True)
    chest_q_new = [round(float(v), 4) for v in new.as_quat()]
    print(f"chest_q_new = {chest_q_new}")
    run("chest_dy_-18", {}, {"joint_offsets.Chest": [[0.005, 0.0, -0.094], chest_q_new]})


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Batch-retarget Unitree G1 CSVs to Agibot X2 Ultra CSVs (G1 -> X2 method).

Standalone entry point, alongside app/bvh_to_csv_converter.py (which is SOMA -> robot).
Consumes G1 retarget CSVs (36 cols: root + 29 DOF, as produced by `bvh_to_csv_converter
--config <... retarget_target: unitree_g1 ...>`) and writes X2 Ultra CSVs (38 cols).

Usage (from the repo root, with the project venv):
    .venv/bin/python app/g1_csv_to_x2_csv.py --g1-dir <dir of G1 csvs> --out-dir <dir>

See docs/g1_to_x2.md for the method write-up and diagram.
"""

import argparse
from pathlib import Path

import warp as wp

from soma_retargeter.pipelines.g1_to_x2_pipeline import G1ToX2Retargeter

_CFG_DIR = Path(__file__).resolve().parents[1] / "soma_retargeter" / "configs" / "agibot_x2_ultra"


def main():
    ap = argparse.ArgumentParser(description="Retarget G1 CSVs to X2 Ultra CSVs")
    ap.add_argument("--g1-dir", required=True, help="directory of G1 retarget CSVs")
    ap.add_argument("--out-dir", required=True, help="directory to write X2 Ultra CSVs")
    ap.add_argument("--config", default=None, help="retargeter-config JSON override")
    ap.add_argument("--calibration", default=None, help="calibration JSON override")
    ap.add_argument("--acrobatic", action="store_true",
                    help="airborne/inverted motion (cartwheels, backflips): pelvis-center "
                         "scaling + feet stabilizer off + geometry floor-clamp. Selects the "
                         "acrobatic config/calibration unless --config/--calibration override.")
    ap.add_argument("--g1-mjcf", default=None, help="G1 MJCF path override (default: newton asset)")
    ap.add_argument("--device", default="cuda:0")
    args = ap.parse_args()

    config, calibration = args.config, args.calibration
    if args.acrobatic:
        config = config or str(_CFG_DIR / "g1_to_x2_ultra_acrobatic_retargeter_config.json")
        calibration = calibration or str(_CFG_DIR / "g1_to_x2_ultra_acrobatic_calibration.json")

    wp.init()
    with wp.ScopedDevice(args.device):
        r = G1ToX2Retargeter(
            retargeter_config=config, calibration=calibration, g1_mjcf=args.g1_mjcf)
        n = r.retarget_dir(args.g1_dir, args.out_dir)
    print(f"Retargeted {n} clip(s): {args.g1_dir} -> {args.out_dir}" + ("  [acrobatic]" if args.acrobatic else ""))


if __name__ == "__main__":
    main()

"""Batched G1->X2 retarget driver for the executed-feasible corpus.

Uses G1ToX2Retargeter.retarget_batch (one GPU IK call per batch) with the
pin-root config. Length-sorted, batched, resumable (skips existing out CSVs).
Run from the soma-retargeter repo root with its .venv:

    .venv/bin/python batched_g1x2_driver.py --g1-dir <executed csv dir> \
        --out-dir <x2 csv dir> [--batch 64] [--limit N]
"""
import argparse, os, glob
import numpy as np
import warp as wp
from soma_retargeter.pipelines.g1_to_x2_pipeline import G1ToX2Retargeter

CFGDIR = "soma_retargeter/configs/agibot_x2_ultra"
CFG = f"{CFGDIR}/g1_to_x2_ultra_pinroot_retargeter_config.json"
ACRO_CFG = f"{CFGDIR}/g1_to_x2_ultra_acrobatic_retargeter_config.json"
ACRO_CAL = f"{CFGDIR}/g1_to_x2_ultra_acrobatic_calibration.json"


def sanity_check(csv_path, fps, max_jump=1.0, max_yaw_rate=6.0, max_tilt_frac=0.01):
    """Post-solve output check: catches wrong-IK-basin solutions (folded/rolled
    body tracking the keypoints from inside a bad basin) that produced the
    2026-07 kplanner corpus corruption. Returns list of failed check names.

    NOTE for upright thresholds: acrobatic clips (cartwheels etc.) legitimately
    tilt; pass max_tilt_frac=1.0 to disable the tilt check for --acrobatic runs.
    """
    m = np.loadtxt(csv_path, delimiter=",", skiprows=1, dtype=np.float64)
    if m.ndim == 1 or m.shape[0] < 3:
        return []
    fails = []
    rx, ry = m[:, 4], m[:, 5]                       # root euler deg
    norm = lambda a: np.abs((a + 180.0) % 360.0 - 180.0)
    tilt = np.maximum(norm(rx), norm(ry)) > 60.0
    if tilt.mean() > max_tilt_frac:
        fails.append(f"tilt({tilt.mean() * 100:.0f}%)")
    dof = np.deg2rad(m[:, 7:])
    j = np.abs(np.diff(dof, axis=0)).max() if dof.shape[0] > 1 else 0.0
    if j > max_jump:
        fails.append(f"jump({j:.2f})")
    # hip pitch far past limits = folded-body basin even with an upright root
    hip = np.abs(dof[:, [0, 6]]) > 2.0              # left/right hip pitch, rad
    if hip.any(axis=1).mean() > max_tilt_frac:
        fails.append(f"hip_pitch({hip.any(axis=1).mean() * 100:.0f}%)")
    yaw = np.unwrap(np.deg2rad(m[:, 6]))
    yr = np.abs(np.diff(yaw)) * fps
    if yr.size and yr.max() > max_yaw_rate:
        fails.append(f"yaw_rate({yr.max():.1f})")
    return fails


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--g1-dir", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--batch", type=int, default=64)
    ap.add_argument("--limit", type=int, default=0, help="only first N clips (for validation)")
    ap.add_argument("--clips", nargs="*", default=None, help="explicit clip stems (validation)")
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--sample-rate", type=float, default=50.0,
                    help="input fps of the G1 CSVs (executed dump is 50, not the 120 default)")
    ap.add_argument("--acrobatic", action="store_true",
                    help="use the floor-clamp (acrobatic) config for ground-contact motions")
    ap.add_argument("--shard-index", type=int, default=0,
                    help="this process's shard k (0-based); processes todo[k::shard_count]")
    ap.add_argument("--shard-count", type=int, default=1,
                    help="total number of parallel shards (disjoint clip partitions, shared out dir)")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    cfg = ACRO_CFG if args.acrobatic else CFG
    cal = ACRO_CAL if args.acrobatic else None

    if args.clips:
        stems = args.clips
    else:
        stems = sorted(os.path.splitext(os.path.basename(p))[0]
                       for p in glob.glob(f"{args.g1_dir}/*.csv"))
    # skip clips already done (resumable)
    todo = [s for s in stems if not os.path.exists(f"{args.out_dir}/{s}.csv")]
    if args.limit:
        todo = todo[:args.limit]
    # length-sort for GPU efficiency (similar lengths per batch)
    def nframes(s):
        try:
            with open(f"{args.g1_dir}/{s}.csv") as f:
                return sum(1 for _ in f)
        except OSError:
            return 0
    todo.sort(key=nframes, reverse=True)
    # length-sorted stride keeps each shard length-balanced (mix of long+short)
    if args.shard_count > 1:
        todo = todo[args.shard_index::args.shard_count]
    print(f"[driver] shard {args.shard_index}/{args.shard_count}: {len(todo)} clips "
          f"(batch={args.batch}, sr={args.sample_rate})", flush=True)

    wp.init()
    done = 0
    rejected = 0
    with wp.ScopedDevice(args.device):
        r = G1ToX2Retargeter(retargeter_config=cfg, calibration=cal)
        print(f"[driver] config={'ACROBATIC/floor-clamp' if args.acrobatic else 'pin-root/offset'} "
              f"floor_clamp={r.floor_clamp} foot_offset={r.foot_ground_offset_cm}", flush=True)
        for bi in range(0, len(todo), args.batch):
            batch = todo[bi:bi + args.batch]
            items = [(f"{args.g1_dir}/{s}.csv", f"{args.out_dir}/{s}.csv") for s in batch]
            # replicate retarget_batch but with the correct input sample rate
            n = r.pipe.num_initialization_frames + r.pipe.num_stabilization_frames
            padded = []
            for g1_csv, _ in items:
                kp = r._keypoints(g1_csv).astype(np.float32)
                p = np.concatenate([np.repeat(kp[:1], n, axis=0), kp], axis=0) if n else kp
                padded.append(np.ascontiguousarray(p, dtype=np.float32))
            r.pipe.clear()
            r.pipe.input_targets = padded
            r.pipe.input_sample_rates = [args.sample_rate] * len(items)
            r.pipe.max_frames = max(len(p) for p in padded)
            results = r.pipe.execute()
            import soma_retargeter.assets.csv as csv_utils
            for (g1_csv, out_csv), res in zip(items, results):
                csv_utils.save_csv(out_csv, res, csv_config=r.csv_cfg)
                r._arm_jointmap(out_csv, g1_csv)
                if r.floor_clamp:
                    r._floor_clamp(out_csv, g1_csv)
                elif r.foot_ground_offset_cm:
                    r._lift_root_z(out_csv, r.foot_ground_offset_cm)
                fails = sanity_check(
                    out_csv, args.sample_rate,
                    max_tilt_frac=1.0 if args.acrobatic else 0.01)
                if fails:
                    rej_dir = os.path.join(args.out_dir, "rejected")
                    os.makedirs(rej_dir, exist_ok=True)
                    rej = os.path.join(rej_dir, os.path.basename(out_csv))
                    os.replace(out_csv, rej)
                    rejected += 1
                    with open(os.path.join(rej_dir, "rejected.log"), "a") as f:
                        f.write(f"{os.path.basename(out_csv)}\t{'+'.join(fails)}\n")
                    print(f"[driver] REJECT {os.path.basename(out_csv)}: {'+'.join(fails)}", flush=True)
            done += len(batch)
            print(f"[driver] {done}/{len(todo)} (rejected {rejected})", flush=True)
    print(f"[driver] DONE: {done} clips -> {args.out_dir}", flush=True)


if __name__ == "__main__":
    main()

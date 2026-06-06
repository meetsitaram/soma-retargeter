# X2 Ultra Retargeter — Three-Config Comparison

Side-by-side playback of three SOMA → X2 Ultra retarget configurations
against the source human BVH (skinned mesh) on three test clips. All CSVs
are full-length (no `--max-frames` truncation).

## Configs

| | `x2_shoulder_fix` *(commit [`601c105`](https://github.com/meetsitaram/soma-retargeter/commit/601c105e2924e63264963405197da302791e719d))* | `x2_uniform_h170_tuned` | `x2_chain_matched` |
|---|---|---|---|
| **Scaler** | uniform | uniform | per-chain |
| `model_height` | 1.70 | 1.70 | 1.80 *(ignored by per-chain)* |
| `Hips.r_weight` | 2.0 | **10.0** | **10.0** |
| `Hand.t_weight` | 2.0 | 2.0 | 2.0 |
| `Hand.r_weight` | 0.1 | **0.2** | **0.2** |
| `shoulder_pitch/yaw` mask | **0.05** | 0.1 | 0.1 |
| `shoulder_roll` mask | 0.0 | 0.0 | 0.0 |
| Wrist smoothing masks | — | **0.1** (pitch+roll, both wrists) | **0.1** (pitch+roll, both wrists) |
| One-line summary | Relaxed shoulder masks → arms hang naturally; default pelvis lock. | Hard pelvis lock + extra wrist smoothing → robust on fast hand swings. | Per-chain rescale of human → robot link lengths; inherits h170_tuned's other tweaks. |

## Clips

| Clip | Frames | Duration |
|---|---:|---:|
| `walk_forward_loop_001__A021` | 2 636 | 22.0 s |
| `dance_latino_chase_mambo_pivot_R_001__A313` | 1 117 | 9.3 s |
| `body_check_004__A444` | 4 618 | 38.5 s |

## Viewer commands

Order from camera-LEFT to camera-RIGHT: human · `shoulder_fix` · `h1.7_tuned` · `chain_matched`. Playback caps to the shortest motion so all four loop in sync.

```bash
cd /home/stickbot/Projects/GR00T-WholeBodyControl/agibot-x2-references/soma-retargeter
BENCH=scratch/bench_20260604_211307

# 1. walk_forward_loop_001__A021 (locomotion, 22 s)
CLIP=walk_forward_loop_001__A021
uv run python app/play_csvs_with_human.py \
  --csv-a $BENCH/csvs/${CLIP}__x2_shoulder_fix.csv          --label-a shoulder_fix \
  --csv-b $BENCH/csvs/${CLIP}__x2_uniform_h170_tuned.csv    --label-b h1.7_tuned \
  --csv-c $BENCH/csvs/${CLIP}__x2_chain_matched.csv         --label-c chain_matched \
  --bvh auto

# 2. dance_latino_chase_mambo_pivot_R_001__A313 (dance, 9 s)
CLIP=dance_latino_chase_mambo_pivot_R_001__A313
uv run python app/play_csvs_with_human.py \
  --csv-a $BENCH/csvs/${CLIP}__x2_shoulder_fix.csv          --label-a shoulder_fix \
  --csv-b $BENCH/csvs/${CLIP}__x2_uniform_h170_tuned.csv    --label-b h1.7_tuned \
  --csv-c $BENCH/csvs/${CLIP}__x2_chain_matched.csv         --label-c chain_matched \
  --bvh auto

# 3. body_check_004__A444 (long deep-crouch, 38 s)
CLIP=body_check_004__A444
uv run python app/play_csvs_with_human.py \
  --csv-a $BENCH/csvs/${CLIP}__x2_shoulder_fix.csv          --label-a shoulder_fix \
  --csv-b $BENCH/csvs/${CLIP}__x2_uniform_h170_tuned.csv    --label-b h1.7_tuned \
  --csv-c $BENCH/csvs/${CLIP}__x2_chain_matched.csv         --label-c chain_matched \
  --bvh auto
```

Layout / camera knobs:

- `--layout row` (default) — human + N robots in a single horizontal line along Y, all at the same X. Use `--row-spacing 1.0` for wider gaps.
- `--layout human-back` — legacy: human placed 1.5 m behind the row of robots.
- Drag entities at runtime with the on-screen gizmos (auto-clamped to floor + yaw).

## Videos

Drop recorded MP4s under `scratch/bench_20260604_211307/videos/` with these names; embed via:

```html
<video src="videos/walk_forward_loop_001__A021__three_configs.mp4" controls width="900"></video>
```

| Clip | Path |
|---|---|
| `walk_forward_loop_001__A021` | `videos/walk_forward_loop_001__A021__three_configs.mp4` |
| `dance_latino_chase_mambo_pivot_R_001__A313` | `videos/dance_latino_chase_mambo_pivot_R_001__A313__three_configs.mp4` |
| `body_check_004__A444` | `videos/body_check_004__A444__three_configs.mp4` |

## What to look for

| | `shoulder_fix` | `h1.7_tuned` | `chain_matched` |
|---|---|---|---|
| Arms-at-side rest pose | **Natural hang** | Flared shoulders | Flared shoulders |
| Wrist behaviour on fast hand swings | More twist excursion | Smooth (wrist smoothing on) | Smooth |
| Trunk / pelvis orientation tracking | Softer | Hard-locked | Hard-locked |
| Foot floor contact during locomotion | Some sinking/floating | Some sinking/floating | **Best (per-chain leg scaling)** |
| Hand height vs human (waist / chest mapping) | Same as h170 (uniform scaler) | Slight elevation (1.70 m scaling) | Best mapping |

## Provenance

| File | Source |
|---|---|
| `configs/x2_shoulder_fix.json` | Verbatim from upstream commit `601c105`. |
| `configs/x2_uniform_h170_tuned.json` | Previous best uniform-scale baseline. |
| `configs/x2_chain_matched.json` | Per-chain scaler (`soma_to_x2_ultra_chain_matched_config.json`) + h170_tuned IK weights. |
| `app/play_csvs_with_human.py` | Newton SOMA viewer + skinned human + 2-3 robots. Added `--csv-c`/`--label-c` and `--layout {row, human-back}`; playback time auto-clamps to `min(BVH, shortest CSV)` so all motions stay in sync. |

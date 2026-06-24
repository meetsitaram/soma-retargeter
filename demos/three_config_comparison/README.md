# Three-Config Comparison Demo

Self-contained data + commands to play back three SOMA → X2 Ultra retargeter
configurations side-by-side against the source human BVH (skinned mesh) in
the Newton SOMA viewer. Run from the repo root after `uv sync`.

## What's in here

```
demos/three_config_comparison/
├── README.md
├── corpus.json                          # used by --bvh auto (relative paths)
├── bvh/                                 # source SOMA human motion (3 clips)
│   ├── walk_forward_loop_001__A021.bvh
│   ├── dance_latino_chase_mambo_pivot_R_001__A313.bvh
│   └── body_check_004__A444.bvh
├── csvs/                                # retargeted X2 Ultra motion (3 clips × 3 configs)
│   ├── walk_forward_loop_001__A021__x2_shoulder_fix.csv
│   ├── walk_forward_loop_001__A021__x2_uniform_h170_tuned.csv
│   ├── walk_forward_loop_001__A021__x2_chain_matched.csv
│   ├── dance_latino_chase_mambo_pivot_R_001__A313__x2_shoulder_fix.csv
│   ├── dance_latino_chase_mambo_pivot_R_001__A313__x2_uniform_h170_tuned.csv
│   ├── dance_latino_chase_mambo_pivot_R_001__A313__x2_chain_matched.csv
│   ├── body_check_004__A444__x2_shoulder_fix.csv
│   ├── body_check_004__A444__x2_uniform_h170_tuned.csv
│   └── body_check_004__A444__x2_chain_matched.csv
└── configs/                             # the retargeter JSON used per config (reference only)
    ├── x2_shoulder_fix.json
    ├── x2_uniform_h170_tuned.json
    └── x2_chain_matched.json
```

## About `model_height`

`model_height` is the assumed **height of the SOMA human actor** (not the
robot). The scaler computes `ratio = model_height / human_height_assumption`
(default `human_height_assumption = 1.8 m`) and multiplies every entry in
`joint_scales` by that ratio
(`soma_retargeter/robotics/human_to_robot_scaler.py:22`). So setting it to
**1.40** uniformly shrinks every joint segment by 22 %. The same config
should work across robots — robot-specific tuning lives in per-joint
`joint_scales` and the `ik_map` `t_weight`/`r_weight`, not in
`model_height`. The cleaner path for hip-twist fixes (used by
`x2_uniform_h170_tuned` and `x2_chain_matched`) is to keep `model_height` at
1.7 and raise `Hips.r_weight` to 10 + add wrist smoothing.

## The three configs

| | `x2_shoulder_fix` ([`601c105`](https://github.com/meetsitaram/soma-retargeter/commit/601c105e2924e63264963405197da302791e719d)) | `x2_uniform_h170_tuned` | `x2_chain_matched` |
|---|---|---|---|
| Scaler | uniform | uniform | per-chain (matches X2 link lengths) |
| `model_height` *(SOMA human height)* | 1.70 | 1.70 | 1.80 *(ratio≈1.0 — per-chain scales override)* |
| `Hips.r_weight` (pelvis rot lock) | 2.0 | **10.0** | **10.0** |
| `Hand.r_weight` | 0.1 | **0.2** | **0.2** |
| `shoulder_pitch/yaw` mask | **0.05** | 0.1 | 0.1 |
| Wrist smoothing masks | — | **0.1** (pitch+roll) | **0.1** (pitch+roll) |
| One-line summary | Relaxed shoulder masks → arms hang naturally at rest. | Hard pelvis lock + extra wrist smoothing → robust on fast hand swings. | Per-chain rescale of human → robot link lengths; inherits h170_tuned's IK tweaks. |

## Run

Order from camera-LEFT to camera-RIGHT: human · `shoulder_fix` · `h1.7_tuned` · `chain_matched`. Playback total time is auto-clamped to the shortest motion so all four loop in sync.

```bash
cd <path-to-soma-retargeter-repo>
DEMO=demos/three_config_comparison

# 1. walk_forward_loop_001__A021 (locomotion, 22 s)
CLIP=walk_forward_loop_001__A021
uv run python app/play_csvs_with_human.py \
  --csv-a $DEMO/csvs/${CLIP}__x2_shoulder_fix.csv         --label-a shoulder_fix \
  --csv-b $DEMO/csvs/${CLIP}__x2_uniform_h170_tuned.csv   --label-b h1.7_tuned \
  --csv-c $DEMO/csvs/${CLIP}__x2_chain_matched.csv        --label-c chain_matched \
  --bvh $DEMO/bvh/${CLIP}.bvh

# 2. dance_latino_chase_mambo_pivot_R_001__A313 (dance, 9 s)
CLIP=dance_latino_chase_mambo_pivot_R_001__A313
uv run python app/play_csvs_with_human.py \
  --csv-a $DEMO/csvs/${CLIP}__x2_shoulder_fix.csv         --label-a shoulder_fix \
  --csv-b $DEMO/csvs/${CLIP}__x2_uniform_h170_tuned.csv   --label-b h1.7_tuned \
  --csv-c $DEMO/csvs/${CLIP}__x2_chain_matched.csv        --label-c chain_matched \
  --bvh $DEMO/bvh/${CLIP}.bvh

# 3. body_check_004__A444 (long deep-crouch, 38 s)
CLIP=body_check_004__A444
uv run python app/play_csvs_with_human.py \
  --csv-a $DEMO/csvs/${CLIP}__x2_shoulder_fix.csv         --label-a shoulder_fix \
  --csv-b $DEMO/csvs/${CLIP}__x2_uniform_h170_tuned.csv   --label-b h1.7_tuned \
  --csv-c $DEMO/csvs/${CLIP}__x2_chain_matched.csv        --label-c chain_matched \
  --bvh $DEMO/bvh/${CLIP}.bvh
```

`--bvh auto` also works in this folder if you pass any `csvs/...csv` to
`--csv-a` whose sibling `corpus.json` (one directory up) is the one shipped
here; the viewer resolves the relative paths in `corpus.json` against that
file's location so the demo is fully portable.

## Layout knobs

- `--layout row` (default) — human + 3 robots in a single horizontal line along Y, all at the same X (`y ∈ {-1.275, -0.425, +0.425, +1.275}` for `--row-spacing 0.85`).
- `--layout human-back` — legacy: human placed 1.5 m behind a row of robots.
- Drag entities at runtime with the on-screen gizmos (auto-clamped to floor + yaw).

## What to look for

| | `shoulder_fix` | `h1.7_tuned` | `chain_matched` |
|---|---|---|---|
| Arms-at-side rest pose | **Natural hang** | Flared shoulders | Flared shoulders |
| Wrist behaviour on fast hand swings | More twist excursion | Smooth (wrist smoothing on) | Smooth |
| Trunk / pelvis tracking | Softer | Hard-locked | Hard-locked |
| Foot floor contact during locomotion | Some sinking/floating | Some sinking/floating | **Best (per-chain leg scaling)** |
| Hand height vs human (waist / chest mapping) | Same as h170 (uniform scaler) | Slight elevation (1.70 m scaling) | Best mapping |

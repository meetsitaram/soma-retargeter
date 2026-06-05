# Joint-saturation clusters  (originally titled "IK failure sections")

> **NOTE — calibration (added 2026-06-05):** The original framing of this
> file as "IK failure sections" is misleading. Every one of the 303 + 125
> entries below was triggered by the saturation clause (≥ 4 of 31 DOFs
> within 5° of any limit). The alternative `FK residual ≥ 0.18 m` clause
> never fired anywhere in the corpus (max observed: 0.171 m). The X2 has
> asymmetric ranges on `wrist_pitch` (±32°), `wrist_roll` (-90°..+41°), and
> `shoulder_roll` (-3.5°..+171.5°) that make 4+ saturated DOFs the
> baseline for any two-hand pose, not a failure indicator.
>
> **For actual single-joint IK failures (slam / pin events), use
> [limit_events.md](limit_events.md) instead.** That file lists the top
> events per config with side-by-side renders comparing v5_ours vs
> colleague at the same frame, and identifies the specific joint and
> velocity. The headline there is that v5_ours has 364 wrist limit
> events across the corpus while colleague has 0 — that is the actionable
> diagnostic.
>
> The listings below are still useful as an "operating envelope" reference
> (which joints brush their limits how often), but should not be read as a
> per-section failure log.

---

A *section* is a contiguous run of frames where the IK is
either close to >= 4 hardware joint limits or has FK position
residual >= 0.18 m. Sections are listed in descending peak severity.

## Config `colleague`

### `walk_forward_relax_003__A005.bvh`  (11 sections, sat-score=44)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 1683 | 1698 | 1683 | 0.13 | saturation | 4 | 0.056 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 1796 | 1807 | 1802 | 0.10 | saturation | 4 | 0.066 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_roll_joint |
| 2324 | 2335 | 2326 | 0.10 | saturation | 4 | 0.070 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_pitch_joint |
| 1885 | 1893 | 1891 | 0.07 | saturation | 4 | 0.070 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_pitch_joint |
| 2641 | 2649 | 2641 | 0.07 | saturation | 4 | 0.065 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 930 | 937 | 930 | 0.07 | saturation | 4 | 0.065 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 984 | 991 | 987 | 0.07 | saturation | 4 | 0.069 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_roll_joint |
| 2493 | 2500 | 2493 | 0.07 | saturation | 4 | 0.067 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 1509 | 1515 | 1509 | 0.06 | saturation | 4 | 0.064 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 149 | 153 | 153 | 0.04 | saturation | 4 | 0.047 | right_elbow_joint, left_elbow_joint, right_shoulder_roll_joint |
| 1707 | 1711 | 1707 | 0.04 | saturation | 4 | 0.053 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f2132.png](frames/walk_forward_relax_003__A005__colleague/flag_pelvis_z_f2132.png)

![flag_wrist_ang_vel_f156.png](frames/walk_forward_relax_003__A005__colleague/flag_wrist_ang_vel_f156.png)

![flag_saturated_dof_f151.png](frames/walk_forward_relax_003__A005__colleague/flag_saturated_dof_f151.png)

![section_00_peak_f1683.png](frames/walk_forward_relax_003__A005__colleague/section_00_peak_f1683.png)

![section_01_peak_f1802.png](frames/walk_forward_relax_003__A005__colleague/section_01_peak_f1802.png)

![section_02_peak_f2326.png](frames/walk_forward_relax_003__A005__colleague/section_02_peak_f2326.png)

![section_03_peak_f1891.png](frames/walk_forward_relax_003__A005__colleague/section_03_peak_f1891.png)

![section_04_peak_f2641.png](frames/walk_forward_relax_003__A005__colleague/section_04_peak_f2641.png)

### `dance_retro_disco_finger_sequence_R_fast_002__A314.bvh`  (10 sections, sat-score=44)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 408 | 524 | 446 | 0.97 | saturation | 6 | 0.115 | left_shoulder_roll_joint, left_ankle_roll_joint, right_ankle_roll_joint |
| 81 | 168 | 92 | 0.73 | saturation | 5 | 0.050 | left_elbow_joint, left_ankle_roll_joint, right_ankle_roll_joint |
| 239 | 296 | 247 | 0.48 | saturation | 5 | 0.056 | right_ankle_roll_joint, left_ankle_roll_joint, right_shoulder_roll_joint |
| 617 | 668 | 652 | 0.43 | saturation | 4 | 0.072 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |
| 325 | 372 | 351 | 0.40 | saturation | 4 | 0.064 | left_shoulder_yaw_joint, left_elbow_joint, right_ankle_roll_joint |
| 765 | 806 | 806 | 0.35 | saturation | 4 | 0.054 | right_elbow_joint, left_elbow_joint, right_ankle_roll_joint |
| 188 | 223 | 200 | 0.30 | saturation | 4 | 0.054 | right_elbow_joint, left_elbow_joint, right_ankle_roll_joint |
| 382 | 404 | 394 | 0.19 | saturation | 4 | 0.056 | right_elbow_joint, left_elbow_joint, right_ankle_roll_joint |
| 543 | 556 | 556 | 0.12 | saturation | 4 | 0.050 | right_elbow_joint, left_elbow_joint, right_ankle_roll_joint |
| 61 | 65 | 61 | 0.04 | saturation | 4 | 0.048 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |

![flag_pelvis_z_f56.png](frames/dance_retro_disco_finger_sequence_R_fast_002__A314__colleague/flag_pelvis_z_f56.png)

![flag_wrist_ang_vel_f373.png](frames/dance_retro_disco_finger_sequence_R_fast_002__A314__colleague/flag_wrist_ang_vel_f373.png)

![flag_saturated_dof_f463.png](frames/dance_retro_disco_finger_sequence_R_fast_002__A314__colleague/flag_saturated_dof_f463.png)

![section_00_peak_f446.png](frames/dance_retro_disco_finger_sequence_R_fast_002__A314__colleague/section_00_peak_f446.png)

![section_01_peak_f92.png](frames/dance_retro_disco_finger_sequence_R_fast_002__A314__colleague/section_01_peak_f92.png)

![section_02_peak_f247.png](frames/dance_retro_disco_finger_sequence_R_fast_002__A314__colleague/section_02_peak_f247.png)

![section_03_peak_f652.png](frames/dance_retro_disco_finger_sequence_R_fast_002__A314__colleague/section_03_peak_f652.png)

![section_04_peak_f351.png](frames/dance_retro_disco_finger_sequence_R_fast_002__A314__colleague/section_04_peak_f351.png)

### `dance_basic_turn_v1_360_R_loop_fast_004__A322.bvh`  (9 sections, sat-score=41)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 87 | 130 | 129 | 0.37 | saturation | 6 | 0.051 | right_ankle_pitch_joint, left_ankle_roll_joint, right_elbow_joint |
| 220 | 240 | 239 | 0.17 | saturation | 5 | 0.046 | right_elbow_joint, right_ankle_pitch_joint, left_elbow_joint |
| 343 | 360 | 358 | 0.15 | saturation | 5 | 0.046 | right_elbow_joint, right_ankle_pitch_joint, left_elbow_joint |
| 157 | 165 | 157 | 0.07 | saturation | 5 | 0.044 | left_ankle_roll_joint, right_ankle_roll_joint, right_ankle_pitch_joint |
| 261 | 271 | 265 | 0.09 | saturation | 4 | 0.045 | left_ankle_pitch_joint, right_ankle_roll_joint, right_ankle_pitch_joint |
| 380 | 390 | 389 | 0.09 | saturation | 4 | 0.046 | left_ankle_pitch_joint, right_ankle_roll_joint, right_ankle_pitch_joint |
| 333 | 341 | 335 | 0.07 | saturation | 4 | 0.048 | right_elbow_joint, left_elbow_joint, right_ankle_pitch_joint |
| 278 | 285 | 285 | 0.07 | saturation | 4 | 0.045 | left_ankle_roll_joint, right_ankle_roll_joint, right_ankle_pitch_joint |
| 468 | 475 | 468 | 0.07 | saturation | 4 | 0.050 | right_elbow_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f420.png](frames/dance_basic_turn_v1_360_R_loop_fast_004__A322__colleague/flag_pelvis_z_f420.png)

![flag_wrist_ang_vel_f123.png](frames/dance_basic_turn_v1_360_R_loop_fast_004__A322__colleague/flag_wrist_ang_vel_f123.png)

![flag_saturated_dof_f129.png](frames/dance_basic_turn_v1_360_R_loop_fast_004__A322__colleague/flag_saturated_dof_f129.png)

![section_00_peak_f129.png](frames/dance_basic_turn_v1_360_R_loop_fast_004__A322__colleague/section_00_peak_f129.png)

![section_01_peak_f239.png](frames/dance_basic_turn_v1_360_R_loop_fast_004__A322__colleague/section_01_peak_f239.png)

![section_02_peak_f358.png](frames/dance_basic_turn_v1_360_R_loop_fast_004__A322__colleague/section_02_peak_f358.png)

![section_03_peak_f157.png](frames/dance_basic_turn_v1_360_R_loop_fast_004__A322__colleague/section_03_peak_f157.png)

![section_04_peak_f265.png](frames/dance_basic_turn_v1_360_R_loop_fast_004__A322__colleague/section_04_peak_f265.png)

### `walk_big_dog_ff_315_loop_R_002__A495.bvh`  (6 sections, sat-score=40)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 151 | 187 | 179 | 0.31 | saturation | 8 | 0.062 | right_elbow_joint, left_hip_roll_joint, right_hip_roll_joint |
| 238 | 271 | 261 | 0.28 | saturation | 8 | 0.070 | right_elbow_joint, left_hip_roll_joint, left_ankle_roll_joint |
| 326 | 361 | 339 | 0.30 | saturation | 7 | 0.065 | right_elbow_joint, right_ankle_roll_joint, left_ankle_roll_joint |
| 82 | 97 | 83 | 0.13 | saturation | 7 | 0.066 | right_elbow_joint, right_ankle_roll_joint, right_ankle_pitch_joint |
| 64 | 79 | 70 | 0.13 | saturation | 6 | 0.060 | right_elbow_joint, left_hip_roll_joint, right_ankle_roll_joint |
| 0 | 14 | 12 | 0.12 | saturation | 4 | 0.065 | left_shoulder_roll_joint, right_hip_roll_joint, right_ankle_roll_joint |

![flag_pelvis_z_f209.png](frames/walk_big_dog_ff_315_loop_R_002__A495__colleague/flag_pelvis_z_f209.png)

![flag_wrist_ang_vel_f20.png](frames/walk_big_dog_ff_315_loop_R_002__A495__colleague/flag_wrist_ang_vel_f20.png)

![flag_saturated_dof_f258.png](frames/walk_big_dog_ff_315_loop_R_002__A495__colleague/flag_saturated_dof_f258.png)

![section_00_peak_f179.png](frames/walk_big_dog_ff_315_loop_R_002__A495__colleague/section_00_peak_f179.png)

![section_01_peak_f261.png](frames/walk_big_dog_ff_315_loop_R_002__A495__colleague/section_01_peak_f261.png)

![section_02_peak_f339.png](frames/walk_big_dog_ff_315_loop_R_002__A495__colleague/section_02_peak_f339.png)

![section_03_peak_f83.png](frames/walk_big_dog_ff_315_loop_R_002__A495__colleague/section_03_peak_f83.png)

![section_04_peak_f70.png](frames/walk_big_dog_ff_315_loop_R_002__A495__colleague/section_04_peak_f70.png)

### `walk_ff_start_270_R_slow_001__A443.bvh`  (7 sections, sat-score=39)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 738 | 877 | 835 | 1.17 | saturation | 7 | 0.054 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_hip_roll_joint |
| 1032 | 1157 | 1116 | 1.05 | saturation | 7 | 0.051 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_hip_roll_joint |
| 480 | 596 | 546 | 0.97 | saturation | 7 | 0.052 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_hip_roll_joint |
| 202 | 305 | 297 | 0.87 | saturation | 6 | 0.055 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_hip_roll_joint |
| 687 | 699 | 687 | 0.11 | saturation | 4 | 0.048 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_pitch_joint |
| 973 | 980 | 980 | 0.07 | saturation | 4 | 0.051 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_roll_joint |
| 313 | 317 | 313 | 0.04 | saturation | 4 | 0.056 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |

![flag_pelvis_z_f211.png](frames/walk_ff_start_270_R_slow_001__A443__colleague/flag_pelvis_z_f211.png)

![flag_wrist_ang_vel_f396.png](frames/walk_ff_start_270_R_slow_001__A443__colleague/flag_wrist_ang_vel_f396.png)

![flag_saturated_dof_f818.png](frames/walk_ff_start_270_R_slow_001__A443__colleague/flag_saturated_dof_f818.png)

![section_00_peak_f835.png](frames/walk_ff_start_270_R_slow_001__A443__colleague/section_00_peak_f835.png)

![section_01_peak_f1116.png](frames/walk_ff_start_270_R_slow_001__A443__colleague/section_01_peak_f1116.png)

![section_02_peak_f546.png](frames/walk_ff_start_270_R_slow_001__A443__colleague/section_02_peak_f546.png)

![section_03_peak_f297.png](frames/walk_ff_start_270_R_slow_001__A443__colleague/section_03_peak_f297.png)

![section_04_peak_f687.png](frames/walk_ff_start_270_R_slow_001__A443__colleague/section_04_peak_f687.png)

### `neutral_dancecard_object_interact_003__A541.bvh`  (8 sections, sat-score=34)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 2540 | 2561 | 2544 | 0.18 | saturation | 5 | 0.071 | right_shoulder_roll_joint, left_ankle_roll_joint, left_shoulder_roll_joint |
| 1920 | 1936 | 1930 | 0.14 | saturation | 5 | 0.096 | left_shoulder_roll_joint, right_hip_roll_joint, left_ankle_roll_joint |
| 785 | 800 | 785 | 0.13 | saturation | 4 | 0.060 | left_shoulder_roll_joint, right_hip_roll_joint, right_ankle_roll_joint |
| 2523 | 2532 | 2523 | 0.08 | saturation | 4 | 0.085 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_knee_joint |
| 1227 | 1234 | 1234 | 0.07 | saturation | 4 | 0.054 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 2451 | 2457 | 2457 | 0.06 | saturation | 4 | 0.069 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_pitch_joint |
| 2507 | 2512 | 2512 | 0.05 | saturation | 4 | 0.071 | right_shoulder_roll_joint, left_elbow_joint, left_knee_joint |
| 2585 | 2590 | 2585 | 0.05 | saturation | 4 | 0.058 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f2504.png](frames/neutral_dancecard_object_interact_003__A541__colleague/flag_pelvis_z_f2504.png)

![flag_wrist_ang_vel_f773.png](frames/neutral_dancecard_object_interact_003__A541__colleague/flag_wrist_ang_vel_f773.png)

![flag_saturated_dof_f2556.png](frames/neutral_dancecard_object_interact_003__A541__colleague/flag_saturated_dof_f2556.png)

![section_00_peak_f2544.png](frames/neutral_dancecard_object_interact_003__A541__colleague/section_00_peak_f2544.png)

![section_01_peak_f1930.png](frames/neutral_dancecard_object_interact_003__A541__colleague/section_01_peak_f1930.png)

![section_02_peak_f785.png](frames/neutral_dancecard_object_interact_003__A541__colleague/section_02_peak_f785.png)

![section_03_peak_f2523.png](frames/neutral_dancecard_object_interact_003__A541__colleague/section_03_peak_f2523.png)

![section_04_peak_f1234.png](frames/neutral_dancecard_object_interact_003__A541__colleague/section_04_peak_f1234.png)

### `painful_stand_on_turn_walk_ff_360_start_R_001__A461_M.bvh`  (8 sections, sat-score=34)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 744 | 881 | 867 | 1.15 | saturation | 5 | 0.065 | left_hip_roll_joint, left_ankle_pitch_joint, right_shoulder_roll_joint |
| 250 | 261 | 253 | 0.10 | saturation | 5 | 0.068 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_pitch_joint |
| 1091 | 1109 | 1109 | 0.16 | saturation | 4 | 0.058 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_hip_roll_joint |
| 986 | 997 | 990 | 0.10 | saturation | 4 | 0.055 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_hip_roll_joint |
| 181 | 190 | 190 | 0.08 | saturation | 4 | 0.063 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_pitch_joint |
| 1220 | 1226 | 1223 | 0.06 | saturation | 4 | 0.055 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_hip_roll_joint |
| 117 | 121 | 117 | 0.04 | saturation | 4 | 0.061 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_pitch_joint |
| 1020 | 1024 | 1020 | 0.04 | saturation | 4 | 0.057 | right_elbow_joint, left_elbow_joint, right_ankle_pitch_joint |

![flag_pelvis_z_f372.png](frames/painful_stand_on_turn_walk_ff_360_start_R_001__A461_M__colleague/flag_pelvis_z_f372.png)

![flag_wrist_ang_vel_f855.png](frames/painful_stand_on_turn_walk_ff_360_start_R_001__A461_M__colleague/flag_wrist_ang_vel_f855.png)

![flag_saturated_dof_f864.png](frames/painful_stand_on_turn_walk_ff_360_start_R_001__A461_M__colleague/flag_saturated_dof_f864.png)

![section_00_peak_f867.png](frames/painful_stand_on_turn_walk_ff_360_start_R_001__A461_M__colleague/section_00_peak_f867.png)

![section_01_peak_f253.png](frames/painful_stand_on_turn_walk_ff_360_start_R_001__A461_M__colleague/section_01_peak_f253.png)

![section_02_peak_f1109.png](frames/painful_stand_on_turn_walk_ff_360_start_R_001__A461_M__colleague/section_02_peak_f1109.png)

![section_03_peak_f990.png](frames/painful_stand_on_turn_walk_ff_360_start_R_001__A461_M__colleague/section_03_peak_f990.png)

![section_04_peak_f190.png](frames/painful_stand_on_turn_walk_ff_360_start_R_001__A461_M__colleague/section_04_peak_f190.png)

### `walking_random_direction_R_001__A431_M.bvh`  (7 sections, sat-score=28)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 1139 | 1155 | 1143 | 0.14 | saturation | 4 | 0.055 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_roll_joint |
| 1881 | 1894 | 1894 | 0.12 | saturation | 4 | 0.052 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_pitch_joint |
| 1028 | 1039 | 1028 | 0.10 | saturation | 4 | 0.045 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |
| 1442 | 1452 | 1448 | 0.09 | saturation | 4 | 0.057 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_roll_joint |
| 1789 | 1799 | 1799 | 0.09 | saturation | 4 | 0.045 | right_elbow_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 730 | 739 | 732 | 0.08 | saturation | 4 | 0.045 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |
| 757 | 763 | 757 | 0.06 | saturation | 4 | 0.045 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |

![flag_pelvis_z_f1911.png](frames/walking_random_direction_R_001__A431_M__colleague/flag_pelvis_z_f1911.png)

![flag_wrist_ang_vel_f721.png](frames/walking_random_direction_R_001__A431_M__colleague/flag_wrist_ang_vel_f721.png)

![flag_saturated_dof_f1447.png](frames/walking_random_direction_R_001__A431_M__colleague/flag_saturated_dof_f1447.png)

![section_00_peak_f1143.png](frames/walking_random_direction_R_001__A431_M__colleague/section_00_peak_f1143.png)

![section_01_peak_f1894.png](frames/walking_random_direction_R_001__A431_M__colleague/section_01_peak_f1894.png)

![section_02_peak_f1028.png](frames/walking_random_direction_R_001__A431_M__colleague/section_02_peak_f1028.png)

![section_03_peak_f1448.png](frames/walking_random_direction_R_001__A431_M__colleague/section_03_peak_f1448.png)

![section_04_peak_f1799.png](frames/walking_random_direction_R_001__A431_M__colleague/section_04_peak_f1799.png)

### `walk_ff_loop_225_005__A059_M.bvh`  (5 sections, sat-score=28)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 0 | 46 | 36 | 0.39 | saturation | 7 | 0.062 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_hip_roll_joint |
| 150 | 193 | 190 | 0.37 | saturation | 7 | 0.071 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_hip_roll_joint |
| 264 | 358 | 335 | 0.79 | saturation | 6 | 0.070 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_roll_joint |
| 198 | 209 | 198 | 0.10 | saturation | 4 | 0.067 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_roll_joint |
| 51 | 56 | 51 | 0.05 | saturation | 4 | 0.066 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_roll_joint |

![flag_pelvis_z_f144.png](frames/walk_ff_loop_225_005__A059_M__colleague/flag_pelvis_z_f144.png)

![flag_wrist_ang_vel_f307.png](frames/walk_ff_loop_225_005__A059_M__colleague/flag_wrist_ang_vel_f307.png)

![flag_saturated_dof_f190.png](frames/walk_ff_loop_225_005__A059_M__colleague/flag_saturated_dof_f190.png)

![section_00_peak_f36.png](frames/walk_ff_loop_225_005__A059_M__colleague/section_00_peak_f36.png)

![section_01_peak_f190.png](frames/walk_ff_loop_225_005__A059_M__colleague/section_01_peak_f190.png)

![section_02_peak_f335.png](frames/walk_ff_loop_225_005__A059_M__colleague/section_02_peak_f335.png)

![section_03_peak_f198.png](frames/walk_ff_loop_225_005__A059_M__colleague/section_03_peak_f198.png)

![section_04_peak_f51.png](frames/walk_ff_loop_225_005__A059_M__colleague/section_04_peak_f51.png)

### `walk_forward_loop_001__A021.bvh`  (6 sections, sat-score=24)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 1016 | 1027 | 1020 | 0.10 | saturation | 4 | 0.046 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_roll_joint |
| 475 | 484 | 476 | 0.08 | saturation | 4 | 0.062 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_pitch_joint |
| 311 | 318 | 311 | 0.07 | saturation | 4 | 0.046 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_roll_joint |
| 452 | 459 | 459 | 0.07 | saturation | 4 | 0.046 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_roll_joint |
| 863 | 868 | 868 | 0.05 | saturation | 4 | 0.045 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_roll_joint |
| 268 | 272 | 268 | 0.04 | saturation | 4 | 0.064 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_pitch_joint |

![flag_pelvis_z_f819.png](frames/walk_forward_loop_001__A021__colleague/flag_pelvis_z_f819.png)

![flag_wrist_ang_vel_f679.png](frames/walk_forward_loop_001__A021__colleague/flag_wrist_ang_vel_f679.png)

![flag_saturated_dof_f685.png](frames/walk_forward_loop_001__A021__colleague/flag_saturated_dof_f685.png)

![section_00_peak_f1020.png](frames/walk_forward_loop_001__A021__colleague/section_00_peak_f1020.png)

![section_01_peak_f476.png](frames/walk_forward_loop_001__A021__colleague/section_01_peak_f476.png)

![section_02_peak_f311.png](frames/walk_forward_loop_001__A021__colleague/section_02_peak_f311.png)

![section_03_peak_f459.png](frames/walk_forward_loop_001__A021__colleague/section_03_peak_f459.png)

![section_04_peak_f868.png](frames/walk_forward_loop_001__A021__colleague/section_04_peak_f868.png)

### `victory_dance_loser_jump_180_R_003__A308.bvh`  (5 sections, sat-score=22)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 275 | 281 | 276 | 0.06 | saturation | 5 | 0.082 | left_shoulder_roll_joint, left_ankle_roll_joint, right_ankle_pitch_joint |
| 228 | 232 | 230 | 0.04 | saturation | 5 | 0.078 | left_shoulder_roll_joint, left_ankle_roll_joint, right_ankle_roll_joint |
| 0 | 9 | 9 | 0.08 | saturation | 4 | 0.048 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |
| 42 | 47 | 47 | 0.05 | saturation | 4 | 0.077 | left_ankle_roll_joint, right_ankle_roll_joint, right_ankle_pitch_joint |
| 138 | 142 | 142 | 0.04 | saturation | 4 | 0.072 | left_ankle_roll_joint, right_ankle_roll_joint, right_ankle_pitch_joint |

![flag_pelvis_z_f230.png](frames/victory_dance_loser_jump_180_R_003__A308__colleague/flag_pelvis_z_f230.png)

![flag_wrist_ang_vel_f15.png](frames/victory_dance_loser_jump_180_R_003__A308__colleague/flag_wrist_ang_vel_f15.png)

![flag_saturated_dof_f367.png](frames/victory_dance_loser_jump_180_R_003__A308__colleague/flag_saturated_dof_f367.png)

![section_00_peak_f276.png](frames/victory_dance_loser_jump_180_R_003__A308__colleague/section_00_peak_f276.png)

![section_01_peak_f230.png](frames/victory_dance_loser_jump_180_R_003__A308__colleague/section_01_peak_f230.png)

![section_02_peak_f9.png](frames/victory_dance_loser_jump_180_R_003__A308__colleague/section_02_peak_f9.png)

![section_03_peak_f47.png](frames/victory_dance_loser_jump_180_R_003__A308__colleague/section_03_peak_f47.png)

![section_04_peak_f142.png](frames/victory_dance_loser_jump_180_R_003__A308__colleague/section_04_peak_f142.png)

### `dance_hiphop_funky_guitar_R_fast_001__A319.bvh`  (5 sections, sat-score=20)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 593 | 674 | 593 | 0.68 | saturation | 4 | 0.058 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |
| 549 | 565 | 549 | 0.14 | saturation | 4 | 0.051 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |
| 330 | 345 | 342 | 0.13 | saturation | 4 | 0.077 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |
| 573 | 587 | 584 | 0.12 | saturation | 4 | 0.063 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |
| 252 | 261 | 252 | 0.08 | saturation | 4 | 0.079 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |

![flag_pelvis_z_f127.png](frames/dance_hiphop_funky_guitar_R_fast_001__A319__colleague/flag_pelvis_z_f127.png)

![flag_wrist_ang_vel_f276.png](frames/dance_hiphop_funky_guitar_R_fast_001__A319__colleague/flag_wrist_ang_vel_f276.png)

![flag_saturated_dof_f608.png](frames/dance_hiphop_funky_guitar_R_fast_001__A319__colleague/flag_saturated_dof_f608.png)

![section_00_peak_f593.png](frames/dance_hiphop_funky_guitar_R_fast_001__A319__colleague/section_00_peak_f593.png)

![section_01_peak_f549.png](frames/dance_hiphop_funky_guitar_R_fast_001__A319__colleague/section_01_peak_f549.png)

![section_02_peak_f342.png](frames/dance_hiphop_funky_guitar_R_fast_001__A319__colleague/section_02_peak_f342.png)

![section_03_peak_f584.png](frames/dance_hiphop_funky_guitar_R_fast_001__A319__colleague/section_03_peak_f584.png)

![section_04_peak_f252.png](frames/dance_hiphop_funky_guitar_R_fast_001__A319__colleague/section_04_peak_f252.png)

### `dance_western_horse_step_with_leg_undercut_R_loop_002__A324.bvh`  (4 sections, sat-score=17)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 329 | 340 | 335 | 0.10 | saturation | 5 | 0.077 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |
| 812 | 819 | 816 | 0.07 | saturation | 4 | 0.077 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_knee_joint |
| 877 | 881 | 879 | 0.04 | saturation | 4 | 0.066 | right_shoulder_roll_joint, left_hip_roll_joint, right_ankle_roll_joint |
| 940 | 944 | 944 | 0.04 | saturation | 4 | 0.076 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_knee_joint |

![flag_pelvis_z_f454.png](frames/dance_western_horse_step_with_leg_undercut_R_loop_002__A324__colleague/flag_pelvis_z_f454.png)

![flag_wrist_ang_vel_f456.png](frames/dance_western_horse_step_with_leg_undercut_R_loop_002__A324__colleague/flag_wrist_ang_vel_f456.png)

![flag_saturated_dof_f333.png](frames/dance_western_horse_step_with_leg_undercut_R_loop_002__A324__colleague/flag_saturated_dof_f333.png)

![section_00_peak_f335.png](frames/dance_western_horse_step_with_leg_undercut_R_loop_002__A324__colleague/section_00_peak_f335.png)

![section_01_peak_f816.png](frames/dance_western_horse_step_with_leg_undercut_R_loop_002__A324__colleague/section_01_peak_f816.png)

![section_02_peak_f879.png](frames/dance_western_horse_step_with_leg_undercut_R_loop_002__A324__colleague/section_02_peak_f879.png)

![section_03_peak_f944.png](frames/dance_western_horse_step_with_leg_undercut_R_loop_002__A324__colleague/section_03_peak_f944.png)

### `dance_retro_jazz_cross_step_180_R_001__A314.bvh`  (3 sections, sat-score=14)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 547 | 557 | 553 | 0.09 | saturation | 6 | 0.049 | right_shoulder_roll_joint, left_elbow_joint, left_shoulder_roll_joint |
| 391 | 396 | 396 | 0.05 | saturation | 4 | 0.060 | right_elbow_joint, left_elbow_joint, left_shoulder_roll_joint |
| 398 | 403 | 403 | 0.05 | saturation | 4 | 0.062 | right_elbow_joint, left_elbow_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f313.png](frames/dance_retro_jazz_cross_step_180_R_001__A314__colleague/flag_pelvis_z_f313.png)

![flag_wrist_ang_vel_f603.png](frames/dance_retro_jazz_cross_step_180_R_001__A314__colleague/flag_wrist_ang_vel_f603.png)

![flag_saturated_dof_f553.png](frames/dance_retro_jazz_cross_step_180_R_001__A314__colleague/flag_saturated_dof_f553.png)

![section_00_peak_f553.png](frames/dance_retro_jazz_cross_step_180_R_001__A314__colleague/section_00_peak_f553.png)

![section_01_peak_f396.png](frames/dance_retro_jazz_cross_step_180_R_001__A314__colleague/section_01_peak_f396.png)

![section_02_peak_f403.png](frames/dance_retro_jazz_cross_step_180_R_001__A314__colleague/section_02_peak_f403.png)

### `dance_vouge_butterfly_step_180_R_fast_002__A319.bvh`  (3 sections, sat-score=14)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 479 | 487 | 482 | 0.07 | saturation | 5 | 0.060 | left_shoulder_roll_joint, left_elbow_joint, left_ankle_roll_joint |
| 201 | 208 | 203 | 0.07 | saturation | 5 | 0.085 | left_shoulder_roll_joint, left_ankle_pitch_joint, left_ankle_roll_joint |
| 508 | 541 | 508 | 0.28 | saturation | 4 | 0.046 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |

![flag_pelvis_z_f190.png](frames/dance_vouge_butterfly_step_180_R_fast_002__A319__colleague/flag_pelvis_z_f190.png)

![flag_wrist_ang_vel_f367.png](frames/dance_vouge_butterfly_step_180_R_fast_002__A319__colleague/flag_wrist_ang_vel_f367.png)

![flag_saturated_dof_f484.png](frames/dance_vouge_butterfly_step_180_R_fast_002__A319__colleague/flag_saturated_dof_f484.png)

![section_00_peak_f482.png](frames/dance_vouge_butterfly_step_180_R_fast_002__A319__colleague/section_00_peak_f482.png)

![section_01_peak_f203.png](frames/dance_vouge_butterfly_step_180_R_fast_002__A319__colleague/section_01_peak_f203.png)

![section_02_peak_f508.png](frames/dance_vouge_butterfly_step_180_R_fast_002__A319__colleague/section_02_peak_f508.png)

### `Loop_Backward_Walk_001__A020.bvh`  (3 sections, sat-score=12)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 679 | 695 | 695 | 0.14 | saturation | 4 | 0.055 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_roll_joint |
| 489 | 494 | 490 | 0.05 | saturation | 4 | 0.068 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_pitch_joint |
| 652 | 657 | 652 | 0.05 | saturation | 4 | 0.063 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_pitch_joint |

![flag_pelvis_z_f629.png](frames/Loop_Backward_Walk_001__A020__colleague/flag_pelvis_z_f629.png)

![flag_wrist_ang_vel_f714.png](frames/Loop_Backward_Walk_001__A020__colleague/flag_wrist_ang_vel_f714.png)

![flag_saturated_dof_f680.png](frames/Loop_Backward_Walk_001__A020__colleague/flag_saturated_dof_f680.png)

![section_00_peak_f695.png](frames/Loop_Backward_Walk_001__A020__colleague/section_00_peak_f695.png)

![section_01_peak_f490.png](frames/Loop_Backward_Walk_001__A020__colleague/section_01_peak_f490.png)

![section_02_peak_f652.png](frames/Loop_Backward_Walk_001__A020__colleague/section_02_peak_f652.png)

### `dance_retro_twist_step_variation_R_fast_002__A314.bvh`  (3 sections, sat-score=12)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 727 | 735 | 732 | 0.07 | saturation | 4 | 0.043 | right_elbow_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 549 | 556 | 553 | 0.07 | saturation | 4 | 0.073 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |
| 236 | 241 | 240 | 0.05 | saturation | 4 | 0.072 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_hip_roll_joint |

![flag_pelvis_z_f377.png](frames/dance_retro_twist_step_variation_R_fast_002__A314__colleague/flag_pelvis_z_f377.png)

![flag_wrist_ang_vel_f707.png](frames/dance_retro_twist_step_variation_R_fast_002__A314__colleague/flag_wrist_ang_vel_f707.png)

![flag_saturated_dof_f710.png](frames/dance_retro_twist_step_variation_R_fast_002__A314__colleague/flag_saturated_dof_f710.png)

![section_00_peak_f732.png](frames/dance_retro_twist_step_variation_R_fast_002__A314__colleague/section_00_peak_f732.png)

![section_01_peak_f553.png](frames/dance_retro_twist_step_variation_R_fast_002__A314__colleague/section_01_peak_f553.png)

![section_02_peak_f240.png](frames/dance_retro_twist_step_variation_R_fast_002__A314__colleague/section_02_peak_f240.png)

### `big_light_two_hands_right_side_high_to_behind_high_R_001__A525.bvh`  (3 sections, sat-score=12)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 619 | 670 | 619 | 0.43 | saturation | 4 | 0.074 | left_elbow_joint, left_hip_roll_joint, left_ankle_roll_joint |
| 674 | 697 | 674 | 0.20 | saturation | 4 | 0.072 | left_elbow_joint, left_ankle_pitch_joint, right_ankle_roll_joint |
| 593 | 604 | 593 | 0.10 | saturation | 4 | 0.075 | left_elbow_joint, left_hip_roll_joint, left_ankle_roll_joint |

![flag_pelvis_z_f808.png](frames/big_light_two_hands_right_side_high_to_behind_high_R_001__A525__colleague/flag_pelvis_z_f808.png)

![flag_wrist_ang_vel_f796.png](frames/big_light_two_hands_right_side_high_to_behind_high_R_001__A525__colleague/flag_wrist_ang_vel_f796.png)

![flag_saturated_dof_f597.png](frames/big_light_two_hands_right_side_high_to_behind_high_R_001__A525__colleague/flag_saturated_dof_f597.png)

![section_00_peak_f619.png](frames/big_light_two_hands_right_side_high_to_behind_high_R_001__A525__colleague/section_00_peak_f619.png)

![section_01_peak_f674.png](frames/big_light_two_hands_right_side_high_to_behind_high_R_001__A525__colleague/section_01_peak_f674.png)

![section_02_peak_f593.png](frames/big_light_two_hands_right_side_high_to_behind_high_R_001__A525__colleague/section_02_peak_f593.png)

### `body_check_001__A461.bvh`  (2 sections, sat-score=9)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 1766 | 1859 | 1773 | 0.78 | saturation | 5 | 0.045 | right_elbow_joint, left_elbow_joint, left_shoulder_roll_joint |
| 2925 | 2940 | 2940 | 0.13 | saturation | 4 | 0.112 | right_elbow_joint, left_shoulder_roll_joint, right_ankle_roll_joint |

![flag_pelvis_z_f2921.png](frames/body_check_001__A461__colleague/flag_pelvis_z_f2921.png)

![flag_wrist_ang_vel_f1043.png](frames/body_check_001__A461__colleague/flag_wrist_ang_vel_f1043.png)

![flag_saturated_dof_f1810.png](frames/body_check_001__A461__colleague/flag_saturated_dof_f1810.png)

![section_00_peak_f1773.png](frames/body_check_001__A461__colleague/section_00_peak_f1773.png)

![section_01_peak_f2940.png](frames/body_check_001__A461__colleague/section_01_peak_f2940.png)

### `turn_start_walk_135_004__A038.bvh`  (2 sections, sat-score=9)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 429 | 444 | 432 | 0.13 | saturation | 5 | 0.078 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |
| 280 | 287 | 280 | 0.07 | saturation | 4 | 0.074 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |

![flag_pelvis_z_f596.png](frames/turn_start_walk_135_004__A038__colleague/flag_pelvis_z_f596.png)

![flag_wrist_ang_vel_f350.png](frames/turn_start_walk_135_004__A038__colleague/flag_wrist_ang_vel_f350.png)

![flag_saturated_dof_f432.png](frames/turn_start_walk_135_004__A038__colleague/flag_saturated_dof_f432.png)

![section_00_peak_f432.png](frames/turn_start_walk_135_004__A038__colleague/section_00_peak_f432.png)

![section_01_peak_f280.png](frames/turn_start_walk_135_004__A038__colleague/section_01_peak_f280.png)

### `walk_ff_start_360_R_003__A267.bvh`  (2 sections, sat-score=9)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 681 | 690 | 690 | 0.08 | saturation | 5 | 0.047 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_roll_joint |
| 643 | 654 | 643 | 0.10 | saturation | 4 | 0.061 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |

![flag_pelvis_z_f764.png](frames/walk_ff_start_360_R_003__A267__colleague/flag_pelvis_z_f764.png)

![flag_wrist_ang_vel_f591.png](frames/walk_ff_start_360_R_003__A267__colleague/flag_wrist_ang_vel_f591.png)

![flag_saturated_dof_f687.png](frames/walk_ff_start_360_R_003__A267__colleague/flag_saturated_dof_f687.png)

![section_00_peak_f690.png](frames/walk_ff_start_360_R_003__A267__colleague/section_00_peak_f690.png)

![section_01_peak_f643.png](frames/walk_ff_start_360_R_003__A267__colleague/section_01_peak_f643.png)

### `body_check_002__A492_M.bvh`  (2 sections, sat-score=9)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 2832 | 2963 | 2850 | 1.10 | saturation | 5 | 0.140 | left_shoulder_roll_joint, left_hip_roll_joint, right_ankle_roll_joint |
| 18 | 23 | 18 | 0.05 | saturation | 4 | 0.042 | right_elbow_joint, left_elbow_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f2855.png](frames/body_check_002__A492_M__colleague/flag_pelvis_z_f2855.png)

![flag_wrist_ang_vel_f2854.png](frames/body_check_002__A492_M__colleague/flag_wrist_ang_vel_f2854.png)

![flag_saturated_dof_f2848.png](frames/body_check_002__A492_M__colleague/flag_saturated_dof_f2848.png)

![section_00_peak_f2850.png](frames/body_check_002__A492_M__colleague/section_00_peak_f2850.png)

![section_01_peak_f18.png](frames/body_check_002__A492_M__colleague/section_01_peak_f18.png)

### `medium_big_heavy_one_hand_walk_ff_start_360_R_001__A506_M.bvh`  (2 sections, sat-score=8)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 856 | 869 | 869 | 0.12 | saturation | 4 | 0.056 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_roll_joint |
| 134 | 138 | 138 | 0.04 | saturation | 4 | 0.044 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f180.png](frames/medium_big_heavy_one_hand_walk_ff_start_360_R_001__A506_M__colleague/flag_pelvis_z_f180.png)

![flag_wrist_ang_vel_f657.png](frames/medium_big_heavy_one_hand_walk_ff_start_360_R_001__A506_M__colleague/flag_wrist_ang_vel_f657.png)

![flag_saturated_dof_f869.png](frames/medium_big_heavy_one_hand_walk_ff_start_360_R_001__A506_M__colleague/flag_saturated_dof_f869.png)

![section_00_peak_f869.png](frames/medium_big_heavy_one_hand_walk_ff_start_360_R_001__A506_M__colleague/section_00_peak_f869.png)

![section_01_peak_f138.png](frames/medium_big_heavy_one_hand_walk_ff_start_360_R_001__A506_M__colleague/section_01_peak_f138.png)

### `dance_basic_slide_360_R_001__A306.bvh`  (2 sections, sat-score=8)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 137 | 150 | 150 | 0.12 | saturation | 4 | 0.049 | left_shoulder_roll_joint, left_elbow_joint, left_ankle_roll_joint |
| 126 | 130 | 130 | 0.04 | saturation | 4 | 0.046 | left_shoulder_roll_joint, left_elbow_joint, left_ankle_roll_joint |

![flag_pelvis_z_f91.png](frames/dance_basic_slide_360_R_001__A306__colleague/flag_pelvis_z_f91.png)

![flag_wrist_ang_vel_f221.png](frames/dance_basic_slide_360_R_001__A306__colleague/flag_wrist_ang_vel_f221.png)

![flag_saturated_dof_f137.png](frames/dance_basic_slide_360_R_001__A306__colleague/flag_saturated_dof_f137.png)

![section_00_peak_f150.png](frames/dance_basic_slide_360_R_001__A306__colleague/section_00_peak_f150.png)

![section_01_peak_f130.png](frames/dance_basic_slide_360_R_001__A306__colleague/section_01_peak_f130.png)

### `painful_stand_on_walk_ff_360_R_001__A461.bvh`  (2 sections, sat-score=8)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 564 | 583 | 564 | 0.17 | saturation | 4 | 0.058 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_pitch_joint |
| 552 | 557 | 555 | 0.05 | saturation | 4 | 0.056 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_pitch_joint |

![flag_pelvis_z_f240.png](frames/painful_stand_on_walk_ff_360_R_001__A461__colleague/flag_pelvis_z_f240.png)

![flag_wrist_ang_vel_f705.png](frames/painful_stand_on_walk_ff_360_R_001__A461__colleague/flag_wrist_ang_vel_f705.png)

![flag_saturated_dof_f573.png](frames/painful_stand_on_walk_ff_360_R_001__A461__colleague/flag_saturated_dof_f573.png)

![section_00_peak_f564.png](frames/painful_stand_on_walk_ff_360_R_001__A461__colleague/section_00_peak_f564.png)

![section_01_peak_f555.png](frames/painful_stand_on_walk_ff_360_R_001__A461__colleague/section_01_peak_f555.png)

### `small_light_two_hands_front_low_to_behind_low_R_001__A517_M.bvh`  (1 sections, sat-score=7)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 185 | 350 | 287 | 1.38 | saturation | 7 | 0.171 | right_shoulder_roll_joint, left_hip_roll_joint, left_ankle_roll_joint |

![flag_pelvis_z_f335.png](frames/small_light_two_hands_front_low_to_behind_low_R_001__A517_M__colleague/flag_pelvis_z_f335.png)

![flag_wrist_ang_vel_f254.png](frames/small_light_two_hands_front_low_to_behind_low_R_001__A517_M__colleague/flag_wrist_ang_vel_f254.png)

![flag_saturated_dof_f257.png](frames/small_light_two_hands_front_low_to_behind_low_R_001__A517_M__colleague/flag_saturated_dof_f257.png)

![section_00_peak_f287.png](frames/small_light_two_hands_front_low_to_behind_low_R_001__A517_M__colleague/section_00_peak_f287.png)

### `walk_ff_start_360_004__A146.bvh`  (1 sections, sat-score=4)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 278 | 282 | 278 | 0.04 | saturation | 4 | 0.062 | right_elbow_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f600.png](frames/walk_ff_start_360_004__A146__colleague/flag_pelvis_z_f600.png)

![flag_wrist_ang_vel_f617.png](frames/walk_ff_start_360_004__A146__colleague/flag_wrist_ang_vel_f617.png)

![flag_saturated_dof_f279.png](frames/walk_ff_start_360_004__A146__colleague/flag_saturated_dof_f279.png)

![section_00_peak_f278.png](frames/walk_ff_start_360_004__A146__colleague/section_00_peak_f278.png)

### `body_check_001__A251.bvh`  (1 sections, sat-score=4)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 1455 | 1565 | 1531 | 0.93 | saturation | 4 | 0.042 | right_elbow_joint, left_elbow_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f2670.png](frames/body_check_001__A251__colleague/flag_pelvis_z_f2670.png)

![flag_wrist_ang_vel_f491.png](frames/body_check_001__A251__colleague/flag_wrist_ang_vel_f491.png)

![flag_saturated_dof_f1512.png](frames/body_check_001__A251__colleague/flag_saturated_dof_f1512.png)

![section_00_peak_f1531.png](frames/body_check_001__A251__colleague/section_00_peak_f1531.png)

### `body_check_002__A497.bvh`  (1 sections, sat-score=4)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 2971 | 2978 | 2978 | 0.07 | saturation | 4 | 0.088 | right_elbow_joint, left_elbow_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f2392.png](frames/body_check_002__A497__colleague/flag_pelvis_z_f2392.png)

![flag_wrist_ang_vel_f2014.png](frames/body_check_002__A497__colleague/flag_wrist_ang_vel_f2014.png)

![flag_saturated_dof_f2978.png](frames/body_check_002__A497__colleague/flag_saturated_dof_f2978.png)

![section_00_peak_f2978.png](frames/body_check_002__A497__colleague/section_00_peak_f2978.png)

### `body_check_001__A527_M.bvh`  (1 sections, sat-score=4)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 2050 | 2285 | 2250 | 1.97 | saturation | 4 | 0.045 | right_elbow_joint, left_elbow_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f2255.png](frames/body_check_001__A527_M__colleague/flag_pelvis_z_f2255.png)

![flag_wrist_ang_vel_f2965.png](frames/body_check_001__A527_M__colleague/flag_wrist_ang_vel_f2965.png)

![flag_saturated_dof_f2126.png](frames/body_check_001__A527_M__colleague/flag_saturated_dof_f2126.png)

![section_00_peak_f2250.png](frames/body_check_001__A527_M__colleague/section_00_peak_f2250.png)
## Config `v5_ours`

### `walk_forward_relax_003__A005.bvh`  (20 sections, sat-score=90)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 1666 | 1712 | 1682 | 0.39 | saturation | 6 | 0.097 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 1873 | 1896 | 1885 | 0.20 | saturation | 6 | 0.103 | right_elbow_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 1591 | 1624 | 1612 | 0.28 | saturation | 5 | 0.095 | right_wrist_pitch_joint, left_elbow_joint, right_shoulder_roll_joint |
| 1070 | 1100 | 1080 | 0.26 | saturation | 5 | 0.100 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_elbow_joint |
| 2884 | 2913 | 2913 | 0.25 | saturation | 5 | 0.098 | right_wrist_pitch_joint, left_elbow_joint, right_shoulder_roll_joint |
| 2957 | 2983 | 2968 | 0.23 | saturation | 5 | 0.098 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_elbow_joint |
| 2060 | 2070 | 2067 | 0.09 | saturation | 5 | 0.092 | right_elbow_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 1905 | 1914 | 1908 | 0.08 | saturation | 5 | 0.097 | right_elbow_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 1026 | 1047 | 1047 | 0.18 | saturation | 4 | 0.104 | right_wrist_pitch_joint, left_elbow_joint, right_shoulder_roll_joint |
| 1350 | 1369 | 1352 | 0.17 | saturation | 4 | 0.099 | right_elbow_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 140 | 156 | 156 | 0.14 | saturation | 4 | 0.100 | right_elbow_joint, left_elbow_joint, right_shoulder_roll_joint |
| 2041 | 2055 | 2041 | 0.12 | saturation | 4 | 0.101 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 2189 | 2202 | 2189 | 0.12 | saturation | 4 | 0.103 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 1644 | 1656 | 1648 | 0.11 | saturation | 4 | 0.101 | right_elbow_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 2290 | 2302 | 2302 | 0.11 | saturation | 4 | 0.095 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 2744 | 2755 | 2755 | 0.10 | saturation | 4 | 0.093 | right_wrist_pitch_joint, left_elbow_joint, right_shoulder_roll_joint |
| 2641 | 2650 | 2641 | 0.08 | saturation | 4 | 0.102 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 2278 | 2284 | 2278 | 0.06 | saturation | 4 | 0.098 | left_shoulder_roll_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 2495 | 2501 | 2495 | 0.06 | saturation | 4 | 0.103 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 933 | 938 | 933 | 0.05 | saturation | 4 | 0.100 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f2761.png](frames/walk_forward_relax_003__A005__v5_ours/flag_pelvis_z_f2761.png)

![flag_wrist_ang_vel_f1206.png](frames/walk_forward_relax_003__A005__v5_ours/flag_wrist_ang_vel_f1206.png)

![flag_saturated_dof_f1683.png](frames/walk_forward_relax_003__A005__v5_ours/flag_saturated_dof_f1683.png)

![section_00_peak_f1682.png](frames/walk_forward_relax_003__A005__v5_ours/section_00_peak_f1682.png)

![section_01_peak_f1885.png](frames/walk_forward_relax_003__A005__v5_ours/section_01_peak_f1885.png)

![section_02_peak_f1612.png](frames/walk_forward_relax_003__A005__v5_ours/section_02_peak_f1612.png)

![section_03_peak_f1080.png](frames/walk_forward_relax_003__A005__v5_ours/section_03_peak_f1080.png)

![section_04_peak_f2913.png](frames/walk_forward_relax_003__A005__v5_ours/section_04_peak_f2913.png)

### `walking_random_direction_R_001__A431_M.bvh`  (18 sections, sat-score=80)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 1046 | 1159 | 1078 | 0.95 | saturation | 7 | 0.083 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_hip_roll_joint |
| 2772 | 2827 | 2785 | 0.47 | saturation | 5 | 0.087 | right_elbow_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 1332 | 1379 | 1343 | 0.40 | saturation | 5 | 0.088 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 717 | 757 | 743 | 0.34 | saturation | 5 | 0.087 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_hip_roll_joint |
| 2257 | 2291 | 2289 | 0.29 | saturation | 5 | 0.084 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_hip_roll_joint |
| 2315 | 2336 | 2331 | 0.18 | saturation | 5 | 0.089 | right_elbow_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 1495 | 1521 | 1495 | 0.23 | saturation | 4 | 0.087 | right_elbow_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 1784 | 1810 | 1810 | 0.23 | saturation | 4 | 0.088 | right_elbow_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 2672 | 2697 | 2692 | 0.22 | saturation | 4 | 0.091 | right_elbow_joint, left_elbow_joint, right_shoulder_roll_joint |
| 1532 | 1550 | 1550 | 0.16 | saturation | 4 | 0.086 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_roll_joint |
| 2241 | 2254 | 2241 | 0.12 | saturation | 4 | 0.085 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 1591 | 1603 | 1600 | 0.11 | saturation | 4 | 0.089 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |
| 1605 | 1617 | 1605 | 0.11 | saturation | 4 | 0.088 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 1439 | 1449 | 1448 | 0.09 | saturation | 4 | 0.092 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_hip_roll_joint |
| 1826 | 1834 | 1834 | 0.07 | saturation | 4 | 0.091 | right_elbow_joint, left_elbow_joint, right_shoulder_roll_joint |
| 2441 | 2448 | 2446 | 0.07 | saturation | 4 | 0.090 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 974 | 980 | 974 | 0.06 | saturation | 4 | 0.089 | right_elbow_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 1030 | 1036 | 1030 | 0.06 | saturation | 4 | 0.087 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |

![flag_pelvis_z_f1927.png](frames/walking_random_direction_R_001__A431_M__v5_ours/flag_pelvis_z_f1927.png)

![flag_wrist_ang_vel_f945.png](frames/walking_random_direction_R_001__A431_M__v5_ours/flag_wrist_ang_vel_f945.png)

![flag_saturated_dof_f1072.png](frames/walking_random_direction_R_001__A431_M__v5_ours/flag_saturated_dof_f1072.png)

![section_00_peak_f1078.png](frames/walking_random_direction_R_001__A431_M__v5_ours/section_00_peak_f1078.png)

![section_01_peak_f2785.png](frames/walking_random_direction_R_001__A431_M__v5_ours/section_01_peak_f2785.png)

![section_02_peak_f1343.png](frames/walking_random_direction_R_001__A431_M__v5_ours/section_02_peak_f1343.png)

![section_03_peak_f743.png](frames/walking_random_direction_R_001__A431_M__v5_ours/section_03_peak_f743.png)

![section_04_peak_f2289.png](frames/walking_random_direction_R_001__A431_M__v5_ours/section_04_peak_f2289.png)

### `dance_retro_twist_step_variation_R_fast_002__A314.bvh`  (14 sections, sat-score=76)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 75 | 211 | 101 | 1.14 | saturation | 7 | 0.088 | right_shoulder_roll_joint, right_wrist_roll_joint, left_wrist_roll_joint |
| 589 | 624 | 605 | 0.30 | saturation | 7 | 0.084 | right_wrist_roll_joint, left_wrist_roll_joint, right_shoulder_roll_joint |
| 528 | 565 | 554 | 0.32 | saturation | 6 | 0.080 | right_wrist_roll_joint, right_shoulder_roll_joint, left_elbow_joint |
| 224 | 248 | 239 | 0.21 | saturation | 6 | 0.082 | right_wrist_roll_joint, left_wrist_roll_joint, right_shoulder_roll_joint |
| 329 | 352 | 335 | 0.20 | saturation | 6 | 0.082 | right_wrist_roll_joint, left_wrist_roll_joint, right_shoulder_roll_joint |
| 387 | 409 | 398 | 0.19 | saturation | 6 | 0.083 | right_shoulder_roll_joint, left_wrist_roll_joint, right_elbow_joint |
| 503 | 525 | 508 | 0.19 | saturation | 6 | 0.085 | right_wrist_roll_joint, left_wrist_roll_joint, right_shoulder_roll_joint |
| 295 | 308 | 302 | 0.12 | saturation | 6 | 0.083 | right_shoulder_roll_joint, left_wrist_roll_joint, left_ankle_roll_joint |
| 648 | 711 | 701 | 0.53 | saturation | 5 | 0.092 | left_shoulder_roll_joint, right_shoulder_roll_joint, left_wrist_roll_joint |
| 250 | 265 | 250 | 0.13 | saturation | 5 | 0.079 | right_wrist_roll_joint, left_wrist_roll_joint, left_wrist_pitch_joint |
| 286 | 293 | 291 | 0.07 | saturation | 4 | 0.086 | right_elbow_joint, left_wrist_roll_joint, right_shoulder_roll_joint |
| 439 | 446 | 444 | 0.07 | saturation | 4 | 0.082 | right_wrist_roll_joint, left_wrist_roll_joint, right_shoulder_roll_joint |
| 52 | 56 | 52 | 0.04 | saturation | 4 | 0.084 | right_wrist_roll_joint, left_wrist_roll_joint, right_shoulder_roll_joint |
| 371 | 375 | 375 | 0.04 | saturation | 4 | 0.078 | right_wrist_roll_joint, left_wrist_roll_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f695.png](frames/dance_retro_twist_step_variation_R_fast_002__A314__v5_ours/flag_pelvis_z_f695.png)

![flag_wrist_ang_vel_f503.png](frames/dance_retro_twist_step_variation_R_fast_002__A314__v5_ours/flag_wrist_ang_vel_f503.png)

![flag_saturated_dof_f102.png](frames/dance_retro_twist_step_variation_R_fast_002__A314__v5_ours/flag_saturated_dof_f102.png)

![section_00_peak_f101.png](frames/dance_retro_twist_step_variation_R_fast_002__A314__v5_ours/section_00_peak_f101.png)

![section_01_peak_f605.png](frames/dance_retro_twist_step_variation_R_fast_002__A314__v5_ours/section_01_peak_f605.png)

![section_02_peak_f554.png](frames/dance_retro_twist_step_variation_R_fast_002__A314__v5_ours/section_02_peak_f554.png)

![section_03_peak_f239.png](frames/dance_retro_twist_step_variation_R_fast_002__A314__v5_ours/section_03_peak_f239.png)

![section_04_peak_f335.png](frames/dance_retro_twist_step_variation_R_fast_002__A314__v5_ours/section_04_peak_f335.png)

### `dance_hiphop_funky_guitar_R_fast_001__A319.bvh`  (15 sections, sat-score=72)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 51 | 159 | 130 | 0.91 | saturation | 6 | 0.087 | left_shoulder_roll_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 589 | 674 | 650 | 0.72 | saturation | 6 | 0.091 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |
| 495 | 564 | 556 | 0.58 | saturation | 6 | 0.093 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_wrist_pitch_joint |
| 351 | 374 | 369 | 0.20 | saturation | 6 | 0.086 | left_shoulder_roll_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 170 | 217 | 192 | 0.40 | saturation | 5 | 0.086 | left_shoulder_roll_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 414 | 441 | 431 | 0.23 | saturation | 5 | 0.088 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 246 | 264 | 256 | 0.16 | saturation | 5 | 0.085 | left_shoulder_roll_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 224 | 240 | 239 | 0.14 | saturation | 5 | 0.084 | left_shoulder_roll_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 299 | 319 | 311 | 0.17 | saturation | 4 | 0.088 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 381 | 401 | 381 | 0.17 | saturation | 4 | 0.087 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 271 | 288 | 288 | 0.15 | saturation | 4 | 0.081 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 471 | 488 | 488 | 0.15 | saturation | 4 | 0.087 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 329 | 342 | 342 | 0.12 | saturation | 4 | 0.083 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 451 | 462 | 461 | 0.10 | saturation | 4 | 0.084 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 32 | 38 | 32 | 0.06 | saturation | 4 | 0.091 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f231.png](frames/dance_hiphop_funky_guitar_R_fast_001__A319__v5_ours/flag_pelvis_z_f231.png)

![flag_wrist_ang_vel_f238.png](frames/dance_hiphop_funky_guitar_R_fast_001__A319__v5_ours/flag_wrist_ang_vel_f238.png)

![flag_saturated_dof_f644.png](frames/dance_hiphop_funky_guitar_R_fast_001__A319__v5_ours/flag_saturated_dof_f644.png)

![section_00_peak_f130.png](frames/dance_hiphop_funky_guitar_R_fast_001__A319__v5_ours/section_00_peak_f130.png)

![section_01_peak_f650.png](frames/dance_hiphop_funky_guitar_R_fast_001__A319__v5_ours/section_01_peak_f650.png)

![section_02_peak_f556.png](frames/dance_hiphop_funky_guitar_R_fast_001__A319__v5_ours/section_02_peak_f556.png)

![section_03_peak_f369.png](frames/dance_hiphop_funky_guitar_R_fast_001__A319__v5_ours/section_03_peak_f369.png)

![section_04_peak_f192.png](frames/dance_hiphop_funky_guitar_R_fast_001__A319__v5_ours/section_04_peak_f192.png)

### `dance_latino_chase_mambo_pivot_R_001__A313.bvh`  (11 sections, sat-score=70)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 627 | 725 | 690 | 0.82 | saturation | 9 | 0.083 | right_wrist_roll_joint, right_wrist_pitch_joint, left_shoulder_roll_joint |
| 171 | 246 | 200 | 0.63 | saturation | 8 | 0.083 | right_wrist_roll_joint, right_wrist_pitch_joint, left_shoulder_roll_joint |
| 870 | 1055 | 977 | 1.55 | saturation | 7 | 0.086 | right_wrist_roll_joint, right_shoulder_roll_joint, right_wrist_pitch_joint |
| 468 | 613 | 490 | 1.22 | saturation | 7 | 0.089 | right_wrist_roll_joint, right_wrist_pitch_joint, right_shoulder_roll_joint |
| 375 | 458 | 396 | 0.70 | saturation | 7 | 0.083 | right_wrist_roll_joint, right_wrist_pitch_joint, right_ankle_roll_joint |
| 20 | 127 | 100 | 0.90 | saturation | 6 | 0.086 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 729 | 827 | 778 | 0.82 | saturation | 6 | 0.084 | right_wrist_roll_joint, right_wrist_pitch_joint, left_shoulder_roll_joint |
| 256 | 316 | 295 | 0.51 | saturation | 6 | 0.086 | right_wrist_roll_joint, right_wrist_pitch_joint, left_shoulder_roll_joint |
| 324 | 368 | 338 | 0.38 | saturation | 6 | 0.085 | right_wrist_roll_joint, right_wrist_pitch_joint, left_elbow_joint |
| 146 | 153 | 146 | 0.07 | saturation | 4 | 0.084 | right_wrist_roll_joint, right_wrist_pitch_joint, right_shoulder_roll_joint |
| 860 | 864 | 861 | 0.04 | saturation | 4 | 0.084 | right_wrist_roll_joint, right_wrist_pitch_joint, right_ankle_roll_joint |

![flag_pelvis_z_f333.png](frames/dance_latino_chase_mambo_pivot_R_001__A313__v5_ours/flag_pelvis_z_f333.png)

![flag_wrist_ang_vel_f143.png](frames/dance_latino_chase_mambo_pivot_R_001__A313__v5_ours/flag_wrist_ang_vel_f143.png)

![flag_saturated_dof_f690.png](frames/dance_latino_chase_mambo_pivot_R_001__A313__v5_ours/flag_saturated_dof_f690.png)

![section_00_peak_f690.png](frames/dance_latino_chase_mambo_pivot_R_001__A313__v5_ours/section_00_peak_f690.png)

![section_01_peak_f200.png](frames/dance_latino_chase_mambo_pivot_R_001__A313__v5_ours/section_01_peak_f200.png)

![section_02_peak_f977.png](frames/dance_latino_chase_mambo_pivot_R_001__A313__v5_ours/section_02_peak_f977.png)

![section_03_peak_f490.png](frames/dance_latino_chase_mambo_pivot_R_001__A313__v5_ours/section_03_peak_f490.png)

![section_04_peak_f396.png](frames/dance_latino_chase_mambo_pivot_R_001__A313__v5_ours/section_04_peak_f396.png)

### `neutral_dancecard_object_interact_003__A541.bvh`  (15 sections, sat-score=70)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 1276 | 1349 | 1278 | 0.62 | saturation | 5 | 0.087 | right_wrist_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |
| 2509 | 2551 | 2528 | 0.36 | saturation | 5 | 0.095 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 2562 | 2592 | 2565 | 0.26 | saturation | 5 | 0.093 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 2476 | 2500 | 2489 | 0.21 | saturation | 5 | 0.091 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 1913 | 1932 | 1922 | 0.17 | saturation | 5 | 0.095 | left_shoulder_roll_joint, left_ankle_roll_joint, right_ankle_roll_joint |
| 1223 | 1241 | 1226 | 0.16 | saturation | 5 | 0.089 | right_wrist_roll_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 754 | 766 | 760 | 0.11 | saturation | 5 | 0.085 | right_wrist_roll_joint, left_shoulder_roll_joint, left_wrist_roll_joint |
| 1264 | 1273 | 1265 | 0.08 | saturation | 5 | 0.084 | right_wrist_roll_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 737 | 744 | 738 | 0.07 | saturation | 5 | 0.086 | right_wrist_roll_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 1249 | 1256 | 1252 | 0.07 | saturation | 5 | 0.076 | right_wrist_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 977 | 1001 | 1001 | 0.21 | saturation | 4 | 0.095 | left_shoulder_roll_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 910 | 925 | 925 | 0.13 | saturation | 4 | 0.093 | right_elbow_joint, left_elbow_joint, right_shoulder_roll_joint |
| 1807 | 1815 | 1811 | 0.07 | saturation | 4 | 0.101 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 789 | 795 | 789 | 0.06 | saturation | 4 | 0.082 | right_elbow_joint, left_shoulder_roll_joint, left_wrist_pitch_joint |
| 1766 | 1770 | 1766 | 0.04 | saturation | 4 | 0.101 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f1900.png](frames/neutral_dancecard_object_interact_003__A541__v5_ours/flag_pelvis_z_f1900.png)

![flag_wrist_ang_vel_f774.png](frames/neutral_dancecard_object_interact_003__A541__v5_ours/flag_wrist_ang_vel_f774.png)

![flag_saturated_dof_f1918.png](frames/neutral_dancecard_object_interact_003__A541__v5_ours/flag_saturated_dof_f1918.png)

![section_00_peak_f1278.png](frames/neutral_dancecard_object_interact_003__A541__v5_ours/section_00_peak_f1278.png)

![section_01_peak_f2528.png](frames/neutral_dancecard_object_interact_003__A541__v5_ours/section_01_peak_f2528.png)

![section_02_peak_f2565.png](frames/neutral_dancecard_object_interact_003__A541__v5_ours/section_02_peak_f2565.png)

![section_03_peak_f2489.png](frames/neutral_dancecard_object_interact_003__A541__v5_ours/section_03_peak_f2489.png)

![section_04_peak_f1922.png](frames/neutral_dancecard_object_interact_003__A541__v5_ours/section_04_peak_f1922.png)

### `dance_western_horse_step_with_leg_undercut_R_loop_002__A324.bvh`  (14 sections, sat-score=68)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 317 | 349 | 331 | 0.28 | saturation | 7 | 0.101 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |
| 1008 | 1041 | 1016 | 0.28 | saturation | 6 | 0.091 | left_shoulder_roll_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 443 | 465 | 453 | 0.19 | saturation | 6 | 0.098 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |
| 76 | 128 | 94 | 0.44 | saturation | 5 | 0.093 | right_wrist_pitch_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 136 | 165 | 151 | 0.25 | saturation | 5 | 0.092 | left_shoulder_roll_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 929 | 946 | 940 | 0.15 | saturation | 5 | 0.102 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |
| 809 | 823 | 814 | 0.12 | saturation | 5 | 0.102 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |
| 531 | 542 | 533 | 0.10 | saturation | 5 | 0.091 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |
| 253 | 289 | 273 | 0.31 | saturation | 4 | 0.091 | left_shoulder_roll_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 195 | 223 | 213 | 0.24 | saturation | 4 | 0.092 | left_shoulder_roll_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 503 | 514 | 504 | 0.10 | saturation | 4 | 0.090 | right_wrist_pitch_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 411 | 417 | 411 | 0.06 | saturation | 4 | 0.090 | left_shoulder_roll_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 569 | 575 | 575 | 0.06 | saturation | 4 | 0.089 | right_wrist_pitch_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 639 | 645 | 644 | 0.06 | saturation | 4 | 0.093 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |

![flag_pelvis_z_f936.png](frames/dance_western_horse_step_with_leg_undercut_R_loop_002__A324__v5_ours/flag_pelvis_z_f936.png)

![flag_wrist_ang_vel_f454.png](frames/dance_western_horse_step_with_leg_undercut_R_loop_002__A324__v5_ours/flag_wrist_ang_vel_f454.png)

![flag_saturated_dof_f331.png](frames/dance_western_horse_step_with_leg_undercut_R_loop_002__A324__v5_ours/flag_saturated_dof_f331.png)

![section_00_peak_f331.png](frames/dance_western_horse_step_with_leg_undercut_R_loop_002__A324__v5_ours/section_00_peak_f331.png)

![section_01_peak_f1016.png](frames/dance_western_horse_step_with_leg_undercut_R_loop_002__A324__v5_ours/section_01_peak_f1016.png)

![section_02_peak_f453.png](frames/dance_western_horse_step_with_leg_undercut_R_loop_002__A324__v5_ours/section_02_peak_f453.png)

![section_03_peak_f94.png](frames/dance_western_horse_step_with_leg_undercut_R_loop_002__A324__v5_ours/section_03_peak_f94.png)

![section_04_peak_f151.png](frames/dance_western_horse_step_with_leg_undercut_R_loop_002__A324__v5_ours/section_04_peak_f151.png)

### `dance_basic_turn_v1_360_R_loop_fast_004__A322.bvh`  (13 sections, sat-score=67)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 261 | 295 | 289 | 0.29 | saturation | 7 | 0.076 | right_wrist_pitch_joint, right_ankle_pitch_joint, left_wrist_pitch_joint |
| 154 | 184 | 163 | 0.26 | saturation | 7 | 0.074 | right_wrist_pitch_joint, right_ankle_pitch_joint, right_ankle_roll_joint |
| 344 | 406 | 357 | 0.53 | saturation | 6 | 0.080 | right_wrist_pitch_joint, right_ankle_roll_joint, right_ankle_pitch_joint |
| 83 | 94 | 86 | 0.10 | saturation | 6 | 0.074 | right_shoulder_roll_joint, left_elbow_joint, left_ankle_roll_joint |
| 411 | 420 | 417 | 0.08 | saturation | 6 | 0.077 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 100 | 129 | 129 | 0.25 | saturation | 5 | 0.077 | right_wrist_pitch_joint, right_elbow_joint, left_elbow_joint |
| 236 | 249 | 237 | 0.12 | saturation | 5 | 0.080 | right_wrist_pitch_joint, left_elbow_joint, right_ankle_roll_joint |
| 221 | 233 | 232 | 0.11 | saturation | 5 | 0.078 | right_wrist_pitch_joint, right_elbow_joint, left_elbow_joint |
| 58 | 72 | 69 | 0.12 | saturation | 4 | 0.076 | left_elbow_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 333 | 341 | 337 | 0.07 | saturation | 4 | 0.078 | right_wrist_pitch_joint, right_elbow_joint, left_elbow_joint |
| 430 | 437 | 437 | 0.07 | saturation | 4 | 0.081 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 441 | 446 | 446 | 0.05 | saturation | 4 | 0.083 | right_wrist_pitch_joint, right_elbow_joint, right_shoulder_roll_joint |
| 207 | 211 | 211 | 0.04 | saturation | 4 | 0.074 | right_elbow_joint, left_elbow_joint, left_wrist_pitch_joint |

![flag_pelvis_z_f332.png](frames/dance_basic_turn_v1_360_R_loop_fast_004__A322__v5_ours/flag_pelvis_z_f332.png)

![flag_wrist_ang_vel_f192.png](frames/dance_basic_turn_v1_360_R_loop_fast_004__A322__v5_ours/flag_wrist_ang_vel_f192.png)

![flag_saturated_dof_f289.png](frames/dance_basic_turn_v1_360_R_loop_fast_004__A322__v5_ours/flag_saturated_dof_f289.png)

![section_00_peak_f289.png](frames/dance_basic_turn_v1_360_R_loop_fast_004__A322__v5_ours/section_00_peak_f289.png)

![section_01_peak_f163.png](frames/dance_basic_turn_v1_360_R_loop_fast_004__A322__v5_ours/section_01_peak_f163.png)

![section_02_peak_f357.png](frames/dance_basic_turn_v1_360_R_loop_fast_004__A322__v5_ours/section_02_peak_f357.png)

![section_03_peak_f86.png](frames/dance_basic_turn_v1_360_R_loop_fast_004__A322__v5_ours/section_03_peak_f86.png)

![section_04_peak_f417.png](frames/dance_basic_turn_v1_360_R_loop_fast_004__A322__v5_ours/section_04_peak_f417.png)

### `dance_retro_disco_finger_sequence_R_fast_002__A314.bvh`  (12 sections, sat-score=65)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 403 | 485 | 450 | 0.69 | saturation | 8 | 0.157 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |
| 487 | 529 | 492 | 0.36 | saturation | 8 | 0.121 | left_shoulder_roll_joint, right_ankle_roll_joint, left_elbow_joint |
| 159 | 342 | 243 | 1.53 | saturation | 6 | 0.091 | right_ankle_roll_joint, left_ankle_roll_joint, left_elbow_joint |
| 599 | 700 | 636 | 0.85 | saturation | 6 | 0.082 | right_ankle_roll_joint, left_ankle_roll_joint, left_shoulder_roll_joint |
| 27 | 78 | 40 | 0.43 | saturation | 6 | 0.078 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_wrist_roll_joint |
| 726 | 804 | 785 | 0.66 | saturation | 5 | 0.079 | left_elbow_joint, left_ankle_roll_joint, right_ankle_roll_joint |
| 86 | 124 | 122 | 0.33 | saturation | 5 | 0.076 | right_ankle_roll_joint, left_ankle_roll_joint, left_elbow_joint |
| 364 | 390 | 365 | 0.23 | saturation | 5 | 0.090 | right_ankle_roll_joint, left_ankle_roll_joint, right_elbow_joint |
| 543 | 565 | 565 | 0.19 | saturation | 4 | 0.087 | right_elbow_joint, left_elbow_joint, right_ankle_roll_joint |
| 578 | 587 | 586 | 0.08 | saturation | 4 | 0.085 | right_wrist_pitch_joint, left_elbow_joint, right_ankle_roll_joint |
| 820 | 826 | 826 | 0.06 | saturation | 4 | 0.084 | right_wrist_roll_joint, right_wrist_pitch_joint, right_shoulder_roll_joint |
| 828 | 832 | 832 | 0.04 | saturation | 4 | 0.092 | right_wrist_pitch_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f453.png](frames/dance_retro_disco_finger_sequence_R_fast_002__A314__v5_ours/flag_pelvis_z_f453.png)

![flag_wrist_ang_vel_f337.png](frames/dance_retro_disco_finger_sequence_R_fast_002__A314__v5_ours/flag_wrist_ang_vel_f337.png)

![flag_saturated_dof_f492.png](frames/dance_retro_disco_finger_sequence_R_fast_002__A314__v5_ours/flag_saturated_dof_f492.png)

![section_00_peak_f450.png](frames/dance_retro_disco_finger_sequence_R_fast_002__A314__v5_ours/section_00_peak_f450.png)

![section_01_peak_f492.png](frames/dance_retro_disco_finger_sequence_R_fast_002__A314__v5_ours/section_01_peak_f492.png)

![section_02_peak_f243.png](frames/dance_retro_disco_finger_sequence_R_fast_002__A314__v5_ours/section_02_peak_f243.png)

![section_03_peak_f636.png](frames/dance_retro_disco_finger_sequence_R_fast_002__A314__v5_ours/section_03_peak_f636.png)

![section_04_peak_f40.png](frames/dance_retro_disco_finger_sequence_R_fast_002__A314__v5_ours/section_04_peak_f40.png)

### `dance_vouge_butterfly_step_180_R_fast_002__A319.bvh`  (8 sections, sat-score=53)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 481 | 541 | 487 | 0.51 | saturation | 7 | 0.089 | right_wrist_pitch_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 420 | 450 | 431 | 0.26 | saturation | 7 | 0.100 | right_wrist_pitch_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 239 | 268 | 243 | 0.25 | saturation | 7 | 0.096 | left_shoulder_roll_joint, right_shoulder_roll_joint, right_wrist_pitch_joint |
| 361 | 387 | 369 | 0.23 | saturation | 7 | 0.093 | right_wrist_roll_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 115 | 138 | 125 | 0.20 | saturation | 7 | 0.093 | right_wrist_roll_joint, right_wrist_pitch_joint, right_shoulder_roll_joint |
| 177 | 214 | 187 | 0.32 | saturation | 6 | 0.106 | right_wrist_pitch_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 47 | 74 | 65 | 0.23 | saturation | 6 | 0.104 | right_wrist_roll_joint, right_wrist_pitch_joint, right_shoulder_roll_joint |
| 302 | 321 | 309 | 0.17 | saturation | 6 | 0.106 | right_wrist_roll_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f96.png](frames/dance_vouge_butterfly_step_180_R_fast_002__A319__v5_ours/flag_pelvis_z_f96.png)

![flag_wrist_ang_vel_f362.png](frames/dance_vouge_butterfly_step_180_R_fast_002__A319__v5_ours/flag_wrist_ang_vel_f362.png)

![flag_saturated_dof_f122.png](frames/dance_vouge_butterfly_step_180_R_fast_002__A319__v5_ours/flag_saturated_dof_f122.png)

![section_00_peak_f487.png](frames/dance_vouge_butterfly_step_180_R_fast_002__A319__v5_ours/section_00_peak_f487.png)

![section_01_peak_f431.png](frames/dance_vouge_butterfly_step_180_R_fast_002__A319__v5_ours/section_01_peak_f431.png)

![section_02_peak_f243.png](frames/dance_vouge_butterfly_step_180_R_fast_002__A319__v5_ours/section_02_peak_f243.png)

![section_03_peak_f369.png](frames/dance_vouge_butterfly_step_180_R_fast_002__A319__v5_ours/section_03_peak_f369.png)

![section_04_peak_f125.png](frames/dance_vouge_butterfly_step_180_R_fast_002__A319__v5_ours/section_04_peak_f125.png)

### `dance_retro_jazz_cross_step_180_R_001__A314.bvh`  (9 sections, sat-score=50)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 113 | 161 | 143 | 0.41 | saturation | 8 | 0.087 | left_wrist_roll_joint, left_shoulder_roll_joint, right_wrist_roll_joint |
| 249 | 299 | 293 | 0.42 | saturation | 7 | 0.088 | right_shoulder_roll_joint, right_wrist_pitch_joint, right_wrist_roll_joint |
| 392 | 442 | 436 | 0.42 | saturation | 7 | 0.085 | left_shoulder_roll_joint, right_wrist_roll_joint, right_wrist_pitch_joint |
| 542 | 587 | 554 | 0.38 | saturation | 7 | 0.082 | right_shoulder_roll_joint, left_wrist_pitch_joint, right_ankle_roll_joint |
| 504 | 508 | 507 | 0.04 | saturation | 5 | 0.082 | right_elbow_joint, left_wrist_roll_joint, left_hip_roll_joint |
| 66 | 76 | 70 | 0.09 | saturation | 4 | 0.077 | right_wrist_roll_joint, left_wrist_roll_joint, right_shoulder_roll_joint |
| 41 | 50 | 41 | 0.08 | saturation | 4 | 0.093 | right_wrist_roll_joint, left_wrist_roll_joint, right_shoulder_roll_joint |
| 604 | 609 | 609 | 0.05 | saturation | 4 | 0.089 | right_wrist_pitch_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 611 | 615 | 613 | 0.04 | saturation | 4 | 0.090 | right_wrist_roll_joint, right_wrist_pitch_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f138.png](frames/dance_retro_jazz_cross_step_180_R_001__A314__v5_ours/flag_pelvis_z_f138.png)

![flag_wrist_ang_vel_f144.png](frames/dance_retro_jazz_cross_step_180_R_001__A314__v5_ours/flag_wrist_ang_vel_f144.png)

![flag_saturated_dof_f143.png](frames/dance_retro_jazz_cross_step_180_R_001__A314__v5_ours/flag_saturated_dof_f143.png)

![section_00_peak_f143.png](frames/dance_retro_jazz_cross_step_180_R_001__A314__v5_ours/section_00_peak_f143.png)

![section_01_peak_f293.png](frames/dance_retro_jazz_cross_step_180_R_001__A314__v5_ours/section_01_peak_f293.png)

![section_02_peak_f436.png](frames/dance_retro_jazz_cross_step_180_R_001__A314__v5_ours/section_02_peak_f436.png)

![section_03_peak_f554.png](frames/dance_retro_jazz_cross_step_180_R_001__A314__v5_ours/section_03_peak_f554.png)

![section_04_peak_f507.png](frames/dance_retro_jazz_cross_step_180_R_001__A314__v5_ours/section_04_peak_f507.png)

### `victory_dance_asarahe_180_R_004__A324.bvh`  (11 sections, sat-score=47)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 709 | 723 | 720 | 0.12 | saturation | 5 | 0.086 | right_elbow_joint, left_elbow_joint, right_wrist_pitch_joint |
| 737 | 746 | 740 | 0.08 | saturation | 5 | 0.088 | right_elbow_joint, left_elbow_joint, left_wrist_roll_joint |
| 770 | 779 | 773 | 0.08 | saturation | 5 | 0.086 | right_wrist_roll_joint, right_elbow_joint, left_elbow_joint |
| 26 | 41 | 40 | 0.13 | saturation | 4 | 0.082 | right_wrist_roll_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 116 | 131 | 128 | 0.13 | saturation | 4 | 0.083 | right_wrist_roll_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 366 | 377 | 377 | 0.10 | saturation | 4 | 0.082 | right_wrist_roll_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 195 | 205 | 195 | 0.09 | saturation | 4 | 0.082 | right_wrist_roll_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 454 | 462 | 456 | 0.07 | saturation | 4 | 0.083 | right_wrist_roll_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 604 | 609 | 604 | 0.05 | saturation | 4 | 0.086 | right_wrist_pitch_joint, right_elbow_joint, left_shoulder_roll_joint |
| 681 | 686 | 681 | 0.05 | saturation | 4 | 0.085 | right_wrist_pitch_joint, right_elbow_joint, left_wrist_pitch_joint |
| 43 | 47 | 43 | 0.04 | saturation | 4 | 0.081 | right_wrist_roll_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f766.png](frames/victory_dance_asarahe_180_R_004__A324__v5_ours/flag_pelvis_z_f766.png)

![flag_wrist_ang_vel_f720.png](frames/victory_dance_asarahe_180_R_004__A324__v5_ours/flag_wrist_ang_vel_f720.png)

![flag_saturated_dof_f720.png](frames/victory_dance_asarahe_180_R_004__A324__v5_ours/flag_saturated_dof_f720.png)

![section_00_peak_f720.png](frames/victory_dance_asarahe_180_R_004__A324__v5_ours/section_00_peak_f720.png)

![section_01_peak_f740.png](frames/victory_dance_asarahe_180_R_004__A324__v5_ours/section_01_peak_f740.png)

![section_02_peak_f773.png](frames/victory_dance_asarahe_180_R_004__A324__v5_ours/section_02_peak_f773.png)

![section_03_peak_f40.png](frames/victory_dance_asarahe_180_R_004__A324__v5_ours/section_03_peak_f40.png)

![section_04_peak_f128.png](frames/victory_dance_asarahe_180_R_004__A324__v5_ours/section_04_peak_f128.png)

### `body_check_002__A492_M.bvh`  (11 sections, sat-score=47)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 2333 | 2456 | 2409 | 1.03 | saturation | 5 | 0.107 | right_wrist_pitch_joint, left_elbow_joint, right_shoulder_roll_joint |
| 2787 | 2839 | 2839 | 0.44 | saturation | 5 | 0.124 | left_shoulder_roll_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 2976 | 2999 | 2981 | 0.20 | saturation | 5 | 0.117 | right_wrist_pitch_joint, right_elbow_joint, left_shoulder_roll_joint |
| 1590 | 1678 | 1677 | 0.74 | saturation | 4 | 0.076 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 1867 | 1947 | 1947 | 0.68 | saturation | 4 | 0.090 | left_shoulder_roll_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 2737 | 2776 | 2767 | 0.33 | saturation | 4 | 0.123 | left_shoulder_roll_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 1526 | 1555 | 1526 | 0.25 | saturation | 4 | 0.076 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 16 | 35 | 16 | 0.17 | saturation | 4 | 0.091 | right_elbow_joint, left_elbow_joint, right_shoulder_roll_joint |
| 1495 | 1513 | 1495 | 0.16 | saturation | 4 | 0.078 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 1405 | 1419 | 1419 | 0.12 | saturation | 4 | 0.078 | right_wrist_pitch_joint, right_elbow_joint, left_wrist_pitch_joint |
| 1026 | 1035 | 1026 | 0.08 | saturation | 4 | 0.077 | right_wrist_roll_joint, right_wrist_pitch_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f2842.png](frames/body_check_002__A492_M__v5_ours/flag_pelvis_z_f2842.png)

![flag_wrist_ang_vel_f2841.png](frames/body_check_002__A492_M__v5_ours/flag_wrist_ang_vel_f2841.png)

![flag_saturated_dof_f2404.png](frames/body_check_002__A492_M__v5_ours/flag_saturated_dof_f2404.png)

![section_00_peak_f2409.png](frames/body_check_002__A492_M__v5_ours/section_00_peak_f2409.png)

![section_01_peak_f2839.png](frames/body_check_002__A492_M__v5_ours/section_01_peak_f2839.png)

![section_02_peak_f2981.png](frames/body_check_002__A492_M__v5_ours/section_02_peak_f2981.png)

![section_03_peak_f1677.png](frames/body_check_002__A492_M__v5_ours/section_03_peak_f1677.png)

![section_04_peak_f1947.png](frames/body_check_002__A492_M__v5_ours/section_04_peak_f1947.png)

### `painful_stand_on_turn_walk_ff_360_start_R_001__A461_M.bvh`  (9 sections, sat-score=45)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 769 | 905 | 848 | 1.14 | saturation | 9 | 0.076 | left_ankle_pitch_joint, left_hip_roll_joint, right_shoulder_roll_joint |
| 908 | 946 | 934 | 0.33 | saturation | 5 | 0.076 | right_wrist_roll_joint, right_shoulder_roll_joint, left_hip_roll_joint |
| 659 | 673 | 666 | 0.12 | saturation | 5 | 0.086 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_pitch_joint |
| 253 | 259 | 255 | 0.06 | saturation | 5 | 0.103 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_pitch_joint |
| 703 | 707 | 705 | 0.04 | saturation | 5 | 0.074 | left_shoulder_roll_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 1385 | 1444 | 1435 | 0.50 | saturation | 4 | 0.077 | right_wrist_roll_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 1232 | 1244 | 1241 | 0.11 | saturation | 4 | 0.084 | right_wrist_roll_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 1258 | 1269 | 1269 | 0.10 | saturation | 4 | 0.080 | right_wrist_roll_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 962 | 967 | 962 | 0.05 | saturation | 4 | 0.076 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_hip_roll_joint |

![flag_pelvis_z_f263.png](frames/painful_stand_on_turn_walk_ff_360_start_R_001__A461_M__v5_ours/flag_pelvis_z_f263.png)

![flag_wrist_ang_vel_f842.png](frames/painful_stand_on_turn_walk_ff_360_start_R_001__A461_M__v5_ours/flag_wrist_ang_vel_f842.png)

![flag_saturated_dof_f848.png](frames/painful_stand_on_turn_walk_ff_360_start_R_001__A461_M__v5_ours/flag_saturated_dof_f848.png)

![section_00_peak_f848.png](frames/painful_stand_on_turn_walk_ff_360_start_R_001__A461_M__v5_ours/section_00_peak_f848.png)

![section_01_peak_f934.png](frames/painful_stand_on_turn_walk_ff_360_start_R_001__A461_M__v5_ours/section_01_peak_f934.png)

![section_02_peak_f666.png](frames/painful_stand_on_turn_walk_ff_360_start_R_001__A461_M__v5_ours/section_02_peak_f666.png)

![section_03_peak_f255.png](frames/painful_stand_on_turn_walk_ff_360_start_R_001__A461_M__v5_ours/section_03_peak_f255.png)

![section_04_peak_f705.png](frames/painful_stand_on_turn_walk_ff_360_start_R_001__A461_M__v5_ours/section_04_peak_f705.png)

### `painful_stand_on_walk_ff_360_R_001__A461.bvh`  (10 sections, sat-score=45)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 1304 | 1717 | 1345 | 3.45 | saturation | 7 | 0.083 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_pitch_joint |
| 1077 | 1143 | 1101 | 0.56 | saturation | 5 | 0.079 | left_shoulder_roll_joint, left_wrist_roll_joint, right_shoulder_roll_joint |
| 896 | 911 | 907 | 0.13 | saturation | 5 | 0.077 | right_wrist_roll_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 1968 | 2036 | 2036 | 0.57 | saturation | 4 | 0.084 | right_wrist_roll_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 1813 | 1876 | 1813 | 0.53 | saturation | 4 | 0.084 | right_wrist_roll_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 2120 | 2172 | 2152 | 0.44 | saturation | 4 | 0.090 | right_wrist_roll_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 436 | 466 | 446 | 0.26 | saturation | 4 | 0.077 | right_wrist_roll_joint, right_wrist_pitch_joint, right_shoulder_roll_joint |
| 172 | 189 | 186 | 0.15 | saturation | 4 | 0.101 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 712 | 728 | 728 | 0.14 | saturation | 4 | 0.078 | right_wrist_roll_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 953 | 958 | 953 | 0.05 | saturation | 4 | 0.084 | right_wrist_roll_joint, right_wrist_pitch_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f415.png](frames/painful_stand_on_walk_ff_360_R_001__A461__v5_ours/flag_pelvis_z_f415.png)

![flag_wrist_ang_vel_f398.png](frames/painful_stand_on_walk_ff_360_R_001__A461__v5_ours/flag_wrist_ang_vel_f398.png)

![flag_saturated_dof_f1347.png](frames/painful_stand_on_walk_ff_360_R_001__A461__v5_ours/flag_saturated_dof_f1347.png)

![section_00_peak_f1345.png](frames/painful_stand_on_walk_ff_360_R_001__A461__v5_ours/section_00_peak_f1345.png)

![section_01_peak_f1101.png](frames/painful_stand_on_walk_ff_360_R_001__A461__v5_ours/section_01_peak_f1101.png)

![section_02_peak_f907.png](frames/painful_stand_on_walk_ff_360_R_001__A461__v5_ours/section_02_peak_f907.png)

![section_03_peak_f2036.png](frames/painful_stand_on_walk_ff_360_R_001__A461__v5_ours/section_03_peak_f2036.png)

![section_04_peak_f1813.png](frames/painful_stand_on_walk_ff_360_R_001__A461__v5_ours/section_04_peak_f1813.png)

### `walk_big_dog_ff_315_loop_R_002__A495.bvh`  (5 sections, sat-score=40)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 233 | 363 | 257 | 1.09 | saturation | 11 | 0.091 | right_wrist_roll_joint, right_elbow_joint, right_wrist_pitch_joint |
| 106 | 208 | 176 | 0.86 | saturation | 11 | 0.086 | right_wrist_roll_joint, right_elbow_joint, right_wrist_pitch_joint |
| 0 | 100 | 87 | 0.84 | saturation | 9 | 0.074 | right_wrist_roll_joint, right_elbow_joint, right_wrist_pitch_joint |
| 392 | 402 | 399 | 0.09 | saturation | 5 | 0.071 | right_wrist_roll_joint, right_elbow_joint, right_wrist_pitch_joint |
| 372 | 387 | 376 | 0.13 | saturation | 4 | 0.072 | right_wrist_roll_joint, right_elbow_joint, right_wrist_pitch_joint |

![flag_pelvis_z_f259.png](frames/walk_big_dog_ff_315_loop_R_002__A495__v5_ours/flag_pelvis_z_f259.png)

![flag_wrist_ang_vel_f23.png](frames/walk_big_dog_ff_315_loop_R_002__A495__v5_ours/flag_wrist_ang_vel_f23.png)

![flag_saturated_dof_f176.png](frames/walk_big_dog_ff_315_loop_R_002__A495__v5_ours/flag_saturated_dof_f176.png)

![section_00_peak_f257.png](frames/walk_big_dog_ff_315_loop_R_002__A495__v5_ours/section_00_peak_f257.png)

![section_01_peak_f176.png](frames/walk_big_dog_ff_315_loop_R_002__A495__v5_ours/section_01_peak_f176.png)

![section_02_peak_f87.png](frames/walk_big_dog_ff_315_loop_R_002__A495__v5_ours/section_02_peak_f87.png)

![section_03_peak_f399.png](frames/walk_big_dog_ff_315_loop_R_002__A495__v5_ours/section_03_peak_f399.png)

![section_04_peak_f376.png](frames/walk_big_dog_ff_315_loop_R_002__A495__v5_ours/section_04_peak_f376.png)

### `walk_ff_start_360_R_003__A267.bvh`  (9 sections, sat-score=40)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 586 | 608 | 603 | 0.19 | saturation | 5 | 0.088 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 419 | 436 | 429 | 0.15 | saturation | 5 | 0.086 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 674 | 690 | 689 | 0.14 | saturation | 5 | 0.089 | right_elbow_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 846 | 851 | 850 | 0.05 | saturation | 5 | 0.088 | right_elbow_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 733 | 751 | 733 | 0.16 | saturation | 4 | 0.094 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 762 | 776 | 776 | 0.12 | saturation | 4 | 0.087 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 206 | 218 | 212 | 0.11 | saturation | 4 | 0.094 | right_elbow_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 643 | 652 | 643 | 0.08 | saturation | 4 | 0.094 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |
| 251 | 256 | 256 | 0.05 | saturation | 4 | 0.088 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |

![flag_pelvis_z_f764.png](frames/walk_ff_start_360_R_003__A267__v5_ours/flag_pelvis_z_f764.png)

![flag_wrist_ang_vel_f850.png](frames/walk_ff_start_360_R_003__A267__v5_ours/flag_wrist_ang_vel_f850.png)

![flag_saturated_dof_f423.png](frames/walk_ff_start_360_R_003__A267__v5_ours/flag_saturated_dof_f423.png)

![section_00_peak_f603.png](frames/walk_ff_start_360_R_003__A267__v5_ours/section_00_peak_f603.png)

![section_01_peak_f429.png](frames/walk_ff_start_360_R_003__A267__v5_ours/section_01_peak_f429.png)

![section_02_peak_f689.png](frames/walk_ff_start_360_R_003__A267__v5_ours/section_02_peak_f689.png)

![section_03_peak_f850.png](frames/walk_ff_start_360_R_003__A267__v5_ours/section_03_peak_f850.png)

![section_04_peak_f733.png](frames/walk_ff_start_360_R_003__A267__v5_ours/section_04_peak_f733.png)

### `walk_ff_start_270_R_slow_001__A443.bvh`  (6 sections, sat-score=34)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 744 | 891 | 833 | 1.23 | saturation | 7 | 0.096 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_roll_joint |
| 1035 | 1142 | 1115 | 0.90 | saturation | 7 | 0.092 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_hip_roll_joint |
| 211 | 303 | 245 | 0.78 | saturation | 6 | 0.097 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_hip_roll_joint |
| 481 | 568 | 554 | 0.73 | saturation | 6 | 0.094 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_hip_roll_joint |
| 577 | 581 | 577 | 0.04 | saturation | 4 | 0.094 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |
| 1162 | 1166 | 1162 | 0.04 | saturation | 4 | 0.094 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |

![flag_pelvis_z_f213.png](frames/walk_ff_start_270_R_slow_001__A443__v5_ours/flag_pelvis_z_f213.png)

![flag_wrist_ang_vel_f567.png](frames/walk_ff_start_270_R_slow_001__A443__v5_ours/flag_wrist_ang_vel_f567.png)

![flag_saturated_dof_f1115.png](frames/walk_ff_start_270_R_slow_001__A443__v5_ours/flag_saturated_dof_f1115.png)

![section_00_peak_f833.png](frames/walk_ff_start_270_R_slow_001__A443__v5_ours/section_00_peak_f833.png)

![section_01_peak_f1115.png](frames/walk_ff_start_270_R_slow_001__A443__v5_ours/section_01_peak_f1115.png)

![section_02_peak_f245.png](frames/walk_ff_start_270_R_slow_001__A443__v5_ours/section_02_peak_f245.png)

![section_03_peak_f554.png](frames/walk_ff_start_270_R_slow_001__A443__v5_ours/section_03_peak_f554.png)

![section_04_peak_f577.png](frames/walk_ff_start_270_R_slow_001__A443__v5_ours/section_04_peak_f577.png)

### `eat_hotdog_standing_fail_R_001__A456_M.bvh`  (7 sections, sat-score=32)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 1211 | 1621 | 1489 | 3.42 | saturation | 6 | 0.091 | left_shoulder_roll_joint, right_shoulder_roll_joint, right_elbow_joint |
| 1087 | 1152 | 1101 | 0.55 | saturation | 5 | 0.079 | left_shoulder_roll_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 728 | 757 | 739 | 0.25 | saturation | 5 | 0.078 | right_elbow_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 403 | 691 | 553 | 2.41 | saturation | 4 | 0.086 | right_elbow_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 198 | 359 | 201 | 1.35 | saturation | 4 | 0.087 | right_elbow_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 1030 | 1074 | 1058 | 0.38 | saturation | 4 | 0.077 | right_wrist_roll_joint, right_wrist_pitch_joint, right_shoulder_roll_joint |
| 819 | 823 | 823 | 0.04 | saturation | 4 | 0.070 | right_wrist_roll_joint, right_wrist_pitch_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f1436.png](frames/eat_hotdog_standing_fail_R_001__A456_M__v5_ours/flag_pelvis_z_f1436.png)

![flag_wrist_ang_vel_f1154.png](frames/eat_hotdog_standing_fail_R_001__A456_M__v5_ours/flag_wrist_ang_vel_f1154.png)

![flag_saturated_dof_f1489.png](frames/eat_hotdog_standing_fail_R_001__A456_M__v5_ours/flag_saturated_dof_f1489.png)

![section_00_peak_f1489.png](frames/eat_hotdog_standing_fail_R_001__A456_M__v5_ours/section_00_peak_f1489.png)

![section_01_peak_f1101.png](frames/eat_hotdog_standing_fail_R_001__A456_M__v5_ours/section_01_peak_f1101.png)

![section_02_peak_f739.png](frames/eat_hotdog_standing_fail_R_001__A456_M__v5_ours/section_02_peak_f739.png)

![section_03_peak_f553.png](frames/eat_hotdog_standing_fail_R_001__A456_M__v5_ours/section_03_peak_f553.png)

![section_04_peak_f201.png](frames/eat_hotdog_standing_fail_R_001__A456_M__v5_ours/section_04_peak_f201.png)

### `victory_dance_loser_jump_180_R_003__A308.bvh`  (7 sections, sat-score=30)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 353 | 368 | 367 | 0.13 | saturation | 5 | 0.071 | right_wrist_roll_joint, left_shoulder_roll_joint, left_wrist_roll_joint |
| 15 | 23 | 19 | 0.07 | saturation | 5 | 0.086 | right_wrist_roll_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 175 | 189 | 182 | 0.12 | saturation | 4 | 0.076 | right_wrist_roll_joint, left_shoulder_roll_joint, left_wrist_roll_joint |
| 412 | 424 | 424 | 0.11 | saturation | 4 | 0.081 | right_wrist_roll_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 224 | 231 | 231 | 0.07 | saturation | 4 | 0.071 | right_wrist_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |
| 45 | 51 | 51 | 0.06 | saturation | 4 | 0.071 | right_wrist_roll_joint, left_wrist_roll_joint, right_ankle_roll_joint |
| 139 | 143 | 143 | 0.04 | saturation | 4 | 0.070 | right_wrist_roll_joint, left_shoulder_roll_joint, right_ankle_roll_joint |

![flag_pelvis_z_f247.png](frames/victory_dance_loser_jump_180_R_003__A308__v5_ours/flag_pelvis_z_f247.png)

![flag_wrist_ang_vel_f19.png](frames/victory_dance_loser_jump_180_R_003__A308__v5_ours/flag_wrist_ang_vel_f19.png)

![flag_saturated_dof_f367.png](frames/victory_dance_loser_jump_180_R_003__A308__v5_ours/flag_saturated_dof_f367.png)

![section_00_peak_f367.png](frames/victory_dance_loser_jump_180_R_003__A308__v5_ours/section_00_peak_f367.png)

![section_01_peak_f19.png](frames/victory_dance_loser_jump_180_R_003__A308__v5_ours/section_01_peak_f19.png)

![section_02_peak_f182.png](frames/victory_dance_loser_jump_180_R_003__A308__v5_ours/section_02_peak_f182.png)

![section_03_peak_f424.png](frames/victory_dance_loser_jump_180_R_003__A308__v5_ours/section_03_peak_f424.png)

![section_04_peak_f231.png](frames/victory_dance_loser_jump_180_R_003__A308__v5_ours/section_04_peak_f231.png)

### `body_check_001__A251.bvh`  (6 sections, sat-score=28)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 1481 | 1582 | 1524 | 0.85 | saturation | 6 | 0.069 | right_wrist_roll_joint, left_wrist_roll_joint, right_shoulder_roll_joint |
| 1428 | 1475 | 1449 | 0.40 | saturation | 6 | 0.069 | right_wrist_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 2597 | 2698 | 2636 | 0.85 | saturation | 4 | 0.116 | right_wrist_pitch_joint, right_elbow_joint, left_shoulder_roll_joint |
| 2313 | 2330 | 2330 | 0.15 | saturation | 4 | 0.118 | right_elbow_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 2459 | 2472 | 2459 | 0.12 | saturation | 4 | 0.125 | right_elbow_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 1054 | 1061 | 1061 | 0.07 | saturation | 4 | 0.077 | right_wrist_roll_joint, left_wrist_roll_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f2607.png](frames/body_check_001__A251__v5_ours/flag_pelvis_z_f2607.png)

![flag_wrist_ang_vel_f243.png](frames/body_check_001__A251__v5_ours/flag_wrist_ang_vel_f243.png)

![flag_saturated_dof_f1452.png](frames/body_check_001__A251__v5_ours/flag_saturated_dof_f1452.png)

![section_00_peak_f1524.png](frames/body_check_001__A251__v5_ours/section_00_peak_f1524.png)

![section_01_peak_f1449.png](frames/body_check_001__A251__v5_ours/section_01_peak_f1449.png)

![section_02_peak_f2636.png](frames/body_check_001__A251__v5_ours/section_02_peak_f2636.png)

![section_03_peak_f2330.png](frames/body_check_001__A251__v5_ours/section_03_peak_f2330.png)

![section_04_peak_f2459.png](frames/body_check_001__A251__v5_ours/section_04_peak_f2459.png)

### `body_check_002__A496.bvh`  (6 sections, sat-score=26)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 2268 | 2407 | 2294 | 1.17 | saturation | 5 | 0.109 | right_elbow_joint, left_shoulder_roll_joint, left_hip_roll_joint |
| 2934 | 2999 | 2978 | 0.55 | saturation | 5 | 0.117 | right_wrist_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 1536 | 1628 | 1586 | 0.78 | saturation | 4 | 0.078 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 631 | 645 | 632 | 0.12 | saturation | 4 | 0.079 | right_wrist_roll_joint, right_wrist_pitch_joint, right_shoulder_roll_joint |
| 2161 | 2174 | 2174 | 0.12 | saturation | 4 | 0.106 | right_wrist_roll_joint, right_elbow_joint, left_wrist_roll_joint |
| 2249 | 2253 | 2249 | 0.04 | saturation | 4 | 0.109 | right_wrist_roll_joint, right_elbow_joint, left_shoulder_roll_joint |

![flag_pelvis_z_f2359.png](frames/body_check_002__A496__v5_ours/flag_pelvis_z_f2359.png)

![flag_wrist_ang_vel_f740.png](frames/body_check_002__A496__v5_ours/flag_wrist_ang_vel_f740.png)

![flag_saturated_dof_f2961.png](frames/body_check_002__A496__v5_ours/flag_saturated_dof_f2961.png)

![section_00_peak_f2294.png](frames/body_check_002__A496__v5_ours/section_00_peak_f2294.png)

![section_01_peak_f2978.png](frames/body_check_002__A496__v5_ours/section_01_peak_f2978.png)

![section_02_peak_f1586.png](frames/body_check_002__A496__v5_ours/section_02_peak_f1586.png)

![section_03_peak_f632.png](frames/body_check_002__A496__v5_ours/section_03_peak_f632.png)

![section_04_peak_f2174.png](frames/body_check_002__A496__v5_ours/section_04_peak_f2174.png)

### `body_check_004__A444.bvh`  (6 sections, sat-score=25)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 2397 | 2454 | 2450 | 0.48 | saturation | 5 | 0.084 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 1030 | 1126 | 1123 | 0.81 | saturation | 4 | 0.073 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 112 | 173 | 126 | 0.52 | saturation | 4 | 0.073 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 970 | 1024 | 1022 | 0.46 | saturation | 4 | 0.073 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 295 | 324 | 319 | 0.25 | saturation | 4 | 0.073 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 2479 | 2497 | 2479 | 0.16 | saturation | 4 | 0.087 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f2343.png](frames/body_check_004__A444__v5_ours/flag_pelvis_z_f2343.png)

![flag_wrist_ang_vel_f1136.png](frames/body_check_004__A444__v5_ours/flag_wrist_ang_vel_f1136.png)

![flag_saturated_dof_f2433.png](frames/body_check_004__A444__v5_ours/flag_saturated_dof_f2433.png)

![section_00_peak_f2450.png](frames/body_check_004__A444__v5_ours/section_00_peak_f2450.png)

![section_01_peak_f1123.png](frames/body_check_004__A444__v5_ours/section_01_peak_f1123.png)

![section_02_peak_f126.png](frames/body_check_004__A444__v5_ours/section_02_peak_f126.png)

![section_03_peak_f1022.png](frames/body_check_004__A444__v5_ours/section_03_peak_f1022.png)

![section_04_peak_f319.png](frames/body_check_004__A444__v5_ours/section_04_peak_f319.png)

### `medium_big_heavy_one_hand_walk_ff_start_360_R_001__A506_M.bvh`  (6 sections, sat-score=24)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 896 | 935 | 917 | 0.33 | saturation | 4 | 0.084 | left_shoulder_roll_joint, left_wrist_roll_joint, right_shoulder_roll_joint |
| 754 | 784 | 761 | 0.26 | saturation | 4 | 0.082 | left_shoulder_roll_joint, left_wrist_roll_joint, right_shoulder_roll_joint |
| 430 | 438 | 430 | 0.07 | saturation | 4 | 0.080 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 742 | 750 | 742 | 0.07 | saturation | 4 | 0.080 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 276 | 283 | 276 | 0.07 | saturation | 4 | 0.081 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 135 | 140 | 140 | 0.05 | saturation | 4 | 0.086 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f431.png](frames/medium_big_heavy_one_hand_walk_ff_start_360_R_001__A506_M__v5_ours/flag_pelvis_z_f431.png)

![flag_wrist_ang_vel_f28.png](frames/medium_big_heavy_one_hand_walk_ff_start_360_R_001__A506_M__v5_ours/flag_wrist_ang_vel_f28.png)

![flag_saturated_dof_f928.png](frames/medium_big_heavy_one_hand_walk_ff_start_360_R_001__A506_M__v5_ours/flag_saturated_dof_f928.png)

![section_00_peak_f917.png](frames/medium_big_heavy_one_hand_walk_ff_start_360_R_001__A506_M__v5_ours/section_00_peak_f917.png)

![section_01_peak_f761.png](frames/medium_big_heavy_one_hand_walk_ff_start_360_R_001__A506_M__v5_ours/section_01_peak_f761.png)

![section_02_peak_f430.png](frames/medium_big_heavy_one_hand_walk_ff_start_360_R_001__A506_M__v5_ours/section_02_peak_f430.png)

![section_03_peak_f742.png](frames/medium_big_heavy_one_hand_walk_ff_start_360_R_001__A506_M__v5_ours/section_03_peak_f742.png)

![section_04_peak_f276.png](frames/medium_big_heavy_one_hand_walk_ff_start_360_R_001__A506_M__v5_ours/section_04_peak_f276.png)

### `body_check_001__A527_M.bvh`  (5 sections, sat-score=23)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 2019 | 2306 | 2126 | 2.40 | saturation | 7 | 0.072 | right_wrist_roll_joint, right_elbow_joint, left_elbow_joint |
| 2408 | 2430 | 2413 | 0.19 | saturation | 4 | 0.078 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 2347 | 2362 | 2347 | 0.13 | saturation | 4 | 0.078 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 2005 | 2017 | 2005 | 0.11 | saturation | 4 | 0.079 | right_wrist_roll_joint, left_wrist_roll_joint, right_shoulder_roll_joint |
| 1798 | 1802 | 1802 | 0.04 | saturation | 4 | 0.078 | right_wrist_pitch_joint, left_elbow_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f2986.png](frames/body_check_001__A527_M__v5_ours/flag_pelvis_z_f2986.png)

![flag_wrist_ang_vel_f2843.png](frames/body_check_001__A527_M__v5_ours/flag_wrist_ang_vel_f2843.png)

![flag_saturated_dof_f2129.png](frames/body_check_001__A527_M__v5_ours/flag_saturated_dof_f2129.png)

![section_00_peak_f2126.png](frames/body_check_001__A527_M__v5_ours/section_00_peak_f2126.png)

![section_01_peak_f2413.png](frames/body_check_001__A527_M__v5_ours/section_01_peak_f2413.png)

![section_02_peak_f2347.png](frames/body_check_001__A527_M__v5_ours/section_02_peak_f2347.png)

![section_03_peak_f2005.png](frames/body_check_001__A527_M__v5_ours/section_03_peak_f2005.png)

![section_04_peak_f1802.png](frames/body_check_001__A527_M__v5_ours/section_04_peak_f1802.png)

### `big_light_two_hands_right_side_high_to_behind_high_R_001__A525.bvh`  (5 sections, sat-score=22)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 518 | 658 | 531 | 1.18 | saturation | 5 | 0.083 | right_wrist_pitch_joint, left_elbow_joint, right_elbow_joint |
| 783 | 849 | 845 | 0.56 | saturation | 5 | 0.079 | right_wrist_roll_joint, right_elbow_joint, left_shoulder_roll_joint |
| 879 | 901 | 901 | 0.19 | saturation | 4 | 0.089 | right_wrist_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 503 | 510 | 510 | 0.07 | saturation | 4 | 0.083 | right_wrist_pitch_joint, right_elbow_joint, left_elbow_joint |
| 745 | 752 | 752 | 0.07 | saturation | 4 | 0.080 | left_wrist_pitch_joint, left_elbow_joint, left_ankle_pitch_joint |

![flag_pelvis_z_f763.png](frames/big_light_two_hands_right_side_high_to_behind_high_R_001__A525__v5_ours/flag_pelvis_z_f763.png)

![flag_wrist_ang_vel_f757.png](frames/big_light_two_hands_right_side_high_to_behind_high_R_001__A525__v5_ours/flag_wrist_ang_vel_f757.png)

![flag_saturated_dof_f531.png](frames/big_light_two_hands_right_side_high_to_behind_high_R_001__A525__v5_ours/flag_saturated_dof_f531.png)

![section_00_peak_f531.png](frames/big_light_two_hands_right_side_high_to_behind_high_R_001__A525__v5_ours/section_00_peak_f531.png)

![section_01_peak_f845.png](frames/big_light_two_hands_right_side_high_to_behind_high_R_001__A525__v5_ours/section_01_peak_f845.png)

![section_02_peak_f901.png](frames/big_light_two_hands_right_side_high_to_behind_high_R_001__A525__v5_ours/section_02_peak_f901.png)

![section_03_peak_f510.png](frames/big_light_two_hands_right_side_high_to_behind_high_R_001__A525__v5_ours/section_03_peak_f510.png)

![section_04_peak_f752.png](frames/big_light_two_hands_right_side_high_to_behind_high_R_001__A525__v5_ours/section_04_peak_f752.png)

### `walk_ff_loop_225_005__A059_M.bvh`  (4 sections, sat-score=22)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 263 | 355 | 335 | 0.78 | saturation | 6 | 0.102 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_roll_joint |
| 149 | 206 | 186 | 0.48 | saturation | 6 | 0.103 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_hip_roll_joint |
| 1 | 53 | 36 | 0.44 | saturation | 6 | 0.104 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_hip_roll_joint |
| 362 | 366 | 362 | 0.04 | saturation | 4 | 0.099 | right_wrist_roll_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f172.png](frames/walk_ff_loop_225_005__A059_M__v5_ours/flag_pelvis_z_f172.png)

![flag_wrist_ang_vel_f146.png](frames/walk_ff_loop_225_005__A059_M__v5_ours/flag_wrist_ang_vel_f146.png)

![flag_saturated_dof_f15.png](frames/walk_ff_loop_225_005__A059_M__v5_ours/flag_saturated_dof_f15.png)

![section_00_peak_f335.png](frames/walk_ff_loop_225_005__A059_M__v5_ours/section_00_peak_f335.png)

![section_01_peak_f186.png](frames/walk_ff_loop_225_005__A059_M__v5_ours/section_01_peak_f186.png)

![section_02_peak_f36.png](frames/walk_ff_loop_225_005__A059_M__v5_ours/section_02_peak_f36.png)

![section_03_peak_f362.png](frames/walk_ff_loop_225_005__A059_M__v5_ours/section_03_peak_f362.png)

### `walk_ff_start_360_004__A146.bvh`  (5 sections, sat-score=20)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 608 | 629 | 608 | 0.18 | saturation | 4 | 0.093 | right_wrist_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 492 | 512 | 512 | 0.17 | saturation | 4 | 0.093 | right_wrist_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 368 | 386 | 386 | 0.16 | saturation | 4 | 0.093 | right_wrist_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 350 | 354 | 350 | 0.04 | saturation | 4 | 0.095 | right_wrist_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 410 | 414 | 410 | 0.04 | saturation | 4 | 0.094 | right_elbow_joint, left_elbow_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f432.png](frames/walk_ff_start_360_004__A146__v5_ours/flag_pelvis_z_f432.png)

![flag_wrist_ang_vel_f278.png](frames/walk_ff_start_360_004__A146__v5_ours/flag_wrist_ang_vel_f278.png)

![flag_saturated_dof_f500.png](frames/walk_ff_start_360_004__A146__v5_ours/flag_saturated_dof_f500.png)

![section_00_peak_f608.png](frames/walk_ff_start_360_004__A146__v5_ours/section_00_peak_f608.png)

![section_01_peak_f512.png](frames/walk_ff_start_360_004__A146__v5_ours/section_01_peak_f512.png)

![section_02_peak_f386.png](frames/walk_ff_start_360_004__A146__v5_ours/section_02_peak_f386.png)

![section_03_peak_f350.png](frames/walk_ff_start_360_004__A146__v5_ours/section_03_peak_f350.png)

![section_04_peak_f410.png](frames/walk_ff_start_360_004__A146__v5_ours/section_04_peak_f410.png)

### `dance_jazz_hands_002__A467.bvh`  (4 sections, sat-score=19)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 286 | 673 | 325 | 3.23 | saturation | 7 | 0.089 | right_wrist_pitch_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 686 | 712 | 712 | 0.23 | saturation | 4 | 0.090 | right_wrist_pitch_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 205 | 221 | 205 | 0.14 | saturation | 4 | 0.080 | left_shoulder_roll_joint, right_ankle_roll_joint, right_ankle_pitch_joint |
| 721 | 732 | 721 | 0.10 | saturation | 4 | 0.089 | right_wrist_pitch_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f294.png](frames/dance_jazz_hands_002__A467__v5_ours/flag_pelvis_z_f294.png)

![flag_wrist_ang_vel_f358.png](frames/dance_jazz_hands_002__A467__v5_ours/flag_wrist_ang_vel_f358.png)

![flag_saturated_dof_f376.png](frames/dance_jazz_hands_002__A467__v5_ours/flag_saturated_dof_f376.png)

![section_00_peak_f325.png](frames/dance_jazz_hands_002__A467__v5_ours/section_00_peak_f325.png)

![section_01_peak_f712.png](frames/dance_jazz_hands_002__A467__v5_ours/section_01_peak_f712.png)

![section_02_peak_f205.png](frames/dance_jazz_hands_002__A467__v5_ours/section_02_peak_f205.png)

![section_03_peak_f721.png](frames/dance_jazz_hands_002__A467__v5_ours/section_03_peak_f721.png)

### `body_check_001__A381_M.bvh`  (4 sections, sat-score=19)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 850 | 1037 | 863 | 1.57 | saturation | 6 | 0.076 | right_shoulder_roll_joint, left_shoulder_roll_joint, right_wrist_roll_joint |
| 166 | 307 | 170 | 1.18 | saturation | 5 | 0.080 | right_wrist_roll_joint, right_wrist_pitch_joint, right_shoulder_roll_joint |
| 1637 | 1647 | 1647 | 0.09 | saturation | 4 | 0.067 | right_wrist_roll_joint, right_elbow_joint, left_wrist_roll_joint |
| 324 | 329 | 324 | 0.05 | saturation | 4 | 0.074 | right_wrist_roll_joint, right_wrist_pitch_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f2971.png](frames/body_check_001__A381_M__v5_ours/flag_pelvis_z_f2971.png)

![flag_wrist_ang_vel_f892.png](frames/body_check_001__A381_M__v5_ours/flag_wrist_ang_vel_f892.png)

![flag_saturated_dof_f964.png](frames/body_check_001__A381_M__v5_ours/flag_saturated_dof_f964.png)

![section_00_peak_f863.png](frames/body_check_001__A381_M__v5_ours/section_00_peak_f863.png)

![section_01_peak_f170.png](frames/body_check_001__A381_M__v5_ours/section_01_peak_f170.png)

![section_02_peak_f1647.png](frames/body_check_001__A381_M__v5_ours/section_02_peak_f1647.png)

![section_03_peak_f324.png](frames/body_check_001__A381_M__v5_ours/section_03_peak_f324.png)

### `body_check_002__A497.bvh`  (4 sections, sat-score=17)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 2977 | 2999 | 2999 | 0.19 | saturation | 5 | 0.101 | right_elbow_joint, left_elbow_joint, right_shoulder_roll_joint |
| 2371 | 2438 | 2438 | 0.57 | saturation | 4 | 0.098 | right_wrist_pitch_joint, right_elbow_joint, left_shoulder_roll_joint |
| 1476 | 1525 | 1525 | 0.42 | saturation | 4 | 0.076 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 2527 | 2532 | 2532 | 0.05 | saturation | 4 | 0.074 | right_elbow_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f1890.png](frames/body_check_002__A497__v5_ours/flag_pelvis_z_f1890.png)

![flag_wrist_ang_vel_f2487.png](frames/body_check_002__A497__v5_ours/flag_wrist_ang_vel_f2487.png)

![flag_saturated_dof_f2995.png](frames/body_check_002__A497__v5_ours/flag_saturated_dof_f2995.png)

![section_00_peak_f2999.png](frames/body_check_002__A497__v5_ours/section_00_peak_f2999.png)

![section_01_peak_f2438.png](frames/body_check_002__A497__v5_ours/section_01_peak_f2438.png)

![section_02_peak_f1525.png](frames/body_check_002__A497__v5_ours/section_02_peak_f1525.png)

![section_03_peak_f2532.png](frames/body_check_002__A497__v5_ours/section_03_peak_f2532.png)

### `walk_forward_loop_001__A021.bvh`  (4 sections, sat-score=16)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 312 | 320 | 320 | 0.07 | saturation | 4 | 0.091 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_roll_joint |
| 1018 | 1026 | 1026 | 0.07 | saturation | 4 | 0.091 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_roll_joint |
| 452 | 458 | 458 | 0.06 | saturation | 4 | 0.090 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_roll_joint |
| 863 | 868 | 868 | 0.05 | saturation | 4 | 0.089 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_roll_joint |

![flag_pelvis_z_f593.png](frames/walk_forward_loop_001__A021__v5_ours/flag_pelvis_z_f593.png)

![flag_wrist_ang_vel_f491.png](frames/walk_forward_loop_001__A021__v5_ours/flag_wrist_ang_vel_f491.png)

![flag_saturated_dof_f314.png](frames/walk_forward_loop_001__A021__v5_ours/flag_saturated_dof_f314.png)

![section_00_peak_f320.png](frames/walk_forward_loop_001__A021__v5_ours/section_00_peak_f320.png)

![section_01_peak_f1026.png](frames/walk_forward_loop_001__A021__v5_ours/section_01_peak_f1026.png)

![section_02_peak_f458.png](frames/walk_forward_loop_001__A021__v5_ours/section_02_peak_f458.png)

![section_03_peak_f868.png](frames/walk_forward_loop_001__A021__v5_ours/section_03_peak_f868.png)

### `look_over_fence_270_R_001__A463_M.bvh`  (3 sections, sat-score=15)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 171 | 604 | 265 | 3.62 | saturation | 6 | 0.079 | left_shoulder_roll_joint, left_elbow_joint, right_ankle_pitch_joint |
| 100 | 110 | 104 | 0.09 | saturation | 5 | 0.070 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |
| 610 | 618 | 610 | 0.07 | saturation | 4 | 0.074 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f652.png](frames/look_over_fence_270_R_001__A463_M__v5_ours/flag_pelvis_z_f652.png)

![flag_wrist_ang_vel_f720.png](frames/look_over_fence_270_R_001__A463_M__v5_ours/flag_wrist_ang_vel_f720.png)

![flag_saturated_dof_f282.png](frames/look_over_fence_270_R_001__A463_M__v5_ours/flag_saturated_dof_f282.png)

![section_00_peak_f265.png](frames/look_over_fence_270_R_001__A463_M__v5_ours/section_00_peak_f265.png)

![section_01_peak_f104.png](frames/look_over_fence_270_R_001__A463_M__v5_ours/section_01_peak_f104.png)

![section_02_peak_f610.png](frames/look_over_fence_270_R_001__A463_M__v5_ours/section_02_peak_f610.png)

### `body_check_001__A361.bvh`  (3 sections, sat-score=13)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 1350 | 1374 | 1359 | 0.21 | saturation | 5 | 0.074 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 2445 | 2512 | 2447 | 0.57 | saturation | 4 | 0.091 | left_elbow_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 1985 | 2014 | 1985 | 0.25 | saturation | 4 | 0.100 | right_wrist_pitch_joint, right_elbow_joint, left_shoulder_roll_joint |

![flag_pelvis_z_f1988.png](frames/body_check_001__A361__v5_ours/flag_pelvis_z_f1988.png)

![flag_wrist_ang_vel_f575.png](frames/body_check_001__A361__v5_ours/flag_wrist_ang_vel_f575.png)

![flag_saturated_dof_f1351.png](frames/body_check_001__A361__v5_ours/flag_saturated_dof_f1351.png)

![section_00_peak_f1359.png](frames/body_check_001__A361__v5_ours/section_00_peak_f1359.png)

![section_01_peak_f2447.png](frames/body_check_001__A361__v5_ours/section_01_peak_f2447.png)

![section_02_peak_f1985.png](frames/body_check_001__A361__v5_ours/section_02_peak_f1985.png)

### `body_check_001__A461.bvh`  (3 sections, sat-score=12)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 1763 | 1883 | 1835 | 1.01 | saturation | 4 | 0.081 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 2918 | 2990 | 2990 | 0.61 | saturation | 4 | 0.110 | right_wrist_pitch_joint, right_elbow_joint, left_shoulder_roll_joint |
| 1982 | 1991 | 1991 | 0.08 | saturation | 4 | 0.076 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f2939.png](frames/body_check_001__A461__v5_ours/flag_pelvis_z_f2939.png)

![flag_wrist_ang_vel_f1859.png](frames/body_check_001__A461__v5_ours/flag_wrist_ang_vel_f1859.png)

![flag_saturated_dof_f1801.png](frames/body_check_001__A461__v5_ours/flag_saturated_dof_f1801.png)

![section_00_peak_f1835.png](frames/body_check_001__A461__v5_ours/section_00_peak_f1835.png)

![section_01_peak_f2990.png](frames/body_check_001__A461__v5_ours/section_01_peak_f2990.png)

![section_02_peak_f1991.png](frames/body_check_001__A461__v5_ours/section_02_peak_f1991.png)

### `small_light_two_hands_front_low_to_behind_low_R_001__A517_M.bvh`  (1 sections, sat-score=11)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 147 | 351 | 261 | 1.71 | saturation | 11 | 0.139 | right_shoulder_roll_joint, right_knee_joint, left_hip_roll_joint |

![flag_pelvis_z_f343.png](frames/small_light_two_hands_front_low_to_behind_low_R_001__A517_M__v5_ours/flag_pelvis_z_f343.png)

![flag_wrist_ang_vel_f330.png](frames/small_light_two_hands_front_low_to_behind_low_R_001__A517_M__v5_ours/flag_wrist_ang_vel_f330.png)

![flag_saturated_dof_f189.png](frames/small_light_two_hands_front_low_to_behind_low_R_001__A517_M__v5_ours/flag_saturated_dof_f189.png)

![section_00_peak_f261.png](frames/small_light_two_hands_front_low_to_behind_low_R_001__A517_M__v5_ours/section_00_peak_f261.png)

### `dance_basic_slide_360_R_001__A306.bvh`  (2 sections, sat-score=11)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 60 | 152 | 72 | 0.78 | saturation | 7 | 0.087 | left_shoulder_roll_joint, left_elbow_joint, right_wrist_roll_joint |
| 164 | 173 | 173 | 0.08 | saturation | 4 | 0.081 | right_wrist_roll_joint, left_elbow_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f99.png](frames/dance_basic_slide_360_R_001__A306__v5_ours/flag_pelvis_z_f99.png)

![flag_wrist_ang_vel_f203.png](frames/dance_basic_slide_360_R_001__A306__v5_ours/flag_wrist_ang_vel_f203.png)

![flag_saturated_dof_f73.png](frames/dance_basic_slide_360_R_001__A306__v5_ours/flag_saturated_dof_f73.png)

![section_00_peak_f72.png](frames/dance_basic_slide_360_R_001__A306__v5_ours/section_00_peak_f72.png)

![section_01_peak_f173.png](frames/dance_basic_slide_360_R_001__A306__v5_ours/section_01_peak_f173.png)

### `small_light_two_hands_walk_ff_loop_180_R_001__A505_M.bvh`  (2 sections, sat-score=11)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 4 | 71 | 63 | 0.57 | saturation | 6 | 0.080 | left_shoulder_roll_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |
| 74 | 352 | 352 | 2.33 | saturation | 5 | 0.083 | right_wrist_pitch_joint, left_wrist_pitch_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f174.png](frames/small_light_two_hands_walk_ff_loop_180_R_001__A505_M__v5_ours/flag_pelvis_z_f174.png)

![flag_wrist_ang_vel_f330.png](frames/small_light_two_hands_walk_ff_loop_180_R_001__A505_M__v5_ours/flag_wrist_ang_vel_f330.png)

![flag_saturated_dof_f60.png](frames/small_light_two_hands_walk_ff_loop_180_R_001__A505_M__v5_ours/flag_saturated_dof_f60.png)

![section_00_peak_f63.png](frames/small_light_two_hands_walk_ff_loop_180_R_001__A505_M__v5_ours/section_00_peak_f63.png)

![section_01_peak_f352.png](frames/small_light_two_hands_walk_ff_loop_180_R_001__A505_M__v5_ours/section_01_peak_f352.png)

### `medium_heavy_two_hands_front_medium_to_front_high_R_001__A521.bvh`  (2 sections, sat-score=10)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 64 | 238 | 154 | 1.46 | saturation | 6 | 0.093 | right_wrist_roll_joint, left_shoulder_roll_joint, right_shoulder_roll_joint |
| 425 | 446 | 446 | 0.18 | saturation | 4 | 0.082 | right_wrist_roll_joint, left_wrist_roll_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f176.png](frames/medium_heavy_two_hands_front_medium_to_front_high_R_001__A521__v5_ours/flag_pelvis_z_f176.png)

![flag_wrist_ang_vel_f364.png](frames/medium_heavy_two_hands_front_medium_to_front_high_R_001__A521__v5_ours/flag_wrist_ang_vel_f364.png)

![flag_saturated_dof_f156.png](frames/medium_heavy_two_hands_front_medium_to_front_high_R_001__A521__v5_ours/flag_saturated_dof_f156.png)

![section_00_peak_f154.png](frames/medium_heavy_two_hands_front_medium_to_front_high_R_001__A521__v5_ours/section_00_peak_f154.png)

![section_01_peak_f446.png](frames/medium_heavy_two_hands_front_medium_to_front_high_R_001__A521__v5_ours/section_01_peak_f446.png)

### `medium_light_two_hands_right_side_high_to_right_side_medium_R_001__A527.bvh`  (2 sections, sat-score=8)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 333 | 354 | 354 | 0.18 | saturation | 4 | 0.083 | right_wrist_roll_joint, left_wrist_roll_joint, right_shoulder_roll_joint |
| 287 | 301 | 301 | 0.12 | saturation | 4 | 0.076 | right_wrist_roll_joint, left_wrist_roll_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f361.png](frames/medium_light_two_hands_right_side_high_to_right_side_medium_R_001__A527__v5_ours/flag_pelvis_z_f361.png)

![flag_wrist_ang_vel_f367.png](frames/medium_light_two_hands_right_side_high_to_right_side_medium_R_001__A527__v5_ours/flag_wrist_ang_vel_f367.png)

![flag_saturated_dof_f333.png](frames/medium_light_two_hands_right_side_high_to_right_side_medium_R_001__A527__v5_ours/flag_saturated_dof_f333.png)

![section_00_peak_f354.png](frames/medium_light_two_hands_right_side_high_to_right_side_medium_R_001__A527__v5_ours/section_00_peak_f354.png)

![section_01_peak_f301.png](frames/medium_light_two_hands_right_side_high_to_right_side_medium_R_001__A527__v5_ours/section_01_peak_f301.png)

### `Loop_Backward_Walk_001__A020.bvh`  (1 sections, sat-score=4)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 675 | 691 | 691 | 0.14 | saturation | 4 | 0.098 | right_shoulder_roll_joint, left_shoulder_roll_joint, left_ankle_roll_joint |

![flag_pelvis_z_f206.png](frames/Loop_Backward_Walk_001__A020__v5_ours/flag_pelvis_z_f206.png)

![flag_wrist_ang_vel_f791.png](frames/Loop_Backward_Walk_001__A020__v5_ours/flag_wrist_ang_vel_f791.png)

![flag_saturated_dof_f688.png](frames/Loop_Backward_Walk_001__A020__v5_ours/flag_saturated_dof_f688.png)

![section_00_peak_f691.png](frames/Loop_Backward_Walk_001__A020__v5_ours/section_00_peak_f691.png)

### `neutral_idle_turn_360_002__A077_M.bvh`  (1 sections, sat-score=4)

| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |
|---:|---:|---:|---:|:---|---:|---:|:---|
| 344 | 374 | 344 | 0.26 | saturation | 4 | 0.081 | left_shoulder_roll_joint, left_elbow_joint, right_shoulder_roll_joint |

![flag_pelvis_z_f288.png](frames/neutral_idle_turn_360_002__A077_M__v5_ours/flag_pelvis_z_f288.png)

![flag_wrist_ang_vel_f409.png](frames/neutral_idle_turn_360_002__A077_M__v5_ours/flag_wrist_ang_vel_f409.png)

![flag_saturated_dof_f345.png](frames/neutral_idle_turn_360_002__A077_M__v5_ours/flag_saturated_dof_f345.png)

![section_00_peak_f344.png](frames/neutral_idle_turn_360_002__A077_M__v5_ours/section_00_peak_f344.png)

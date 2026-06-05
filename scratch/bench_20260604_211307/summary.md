# A/B benchmark — aggregate metrics

Corpus: **49 clips** across categories: dances, loco-manipulation, locowalk, standing-manipulation

**Configs compared in this run** (labelled by their distinguishing
parameters):

| config | summary |
|---|---|
| **h=1.40**              | `model_height=1.40`, `Hips.r_weight=2`, `Hand.t/r=1.0/0.1`, *no* wrist smoothing |
| **h=1.70+wrist_smooth** | `model_height=1.70`, `Hips.r_weight=10`, `Hand.t/r=2.0/0.2`, wrist_pitch/roll smoothing at 0.1 |

## Visual summary

<img src="frames/charts/event_counts.png"    width="520" alt="event_counts">

<img src="frames/charts/twist_severity.png"  width="440" alt="twist_severity">

<img src="frames/charts/events_by_group.png" width="600" alt="events_by_group">

## Headline numbers (aggregate over the full corpus)

| metric | h=1.40 | h=1.70+wrist_smooth | better |
|---|---:|---:|:---|
| Saturation overall % (lower=better) (mean) | 9.42 | 6.75 | **h=1.70+wrist_smooth** (28% better) |
| FK position residual (m) (lower=better) (mean) | 0.0870 | 0.0542 | **h=1.70+wrist_smooth** (38% better) |
| FK position residual p95 (m) (median) | 0.0969 | 0.0652 | **h=1.70+wrist_smooth** (33% better) |
| Smoothness deg/s^2 (lower=smoother) (mean) | 1357.0 | 1604.5 | **h=1.40** (15% better) |
| Hip yaw wobble (deg/s) (mean) | 65.8 | 65.6 | **h=1.70+wrist_smooth** (0% better) |
| Waist yaw wobble (deg/s) (mean) | 38.8 | 35.2 | **h=1.70+wrist_smooth** (9% better) |
| Shoulder yaw |mean| (deg) (mean) | 41.87 | 39.32 | **h=1.70+wrist_smooth** (6% better) |
| L hand-pelvis dist (m) (mean) | 0.280 | 0.324 | **h=1.70+wrist_smooth** (16% better) |
| R hand-pelvis dist (m) (mean) | 0.281 | 0.324 | **h=1.70+wrist_smooth** (15% better) |
| Root XY travel (m) (mean) | 2.04 | 2.45 | **h=1.70+wrist_smooth** (20% better) |

## Single-DOF slam / pin / twist event count (lower=better) — primary IK-failure metric

Per-joint detector:
- `slam`  — joint at its hard stop with peak |vel| ≥ 200 deg/s (basin hop)
- `pin`   — pinned for ≥ 8 frames without a sudden transition (workspace exhaustion)
- `twist` — wrist_yaw (palm-twist DOF) sustained at `|angle| ≥ 80°` for ≥ 8 frames,
            i.e. palm-twist past natural human pronation/supination range, even though
            the mechanical ±146° limit is never touched (slam/pin miss this entirely).
            Severity `severe` once `|wrist_yaw| ≥ 100°`.

Rest-side asymmetric limits filtered out. See `limit_events.md` for details + side-by-side renders.

| | h=1.40 | h=1.70+wrist_smooth |
|---|---:|---:|
| Total slam events across corpus | 486 | 399 |
| Total pin events across corpus  | 302 | 178 |
| Total twist events across corpus | **110** | 38 |
| &nbsp;&nbsp;of which `severe` (\|wrist_yaw\| ≥ 100°) | **17** | 3 |
| Peak \|wrist_yaw\| reached (deg) | **130.7** | 114.1 |
| Longest twist run (frames @ 120fps) | **541 (4.5 s)** | 260 (2.2 s) |
| Peak \|vel\| p95 (deg/s)         | 1386 | 872 |
| Peak \|vel\| max (deg/s)         | 11109 | 4608 |
| **Wrist events (slam + pin)**    | **364** | **0** |
| **Wrist events (twist excursion)** | **110** | **38** |
| Elbow events (slam + pin)       | 87 | 87 |
| Shoulder events (slam + pin)    | 56 | 64 |

Two headlines coexist here:

1. **Wrist slam/pin (palm-sideways + palm-flex DOFs):** h=1.70+wrist_smooth's wrist smoothing +
   `r_weight` reduces these to **zero** across the entire 49-clip corpus, while h=1.40 has 364.
2. **Wrist twist (palm-twist DOF):** the picture flips — both configs have twist excursions, but
   h=1.40 has **2.9× more events overall** and **5.7× more `severe`** ones (palm twisted past
   natural human range). h=1.40 also reaches more extreme angles (peak 130.7° vs 114.1°) and
   sustains them longer (541-frame run vs 260-frame run).

In other words: h=1.40 buys "no wrist_pitch/roll pins" by spending the wrist_yaw DOF much further
past natural human range than h=1.70+wrist_smooth ever does. The slam/pin detector alone
masks this trade-off because wrist_yaw never touches its mechanical ±146° hard stop.

## Joint-saturation cluster count (envelope clearance, NOT IK failure)

These were originally labelled "IK failure sections" but are better
understood as joint-saturation clusters — contiguous frames where ≥ 4 of
31 DOFs sit within 5° of any limit. The X2's asymmetric ranges
(`shoulder_roll`, `wrist_pitch`) make this fire on benign poses; many
"sections" render to perfectly normal-looking frames. The FK-residual
trigger (≥ 0.18 m) never fired in the corpus. See `ik_failures.md` for
the legacy listing.

| | h=1.40 | h=1.70+wrist_smooth |
|---|---:|---:|
| Total sections across corpus | 303 | 125 |
| Mean sections per clip | 6.18 | 2.55 |

## Saturation % by joint group (lower=better)

| group | h=1.40 | h=1.70+wrist_smooth |
|---|---:|---:|
| hip | 1.43 | 1.67 |
| knee | 0.63 | 0.58 |
| ankle | 7.54 | 10.17 |
| waist | 0.00 | 0.00 |
| shoulder | 27.36 | 23.56 |
| elbow | 12.18 | 7.93 |
| wrist | 10.57 | 0.00 |
| head | 0.00 | 0.00 |

## Per-clip headline metrics

| clip | tier | category | frames | h=1.40.sat% | h=1.40.fkres_m | h=1.40.shYaw | h=1.70+wrist_smooth.sat% | h=1.70+wrist_smooth.fkres_m | h=1.70+wrist_smooth.shYaw |
|---|---|---|---|---:|---:|---:|---:|---:|---:|
| walk_forward_loop_001__A021.bvh | anchor | locowalk | 0 | 6.8 | 0.0949 | 44.6 | 7.0 | 0.0496 | 45.7 |
| dance_basic_turn_v1_360_R_loop_fast_004__A322.bvh | hip | dances | 584 | 11.1 | 0.0805 | 32.3 | 8.2 | 0.0459 | 32.0 |
| dance_latino_chase_mambo_pivot_R_001__A313.bvh | hip | dances | 1117 | 15.2 | 0.0831 | 29.2 | 5.9 | 0.0517 | 22.3 |
| Loop_Backward_Walk_001__A020.bvh | hip | loco-manipulation | 15566 | 6.8 | 0.0984 | 46.7 | 7.2 | 0.0529 | 46.0 |
| walk_ff_start_360_004__A146.bvh | hip | loco-manipulation | 707 | 8.1 | 0.0929 | 37.3 | 6.8 | 0.0516 | 35.7 |
| walk_forward_relax_003__A005.bvh | hip | locowalk | 3853 | 8.3 | 0.0967 | 33.0 | 7.2 | 0.0559 | 30.8 |
| walking_random_direction_R_001__A431_M.bvh | hip | locowalk | 3366 | 8.5 | 0.0882 | 37.2 | 6.2 | 0.0472 | 33.6 |
| medium_big_heavy_one_hand_walk_ff_start_360_R_001__A506_M.bvh | hip | standing-manipulation | 1001 | 7.8 | 0.0835 | 26.1 | 6.0 | 0.0468 | 25.4 |
| medium_light_two_hands_right_side_high_to_right_side_medium_R_001__A527.bvh | hip | standing-manipulation | 673 | 7.7 | 0.0834 | 39.9 | 6.5 | 0.0486 | 40.7 |
| dance_retro_jazz_cross_step_180_R_001__A314.bvh | wrist | dances | 727 | 10.3 | 0.0838 | 30.5 | 5.5 | 0.0529 | 31.5 |
| victory_dance_asarahe_180_R_004__A324.bvh | wrist | dances | 1095 | 8.4 | 0.0831 | 43.6 | 4.2 | 0.0554 | 33.9 |
| medium_heavy_two_hands_front_medium_to_front_high_R_001__A521.bvh | wrist | loco-manipulation | 551 | 8.4 | 0.0797 | 38.2 | 4.3 | 0.0512 | 36.4 |
| neutral_dancecard_object_interact_003__A541.bvh | wrist | loco-manipulation | 16481 | 7.8 | 0.0944 | 36.8 | 6.6 | 0.0510 | 37.9 |
| body_check_001__A461.bvh | wrist | locowalk | 4493 | 7.4 | 0.0854 | 52.1 | 7.5 | 0.0547 | 48.3 |
| body_check_002__A496.bvh | wrist | locowalk | 3370 | 8.0 | 0.0894 | 52.8 | 4.4 | 0.0672 | 51.9 |
| body_check_001__A251.bvh | wrist | standing-manipulation | 3499 | 7.8 | 0.0851 | 62.8 | 5.3 | 0.0585 | 57.6 |
| eat_hotdog_standing_fail_R_001__A456_M.bvh | wrist | standing-manipulation | 1622 | 11.1 | 0.0821 | 19.9 | 5.6 | 0.0413 | 17.3 |
| dance_retro_twist_step_variation_R_fast_002__A314.bvh | leg | dances | 769 | 11.9 | 0.0815 | 33.1 | 4.3 | 0.0609 | 31.1 |
| body_check_002__A497.bvh | leg | loco-manipulation | 3308 | 6.5 | 0.0851 | 49.9 | 4.2 | 0.0602 | 49.8 |
| walk_big_dog_ff_315_loop_R_002__A495.bvh | leg | locowalk | 403 | 18.4 | 0.0752 | 30.7 | 10.9 | 0.0594 | 26.1 |
| body_check_001__A527_M.bvh | leg | standing-manipulation | 5103 | 7.0 | 0.0800 | 55.7 | 5.0 | 0.0460 | 50.4 |
| dance_retro_disco_finger_sequence_R_fast_002__A314.bvh | shoulder | dances | 875 | 13.6 | 0.0854 | 57.9 | 11.2 | 0.0527 | 58.4 |
| painful_stand_on_turn_walk_ff_360_start_R_001__A461_M.bvh | shoulder | loco-manipulation | 1820 | 8.9 | 0.0813 | 32.9 | 7.8 | 0.0524 | 27.8 |
| body_check_004__A444.bvh | shoulder | locowalk | 4618 | 8.0 | 0.0787 | 57.5 | 4.0 | 0.0449 | 52.5 |
| body_check_001__A361.bvh | shoulder | standing-manipulation | 2695 | 6.3 | 0.0866 | 57.7 | 4.7 | 0.0616 | 56.2 |
| small_light_two_hands_front_low_to_behind_low_R_001__A517_M.bvh | pelvis | standing-manipulation | 576 | 13.0 | 0.1098 | 57.9 | 9.8 | 0.1006 | 55.5 |
| victory_dance_loser_jump_180_R_003__A308.bvh | ankle | dances | 484 | 8.5 | 0.0745 | 34.0 | 6.8 | 0.0584 | 28.3 |
| idle_turn_360_R_004__A237.bvh | ankle | locowalk | 655 | 6.7 | 0.0935 | 30.9 | 6.5 | 0.0472 | 22.7 |
| big_light_two_hands_right_side_high_to_behind_high_R_001__A525.bvh | ankle | standing-manipulation | 1227 | 7.6 | 0.0850 | 28.0 | 6.5 | 0.0573 | 44.0 |
| dance_basic_slide_360_R_001__A306.bvh | random | dances | 391 | 9.2 | 0.0837 | 37.5 | 6.9 | 0.0465 | 33.9 |
| dance_hiphop_funky_guitar_R_fast_001__A319.bvh | random | dances | 675 | 13.1 | 0.0865 | 28.1 | 8.5 | 0.0609 | 28.3 |
| dance_jazz_hands_002__A467.bvh | random | dances | 763 | 14.0 | 0.0842 | 31.9 | 7.3 | 0.0534 | 36.0 |
| dance_vouge_butterfly_step_180_R_fast_002__A319.bvh | random | dances | 542 | 11.4 | 0.0846 | 19.0 | 6.6 | 0.0680 | 21.1 |
| dance_western_horse_step_with_leg_undercut_R_loop_002__A324.bvh | random | dances | 1107 | 10.0 | 0.0913 | 69.7 | 6.2 | 0.0601 | 68.6 |
| Turn_Start_Walk_0045_001__A018.bvh | random | loco-manipulation | 10905 | 6.5 | 0.0936 | 41.3 | 6.5 | 0.0459 | 39.7 |
| neutral_idle_turn_360_002__A077_M.bvh | random | loco-manipulation | 610 | 7.1 | 0.0851 | 46.1 | 6.7 | 0.0437 | 43.1 |
| painful_stand_on_walk_ff_360_R_001__A461.bvh | random | loco-manipulation | 2173 | 9.7 | 0.0822 | 28.5 | 7.9 | 0.0520 | 21.4 |
| turn_walk_270_R_002__A256_M.bvh | random | loco-manipulation | 696 | 6.6 | 0.0936 | 39.4 | 6.0 | 0.0537 | 33.3 |
| walk_ff_start_270_R_slow_001__A443.bvh | random | loco-manipulation | 1297 | 10.5 | 0.0949 | 36.8 | 11.7 | 0.0522 | 32.5 |
| idle_turn_360_003__A177_M.bvh | random | locowalk | 699 | 6.7 | 0.0888 | 46.7 | 6.7 | 0.0428 | 44.9 |
| turn_start_walk_135_004__A038.bvh | random | locowalk | 638 | 7.1 | 0.1058 | 39.2 | 7.7 | 0.0650 | 39.9 |
| walk_arc_cw_start_003__A067.bvh | random | locowalk | 468 | 9.2 | 0.0921 | 29.5 | 6.5 | 0.0498 | 19.3 |
| walk_ff_loop_225_005__A059_M.bvh | random | locowalk | 383 | 13.2 | 0.0988 | 39.4 | 12.0 | 0.0586 | 34.7 |
| walk_ff_start_360_R_003__A267.bvh | random | locowalk | 852 | 9.2 | 0.0914 | 43.6 | 7.3 | 0.0516 | 42.8 |
| body_check_001__A381_M.bvh | random | standing-manipulation | 3858 | 8.5 | 0.0854 | 55.5 | 4.4 | 0.0630 | 50.3 |
| body_check_002__A492_M.bvh | random | standing-manipulation | 3324 | 8.6 | 0.0862 | 48.6 | 4.8 | 0.0626 | 46.8 |
| crossed_arms_idle_R_001__A457_M.bvh | random | standing-manipulation | 1139 | 9.7 | 0.0793 | 93.7 | 6.5 | 0.0478 | 85.7 |
| look_over_fence_270_R_001__A463_M.bvh | random | standing-manipulation | 872 | 11.4 | 0.0776 | 38.0 | 8.2 | 0.0500 | 33.4 |
| small_light_two_hands_walk_ff_loop_180_R_001__A505_M.bvh | random | standing-manipulation | 353 | 13.9 | 0.0792 | 50.0 | 6.7 | 0.0431 | 40.7 |

"""Aggregate metrics computed per (clip, config).

Seven aggregate metrics:
  1. saturation_pct      — mean % frames within `near_deg` of either limit.
                          Reported per group (hip/wrist/shoulder/elbow/...) and overall.
  2. fk_pos_residual_m   — mean Euclidean error between IK target position and
                          FK-achieved body position (per effector + overall).
  3. smoothness_deg_s2   — mean magnitude of second-difference of joint angles,
                          divided by dt^2 (deg/s^2). Smaller = smoother.
  4. hand_pelvis_dist_m  — mean Euclidean distance from left/right wrist_roll
                          to pelvis (cm above = positive).
  5. root_travel_m       — total horizontal path length of the root (XY plane).
  6. hip_yaw_wobble_dps  — RMS of frame-to-frame change in hip yaw (deg/s).
                          Captures the "twisting" symptom directly.
  7. shoulder_yaw_mean_abs_deg — mean abs of shoulder yaw joints (deg). High
                                 values indicate IK is compensating with yaw.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict

import numpy as np

from scripts.bench.joint_limits import (
    CSV_HEADER, JOINT_NAMES, JOINT_LIMITS_DEG, JOINT_GROUPS, csv_col,
)
from scripts.bench import kinematics


NEAR_DEG_DEFAULT = 5.0


@dataclass
class ClipMetrics:
    clip: str
    config: str
    num_frames: int
    fps: float

    # Saturation: per-group % near either limit
    saturation_pct: Dict[str, float] = field(default_factory=dict)
    saturation_pct_overall: float = 0.0
    # Per-joint near%, for the per-joint table in the report
    per_joint_near_pct: Dict[str, float] = field(default_factory=dict)
    # Per-joint excursion beyond hard limit (deg) — should be 0 with clamper on
    per_joint_overshoot_deg: Dict[str, float] = field(default_factory=dict)

    # FK position residual (m)
    fk_pos_residual_m_mean: float = 0.0
    fk_pos_residual_m_p95: float = 0.0
    fk_pos_residual_m_max: float = 0.0
    per_effector_pos_residual_m_mean: Dict[str, float] = field(default_factory=dict)

    # Smoothness (deg/s^2)
    smoothness_deg_s2_mean: float = 0.0
    smoothness_deg_s2_max: float = 0.0
    per_joint_smoothness_deg_s2_mean: Dict[str, float] = field(default_factory=dict)

    # Hand-vs-pelvis distance (m)
    left_hand_pelvis_dist_m_mean: float = 0.0
    right_hand_pelvis_dist_m_mean: float = 0.0
    left_hand_pelvis_dist_m_std: float = 0.0
    right_hand_pelvis_dist_m_std: float = 0.0

    # Root horizontal travel (m)
    root_travel_m: float = 0.0

    # Hip yaw wobble: RMS of d/dt of hip yaw (deg/s)
    hip_yaw_wobble_dps: float = 0.0
    # And of the waist yaw (controls torso twist relative to hips)
    waist_yaw_wobble_dps: float = 0.0

    # Shoulder yaw absolute mean (deg) — captures arm twist compensation
    shoulder_yaw_mean_abs_deg: float = 0.0
    shoulder_yaw_max_abs_deg: float = 0.0


# ---------------------------------------------------------------------------
# Saturation
# ---------------------------------------------------------------------------

def _saturation_per_joint(joint_angles_deg: np.ndarray, near_deg: float) -> Dict[str, dict]:
    """joint_angles_deg: (N, 31) array of joint angles in deg."""
    out: Dict[str, dict] = {}
    for k, jn in enumerate(JOINT_NAMES):
        vals = joint_angles_deg[:, k]
        lo_d, hi_d = JOINT_LIMITS_DEG[jn]
        near_lo = float((vals <= lo_d + near_deg).mean()) * 100.0
        near_hi = float((vals >= hi_d - near_deg).mean()) * 100.0
        overshoot_lo = float(max(0.0, lo_d - vals.min()))
        overshoot_hi = float(max(0.0, vals.max() - hi_d))
        out[jn] = dict(
            near_lo=near_lo,
            near_hi=near_hi,
            near=near_lo + near_hi,
            overshoot=max(overshoot_lo, overshoot_hi),
        )
    return out


# ---------------------------------------------------------------------------
# Top-level compute
# ---------------------------------------------------------------------------

def compute_clip_metrics(
    clip_name: str,
    config_name: str,
    csv_path: Path,
    bvh_path: Path,
    retarget_config_path: Path,
    fps: float = 120.0,
    near_deg: float = NEAR_DEG_DEFAULT,
    max_frames: int | None = None,
) -> tuple[ClipMetrics, dict]:
    """Compute all metrics for one (clip, config). Returns (metrics, per_frame).

    per_frame: dict with arrays of shape (num_frames,) for downstream
    per-frame flagging and IK section detection.
    """
    csv_data = kinematics.load_csv_matrix(csv_path)
    num_frames = int(csv_data.shape[0])
    dt = 1.0 / fps

    # Joint angles in deg (columns 7..7+31)
    joint_angles_deg = csv_data[:, 7:7 + len(JOINT_NAMES)]

    # 1) saturation
    per_joint_sat = _saturation_per_joint(joint_angles_deg, near_deg)
    sat_by_group: Dict[str, float] = {}
    for grp, joints in JOINT_GROUPS.items():
        vals = [per_joint_sat[j]["near"] for j in joints if j in per_joint_sat]
        sat_by_group[grp] = float(np.mean(vals)) if vals else 0.0
    overall_sat = float(np.mean([per_joint_sat[j]["near"] for j in JOINT_NAMES]))

    # 3) smoothness — finite difference of joint angles
    if num_frames >= 3:
        d2 = np.diff(joint_angles_deg, n=2, axis=0)  # (N-2, 31)
        accel = d2 / (dt * dt)  # deg/s^2
        accel_mag = np.abs(accel)
        per_joint_smoothness = {jn: float(accel_mag[:, k].mean()) for k, jn in enumerate(JOINT_NAMES)}
        smoothness_mean = float(accel_mag.mean())
        smoothness_max = float(accel_mag.max())
    else:
        per_joint_smoothness = {jn: 0.0 for jn in JOINT_NAMES}
        smoothness_mean = 0.0
        smoothness_max = 0.0

    # 5) root travel — XY plane
    root_xy_cm = csv_data[:, 1:3]
    diffs_xy = np.diff(root_xy_cm, axis=0)
    seg = np.linalg.norm(diffs_xy, axis=1) * 0.01
    root_travel_m = float(seg.sum())

    # 6) hip yaw wobble — rms of d(hip_yaw)/dt at the *hip yaw joint* level.
    # We use left_hip_yaw_joint and right_hip_yaw_joint as a proxy. For "true"
    # pelvis yaw we'd also want waist_yaw.
    def _wobble(col_name: str) -> float:
        c = csv_col(col_name)
        vals = csv_data[:, c]
        d = np.diff(vals) / dt  # deg/s
        return float(np.sqrt(np.mean(d * d))) if len(d) else 0.0

    hip_yaw_wobble = 0.5 * (_wobble("left_hip_yaw_joint") + _wobble("right_hip_yaw_joint"))
    waist_yaw_wobble = _wobble("waist_yaw_joint")

    # 7) shoulder yaw |mean| — IK compensation indicator
    l_sh_yaw = joint_angles_deg[:, JOINT_NAMES.index("left_shoulder_yaw_joint")]
    r_sh_yaw = joint_angles_deg[:, JOINT_NAMES.index("right_shoulder_yaw_joint")]
    sh_yaw_mean_abs = float(0.5 * (np.abs(l_sh_yaw).mean() + np.abs(r_sh_yaw).mean()))
    sh_yaw_max_abs = float(max(np.abs(l_sh_yaw).max(), np.abs(r_sh_yaw).max()))

    # 2 + 4) FK and hand-vs-pelvis distances. Both need the MuJoCo data driver.
    import mujoco  # local import to avoid penalising bench startup
    model, data = kinematics.load_x2_mj_model()
    csv_to_qpos = kinematics.build_csv_to_qpos_map(model)
    pelvis_id = kinematics.body_name_to_id(model, "pelvis")
    lwrist_id = kinematics.body_name_to_id(model, "left_wrist_roll_link")
    rwrist_id = kinematics.body_name_to_id(model, "right_wrist_roll_link")

    # Compute IK targets for the BVH so we have positional ground truth.
    targets, eff_order, cfg = kinematics.compute_targets(
        bvh_path, retarget_config_path, max_frames=max_frames,
    )

    ik_map = cfg["ik_map"]  # eff_name -> {t_body, r_body, t_weight, r_weight}

    # We only score residuals for effectors present in ik_map (LeftToeBase/
    # RightToeBase are added by the scaler but aren't IK targets).
    effs_for_residual = [n for n in eff_order if n in ik_map]
    eff_body_ids = {
        n: kinematics.body_name_to_id(model, ik_map[n]["t_body"]) for n in effs_for_residual
    }

    n_targets = min(num_frames, *(targets[n].shape[0] for n in effs_for_residual))

    per_eff_dists: Dict[str, list[float]] = {n: [] for n in effs_for_residual}
    saturated_dof_count = np.zeros(num_frames, dtype=np.int32)
    left_hand_dist = np.zeros(num_frames, dtype=np.float64)
    right_hand_dist = np.zeros(num_frames, dtype=np.float64)
    fk_residual_per_frame = np.zeros(num_frames, dtype=np.float64)  # mean over effectors

    # Wrist angular velocity proxy (per-frame, deg/s, max of the 6 wrist joints)
    wrist_joint_idx = [JOINT_NAMES.index(j) for j in JOINT_GROUPS["wrist"]]
    wrist_ang_vel_dps = np.zeros(num_frames, dtype=np.float64)

    # Saturated-DOF count threshold per joint: |val - mid| >= 0.95 * half_range
    # OR: vals within near_deg of a limit. Use the simpler "near" criterion.
    near = near_deg
    joint_lo_hi_deg = np.array([JOINT_LIMITS_DEG[j] for j in JOINT_NAMES])  # (31,2)

    pelvis_z_arr = np.zeros(num_frames, dtype=np.float64)

    for f in range(num_frames):
        row = csv_data[f]
        kinematics.apply_csv_row(model, data, row, csv_to_qpos)
        pelvis_pos = np.array(data.xpos[pelvis_id], dtype=np.float64)
        pelvis_z_arr[f] = float(pelvis_pos[2])

        # Hand distances
        lwrist_pos = np.array(data.xpos[lwrist_id], dtype=np.float64)
        rwrist_pos = np.array(data.xpos[rwrist_id], dtype=np.float64)
        left_hand_dist[f] = float(np.linalg.norm(lwrist_pos - pelvis_pos))
        right_hand_dist[f] = float(np.linalg.norm(rwrist_pos - pelvis_pos))

        # Per-effector residual (only if target frame exists)
        if f < n_targets:
            frame_residual_sum = 0.0
            count = 0
            for n in effs_for_residual:
                target_pos = targets[n][f, :3]
                bid = eff_body_ids[n]
                achieved_pos = np.array(data.xpos[bid], dtype=np.float64)
                d = float(np.linalg.norm(target_pos - achieved_pos))
                per_eff_dists[n].append(d)
                frame_residual_sum += d
                count += 1
            fk_residual_per_frame[f] = frame_residual_sum / max(count, 1)

        # Saturated DOFs
        vals = joint_angles_deg[f]
        near_lo_mask = vals <= (joint_lo_hi_deg[:, 0] + near)
        near_hi_mask = vals >= (joint_lo_hi_deg[:, 1] - near)
        saturated_dof_count[f] = int((near_lo_mask | near_hi_mask).sum())

        # Wrist angular velocity (deg / s)
        if f >= 1:
            w_now = joint_angles_deg[f, wrist_joint_idx]
            w_prev = joint_angles_deg[f - 1, wrist_joint_idx]
            wrist_ang_vel_dps[f] = float(np.max(np.abs(w_now - w_prev)) / dt)

    fk_mean = float(fk_residual_per_frame[:n_targets].mean()) if n_targets > 0 else 0.0
    fk_p95 = float(np.percentile(fk_residual_per_frame[:n_targets], 95)) if n_targets > 0 else 0.0
    fk_max = float(fk_residual_per_frame[:n_targets].max()) if n_targets > 0 else 0.0

    per_eff_residual_mean = {n: (float(np.mean(per_eff_dists[n])) if per_eff_dists[n] else 0.0)
                             for n in effs_for_residual}

    m = ClipMetrics(
        clip=clip_name,
        config=config_name,
        num_frames=num_frames,
        fps=fps,

        saturation_pct=sat_by_group,
        saturation_pct_overall=overall_sat,
        per_joint_near_pct={j: per_joint_sat[j]["near"] for j in JOINT_NAMES},
        per_joint_overshoot_deg={j: per_joint_sat[j]["overshoot"] for j in JOINT_NAMES},

        fk_pos_residual_m_mean=fk_mean,
        fk_pos_residual_m_p95=fk_p95,
        fk_pos_residual_m_max=fk_max,
        per_effector_pos_residual_m_mean=per_eff_residual_mean,

        smoothness_deg_s2_mean=smoothness_mean,
        smoothness_deg_s2_max=smoothness_max,
        per_joint_smoothness_deg_s2_mean=per_joint_smoothness,

        left_hand_pelvis_dist_m_mean=float(left_hand_dist.mean()),
        right_hand_pelvis_dist_m_mean=float(right_hand_dist.mean()),
        left_hand_pelvis_dist_m_std=float(left_hand_dist.std()),
        right_hand_pelvis_dist_m_std=float(right_hand_dist.std()),

        root_travel_m=root_travel_m,

        hip_yaw_wobble_dps=hip_yaw_wobble,
        waist_yaw_wobble_dps=waist_yaw_wobble,

        shoulder_yaw_mean_abs_deg=sh_yaw_mean_abs,
        shoulder_yaw_max_abs_deg=sh_yaw_max_abs,
    )

    per_frame = dict(
        pelvis_z=pelvis_z_arr,
        left_hand_dist=left_hand_dist,
        right_hand_dist=right_hand_dist,
        fk_residual=fk_residual_per_frame,
        saturated_dof_count=saturated_dof_count,
        wrist_ang_vel_dps=wrist_ang_vel_dps,
    )

    return m, per_frame

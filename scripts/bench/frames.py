"""Per-frame failure flagging + contiguous IK failure section detection.

Three independent flag categories (each produces a top-N list of frames):

  1. pelvis_z_bobbing : second derivative magnitude of pelvis Z (m/s^2)
  2. wrist_ang_vel    : max wrist joint angular velocity across the 6 wrist DOFs (deg/s)
  3. saturated_dof    : count of DOFs within `near_deg` of either limit

IK failure sections are contiguous frame ranges where either:
  - saturated_dof_count >= sat_count_threshold, OR
  - fk_residual_m >= fk_residual_threshold
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import List

import numpy as np


@dataclass
class FrameFlag:
    category: str          # "pelvis_z" / "wrist_ang_vel" / "saturated_dof"
    frame_idx: int
    value: float
    note: str = ""


@dataclass
class IKSection:
    start_frame: int
    end_frame: int         # inclusive
    peak_frame: int
    peak_saturated_dofs: int
    peak_fk_residual_m: float
    duration_frames: int
    duration_s: float
    dominant_joints: list[str]
    trigger: str           # "saturation" / "residual" / "both"


def _top_k(values: np.ndarray, k: int, descending: bool = True) -> list[int]:
    if len(values) == 0 or k <= 0:
        return []
    order = np.argsort(values)
    if descending:
        order = order[::-1]
    return order[:k].tolist()


def flag_top_frames(
    per_frame: dict,
    fps: float,
    k_per_category: int = 1,
) -> List[FrameFlag]:
    """Pick top-K worst frames per category."""
    flags: List[FrameFlag] = []

    # Pelvis Z bobbing: |d^2(pelvis_z)/dt^2|
    pz = per_frame["pelvis_z"]
    if len(pz) >= 3:
        d2 = np.diff(pz, n=2) / (1.0 / fps) ** 2
        d2_mag = np.abs(d2)
        # d2 is offset by 1 — index i in d2 corresponds to original frame i+1
        for k in _top_k(d2_mag, k_per_category):
            flags.append(FrameFlag(
                category="pelvis_z",
                frame_idx=int(k + 1),
                value=float(d2_mag[k]),
                note=f"|d2 pelvis_z|={d2_mag[k]:.2f} m/s^2",
            ))

    # Wrist angular velocity
    wv = per_frame["wrist_ang_vel_dps"]
    for k in _top_k(wv, k_per_category):
        flags.append(FrameFlag(
            category="wrist_ang_vel",
            frame_idx=int(k),
            value=float(wv[k]),
            note=f"wrist_ang_vel={wv[k]:.1f} deg/s",
        ))

    # Saturated DOF count
    sd = per_frame["saturated_dof_count"]
    for k in _top_k(sd, k_per_category):
        flags.append(FrameFlag(
            category="saturated_dof",
            frame_idx=int(k),
            value=float(sd[k]),
            note=f"saturated_dofs={int(sd[k])}",
        ))

    return flags


def detect_ik_failure_sections(
    per_frame: dict,
    joint_angles_deg: np.ndarray,
    joint_names: list[str],
    joint_limits_deg: list[tuple[float, float]],
    fps: float,
    sat_threshold: int = 4,
    fk_residual_threshold_m: float = 0.18,
    min_duration_frames: int = 5,
    near_deg: float = 5.0,
    max_sections: int = 20,
) -> List[IKSection]:
    """Find contiguous frame ranges where IK is failing.

    A frame counts as "failing" if it has either >= sat_threshold saturated
    DOFs or fk_residual >= threshold. Contiguous runs of failing frames longer
    than `min_duration_frames` become an IKSection. Dominant joints for each
    section are picked by % of section frames during which that joint was
    saturated.
    """
    sat_count = per_frame["saturated_dof_count"]
    fk_res = per_frame["fk_residual"]

    fail_mask = (sat_count >= sat_threshold) | (fk_res >= fk_residual_threshold_m)

    sections: List[IKSection] = []
    in_run = False
    run_start = 0
    for f in range(len(fail_mask)):
        if fail_mask[f] and not in_run:
            in_run = True
            run_start = f
        elif not fail_mask[f] and in_run:
            in_run = False
            _emit(sections, run_start, f - 1, per_frame, joint_angles_deg, joint_names,
                  joint_limits_deg, fps, sat_threshold, fk_residual_threshold_m,
                  min_duration_frames, near_deg)
    if in_run:
        _emit(sections, run_start, len(fail_mask) - 1, per_frame, joint_angles_deg,
              joint_names, joint_limits_deg, fps, sat_threshold,
              fk_residual_threshold_m, min_duration_frames, near_deg)

    # Sort sections by peak severity (peak_saturated_dofs first, then duration)
    sections.sort(key=lambda s: (s.peak_saturated_dofs, s.duration_frames), reverse=True)
    return sections[:max_sections]


def _emit(out, start, end, per_frame, joint_angles_deg, joint_names, joint_limits_deg,
          fps, sat_threshold, fk_thr, min_dur, near_deg):
    if (end - start + 1) < min_dur:
        return

    sat_count = per_frame["saturated_dof_count"]
    fk_res = per_frame["fk_residual"]

    section_sat = sat_count[start:end + 1]
    section_fk = fk_res[start:end + 1]

    # Peak frame: maximize (sat_count, fk_residual)
    composite = section_sat.astype(np.float64) + section_fk / (fk_thr + 1e-9)
    peak_local = int(np.argmax(composite))
    peak_frame = start + peak_local
    peak_sat = int(section_sat[peak_local])
    peak_fk = float(section_fk[peak_local])

    # Dominant joints: count how often each joint is "saturated" in the section
    section_angles = joint_angles_deg[start:end + 1]
    joint_lo = np.array([lh[0] for lh in joint_limits_deg])
    joint_hi = np.array([lh[1] for lh in joint_limits_deg])
    near_mask = (section_angles <= (joint_lo + near_deg)) | (section_angles >= (joint_hi - near_deg))
    per_joint_freq = near_mask.mean(axis=0)
    dominant_idx = np.argsort(per_joint_freq)[::-1]
    dominant = [joint_names[i] for i in dominant_idx if per_joint_freq[i] >= 0.3][:5]

    triggers = []
    if peak_sat >= sat_threshold:
        triggers.append("saturation")
    if peak_fk >= fk_thr:
        triggers.append("residual")
    trigger = "+".join(triggers) if triggers else "none"

    out.append(IKSection(
        start_frame=int(start),
        end_frame=int(end),
        peak_frame=int(peak_frame),
        peak_saturated_dofs=peak_sat,
        peak_fk_residual_m=peak_fk,
        duration_frames=int(end - start + 1),
        duration_s=float((end - start + 1) / fps),
        dominant_joints=dominant,
        trigger=trigger,
    ))


def to_dicts(items: list) -> list[dict]:
    return [asdict(i) for i in items]

"""X2 Ultra joint limits and CSV column layout.

Limits transcribed from soma_retargeter/robot_assets/agibot_x2_ultra/x2_ultra.xml
(joint range="lower upper" in radians). Single source of truth shared by every
component of the bench so we never disagree on what counts as "saturated".
"""

from __future__ import annotations

import math


JOINT_LIMITS_RAD: dict[str, tuple[float, float]] = {
    "left_hip_pitch_joint":       (-2.704,  2.556),
    "left_hip_roll_joint":        (-0.235,  2.906),
    "left_hip_yaw_joint":         (-1.684,  3.430),
    "left_knee_joint":            ( 0.000,  2.4073),
    "left_ankle_pitch_joint":     (-0.803,  0.453),
    "left_ankle_roll_joint":      (-0.262,  0.262),
    "right_hip_pitch_joint":      (-2.704,  2.556),
    "right_hip_roll_joint":       (-2.906,  0.235),
    "right_hip_yaw_joint":        (-3.430,  1.684),
    "right_knee_joint":           ( 0.000,  2.4073),
    "right_ankle_pitch_joint":    (-0.803,  0.453),
    "right_ankle_roll_joint":     (-0.2625, 0.2625),
    "waist_yaw_joint":            (-1.5708, 1.5708),
    "waist_pitch_joint":          (-0.785,  0.785),
    "waist_roll_joint":           (-0.785,  0.785),
    "left_shoulder_pitch_joint":  (-3.08,   2.04),
    "left_shoulder_roll_joint":   (-0.061,  2.993),
    "left_shoulder_yaw_joint":    (-2.556,  2.556),
    "left_elbow_joint":           (-2.3556, 0.0),
    "left_wrist_yaw_joint":       (-2.556,  2.556),
    "left_wrist_pitch_joint":     (-0.558,  0.558),
    "left_wrist_roll_joint":      (-1.571,  0.724),
    "right_shoulder_pitch_joint": (-3.08,   2.04),
    "right_shoulder_roll_joint":  (-2.993,  0.061),
    "right_shoulder_yaw_joint":   (-2.556,  2.556),
    "right_elbow_joint":          (-2.3556, 0.0),
    "right_wrist_yaw_joint":      (-2.556,  2.556),
    "right_wrist_pitch_joint":    (-0.558,  0.558),
    "right_wrist_roll_joint":     (-0.724,  1.571),
    "head_yaw_joint":             (-2.0944, 2.0944),
    "head_pitch_joint":           (-0.523,  0.785),
}

JOINT_LIMITS_DEG: dict[str, tuple[float, float]] = {
    j: (math.degrees(lo), math.degrees(hi)) for j, (lo, hi) in JOINT_LIMITS_RAD.items()
}


# 31-DOF X2 Ultra CSV column order (matches AgibotX2Ultra31DOF_CSVConfig in
# soma_retargeter/assets/csv.py). All values are degrees in this CSV — root
# translation is the only non-deg portion (cm).
CSV_HEADER: list[str] = [
    "Frame",
    "root_translateX", "root_translateY", "root_translateZ",
    "root_rotateX", "root_rotateY", "root_rotateZ",
    "left_hip_pitch_joint_dof", "left_hip_roll_joint_dof", "left_hip_yaw_joint_dof",
    "left_knee_joint_dof", "left_ankle_pitch_joint_dof", "left_ankle_roll_joint_dof",
    "right_hip_pitch_joint_dof", "right_hip_roll_joint_dof", "right_hip_yaw_joint_dof",
    "right_knee_joint_dof", "right_ankle_pitch_joint_dof", "right_ankle_roll_joint_dof",
    "waist_yaw_joint_dof", "waist_pitch_joint_dof", "waist_roll_joint_dof",
    "left_shoulder_pitch_joint_dof", "left_shoulder_roll_joint_dof",
    "left_shoulder_yaw_joint_dof", "left_elbow_joint_dof",
    "left_wrist_yaw_joint_dof", "left_wrist_pitch_joint_dof", "left_wrist_roll_joint_dof",
    "right_shoulder_pitch_joint_dof", "right_shoulder_roll_joint_dof",
    "right_shoulder_yaw_joint_dof", "right_elbow_joint_dof",
    "right_wrist_yaw_joint_dof", "right_wrist_pitch_joint_dof", "right_wrist_roll_joint_dof",
    "head_yaw_joint_dof", "head_pitch_joint_dof",
]

JOINT_NAMES: list[str] = [c.replace("_dof", "") for c in CSV_HEADER[7:]]
NUM_DOFS: int = len(JOINT_NAMES)


# Coarse joint groupings used by the aggregate report.
JOINT_GROUPS: dict[str, list[str]] = {
    "hip":       [j for j in JOINT_NAMES if "hip" in j],
    "knee":      [j for j in JOINT_NAMES if "knee" in j],
    "ankle":     [j for j in JOINT_NAMES if "ankle" in j],
    "waist":     [j for j in JOINT_NAMES if "waist" in j],
    "shoulder":  [j for j in JOINT_NAMES if "shoulder" in j],
    "elbow":     [j for j in JOINT_NAMES if "elbow" in j],
    "wrist":     [j for j in JOINT_NAMES if "wrist" in j],
    "head":      [j for j in JOINT_NAMES if "head" in j],
}


def csv_col(joint_name: str) -> int:
    """Return the CSV column index of a joint by name.

    Accepts either the base joint name ("left_hip_yaw_joint") or the dof-suffixed
    form ("left_hip_yaw_joint_dof"). Also matches the floating-base columns
    ("root_translateX", "root_rotateY", "Frame", etc.) by exact header name.
    """
    base = joint_name[:-4] if joint_name.endswith("_dof") else joint_name
    for i, c in enumerate(CSV_HEADER):
        c_base = c[:-4] if c.endswith("_dof") else c
        if c_base == base:
            return i
    raise KeyError(joint_name)

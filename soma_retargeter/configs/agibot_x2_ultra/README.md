# Agibot X2 Ultra -- SOMA Retargeter Integration

## Overview

This directory contains the retargeting configuration for the **Agibot X2 Ultra** (31-DOF humanoid) as a target robot in the SOMA Retargeter pipeline. The X2 Ultra joins the Unitree G1 (29-DOF) as a supported retarget target.

## Files

| File | Purpose |
|------|---------|
| `soma_to_x2_ultra_retargeter_config.json` | IK map, weights, and pipeline settings |
| `soma_to_x2_ultra_scaler_config.json` | Human-to-robot scaling, joint offsets, and joint hierarchy |
| `x2_ultra_feet_stabilizer_config.json` | Foot contact effectors for ground stabilization |

## Result

<table>
<tr>
<td align="center"><b>Dance 1</b></td>
<td align="center"><b>Dance 2</b></td>
</tr>
<tr>
<td><img src="soma-retargetter-agibot-x2.gif" width="400" /></td>
<td><img src="soma-retargetter-agibot-x2-dance2.gif" width="400" /></td>
</tr>
</table>

## Arm Retargeting: Challenges and Solution

### Problem

When initially porting the G1 configuration to the X2 Ultra, the retargeted arm motion was completely wrong -- arms flipped over the robot's head instead of swinging naturally at its sides during a walking motion.

### Root Cause Analysis

The X2 Ultra and G1 have different arm kinematic structures. While the **shoulder joints** share the same axis conventions (pitch=Y, roll=X, yaw=Z) and the **leg chains** are structurally identical between both robots, two critical differences exist in the arm chain:

1. **Elbow joint axis**: G1 uses an **X-axis** elbow, while X2 uses a **Y-axis** elbow.
2. **Wrist chain order**: G1 orders its wrist joints as roll-pitch-yaw; X2 orders them as yaw-pitch-roll.

The SOMA retargeter uses weighted IK objectives for both position and rotation matching. The G1 configuration assigns high rotation weights to the forearm (`r_weight: 1.0`) and hand (`r_weight: 1.2`) effectors. These rotation targets encode world-frame orientations calibrated for G1's X-axis elbow.

When the same rotation targets are applied to the X2's Y-axis elbow, the IK solver cannot achieve the requested orientation through normal joint configurations. Instead, it finds a solution by driving the shoulder pitch joint to its extreme limit (~-3 radians), flipping the entire arm over the head -- a valid solution from the solver's perspective that minimizes rotation error but produces physically nonsensical motion.

### Debugging Approach

1. **Visual comparison**: Launched MuJoCo viewer (fixed base, no gravity) alongside the SOMA viewer to interactively explore joint limits and compare against retargeted output.
2. **Joint limit verification**: Confirmed that Newton's IK operates with correct joint limits by reading them directly from MuJoCo (not through Newton's body-indexed arrays, which have an off-by-one due to free joint coordinate vs DOF indexing).
3. **Axis comparison**: Systematically compared all joint axes and limits between G1 and X2 from MuJoCo ground truth, identifying the elbow axis difference as the structural root cause.
4. **Shoulder pitch tracing**: Verified in MuJoCo that the shoulder pitch range [-3.08, 2.04] rad allows the arm to go fully over the head at the negative extreme -- exactly matching the SOMA retarget's erroneous output.

### Solution

Reduced the IK rotation weights for forearm and hand effectors in `soma_to_x2_ultra_retargeter_config.json`:

| Effector | G1 `r_weight` | X2 `r_weight` |
|----------|---------------|---------------|
| LeftForeArm | 1.0 | 0.1 |
| LeftHand | 1.2 | 0.1 |
| RightForeArm | 1.0 | 0.1 |
| RightHand | 1.2 | 0.1 |

With reduced rotation weights, the position objectives dominate the arm IK. Since position tracking is axis-agnostic (it only cares about endpoint locations, not body-frame orientations), the solver finds natural arm configurations regardless of the elbow axis convention. The shoulder and arm position weights (`t_weight: 1.5` and `t_weight: 1.0`) remain unchanged and provide accurate arm tracking.

### Key Takeaway

When onboarding a new robot with different joint axis conventions, the IK rotation weights must be tuned per-robot. High rotation weights that work for one kinematic structure can cause catastrophic IK failures on another, even when the robots are superficially similar humanoids. Position-based tracking is more robust across different kinematic chains.

## Wrist Offset Tuning

### Problem

After fixing arm retargeting, the X2 Ultra's wrist joints were stuck at their physical limits during most of the motion. Both wrists appeared bent backward or locked in unnatural poses.

### Root Cause

The X2 Ultra has much tighter wrist joint limits than the G1:

| Joint | G1 range | X2 range |
|-------|----------|----------|
| Wrist pitch | ±92.5° | ±32° |
| Wrist roll | ±45° | ±28° |

The `joint_offsets` for `LeftHand` and `RightHand` in `soma_to_x2_ultra_scaler_config.json` were copied from the G1 configuration. These offsets define the transform from SOMA joint frames to robot link frames. Since the G1 offsets assumed wider wrist ranges, they placed the X2's neutral wrist pose outside its physical limits, causing the `JointLimitClamper` to hard-clamp the joints every frame.

### Debugging Approach

1. **CSV analysis**: Exported retargeted joint data and plotted human (BVH) vs robot (CSV) wrist positions, confirming 100% limit saturation on wrist pitch and roll.
2. **Empirical axis sweep**: Systematically applied large Euler rotations (±30° to ±90°) on individual axes of the Hand offset quaternion and observed which robot wrist joints responded. This revealed the mapping: Y-rotation in the offset controls robot wrist pitch, X-rotation controls wrist roll.
3. **Automated optimization was attempted** (`scipy.optimize.differential_evolution`) but small offset changes could not overcome the hard clamping, and numerically optimal solutions often looked visually unnatural.

### Solution

Applied targeted Euler corrections on top of the original G1 Hand offset quaternions to center the neutral wrist pose within X2's tighter limits:

- **LeftHand**: Y (pitch) = -55°, X (roll) = +80° applied on top of G1 offset
- **RightHand**: Y (pitch) = -55°, X (roll) = +80° applied on top of G1 offset

This preserved the natural orientation of the G1 offsets while shifting the operating point into X2's valid range.

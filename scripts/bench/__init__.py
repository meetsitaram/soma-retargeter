"""Retargeter config A/B benchmark utilities (untracked scratch territory).

Modules:
- joint_limits : X2 Ultra MJCF joint limits and 31-DOF CSV column layout
- corpus       : BVH motion-statistics screening + 4-tier corpus assembly
- retarget     : invoke SOMA->X2 retargeting with a config override
- metrics      : seven aggregate metrics per (clip, config)
- frames       : per-frame failure flagging + contiguous IK failure sections
- render       : embodiment-agnostic IK target renderer (PNG/strip/interactive)
- aggregate    : summary.md, ik_failures.md, REPORT.md, metrics.json builders
"""

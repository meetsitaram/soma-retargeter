"""Single-DOF slam/pin event detection from retargeted CSVs.

This module replaces the misleading "IK failure section" metric (which fired
on any cluster of >= 4 of 31 DOFs near a limit) with a per-joint detector that
matches the actual failure mode we observe in the viewer: a single joint
slammed to its hard stop, often with a basin-hop jerk.

Definitions (all per joint, per (clip, config)):

  - `pinned[f]`  = |angle[f] - lo| <= pin_tol_deg or |hi - angle[f]| <= pin_tol_deg
  - `vel[f]`     = (angle[f] - angle[f-1]) * fps                  (deg/s)

A contiguous run of `pinned` frames (allowing internal gaps of up to
`max_gap_frames`) becomes an event:

  - if max |vel| over [run_start-1 .. run_end+1] >= slam_vel_dps  -> "slam"
    (sudden snap to / off the stop; characteristic of an IK basin hop)
  - elif duration >= min_pin_frames                                -> "pin"
    (sustained against the stop without a sudden transition;
     characteristic of target unreachable / workspace exhaustion)
  - else discard (short pin without high velocity = noise)

Operates purely on the existing `csvs/*.csv` produced by the bench — no
retargeting re-run, no MuJoCo, no BVH load.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, asdict, field
from pathlib import Path
from typing import Iterable

import numpy as np

from scripts.bench.joint_limits import JOINT_NAMES, JOINT_LIMITS_DEG


# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------

DEFAULTS = dict(
    pin_tol_deg=1.0,
    slam_vel_dps=200.0,
    min_pin_frames=8,
    max_gap_frames=2,
    fps=120.0,
    # If a candidate event's max |vel| over the run never exceeds this, the
    # joint never actually moved during the "event" — it's the rest pose
    # sitting against an asymmetric hardware limit (shoulder_roll left lower
    # bound at -3.5 deg, right upper bound at +3.5 deg). Drop these from the
    # event list; they distort headline counts and aren't IK failures.
    min_motion_dps=10.0,
    # Joints whose 0-rad rest pose is within `rest_side_tol_deg` of a limit
    # (shoulder_roll, knee, elbow) generate "rest pose pins" on that side
    # whenever the arm/leg is in its neutral configuration. Drop pin events
    # on the rest side for those joints; slam events on either side are kept.
    rest_side_tol_deg=5.0,
    # ---- Palm-twist excursion detector (wrist_yaw only) ----
    # The wrist_yaw mechanical limit is ±146°, but natural human pronation /
    # supination tops out at ~80° on each side. A retargeter that drives the
    # palm twist past 80° produces visibly non-anatomical poses even though
    # the hard limit is never touched (so slam/pin detectors miss it entirely).
    # We flag sustained runs above two thresholds as a `twist` event, and
    # tag each with a severity:
    #   - "warn"   :  threshold_deg <= |angle| < severe_threshold_deg
    #   - "severe" :  |angle| >= severe_threshold_deg
    twist_threshold_deg=80.0,
    twist_severe_threshold_deg=100.0,
    twist_min_frames=8,
    twist_max_gap_frames=4,
)


# Joints whose threshold-based "palm twist" detector should run. Other joints
# either have small ranges (`wrist_pitch`, `wrist_roll`) or different anatomy
# (knees, hips) so the 80° threshold is meaningless for them.
TWIST_JOINTS = ("left_wrist_yaw_joint", "right_wrist_yaw_joint")


def _is_rest_side(lo_deg: float, hi_deg: float, side: str, tol_deg: float) -> bool:
    """Whether 0-rad rest pose sits within `tol_deg` of the given limit side.

    Joints like shoulder_roll (-3.5..+171.4), elbow (-135..0), knee (0..138) all
    have one limit coincident with the rest pose. Pin events on that side are
    just the joint sitting at rest, not IK failures.
    """
    if side == "lo":
        return abs(lo_deg) <= tol_deg
    return abs(hi_deg) <= tol_deg


# ---------------------------------------------------------------------------
# Dataclasses
# ---------------------------------------------------------------------------

@dataclass
class LimitEvent:
    clip: str
    config: str
    joint: str
    side: str           # "lo" | "hi"
    category: str       # "slam" | "pin"
    start_f: int
    end_f: int          # inclusive
    peak_f: int
    peak_vel_dps: float
    peak_angle_deg: float
    limit_deg: float
    duration_frames: int
    duration_s: float


# ---------------------------------------------------------------------------
# Core detector (one joint)
# ---------------------------------------------------------------------------

def _runs_with_gap(mask: np.ndarray, max_gap: int) -> list[tuple[int, int]]:
    """Return list of (start, end_inclusive) where mask is True, allowing
    short gaps of up to `max_gap` False frames within a run."""
    runs: list[tuple[int, int]] = []
    n = len(mask)
    i = 0
    while i < n:
        if not mask[i]:
            i += 1
            continue
        s = i
        e = i
        j = i + 1
        while j < n:
            if mask[j]:
                e = j
                j += 1
            else:
                # look-ahead: how many consecutive False frames here?
                k = j
                while k < n and not mask[k]:
                    k += 1
                gap = k - j
                if gap <= max_gap and k < n:
                    j = k
                    continue
                else:
                    break
        runs.append((s, e))
        i = e + 1
    return runs


def detect_events_for_joint(
    angles_deg: np.ndarray,         # shape (N,)
    joint_name: str,
    lo: float, hi: float,
    fps: float,
    *,
    pin_tol_deg: float,
    slam_vel_dps: float,
    min_pin_frames: int,
    max_gap_frames: int,
    min_motion_dps: float = DEFAULTS["min_motion_dps"],
    rest_side_tol_deg: float = DEFAULTS["rest_side_tol_deg"],
) -> list[dict]:
    """Detect slam/pin events for a single joint over a clip.

    Returns a list of dict (without clip/config — caller fills those)."""
    n = len(angles_deg)
    if n < 2:
        return []

    pin_lo = angles_deg <= (lo + pin_tol_deg)
    pin_hi = angles_deg >= (hi - pin_tol_deg)
    pin_any = pin_lo | pin_hi
    if not np.any(pin_any):
        return []

    # Frame-to-frame velocity (deg/s). Pad first entry with 0.
    vel = np.empty(n, dtype=np.float64)
    vel[0] = 0.0
    vel[1:] = (angles_deg[1:] - angles_deg[:-1]) * fps

    out: list[dict] = []
    for s, e in _runs_with_gap(pin_any, max_gap=max_gap_frames):
        # Side determined by majority pinning during the run.
        run_lo = int(pin_lo[s:e + 1].sum())
        run_hi = int(pin_hi[s:e + 1].sum())
        side = "lo" if run_lo >= run_hi else "hi"
        limit_val = lo if side == "lo" else hi

        # Peak velocity considers the entry/exit transitions too.
        v_lo = max(0, s - 1)
        v_hi = min(n - 1, e + 1)
        peak_local = v_lo + int(np.argmax(np.abs(vel[v_lo:v_hi + 1])))
        peak_vel = float(vel[peak_local])

        # Peak frame within the run: prefer the most extreme angle relative to
        # the side, so the rendered frame shows the joint actually at its stop.
        run_angles = angles_deg[s:e + 1]
        if side == "lo":
            peak_f = s + int(np.argmin(run_angles))
            peak_angle = float(run_angles.min())
        else:
            peak_f = s + int(np.argmax(run_angles))
            peak_angle = float(run_angles.max())

        duration = e - s + 1
        if abs(peak_vel) >= slam_vel_dps:
            category = "slam"
        elif duration >= min_pin_frames:
            # Pins require some motion across the run; otherwise the joint
            # is simply parked at its hardware rest-pose limit (asymmetric
            # shoulder_roll, etc.) and is not an IK failure.
            run_motion_dps = float(np.max(np.abs(vel[s:e + 1]))) if e > s else 0.0
            if run_motion_dps < min_motion_dps:
                continue
            # Skip pins on the rest side of asymmetric joints — that's the
            # neutral arm/leg posture, not workspace exhaustion.
            if _is_rest_side(lo, hi, side, rest_side_tol_deg):
                continue
            category = "pin"
        else:
            continue

        out.append(dict(
            joint=joint_name,
            side=side,
            category=category,
            start_f=int(s),
            end_f=int(e),
            peak_f=int(peak_f),
            peak_vel_dps=float(peak_vel),
            peak_angle_deg=float(peak_angle),
            limit_deg=float(limit_val),
            duration_frames=int(duration),
            duration_s=float(duration / fps),
        ))

    return out


def detect_twist_events_for_joint(
    angles_deg: np.ndarray,
    joint_name: str,
    fps: float,
    *,
    threshold_deg: float,
    severe_threshold_deg: float,
    min_frames: int,
    max_gap_frames: int,
) -> list[dict]:
    """Detect 'twist excursion' events on a wrist_yaw joint.

    The detector flags contiguous runs where the joint angle magnitude exceeds
    `threshold_deg` (default 80° — comfortable upper bound of human pronation /
    supination). This catches the failure mode where the IK drives the palm
    twist past natural human range to satisfy other constraints; the mechanical
    ±146° limit is never touched so the slam/pin detectors miss it entirely.

    Returns a list of dict (without `clip` / `config` — caller fills those)."""
    n = len(angles_deg)
    if n < 2:
        return []

    abs_a = np.abs(angles_deg)
    mask = abs_a >= threshold_deg
    if not np.any(mask):
        return []

    # Per-frame velocity (deg/s); padded with 0 at the start.
    vel = np.empty(n, dtype=np.float64)
    vel[0] = 0.0
    vel[1:] = (angles_deg[1:] - angles_deg[:-1]) * fps

    out: list[dict] = []
    for s, e in _runs_with_gap(mask, max_gap=max_gap_frames):
        duration = e - s + 1
        if duration < min_frames:
            continue
        run_abs = abs_a[s:e + 1]
        peak_local = int(np.argmax(run_abs))
        peak_f = s + peak_local
        peak_angle = float(angles_deg[peak_f])
        peak_abs = float(run_abs[peak_local])
        severity = "severe" if peak_abs >= severe_threshold_deg else "warn"
        # Sign of the peak indicates which way the palm is twisted (lo / hi only
        # so that the dataclass schema stays compatible with pin / slam).
        side = "lo" if peak_angle < 0 else "hi"
        peak_vel = float(vel[peak_f])
        out.append(dict(
            joint=joint_name,
            side=side,
            category="twist",
            severity=severity,
            start_f=int(s),
            end_f=int(e),
            peak_f=int(peak_f),
            peak_vel_dps=float(peak_vel),
            peak_angle_deg=float(peak_angle),
            peak_abs_deg=float(peak_abs),
            # `limit_deg` here is the threshold the run crossed, signed so it
            # matches the side of the excursion (negative side -> -threshold).
            limit_deg=float(-threshold_deg if peak_angle < 0 else threshold_deg),
            duration_frames=int(duration),
            duration_s=float(duration / fps),
        ))

    return out


def detect_events(
    angles_deg: np.ndarray,        # shape (N, num_joints)
    joint_names: list[str],
    joint_limits_deg: list[tuple[float, float]],
    fps: float,
    *,
    pin_tol_deg: float = DEFAULTS["pin_tol_deg"],
    slam_vel_dps: float = DEFAULTS["slam_vel_dps"],
    min_pin_frames: int = DEFAULTS["min_pin_frames"],
    max_gap_frames: int = DEFAULTS["max_gap_frames"],
    min_motion_dps: float = DEFAULTS["min_motion_dps"],
    rest_side_tol_deg: float = DEFAULTS["rest_side_tol_deg"],
    twist_threshold_deg: float = DEFAULTS["twist_threshold_deg"],
    twist_severe_threshold_deg: float = DEFAULTS["twist_severe_threshold_deg"],
    twist_min_frames: int = DEFAULTS["twist_min_frames"],
    twist_max_gap_frames: int = DEFAULTS["twist_max_gap_frames"],
) -> list[dict]:
    """Detect slam / pin / twist events across every joint for one clip."""
    out: list[dict] = []
    for k, jn in enumerate(joint_names):
        lo, hi = joint_limits_deg[k]
        out.extend(
            detect_events_for_joint(
                angles_deg[:, k], jn, lo, hi, fps,
                pin_tol_deg=pin_tol_deg,
                slam_vel_dps=slam_vel_dps,
                min_pin_frames=min_pin_frames,
                max_gap_frames=max_gap_frames,
                min_motion_dps=min_motion_dps,
                rest_side_tol_deg=rest_side_tol_deg,
            )
        )
        if jn in TWIST_JOINTS:
            out.extend(
                detect_twist_events_for_joint(
                    angles_deg[:, k], jn, fps,
                    threshold_deg=twist_threshold_deg,
                    severe_threshold_deg=twist_severe_threshold_deg,
                    min_frames=twist_min_frames,
                    max_gap_frames=twist_max_gap_frames,
                )
            )
    return out


# ---------------------------------------------------------------------------
# Bench-dir processor
# ---------------------------------------------------------------------------

def _parse_csv_name(p: Path, configs: list[str]) -> tuple[str, str] | None:
    """Recover (clip_stem, config) from a CSV filename of the form
    `<clip_stem>__<config>.csv`. Returns None if it doesn't match."""
    name = p.stem  # strip ".csv"
    for cfg in configs:
        suf = "__" + cfg
        if name.endswith(suf):
            return name[: -len(suf)], cfg
    return None


def process_bench_dir(
    bench_dir: Path,
    *,
    configs: list[str] | None = None,
    fps: float = DEFAULTS["fps"],
    pin_tol_deg: float = DEFAULTS["pin_tol_deg"],
    slam_vel_dps: float = DEFAULTS["slam_vel_dps"],
    min_pin_frames: int = DEFAULTS["min_pin_frames"],
    max_gap_frames: int = DEFAULTS["max_gap_frames"],
    min_motion_dps: float = DEFAULTS["min_motion_dps"],
    rest_side_tol_deg: float = DEFAULTS["rest_side_tol_deg"],
    twist_threshold_deg: float = DEFAULTS["twist_threshold_deg"],
    twist_severe_threshold_deg: float = DEFAULTS["twist_severe_threshold_deg"],
    twist_min_frames: int = DEFAULTS["twist_min_frames"],
    twist_max_gap_frames: int = DEFAULTS["twist_max_gap_frames"],
    verbose: bool = True,
) -> dict:
    """Scan {bench_dir}/csvs/*.csv and detect events for every (clip, config).

    Writes `{bench_dir}/limit_events.json` and returns the in-memory structure:

        {
          "params": {...},
          "configs": [...],
          "clips": {
              clip_stem: {
                  config_name: {
                      "num_frames": int,
                      "events": [LimitEvent-as-dict, ...]
                  }
              }
          }
        }
    """
    csv_dir = bench_dir / "csvs"
    if not csv_dir.is_dir():
        raise SystemExit(f"no csvs/ directory in {bench_dir}")

    # Infer configs from filenames if not given.
    csvs = sorted(csv_dir.glob("*.csv"))
    if configs is None:
        # Tail token after "__" before ".csv" — but clip stems contain "__" too.
        # Best heuristic: take last "__"-separated token from each filename.
        configs_found = set()
        for c in csvs:
            stem = c.stem
            if "__" in stem:
                configs_found.add(stem.rsplit("__", 1)[1])
        configs = sorted(configs_found)
    if not configs:
        raise SystemExit("could not infer configs from csv filenames; pass --configs")

    joint_limits_deg = [JOINT_LIMITS_DEG[j] for j in JOINT_NAMES]

    by_clip: dict[str, dict[str, dict]] = {}
    total_events = 0
    for i, csv in enumerate(csvs):
        parsed = _parse_csv_name(csv, configs)
        if parsed is None:
            continue
        clip_stem, cfg = parsed
        try:
            data = np.loadtxt(csv, delimiter=",", skiprows=1, dtype=np.float64)
        except Exception as e:
            if verbose:
                print(f"[WARN] {csv.name}: load failed: {e}")
            continue
        angles_deg = data[:, 7:7 + len(JOINT_NAMES)]
        events = detect_events(
            angles_deg, JOINT_NAMES, joint_limits_deg, fps,
            pin_tol_deg=pin_tol_deg,
            slam_vel_dps=slam_vel_dps,
            min_pin_frames=min_pin_frames,
            max_gap_frames=max_gap_frames,
            min_motion_dps=min_motion_dps,
            rest_side_tol_deg=rest_side_tol_deg,
            twist_threshold_deg=twist_threshold_deg,
            twist_severe_threshold_deg=twist_severe_threshold_deg,
            twist_min_frames=twist_min_frames,
            twist_max_gap_frames=twist_max_gap_frames,
        )
        for ev in events:
            ev["clip"] = clip_stem
            ev["config"] = cfg
        by_clip.setdefault(clip_stem, {})[cfg] = dict(
            num_frames=int(data.shape[0]),
            events=events,
        )
        total_events += len(events)
        if verbose and (i + 1) % 20 == 0:
            print(f"[INFO] processed {i+1}/{len(csvs)} csvs ({total_events} events so far)")

    out = dict(
        params=dict(
            fps=fps,
            pin_tol_deg=pin_tol_deg,
            slam_vel_dps=slam_vel_dps,
            min_pin_frames=min_pin_frames,
            max_gap_frames=max_gap_frames,
            min_motion_dps=min_motion_dps,
            rest_side_tol_deg=rest_side_tol_deg,
            twist_threshold_deg=twist_threshold_deg,
            twist_severe_threshold_deg=twist_severe_threshold_deg,
            twist_min_frames=twist_min_frames,
            twist_max_gap_frames=twist_max_gap_frames,
        ),
        configs=configs,
        clips=by_clip,
    )

    out_path = bench_dir / "limit_events.json"
    out_path.write_text(json.dumps(out, indent=2, default=float))
    if verbose:
        print(f"[OK] wrote {out_path}  ({total_events} total events across {len(by_clip)} clips)")
    return out


# ---------------------------------------------------------------------------
# Summary helpers
# ---------------------------------------------------------------------------

def aggregate_events(data: dict) -> dict:
    """Roll up event counts per config / category / joint / group.

    Input: output of `process_bench_dir` (or its on-disk JSON form).
    Returns a structured summary suitable for markdown rendering.
    """
    from scripts.bench.joint_limits import JOINT_GROUPS

    configs = data["configs"]
    per_cfg: dict[str, dict] = {}
    for cfg in configs:
        slam_events: list[dict] = []
        pin_events: list[dict] = []
        twist_events: list[dict] = []
        for clip, by_cfg in data["clips"].items():
            for ev in by_cfg.get(cfg, {}).get("events", []):
                cat = ev["category"]
                if cat == "slam":
                    slam_events.append(ev)
                elif cat == "pin":
                    pin_events.append(ev)
                elif cat == "twist":
                    twist_events.append(ev)

        twist_warn   = [e for e in twist_events if e.get("severity") == "warn"]
        twist_severe = [e for e in twist_events if e.get("severity") == "severe"]

        per_joint: dict[str, dict] = {}
        for ev in slam_events + pin_events + twist_events:
            jn = ev["joint"]
            per_joint.setdefault(jn, dict(slam=0, pin=0, twist=0))
            per_joint[jn][ev["category"]] += 1

        per_group = {g: dict(slam=0, pin=0, twist=0) for g in JOINT_GROUPS}
        for jn, counts in per_joint.items():
            for g, joints in JOINT_GROUPS.items():
                if jn in joints:
                    per_group[g]["slam"] += counts["slam"]
                    per_group[g]["pin"] += counts["pin"]
                    per_group[g]["twist"] += counts["twist"]
                    break

        # Total frames spent under the twist threshold + run statistics
        total_twist_frames = sum(int(e["duration_frames"]) for e in twist_events)
        peak_twist_abs = float(max((float(e.get("peak_abs_deg", abs(e["peak_angle_deg"]))) for e in twist_events), default=0.0))
        longest_twist_run = max((int(e["duration_frames"]) for e in twist_events), default=0)

        per_cfg[cfg] = dict(
            total_slam=len(slam_events),
            total_pin=len(pin_events),
            total_twist=len(twist_events),
            total_twist_warn=len(twist_warn),
            total_twist_severe=len(twist_severe),
            mean_slam_per_clip=len(slam_events) / max(len(data["clips"]), 1),
            mean_pin_per_clip=len(pin_events) / max(len(data["clips"]), 1),
            mean_twist_per_clip=len(twist_events) / max(len(data["clips"]), 1),
            twist_total_frames=int(total_twist_frames),
            twist_peak_abs_deg=peak_twist_abs,
            twist_longest_run_frames=int(longest_twist_run),
            per_joint=per_joint,
            per_group=per_group,
            peak_vel_p95=float(np.percentile([abs(e["peak_vel_dps"]) for e in slam_events], 95)) if slam_events else 0.0,
            peak_vel_max=float(max((abs(e["peak_vel_dps"]) for e in slam_events), default=0.0)),
        )

    return dict(
        configs=configs,
        per_cfg=per_cfg,
    )


# ---------------------------------------------------------------------------
# Markdown reports
# ---------------------------------------------------------------------------

def write_limit_events_md(
    out_path: Path,
    data: dict,
    *,
    sxs_pngs_by_event_id: dict[str, Path] | None = None,
    corpus_entries: list[dict] | None = None,
    top_events_per_cfg: int | None = None,
    pin_rank_key: str = "duration_frames",
    display_names: dict[str, str] | None = None,
    chart_pngs: dict[str, Path] | None = None,
) -> None:
    """Write `limit_events.md` with per-clip event tables + optional inline images.

    `sxs_pngs_by_event_id` maps a stable event id to its side-by-side PNG.
    `top_events_per_cfg` truncates each config's table to its top-N events
    by |peak_vel_dps|.
    """
    out_path.parent.mkdir(parents=True, exist_ok=True)

    configs = data["configs"]
    params = data["params"]
    by_name = {e["name"]: e for e in (corpus_entries or [])}
    display_names = display_names or {}

    def _disp(cfg: str) -> str:
        return display_names.get(cfg, cfg)

    lines: list[str] = []
    lines.append("# Single-DOF slam / pin events")
    lines.append("")
    lines.append("Per-joint detector that matches the actual failure mode observed in the viewer:")
    lines.append("a *single* joint slammed against its hard stop, optionally with a basin-hop jerk.")
    lines.append("")
    lines.append("**Definitions** (all per joint, per (clip, config)):")
    lines.append("")
    lines.append(f"- `pinned[f]` = `|angle[f] - limit| <= {params['pin_tol_deg']} deg`")
    lines.append(f"- `slam`      = pinned run with `max |vel| >= {params['slam_vel_dps']:.0f} deg/s` at entry or exit (sudden snap)")
    lines.append("  - *measured on the retargeted robot DOF only — the human BVH velocity does not enter this calc.*")
    lines.append("  - *most slams = IK basin flip (solver jumped between wrist/elbow branches, robot DOF snaps in 1–2 frames). Fast human motion can also drive a slam but is the minority case.*")
    lines.append(f"- `pin`       = pinned run that lasts `>= {params['min_pin_frames']}` frames without a sudden transition (workspace exhaustion)")
    lines.append(f"- `twist`     = sustained run where `|wrist_yaw| >= {params.get('twist_threshold_deg', 80.0):.0f} deg`")
    lines.append(f"                (palm-twist past natural human pronation/supination, even though the mechanical ±146° limit is never hit;")
    lines.append(f"                severity `severe` once `|wrist_yaw| >= {params.get('twist_severe_threshold_deg', 100.0):.0f} deg`)")
    lines.append(f"- short pins without a velocity transition are discarded as noise")
    lines.append("")
    lines.append(f"FPS: {params['fps']}. Internal gaps up to {params['max_gap_frames']} frames are tolerated within a slam/pin run, ")
    lines.append(f"and up to {params.get('twist_max_gap_frames', 4)} frames within a twist run.")
    lines.append("")
    lines.append("**X2 wrist-joint glossary** (the joint names below are mechanical labels and do *not* match anatomical pitch/roll/yaw):")
    lines.append("")
    lines.append("| joint name | mechanical range | anatomical action |")
    lines.append("|---|---|---|")
    lines.append("| `*_wrist_yaw_joint`   | ±146°        | **palm TWIST** (pronation / supination) |")
    lines.append("| `*_wrist_pitch_joint` | ±32°         | palm SIDE-TO-SIDE (radial / ulnar deviation) |")
    lines.append("| `*_wrist_roll_joint`  | −90° / +41°  | palm FORWARD/BACK BEND (flexion / extension) |")
    if sxs_pngs_by_event_id:
        lines.append("")
        lines.append("Side-by-side renders shown below were generated with `--limit-events-groups wrist,elbow,shoulder` ")
        lines.append("(top events on those groups only — leg/ankle slams are mostly foot-strike events, not IK failures, ")
        lines.append("so we skip rendering them). Each render is a 3-row strip: ")
        lines.append("**(top) human BVH stick figure**, **(middle/bottom) the two retargeter configs**, with columns ")
        lines.append("`peak-3 / peak / peak+3`. Red/green/blue axis tripods = the SOMA IK target pose; yellow lines = ")
        lines.append("the FK residual to the achieved body position.")
        if display_names:
            lines.append("")
            lines.append("**Config legend:**")
            for cfg in configs:
                lines.append(f"- `{cfg}` → **{_disp(cfg)}**")
    lines.append("")

    # Overall summary
    summary = aggregate_events(data)
    lines.append("## Overall event counts")
    lines.append("")
    lines.append("| | " + " | ".join(_disp(c) for c in configs) + " |")
    lines.append("|---|" + "|".join(["---:"] * len(configs)) + "|")
    lines.append("| Total slam events  | " + " | ".join(str(summary["per_cfg"][c]["total_slam"]) for c in configs) + " |")
    lines.append("| Total pin events   | " + " | ".join(str(summary["per_cfg"][c]["total_pin"]) for c in configs) + " |")
    lines.append("| Total twist events | " + " | ".join(str(summary["per_cfg"][c]["total_twist"]) for c in configs) + " |")
    lines.append("| &nbsp;&nbsp;of which severe (|twist| ≥ "
                 + f"{params.get('twist_severe_threshold_deg', 100.0):.0f}°) | "
                 + " | ".join(str(summary["per_cfg"][c]["total_twist_severe"]) for c in configs) + " |")
    lines.append("| Mean slam / clip  | " + " | ".join(f"{summary['per_cfg'][c]['mean_slam_per_clip']:.2f}" for c in configs) + " |")
    lines.append("| Mean pin / clip   | " + " | ".join(f"{summary['per_cfg'][c]['mean_pin_per_clip']:.2f}" for c in configs) + " |")
    lines.append("| Mean twist / clip | " + " | ".join(f"{summary['per_cfg'][c]['mean_twist_per_clip']:.2f}" for c in configs) + " |")
    lines.append("| Peak |twist| (deg)| " + " | ".join(f"{summary['per_cfg'][c]['twist_peak_abs_deg']:.1f}" for c in configs) + " |")
    lines.append("| Longest twist run (frames) | "
                 + " | ".join(str(summary["per_cfg"][c]["twist_longest_run_frames"]) for c in configs) + " |")
    lines.append("| Peak |vel| (p95)  | " + " | ".join(f"{summary['per_cfg'][c]['peak_vel_p95']:.0f}" for c in configs) + " |")
    lines.append("| Peak |vel| (max)  | " + " | ".join(f"{summary['per_cfg'][c]['peak_vel_max']:.0f}" for c in configs) + " |")
    lines.append("")

    # Chart embeds — if the caller passed them, surface them right under the
    # headline table so a reader can see the comparison at a glance instead
    # of parsing the numbers. We use HTML img so we can control display width
    # (GitHub / VS Code preview / Cursor all respect this).
    if chart_pngs:
        lines.append("### Visual summary")
        lines.append("")
        chart_widths = {
            "event_counts":    520,
            "twist_severity":  440,
            "events_by_group": 600,
        }
        for label, path in chart_pngs.items():
            try:
                rel = path.relative_to(out_path.parent)
                w = chart_widths.get(label, 520)
                lines.append(f'<img src="{rel}" width="{w}" alt="{label}">')
                lines.append("")
            except ValueError:
                pass

    # ---- Defer the dense per-joint / per-group breakdowns to an appendix
    #      at the bottom so the doc opens to the headline numbers + the
    #      side-by-side renders instead of a wall of numbers.
    appendix_lines: list[str] = []

    appendix_lines.append("## Appendix A — Events per joint")
    appendix_lines.append("")
    all_joints = sorted({
        jn for cfg in configs for jn in summary["per_cfg"][cfg]["per_joint"]
    })
    if all_joints:
        header_cells = (
            [f"{_disp(c)} slam"  for c in configs]
            + [f"{_disp(c)} pin"   for c in configs]
            + [f"{_disp(c)} twist" for c in configs]
        )
        appendix_lines.append("| joint | " + " | ".join(header_cells) + " |")
        appendix_lines.append("|---|" + "|".join(["---:"] * (3 * len(configs))) + "|")
        for jn in all_joints:
            slams  = [summary["per_cfg"][c]["per_joint"].get(jn, {}).get("slam",  0) for c in configs]
            pins   = [summary["per_cfg"][c]["per_joint"].get(jn, {}).get("pin",   0) for c in configs]
            twists = [summary["per_cfg"][c]["per_joint"].get(jn, {}).get("twist", 0) for c in configs]
            appendix_lines.append(
                "| " + jn + " | "
                + " | ".join(str(s) for s in slams)
                + " | " + " | ".join(str(p) for p in pins)
                + " | " + " | ".join(str(t) for t in twists)
                + " |"
            )
        appendix_lines.append("")

    appendix_lines.append("## Appendix B — Events per joint group")
    appendix_lines.append("")
    from scripts.bench.joint_limits import JOINT_GROUPS
    appendix_lines.append(
        "| group | "
        + " | ".join(f"{_disp(c)} slam"  for c in configs)
        + " | " + " | ".join(f"{_disp(c)} pin"   for c in configs)
        + " | " + " | ".join(f"{_disp(c)} twist" for c in configs)
        + " |"
    )
    appendix_lines.append("|---|" + "|".join(["---:"] * (3 * len(configs))) + "|")
    for g in JOINT_GROUPS:
        slams  = [summary["per_cfg"][c]["per_group"].get(g, {}).get("slam",  0) for c in configs]
        pins   = [summary["per_cfg"][c]["per_group"].get(g, {}).get("pin",   0) for c in configs]
        twists = [summary["per_cfg"][c]["per_group"].get(g, {}).get("twist", 0) for c in configs]
        appendix_lines.append(
            f"| {g} | "
            + " | ".join(str(s) for s in slams)
            + " | " + " | ".join(str(p) for p in pins)
            + " | " + " | ".join(str(t) for t in twists)
            + " |"
        )
    appendix_lines.append("")

    # ------------------------------------------------------------------
    # Anatomical-action sections (the easy-to-share view).
    #
    # Each section corresponds to one DOF / joint group and shows the top-N
    # events on it across BOTH configs. This is the layout that makes the
    # doc scannable: instead of six (category x config) tables, the reader
    # gets one section per body action they care about.
    # ------------------------------------------------------------------
    if top_events_per_cfg is not None:
        anatomical_sections = [
            {
                "key": "wrist_yaw",
                "title": "Wrist — palm TWIST (wrist_yaw) — palm rotates around the forearm",
                "joints": ["left_wrist_yaw_joint", "right_wrist_yaw_joint"],
                "categories": ["twist"],
                "rank_by": "peak_abs_deg",
                "summary": (
                    "Threshold-based detector: palm twist past natural human pronation/supination "
                    "(`|wrist_yaw| ≥ 80°`). Severity flips to `severe` past 100°. The mechanical "
                    "±146° limit is never touched, so slam/pin miss this entirely."
                ),
            },
            {
                "key": "wrist_pitch",
                "title": "Wrist — palm SIDE-TO-SIDE (wrist_pitch) — radial / ulnar deviation, ±32°",
                "joints": ["left_wrist_pitch_joint", "right_wrist_pitch_joint"],
                "categories": ["slam", "pin"],
                "rank_by": "duration_then_vel",
                "summary": "The small ±32° DOF. Pins here mean the IK gave up palm-sideways accuracy.",
            },
            {
                "key": "wrist_roll",
                "title": "Wrist — palm FORWARD/BACK BEND (wrist_roll) — flex / extend, −90° / +41° (asymmetric)",
                "joints": ["left_wrist_roll_joint", "right_wrist_roll_joint"],
                "categories": ["slam", "pin"],
                "rank_by": "vel_then_duration",
                "summary": "The asymmetric flex/extend DOF. Slams here are the classic wrist-flip basin hops.",
            },
            {
                "key": "elbow",
                "title": "Elbow (left_elbow_joint, right_elbow_joint)",
                "joints": ["left_elbow_joint", "right_elbow_joint"],
                "categories": ["slam", "pin"],
                "rank_by": "vel_then_duration",
                "summary": "Elbow against its limits. Most slams are basin hops on full extension.",
            },
            {
                "key": "shoulder",
                "title": "Shoulder (shoulder_roll / shoulder_yaw / shoulder_pitch)",
                "joints": [
                    "left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
                    "left_shoulder_roll_joint",  "right_shoulder_roll_joint",
                    "left_shoulder_yaw_joint",   "right_shoulder_yaw_joint",
                ],
                "categories": ["slam", "pin"],
                "rank_by": "vel_then_duration",
                "summary": "Shoulder workspace bound; mostly the asymmetric shoulder_roll hitting its hardware stop.",
            },
        ]

        def _rank_key(rank_mode: str):
            if rank_mode == "peak_abs_deg":
                return lambda ce: (
                    -float(ce[1].get("peak_abs_deg", abs(ce[1]["peak_angle_deg"]))),
                    -ce[1]["duration_frames"],
                )
            if rank_mode == "duration_then_vel":
                return lambda ce: (
                    -ce[1]["duration_frames"],
                    -abs(ce[1]["peak_vel_dps"]),
                )
            # default: vel_then_duration
            return lambda ce: (
                -abs(ce[1]["peak_vel_dps"]),
                -ce[1]["duration_frames"],
            )

        lines.append("# Renders — grouped by anatomical action")
        lines.append("")
        lines.append(
            "Each section below shows the top events on one DOF (or joint group) — "
            "**both configs are visible in every render** (rows: human BVH on top, "
            "then each config). The previous category-x-config grouping is now in "
            "the appendices at the bottom of this file."
        )
        lines.append("")

        for sec in anatomical_sections:
            lines.append(f"## {sec['title']}")
            lines.append("")
            lines.append(sec["summary"])
            lines.append("")

            joints = set(sec["joints"])
            cats = set(sec["categories"])
            flat = []
            for clip, by_cfg in data["clips"].items():
                for cfg in configs:
                    for e in by_cfg.get(cfg, {}).get("events", []):
                        if e["joint"] in joints and e["category"] in cats:
                            flat.append((clip, e))
            # Deduplicate by event_id so we don't list (clip, joint, peak_f) twice
            # when both configs flagged it.
            uniq: dict[str, tuple[str, dict]] = {}
            for clip, e in flat:
                key = _event_id(clip, e)
                if key not in uniq:
                    uniq[key] = (clip, e)
            flat = list(uniq.values())
            flat.sort(key=_rank_key(sec["rank_by"]))
            flat = flat[:top_events_per_cfg]

            if not flat:
                lines.append("_(no events on this DOF in either config)_")
                lines.append("")
                continue

            # Per-section table — compact, anatomical labels per row
            lines.append("| # | clip | joint | cat | start | end | peak frame | duration (s) | summary metric |")
            lines.append("|---:|---|---|---|---:|---:|---:|---:|---:|")
            for i, (clip, e) in enumerate(flat, 1):
                cat = e["category"]
                if cat == "twist":
                    peak_abs = float(e.get("peak_abs_deg", abs(e["peak_angle_deg"])))
                    metric = f"|twist|={peak_abs:.1f}° (sev `{e.get('severity','warn')}`)"
                elif cat == "slam":
                    metric = f"peak vel={e['peak_vel_dps']:+.0f} deg/s"
                else:  # pin
                    metric = f"angle@stop={e['peak_angle_deg']:+.1f}° for {e['duration_frames']} f"
                lines.append(
                    f"| {i} | `{clip}` | {e['joint']} | {cat} | "
                    f"{e['start_f']} | {e['end_f']} | {e['peak_f']} | "
                    f"{e['duration_s']:.2f} | {metric} |"
                )
            lines.append("")

            # Inline renders below the table
            if sxs_pngs_by_event_id is not None:
                legend = " / ".join(["human"] + [_disp(c) for c in configs])
                lines.append(f"_Renders (rows: {legend}). Click an image to expand._")
                lines.append("")
                for i, (clip, e) in enumerate(flat, 1):
                    eid = _event_id(clip, e)
                    png = sxs_pngs_by_event_id.get(eid)
                    if png is None:
                        continue
                    try:
                        rel = png.relative_to(out_path.parent)
                        cat = e["category"]
                        if cat == "twist":
                            peak_abs = float(e.get("peak_abs_deg", abs(e["peak_angle_deg"])))
                            cap = f"|twist|={peak_abs:.1f}° dur={e['duration_s']:.2f}s"
                        elif cat == "slam":
                            cap = f"peak vel={e['peak_vel_dps']:+.0f} deg/s"
                        else:
                            cap = f"angle@stop={e['peak_angle_deg']:+.1f}° dur={e['duration_s']:.2f}s"
                        lines.append(f"**#{i} — `{clip}` :: {e['joint']} @ f{e['peak_f']} ({cat}, {cap})**")
                        lines.append("")
                        lines.append(f"![{eid}]({rel})")
                        lines.append("")
                    except ValueError:
                        pass
    else:
        # No top-N truncation requested → emit full per-clip event listings
        # grouped by config.
        for cfg in configs:
            lines.append(f"## Config `{cfg}` — full event listing")
            lines.append("")
            clip_blocks: list[tuple[str, list[dict]]] = []
            for clip, by_cfg in data["clips"].items():
                evs = list(by_cfg.get(cfg, {}).get("events", []))
                if not evs:
                    continue
                evs.sort(key=lambda e: (e["category"] != "slam", -abs(e["peak_vel_dps"])))
                clip_blocks.append((clip, evs))
            clip_blocks.sort(key=lambda kv: -max(abs(e["peak_vel_dps"]) for e in kv[1]))
            total = sum(len(evs) for _, evs in clip_blocks)
            lines.append(f"Total events: **{total}** across **{len(clip_blocks)}** clips")
            for clip, evs in clip_blocks:
                tier = by_name.get(clip + ".bvh", {}).get("tier", "?")
                lines.append("")
                lines.append(f"### `{clip}.bvh`  ({len(evs)} events, tier={tier})")
                lines.append("")
                lines.append("| joint | cat | side | start | end | peak | dur(s) | peak vel (deg/s) | peak angle (deg) |")
                lines.append("|---|---|---|---:|---:|---:|---:|---:|---:|")
                for e in evs:
                    lines.append(
                        f"| {e['joint']} | {e['category']} | {e['side']} | "
                        f"{e['start_f']} | {e['end_f']} | {e['peak_f']} | "
                        f"{e['duration_s']:.2f} | {e['peak_vel_dps']:+.0f} | {e['peak_angle_deg']:+.1f} |"
                    )

    # Dense per-joint / per-group breakdowns go at the very bottom so the
    # report opens to the headline numbers + side-by-side renders.
    if appendix_lines:
        lines.append("---")
        lines.append("")
        lines.extend(appendix_lines)

    out_path.write_text("\n".join(lines) + "\n")


def _event_id(clip: str, ev: dict) -> str:
    return f"{clip}__{ev['joint']}__{ev['category']}__f{ev['peak_f']}"


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> None:
    import argparse
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bench-dir", type=Path, required=True, help="Existing bench output directory containing csvs/")
    ap.add_argument("--fps", type=float, default=DEFAULTS["fps"])
    ap.add_argument("--pin-tol-deg", type=float, default=DEFAULTS["pin_tol_deg"])
    ap.add_argument("--slam-vel-dps", type=float, default=DEFAULTS["slam_vel_dps"])
    ap.add_argument("--min-pin-frames", type=int, default=DEFAULTS["min_pin_frames"])
    ap.add_argument("--max-gap-frames", type=int, default=DEFAULTS["max_gap_frames"])
    ap.add_argument("--min-motion-dps", type=float, default=DEFAULTS["min_motion_dps"],
                    help="Drop events where the joint never moved (rest-pose pins from asymmetric ranges)")
    ap.add_argument("--rest-side-tol-deg", type=float, default=DEFAULTS["rest_side_tol_deg"],
                    help="Skip pin events on the rest side of joints whose 0-rad rest pose lies near a limit")
    ap.add_argument("--twist-threshold-deg", type=float, default=DEFAULTS["twist_threshold_deg"],
                    help="Palm-twist (wrist_yaw) excursion threshold (default ~80°, the natural human comfort limit)")
    ap.add_argument("--twist-severe-threshold-deg", type=float, default=DEFAULTS["twist_severe_threshold_deg"],
                    help="|wrist_yaw| >= this deg is flagged severity='severe' instead of 'warn'")
    ap.add_argument("--twist-min-frames", type=int, default=DEFAULTS["twist_min_frames"],
                    help="Minimum contiguous frames over threshold for a twist event")
    ap.add_argument("--twist-max-gap-frames", type=int, default=DEFAULTS["twist_max_gap_frames"],
                    help="Tolerate dips below threshold within a twist run for up to N frames")
    ap.add_argument("--configs", nargs="*", default=None, help="Override config name list (else inferred from csvs/)")
    args = ap.parse_args(argv)

    data = process_bench_dir(
        args.bench_dir.resolve(),
        configs=args.configs,
        fps=args.fps,
        pin_tol_deg=args.pin_tol_deg,
        slam_vel_dps=args.slam_vel_dps,
        min_pin_frames=args.min_pin_frames,
        max_gap_frames=args.max_gap_frames,
        min_motion_dps=args.min_motion_dps,
        rest_side_tol_deg=args.rest_side_tol_deg,
        twist_threshold_deg=args.twist_threshold_deg,
        twist_severe_threshold_deg=args.twist_severe_threshold_deg,
        twist_min_frames=args.twist_min_frames,
        twist_max_gap_frames=args.twist_max_gap_frames,
    )
    write_limit_events_md(args.bench_dir / "limit_events.md", data)
    print(f"[OK] wrote limit_events.md")


if __name__ == "__main__":
    main()

"""Aggregate per-(clip, config) records into summary.md / ik_failures.md / REPORT.md."""

from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path
from typing import Dict, List

import numpy as np


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _mean(items: list[float]) -> float:
    return float(np.mean(items)) if items else 0.0


def _safe_div(a: float, b: float) -> float:
    return a / b if abs(b) > 1e-12 else 0.0


# ---------------------------------------------------------------------------
# metrics.json
# ---------------------------------------------------------------------------

def dump_metrics_json(path: Path, per_clip: Dict[str, dict]) -> None:
    """per_clip[clip_name] = {config_name: ClipMetrics-asdict, ...}."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(per_clip, indent=2, default=float))


# ---------------------------------------------------------------------------
# summary.md
# ---------------------------------------------------------------------------

def _aggregate(per_clip: Dict[str, dict], config_name: str, key: str) -> tuple[float, float, float]:
    """Aggregate a scalar metric across clips: (mean, median, max)."""
    vals = []
    for clip_name, by_cfg in per_clip.items():
        if config_name in by_cfg:
            v = by_cfg[config_name].get(key)
            if v is None:
                continue
            vals.append(float(v))
    if not vals:
        return 0.0, 0.0, 0.0
    arr = np.asarray(vals)
    return float(arr.mean()), float(np.median(arr)), float(arr.max())


def _aggregate_group(per_clip: Dict[str, dict], config_name: str, key: str, group: str) -> float:
    """Aggregate a per-group metric (e.g. saturation_pct[wrist]) -> mean across clips."""
    vals = []
    for clip_name, by_cfg in per_clip.items():
        if config_name in by_cfg:
            sub = by_cfg[config_name].get(key, {})
            if group in sub:
                vals.append(float(sub[group]))
    return _mean(vals)


def write_summary_md(
    out_path: Path,
    per_clip: Dict[str, dict],
    configs: list[str],
    corpus_entries: list[dict],
    section_counts: Dict[tuple[str, str], int],
) -> None:
    """Build the aggregate metric table."""
    out_path.parent.mkdir(parents=True, exist_ok=True)

    lines: List[str] = ["# A/B benchmark — aggregate metrics", ""]
    lines.append(f"Corpus: **{len(corpus_entries)} clips** across categories: " +
                 ", ".join(sorted(set(e["category"] for e in corpus_entries))))
    lines.append(f"Configs evaluated: {', '.join(configs)}")
    lines.append("")

    lines.append("## Headline numbers (aggregate over the full corpus)")
    lines.append("")
    lines.append("| metric | " + " | ".join(configs) + " | better |")
    lines.append("|---|" + "|".join(["---:"] * len(configs)) + "|:---|")

    def _row(label: str, key: str, agg: str = "mean", lower_is_better: bool = True, scale: float = 1.0, fmt: str = "{:.3f}") -> None:
        vals = []
        for cfg in configs:
            m, med, mx = _aggregate(per_clip, cfg, key)
            v = {"mean": m, "median": med, "max": mx}[agg] * scale
            vals.append(v)
        # which one is better
        if len(configs) == 2:
            if abs(vals[0] - vals[1]) < 1e-9:
                marker = "tie"
            elif (vals[0] < vals[1]) == lower_is_better:
                marker = f"**{configs[0]}** ({abs(vals[0] - vals[1]) / max(abs(vals[1]), 1e-9) * 100:.0f}% better)"
            else:
                marker = f"**{configs[1]}** ({abs(vals[0] - vals[1]) / max(abs(vals[0]), 1e-9) * 100:.0f}% better)"
        else:
            marker = ""
        formatted = " | ".join(fmt.format(v) for v in vals)
        lines.append(f"| {label} ({agg}) | {formatted} | {marker} |")

    _row("Saturation overall % (lower=better)", "saturation_pct_overall", lower_is_better=True, fmt="{:.2f}")
    _row("FK position residual (m) (lower=better)", "fk_pos_residual_m_mean", lower_is_better=True, fmt="{:.4f}")
    _row("FK position residual p95 (m)", "fk_pos_residual_m_p95", agg="median", lower_is_better=True, fmt="{:.4f}")
    _row("Smoothness deg/s^2 (lower=smoother)", "smoothness_deg_s2_mean", lower_is_better=True, fmt="{:.1f}")
    _row("Hip yaw wobble (deg/s)", "hip_yaw_wobble_dps", lower_is_better=True, fmt="{:.1f}")
    _row("Waist yaw wobble (deg/s)", "waist_yaw_wobble_dps", lower_is_better=True, fmt="{:.1f}")
    _row("Shoulder yaw |mean| (deg)", "shoulder_yaw_mean_abs_deg", lower_is_better=True, fmt="{:.2f}")
    _row("L hand-pelvis dist (m)", "left_hand_pelvis_dist_m_mean", lower_is_better=False, fmt="{:.3f}")
    _row("R hand-pelvis dist (m)", "right_hand_pelvis_dist_m_mean", lower_is_better=False, fmt="{:.3f}")
    _row("Root XY travel (m)", "root_travel_m", lower_is_better=False, fmt="{:.2f}")

    # IK failure section counts
    lines.append("")
    lines.append("## IK failure section count (lower=better)")
    lines.append("")
    lines.append("| | " + " | ".join(configs) + " |")
    lines.append("|---|" + "|".join(["---:"] * len(configs)) + "|")
    total_sections = {cfg: 0 for cfg in configs}
    for (clip, cfg), c in section_counts.items():
        total_sections[cfg] = total_sections.get(cfg, 0) + c
    lines.append("| Total sections across corpus | " + " | ".join(str(total_sections.get(cfg, 0)) for cfg in configs) + " |")
    lines.append("| Mean sections per clip | " + " | ".join(
        f"{(total_sections.get(cfg, 0) / max(len(corpus_entries), 1)):.2f}" for cfg in configs) + " |")

    # Per-group saturation table
    lines.append("")
    lines.append("## Saturation % by joint group (lower=better)")
    lines.append("")
    groups = ["hip", "knee", "ankle", "waist", "shoulder", "elbow", "wrist", "head"]
    lines.append("| group | " + " | ".join(configs) + " |")
    lines.append("|---|" + "|".join(["---:"] * len(configs)) + "|")
    for g in groups:
        row_vals = []
        for cfg in configs:
            row_vals.append(_aggregate_group(per_clip, cfg, "saturation_pct", g))
        lines.append(f"| {g} | " + " | ".join(f"{v:.2f}" for v in row_vals) + " |")

    # Per-clip side-by-side table (FK residual + saturation overall)
    lines.append("")
    lines.append("## Per-clip headline metrics")
    lines.append("")
    headers = ["clip", "tier", "category", "frames"]
    for cfg in configs:
        headers += [f"{cfg}.sat%", f"{cfg}.fkres_m", f"{cfg}.shYaw"]
    lines.append("| " + " | ".join(headers) + " |")
    lines.append("|" + "|".join(["---" if i < 4 else "---:" for i in range(len(headers))]) + "|")
    by_name = {e["name"]: e for e in corpus_entries}
    # Stable ordering by tier then category
    for clip_name in sorted(per_clip.keys(), key=lambda n: (
        {"anchor": 0, "hip": 1, "wrist": 2, "leg": 3, "shoulder": 4, "pelvis": 5, "ankle": 6, "random": 7}.get(by_name.get(n, {}).get("tier", "random"), 99),
        by_name.get(n, {}).get("category", ""), n,
    )):
        row = [clip_name, by_name.get(clip_name, {}).get("tier", "?"),
               by_name.get(clip_name, {}).get("category", "?"),
               str(by_name.get(clip_name, {}).get("num_frames", "?"))]
        for cfg in configs:
            cm = per_clip[clip_name].get(cfg, {})
            sat = cm.get("saturation_pct_overall", 0.0)
            fk = cm.get("fk_pos_residual_m_mean", 0.0)
            sh = cm.get("shoulder_yaw_mean_abs_deg", 0.0)
            row += [f"{sat:.1f}", f"{fk:.4f}", f"{sh:.1f}"]
        lines.append("| " + " | ".join(row) + " |")

    out_path.write_text("\n".join(lines) + "\n")


# ---------------------------------------------------------------------------
# ik_failures.md
# ---------------------------------------------------------------------------

def write_ik_failures_md(
    out_path: Path,
    sections_per_pair: Dict[tuple[str, str], list[dict]],
    section_pngs: Dict[tuple[str, str], list[Path]],
) -> None:
    """Roll up IK failure sections grouped first by config, then by clip."""
    out_path.parent.mkdir(parents=True, exist_ok=True)

    lines = ["# IK failure sections", ""]
    lines.append("A *section* is a contiguous run of frames where the IK is")
    lines.append("either close to >= 4 hardware joint limits or has FK position")
    lines.append("residual >= 0.18 m. Sections are listed in descending peak severity.")
    lines.append("")

    by_cfg: Dict[str, list[tuple[str, list[dict], list[Path]]]] = {}
    for (clip, cfg), sects in sections_per_pair.items():
        if not sects:
            continue
        by_cfg.setdefault(cfg, []).append((clip, sects, section_pngs.get((clip, cfg), [])))

    for cfg in sorted(by_cfg.keys()):
        lines.append(f"## Config `{cfg}`")
        clip_list = sorted(by_cfg[cfg], key=lambda x: -sum(s["peak_saturated_dofs"] for s in x[1]))
        for clip, sects, pngs in clip_list:
            total_sat_score = sum(s["peak_saturated_dofs"] for s in sects)
            lines.append("")
            lines.append(f"### `{clip}`  ({len(sects)} sections, sat-score={total_sat_score})")
            lines.append("")
            lines.append("| start | end | peak | dur(s) | trigger | peak_sat | peak_fk(m) | dominant joints |")
            lines.append("|---:|---:|---:|---:|:---|---:|---:|:---|")
            for s in sects:
                dom = ", ".join(s["dominant_joints"][:3]) if s["dominant_joints"] else "-"
                lines.append(
                    f"| {s['start_frame']} | {s['end_frame']} | {s['peak_frame']} | {s['duration_s']:.2f} | "
                    f"{s['trigger']} | {s['peak_saturated_dofs']} | {s['peak_fk_residual_m']:.3f} | {dom} |"
                )
            for p in pngs:
                try:
                    rel = p.relative_to(out_path.parent)
                    lines.append("")
                    lines.append(f"![{p.name}]({rel})")
                except ValueError:
                    pass

    out_path.write_text("\n".join(lines) + "\n")


# ---------------------------------------------------------------------------
# REPORT.md skeleton
# ---------------------------------------------------------------------------

def write_report_md_skeleton(
    out_path: Path,
    out_dir: Path,
    per_clip: Dict[str, dict],
    configs: list[str],
    corpus_entries: list[dict],
    section_counts: Dict[tuple[str, str], int],
    section_total: Dict[str, int],
    runtime_summary: dict,
) -> None:
    """Generate a comprehensive REPORT.md whose data sections are pre-filled.

    Sections Conclusion / Failure-points narrative / Next-steps are stubs that
    the analyst hand-edits after eyeballing the rendered PNGs.
    """
    out_path.parent.mkdir(parents=True, exist_ok=True)

    cats: dict[str, int] = {}
    tiers: dict[str, int] = {}
    for e in corpus_entries:
        cats[e["category"]] = cats.get(e["category"], 0) + 1
        tiers[e["tier"]] = tiers.get(e["tier"], 0) + 1

    lines: list[str] = []
    lines.append("# Retargeter config A/B benchmark — REPORT")
    lines.append("")
    lines.append(f"Output dir: `{out_dir}`")
    lines.append("")
    lines.append("## 1. Overview")
    lines.append("")
    lines.append("Empirical comparison of two SOMA -> X2 Ultra retargeter configs:")
    lines.append("")
    for cfg in configs:
        lines.append(f"- `{cfg}` — see `configs/{cfg}.json`")
    lines.append("")
    lines.append("Same MJCF, same Newton-IK solver, same scaler — only the retargeter")
    lines.append("config JSON differs. The two configs in this run differ along:")
    lines.append("")
    lines.append("- `model_height` (global SOMA->robot scale)")
    lines.append("- `Hips.r_weight` (rotational IK weight on pelvis)")
    lines.append("- `LeftHand`/`RightHand.t_weight` and `r_weight` (wrist target weights)")
    lines.append("- `smooth_joint_filter_objective_body_masks` (anti-jitter masks)")
    lines.append("")

    lines.append("## 2. Testing criteria")
    lines.append("")
    lines.append("### 2.1 Corpus")
    lines.append("")
    lines.append(f"Total clips: **{len(corpus_entries)}** drawn from the bones-seed dataset")
    lines.append("after a screening pass over a sampled candidate pool.")
    lines.append("")
    lines.append("- **By category**: " + ", ".join(f"{k}={v}" for k, v in sorted(cats.items())))
    lines.append("- **By tier**:")
    for t in ["anchor", "hip", "wrist", "leg", "shoulder", "pelvis", "ankle", "random"]:
        if t in tiers:
            lines.append(f"    - `{t}`: {tiers[t]} clip(s)")
    lines.append("")
    lines.append("Tier semantics:")
    lines.append("- `anchor`     — fixed walk_forward_loop_001__A021 reference")
    lines.append("- `hip`        — high BVH Hips-yaw range (top per category)")
    lines.append("- `wrist`      — high LeftHand/RightHand Euler range (top per category)")
    lines.append("- `leg`        — high LeftLeg/RightLeg X range (kicks, squats)")
    lines.append("- `shoulder`   — high LeftArm/RightArm Z range (big arm swings)")
    lines.append("- `pelvis`     — lowest pelvis Y (deep crouch / sit / lunge)")
    lines.append("- `ankle`      — high ankle Y range (ankle swing)")
    lines.append("- `random`     — uniform sample per category (baseline)")
    lines.append("")
    lines.append("Full per-clip BVH statistics are in `corpus_stats.md`/`.json`.")
    lines.append("")

    lines.append("### 2.2 Metrics")
    lines.append("")
    lines.append("Seven aggregate metrics computed per (clip, config):")
    lines.append("")
    lines.append("1. **Saturation %** — frames within 5° of any hardware joint limit, reported")
    lines.append("   overall and broken down by joint group (hip/wrist/shoulder/...).")
    lines.append("2. **FK position residual (m)** — Euclidean distance between the SOMA IK")
    lines.append("   target position (post-scaler) and the FK-achieved body position after")
    lines.append("   the joint-limit clamper has run. mean/p95/max.")
    lines.append("3. **Smoothness (deg/s²)** — mean magnitude of joint-angle second")
    lines.append("   difference. Smaller = smoother trajectory.")
    lines.append("4. **Hand-vs-pelvis distance (m)** — left/right wrist-roll to pelvis,")
    lines.append("   mean and std across frames.")
    lines.append("5. **Root XY travel (m)** — total horizontal path length of the root.")
    lines.append("6. **Hip-yaw wobble (deg/s)** — RMS of frame-to-frame change in left/right")
    lines.append("   hip yaw — captures the 'twisting hips' symptom directly.")
    lines.append("7. **Shoulder yaw |mean| (deg)** — mean absolute value of shoulder yaw")
    lines.append("   joints. High values flag IK compensating with shoulder yaw for")
    lines.append("   workspace limits.")
    lines.append("")

    lines.append("### 2.3 Per-frame flagging and IK failure sections")
    lines.append("")
    lines.append("Per (clip, config), the bench flags top-1 worst frames in three categories")
    lines.append("(pelvis-Z bobbing, wrist angular velocity, saturated-DOF count) and detects")
    lines.append("contiguous IK failure sections (>= 4 saturated DOFs or FK residual >= 0.18 m")
    lines.append("for at least 5 frames). Up to 5 section peaks per pair are rendered as PNGs.")
    lines.append("")

    lines.append("## 3. Experiments executed")
    lines.append("")
    if runtime_summary:
        lines.append(f"- Total retargeting runtime: **{runtime_summary.get('retarget_s', 0):.0f} s**")
        lines.append(f"- Total metrics runtime: **{runtime_summary.get('metrics_s', 0):.0f} s**")
        lines.append(f"- Total render runtime: **{runtime_summary.get('render_s', 0):.0f} s**")
        lines.append(f"- Wall clock end-to-end: **{runtime_summary.get('total_s', 0):.0f} s**")
    lines.append(f"- Output CSVs: `{out_dir.name}/csvs/<clip>__<config>.csv`")
    lines.append(f"- Per-clip metrics: see `metrics.json`")
    lines.append(f"- Per-clip rendered frames: `{out_dir.name}/frames/<clip>__<config>/*.png`")
    lines.append("")

    lines.append("## 4. Evaluation")
    lines.append("")
    lines.append("See `summary.md` for the full table. Headline aggregate numbers:")
    lines.append("")
    for cfg in configs:
        sat_mean, _, _ = _aggregate(per_clip, cfg, "saturation_pct_overall")
        fk_mean, _, _ = _aggregate(per_clip, cfg, "fk_pos_residual_m_mean")
        sh_mean, _, _ = _aggregate(per_clip, cfg, "shoulder_yaw_mean_abs_deg")
        hipw, _, _ = _aggregate(per_clip, cfg, "hip_yaw_wobble_dps")
        section_count = section_total.get(cfg, 0)
        lines.append(f"- **{cfg}**")
        lines.append(f"    - Saturation %: **{sat_mean:.2f}**")
        lines.append(f"    - FK pos residual (m): **{fk_mean:.4f}**")
        lines.append(f"    - Shoulder yaw |mean| (deg): **{sh_mean:.2f}**")
        lines.append(f"    - Hip yaw wobble (deg/s): **{hipw:.1f}**")
        lines.append(f"    - IK failure sections (total): **{section_count}**")
    lines.append("")

    lines.append("Per-clip and per-section detail:")
    lines.append("")
    lines.append("- `summary.md` — full per-clip metric tables")
    lines.append("- `ik_failures.md` — IK failure section listing with linked PNGs")
    lines.append("- `corpus_stats.md` — BVH motion-statistics roll-up used for tier selection")
    lines.append("")

    lines.append("## 5. Conclusion")
    lines.append("")
    lines.append("> _TODO (analyst):_ Inspect the rendered PNGs under `frames/` and the")
    lines.append("> per-section dominant-joint listings in `ik_failures.md`, then write a")
    lines.append("> short verdict (one to four bullet points) on which config wins overall,")
    lines.append("> which loses, and where the differences are largest.")
    lines.append("")

    lines.append("## 6. Failure points")
    lines.append("")
    lines.append("> _TODO (analyst):_ For each config, narrate the dominant IK failure modes")
    lines.append("> (e.g. 'left_shoulder_roll saturates during forward reaches'). Use")
    lines.append("> `ik_failures.md` and the rendered section PNGs to drive this. This")
    lines.append("> section should supersede the *Known limitations (unsolved)* block in")
    lines.append("> `soma_retargeter/configs/agibot_x2_ultra/README.md` if the picture has")
    lines.append("> changed.")
    lines.append("")

    lines.append("## 7. Next steps")
    lines.append("")
    lines.append("> _TODO (analyst):_ Propose follow-up. Possibilities:")
    lines.append("")
    lines.append("- Adopt the winning config wholesale; PR it into `agibot-x2-references/soma-retargeter`.")
    lines.append("- Cherry-pick the winning fields (e.g. `Hips.r_weight=10`, wrist smoothing")
    lines.append("  masks) onto our `v5` base while keeping the parts where `v5` outperforms.")
    lines.append("- Investigate the failure regions surfaced above (workspace, shoulder")
    lines.append("  asymmetry, wrist-roll asymmetry) and re-run with proposed offsets.")
    lines.append("- Plan B (rotor inertias) and Plan C (cross-embodiment DOF map) are tracked")
    lines.append("  separately in this branch but not measured here.")
    lines.append("")

    lines.append("---")
    lines.append("")
    lines.append("Auto-generated by `scripts/bench_configs.py`. See `bench/aggregate.py` for")
    lines.append("the templating; data sections refresh on each run.")

    out_path.write_text("\n".join(lines) + "\n")

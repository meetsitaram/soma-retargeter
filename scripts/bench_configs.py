"""End-to-end retargeter config A/B benchmark driver.

Pipeline:
  0. (optional)  Pre-screen ~1k BVH clips -> corpus_stats.json/.md
  1. Assemble a 4-tier corpus -> corpus.json
  2. Stage both retargeter configs in `configs/`
  3. For every (clip, config) pair:
       - retarget the BVH -> CSV
       - compute aggregate metrics (saturation, FK residual, ...)
       - flag top per-frame failures + contiguous IK failure sections
       - render PNGs for the flagged frames and section peaks
  4. Aggregate everything into summary.md / ik_failures.md / REPORT.md / metrics.json

The bench writes its outputs under `scratch/bench_<timestamp>/`. Nothing else
in the repo is modified. The corresponding feature branch
`retarget-eval-2026-06-04` (already created in parent + submodule) keeps any
auxiliary commits isolated.

Quick-start:

    # Smoke run (one walk, one manipulation clip):
    python scripts/bench_configs.py \\
        --smoke

    # Full run:
    python scripts/bench_configs.py \\
        --ours soma_retargeter/configs/agibot_x2_ultra/soma_to_x2_ultra_retargeter_config.json \\
        --theirs /home/stickbot/Downloads/soma_to_x2_ultra_retargeter_config.json \\
        --our-label x2_uniform_h140 --their-label x2_uniform_h170_tuned \\
        --max-frames 3000
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from scripts.bench import corpus as bench_corpus
from scripts.bench import retarget as bench_retarget
from scripts.bench import metrics as bench_metrics
from scripts.bench import frames as bench_frames
from scripts.bench import render as bench_render
from scripts.bench import aggregate as bench_aggregate
from scripts.bench import kinematics as bench_kinematics
from scripts.bench import limit_events as bench_limit_events
from scripts.bench import charts as bench_charts
from scripts.bench import side_by_side as bench_side_by_side
from scripts.bench.joint_limits import JOINT_NAMES, JOINT_LIMITS_DEG


# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------

DEFAULT_OURS = REPO_ROOT / "soma_retargeter" / "configs" / "agibot_x2_ultra" / "soma_to_x2_ultra_retargeter_config.json"
DEFAULT_THEIRS = Path("/home/stickbot/Downloads/soma_to_x2_ultra_retargeter_config.json")


# ---------------------------------------------------------------------------
# Pipeline pieces
# ---------------------------------------------------------------------------

def stage_configs(
    out_dir: Path,
    primary_configs: list[tuple[Path, str]],
) -> dict[str, Path]:
    """Stage retargeter config JSONs into `<out_dir>/configs/<label>.json`.

    ``primary_configs`` is an ordered list of ``(src_path, label)`` pairs.
    The pipeline supports an arbitrary number of configs (N >= 2) — the
    first two are the historical ``--ours`` / ``--theirs`` pair, but
    additional configs come from ``--extra-config LABEL=PATH``.
    """
    cfg_dir = out_dir / "configs"
    cfg_dir.mkdir(parents=True, exist_ok=True)
    out_paths: dict[str, Path] = {}
    for src, label in primary_configs:
        dst = cfg_dir / f"{label}.json"
        # Allow passing an --extra-config whose path is already inside the
        # bench dir's configs/ folder (idempotent re-runs); only copy when
        # source and destination differ.
        if src.resolve() != dst.resolve():
            shutil.copyfile(src, dst)
        out_paths[label] = dst
    print(f"[INFO]: staged {len(primary_configs)} configs into {cfg_dir}: "
          f"{', '.join(label for _, label in primary_configs)}")
    return out_paths


def _parse_extra_configs(extras: list[str] | None) -> list[tuple[str, Path]]:
    """Parse ``--extra-config LABEL=PATH`` strings into [(label, Path), ...].

    Validates that PATH exists and that LABEL is a usable directory name
    (no slashes / spaces). Duplicate labels raise an error early.
    """
    if not extras:
        return []
    out: list[tuple[str, Path]] = []
    seen: set[str] = set()
    for spec in extras:
        if "=" not in spec:
            raise SystemExit(f"--extra-config expects LABEL=PATH, got: {spec!r}")
        label, raw_path = spec.split("=", 1)
        label = label.strip()
        if not label or any(c in label for c in "/\\ "):
            raise SystemExit(f"--extra-config LABEL must be a simple identifier, got: {label!r}")
        if label in seen:
            raise SystemExit(f"--extra-config LABEL {label!r} specified twice")
        path = Path(raw_path).expanduser().resolve()
        if not path.is_file():
            raise SystemExit(f"--extra-config {label} -> {path} (file not found)")
        seen.add(label)
        out.append((label, path))
    return out


def run_pair(
    clip_entry: dict,
    config_label: str,
    *,
    skip_existing_csv: bool = False,
    config_path: Path,
    out_dir: Path,
    max_frames: int | None,
    render_section_cap: int,
    fps: float = 120.0,
) -> dict:
    """Retarget one BVH with one config, compute metrics + flags + sections, render PNGs."""
    clip_name = clip_entry["name"]
    bvh_path = Path(clip_entry["path"]).resolve()

    csv_dir = out_dir / "csvs"
    csv_path = csv_dir / f"{Path(clip_name).stem}__{config_label}.csv"
    frames_dir = out_dir / "frames" / f"{Path(clip_name).stem}__{config_label}"

    record = {"clip": clip_name, "config": config_label}

    # 1) Retarget (or skip if CSV already exists and skip_existing_csv is set)
    if skip_existing_csv and csv_path.is_file():
        n_frames = int(np.loadtxt(csv_path, delimiter=",", skiprows=1, dtype=np.float64).shape[0])
        record["retarget_s"] = 0.0
        record["num_frames"] = n_frames
        record["skipped_retarget"] = True
        print(f"[INFO]:   skip retarget (csv exists, {n_frames} frames)")
    else:
        t0 = time.time()
        n_frames = bench_retarget.retarget_clip(bvh_path, csv_path, config_path, max_frames=max_frames)
        record["retarget_s"] = float(time.time() - t0)
        record["num_frames"] = int(n_frames)

    # 2) Metrics + per-frame data
    t0 = time.time()
    metrics_obj, per_frame = bench_metrics.compute_clip_metrics(
        clip_name=clip_name,
        config_name=config_label,
        csv_path=csv_path,
        bvh_path=bvh_path,
        retarget_config_path=config_path,
        fps=fps,
        max_frames=max_frames,
    )
    record["metrics"] = asdict(metrics_obj)
    record["metrics_s"] = float(time.time() - t0)

    # 3) Per-frame flags
    flags = bench_frames.flag_top_frames(per_frame, fps=fps, k_per_category=1)
    record["flags"] = bench_frames.to_dicts(flags)

    # 4) IK failure sections
    csv_data = np.loadtxt(csv_path, delimiter=",", skiprows=1, dtype=np.float64)
    joint_angles_deg = csv_data[:, 7:7 + len(JOINT_NAMES)]
    sections = bench_frames.detect_ik_failure_sections(
        per_frame=per_frame,
        joint_angles_deg=joint_angles_deg,
        joint_names=JOINT_NAMES,
        joint_limits_deg=[JOINT_LIMITS_DEG[j] for j in JOINT_NAMES],
        fps=fps,
        sat_threshold=4,
        fk_residual_threshold_m=0.18,
        min_duration_frames=5,
    )
    record["sections"] = bench_frames.to_dicts(sections)

    # 5) Render PNGs (top-1 frame per flag category + top-N section peaks).
    # When skip_existing_csv was active and the PNG already exists on disk,
    # reuse it rather than re-rendering — these are per-clip diagnostic
    # frames, not the side-by-side report renders, so re-rendering would be
    # pure waste during an --extra-config pass.
    t0 = time.time()
    pngs: list[Path] = []
    # Flag PNGs
    for flag in flags:
        png = frames_dir / f"flag_{flag.category}_f{flag.frame_idx}.png"
        if skip_existing_csv and png.is_file():
            pngs.append(png)
            continue
        try:
            bench_render.render_frame(
                csv_path=csv_path, bvh_path=bvh_path,
                retarget_config_path=config_path,
                frame_idx=flag.frame_idx,
                out_png=png,
                width=960, height=720,
                max_frames=max_frames,
            )
            pngs.append(png)
        except Exception as e:
            print(f"[WARN]: failed to render flag {flag.category}@{flag.frame_idx}: {e}")
    # Section PNGs (up to render_section_cap)
    for i, sec in enumerate(sections[:render_section_cap]):
        png = frames_dir / f"section_{i:02d}_peak_f{sec.peak_frame}.png"
        if skip_existing_csv and png.is_file():
            pngs.append(png)
            continue
        try:
            bench_render.render_frame(
                csv_path=csv_path, bvh_path=bvh_path,
                retarget_config_path=config_path,
                frame_idx=sec.peak_frame,
                out_png=png,
                width=960, height=720,
                max_frames=max_frames,
            )
            pngs.append(png)
        except Exception as e:
            print(f"[WARN]: failed to render section peak @{sec.peak_frame}: {e}")
    record["render_s"] = float(time.time() - t0)
    record["pngs"] = [str(p) for p in pngs]

    return record


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _run_limit_events_analysis(args) -> None:
    """Run slam/pin detection + (optional) side-by-side render on an existing bench dir."""
    bench_dir = args.analyze_limit_events.resolve()
    if not bench_dir.is_dir():
        raise SystemExit(f"--analyze-limit-events directory does not exist: {bench_dir}")

    print(f"[INFO] analyzing limit events under {bench_dir}")
    data = bench_limit_events.process_bench_dir(bench_dir)

    sxs_pngs: dict[str, Path] = {}
    display_names: dict[str, str] = {}
    if args.display_name:
        for pair in args.display_name:
            if "=" in pair:
                k, v = pair.split("=", 1)
                display_names[k.strip()] = v.strip()
    if args.side_by_side_events and args.side_by_side_events > 0:
        # Resolve config paths (the bench dir stages them under configs/)
        cfg_dir = bench_dir / "configs"
        cfg_paths = {cfg: cfg_dir / f"{cfg}.json" for cfg in data["configs"]}
        for c, p in cfg_paths.items():
            if not p.is_file():
                raise SystemExit(f"staged config missing: {p}")

        # BVH lookup via corpus.json (preferred; direct path mapping)
        corpus_json = bench_dir / "corpus.json"
        bvh_lookup = corpus_json if corpus_json.is_file() else (bench_dir.parents[1] / "bones-seed" / "extracted")

        groups = None
        if args.limit_events_groups:
            groups = [g.strip() for g in args.limit_events_groups.split(",") if g.strip()]

        if args.limit_events_category == "both":
            cats = ["slam", "pin"]
        elif args.limit_events_category == "all":
            cats = ["slam", "pin", "twist"]
        else:
            cats = [args.limit_events_category]
        for cat in cats:
            print(f"[INFO] rendering top-{args.side_by_side_events} {cat} events per config "
                  f"(groups={groups or 'all'}, human_row={not args.no_human_row})")
            cat_pngs = bench_side_by_side.build_all(
                bench_dir=bench_dir,
                bvh_dir=bvh_lookup,
                cfg_paths=cfg_paths,
                per_config=args.side_by_side_events,
                category=cat,
                joint_groups=groups,
                include_human_row=not args.no_human_row,
                display_names=display_names or None,
            )
            sxs_pngs.update(cat_pngs)
            bench_kinematics.clear_targets_cache()  # bound memory between passes

        # Anatomical fill: ensure each anatomical section in the doc has a
        # decent number of rendered examples. The per-category passes above
        # are biased toward the dominant joint per category (e.g. elbow
        # slams shadow wrist_roll slams in the corpus-wide top-N), so this
        # pass adds the gaps without re-rendering anything already on disk.
        per_zone = max(5, min(args.side_by_side_events, 8))
        print(f"[INFO] anatomical-fill pass: per_zone={per_zone}")
        fill_pngs = bench_side_by_side.build_anatomical_fill(
            bench_dir=bench_dir,
            bvh_dir=bvh_lookup,
            cfg_paths=cfg_paths,
            per_zone=per_zone,
            include_human_row=not args.no_human_row,
            display_names=display_names or None,
        )
        sxs_pngs.update(fill_pngs)
        bench_kinematics.clear_targets_cache()

    # Reload corpus entries (for tier/category annotations in the md)
    corpus_entries = []
    corpus_json = bench_dir / "corpus.json"
    if corpus_json.is_file():
        try:
            corpus_entries = json.loads(corpus_json.read_text())
        except Exception:
            corpus_entries = []

    # Build the chart set first so it can be embedded in limit_events.md.
    chart_pngs: dict[str, Path] = {}
    try:
        charts = bench_charts.build_all(
            bench_dir,
            display_names=display_names or None,
            verbose=True,
        )
        # Order matters: this is the order they appear in the doc.
        for key in ("event_counts", "twist_severity", "events_by_group"):
            if key in charts:
                chart_pngs[key] = charts[key]
    except Exception as exc:
        print(f"[WARN] chart generation failed: {exc}")

    bench_limit_events.write_limit_events_md(
        bench_dir / "limit_events.md",
        data,
        sxs_pngs_by_event_id=sxs_pngs if sxs_pngs else None,
        corpus_entries=corpus_entries,
        top_events_per_cfg=args.side_by_side_events if args.side_by_side_events else None,
        display_names=display_names or None,
        chart_pngs=chart_pngs or None,
    )
    print(f"[OK] wrote {bench_dir / 'limit_events.md'}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ours", type=Path, default=DEFAULT_OURS, help="First retargeter config JSON (config A)")
    ap.add_argument("--theirs", type=Path, default=DEFAULT_THEIRS, help="Second retargeter config JSON (config B)")
    ap.add_argument("--our-label", type=str, default="config_a",
                    help="Short identifier for the first config (used in JSON keys / on-disk render paths). "
                         "Pass --display-name to give it a human-readable label in the docs.")
    ap.add_argument("--their-label", type=str, default="config_b",
                    help="Short identifier for the second config (used in JSON keys / on-disk render paths). "
                         "Pass --display-name to give it a human-readable label in the docs.")
    ap.add_argument("--extra-config", action="append", default=None, metavar="LABEL=PATH",
                    help="Add an additional retargeter config to the run, beyond --ours/--theirs. "
                         "Repeatable: --extra-config hybrid=path/to/cfg.json --extra-config foo=path/to/foo.json")
    ap.add_argument("--skip-existing-csv", action="store_true",
                    help="During the retarget loop, skip any (clip, config) pair whose CSV already "
                         "exists under csvs/. Use this when adding a new config via --extra-config "
                         "to a bench dir whose other configs are already retargeted.")

    ap.add_argument("--out-dir", type=Path, default=None,
                    help="Override output dir (default: scratch/bench_<ts>)")
    ap.add_argument("--use-existing-dir", type=Path, default=None,
                    help="Reuse an existing bench dir (skip screening if corpus_stats.json present)")

    ap.add_argument("--max-frames", type=int, default=3000,
                    help="Cap BVH frames per clip during retargeting (default 3000 ~ 25s @ 120fps)")
    ap.add_argument("--render-section-cap", type=int, default=5,
                    help="Max section peaks to render per (clip, config) pair")
    ap.add_argument("--smoke", action="store_true",
                    help="Smoke run: 1 walk + 1 manipulation clip only")
    ap.add_argument("--corpus-limit", type=int, default=None,
                    help="Limit the corpus to the first N entries (after assembly)")

    ap.add_argument("--screen-only", action="store_true",
                    help="Run only the BVH screening + corpus assembly, then stop")
    ap.add_argument("--screen-max-files", type=int, default=None,
                    help="Limit number of BVH candidates during screening (debugging)")

    # ----- slam/pin (limit-events) analysis on an EXISTING bench dir -----
    ap.add_argument("--analyze-limit-events", type=Path, default=None,
                    help="Existing bench dir to analyze for slam/pin events. Skips retargeting; reads csvs/ and writes limit_events.json + limit_events.md.")
    ap.add_argument("--side-by-side-events", type=int, default=0,
                    help="When set with --analyze-limit-events, render top-N events per config as side-by-side strips.")
    ap.add_argument("--limit-events-category", type=str, default="both",
                    choices=["slam", "pin", "twist", "both", "all"],
                    help="Which category to render side-by-side for. 'both' = slam+pin (default), 'all' = slam+pin+twist.")
    ap.add_argument("--limit-events-groups", type=str, default="wrist,elbow,shoulder",
                    help="Comma-separated joint groups to restrict side-by-side events to (default: wrist,elbow,shoulder). Pass empty string for all.")
    ap.add_argument("--display-name", action="append", default=None,
                    help="Map a config key to a human-readable label, repeatable: "
                         "--display-name x2_uniform_h140=h=1.40 --display-name x2_uniform_h170_tuned=h=1.70+wrist_smooth")
    ap.add_argument("--no-human-row", action="store_true",
                    help="Skip the human BVH skeleton row in side-by-side renders")
    args = ap.parse_args()

    # Short-circuit: analysis on existing bench dir, no retargeting.
    if args.analyze_limit_events is not None:
        _run_limit_events_analysis(args)
        return

    t_total = time.time()

    if args.use_existing_dir is not None:
        out_dir = args.use_existing_dir.resolve()
        if not out_dir.is_dir():
            raise SystemExit(f"--use-existing-dir does not exist: {out_dir}")
    elif args.out_dir is not None:
        out_dir = args.out_dir.resolve()
        out_dir.mkdir(parents=True, exist_ok=True)
    else:
        ts = time.strftime("%Y%m%d_%H%M%S")
        out_dir = (REPO_ROOT / "scratch" / f"bench_{ts}").resolve()
        out_dir.mkdir(parents=True, exist_ok=True)
    print(f"[INFO]: bench out_dir = {out_dir}")

    # Stage configs (primary pair + any --extra-config additions)
    extra_configs = _parse_extra_configs(args.extra_config)
    primary_list = [
        (args.ours.resolve(),   args.our_label),
        (args.theirs.resolve(), args.their_label),
    ] + [(path, label) for label, path in extra_configs]
    cfg_paths = stage_configs(out_dir, primary_list)

    # Screening + corpus assembly
    corpus_path = out_dir / "corpus.json"
    stats_path = out_dir / "corpus_stats.json"
    if stats_path.is_file() and corpus_path.is_file() and args.use_existing_dir is not None:
        print(f"[INFO]: reusing existing corpus_stats + corpus from {out_dir}")
        stats_raw = json.loads(stats_path.read_text())
        # Reconstruct minimal ClipStats objects for assembly. Easier just to skip
        # rebuilding if corpus.json is present.
        corpus_entries = json.loads(corpus_path.read_text())
    else:
        stats = bench_corpus.run_screening(out_dir, max_files=args.screen_max_files)
        entries = bench_corpus.assemble_corpus(stats, out_dir)
        corpus_entries = [asdict(e) for e in entries]

    if args.smoke:
        # Keep 1 walk + 1 manipulation clip
        walk = next((e for e in corpus_entries if e["category"] == "locowalk"), None)
        manip = next((e for e in corpus_entries if e["category"] == "standing-manipulation"), None)
        corpus_entries = [c for c in [walk, manip] if c is not None]
        print(f"[INFO]: smoke mode -> {len(corpus_entries)} clips")
    elif args.corpus_limit is not None:
        corpus_entries = corpus_entries[: args.corpus_limit]
        print(f"[INFO]: corpus-limit -> {len(corpus_entries)} clips")

    if args.screen_only:
        print("[INFO]: --screen-only set; stopping after screening + corpus assembly.")
        return

    # Main loop
    configs_order = [args.our_label, args.their_label] + [label for label, _ in extra_configs]
    per_clip: dict[str, dict] = {}
    sections_per_pair: dict[tuple[str, str], list[dict]] = {}
    section_pngs: dict[tuple[str, str], list[Path]] = {}
    section_counts: dict[tuple[str, str], int] = {}
    section_total = {c: 0 for c in configs_order}
    runtime_acc = dict(retarget_s=0.0, metrics_s=0.0, render_s=0.0)

    for i, entry in enumerate(corpus_entries):
        print(f"\n[INFO]: ({i+1}/{len(corpus_entries)}) [{entry['tier']}/{entry['category']}] {entry['name']}")
        per_clip.setdefault(entry["name"], {})
        for cfg_label in configs_order:
            try:
                rec = run_pair(
                    clip_entry=entry,
                    config_label=cfg_label,
                    skip_existing_csv=args.skip_existing_csv,
                    config_path=cfg_paths[cfg_label],
                    out_dir=out_dir,
                    max_frames=args.max_frames,
                    render_section_cap=args.render_section_cap,
                )
            except Exception as e:
                print(f"[ERROR]: clip {entry['name']} with config {cfg_label} failed: {e}")
                continue

            per_clip[entry["name"]][cfg_label] = rec["metrics"]
            sections_per_pair[(entry["name"], cfg_label)] = rec["sections"]
            section_pngs[(entry["name"], cfg_label)] = [Path(p) for p in rec["pngs"]]
            section_counts[(entry["name"], cfg_label)] = len(rec["sections"])
            section_total[cfg_label] += len(rec["sections"])
            runtime_acc["retarget_s"] += rec.get("retarget_s", 0.0)
            runtime_acc["metrics_s"] += rec.get("metrics_s", 0.0)
            runtime_acc["render_s"] += rec.get("render_s", 0.0)

            # Persist a per-pair JSON for forensics
            forensic_dir = out_dir / "frames" / f"{Path(entry['name']).stem}__{cfg_label}"
            forensic_dir.mkdir(parents=True, exist_ok=True)
            (forensic_dir / "_sections.json").write_text(
                json.dumps({"flags": rec["flags"], "sections": rec["sections"]}, indent=2, default=float)
            )

        # Drop cached BVH targets between clips to keep memory bounded.
        bench_kinematics.clear_targets_cache()

    runtime_acc["total_s"] = float(time.time() - t_total)

    # Parse --display-name flags (also accepted on the analysis path).
    display_names: dict[str, str] = {}
    for spec in (args.display_name or []):
        if "=" in spec:
            k, v = spec.split("=", 1)
            display_names[k.strip()] = v.strip()

    # Aggregate outputs
    bench_aggregate.dump_metrics_json(out_dir / "metrics.json", per_clip)
    bench_aggregate.write_summary_md(
        out_dir / "summary.md",
        per_clip=per_clip,
        configs=configs_order,
        corpus_entries=corpus_entries,
        section_counts=section_counts,
        display_names=display_names,
    )

    # Foot-floor contact chart (FK-derived). The limit_events.md flow rebuilds
    # all charts via bench_charts.build_all; here we only need the
    # metrics-driven chart since limit_events.json may not exist yet.
    try:
        bench_charts.chart_foot_floor(
            per_clip=per_clip,
            configs=configs_order,
            out_path=out_dir / "frames" / "charts" / "foot_floor.png",
            display_names=display_names or None,
        )
        print(f"[OK] chart `foot_floor` -> {out_dir / 'frames' / 'charts' / 'foot_floor.png'}")
    except Exception as exc:
        print(f"[WARN] foot_floor chart skipped: {exc}")
    # NOTE: ik_failures.md is no longer written. It was a 4-saturated-DOF
    # detector that fires on benign multi-limit poses (mostly leg/ankle near
    # foot strike), which proved noisy and misleading. The slam/pin/twist
    # events in limit_events.md replace it.
    bench_aggregate.write_report_md_skeleton(
        out_dir / "REPORT.md",
        out_dir=out_dir,
        per_clip=per_clip,
        configs=configs_order,
        corpus_entries=corpus_entries,
        section_counts=section_counts,
        section_total=section_total,
        runtime_summary=runtime_acc,
        display_names=display_names,
    )
    print(f"\n[OK] bench complete in {runtime_acc['total_s']:.0f}s — see {out_dir}")
    print(f"     summary.md / REPORT.md / metrics.json")


if __name__ == "__main__":
    main()

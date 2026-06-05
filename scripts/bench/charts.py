"""Bar charts summarising slam / pin / twist events for a bench dir.

Reads `{bench_dir}/limit_events.json` and writes a small set of PNGs under
`{bench_dir}/frames/charts/`:

  - `event_counts.png`     — total slam vs pin vs twist per config (grouped bars)
  - `twist_severity.png`   — twist warn vs severe per config
  - `events_by_group.png`  — slam+pin+twist counts per joint group, per config

These charts are designed to be embeddable in `summary.md` / `REPORT.md` /
`limit_events.md` so the rest of the docs don't need to lean on dense tables.
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


# A neutral, print-friendly palette
_PALETTE = {
    "slam":  "#E74C3C",  # red
    "pin":   "#F1C40F",  # yellow
    "twist": "#3498DB",  # blue
    "warn":  "#F39C12",  # amber
    "severe": "#C0392B", # dark red
}


def _categorize(data: dict) -> dict:
    """Return {cfg: {slam: n, pin: n, twist: n, twist_warn: n, twist_severe: n}}."""
    out: dict[str, dict[str, int]] = {}
    for cfg in data["configs"]:
        n = dict(slam=0, pin=0, twist=0, twist_warn=0, twist_severe=0)
        for clip, by_cfg in data["clips"].items():
            for ev in by_cfg.get(cfg, {}).get("events", []):
                cat = ev["category"]
                n[cat] = n.get(cat, 0) + 1
                if cat == "twist":
                    sev = ev.get("severity", "warn")
                    n[f"twist_{sev}"] += 1
        out[cfg] = n
    return out


def _by_group(data: dict) -> dict:
    """Return {cfg: {group: {slam: n, pin: n, twist: n}}}."""
    from scripts.bench.joint_limits import JOINT_GROUPS
    out: dict[str, dict[str, dict[str, int]]] = {}
    for cfg in data["configs"]:
        per_group = {g: dict(slam=0, pin=0, twist=0) for g in JOINT_GROUPS}
        for clip, by_cfg in data["clips"].items():
            for ev in by_cfg.get(cfg, {}).get("events", []):
                jn = ev["joint"]
                for g, joints in JOINT_GROUPS.items():
                    if jn in joints:
                        per_group[g][ev["category"]] += 1
                        break
        out[cfg] = per_group
    return out


def _style(ax: "plt.Axes", title: str = "", ylabel: str = "events") -> None:
    ax.set_title(title, fontsize=13, fontweight="bold", color="#222")
    ax.set_ylabel(ylabel, fontsize=10)
    ax.grid(axis="y", alpha=0.25, linestyle="--", linewidth=0.5)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def _annotate(ax: "plt.Axes", rects: "list[plt.Rectangle]", color: str = "#222") -> None:
    for r in rects:
        h = r.get_height()
        if h <= 0:
            continue
        ax.annotate(
            f"{int(h)}",
            xy=(r.get_x() + r.get_width() / 2.0, h),
            xytext=(0, 3),
            textcoords="offset points",
            ha="center", va="bottom",
            fontsize=9, color=color, fontweight="bold",
        )


def chart_event_counts(
    data: dict,
    out_path: Path,
    *,
    display_names: dict[str, str] | None = None,
) -> Path:
    """Grouped bars: slam / pin / twist for each config."""
    display_names = display_names or {}
    cfgs = list(data["configs"])
    cat_counts = _categorize(data)
    cats = ["slam", "pin", "twist"]
    x = np.arange(len(cats))
    width = 0.36

    fig, ax = plt.subplots(figsize=(5.4, 3.2), dpi=110)
    for i, cfg in enumerate(cfgs):
        vals = [cat_counts[cfg][c] for c in cats]
        rects = ax.bar(
            x + (i - 0.5) * width,
            vals,
            width,
            label=display_names.get(cfg, cfg),
            color=[_PALETTE["slam"], _PALETTE["pin"], _PALETTE["twist"]],
            alpha=0.55 if i == 0 else 0.95,
            edgecolor="#111",
            linewidth=0.6,
        )
        _annotate(ax, rects)
    ax.set_xticks(x)
    ax.set_xticklabels(["slam\n(basin hop)", "pin\n(workspace exhaustion)", "twist\n(palm-twist > 80°)"], fontsize=10)
    _style(ax, title="IK failure events across 49 clips", ylabel="event count")
    ax.legend(loc="upper right", frameon=False, fontsize=10)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return out_path


def chart_twist_severity(
    data: dict,
    out_path: Path,
    *,
    display_names: dict[str, str] | None = None,
) -> Path:
    """Stacked bars: twist warn + severe per config."""
    display_names = display_names or {}
    cfgs = list(data["configs"])
    cc = _categorize(data)
    fig, ax = plt.subplots(figsize=(4.6, 3.0), dpi=110)
    x = np.arange(len(cfgs))
    width = 0.55
    warns  = [cc[c]["twist_warn"]   for c in cfgs]
    severs = [cc[c]["twist_severe"] for c in cfgs]
    r1 = ax.bar(x, warns,  width, color=_PALETTE["warn"],   edgecolor="#111", linewidth=0.6, label="warn  (80° ≤ |palm-twist| < 100°)")
    r2 = ax.bar(x, severs, width, bottom=warns, color=_PALETTE["severe"], edgecolor="#111", linewidth=0.6, label="severe (|palm-twist| ≥ 100°)")
    _annotate(ax, r1)
    for r, w in zip(r2, warns):
        h = r.get_height()
        if h <= 0:
            continue
        ax.annotate(
            f"+{int(h)}",
            xy=(r.get_x() + r.get_width() / 2.0, w + h),
            xytext=(0, 3),
            textcoords="offset points",
            ha="center", va="bottom",
            fontsize=9, color="#111", fontweight="bold",
        )
    ax.set_xticks(x)
    ax.set_xticklabels([display_names.get(c, c) for c in cfgs], fontsize=11)
    _style(ax, title="Palm-twist excursions (wrist_yaw) by severity", ylabel="event count")
    ax.legend(loc="upper left", frameon=False, fontsize=9)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return out_path


def chart_events_by_group(
    data: dict,
    out_path: Path,
    *,
    display_names: dict[str, str] | None = None,
    groups: list[str] | None = None,
) -> Path:
    """Per joint group, grouped bars by config (slam+pin+twist combined)."""
    display_names = display_names or {}
    cfgs = list(data["configs"])
    by_g = _by_group(data)
    if groups is None:
        groups = ["wrist", "elbow", "shoulder", "hip", "knee", "ankle", "waist"]
    # Combined event count = slam + pin + twist
    fig, ax = plt.subplots(figsize=(6.4, 3.4), dpi=110)
    x = np.arange(len(groups))
    width = 0.36
    for i, cfg in enumerate(cfgs):
        totals = [
            by_g[cfg][g]["slam"] + by_g[cfg][g]["pin"] + by_g[cfg][g]["twist"]
            for g in groups
        ]
        rects = ax.bar(
            x + (i - 0.5) * width,
            totals,
            width,
            label=display_names.get(cfg, cfg),
            color=("#7F8C8D" if i == 0 else "#34495E"),
            edgecolor="#111", linewidth=0.6,
        )
        _annotate(ax, rects)
    ax.set_xticks(x)
    ax.set_xticklabels(groups, fontsize=10)
    _style(ax, title="Total IK-failure events by joint group  (slam + pin + twist)", ylabel="event count")
    ax.legend(loc="upper right", frameon=False, fontsize=10)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return out_path


def build_all(
    bench_dir: Path,
    *,
    display_names: dict[str, str] | None = None,
    verbose: bool = True,
) -> dict[str, Path]:
    """Build the standard chart set under `{bench_dir}/frames/charts/`."""
    data = json.loads((bench_dir / "limit_events.json").read_text())
    out_dir = bench_dir / "frames" / "charts"
    out_dir.mkdir(parents=True, exist_ok=True)

    out: dict[str, Path] = {}
    out["event_counts"]    = chart_event_counts(data, out_dir / "event_counts.png", display_names=display_names)
    out["twist_severity"]  = chart_twist_severity(data, out_dir / "twist_severity.png", display_names=display_names)
    out["events_by_group"] = chart_events_by_group(data, out_dir / "events_by_group.png", display_names=display_names)
    if verbose:
        for name, p in out.items():
            print(f"[OK] chart `{name}` -> {p}")
    return out


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> None:
    import argparse
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bench-dir", type=Path, required=True)
    ap.add_argument("--display-name", action="append", default=None,
                    help="Map a config key to a display label; repeatable: --display-name v5_ours=h=1.40")
    args = ap.parse_args()

    display_names: dict[str, str] = {}
    if args.display_name:
        for pair in args.display_name:
            if "=" in pair:
                k, v = pair.split("=", 1)
                display_names[k.strip()] = v.strip()

    build_all(args.bench_dir.resolve(), display_names=display_names or None)


if __name__ == "__main__":
    main()

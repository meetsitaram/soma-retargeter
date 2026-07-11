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


def _bar_layout(n_configs: int, group_span: float = 0.86) -> tuple[float, "list[float]"]:
    """Return (bar_width, [offsets]) for `n_configs` bars centred on each tick.

    `group_span` is the total fraction of the tick spacing occupied by all
    bars together. A smaller span = wider gaps between groups.
    """
    n = max(1, int(n_configs))
    width = group_span / n
    # Offsets so the bars are centred around each x tick:
    #   for n=2 -> [-0.5w, +0.5w]
    #   for n=3 -> [-w, 0, +w]
    offsets = [(i - (n - 1) / 2.0) * width for i in range(n)]
    return width, offsets


# Per-config visual styling. For category-coloured charts (event_counts),
# we vary alpha so each config still uses the slam/pin/twist palette but is
# distinguishable. For solid-colour charts (events_by_group), we cycle
# through a 4-tier neutral palette.
_CONFIG_ALPHA = [0.45, 0.7, 0.95, 1.0]
_CONFIG_NEUTRAL = ["#95A5A6", "#566573", "#2C3E50", "#17202A"]


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
    width, offsets = _bar_layout(len(cfgs))

    # Scale figure width with number of configs so the bars don't squish.
    fig, ax = plt.subplots(figsize=(5.4 + 0.6 * max(0, len(cfgs) - 2), 3.2), dpi=110)
    for i, cfg in enumerate(cfgs):
        vals = [cat_counts[cfg][c] for c in cats]
        rects = ax.bar(
            x + offsets[i],
            vals,
            width,
            label=display_names.get(cfg, cfg),
            color=[_PALETTE["slam"], _PALETTE["pin"], _PALETTE["twist"]],
            alpha=_CONFIG_ALPHA[min(i, len(_CONFIG_ALPHA) - 1)],
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
    # Widen the figure based on the number of configs so long descriptive
    # labels (e.g. "h=1.40+wrist_smooth") don't collide on the x axis.
    fig_w = max(4.6, 2.2 * len(cfgs) + 1.0)
    fig, ax = plt.subplots(figsize=(fig_w, 3.4), dpi=110)
    x = np.arange(len(cfgs))
    width = 0.55
    warns  = [cc[c]["twist_warn"]   for c in cfgs]
    severs = [cc[c]["twist_severe"] for c in cfgs]
    r1 = ax.bar(x, warns,  width, color=_PALETTE["warn"],   edgecolor="#111", linewidth=0.6, label="warn  (80° ≤ |palm-twist| < 100°)")
    r2 = ax.bar(x, severs, width, bottom=warns, color=_PALETTE["severe"], edgecolor="#111", linewidth=0.6, label="severe (|palm-twist| ≥ 100°)")
    # Annotate the "warn" bar on top of itself (inside the stack) so the
    # number doesn't collide with the "+severe" annotation above it.
    for r in r1:
        h = r.get_height()
        if h <= 0:
            continue
        ax.annotate(
            f"{int(h)}",
            xy=(r.get_x() + r.get_width() / 2.0, h / 2.0),
            ha="center", va="center",
            fontsize=9, color="#111", fontweight="bold",
        )
    for r, w in zip(r2, warns):
        h = r.get_height()
        if h <= 0:
            continue
        ax.annotate(
            f"+{int(h)} severe",
            xy=(r.get_x() + r.get_width() / 2.0, w + h),
            xytext=(0, 3),
            textcoords="offset points",
            ha="center", va="bottom",
            fontsize=9, color="#111", fontweight="bold",
        )
    ax.set_xticks(x)
    # Mild rotation if any descriptive label is long.
    labels = [display_names.get(c, c) for c in cfgs]
    rot = 12 if max((len(l) for l in labels), default=0) > 10 else 0
    ax.set_xticklabels(labels, fontsize=10, rotation=rot, ha="right" if rot else "center")
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
    fig, ax = plt.subplots(figsize=(6.4 + 0.6 * max(0, len(cfgs) - 2), 3.4), dpi=110)
    x = np.arange(len(groups))
    width, offsets = _bar_layout(len(cfgs))
    for i, cfg in enumerate(cfgs):
        totals = [
            by_g[cfg][g]["slam"] + by_g[cfg][g]["pin"] + by_g[cfg][g]["twist"]
            for g in groups
        ]
        rects = ax.bar(
            x + offsets[i],
            totals,
            width,
            label=display_names.get(cfg, cfg),
            color=_CONFIG_NEUTRAL[min(i, len(_CONFIG_NEUTRAL) - 1)],
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


def chart_foot_floor(
    per_clip: dict,
    configs: list[str],
    out_path: Path,
    *,
    display_names: dict[str, str] | None = None,
) -> Path:
    """Grouped bar chart of floating/penetration % per config.

    Input data is the metrics dict ({clip: {config: ClipMetrics-asdict, ...}})
    rather than ``limit_events.json``, since these are FK-derived metrics
    rather than slam/pin/twist events.
    """
    display_names = display_names or {}
    cfgs = list(configs)
    # Aggregate (mean across clips) — same convention as summary.md row.
    def _mean_for(cfg: str, key: str) -> float:
        vals = []
        for clip_name, by_cfg in per_clip.items():
            if cfg in by_cfg:
                v = by_cfg[cfg].get(key)
                if v is None:
                    continue
                vals.append(float(v))
        return float(np.mean(vals)) if vals else 0.0

    cats = ["floating_pct", "penetration_pct"]
    cat_labels = ["Floating\n(foot above floor)", "Penetration\n(foot below floor)"]
    x = np.arange(len(cats))
    width, offsets = _bar_layout(len(cfgs))

    fig, ax = plt.subplots(figsize=(5.4 + 0.6 * max(0, len(cfgs) - 2), 3.2), dpi=110)
    # Use red for both because the failure mode is "off the floor in either
    # direction"; vary alpha per config to distinguish.
    for i, cfg in enumerate(cfgs):
        vals = [_mean_for(cfg, k) for k in cats]
        rects = ax.bar(
            x + offsets[i],
            vals,
            width,
            label=display_names.get(cfg, cfg),
            color=["#E67E22", "#E74C3C"],
            alpha=_CONFIG_ALPHA[min(i, len(_CONFIG_ALPHA) - 1)],
            edgecolor="#111",
            linewidth=0.6,
        )
        # Annotate as percentage with one decimal.
        for r in rects:
            h = r.get_height()
            ax.annotate(
                f"{h:.1f}%",
                xy=(r.get_x() + r.get_width() / 2.0, h),
                xytext=(0, 3),
                textcoords="offset points",
                ha="center", va="bottom",
                fontsize=9, color="#111", fontweight="bold",
            )
    ax.set_xticks(x)
    ax.set_xticklabels(cat_labels, fontsize=10)
    _style(ax, title="Foot-floor contact (mean across corpus)", ylabel="% of frames")
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
    # Foot-floor needs metrics.json (FK-derived) rather than limit_events.json.
    metrics_path = bench_dir / "metrics.json"
    if metrics_path.is_file():
        try:
            per_clip = json.loads(metrics_path.read_text())
            configs = list(data["configs"])
            out["foot_floor"] = chart_foot_floor(
                per_clip, configs, out_dir / "foot_floor.png",
                display_names=display_names,
            )
        except Exception as exc:
            print(f"[WARN] foot_floor chart skipped: {exc}")
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
                    help="Map a config key to a display label; repeatable: --display-name x2_uniform_h140=h=1.40")
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

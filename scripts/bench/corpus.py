"""BVH motion-statistics screening + 4-tier corpus assembly.

We compute per-joint Euler-angle statistics directly from the BVH joint tree
(no warp/IK pipeline involved) over a fixed sample of candidate clips, then
query the resulting table to select stress-clips for hips, wrists, and a few
"other extreme" joint regimes.
"""

from __future__ import annotations

import json
import random
from dataclasses import dataclass, asdict, field
from pathlib import Path
from typing import Iterable

import numpy as np

# Reuse the BVH parser already in the package.
from soma_retargeter.assets.bvh import BVHImporter


# ---------------------------------------------------------------------------
# Candidate pool
# ---------------------------------------------------------------------------

# scripts/bench/corpus.py -> .../soma-retargeter/scripts/bench -> parents[3] is
# agibot-x2-references; bones-seed sits next to soma-retargeter under it.
_THIS = Path(__file__).resolve()
SOMA_RETARGETER_ROOT = _THIS.parents[2]
BONES_SEED_ROOT = _THIS.parents[3] / "bones-seed"

CATEGORIES: dict[str, dict] = {
    # name -> dict with filename list path and extracted subdir name
    "locowalk":  {
        "list_path": BONES_SEED_ROOT / "x2-locowalk-filenames.txt",
        "extract_dir": BONES_SEED_ROOT / "extracted" / "locowalk",
        "sample_size": 200,
    },
    "dances": {
        "list_path": BONES_SEED_ROOT / "x2-dances-filenames.txt",
        "extract_dir": BONES_SEED_ROOT / "extracted" / "dances",
        "sample_size": None,  # process all (only 34 files)
    },
    "standing-manipulation": {
        "list_path": BONES_SEED_ROOT / "x2-standing-manipulation-filenames.txt",
        "extract_dir": BONES_SEED_ROOT / "extracted" / "standing-manipulation",
        "sample_size": None,  # process all (550 files)
    },
    "loco-manipulation": {
        "list_path": BONES_SEED_ROOT / "x2-loco-manipulation-filenames.txt",
        "extract_dir": BONES_SEED_ROOT / "extracted" / "loco-manipulation",
        "sample_size": 200,
    },
}


# BVH joints whose stats we care about for corpus screening (SOMA skeleton names).
SCREENING_JOINTS: list[str] = [
    "Hips",
    "Chest",
    "Neck1",
    "LeftArm", "RightArm",
    "LeftForeArm", "RightForeArm",
    "LeftHand", "RightHand",
    "LeftLeg", "RightLeg",
    "LeftShin", "RightShin",
    "LeftFoot", "RightFoot",
]


# ---------------------------------------------------------------------------
# Per-clip stats
# ---------------------------------------------------------------------------

@dataclass
class JointStat:
    min_deg: float = 0.0
    max_deg: float = 0.0
    range_deg: float = 0.0
    mean_deg: float = 0.0
    std_deg: float = 0.0


@dataclass
class ClipStats:
    name: str
    category: str
    path: str
    num_frames: int
    fps: float

    # Per-joint per-axis stats: stats[joint][axis_letter] = JointStat
    stats: dict[str, dict[str, JointStat]] = field(default_factory=dict)

    # Root translation stats (cm in BVH; converted to m below).
    root_tx_m: JointStat = field(default_factory=JointStat)
    root_ty_m: JointStat = field(default_factory=JointStat)
    root_tz_m: JointStat = field(default_factory=JointStat)

    # Composite stress scores (computed by build_scores()).
    hip_stress: float = 0.0
    wrist_stress: float = 0.0
    hip_leg_stress: float = 0.0
    shoulder_swing: float = 0.0
    ankle_swing: float = 0.0
    pelvis_drop_abs_min_m: float = 0.0


def _walk_bvh_joints(root) -> dict[str, object]:
    """Flatten a BVH joint tree to {name: BVHJoint}."""
    out: dict[str, object] = {}
    stack = [root]
    while stack:
        j = stack.pop()
        out[j.name] = j
        stack.extend(j.children)
    return out


def _compute_clip_stats(path: Path, category: str) -> ClipStats | None:
    """Parse a single BVH and accumulate Euler-angle stats per joint axis."""
    try:
        root = BVHImporter.bvh_parser(str(path))
    except Exception as e:
        print(f"[WARN]: skipping {path.name}: {e}")
        return None

    joints = _walk_bvh_joints(root)
    num_frames, fps = BVHImporter.get_frame_range(root)

    cs = ClipStats(
        name=path.name,
        category=category,
        path=str(path),
        num_frames=int(num_frames),
        fps=float(fps),
    )

    # Walk each screening joint and pull its per-axis rotation data.
    for jname in SCREENING_JOINTS:
        if jname not in joints:
            continue
        j = joints[jname]
        rotate_order = j.rotate_order  # e.g. "zyx"
        # j.animation is a list of [channels...] per frame; pick out the rotation
        # channels in order.
        rot_idx_in_frame: list[int] = []
        rot_axis: list[str] = []
        for ch_i, ch in enumerate(j.channels):
            if "rotation" in ch:
                rot_idx_in_frame.append(ch_i)
                rot_axis.append(ch[0].lower())  # 'x'/'y'/'z'

        if not rot_idx_in_frame:
            continue

        # Build per-axis arrays of length num_frames.
        per_axis: dict[str, list[float]] = {"x": [], "y": [], "z": []}
        for f in range(num_frames):
            row = j.animation[f]
            for ch_i, ax in zip(rot_idx_in_frame, rot_axis):
                per_axis[ax].append(float(row[ch_i]))

        cs.stats[jname] = {}
        for ax, vals in per_axis.items():
            if not vals:
                continue
            arr = np.asarray(vals, dtype=np.float64)
            cs.stats[jname][ax] = JointStat(
                min_deg=float(arr.min()),
                max_deg=float(arr.max()),
                range_deg=float(arr.max() - arr.min()),
                mean_deg=float(arr.mean()),
                std_deg=float(arr.std()),
            )

    # Hips root translation (cm -> m). Hips has both position + rotation channels.
    hips = joints.get("Hips")
    if hips is not None:
        pos_idx: list[int] = []
        pos_axis: list[str] = []
        for ch_i, ch in enumerate(hips.channels):
            if "position" in ch:
                pos_idx.append(ch_i)
                pos_axis.append(ch[0].lower())
        per_axis_pos: dict[str, list[float]] = {"x": [], "y": [], "z": []}
        for f in range(num_frames):
            row = hips.animation[f]
            for ch_i, ax in zip(pos_idx, pos_axis):
                per_axis_pos[ax].append(float(row[ch_i]) * 0.01)  # cm -> m
        for ax, arr_list in per_axis_pos.items():
            if not arr_list:
                continue
            arr = np.asarray(arr_list, dtype=np.float64)
            stat = JointStat(
                min_deg=float(arr.min()),
                max_deg=float(arr.max()),
                range_deg=float(arr.max() - arr.min()),
                mean_deg=float(arr.mean()),
                std_deg=float(arr.std()),
            )
            if ax == "x":
                cs.root_tx_m = stat
            elif ax == "y":
                cs.root_ty_m = stat
            elif ax == "z":
                cs.root_tz_m = stat

    _populate_scores(cs)
    return cs


def _range(cs: ClipStats, joint: str, axis: str) -> float:
    return cs.stats.get(joint, {}).get(axis, JointStat()).range_deg


def _populate_scores(cs: ClipStats) -> None:
    """Composite stress scores used for tier-2/3/4 selection.

    SOMA BVH convention (observed): Y is the vertical axis (pelvis height
    ~0.93-1.01m on a walk), Z is the forward walking direction, X is lateral
    sway. The Hips Y-rotation channel is hip yaw twist — that's the user's
    primary complaint, so we drive hip_stress off that range. We deliberately
    do NOT include translation Z range (forward walking distance) here because
    that biases toward "longer walks" rather than "more twisting".
    """
    cs.hip_stress = (
        _range(cs, "Hips", "y")
        + 0.5 * _range(cs, "Chest", "y")
    )
    cs.wrist_stress = (
        (_range(cs, "LeftHand", "x") + _range(cs, "LeftHand", "y") + _range(cs, "LeftHand", "z")) / 3.0
        + (_range(cs, "RightHand", "x") + _range(cs, "RightHand", "y") + _range(cs, "RightHand", "z")) / 3.0
        + 0.5 * (_range(cs, "LeftForeArm", "y") + _range(cs, "RightForeArm", "y"))
    )
    cs.hip_leg_stress = _range(cs, "LeftLeg", "x") + _range(cs, "RightLeg", "x")
    cs.shoulder_swing = _range(cs, "LeftArm", "z") + _range(cs, "RightArm", "z")
    cs.ankle_swing = _range(cs, "LeftFoot", "y") + _range(cs, "RightFoot", "y")
    # Lowest pelvis Y (= deepest crouch / squat / lunge). Using the absolute
    # *value* of the min so 0.0 means "never went below pelvis_min during clip".
    cs.pelvis_drop_abs_min_m = max(0.0, 0.95 - cs.root_ty_m.min_deg)  # m below ~normal stance


# ---------------------------------------------------------------------------
# Top-level screening pass
# ---------------------------------------------------------------------------

def sample_candidates(seed: int = 0) -> list[tuple[str, Path]]:
    """Return a list of (category, bvh_path) candidates from all categories."""
    rng = random.Random(seed)
    out: list[tuple[str, Path]] = []
    for cat, info in CATEGORIES.items():
        list_path: Path = info["list_path"]
        extract_dir: Path = info["extract_dir"]
        sample_size = info["sample_size"]

        if not list_path.is_file():
            print(f"[WARN]: filename list not found: {list_path}")
            continue
        names = [ln.strip() for ln in list_path.read_text().splitlines() if ln.strip()]
        if sample_size is not None and sample_size < len(names):
            names = rng.sample(names, sample_size)
        for n in names:
            p = extract_dir / n
            if p.is_file():
                out.append((cat, p))
    return out


def run_screening(out_dir: Path, seed: int = 0, max_files: int | None = None) -> list[ClipStats]:
    """Run the full BVH screening pass and write corpus_stats.json/.md."""
    candidates = sample_candidates(seed=seed)
    if max_files is not None:
        candidates = candidates[:max_files]
    print(f"[INFO]: screening {len(candidates)} BVH candidates ...")

    all_stats: list[ClipStats] = []
    for i, (cat, p) in enumerate(candidates):
        if i % 50 == 0:
            print(f"  [{i}/{len(candidates)}] {cat}/{p.name}")
        cs = _compute_clip_stats(p, cat)
        if cs is not None:
            all_stats.append(cs)

    out_dir.mkdir(parents=True, exist_ok=True)
    _write_corpus_stats_json(out_dir / "corpus_stats.json", all_stats)
    _write_corpus_stats_md(out_dir / "corpus_stats.md", all_stats)
    print(f"[INFO]: corpus_stats -> {out_dir}/corpus_stats.{{json,md}}  ({len(all_stats)} clips)")
    return all_stats


def _stats_to_dict(cs: ClipStats) -> dict:
    d = asdict(cs)
    # Convert nested JointStat dataclasses (already dicts via asdict()) — fine.
    return d


def _write_corpus_stats_json(path: Path, stats: list[ClipStats]) -> None:
    path.write_text(json.dumps([_stats_to_dict(cs) for cs in stats], indent=2))


def _write_corpus_stats_md(path: Path, stats: list[ClipStats]) -> None:
    lines = [
        "# Corpus screening — per-clip stress scores",
        "",
        "Top-3 joints by Euler-axis range, plus the composite stress scores used for corpus selection.",
        "",
        "| clip | cat | frames | top1 range | top2 range | top3 range | hip_stress | wrist_stress | hip_leg | shoulder_swing | ankle_swing | pelvis_drop_m |",
        "|------|-----|-------:|-----------:|-----------:|-----------:|-----------:|-------------:|--------:|---------------:|------------:|--------------:|",
    ]
    for cs in stats:
        flat = []
        for jn, axes in cs.stats.items():
            for ax, st in axes.items():
                flat.append((st.range_deg, f"{jn}.{ax}={st.range_deg:.0f}d"))
        flat.sort(reverse=True)
        top3 = [f for _, f in flat[:3]] + ["-"] * 3
        lines.append(
            f"| {cs.name} | {cs.category} | {cs.num_frames} | {top3[0]} | {top3[1]} | {top3[2]} "
            f"| {cs.hip_stress:.1f} | {cs.wrist_stress:.1f} "
            f"| {cs.hip_leg_stress:.1f} | {cs.shoulder_swing:.1f} | {cs.ankle_swing:.1f} "
            f"| {cs.pelvis_drop_abs_min_m:.2f} |"
        )
    path.write_text("\n".join(lines) + "\n")


# ---------------------------------------------------------------------------
# 4-tier corpus assembly
# ---------------------------------------------------------------------------

@dataclass
class CorpusEntry:
    name: str
    category: str
    path: str
    tier: str             # "random" / "hip" / "wrist" / "leg" / "shoulder" / "pelvis" / "ankle" / "anchor"
    score: float          # the score that selected it (or 0 for random/anchor)
    num_frames: int


def _top_per_category_by(stats: list[ClipStats], score_fn, k_per_cat: int, exclude: set[str]) -> list[ClipStats]:
    """Pick top-k clips per category by score_fn, skipping any already in exclude."""
    by_cat: dict[str, list[ClipStats]] = {}
    for cs in stats:
        if cs.name in exclude:
            continue
        by_cat.setdefault(cs.category, []).append(cs)

    picked: list[ClipStats] = []
    for cat, items in by_cat.items():
        items.sort(key=score_fn, reverse=True)
        picked.extend(items[:k_per_cat])
    return picked


def assemble_corpus(
    stats: list[ClipStats],
    out_dir: Path,
    anchor_clip: str = "walk_forward_loop_001__A021.bvh",
    seed: int = 0,
) -> list[CorpusEntry]:
    """Build the 4-tier corpus and write corpus.json. Returns the entry list."""

    name_to_stats: dict[str, ClipStats] = {cs.name: cs for cs in stats}
    selected: dict[str, CorpusEntry] = {}

    def add(cs: ClipStats, tier: str, score: float) -> None:
        if cs.name in selected:
            return
        selected[cs.name] = CorpusEntry(
            name=cs.name,
            category=cs.category,
            path=cs.path,
            tier=tier,
            score=float(score),
            num_frames=cs.num_frames,
        )

    # Tier 1 — 5 random per category (20 clips).
    rng = random.Random(seed)
    by_cat: dict[str, list[ClipStats]] = {}
    for cs in stats:
        by_cat.setdefault(cs.category, []).append(cs)
    for cat, items in sorted(by_cat.items()):
        pool = list(items)
        rng.shuffle(pool)
        for cs in pool[:5]:
            add(cs, "random", 0.0)

    # Tier 2 — hip-stress, top-2 per category.
    for cs in _top_per_category_by(stats, lambda c: c.hip_stress, 2, set(selected.keys())):
        add(cs, "hip", cs.hip_stress)

    # Tier 3 — wrist-stress, top-2 per category.
    for cs in _top_per_category_by(stats, lambda c: c.wrist_stress, 2, set(selected.keys())):
        add(cs, "wrist", cs.wrist_stress)

    # Tier 4 — other-joint extremes.
    for cs in _top_per_category_by(stats, lambda c: c.hip_leg_stress, 1, set(selected.keys())):
        add(cs, "leg", cs.hip_leg_stress)
    for cs in _top_per_category_by(stats, lambda c: c.shoulder_swing, 1, set(selected.keys())):
        add(cs, "shoulder", cs.shoulder_swing)
    # pelvis_drop — 1 overall (max abs(min z))
    drop_sorted = sorted([cs for cs in stats if cs.name not in selected], key=lambda c: c.pelvis_drop_abs_min_m, reverse=True)
    if drop_sorted:
        add(drop_sorted[0], "pelvis", drop_sorted[0].pelvis_drop_abs_min_m)
    for cs in _top_per_category_by(stats, lambda c: c.ankle_swing, 1, set(selected.keys())):
        add(cs, "ankle", cs.ankle_swing)

    # Anchor — fixed clip for continuity. Add even if it's not in the screening pool.
    if anchor_clip in name_to_stats:
        cs = name_to_stats[anchor_clip]
        # Force-add (override tier) so anchor is preserved.
        selected[cs.name] = CorpusEntry(
            name=cs.name, category=cs.category, path=cs.path,
            tier="anchor", score=0.0, num_frames=cs.num_frames,
        )
    else:
        # Look up directly on disk if it isn't in the sampled stats.
        for cat_info in CATEGORIES.values():
            p = cat_info["extract_dir"] / anchor_clip
            if p.is_file():
                selected[anchor_clip] = CorpusEntry(
                    name=anchor_clip, category="locowalk", path=str(p),
                    tier="anchor", score=0.0, num_frames=0,
                )
                break

    entries = sorted(selected.values(), key=lambda e: (
        # Tier order then category then name (deterministic)
        {"anchor": 0, "hip": 1, "wrist": 2, "leg": 3, "shoulder": 4, "pelvis": 5, "ankle": 6, "random": 7}.get(e.tier, 99),
        e.category, e.name,
    ))

    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "corpus.json").write_text(json.dumps([asdict(e) for e in entries], indent=2))
    print(f"[INFO]: corpus.json with {len(entries)} clips -> {out_dir}/corpus.json")

    # Brief stdout summary.
    by_tier: dict[str, int] = {}
    for e in entries:
        by_tier[e.tier] = by_tier.get(e.tier, 0) + 1
    print("[INFO]:   tier counts:", by_tier)

    return entries

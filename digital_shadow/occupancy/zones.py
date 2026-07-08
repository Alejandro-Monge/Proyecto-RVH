"""Approach occupancy zones — fail-fast loader + PURE instantaneous per-frame counting.

Layer independent of the causal pipeline. It reads
config/sites/<site>/approach_zones.yaml (a file SEPARATE from zones.yaml on purpose:
its own life cycle and provenance pin) and offers the occupancy count per arm/lane of
the present frame. No causal production imports this package.

WHAT IT IS AND WHAT IT IS NOT (explicit boundary):
  `occupancy_counts` produces the RAW INSTANTANEOUS COUNT per frame — how many
  ground contacts fall inside each polygon in frame t. No track_id, no time, no state.
  It is NOT the final measured occupancy: the dwell threshold and the comparison
  against the occupancy GT are a LATER LAYER that does require temporal identity. This
  function is the bottom brick, not the number that goes into the paper. See
  docs/modules/occupancy_zones.md.

Two-level SEMANTICS (honest coverage):
  - total (arm): inside the 'arm' polygon — reliable down to the back of the queue.
  - lanes[k]: inside the polygon of lane k — independent depth per lane, with coverage
    DECLARED in config (full|partial + note).
  - sin_carril: inside the arm but outside every lane (the non-separable background).
    A fallback to a neighbor lane is FORBIDDEN: it would contaminate that lane's metric.

CAUSALITY: it satisfies the causal contract by form (I1 pure lookup, I3 no complete
event — digital_shadow/causal/contract.py); verified in tests with `check_pure_lookup`.
Only information from the present frame.
"""
from __future__ import annotations

import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
import derive_track_events as dte  # production geometry/loaders (same precedent as the orchestrator)


# --------------------------------------------------------------------------- #
# Model
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class LaneZone:
    polygon: np.ndarray            # (N,2) image px, implicit closure
    entry_side: tuple              # indices of the vertices of the ENTRY side (exterior)
    coverage: str                  # 'full' | 'partial' (declared in config, not inferred)
    coverage_note: Optional[str] = None


@dataclass(frozen=True)
class ArmZone:
    polygon: np.ndarray
    entry_side: tuple
    lanes: dict = field(default_factory=dict)   # int -> LaneZone


@dataclass(frozen=True)
class ApproachZones:
    site_id: str
    frame_width: int
    frame_height: int
    arms: dict = field(default_factory=dict)    # str -> ArmZone


# --------------------------------------------------------------------------- #
# Geometry (wrappers over production)
# --------------------------------------------------------------------------- #
def _inside(pts: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Production points_in_polygon; silences the masked 0/0 division warning that the
    horizontal edges emit (the result is correct)."""
    with np.errstate(divide="ignore", invalid="ignore"):
        return dte.points_in_polygon(np.asarray(pts, float), np.asarray(poly, float))


def _sample_points(poly: np.ndarray) -> np.ndarray:
    """Vertices + edge midpoints + centroid (sampling check; the preview of the tracing
    tool is the second barrier)."""
    mids = (poly + np.roll(poly, -1, axis=0)) / 2.0
    return np.vstack([poly, mids, poly.mean(axis=0, keepdims=True)])


def _contains(inner: np.ndarray, outer: np.ndarray) -> bool:
    return bool(_inside(_sample_points(inner), outer).all())


def _disjoint(a: np.ndarray, b: np.ndarray) -> bool:
    return not (_inside(_sample_points(a), b).any() or _inside(_sample_points(b), a).any())


def _entry_side_ok(poly: np.ndarray, entry_side, stop_seg: np.ndarray,
                   center: np.ndarray) -> bool:
    """BLOCKING: the entry edge must be FARTHER from the stop line than the centroid
    (signed distance, + towards the center). Inverted, it would ruin the entry times of
    the occupancy GT."""
    p1, nh = dte._inward_normal(stop_seg, center)
    edge = poly[list(entry_side)]
    d_mid = float((edge.mean(axis=0) - p1) @ nh)
    d_cen = float((poly.mean(axis=0) - p1) @ nh)
    return d_mid < d_cen


# --------------------------------------------------------------------------- #
# Fail-fast loader
# --------------------------------------------------------------------------- #
def load_approach_zones(site_id: str, path: Optional[Path] = None) -> ApproachZones:
    """Load and VALIDATE approach_zones.yaml. Aborts (ValueError with all the failures,
    naming the offending polygon) if the config does not add up: a hand-mutated geometry
    does not enter silently. `path` only for tests (synthetic fixtures)."""
    from digital_shadow.config.site import load_site_config

    path = path or ROOT / "config" / "sites" / site_id / "approach_zones.yaml"
    with open(path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    site = load_site_config(site_id)
    stop_lines = dte.load_stop_lines(site_id)
    center = np.array([site.frame_width / 2.0, site.frame_height / 2.0])

    fails: list[str] = []
    if (int(data["frame_width"]), int(data["frame_height"])) != (site.frame_width,
                                                                 site.frame_height):
        fails.append(f"dims {data['frame_width']}x{data['frame_height']} != site.yaml "
                     f"{site.frame_width}x{site.frame_height}")

    def check_poly(name: str, node: dict) -> Optional[tuple]:
        poly = np.asarray(node["polygon_px"], float)
        es = tuple(int(i) for i in node.get("entry_side", [0, 1]))
        if len(poly) < 4:
            fails.append(f"{name}: {len(poly)} vertices (<4)")
            return None
        if len(es) != 2 or any(i < 0 or i >= len(poly) for i in es):
            fails.append(f"{name}: entry_side {es} fuera de rango")
            return None
        arm = name.split("_")[0]
        if arm not in stop_lines:
            fails.append(f"{name}: brazo '{arm}' sin linea de pare en zones.yaml")
            return None
        if not _entry_side_ok(poly, es, stop_lines[arm], center):
            fails.append(f"{name}: entry_side INVERTIDO (la arista de entrada no es el "
                         f"lado exterior respecto a la pare)")
            return None
        return poly, es

    arms: dict = {}
    for arm, node in (data.get("approach_zones") or {}).items():
        arm_ok = check_poly(f"{arm}_arm", node["arm"]) if "arm" in node else None
        if "arm" not in node:
            fails.append(f"{arm}: sin poligono de brazo ('arm')")
        lanes: dict = {}
        for k, ln in (node.get("lanes") or {}).items():
            got = check_poly(f"{arm}_lane{k}", ln)
            if got is None:
                continue
            lp, les = got
            if ln.get("coverage") not in ("full", "partial"):
                fails.append(f"{arm}_lane{k}: coverage '{ln.get('coverage')}' invalida "
                             f"(full|partial)")
            if arm_ok is not None and not _contains(lp, arm_ok[0]):
                fails.append(f"{arm}_lane{k}: carril NO contenido en el brazo")
            lanes[int(k)] = LaneZone(lp, les, str(ln.get("coverage")),
                                     ln.get("coverage_note"))
        ks = sorted(lanes)
        for i in range(len(ks)):
            for j in range(i + 1, len(ks)):
                if not _disjoint(lanes[ks[i]].polygon, lanes[ks[j]].polygon):
                    fails.append(f"{arm}_lane{ks[i]}: solapa con {arm}_lane{ks[j]}")
        if arm_ok is not None:
            arms[arm] = ArmZone(arm_ok[0], arm_ok[1], lanes)
    names = sorted(arms)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            if not _disjoint(arms[names[i]].polygon, arms[names[j]].polygon):
                fails.append(f"{names[i]}_arm: solapa con {names[j]}_arm")

    if fails:
        raise ValueError("approach_zones invalida (fail-fast, nada se carga):\n  - "
                         + "\n  - ".join(fails))
    return ApproachZones(str(data.get("site_id", site_id)),
                         int(data["frame_width"]), int(data["frame_height"]), arms)


# --------------------------------------------------------------------------- #
# Instantaneous counting (PURE per-frame function)
# --------------------------------------------------------------------------- #
def classify_points(points, zones: ApproachZones, exclusion_polygons=()):
    """THE single source of point→zone membership, shared by the counting, the
    WaitingBoxLayer classification, and the masks of the metric instrument: a single
    membership logic (verified equivalent, 0/106480 cells over the reference segment)
    keeps those three consumers from diverging.

    Returns (arm_col, lane_col) aligned with points:
      arm_col: str — arm, or "" (excluded point or outside every arm).
      lane_col: int — lane k, or -1 (= sin_carril inside the arm; also stays -1 when
                arm_col is "": filter by arm_col first).
    UNIQUE assignment per point — the loader guarantees disjoint arms and lanes ⊂ arm
    and disjoint (fail-fast), so the assignment is well defined; in the face of an
    overlap that slipped past the sampling validation, the last one in the YAML order
    wins instead of duplicating the count (deliberate hardening). No fallback to a
    neighbor lane."""
    pts = np.asarray(list(points) if not isinstance(points, np.ndarray) else points,
                     float).reshape(-1, 2)
    arm_col = np.full(len(pts), "", object)
    lane_col = np.full(len(pts), -1, int)
    if not len(pts):
        return arm_col, lane_col
    keep = np.ones(len(pts), bool)
    for poly in exclusion_polygons:
        keep &= ~_inside(pts, poly)
    for arm, az in zones.arms.items():
        m = keep & _inside(pts, az.polygon)
        arm_col[m] = arm
        for k, lz in az.lanes.items():
            lane_col[m & _inside(pts, lz.polygon)] = int(k)
    return arm_col, lane_col


def occupancy_counts(points, zones: ApproachZones, exclusion_polygons=()) -> dict:
    """RAW INSTANTANEOUS count of the present frame. It is NOT the final measured
    occupancy (see the module docstring): no track_id, no time, no state, no fallback.

    points: ground contacts (gx, gy) of frame t — iterable of pairs or (N,2) array.
    exclusion_polygons: polygons to discard BEFORE counting (e.g. parking lot, the same
      ones from zones.yaml that the pipeline applies).
    Returns {arm: {"total": int, "lanes": {k: int}, "sin_carril": int}}.
    Invariant: total == sum(lanes.values()) + sin_carril.

    It is the AGGREGATION of classify_points: a single membership logic for counting,
    waiting boxes, and metric."""
    arm_col, lane_col = classify_points(points, zones, exclusion_polygons)
    out: dict = {}
    for arm, az in zones.arms.items():
        in_arm = arm_col == arm
        lanes = {k: int((in_arm & (lane_col == k)).sum()) for k in az.lanes}
        out[arm] = {"total": int(in_arm.sum()), "lanes": lanes,
                    "sin_carril": int((in_arm & (lane_col == -1)).sum())}
    return out

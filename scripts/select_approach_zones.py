"""Interactive selector of APPROACH ZONES (occupancy) — approach + lanes, multi-slot.

MARKING INFRASTRUCTURE ONLY (like select_exclusion_zone.py, from which it copies the
tested view mechanics). It is not production and does NOT touch the causal pipeline: it writes
only captures in verify_out/zones/rusia/approach/. PROMOTING the draft to
config/sites/rusia/approach_zones.yaml is a separate step, after reviewing the preview.

ZONE STRUCTURE (two levels per approach): an APPROACH polygon (covers the queue to the
back → total count) and up to 3 LANE polygons with INDEPENDENT depth
(as far as that lane separates visually → per-lane count, declared coverage).

ORIENTATION CONVENTION (blocking): the FIRST TWO CLICKS of each polygon are
the ENTRY side (exterior, back of the queue). It is validated against the approach's stop line:
the v0→v1 edge must be FARTHER from the stop line than the polygon's
centroid. An inverted entry side would ruin the occupancy GT times →
if it fails, NO draft is emitted.

TRACING BACKGROUND (the separability guide, not decoration):
RECONNECTED trajectories matched with the GT, trimmed to their APPROACH segment (up to the
stop-line-1 crossing) and colored by the GT entry lane. Where the colors of
neighboring lanes mix, that lane stops being separable: that is where the
depth of its polygon ends. Also: statics in red (stopped queue), stop
lines, parking lot (exclusion) and c1/c2/c3 labels over each stop line.

CONTROLS (focused window):
  n / e / s / o  : choose active APPROACH (norte/este/sur/oeste)
  a / 1 / 2 / 3  : choose POLYGON of the active approach (a=approach, 1-3=lane)
  LEFT click     : add vertex to the active slot (the first 2 = ENTRY side)
  RIGHT click    : center the view (pan)          +/- : zoom   wheel: zoom
  u              : undo last vertex (active slot)
  c              : clear the active slot (the others are untouched)
  r              : reset view    w : save capture    q : save and exit

OUTPUT (reproducible capture, NOT the config):
  verify_out/zones/rusia/approach/<approach>_{arm|laneK}_polygon.csv
  verify_out/zones/rusia/approach/approach_preview.png   (composite, conflicts in red)
  verify_out/zones/rusia/approach/approach_zones_draft.yaml
    — ONLY if ALL validations pass: lanes ⊂ approach, disjoint lanes,
      disjoint approaches, dims == site.yaml, valid and well-oriented entry_side.
On startup the previous captures of the directory are RE-LOADED (iterative tracing:
correcting one polygon does not force re-tracing the rest).

Usage:
  python scripts/select_approach_zones.py [--selftest | --build-bg-only]
"""
from __future__ import annotations

import csv
import os
import sys
from datetime import date
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
import derive_track_events as dte  # production loaders/geometry (zero duplication)
from digital_shadow.config import load_site_config
from digital_shadow.geometry import add_ground_contact

FRAME = ROOT / "verify_out/homography_improved/clean_frame_2541.png"  # same frame as zones.yaml
TRACKS_RECONNECTED = ROOT / "verify_out/events/rusia/reconnected/tracks_reconnected.csv"
MATCHES_RECONNECTED = ROOT / "verify_out/events/rusia/reconnected/matches.csv"
GT_CSV = ROOT / "data/ground_truth/rusia/ground_truth_rusia_v2_carril_processed.csv"
OUT_DIR = ROOT / "verify_out/zones/rusia/approach"
OUT_BG = OUT_DIR / "bg_approach.png"
OUT_PREVIEW = OUT_DIR / "approach_preview.png"
OUT_DRAFT = OUT_DIR / "approach_zones_draft.yaml"
WIN = "Zonas de APPROACH (ocupacion) - brazo n/e/s/o, poligono a/1/2/3"

ARMS = {"n": "norte", "e": "este", "s": "sur", "o": "oeste"}
KINDS = ("a", "1", "2", "3")  # a = approach polygon; 1-3 = lane
MIN_OBS = 25            # same filters as select_exclusion_zone.py
STATIC_MIN_OBS = 200
STATIC_MAX_SPAN = 40

# BGR. Base color per approach (active slot saturated; finished fades toward gray).
ARM_COLOR = {"norte": (255, 150, 40), "este": (60, 200, 60),
             "sur": (0, 215, 255), "oeste": (255, 0, 150)}
LANE_TRAJ = {1: (70, 200, 70), 2: (0, 150, 255), 3: (255, 120, 0)}  # GT lane 1/2/3
COL_ENTRY = (255, 0, 255)      # ENTRY edge (v0→v1)
COL_STATIC = (0, 0, 255)       # stopped queue
COL_UNMATCHED = (150, 150, 150)
COL_STOPLINE = (255, 255, 255)
COL_EXCLUSION = (0, 165, 255)
COL_FAIL = (0, 0, 255)


def slot_name(arm: str, kind: str) -> str:
    return f"{arm}_{'arm' if kind == 'a' else 'lane' + kind}"


# --------------------------------------------------------------------------- #
# Fail-fast validations (pure functions; reused by the selftest)
# --------------------------------------------------------------------------- #
def _sample_points(poly: np.ndarray) -> np.ndarray:
    """Vertices + edge midpoints + centroid (sampling for the geometric check;
    the visual preview is the second barrier for extreme non-convex cases)."""
    mids = (poly + np.roll(poly, -1, axis=0)) / 2.0
    return np.vstack([poly, mids, poly.mean(axis=0, keepdims=True)])


def _inside(pts: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """production points_in_polygon; silences the horizontal-edge warning
    (0/0 division masked by the condition — the result is correct)."""
    with np.errstate(divide="ignore", invalid="ignore"):
        return dte.points_in_polygon(pts, poly)


def poly_contains_poly(inner: np.ndarray, outer: np.ndarray) -> bool:
    return bool(_inside(_sample_points(inner), outer).all())


def polys_disjoint(a: np.ndarray, b: np.ndarray) -> bool:
    return not (_inside(_sample_points(a), b).any()
                or _inside(_sample_points(b), a).any())


def entry_side_ok(poly: np.ndarray, stop_seg: np.ndarray, center: np.ndarray) -> bool:
    """BLOCKING: the v0→v1 edge (entry side) must be FARTHER from the stop line
    than the centroid (signed distance, + toward the intersection center)."""
    p1, nh = dte._inward_normal(stop_seg, center)
    d_mid = float((poly[:2].mean(axis=0) - p1) @ nh)
    d_cen = float((poly.mean(axis=0) - p1) @ nh)
    return d_mid < d_cen


def validate_slots(slots: dict, stop_lines: dict, center: np.ndarray,
                   bg_shape: tuple, expected_dims: tuple) -> list[tuple[str, str]]:
    """Runs ALL validations over what was captured. Returns [(slot, reason), ...];
    empty list = emittable draft. It only validates what was traced (partial drafts OK)."""
    fails: list[tuple[str, str]] = []
    if (bg_shape[1], bg_shape[0]) != expected_dims:
        fails.append(("_dims", f"fondo {bg_shape[1]}x{bg_shape[0]} != site.yaml "
                               f"{expected_dims[0]}x{expected_dims[1]}"))
    polys = {k: np.asarray(v, float) for k, v in slots.items() if len(v) > 0}
    for (arm, kind), poly in polys.items():
        name = slot_name(arm, kind)
        if len(poly) < 4:
            fails.append((name, f"{len(poly)} vertices (<4: hace falta arista de "
                                f"entrada + lado opuesto)"))
            continue
        if arm not in stop_lines:
            fails.append((name, f"brazo '{arm}' sin linea de pare en zones.yaml"))
            continue
        if not entry_side_ok(poly, stop_lines[arm], center):
            fails.append((name, "entry_side INVERTIDO: la arista v0-v1 no es el lado "
                                "exterior (mas lejos de la pare que el centroide)"))
    # lanes ⊂ approach and disjoint lanes (within each approach)
    for arm in {a for (a, _k) in polys}:
        lanes = {k: polys[(arm, k)] for k in ("1", "2", "3")
                 if (arm, k) in polys and len(polys[(arm, k)]) >= 4}
        arm_poly = polys.get((arm, "a"))
        for k, lp in lanes.items():
            if arm_poly is None or len(arm_poly) < 4:
                fails.append((slot_name(arm, k), "carril sin poligono de BRAZO trazado"))
            elif not poly_contains_poly(lp, arm_poly):
                fails.append((slot_name(arm, k), "carril NO contenido en el brazo"))
        ks = sorted(lanes)
        for i in range(len(ks)):
            for j in range(i + 1, len(ks)):
                if not polys_disjoint(lanes[ks[i]], lanes[ks[j]]):
                    fails.append((slot_name(arm, ks[i]),
                                  f"solapa con {slot_name(arm, ks[j])}"))
    # approaches disjoint from each other
    arm_polys = {a: polys[(a, "a")] for (a, k) in polys if k == "a"
                 and len(polys[(a, "a")]) >= 4}
    names = sorted(arm_polys)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            if not polys_disjoint(arm_polys[names[i]], arm_polys[names[j]]):
                fails.append((slot_name(names[i], "a"),
                              f"solapa con {slot_name(names[j], 'a')}"))
    return fails


# --------------------------------------------------------------------------- #
# Captures: CSV per polygon + preview + draft YAML
# --------------------------------------------------------------------------- #
def save_captures(slots: dict, bg, stop_lines: dict, center: np.ndarray,
                  expected_dims: tuple, out_dir: Path) -> list[tuple[str, str]]:
    """Writes CSVs + preview ALWAYS (evidence); the draft YAML ONLY if all
    validations pass. Returns the list of failures (empty = draft emitted)."""
    out_dir.mkdir(parents=True, exist_ok=True)
    captured = {k: v for k, v in slots.items() if len(v) >= 3}
    for (arm, kind), verts in captured.items():
        with open(out_dir / f"{slot_name(arm, kind)}_polygon.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["idx", "pixel_x", "pixel_y"])
            for i, (px, py) in enumerate(verts):
                w.writerow([i, f"{px:.2f}", f"{py:.2f}"])
    fails = validate_slots(slots, stop_lines, center, bg.shape, expected_dims)
    failed_slots = {s for s, _ in fails}

    # composite preview (conflicts in red, with the reason)
    prev = bg.copy()
    shade = bg.copy()
    for (arm, kind), verts in captured.items():
        poly = np.asarray(verts, np.int32)
        bad = slot_name(arm, kind) in failed_slots
        col = COL_FAIL if bad else ARM_COLOR[arm]
        cv2.fillPoly(shade, [poly], col)
        cv2.polylines(prev, [poly], True, col, 2, cv2.LINE_AA)
        if len(poly) >= 2:
            cv2.line(prev, tuple(poly[0]), tuple(poly[1]), COL_ENTRY, 3, cv2.LINE_AA)
        cx, cy = poly.mean(axis=0).astype(int)
        cv2.putText(prev, slot_name(arm, kind), (cx - 30, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2, cv2.LINE_AA)
    cv2.addWeighted(shade, 0.20, prev, 0.80, 0, prev)
    for i, (s, why) in enumerate(fails):
        cv2.putText(prev, f"FALLA {s}: {why}", (10, 40 + 20 * i),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, COL_FAIL, 2, cv2.LINE_AA)
    cv2.imwrite(str(out_dir / OUT_PREVIEW.name), prev)

    draft_path = out_dir / OUT_DRAFT.name
    if fails:
        if draft_path.exists():
            draft_path.unlink()   # an old draft must not survive an invalid capture
        print(f"  [!] {len(fails)} validacion(es) FALLARON -> SIN borrador YAML:")
        for s, why in fails:
            print(f"      - {s}: {why}")
        return fails

    zones: dict = {}
    for arm in sorted({a for (a, _k) in captured}):
        entry = {"marked_by": "israel", "marked_date": date.today().isoformat(),
                 "source": "scripts/select_approach_zones.py"}
        if (arm, "a") in captured:
            entry["arm"] = {
                "polygon_px": [[float(round(x, 2)), float(round(y, 2))]
                               for x, y in captured[(arm, "a")]],
                "entry_side": [0, 1]}
        lanes = {}
        for k in ("1", "2", "3"):
            if (arm, k) in captured:
                lanes[int(k)] = {
                    "polygon_px": [[float(round(x, 2)), float(round(y, 2))]
                                   for x, y in captured[(arm, k)]],
                    "entry_side": [0, 1],
                    "coverage": "partial",
                    "coverage_note": "REVISAR: declarar cobertura (full|partial) en la promocion"}
        if lanes:
            entry["lanes"] = lanes
        zones[arm] = entry
    draft = {"site_id": "rusia",
             "reference_frame": "verify_out/homography_improved/clean_frame_2541.png",
             "frame_width": expected_dims[0], "frame_height": expected_dims[1],
             "approach_zones": zones}
    with open(draft_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(draft, f, sort_keys=False, allow_unicode=True,
                       default_flow_style=None)
    shown = draft_path.relative_to(ROOT) if draft_path.is_relative_to(ROOT) else draft_path
    print(f"  [ok] validaciones OK -> borrador {shown}")
    return []


def load_captures(out_dir: Path) -> dict:
    """Re-loads the previous captures (iterative tracing between runs)."""
    slots = {(a, k): [] for a in ARMS.values() for k in KINDS}
    if not out_dir.is_dir():
        return slots
    for arm in ARMS.values():
        for kind in KINDS:
            p = out_dir / f"{slot_name(arm, kind)}_polygon.csv"
            if p.is_file():
                with open(p) as f:
                    rows = list(csv.DictReader(f))
                slots[(arm, kind)] = [(float(r["pixel_x"]), float(r["pixel_y"]))
                                      for r in rows]
    n = sum(1 for v in slots.values() if v)
    if n:
        shown = out_dir.relative_to(ROOT) if out_dir.is_relative_to(ROOT) else out_dir
        print(f"[recarga] {n} poligono(s) previos cargados de {shown}")
    return slots


# --------------------------------------------------------------------------- #
# Background: clean frame + per-lane GT separability guide
# --------------------------------------------------------------------------- #
def build_background():
    """RECONNECTED trajectories matched with GT, trimmed to the APPROACH segment
    (up to the stop-line-1 crossing) and colored by GT lane — the cut criterion of the
    tracing. Statics in red; unmatched in gray; stop line/exclusion/c1-c3 labels."""
    img = cv2.imread(str(FRAME))
    if img is None:
        raise FileNotFoundError(FRAME)
    H, W = img.shape[:2]
    site = load_site_config("rusia")
    center = np.array([site.frame_width / 2.0, site.frame_height / 2.0])
    stop_lines = dte.load_stop_lines("rusia")
    excl, _, _ = dte.load_exclusion_polygons("rusia")
    bands = dte.load_lane_bands("rusia").get("entry", {})

    df = pd.read_csv(TRACKS_RECONNECTED)
    add_ground_contact(df)
    m = pd.read_csv(MATCHES_RECONNECTED)
    gt = pd.read_csv(GT_CSV).set_index("event_id")
    lane_of = {int(r.track_id): (gt.loc[int(r.event_id), "entry_arm"],
                                 int(gt.loc[int(r.event_id), "entry_lane"]))
               for r in m.itertuples()}

    base = img.copy()
    overlay = img.copy()
    statics = []
    n_pre = 0
    for tid, g in df.groupby("track_id"):
        if len(g) < MIN_OBS:
            continue
        g = g.sort_values("frame_index")
        span = max(g.gx.max() - g.gx.min(), g.gy.max() - g.gy.min())
        if len(g) >= STATIC_MIN_OBS and span < STATIC_MAX_SPAN:
            pts = g[["gx", "gy"]].to_numpy(np.int32).reshape(-1, 1, 2)
            cv2.polylines(overlay, [pts], False, COL_STATIC, 2, cv2.LINE_AA)
            statics.append((int(tid), float(g.gx.mean()), float(g.gy.mean())))
            continue
        arm_lane = lane_of.get(int(tid))
        cross_t = None
        if arm_lane is not None and arm_lane[0] in stop_lines:
            cross_t = dte.stopline_crossing_time(
                g[["timestamp_s", "gx", "gy"]], stop_lines[arm_lane[0]], center, "entry")
        if arm_lane is not None and cross_t is not None:
            # ONLY the approach segment, colored by GT lane (cut guide)
            pre = g[g.timestamp_s <= cross_t]
            if len(pre) >= 2:
                pts = pre[["gx", "gy"]].to_numpy(np.int32).reshape(-1, 1, 2)
                cv2.polylines(overlay, [pts], False, LANE_TRAJ[arm_lane[1]], 2, cv2.LINE_AA)
                n_pre += 1
        else:
            pts = g[["gx", "gy"]].to_numpy(np.int32).reshape(-1, 1, 2)
            cv2.polylines(overlay, [pts], False, COL_UNMATCHED, 1, cv2.LINE_AA)
    cv2.addWeighted(overlay, 0.80, base, 0.20, 0, base)

    for tid, mx, my in statics:
        cv2.putText(base, f"tid{tid}", (int(mx) + 4, int(my) - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA)
    for poly in excl.values():
        cv2.polylines(base, [poly.astype(np.int32)], True, COL_EXCLUSION, 2, cv2.LINE_AA)
    for arm, seg in stop_lines.items():
        p1, p2 = seg.astype(int)
        cv2.line(base, tuple(p1), tuple(p2), COL_STOPLINE, 2, cv2.LINE_AA)
        mid = seg.mean(axis=0).astype(int)
        cv2.putText(base, f"pare {arm}", (mid[0] + 6, mid[1] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, COL_STOPLINE, 1, cv2.LINE_AA)
        band = bands.get(arm) or {}
        u = seg[1] - seg[0]
        u = u / np.linalg.norm(u)
        for k, s_val in (band.get("centroids_s") or {}).items():
            pt = (seg[0] + float(s_val) * u).astype(int)
            cv2.circle(base, tuple(pt), 5, LANE_TRAJ[int(k)], -1, cv2.LINE_AA)
            cv2.putText(base, f"c{k}", (pt[0] + 6, pt[1] + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, LANE_TRAJ[int(k)], 2, cv2.LINE_AA)

    legend = [("aprox. GT c1", LANE_TRAJ[1]), ("c2", LANE_TRAJ[2]), ("c3", LANE_TRAJ[3]),
              ("estatico (cola)", COL_STATIC), ("no casado", COL_UNMATCHED),
              ("pare", COL_STOPLINE), ("exclusion", COL_EXCLUSION)]
    x = 10
    for txt, col in legend:
        cv2.putText(base, txt, (x, H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.48, col, 2, cv2.LINE_AA)
        x += 12 * len(txt) + 18
    corners = [("SUR (sup-izq)", (10, 22)), ("OESTE (sup-der)", (W - 190, 22)),
               ("ESTE (inf-izq)", (10, H - 34)), ("NORTE (inf-der)", (W - 190, H - 34))]
    for txt, (cx, cy) in corners:
        cv2.putText(base, txt, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (200, 200, 255), 1, cv2.LINE_AA)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(OUT_BG), base)
    print(f"[bg] {n_pre} aproximaciones GT coloreadas, {len(statics)} estaticos -> "
          f"{OUT_BG.relative_to(ROOT)}")
    return base, stop_lines, center, (site.frame_width, site.frame_height)


# --------------------------------------------------------------------------- #
# Multi-slot selector (view/zoom/pan copied from select_exclusion_zone.py, tested)
# --------------------------------------------------------------------------- #
class ApproachZoneSelector:
    def __init__(self, bg, stop_lines, center, dims, out_dir=OUT_DIR,
                 win_w=1280, win_h=760):
        self.img = bg
        self.H, self.W = bg.shape[:2]
        self.stop_lines, self.center, self.dims = stop_lines, center, dims
        self.out_dir = out_dir
        self.win_w, self.win_h = win_w, win_h
        self.slots = load_captures(out_dir)
        self.active = ("sur", "a")
        self.mouse = (0, 0)
        self.reset_view()

    # --- view (identical to select_exclusion_zone.py) ---
    def reset_view(self):
        self.z = min(self.win_w / self.W, self.win_h / self.H)
        self.cx, self.cy = self.W / 2.0, self.H / 2.0

    def _affine(self):
        tx = self.win_w / 2.0 - self.z * self.cx
        ty = self.win_h / 2.0 - self.z * self.cy
        return self.z, tx, ty

    def win_to_src(self, wx, wy):
        z, tx, ty = self._affine()
        return (wx - tx) / z, (wy - ty) / z

    def src_to_win(self, fx, fy):
        z, tx, ty = self._affine()
        return z * fx + tx, z * fy + ty

    def zoom(self, factor, about=None):
        if about is None:
            self.z *= factor
            return
        ax, ay = about
        sx, sy = self.win_to_src(ax, ay)
        self.z = max(0.1, min(40.0, self.z * factor))
        self.cx = sx - (ax - self.win_w / 2.0) / self.z
        self.cy = sy - (ay - self.win_h / 2.0) / self.z

    # --- drawing ---
    def _draw_slot(self, view, key, verts):
        arm, _kind = key
        is_active = key == self.active
        col = ARM_COLOR[arm]
        if not is_active:  # finished: faded toward gray, thin
            col = tuple(int(0.45 * c + 0.55 * 128) for c in col)
        wpts = [self.src_to_win(x, y) for (x, y) in verts]
        th = 2 if is_active else 1
        for i in range(1, len(wpts)):
            edge_col = COL_ENTRY if i == 1 else col      # v0→v1 edge = ENTRY
            edge_th = 3 if i == 1 else th
            cv2.line(view, (int(wpts[i - 1][0]), int(wpts[i - 1][1])),
                     (int(wpts[i][0]), int(wpts[i][1])), edge_col, edge_th, cv2.LINE_AA)
        if len(wpts) >= 2:
            mx = int((wpts[0][0] + wpts[1][0]) / 2)
            my = int((wpts[0][1] + wpts[1][1]) / 2)
            cv2.putText(view, "ENTRADA", (mx + 4, my - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, COL_ENTRY, 1, cv2.LINE_AA)
        if len(wpts) >= 3:
            cv2.line(view, (int(wpts[-1][0]), int(wpts[-1][1])),
                     (int(wpts[0][0]), int(wpts[0][1])), col, 1, cv2.LINE_AA)
        if is_active and wpts:
            cv2.line(view, (int(wpts[-1][0]), int(wpts[-1][1])),
                     (int(self.mouse[0]), int(self.mouse[1])), (0, 200, 200), 1, cv2.LINE_AA)
        for i, (wx, wy) in enumerate(wpts):
            cv2.drawMarker(view, (int(wx), int(wy)), col, cv2.MARKER_CROSS,
                           14 if is_active else 8, 2 if is_active else 1)
            if is_active:
                cv2.putText(view, str(i), (int(wx) + 6, int(wy) - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
        if len(wpts) >= 3:
            cx, cy = np.mean([[p[0], p[1]] for p in wpts], axis=0).astype(int)
            cv2.putText(view, slot_name(*key), (cx - 25, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1 + is_active, cv2.LINE_AA)

    def render(self):
        z, tx, ty = self._affine()
        M = np.array([[z, 0, tx], [0, z, ty]], dtype=np.float32)
        view = cv2.warpAffine(self.img, M, (self.win_w, self.win_h),
                              flags=cv2.INTER_NEAREST, borderValue=(30, 30, 30))
        for key, verts in self.slots.items():
            if verts and key != self.active:
                self._draw_slot(view, key, verts)
        if self.slots[self.active]:
            self._draw_slot(view, self.active, self.slots[self.active])

        arm, kind = self.active
        counts = " ".join(
            f"{a[0]}:{''.join(k if self.slots[(a, k)] else '.' for k in KINDS)}"
            for a in ARMS.values())
        lines = [
            f"SLOT ACTIVO: {slot_name(arm, kind)}  ({len(self.slots[self.active])} vertices)"
            f"   |   trazados {counts}",
            "2 PRIMEROS CLICS = lado ENTRADA (fondo de cola, magenta).  brazo: n/e/s/o  "
            "poligono: a/1/2/3",
            "clicIZQ=vertice  u=deshacer  c=limpiar-slot  clicDER=centrar  +/-=zoom  "
            "r=reset  w=guardar  q=guardar y salir",
        ]
        bh = 14 + 20 * len(lines)
        ov = view.copy()
        cv2.rectangle(ov, (0, 0), (self.win_w, bh), (0, 0, 0), -1)
        cv2.addWeighted(ov, 0.55, view, 0.45, 0, view)
        y = 20
        for i, ln in enumerate(lines):
            col = (0, 255, 255) if i == 0 else (180, 180, 180)
            cv2.putText(view, ln, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, col, 1, cv2.LINE_AA)
            y += 20
        cv2.putText(view, f"zoom x{self.z:.1f}", (self.win_w - 120, self.win_h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.imshow(WIN, view)

    # --- mouse / keyboard ---
    def on_mouse(self, event, x, y, flags, param):
        self.mouse = (x, y)
        if event == cv2.EVENT_LBUTTONDOWN:
            fx, fy = self.win_to_src(x, y)
            fx = float(np.clip(fx, 0, self.W - 1))
            fy = float(np.clip(fy, 0, self.H - 1))
            self.slots[self.active].append((fx, fy))
            print(f"  {slot_name(*self.active)} +v{len(self.slots[self.active]) - 1}: "
                  f"({fx:.1f},{fy:.1f})")
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.cx, self.cy = self.win_to_src(x, y)
        elif event == cv2.EVENT_MOUSEWHEEL:
            self.zoom(1.2 if flags > 0 else 1 / 1.2, about=(x, y))

    def handle_key(self, ch: str) -> bool:
        """Returns False when it should exit. Separated from the loop for the selftest."""
        arm, kind = self.active
        if ch in ARMS:
            self.active = (ARMS[ch], kind)
        elif ch in KINDS:
            self.active = (arm, ch)
        elif ch == "u" and self.slots[self.active]:
            v = self.slots[self.active].pop()
            print(f"  {slot_name(*self.active)} -deshacer ({v[0]:.1f},{v[1]:.1f})")
        elif ch == "c":
            self.slots[self.active] = []
            print(f"  {slot_name(*self.active)} limpiado")
        elif ch in "+=":
            self.zoom(1.25, about=self.mouse)
        elif ch in "-_":
            self.zoom(1 / 1.25, about=self.mouse)
        elif ch == "r":
            self.reset_view()
        elif ch == "w":
            save_captures(self.slots, self.img, self.stop_lines, self.center,
                          self.dims, self.out_dir)
        elif ch == "q":
            save_captures(self.slots, self.img, self.stop_lines, self.center,
                          self.dims, self.out_dir)
            return False
        return True

    def run(self):
        cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WIN, self.on_mouse)
        while True:
            self.render()
            k = cv2.waitKey(20) & 0xFF
            if k == 255:
                continue
            if not self.handle_key(chr(k)):
                break
        cv2.destroyAllWindows()


# --------------------------------------------------------------------------- #
# Selftest without a window
# --------------------------------------------------------------------------- #
def selftest():
    import tempfile
    out = Path(tempfile.mkdtemp(prefix="approach_selftest_"))
    bg = np.zeros((646, 1200, 3), np.uint8)
    center = np.array([600.0, 323.0])
    # synthetic horizontal stop line at y=100; the center is below → approach = y<100
    stop = {"sur": np.array([[300.0, 100.0], [500.0, 100.0]])}
    dims = (1200, 646)

    # 1) win<->src mapping + zoom-about (inherited mechanics)
    s = ApproachZoneSelector(bg, stop, center, dims, out_dir=out)
    s.z, s.cx, s.cy = 3.7, 400.0, 220.0
    for fx, fy in [(0, 0), (600, 300), (1199, 645), (137.5, 88.4)]:
        wx, wy = s.src_to_win(fx, fy)
        bx, by = s.win_to_src(wx, wy)
        assert abs(bx - fx) < 1e-3 and abs(by - fy) < 1e-3
    s.reset_view()
    sx0, sy0 = s.win_to_src(300, 400)
    s.zoom(1.5, about=(300, 400))
    sx1, sy1 = s.win_to_src(300, 400)
    assert abs(sx0 - sx1) < 1e-3 and abs(sy0 - sy1) < 1e-3

    # 2) slot bookkeeping: independence between slots and re-entry
    s.active = ("sur", "a")
    s.slots[("sur", "a")] = [(300.0, 20.0), (500.0, 20.0), (500.0, 95.0), (300.0, 95.0)]
    s.handle_key("1")
    assert s.active == ("sur", "1")
    s.slots[("sur", "1")] = [(310.0, 30.0), (380.0, 30.0), (380.0, 90.0), (310.0, 90.0)]
    s.handle_key("u")
    assert len(s.slots[("sur", "1")]) == 3 and len(s.slots[("sur", "a")]) == 4
    s.slots[("sur", "1")].append((310.0, 90.0))
    s.handle_key("n")
    assert s.active == ("norte", "1")
    s.handle_key("c")            # clearing norte_lane1 (empty) does not touch sur
    assert len(s.slots[("sur", "1")]) == 4
    s.handle_key("s")
    s.handle_key("a")
    assert s.active == ("sur", "a")

    # 3) geometric validations
    arm_ok = np.array([[300.0, 20.0], [500.0, 20.0], [500.0, 95.0], [300.0, 95.0]])
    lane_in = np.array([[310.0, 30.0], [380.0, 30.0], [380.0, 90.0], [310.0, 90.0]])
    lane_out = np.array([[450.0, 30.0], [560.0, 30.0], [560.0, 90.0], [450.0, 90.0]])
    lane_overlap = np.array([[350.0, 30.0], [420.0, 30.0], [420.0, 90.0], [350.0, 90.0]])
    inverted = arm_ok[[2, 3, 0, 1]]   # v0-v1 ends up on the stop-line side → inverted entry
    assert poly_contains_poly(lane_in, arm_ok)
    assert not poly_contains_poly(lane_out, arm_ok)
    assert not polys_disjoint(lane_in, lane_overlap)
    assert entry_side_ok(arm_ok, stop["sur"], center)
    assert not entry_side_ok(inverted, stop["sur"], center)

    base = {(a, k): [] for a in ARMS.values() for k in KINDS}
    good = dict(base)
    good[("sur", "a")] = [tuple(p) for p in arm_ok]
    good[("sur", "1")] = [tuple(p) for p in lane_in]
    assert validate_slots(good, stop, center, bg.shape, dims) == []
    bad = dict(base)
    bad[("sur", "a")] = [tuple(p) for p in inverted]        # inverted entry -> BLOCKS
    bad[("sur", "1")] = [tuple(p) for p in lane_out]        # lane outside the approach
    bad[("sur", "2")] = [tuple(p) for p in lane_in]
    bad[("sur", "3")] = [tuple(p) for p in lane_overlap]    # overlaps with lane2
    fails = validate_slots(bad, stop, center, bg.shape, dims)
    reasons = " | ".join(w for _s, w in fails)
    assert "INVERTIDO" in reasons and "NO contenido" in reasons and "solapa" in reasons
    assert validate_slots(good, stop, center, bg.shape, (999, 999))  # wrong dims -> fails

    # 4) capture: draft ONLY if it validates; CSV roundtrip
    assert save_captures(good, bg, stop, center, dims, out) == []
    draft = yaml.safe_load((out / OUT_DRAFT.name).read_text())
    z = draft["approach_zones"]["sur"]
    assert z["arm"]["entry_side"] == [0, 1] and len(z["arm"]["polygon_px"]) == 4
    assert z["lanes"][1]["coverage"] == "partial"
    reloaded = load_captures(out)
    assert reloaded[("sur", "a")] == good[("sur", "a")]
    assert save_captures(bad, bg, stop, center, dims, out) != []
    assert not (out / OUT_DRAFT.name).exists()               # the old draft does NOT survive
    assert (out / "approach_preview.png").is_file()
    print("selftest OK: vista exacta, slots independientes y re-entrables, validaciones "
          "(contencion/disyuncion/entry_side BLOQUEANTE/dims), borrador solo-si-valida, "
          "roundtrip CSV.")


def main():
    if "--selftest" in sys.argv:
        selftest()
        return
    bg, stop_lines, center, dims = build_background()
    if "--build-bg-only" in sys.argv:
        print("Fondo generado. Revisa el PNG antes de marcar.")
        return
    print(__doc__)
    print("\nAbriendo ventana OpenCV... (necesita display)\n")
    ApproachZoneSelector(bg, stop_lines, center, dims).run()


if __name__ == "__main__":
    main()

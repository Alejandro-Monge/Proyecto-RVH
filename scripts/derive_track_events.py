"""EVENT-PER-TRACK layer (sub-step A of the shadow↔GT matching).

Converts each track from the tracking CSV into an EVENT with entry and exit approach and time,
classifying the approach COARSELY by the IMAGE CORNER where the track appears
(entry) and disappears (exit). It does NOT match against the GT (that is sub-step B): it only produces
the unit the matching will operate on, and marks it as complete or partial.

INPUT:
  - verify_out/tracks_rusia.csv : tracking output (track_id, frame_index, timestamp_s,
    x1,y1,x2,y2, cx,cy, class_id, confidence). ~96.5k observations, 194 tracks.
  - config/sites/<id>/site.yaml : frame_width/height (image center) and arm_set.
  - config/sites/<id>/zones.yaml : EXCLUSION polygon(s) (west parking lot) and per-approach
    STOP-LINE SEGMENTS (stop_lines), for the crossing instant.
  - verify_out/homography_gcp/control_points.csv : ONLY for the cross-check
    (per-approach calibrated anchor). It is not the source of the production classifier.

OUTPUT:
  - verify_out/events/<id>/track_events.csv : one event per NON-excluded track. Time
    columns: first_seen_s/last_seen_s (track APPEARANCE) and entry_time_s/exit_time_s (CROSSING of
    the stop line = physical instant the GT measures), with *_fallback flags.

REFERENCE POINT: the GROUND CONTACT (gx, gy) = ((x1+x2)/2, y2), via
  digital_shadow/geometry/ground_contact.py. NOT the centroid (height parallax).

INSTANTS (key to the matching): the APPROACH is classified by the APPEARANCE corner
  (validated), but the entry/exit TIME is NOT the appearance — it is the CROSSING OF THE approach's
  STOP LINE, the same physical event the human GT marks. The appearance happens 20-40 s
  earlier on far approaches (the vehicle is seen on the horizon long before crossing, and may
  wait at red), so using it as the entry time made shadow and GT incomparable. The
  appearance is kept as first_seen_s/last_seen_s (real track fact, useful for delay/
  fragmentation). Crossing: signed distance from the ground contact to the stop_lines segment,
  oriented toward the center; entry = first crossing −→+, exit = last +→−; if it does not cross, it falls
  back to the appearance (*_fallback flag).

CORNER CRITERION (coarse, justified — see docs/modules/track_events.md):
  - Entry point = median of the ground contact over the first WINDOW_FRAMES frames of the
    track; exit point = median over the last WINDOW_FRAMES. The median absorbs the
    detection jitter without averaging over real motion (at ~29.6 fps, 5 frames ≈ 0.17 s,
    in which a vehicle in the periphery moves <30 px, well below the
    ~300 px scale that separates one corner from another → the classification is insensitive to WINDOW).
  - Approach = QUADRANT of the point relative to the IMAGE CENTER (W/2, H/2), per the site's
    corner mapping (HARD-CODED, docs/site_rusia_geometry.md Section 2):
        top-left = SOUTH · top-right = WEST · bottom-left = EAST · bottom-right = NORTH
    The image center was chosen (and not the calibrated J5 core) because for the periphery
    points —those of clean events— both splits give the SAME result (verified: 79
    identical cross-arm events; the only tracks that differ are stuck to the center and are
    partial anyway), and the image center introduces no calibration constants into
    the logic (W,H already live in site.yaml).
  - PARTIAL if the entry or exit point falls within CORE_RADIUS_PX of the center: the track
    is born or dies INSIDE the intersection (occlusion fragment or ghost), with no clear approach
    mouth. It is not "fixed" here (reconnection is a separate front); it is MARKED and counted.

EXCLUSION: before deriving, tracks whose ground contact falls mostly
  (> EXCLUSION_FRACTION) inside an exclusion polygon (west parking lot in zones.yaml) are discarded,
  so as not to generate events for parked vehicles.

SCOPE NOTE (config): the corner→cardinal mapping and the WINDOW/CORE_RADIUS/
  EXCLUSION_FRACTION parameters are here as documented constants (with their provenance), NOT
  eyeballed. If this layer is promoted to a module, the corner→cardinal mapping (site datum) and
  the thresholds should move to per-site config.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from digital_shadow.config.site import load_site_config
from digital_shadow.geometry.ground_contact import GROUND_X, GROUND_Y, add_ground_contact

# --- Algorithm parameters (documented; see docstring). Not site geometry. ---
WINDOW_FRAMES = 5        # window for the entry/exit median (~0.17 s at 29.6 fps)
CORE_RADIUS_PX = 120     # core radius: < every stop line (~249-442 px from the center) and
                         # inside the J5 box (CB vertices ~115-258 px) → isolates the center
EXCLUSION_FRACTION = 0.5  # fraction of ground contact inside exclusion to discard the track

# Cache de líneas de pare activas para llamadas sin argumento explícito `stop_lines`.
_ACTIVE_STOP_LINES: dict[str, np.ndarray] = {}
CORE = "core"  # label when the point falls in the core (no clear approach mouth)

# COCO ids of the site's class_set → name, VERIFIED against model.names (yolov8x):
# 2=car, 5=bus, 7=truck (matches site.yaml class_set; the detector filters inference to
# these ids). The per-track class vote is done over these raw detector classes (the
# tracker is class-agnostic: it passes the detection's class per frame, does not modify it).
CLASS_BY_COCO = {2: "car", 5: "bus", 7: "truck"}


def points_in_polygon(pts: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Vectorized ray-casting (pure numpy): is each point of `pts` (N,2) in `poly` (M,2)?"""
    x, y = pts[:, 0], pts[:, 1]
    inside = np.zeros(len(pts), dtype=bool)
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        cond = ((yi > y) != (yj > y)) & (x < (xj - xi) * (y - yi) / (yj - yi) + xi)
        inside ^= cond
        j = i
    return inside


def load_exclusion_polygons(site_id: str):
    """Loads zones.yaml: returns (exclusion polygons in px, frame_width, frame_height)."""
    zones_yaml = ROOT / "config" / "sites" / site_id / "zones.yaml"
    with open(zones_yaml, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    out = {}
    for name, z in (data.get("exclusion_zones") or {}).items():
        out[name] = np.asarray(z["polygon_px"], dtype=float)
    # Sanity: the resolution the zones were marked on must match the site.
    return out, int(data["frame_width"]), int(data["frame_height"])


def arm_of(pt: np.ndarray, center: np.ndarray, core_radius: float,
           stop_lines: dict[str, np.ndarray] | None = None) -> str:
    """Clasifica el brazo por distancia euclidiana al punto medio de cada stop_line en zones.yaml;
    retorna CORE si está dentro de `core_radius` del centro geométrico de la intersección."""
    sl = stop_lines if stop_lines is not None else _ACTIVE_STOP_LINES
    if not sl:
        sl = load_stop_lines("rusia")
    mids = {arm: np.asarray(seg, dtype=float).mean(axis=0) for arm, seg in sl.items()}
    ref_center = np.mean(list(mids.values()), axis=0) if len(mids) >= 2 else np.asarray(center, dtype=float)
    if np.hypot(*(np.asarray(pt, dtype=float) - ref_center)) <= core_radius:
        return CORE
    return min(mids, key=lambda arm: float(np.hypot(*(np.asarray(pt, dtype=float) - mids[arm]))))


def anchors_from_control_points(site_id: str) -> dict[str, np.ndarray]:
    """Per-approach calibrated anchor (centroid of the control points) — cross-check only."""
    cp = ROOT / "verify_out" / "homography_gcp" / "control_points.csv"
    if not cp.is_file():
        return {}
    df = pd.read_csv(cp)
    df = df[df["available"].astype(str).str.lower().isin({"true", "1"})]
    return {arm.lower(): g[["pixel_x", "pixel_y"]].mean().to_numpy()
            for arm, g in df.groupby("arm")}


def nearest_anchor(pt: np.ndarray, anchors: dict[str, np.ndarray]) -> str:
    return min(anchors, key=lambda a: np.hypot(*(pt - anchors[a])))


def load_stop_lines(site_id: str) -> dict[str, np.ndarray]:
    """Per-approach stop-line segments from zones.yaml (px): arm -> array (2,2)."""
    zones_yaml = ROOT / "config" / "sites" / site_id / "zones.yaml"
    with open(zones_yaml, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    sl = {arm: np.asarray(seg, dtype=float) for arm, seg in (data.get("stop_lines") or {}).items()}
    _ACTIVE_STOP_LINES.clear()
    _ACTIVE_STOP_LINES.update(sl)
    return sl


def load_lane_bands(site_id: str) -> dict:
    """Lane bands from zones.yaml (centroids of the transverse coordinate s in px on the stop
    line). {} if not yet calibrated. Structure: {entry: {arm: ...}, exit: {arm: ...}} where
    each approach is {resolution: per_lane|grouped|arm_only, centroids_s: {label: value}}. Entry and
    exit share the plane (stop_lines) but have different centroids. See
    docs/modules/lane_classifier.md."""
    zones_yaml = ROOT / "config" / "sites" / site_id / "zones.yaml"
    with open(zones_yaml, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    return data.get("lane_bands") or {}


def _inward_normal(seg: np.ndarray, center: np.ndarray):
    """Unit normal of the stop-line segment, oriented TOWARD the image center."""
    p1, p2 = seg[0], seg[1]
    v = p2 - p1
    n = np.array([-v[1], v[0]], dtype=float)
    if n.dot(center - p1) < 0:
        n = -n
    return p1, n / np.linalg.norm(n)


def stopline_crossing_time(g: pd.DataFrame, seg: np.ndarray, center: np.ndarray, kind: str):
    """Instant at which the ground contact CROSSES the stop line (None if it does not cross it).

    Signed distance to the segment, oriented toward the center (+ = intersection side).
    `kind='entry'`: first crossing −→+ (from outside inward). `kind='exit'`: last +→−.
    Crossing interpolated linearly between the two frames that bracket it → sub-frame.
    """
    p1, nh = _inward_normal(seg, center)
    pts = g[[GROUND_X, GROUND_Y]].to_numpy()
    t = g["timestamp_s"].to_numpy()
    d = (pts - p1) @ nh
    if kind == "entry":
        idx = [i for i in range(len(d) - 1) if d[i] <= 0 < d[i + 1]]
        if not idx:
            return None
        i = idx[0]
    else:
        idx = [i for i in range(len(d) - 1) if d[i] > 0 >= d[i + 1]]
        if not idx:
            return None
        i = idx[-1]
    return float(t[i] + (t[i + 1] - t[i]) * (0.0 - d[i]) / (d[i + 1] - d[i]))


def lane_cross_coord(g: pd.DataFrame, seg: np.ndarray, center: np.ndarray,
                     window: int = 3, kind: str = "entry"):
    """Transverse coordinate (projection of the ground contact onto the stop line) AT THE CROSSING.
    None if there is no clean crossing. It is the variable on which the lane bands are
    calibrated and applied (see docs/modules/lane_classifier.md): it is measured RIGHT at the stop line (narrow window), not
    by averaging — verified that on exit it also separates better at the crossing than farther out.
    `kind='entry'`: first crossing −→+ (approach). `kind='exit'`: last +→− (egress, same
    stop-line plane crossed outward — no new reference)."""
    p1, nh = _inward_normal(seg, center)
    pts = g[[GROUND_X, GROUND_Y]].to_numpy()
    d = (pts - p1) @ nh
    if kind == "entry":
        idx = [i for i in range(len(d) - 1) if d[i] <= 0 < d[i + 1]]
        if not idx:
            return None
        i = idx[0]
    else:                                                          # exit: last crossing +→−
        idx = [i for i in range(len(d) - 1) if d[i] > 0 >= d[i + 1]]
        if not idx:
            return None
        i = idx[-1]
    lo, hi = max(0, i - window + 1), min(len(g), i + window + 1)    # narrow window at the crossing
    pt = np.median(pts[lo:hi], axis=0)
    u = seg[1] - seg[0]
    u = u / np.linalg.norm(u)                                       # stop-line direction
    return float((pt - p1) @ u)


def classify_lane(s, arm_band) -> str:
    """Lane by nearest CENTROID over the transverse coordinate s. '' if not resolvable
    (no crossing, no bands, or approach at 'arm_only' resolution)."""
    if s is None or not arm_band or arm_band.get("resolution") == "arm_only":
        return ""
    cents = arm_band["centroids_s"]
    return str(min(cents, key=lambda k: abs(float(cents[k]) - s)))


def derive_events(df: pd.DataFrame, center: np.ndarray, *, window: int,
                  core_radius: float, excl_polys: dict[str, np.ndarray],
                  stop_lines: dict[str, np.ndarray], lane_bands: dict | None = None,
                  anchors: dict[str, np.ndarray] | None = None) -> pd.DataFrame:
    """Derives one event per track. Returns a DataFrame with all tracks (with excluded flag)."""
    add_ground_contact(df)
    pts = df[[GROUND_X, GROUND_Y]].to_numpy()

    # Fraction of each track inside ANY exclusion polygon.
    in_excl = np.zeros(len(df), dtype=bool)
    for poly in excl_polys.values():
        in_excl |= points_in_polygon(pts, poly)
    df = df.assign(_in_excl=in_excl)
    frac = df.groupby("track_id")["_in_excl"].mean()
    excluded = set(frac[frac > EXCLUSION_FRACTION].index)

    rows = []
    for tid, g in df.groupby("track_id"):
        g = g.sort_values("frame_index")
        head, tail = g.head(window), g.tail(window)
        entry = head[[GROUND_X, GROUND_Y]].median().to_numpy()
        exit_ = tail[[GROUND_X, GROUND_Y]].median().to_numpy()
        d_entry = float(np.hypot(*(entry - center)))
        d_exit = float(np.hypot(*(exit_ - center)))
        entry_arm = arm_of(entry, center, core_radius, stop_lines=stop_lines)
        exit_arm = arm_of(exit_, center, core_radius, stop_lines=stop_lines)
        is_excl = tid in excluded
        complete = (entry_arm != CORE) and (exit_arm != CORE)
        status = "excluded" if is_excl else ("complete" if complete else "partial")
        movement = "" if not complete else ("same" if entry_arm == exit_arm else "cross")

        # CLASS stabilized by simple MAJORITY VOTE (per-frame frequency) over the track
        # (reconnected). Standard MOT technique: corrects the class blinking of isolated frames.
        # Weighted variants (area/confidence) explored, with no conclusive improvement on this sample.
        vc = g["class_id"].map(CLASS_BY_COCO).value_counts()
        vehicle_class = str(vc.idxmax()) if len(vc) else ""

        # APPEARANCE (track fact) vs PHYSICAL INSTANT (stop-line crossing = what the
        # GT measures). The approach is still classified by the appearance corner (validated); the entry/exit
        # TIME is the crossing of THAT approach's stop line. If the approach is CORE
        # (partial) or the track does not cross the line, it falls back to the appearance and the fallback is marked.
        first_seen = float(g["timestamp_s"].iloc[0])
        last_seen = float(g["timestamp_s"].iloc[-1])
        t_in = stopline_crossing_time(g, stop_lines[entry_arm], center, "entry") if entry_arm in stop_lines else None
        t_out = stopline_crossing_time(g, stop_lines[exit_arm], center, "exit") if exit_arm in stop_lines else None
        entry_fb, exit_fb = t_in is None, t_out is None

        # Entry and exit LANE: image-plane bands, measured at the stop-line crossing
        # (entry = first crossing −→+; exit = last +→−, same plane crossed outward).
        # Per-approach resolution by calibration (per_lane / grouped / arm_only). '' if not resolvable.
        lb = lane_bands or {}
        entry_band = lb.get("entry", {}).get(entry_arm)
        s_entry = (lane_cross_coord(g, stop_lines[entry_arm], center, kind="entry")
                   if (entry_band and entry_arm in stop_lines) else None)
        entry_lane = classify_lane(s_entry, entry_band)
        exit_band = lb.get("exit", {}).get(exit_arm)
        s_exit = (lane_cross_coord(g, stop_lines[exit_arm], center, kind="exit")
                  if (exit_band and exit_arm in stop_lines) else None)
        exit_lane = classify_lane(s_exit, exit_band)
        rows.append(dict(
            track_id=tid, n_obs=len(g),
            entry_frame=int(g["frame_index"].iloc[0]), exit_frame=int(g["frame_index"].iloc[-1]),
            first_seen_s=round(first_seen, 6), last_seen_s=round(last_seen, 6),
            entry_time_s=round(first_seen if entry_fb else t_in, 6),
            exit_time_s=round(last_seen if exit_fb else t_out, 6),
            entry_time_fallback=entry_fb, exit_time_fallback=exit_fb,
            entry_gx=round(entry[0], 2), entry_gy=round(entry[1], 2),
            exit_gx=round(exit_[0], 2), exit_gy=round(exit_[1], 2),
            entry_dist_center_px=round(d_entry, 1), exit_dist_center_px=round(d_exit, 1),
            entry_arm=entry_arm, entry_lane=entry_lane, exit_arm=exit_arm, exit_lane=exit_lane,
            vehicle_class=vehicle_class,
            entry_arm_xcheck=nearest_anchor(entry, anchors) if anchors else "",
            exit_arm_xcheck=nearest_anchor(exit_, anchors) if anchors else "",
            status=status, movement=movement,
        ))
    return pd.DataFrame(rows).sort_values("track_id").reset_index(drop=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--site", default="rusia")
    ap.add_argument("--tracks", default="verify_out/tracks_rusia.csv")
    ap.add_argument("--out", default=None, help="por defecto verify_out/events/<site>/track_events.csv")
    ap.add_argument("--window", type=int, default=WINDOW_FRAMES)
    ap.add_argument("--core-radius", type=float, default=CORE_RADIUS_PX)
    args = ap.parse_args()

    site = load_site_config(args.site)
    center = np.array([site.frame_width / 2.0, site.frame_height / 2.0])
    excl_polys, zw, zh = load_exclusion_polygons(args.site)
    if (zw, zh) != (site.frame_width, site.frame_height):
        print(f"ABORTA: zones.yaml ({zw}x{zh}) != site.yaml "
              f"({site.frame_width}x{site.frame_height})", file=sys.stderr)
        return 2
    anchors = anchors_from_control_points(args.site)
    stop_lines = load_stop_lines(args.site)
    lane_bands = load_lane_bands(args.site)
    missing = set(site.arm_set) - set(stop_lines)
    if missing:
        print(f"ABORTA: faltan líneas de pare en zones.yaml para {sorted(missing)}", file=sys.stderr)
        return 2

    tracks_path = ROOT / args.tracks
    df = pd.read_csv(tracks_path)
    ev = derive_events(df, center, window=args.window, core_radius=args.core_radius,
                       excl_polys=excl_polys, stop_lines=stop_lines, lane_bands=lane_bands,
                       anchors=anchors)

    out = Path(args.out) if args.out else ROOT / "verify_out" / "events" / args.site / "track_events.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    # The events CSV only contains NON-excluded tracks (no events are generated for parked vehicles).
    ev_out = ev[ev.status != "excluded"].drop(columns=["entry_arm_xcheck", "exit_arm_xcheck"]).copy()
    ev_out.to_csv(out, index=False)

    # --- Summary ---
    excl = ev[ev.status == "excluded"]
    act = ev[ev.status != "excluded"]
    comp = act[act.status == "complete"]
    print(f"Tracks totales: {len(ev)}")
    print(f"Excluidos (parqueadero, >{EXCLUSION_FRACTION:.0%} en exclusión): "
          f"{len(excl)} -> {sorted(excl.track_id.tolist())}")
    try:
        shown = out.relative_to(ROOT)
    except ValueError:
        shown = out
    print(f"Eventos escritos (no excluidos): {len(act)}  ->  {shown}")
    print(f"  COMPLETOS (entrada Y salida por esquina clara): {len(comp)}")
    print(f"    cross-arm (cruce real entrada≠salida):       {(comp.movement=='cross').sum()}")
    print(f"    same-arm  (mismo brazo; GT no tiene U-turns): {(comp.movement=='same').sum()}")
    print(f"  PARCIALES (entrada o salida en el núcleo):      {(act.status=='partial').sum()}")
    ne, nx = act[act.entry_arm != CORE], act[act.exit_arm != CORE]
    print(f"  contraste cuadrante==ancla-calibrada (solo no-core): entrada "
          f"{(ne.entry_arm==ne.entry_arm_xcheck).mean():.3f}, "
          f"salida {(nx.exit_arm==nx.exit_arm_xcheck).mean():.3f}")
    cr = comp[comp.movement == "cross"]
    print(f"  tiempos por CRUCE de línea de pare; fallback a aparición (no cruza) en cross-arm: "
          f"entrada {int(cr.entry_time_fallback.sum())}/{len(cr)}, "
          f"salida {int(cr.exit_time_fallback.sum())}/{len(cr)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

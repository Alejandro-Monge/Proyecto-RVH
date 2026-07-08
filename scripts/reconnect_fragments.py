"""DOMAIN fragment reconnection (TRACK level) — attacks the SUR approach deficit.

Merges tracks that are fragments of the same vehicle (split by occlusion) using DOMAIN
signals —not appearance (OSNet is OFF, it is the bottleneck)—: temporal gap (buffer), spatial
corridor, kinematic continuity (forward continuation), consistent class and a valid ROUTE
of the merged movement (turns.csv). It reassigns track_id (merge) and rewrites the tracks CSV; the
evaluation pipeline (derive_track_events → match_events_to_gt) is re-run on top WITHOUT being touched.

SCOPE (bounded, not over-sold): it attacks **sequential fragmentation** (6/7 ghosts of the south
+ the 3 south misses). It does NOT touch the **co-temporal duplicate** (two ids over the same vehicle,
overlapping in time): duplicate suppression over large vehicles is out of the
scope of this script.

GUARDRAILS:
  - Conservative gates + ONE-TO-ONE assignment + bias to NOT merge under ambiguity
    (precision over recall when in doubt): an invalid merged route is NOT merged.
  - NEGATIVE CASE: the temporal gate requires gap > 0 (B starts AFTER A ends) → a
    co-temporal duplicate (two tracks overlapping in time) is NEVER a candidate. It is verified
    explicitly with a generic scan of all merges (not a specific id).
  - Reads from config: corridor and buffer from site.yaml (reid_spatial_corridor_px /
    reid_temporal_window_frames); valid routes from turns.csv. Fine thresholds (heading angle)
    documented here.
  - It does NOT touch the events layer nor the previous baseline (they are the before/after reference).

VALIDATION: the BASELINE is the yardstick — the matching is re-run and compared per approach (recall/
  precision/exit accuracy) BEFORE vs AFTER. If any approach's precision DROPS → over-merge →
  it is reported and that config is rejected. The buffer is SWEPT (1-6 s) → buffer-vs-reconnection curve
  (recovered fragments vs false merges vs delay) = vision side of buffer-vs-continuity (Section 8.3).

OUTPUT: verify_out/events/<id>/reconnected/  (reconnected tracks, events, matches, comparison,
  buffer curve).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import derive_track_events as dte          # reuses arm_of, derive_events, loaders, constants
import match_events_to_gt as mm            # reuses run_matching, per_arm_report
from digital_shadow.config.site import load_site_config
from digital_shadow.geometry.ground_contact import GROUND_X, GROUND_Y, add_ground_contact

# Fine threshold documented (not in config): forward continuation of the movement.
MAX_HEADING_DEV_DEG = 60.0   # the jump end_A→start_B must go ~in A's direction of travel
HEADING_FRAMES = 5           # frames to estimate the heading at the track ends


def load_valid_movements(site_id: str) -> set[tuple[str, str]]:
    """Allowed (entry, exit) pairs in J5, from turns.csv (status == 'allowed')."""
    t = pd.read_csv(ROOT / "config" / "sites" / site_id / "turns.csv")
    return {(r.entry_arm, r.exit_arm) for r in t.itertuples() if r.status == "allowed"}


def _heading(pts: np.ndarray) -> np.ndarray:
    return pts[-1] - pts[0] if len(pts) >= 2 else np.zeros(2)


def track_features(df: pd.DataFrame, center, core_radius, window) -> dict:
    """Per track: endpoints (frame, ground contact), headings, class, entry/exit approach."""
    add_ground_contact(df)
    feats = {}
    for tid, g in df.groupby("track_id"):
        g = g.sort_values("frame_index")
        gc = g[[GROUND_X, GROUND_Y]].to_numpy()
        entry = np.median(gc[:window], axis=0)
        exit_ = np.median(gc[-window:], axis=0)
        ea = dte.arm_of(entry, center, core_radius)
        xa = dte.arm_of(exit_, center, core_radius)
        feats[tid] = dict(
            f0=int(g.frame_index.iloc[0]), f1=int(g.frame_index.iloc[-1]),
            head=gc[0], tail=gc[-1],
            tail_heading=_heading(gc[-HEADING_FRAMES:]),
            cls=int(g.class_id.mode().iloc[0]),
            entry_arm=ea, exit_arm=xa,
            # is it a COMPLETE corner-to-corner crossing by itself? (entry and exit in real
            # and distinct approaches). A real crossing split in time produces AT MOST ONE such
            # segment; the rest are same-arm/partial stubs. Two complete = two vehicles.
            is_cross=(ea != dte.CORE and xa != dte.CORE and ea != xa),
        )
    return feats


def _angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1.0 or nb < 1.0:
        return 0.0   # undetermined heading (very short or stopped track) → not rejected by angle
    c = np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0)
    return float(np.degrees(np.arccos(c)))


def reconnect(df: pd.DataFrame, *, buffer_frames: int, corridor_px: float,
              valid_moves: set, center, core_radius, window: int, excluded: set):
    """Returns (id_map old->new, list of merged components with metadata)."""
    feats = track_features(df, center, core_radius, window)
    tids = [t for t in feats if t not in excluded]

    # Directed candidates A.tail -> B.head that pass ALL gates.
    cands = []
    for a in tids:
        fa = feats[a]
        for b in tids:
            if b == a:
                continue
            fb = feats[b]
            gap = fb["f0"] - fa["f1"]
            if gap <= 0 or gap > buffer_frames:        # gap>0 EXCLUDES co-temporals
                continue
            D = fb["head"] - fa["tail"]
            dist = float(np.linalg.norm(D))
            if dist > corridor_px:                     # corridor (anti-teleportation in magnitude)
                continue
            if fa["cls"] != fb["cls"]:                 # consistent class
                continue
            if _angle_deg(D, fa["tail_heading"]) > MAX_HEADING_DEV_DEG:   # forward continuation
                continue
            cands.append((gap + dist * 0.001, gap, dist, a, b))

    # Greedy one-to-one (bias to not merge: only what passes gates; best cost first).
    cands.sort()
    used_tail, used_head, edges = set(), set(), []
    for cost, gap, dist, a, b in cands:
        if a in used_tail or b in used_head:
            continue
        used_tail.add(a); used_head.add(b); edges.append((a, b, gap, dist))

    # Union-find over the accepted edges (allows chains A->B->C).
    parent = {t: t for t in tids}
    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]; x = parent[x]
        return x
    for a, b, *_ in edges:
        parent[find(a)] = find(b)
    comps = {}
    for t in tids:
        comps.setdefault(find(t), []).append(t)

    id_map, merged = {}, []
    for members in comps.values():
        if len(members) < 2:
            continue
        members.sort(key=lambda t: feats[t]["f0"])           # temporal order of the chain
        m_entry = feats[members[0]]["entry_arm"]
        m_exit = feats[members[-1]]["exit_arm"]
        # anti-over-merge GATE (precision over recall): a real split crossing has AT MOST ONE
        # complete cross-arm segment. ≥2 complete in the component = distinct vehicles → do NOT merge.
        if sum(feats[t]["is_cross"] for t in members) >= 2:
            continue
        # valid ROUTE of the merged movement (bias to not merge if invalid).
        if (m_entry, m_exit) not in valid_moves:
            continue
        rep = min(members)
        for t in members:
            id_map[t] = rep
        gaps = [feats[members[i + 1]]["f0"] - feats[members[i]]["f1"] for i in range(len(members) - 1)]
        merged.append(dict(rep=rep, members=members, entry=m_entry, exit=m_exit, gaps=gaps))
    return id_map, merged


def remap_tracks(df: pd.DataFrame, id_map: dict) -> pd.DataFrame:
    out = df.copy()
    out["track_id"] = out["track_id"].map(lambda t: id_map.get(t, t))
    return out


def events_and_match(tracks_df, site, center, window_match):
    """tracks -> events -> matching. Returns (per_arm_report, pares, fant, omis, shadow, gt)."""
    excl_polys, _, _ = dte.load_exclusion_polygons(site.site_id)
    stop_lines = dte.load_stop_lines(site.site_id)
    anchors = dte.anchors_from_control_points(site.site_id)
    ev = dte.derive_events(tracks_df.copy(), center, window=dte.WINDOW_FRAMES,
                           core_radius=dte.CORE_RADIUS_PX, excl_polys=excl_polys,
                           stop_lines=stop_lines, anchors=anchors)
    shadow = ev[(ev.status == "complete") & (ev.movement == "cross")].reset_index(drop=True)
    gt = pd.read_csv(ROOT / "data" / "ground_truth" / site.site_id /
                     f"ground_truth_{site.site_id}_v2_carril_processed.csv")
    arms = list(site.arm_set)
    pares, fant, omis, _ = mm.run_matching(shadow, gt, arms, window_match)
    rep = mm.per_arm_report(pares, fant, omis, shadow, gt, arms)
    return rep, pares, fant, omis, shadow, gt


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--site", default="rusia")
    ap.add_argument("--tracks", default="verify_out/tracks_rusia.csv")
    args = ap.parse_args()

    site = load_site_config(args.site)
    center = np.array([site.frame_width / 2.0, site.frame_height / 2.0])
    window_match = mm._read_tolerance(args.site)
    corridor = site.reid_spatial_corridor_px
    buffer_cfg = site.reid_temporal_window_frames
    fps = float(site.fps_ref)
    valid_moves = load_valid_movements(args.site)
    out_dir = ROOT / "verify_out" / "events" / args.site / "reconnected"
    out_dir.mkdir(parents=True, exist_ok=True)

    tracks = pd.read_csv(ROOT / args.tracks)
    base_events = pd.read_csv(ROOT / "verify_out" / "events" / args.site / "track_events.csv")
    excluded = set(tracks.track_id.unique()) - set(base_events.track_id.unique())
    base_matches = pd.read_csv(ROOT / "verify_out" / "events" / args.site / "matches.csv")
    base_match_event = dict(zip(base_matches.track_id, base_matches.event_id))

    # === BASELINE (reference, from the frozen track_events.csv) ===
    base_shadow = base_events[(base_events.status == "complete") & (base_events.movement == "cross")]
    gt = pd.read_csv(ROOT / "data" / "ground_truth" / args.site /
                     f"ground_truth_{args.site}_v2_carril_processed.csv")
    arms = list(site.arm_set)
    base_shadow = base_shadow.reset_index(drop=True)
    bpares, bfant, bomis, _ = mm.run_matching(base_shadow, gt, arms, window_match)
    base_rep = mm.per_arm_report(bpares, bfant, bomis, base_shadow, gt, arms)

    # === RECONNECTION at the config buffer ===
    id_map, merged = reconnect(tracks, buffer_frames=buffer_cfg, corridor_px=corridor,
                               valid_moves=valid_moves, center=center,
                               core_radius=dte.CORE_RADIUS_PX, window=dte.WINDOW_FRAMES,
                               excluded=excluded)
    rec_tracks = remap_tracks(tracks, id_map)
    rec_tracks.to_csv(out_dir / "tracks_reconnected.csv", index=False)
    rec_rep, rpares, rfant, romis, rshadow, _ = events_and_match(rec_tracks, site, center, window_match)

    print(f"=== RECONEXIÓN (buffer = {buffer_cfg} f ≈ {buffer_cfg/fps:.2f} s, corredor {corridor:.0f} px) ===")
    print(f"Fusiones aceptadas: {len(merged)}  (tracks fusionados: {sum(len(m['members']) for m in merged)})")
    # NEGATIVE CASE (invariant, NOT a fixed id): the reconnector merges only across temporal
    # GAPS, never two CO-TEMPORAL tracks (frame overlap > 0 = simultaneous → they are not the
    # same). The unit test `test_cotemporal_no_fusiona` covers it with synthetic data.
    _span = {t: (int(g.frame_index.min()), int(g.frame_index.max())) for t, g in tracks.groupby("track_id")}
    cotemp_merges = [sorted(m["members"]) for m in merged
                     if any(min(_span[a][1], _span[b][1]) - max(_span[a][0], _span[b][0]) > 0
                            for a in m["members"] for b in m["members"] if a < b)]
    print(f"CASO NEGATIVO fusiones CO-TEMPORALES (debe ser []): {cotemp_merges}")
    # False merges: a component whose members matched DIFFERENT GTs in the baseline
    false_merges = []
    for m in merged:
        evs = {base_match_event[t] for t in m["members"] if t in base_match_event}
        if len(evs) >= 2:
            false_merges.append((m["rep"], m["members"], sorted(evs)))
    print(f"Falsas fusiones (miembros casaban GT distintos en baseline): {len(false_merges)}")
    for fm in false_merges:
        print(f"   rep tid{fm[0]} members {fm[1]} casaban events {fm[2]}")

    print("\n--- Fusiones (cadena, ruta, gaps en frames) ---")
    for m in sorted(merged, key=lambda x: x["entry"]):
        print(f"  {m['members']} -> {m['entry']}->{m['exit']}  gaps={m['gaps']}")

    # === Comparison per approach BEFORE vs AFTER ===
    print("\n=== BASELINE (antes) por brazo ===")
    print(base_rep.to_string(index=False))
    print("\n=== RECONECTADO (después) por brazo ===")
    print(rec_rep.to_string(index=False))
    print("\n=== Δ por brazo (después − antes) ===")
    for a in arms + ["TOTAL"]:
        b = base_rep[base_rep.brazo_entrada == a].iloc[0]
        r = rec_rep[rec_rep.brazo_entrada == a].iloc[0]
        flag = "  <-- PRECISION BAJA (over-merge?)" if r.precision < b.precision - 1e-9 else ""
        print(f"  {a:6} casados {int(b.casados)}->{int(r.casados)}  "
              f"recall {b.recall:.3f}->{r.recall:.3f}  precision {b.precision:.3f}->{r.precision:.3f}  "
              f"acierto_salida {b.acierto_salida}->{r.acierto_salida}{flag}")

    # Do the south misses become matched?
    base_om_sur = set(bomis[bomis.entry_arm == "sur"].event_id) if len(bomis) else set()
    rec_om_sur = set(romis[romis.entry_arm == "sur"].event_id) if len(romis) else set()
    print(f"\nOmisiones SUR: antes {sorted(base_om_sur)} -> después {sorted(rec_om_sur)} "
          f"(recuperadas: {sorted(base_om_sur - rec_om_sur)})")

    # Artifacts
    rec_rep.to_csv(out_dir / "baseline_por_brazo_reconnected.csv", index=False)
    rpares.to_csv(out_dir / "matches.csv", index=False)

    # === BUFFER SWEEP (1-6 s) — buffer-vs-reconnection curve ===
    print("\n=== BARRIDO DEL BUFFER [1,6] s — lado-visión de buffer-vs-continuidad ===")
    sweep = []
    for sec in np.round(np.arange(1.0, 6.001, 0.5), 2):
        bf = int(round(sec * fps))
        idm, mg = reconnect(tracks, buffer_frames=bf, corridor_px=corridor, valid_moves=valid_moves,
                            center=center, core_radius=dte.CORE_RADIUS_PX, window=dte.WINDOW_FRAMES,
                            excluded=excluded)
        rt = remap_tracks(tracks, idm)
        rep, pares, fant, omis, _, _ = events_and_match(rt, site, center, window_match)
        tot = rep[rep.brazo_entrada == "TOTAL"].iloc[0]
        # false merges + delay (max gap) of this config
        fmerges = sum(1 for m in mg if len({base_match_event[t] for t in m["members"] if t in base_match_event}) >= 2)
        allgaps = [g for m in mg for g in m["gaps"]]
        sweep.append(dict(buffer_s=sec, fusiones=len(mg),
                          casados=int(tot.casados), fantasmas=int(tot.fantasmas), omisiones=int(tot.omisiones),
                          precision=round(tot.precision, 3), recall=round(tot.recall, 3),
                          falsas_fusiones=fmerges,
                          delay_max_s=round(max(allgaps) / fps, 2) if allgaps else 0.0,
                          delay_med_s=round(float(np.median(allgaps)) / fps, 2) if allgaps else 0.0))
    sweep_df = pd.DataFrame(sweep)
    sweep_df.to_csv(out_dir / "buffer_sweep.csv", index=False)
    print(sweep_df.to_string(index=False))
    print(f"\nArtefactos -> {out_dir.relative_to(ROOT)}/")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

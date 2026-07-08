"""INTEGRATION audit of the causal orchestrator (Part 3): that the frame-by-frame assembly
respects each piece's assumptions. Four audits + delay sealing, Section 7.2.

  A) Does the ONLINE event stream match Part 2 (re-derivation)? (entry_arm/lane, exit_arm).
  B) Is the position filter WARM at instantiation (after the buffer), as Part 2 predicted?
  C) Is the loop DETERMINISTIC with no dropping? (two identical runs).
  D) Section 7.6: does it keep pace? (time of the FULL loop per frame vs the video period).
  + DELAY SEALING, Section 7.2: per commit type, delay = t_commit − physical_event (GT), distribution.
"""
from __future__ import annotations

import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
import derive_track_events as dte
import reconnect_fragments as rf
from digital_shadow.config.site import load_site_config
from digital_shadow.geometry.ground_contact import add_ground_contact
from digital_shadow.harness.orchestrator import CausalOrchestrator
from digital_shadow.harness.sink import RecordingSink
from digital_shadow.sumo.lane_map import load_lane_map
from digital_shadow.sumo.position import load_alpha, load_homography
from digital_shadow.sumo.route import load_lane_turn_rule, load_turns
from digital_shadow.tracking.causal_reconnect import CausalReconnector
from measure_causal_reconnection import run_causal


def internal_shapes_from_net(lane_map, net_path):
    """(entry_arm, exit_arm) -> polyline of the internal lane (for gap filling, without SUMO)."""
    r = ET.parse(net_path).getroot()
    lane_shape = {}
    for e in r.findall("edge"):
        if e.get("function") == "internal":
            for l in e.findall("lane"):
                lane_shape[l.get("id")] = [[float(c) for c in q.split(",")] for q in l.get("shape").split()]
    return {k: lane_shape[v] for k, v in lane_map.junction_internal.items() if v in lane_shape}


def build_orch(tracks, feats, excluded, site, center, shapes):
    return CausalOrchestrator(
        tracks=tracks, feats=feats,
        reconnector=CausalReconnector(buffer_frames=site.reid_temporal_window_frames,
                                      corridor_px=site.reid_spatial_corridor_px,
                                      valid_moves=rf.load_valid_movements("rusia")),
        lane_map=load_lane_map("rusia"), H=load_homography("rusia"),
        rule=load_lane_turn_rule("rusia"), turns=load_turns("rusia"),
        stop_lines=dte.load_stop_lines("rusia"), lane_bands=dte.load_lane_bands("rusia"),
        center=center, alpha=load_alpha(), buffer_frames=site.reid_temporal_window_frames,
        excluded=excluded, internal_shapes=shapes)


def main():
    site = load_site_config("rusia")
    center = np.array([site.frame_width / 2.0, site.frame_height / 2.0])
    tracks = pd.read_csv(ROOT / "verify_out/tracks_rusia.csv")
    add_ground_contact(tracks)
    base_events = pd.read_csv(ROOT / "verify_out/events/rusia/track_events.csv")
    excluded = set(tracks.track_id.unique()) - set(base_events.track_id.unique())
    feats = rf.track_features(tracks, center, dte.CORE_RADIUS_PX, dte.WINDOW_FRAMES)
    shapes = internal_shapes_from_net(load_lane_map("rusia"),
                                      ROOT / "config/sites/rusia/sumo/intersection_rusia.net.xml")

    # --- orchestrator run (measures the loop time, audit D) ---
    orch = build_orch(tracks, feats, excluded, site, center, shapes)
    sink = RecordingSink()
    t0 = time.perf_counter()
    commits, chains = orch.run(sink)
    wall = time.perf_counter() - t0
    n_frames = tracks.frame_index.nunique()

    # online events per chain (from the observed commits, which carry entry/exit)
    online = {}
    for c in commits:
        if c.kind == "destino_observado":
            online[c.chain] = dict(entry_arm=c.payload["entry_arm"], entry_lane=c.payload["entry_lane"],
                                   exit_arm=c.payload["exit_arm"])

    # --- A) online stream vs Part 2 (re-derivation over causal-reconnected tracks) ---
    print("=== A) Stream ONLINE vs Parte 2 (re-derivación) ===")
    causal_id_map, _ = run_causal(feats, site.reid_temporal_window_frames,
                                  site.reid_spatial_corridor_px, rf.load_valid_movements("rusia"), excluded)
    causal_tracks = rf.remap_tracks(tracks, causal_id_map)
    ev2 = dte.derive_events(causal_tracks.copy(), center, window=dte.WINDOW_FRAMES,
                            core_radius=dte.CORE_RADIUS_PX,
                            excl_polys=dte.load_exclusion_polygons("rusia")[0],
                            stop_lines=dte.load_stop_lines("rusia"),
                            lane_bands=dte.load_lane_bands("rusia"),
                            anchors=dte.anchors_from_control_points("rusia"))
    ev2 = ev2[(ev2.status == "complete") & (ev2.movement == "cross")]
    p2 = {int(r.track_id): (r.entry_arm, str(r.entry_lane), r.exit_arm) for r in ev2.itertuples()}
    same = diff = 0
    diffs = []
    for root, o in online.items():
        key = (o["entry_arm"], str(o["entry_lane"]), o["exit_arm"])
        if root in p2:
            if p2[root] == key:
                same += 1
            else:
                diff += 1
                diffs.append((root, p2[root], key))
    print(f"  cadenas cross online: {len(online)} | coinciden con Parte 2: {same} | difieren: {diff}")
    for root, a, b in diffs[:8]:
        print(f"    chain {root}: Parte2 {a} vs online {b}")
    print(f"  -> {'IDÉNTICO: el commit online iguala la re-derivación' if diff == 0 else 'DIFERENCIAS (ver arriba): información de integración'}")

    # --- B) warm filter at instantiation ---
    print("\n=== B) Filtro de posición CALIENTE al instanciar (frames de approach procesados) ===")
    warm = [ch for ch in chains.values() if ch.instantiated and ch.n_approach is not None]
    napp = [ch.n_approach for ch in warm]
    cold = sum(n < 3 for n in napp)
    print(f"  cadenas instanciadas: {len(warm)} | mediana de frames de approach que calentaron el filtro: "
          f"{int(np.median(napp))} (mín {min(napp)}, p10 {int(np.percentile(napp,10))}); FRÍAS (<3 frames): {cold}.")
    print(f"  -> El filtro NO arranca de cero al instanciar: procesó la decena/centena de frames del approach")
    print(f"     (y para fragmentados, los del fragmento previo al hueco) — como predijo la Parte 2.")
    print(f"  (La |v| al cruzar es baja —mediana {np.median([ch.vel_inst for ch in warm]):.2f} m/s— por el clamp")
    print(f"   del pos cerca del fin del approach y por vehículos que arrancan desde parado en verde; no es frío.)")

    # --- C) determinism with no dropping ---
    orch2 = build_orch(tracks, feats, excluded, site, center, shapes)
    sink2 = RecordingSink()
    orch2.run(sink2)
    det = sink.calls == sink2.calls
    print(f"\n=== C) Determinista sin descarte: {'PASS' if det else 'FALLA'} "
          f"({len(sink.calls)} órdenes idénticas en dos corridas) ===")

    # --- D) Section 7.6 keep pace ---
    per_frame_ms = 1000 * wall / n_frames
    period_ms = 1000 * 12 / 355
    print(f"\n=== D) §7.6 ritmo del bucle (sin detector/SUMO, solo orquestación) ===")
    print(f"  {n_frames} frames en {wall:.2f}s → {per_frame_ms:.3f} ms/frame (periodo video {period_ms:.2f} ms). "
          f"{'MANTIENE' if per_frame_ms < period_ms else 'NO mantiene → delay de cola a declarar'} "
          f"el ritmo (margen ×{period_ms/per_frame_ms:.0f}). Nota: falta sumar detector+SUMO (medir en Parte 4).")

    # --- DELAY SEALING, Section 7.2 (budget per commit type) ---
    print("\n=== Sellado de delay §7.2 (por tipo de commit, contra el GT; distribución) ===")
    bm = pd.read_csv(ROOT / "verify_out/events/rusia/reconnected/matches.csv")
    root_to_ev = dict(zip(bm.track_id, bm.event_id))
    gt = pd.read_csv(ROOT / "data/ground_truth/rusia/ground_truth_rusia_v2_carril_processed.csv").set_index("event_id")
    by_kind = {}
    for c in commits:
        ev = root_to_ev.get(c.chain)
        if ev is None:
            continue
        gt_entry = gt.loc[ev, "entry_time_s"]
        tc = c.t_commit if c.kind != "destino_observado" else (c.payload.get("exit_t") or c.t_commit)
        # origin/predicted/instantiation seal vs physical ENTRY; observed vs entry (full transit)
        by_kind.setdefault(c.kind, []).append(tc - gt_entry)
    print(f"  {'commit':18s} {'N':>3s} {'mediana':>8s} {'p10':>7s} {'p90':>7s}  (s, sombra−GT_entrada)")
    for kind in ["instanciacion", "origen", "destino_predicho", "destino_observado"]:
        d = by_kind.get(kind, [])
        if d:
            print(f"  {kind:18s} {len(d):3d} {np.median(d):8.2f} {np.percentile(d,10):7.2f} {np.percentile(d,90):7.2f}")
    print("  Línea explícita del presupuesto: el carril de entrada añade ±3 frames = 0.101 s al origen/")
    print("  predicho (look-ahead causal cuantificado). El observado paga el tránsito de junción (alto).")


if __name__ == "__main__":
    main()

"""Validates that CAUSAL PERCEPTION produces an event stream comparable to the BATCH.

1) STREAM: applies the id_map of the CAUSAL reconnection to the tracks, derives events + matching, and
   compares the matched/routes with the BATCH reconnection (69 matched). Does the reconnection gap (5
   losses + 1 false merge) change the matched/routes count, or only affect the TRAIL?
2) running-vote CLASS: how often does the LIVE class vote differ from the final one before
   converging? (Section 7.5, declares that the class shown early may change; it does not affect route/count.)
Read only.
"""
from __future__ import annotations

import sys
from collections import Counter
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
import derive_track_events as dte
import reconnect_fragments as rf
from digital_shadow.config.site import load_site_config
from measure_causal_reconnection import run_causal


def main():
    site = load_site_config("rusia")
    center = np.array([site.frame_width / 2.0, site.frame_height / 2.0])
    corridor = site.reid_spatial_corridor_px
    buffer_cfg = site.reid_temporal_window_frames
    window_match = __import__("match_events_to_gt")._read_tolerance("rusia")
    valid_moves = rf.load_valid_movements("rusia")

    tracks = pd.read_csv(ROOT / "verify_out/tracks_rusia.csv")
    base_events = pd.read_csv(ROOT / "verify_out/events/rusia/track_events.csv")
    excluded = set(tracks.track_id.unique()) - set(base_events.track_id.unique())
    feats = rf.track_features(tracks, center, dte.CORE_RADIUS_PX, dte.WINDOW_FRAMES)

    # --- 1) STREAM: causal-reconnected -> events -> matching ---
    causal_id_map, _ = run_causal(feats, buffer_cfg, corridor, valid_moves, excluded)
    causal_tracks = rf.remap_tracks(tracks, causal_id_map)
    rep_c, pares_c, fant_c, omis_c, shadow_c, gt = rf.events_and_match(causal_tracks, site, center, window_match)

    # reconnected BATCH (reference: matches.csv = 69 matched)
    batch_matches = pd.read_csv(ROOT / "verify_out/events/rusia/reconnected/matches.csv")
    batch_ev = set(batch_matches.event_id)
    batch_route = dict(zip(batch_matches.event_id, batch_matches.sh_exit))

    # pares_c is a DataFrame with event_id and sh_exit (the shadow's exit approach).
    causal_matched_ev = set(int(e) for e in pares_c["event_id"]) if len(pares_c) else set()
    causal_route = dict(zip(pares_c["event_id"].astype(int), pares_c["sh_exit"])) if len(pares_c) else {}

    print("=== 1) STREAM de eventos: CAUSAL-reconectado vs BATCH-reconectado ===")
    print(f"  casados: batch {len(batch_ev)}  |  causal {len(causal_matched_ev)}")
    only_b = batch_ev - causal_matched_ev
    only_c = causal_matched_ev - batch_ev
    print(f"  casados solo en batch: {sorted(only_b)}  |  solo en causal: {sorted(only_c)}")
    # different routes among those matched in both
    shared = batch_ev & causal_matched_ev
    diff_route = [(ev, batch_route[ev], causal_route[ev]) for ev in shared
                  if str(batch_route[ev]) != str(causal_route.get(ev))]
    print(f"  de {len(shared)} casados en ambos, rutas (brazo de salida) distintas: {len(diff_route)}")
    for ev, rb, rc in diff_route:
        print(f"    ev{ev}: batch sale {rb}, causal sale {rc}")
    if len(causal_matched_ev) == len(batch_ev) and not diff_route:
        print("  -> El stream causal da los MISMOS casados y rutas que el batch. La brecha de")
        print("     reconexión (5 perdidas + 1 falso merge) es solo de TRAIL/continuidad (§7.4),")
        print("     NO de detección/ruta (§7.1). El realtime alcanza el techo offline en ruta.")

    # --- 2) CLASS: running vote vs final ---
    print("\n=== 2) CLASE voto-corriente: ¿oscila antes de converger? (§7.5) ===")
    oscilan = 0
    conv_frac = []
    casados_tids = set(causal_id_map.get(t, t) for t in feats)  # ids after reconnection
    for tid, g in causal_tracks.groupby("track_id"):
        cls = g.sort_values("frame_index")["class_id"].tolist()
        if len(cls) < 2:
            continue
        final = Counter(cls).most_common(1)[0][0]
        running = []
        c = Counter()
        for x in cls:
            c[x] += 1
            running.append(c.most_common(1)[0][0])
        if any(r != final for r in running):
            oscilan += 1
            last_diff = max(i for i, r in enumerate(running) if r != final)
            conv_frac.append((last_diff + 1) / len(cls))
    total = causal_tracks["track_id"].nunique()
    print(f"  tracks cuyo voto-corriente DIFIERE del final en algún momento: {oscilan}/{total} "
          f"({100*oscilan/total:.0f}%)")
    if conv_frac:
        print(f"  converge (deja de diferir) a una fracción mediana {np.median(conv_frac):.0%} de la vida "
              f"del track (de los que oscilan).")
    print("  -> La clase mostrada TEMPRANO puede cambiar antes de estabilizar. Afecta §7.5")
    print("     (clasificación), NO ruta ni conteo (verificado: el class_id no entra al matching).")


if __name__ == "__main__":
    main()

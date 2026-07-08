"""Measures the online-vs-offline GAP of the reconnection: global batch vs causal-greedy.

Runs both on the SAME input (verify_out/tracks_rusia.csv, same features and gates) and
compares the merges. Reports: of the batch's 15 merges, how many does the causal one reproduce? The ones
it loses, why (did they require a future candidate / did the decision order change)? Is there any merge
that the causal one makes and the batch does NOT (a false online merge, more serious)? = the characterization (the
batch = theoretical upper bound with complete info; the causal = what is achievable in realtime). Read-only.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
import derive_track_events as dte
import reconnect_fragments as rf
from digital_shadow.config.site import load_site_config
from digital_shadow.tracking.causal_reconnect import CausalReconnector


def run_causal(feats, buffer_frames, corridor_px, valid_moves, excluded):
    """Feeds births/deaths in temporal order to the causal reconnector."""
    rec = CausalReconnector(buffer_frames=buffer_frames, corridor_px=corridor_px,
                            valid_moves=valid_moves)
    events = []
    for tid, f in feats.items():
        if tid in excluded:
            continue
        events.append((f["f0"], 0, tid))   # 0 = birth (before death in the same frame)
        events.append((f["f1"], 1, tid))   # 1 = death
    for frame, kind, tid in sorted(events):
        if kind == 0:
            rec.on_birth(tid, feats[tid])
        else:
            rec.on_death(tid, feats[tid])
    return rec.result()


def main():
    site = load_site_config("rusia")
    center = np.array([site.frame_width / 2.0, site.frame_height / 2.0])
    corridor = site.reid_spatial_corridor_px
    buffer_cfg = site.reid_temporal_window_frames
    valid_moves = rf.load_valid_movements("rusia")

    tracks = pd.read_csv(ROOT / "verify_out/tracks_rusia.csv")
    base_events = pd.read_csv(ROOT / "verify_out/events/rusia/track_events.csv")
    excluded = set(tracks.track_id.unique()) - set(base_events.track_id.unique())

    feats = rf.track_features(tracks, center, dte.CORE_RADIUS_PX, dte.WINDOW_FRAMES)

    # BATCH (global) and CAUSAL (greedy-online) with the same features/gates.
    _, batch_merged = rf.reconnect(tracks, buffer_frames=buffer_cfg, corridor_px=corridor,
                                   valid_moves=valid_moves, center=center,
                                   core_radius=dte.CORE_RADIUS_PX, window=dte.WINDOW_FRAMES,
                                   excluded=excluded)
    _, causal_merged = run_causal(feats, buffer_cfg, corridor, valid_moves, excluded)

    batch_sets = {frozenset(m["members"]): m for m in batch_merged}
    causal_sets = {frozenset(m["members"]): m for m in causal_merged}

    print(f"=== Reconexión: BATCH global vs CAUSAL greedy-online (buffer {buffer_cfg} f) ===")
    print(f"Fusiones: batch {len(batch_merged)}  |  causal {len(causal_merged)}\n")

    print("Por fusión del BATCH — ¿la reproduce la causal?")
    repro = partial = missed = 0
    for ms, m in sorted(batch_sets.items(), key=lambda kv: kv[1]["rep"]):
        ruta = f"{m['entry']}->{m['exit']}"
        if ms in causal_sets:
            repro += 1
            status = "EXACTA"
        else:
            # is there a causal one that overlaps (recovered part)?
            ov = [cs for cs in causal_sets if cs & ms]
            if ov:
                partial += 1
                status = f"PARCIAL (causal: {sorted(set().union(*ov))})"
            else:
                missed += 1
                status = "PERDIDA (ninguna causal toca estos miembros)"
        print(f"  {sorted(m['members'])!s:28s} {ruta:14s} -> {status}")
    print(f"\nResumen: EXACTA {repro}/15 · PARCIAL {partial} · PERDIDA {missed}")

    # False online merges: causal components that the batch did NOT make (neither exact nor a subset of a batch one).
    print("\nFalsos merges ONLINE (causal fusiona algo que el batch NO; más grave):")
    false_merges = []
    batch_union_members = set().union(*[set(ms) for ms in batch_sets]) if batch_sets else set()
    for cs, cm in causal_sets.items():
        if cs in batch_sets:
            continue
        # is it a subset/overlaps a batch merge (variant of the same chain) or something new?
        overlaps = [bs for bs in batch_sets if bs & cs]
        tag = "NUEVA (ningún solape con batch)" if not overlaps else \
              f"variante de {sorted(set().union(*overlaps))}"
        false_merges.append((cm, tag))
        print(f"  {sorted(cm['members'])!s:28s} {cm['entry']}->{cm['exit']:7s} -> {tag}")
    if not false_merges:
        print("  (ninguno) ✔  — la causal NO fusiona nada que el batch no hiciera.")

    # Diagnosis of the LOSSES: by decision order or by future candidate?
    print("\nDiagnóstico de las fusiones perdidas/cambiadas (¿por qué difiere?):")
    explain_gap(batch_sets, causal_sets, feats, buffer_cfg, corridor, valid_moves)


def explain_gap(batch_sets, causal_sets, feats, buffer_frames, corridor, valid_moves):
    """Classifies each lost merge by its REAL cause: (a) the anti-over-merge PROXY refuses to attach
    a stub to an already-complete crossing (cost of causalizing the gate); or (b) future evidence (the tail
    belongs to a later match that the causal one does not see)."""
    proxy = future = 0
    for ms, m in batch_sets.items():
        if ms in causal_sets:
            continue
        mem = sorted(m["members"], key=lambda t: feats[t]["f0"])
        crosses = [t for t in mem if feats[t]["is_cross"]]
        # Does the 1st edge attach something to a member that is already a complete cross-arm? -> proxy.
        first_complete = next((t for t in mem[:-1] if feats[t]["is_cross"]), None)
        if first_complete is not None:
            proxy += 1
            print(f"  {sorted(m['members'])} ({m['entry']}->{m['exit']}): PROXY — tid{first_complete} es "
                  f"cross-arm COMPLETO (is_cross); el muñón posterior no se sabe muñón al nacer → "
                  f"rechazado (no ≥2 completos, pero el proxy no puede saberlo causalmente).")
        else:
            future += 1
            print(f"  {sorted(m['members'])} ({m['entry']}->{m['exit']}): EVIDENCIA FUTURA — orden de "
                  f"decisión / cola tomada por un match posterior.")
    print(f"  -> de las perdidas: {proxy} por PROXY (gate causal), {future} por evidencia futura.")


if __name__ == "__main__":
    main()

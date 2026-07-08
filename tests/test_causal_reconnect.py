"""Tests of the causal-greedy reconnector (`digital_shadow/tracking/causal_reconnect.py`), PURE.

Causal decisions over synthetic events; the gap vs the batch is measured in
`scripts/measure_causal_reconnection.py`. pytest or `python -m tests.test_causal_reconnect`.
"""

from __future__ import annotations

import numpy as np

from digital_shadow.tracking.causal_reconnect import CausalReconnector

VM = {("sur", "oeste"), ("este", "oeste")}


def feat(f0, f1, head, tail, heading=(1.0, 0.0), cls=2, entry="sur", exit="core", is_cross=False):
    return dict(f0=f0, f1=f1, head=np.array(head, float), tail=np.array(tail, float),
                tail_heading=np.array(heading, float), cls=cls, entry_arm=entry,
                exit_arm=exit, is_cross=is_cross)


def test_merge_basico_al_nacer():
    """B is born near A's tail (within the buffer, gates ok) → merges."""
    rec = CausalReconnector(buffer_frames=140, corridor_px=250, valid_moves=VM)
    A = feat(0, 10, head=(0, 0), tail=(0, 0), entry="sur", exit="core")
    B = feat(15, 25, head=(1, 0), tail=(50, 0), entry="core", exit="oeste")
    rec.on_birth(0, A); rec.on_death(0, A)
    edge = rec.on_birth(1, B); rec.on_death(1, B)
    assert edge is not None and edge[:2] == (0, 1)
    id_map, merged = rec.result()
    assert len(merged) == 1 and merged[0]["entry"] == "sur" and merged[0]["exit"] == "oeste"


def test_cotemporal_no_fusiona():
    """gap=0 (B is born when A dies) → never a candidate (excludes co-temporal duplicates)."""
    rec = CausalReconnector(buffer_frames=140, corridor_px=250, valid_moves=VM)
    A = feat(0, 10, head=(0, 0), tail=(0, 0))
    B = feat(10, 20, head=(1, 0), tail=(50, 0))   # f0_B == f1_A → gap 0
    rec.on_birth(0, A); rec.on_death(0, A)
    assert rec.on_birth(1, B) is None


def test_proxy_bloquea_muñon_sobre_cruce_completo():
    """A is a COMPLETE cross-arm; the causal proxy refuses to attach a later stub to it (it is not
    known to be a stub at birth). This is the cost of making the anti-over-merge gate causal."""
    rec = CausalReconnector(buffer_frames=140, corridor_px=250, valid_moves=VM)
    A = feat(0, 10, head=(0, 0), tail=(0, 0), entry="este", exit="oeste", is_cross=True)  # complete
    B = feat(15, 25, head=(1, 0), tail=(50, 0), entry="core", exit="oeste")
    rec.on_birth(0, A); rec.on_death(0, A)
    assert rec.on_birth(1, B) is None        # proxy: A's chain already has a complete one


def test_fuera_de_buffer_no_fusiona():
    rec = CausalReconnector(buffer_frames=140, corridor_px=250, valid_moves=VM)
    A = feat(0, 10, head=(0, 0), tail=(0, 0))
    B = feat(200, 210, head=(1, 0), tail=(50, 0))   # gap 190 > 140
    rec.on_birth(0, A); rec.on_death(0, A)
    assert rec.on_birth(1, B) is None


def test_corredor_excede_no_fusiona():
    rec = CausalReconnector(buffer_frames=140, corridor_px=250, valid_moves=VM)
    A = feat(0, 10, head=(0, 0), tail=(0, 0))
    B = feat(15, 25, head=(500, 0), tail=(600, 0))   # dist 500 > 250
    rec.on_birth(0, A); rec.on_death(0, A)
    assert rec.on_birth(1, B) is None


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"OK  {name}")
    print("todos los tests pasaron")

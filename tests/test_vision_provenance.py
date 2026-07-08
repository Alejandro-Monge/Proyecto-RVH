"""Piece 3 (vision emitter) — test of the provenance BRANCHES of causal_pos.

Provenance is NEW data: the byte-compare of the readout does NOT detect it (the
position does not change). Here each branch is EXERCISED and it is verified that
the label matches the branch taken (contract v3 Section 3.5, vision/sumo subset):
  - tshift on a node (w≈0)                     -> OBSERVED
  - interpolation between nodes, gap ≤ GAP_DT  -> INTERPOLATED
  - causal extrapolation over the path         -> EXTRAPOLATED
  - frozen without path (seg=None)             -> HELD

.venv/bin/python -m pytest tests/test_vision_provenance.py -v
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "scripts" / "demo_causal"))

import run_harness_causal as rhc  # noqa: E402

DT, GAP_DT = rhc.DT, rhc.GAP_DT


def _sonda():
    return rhc.SondaTotal({}, timeout_s=999.0)


def _veh(rt, draw, segs=(), t_exit_commit=None):
    return {"rt": np.asarray(rt, float), "draw": np.asarray(draw, float),
            "segs": list(segs), "t_exit_commit": t_exit_commit}


def _seg_recta():
    path = np.array([[0.0, 0.0], [10.0, 0.0]])
    arc = rhc._arc_of(path)
    return {"lo": 0, "hi": 2, "path": path, "arc": arc, "s": np.array([0.0, 10.0])}


def test_observed_tshift_sobre_nodo():
    v = _veh([0.0, DT], [[0.0, 0.0], [1.0, 1.0]])
    pos, prov = rhc.causal_pos(v, 0.0, _sonda())     # tshift == rt[0] -> w=0
    assert prov == "OBSERVED"
    assert np.allclose(pos, [0.0, 0.0])


def test_interpolated_entre_nodos():
    v = _veh([0.0, DT], [[0.0, 0.0], [1.0, 1.0]])
    pos, prov = rhc.causal_pos(v, DT / 2.0, _sonda())  # w=0.5, gap=DT ≤ GAP_DT
    assert prov == "INTERPOLATED"
    assert np.allclose(pos, [0.5, 0.5])


def test_extrapolated_sobre_path():
    s = _sonda()
    v = _veh([0.0, 5 * DT], [[0.0, 0.0], [1.0, 1.0]], segs=[_seg_recta()])
    # gap 5·DT > GAP_DT -> falls to causal extrapolation; seg present
    pos, prov = rhc.causal_pos(v, 0.05, s)
    assert prov == "EXTRAPOLATED"
    assert s.n_extrap == 1                            # counted the extrapolation
    assert pos is not None


def test_held_sin_path():
    v = _veh([0.0, 5 * DT], [[3.0, 4.0], [1.0, 1.0]], segs=[])  # no seg
    pos, prov = rhc.causal_pos(v, 0.05, _sonda())
    assert prov == "HELD"
    assert np.allclose(pos, [3.0, 4.0])               # freezes the raw draw[i]


def test_sin_posicion_devuelve_none_none():
    v = _veh([1.0, 1.0 + DT], [[0.0, 0.0], [1.0, 1.0]])
    pos, prov = rhc.causal_pos(v, 0.0, _sonda())       # tshift < rt[0] -> i<0
    assert pos is None and prov is None


def test_constantes_traci_literales_coinciden_con_traci():
    import os
    os.environ.setdefault("SUMO_HOME", "/usr/share/sumo")
    import traci.constants as C
    assert (rhc.VAR_POSITION, rhc.VAR_ANGLE, rhc.VAR_LANE_ID, rhc.VAR_ROAD_ID) == (
        C.VAR_POSITION, C.VAR_ANGLE, C.VAR_LANE_ID, C.VAR_ROAD_ID)

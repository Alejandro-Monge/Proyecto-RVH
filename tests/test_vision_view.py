"""Piece 6 (vision emitter) — `from_vision_tick` + graph BLOCKLIST.

The REAL REACHABILITY test (transitive closure
of gc.get_referents over the view) with sentinel baits in veh, wb, v["draw"], chains,
inst_t — plus the NEGATIVE CONTROL: a buggy view that retains veh MUST be exposed
(a probe that cannot fail is just for show).

.venv/bin/python -m pytest tests/test_vision_view.py -v
"""
from __future__ import annotations

import dataclasses
import gc
import inspect
import sys
from pathlib import Path
from types import MappingProxyType

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ws_emitter.view import (  # noqa: E402
    VAR_ANGLE,
    VAR_POSITION,
    BoxEntity,
    DecidedTickView,
    HandoffEntity,
    SumoEntity,
)


def _scenario():
    """SCALAR feeders + future-carrying structures baited behind them (as in a
    real run: veh/wb/chains/v["draw"]/inst_t exist, but the view must not reach them)."""
    sent = {n: object() for n in ("veh", "wb", "draw", "chains", "inst_t")}
    draw = [sent["draw"], [0.0, 0.0], [1.0, 1.0]]          # v["draw"] with bait
    veh = {"v10": {"t": [0.0, 0.033], "draw": draw, "cross_t": 1.0, "delay": 0.37},
           "__bait__": sent["veh"]}

    class _WB:                                             # bait of the WaitingBoxLayer
        pass
    wb = _WB()
    wb._bait = sent["wb"]
    chains = {"root10": sent["chains"]}                    # covers the {tid:root} path
    inst_t = {"v10": sent["inst_t"]}

    pt = draw[1]                                           # a position from the baited draw
    feeders = dict(
        seq=7, t_sim=12.5,
        sumo_subs={
            "v10": {VAR_POSITION: (float(pt[0]), float(pt[1])), VAR_ANGLE: 1.2},
            "v_sin_decidir": {VAR_POSITION: (9.0, 9.0), VAR_ANGLE: 0.0}},
        sumo_prov={"v10": "EXTRAPOLATED"},                # v_sin_decidir NOT decided
        sumo_vclass={"v10": "car"},
        sumo_generation={"v10": 0},
        boxes=[("v11", "OBSERVED", 5.0, 6.0, 0.5, "lane", 2, "bus"),
               ("v12", "HELD", 7.0, 8.0, 1.1, "raw_homography", None, "car")],
        handoffs=[("v13", 3.0, 4.0, 11.0, "truck")],
        occupancy_pixel={"sur": {1: 3, 2: 0}, "norte": {1: 1}},
        guard_report={"n_extrap": 9800, "max_interp_gap": 0.06, "mode": "assert"},
    )
    baits = {n: id(s) for n, s in sent.items()}
    structs = {"veh": veh, "wb": wb, "chains": chains, "inst_t": inst_t, "draw": draw}
    return feeders, baits, structs


def _reachable_ids(root) -> set:
    seen, stack = set(), [root]
    while stack:
        obj = stack.pop()
        if id(obj) in seen:
            continue
        seen.add(id(obj))
        stack.extend(gc.get_referents(obj))
    return seen


def _leaked(view, baits) -> set:
    reach = _reachable_ids(view)
    return {name for name, oid in baits.items() if oid in reach}


# --------------------------------------------------------------------------- #
# BLOCKLIST — positive (anti-leak guarantee)
def test_lista_negra_inalcanzable():
    feeders, baits, _ = _scenario()
    view = DecidedTickView.from_vision_tick(**feeders)
    leaked = _leaked(view, baits)
    assert leaked == set(), f"cebos ALCANZABLES desde el view: {leaked}"


# NEGATIVE CONTROL — the probe MUST fail when there is a leak (green-when-it-should-fail)
def test_control_negativo_detecta_fuga():
    feeders, baits, structs = _scenario()

    class _ViewBugueado:
        """buggy from_vision_tick: retains a feeder that references veh."""
        def __init__(self, **kw):
            self.__dict__.update(kw)
            self._fuga = structs["veh"]                   # <-- reference to veh (with bait)

    bugueado = _ViewBugueado(**feeders)
    leaked = _leaked(bugueado, baits)
    assert "veh" in leaked, "el detector NO vio la fuga: sonda de teatro"
    assert "draw" in leaked                               # draw lives inside veh
    # and the REAL view, with the SAME baits, leaks nothing:
    real = DecidedTickView.from_vision_tick(**feeders)
    assert _leaked(real, baits) == set()


def test_sin_parametro_con_futuro():
    # structural: the signature has no veh/chains/wb/draw/inst_t/tid_root
    params = set(inspect.signature(DecidedTickView.from_vision_tick).parameters)
    prohibidos = {"veh", "chains", "wb", "draw", "draws", "inst_t", "tid_root",
                  "commits", "store"}
    assert params & prohibidos == set()


# --------------------------------------------------------------------------- #
# Assembly of the THREE sources
def test_tres_fuentes():
    feeders, _, _ = _scenario()
    v = DecidedTickView.from_vision_tick(**feeders)
    assert [e.id for e in v.sumo_entities] == ["v10"]          # v_sin_decidir excluded
    assert [e.id for e in v.box_entities] == ["v11", "v12"]    # ordered by id
    assert [e.id for e in v.handoff_entities] == ["v13"]
    assert all(isinstance(e, SumoEntity) for e in v.sumo_entities)
    assert all(isinstance(e, BoxEntity) for e in v.box_entities)
    assert all(isinstance(e, HandoffEntity) for e in v.handoff_entities)


def test_sumo_entity_campos():
    feeders, _, structs = _scenario()
    pt = structs["draw"][1]
    v = DecidedTickView.from_vision_tick(**feeders)
    e = v.sumo_entities[0]
    assert e.source == "sumo" and e.provenance == "EXTRAPOLATED"
    assert e.pos_plane == (float(pt[0]), float(pt[1]))
    assert e.heading_raw == (1.2, "sumo_angle_post_step")
    assert e.vclass == "car" and e.generation == 0


def test_box_anchor_discriminado():
    feeders, _, _ = _scenario()
    v = DecidedTickView.from_vision_tick(**feeders)
    b = {e.id: e for e in v.box_entities}
    assert b["v11"].anchor == "lane" and b["v11"].carril == 2
    assert b["v12"].anchor == "raw_homography" and b["v12"].carril is None
    assert b["v11"].heading_raw == (0.5, "lane_angle")
    assert b["v11"].source == "vision_box" and b["v11"].vclass == "bus"


def test_handoff_claves_cerradas():
    feeders, _, _ = _scenario()
    v = DecidedTickView.from_vision_tick(**feeders)
    h = v.handoff_entities[0]
    assert h.id == "v13" and h.source == "handoff" and h.provenance == "HELD"
    assert h.last_pos == (3.0, 4.0, 11.0) and h.vclass == "truck"
    # CLOSED set: no pos_plane, no heading, no ETA/duration/progress
    campos = {f.name for f in dataclasses.fields(HandoffEntity)}
    assert campos == {"id", "source", "provenance", "last_pos", "vclass"}


def test_occupancy_pixel_zones_conteo_crudo():
    feeders, _, _ = _scenario()
    v = DecidedTickView.from_vision_tick(**feeders)
    occ = v.occupancy
    assert occ["kind"] == "pixel_zones"
    assert occ["semantics"] == "conteo_crudo"             # NOT a paper metric
    assert occ["per_zone"]["sur"]["1"] == 3 and occ["per_zone"]["sur"]["2"] == 0
    assert occ["per_zone"]["norte"]["1"] == 1
    assert isinstance(occ, MappingProxyType)


def test_inmutabilidad_y_determinismo():
    feeders, _, _ = _scenario()
    v1 = DecidedTickView.from_vision_tick(**feeders)
    v2 = DecidedTickView.from_vision_tick(**_scenario()[0])
    assert v1 == v2                                       # deterministic content
    with pytest.raises(dataclasses.FrozenInstanceError):
        v1.box_entities[0].anchor = "lane"
    with pytest.raises(TypeError):
        v1.occupancy["kind"] = "otro"
    with pytest.raises(TypeError):
        v1.occupancy["per_zone"]["sur"]["1"] = 99

"""Tests for the messages DecidedTickView (`ws_emitter/view.py`) — STEP 2.2.

Covers 7 tests, including the BLOCKLIST-by-REAL-reachability one (transitive
closure of gc.get_referents over the constructed view, with bait objects that
include st belonging to a FakeTracker — the path by which trackers[].states
could become reachable).

pytest tests/test_decided_tick_view.py from the root of implementacion/.
"""
from __future__ import annotations

import dataclasses
import gc
import sys
from pathlib import Path
from types import MappingProxyType

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ws_emitter.view import (  # noqa: E402
    VAR_ANGLE,
    VAR_POSITION,
    DecidedTickView,
    SumoEntity,
)


# --------------------------------------------------------------------------- #
# Baits: reproduce the REAL shape of the player state that must NOT be reached
class FakeStore:
    """Bait of the GuardedMessageStore (blocklist Section 5)."""
    def __init__(self):
        self.rows = [{"t": 99.9, "x": 1.0}]

    def guard_report(self):
        return {"mode": "assert", "accesses": 10, "violations": 0,
                "max_lookahead_s": 0.4}


class FakeTracker:
    """Bait of the VehicleTracker: its states[k] are THE SAME dicts that the player
    stores in last_applied — the retention path the test must cover."""
    def __init__(self):
        self.states = [
            {"sx": 10.0, "sy": 20.0, "angle": 90.0, "provenance": "OBSERVED",
             "held_frame": False, "sim_time_csv": 3.5, "speed_csv": 4.2},
            {"sx": 11.0, "sy": 21.0, "angle": 91.0, "provenance": "INTERPOLATED",
             "held_frame": True, "sim_time_csv": None, "speed_csv": 0.0},
        ]


def _feeders():
    """Feeders like those at the coupling point, with real baits behind them."""
    store = FakeStore()
    trk_a, trk_b, trk_c = FakeTracker(), FakeTracker(), FakeTracker()
    veh_bait = {"veh_a": {"t": [1, 2, 3], "cross_t": 1.0, "delay": 0.37}}
    chains_bait, commits_bait = {"c": object()}, [object()]
    subs = {
        "veh_a": {0x50: "E1", 0x51: "E1_0", 0x40: 3.0,
                  VAR_POSITION: (57.0, 74.4), VAR_ANGLE: 90.0},
        "veh_b": {0x50: "E2", 0x51: "E2_1", 0x40: 0.0,
                  VAR_POSITION: (12.5, -3.25), VAR_ANGLE: 180.0},
        "veh_parked": {VAR_POSITION: (1.0, 1.0), VAR_ANGLE: 0.0},
        "veh_staging": {VAR_POSITION: (2.0, 2.0), VAR_ANGLE: 0.0},
    }
    kw = dict(
        seq=1234, t_sim=123.4,
        active={"veh_a", "veh_b", "veh_parked", "veh_staging"},
        parked={"veh_parked"},
        last_applied={"veh_a": trk_a.states[0], "veh_b": trk_b.states[1],
                      "veh_staging": None, "veh_parked": trk_c.states[0]},
        applied_ids={"veh_a"},
        subs=subs,
        generation={"veh_a": 0, "veh_b": 2},
        assigned_type={"veh_a": "shadow_car", "veh_b": "shadow_bus"},
        occupancy_counts={("norte", 1): {"ocupacion": 2, "detenidos": 1},
                          ("norte", 2): {"ocupacion": 0, "detenidos": 0},
                          ("sur", 1): {"ocupacion": 3, "detenidos": 3}},
        guard_report=store.guard_report(),
    )
    baits = {"store": store, "trackers": (trk_a, trk_b, trk_c),
             "veh": veh_bait, "chains": chains_bait, "commits": commits_bait,
             "subs": subs, "last_applied": kw["last_applied"]}
    return kw, baits


# --------------------------------------------------------------------------- #
# 1. Ghost exclusion (same rule as occupancy)
def test_exclusion_fantasmas():
    kw, _ = _feeders()
    view = DecidedTickView.from_messages_tick(**kw)
    ids = [e.id for e in view.sumo_entities]
    assert ids == ["veh_a", "veh_b"]          # ordered, no parked or staging
    assert "veh_parked" not in ids            # transparent parked: ghost
    assert "veh_staging" not in ids           # no applied state: staging


# --------------------------------------------------------------------------- #
# 2. BLOCKLIST by REAL reachability
def _reachable_ids(root) -> set:
    """Transitive closure of gc.get_referents from root (object ids)."""
    seen, stack = set(), [root]
    while stack:
        obj = stack.pop()
        if id(obj) in seen:
            continue
        seen.add(id(obj))
        stack.extend(gc.get_referents(obj))
    return seen


def test_lista_negra_inalcanzable():
    kw, baits = _feeders()
    view = DecidedTickView.from_messages_tick(**kw)
    reach = _reachable_ids(view)
    forbidden = {}
    forbidden["store"] = id(baits["store"])
    forbidden["veh(dict con futuro)"] = id(baits["veh"])
    forbidden["chains"] = id(baits["chains"])
    forbidden["commits"] = id(baits["commits"])
    forbidden["subs(caché TraCI)"] = id(baits["subs"])
    forbidden["last_applied(dict del player)"] = id(baits["last_applied"])
    for k, trk in enumerate(baits["trackers"]):
        forbidden[f"tracker{k}"] = id(trk)
        forbidden[f"tracker{k}.states"] = id(trk.states)
        for j, st in enumerate(trk.states):
            forbidden[f"tracker{k}.states[{j}]"] = id(st)   # the st from last_applied
    leaked = {name for name, oid in forbidden.items() if oid in reach}
    assert not leaked, f"objetos de la lista negra ALCANZABLES desde el view: {leaked}"


# --------------------------------------------------------------------------- #
# 3. Immutability
def test_inmutabilidad():
    kw, _ = _feeders()
    view = DecidedTickView.from_messages_tick(**kw)
    with pytest.raises(dataclasses.FrozenInstanceError):
        view.seq = 9
    with pytest.raises(dataclasses.FrozenInstanceError):
        view.sumo_entities[0].vclass = "truck"
    with pytest.raises(TypeError):
        view.occupancy["kind"] = "otro"
    with pytest.raises(TypeError):
        view.occupancy["per_lane"]["norte"]["1"]["ocupacion"] = 99
    with pytest.raises(TypeError):
        view.guard_counters["violations"] = 1
    assert isinstance(view.occupancy, MappingProxyType)


# --------------------------------------------------------------------------- #
# 4. Provenance mapping (HELD by both paths + native)
def test_provenance():
    kw, _ = _feeders()
    view = DecidedTickView.from_messages_tick(**kw)
    by_id = {e.id: e for e in view.sumo_entities}
    assert by_id["veh_a"].provenance == "OBSERVED"     # applied, no held_frame
    assert by_id["veh_b"].provenance == "HELD"         # not applied this tick
    # held_frame=True forces HELD even if it is in applied_ids
    kw2, _ = _feeders()
    kw2["applied_ids"] = {"veh_a", "veh_b"}
    view2 = DecidedTickView.from_messages_tick(**kw2)
    assert {e.id: e.provenance for e in view2.sumo_entities}["veh_b"] == "HELD"
    # native ones are carried over as-is (REACQUIRED_SNAP etc.)
    kw3, _ = _feeders()
    kw3["last_applied"]["veh_a"] = dict(kw3["last_applied"]["veh_a"],
                                        provenance="REACQUIRED_SNAP")
    view3 = DecidedTickView.from_messages_tick(**kw3)
    assert {e.id: e.provenance for e in view3.sumo_entities}["veh_a"] == "REACQUIRED_SNAP"


# --------------------------------------------------------------------------- #
# 5. Determinism and content
def test_determinismo_y_campos():
    kw, _ = _feeders()
    v1 = DecidedTickView.from_messages_tick(**kw)
    kw2, _ = _feeders()
    v2 = DecidedTickView.from_messages_tick(**kw2)
    assert v1 == v2                                   # dataclass eq: same content
    a = {e.id: e for e in v1.sumo_entities}["veh_a"]
    assert a.pos_plane == (57.0, 74.4)                # VAR_POSITION post-step
    assert a.heading_raw == (90.0, "sumo_angle_post_step")
    assert a.vclass == "car" and a.generation == 0
    b = {e.id: e for e in v1.sumo_entities}["veh_b"]
    assert b.vclass == "bus" and b.generation == 2
    assert v1.seq == 1234 and v1.t_sim == 123.4


# --------------------------------------------------------------------------- #
# 6. Uniform interface across shadows
def test_interfaz_uniforme():
    kw, _ = _feeders()
    view = DecidedTickView.from_messages_tick(**kw)
    assert view.box_entities == ()          # present and empty IN MESSAGES
    assert view.handoff_entities == ()
    # from_vision_tick is already implemented (block C); its coverage lives in
    # tests/test_vision_view.py. Here we only assert that MESSAGES does not use it.


# --------------------------------------------------------------------------- #
# 7. Occupancy shape (discriminated union from contract Section 3.4)
def test_occupancy_forma():
    kw, _ = _feeders()
    view = DecidedTickView.from_messages_tick(**kw)
    occ = view.occupancy
    assert occ["kind"] == "sumo_lanes"
    assert occ["per_lane"]["norte"]["1"] == {"ocupacion": 2, "detenidos": 1}
    assert occ["per_lane"]["norte"]["2"] == {"ocupacion": 0, "detenidos": 0}
    assert occ["per_lane"]["sur"]["1"]["detenidos"] == 3
    assert view.guard_counters["violations"] == 0

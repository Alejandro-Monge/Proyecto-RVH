"""Tests for the approach/lane ↔ edge/lane lookup (`digital_shadow/sumo/lane_map.py`).

Two levels, both deterministic:

  * **PURE contract** over a synthetic mapping (no I/O): `target`/`lane_id`,
    entry/exit separation, clear errors on an invalid request, idempotency.
  * **Smoke against the real config** (`lane_map.yaml` derived from the net): the
    sur→oeste anchor / sur→este counter-anchor and the lane-1 rule, to catch a
    re-derivation that breaks the mapping.

Runnable with pytest or as a script (`python -m tests.test_sumo_lane_map`).
"""

from __future__ import annotations

from types import MappingProxyType

from digital_shadow.sumo.lane_map import (
    ArmEdges,
    SumoLaneMap,
    load_lane_map,
    sumo_target,
)


def _synthetic_map() -> SumoLaneMap:
    """Minimal mapping of one approach, built by hand (does not depend on the net)."""
    arms = {
        "sur": ArmEdges(
            entry_edge="E5",
            entry_lanes=MappingProxyType({1: 0, 2: 1, 3: 2}),
            exit_edge="-E5",
            exit_lanes=MappingProxyType({1: 0}),
        )
    }
    return SumoLaneMap("test", "J5", MappingProxyType(arms))


# --------------------------------------------------------------------------- #
# Pure contract
# --------------------------------------------------------------------------- #
def test_target_carril_uno_es_indice_cero():
    """carril_GT 1 (right) -> index 0; the lane−1 rule in both frames."""
    m = _synthetic_map()
    assert m.target("sur", "entry", 1) == ("E5", 0)
    assert m.target("sur", "entry", 3) == ("E5", 2)
    assert m.target("sur", "exit", 1) == ("-E5", 0)


def test_entry_y_exit_usan_edges_distintos():
    """Opposite frames: entry -> E_n (approach), exit -> -E_n (departure)."""
    m = _synthetic_map()
    assert m.target("sur", "entry", 1)[0] == "E5"
    assert m.target("sur", "exit", 1)[0] == "-E5"


def test_lane_id_formato():
    m = _synthetic_map()
    assert m.lane_id("sur", "entry", 3) == "E5_2"
    assert m.lane_id("sur", "exit", 1) == "-E5_0"


def test_carril_de_egreso_inexistente_es_error():
    """Requesting exit lane 2 of the south approach (only 1 lane) is invalid, not a default."""
    m = _synthetic_map()
    try:
        m.target("sur", "exit", 2)
    except KeyError:
        pass
    else:
        raise AssertionError("se esperaba KeyError por carril de egreso inexistente")


def test_marco_y_brazo_invalidos():
    m = _synthetic_map()
    for fn in (lambda: m.target("sur", "approach", 1), lambda: m.target("sur", "in", 1)):
        try:
            fn()
        except ValueError:
            pass
        else:
            raise AssertionError("se esperaba ValueError por marco inválido")
    try:
        m.target("oeste", "entry", 1)
    except KeyError:
        pass
    else:
        raise AssertionError("se esperaba KeyError por brazo inexistente")


def test_lookup_es_puro_e_idempotente():
    """Same result regardless of order or how many times it is called (stateless)."""
    m = _synthetic_map()
    a1 = sumo_target(m, "sur", "entry", 2)
    _ = sumo_target(m, "sur", "exit", 1)
    a2 = sumo_target(m, "sur", "entry", 2)
    assert a1 == a2 == ("E5", 1)


# --------------------------------------------------------------------------- #
# Smoke against the real config (mapping derived from the net)
# --------------------------------------------------------------------------- #
def test_config_real_ancla_sur_oeste():
    """Anchor: sur→oeste (left turn) enters via E5 lane 3 and exits via -E1 lane 2;
    counter-anchor: sur→este (right turn) enters via E5 lane 1."""
    m = load_lane_map("rusia")
    assert m.target("sur", "entry", 3) == ("E5", 2)
    assert m.target("oeste", "exit", 2) == ("-E1", 1)
    assert m.target("sur", "entry", 1) == ("E5", 0)
    # North/south exits have 1 lane; east/west have 2.
    assert m.target("norte", "exit", 1) == ("-E6", 0)
    assert m.target("este", "exit", 2) == ("-E3", 1)


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"OK  {name}")
    print("todos los tests pasaron")

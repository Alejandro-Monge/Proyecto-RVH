"""Tests of the position piece (`digital_shadow/sumo/position.py`).

Pure contract (transform) + causal filter + positioner, deterministic. Smoke against the real config
(homography + lane_map). pytest or `python -m tests.test_position`.
"""

from __future__ import annotations

import numpy as np

from digital_shadow.sumo.lane_map import ApproachAxis, load_lane_map
from digital_shadow.sumo.position import (
    AlphaBetaFilter,
    VehiclePositioner,
    longitudinal_pos,
    load_homography,
    project_to_world,
    resolve_lane,
)

AXIS = ApproachAxis(origin=(0.0, 0.0), direction=(1.0, 0.0), length=10.0)


# --------------------------------------------------------------------------- #
# Pure transform
# --------------------------------------------------------------------------- #
def test_longitudinal_descarta_lateral_y_clipa():
    """pos = projection onto dir; the lateral component is ignored; clip to [0, length]."""
    assert longitudinal_pos(AXIS, (5.0, 3.0)) == 5.0      # lateral 3 ignored
    assert longitudinal_pos(AXIS, (12.0, 0.0)) == 10.0    # clip above
    assert longitudinal_pos(AXIS, (-2.0, 1.0)) == 0.0     # clip below


def test_project_to_world_identidad():
    """With H = identity, projecting returns the same point (pure, deterministic)."""
    H = np.eye(3)
    assert project_to_world(H, (4.0, 7.0)) == (4.0, 7.0)
    assert project_to_world(H, (4.0, 7.0)) == project_to_world(H, (4.0, 7.0))


# --------------------------------------------------------------------------- #
# Alpha-beta filter
# --------------------------------------------------------------------------- #
def test_beta_critico_amortiguado():
    f = AlphaBetaFilter(0.4)
    assert abs(f.beta - 0.4 ** 2 / (2 - 0.4)) < 1e-12


def test_filtro_sigue_rampa_sin_lag():
    """At constant velocity the alpha-beta does NOT bias (lag≈0 at steady state)."""
    f = AlphaBetaFilter(0.4)
    dt, v = 0.0338, 4.0
    out, true = [], []
    for i in range(120):
        t = i * dt
        f.observe(t, v * t)
        out.append(f.value())
        true.append(v * t)
    lag = np.median(np.array(out[20:]) - np.array(true[20:]))
    assert abs(lag) < 0.05, f"lag inesperado: {lag}"


def test_filtro_reduce_ruido():
    rng = np.random.default_rng(3)
    f = AlphaBetaFilter(0.4)
    dt, v = 0.0338, 4.0
    out, true = [], []
    for i in range(150):
        t = i * dt
        f.observe(t, v * t + rng.normal(0, 0.15))
        out.append(f.value())
        true.append(v * t)
    res = np.array(out[30:]) - np.array(true[30:])
    assert np.std(res) < 0.15, "el filtro no redujo el ruido"


def test_value_antes_de_observe_falla():
    try:
        AlphaBetaFilter(0.4).value()
    except ValueError:
        return
    raise AssertionError("value() antes de observe() debió fallar")


# --------------------------------------------------------------------------- #
# resolve_lane + positioner (real config)
# --------------------------------------------------------------------------- #
def test_resolve_lane_modos():
    lm = load_lane_map("rusia")
    assert resolve_lane(lm, "sur", "3") == ("E5_2", "exact")
    assert resolve_lane(lm, "oeste", "2-3") == ("E1_1", "grouped")  # representative = lane 2
    assert resolve_lane(lm, "sur", "") == ("E5_0", "fallback")      # arm_only -> lane 1


def test_positioner_lateral_se_compromete_en_el_cruce():
    """The longitudinal runs from birth; the laneID is None until commit_lane."""
    lm = load_lane_map("rusia")
    H = load_homography("rusia")
    vp = VehiclePositioner("norte", lm, H, alpha=0.4)
    for i in range(5):
        vp.observe(i * 0.0338, (800.0 - i, 400.0 + i))
    p = vp.placement()
    assert p.lane_id is None and isinstance(p.pos, float)   # pos available, lane not yet
    vp.commit_lane("1")
    p = vp.placement()
    assert p.lane_id == "E6_0" and p.lane_mode == "exact"


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"OK  {name}")
    print("todos los tests pasaron")

"""Tests of the gap-filling interpolation (`digital_shadow/sumo/stage.py`), PURE (no SUMO).

The `SumoStage` (traci) is verified with `scripts/smoke_sumo_stage.py` against a live SUMO; here only
the pure geometry. pytest or `python -m tests.test_stage`.
"""

from __future__ import annotations

from digital_shadow.sumo.stage import GapFiller, point_at_arclen, polyline_arclen

# An "L": 3 m east, then 4 m north (total length 7 m).
SHAPE = [(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)]


def test_arclen_acumulado():
    L = polyline_arclen(SHAPE)
    assert list(L) == [0.0, 3.0, 7.0]


def test_point_at_arclen_y_clip():
    assert point_at_arclen(SHAPE, 0.0) == (0.0, 0.0)
    assert point_at_arclen(SHAPE, 1.5) == (1.5, 0.0)       # on the 1st segment
    assert point_at_arclen(SHAPE, 5.0) == (3.0, 2.0)       # 2 m into the 2nd segment
    assert point_at_arclen(SHAPE, 99.0) == (3.0, 4.0)      # clip above
    assert point_at_arclen(SHAPE, -5.0) == (0.0, 0.0)      # clip below


def test_gapfiller_avanza_a_velocidad():
    gf = GapFiller(SHAPE, speed=2.0, t0=10.0)              # 2 m/s from t0=10
    assert gf.at(10.0) == (0.0, 0.0)                       # arclen 0
    assert gf.at(11.0) == (2.0, 0.0)                       # 2 m -> on the 1st segment
    assert gf.at(12.0) == (3.0, 1.0)                       # 4 m -> 1 m into the 2nd segment
    assert abs(gf.length - 7.0) < 1e-9


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"OK  {name}")
    print("todos los tests pasaron")

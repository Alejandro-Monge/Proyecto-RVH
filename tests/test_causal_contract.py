"""Tests of the causal contract (`digital_shadow/causal/contract.py`).

They verify that the checkers (a) ACCEPT pieces that comply and (b) CATCH violations —if they
caught nothing, they would not be a contract. Deterministic. pytest or `python -m tests.test_causal_contract`.
"""

from __future__ import annotations

from digital_shadow.causal import (
    check_backward_only,
    check_deferred_commit,
    check_pure_lookup,
)
from digital_shadow.sumo.lane_map import load_lane_map
from digital_shadow.sumo.position import AlphaBetaFilter
from digital_shadow.sumo.route import RouteCommitter, load_lane_turn_rule, load_turns


# --------------------------------------------------------------------------- #
# check_pure_lookup
# --------------------------------------------------------------------------- #
def test_pure_lookup_acepta_funcion_pura():
    check_pure_lookup(lambda a, b: a + b, [(1, 2), (3, 4)], label="suma")


def test_pure_lookup_atrapa_estado_oculto():
    """A stateful function (counter) depends on order -> must fail."""
    state = {"n": 0}

    def impure(x):
        state["n"] += 1
        return x + state["n"]

    try:
        check_pure_lookup(impure, [(10,), (20,), (30,)], label="impura")
    except AssertionError:
        return
    raise AssertionError("check_pure_lookup NO atrapó la impureza")


def test_mapeo_real_es_lookup_puro():
    """The approach↔edge mapping (1st instance of the contract) satisfies clause (i)."""
    lm = load_lane_map("rusia")
    check_pure_lookup(lm.target,
                      [("sur", "entry", 3), ("oeste", "exit", 2), ("este", "entry", 1)],
                      label="lane_map.target")


# --------------------------------------------------------------------------- #
# check_backward_only
# --------------------------------------------------------------------------- #
def _stream():
    return [(i * 0.0338, 10.0 + 4.0 * i * 0.0338 + (0.1 if i % 2 else -0.1)) for i in range(30)]


def test_backward_only_acepta_filtro_causal():
    check_backward_only(lambda: AlphaBetaFilter(0.4), _stream(), label="alpha-beta")


class _LeakyEstimator:
    """NON-causal estimator: shares GLOBAL state across instances -> the prefix does not reproduce the
    trace (a new instance 'sees' what another processed). Must be caught."""

    _global_calls = 0

    def __init__(self):
        self._x = 0.0

    def observe(self, t, z):
        _LeakyEstimator._global_calls += 1
        self._x = z + _LeakyEstimator._global_calls  # depends on calls from OTHER instances

    def value(self):
        return self._x


def test_backward_only_atrapa_estado_global():
    try:
        check_backward_only(_LeakyEstimator, _stream(), label="leaky")
    except AssertionError:
        return
    raise AssertionError("check_backward_only NO atrapó el estado global no causal")


# --------------------------------------------------------------------------- #
# check_deferred_commit (extended clause ii: sealed deferred commit)
# --------------------------------------------------------------------------- #
def test_deferred_commit_acepta_route_committer():
    """The route piece (2nd instance with deferred commit) is append-only."""
    lm, turns, rule = load_lane_map("rusia"), load_turns("rusia"), load_lane_turn_rule("rusia")
    actions = [
        lambda c: c.commit_origin(10.0, "2"),
        lambda c: c.commit_destination_predicted(10.2, "2"),
        lambda c: c.commit_destination_observed(15.0, "este"),
    ]
    check_deferred_commit(lambda: RouteCommitter("norte", lm, turns, rule), actions, label="ruta")


class _RewritingCommitter:
    """INVALID committer: it REWRITES commit 0 instead of appending. Must be caught."""

    def __init__(self):
        self._c = []

    def commit(self, x):
        if self._c:
            self._c[0] = x          # rewrites a sealed commit -> violation
        else:
            self._c.append(x)

    def commits(self):
        return tuple(self._c)


def test_deferred_commit_atrapa_reescritura():
    actions = [lambda o: o.commit(1), lambda o: o.commit(2)]
    try:
        check_deferred_commit(_RewritingCommitter, actions, label="rewriter")
    except AssertionError:
        return
    raise AssertionError("check_deferred_commit NO atrapó la reescritura de un commit sellado")


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"OK  {name}")
    print("todos los tests pasaron")

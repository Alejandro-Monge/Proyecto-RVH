"""Causal contract — invariants + testable verifiers.

The final shadow is **realtime causal**: at instant t only what happened up to t is known. The
fidelity chain ran DEFERRED (correct for measuring Section 7.1/Section 7.5), but on entering SUMO the
DELAY (Section 7.2) and the continuity (Section 7.4) require reproducing the causal flow. So that the
pieces do not stay batch (to be redone when causalizing), each one must satisfy this contract.

THIN and CO-EVOLVED (do not freeze): it will be EXTENDED with the route piece. Two clauses:

  (i)  **pure lookup** — pure function, stateless, without looking at the future, without requiring
       the complete event/track (entry and exit resolvable separately).
  (ii) **causal commit with backward-only state** — if the piece accumulates state (filters,
       buffers), that state only integrates the PAST; the commit at t depends only on observations ≤ t.

The "when" (commit instant) lives OUTSIDE the piece (in the causal harness), not inside.

Instances that validate it: the arm↔edge mapping (`digital_shadow/sumo/lane_map.py`) satisfies (i);
the position piece (`digital_shadow/sumo/position.py`) satisfies (i) in its transform and (ii) in its
longitudinal filter. Those are the first two instances against which the contract is tested.

Honest note on the SCOPE of the verifiers: the "no future" guarantee comes mostly from the SHAPE of
the interface (`CausalEstimator.observe` receives ONE observation at a time → it is physically
impossible to pass it the future). The verifiers check what is MECHANICALLY checkable:
determinism/idempotence, order independence (lookups), and prefix consistency (later observations do
not rewrite past commits). They do not "prove" causality on their own; they support it together with
the interface.
"""

from __future__ import annotations

import random
from typing import Callable, Protocol, Sequence, runtime_checkable

# Named invariants (the checklist). The pieces are audited against these.
INVARIANTS = {
    "I1_pure_lookup": "Función pura: mismo input → mismo output; sin estado; determinista.",
    "I2_no_future": "Ninguna decisión usa observaciones futuras; el commit en t depende solo de ≤ t.",
    "I3_no_whole_event": "No exige el evento/track completo; entry y exit se resuelven por separado.",
    "I4_backward_state": "Si hay estado (filtro/buffer), solo integra el pasado (backward-only).",
    "I5_when_external": "El 'cuándo' (instante de commit) vive fuera de la pieza, en el harness.",
}


@runtime_checkable
class CausalEstimator(Protocol):
    """Interface of a causal estimator with backward-only state.

    The SHAPE prevents looking at the future: `observe` receives one observation at a time
    (non-decreasing time) and `value` reads the current estimate (a function only of the observed
    past). There is no method that receives the complete sequence nor the future.
    """

    def observe(self, t: float, z: float) -> None:
        """Integrate the observation z at instant t (t non-decreasing)."""
        ...

    def value(self) -> float:
        """Current estimate (a function only of what has been observed so far)."""
        ...


def check_pure_lookup(
    fn: Callable, inputs: Sequence[tuple], *, label: str = "lookup"
) -> dict:
    """Verify I1/I3 over a pure lookup: determinism, idempotence, and order independence.

    `fn(*input)` is called with each tuple of `inputs`. Fails (AssertionError) if:
      - repeating the same call gives a different result (non-deterministic/idempotent), or
      - calling in another order changes the per-input result (there is hidden state).
    """
    base = [fn(*x) for x in inputs]
    again = [fn(*x) for x in inputs]
    assert base == again, f"{label}: NO idempotente (mismo input → distinto output)"
    order = list(range(len(inputs)))
    random.Random(12345).shuffle(order)
    shuffled = {i: fn(*inputs[i]) for i in order}
    for i in range(len(inputs)):
        assert shuffled[i] == base[i], (
            f"{label}: depende del ORDEN (estado oculto) en input {i}: "
            f"{shuffled[i]!r} != {base[i]!r}"
        )
    return {"label": label, "n": len(inputs), "I1_pure_lookup": True, "I3_no_whole_event": True}


def check_backward_only(
    factory: Callable[[], CausalEstimator],
    stream: Sequence[tuple[float, float]],
    *,
    label: str = "estimator",
) -> dict:
    """Verify I2/I4 over a causal estimator: PREFIX consistency.

    `factory()` creates a fresh instance. The whole `stream` (list of (t, z)) is fed, recording
    `value()` after each `observe` (the causal trace). Then, for each k, a NEW instance fed only with
    `stream[:k+1]` must give the same `value()` as the trace at k. If a later observation were to
    rewrite a past commit (look-ahead, non-causal state, or shared global state), it would fail.
    """
    est = factory()
    trace = []
    for t, z in stream:
        est.observe(t, z)
        trace.append(est.value())
    for k in range(len(stream)):
        e = factory()
        for t, z in stream[: k + 1]:
            e.observe(t, z)
        v = e.value()
        assert v == trace[k] or abs(v - trace[k]) <= 1e-12, (
            f"{label}: el prefijo[:{k+1}] no reproduce la traza en {k} "
            f"({v!r} != {trace[k]!r}) → no es backward-only"
        )
    return {"label": label, "n": len(stream), "I2_no_future": True, "I4_backward_state": True}


def check_deferred_commit(
    factory: Callable, actions: Sequence[Callable], *, snapshot=lambda o: o.commits(),
    label: str = "committer",
) -> dict:
    """Verify the EXTENDED clause (ii) for DEFERRED COMMIT pieces (e.g. the route: destination
    predicted on crossing, observed on exit = two sealed commits).

    `factory()` creates the object; each `action(obj)` triggers one or more commits (in temporal
    order); `snapshot(obj)` reads the sealed list of commits. Invariant: the list is **append-only**
    — a sealed commit is NEVER rewritten by a later action. After each action, the previous snapshot
    must be a PREFIX of the new one (what arrives later is a NEW commit, not an overwrite). Fails if a
    later action mutates or shortens an already-sealed commit.
    """
    obj = factory()
    prev = tuple(snapshot(obj))
    for i, action in enumerate(actions):
        action(obj)
        cur = tuple(snapshot(obj))
        assert cur[: len(prev)] == prev, (
            f"{label}: la acción {i} REESCRIBIÓ un commit sellado "
            f"(prefijo {prev!r} != inicio de {cur!r}) → no es append-only/diferido válido"
        )
        assert len(cur) >= len(prev), f"{label}: la acción {i} ACORTÓ los commits sellados"
        prev = cur
    return {"label": label, "n_actions": len(actions), "I2_no_future": True,
            "I4_backward_state": True, "deferred_append_only": True}

# Module — Causal contract of the SUMO block (`digital_shadow.causal`)

Defines and verifies the contract that each piece of the SUMO block satisfies in order to operate in
real time causally. The code lives in `digital_shadow/causal/contract.py`; the tests, in
`tests/test_causal_contract.py`.

## Motivation

The final shadow is causal in real time: at instant *t* only what happened up to *t* is known.
The fidelity chain ran offline, which is correct for measuring the fidelity metrics 7.1 and 7.5,
because fidelity is a property of *what* came out, not of *when*. On entering SUMO, delay (7.2) and
continuity (7.4) require reproducing the causal flow. So that the pieces do not remain in batch mode
(to be redone when causalized), each one satisfies this contract. It is a named checklist, not implicit
discipline: it can be audited and tested.

## Invariants

`INVARIANTS` (in the module) lists five, grouped into two clauses:

- **(i) Pure lookup:** `I1` pure function (same input produces the same output, no state,
  deterministic); `I3` does not require the full event or track (entry and exit resolvable
  separately).
- **(ii) Causal commit with backward-only state:** `I2` no decision uses the future (the commit
  at *t* depends only on observations ≤ *t*); `I4` if there is state (filter or buffer), it only
  integrates the past.
- **Cross-cutting:** `I5` the commit instant lives **outside** the piece, in the harness.

## Testable verifiers

- **`check_pure_lookup(fn, inputs)`** — clause (i). Fails if `fn` is not idempotent (same input
  produces a different output) or if it depends on the **order** of the calls (hidden state). It tests
  this by calling in normal and shuffled order and comparing per input.
- **`check_backward_only(factory, stream)`** — clause (ii). Feeds the whole `(t, z)` `stream`,
  recording `value()` after each `observe` (the causal trace); then, for each *k*, a **new**
  instance fed only with `stream[:k+1]` must reproduce the trace at *k*. If a later observation
  rewrote a past commit (look-ahead, non-causal global state), it fails.

**Interface `CausalEstimator`** (Protocol): `observe(t, z)` (one observation at a time, *t*
non-decreasing) plus `value()` (current estimate). The **shape** already prevents looking at the
future: there is no method that receives the full sequence.

## Scope of the verifiers

The "no future" guarantee comes above all from the shape of the interface: `observe` receives one
observation at a time, so it is physically impossible to pass it the future. The verifiers check what
is mechanically checkable: determinism and idempotency, order independence (lookups), prefix
consistency (later data does not rewrite past data). They do not prove causality on their own; they
support it together with the interface. The tests include **violation cases** (a function with state;
an estimator with global state) and verify that the verifiers **catch** them: if they caught nothing,
they would not be a contract.

## Validated instances

| Instance | Clause | Verified by |
|---|---|---|
| Approach↔edge mapping (`sumo/lane_map.py`, `target`) | (i) | `check_pure_lookup` |
| Position piece — transform (`sumo/position.py`, `project_to_world`/`raw_pos`) | (i) | `check_pure_lookup` |
| Position piece — filter (`AlphaBetaFilter`) | (ii) | `check_backward_only` |

The mapping (first instance, a trivial static lookup) exercises only the easy clause; the position
piece (second instance) adds the backward-only state (the filter), which the mapping did not exhibit.
That is why the contract is co-evolved, not frozen from a single instance.

## Deferred commit (route piece)

The route piece (third instance) extended clause (ii) with deferred commit:

- **`check_deferred_commit(factory, actions)`** — verifies that a piece emitting commits over time is
  **append-only**: a sealed commit is never rewritten by a later action. After each action, the
  previous snapshot must be a **prefix** of the new one (what arrives later is a new commit, not an
  overwrite). It catches rewrites (test with a committer that rewrites commit 0).
- **Instance:** `RouteCommitter` (`sumo/route.py`): the predicted destination (at crossing) and the
  observed destination (at exit) are **two distinct sealed commits**, not a mutation. The commit at
  *t* does not change with evidence after *t*. It models "predicted with low delay, refined by the
  observed one with high delay" without rewriting the past, which enables the delay-vs-accuracy curve
  of metric 8.2.

The three contract instances so far: approach↔edge mapping (i), position piece (i plus the clause-ii
filter), route piece (i plus the clause-ii deferred commit). The contract keeps co-evolving if a
future piece exhibits a new causal shape.

## References

- `docs/modules/route.md`: route piece (`RouteCommitter`), the deferred-commit instance.
- `docs/modules/position.md`: position piece and its backward-only filter.
- `docs/modules/sumo_lane_map.md`: approach↔edge mapping.
- Module and tests: `digital_shadow/causal/contract.py`, `tests/test_causal_contract.py`.

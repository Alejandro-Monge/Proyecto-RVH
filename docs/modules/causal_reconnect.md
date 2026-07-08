# Module — Causal-greedy reconnection + causal perception (`digital_shadow.tracking.causal_reconnect`)

Runs the perception chain under a causal constraint (past information only). The detector, the
tracker, the position, and the route are already causal; what remained was to causalize the
**reconnection** (which in batch is global) and the **event commit** (which in batch uses the whole
track). The design principle is not to force the causal version to imitate the batch one, but to
measure the gap between online and offline reliably.

## Files

```
digital_shadow/tracking/causal_reconnect.py   CausalReconnector (greedy-online reconnection)
scripts/measure_causal_reconnection.py         measures the online-vs-offline gap
scripts/validate_causal_perception.py          validates the causal event stream
tests/test_causal_reconnect.py                 tests of the causal reconnector
```

## Causal-greedy reconnection

The batch version (`reconnect_fragments.py`) matches **globally** (all candidates by cost,
union-find, complete information). In real time, when a track B is **born** there are only **past**
candidates (dead tails within the buffer) → a greedy-online decision at birth, without the future.
`CausalReconnector` is a discrete-event simulation: `on_death` (enters the pool + updates the chain's
cross) and `on_birth` (looks for the best past tail and merges). Same conditions (gap∈(0,buffer],
corridor, class, continuation ≤60°, valid route). The **anti-over-merge criterion** is re-expressed as
a **causal proxy**: it refuses to merge B onto A's chain if A's chain already contains a complete
cross-arm fragment (A's members are dead → their `is_cross` is known; B's is not).

## The online-vs-offline gap (the result)

Batch 15 merges · causal **10/15 exact, 5 missed, 1 false merge**. The causes are distinct and both
reliable:

- **5 missed = causality of the anti-over-merge criterion (proxy).** The 5 join a short fragment to an
  already complete crossing (tid 42/89/105/170/189 are complete cross-arm). The batch allows it (1
  complete < 2); the causal proxy refuses it because at the birth of the short fragment one cannot know
  it is a short fragment (its `is_cross` is future) → conservative to avoid ≥2 complete ones. It is not
  future-candidate theft.
- **1 false merge `[109,122]` = future-evidence gap.** The tail of tid109 belongs **globally** to a
  later match (which the batch sees, leaving 109 standalone); the causal one, without the future,
  greedily gives it to tid122. It is "deciding badly for lack of the future".

## The event stream is identical

Applying the causal `id_map` → events → matching: **69 matched, identical to the batch; 0 different
routes.** The reconnection gap is **only trail/continuity, not detection or route**. Real time equals
the offline result on the route metric. Why: the 5 missed are short fragments of the crossing tail that
already match through their complete part (losing the short fragment does not lose the vehicle); the
false merge joins a short east fragment to an east→west crossing → the route stays east→west (same
match). The cascade risk did not materialize here (the wrong decision stayed contained to the trail; it
changed neither route nor count). *Acknowledged risk:* a false merge that joined two vehicles of a
**different route** would indeed cascade; it does not happen in this stretch, and it is declared as a
risk.

## Causal event commit

- **Stop-line crossing:** detected when it happens (not over the whole trajectory). Origin + entry
  lane are committed at crossing.
- **Entry lane:** a ±3 frame window at the crossing = **0.101 s of look-ahead** (3/29.58 fps). It is a
  **quantified causality violation**: it enters as an explicit line in the orchestrator's delay budget,
  not as a footnote.
- **Class: running vote** (not majority of the whole track). Measured: **47/179 tracks (26 %)** have a
  running vote that differs from the final one before converging (median at 18 % of the track's
  lifetime). The class shown early may change; it affects the classification metric, not route or count
  (`class_id` does not enter matching, verified).
- **Destination:** predicted at crossing / observed at death (route piece, already closed).

## Verifications

`CausalReconnector` is pure with respect to perception (it receives `feat` per track; whoever computes
them —batch for measuring, incremental tracker in the orchestrator— is separate). It reuses
detector/tracker/position/route/contract. Detection and classification fidelity is measured offline
against the GT (it does not enter the harness).

## Known limit: it only reconnects sequential fragments (not concurrent ones)

`on_birth` only looks at tails of **dead** tracks (`_pool`) with `gap = f0 − f1 > 0` (the "gap>0
excludes co-temporal" rule). By design it does not merge two **concurrent** ids (both alive at once),
and the anti-over-merge criterion assumes "two complete crossings = two vehicles". **Concurrent
duplication** (a moving vehicle with 2+ stacked ids, from the approach to the exit stop) is a failure
of the tracking layer (ByteTrack creates concurrent ids from a double detection by the detector),
upstream of the reconnector and outside its model. It is addressed in the detector/tracker
(`agnostic_nms` for cross-class; init-IoU for same-class), not here. The reconnector does its own job
well (fragments from occlusion).

## References

- `docs/modules/reconnect_fragments.md`: the batch (global) reconnection that this module causalizes,
  with the same conditions and anti-over-merge criterion.
- `docs/modules/tracking.md`, `docs/modules/track_events.md`, `docs/modules/matching_sombras.md`:
  the rest of the perception and evaluation chain.
- `docs/modules/harness_orchestrator.md`: the orchestrator that assembles causal perception.
- Tests: `tests/test_causal_reconnect.py`.

# reconstruccion_causal

**Causal** reimplementation of the messages-shadow reconstructor
(V2I messages → SUMO), *batch with guard* pattern: the full CSV is held in
memory but an assert forbids querying messages with
`timestamp > t_decision + buffer` (buffer = 1 s).

- Module documentation: [`docs/reconstructor_causal.md`](docs/reconstructor_causal.md)
- The previous base version (not included in this repository)
  is kept **intact** as the comparison baseline; its code is neither imported
  nor referenced here.
- It runs over a Monte Carlo sweep generated offline (not included in the repository),
  mobility `f712b746`.

Structure: `scripts/` (guard, core, player, comparator, preparer),
`inputs/` (network + adapted CSVs), `tests/` (guard), `runs/` (runs and
comparisons).

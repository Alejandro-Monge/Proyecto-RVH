"""Causal contract of the SUMO block (the final shadow is realtime causal).

Defines the invariants that EVERY causal piece must satisfy + testable verifiers. It is a
**named checklist**, not implicit discipline. THIN and CO-EVOLVED: it is extracted from what the
arm↔edge mapping already satisfied (1st instance) and from what the position piece needs (2nd
instance); it will be EXTENDED with the route piece. See `digital_shadow/causal/contract.py` and
`docs/modules/causal_contract.md`.
"""

from digital_shadow.causal.contract import (
    INVARIANTS,
    CausalEstimator,
    check_backward_only,
    check_deferred_commit,
    check_pure_lookup,
)

__all__ = [
    "INVARIANTS",
    "CausalEstimator",
    "check_pure_lookup",
    "check_backward_only",
    "check_deferred_commit",
]

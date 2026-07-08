"""ROUTE piece of the shadow in SUMO (incremental origin/destination commit, with sealed evidence).

Decides the **route** of the replicated vehicle: origin (when crossing the stop line) and destination
(exit arm/edge), for `traci.vehicle.changeTarget`. Design audited with data:

  - **Origin** = `entry_arm` + `entry_lane`, committed at the **stop-line crossing** (causal instant).
  - **Destination — three paths with RECORDED EVIDENCE** (not metadata: it is the measurement of the
    delay-vs-hit curve Section 8.2):
      * **predicho_carril** (on crossing): `lane→turn` (rule calibrated from the GT,
        `lane_turn_rule.yaml`, 88–100 % LOO) → exit arm/edge. **Low delay.** Enables junction gap
        filling.
      * **observado** (on crossing the exit): full enter-curve-exit trail. **High delay**, maximum
        fidelity.
      * **arm_only** (sur): the classifier does not read the sur lane from the image (`arm_only`; the
        sur GT is complete, it is the IMAGE that is insufficient), and its arm prior is ~uniform →
        **fallback without turn prediction** (declared, not hidden).
  - **Junction gap filling:** when the destination is predicted, `changeTarget(exit_edge)` gives a
    route; in the gap SUMO advances along the route (curve) → it does not cut the corner, it does not
    teleport. On observed frames the pos comes from the position piece.

Causal contract (extended clause ii — DEFERRED commit): predicted and observed are **TWO SEALED
commits, append-only**; the commit at t is NOT rewritten by evidence > t (what arrives later is a
NEW sealed commit). Verified with `check_deferred_commit`. Runtime reads only config.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Mapping, Optional

import yaml

from digital_shadow.config.site import find_project_root
from digital_shadow.sumo.lane_map import SumoLaneMap

# Possible evidence values (for the Section 8.2 log).
EV_PREDICHO = "predicho_carril"
EV_OBSERVADO = "observado"
EV_ARM_ONLY = "arm_only"
EV_HUECO = "hueco_inferido"


# ---------------------------------------------------------------------------- #
# Config loading (runtime reads ONLY config)
# ---------------------------------------------------------------------------- #
def load_lane_turn_rule(site_id: str = "rusia", project_root: Optional[Path] = None) -> dict:
    """lane→turn rule calibrated from the GT (`config/sites/<id>/lane_turn_rule.yaml`)."""
    root = project_root or find_project_root()
    with open(root / "config" / "sites" / site_id / "lane_turn_rule.yaml", encoding="utf-8") as fh:
        return yaml.safe_load(fh)["rule"]


def load_turns(site_id: str = "rusia", project_root: Optional[Path] = None) -> dict:
    """(entry_arm, turn) → exit_arm from `turns.csv` (only allowed movements)."""
    import csv

    root = project_root or find_project_root()
    out = {}
    with open(root / "config" / "sites" / site_id / "turns.csv", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            if row["status"] == "allowed" and row["turn"]:
                out[(row["entry_arm"], row["turn"])] = row["exit_arm"]
    return out


# ---------------------------------------------------------------------------- #
# PURE lookups (contract clause i)
# ---------------------------------------------------------------------------- #
def predict_turn(rule: dict, arm: str, lane_label) -> Optional[str]:
    """Turn predicted by the lane (None if the arm is arm_only/not usable or the lane is not there)."""
    spec = rule.get(arm)
    if not spec or not spec.get("usable"):
        return None
    return spec.get("lane_turn", {}).get(str(lane_label).strip()) or None


def predicted_exit(
    rule: dict, turns: dict, lane_map: SumoLaneMap, arm: str, lane_label
) -> tuple[Optional[str], Optional[str], Optional[str]]:
    """(turn, exit_arm, exit_edge) predicted from the lane. Any None if there is no prediction."""
    turn = predict_turn(rule, arm, lane_label)
    if turn is None:
        return None, None, None
    exit_arm = turns.get((arm, turn))
    if exit_arm is None:
        return turn, None, None
    return turn, exit_arm, lane_map.arms[exit_arm].exit_edge


# ---------------------------------------------------------------------------- #
# Sealed commit (extended contract clause ii): append-only
# ---------------------------------------------------------------------------- #
@dataclass(frozen=True)
class RouteCommit:
    """A SEALED commit (immutable). `field`: origin|destination|gap_fill. For Section 8.2:
    `evidence`, `commit_time`, `delay` (= commit_time − entry_time)."""

    field: str
    arm: str
    evidence: str
    commit_time: float
    lane_label: Optional[str] = None
    turn: Optional[str] = None
    exit_arm: Optional[str] = None
    exit_edge: Optional[str] = None
    entry_time: Optional[float] = None
    delay: Optional[float] = None
    extra: Optional[tuple] = None  # e.g. the filled gap (frames)


class RouteCommitter:
    """Per-vehicle state: emits SEALED, append-only commits. Predicted and observed are distinct
    commits (no overwriting). `target_edge()` = the last committed destination (for `changeTarget`).
    The "when" is decided by the harness (it calls the `commit_*` at its instant)."""

    def __init__(self, arm: str, lane_map: SumoLaneMap, turns: dict, rule: dict) -> None:
        self.arm = arm
        self._lm = lane_map
        self._turns = turns
        self._rule = rule
        self._commits: list[RouteCommit] = []
        self._entry_time: Optional[float] = None

    def _delay(self, t: float) -> Optional[float]:
        return None if self._entry_time is None else t - self._entry_time

    def commit_origin(self, t: float, lane_label) -> RouteCommit:
        """At the stop-line crossing: fixes the origin (entry_arm + entry_lane)."""
        self._entry_time = t
        c = RouteCommit("origin", self.arm, EV_OBSERVADO, t, lane_label=str(lane_label),
                        entry_time=t)
        self._commits.append(c)
        return c

    def commit_destination_predicted(self, t: float, lane_label) -> RouteCommit:
        """At the crossing: destination PREDICTED by lane→turn (or arm_only if sur does not resolve it)."""
        turn, exit_arm, exit_edge = predicted_exit(self._rule, self._turns, self._lm, self.arm, lane_label)
        ev = EV_PREDICHO if turn is not None else EV_ARM_ONLY
        c = RouteCommit("destination", self.arm, ev, t, lane_label=str(lane_label), turn=turn,
                        exit_arm=exit_arm, exit_edge=exit_edge, entry_time=self._entry_time,
                        delay=self._delay(t))
        self._commits.append(c)
        return c

    def commit_destination_observed(self, t: float, exit_arm: str) -> RouteCommit:
        """At the EXIT crossing: OBSERVED destination (full trail). NEW commit, does not rewrite."""
        c = RouteCommit("destination", self.arm, EV_OBSERVADO, t, exit_arm=exit_arm,
                        exit_edge=self._lm.arms[exit_arm].exit_edge, entry_time=self._entry_time,
                        delay=self._delay(t))
        self._commits.append(c)
        return c

    def record_gap_fill(self, t: float, gap_frames: tuple) -> RouteCommit:
        """Marks that a junction gap was filled along the predicted route (hueco_inferido evidence,
        for Section 7.4/Section 8). It does not change the destination; it records the event."""
        tgt = self.target_edge()
        c = RouteCommit("gap_fill", self.arm, EV_HUECO, t, exit_edge=tgt,
                        entry_time=self._entry_time, delay=self._delay(t), extra=tuple(gap_frames))
        self._commits.append(c)
        return c

    def commits(self) -> tuple[RouteCommit, ...]:
        """SEALED list of commits (append-only; never rewritten)."""
        return tuple(self._commits)

    def target_edge(self) -> Optional[str]:
        """Last committed exit edge (for changeTarget); None if there is none yet."""
        for c in reversed(self._commits):
            if c.field == "destination" and c.exit_edge:
                return c.exit_edge
        return None

    def prediction_hit(self) -> Optional[bool]:
        """If there was predicted and observed: did the exit_arm match? (prediction-vs-observed hit,
        for Section 8.2). None if either is missing."""
        pred = next((c for c in self._commits if c.field == "destination" and c.evidence == EV_PREDICHO), None)
        obs = next((c for c in self._commits if c.field == "destination" and c.evidence == EV_OBSERVADO), None)
        if pred is None or obs is None:
            return None
        return pred.exit_arm == obs.exit_arm

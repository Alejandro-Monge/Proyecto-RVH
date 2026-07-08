"""Orchestrator sinks: where the drawing orders go (instantiate/move/retire).

The causal loop does not know whether it draws in SUMO or only records: it talks to a `Sink`.
`RecordingSink` (measurement, without SUMO, deterministic) records the orders; `SumoStageSink`
translates them to `SumoStage` (live SUMO). This way the causal core is measured without launching
SUMO and rendered with SUMO without changing the loop.
"""

from __future__ import annotations

from dataclasses import dataclass, field


class Sink:
    """Minimal sink interface (position-driven: the loop dictates the position)."""

    def instantiate(self, vid, edge, lane_index, pos): ...
    def set_target(self, vid, exit_edge): ...
    def move_xy(self, vid, x, y): ...
    def retire(self, vid): ...
    def sync(self, t): ...


@dataclass
class RecordingSink(Sink):
    """Records the orders without SUMO. To measure the stream + the delays deterministically."""

    calls: list = field(default_factory=list)
    live_vehicles: set = field(default_factory=set)

    def instantiate(self, vid, edge, lane_index, pos):
        self.calls.append(("instantiate", vid, edge, lane_index, round(pos, 2)))
        self.live_vehicles.add(vid)

    def set_target(self, vid, exit_edge):
        self.calls.append(("set_target", vid, exit_edge))

    def move_xy(self, vid, x, y):
        self.calls.append(("move_xy", vid, round(x, 2), round(y, 2)))

    def retire(self, vid):
        self.calls.append(("retire", vid))
        self.live_vehicles.discard(vid)

    def sync(self, t):
        pass  # in measurement there is no SUMO clock


class SumoStageSink(Sink):
    """Translates the orders to a `SumoStage` (live SUMO). For the integration/viz.

    SHIELDED: SUMO may retire a vehicle on its own — confirmed: on ARRIVING at its route (end of the
    egress ≈ leaving the intersection) — before the orchestrator retires it by end-of-chain. The sink
    detects it (exception / absent vehicle), counts it (`dropped`), and stops moving it, without
    bringing down the loop. It is an integration FINDING (SUMO's retire vs the orchestrator's) to
    reconcile in the viz; it does NOT affect the stream/delays. The bounds-guard (`out_of_net`) is an
    extra safeguard against coords outside the network (caught 0 in the measured segment)."""

    def __init__(self, stage, margin=20.0):
        self._stage = stage
        self._live = set()
        self.dropped = set()   # vehicles that SUMO discarded (invalid coords)
        self.out_of_net = 0    # moves skipped for falling outside the network (far H error)
        self._bounds = None
        self._margin = margin

    def _in_net(self, x, y):
        if self._bounds is None:
            (x0, y0), (x1, y1) = self._stage._traci.simulation.getNetBoundary()
            self._bounds = (x0 - self._margin, y0 - self._margin, x1 + self._margin, y1 + self._margin)
        x0, y0, x1, y1 = self._bounds
        return x0 <= x <= x1 and y0 <= y <= y1

    def instantiate(self, vid, edge, lane_index, pos):
        self._stage.add(vid, edge, lane_index, pos)
        self._live.add(vid)

    def set_target(self, vid, exit_edge):
        if exit_edge and vid in self._live and vid not in self.dropped:
            try:
                self._stage.set_target(vid, exit_edge)
            except Exception:
                self.dropped.add(vid)

    def move_xy(self, vid, x, y):
        if vid not in self._live or vid in self.dropped:
            return
        if not self._in_net(x, y):     # world point outside the network (far H error) → keep last pos
            self.out_of_net += 1
            return
        try:
            self._stage.place_xy(vid, x, y, keep_route=2)
        except Exception:
            self.dropped.add(vid)   # SUMO discarded the vehicle (invalid / distant coords)

    def retire(self, vid):
        self._live.discard(vid)
        if vid in self.dropped:
            return
        try:
            self._stage.remove(vid)
        except Exception:
            pass

    def sync(self, t):
        self._stage.sync(t)

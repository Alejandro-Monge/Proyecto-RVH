"""Orchestrator of the causal loop. Assembles, frame by frame and WITHOUT seeing the future:

  source(replay tracks) → causal reconnection → STREAMING event commit → position/route → Sink(SUMO).

Per frame f at t_v = f·frame_dt (= 1/site.fps_ref; 12/355 in rusia): (1) births → reconnection `on_birth`
(assigns to a chain); (2) observations → chains (incremental stop-line crossing, running class vote,
position filter from birth); (3) upon detecting the entry crossing (+ window ±3 = 0.101 s) → ORIGIN commit
+ PREDICTED destination → instantiate in SUMO; (4) each frame → move (observed position / gap filling);
(5) deaths → `on_death`; (6) confirmed end of chain (last death + buffer) → OBSERVED destination commit
→ retire; (7) `sync(t_v)`; SEALS the delay of each commit (physical instant − physical event).

Reuses detector/tracker (CSV replay), `CausalReconnector`, `VehiclePositioner`, `RouteCommitter`.
The reconnection uses the same order/decision as the reference reconnection (online, given features); the
event commit is online incremental (it does not re-derive at the end) — it is AUDITED that the stream
matches that reference reconnection.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from fractions import Fraction
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
import derive_track_events as dte
from digital_shadow.geometry.ground_contact import GROUND_X, GROUND_Y
from digital_shadow.harness.sink import RecordingSink, Sink
from digital_shadow.sumo.position import VehiclePositioner, project_to_world
from digital_shadow.sumo.route import RouteCommitter, predicted_exit
from digital_shadow.sumo.stage import GapFiller, point_at_arclen


@dataclass
class Commit:
    """A sealed commit of the orchestrator (for the Section 7.2 delay budget)."""
    chain: int
    kind: str            # instanciacion | origen | destino_predicho | destino_observado
    evidence: str
    frame: int
    t_commit: float      # PHYSICAL instant of the commit (s)
    payload: dict = field(default_factory=dict)


class ChainState:
    """Per-CHAIN state (reconnected track): accumulated trajectory, incremental entry crossing,
    running class vote, position/route. Past only."""

    def __init__(self, root, harness):
        self.root = root
        self.h = harness
        self.fr, self.ts, self.gx, self.gy, self.cl = [], [], [], [], []
        self.entry_arm = None
        self.prev_d = None
        self.cross_idx = None          # index (in the list) of the frame just after the −→+ crossing
        self.cross_t = None
        self.origin_done = False
        self.exit_done = False
        self.positioner: Optional[VehiclePositioner] = None
        self.route = RouteCommitter(None, harness.lane_map, harness.turns, harness.rule)
        self.vid = f"v{root}"
        self.instantiated = False
        self.last_obs_frame = None
        self.gapfiller: Optional[GapFiller] = None
        self.vel_inst = None           # |velocity| of the filter at instantiation (warm-up audit)
        self.n_approach = None         # approach frames processed by the filter before instantiating
        self.n_obs_filter = 0          # real observations fed to the filter (counts the warm-up)

    # ---- running class vote (Section 7.5; does not affect route/count) ----
    def running_class(self):
        from collections import Counter
        return Counter(self.cl).most_common(1)[0][0]

    def observe(self, f, t, gx, gy, cls):
        self.fr.append(f); self.ts.append(t); self.gx.append(gx); self.gy.append(gy); self.cl.append(cls)
        self.last_obs_frame = f
        # entry_arm: from the median of the first WINDOW positions (head)
        if self.entry_arm is None and len(self.fr) >= dte.WINDOW_FRAMES:
            head = np.array([np.median(self.gx[:dte.WINDOW_FRAMES]), np.median(self.gy[:dte.WINDOW_FRAMES])])
            self.entry_arm = dte.arm_of(head, self.h.center, dte.CORE_RADIUS_PX)
            if self.entry_arm != dte.CORE:
                self.positioner = VehiclePositioner(self.entry_arm, self.h.lane_map, self.h.H, self.h.alpha)
                self.route.arm = self.entry_arm
        # longitudinal position filter ONLY in the approach (until the origin commit; afterward the
        # pos comes from observed moveToXY, not from the filter, which only holds over the approach axis).
        if self.positioner is not None and not self.origin_done:
            self.positioner.observe(t, (gx, gy))
            self.n_obs_filter += 1
        # INCREMENTAL entry crossing (sign of the distance to the entry_arm stop line)
        if self.entry_arm not in (None, dte.CORE) and self.cross_idx is None:
            seg = self.h.stop_lines.get(self.entry_arm)
            if seg is not None:
                p1, nh = dte._inward_normal(seg, self.h.center)
                d = float((np.array([gx, gy]) - p1) @ nh)
                if self.prev_d is not None and self.prev_d <= 0 < d:
                    # crossing between the previous frame and this one; interpolate the instant
                    frac = (0.0 - self.prev_d) / (d - self.prev_d)
                    self.cross_t = self.ts[-2] + (t - self.ts[-2]) * frac
                    self.cross_idx = len(self.fr) - 1
                self.prev_d = d

    def try_commit_origin(self, sink: Sink):
        """When the crossing happened and there are ±WINDOW frames after → ORIGIN commit + predicted + instantiate."""
        if self.origin_done or self.cross_idx is None or self.entry_arm in (None, dte.CORE):
            return None
        if self.entry_arm not in self.h.lane_map.arms or self.h.lane_map.arms[self.entry_arm].entry_edge is None:
            return None
        if len(self.fr) < self.cross_idx + 3 + 1:      # ±3 lane window = 0.101 s look-ahead
            return None
        g = pd.DataFrame({"frame_index": self.fr, "timestamp_s": self.ts, GROUND_X: self.gx, GROUND_Y: self.gy})
        seg = self.h.stop_lines[self.entry_arm]
        s = dte.lane_cross_coord(g, seg, self.h.center, kind="entry")
        lane = dte.classify_lane(s, self.h.lane_bands.get("entry", {}).get(self.entry_arm))
        f, t = self.fr[-1], self.ts[-1]
        self.route.commit_origin(self.cross_t, lane)
        dest = self.route.commit_destination_predicted(self.cross_t, lane)
        self.positioner.commit_lane(lane)
        self.vel_inst = abs(self.positioner._filter.velocity)   # warm-up: |v| at instantiation
        self.n_approach = self.n_obs_filter                     # approach frames that warmed up the filter
        self.origin_done = True
        # instantiate in SUMO at (entry edge, lane index, longitudinal pos)
        place = self.positioner.placement()
        edge = self.h.lane_map.arms[self.entry_arm].entry_edge
        lane_idx = int(place.lane_id.split("_")[-1]) if place.lane_id else 0
        sink.instantiate(self.vid, edge, lane_idx, place.pos)
        sink.set_target(self.vid, dest.exit_edge)
        self.instantiated = True
        return [Commit(self.root, "origen", "observado_cruce", f, t,
                       dict(entry_arm=self.entry_arm, entry_lane=lane, cross_t=self.cross_t)),
                Commit(self.root, "destino_predicho", dest.evidence, f, t,
                       dict(turn=dest.turn, exit_arm=dest.exit_arm)),
                Commit(self.root, "instanciacion", "origen", f, t, dict(edge=edge, pos=place.pos))]

    def move(self, f, t, sink: Sink):
        """Position-driven: dictates the observed position; if there is a gap, fill along the internal lane."""
        if not self.instantiated or self.exit_done:
            return
        if self.last_obs_frame == f and self.gx:
            x, y = project_to_world(self.h.H, (self.gx[-1], self.gy[-1]))
            sink.move_xy(self.vid, x, y)
            self.gapfiller = None
        else:
            # GAP: interpolate over the shape of the predicted internal lane at the last velocity
            shape = self.h.internal_shape(self.route)
            if shape is not None:
                if self.gapfiller is None:
                    speed = self.positioner._filter.velocity if self.positioner else 0.0
                    self.gapfiller = GapFiller(shape, abs(speed), t)
                x, y = self.gapfiller.at(t)
                sink.move_xy(self.vid, x, y)

    def commit_exit(self, sink: Sink):
        if self.exit_done or not self.instantiated:
            self.exit_done = True
            return None
        g = pd.DataFrame({"frame_index": self.fr, "timestamp_s": self.ts, GROUND_X: self.gx, GROUND_Y: self.gy})
        tail = np.array([np.median(self.gx[-dte.WINDOW_FRAMES:]), np.median(self.gy[-dte.WINDOW_FRAMES:])])
        exit_arm = dte.arm_of(tail, self.h.center, dte.CORE_RADIUS_PX)
        exit_t = None
        f, t = self.fr[-1], self.ts[-1]
        self.exit_done = True
        if exit_arm != dte.CORE and exit_arm in self.h.stop_lines:
            exit_t = dte.stopline_crossing_time(g, self.h.stop_lines[exit_arm], self.h.center, "exit")
            self.route.commit_destination_observed(t, exit_arm)
        sink.retire(self.vid)
        return Commit(self.root, "destino_observado", "observado", f, t,
                      dict(exit_arm=exit_arm, exit_t=exit_t, entry_arm=self.entry_arm,
                           entry_lane=self.route.commits()[0].lane_label if self.route.commits() else None,
                           cross_t=self.cross_t))


class CausalOrchestrator:
    """End-to-end causal loop. `run` processes the tracks frame by frame and seals the delay."""

    def __init__(self, *, tracks, feats, reconnector, lane_map, H, rule, turns,
                 stop_lines, lane_bands, center, alpha, buffer_frames,
                 excluded=frozenset(), internal_shapes=None, frame_dt=None):
        self.tracks = tracks
        self.feats = feats
        self.reconnector = reconnector
        self.lane_map = lane_map
        self.H = H
        self.rule = rule
        self.turns = turns
        self.stop_lines = stop_lines
        self.lane_bands = lane_bands
        self.center = center
        self.alpha = alpha
        self.buffer = buffer_frames
        self.excluded = excluded
        self._shapes = internal_shapes or {}  # (entry_arm, exit_arm) -> polyline
        # EXACT frame period (Fraction, typically 1/site.fps_ref). The clock arithmetic
        # is kept RATIONAL (float(f*frame_dt): a single rounding); injecting a float would
        # break 129695/200000 t_v in the last bit by double rounding. None = compat of the
        # measurement callers not yet migrated (noted debt): value inherited from the rusia
        # site, behavior unchanged.
        self.frame_dt = frame_dt if frame_dt is not None else Fraction(12, 355)

    def internal_shape(self, route: RouteCommitter):
        """Polyline of the internal lane of the PREDICTED movement (for gap filling)."""
        tgt = next((c for c in route.commits() if c.field == "destination" and c.exit_arm), None)
        if tgt is None:
            return None
        return self._shapes.get((route.arm, tgt.exit_arm))

    def run(self, sink: Sink):
        df = self.tracks[~self.tracks.track_id.isin(self.excluded)]
        by_frame = {f: g for f, g in df.groupby("frame_index")}
        births = {}
        deaths = {}
        for tid, f in self.feats.items():
            if tid in self.excluded:
                continue
            births.setdefault(f["f0"], []).append(tid)
            deaths.setdefault(f["f1"], []).append(tid)
        frames = sorted(by_frame)
        chains: dict = {}
        track_root: dict = {}
        commits: list = []
        pending_end: dict = {}     # root -> frame of the last death (for end = +buffer)
        for f in frames:
            t_v = float(f * self.frame_dt)
            # 1) BIRTHS → reconnection
            for tid in births.get(f, []):
                edge = self.reconnector.on_birth(tid, self.feats[tid])
                if edge is not None:
                    root = track_root[edge[0]]
                    track_root[tid] = root             # tid's obs are routed to A's chain
                    pending_end.pop(root, None)        # the chain revives (a fragment arrived)
                else:
                    root = tid
                    track_root[tid] = root
                    chains[root] = ChainState(root, self)
            # 2) OBSERVATIONS → chains
            g = by_frame.get(f)
            if g is not None:
                for r in g.itertuples():
                    root = track_root.get(r.track_id)
                    if root is None or root not in chains:
                        continue
                    chains[root].observe(f, t_v, r.gx, r.gy, int(r.class_id))
            # 3) ORIGIN commit + instantiate; 4) MOVE
            for ch in chains.values():
                c = ch.try_commit_origin(sink)
                if c:
                    commits.extend(c)
                ch.move(f, t_v, sink)
            # 5) DEATHS → reconnection + pending-end mark
            for tid in deaths.get(f, []):
                self.reconnector.on_death(tid, self.feats[tid])
                pending_end[track_root[tid]] = f
            # 6) confirmed end of chain (last death + buffer, without reviving)
            for root, fdeath in list(pending_end.items()):
                if f - fdeath >= self.buffer:
                    c = chains[root].commit_exit(sink)
                    if c:
                        commits.append(c)
                    del pending_end[root]
            # 7) SUMO clock
            sink.sync(t_v)
        # closing: chains alive at the end of the segment
        for root in pending_end:
            c = chains[root].commit_exit(sink)
            if c:
                commits.append(c)
        return commits, chains

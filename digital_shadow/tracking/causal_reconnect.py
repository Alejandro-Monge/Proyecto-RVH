"""CAUSAL-GREEDY fragment reconnection (online), counterpart to the global batch one.

The closed reconnection (batch, `scripts/reconnect_fragments.py`) does a **global**
matching: it sorts ALL candidates by cost and assigns one-to-one (union-find), with
complete info for each track. In realtime that is impossible: when a track B is
**born**, only **past** candidates exist (dead tracks within the buffer). This module
decides **at the moment of birth**, greedy, without seeing the future — and that is why
it can **differ** from the batch (online-vs-offline gap, not a defect: the batch is the
theoretical upper bound with complete information; this is what is achievable in
realtime).

Same gates as the batch (gap∈(0,buffer], corridor, class, continuation ≤60°, valid route)
and the **anti-over-merge gate** ("never ≥2 COMPLETE cross-arm fragments"), but re-expressed
causally: when merging B onto A's chain it is rejected if A's chain **already** contains a
complete cross-arm fragment (all members of A's chain are dead → their `is_cross` is known;
B's is not, it becomes known at its death). Discrete event simulation: `on_death` (enters
the pool of tails + updates the chain's cross) and `on_birth` (looks for the best past tail
and merges).

PURE with respect to perception: it receives the per-track `feat` (endpoints, headings, class,
arms, is_cross) and emits the `id_map`/components. Whoever computes them (batch for measuring;
incremental tracker in the orchestrator) is separate.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional


def _angle_deg(ax, ay, bx, by) -> float:
    na = math.hypot(ax, ay)
    nb = math.hypot(bx, by)
    if na < 1.0 or nb < 1.0:
        return 0.0  # indeterminate heading (short/stopped track) → not rejected by angle
    c = max(-1.0, min(1.0, (ax * bx + ay * by) / (na * nb)))
    return math.degrees(math.acos(c))


@dataclass
class CausalReconnector:
    """Greedy-online reconnector. Feed `on_death`/`on_birth` in temporal order."""

    buffer_frames: int
    corridor_px: float
    valid_moves: set
    max_heading_dev_deg: float = 60.0

    _pool: dict = field(default_factory=dict)        # tid (free tail, dead) -> feat
    _parent: dict = field(default_factory=dict)      # union-find: tid -> root
    _members: dict = field(default_factory=dict)     # root -> [tids] in merge order
    _has_cross: dict = field(default_factory=dict)   # root -> is any DEAD member a complete cross-arm?
    _edges: list = field(default_factory=list)       # (a, b, gap, dist) accepted

    def _root(self, t):
        while self._parent.get(t, t) != t:
            self._parent[t] = self._parent.get(self._parent[t], self._parent[t])
            t = self._parent[t]
        return t

    def _ensure(self, tid, feat):
        if tid not in self._parent:
            self._parent[tid] = tid
            self._members[tid] = [tid]
            self._has_cross[tid] = False
            self._feat = getattr(self, "_feat", {})
            self._feat[tid] = feat

    def on_death(self, tid, feat) -> None:
        """The track dies: its tail becomes available and its `is_cross` (now known) updates the chain."""
        self._ensure(tid, feat)
        self._pool[tid] = feat
        r = self._root(tid)
        self._has_cross[r] = self._has_cross[r] or bool(feat["is_cross"])

    def on_birth(self, tid, feat) -> Optional[tuple]:
        """The track is born: look for the best PAST tail (within the buffer, gates, anti-over-merge
        proxy) and merge. Returns the edge (a, b, gap, dist) or None."""
        self._ensure(tid, feat)
        best = None  # (cost, gap, dist, a)
        for a, fa in self._pool.items():
            gap = feat["f0"] - fa["f1"]
            if gap <= 0 or gap > self.buffer_frames:           # gap>0 excludes co-temporal ones
                continue
            dx, dy = feat["head"][0] - fa["tail"][0], feat["head"][1] - fa["tail"][1]
            dist = math.hypot(dx, dy)
            if dist > self.corridor_px:                        # corridor
                continue
            if fa["cls"] != feat["cls"]:                       # consistent class
                continue
            if _angle_deg(dx, dy, fa["tail_heading"][0], fa["tail_heading"][1]) > self.max_heading_dev_deg:
                continue
            if self._has_cross[self._root(a)]:                 # causal anti-over-merge PROXY
                continue                                       # A's chain already has a complete cross-arm
            cost = gap + dist * 0.001
            if best is None or cost < best[0]:
                best = (cost, gap, dist, a)
        if best is None:
            return None
        cost, gap, dist, a = best
        ra = self._root(a)
        self._parent[self._root(tid)] = ra                     # B joins A's chain
        self._members[ra].append(tid)
        del self._pool[a]                                      # A's tail is consumed (one-to-one)
        self._edges.append((a, tid, gap, dist))
        return (a, tid, gap, dist)

    def result(self) -> tuple[dict, list]:
        """(id_map old->rep, merged components with metadata), applying the final valid route."""
        comps = {}
        for t in self._parent:
            comps.setdefault(self._root(t), []).append(t)
        id_map, merged = {}, []
        for root, members in comps.items():
            if len(members) < 2:
                continue
            members = sorted(members, key=lambda t: self._feat[t]["f0"])
            m_entry = self._feat[members[0]]["entry_arm"]
            m_exit = self._feat[members[-1]]["exit_arm"]
            if (m_entry, m_exit) not in self.valid_moves:      # valid route of the merged movement
                continue
            rep = min(members)
            for t in members:
                id_map[t] = rep
            gaps = [self._feat[members[i + 1]]["f0"] - self._feat[members[i]]["f1"]
                    for i in range(len(members) - 1)]
            merged.append(dict(rep=rep, members=members, entry=m_entry, exit=m_exit, gaps=gaps))
        return id_map, merged

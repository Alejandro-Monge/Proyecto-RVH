"""CAUSAL extension of the core via SUBCLASS (R4); the production core stays intact.

`ChainCausalExit` adds to the production ChainState the INCREMENTAL detection of the EXIT
stop-line crossing (mirror of the entry crossing): after the origin commit, at each observation
it evaluates the +→− crossing of the stop lines of the approaches ≠ entry, requiring that the s
coordinate of the crossing fall within the real segment ± margin (kills false crossings with the
extended line of another stop line). Once the crossing is confirmed and the SAME declared ±3-frame
window of the origin (0.101 s) has passed, it emits the `destino_observado` COMMIT AT THAT INSTANT
— information available live — with the exit lane classified by the SAME production function
(`dte.lane_cross_coord(kind="exit")` over the accumulated trail, which contains only the past).

The end-of-chain commit (last death + buffer) remains a RETIREMENT; if the destination was already
committed causally, it is not re-committed (clause ii: sealed commits, no rewriting).

`CausalOrchestratorExit` is the production run() with two changes: it instantiates
`ChainCausalExit` and drains the causal exit commits per frame. (Copy of the run()
in digital_shadow/harness/orchestrator.py.)
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import derive_track_events as dte  # noqa: E402
from digital_shadow.geometry.ground_contact import GROUND_X, GROUND_Y  # noqa: E402
from digital_shadow.harness.orchestrator import CausalOrchestrator, ChainState, Commit  # noqa: E402

SEG_MARGIN_PX = 40.0     # tolerance of membership in the stop-line segment (declared)


class ChainCausalExit(ChainState):
    """ChainState + observed-destination commit AT THE EXIT STOP-LINE CROSSING (causal)."""

    def __init__(self, root, harness):
        super().__init__(root, harness)
        self.exit_obs_done = False
        self.exit_obs_arm = None
        self.exit_obs_t = None        # physical instant of the crossing (interpolated)
        self.exit_obs_idx = None      # list index of the frame just after the crossing
        self.t_exit_commit = None     # instant of the COMMIT (crossing + ±3 window)
        self.exit_carril_obs = None
        self._exit_prev: dict = {}    # arm -> previous signed distance

    def observe(self, f, t, gx, gy, cls):
        super().observe(f, t, gx, gy, cls)
        if not self.origin_done or self.exit_obs_done:
            return
        if self.exit_obs_idx is None:
            self._detect_exit_crossing(t, gx, gy)
        if self.exit_obs_idx is not None and len(self.fr) >= self.exit_obs_idx + dte.WINDOW_FRAMES:
            self._commit_exit_causal(f, t)

    def _detect_exit_crossing(self, t, gx, gy):
        p = np.array([gx, gy])
        for arm, seg in self.h.stop_lines.items():
            if arm == self.entry_arm:
                continue
            p1, nh = dte._inward_normal(seg, self.h.center)
            d = float((p - p1) @ nh)
            prev = self._exit_prev.get(arm)
            self._exit_prev[arm] = d
            if prev is None or not (prev > 0 >= d):          # +→− crossing (outward)
                continue
            u = seg[1] - seg[0]
            seg_len = float(np.linalg.norm(u))
            s = float((p - seg[0]) @ (u / seg_len))
            if not (-SEG_MARGIN_PX <= s <= seg_len + SEG_MARGIN_PX):
                continue                                     # crossing with the line, not with the stop line
            frac = prev / (prev - d)
            self.exit_obs_t = self.ts[-2] + (t - self.ts[-2]) * frac
            self.exit_obs_idx = len(self.fr) - 1
            self.exit_obs_arm = arm
            return

    def _commit_exit_causal(self, f, t):
        """Commit of the observed destination with the SAME production lane classification,
        over the accumulated trail (only the past: the last +→− crossing of the trail IS this one)."""
        arm = self.exit_obs_arm
        g = pd.DataFrame({GROUND_X: self.gx, GROUND_Y: self.gy})
        band = self.h.lane_bands.get("exit", {}).get(arm)
        carril = None
        if band and arm in self.h.stop_lines:
            s = dte.lane_cross_coord(g, self.h.stop_lines[arm], self.h.center, kind="exit")
            if s is not None:
                lab = dte.classify_lane(s, band)
                carril = int(lab) if lab.isdigit() else None
        self.route.commit_destination_observed(t, arm)
        self.exit_carril_obs = carril
        self.t_exit_commit = t
        self.exit_obs_done = True
        self.h.extra_commits.append(Commit(
            self.root, "destino_observado", "observado_causal", f, t,
            dict(exit_arm=arm, exit_t=self.exit_obs_t, exit_carril=carril,
                 entry_arm=self.entry_arm, cross_t=self.cross_t)))

    def commit_exit(self, sink):
        """End of chain = RETIREMENT. If the destination was already committed causally, it is NOT re-committed
        (sealed commits); if it never crossed the exit, it falls back to the production behavior."""
        if self.exit_obs_done:
            if self.exit_done or not self.instantiated:
                self.exit_done = True
                return None
            f, t = self.fr[-1], self.ts[-1]
            self.exit_done = True
            sink.retire(self.vid)
            return Commit(self.root, "fin_cadena", "retiro", f, t,
                          dict(exit_arm=self.exit_obs_arm, exit_t=self.exit_obs_t,
                               entry_arm=self.entry_arm, cross_t=self.cross_t))
        return super().commit_exit(sink)


class CausalOrchestratorExit(CausalOrchestrator):
    """Production run() (quoted copy) with ChainCausalExit + draining of causal commits."""

    def run(self, sink):
        self.extra_commits = []
        df = self.tracks[~self.tracks.track_id.isin(self.excluded)]
        by_frame = {f: g for f, g in df.groupby("frame_index")}
        births, deaths = {}, {}
        for tid, ft in self.feats.items():
            if tid in self.excluded:
                continue
            births.setdefault(ft["f0"], []).append(tid)
            deaths.setdefault(ft["f1"], []).append(tid)
        frames = sorted(by_frame)
        chains, track_root, commits, pending_end = {}, {}, [], {}

        for f in frames:
            t_v = float(f * self.frame_dt)
            for tid in births.get(f, []):
                edge = self.reconnector.on_birth(tid, self.feats[tid])
                if edge is not None:
                    root = track_root[edge[0]]
                    track_root[tid] = root
                    pending_end.pop(root, None)
                else:
                    root = tid
                    track_root[tid] = root
                    chains[root] = ChainCausalExit(root, self)   # <-- ONLY class change
            g = by_frame.get(f)
            if g is not None:
                for r in g.itertuples():
                    root = track_root.get(r.track_id)
                    if root is None or root not in chains:
                        continue
                    chains[root].observe(f, t_v, r.gx, r.gy, int(r.class_id))
            for ch in chains.values():
                c = ch.try_commit_origin(sink)
                if c:
                    commits.extend(c)
                ch.move(f, t_v, sink)
            if self.extra_commits:                               # <-- drain causal commits
                commits.extend(self.extra_commits)
                self.extra_commits = []
            for tid in deaths.get(f, []):
                self.reconnector.on_death(tid, self.feats[tid])
                pending_end[track_root[tid]] = f
            for root, fdeath in list(pending_end.items()):
                if f - fdeath >= self.buffer:
                    c = chains[root].commit_exit(sink)
                    if c:
                        commits.append(c)
                    del pending_end[root]
            sink.sync(t_v)
        for root in pending_end:
            c = chains[root].commit_exit(sink)
            if c:
                commits.append(c)
        # ADDITIVE EXPOSURE: map {track_id → root} for the handoff registry
        # (box↔vehicle correlation). It does not change commits/chains nor any decision
        # → byte-identical readout (verified). It is IDENTITY (re-id), not future
        # trajectory.
        self.track_root = dict(track_root)
        return commits, chains

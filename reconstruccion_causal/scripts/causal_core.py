"""
causal_core — per-vehicle causal reconstruction (batch with guard).

Reimplementation of the reconstruction logic of the previous base version
(not included in this repository) reorganized
into PER-SLOT decisions executable at the instant they become decidable, with
all message access mediated by guardia_causal.GuardedMessageStore.

PRESERVED without numeric change (formulas copied from the base, same
semantics and same thresholds):
  - inverse Veins transform (omnet_to_sumo, 25 m margin),
  - per-slot fill rule: INFERRED if the closing anchor falls within
    (tk, tk+buffer], otherwise UNCERTAIN by dead-reckoning using only the past,
  - mixed reacquisition policy (controlled snap, reacq_snap_th threshold),
  - linear interpolation between consecutive states (linear position, circular
    heading by short arc, hold on snap).

CORRECTED relative to the base (the three audited leaks):
  - REMOVAL: by causal timeout (period + buffer with no messages), not by
    knowledge of the end of the trace. The timeout is the ONLY vehicle
    termination mechanism and does NOT distinguish the cause of the silence
    (loss, leaving the map, shutdown): live those causes are indistinguishable
    and distinguishing them would require future information.
  - ROUTE: it is built incrementally with the edges of the ALREADY-arrived
    messages (norm_edges grows with each arrival), not with the complete path.
  - FINAL SILENCE TAIL: the slots after the last received message are
    extrapolated UNCERTAIN (same formula) until the timeout removes it — the
    base did not generate them because the batch knows where the trace ends.
"""

import math

TICK = 0.1  # step-length (s) — identical to the base

OBSERVED = "OBSERVED"
REACQUIRED = "REACQUIRED"
REACQUIRED_SNAP = "REACQUIRED_SNAP"
INFERRED = "INFERRED"
UNCERTAIN = "UNCERTAIN"
INTERPOLATED = "INTERPOLATED"


# --------------------------------------------------------------------------- #
# Time and coordinates — copied verbatim from the base (sombra_core.py)
# --------------------------------------------------------------------------- #
def tick_of(sim_time):
    """Nearest integer tick index to a simTime (snapping to 0.1 s)."""
    return int(round(sim_time / TICK))


def quant_error(sim_time):
    """Temporal quantization error (s)."""
    return sim_time - tick_of(sim_time) * TICK


def omnet_to_sumo(ox, oy, xmin, ymax, margin=25.0):
    """OMNeT++/Veins (x,y) -> SUMO net (x,y). Inverted y axis + margin."""
    return ox + xmin - margin, ymax + margin - oy


def ang_shortest_delta(a, b):
    """b-a difference by the shortest arc, in (-180, 180]."""
    return (b - a + 180) % 360 - 180


def ang_lerp(a, b, frac):
    """Circular heading interpolation (short arc)."""
    return (a + frac * ang_shortest_delta(a, b)) % 360


# --------------------------------------------------------------------------- #
# Shared tracker configuration
# --------------------------------------------------------------------------- #
class TrackerConfig:
    def __init__(self, period_s, buffer_s, delay_s, xmin, ymax,
                 margin=25.0, reacq_snap_th=3.0, interp="linear",
                 lost_after_s=3.0):
        self.period_s = period_s
        self.buffer_s = buffer_s
        self.delay_s = delay_s
        self.delay_ticks = int(round(delay_s / TICK))
        self.xmin = xmin
        self.ymax = ymax
        self.margin = margin
        self.reacq_snap_th = reacq_snap_th
        self.interp = interp
        # EXTRAPOLATION WINDOW: under silence, UNCERTAIN is extrapolated until
        # declaring it LOST at lost_after_s; then removal (the only termination
        # mechanism; it does not distinguish the cause of the silence). The
        # period+buffer floor guarantees that a silence shorter than the nominal
        # cadence itself never removes. The value (3 s default) is a threshold:
        # it will come with a sensitivity analysis for the paper, not set by eye.
        self.lost_after_s = lost_after_s
        self.timeout_s = max(lost_after_s, period_s + buffer_s)


# --------------------------------------------------------------------------- #
# Incremental per-vehicle tracker
# --------------------------------------------------------------------------- #
class VehicleTracker:
    """Incremental state of a vehicle: it only knows what arrived (via the guard).

    The grid of nominal slots is anchored at t0 = simTime of the first message
    (same as the base: reconstruct_nominal). Slot k represents the instant
    tk = t0 + k*period and is decided with window tk + buffer.
    """

    def __init__(self, first_msg, cfg):
        self.veh = first_msg.veh
        self.cfg = cfg
        self.t0 = first_msg.t
        self.vtype = first_msg.vtype
        self.last_arrival = first_msg.t
        self.last_lane = first_msg.lane
        self.last_edge = first_msg.edge
        self.norm_edges = []            # normal edges of ARRIVED messages
        self._note_edge(first_msg.edge)
        self.recon = {}                 # k -> reconstructed row (OMNeT coords)
        self.states = {}                # k -> SUMO state ready for moveToXY
        self._next_decide = 0
        self.snap_events = []
        self.retired = False
        self.retire_tick = None

    # ------------------------------------------------------------ arrivals
    def on_message(self, msg):
        """Registers an arrival. Returns True if it contributes a new normal edge."""
        self.last_arrival = msg.t
        self.last_lane = msg.lane
        self.last_edge = msg.edge
        return self._note_edge(msg.edge)

    def _note_edge(self, e):
        # same rule as the base: only normal edges, consecutive dedup
        if e.startswith(":"):
            return False
        if self.norm_edges and self.norm_edges[-1] == e:
            return False
        self.norm_edges.append(e)
        return True

    def timed_out(self, t):
        """Causal timeout: strict silence greater than (period + buffer)."""
        return t - self.last_arrival > self.cfg.timeout_s + 1e-9

    # ------------------------------------------------------------ grid
    def slot_time(self, k):
        return self.t0 + k * self.cfg.period_s

    def app_tick(self, k):
        """Application tick of slot k: grid + delay (same as the base:
        tick_of(simTime) + delay_ticks over the reconstructed row)."""
        return tick_of(self.slot_time(k)) + self.cfg.delay_ticks

    def _slot_of(self, msg):
        return round((msg.t - self.t0) / self.cfg.period_s)

    # ------------------------------------------------------------ decision
    def decide_up_to(self, k, store):
        """Sequentially decides the pending slots up to k inclusive (the
        snap needs the previous slot already decided, as in the base's
        sequential pass)."""
        for kk in range(self._next_decide, k + 1):
            self._decide(kk, store)

    def _decide(self, k, store):
        cfg = self.cfg
        tk = self.slot_time(k)
        view = store.slot_view(self.veh, tk)
        half = cfg.period_s / 2.0

        obs = view.msg_near(tk, half)
        if obs is not None and self._slot_of(obs) != k:
            obs = None

        if obs is not None:
            # OBSERVED / REACQUIRED: same rule as the base (slot k-1 empty)
            prev_msg = view.msg_near(tk - cfg.period_s, half)
            prev_ok = prev_msg is not None and self._slot_of(prev_msg) == k - 1
            prov = OBSERVED if (k == 0 or prev_ok) else REACQUIRED
            row = {"t": tk, "x": obs.x, "y": obs.y, "h": obs.h, "spd": obs.spd,
                   "acc": obs.acc, "edge": obs.edge, "lane": obs.lane,
                   "vtype": obs.vtype, "prov": prov, "snap": False,
                   "vx": 0.0, "vy": 0.0, "max_msg_ts": obs.t}
        else:
            oa = view.last_at_or_before(tk)      # last real past one (exists: k>0)
            ob = view.next_within_window(tk)     # closing anchor WITHIN the buffer
            if ob is not None:
                # INFERRED — interpolation between two real ones (base formula)
                a = (tk - oa.t) / (ob.t - oa.t)
                row = {"t": tk,
                       "x": oa.x + a * (ob.x - oa.x),
                       "y": oa.y + a * (ob.y - oa.y),
                       "h": ang_lerp(oa.h, ob.h, a),
                       "spd": oa.spd + a * (ob.spd - oa.spd),
                       "acc": 0.0, "edge": oa.edge, "lane": oa.lane,
                       "vtype": oa.vtype, "prov": INFERRED, "snap": False,
                       "vx": 0.0, "vy": 0.0, "max_msg_ts": ob.t}
            else:
                # UNCERTAIN — constant-velocity dead-reckoning, only the past.
                # It also covers the final silence tail: with no closing anchor,
                # it is extrapolated like any loss (leak 3 corrected).
                op = view.prev_before(oa)
                vx = vy = 0.0
                if op is not None:
                    dt = oa.t - op.t
                    vx, vy = (oa.x - op.x) / dt, (oa.y - op.y) / dt
                d = tk - oa.t
                row = {"t": tk, "x": oa.x + vx * d, "y": oa.y + vy * d,
                       "h": oa.h, "spd": oa.spd, "acc": 0.0,
                       "edge": oa.edge, "lane": oa.lane, "vtype": oa.vtype,
                       "prov": UNCERTAIN, "snap": False,
                       "vx": vx, "vy": vy, "max_msg_ts": oa.t}

        # mixed reacquisition policy (copied from the base): divergence
        # between the real one and the continued extrapolation when closing an
        # UNCERTAIN zone
        prev = self.recon.get(k - 1)
        if prev is not None and prev["prov"] == UNCERTAIN and row["prov"] != UNCERTAIN:
            dt = row["t"] - prev["t"]
            pred_x = prev["x"] + prev["vx"] * dt
            pred_y = prev["y"] + prev["vy"] * dt
            diverg = math.hypot(row["x"] - pred_x, row["y"] - pred_y)
            if diverg > cfg.reacq_snap_th:
                row["snap"] = True
                if row["prov"] == REACQUIRED:
                    row["prov"] = REACQUIRED_SNAP
                self.snap_events.append({"veh": self.veh, "t": round(row["t"], 4),
                                         "diverg_m": round(diverg, 3),
                                         "into": row["prov"]})

        self.recon[k] = row
        sx, sy = omnet_to_sumo(row["x"], row["y"], cfg.xmin, cfg.ymax, cfg.margin)
        self.states[k] = {
            "tick": self.app_tick(k),
            "sim_time_csv": tk,
            "quant_error_s": quant_error(tk),
            "sx": sx, "sy": sy, "angle": row["h"],
            "speed_csv": row["spd"], "accel_csv": row["acc"],
            "edge_csv": row["edge"], "lane_csv": row["lane"],
            "provenance": row["prov"], "snap": row["snap"],
            "alpha": None, "seg": None,
            "max_msg_ts": row["max_msg_ts"],
        }
        self._next_decide = k + 1

    # ------------------------------------------------------------ interp
    def frame_between(self, k, tick_i):
        """INTERPOLATED state at a tick between app_tick(k) and app_tick(k+1)
        (base build_schedule interp='linear' formula: linear in position,
        circular in heading, hold if the closing state comes with snap).

        CAUSAL FRAME-FILL POLICY (DR-until-usable-anchor): if the
        closing state embeds
        a message newer than (frame instant) + buffer, the frame is
        EXTRAPOLATED by dead-reckoning with the PAST velocity (positions of
        the two last decided slots) — the car advances smoothly instead of
        freezing and jumping. When the anchor enters the window, the frame
        returns to the normal interpolation (convergence adjustment ≤ the HELD
        jump). It is just as causal as freezing: the decision compares only the
        embedded TIMESTAMP against the window (the knowledge of the arrival
        clock) and the content of the extrapolated frame derives solely from the
        past. With no previous slot (k=0) it degrades to hold (unknown velocity:
        it is not invented)."""
        sa, sb = self.states[k], self.states[k + 1]
        ta, tb = sa["tick"], sb["tick"]
        gap = tb - ta
        a = (tick_i - ta) / gap
        s_frame = tick_i * TICK - self.cfg.delay_s   # represented instant
        dr = sb["max_msg_ts"] > s_frame + self.cfg.buffer_s + 1e-9
        seg = f"{sa['provenance']}->{sb['provenance']}"
        if sb["snap"]:
            # flagged snap: base policy preserved (hold and jump at tb)
            sx, sy, angle, alpha = sa["sx"], sa["sy"], sa["angle"], 0.0
            seg += "(SNAP)"
        elif dr:
            prev = self.states.get(k - 1)
            dt = (tick_i - ta) * TICK
            if prev is not None and ta > prev["tick"]:
                dtp = (ta - prev["tick"]) * TICK
                vx = (sa["sx"] - prev["sx"]) / dtp
                vy = (sa["sy"] - prev["sy"]) / dtp
            else:
                vx = vy = 0.0
            sx, sy = sa["sx"] + vx * dt, sa["sy"] + vy * dt
            angle, alpha = sa["angle"], 0.0
            seg += "(DR)"
        else:
            sx = sa["sx"] + a * (sb["sx"] - sa["sx"])
            sy = sa["sy"] + a * (sb["sy"] - sa["sy"])
            angle = ang_lerp(sa["angle"], sb["angle"], a)
            alpha = a
        return {
            "tick": tick_i,
            "sim_time_csv": None, "quant_error_s": None,
            "sx": sx, "sy": sy, "angle": angle,
            "speed_csv": sa["speed_csv"] + a * (sb["speed_csv"] - sa["speed_csv"]),
            "accel_csv": sa["accel_csv"] + a * (sb["accel_csv"] - sa["accel_csv"]),
            "edge_csv": sa["edge_csv"], "lane_csv": sa["lane_csv"],
            "provenance": INTERPOLATED, "snap": False,
            "alpha": alpha, "seg": seg, "held_frame": dr,
            # honest transitive accounting: extrapolated -> only the past
            "max_msg_ts": (sa["max_msg_ts"] if dr
                           else max(sa["max_msg_ts"], sb["max_msg_ts"])),
        }

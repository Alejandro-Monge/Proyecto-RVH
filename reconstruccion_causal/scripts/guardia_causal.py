"""
guardia_causal — the only access path to the messages of the causal reconstructor.

"Batch with guard" pattern (analogous to the causality probe of the vision
shadow): the full CSV is in memory (comparability over the sweep), but all
consumption passes through this guard, which verifies the causality criterion:

    To decide the state of a vehicle at instant t, only messages with
    timestamp <= t + BUFFER (BUFFER = 1 s) may be used.

Two access modes, both watched:

1. Clock-based delivery (`deliver_until(t)`): ARRIVAL events — a message with
   timestamp m "arrives" at m (instantaneous emission; the rx-sim latency
   measured in the sweep is <= 1 ms). It feeds vehicle creation, route extension
   and removal timeout: those decisions only see already-arrived messages
   (implicit window = clock, stricter than t+buffer).

2. Per-decision window (`slot_view(veh, t_state)`): reconstruction of the state
   at instant t_state. The view may TOUCH only messages with
   timestamp <= t_state + buffer; any access beyond raises
   CausalityViolation (assert mode) or counts it (audit mode). The "peek"
   needed to know whether the next message falls within the window happens
   INSIDE the guard (trust boundary): it is equivalent to the knowledge that
   live is given by closing the window by clock, and the consumer never sees the
   content of a message outside the window.

Transitive accounting: each reconstructed state carries max_msg_ts (the
newest timestamp that influenced it). When a state is used to compose
another state at an instant s (e.g. an interpolated frame), `check_state_use`
verifies max_msg_ts <= s + buffer. Thus, "using a future anchor through an
intermediate state" is also caught — it is the watched invariant that governs
the linear interpolation without a bound inherited from the base.

Documented limit: the guard watches ACCESSES to messages. It cannot see leaks of
negative information (knowing that a message does NOT exist before the window
closes); that case is analyzed in the module documentation.
"""

import bisect

EPS = 1e-9


class CausalityViolation(AssertionError):
    """Access to a message (direct or transitive) beyond t + buffer."""


class Msg:
    """V2X message parsed once. Immutable by convention."""
    __slots__ = ("veh", "t", "rx", "x", "y", "h", "spd", "acc", "edge", "lane", "vtype")

    def __init__(self, row):
        self.veh = row["vehicleId"]
        self.t = float(row["simTime"])       # instant of emission = instant of arrival
        self.rx = float(row.get("rxTime", row["simTime"]))
        self.x = float(row["x"])
        self.y = float(row["y"])
        self.h = float(row["headingDeg"])
        self.spd = float(row["speed"])
        self.acc = float(row["acceleration"])
        self.edge = row["edgeId"]
        self.lane = row["laneId"]
        self.vtype = (row.get("vType") or row.get("vtype") or "").strip().lower()


class GuardedMessageStore:
    """Owns all the messages; exposes them only under watch."""

    def __init__(self, rows, buffer_s, mode="assert"):
        if mode not in ("assert", "audit"):
            raise ValueError(f"modo de guardia desconocido: {mode}")
        self.buffer_s = buffer_s
        self.mode = mode
        self.clock = float("-inf")

        msgs = [Msg(r) for r in rows]
        # global delivery by arrival instant (= simTime, instantaneous emission)
        self._delivery = sorted(msgs, key=lambda m: m.t)
        self._delivered = 0
        # index per vehicle, sorted by simTime (monotonicity verified separately)
        self._by_veh = {}
        for m in self._delivery:
            self._by_veh.setdefault(m.veh, []).append(m)
        self._times_by_veh = {v: [m.t for m in ms] for v, ms in self._by_veh.items()}

        self.stats = {
            "accesses": 0,
            "max_lookahead_s": float("-inf"),   # max(msg.t - t_decision) over accesses
            "violations": 0,
            "violation_examples": [],           # first N for the report
        }

    # ------------------------------------------------------------------ clock
    def set_clock(self, t):
        if t < self.clock - EPS:
            raise ValueError(f"el reloj no puede retroceder: {self.clock} -> {t}")
        self.clock = t

    def deliver_until(self, t):
        """Messages with arrival <= t not yet delivered, in arrival order."""
        if t > self.clock + EPS:
            raise ValueError("deliver_until por delante del reloj: set_clock primero")
        out = []
        while self._delivered < len(self._delivery) and \
                self._delivery[self._delivered].t <= t + EPS:
            out.append(self._delivery[self._delivered])
            self._delivered += 1
        return out

    def exhausted(self):
        """True if no messages remain to deliver (loop termination; it does not
        intervene in any reconstruction decision)."""
        return self._delivered >= len(self._delivery)

    def first_arrival_time(self):
        """Instant of the first global arrival. ONLY to set SUMO's --begin
        (setup/termination, analogous to the trivial --end): it does not feed
        any state decision."""
        return self._delivery[0].t if self._delivery else 0.0

    # ------------------------------------------------------- accounting
    def _record(self, msg_t, t_decision, ctx):
        self.stats["accesses"] += 1
        look = msg_t - t_decision
        if look > self.stats["max_lookahead_s"]:
            self.stats["max_lookahead_s"] = look
        if msg_t > t_decision + self.buffer_s + EPS:
            self.stats["violations"] += 1
            if len(self.stats["violation_examples"]) < 20:
                self.stats["violation_examples"].append(
                    {"ctx": ctx, "t_decision": round(t_decision, 4),
                     "msg_t": round(msg_t, 4),
                     "exceso_s": round(msg_t - (t_decision + self.buffer_s), 4)})
            if self.mode == "assert":
                raise CausalityViolation(
                    f"[GUARDIA] {ctx}: decisión en t={t_decision:.4f} consultó "
                    f"timestamp {msg_t:.4f} > t+buffer={t_decision + self.buffer_s:.4f}")

    def check_state_use(self, max_msg_ts, t_state, ctx):
        """Transitive use: an already-reconstructed state (which embeds messages
        up to max_msg_ts) is used to compose the state at instant t_state."""
        self._record(max_msg_ts, t_state, ctx)

    def slot_view(self, veh, t_state):
        return _DecisionView(self, veh, t_state)

    def guard_report(self):
        s = self.stats
        return {
            "mode": self.mode,
            "buffer_s": self.buffer_s,
            "accesses": s["accesses"],
            "max_lookahead_s": (round(s["max_lookahead_s"], 6)
                                if s["accesses"] else None),
            "violations": s["violations"],
            "violation_examples": s["violation_examples"],
            "status": "PASS" if s["violations"] == 0 else "FAIL",
        }


class _DecisionView:
    """View of the messages of ONE vehicle to decide the state at t_state.
    Every returned message passed through the guard with window t_state + buffer."""

    def __init__(self, store, veh, t_state):
        self._store = store
        self._msgs = store._by_veh.get(veh, [])
        self._times = store._times_by_veh.get(veh, [])
        self.veh = veh
        self.t_state = t_state
        self._W = t_state + store.buffer_s

    def _touch(self, msg, ctx):
        self._store._record(msg.t, self.t_state, ctx)
        return msg

    def msg_near(self, t_target, tol):
        """Message with |t - t_target| <= tol, or None. (Search for the message
        of the slot itself; its timestamp is within the window by construction,
        but it still passes through the guard.)"""
        i = bisect.bisect_left(self._times, t_target - tol)
        if i < len(self._msgs) and abs(self._msgs[i].t - t_target) <= tol:
            return self._touch(self._msgs[i], "msg_near")
        return None

    def next_within_window(self, t_after):
        """First message with t > t_after IF its timestamp falls within the
        window (<= t_state + buffer); if the next message exists but is outside
        the window, it returns None WITHOUT exposing it (internal peek: it is the
        same knowledge that live is given by closing the window by clock)."""
        i = bisect.bisect_right(self._times, t_after + EPS)
        if i >= len(self._msgs):
            return None
        cand = self._msgs[i]
        if cand.t <= self._W + EPS:
            return self._touch(cand, "next_within_window")
        return None

    def last_at_or_before(self, t_ref):
        """Last message with t <= t_ref (past; t_ref must be <= window)."""
        i = bisect.bisect_right(self._times, t_ref + EPS) - 1
        if i < 0:
            return None
        return self._touch(self._msgs[i], "last_at_or_before")

    def prev_before(self, msg):
        """Message preceding a given one (past)."""
        i = bisect.bisect_left(self._times, msg.t - EPS) - 1
        if i < 0:
            return None
        return self._touch(self._msgs[i], "prev_before")

    # --- ONLY for the negative control (--leak-probe): positional access that
    # non-causal logic would attempt; the guard must catch it. Do not use in the
    # reconstructor's logic. ---
    def peek_absolute(self, idx, ctx="leak_probe"):
        return self._touch(self._msgs[idx], ctx)

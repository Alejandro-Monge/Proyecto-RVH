"""MessagesEmitter — DETERMINISTIC side of the WS emitter (architecture B).

Lives in the player thread and connects the three closed pieces:
DecidedTickView → step_message() → canonical_bytes() → CANONICAL LOG on
disk (synchronous, --dense-out pattern) → put_nowait to the thread queue (server.py).

INVARIABLE order of emit_step: map → canonicalize → WRITE LOG → enqueue. The log is
written BEFORE the put: a drop due to a full queue (Full → dropped += 1, never blocks)
affects only the network stream, never the reproducible truth that layer 3 audits.
t_emit does NOT exist in this module: the thread injects it (wire_frame).

HEADING CONVERSION (contract Section 3.6): the SUMO vehicle angle is in NAVIGATIONAL
degrees — official moveToXY doc (TraCI/Change_Vehicle_State): "The angle value is
assumed to be in navigational degrees (between 0 and 360 with 0 at the top, going
clockwise)"; after the mapping the vehicle assumes that angle "for drawing and all
outputs" (same page), and VAR_ANGLE 0x43 returns "the angle of the named vehicle
within the last step [°]" (Vehicle Value Retrieval). Contract Section 3.6: radians,
0 = +x (east), counterclockwise, [0, 2π). Formula: rad = radians((90 − deg) mod 360).
Table pinned in tests/test_ws_emitter.py.

Emitter errors are COUNTED and reported (emit_errors), they never take the player
down: the observation layer cannot kill the reconstruction (passivity). A corrupt log
is not silent: the counter goes to the summary and to the 'end'.
"""
from __future__ import annotations

import math
import queue as queue_mod

from ws_emitter.canonical import CONTRACT_VERSION, canonical_bytes
from ws_emitter.server import WsBroadcastServer

_HEADING_CONVERTERS = {
    # view label -> conversion to the contract convention Section 3.6
    "sumo_angle_post_step": lambda deg: math.radians((90.0 - float(deg)) % 360.0),
    # lane_angle (vision boxes): lane_angle_towards_stop returns arctan2(dy,dx) in
    # world radians, which IS the contract convention Section 3.6 (rad, 0=+x east, CCW —
    # "the same as lane_angle_towards_stop"). Only the range is normalized to [0, 2π).
    "lane_angle": lambda rad: float(rad) % (2.0 * math.pi),
}


def heading_to_contract(heading_raw) -> float:
    """(value, source_label) → contract radians. Unknown label = error: better to
    blow up the emitter (counted) than to emit a lying orientation."""
    value, tag = heading_raw
    try:
        conv = _HEADING_CONVERTERS[tag]
    except KeyError:
        raise ValueError(f"convención de heading desconocida: {tag!r}")
    return conv(value)


def _occupancy_message(occ) -> dict:
    """Branch by `kind` (Section 3.4). sumo_lanes: per_lane {approach:{lane:{ocupacion,
    detenidos}}} (messages, INTACT). pixel_zones: per_zone {zone:{lane:count}} +
    semantics (vision, RAW COUNT)."""
    kind = str(occ["kind"])
    if kind == "sumo_lanes":
        return {"kind": "sumo_lanes",
                "per_lane": {b: {c: dict(v) for c, v in lanes.items()}
                             for b, lanes in occ["per_lane"].items()}}
    if kind == "pixel_zones":
        return {"kind": "pixel_zones", "semantics": str(occ["semantics"]),
                "per_zone": {z: {c: int(n) for c, n in lanes.items()}
                             for z, lanes in occ["per_zone"].items()}}
    raise ValueError(f"occupancy.kind desconocido: {kind!r}")


def step_message(view) -> dict:
    """DecidedTickView → dict of the v3 contract `step` message (Section 3.2/Section 3.3).
    PURE function: it touches neither disk, nor queue, nor TraCI. Serializes the THREE
    sources; in messages box/handoff arrive () and add nothing → IDENTICAL bytes
    (no regression)."""
    entities = []
    for e in view.sumo_entities:
        entities.append({
            "id": e.id, "source": e.source, "provenance": e.provenance,
            "pos_plane": [float(e.pos_plane[0]), float(e.pos_plane[1])],
            "heading": heading_to_contract(e.heading_raw),
            "vclass": e.vclass, "generation": int(e.generation),
        })
    for e in view.box_entities:                 # VISION ONLY (empty in messages)
        entities.append({
            "id": e.id, "source": e.source, "provenance": e.provenance,
            "pos_plane": [float(e.pos_plane[0]), float(e.pos_plane[1])],
            "heading": heading_to_contract(e.heading_raw),
            "anchor": e.anchor,
            "carril": (None if e.carril is None else int(e.carril)),
            "vclass": e.vclass,
        })
    for e in view.handoff_entities:             # VISION ONLY: closed keys, no pos/heading
        entities.append({
            "id": e.id, "source": e.source, "provenance": e.provenance,
            "last_pos": {"x": float(e.last_pos[0]), "y": float(e.last_pos[1]),
                         "t": float(e.last_pos[2])},
            "vclass": e.vclass,
        })
    return {"type": "step", "seq": int(view.seq), "t_sim": float(view.t_sim),
            "entities": entities, "occupancy": _occupancy_message(view.occupancy),
            "guard": dict(view.guard_counters)}


class MessagesEmitter:
    """Emitter of the messages profile. Creates (if ws_port) the queue and the server
    thread; the player only calls emit_hello / emit_step / emit_end / close."""

    def __init__(self, *, log_path: str, hello_fields: dict,
                 ws_host: str = "127.0.0.1", ws_port: int = 0,
                 queue_max: int = 1024, stall_drain_s: float = 0.0):
        self._log = open(log_path, "wb")
        # the hello is canonicalized NOW (immutable session state): the server retains
        # it from its construction and delivers it per connection (retained hello);
        # emit_hello() only writes it to the log in its turn
        self._hello_data = canonical_bytes(
            {"type": "hello", "schema_version": CONTRACT_VERSION,
             **dict(hello_fields)})
        self._q = None
        self._server = None
        if ws_port:
            self._q = queue_mod.Queue(maxsize=int(queue_max))
            self._server = WsBroadcastServer(ws_host, int(ws_port), self._q,
                                             stall_drain_s=stall_drain_s,
                                             hello_bytes=self._hello_data).start()
        self.steps = 0
        self.dropped = 0
        self.emit_errors = 0
        self._last_seq = None
        self._hello_sent = False
        self._end_sent = False

    # ------------------------------------------------------------- internal
    def _write_and_enqueue(self, msg: dict):
        data = canonical_bytes(msg)      # validates (t_emit forbidden, NaN, types)
        self._log.write(data)            # THE TRUTH: always before enqueuing
        self._log.flush()                # per line: a crash does not truncate the truth
                                         # that layer 3 audits (trivial cost ~10 Hz)
        if self._q is not None:
            try:
                self._q.put_nowait(data)
                self._server.notify()
            except queue_mod.Full:
                self.dropped += 1        # only the network loses; the log is already there

    # ------------------------------------------------------------- interface
    def emit_hello(self):
        """BEFORE the first simulationStep (verifiable: first line of the log).

        RETAINED HELLO: the hello is NOT enqueued/broadcast — the server retains it
        from its construction and each client receives it in its on-connect handler,
        synchronous and atomic with the registration (connecting before/during/after
        no longer degenerates into "step before hello"; a single path = no duplicate-
        hello race; atomic registration = no race with the first step). The canonical
        log does not change: same line, same position."""
        assert not self._hello_sent, "hello ya emitido"
        self._log.write(self._hello_data)
        self._log.flush()
        self._hello_sent = True

    def emit_step(self, view):
        try:
            self._write_and_enqueue(step_message(view))
            self.steps += 1
            self._last_seq = int(view.seq)
        except Exception:
            self.emit_errors += 1        # counted and reported; the player continues

    def emit_end(self, *, guard_final: dict, status: str, detail: str = ""):
        """AFTER the last step (verifiable: last line of the log).

        BEST-EFFORT DELIVERY of the end on the wire: the steps are ephemeral and
        droppable, but the end carries THE VERDICT — with the queue saturated at close
        it fell off the wire and an honest client was left SIN_VEREDICTO. Here put with
        a bounded timeout is used: this runs in the player's finally, AFTER finishing
        the simulation (traci already closed) — waiting ≤2 s touches no dynamics
        (passivity intact; the log was already written before, as always). If it still
        does not fit, it is counted as dropped: honest, not silent."""
        if self._end_sent:
            return
        msg = {"type": "end",
               "seq_final": self._last_seq if self._last_seq is not None else -1,
               "sonda": {"status": str(status), "detail": str(detail)},
               "guard_final": dict(guard_final),
               "totals": {"steps": self.steps, "dropped": self.dropped,
                          "emit_errors": self.emit_errors, "handoffs": 0}}
        data = canonical_bytes(msg)
        self._log.write(data)
        self._log.flush()
        if self._q is not None:
            try:
                self._q.put(data, timeout=2.0)   # post-simulation: no dynamics
                self._server.notify()
            except queue_mod.Full:
                self.dropped += 1
        self._end_sent = True

    def close(self) -> dict:
        """Closes log and server; returns the report for the player's summary."""
        try:
            self._log.close()
        except Exception:
            self.emit_errors += 1
        server_stats = None
        if self._q is not None:
            try:
                self._q.put(None, timeout=2.0)   # sentinel: post-simulation,
            except queue_mod.Full:               # same logic as the end
                pass                             # join with timeout covers it
            server_stats = self._server.close()
        return {"steps": self.steps, "dropped": self.dropped,
                "emit_errors": self.emit_errors, "server": server_stats}

"""Tests for the messages emitter (`ws_emitter/emitter.py` + `server.py`) — STEP 2.3.

Includes the PINNED TABLE of the heading conversion with its official source:
moveToXY doc (TraCI/Change_Vehicle_State): "The angle value is assumed to be in
navigational degrees (between 0 and 360 with 0 at the top, going clockwise)"; the
vehicle assumes that angle for drawing and outputs, and VAR_ANGLE 0x43 returns it
("angle of the named vehicle within the last step [°]", Vehicle Value Retrieval).
Contract Section 3.6: radians, 0 = +x (east), counterclockwise, [0, 2π).

pytest tests/test_ws_emitter.py from the implementacion/ root.
"""
from __future__ import annotations

import json
import math
import queue as queue_mod
import socket
import sys
import threading
import time
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ws_emitter.canonical import canonical_bytes  # noqa: E402
from ws_emitter.emitter import (  # noqa: E402
    MessagesEmitter,
    heading_to_contract,
    step_message,
)
from ws_emitter.view import VAR_ANGLE, VAR_POSITION, DecidedTickView  # noqa: E402

HELLO_FIELDS = {
    "shadow": "messages", "mode": "replay", "dt": 0.1, "site": "rusia",
    "frame_geometry": None, "net_md5": "a" * 32,
    "artifact_md5": {"mensajes_csv": "b" * 32},
    "occ_cfg_md5": {"occupancy_yaml": "c" * 32},
    "guard_mode": "assert", "zone_coverage": None, "layers": ["sumo"],
    "vclass_dims": {"car": {"length_m": 5.0, "width_m": 1.8}},
    "occupancy_semantics": {"kind": "sumo_lanes", "halting_speed_mps": 0.1},
}


def _view(seq=7, ang=90.0):
    return DecidedTickView.from_messages_tick(
        seq=seq, t_sim=seq * 0.1, active={"veh_1"}, parked=set(),
        last_applied={"veh_1": {"provenance": "OBSERVED", "held_frame": False}},
        applied_ids={"veh_1"},
        subs={"veh_1": {VAR_POSITION: (10.0, 20.0), VAR_ANGLE: ang}},
        generation={"veh_1": 0}, assigned_type={"veh_1": "shadow_car"},
        occupancy_counts={("norte", 1): {"ocupacion": 1, "detenidos": 0}},
        guard_report={"mode": "assert", "accesses": 5, "violations": 0,
                      "max_lookahead_s": 0.4})


# --------------------------------------------------------------------------- #
# Heading conversion: pinned table with the citation (module docstring)
def test_heading_tabla_pineada():
    # navigation (0=north, clockwise) -> contract (rad, 0=east, counterclockwise)
    assert heading_to_contract((0.0, "sumo_angle_post_step")) == pytest.approx(math.pi / 2)    # N
    assert heading_to_contract((90.0, "sumo_angle_post_step")) == pytest.approx(0.0)           # E
    assert heading_to_contract((180.0, "sumo_angle_post_step")) == pytest.approx(3 * math.pi / 2)  # S
    assert heading_to_contract((270.0, "sumo_angle_post_step")) == pytest.approx(math.pi)      # W
    assert heading_to_contract((450.0, "sumo_angle_post_step")) == pytest.approx(0.0)          # mod 360
    for deg in (0.0, 37.5, 90.0, 180.0, 270.0, 359.9):
        r = heading_to_contract((deg, "sumo_angle_post_step"))
        assert 0.0 <= r < 2 * math.pi
    with pytest.raises(ValueError):
        heading_to_contract((0.0, "convencion_desconocida"))


# --------------------------------------------------------------------------- #
# view -> contract step message mapping
def test_step_message_mapeo():
    m = step_message(_view(seq=42, ang=90.0))
    assert m["type"] == "step" and m["seq"] == 42 and m["t_sim"] == pytest.approx(4.2)
    (e,) = m["entities"]
    assert e == {"id": "veh_1", "source": "sumo", "provenance": "OBSERVED",
                 "pos_plane": [10.0, 20.0], "heading": pytest.approx(0.0),
                 "vclass": "car", "generation": 0}
    assert m["occupancy"] == {"kind": "sumo_lanes",
                              "per_lane": {"norte": {"1": {"ocupacion": 1,
                                                           "detenidos": 0}}}}
    assert m["guard"]["violations"] == 0
    assert "t_emit" not in json.dumps(m)
    canonical_bytes(m)                     # canonicalizes without error


# --------------------------------------------------------------------------- #
# hello/end: structure, ORDER in the log, and t_emit absent from the log
def test_log_orden_hello_steps_end(tmp_path):
    log = tmp_path / "ws.jsonl"
    em = MessagesEmitter(log_path=str(log), hello_fields=HELLO_FIELDS)
    em.emit_hello()
    em.emit_step(_view(seq=1))
    em.emit_step(_view(seq=2))
    em.emit_end(guard_final={"violations": 0, "accesses": 9}, status="PASS")
    em.close()
    lines = log.read_bytes().splitlines()
    msgs = [json.loads(l) for l in lines]
    assert msgs[0]["type"] == "hello" and msgs[0]["schema_version"] == "3.0.0"
    assert msgs[0]["frame_geometry"] is None          # messages profile
    assert [m["seq"] for m in msgs if m["type"] == "step"] == [1, 2]
    assert msgs[-1]["type"] == "end"
    assert msgs[-1]["sonda"]["status"] == "PASS"
    assert msgs[-1]["seq_final"] == 2
    assert b"t_emit" not in log.read_bytes()          # t_emit is never persisted in the log
    # double end is a no-op; a double hello blows up
    em.emit_end(guard_final={}, status="PASS")
    with pytest.raises(AssertionError):
        em.emit_hello()


# --------------------------------------------------------------------------- #
# Drop due to full queue: counted, non-blocking, COMPLETE log
def test_descarte_cola_llena(tmp_path):
    log = tmp_path / "ws.jsonl"
    em = MessagesEmitter(log_path=str(log), hello_fields=HELLO_FIELDS)
    # minimal queue and a "server" that never drains (notify no-op): pure adversary
    em._q = queue_mod.Queue(maxsize=1)
    em._server = types.SimpleNamespace(notify=lambda: None,
                                       close=lambda timeout=5.0: {"sent": 0},
                                       stats=lambda: {})
    for k in range(5):
        em.emit_step(_view(seq=k))
    assert em.dropped == 4                 # 1 got in, 4 dropped
    assert em.steps == 5                   # all emitted to the LOG
    assert len(log.read_bytes().splitlines()) == 5   # the truth loses nothing
    rep = em.close()
    assert rep["dropped"] == 4 and rep["emit_errors"] == 0


# --------------------------------------------------------------------------- #
# Integration round-trip: real server + websockets client, FIFO order,
# t_emit on the wire, wire without t_emit == canonical log line
def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def test_roundtrip_ws(tmp_path):
    import asyncio
    from websockets.asyncio.client import connect

    port = _free_port()
    log = tmp_path / "ws.jsonl"
    em = MessagesEmitter(log_path=str(log), hello_fields=HELLO_FIELDS,
                         ws_port=port)
    frames = []

    def client():
        async def run():
            async with connect(f"ws://127.0.0.1:{port}") as ws:
                async for fr in ws:
                    d = json.loads(fr)
                    frames.append(d)
                    if d.get("type") == "end":
                        return
        asyncio.run(run())

    th = threading.Thread(target=client, daemon=True)
    th.start()
    deadline = time.time() + 5
    while not em._server._conns and time.time() < deadline:
        time.sleep(0.02)                   # wait for the connection before emitting
    assert em._server._conns, "el cliente de prueba no conectó"
    em.emit_hello()
    em.emit_step(_view(seq=1))
    em.emit_step(_view(seq=2))
    em.emit_end(guard_final={"violations": 0}, status="PASS")
    th.join(10)
    rep = em.close()
    assert not th.is_alive()
    assert [d["type"] for d in frames] == ["hello", "step", "step", "end"]
    assert [d["seq"] for d in frames if d["type"] == "step"] == [1, 2]  # FIFO
    assert all("t_emit" in d for d in frames)          # the wire DOES carry it
    # wire without t_emit re-canonicalized == the corresponding line of the canonical log
    log_lines = log.read_bytes().splitlines()
    for d, line in zip(frames, log_lines):
        d2 = dict(d)
        del d2["t_emit"]
        assert canonical_bytes(d2) == line + b"\n"
    # sent counts what was drained from the queue (steps+end = 3): the hello goes
    # via the per-connection retained path, not via broadcast/queue
    assert rep["server"]["sent"] == 3 and rep["dropped"] == 0

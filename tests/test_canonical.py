"""Tests for the canonical serializer (`ws_emitter/canonical.py`) — STEP 2.0.

The 4 tests cover:
  1. direct idempotency;
  2. equivalence between different constructions + pinned rounding BOUNDARIES
     (the guarantee is determinism, not exact decimal precision);
  3. rejections (t_emit, NaN/Inf) + byte-by-byte round-trip of wire_frame;
  4. stability ACROSS PROCESSES with crossed PYTHONHASHSEED (safeguard against the
     hash-seed class of bug).
Plus the PIN of the frozen contract: if ws_payload_contract.md mutates without
bumping CONTRACT_VERSION, the test fails (silent mutation = visible break).

pytest or `python -m pytest tests/test_canonical.py` from the implementacion/ root.
"""
from __future__ import annotations

import hashlib
import json
import math
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ws_emitter.canonical import (  # noqa: E402
    CONTRACT_DOC,
    CONTRACT_MD5,
    CanonicalError,
    canonical_bytes,
    wire_frame,
)


def _step_msg():
    """Representative step message of the v3 contract (messages profile)."""
    return {
        "type": "step",
        "seq": 42,
        "t_sim": 4.2,
        "entities": [
            {"id": "veh_7", "source": "sumo", "provenance": "OBSERVED",
             "pos_plane": [57.123456, 74.4], "heading": 1.57079632,
             "vclass": "car", "generation": 0},
            {"id": "veh_12", "source": "sumo", "provenance": "HELD",
             "pos_plane": [12.0005, -3.25], "heading": 0.0,
             "vclass": "bus", "generation": 1},
        ],
        "occupancy": {"kind": "sumo_lanes",
                      "per_lane": {"norte": {"1": {"ocupacion": 2, "detenidos": 1}}}},
        "guard": {"accesses": 100, "violations": 0, "max_lookahead_s": -0.9},
    }


# --------------------------------------------------------------------------- #
# 1. Direct idempotency
def test_idempotencia_directa():
    m = _step_msg()
    b1, b2 = canonical_bytes(m), canonical_bytes(m)
    assert b1 == b2
    assert b1.endswith(b"\n")
    json.loads(b1)                       # is valid JSON


# --------------------------------------------------------------------------- #
# 2. Equivalence between different constructions + rounding boundaries
def test_equivalencia_construcciones():
    a = _step_msg()
    # same information, different insertion order, entities reversed and floats via
    # different arithmetic paths that quantize the same
    b = {
        "guard": {"max_lookahead_s": -0.9, "violations": 0, "accesses": 100},
        "occupancy": {"per_lane": {"norte": {"1": {"detenidos": 1, "ocupacion": 2}}},
                      "kind": "sumo_lanes"},
        "entities": list(reversed([
            {"generation": 0, "vclass": "car", "heading": 1.57079632,
             "pos_plane": [57.123456, 74.4], "provenance": "OBSERVED",
             "source": "sumo", "id": "veh_7"},
            {"generation": 1, "vclass": "bus", "heading": 0.1 + 0.2 - 0.3,  # ~5.5e-17 -> "0.0000"
             "pos_plane": [12.0005, -13.25 + 10.0], "provenance": "HELD",
             "source": "sumo", "id": "veh_12"},
        ])),
        "t_sim": 4.2, "seq": 42, "type": "step",
    }
    # heading of veh_12: 0.0 in `a`, ~5.5e-17 in `b` -> both quantize to "0.0000"
    assert canonical_bytes(a) == canonical_bytes(b)


def test_fronteras_redondeo_pineadas():
    """PIN of the boundary behavior: rounding over the BINARY (fixed format).
    The guarantee is determinism; the direction at the boundary depends on the real double:
    0.0005 and 0.0015 round UP, 1.0005 rounds DOWN, 2.675 to 2 dec = 2.67."""
    def pos_token(x):
        b = canonical_bytes({"type": "t", "e": {"pos_plane": [x, 0.0]}})
        return json.loads(b)  # returns the already-quantized value

    assert pos_token(0.0005)["e"]["pos_plane"][0] == 0.001    # rounds up (binary > half)
    assert pos_token(0.0015)["e"]["pos_plane"][0] == 0.002    # rounds up
    assert pos_token(1.0005)["e"]["pos_plane"][0] == 1.0      # rounds DOWN (binary < half)
    assert f"{2.675:.2f}" == "2.67"                           # the classic one, documented
    # -0.0 and negatives that collapse to zero stay UNSIGNED
    raw = canonical_bytes({"type": "t", "e": {"pos_plane": [-0.0, -0.0004]}})
    assert b"-0.000" not in raw
    assert json.loads(raw)["e"]["pos_plane"] == [0.0, 0.0]
    # fixed width: 74.4 is serialized as "74.400" (textual stability)
    raw2 = canonical_bytes({"type": "t", "e": {"pos_plane": [74.4, 0.0]}})
    assert b"74.400" in raw2
    # rule 5: config floats / no-rule floats go via exact repr (dt is NOT quantized)
    raw3 = canonical_bytes({"type": "hello", "dt": 12 / 355})
    assert repr(12 / 355).encode() in raw3


# --------------------------------------------------------------------------- #
# 3. Rejections + wire_frame byte-by-byte round-trip
def test_rechazos():
    with pytest.raises(CanonicalError):
        canonical_bytes({"type": "step", "t_emit": 1.0})          # t_emit at root
    with pytest.raises(CanonicalError):
        canonical_bytes({"type": "step", "meta": {"t_emit": 1.0}})  # nested t_emit
    with pytest.raises(CanonicalError):
        canonical_bytes({"type": "step", "x": float("nan")})
    with pytest.raises(CanonicalError):
        canonical_bytes({"type": "step", "e": {"pos_plane": [float("inf"), 0.0]}})
    with pytest.raises(CanonicalError):
        canonical_bytes({})                                        # empty


def test_wire_frame_round_trip():
    m = _step_msg()
    cb = canonical_bytes(m)
    w = wire_frame(cb, 123.456789)
    # (i) the frame differs from the canonical one EXACTLY in the inserted segment
    assert w == b'{"t_emit":123.456789,' + cb[1:]
    # (ii) is valid JSON and carries t_emit
    d = json.loads(w)
    assert d["t_emit"] == 123.456789
    # (iii) re-canonicalized (removing t_emit) == original canonical, byte by byte
    del d["t_emit"]
    assert canonical_bytes(d) == cb
    # (iv) non-finite t_emit is also rejected on the wire
    with pytest.raises(CanonicalError):
        wire_frame(cb, math.inf)


# --------------------------------------------------------------------------- #
# 4. Stability across processes (crossed PYTHONHASHSEED)
_CHILD = r"""
import hashlib, sys
sys.path.insert(0, {root!r})
from ws_emitter.canonical import canonical_bytes
m = {{
  "type": "step", "seq": 7, "t_sim": 0.7,
  "entities": [
    {{"id": "b", "source": "sumo", "pos_plane": [1.0005, -0.0], "heading": 0.12345}},
    {{"id": "a", "source": "sumo", "pos_plane": [0.0015, 2.675], "heading": 0.0}},
  ],
  "guard": {{"violations": 0}},
}}
print(hashlib.md5(canonical_bytes(m)).hexdigest())
"""


def test_estabilidad_entre_procesos():
    md5s = []
    for seed in ("1", "20260704"):
        r = subprocess.run(
            [sys.executable, "-c", _CHILD.format(root=str(ROOT))],
            capture_output=True, text=True,
            env={"PYTHONHASHSEED": seed, "PATH": "/usr/bin:/bin"})
        assert r.returncode == 0, r.stderr
        md5s.append(r.stdout.strip())
    assert md5s[0] == md5s[1] and len(md5s[0]) == 32


# --------------------------------------------------------------------------- #
# PIN of the frozen contract (safeguard)
def test_pin_contrato_congelado():
    """If the contract mutates without bumping CONTRACT_VERSION, this test BREAKS on purpose:
    the serializer is tied to the frozen contract."""
    got = hashlib.md5((ROOT / CONTRACT_DOC).read_bytes()).hexdigest()
    assert got == CONTRACT_MD5, (
        f"ws_payload_contract.md cambió (md5 {got} != pin {CONTRACT_MD5}) sin subir "
        f"CONTRACT_VERSION en ws_emitter/canonical.py — mutación no autorizada del "
        f"contrato congelado o falta de re-pineo aprobado")

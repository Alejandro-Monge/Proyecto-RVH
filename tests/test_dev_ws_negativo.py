"""The NEGATIVE CONTROL is verified too: dev_ws_negativo must emit
EXACTLY the agreed sequences per case — a scaffolding server that lies
would invalidate the observation of the stream.js state machine.

Uses the threading/sync websockets client implementation against the real
server in a subprocess. pytest tests/test_dev_ws_negativo.py.
"""
from __future__ import annotations

import json
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ws_emitter.dev_ws_negativo import CASES  # noqa: E402


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _collect(port, n_expected, timeout=5.0):
    from websockets.sync.client import connect
    frames = []
    with connect(f"ws://127.0.0.1:{port}", open_timeout=timeout) as ws:
        try:
            while True:
                frames.append(ws.recv(timeout=timeout))
        except Exception:
            pass    # server close = end of the sequence
    return frames


@pytest.mark.parametrize("caso", sorted(CASES))
def test_secuencia_pactada(caso):
    port = _free_port()
    proc = subprocess.Popen(
        [sys.executable, "-m", "ws_emitter.dev_ws_negativo",
         "--port", str(port), "--caso", caso],
        cwd=str(ROOT), stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        deadline = time.time() + 5
        while time.time() < deadline:
            try:
                frames = _collect(port, len(CASES[caso]))
                break
            except (ConnectionRefusedError, OSError):
                time.sleep(0.1)
        else:
            pytest.fail("el servidor de control negativo no levantó")
        pactada = CASES[caso]
        got = [f if isinstance(f, bytes) else f.encode() for f in frames]
        assert got == list(pactada), f"{caso}: secuencia distinta a la pactada"
        # and the properties each case must have to exercise its transition
        if caso == "step-primero":
            assert json.loads(got[0])["type"] == "step"
        elif caso == "schema-malo":
            assert json.loads(got[0])["schema_version"] != "3.0.0"
        elif caso == "hello-doble":
            assert [json.loads(f)["type"] for f in got] == ["hello", "hello"]
        elif caso == "json-roto":
            with pytest.raises(ValueError):
                json.loads(got[0])
        elif caso == "cierre-sin-end":
            types = [json.loads(f)["type"] for f in got]
            assert types[0] == "hello" and "end" not in types
    finally:
        proc.terminate()
        proc.wait(timeout=5)

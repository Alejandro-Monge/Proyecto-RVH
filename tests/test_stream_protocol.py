"""AUTOMATED VERIFICATION of the 7 stream.js cases (real integration).

Runtime: node >= 22 (global WebSocket) runs the REAL stream.js MACHINE via
tests/stream_harness.mjs — the same file the client serves, no port or mock.
Servers: the REAL emitter (causal_player with --ws-port) for cases 1/2/7 and
dev_ws_negativo for 3/4/6 (5 uses the negative one as the hello + expect source).

Duration: ~3 player runs (~2-4 min). Case 7 also INSTITUTIONALIZES
passivity: the validation_log.csv of the adversarial run must remain
byte-identical to baseline 2.1 (pinned md5).

pytest tests/test_stream_protocol.py -v
"""
from __future__ import annotations

import json
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
HARNESS = ROOT / "tests" / "stream_harness.mjs"
CSV = ROOT / "reconstruccion_causal" / "inputs" / "cert" / "mc_2Hz_P00.csv"
NET = ROOT / "reconstruccion_causal" / "inputs" / "intersection_rusia.net.xml"
PLAYER = ROOT / "reconstruccion_causal" / "scripts" / "causal_player.py"
VALIDATION_LOG_MD5 = "1c49a6c02e7d2b8b6318abc3c252b909"   # baseline 2.1


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _player(tmp, port, extra=()):
    out = tmp / "run"
    out.mkdir(parents=True, exist_ok=True)
    # stdout to a FILE: a PIPE with no reader can block SUMO/player
    log = open(tmp / "player_out.txt", "wb")
    return subprocess.Popen(
        [sys.executable, str(PLAYER), "--csv", str(CSV), "--net", str(NET),
         "--out", str(out), "--period-s", "0.5",
         "--occupancy-out", str(out / "occ.csv"),
         "--ws-log", str(out / "ws_log.jsonl"), "--ws-port", str(port),
         *extra],
        cwd=str(ROOT / "reconstruccion_causal"),
        stdout=log, stderr=subprocess.STDOUT)


def _wait_steps(log_path, n, timeout_s=60):
    """Synchronization by log CONTENT (per-line flush), not by sleeps:
    the full run lasts ~10 s (~180 steps/s) — any blind sleep
    lands in the wrong place (root cause of the first batch of failures)."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            if Path(log_path).read_bytes().count(b'"type":"step"') >= n:
                return
        except FileNotFoundError:
            pass
        time.sleep(0.1)
    raise AssertionError(f"el log no alcanzó {n} steps en {timeout_s}s")


def _harness(port, expect=None, timeout_s=240):
    cmd = ["node", str(HARNESS), f"url=ws://127.0.0.1:{port}",
           f"timeoutMs={timeout_s * 1000}"]
    if expect:
        cmd.append(f"expect={expect}")
    r = subprocess.run(cmd, capture_output=True, text=True,
                       timeout=timeout_s + 30, cwd=str(ROOT / "tests"))
    assert r.returncode in (0, 1), r.stderr
    return json.loads(r.stdout.strip().splitlines()[-1])


def _negativo(caso):
    port = _free_port()
    proc = subprocess.Popen(
        [sys.executable, "-m", "ws_emitter.dev_ws_negativo",
         "--port", str(port), "--caso", caso],
        cwd=str(ROOT), stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return proc, port


def _log_first_step_seq(log_path):
    for line in Path(log_path).read_bytes().splitlines():
        d = json.loads(line)
        if d["type"] == "step":
            return d["seq"]
    return None


# --------------------------------------------------------------------------- #
def test_caso1_feliz_con_cliente_TARDIO(tmp_path):
    """Retained hello: connecting AFTER the first step must still give
    hello -> STREAMING -> ENDED PASS; and enter 'at the now' (earlier steps
    are not re-emitted: firstSeq > first seq of the log)."""
    port = _free_port()
    p = _player(tmp_path, port)
    try:
        # LATE client guaranteed by CONTENT: ≥300 steps already emitted
        _wait_steps(tmp_path / "run" / "ws_log.jsonl", 300)
        rep = _harness(port)
        p.wait(timeout=240)
    finally:
        p.kill()
    assert rep["state"] == "ENDED", rep
    assert rep["hello"] is not None and rep["end"]["status"] == "PASS"
    first_log = _log_first_step_seq(tmp_path / "run" / "ws_log.jsonl")
    assert rep["stats"]["firstSeq"] > first_log, \
        "el tardío debería entrar al AHORA (steps pasados no se re-emiten)"
    assert rep["stats"]["received"] < rep["end"]["totals"]["steps"]


def test_caso2_cierre_sin_end(tmp_path):
    port = _free_port()
    p = _player(tmp_path, port)
    h = subprocess.Popen(
        ["node", str(HARNESS), f"url=ws://127.0.0.1:{port}", "timeoutMs=60000"],
        cwd=str(ROOT / "tests"), stdout=subprocess.PIPE, text=True)
    try:
        # STREAMING in progress guaranteed by content (not by clock): ≥150
        # steps emitted and ~1700 remain ahead (the run lasts ~10 s)
        _wait_steps(tmp_path / "run" / "ws_log.jsonl", 150)
        p.send_signal(signal.SIGKILL)       # sudden death: end never arrives
        out, _ = h.communicate(timeout=60)
        rep = json.loads(out.strip().splitlines()[-1])
    finally:
        p.kill()
        h.kill()
    assert rep["state"] == "SIN_VEREDICTO", rep
    assert "sin end" in rep["reason"]
    assert rep["end"] is None and rep["stats"]["received"] > 0


@pytest.mark.parametrize("caso,estado,frag", [
    ("step-primero", "RECHAZADO", "antes de hello"),     # case 3
    ("schema-malo", "RECHAZADO", "schema_version"),      # case 4
    ("hello-doble", "RECHAZADO", "hello duplicado"),     # case 6a
    ("json-roto", "RECHAZADO", "JSON inválido"),         # case 6b
])
def test_casos_negativos(caso, estado, frag):
    proc, port = _negativo(caso)
    try:
        rep = _harness(port, timeout_s=30)
    finally:
        proc.terminate()
        proc.wait(timeout=5)
    assert rep["state"] == estado, rep
    assert frag in rep["reason"], rep


def test_caso5_net_md5_mismatch_via_reject():
    """The reject() path that main.js will use: hello arrives, the expected md5
    does not match -> RECHAZADO (hello source: the cierre-sin-end negative)."""
    proc, port = _negativo("cierre-sin-end")
    try:
        rep = _harness(port, expect="deadbeef" * 4, timeout_s=30)
    finally:
        proc.terminate()
        proc.wait(timeout=5)
    assert rep["state"] == "RECHAZADO", rep
    assert "net_md5" in rep["reason"] and "esperado" in rep["reason"]


def test_caso7_wiregaps_coherencia_y_pasividad(tmp_path):
    """Adversarial (queue 2 + slow drain): wireGaps > 0 and the EXACT IDENTITY
    received == end.totals.steps - end.totals.dropped (wireGaps>0 is not enough —
    the accounting of the two layers must add up).
    Requires a complete client from the start: firstSeq == first seq of the
    log is checked BEFORE requiring the identity (if it lost the head, the test
    FAILS with a diagnostic, it does not tolerate silently). Also: institutionalized
    passivity (validation_log byte-identical to baseline 2.1 despite the emitter+client)."""
    import hashlib
    port = _free_port()
    # adversity calibrated to the MEASURED rates (~1400 steps/s of streaming in
    # ~1.3 s): drain at ~500/s (stall 2 ms/item) with queue 4 => massive drops
    # WITHOUT saturating the event loop to the point of not flushing; and 3 s of grace
    # after hello so the client is registered BEFORE the first step
    # (without grace, the first step goes out ms after opening the port: no
    # client arrives "from the start" — a finding of this verification)
    p = _player(tmp_path, port,
                extra=("--ws-queue-max", "4", "--ws-stall-drain", "0.002",
                       "--ws-open-grace", "3"))
    try:
        rep = _harness(port)                # connects during the grace
        p.wait(timeout=240)
    finally:
        p.kill()
    assert rep["state"] == "ENDED", rep
    first_log = _log_first_step_seq(tmp_path / "run" / "ws_log.jsonl")
    assert rep["stats"]["firstSeq"] == first_log, (
        "el cliente perdió la cabeza del stream: la identidad exacta no es "
        f"evaluable (firstSeq={rep['stats']['firstSeq']} vs log={first_log})")
    t = rep["end"]["totals"]
    assert rep["stats"]["wireGaps"] > 0, "la cola nunca se saturó: caso no adverso"
    assert rep["stats"]["received"] == t["steps"] - t["dropped"], (
        f"DESAJUSTE de contabilidad entre capas: received={rep['stats']['received']} "
        f"!= steps({t['steps']}) - dropped({t['dropped']})")
    got = hashlib.md5(
        (tmp_path / "run" / "results" / "validation_log.csv").read_bytes()).hexdigest()
    assert got == VALIDATION_LOG_MD5, "PASIVIDAD ROTA: validation_log difiere del baseline"

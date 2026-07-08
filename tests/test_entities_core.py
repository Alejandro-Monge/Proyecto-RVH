"""Automated verification of the entities module (3D client front-end):

1. The reconciliation CORE (entities_core.js) headless in node — asserts for
   create/move/remove, generation→recreate, layers, unknown vclass and the
   no-orphans-no-leaks INVARIANT under 500 pseudorandom steps.
2. The REALTIME pacing (--ws-realtime, the only mode): passivity
   (raw validation_log == realtime, same artifact) and anchoring to t_sim
   (drift ≤ 2 ms, the stage quantization).

The THREE glue (entities.js) is validated by visual inspection.
pytest tests/test_entities_core.py -v
"""
from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VALIDATION_LOG_MD5 = "1c49a6c02e7d2b8b6318abc3c252b909"   # baseline 2.1


def test_core_reconciliacion_headless():
    r = subprocess.run(["node", str(ROOT / "tests" / "entities_core_check.mjs")],
                       capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, f"core FALLÓ:\n{r.stdout}\n{r.stderr}"
    assert "OK:" in r.stdout


def _run_player(csv, out, extra=()):
    out.mkdir(parents=True, exist_ok=True)
    r = subprocess.run(
        [sys.executable,
         str(ROOT / "reconstruccion_causal" / "scripts" / "causal_player.py"),
         "--csv", str(csv),
         "--net", str(ROOT / "reconstruccion_causal/inputs/intersection_rusia.net.xml"),
         "--out", str(out), "--period-s", "0.5",
         "--occupancy-out", str(out / "occ.csv"),
         "--ws-log", str(out / "ws.jsonl"), *extra],
        cwd=str(ROOT / "reconstruccion_causal"),
        capture_output=True, text=True, timeout=300)
    assert r.returncode == 0, r.stderr[-1500:]
    return out


def test_realtime_pasividad_y_anclaje(tmp_path):
    """--ws-realtime (the only pacing mode): (a) PASSIVITY — the
    validation_log of the realtime run is byte-identical to that of the raw
    run of the SAME artifact; (b) ANCHORING TO t_sim — the wall-clock between the
    first and last emitted step equals the elapsed t_sim (drift ≤ 2 ms, the
    stage quantization declared in contract Section 3.2). Truncated artifact
    (~20 s of t_sim) so the test lasts ~half a minute; the full-duration acid
    test runs separately."""
    sys.path.insert(0, str(ROOT))
    from ws_emitter.prefix_audit import simtime_span, truncate_csv
    full = ROOT / "reconstruccion_causal/inputs/cert/mc_2Hz_P00.csv"
    lo, _hi = simtime_span(str(full))
    short = tmp_path / "corto.csv"
    truncate_csv(str(full), lo + 20.0, str(short))

    crudo = _run_player(short, tmp_path / "crudo")
    rt = _run_player(short, tmp_path / "rt", extra=("--ws-realtime",))

    a = (crudo / "results" / "validation_log.csv").read_bytes()
    b = (rt / "results" / "validation_log.csv").read_bytes()
    assert a == b, "PASIVIDAD ROTA: realtime altera la dinámica"

    ws = json.load(open(rt / "results" / "summary.json"))["ws_emitter"]
    assert ws["realtime"] is True and ws["pace_sleep_s"] > 5.0, \
        f"realtime no durmió (¿ancla rota?): {ws}"
    assert abs(ws["realtime_drift_s"]) <= 0.002, \
        f"drift {ws['realtime_drift_s']} s > cuantización 2 ms"
    # and the raw run did NOT sleep (realtime is the only mode, no free factor)
    ws0 = json.load(open(crudo / "results" / "summary.json"))["ws_emitter"]
    assert ws0["realtime"] is False and ws0["pace_sleep_s"] == 0.0

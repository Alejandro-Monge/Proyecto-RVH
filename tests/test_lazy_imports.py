"""Verified LAZY import (mode 5, non-negotiable constraint): modes 1-4
and the CERTIFICATION load nothing web — neither `websockets` nor `ws_emitter`
enter sys.modules when the player runs without ws flags. Verified by running the
REAL player (full main(), short truncated artifact) and asserting on
sys.modules at the end — it is not a docstring promise.

pytest tests/test_lazy_imports.py -v
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

_CHILD = r'''
import sys
sys.path.insert(0, r"{root}")
from ws_emitter.prefix_audit import truncate_csv, simtime_span
# truncar ANTES de tocar al player; luego limpiar el rastro de ws_emitter que
# ESTE wrapper introdujo (el sujeto del test es el PLAYER, no el wrapper)
full = r"{root}/reconstruccion_causal/inputs/cert/mc_2Hz_P00.csv"
lo, _ = simtime_span(full)
truncate_csv(full, lo + 10.0, r"{tmp}/corto.csv")
for m in [m for m in list(sys.modules) if m == "ws_emitter" or m.startswith("ws_emitter.")]:
    del sys.modules[m]

sys.path.insert(0, r"{root}/reconstruccion_causal/scripts")
sys.argv = ["causal_player.py", "--csv", r"{tmp}/corto.csv",
            "--net", r"{root}/reconstruccion_causal/inputs/intersection_rusia.net.xml",
            "--out", r"{tmp}/out", "--delay-s", "1.0", "--period-s", "0.5",
            "--interp", "linear", "--guard-mode", "assert"]
import causal_player
causal_player.main()
web = [m for m in sys.modules
       if m == "websockets" or m.startswith("websockets.")
       or m == "ws_emitter" or m.startswith("ws_emitter.")]
assert not web, f"la certificacion cargo modulos web: {{web}}"
print("LAZY_OK")
'''


def test_certificacion_no_carga_web(tmp_path):
    child = _CHILD.format(root=str(ROOT), tmp=str(tmp_path))
    r = subprocess.run([sys.executable, "-c", child],
                       cwd=str(ROOT / "reconstruccion_causal"),
                       capture_output=True, text=True, timeout=300)
    assert r.returncode == 0, r.stderr[-2000:]
    assert "LAZY_OK" in r.stdout

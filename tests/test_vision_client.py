"""VISION client (3D front-end): automated gates. The THREE glue
(render of the three sources) is validated by visual inspection; here what is
decidable by code:

1. The reconcile with the three sources headless (node): handoff without pos_plane
   does not break, source change → recreate, anchor/lane carried over.
2. NON-REGRESSION of messages: the messages reconcile (entities_core_check.mjs)
   still passes with the source-aware core (existing module intact).

pytest tests/test_vision_client.py -v
"""
from __future__ import annotations

import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_reconcile_tres_fuentes_headless():
    r = subprocess.run(["node", str(ROOT / "tests" / "vision_core_check.mjs")],
                       capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, f"reconcile visión FALLÓ:\n{r.stdout}\n{r.stderr}"
    assert "OK:" in r.stdout


def test_no_regresion_reconcile_mensajes():
    # the same core, with messages (LAYERS=['sumo'], all pos_plane): still green
    r = subprocess.run(["node", str(ROOT / "tests" / "entities_core_check.mjs")],
                       capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, f"reconcile MENSAJES regresó:\n{r.stdout}\n{r.stderr}"
    assert "OK:" in r.stdout


def test_escena_vision_cuadra_con_su_net():
    # the client REJECTS the stream if scene.net_md5 != hello.net_md5. The vision
    # shadow runs on a DIFFERENT net than messages → it serves its own scene. Verify that
    # the vision scene was derived from the vision net (same net_md5 as the hello).
    import hashlib
    import json

    import yaml
    web3d = yaml.safe_load((ROOT / "config/sites/rusia/web3d.yaml").read_text())
    scene = json.loads((ROOT / "ws_emitter/web" / web3d["vision_scene"]).read_text())
    vnet = ROOT / "config/sites/rusia/sumo/intersection_rusia.net.xml"
    net_md5 = hashlib.md5(vnet.read_bytes()).hexdigest()
    assert scene["net_md5"] == net_md5, \
        "la escena de visión no derivó del net de visión → el cliente rechazaría"
    # and it is NOT the messages scene (different nets, check that they were not mixed up)
    msg_scene = json.loads((ROOT / "ws_emitter/web" / web3d["scene"]).read_text())
    assert msg_scene["net_md5"] != net_md5, "visión y mensajes comparten net_md5 (¿misma escena?)"

"""LANZADOR MULTI-CÁMARA — Fusión Cinemática + Car-Following en Punto Ciego (Cuenca).

Lee exclusivamente `verify_out/events/merged/track_events_merged.csv` y coordina en TraCI:
  - Vehículos fusionados (`global_*`): Cam 1 (percepción) -> punto ciego (`334948975#2`,
    `334948975#3` en Car-Following SUMO) -> Cam 2 (resincronización y retiro).
  - Huérfanos (`c1_*`, `c2_*`): inyección y retiro local en su intersección correspondiente.

Uso:
  python3 scripts/run_multicam_harness.py            # LIVE (sumo-gui viendo ambas intersecciones + ffplay)
  python3 scripts/run_multicam_harness.py --web3d    # VISOR 3D WEB (HTTP + WebSocket + TraCI)
  python3 scripts/run_multicam_harness.py --split    # video compuesto (Cam1/Cam2 | SUMO Multi-Cámara)
  python3 scripts/run_multicam_harness.py --readout  # volcado headless de trayectorias en SUMO
"""
from __future__ import annotations

import argparse
import atexit
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

AVISO = """
======================================================================
 GEMELO DIGITAL MULTI-CAMARA (Cuenca Oeste + Cuenca)
 Fusión Cinemática + Car-Following en Punto Ciego
======================================================================
"""


def _run_web3d(args, veh) -> None:
    """Inicia servidor HTTP estático + emisor WebSocket y transmite todos los vehículos al visor 3D."""
    import yaml

    from demo_causal.run_multicam_causal import OUT_DIR, multicam_hello_fields, run_live
    from ws_emitter.emitter import MessagesEmitter

    cfg = yaml.safe_load((ROOT / "config/sites/cuenca_multicam/web3d.yaml").read_text())
    http_host = str(os.environ.get("HTTP_HOST", cfg["http_host"]))
    http_port = int(os.environ.get("HTTP_PORT", cfg["http_port"]))
    ws_host = str(os.environ.get("WS_HOST", cfg["ws_host"]))
    ws_port = int(os.environ.get("WS_PORT", cfg["ws_port"]))
    scene = cfg["scene"]
    grace = float(os.environ.get("GRACE", args.grace))

    out = OUT_DIR / "web3d_multicam"
    out.mkdir(parents=True, exist_ok=True)

    web = subprocess.Popen(
        [sys.executable, "-m", "ws_emitter.webserver", "--site", "cuenca_multicam",
         "--http-host", http_host, "--http-port", str(http_port),
         "--ws-host", ws_host, "--ws-port", str(ws_port), "--scene", scene],
        cwd=str(ROOT),
    )
    ws_em = None

    def _cleanup(*_a):
        try:
            web.terminate()
        except Exception:
            pass
        if ws_em is not None:
            try:
                ws_em.close()
            except Exception:
                pass

    atexit.register(_cleanup)
    for _s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(_s, lambda *a: (_cleanup(), sys.exit(0)))

    time.sleep(1.0)
    browser_host = "localhost" if http_host in ("0.0.0.0", "127.0.0.1") else http_host
    print("=" * 66)
    print(f">>> VISOR WEB3D MULTI-CÁMARA:  http://{browser_host}:{http_port}/index.html")
    print(f">>> WebSocket stream en:       ws://{ws_host}:{ws_port}")
    print(f">>> Tienes {grace:.0f}s (gracia) para abrir el navegador antes de iniciar")
    print("=" * 66, flush=True)

    # Bind WS en 0.0.0.0 para aceptar conexiones del navegador desde la máquina host (Docker)
    ws_em = MessagesEmitter(
        log_path=str(out / "ws_log.jsonl"),
        hello_fields=multicam_hello_fields(args.merged_csv),
        ws_host="0.0.0.0",
        ws_port=ws_port,
    )
    try:
        time.sleep(grace)
        run_live(veh, ws_em=ws_em, gui=not args.no_gui, play_videos=not args.no_gui)
    finally:
        _cleanup()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--merged-csv",
        type=Path,
        default=ROOT / "verify_out/events/merged/track_events_merged.csv",
        help="Ruta al CSV unificado de eventos multi-cámara",
    )
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--web3d", "--3d", dest="web3d", action="store_true",
                      help="lanza el servidor HTTP + WebSocket del visor Web3D en segundo plano")
    mode.add_argument("--split", action="store_true", help="renderiza video compuesto en vez de vivo")
    mode.add_argument("--readout", action="store_true", help="volcado headless de posiciones TraCI")
    ap.add_argument("--grace", type=float, default=15.0,
                    help="segundos de espera para abrir el navegador en --web3d (default: 15)")
    ap.add_argument("--no-gui", action="store_true",
                    help="con --web3d: corre SUMO headless y sin ventanas ffplay (solo visor 3D web)")
    args = ap.parse_args()

    print(AVISO)
    os.environ.setdefault("SUMO_HOME", "/usr/share/sumo")
    os.environ.setdefault("DISPLAY", ":1")

    from demo_causal.run_multicam_causal import (
        build_multicam_vehicles,
        run_live,
        run_readout,
        run_split,
    )

    veh = build_multicam_vehicles(args.merged_csv)
    n_global = sum(1 for v in veh.values() if v["status"] == "matched")
    n_c1 = sum(1 for v in veh.values() if v["status"] == "orphan_cam1")
    n_c2 = sum(1 for v in veh.values() if v["status"] == "orphan_cam2")
    print(f"[0] Eventos cargados: {len(veh)} vehículos "
          f"({n_global} globales, {n_c1} huérfanos Cam 1, {n_c2} huérfanos Cam 2)")

    if args.readout:
        run_readout(veh)
    elif args.split:
        run_split(veh)
    elif args.web3d:
        _run_web3d(args, veh)
    else:
        run_live(veh)


if __name__ == "__main__":
    main()

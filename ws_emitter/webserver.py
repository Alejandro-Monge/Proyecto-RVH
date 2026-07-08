"""webserver — static HTTP for the 3D client (mode 5). Pure stdlib, a process SEPARATE
from the player: it does not import traci, it does not touch the simulation (passivity
by process isolation). Serves ws_emitter/web/ and exposes /config.json GENERATED from
config/sites/<site>/web3d.yaml (single source: the YAML; the browser consumes it by
fetch — index.html with no query params or hardcode).

Usage (launched by vista_dos_ventanas_3d.sh; also useful for manual inspection):
  python -m ws_emitter.webserver --site rusia
  python -m ws_emitter.webserver --site rusia --http-port 8310 --ws-port 8941
"""
from __future__ import annotations

import argparse
import functools
import json
import sys
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WEB = ROOT / "ws_emitter" / "web"


def load_cfg(site: str) -> dict:
    import yaml
    with open(ROOT / "config" / "sites" / site / "web3d.yaml") as f:
        return yaml.safe_load(f)


class Handler(SimpleHTTPRequestHandler):
    """Static + /config.json (the YAML config exposed to the browser)."""
    config_payload: bytes = b"{}"

    def do_GET(self):
        if self.path.split("?")[0] == "/config.json":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(self.config_payload)))
            self.end_headers()
            self.wfile.write(self.config_payload)
            return
        super().do_GET()

    def log_message(self, fmt, *args):   # silent except for errors
        if args and str(args[1]).startswith(("4", "5")):
            super().log_message(fmt, *args)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--site", default="rusia")
    ap.add_argument("--http-host", default=None)
    ap.add_argument("--http-port", type=int, default=None)
    ap.add_argument("--ws-host", default=None)
    ap.add_argument("--ws-port", type=int, default=None)
    ap.add_argument("--scene", default=None,
                    help="escena a servir en /config.json (override; visión usa la suya "
                         "derivada de su net, distinto del de mensajes)")
    args = ap.parse_args()
    cfg = load_cfg(args.site)
    http_host = args.http_host or cfg["http_host"]
    http_port = args.http_port or int(cfg["http_port"])
    ws_host = args.ws_host or cfg["ws_host"]
    ws_port = args.ws_port or int(cfg["ws_port"])
    scene = args.scene or cfg["scene"]

    Handler.config_payload = json.dumps({
        "site": cfg["site"],
        "ws_url": f"ws://{ws_host}:{ws_port}",
        "scene": scene,
        "presentation_3d": cfg.get("presentation_3d", {}),   # cosmetic (net_scene applies it)
    }).encode()

    handler = functools.partial(Handler, directory=str(WEB))
    srv = ThreadingHTTPServer((http_host, http_port), handler)
    print(f"[web3d] sirviendo {WEB}", flush=True)
    print(f"[web3d] URL: http://{http_host}:{http_port}/index.html", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    sys.exit(main())

"""SCAFFOLDING / NEGATIVE CONTROL — WS server that serves deliberately BROKEN
sequences of the v3 protocol, to exercise the REJECT transitions of stream.js (the
honest emitter never produces them). Precedent: --leak-probe, _mutant_player.

It is NOT part of the client or the emitter; only observable validation scaffolding.
The pytest tests/test_dev_ws_negativo.py verifies that each case emits EXACTLY the
agreed sequence (the scaffolding server can also lie).

Usage:  python -m ws_emitter.dev_ws_negativo --port 8940 --caso step-primero
Cases: step-primero | schema-malo | hello-doble | json-roto | cierre-sin-end
Each connection receives the case's sequence (BINARY frames, like the real emitter)
and closes. The server serves connections indefinitely (Ctrl-C to exit).
"""
from __future__ import annotations

import argparse
import asyncio
import json
import sys


def _j(d: dict) -> bytes:
    return json.dumps(d).encode()


def _hello(schema="3.0.0", net_md5="f" * 32) -> bytes:
    return _j({"type": "hello", "schema_version": schema, "shadow": "messages",
               "mode": "replay", "dt": 0.1, "site": "rusia",
               "frame_geometry": None, "net_md5": net_md5,
               "artifact_md5": {}, "occ_cfg_md5": {}, "guard_mode": "assert",
               "zone_coverage": None, "layers": ["sumo"],
               "vclass_dims": {"car": {"length_m": 5.0, "width_m": 1.8}},
               "occupancy_semantics": {"kind": "sumo_lanes"}})


def _step(seq: int) -> bytes:
    return _j({"type": "step", "seq": seq, "t_sim": round(seq * 0.1, 4),
               "t_emit": 1.0, "entities": [], "guard": {"violations": 0},
               "occupancy": {"kind": "sumo_lanes", "per_lane": {}}})


# AGREED sequences per case (the pytest verifies them as-is)
CASES = {
    "step-primero": [_step(1)],
    "schema-malo": [_hello(schema="2.9.9")],
    "hello-doble": [_hello(), _hello()],
    "json-roto": [b"{esto no es json"],
    "cierre-sin-end": [_hello(), _step(1), _step(2), _step(3)],
}


async def _serve(port: int, caso: str):
    from websockets.asyncio.server import serve

    async def handler(ws):
        for frame in CASES[caso]:
            await ws.send(frame)
        await ws.close()

    async with serve(handler, "127.0.0.1", port):
        print(f"[dev_ws_negativo] caso={caso} en ws://127.0.0.1:{port} "
              f"({len(CASES[caso])} frames por conexión)", flush=True)
        await asyncio.Future()      # serves until Ctrl-C


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, required=True)
    ap.add_argument("--caso", choices=sorted(CASES), required=True)
    args = ap.parse_args()
    try:
        asyncio.run(_serve(args.port, args.caso))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    sys.exit(main())

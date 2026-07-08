"""ws_replay_server — MINIMAL WebSocket server that replays a ws_log for the
SPOT-CHECK: it feeds the LIVE client (index.html) with the same stream it
received during the run, to compare its render against the offline replay (puppeteer).

It is NOT production nor part of the pipeline: it is verification tooling. It speaks the same
wire as the real emitter (BINARY frames of UTF-8 JSON; hello on-connect). It sends hello
+ steps with seq ≤ --stop-seq and then HOLDS the connection (does not send 'end' → the client
stays STREAMING at the last step = stop-seq, ready to capture).

  python ws_replay_server.py --ws-log <log> --port 8971 --stop-seq 2124
"""
from __future__ import annotations

import argparse
import asyncio
import json

import websockets


def load(ws_log: str, stop_seq: int):
    hello, steps = None, []
    for line in open(ws_log, encoding="utf-8"):
        line = line.strip()
        if not line:
            continue
        m = json.loads(line)
        if m.get("type") == "hello":
            hello = line.encode("utf-8")
        elif m.get("type") == "step" and int(m["seq"]) <= stop_seq:
            steps.append(line.encode("utf-8"))
    return hello, steps


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ws-log", required=True)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, required=True)
    ap.add_argument("--stop-seq", type=int, required=True)
    ap.add_argument("--pace-ms", type=float, default=0.0,
                    help="ms entre steps; 0=lo más rápido. Con >0 el cliente vivo NO salta "
                         "(reproduce el realtime de la demo, donde render>step ⇒ sin skip).")
    args = ap.parse_args()
    hello, steps = load(args.ws_log, args.stop_seq)
    if hello is None:
        raise SystemExit("ws_log sin hello")

    async def handler(ws):
        await ws.send(hello)                     # hello on-connect (like the real emitter)
        for b in steps:
            await ws.send(b)                     # binary steps, in order
            if args.pace_ms > 0:
                await asyncio.sleep(args.pace_ms / 1000.0)
        await asyncio.Future()                   # HOLD (no 'end'): client stays at stop-seq

    print(f"[replay] :{args.port} hello + {len(steps)} steps (≤seq {args.stop_seq}); mantiene", flush=True)
    async with websockets.serve(handler, args.host, args.port):
        await asyncio.Future()


if __name__ == "__main__":
    asyncio.run(main())

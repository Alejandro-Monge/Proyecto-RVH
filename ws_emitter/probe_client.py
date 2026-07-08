"""probe_client — DIAGNOSTIC client of the WS emitter (step 2.3). It is NOT the
Three.js client nor a demo deliverable: it verifies reception, order and the presence
of t_emit on the wire, and serves as the ADVERSARY in the passivity test
(--no-consume: it connects and never reads — the laggard that websockets disconnects
by ping_timeout, doc 16.0 topics/broadcast).

Usage:
  python -m ws_emitter.probe_client --port 8901 [--out frames.jsonl]
  python -m ws_emitter.probe_client --port 8901 --no-consume --hold-s 60
"""
from __future__ import annotations

import argparse
import asyncio
import json
import sys


async def _run(args):
    from websockets.asyncio.client import connect
    uri = f"ws://{args.host}:{args.port}"
    n, out = 0, None
    if args.out:
        out = open(args.out, "w", encoding="utf-8")
    try:
        async with connect(uri) as ws:
            if args.no_consume:
                # adversary: live connection without reading — the server buffer
                # fills up and websockets cuts it by ping_timeout; we never read
                await asyncio.sleep(args.hold_s)
                return 0
            async for frame in ws:
                n += 1
                d = json.loads(frame)
                assert "t_emit" in d, "frame de red SIN t_emit (viola contrato §3.2)"
                if out:
                    out.write(frame.decode() if isinstance(frame, bytes) else frame)
                if args.slow_s > 0:
                    await asyncio.sleep(args.slow_s)   # deliberate slow consumer
                if d.get("type") == "end":
                    break
    except Exception as e:
        print(f"[probe] conexión terminada: {type(e).__name__}: {e}", file=sys.stderr)
    finally:
        if out:
            out.close()
        print(f"[probe] frames recibidos: {n}")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, required=True)
    ap.add_argument("--out", default="", help="volcar frames recibidos (JSONL)")
    ap.add_argument("--slow-s", type=float, default=0.0,
                    help="dormir tras cada frame (consumidor lento)")
    ap.add_argument("--no-consume", action="store_true",
                    help="conectarse y NO leer (adversario de pasividad)")
    ap.add_argument("--hold-s", type=float, default=60.0,
                    help="segundos que el no-consumidor mantiene la conexión")
    sys.exit(asyncio.run(_run(ap.parse_args())))


if __name__ == "__main__":
    main()

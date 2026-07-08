"""WsBroadcastServer — emitter thread of architecture B.

WebSocket server in its OWN THREAD with its OWN asyncio event loop. The deterministic
side (player loop) only does `put_nowait(bytes)` + `notify()`; this thread consumes the
queue and publishes. Guarantees and their sources:

  - Thread→loop bridge: `loop.call_soon_threadsafe` — official Python doc
    (asyncio-dev): "To schedule a callback from another OS thread, the
    loop.call_soon_threadsafe() method should be used".
  - Publishing: `websockets.asyncio.server.broadcast` (websockets 16.0, pinned in
    requirements.txt). Docstring of the installed version: "pushes the message
    synchronously to all connections even if their write buffers are
    overflowing. There's no backpressure". Versioned doc (16.0,
    topics/broadcast): the lagging client "eventually ... reaches the limit
    defined by ping_timeout and websockets terminates the connection" — a
    client's slowness dies in the websockets layer, it never flows back to the loop.
  - This thread does NOT import traci, does NOT receive references to the player: its
    constructor accepts (host, port, queue) and nothing else. It only sees immutable,
    already-serialized bytes; t_emit is injected HERE with wire_frame() over a copy
    (contract Section 3.2: the server's monotonic clock, excluded from the canonical
    log).
  - Every exception in the thread is caught and COUNTED (reported in stats()); it is
    never propagated to the player (passivity).

DIAGNOSTIC stall_drain_s: artificial delay of the drain (same spirit as the player's
--leak-probe: deliberate adverse control) to saturate the queue and exercise the
drop-by-Full path in the passivity test. 0 = off.
"""
from __future__ import annotations

import asyncio
import queue as queue_mod
import threading
import time

from ws_emitter.canonical import wire_frame


class WsBroadcastServer:
    def __init__(self, host: str, port: int, out_queue: "queue_mod.Queue",
                 stall_drain_s: float = 0.0, hello_bytes: bytes | None = None):
        self._host = host
        self._port = port
        self._q = out_queue
        self._stall = float(stall_drain_s)
        # RETAINED HELLO: IMMUTABLE session state, fixed at construction (known before
        # the player loop). The ONLY delivery path of the hello: the on-connect handler
        # — never broadcast/queue (if it coexisted with broadcast there would be a
        # duplicate-hello race).
        self._hello_bytes = hello_bytes
        self._loop = None
        self._thread = None
        self._conns: set = set()
        self._ready = threading.Event()
        self._stop_ev = None
        self.sent = 0
        self.errors = 0
        self.bound_port = None      # real port (useful with port=0 in tests)

    # ------------------------------------------------------------------ thread
    def start(self, timeout: float = 5.0) -> "WsBroadcastServer":
        self._thread = threading.Thread(target=self._run, name="ws-emitter",
                                        daemon=True)
        self._thread.start()
        if not self._ready.wait(timeout):
            self.errors += 1        # server did not come up: reported, not aborted
        return self

    def _run(self):
        try:
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
            self._loop.run_until_complete(self._main())
        except Exception:
            self.errors += 1
            self._ready.set()       # do not leave the player waiting on the start join
        finally:
            try:
                self._loop.close()
            except Exception:
                pass

    async def _main(self):
        from websockets.asyncio.server import serve
        self._stop_ev = asyncio.Event()
        async with serve(self._handler, self._host, self._port) as server:
            socks = server.sockets or []
            self.bound_port = socks[0].getsockname()[1] if socks else self._port
            self._ready.set()
            await self._stop_ev.wait()
            # best-effort final drain before closing (the 'end' was already published
            # in the drain that saw the sentinel; the log is the truth on disk)

    async def _handler(self, ws):
        # RETAINED HELLO: broadcast() only reaches those connected AT THAT INSTANT
        # (doc 16.0, topics/broadcast: the pattern is a set of registered connections)
        # — a late client never received the hello and degenerated into "step before
        # hello". Each connection receives the retained hello here and registers IN THE
        # SAME BLOCK WITHOUT await: synchronous delivery (broadcast([ws], …) — "pushes
        # the message synchronously", doc 16.0) + registration atomic within the single-
        # thread event loop => on this connection's wire the hello precedes any step,
        # and no drain can be interleaved (with `await send` the drain could sneak in
        # during the yield: registration race). Connecting before/during/after startup
        # => hello -> STREAMING. The STEPS (and the end) remain EPHEMERAL: whoever
        # enters mid-stream receives the "now" (latest-step-wins), not the past — it is
        # replay, you enter the now; whoever enters after the end receives hello and the
        # close => SIN_VEREDICTO for that viewer (declared).
        from websockets.asyncio.server import broadcast
        if self._hello_bytes is not None:
            broadcast({ws}, wire_frame(self._hello_bytes, time.monotonic()))
            self._conns.add(ws)
        try:
            await ws.wait_closed()
        finally:
            self._conns.discard(ws)

    # ------------------------------------------------- loop-side calls
    def notify(self):
        """Player-thread → event loop bridge (call_soon_threadsafe, official doc).
        Never blocks; if the loop already died, it counts and continues."""
        loop = self._loop
        if loop is None:
            return
        try:
            loop.call_soon_threadsafe(self._drain)
        except RuntimeError:
            self.errors += 1        # closed loop: the network ended, the log rules

    # --------------------------------------------------- inside the event loop
    def _drain(self):
        from websockets.asyncio.server import broadcast
        while True:
            try:
                item = self._q.get_nowait()
            except queue_mod.Empty:
                return
            if item is None:            # emitter close sentinel
                if self._stop_ev is not None:
                    self._stop_ev.set()
                return
            if self._stall > 0:
                # DIAGNOSTIC: delay PER DRAINED ITEM (saturates the queue just the same);
                # sleeping before the get accumulated sleeps with an empty queue — after
                # the player finished, ~minutes of sleepy callbacks remained and the end
                # never got flushed
                time.sleep(self._stall)
            try:
                broadcast(self._conns, wire_frame(item, time.monotonic()))
                self.sent += 1
            except Exception:
                self.errors += 1

    def close(self, timeout: float = 5.0) -> dict:
        """Close from the player side (the sentinel must already be enqueued)."""
        self.notify()
        if self._thread is not None:
            self._thread.join(timeout)
            if self._thread.is_alive():
                self.errors += 1        # hung thread: daemon, it will die with the process
        return self.stats()

    def stats(self) -> dict:
        return {"sent": self.sent, "errors": self.errors,
                "bound_port": self.bound_port}

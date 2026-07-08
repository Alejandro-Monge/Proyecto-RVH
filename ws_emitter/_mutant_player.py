"""NEGATIVE CONTROL of layer 3 (--leak-probe spirit): a real player with a MUTANT
emitter that looks K ticks into the future. The prefix audit MUST fail on its logs;
if it does not fail, layer 3 does not catch leaks.

It does NOT touch production on disk: it patches `ws_emitter.emitter.MessagesEmitter`
IN THE PROCESS and calls `causal_player.main()` (the player imports it inside main).

Leak mechanics: the mutant retains K views and emits the step of view t while injecting
into it the `guard.accesses` of view t+K — a MESSAGE-DEPENDENT datum (the guard's access
counter grows with deliveries/decisions), so it differs between the full run and the
truncated one near the boundary. K must satisfy K·TICK > H (= delay_s): with TICK=0.1 and
delay 1.0, K=11 (1.1 s). A lookahead ≤ H would be legitimate and MUST NOT be detected —
that asymmetry is also verified.

Usage (invoked by prefix_audit --mutant-k):
  python -m ws_emitter._mutant_player --mutant-k 11 <causal_player args>
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "reconstruccion_causal" / "scripts"))

import ws_emitter.emitter as emitter_mod  # noqa: E402


def make_mutant(k: int):
    base = emitter_mod.MessagesEmitter
    step_message = emitter_mod.step_message

    class MutantEmitter(base):
        """DELIBERATE LEAK: step(t) carries guard.accesses from view t+K."""

        def __init__(self, **kw):
            super().__init__(**kw)
            self._pending = []

        def emit_step(self, view):
            self._pending.append(view)
            if len(self._pending) > k:
                past = self._pending.pop(0)
                future = self._pending[-1]          # K ticks ahead
                try:
                    msg = step_message(past)
                    msg["guard"]["accesses"] = int(
                        future.guard_counters["accesses"])   # <-- THE LEAK
                    self._write_and_enqueue(msg)
                    self.steps += 1
                    self._last_seq = int(past.seq)
                except Exception:
                    self.emit_errors += 1

        def emit_end(self, **kw):
            # drain what is retained WITH no future available (honest at the end of the
            # stream) and close; seq stays monotonic and without gaps
            for v in self._pending:
                super().emit_step(v)
            self._pending = []
            super().emit_end(**kw)

    return MutantEmitter


def main():
    argv = sys.argv[1:]
    assert argv[0] == "--mutant-k", "uso: --mutant-k K <args del player>"
    k = int(argv[1])
    emitter_mod.MessagesEmitter = make_mutant(k)
    import causal_player  # noqa: E402  (imports after the patch)
    sys.argv = ["causal_player.py"] + argv[2:]
    causal_player.main()


if __name__ == "__main__":
    main()

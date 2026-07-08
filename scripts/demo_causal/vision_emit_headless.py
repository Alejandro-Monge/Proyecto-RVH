"""HEADLESS runner of the vision emitter — produces the canonical log (ws_log.jsonl)
WITHOUT run_live (no ffplay, no realtime): build_veh_causal + loop (place_causal + step +
readback + build_vision_view) emitting to a LOG-ONLY MessagesEmitter (no ws_port).

It is the reusable reference of the vision layer-3 audit (determinism,
truncation, H measurement, mutant). Deterministic given the tracks CSV (+ events).

Usage:
  python -m demo_causal.vision_emit_headless --out LOG [--video tramo]
         [--tracks ABS.csv --events ABS.csv] [--frame-cap N]
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "scripts" / "demo_causal"))
# traci lives in $SUMO_HOME/tools (not pip): add it explicitly, robust to -m mode
sys.path.insert(0, os.path.join(os.environ.get("SUMO_HOME", "/usr/share/sumo"), "tools"))


def make_vision_mutant(k: int):
    """NEGATIVE CONTROL (layer 3): emitter that emits the step of seq t with the ENTITIES
    (POSITIONS) of step t+K — it looks K frames into the FUTURE. The audit MUST fail if K
    exceeds H_effective (140): with K≤140 the leak falls outside the window ≤ N−140
    (a false pass, like K=11 in messages); with K>140 it enters and FAILS by naming the
    position."""
    from ws_emitter.emitter import MessagesEmitter, step_message

    class _VisionMutant(MessagesEmitter):
        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            self._buf = []          # (seq, step_msg) of the last K+1

        def emit_step(self, view):
            try:
                self._buf.append((int(view.seq), step_message(view)))
                if len(self._buf) > k:
                    seq_i, msg_i = self._buf.pop(0)
                    _, msg_fut = self._buf[-1]                 # K ahead
                    msg_i["entities"] = msg_fut["entities"]    # <-- THE LEAK (positions)
                    msg_i["occupancy"] = msg_fut["occupancy"]
                    self._write_and_enqueue(msg_i)
                    self.steps += 1
                    self._last_seq = seq_i
            except Exception:
                self.emit_errors += 1

        def emit_end(self, **kw):
            for seq_i, msg_i in self._buf:      # drain the last K (out of window)
                self._write_and_enqueue(msg_i)
                self.steps += 1
                self._last_seq = seq_i
            self._buf = []
            super().emit_end(**kw)
    return _VisionMutant


def emit_vision_log(out_log, which="tramo", tracks_path=None, events_path=None,
                    frame_cap=None, mutant_k=0) -> dict:
    """Emits the canonical vision log. Returns a small summary (frames, veh)."""
    import run_harness_causal as rhc
    from digital_shadow.sumo.presentation import load_handoff_max_hold
    from ws_emitter.emitter import MessagesEmitter

    veh, chains, _commits, tid_root = rhc.build_veh_causal(
        which, tracks_path=tracks_path, events_path=events_path)
    stage, tr, sonda, VTYPE = rhc._setup(veh)
    wb = rhc.WaitingBoxLayer(tr, which, chains)
    handoff = rhc.HandoffRegistry(tid_root, load_handoff_max_hold("rusia"))

    # number of frames: it must cover ALL frames with tracks (BOXES, not just
    # vehicles) + the retention. The max-veh-end of run_readout OMITS the boxes that
    # exist after the last vehicle (finding from layer-3 step 3) → the max
    # frame_index of the tracks + timeout is used (as run_live covers the whole video).
    # Deterministic given the CSV. frame_cap bounds it for tests.
    import pandas as _pd
    _tpath = tracks_path or (rhc.ROOT / rhc.DATA[which][0])
    _max_frame = int(_pd.read_csv(_tpath, usecols=["frame_index"])["frame_index"].max())
    n = _max_frame + int(round(sonda.timeout_s / rhc.DT)) + 2
    if frame_cap is not None:
        n = min(n, int(frame_cap))

    _EmCls = make_vision_mutant(int(mutant_k)) if mutant_k else MessagesEmitter
    em = _EmCls(log_path=str(out_log),
                hello_fields=rhc.vision_hello_fields(which))            # no ws_port
    em.emit_hello()
    live, marks = {}, []
    try:
        for fi in range(n):
            t_v = fi * rhc.DT
            wb.draw(t_v)
            prov = {}
            rhc.place_causal(stage, tr, veh, live, sonda, t_v, VTYPE, prov_out=prov)
            tr.simulationStep(t_v)
            drawn = tr.vehicle.getAllSubscriptionResults()
            marks = handoff.update(t_v, wb.decided_boxes, set(drawn))
            em.emit_step(rhc.build_vision_view(
                seq=fi, t_sim=t_v, drawn=drawn, prov_out=prov,
                decided_boxes=wb.decided_boxes, tid_root=tid_root, marks=marks,
                occ_counts=wb.occupancy_counts(fi), sonda=sonda))
        em.emit_end(guard_final={"n_extrap": int(sonda.n_extrap)}, status="PASS")
    finally:
        em.close()
        try:
            stage.close()
        except Exception:
            pass
    return {"frames": n, "vehiculos": len(veh)}


def main():
    import os
    os.environ.setdefault("SUMO_HOME", "/usr/share/sumo")
    os.environ.setdefault("DISPLAY", ":1")
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", required=True)
    ap.add_argument("--video", default="tramo", choices=["tramo", "largo"])
    ap.add_argument("--tracks", default=None)
    ap.add_argument("--events", default=None)
    ap.add_argument("--frame-cap", type=int, default=None)
    ap.add_argument("--mutant-k", type=int, default=0,
                    help="CONTROL NEGATIVO: emisor que mira K frames al futuro (fuga)")
    args = ap.parse_args()
    rep = emit_vision_log(args.out, args.video, args.tracks, args.events,
                          args.frame_cap, args.mutant_k)
    print(f"[vision_emit] {rep} -> {args.out}", flush=True)


if __name__ == "__main__":
    main()

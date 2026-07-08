"""3-panel compositor of the demo video (OFFLINE composition).

REALITY (GT2) | SHADOW (reconstruction) | SHADOW 3D — paired by SIM label. The three
panels are recordings already made:
  - real   : driver_replay standalone, shots sh_<ms>.png labeled by sim T;
  - shadow : causal_player, shots sh_<ms>.png labeled by sim T;
  - 3D     : replay_ws3d.mjs --all, frames step_<seq>.png (seq from the ws_log).

PAIRING: real[T] | shadow[T] | 3D[T]. The shadow already runs
delay_s (=1 s, Section 7.2) behind reality BY CONSTRUCTION (its frame shows the
instant T−delay_s); delay_s is NOT subtracted again in the pairing (that would give 2 s of lag,
false). The correct visible lag = delay_s comes from aligning both panels by the SAME
sim label T. The 3D panel is the headless render of the shadow ws_log
(byte-identical to the live client, Part 1), at the same T.

STATEMENT (goes in the demo doc): it is a deterministic RECONSTRUCTION of real DATA
(real causal lag by construction, real content), NOT a live-session capture.
The video itself is a demo recording, not a byte-reproducible artifact — and
it does not need to be (companion video, not a measurement).

The FRAMING of each SUMO panel was set by the operator by hand during recording; here the
shots are only composed. Config-driven except for the framing (which is by eye).

Usage:
  python3 scripts/demo_causal/compose_3panel.py \
      --real-shots  <dir sh_*.png reality> \
      --shadow-shots <dir sh_*.png shadow> \
      --frames-3d   <dir step_*.png 3D> \
      --ws-log      <shadow ws_log.jsonl> \
      --out         verify_out/demo_causal/demo_mensajes.mp4 \
      --title "MENSAJES (V2X 5 Hz, 30% perdida)" [--delay-s 1.0] [--grid-s 0.05]
      [--fps 20] [--t0 <s>] [--t1 <s>]
"""
from __future__ import annotations

import argparse
import bisect
import json
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

HUD_H = 64


def index_shots(d: Path) -> tuple[list[int], dict[int, Path]]:
    """dir of sh_<ms>.png -> (sorted list of ms, {ms: path})."""
    m = {}
    for p in d.glob("sh_*.png"):
        try:
            m[int(p.stem.split("_")[1])] = p
        except (IndexError, ValueError):
            continue
    if not m:
        sys.exit(f"sin shots sh_*.png en {d}")
    return sorted(m), m


def load_ws_steps(ws_log: Path) -> list[dict]:
    """ws_log steps in order: [{t_sim, seq, n_ent}]."""
    steps = []
    for line in ws_log.open():
        line = line.strip()
        if not line:
            continue
        o = json.loads(line)
        if o.get("type") == "step":
            steps.append({"t_sim": float(o["t_sim"]), "seq": int(o["seq"]),
                          "n_ent": len(o.get("entities", []))})
    if not steps:
        sys.exit(f"ws_log sin steps: {ws_log}")
    return steps


def nearest_idx(keys: list, target) -> int:
    """index of the element of `keys` (sorted) closest to target."""
    i = bisect.bisect_left(keys, target)
    if i == 0:
        return 0
    if i >= len(keys):
        return len(keys) - 1
    return i if (keys[i] - target) < (target - keys[i - 1]) else i - 1


def put(canvas, text, org, scale, color, thick=1):
    cv2.putText(canvas, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--real-shots", required=True, type=Path)
    ap.add_argument("--shadow-shots", required=True, type=Path)
    ap.add_argument("--frames-3d", required=True, type=Path)
    ap.add_argument("--ws-log", required=True, type=Path)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--title", default="SOMBRA", help="etiqueta de la fuente (HUD panel central)")
    ap.add_argument("--delay-s", type=float, default=1.0, help="rezago causal para el HUD (§7.2)")
    ap.add_argument("--grid-s", type=float, default=0.05,
                    help="paso de sim (s) por frame de salida (default = cadencia realidad)")
    ap.add_argument("--fps", type=float, default=20.0,
                    help="fps de reproducción (default 20 = tiempo real con grid 0.05)")
    ap.add_argument("--t0", type=float, default=None, help="recorte: sim inicial (s)")
    ap.add_argument("--t1", type=float, default=None, help="recorte: sim final (s)")
    ap.add_argument("--panel-crop", default="",
                    help="recorte 'x0,y0,x1,y1' (px del shot SUMO) aplicado a los paneles "
                         "REALIDAD y SOMBRA antes del re-escalado al tamaño de panel → ZOOM "
                         "que descarta el borde inferior (artefacto de grabación del "
                         "gui.screenshot). El 3D no se recorta (su encuadre es la cámara).")
    a = ap.parse_args()
    crop = tuple(int(v) for v in a.panel_crop.split(",")) if a.panel_crop else None

    real_ms, real_map = index_shots(a.real_shots)
    shadow_ms, shadow_map = index_shots(a.shadow_shots)
    steps = load_ws_steps(a.ws_log)
    step_ts = [s["t_sim"] for s in steps]                 # ordered by emission (seq)
    frames3d = {int(p.stem.split("_")[1]): p for p in a.frames_3d.glob("step_*.png")}
    if not frames3d:
        sys.exit(f"sin frames step_*.png en {a.frames_3d}")

    # common sim range (seconds): where all three panels exist
    lo = max(real_ms[0] / 1000, shadow_ms[0] / 1000, step_ts[0])
    hi = min(real_ms[-1] / 1000, shadow_ms[-1] / 1000, step_ts[-1])
    if a.t0 is not None:
        lo = max(lo, a.t0)
    if a.t1 is not None:
        hi = min(hi, a.t1)
    if hi <= lo:
        sys.exit(f"rango de sim vacío: real[{real_ms[0]/1000:.2f},{real_ms[-1]/1000:.2f}] "
                 f"shadow[{shadow_ms[0]/1000:.2f},{shadow_ms[-1]/1000:.2f}] "
                 f"3d[{step_ts[0]:.2f},{step_ts[-1]:.2f}]")
    print(f"[compose] rango sim común: [{lo:.2f}, {hi:.2f}] s | grid {a.grid_s}s | "
          f"delay {a.delay_s}s | fps {a.fps}", flush=True)

    # panel size = dimensions of the reality shot (what the operator framed)
    ref = cv2.imread(str(real_map[real_ms[0]]))
    if ref is None:
        sys.exit("no pude leer el primer shot de realidad")
    Hp, Wp = ref.shape[:2]
    canvas_w, canvas_h = 3 * Wp, Hp + HUD_H
    if canvas_w % 2 or canvas_h % 2:
        sys.exit(f"canvas {canvas_w}x{canvas_h} no es par (yuv420p lo exige)")

    a.out.parent.mkdir(parents=True, exist_ok=True)
    ff = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "warning", "-f", "rawvideo", "-pix_fmt", "bgr24",
         "-s", f"{canvas_w}x{canvas_h}", "-r", f"{a.fps:.3f}", "-i", "-",
         "-c:v", "libx264", "-crf", "18", "-preset", "medium", "-pix_fmt", "yuv420p",
         str(a.out)], stdin=subprocess.PIPE)

    def panel(img_path, last, crop_box=None):
        img = cv2.imread(str(img_path)) if img_path else None
        if img is None:
            return last                       # missing frame: repeat the last (already at Wp×Hp)
        if crop_box is not None:              # ZOOM to the box (discards the bottom border)
            x0, y0, x1, y1 = crop_box
            img = img[y0:y1, x0:x1]
        if img.shape[:2] != (Hp, Wp):
            img = cv2.resize(img, (Wp, Hp))
        return img

    black = np.zeros((Hp, Wp, 3), np.uint8)
    last_real = last_shadow = last_3d = black
    n = int(round((hi - lo) / a.grid_s)) + 1
    written = 0
    for f in range(n):
        T = lo + f * a.grid_s
        Tms = int(round(T * 1000))
        real_img = panel(real_map[real_ms[nearest_idx(real_ms, Tms)]], last_real, crop)
        shadow_img = panel(shadow_map[shadow_ms[nearest_idx(shadow_ms, Tms)]], last_shadow, crop)
        si = nearest_idx(step_ts, T)
        seq = steps[si]["seq"]
        n_ent = steps[si]["n_ent"]
        img3d = panel(frames3d.get(seq), last_3d)   # 3D without crop (its framing is the camera)
        last_real, last_shadow, last_3d = real_img, shadow_img, img3d

        canvas = np.zeros((canvas_h, canvas_w, 3), np.uint8)
        canvas[HUD_H:, 0:Wp] = real_img
        canvas[HUD_H:, Wp:2 * Wp] = shadow_img
        canvas[HUD_H:, 2 * Wp:3 * Wp] = img3d
        # HUD
        cv2.rectangle(canvas, (0, 0), (canvas_w, HUD_H), (14, 14, 14), -1)
        put(canvas, "REALIDAD (GT2)", (14, 26), 0.72, (255, 255, 255), 2)
        put(canvas, a.title, (Wp + 14, 26), 0.72, (90, 220, 255), 2)
        put(canvas, "SOMBRA 3D", (2 * Wp + 14, 26), 0.72, (170, 255, 170), 2)
        # NOTE: OpenCV's Hershey font has no section-sign glyph (renders as "??"): I use "Sec."
        put(canvas,
            f"t_sim = {T:6.2f} s    vehiculos (sombra): {n_ent:2d}    "
            f"rezago causal (delay) = {a.delay_s:.2f} s [Sec. 7.2]: la sombra muestra "
            f"el instante t_sim - {a.delay_s:.2f} s",
            (14, 52), 0.56, (200, 200, 200), 1)
        for x in (Wp, 2 * Wp):
            cv2.line(canvas, (x, 0), (x, canvas_h), (40, 40, 40), 2)

        ff.stdin.write(canvas.tobytes())
        written += 1
        if written % 500 == 0:
            print(f"  {written}/{n} frames (t_sim {T:.1f}s)…", flush=True)

    ff.stdin.close()
    ff.wait()
    dur = written / a.fps
    print(f"[ok] {a.out}  ({written} frames, {dur:.1f}s @ {a.fps} fps)  "
          f"REALIDAD | {a.title} | 3D", flush=True)


if __name__ == "__main__":
    main()

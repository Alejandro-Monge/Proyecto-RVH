"""PIXEL selector for the 7 EXTRA control points (improved homography).

ANALYSIS ONLY. Reuses the tested mechanics of select_homography_points.py (own zoom/pan
via warpAffine, mouse callback, point-by-point navigation). SINGLE frame:
clean_frame_2541.png (the same as the 10 already-confirmed points).

HARD RULE: this script does NOT open or write control_points_improved.csv. It only collects the
PIXEL + NAME of each extra in a separate STAGING file
(control_points_extra_pixels.csv). The WORLD coordinate is read in netedit and
paired LATER (another step). This way the 10 originals stay intact by construction.

The 7 extras (the feature is chosen by eye; the descriptions only guide):
  - 4 in the SUR, spread toward the horizon (DEPTH, not clumped)
  - 1 in the OESTE (depth)
  - 1 in the ESTE
  - 1 in the NORTE (crosswalk, more centered than the current ones)

CONTROLS (focused window):
  LEFT click     : marks/re-marks the pixel of the CURRENT point (overwrites)
  RIGHT click    : centers the view (pan)
  + / =          : zoom IN          - / _ : zoom OUT       wheel: zoom
  n : next   b : previous   r : reset view
  w : save CSV   q : save and exit
"""

import csv
import os
import sys
import cv2
import numpy as np

FRAME = "verify_out/homography_improved/clean_frame_2541.png"
OUT_CSV = "verify_out/homography_improved/control_points_extra_pixels.csv"
WIN = "Selector EXTRAS (solo pixel + nombre) - homografia mejorada"

# (id, arm_hint, click_instruction). World is NOT requested here.
# Corner mapping: SUR=top-left, OESTE=top-right, ESTE=bottom-left, NORTE=bottom-right.
POINTS = [
    ("Punto-4-SUR", "SUR",
     "SUR (boca SUP-IZQ): rasgo fijo del suelo. PROFUNDIDAD 1 (mas cercano de los 4 extra del sur)."),
    ("Punto-5-SUR", "SUR",
     "SUR (sup-izq): PROFUNDIDAD 2 (un poco mas hacia el horizonte). Reparte, no amontones."),
    ("Punto-6-SUR", "SUR",
     "SUR (sup-izq): PROFUNDIDAD 3 (mas lejos aun)."),
    ("Punto-7-SUR", "SUR",
     "SUR (sup-izq): PROFUNDIDAD 4 (el mas lejano, cerca del horizonte si hay rasgo nitido)."),
    ("Punto-3-OESTE", "OESTE",
     "OESTE (boca SUP-DER): rasgo fijo del suelo a mayor PROFUNDIDAD del oeste."),
    ("Punto-extra-ESTE", "ESTE",
     "ESTE (boca INF-IZQ): rasgo fijo del suelo, extra en el este."),
    ("Punto-extra-NORTE", "NORTE",
     "NORTE (boca INF-DER): PASO DE CEBRA, mas CENTRADO que los puntos de pare actuales."),
]


def wrap(text, width=84):
    words, lines, cur = text.split(), [], ""
    for w in words:
        if len(cur) + len(w) + 1 > width:
            lines.append(cur); cur = w
        else:
            cur = (cur + " " + w).strip()
    if cur:
        lines.append(cur)
    return lines


class Selector:
    def __init__(self, win_w=1280, win_h=760):
        self.img = cv2.imread(FRAME)
        if self.img is None:
            raise FileNotFoundError(FRAME)
        self.H, self.W = self.img.shape[:2]
        self.win_w, self.win_h = win_w, win_h
        self.idx = 0
        self.clicks = {}
        self.mouse = (0, 0)
        self.reset_view()

    def reset_view(self):
        self.z = min(self.win_w / self.W, self.win_h / self.H)
        self.cx, self.cy = self.W / 2.0, self.H / 2.0

    def _affine(self):
        tx = self.win_w / 2.0 - self.z * self.cx
        ty = self.win_h / 2.0 - self.z * self.cy
        return self.z, tx, ty

    def win_to_src(self, wx, wy):
        z, tx, ty = self._affine()
        return (wx - tx) / z, (wy - ty) / z

    def src_to_win(self, fx, fy):
        z, tx, ty = self._affine()
        return z * fx + tx, z * fy + ty

    def zoom(self, factor, about=None):
        if about is None:
            self.z *= factor
            return
        ax, ay = about
        sx, sy = self.win_to_src(ax, ay)
        self.z = max(0.1, min(40.0, self.z * factor))
        self.cx = sx - (ax - self.win_w / 2.0) / self.z
        self.cy = sy - (ay - self.win_h / 2.0) / self.z

    def render(self):
        z, tx, ty = self._affine()
        M = np.array([[z, 0, tx], [0, z, ty]], dtype=np.float32)
        view = cv2.warpAffine(self.img, M, (self.win_w, self.win_h),
                              flags=cv2.INTER_NEAREST, borderValue=(30, 30, 30))
        for i, (pid, arm, instr) in enumerate(POINTS):
            c = self.clicks.get(pid)
            if c is None:
                continue
            sx, sy = self.src_to_win(*c)
            if 0 <= sx < self.win_w and 0 <= sy < self.win_h:
                cur = (i == self.idx)
                col = (0, 255, 0) if cur else (0, 200, 255)
                cv2.drawMarker(view, (int(sx), int(sy)), col, cv2.MARKER_CROSS, 22, 2)
                cv2.circle(view, (int(sx), int(sy)), 5, col, 1)
                cv2.putText(view, pid, (int(sx) + 8, int(sy) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)

        pid, arm, instr = POINTS[self.idx]
        c = self.clicks.get(pid)
        estado = f"marcado px=({c[0]:.0f},{c[1]:.0f})" if c else "sin marcar"
        header = f"[{self.idx+1}/{len(POINTS)}] {pid}  [{arm}]  -> {estado}"
        lines = [header] + wrap(instr)
        lines.append("clicIZQ=marcar  clicDER=centrar  +/-=zoom  n=sig  b=atras  "
                     "r=reset  w=guardar  q=salir")
        bh = 20 + 20 * len(lines)
        overlay = view.copy()
        cv2.rectangle(overlay, (0, 0), (self.win_w, bh), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, view, 0.45, 0, view)
        y = 22
        for i, ln in enumerate(lines):
            col = (0, 255, 255) if i == 0 else (
                (180, 180, 180) if i == len(lines) - 1 else (255, 255, 255))
            cv2.putText(view, ln, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1,
                        cv2.LINE_AA)
            y += 20
        cv2.putText(view, f"zoom x{self.z:.1f}", (self.win_w - 120, self.win_h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.imshow(WIN, view)

    def on_mouse(self, event, x, y, flags, param):
        self.mouse = (x, y)
        if event == cv2.EVENT_LBUTTONDOWN:
            fx, fy = self.win_to_src(x, y)
            fx = float(np.clip(fx, 0, self.W - 1))
            fy = float(np.clip(fy, 0, self.H - 1))
            self.clicks[POINTS[self.idx][0]] = (fx, fy)
            print(f"  {POINTS[self.idx][0]}: px=({fx:.1f},{fy:.1f})  (n=siguiente)")
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.cx, self.cy = self.win_to_src(x, y)
        elif event == cv2.EVENT_MOUSEWHEEL:
            self.zoom(1.2 if flags > 0 else 1 / 1.2, about=(x, y))

    def run(self):
        cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WIN, self.on_mouse)
        while True:
            self.render()
            k = cv2.waitKey(20) & 0xFF
            if k == 255:
                continue
            ch = chr(k)
            if ch == 'n':
                self.idx = min(self.idx + 1, len(POINTS) - 1)
            elif ch == 'b':
                self.idx = max(self.idx - 1, 0)
            elif ch in '+=':
                self.zoom(1.25, about=self.mouse)
            elif ch in '-_':
                self.zoom(1 / 1.25, about=self.mouse)
            elif ch == 'r':
                self.reset_view()
            elif ch == 'w':
                self.save()
            elif ch == 'q':
                self.save()
                break
        cv2.destroyAllWindows()

    def save(self):
        os.makedirs(os.path.dirname(OUT_CSV), exist_ok=True)
        with open(OUT_CSV, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["id", "arm_hint", "pixel_x", "pixel_y", "available"])
            for pid, arm, instr in POINTS:
                c = self.clicks.get(pid)
                avail = c is not None
                px = f"{c[0]:.2f}" if avail else ""
                py = f"{c[1]:.2f}" if avail else ""
                w.writerow([pid, arm, px, py, avail])
        n_ok = sum(1 for v in self.clicks.values() if v is not None)
        print(f"\nGuardado {OUT_CSV}: {n_ok}/{len(POINTS)} marcados.")


def selftest():
    s = Selector()
    s.z, s.cx, s.cy = 3.7, 400.0, 220.0
    for (fx, fy) in [(0, 0), (600, 300), (1199, 645), (137.5, 88.4)]:
        sx, sy = s.src_to_win(fx, fy)
        bx, by = s.win_to_src(sx, sy)
        assert abs(bx - fx) < 1e-3 and abs(by - fy) < 1e-3
    s.clicks = {"Punto-4-SUR": (350.0, 120.0), "Punto-extra-NORTE": (800.0, 420.0)}
    s.save()
    with open(OUT_CSV) as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == len(POINTS)
    by_id = {r["id"]: r for r in rows}
    assert by_id["Punto-4-SUR"]["available"] == "True"
    assert by_id["Punto-4-SUR"]["pixel_x"] == "350.00"
    assert by_id["Punto-3-OESTE"]["available"] == "False"
    # the main file must NOT be touched by this script (different paths)
    assert OUT_CSV != "verify_out/homography_improved/control_points_improved.csv"
    print("selftest OK: mapeo exacto, guardado en staging aparte, principal intacto.")


def main():
    if "--selftest" in sys.argv:
        selftest()
        return
    print(__doc__)
    print("Extras a marcar (en orden):")
    for i, (pid, arm, instr) in enumerate(POINTS, 1):
        print(f"  {i}. {pid:18s} [{arm}]")
    print("\nAbriendo ventana OpenCV... (necesita display)\n")
    Selector().run()


if __name__ == "__main__":
    main()

"""Pixel selector for the IMPROVED homography (10 control points).

ANALYSIS INFRASTRUCTURE ONLY. It is not a production module. Reuses the tested
mechanics of `select_homography_points.py` (own zoom/pan via warpAffine, mouse
callback, point-by-point navigation). The original is kept intact.

Key difference from the preliminary one:
  - The WORLD coordinates (net meters) are given for the 10 points; here only
    the PIXEL of each (its click) is collected. The pixel<->world pairing is done
    POINT BY POINT and NAMED (a pairing swap already caused a 180° turn).
  - TWO frames: the first 9 over the clean frame (clean_frame_2541.png); the 10th
    (OESTE extra, deep) over a frame with the far west cleared
    (west_clear_frame_1.png). The camera is fixed → the feature's pixel is valid even
    if it comes from another frame.

It does NOT calibrate. It only saves the pixel<->world pairs in a reproducible CSV for review.

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

OUT_CSV = "verify_out/homography_improved/control_points_improved.csv"
WIN = "Selector puntos de control (homografia MEJORADA)"

FRAMES = {
    "clean2541": "verify_out/homography_improved/clean_frame_2541.png",
    "westfar":   "verify_out/homography_improved/west_clear_frame_1.png",
}

# (id, world_x, world_y, arm, frame_key, click_instruction)
# World = net meters. The click only contributes the pixel.
# Corner mapping: SUR=top-left, OESTE=top-right, ESTE=bottom-left, NORTE=bottom-right.
POINTS = [
    ("SUR-SL-primer",  57.55, 45.45, "SUR",   "clean2541",
     "SUR (boca SUP-IZQ, lejano): EXTREMO de la linea de pare, lado del PRIMER CARRIL."),
    ("SUR-SL-giroD",   67.97, 47.43, "SUR",   "clean2541",
     "SUR (sup-izq): EXTREMO de la linea de pare, lado del carril de GIRO A LA DERECHA."),
    ("SUR-extra-prof", 61.90, 31.18, "SUR",   "clean2541",
     "SUR (sup-izq): punto EXTRA a mayor PROFUNDIDAD (mas hacia el horizonte, dentro del brazo)."),
    ("NORTE-SL-primer",59.98, 97.86, "NORTE", "clean2541",
     "NORTE (boca INF-DER, cercano): EXTREMO de la linea de pare, lado del PRIMER CARRIL."),
    ("NORTE-SL-giroD", 46.52, 94.93, "NORTE", "clean2541",
     "NORTE (inf-der): EXTREMO de la linea de pare, lado del carril de GIRO A LA DERECHA."),
    ("ESTE-SL-primer", 77.33, 71.46, "ESTE",  "clean2541",
     "ESTE (boca INF-IZQ, medio): EXTREMO de la linea de pare, lado del PRIMER CARRIL."),
    ("ESTE-SL-giroD",  74.16, 89.37, "ESTE",  "clean2541",
     "ESTE (inf-izq): EXTREMO de la linea de pare, lado del carril de GIRO A LA DERECHA."),
    ("OESTE-SL-primer",38.16, 78.03, "OESTE", "clean2541",
     "OESTE (boca SUP-DER, lejano): EXTREMO de la linea de pare, lado del PRIMER CARRIL."),
    ("OESTE-SL-giroD", 40.37, 61.03, "OESTE", "clean2541",
     "OESTE (sup-der): EXTREMO de la linea de pare, lado del carril de GIRO A LA DERECHA."),
    ("OESTE-extra-prof",7.46, 54.09, "OESTE", "westfar",
     "OESTE (sup-der): punto EXTRA a mayor PROFUNDIDAD (lejos hacia J0). [FRAME DISTINTO: oeste despejado]"),
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
        self.win_w, self.win_h = win_w, win_h
        self.idx = 0
        self.clicks = {}          # id -> (px,py)
        self.mouse = (0, 0)
        self.cur_key = None
        self.img = None
        self.ensure_frame()

    # loads the frame of the current point if it changed
    def ensure_frame(self):
        key = POINTS[self.idx][4]
        if key != self.cur_key:
            path = FRAMES[key]
            img = cv2.imread(path)
            if img is None:
                raise FileNotFoundError(path)
            self.img = img
            self.H, self.W = img.shape[:2]
            self.cur_key = key
            self.reset_view()

    # --- view (zoom/pan via warpAffine) ---
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

    # --- drawing ---
    def render(self):
        self.ensure_frame()
        z, tx, ty = self._affine()
        M = np.array([[z, 0, tx], [0, z, ty]], dtype=np.float32)
        view = cv2.warpAffine(self.img, M, (self.win_w, self.win_h),
                              flags=cv2.INTER_NEAREST, borderValue=(30, 30, 30))

        # markers: only those of points on the SAME frame as the current one
        for i, (pid, wx, wy, arm, key, instr) in enumerate(POINTS):
            if key != self.cur_key:
                continue
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

        pid, wx_, wy_, arm, key, instr = POINTS[self.idx]
        c = self.clicks.get(pid)
        estado = f"marcado px=({c[0]:.0f},{c[1]:.0f})" if c else "sin marcar"
        header = (f"[{self.idx+1}/{len(POINTS)}] {pid}  mundo=({wx_},{wy_})m  "
                  f"frame={key}  -> {estado}")
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

    # --- mouse ---
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

    # --- loop ---
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
            w.writerow(["id", "arm", "world_x", "world_y", "frame",
                        "pixel_x", "pixel_y", "available", "description"])
            for pid, wx, wy, arm, key, instr in POINTS:
                c = self.clicks.get(pid)
                avail = c is not None
                px = f"{c[0]:.2f}" if avail else ""
                py = f"{c[1]:.2f}" if avail else ""
                w.writerow([pid, arm, wx, wy, FRAMES[key], px, py, avail, instr])
        n_ok = sum(1 for v in self.clicks.values() if v is not None)
        print(f"\nGuardado {OUT_CSV}: {n_ok}/{len(POINTS)} marcados.")


def selftest():
    """Verifies win<->src mapping, zoom-about, frame change and saving, without a window."""
    s = Selector()
    s.z, s.cx, s.cy = 3.7, 400.0, 220.0
    for (fx, fy) in [(0, 0), (600, 300), (1199, 645), (137.5, 88.4)]:
        sx, sy = s.src_to_win(fx, fy)
        bx, by = s.win_to_src(sx, sy)
        assert abs(bx - fx) < 1e-3 and abs(by - fy) < 1e-3, (fx, fy, bx, by)
    s.reset_view()
    about = (300, 400)
    sx0, sy0 = s.win_to_src(*about)
    s.zoom(1.5, about=about)
    sx1, sy1 = s.win_to_src(*about)
    assert abs(sx0 - sx1) < 1e-3 and abs(sy0 - sy1) < 1e-3
    # frame change: going to the last point (westfar) reloads the image
    assert s.cur_key == "clean2541"
    s.idx = len(POINTS) - 1
    s.ensure_frame()
    assert s.cur_key == "westfar", s.cur_key
    # saving: simulates 2 clicks
    s.clicks = {"SUR-SL-primer": (303.0, 128.0), "OESTE-extra-prof": (1021.0, 66.0)}
    s.save()
    with open(OUT_CSV) as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == len(POINTS)
    by_id = {r["id"]: r for r in rows}
    assert by_id["SUR-SL-primer"]["available"] == "True"
    assert by_id["SUR-SL-primer"]["pixel_x"] == "303.00"
    assert by_id["NORTE-SL-primer"]["available"] == "False"
    assert by_id["OESTE-extra-prof"]["frame"].endswith("west_clear_frame_1.png")
    print("selftest OK: mapeo exacto, zoom-about estable, cambio de frame, CSV correcto.")


def main():
    if "--selftest" in sys.argv:
        selftest()
        return
    print(__doc__)
    print("Puntos (en orden de presentacion):")
    for i, (pid, wx, wy, arm, key, instr) in enumerate(POINTS, 1):
        print(f"  {i:2d}. {pid:16s} mundo=({wx},{wy}) [{arm}] frame={key}")
    print("\nAbriendo ventana OpenCV... (necesita display)\n")
    Selector().run()


if __name__ == "__main__":
    main()

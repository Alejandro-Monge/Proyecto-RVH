"""Interactive control-point selector for the preliminary homography (OpenCV).

ANALYSIS INFRASTRUCTURE ONLY. It is not a production module.

Opens the camera frame and guides point by point: for each point it shows
its ID and WHICH physical FEATURE to click; it records the EXACT pixel of the click (in
full-resolution image coordinates, even while zoomed) and associates it with the
WORLD coordinate (meters) taken from the SUMO net.xml. It allows skipping what is not visible,
redoing and navigating. It saves the pixel<->world pairs in a reproducible CSV.

OpenCV is used (cv2.setMouseCallback + cv2.waitKey) instead of matplotlib because in
TkAgg the events were not captured reliably. The zoom/pan is our own (warpAffine),
not dependent on any backend's toolbar.

The WORLD coordinates are FIXED (they come from net.xml; see 'net_source'); the click only
contributes the pixel. This way the fit does not depend on numbers read by eye.

Families:
  - SL-*  stop line of each approach: midpoint of the painted bar.
          World = mean of the lane endpoints of that approach on reaching J5.
  - CB-*  island curb tip at an intersection corner.
          World = vertex of the J5 junction polygon.
C-SW (far south, featureless asphalt) is skipped: better one point fewer than a bad one.

CONTROLS (with the window focused):
  LEFT click      : marks/re-marks the pixel of the CURRENT point (overwrites)
  RIGHT click     : centers the view on that point (pan)
  + / =           : zoom IN (centered on the view)     - / _ : zoom OUT
  mouse wheel     : zoom (if your build delivers it; optional)
  n : confirm and next     b : previous     s : skip (not available)
  r : reset the view (fit everything)
  w : save CSV now         q : save CSV and exit
"""

import csv
import os
import sys
import cv2
import numpy as np

FRAME = "verify_out/first_frame.png"
OUT_CSV = "verify_out/homography_gcp/control_points.csv"
WIN = "Selector de puntos de control (homografia preliminar)"

# (id, world_x, world_y, zona, net_source, click_instruction)
POINTS = [
    # world paired with the PHYSICAL APPROACH of each pixel (net north-up). The pixel
    # is where the camera sees it (corner mapping: bottom-left=ESTE,
    # bottom-right=NORTE, top-right=OESTE, top-left=SUR); it is paired with the edge of THAT approach:
    # ESTE↔E3, NORTE↔E6, OESTE↔E1, SUR↔E5. The clicks (pixel domain) do NOT change.
    # NOTE: the edge names E1/E3/E5/E6 are NOT cardinal — E1 is the OESTE approach.
    ("SL-ESTE", 77.16, 83.10, "near", "media extremos lanes E3 en J5 (brazo ESTE, +X->J4)",
     "ESTE (boca inf-izq, cercano): CENTRO de la barra de pare pintada que cruza la "
     "calzada. La cebra ayuda a ubicarla."),
    ("SL-NORTE", 50.67, 98.17, "near", "media extremos lanes E6 en J5 (brazo NORTE, +Y->J6)",
     "NORTE (boca inf-der, la mas cercana): CENTRO de la barra de pare pintada."),
    ("SL-OESTE", 36.89, 65.30, "mid", "media extremos lanes E1 en J5 (brazo OESTE, -X->J0/J1)",
     "OESTE (boca sup-der, lejano): CENTRO de la barra de pare. Si NO hay barra "
     "visible, SALTAR (s)."),
    ("SL-SUR", 63.23, 50.44, "far", "media extremos lanes E5 en J5 (brazo SUR, -Y->J8)",
     "SUR (boca sup-izq, lejano, bajo la cola del bus) [BAJA CONFIANZA]: borde "
     "interior de la cebra/pare. Si esta muy borroso, SALTAR (s)."),
    ("CB-NE", 61.79, 91.86, "near", "vertice poligono J5 lado NE fisico (proy del pixel)",
     "Esquina NORTE-ESTE (centro-inferior, cercana): PUNTA del bordillo curvo de la "
     "isleta. Si no hay bordillo, SALTAR."),
    ("CB-NW", 42.49, 79.98, "near", "vertice poligono J5 lado NW fisico (proy del pixel)",
     "Esquina OESTE-NORTE (lado derecho): PUNTA del bordillo de la isleta. Si no hay "
     "bordillo, SALTAR."),
    ("CB-SE", 69.05, 65.51, "mid", "vertice poligono J5 lado SE fisico (proy del pixel)",
     "Esquina ESTE-SUR (lado izquierdo): PUNTA del bordillo de la isleta si es "
     "visible; si no, SALTAR."),
]


def wrap(text, width=64):
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
        self.clicks = {}          # id -> (px,py) or None (skipped)
        self.mouse = (0, 0)       # mouse position in window
        self.reset_view()

    # --- view (zoom/pan via warpAffine) ---
    def reset_view(self):
        self.z = min(self.win_w / self.W, self.win_h / self.H)
        self.cx, self.cy = self.W / 2.0, self.H / 2.0

    def _affine(self):
        # window = z*source + t ; centers (cx,cy) at the center of the window
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
        # zoom keeping the source point under 'about' fixed (or the center)
        if about is None:
            self.z *= factor
            return
        ax, ay = about
        sx, sy = self.win_to_src(ax, ay)
        self.z = max(0.1, min(40.0, self.z * factor))
        # reposition so (sx,sy) stays under (ax,ay)
        self.cx = sx - (ax - self.win_w / 2.0) / self.z
        self.cy = sy - (ay - self.win_h / 2.0) / self.z

    # --- drawing ---
    def render(self):
        z, tx, ty = self._affine()
        M = np.array([[z, 0, tx], [0, z, ty]], dtype=np.float32)
        view = cv2.warpAffine(self.img, M, (self.win_w, self.win_h),
                              flags=cv2.INTER_NEAREST, borderValue=(30, 30, 30))

        # markers already placed
        for pid, c in self.clicks.items():
            if c is None:
                continue
            wx, wy = self.src_to_win(*c)
            if 0 <= wx < self.win_w and 0 <= wy < self.win_h:
                cur = (pid == POINTS[self.idx][0])
                col = (0, 255, 0) if cur else (0, 200, 255)
                cv2.drawMarker(view, (int(wx), int(wy)), col,
                               cv2.MARKER_CROSS, 22, 2)
                cv2.circle(view, (int(wx), int(wy)), 5, col, 1)
                cv2.putText(view, pid, (int(wx) + 8, int(wy) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)

        # instructions banner
        pid, wx_, wy_, zona, src, instr = POINTS[self.idx]
        c = self.clicks.get(pid, "NOCLICK")
        if c == "NOCLICK":
            estado = "sin marcar"
        elif c is None:
            estado = "SALTADO"
        else:
            estado = f"marcado px=({c[0]:.0f},{c[1]:.0f})"
        header = (f"[{self.idx+1}/{len(POINTS)}] {pid}  mundo=({wx_},{wy_})m  "
                  f"zona={zona}  -> {estado}")
        lines = [header] + wrap(instr, 84)
        ctrl = ("clicIZQ=marcar  clicDER=centrar  +/-=zoom  n=sig  b=atras  "
                "s=saltar  r=reset  w=guardar  q=salir")
        lines.append(ctrl)
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
            print(f"  {POINTS[self.idx][0]}: px=({fx:.1f},{fy:.1f})  (n=confirmar)")
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
            elif ch == 's':
                self.clicks[POINTS[self.idx][0]] = None
                print(f"  {POINTS[self.idx][0]}: SALTADO")
                self.idx = min(self.idx + 1, len(POINTS) - 1)
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
            w.writerow(["id", "world_x", "world_y", "zone", "pixel_x", "pixel_y",
                        "available", "net_source"])
            for pid, wx, wy, zona, src, instr in POINTS:
                c = self.clicks.get(pid, "NOCLICK")
                if c == "NOCLICK":
                    continue
                avail = c is not None
                px = f"{c[0]:.2f}" if avail else ""
                py = f"{c[1]:.2f}" if avail else ""
                w.writerow([pid, wx, wy, zona, px, py, avail, src])
        n_ok = sum(1 for v in self.clicks.values() if v is not None)
        n_sk = sum(1 for v in self.clicks.values() if v is None)
        print(f"\nGuardado {OUT_CSV}: {n_ok} marcados, {n_sk} saltados.")


def selftest():
    """Verifies the mapping math and the saving, without opening a window."""
    s = Selector()
    # round trip win<->src under various zoom/pan conditions
    s.z, s.cx, s.cy = 3.7, 400.0, 220.0
    for (fx, fy) in [(0, 0), (600, 300), (1199, 645), (137.5, 88.4)]:
        wx, wy = s.src_to_win(fx, fy)
        bx, by = s.win_to_src(wx, wy)
        assert abs(bx - fx) < 1e-3 and abs(by - fy) < 1e-3, (fx, fy, bx, by)
    # zoom 'about' keeps the source point under the cursor fixed
    s.reset_view()
    about = (300, 400)
    sx0, sy0 = s.win_to_src(*about)
    s.zoom(1.5, about=about)
    sx1, sy1 = s.win_to_src(*about)
    assert abs(sx0 - sx1) < 1e-3 and abs(sy0 - sy1) < 1e-3, (sx0, sy0, sx1, sy1)
    # saving: simulates 2 marked + 1 skipped
    s.clicks = {"SL-ESTE": (512.0, 300.0), "SL-NORTE": (640.0, 360.0),
                "SL-OESTE": None}
    s.save()
    with open(OUT_CSV) as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 3
    assert rows[0]["id"] == "SL-ESTE" and rows[0]["available"] == "True"
    assert rows[2]["available"] == "False" and rows[2]["pixel_x"] == ""
    print("selftest OK: mapeo win<->src exacto, zoom-about estable, CSV correcto.")


def main():
    if "--selftest" in sys.argv:
        selftest()
        return
    print(__doc__)
    print("Puntos a marcar (en orden):")
    for i, (pid, wx, wy, zona, src, instr) in enumerate(POINTS, 1):
        print(f"  {i}. {pid:9s} mundo=({wx},{wy}) [{zona}] <- {src}")
    print("\nAbriendo ventana OpenCV... (necesita display)\n")
    Selector().run()


if __name__ == "__main__":
    main()

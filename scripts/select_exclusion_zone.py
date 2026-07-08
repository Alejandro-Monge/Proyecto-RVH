"""Interactive exclusion POLYGON selector (OpenCV) — WEST mouth (parking lot).

MARKING INFRASTRUCTURE ONLY. It is not a production module and does NOT apply anything to the
pipeline (the exclusion layer is applied separately, after the detector and before
tracking/inference; the detector stays "dumb").

What it does: opens the camera frame WITH THE ACCUMULATED TRAJECTORIES overlaid in
PIXEL-space (1:1 with the image, not the matplotlib PNG with axes/margins, which
would break the click->pixel mapping). It highlights the LONG and STATIC tracks (candidates for
parked vehicles, e.g. tid12 ~4669 frames) with their id, to locate where the
ground-contact points of the stopped cars fall and to enclose ONLY the real parking lot
of the WEST mouth (top-right) WITHOUT biting into the roadway where real traffic passes.

ORIENTATION NOTE (verified): corner mapping top-left=SUR, top-right=OESTE,
bottom-left=ESTE, bottom-right=NORTE. The real parking is at OESTE (top-right). The
static SUR cluster (top-left) is STOPPED TRAFFIC IN RED -> it is NOT excluded. The tool
labels both clusters; the operator decides the vertices.

Unlike the homography selector (fixed list of points), here an ARBITRARY
number of vertices is marked in order; the polygon closes from the last to the first.

The zoom/pan mechanics (win<->src via warpAffine) are the SAME, already tested, as the
homography selector (scripts/select_homography_points.py).

CONTROLS (with the window focused):
  LEFT click     : adds a vertex at that pixel (at the end of the sequence)
  RIGHT click    : centers the view on that point (pan)
  + / =          : zoom IN (centered on the cursor)     - / _ : zoom OUT
  mouse wheel    : zoom (if your build delivers it)
  u              : undo the last vertex
  c              : clear ALL vertices (start over)
  r              : reset the view (fit everything)
  w              : save capture (CSV + preview PNG) now
  q              : save and exit

OUTPUT (reproducible capture, NOT the config yet):
  verify_out/zones/rusia/exclusion_oeste_polygon.csv  (idx, pixel_x, pixel_y)
  verify_out/zones/rusia/exclusion_oeste_preview.png  (polygon drawn for review)
PROMOTING the polygon to the site config (config/sites/rusia/zones.yaml,
named as the west exclusion zone) is a separate step, after reviewing the
preview — just like the homography promotion.
"""

import csv
import os
import sys

import cv2
import numpy as np
import pandas as pd

from digital_shadow.geometry import add_ground_contact

FRAME = "verify_out/homography_improved/clean_frame_2541.png"   # same clean frame as plot_camera_trajectories
TRACKS = "verify_out/tracks_rusia.csv"
OUT_DIR = "verify_out/zones/rusia"
OUT_CSV = os.path.join(OUT_DIR, "exclusion_oeste_polygon.csv")
OUT_BG = os.path.join(OUT_DIR, "bg_exclusion_oeste.png")
OUT_PREVIEW = os.path.join(OUT_DIR, "exclusion_oeste_preview.png")
WIN = "Zona de exclusion OESTE (parqueadero) - marca el poligono"

MIN_OBS = 25          # same filter as plot_camera_trajectories / project_tracks_birdseye
STATIC_MIN_OBS = 200  # parked candidates: long...
STATIC_MAX_SPAN = 40  # ...and with almost no ground-contact displacement (px)


def build_background():
    """Clean frame + trajectories (pixel-space, 1:1) with statics highlighted.

    Returns a 1200x646 BGR image ready for the selector. The trajectories are
    drawn with the SAME reference point (ground contact gx/gy) and filter as the
    image-space control figure, but with cv2 (not matplotlib) to preserve
    the exact click->pixel correspondence.
    """
    img = cv2.imread(FRAME)
    if img is None:
        raise FileNotFoundError(FRAME)
    H, W = img.shape[:2]

    df = pd.read_csv(TRACKS)
    add_ground_contact(df)   # gx,gy = ground contact (px)

    base = img.copy()
    overlay = img.copy()
    statics = []  # (tid, mean_gx, mean_gy)

    for tid, g in df.groupby("track_id"):
        if len(g) < MIN_OBS:
            continue
        g = g.sort_values("frame_index")
        pts = g[["gx", "gy"]].to_numpy(np.int32).reshape(-1, 1, 2)
        span = max(g.gx.max() - g.gx.min(), g.gy.max() - g.gy.min())
        is_static = len(g) >= STATIC_MIN_OBS and span < STATIC_MAX_SPAN
        if is_static:
            # parked/stopped: red, thick, with dots -> stands out
            cv2.polylines(overlay, [pts], False, (0, 0, 255), 2, cv2.LINE_AA)
            for x, y in g[["gx", "gy"]].to_numpy(np.int32):
                cv2.circle(overlay, (x, y), 2, (0, 0, 255), -1)
            statics.append((int(tid), float(g.gx.mean()), float(g.gy.mean())))
        else:
            cv2.polylines(overlay, [pts], False, (180, 180, 180), 1, cv2.LINE_AA)

    cv2.addWeighted(overlay, 0.75, base, 0.25, 0, base)

    # id labels of the static clusters
    for tid, mx, my in statics:
        cv2.putText(base, f"tid{tid}", (int(mx) + 4, int(my) - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

    # orientation reminder in the corners (verified mapping)
    corners = [
        ("SUR (sup-izq) - NO excluir (cola en rojo)", (10, 24), (200, 200, 255)),
        ("OESTE (sup-der) - PARQUEO real a excluir", (W - 430, 24), (0, 255, 0)),
        ("ESTE (inf-izq)", (10, H - 14), (200, 200, 200)),
        ("NORTE (inf-der)", (W - 200, H - 14), (200, 200, 200)),
    ]
    for txt, (x, y), col in corners:
        cv2.putText(base, txt, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 1, cv2.LINE_AA)

    os.makedirs(OUT_DIR, exist_ok=True)
    cv2.imwrite(OUT_BG, base)
    print(f"[bg] fondo con {len(statics)} clusters estaticos resaltados -> {OUT_BG}")
    print(f"[bg] estaticos (tid, gx, gy): {[(t, round(x), round(y)) for t,x,y in statics]}")
    return base


class PolygonSelector:
    """Polygon marking with zoom/pan (same math as the homography selector)."""

    def __init__(self, bg, win_w=1280, win_h=760):
        self.img = bg
        self.H, self.W = self.img.shape[:2]
        self.win_w, self.win_h = win_w, win_h
        self.verts = []          # ordered list of (px,py) in image coordinates
        self.mouse = (0, 0)
        self.reset_view()

    # --- view (identical to select_homography_points.py) ---
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
    def _draw_polygon(self, view):
        wpts = [self.src_to_win(x, y) for (x, y) in self.verts]
        # edges between consecutive vertices
        for i in range(1, len(wpts)):
            cv2.line(view, (int(wpts[i - 1][0]), int(wpts[i - 1][1])),
                     (int(wpts[i][0]), int(wpts[i][1])), (0, 255, 0), 2, cv2.LINE_AA)
        # closing edge (last -> first), visually dashed with a faint color
        if len(wpts) >= 3:
            cv2.line(view, (int(wpts[-1][0]), int(wpts[-1][1])),
                     (int(wpts[0][0]), int(wpts[0][1])), (0, 160, 0), 1, cv2.LINE_AA)
        # elastic band from the last vertex to the cursor
        if wpts:
            cv2.line(view, (int(wpts[-1][0]), int(wpts[-1][1])),
                     (int(self.mouse[0]), int(self.mouse[1])), (0, 200, 200), 1, cv2.LINE_AA)
        # numbered vertices
        for i, (wx, wy) in enumerate(wpts):
            cv2.drawMarker(view, (int(wx), int(wy)), (0, 255, 0), cv2.MARKER_CROSS, 16, 2)
            cv2.putText(view, str(i), (int(wx) + 6, int(wy) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

    def render(self):
        z, tx, ty = self._affine()
        M = np.array([[z, 0, tx], [0, z, ty]], dtype=np.float32)
        view = cv2.warpAffine(self.img, M, (self.win_w, self.win_h),
                              flags=cv2.INTER_NEAREST, borderValue=(30, 30, 30))
        self._draw_polygon(view)

        lines = [
            f"Vertices: {len(self.verts)}   (encierra SOLO el parqueo de OESTE = sup-der; "
            f"NO el cluster de SUR = sup-izq)",
            "clicIZQ=anadir  u=deshacer  c=limpiar  clicDER=centrar  +/-=zoom  "
            "r=reset  w=guardar  q=salir",
        ]
        bh = 16 + 22 * len(lines)
        ov = view.copy()
        cv2.rectangle(ov, (0, 0), (self.win_w, bh), (0, 0, 0), -1)
        cv2.addWeighted(ov, 0.55, view, 0.45, 0, view)
        y = 22
        for i, ln in enumerate(lines):
            col = (0, 255, 255) if i == 0 else (180, 180, 180)
            cv2.putText(view, ln, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
            y += 22
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
            self.verts.append((fx, fy))
            print(f"  +v{len(self.verts)-1}: px=({fx:.1f},{fy:.1f})")
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
            if ch == 'u' and self.verts:
                v = self.verts.pop()
                print(f"  -deshacer: ({v[0]:.1f},{v[1]:.1f})")
            elif ch == 'c':
                self.verts.clear()
                print("  limpiado")
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
        if len(self.verts) < 3:
            print(f"  (no se guarda: {len(self.verts)} vertices; un poligono necesita >=3)")
            return
        os.makedirs(OUT_DIR, exist_ok=True)
        with open(OUT_CSV, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["idx", "pixel_x", "pixel_y"])
            for i, (px, py) in enumerate(self.verts):
                w.writerow([i, f"{px:.2f}", f"{py:.2f}"])
        # preview: the polygon drawn over the background, at full resolution
        prev = self.img.copy()
        poly = np.array([[int(x), int(y)] for x, y in self.verts], np.int32)
        shade = prev.copy()
        cv2.fillPoly(shade, [poly], (0, 0, 255))
        cv2.addWeighted(shade, 0.25, prev, 0.75, 0, prev)
        cv2.polylines(prev, [poly], True, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imwrite(OUT_PREVIEW, prev)
        print(f"\nGuardado {OUT_CSV} ({len(self.verts)} vertices) y preview {OUT_PREVIEW}.")


def selftest():
    """Verifies the win<->src mapping, the zoom-about and the saving, without opening a window."""
    bg = np.zeros((646, 1200, 3), np.uint8)
    s = PolygonSelector(bg)
    s.z, s.cx, s.cy = 3.7, 400.0, 220.0
    for (fx, fy) in [(0, 0), (600, 300), (1199, 645), (137.5, 88.4)]:
        wx, wy = s.src_to_win(fx, fy)
        bx, by = s.win_to_src(wx, wy)
        assert abs(bx - fx) < 1e-3 and abs(by - fy) < 1e-3, (fx, fy, bx, by)
    s.reset_view()
    about = (300, 400)
    sx0, sy0 = s.win_to_src(*about)
    s.zoom(1.5, about=about)
    sx1, sy1 = s.win_to_src(*about)
    assert abs(sx0 - sx1) < 1e-3 and abs(sy0 - sy1) < 1e-3
    s.verts = [(1100.0, 80.0), (1180.0, 80.0), (1180.0, 140.0), (1100.0, 140.0)]
    s.save()
    with open(OUT_CSV) as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 4 and rows[0]["idx"] == "0"
    assert abs(float(rows[2]["pixel_x"]) - 1180.0) < 1e-6
    print("selftest OK: mapeo win<->src exacto, zoom-about estable, CSV de poligono correcto.")


def main():
    if "--selftest" in sys.argv:
        selftest()
        return
    if "--build-bg-only" in sys.argv:
        build_background()
        print("Fondo generado. Revisa el PNG antes de marcar.")
        return
    print(__doc__)
    bg = build_background()
    print("\nAbriendo ventana OpenCV... (necesita display)\n")
    PolygonSelector(bg).run()


if __name__ == "__main__":
    main()

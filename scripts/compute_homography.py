"""Preliminary image->ground-plane homography (computation + validation).

ANALYSIS ONLY / preliminary infrastructure. Not a production module.

DIAGNOSIS: assigning WORLD coordinates to the curb points (CB-*) directly
gives a reprojection error of ~8 m at ALL points, because those J5 polygon
vertices end up misassigned (ambiguous). The 4 stop-line points (SL-*) fit
EXACTLY. The curb clicks are good: projected with the SL homography they fall
1-3 m from real net vertices. Therefore:

  - HOMOGRAPHY = the 4 SL points (stop lines): unambiguous world coords (lane
    endpoints at J5), well distributed, exact fit. 4 points determine the homography.
  - INDEPENDENT VALIDATION = the 3 curb clicks (CB-*): they are projected to the world and
    their distance to the nearest J5 polygon vertex is measured. This gives an error bound
    in the near/mid field WITHOUT fit circularity (the vertex comes from the net, not the
    fit). The SOUTH has no independent check point -> its reliability is evaluated
    only by the shape of the projected trajectories (separate script), not numerically.

The homography maps pixel (image) -> meters (SUMO ground plane):
    [X, Y, 1]^T ~ H · [u, v, 1]^T   (homogeneous; divided by the 3rd component).
"""

import csv
import numpy as np
import cv2

CSV = "verify_out/homography_gcp/control_points.csv"
OUT_NPY = "verify_out/homography_gcp/homography_img2ground.npy"
OUT_TXT = "verify_out/homography_gcp/homography_img2ground.txt"

# Vertices of the J5 junction polygon — world reference to validate the curb
# clicks independently of the fit. Taken from the canonical net
# (intersection_rusia.net.xml).
J5_POLY = np.array([
    (76.26, 87.82), (79.24, 72.10), (74.83, 70.75), (71.44, 68.55), (69.05, 65.51),
    (67.67, 61.63), (67.30, 56.91), (67.95, 51.34), (55.37, 48.93), (54.10, 53.35),
    (52.17, 56.78), (49.59, 59.22), (46.34, 60.67), (42.43, 61.13), (37.87, 60.60),
    (34.60, 76.27), (39.06, 77.71), (42.49, 79.98), (44.89, 83.06), (46.27, 86.97),
    (46.63, 91.69), (45.96, 97.24), (58.52, 99.72), (59.82, 95.30), (61.79, 91.86),
    (64.41, 89.39), (67.70, 87.89), (71.65, 87.37),
])

SL_IDS = ["SL-ESTE", "SL-NORTE", "SL-OESTE", "SL-SUR"]


def load():
    d = {}
    with open(CSV) as f:
        for r in csv.DictReader(f):
            if r["available"] != "True":
                continue
            d[r["id"]] = dict(px=[float(r["pixel_x"]), float(r["pixel_y"])],
                              world=[float(r["world_x"]), float(r["world_y"])],
                              zone=r["zone"])
    return d


def project(H, px):
    px = np.atleast_2d(px)
    pts = np.hstack([px, np.ones((len(px), 1))])
    w = (H @ pts.T).T
    return w[:, :2] / w[:, 2:3]


def main():
    d = load()
    sl = [i for i in SL_IDS if i in d]
    cb = [i for i in d if i.startswith("CB")]
    print(f"Puntos: {len(d)} disponibles | SL (homografía): {sl} | "
          f"CB (validación): {cb}\n")
    if len(sl) < 4:
        print(f"ERROR: hacen falta 4 líneas de pare; solo hay {len(sl)}.")
        return

    src = np.array([d[i]["px"] for i in sl])
    dst = np.array([d[i]["world"] for i in sl])
    H, _ = cv2.findHomography(src, dst, 0)

    # --- fit over the SL (should be ~0: 4 points determine the homography) ---
    print("Ajuste sobre los 4 SL (líneas de pare) — error de reproyección:")
    for i in sl:
        e = np.linalg.norm(project(H, d[i]["px"])[0] - d[i]["world"])
        print(f"  {i:9s} {d[i]['zone']:5s} {e:6.3f} m")
    print("  (exacto por construcción: 4 puntos => homografía única; el error de los\n"
          "   SL NO mide fiabilidad — para eso están los CB independientes abajo)\n")

    # --- independent validation with the curb clicks ---
    print("Validación independiente — clic de bordillo -> mundo vs vértice J5 más cercano:")
    print(f"  {'id':7s} {'zona':5s} {'proyección (m)':>20s} {'vértice net (m)':>20s} {'dist':>7s}")
    by_zone = {}
    for i in cb:
        p = project(H, d[i]["px"])[0]
        k = np.argmin(np.linalg.norm(J5_POLY - p, axis=1))
        v = J5_POLY[k]
        dist = np.linalg.norm(p - v)
        by_zone.setdefault(d[i]["zone"], []).append(dist)
        print(f"  {i:7s} {d[i]['zone']:5s} ({p[0]:7.2f},{p[1]:7.2f})    "
              f"({v[0]:7.2f},{v[1]:7.2f})   {dist:5.2f}")
    print("\nError independiente por zona (distancia clic-proyectado al vértice net):")
    for z in ["near", "mid", "far"]:
        if z in by_zone:
            e = np.array(by_zone[z])
            print(f"  {z:5s} (n={len(e)}): media={e.mean():.2f} m  max={e.max():.2f} m")
        elif z == "far":
            print("  far  (n=0): SIN punto de chequeo independiente -> "
                  "fiabilidad del SUR no validada numéricamente (ver sanity visual).")

    # --- local scale at the core ---
    c = src.mean(axis=0)
    p0, p1 = project(H, c)[0], project(H, c + [10, 0])[0]
    print(f"\nEscala local en el núcleo: ~{np.linalg.norm(p1-p0)/10:.3f} m/px")

    # --- save traceable ---
    np.save(OUT_NPY, H)
    with open(OUT_TXT, "w") as f:
        f.write("# Homografia PRELIMINAR imagen->plano suelo (metros, marco SUMO net)\n")
        f.write("# [X,Y,1]^T ~ H * [u,v,1]^T   (u,v=pixel ; X,Y=metros)\n")
        f.write(f"# Ajustada con los 4 puntos de linea de pare: {', '.join(sl)}\n")
        f.write("# cv2.findHomography metodo=0. 4 puntos => ajuste exacto.\n")
        f.write("# Validacion independiente (clics de bordillo vs vertices J5):\n")
        for z in ["near", "mid", "far"]:
            if z in by_zone:
                e = np.array(by_zone[z])
                f.write(f"#   {z}: media {e.mean():.2f} m, max {e.max():.2f} m "
                        f"(n={len(e)})\n")
        f.write("#   far(SUR): sin chequeo independiente; no validado numericamente.\n")
        for row in H:
            f.write("  ".join(f"{v: .10e}" for v in row) + "\n")
    print(f"\nGuardado:\n  {OUT_NPY}\n  {OUT_TXT}")


if __name__ == "__main__":
    main()

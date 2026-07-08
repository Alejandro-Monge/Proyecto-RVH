"""IMPROVED image->ground-plane homography — calibration + validation (CANDIDATE).

ANALYSIS ONLY. Promotes nothing to official: writes the homography as a separate CANDIDATE
(`*_CANDIDATE.*`) and the review artifacts in verify_out/homography_improved/. The
official one (verify_out/homography_gcp/) is NOT touched.

Difference from the preliminary one:
  - Preliminary: 4 points (stop lines) => EXACT fit; validation with 3 curb clicks
    (near/mid) vs J5 vertices; SOUTH not validated numerically.
  - Improved: 10 distributed points (includes far SOUTH) => OVERDETERMINED system,
    LEAST-SQUARES fit (cv2.findHomography method=0); the error is SPREAD instead
    of hidden in an exact fit.

Reports:
  1) FIT RESIDUAL: reprojection error over the 10 points themselves (not the
     honest number; with LS it is not zero but it is optimistic).
  2) LEAVE-ONE-OUT: for each point, fits with the other 9 and measures the excluded point's error
     => HONEST number (what H does at a point it did not see). Highlights SUR-extra-prof,
     the first far-field SOUTH point with an independent measurement.
  3) COMPARISON with the preliminary one (raw numbers, unqualified).
  4) TURN TEST south->west (tid99/68/88/92): must enter from the SOUTH (J8, bottom) and
     exit through the WEST (J0). If it enters from the NORTH => a reintroduced 180° turn => STOP.

The homography maps pixel -> meters:  [X,Y,1]^T ~ H · [u,v,1]^T.
"""

import csv
import argparse
import numpy as np
import cv2
import pandas as pd
import re
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from digital_shadow.geometry import add_ground_contact

# Defaults = 10-point candidate (original behavior). Overridable via CLI
# to reuse the same pipeline with another set (e.g. 17 points) WITHOUT overwriting the 10-point outputs.
CSV = "verify_out/homography_improved/control_points_improved.csv"
OUT_NPY = "verify_out/homography_improved/homography_img2ground_CANDIDATE.npy"
OUT_TXT = "verify_out/homography_improved/homography_img2ground_CANDIDATE.txt"
TRACKS = "verify_out/tracks_rusia.csv"
NET = "config/sites/rusia/sumo/intersection_rusia.net.xml"
FIG = "verify_out/homography_improved/birdseye_sanity_candidate.png"

# Net dead-ends (approach endpoints) — reference to classify entry/exit.
ARMS = {"NORTE": (44.29, 138.61), "SUR": (73.92, -13.83),
        "ESTE": (167.22, 95.28), "OESTE": (-72.24, 47.35)}
J5 = (57.01, 74.40)

# Preliminary (published independent validation) for the side-by-side comparison.
PRELIM = {"near": (2.02, 2.77, 2), "mid": (0.91, 0.91, 1), "far/SUR": None}

ARM_EDGES = {"E1", "-E1", "E3", "-E3", "E5", "-E5", "E6", "-E6", "E0", "-E0"}


def load_points():
    pts = []
    with open(CSV) as f:  # CSV is global; main() can override it from the CLI
        for r in csv.DictReader(f):
            if r["available"] != "True":
                continue
            pts.append(dict(id=r["id"], arm=r["arm"],
                            px=[float(r["pixel_x"]), float(r["pixel_y"])],
                            world=[float(r["world_x"]), float(r["world_y"])]))
    return pts


def fit(src, dst):
    H, _ = cv2.findHomography(np.array(src), np.array(dst), 0)
    return H


def project(H, px):
    px = np.atleast_2d(np.asarray(px, float))
    pts = np.hstack([px, np.ones((len(px), 1))])
    w = (H @ pts.T).T
    return w[:, :2] / w[:, 2:3]


def nearest_arm(p):
    return min(ARMS, key=lambda a: np.linalg.norm(np.array(ARMS[a]) - p))


def net_lane_shapes():
    txt = open(NET).read()
    shapes = []
    for m in re.finditer(r'<edge id="([^"]+)"[^>]*>(.*?)</edge>', txt, re.S):
        eid, body = m.group(1), m.group(2)
        if eid not in ARM_EDGES:
            continue
        for lm in re.finditer(r'shape="([^"]+)"', body):
            p = [tuple(map(float, q.split(","))) for q in lm.group(1).split()]
            shapes.append(np.array(p))
    return shapes


def main():
    global CSV, OUT_NPY, OUT_TXT, FIG
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=CSV)
    ap.add_argument("--out-npy", default=OUT_NPY)
    ap.add_argument("--out-txt", default=OUT_TXT)
    ap.add_argument("--fig", default=FIG)
    a = ap.parse_args()
    CSV, OUT_NPY, OUT_TXT, FIG = a.csv, a.out_npy, a.out_txt, a.fig
    print(f"CSV={CSV}\nOUT={OUT_NPY}\nFIG={FIG}\n")

    pts = load_points()
    n = len(pts)
    print(f"Puntos cargados: {n}\n")
    if n < 4:
        print("ERROR: <4 puntos."); return

    src = [p["px"] for p in pts]
    dst = [p["world"] for p in pts]
    H = fit(src, dst)

    # --- 1) fit residual over the 10 ---
    print("=== 1) RESIDUO DE AJUSTE (sobre los 10 puntos del propio ajuste) ===")
    print("  (mínimos cuadrados: NO es cero; optimista, no es el número honesto)")
    fit_err = []
    for p in pts:
        e = float(np.linalg.norm(project(H, p["px"])[0] - p["world"]))
        fit_err.append(e)
        print(f"  {p['id']:16s} [{p['arm']:5s}]  {e:6.3f} m")
    fit_err = np.array(fit_err)
    print(f"  --> media={fit_err.mean():.3f} m  max={fit_err.max():.3f} m\n")

    # --- 2) leave-one-out ---
    print("=== 2) VALIDACIÓN CRUZADA LEAVE-ONE-OUT (número HONESTO) ===")
    print("  (ajusta con 9, mide el error del punto excluido que la H no vio)")
    loo = {}
    for i, p in enumerate(pts):
        s = [src[j] for j in range(n) if j != i]
        d = [dst[j] for j in range(n) if j != i]
        Hi = fit(s, d)
        e = float(np.linalg.norm(project(Hi, p["px"])[0] - p["world"]))
        loo[p["id"]] = e
        star = "   <== SUR lejano, 1ª medición independiente" if p["id"] == "SUR-extra-prof" else ""
        print(f"  {p['id']:16s} [{p['arm']:5s}]  {e:6.3f} m{star}")
    loo_v = np.array(list(loo.values()))
    print(f"  --> media={loo_v.mean():.3f} m  max={loo_v.max():.3f} m")
    print(f"  --> DESTACADO SUR-extra-prof (mundo 61.90,31.18): "
          f"{loo['SUR-extra-prof']:.3f} m  (error de la H en el horizonte del SUR)\n")

    # leave-one-out grouped by approach (to compare near field vs far)
    print("  Leave-one-out por brazo:")
    by_arm = {}
    for p in pts:
        by_arm.setdefault(p["arm"], []).append(loo[p["id"]])
    for a in ["NORTE", "ESTE", "OESTE", "SUR"]:
        if a in by_arm:
            e = np.array(by_arm[a])
            print(f"    {a:5s} (n={len(e)}): media={e.mean():.3f} m  max={e.max():.3f} m")
    print()

    # --- 3) comparison with the preliminary one ---
    print("=== 3) COMPARACIÓN con la PRELIMINAR (números crudos, sin calificar) ===")
    print(f"  {'':14s} {'PRELIMINAR (valid. indep.)':<32s} {'MEJORADA (leave-one-out)':<28s}")
    print(f"  {'cercano':14s} {'near media 2.02 / max 2.77 (n=2)':<32s} ", end="")
    near = [loo[p['id']] for p in pts if p['arm'] in ('NORTE', 'ESTE')]
    near = np.array(near)
    print(f"NORTE+ESTE media {near.mean():.2f} / max {near.max():.2f} (n={len(near)})")
    print(f"  {'medio/lejano':14s} {'mid 0.91 (n=1)':<32s} ", end="")
    west = np.array([loo[p['id']] for p in pts if p['arm'] == 'OESTE'])
    print(f"OESTE media {west.mean():.2f} / max {west.max():.2f} (n={len(west)})")
    print(f"  {'SUR/horizonte':14s} {'SIN VALIDAR (n=0)':<32s} ", end="")
    south = np.array([loo[p['id']] for p in pts if p['arm'] == 'SUR'])
    print(f"SUR media {south.mean():.2f} / max {south.max():.2f} (n={len(south)})  "
          f"<-- 1er número en el SUR")
    print()

    # --- 4) turn test south->west ---
    print("=== 4) TEST DEL GIRO sur->oeste (tid99/68/88/92) ===")
    df = pd.read_csv(TRACKS)
    add_ground_contact(df)                       # gx,gy = ground contact (px); see module
    turn_ok = True
    for tid in [99, 68, 88, 92]:
        g = df[df.track_id == tid].sort_values("frame_index")
        if g.empty:
            print(f"  tid{tid}: NO existe"); continue
        w = project(H, g[["gx", "gy"]].to_numpy())
        k = max(3, len(w) // 20)
        start = w[:k].mean(axis=0)
        end = w[-k:].mean(axis=0)
        a_in, a_out = nearest_arm(start), nearest_arm(end)
        ok = (a_in == "SUR" and a_out == "OESTE")
        turn_ok = turn_ok and ok
        print(f"  tid{tid}: entra~({start[0]:6.1f},{start[1]:6.1f})->{a_in:5s}  "
              f"sale~({end[0]:6.1f},{end[1]:6.1f})->{a_out:5s}  "
              f"{'OK' if ok else 'XX'}")
    if turn_ok:
        print("  --> TEST DEL GIRO PASA: entra por SUR (J8, abajo), sale por OESTE (J0).")
    else:
        print("  --> *** TEST DEL GIRO FALLA *** revisar: posible cruce / giro reintroducido.")

    # --- save candidate ---
    np.save(OUT_NPY, H)
    with open(OUT_TXT, "w") as f:
        f.write("# Homografia MEJORADA (CANDIDATA) imagen->plano suelo (metros, marco net)\n")
        f.write("# [X,Y,1]^T ~ H * [u,v,1]^T\n")
        f.write(f"# Ajuste LS (cv2.findHomography metodo=0) con {n} puntos.\n")
        f.write(f"# Residuo ajuste: media {fit_err.mean():.3f} m, max {fit_err.max():.3f} m\n")
        f.write(f"# Leave-one-out: media {loo_v.mean():.3f} m, max {loo_v.max():.3f} m\n")
        f.write(f"# LOO SUR-extra-prof (horizonte sur): {loo['SUR-extra-prof']:.3f} m\n")
        f.write(f"# Test del giro sur->oeste: {'PASA' if turn_ok else 'FALLA'}\n")
        for row in H:
            f.write("  ".join(f"{v: .10e}" for v in row) + "\n")
    print(f"\nGuardada CANDIDATA:\n  {OUT_NPY}\n  {OUT_TXT}")

    # --- candidate sanity figure ---
    world_all = project(H, df[["gx", "gy"]].to_numpy())
    df["X"], df["Y"] = world_all[:, 0], world_all[:, 1]
    shapes = net_lane_shapes()
    fig, axes = plt.subplots(1, 2, figsize=(20, 9))
    titles = ["Vista de pájaro — extensión completa (CANDIDATA mejorada)",
              "Zoom al núcleo / campo cercano (CANDIDATA mejorada)"]
    bounds = [dict(xlim=(-90, 180), ylim=(-30, 150)),
              dict(xlim=(20, 130), ylim=(30, 120))]
    cmap = plt.get_cmap("tab20")
    for ax, title, bb in zip(axes, titles, bounds):
        for s in shapes:
            ax.plot(s[:, 0], s[:, 1], color="0.6", lw=2, zorder=1)
        # control points (green) to locate the fit
        for p in pts:
            ax.plot(*p["world"], "g+", ms=10, mew=2, zorder=4)
        k = 0
        for tid, gg in df.groupby("track_id"):
            if len(gg) < 25:
                continue
            gg = gg.sort_values("frame_index")
            ax.plot(gg["X"], gg["Y"], color=cmap(k % 20), lw=0.7, alpha=0.7, zorder=2)
            k += 1
        ax.set_title(f"{title}\n(tracks={k}; gris=carriles SUMO; verde=puntos de control)",
                     fontsize=10)
        ax.set_xlabel("X mundo (m, +este)"); ax.set_ylabel("Y mundo (m, +norte)")
        ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
        ax.set_xlim(*bb["xlim"]); ax.set_ylim(*bb["ylim"])
    fig.tight_layout()
    fig.savefig(FIG, dpi=110)
    print(f"  {FIG}")


if __name__ == "__main__":
    main()

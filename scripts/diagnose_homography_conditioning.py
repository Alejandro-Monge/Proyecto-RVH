"""Homography CONDITIONING diagnostic — configurations A/B/C/D.

ANALYSIS / MEASUREMENT ONLY. It does not calibrate a new candidate, promotes nothing, deletes
or overwrites no previous H/figure. It reads the 17 already-marked points
(control_points_17pts.csv) and does NOT mark new points.

Motivation (literature, arXiv:1803.03025 and classic DLT): what degrades the fit of
a homography is not the NUMBER of points but the CONDITIONING of its configuration.
Points near the horizon get compressed against the vanishing point: in the DLT system
their rows are nearly collinear with those of their neighbors => the condition number of the
system matrix blows up and the pixel noise is amplified over the WHOLE global fit,
not just over them. This is MEASURED with the condition number.

Configurations (all over the SAME 17 already-marked points):
  A = 10 (the originals, source=original)            -> reference (previous candidate)
  B = 17 (all)                                        -> the one that blew up
  C = 17 - {Punto-6-SUR, Punto-7-SUR} (horizon y<55px) = 15
  D = 17 - {Punto-4..7-SUR} (all the south extras) = 13

Metrics per configuration:
  - condition number of the DLT system (A matrix 2N×9), with Hartley normalization
    to isolate the GEOMETRY from the unit scale. Reported as sigma_1/sigma_8
    (how close the system is to degeneracy; sigma_9≈0 is the solution).
  - leave-one-out (LOO) aggregated and per approach (honest number: fits with N-1, measures the
    excluded).
  - LOO of the near/mid SOUTH points that are present (SUR-extra-prof, P4, P5)
    to see whether the near south IMPROVES when the horizon that contaminates it is removed.

Question the table answers: does removing the 2 horizon points (C) recover or beat
A in aggregate AND keep/improve the near south? The horizon ITSELF (far south
lane) is not fixed by any single plane — that is TPS/zones, separately.

The homography maps pixel -> meters:  [X,Y,1]^T ~ H · [u,v,1]^T.
"""

import csv
import numpy as np
import cv2

CSV = "verify_out/homography_improved/control_points_17pts.csv"

# Near/mid SOUTH points whose LOO we track explicitly (those present in each
# config). Excludes the extreme horizon (P6/P7), which is exactly the suspect to remove.
SOUTH_NEAR_MID = ["SUR-extra-prof", "Punto-4-SUR", "Punto-5-SUR"]

# Extreme south horizon (pixel y<55): the candidates for "poison".
HORIZON = ["Punto-6-SUR", "Punto-7-SUR"]
SOUTH_EXTRA = ["Punto-4-SUR", "Punto-5-SUR", "Punto-6-SUR", "Punto-7-SUR"]


def load_points():
    pts = []
    with open(CSV) as f:
        for r in csv.DictReader(f):
            if r["available"] != "True":
                continue
            pts.append(dict(id=r["id"], arm=r["arm"], source=r["source"],
                            px=[float(r["pixel_x"]), float(r["pixel_y"])],
                            world=[float(r["world_x"]), float(r["world_y"])]))
    return pts


def fit(src, dst):
    H, _ = cv2.findHomography(np.array(src), np.array(dst), 0)
    return H


def project(H, px):
    px = np.atleast_2d(np.asarray(px, float))
    h = np.hstack([px, np.ones((len(px), 1))])
    w = (H @ h.T).T
    return w[:, :2] / w[:, 2:3]


def _normalize(coords):
    """Hartley isotropic normalization: centroid to the origin, mean distance sqrt(2)."""
    coords = np.asarray(coords, float)
    c = coords.mean(axis=0)
    d = coords - c
    mean_dist = np.sqrt((d ** 2).sum(axis=1)).mean()
    s = np.sqrt(2) / mean_dist if mean_dist > 0 else 1.0
    T = np.array([[s, 0, -s * c[0]], [0, s, -s * c[1]], [0, 0, 1]])
    return T


def dlt_matrix(src, dst, normalize=True):
    """DLT A matrix (2N×9). normalize=True applies Hartley in pixel and world."""
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    if normalize:
        Ts, Td = _normalize(src), _normalize(dst)
    else:
        Ts = Td = np.eye(3)
    sh = (Ts @ np.hstack([src, np.ones((len(src), 1))]).T).T
    dh = (Td @ np.hstack([dst, np.ones((len(dst), 1))]).T).T
    rows = []
    for (x, y, _), (X, Y, _) in zip(sh, dh):
        rows.append([-x, -y, -1, 0, 0, 0, X * x, X * y, X])
        rows.append([0, 0, 0, -x, -y, -1, Y * x, Y * y, Y])
    return np.array(rows)


def condition_number(src, dst):
    """DLT system diagnostics (sigma_9≈0 is the solution).

    Returns:
      cond_raw   = sigma1/sigma8 UN-normalized (what the literature sees "blow up"
                   when there are horizon points; sensitive to unit scale).
      cond_norm  = sigma1/sigma8 with Hartley (isolates geometry; Hartley exists TO
                   fix the conditioning, so it almost always comes out benign).
      s9_norm    = normalized sigma9 = residual of a SINGLE plane: how much ALL
                   the points deviate from a single homography. It grows if a point (horizon)
                   is not coplanar/consistent with the rest. It survives normalization.
    """
    sv_raw = np.linalg.svd(dlt_matrix(src, dst, normalize=False), compute_uv=False)
    sv_n = np.linalg.svd(dlt_matrix(src, dst, normalize=True), compute_uv=False)
    return sv_raw[0] / sv_raw[7], sv_n[0] / sv_n[7], sv_n[-1]


def loo(pts):
    n = len(pts)
    src = [p["px"] for p in pts]
    dst = [p["world"] for p in pts]
    out = {}
    for i, p in enumerate(pts):
        s = [src[j] for j in range(n) if j != i]
        d = [dst[j] for j in range(n) if j != i]
        Hi = fit(s, d)
        out[p["id"]] = float(np.linalg.norm(project(Hi, p["px"])[0] - p["world"]))
    return out


def summarize(name, desc, pts):
    src = [p["px"] for p in pts]
    dst = [p["world"] for p in pts]
    c_raw, c_norm, s9 = condition_number(src, dst)
    lo = loo(pts)
    lo_v = np.array(list(lo.values()))
    by_arm = {}
    for p in pts:
        by_arm.setdefault(p["arm"], []).append(lo[p["id"]])
    south_nm = {pid: lo[pid] for pid in SOUTH_NEAR_MID if pid in lo}
    return dict(name=name, desc=desc, n=len(pts), cond_raw=c_raw, cond_norm=c_norm, s9=s9,
                loo=lo, loo_mean=lo_v.mean(), loo_max=lo_v.max(),
                by_arm=by_arm, south_nm=south_nm)


def main():
    pts = load_points()
    by_id = {p["id"]: p for p in pts}
    print(f"Puntos cargados desde {CSV}: {len(pts)}\n")

    configs = {
        "A": ("10 originales (referencia)",
              [p for p in pts if p["source"] == "original"]),
        "B": ("17 (todos)", list(pts)),
        "C": ("15 = 17 - horizonte {P6,P7-SUR}",
              [p for p in pts if p["id"] not in HORIZON]),
        "D": ("13 = 17 - todos los extra-SUR {P4..P7}",
              [p for p in pts if p["id"] not in SOUTH_EXTRA]),
    }

    res = {}
    for k, (desc, cpts) in configs.items():
        res[k] = summarize(k, desc, cpts)

    # --- detail per configuration ---
    for k in ["A", "B", "C", "D"]:
        r = res[k]
        print("=" * 78)
        print(f"CONFIG {k}: {r['desc']}  (N={r['n']})")
        print(f"  Condición DLT:  cruda(sin norm) sigma1/sigma8 = {r['cond_raw']:.3e}   |   "
              f"Hartley = {r['cond_norm']:.3e}   |   residual 1-plano sigma9 = {r['s9']:.3e}")
        print(f"  LOO agregado:  media={r['loo_mean']:.3f} m   max={r['loo_max']:.3f} m")
        print("  LOO por brazo:")
        for a in ["NORTE", "ESTE", "OESTE", "SUR"]:
            if a in r["by_arm"]:
                e = np.array(r["by_arm"][a])
                print(f"    {a:5s} (n={len(e)}): media={e.mean():.3f}  max={e.max():.3f}")
        print("  LOO sur cercano/medio:")
        for pid in SOUTH_NEAR_MID:
            if pid in r["south_nm"]:
                print(f"    {pid:16s} {r['south_nm'][pid]:7.3f} m")
            else:
                print(f"    {pid:16s}    --- (no está en esta config)")
        print()

    # --- compact comparison table ---
    print("=" * 78)
    print("TABLA COMPARATIVA A/B/C/D")
    print("-" * 78)
    hdr = f"{'cfg':3s} {'N':>3s} {'cond_raw':>10s} {'sigma9':>9s} {'LOO med':>9s} "\
          f"{'LOO max':>9s} {'SUR med':>9s} {'SUR max':>9s} {'SUR-prof':>9s} "\
          f"{'P4':>7s} {'P5':>7s}"
    print(hdr)
    for k in ["A", "B", "C", "D"]:
        r = res[k]
        sur = np.array(r["by_arm"].get("SUR", [np.nan]))
        sp = r["south_nm"].get("SUR-extra-prof", np.nan)
        p4 = r["south_nm"].get("Punto-4-SUR", np.nan)
        p5 = r["south_nm"].get("Punto-5-SUR", np.nan)

        def f(v, w=9, p=3):
            return (" " * (w - 3) + "---") if (isinstance(v, float) and np.isnan(v)) \
                else f"{v:>{w}.{p}f}"
        print(f"{k:3s} {r['n']:>3d} {r['cond_raw']:>10.2e} {r['s9']:>9.3e} "
              f"{r['loo_mean']:>9.3f} {r['loo_max']:>9.3f} "
              f"{np.nanmean(sur):>9.3f} {np.nanmax(sur):>9.3f} "
              f"{f(sp)} {f(p4,7)} {f(p5,7)}")
    print("-" * 78)
    print("cond_raw=sigma1/sigma8 DLT SIN normalizar (lo que 'explota' por horizonte);")
    print("sigma9=residual de un solo plano (crece si un punto no es coplanar/consistente).")

    # --- automatic verdict on the concrete question ---
    A, B, C, D = res["A"], res["B"], res["C"], res["D"]
    print("\n=== LECTURA (pregunta: ¿C recupera/supera a A en agregado y mantiene el "
          "sur cercano?) ===")
    print(f"  Cond cruda:  A={A['cond_raw']:.2e}  B={B['cond_raw']:.2e}  "
          f"C={C['cond_raw']:.2e}  D={D['cond_raw']:.2e}")
    print(f"  sigma9 (1-plano):  A={A['s9']:.3e}  B={B['s9']:.3e}  "
          f"C={C['s9']:.3e}  D={D['s9']:.3e}")
    print(f"  LOO agregado:  A={A['loo_mean']:.3f}  B={B['loo_mean']:.3f}  "
          f"C={C['loo_mean']:.3f}  D={D['loo_mean']:.3f}")
    sp = {k: res[k]['south_nm'].get('SUR-extra-prof', np.nan) for k in res}
    print(f"  SUR-extra-prof (sur cercano/medio, presente en todas):  "
          f"A={sp['A']:.3f}  B={sp['B']:.3f}  C={sp['C']:.3f}  D={sp['D']:.3f}")

    best = min(["A", "B", "C", "D"], key=lambda k: res[k]["loo_mean"])
    print(f"\n  Menor LOO agregado: CONFIG {best} ({res[best]['loo_mean']:.3f} m).")
    if best == "C":
        print("  => El horizonte (P6/P7) era el veneno: quitarlo recupera/supera a A y el "
              "sur cercano/medio queda con plano único. El sur LEJANO sigue siendo TPS/zona.")
    else:
        print(f"  => C no es la mejor; la comparación queda pendiente. "
              f"No se promueve nada.")
    print("\n(No se ha calibrado ninguna candidata nueva ni tocado figuras/H previas.)")


if __name__ == "__main__":
    main()

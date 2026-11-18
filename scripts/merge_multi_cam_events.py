"""Fusión Multi-Cámara (Re-ID) basada en cinemática entre Cámara 1 (cuenca_oeste) y Cámara 2 (cuenca).

Une vehículos que salen de la Cámara 1 hacia el primer edge conector y aparecen en la Cámara 2
desde el último edge conector dentro de una ventana cinemática:
    t_esperado = t_exit_cam1 + TIEMPO_VIAJE_IDEAL
    |t_entry_cam2 - t_esperado| < 5.0 s

Los vehículos sin pareja se conservan como huérfanos con prefijo `c1_` o `c2_`.
Salida: verify_out/events/merged/track_events_merged.csv
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from digital_shadow.sumo.lane_map import load_lane_map  # noqa: E402

# Constantes cinemáticas de la red entre los nodos 453731125 (Cámara 1) y 577206092 (Cámara 2).
EDGES_CONECTORES = ["334948975#2", "334948975#3"]
DISTANCIA_M = 168.71
VELOCIDAD_MAX_MS = 13.89
TIEMPO_VIAJE_IDEAL = DISTANCIA_M / VELOCIDAD_MAX_MS  # ~12.146 s
TOLERANCIA_S = 5.0

DEFAULT_CAM1_CSV = ROOT / "verify_out/events/cuenca_oeste/track_events_clean.csv"
DEFAULT_CAM2_CSV = ROOT / "verify_out/events/cuenca/track_events_clean.csv"
DEFAULT_OUT_CSV = ROOT / "verify_out/events/merged/track_events_merged.csv"


def _edge_for(lane_map, arm: str, marco: str) -> str | None:
    if not isinstance(arm, str) or arm not in lane_map.arms:
        return None
    return lane_map.arms[arm].edge(marco)


def _clean_route(edges: list[str | None]) -> list[str]:
    """Elimina None y duplicados consecutivos manteniendo el orden."""
    out: list[str] = []
    for e in edges:
        if e and (not out or out[-1] != e):
            out.append(e)
    return out


def merge_events(
    cam1_df: pd.DataFrame,
    cam2_df: pd.DataFrame,
    lm1,
    lm2,
    *,
    tolerance_s: float = TOLERANCIA_S,
) -> pd.DataFrame:
    """Ejecuta el matching cinemático 1-a-1 y construye la tabla unificada."""
    c1 = cam1_df.copy()
    c2 = cam2_df.copy()
    c1["entry_edge"] = c1["entry_arm"].map(lambda a: _edge_for(lm1, a, "entry"))
    c1["exit_edge"] = c1["exit_arm"].map(lambda a: _edge_for(lm1, a, "exit"))
    c2["entry_edge"] = c2["entry_arm"].map(lambda a: _edge_for(lm2, a, "entry"))
    c2["exit_edge"] = c2["exit_arm"].map(lambda a: _edge_for(lm2, a, "exit"))

    first_conn = EDGES_CONECTORES[0]
    last_conn = EDGES_CONECTORES[-1]

    # Generar todos los pares candidatos válidos ordenados por cercanía temporal |t_entry - t_esperado|.
    candidates: list[tuple[float, int, int, float]] = []
    for i1, r1 in c1.iterrows():
        if r1["exit_edge"] != first_conn:
            continue
        t_exit = float(r1["exit_time_s"])
        t_esp = t_exit + TIEMPO_VIAJE_IDEAL
        for i2, r2 in c2.iterrows():
            if r2["entry_edge"] != last_conn:
                continue
            t_entry = float(r2["entry_time_s"])
            if t_entry <= t_exit:
                continue
            err = abs(t_entry - t_esp)
            if err < tolerance_s:
                candidates.append((err, int(i1), int(i2), t_esp))

    candidates.sort(key=lambda item: item[0])
    used_c1: set[int] = set()
    used_c2: set[int] = set()
    matches: list[tuple[int, int, float, float]] = []
    for err, i1, i2, t_esp in candidates:
        if i1 in used_c1 or i2 in used_c2:
            continue
        used_c1.add(i1)
        used_c2.add(i2)
        matches.append((i1, i2, t_esp, err))

    rows: list[dict] = []

    # 1) Vehículos unificados (match Cámara 1 + punto ciego + Cámara 2)
    for k, (i1, i2, t_esp, err) in enumerate(
        sorted(matches, key=lambda m: float(c1.loc[m[0], "first_seen_s"])), start=1
    ):
        r1 = c1.loc[i1]
        r2 = c2.loc[i2]
        route = _clean_route([r1["entry_edge"], *EDGES_CONECTORES, r2["exit_edge"]])
        rows.append(
            {
                "global_id": f"global_{k}",
                "status": "matched",
                "cam1_track_id": int(r1["track_id"]),
                "cam2_track_id": int(r2["track_id"]),
                "vehicle_class": r2["vehicle_class"] or r1["vehicle_class"],
                "entry_arm": r1["entry_arm"],
                "exit_arm": r2["exit_arm"],
                "entry_edge": r1["entry_edge"],
                "exit_edge": r2["exit_edge"],
                "route_edges": json.dumps(route),
                "first_seen_s": round(float(r1["first_seen_s"]), 6),
                "last_seen_s": round(float(r2["last_seen_s"]), 6),
                "entry_time_s": round(float(r1["entry_time_s"]), 6),
                "exit_time_s": round(float(r2["exit_time_s"]), 6),
                "t_exit_cam1_s": round(float(r1["exit_time_s"]), 6),
                "t_esperado_cam2_s": round(t_esp, 6),
                "t_entry_cam2_s": round(float(r2["entry_time_s"]), 6),
                "match_error_s": round(err, 4),
            }
        )

    # 2) Huérfanos de Cámara 1
    for i1, r1 in c1.iterrows():
        if i1 in used_c1:
            continue
        route = _clean_route([r1["entry_edge"], r1["exit_edge"]])
        rows.append(
            {
                "global_id": f"c1_{int(r1['track_id'])}",
                "status": "orphan_cam1",
                "cam1_track_id": int(r1["track_id"]),
                "cam2_track_id": pd.NA,
                "vehicle_class": r1["vehicle_class"],
                "entry_arm": r1["entry_arm"],
                "exit_arm": r1["exit_arm"],
                "entry_edge": r1["entry_edge"] or "",
                "exit_edge": r1["exit_edge"] or "",
                "route_edges": json.dumps(route),
                "first_seen_s": round(float(r1["first_seen_s"]), 6),
                "last_seen_s": round(float(r1["last_seen_s"]), 6),
                "entry_time_s": round(float(r1["entry_time_s"]), 6),
                "exit_time_s": round(float(r1["exit_time_s"]), 6),
                "t_exit_cam1_s": round(float(r1["exit_time_s"]), 6),
                "t_esperado_cam2_s": pd.NA,
                "t_entry_cam2_s": pd.NA,
                "match_error_s": pd.NA,
            }
        )

    # 3) Huérfanos de Cámara 2
    for i2, r2 in c2.iterrows():
        if i2 in used_c2:
            continue
        route = _clean_route([r2["entry_edge"], r2["exit_edge"]])
        rows.append(
            {
                "global_id": f"c2_{int(r2['track_id'])}",
                "status": "orphan_cam2",
                "cam1_track_id": pd.NA,
                "cam2_track_id": int(r2["track_id"]),
                "vehicle_class": r2["vehicle_class"],
                "entry_arm": r2["entry_arm"],
                "exit_arm": r2["exit_arm"],
                "entry_edge": r2["entry_edge"] or "",
                "exit_edge": r2["exit_edge"] or "",
                "route_edges": json.dumps(route),
                "first_seen_s": round(float(r2["first_seen_s"]), 6),
                "last_seen_s": round(float(r2["last_seen_s"]), 6),
                "entry_time_s": round(float(r2["entry_time_s"]), 6),
                "exit_time_s": round(float(r2["exit_time_s"]), 6),
                "t_exit_cam1_s": pd.NA,
                "t_esperado_cam2_s": pd.NA,
                "t_entry_cam2_s": round(float(r2["entry_time_s"]), 6),
                "match_error_s": pd.NA,
            }
        )

    out_df = pd.DataFrame(rows)
    if not out_df.empty:
        out_df = out_df.sort_values(["first_seen_s", "global_id"]).reset_index(drop=True)
    return out_df


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cam1", type=Path, default=DEFAULT_CAM1_CSV)
    ap.add_argument("--cam2", type=Path, default=DEFAULT_CAM2_CSV)
    ap.add_argument("--site1", default="cuenca_oeste")
    ap.add_argument("--site2", default="cuenca")
    ap.add_argument("--tolerance", type=float, default=TOLERANCIA_S)
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT_CSV)
    args = ap.parse_args()

    cam1_df = pd.read_csv(args.cam1)
    cam2_df = pd.read_csv(args.cam2)
    lm1 = load_lane_map(args.site1)
    lm2 = load_lane_map(args.site2)

    merged = merge_events(cam1_df, cam2_df, lm1, lm2, tolerance_s=args.tolerance)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    merged.to_csv(args.out, index=False)

    n_match = int((merged["status"] == "matched").sum()) if not merged.empty else 0
    n_c1 = int((merged["status"] == "orphan_cam1").sum()) if not merged.empty else 0
    n_c2 = int((merged["status"] == "orphan_cam2").sum()) if not merged.empty else 0
    print("=" * 70)
    print(f"FUSIÓN MULTI-CÁMARA ({args.site1} -> {args.site2})")
    print(f"  Conectores         : {EDGES_CONECTORES}")
    print(f"  Viaje ideal        : {DISTANCIA_M} m / {VELOCIDAD_MAX_MS} m/s = {TIEMPO_VIAJE_IDEAL:.2f} s "
          f"(tolerancia ±{args.tolerance:.1f} s)")
    print(f"  Matches globales   : {n_match}")
    print(f"  Huérfanos Cam 1    : {n_c1}")
    print(f"  Huérfanos Cam 2    : {n_c2}")
    print(f"  Total registros    : {len(merged)} -> {args.out}")
    print("=" * 70)
    if n_match:
        cols = ["global_id", "cam1_track_id", "cam2_track_id", "t_exit_cam1_s",
                "t_esperado_cam2_s", "t_entry_cam2_s", "match_error_s", "route_edges"]
        print(merged[merged["status"] == "matched"][cols].to_string(index=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

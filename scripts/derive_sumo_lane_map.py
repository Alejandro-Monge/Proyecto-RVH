"""Derives the approach/lane ↔ edge/lane mapping from the SUMO net and emits
`config/sites/<id>/sumo/lane_map.yaml` with provenance.

BY GEOMETRY, not by inherited name (the names E1..E6 are NOT cardinals; the
misalignment already cost a rotated homography):

  * **approach → edge** by junction COORDINATES: the approach is classified by the
    dominant component of (approach_junction − central_junction). Each approach
    `E_n` (to=central) is paired with its departure `-E_n` (from=central, to=same
    approach junction).
  * **carril_GT → lane index** by the RIGHT-NORMAL of each edge's travel
    direction: the lanes are ordered from right to left and it is verified that
    SUMO index 0 is the rightmost lane → `index = carril_GT − 1`. Entry
    uses `E_n`, exit uses `-E_n`; the rule holds in both frames because each uses
    its own edge (lane 1 on opposite physical sides; `site_rusia_geometry Section 2bis`).

VALIDATION (5 checks + anchor): structural, coordinate anchoring vs the fixed
physical mapping, lane order by geometry, turns.csv coverage (dir vs turn),
coherence with GT v2, and the south→west anchor / south→east counter-anchor. FAIL-CLOSED:
if a check does not match, it ABORTS without writing and prints the discrepancy.

Runtime does NOT use this script: it reads the yaml via digital_shadow/sumo/lane_map.py.
"""
from __future__ import annotations

import argparse
import hashlib
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from digital_shadow.config.site import load_site_config  # noqa: E402

# FIXED physical mapping (hard-coded, by physics). The one derived
# by coordinates must MATCH this: it is the anchoring check (#2).
EXPECTED_ARM_EDGE = {"sur": "E5", "norte": "E6", "este": "E3", "oeste": "E1"}
# turn (turns.csv) -> dir (net connection attribute).
TURN_TO_DIR = {"left": "l", "right": "r", "through": "s"}
# Semantic anchors (known turns): (entry_arm, exit_arm, lane_entry, lane_exit, turn).
ANCHORS = [
    ("sur", "oeste", 3, 2, "left"),    # anchor: left turn from high lane
    ("sur", "este", 1, None, "right"),  # counter-anchor: right turn from lane 1
]


def parse_pts(shape: str) -> np.ndarray:
    """'x1,y1 x2,y2 ...' -> array (N,2)."""
    return np.array([[float(c) for c in p.split(",")] for p in shape.split()])


def load_net(net_path: Path):
    """Returns (junctions, edges) of the net. junctions[id]=(x,y,type);
    edges[id]={'from','to','lanes':[(index, pts)]} only for real edges (not internal)."""
    tree = ET.parse(net_path)
    root = tree.getroot()
    junctions = {}
    for j in root.findall("junction"):
        if j.get("function") == "internal" or j.get("id", "").startswith(":"):
            continue
        junctions[j.get("id")] = (float(j.get("x")), float(j.get("y")), j.get("type"))
    edges = {}
    for e in root.findall("edge"):
        if e.get("function") == "internal" or e.get("id", "").startswith(":"):
            continue
        lanes = sorted(
            ((int(l.get("index")), parse_pts(l.get("shape")), float(l.get("length")))
             for l in e.findall("lane")),
            key=lambda t: t[0],
        )
        edges[e.get("id")] = {"from": e.get("from"), "to": e.get("to"), "lanes": lanes}
    connections = []
    for c in root.findall("connection"):
        connections.append(
            {
                "from": c.get("from"),
                "to": c.get("to"),
                "fromLane": int(c.get("fromLane")),
                "toLane": int(c.get("toLane")),
                "dir": c.get("dir"),
                "via": c.get("via"),   # internal lane of the junction (e.g. :J5_6_0), for gap filling
            }
        )
    return junctions, edges, connections


def classify_arm(arm_xy, central_xy) -> str:
    """Classifies the approach by the dominant component of (approach − central).
    In the net (north-up) the approaches align with the cardinals: south=−Y,
    north=+Y, east=+X, west=−X. Anchoring by COORDINATES, not by name."""
    dx, dy = arm_xy[0] - central_xy[0], arm_xy[1] - central_xy[1]
    if abs(dx) >= abs(dy):
        return "este" if dx > 0 else "oeste"
    return "norte" if dy > 0 else "sur"


def order_lanes_right_to_left(lanes):
    """Orders the lanes from RIGHT to LEFT of the travel direction and returns
    [(carril_GT, SUMO_index, proj)]. carril_GT 1 = the rightmost.

    Travel direction = from the first to the last point (SUMO shapes follow the
    direction of circulation). Right-normal of (dx,dy) in the world (X east, Y north)
    = (dy,−dx) (facing north, the right is the east). Each lane's start point is
    projected onto that normal; higher projection = farther to the right."""
    dirs = np.array([pts[-1] - pts[0] for _, pts, _ in lanes])
    d = dirs.sum(axis=0)
    d = d / np.linalg.norm(d)
    right = np.array([d[1], -d[0]])  # right-normal
    starts = np.array([pts[0] for _, pts, _ in lanes])
    center = starts.mean(axis=0)
    proj = (starts - center) @ right
    order = np.argsort(-proj)  # descending: the rightmost first
    return [(rank + 1, lanes[i][0], float(proj[i])) for rank, i in enumerate(order)]


def derive(net_path: Path):
    junctions, edges, connections = load_net(net_path)
    central = next(jid for jid, (x, y, t) in junctions.items() if t == "traffic_light")
    cx, cy, _ = junctions[central]
    central_xy = (cx, cy)

    approach = {eid: e for eid, e in edges.items() if e["to"] == central}
    departure = {eid: e for eid, e in edges.items() if e["from"] == central}

    arms = {}      # approach -> {'entry_edge','entry_order','exit_edge','exit_order','arm_jct'}
    report = []    # lines of the validation report
    ok = True

    def check(cond, label):
        nonlocal ok
        report.append(f"  [{'PASS' if cond else 'FALLA'}] {label}")
        ok = ok and cond
        return cond

    report.append("CONTROL 1+2 — estructura del net + anclaje por COORDENADAS (no por nombre):")
    for eid, e in approach.items():
        arm_jct = e["from"]
        arm = classify_arm(junctions[arm_jct][:2], central_xy)
        # departure = edge from=central, to=arm_jct (the one adjacent to J5; upstream extension = route building block).
        dep = [d for d, de in departure.items() if de["to"] == arm_jct]
        check(len(dep) == 1, f"{arm}: un único departure contiguo a {central} ({dep})")
        dep_edge = dep[0]
        # Check 2: the edge derived by coordinates matches the fixed physical mapping.
        check(
            EXPECTED_ARM_EDGE.get(arm) == eid,
            f"{arm} -> approach {eid} (esperado {EXPECTED_ARM_EDGE.get(arm)}; "
            f"junction {arm_jct} en {junctions[arm_jct][:2]})",
        )
        # APPROACH longitudinal axis (for the pos in SUMO): origin = first point of lane 0
        # (where SUMO measures pos=0), direction = unit toward J5, length = lane length (from moveTo).
        idx0, pts0, len0 = next(l for l in e["lanes"] if l[0] == 0)
        u = pts0[-1] - pts0[0]
        u = u / np.linalg.norm(u)
        arms[arm] = {
            "entry_edge": eid,
            "exit_edge": dep_edge,
            "arm_jct": arm_jct,
            "entry_order": order_lanes_right_to_left(e["lanes"]),
            "exit_order": order_lanes_right_to_left(edges[dep_edge]["lanes"]),
            "axis": {"origin": [round(float(pts0[0][0]), 2), round(float(pts0[0][1]), 2)],
                     "dir": [round(float(u[0]), 4), round(float(u[1]), 4)],
                     "length": round(float(len0), 2)},
        }
    # Approach feeds J5 (incLanes): the 4 approaches exist and flow into central.
    check(set(arms) == {"sur", "norte", "este", "oeste"}, "los 4 brazos derivados")

    report.append("CONTROL 1 — conteo de lanes (approach 3; departure 1 en N/S, 2 en E/O):")
    expected_dep = {"sur": 1, "norte": 1, "este": 2, "oeste": 2}
    for arm, a in arms.items():
        n_app = len(a["entry_order"])
        n_dep = len(a["exit_order"])
        check(n_app == 3, f"{arm} approach {a['entry_edge']} tiene 3 lanes (={n_app})")
        check(
            n_dep == expected_dep[arm],
            f"{arm} departure {a['exit_edge']} tiene {expected_dep[arm]} lane(s) (={n_dep})",
        )

    report.append("CONTROL 3 — orden de lanes por GEOMETRÍA: índice 0 de SUMO = más a la derecha:")
    for arm, a in arms.items():
        for marco, order in (("entry", a["entry_order"]), ("exit", a["exit_order"])):
            seq = [idx for _, idx, _ in order]  # SUMO indices in right->left order
            check(
                seq == list(range(len(seq))),
                f"{arm}/{marco} ({a[marco + '_edge']}): carril_GT 1..N -> índices {seq} "
                f"(se espera [0..N-1] = índice 0 a la derecha)",
            )

    report.append("CONTROL 4 — cobertura de turns.csv: dir de la conexión == turn (12 movimientos):")
    turns = pd.read_csv(ROOT / "config" / "sites" / "rusia" / "turns.csv")
    allowed = turns[turns["status"] == "allowed"]
    for _, t in allowed.iterrows():
        ea, xa, turn = t["entry_arm"], t["exit_arm"], t["turn"]
        e_app = EXPECTED_ARM_EDGE[ea]
        x_dep = "-" + EXPECTED_ARM_EDGE[xa]
        conns = [c for c in connections if c["from"] == e_app and c["to"] == x_dep]
        dirs = {c["dir"] for c in conns}
        check(
            len(conns) >= 1 and dirs == {TURN_TO_DIR[turn]},
            f"{ea}->{xa} ({turn}): {e_app}->{x_dep} dir={dirs or '∅'} (espera {{'{TURN_TO_DIR[turn]}'}})",
        )

    report.append("CONTROL 5 — ancla sur->oeste / contra-ancla sur->este (giro predice el carril):")
    idx_by = {}  # (arm, marco) -> {carril_GT: index}
    for arm, a in arms.items():
        idx_by[(arm, "entry")] = {g: i for g, i, _ in a["entry_order"]}
        idx_by[(arm, "exit")] = {g: i for g, i, _ in a["exit_order"]}
    for ea, xa, c_in, c_out, turn in ANCHORS:
        e_app, x_dep = EXPECTED_ARM_EDGE[ea], "-" + EXPECTED_ARM_EDGE[xa]
        in_idx = idx_by[(ea, "entry")][c_in]
        conns = [
            c for c in connections
            if c["from"] == e_app and c["to"] == x_dep
            and c["fromLane"] == in_idx and c["dir"] == TURN_TO_DIR[turn]
        ]
        check(
            len(conns) >= 1,
            f"{ea}->{xa} {turn}: entry carril {c_in} -> {e_app}_{in_idx}; "
            f"conexión {TURN_TO_DIR[turn]} desde fromLane={in_idx} existe ({len(conns)})",
        )
        if c_out is not None:
            out_idx = idx_by[(xa, "exit")][c_out]
            check(
                any(c["toLane"] == out_idx for c in conns),
                f"{ea}->{xa}: exit carril {c_out} -> {x_dep}_{out_idx}; toLane={out_idx} existe",
            )

    report.append("CONTROL 6 — coherencia con el GT v2: exit_lane de egresos norte/sur == 1 (1 lane):")
    gt = pd.read_csv(
        ROOT / "data" / "ground_truth" / "rusia" / "ground_truth_rusia_v2_carril_processed.csv"
    )
    ns_exits = gt[gt["exit_arm"].isin(["norte", "sur"])]
    bad = ns_exits[ns_exits["exit_lane"].astype(int) != 1]
    check(
        len(bad) == 0,
        f"egresos norte/sur con exit_lane!=1: {len(bad)} (de {len(ns_exits)})",
    )
    # And all the GT lanes fit within the available lanes.
    for _, ev in gt.iterrows():
        in_ok = int(ev["entry_lane"]) in idx_by[(ev["entry_arm"], "entry")]
        out_ok = int(ev["exit_lane"]) in idx_by[(ev["exit_arm"], "exit")]
        if not (in_ok and out_ok):
            check(False, f"ev{ev['event_id']} {ev['entry_arm']}{ev['entry_lane']}->"
                         f"{ev['exit_arm']}{ev['exit_lane']} fuera de rango de lanes")
            break
    else:
        check(True, "todos los carriles del GT v2 caben en los lanes del net")

    # JUNCTION_INTERNAL: (entry_arm, exit_arm) -> internal lane of the junction (via), for gap
    # filling by interpolation over its shape (the harness requests it from traci.lane.getShape).
    app2arm = {v: k for k, v in EXPECTED_ARM_EDGE.items()}
    dep2arm = {"-" + v: k for k, v in EXPECTED_ARM_EDGE.items()}
    junction_internal = {}
    for c in connections:
        if c["from"] in app2arm and c["to"] in dep2arm and c["via"]:
            key = (app2arm[c["from"]], dep2arm[c["to"]])
            junction_internal.setdefault(key, c["via"])   # first via (represents the turn curve)
    report.append("CONTROL 7 — lane interna por movimiento (relleno de huecos): "
                  f"{len(junction_internal)}/12 movimientos con via")
    check(len(junction_internal) == 12, f"12 movimientos con lane interna ({len(junction_internal)})")

    return arms, central, report, ok, junction_internal


def emit_yaml(arms, central, net_path: Path, net_md5: str, junction_internal: dict) -> str:
    """Builds the lane_map.yaml text with provenance (without yaml.dump to
    preserve the comments)."""
    lines = [
        "# Mapeo brazo/carril <-> edge/lane del net de SUMO (sitio rusia).",
        "# DERIVADO del net por scripts/derive_sumo_lane_map.py — POR GEOMETRÍA, no por nombre",
        "# heredado (E1..E6 NO son cardinales; el desfase costó una H girada).",
        "#   brazo->edge: por coordenadas de junction (componente dominante vs el junction central).",
        "#   carril_GT->índice: por la normal-derecha del sentido de marcha (índice 0 = lane derecha).",
        "# NO EDITAR A MANO: re-deriva con el script. Runtime lee SOLO este archivo",
        "# (digital_shadow/sumo/lane_map.py); el net lo toca solo el script de derivación.",
        "#",
        "# Cruce de vocabularios (trampa de los tres 'entrada/salida'; site.yaml, geometry §2bis):",
        "#   marco entry = vehículo APROXIMÁNDOSE = edge E_n (approach a J5) = 'salida' de site.yaml.",
        "#   marco exit  = vehículo ALEJÁNDOSE   = edge -E_n (departure)     = 'entrada' de site.yaml.",
        "#   carril_GT 1 = derecha del conductor en su marcha = índice de lane 0 (regla carril-1,",
        "#   verificada por geometría; entry y exit usan cada uno su propio edge).",
        "# `lanes` mapea carril_GT (1=derecha) -> índice de lane de SUMO; el id de lane es <edge>_<índice>.",
        "#",
        "# Validado (5 controles + ancla sur->oeste).",
        "site_id: rusia",
        f"net_ref: {net_path.relative_to(ROOT)}",
        f"net_md5: {net_md5}",
        "derived_by: scripts/derive_sumo_lane_map.py",
        "derived_date: \"2026-06-28\"",
        f"central_junction: {central}",
        "arms:",
    ]
    for arm in ["sur", "norte", "este", "oeste"]:
        a = arms[arm]
        en = {g: i for g, i, _ in a["entry_order"]}
        ex = {g: i for g, i, _ in a["exit_order"]}
        en_s = ", ".join(f"{g}: {i}" for g, i in sorted(en.items()))
        ex_s = ", ".join(f"{g}: {i}" for g, i in sorted(ex.items()))
        lines.append(f"  {arm}:")
        lines.append(f"    entry: {{edge: {a['entry_edge']}, lanes: {{{en_s}}}}}   # approach hacia {central}")
        lines.append(f"    exit:  {{edge: \"{a['exit_edge']}\", lanes: {{{ex_s}}}}}   # departure desde {central}")
    lines += [
        "",
        "# Eje longitudinal de APPROACH por brazo (pieza de posición position-driven; del net, lane 0):",
        "#   origin = punto donde SUMO mide pos=0 (inicio de la lane, lado del brazo).",
        "#   dir    = unitario en sentido de marcha (hacia el central); length = longitud de lane (m).",
        "#   pos longitudinal = clip((world - origin)·dir, 0, length) -> traci.vehicle.moveTo(laneID, pos).",
        "#   El lateral NO sale de aquí: lo fija el carril clasificado (lane_map.entry) -> SUMO centra.",
        "approach_axis:",
    ]
    for arm in ["sur", "norte", "este", "oeste"]:
        ax = arms[arm]["axis"]
        lines.append(f"  {arm}: {{origin: {ax['origin']}, dir: {ax['dir']}, length: {ax['length']}}}")
    lines += [
        "",
        "# Lane INTERNA de la junción por movimiento (entry_arm->exit_arm), para el RELLENO DE HUECOS:",
        "#   el harness interpola la pos sobre la shape de esta lane (pedida a traci.lane.getShape) a la",
        "#   última velocidad observada (el giro recorrido por una recta cortaría 6-8 m de esquina).",
        "junction_internal:",
    ]
    for (ea, xa), via in sorted(junction_internal.items()):
        lines.append(f"  {ea}->{xa}: \"{via}\"")
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--site", default="rusia")
    ap.add_argument("--write", action="store_true", help="escribe el yaml si todos los controles pasan")
    args = ap.parse_args()

    site = load_site_config(args.site)
    net_path = ROOT / "config" / "sites" / args.site / "sumo" / "intersection_rusia.net.xml"
    net_md5 = hashlib.md5(net_path.read_bytes()).hexdigest()

    arms, central, report, ok, junction_internal = derive(net_path)

    print(f"=== Derivación del mapeo brazo/carril <-> edge/lane ({args.site}) ===")
    print(f"net: {net_path.relative_to(ROOT)}  md5={net_md5}  central={central}  brazos={site.arm_set}\n")
    print("MAPEO DERIVADO:")
    for arm in ["sur", "norte", "este", "oeste"]:
        a = arms[arm]
        en = {g: f"{a['entry_edge']}_{i}" for g, i, _ in a["entry_order"]}
        ex = {g: f"{a['exit_edge']}_{i}" for g, i, _ in a["exit_order"]}
        print(f"  {arm:6s} entry {a['entry_edge']:3s} {en}   exit {a['exit_edge']:4s} {ex}")
    print()
    for line in report:
        print(line)
    print()

    if not ok:
        print("RESULTADO: ALGÚN CONTROL FALLA -> NO se escribe el yaml (fail-closed). Revisa arriba.")
        sys.exit(1)
    print("RESULTADO: 5 controles + ancla PASAN.")

    yaml_text = emit_yaml(arms, central, net_path, net_md5, junction_internal)
    out = ROOT / "config" / "sites" / args.site / "sumo" / "lane_map.yaml"
    if args.write:
        out.write_text(yaml_text, encoding="utf-8")
        print(f"\nESCRITO: {out.relative_to(ROOT)}")
    else:
        print("\n--- lane_map.yaml (vista previa; usa --write para escribir) ---")
        print(yaml_text)


if __name__ == "__main__":
    main()

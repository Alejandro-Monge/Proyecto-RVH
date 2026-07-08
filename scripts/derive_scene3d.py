"""derive_scene3d — generates the STATIC SCENE for the 3D client from the SUMO net.

OFFLINE script, once per site/network (like the calibration): reads the .net.xml with
sumolib (the same network-reading stack as production — zero duplicated parsing
in the JS client) and emits `scene_<site>.json` (scene3d/1 contract, doc in
docs/modules/ws_scene3d.md). It does not run in the player or the emitter.

The JSON carries the `net_md5` of the exact input FILE: the client compares it
against `hello.net_md5` and REJECTS the stream if its scene did not derive from the same network
(the two real nets in the repo — vision 90cc6b86…, messages c6c84b27… — are
distinct files: the pin distinguishes exactly that confusion case).

Artifact DETERMINISM (same standard as the WS line): coordinates and widths
rounded to 3 decimals (mm, the quantization of the canonical pos_plane), lists
ordered by id, JSON with sort_keys and compact separators: same net -> identical
bytes.

Usage:
  python scripts/derive_scene3d.py \
      --net reconstruccion_causal/inputs/intersection_rusia.net.xml \
      --site rusia --out ws_emitter/web/scene_rusia.json
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _r3(v: float) -> float:
    return round(float(v), 3)


def _shape3(shape) -> list:
    return [[_r3(x), _r3(y)] for x, y in shape]


# lane (lane_map) -> official turn direction: c1=right,
# c2=straight, c3=left. Official signage; presentation, not measurement.
_CARRIL_MOVE = {1: "right", 2: "straight", 3: "left"}
_ARROW_SETBACK_M = 7.0   # the arrow is placed at this distance BEFORE the lane end


def _lane_arrow(net, entry_edge: str, lane_idx: int, carril: int):
    """Direction arrow for the entry lane: pos (world, SETBACK before the end,
    toward the junction), heading (travel direction), movement (lane→R/S/L). None if
    the lane does not exist in THIS net (different nets)."""
    import math
    try:
        lane = net.getEdge(entry_edge).getLanes()[lane_idx]
    except Exception:
        return None
    pts = [(float(x), float(y)) for x, y in lane.getShape()]
    if len(pts) < 2:
        return None
    (x0, y0), (x1, y1) = pts[-2], pts[-1]
    heading = math.atan2(y1 - y0, x1 - x0)        # travel direction (toward the junction)
    remain, px, py = _ARROW_SETBACK_M, x1, y1     # walk backward from the end
    for i in range(len(pts) - 1, 0, -1):
        ax, ay = pts[i]
        bx, by = pts[i - 1]
        seg = math.hypot(bx - ax, by - ay)
        if seg >= remain:
            t = remain / seg
            px, py = ax + (bx - ax) * t, ay + (by - ay) * t
            break
        remain -= seg
    return {"lane": lane.getID(), "pos": [_r3(px), _r3(py)],
            "heading": _r3(heading), "movement": _CARRIL_MOVE.get(int(carril), "straight")}


def build_scene(net_path: str, site: str) -> dict:
    """net.xml -> dict of the scene3d/2 contract (pure net + config overlays: projected
    stop_lines and direction arrows). The overlays are presentation derived from the
    SAME config as the player (stop_lines = cross_t boundary; lane_map)."""
    import sumolib
    net_path = os.path.abspath(net_path)
    net = sumolib.net.readNet(net_path, withInternal=True)
    (xmin, ymin), (xmax, ymax) = net.getBBoxXY()

    lanes, internal = [], []
    for edge in net.getEdges():
        dest = internal if edge.getFunction() == "internal" else lanes
        for lane in edge.getLanes():
            dest.append({"id": lane.getID(), "width": _r3(lane.getWidth()),
                         "shape": _shape3(lane.getShape())})
    junctions = [{"id": n.getID(), "shape": _shape3(n.getShape())}
                 for n in net.getNodes() if n.getShape()]

    # --- config overlays (presentation) ---
    sys.path.insert(0, str(ROOT))
    sys.path.insert(0, str(ROOT / "scripts"))
    from digital_shadow.sumo.lane_map import load_lane_map
    import math
    # stop_lines (3D presentation, NOT measurement): the street↔junction EDGE per approach = end
    # of the entry edge + "toward the junction" direction (into). The client draws the
    # stop line at the edge and the crosswalk offset by `into` (offsets in config). The cross_t
    # measurement lives in the pipeline (px/homography), independent of this line.
    stop_lines = {}
    arrows = []
    for arm, ae in load_lane_map(site).arms.items():
        try:
            edge = net.getEdge(ae.entry_edge)
            shp = [(float(x), float(y)) for x, y in edge.getShape()]
            (x0, y0), (x1, y1) = shp[-2], shp[-1]
            dx, dy = x1 - x0, y1 - y0
            L = math.hypot(dx, dy) or 1.0
            ix, iy = dx / L, dy / L                  # unit toward the junction
            px, py = -iy, ix                         # perpendicular (roadway width)
            W = sum(l.getWidth() for l in edge.getLanes())   # width of the ENTRY only
            # STOP line: at the edge, width of the ENTRY (the 3 lanes)
            edge_seg = [[_r3(x1 - px * W / 2), _r3(y1 - py * W / 2)],
                        [_r3(x1 + px * W / 2), _r3(y1 + py * W / 2)]]
            # CROSSWALK: the full width of the APPROACH = entry + exit (opposite edge). It
            # extends from the outer edge of the entry to the outer edge of the exit.
            cross = edge_seg
            try:
                ex = net.getEdge(ae.exit_edge)
                exs = [(float(x), float(y)) for x, y in ex.getShape()]
                Px = exs[0]                          # junction side of the exit (leaves the crossing)
                Wx = sum(l.getWidth() for l in ex.getLanes())
                side = (Px[0] - x1) * px + (Px[1] - y1) * py   # which side is the exit on?
                sgn = 1.0 if side >= 0 else -1.0
                a = [x1 - sgn * px * W / 2, y1 - sgn * py * W / 2]     # entry, far side
                b = [Px[0] + sgn * px * Wx / 2, Px[1] + sgn * py * Wx / 2]  # exit, far side
                cross = [[_r3(a[0]), _r3(a[1])], [_r3(b[0]), _r3(b[1])]]
            except Exception:
                pass
            stop_lines[arm] = {"edge": edge_seg, "cross": cross, "into": [_r3(ix), _r3(iy)]}
        except Exception:
            pass
        # direction arrows per entry lane (c1/c2/c3)
        for carril, idx in ae.entry_lanes.items():
            a = _lane_arrow(net, ae.entry_edge, int(idx), int(carril))
            if a is not None:
                arrows.append(a)

    with open(net_path, "rb") as f:
        net_md5 = hashlib.md5(f.read()).hexdigest()
    try:
        rel = os.path.relpath(net_path, ROOT)
    except ValueError:
        rel = net_path
    return {
        "schema": "scene3d/2",
        "site": str(site),
        "net_md5": net_md5,
        "net_path": rel,
        "bounds": {"xmin": _r3(xmin), "ymin": _r3(ymin),
                   "xmax": _r3(xmax), "ymax": _r3(ymax)},
        "lanes": sorted(lanes, key=lambda l: l["id"]),
        "internal_lanes": sorted(internal, key=lambda l: l["id"]),
        "junctions": sorted(junctions, key=lambda j: j["id"]),
        "stop_lines": {a: stop_lines[a] for a in sorted(stop_lines)},
        "lane_arrows": sorted(arrows, key=lambda a: a["lane"]),
    }


def scene_bytes(scene: dict) -> bytes:
    """Deterministic serialization: sort_keys + compact separators + \\n."""
    return (json.dumps(scene, sort_keys=True, separators=(",", ":"),
                       ensure_ascii=True) + "\n").encode("utf-8")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--net", required=True)
    ap.add_argument("--site", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    scene = build_scene(args.net, args.site)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    data = scene_bytes(scene)
    out.write_bytes(data)
    print(f"[scene3d] {out} escrito: net_md5={scene['net_md5']}, "
          f"lanes={len(scene['lanes'])}, internas={len(scene['internal_lanes'])}, "
          f"junctions={len(scene['junctions'])}, bounds={scene['bounds']}, "
          f"md5_artefacto={hashlib.md5(data).hexdigest()}")


if __name__ == "__main__":
    sys.exit(main())

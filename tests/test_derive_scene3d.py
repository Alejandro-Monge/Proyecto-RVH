"""Tests for derive_scene3d (scene3d/1 contract) over the REAL messages net.

pytest tests/test_derive_scene3d.py from the root of implementacion/.
"""
from __future__ import annotations

import hashlib
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from derive_scene3d import build_scene, scene_bytes  # noqa: E402

NET = ROOT / "reconstruccion_causal" / "inputs" / "intersection_rusia.net.xml"


def test_contrato_estructura_y_orden():
    s = build_scene(str(NET), "rusia")
    assert s["schema"] == "scene3d/2" and s["site"] == "rusia"
    assert set(s) == {"schema", "site", "net_md5", "net_path", "bounds",
                      "lanes", "internal_lanes", "junctions",
                      "stop_lines", "lane_arrows"}
    assert s["lanes"] and s["internal_lanes"] and s["junctions"]
    for coll in ("lanes", "internal_lanes", "junctions"):
        ids = [e["id"] for e in s[coll]]
        assert ids == sorted(ids)                       # order by id
    assert all(l["id"].startswith(":") for l in s["internal_lanes"])
    assert not any(l["id"].startswith(":") for l in s["lanes"])


def test_overlays_scene3d2():
    # stop_lines (world) per approach + direction arrows per entry lane
    s = build_scene(str(NET), "rusia")
    assert set(s["stop_lines"]) == {"norte", "sur", "este", "oeste"}
    for sl in s["stop_lines"].values():
        assert set(sl) == {"edge", "cross", "into"}              # stop line(entry)+crosswalk(approach)+into
        for seg in (sl["edge"], sl["cross"]):
            assert len(seg) == 2 and all(len(p) == 2 for p in seg)
        assert len(sl["into"]) == 2
    # 4 approaches × 3 lanes = 12 arrows, order by lane, lane movement
    arrows = s["lane_arrows"]
    assert len(arrows) == 12 and [a["lane"] for a in arrows] == sorted(a["lane"] for a in arrows)
    moves = {a["movement"] for a in arrows}
    assert moves == {"right", "straight", "left"}
    for a in arrows:
        assert set(a) == {"lane", "pos", "heading", "movement"} and len(a["pos"]) == 2


def test_redondeo_3_decimales():
    s = build_scene(str(NET), "rusia")
    raw = scene_bytes(s).decode()
    # no number serialized with more than 3 decimals
    assert not re.search(r"\d+\.\d{4,}", raw)
    for l in s["lanes"][:5]:
        for x, y in l["shape"]:
            assert round(x, 3) == x and round(y, 3) == y


def test_net_md5_y_bounds():
    s = build_scene(str(NET), "rusia")
    assert s["net_md5"] == hashlib.md5(NET.read_bytes()).hexdigest()
    b = s["bounds"]
    # convBoundary declared in the messages net
    assert (b["xmin"], b["ymin"], b["xmax"], b["ymax"]) == \
        (-72.24, -13.83, 167.22, 138.61)


def test_determinismo_bytes(tmp_path):
    b1 = scene_bytes(build_scene(str(NET), "rusia"))
    b2 = scene_bytes(build_scene(str(NET), "rusia"))
    assert b1 == b2
    assert json.loads(b1)["net_md5"] == json.loads(b2)["net_md5"]

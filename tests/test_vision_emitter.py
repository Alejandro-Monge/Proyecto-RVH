"""Block D — vision emitter: lane_angle converter + step_message of the THREE
sources + occupancy pixel_zones. (The MESSAGES non-regression lives in
test_ws_emitter.py, which pins the sumo step_message byte-exact.)

.venv/bin/python -m pytest tests/test_vision_emitter.py -v
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ws_emitter.canonical import canonical_bytes  # noqa: E402
from ws_emitter.emitter import heading_to_contract, step_message  # noqa: E402
from ws_emitter.view import VAR_ANGLE, VAR_POSITION, DecidedTickView  # noqa: E402


def test_lane_angle_conversor_verificado_contra_3_6():
    # Section 3.6: the contract convention IS that of lane_angle_towards_stop (rad, 0=+x
    # east, CCW). The converter just normalizes the range to [0, 2π).
    assert heading_to_contract((0.0, "lane_angle")) == pytest.approx(0.0)
    assert heading_to_contract((math.pi / 2, "lane_angle")) == pytest.approx(math.pi / 2)
    assert heading_to_contract((-math.pi / 2, "lane_angle")) == pytest.approx(3 * math.pi / 2)
    assert heading_to_contract((3 * math.pi, "lane_angle")) == pytest.approx(math.pi)
    for rad in (-3.0, -0.1, 0.0, 1.0, math.pi, 6.0, 10.0):
        r = heading_to_contract((rad, "lane_angle"))
        assert 0.0 <= r < 2 * math.pi


def _vision_view():
    return DecidedTickView.from_vision_tick(
        seq=5, t_sim=1.0,
        sumo_subs={"v1": {VAR_POSITION: (10.0, 20.0), VAR_ANGLE: 90.0}},  # 90°=east
        sumo_prov={"v1": "EXTRAPOLATED"}, sumo_vclass={"v1": "car"},
        boxes=[("v2", "OBSERVED", 5.0, 6.0, math.pi / 2, "lane", 2, "bus"),
               ("v3", "HELD", 7.0, 8.0, -1.0, "raw_homography", None, "car")],
        handoffs=[("v4", 3.0, 4.0, 0.9, "truck")],
        occupancy_pixel={"sur": {1: 3, 2: 0}}, guard_report={"n_extrap": 10})


def test_step_message_tres_fuentes():
    m = step_message(_vision_view())
    assert m["type"] == "step" and m["seq"] == 5
    by = {e["id"]: e for e in m["entities"]}
    # sumo: heading via sumo_angle_post_step (90°=east→0 rad)
    assert by["v1"]["source"] == "sumo" and by["v1"]["provenance"] == "EXTRAPOLATED"
    assert by["v1"]["heading"] == pytest.approx(0.0) and by["v1"]["generation"] == 0
    # vision_box: anchor + lane + heading via lane_angle
    assert by["v2"]["source"] == "vision_box" and by["v2"]["anchor"] == "lane"
    assert by["v2"]["carril"] == 2 and by["v2"]["heading"] == pytest.approx(math.pi / 2)
    assert by["v3"]["anchor"] == "raw_homography" and by["v3"]["carril"] is None
    assert "generation" not in by["v2"]                # box carries no generation
    # handoff: closed keys, no pos_plane or heading
    assert by["v4"]["source"] == "handoff" and by["v4"]["provenance"] == "HELD"
    assert by["v4"]["last_pos"] == {"x": 3.0, "y": 4.0, "t": 0.9}
    assert "pos_plane" not in by["v4"] and "heading" not in by["v4"]


def test_occupancy_pixel_zones_en_el_step():
    m = step_message(_vision_view())
    occ = m["occupancy"]
    assert occ["kind"] == "pixel_zones" and occ["semantics"] == "conteo_crudo"
    assert occ["per_zone"]["sur"]["1"] == 3 and occ["per_zone"]["sur"]["2"] == 0
    assert "per_lane" not in occ                       # not the messages union


def test_step_visión_canoniza_sin_error():
    m = step_message(_vision_view())
    b = canonical_bytes(m)
    assert b"t_emit" not in b                           # forbidden in the log
    assert json.loads(b.decode())["seq"] == 5          # valid round-trip

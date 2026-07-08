"""Unit tests of the occupancy rule.

Includes check (a) of the smoke gating: the REAL config lane_map (not a
mock) must produce EXACTLY the 12 approach lane IDs to J5. If the
shape of the YAML changes and load_approach_lanes stops matching, this FAILS:
it is reported, not silently patched.
"""
import os
import sys

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import occupancy_core as oc

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
LANE_MAP_REAL = os.path.join(REPO_ROOT, "config", "sites", "rusia", "sumo",
                             "lane_map.yaml")

LANE_MAP_MOCK = {"arms": {
    "sur":   {"entry": {"edge": "E5", "lanes": {1: 0, 2: 1, 3: 2}}},
    "norte": {"entry": {"edge": "E6", "lanes": {1: 0, 2: 1, 3: 2}}},
    "este":  {"entry": {"edge": "E3", "lanes": {1: 0, 2: 1, 3: 2}}},
    "oeste": {"entry": {"edge": "E1", "lanes": {1: 0, 2: 1, 3: 2}}},
}}
AL = oc.load_approach_lanes(LANE_MAP_MOCK)


def test_lane_map_real_produce_los_12_ids_exactos():
    """(a) of the gating: against the real lane_map, versioned in config."""
    with open(LANE_MAP_REAL) as f:
        lane_map = yaml.safe_load(f)
    al = oc.load_approach_lanes(lane_map)
    esperados = {"E1_0", "E1_1", "E1_2", "E3_0", "E3_1", "E3_2",
                 "E5_0", "E5_1", "E5_2", "E6_0", "E6_1", "E6_2"}
    assert set(al.keys()) == esperados
    # correct arms by the documented geometry of the lane_map (not cardinals
    # by name): E1=oeste, E3=este, E5=sur, E6=norte; lane 1 = index 0
    assert al["E1_0"] == ("oeste", 1)
    assert al["E3_1"] == ("este", 2)
    assert al["E5_2"] == ("sur", 3)
    assert al["E6_0"] == ("norte", 1)


def test_mapa_12_carriles():
    assert len(AL) == 12
    assert AL["E5_0"] == ("sur", 1) and AL["E1_2"] == ("oeste", 3)


def test_vacio_emite_ceros_explicitos():
    counts = oc.occupancy_counts([], AL, 0.1)
    assert len(counts) == 12
    assert all(c == {"ocupacion": 0, "detenidos": 0} for c in counts.values())


def test_internos_departures_y_vacios_fuera():
    counts = oc.occupancy_counts(
        [("a", ":J5_6_0", 0.0), ("b", "-E5_0", 0.0), ("c", "", 0.0)], AL, 0.1)
    assert sum(c["ocupacion"] for c in counts.values()) == 0


def test_conteo_y_borde_del_umbral():
    counts = oc.occupancy_counts(
        [("a", "E5_0", 0.05),    # stopped (< 0.1)
         ("b", "E5_0", 0.1),     # NOT stopped: STRICT comparison, like SUMO
         ("c", "E5_1", 5.0)], AL, 0.1)
    assert counts[("sur", 1)] == {"ocupacion": 2, "detenidos": 1}
    assert counts[("sur", 2)] == {"ocupacion": 1, "detenidos": 0}
    # per arm = sum of its lanes
    assert sum(counts[("sur", k)]["ocupacion"] for k in (1, 2, 3)) == 3

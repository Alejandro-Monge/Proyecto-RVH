"""Tests for the occupancy layer: fail-fast loader + pure instantaneous count.

The synthetic fixtures are built RELATIVE TO THE REAL south stop line (site
geometry), so that the orientation check is tested against the same reference
production uses and not against a convention invented by the test.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
import derive_track_events as dte
from digital_shadow.causal.contract import check_pure_lookup
from digital_shadow.occupancy import (ApproachZones, ArmZone, LaneZone,
                                      load_approach_zones, occupancy_counts)

DIMS = (1200, 646)


# --------------------------------------------------------------------------- #
# Synthetic geometry anchored to the real south stop line
# --------------------------------------------------------------------------- #
def _sur_frame():
    """(p1, u, n_out): origin of the south stop line, direction of the stop line and
    the OUTWARD normal (toward the approach side, away from the center)."""
    seg = dte.load_stop_lines("rusia")["sur"]
    center = np.array([DIMS[0] / 2.0, DIMS[1] / 2.0])
    p1, nh = dte._inward_normal(seg, center)   # nh points TOWARD the center
    u = seg[1] - seg[0]
    u = u / np.linalg.norm(u)
    return p1, u, -nh


def _quad(s0, s1, d0, d1):
    """Quadrilateral in (s=along stop line, d=outward depth) coords: the FIRST
    TWO vertices at d1 (outer side = ENTRY), the last two at d0."""
    p1, u, n_out = _sur_frame()
    a = p1 + s0 * u + d1 * n_out
    b = p1 + s1 * u + d1 * n_out
    c = p1 + s1 * u + d0 * n_out
    d = p1 + s0 * u + d0 * n_out
    return [[float(x), float(y)] for x, y in (a, b, c, d)]


def _pt(s, d):
    p1, u, n_out = _sur_frame()
    p = p1 + s * u + d * n_out
    return (float(p[0]), float(p[1]))


def _yaml_doc(arm_poly, lanes, dims=DIMS):
    return {"site_id": "rusia", "reference_frame": "x", "frame_width": dims[0],
            "frame_height": dims[1],
            "approach_zones": {"sur": {
                "arm": {"polygon_px": arm_poly, "entry_side": [0, 1]},
                "lanes": {k: {"polygon_px": p, "entry_side": es,
                              "coverage": "partial"}
                          for k, (p, es) in lanes.items()}}}}


def _write(tmp_path, doc):
    p = tmp_path / "approach_zones.yaml"
    p.write_text(yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")
    return p


ARM = _quad(5, 115, 5, 200)          # approach: stop-line width, 200 px deep
LANE_A = _quad(10, 40, 10, 180)      # lane inside, does not overlap B
LANE_B = _quad(50, 80, 10, 120)
LANE_OUT = _quad(90, 150, 10, 120)   # sticks out of the approach (s up to 150 > 115)
LANE_OVERLAP = _quad(30, 60, 10, 120)  # overlaps LANE_A


# --------------------------------------------------------------------------- #
# Loader: promoted real config
# --------------------------------------------------------------------------- #
def test_load_real_config():
    z = load_approach_zones("rusia")
    assert sorted(z.arms) == ["este", "norte", "oeste", "sur"]
    assert (z.frame_width, z.frame_height) == DIMS
    for arm, az in z.arms.items():
        assert len(az.polygon) >= 4 and sorted(az.lanes) == [1, 2, 3]
        for lz in az.lanes.values():
            assert lz.coverage in ("full", "partial")
    # DECLARED coverages
    assert all(z.arms[a].lanes[k].coverage == "full"
               for a in ("norte", "este") for k in (1, 2, 3))
    assert z.arms["sur"].lanes[1].coverage == "full"
    assert z.arms["sur"].lanes[2].coverage == "partial"
    assert z.arms["oeste"].lanes[2].coverage == "partial"


# --------------------------------------------------------------------------- #
# Loader: fail-fast with synthetic fixtures (names the offending polygon)
# --------------------------------------------------------------------------- #
def test_load_aborta_entry_side_invertido(tmp_path):
    inv = [ARM[2], ARM[3], ARM[0], ARM[1]]     # v0-v1 ends up flush with the stop line
    p = _write(tmp_path, _yaml_doc(inv, {1: (LANE_A, [0, 1])}))
    with pytest.raises(ValueError, match=r"sur_arm.*INVERTIDO"):
        load_approach_zones("rusia", path=p)


def test_load_aborta_carril_fuera_del_brazo(tmp_path):
    p = _write(tmp_path, _yaml_doc(ARM, {1: (LANE_OUT, [0, 1])}))
    with pytest.raises(ValueError, match=r"sur_lane1.*NO contenido"):
        load_approach_zones("rusia", path=p)


def test_load_aborta_carriles_solapados(tmp_path):
    p = _write(tmp_path, _yaml_doc(ARM, {1: (LANE_A, [0, 1]),
                                         2: (LANE_OVERLAP, [0, 1])}))
    with pytest.raises(ValueError, match=r"sur_lane1: solapa con sur_lane2"):
        load_approach_zones("rusia", path=p)


def test_load_aborta_dims_distintas(tmp_path):
    p = _write(tmp_path, _yaml_doc(ARM, {1: (LANE_A, [0, 1])}, dims=(999, 999)))
    with pytest.raises(ValueError, match=r"dims"):
        load_approach_zones("rusia", path=p)


# --------------------------------------------------------------------------- #
# Instantaneous count (synthetic zones built directly, without YAML)
# --------------------------------------------------------------------------- #
def _zones():
    return ApproachZones("rusia", *DIMS, arms={"sur": ArmZone(
        np.asarray(ARM, float), (0, 1),
        {1: LaneZone(np.asarray(LANE_A, float), (0, 1), "full"),
         2: LaneZone(np.asarray(LANE_B, float), (0, 1), "partial")})})


def test_counts_carril_total_y_residual():
    z = _zones()
    pts = [_pt(25, 50),    # inside lane1
           _pt(65, 50),    # inside lane2
           _pt(100, 50),   # inside the approach, outside lanes -> sin_carril
           _pt(60, 300)]   # outside the approach (deeper than 200) -> not counted
    c = occupancy_counts(pts, z)["sur"]
    assert c == {"total": 3, "lanes": {1: 1, 2: 1}, "sin_carril": 1}
    assert c["total"] == sum(c["lanes"].values()) + c["sin_carril"]


def test_counts_sin_fallback_a_carril_vecino():
    # back of lane2 (deeper than its 120 px, inside the approach): NEVER assigned
    # to a lane; falls into the explicit residual channel.
    c = occupancy_counts([_pt(65, 160)], _zones())["sur"]
    assert c == {"total": 1, "lanes": {1: 0, 2: 0}, "sin_carril": 1}


def test_counts_exclusion_antes_de_contar():
    z = _zones()
    excl = np.asarray(_quad(55, 90, 30, 80), float)   # covers the lane2 point
    pts = [_pt(25, 50), _pt(65, 50)]
    c = occupancy_counts(pts, z, exclusion_polygons=[excl])["sur"]
    assert c == {"total": 1, "lanes": {1: 1, 2: 0}, "sin_carril": 0}


def test_counts_frame_vacio():
    assert occupancy_counts([], _zones())["sur"] == \
        {"total": 0, "lanes": {1: 0, 2: 0}, "sin_carril": 0}


def test_counts_config_real_invariante():
    """Over the PROMOTED config: synthetic points and the structural invariant."""
    z = load_approach_zones("rusia")
    rng = np.random.default_rng(7)
    pts = rng.uniform([0, 0], DIMS, size=(500, 2))
    out = occupancy_counts(pts, z)
    for arm, c in out.items():
        assert c["total"] == sum(c["lanes"].values()) + c["sin_carril"], arm
        assert all(v >= 0 for v in c["lanes"].values())


# --------------------------------------------------------------------------- #
# Causal contract: pure lookup (I1/I3), with the production verifier
# --------------------------------------------------------------------------- #
def test_occupancy_es_lookup_puro():
    z = _zones()
    inputs = [((_pt(25, 50), _pt(65, 50)), z),
              ((_pt(100, 50),), z),
              ((), z),
              ((_pt(25, 50), _pt(25, 50), _pt(60, 300)), z)]
    res = check_pure_lookup(occupancy_counts, inputs, label="occupancy_counts")
    assert res["I1_pure_lookup"] and res["I3_no_whole_event"]

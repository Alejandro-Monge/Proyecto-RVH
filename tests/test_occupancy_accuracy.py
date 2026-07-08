"""Tests for the occupancy measurement instrument (pure functions of
scripts/measure_occupancy_accuracy.py): monotone aligner, GT interval builder
and system interval merge/threshold. The edge cases reproduce those of the real GT
(ev74/75 censoring tie, ev34 entry inversion, `na` convention of ev77-79)."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))

from measure_occupancy_accuracy import (  # noqa: E402
    align_monotone, apply_threshold, build_gt_intervals, count_series,
    merge_intervals, presence_intervals,
)


def ev(t0, t1, cens_in=False, cens_out=False, **kw):
    return dict(t0=t0, t1=t1, cens_in=cens_in, cens_out=cens_out, **kw)


# --------------------------- monotone aligner --------------------------- #
def test_align_identidad_simple():
    gt = [ev(0, 10), ev(5, 20), ev(12, 30)]
    sy = [ev(0.8, 10.2), ev(5.9, 20.3), ev(13.1, 30.1)]
    pairs, gt_um, sy_um = align_monotone(gt, sy)
    assert sorted(pairs) == [(0, 0), (1, 1), (2, 2)] and not gt_um and not sy_um


def test_align_fn_como_hueco():
    """The system loses the middle vehicle: gap in GT, no index carry-over."""
    gt = [ev(0, 10), ev(5, 20), ev(12, 30)]
    sy = [ev(0.8, 10.2), ev(13.1, 30.1)]
    pairs, gt_um, sy_um = align_monotone(gt, sy)
    assert sorted(pairs) == [(0, 0), (2, 1)] and gt_um == [1] and not sy_um


def test_align_fp_como_hueco():
    """Spurious system interval: left unmatched, does not steal a neighbor's."""
    gt = [ev(0, 10), ev(12, 30)]
    sy = [ev(0.8, 10.2), ev(40.0, 45.0), ev(13.1, 30.1)]
    pairs, gt_um, sy_um = align_monotone(gt, sy)
    assert sorted(pairs) == [(0, 0), (1, 2)] and not gt_um and sy_um == [1]


def test_align_empate_censura_lo_resuelve_la_entrada():
    """Real ev74/75 case: both with censored exit (t1=180) — only the entry separates them."""
    gt = [ev(93.9, 180.0, cens_out=True), ev(98.2, 180.0, cens_out=True)]
    sy = [ev(98.9, 180.0, cens_out=True), ev(94.5, 180.0, cens_out=True)]
    pairs, _, _ = align_monotone(gt, sy)
    assert sorted(pairs) == [(0, 1), (1, 0)]


def test_align_inversion_de_entrada_bien_por_salida():
    """Real ev34 case (sur/c1): enters first but exits last — the EXIT order rules."""
    gt = [ev(43.9, 64.9), ev(54.3, 54.6), ev(57.7, 58.3)]
    sy = [ev(44.8, 65.1), ev(54.9, 54.9), ev(58.2, 58.6)]
    pairs, gt_um, sy_um = align_monotone(gt, sy)
    assert sorted(pairs) == [(0, 0), (1, 1), (2, 2)] and not gt_um and not sy_um


# ------------------------- GT builder / conventions ------------------------- #
def _occ_row(ev_id, crosses, arm, lane, cens, enter, exit_, ):
    return dict(event_id=ev_id, crosses=crosses, entry_arm=arm, entry_lane=lane,
                vehicle_class="car", entrada_censurada=cens,
                cruce_pare_time_s_ref=np.nan, zone_enter_time_s=enter,
                zone_exit_time_s=exit_, notes="")


def test_build_gt_censuras_na_y_exit_de_cruzador():
    occ = pd.DataFrame([
        _occ_row(1, "si", "este", "2", "no", 10.0, np.nan),      # normal crosser
        _occ_row(2, "si", "este", "2", "si", np.nan, np.nan),    # entry censoring
        _occ_row(74, "no", "norte", "1", "no", 93.9, np.nan),    # exit censoring
        _occ_row(77, "no", "sur", "sin_carril", "no", "na", "na"),  # never crosses detector
    ])
    cruces = pd.DataFrame([dict(event_id=1, entry_time_s=25.5),
                           dict(event_id=2, entry_time_s=30.0)])
    iv, na = build_gt_intervals(occ, cruces)
    r1 = iv[iv.ev == 1].iloc[0]
    assert (r1.t0, r1.t1, r1.cens_in, r1.cens_out) == (10.0, 25.5, False, False)
    r2 = iv[iv.ev == 2].iloc[0]
    assert (r2.t0, r2.t1, r2.cens_in) == (0.0, 30.0, True)
    r74 = iv[iv.ev == 74].iloc[0]
    assert (r74.t0, r74.t1, r74.cens_out) == (93.9, 180.0, True)
    assert list(na.ev) == [77] and 77 not in set(iv.ev)


def test_build_gt_aborta_con_vacio_sin_censura():
    occ = pd.DataFrame([_occ_row(9, "no", "sur", "2", "no", np.nan, np.nan)])
    with pytest.raises(ValueError, match="ev9"):
        build_gt_intervals(occ, pd.DataFrame(columns=["event_id", "entry_time_s"]))


# ------------------------- system intervals ------------------------- #
def test_presencia_fusion_umbral_y_serie():
    dt = 12.0 / 355.0
    ts = np.concatenate([np.arange(0, 1.0, dt),          # presence 0-1s
                         np.arange(1.6, 2.0, dt),        # 0.6s gap
                         np.arange(9.0, 9.05, dt)])      # short isolated blip
    raw = presence_intervals(ts)
    assert len(raw) == 3
    fus = merge_intervals(raw, gap=1.0)                  # merges the 0.6s gap
    assert len(fus) == 2 and fus[0][0] == 0.0 and abs(fus[0][1] - 2.0) < 0.1
    fil = apply_threshold(fus, theta=0.5)                # the blip (<0.1s) dies
    assert len(fil) == 1
    grid = np.arange(0.0, 3.0, dt)
    n = count_series(fil, grid)
    assert n.max() == 1 and n[0] == 1 and n[-1] == 0


def test_defaults_del_punto_de_trabajo_vienen_de_config():
    """The fixed working point lives in approach_zones.yaml:measurement,
    not embedded in the instrument."""
    from measure_occupancy_accuracy import load_measurement_defaults
    theta, gap = load_measurement_defaults()
    assert (theta, gap) == (0.75, 0.5)

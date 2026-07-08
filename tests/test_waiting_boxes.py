"""Tests for the pure logic of the waiting boxes (digital_shadow/occupancy/waiting_boxes.py):
rectangle geometry (nose at the stop line, SUMO convention), lane angle, and the causal life
cycle of the tracker (appearance after accumulated θ, persistence ≤ g, extinction on stop-line
crossing / exhausted gap, class vote from the past)."""
from __future__ import annotations

import numpy as np

from digital_shadow.occupancy.waiting_boxes import (
    WaitingBoxTracker, anchor_to_lane, box_corners, lane_angle_towards_stop,
    lane_axis_from_shape,
)

DT = 12.0 / 355.0


def test_box_corners_eje_x():
    """Nose at the given point, body backward (SUMO convention: position = front)."""
    c = box_corners((10.0, 0.0), 0.0, 5.0, 2.0)
    assert np.allclose(sorted(c[:, 0]), [5.0, 5.0, 10.0, 10.0])   # x: tail at 5, nose at 10
    assert np.allclose(sorted(c[:, 1]), [-1.0, -1.0, 1.0, 1.0])   # width centered
    # rectangle area = L*W (shoelace)
    x, y = c[:, 0], c[:, 1]
    area = 0.5 * abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))
    assert np.isclose(area, 10.0)


def test_box_corners_rotado_45():
    ang = np.pi / 4
    c = box_corners((0.0, 0.0), ang, 12.0, 2.5)
    centro = c.mean(axis=0)                      # geometric center L/2 behind the nose
    assert np.allclose(centro, [-6.0 * np.cos(ang), -6.0 * np.sin(ang)], atol=1e-9)
    x, y = c[:, 0], c[:, 1]
    area = 0.5 * abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))
    assert np.isclose(area, 30.0)


def test_lane_angle_ultimo_segmento():
    shape = [(0.0, 0.0), (10.0, 0.0), (10.0, 5.0)]   # ends heading toward +y
    assert np.isclose(lane_angle_towards_stop(shape), np.pi / 2)


def _feed(trk, t0, n, tid=1, xy=(3.0, 4.0), cls=5):
    out = {}
    for k in range(n):
        out = trk.update(t0 + k * DT, [(tid, *xy, cls, None)])
    return out


def test_tracker_aparece_tras_theta_acumulada():
    trk = WaitingBoxTracker(theta_s=0.5, gap_s=0.5, cross_t_by_track={}, frame_dt=DT)
    vis = _feed(trk, 0.0, 10)                    # 10 frames ≈ 0.34 s < θ
    assert vis == {}
    vis = _feed(trk, 10 * DT, 6)                 # accumulates ≈ 0.54 s ≥ θ
    assert 1 in vis and vis[1][2] == 5           # visible, class = bus (vote)


def test_tracker_persiste_hueco_corto_y_muere_en_hueco_largo():
    trk = WaitingBoxTracker(theta_s=0.1, gap_s=0.5, cross_t_by_track={}, frame_dt=DT)
    _feed(trk, 0.0, 5)
    # gap of 0.3 s (< g): no presence, but the box stays visible
    vis = trk.update(5 * DT + 0.3, [])
    assert 1 in vis
    # gap of 0.6 s (> g): the box dies
    vis = trk.update(5 * DT + 0.3 + 0.6, [])
    assert vis == {}


def test_tracker_extincion_al_cruzar_pare():
    trk = WaitingBoxTracker(theta_s=0.1, gap_s=1.0, cross_t_by_track={1: 1.0}, frame_dt=DT)
    vis = _feed(trk, 0.0, 10)
    assert 1 in vis                              # waiting, visible
    vis = trk.update(1.0, [(1, 3.0, 4.0, 5, None)])  # t = cross_t: the causal is born, box out
    assert vis == {}


def test_tracker_voto_de_clase_del_pasado():
    trk = WaitingBoxTracker(theta_s=0.0, gap_s=1.0, cross_t_by_track={}, frame_dt=DT)
    for k in range(3):
        vis = trk.update(k * DT, [(1, 0.0, 0.0, 2, None)])   # 3 frames car
    for k in range(3, 8):
        vis = trk.update(k * DT, [(1, 0.0, 0.0, 5, None)])   # 5 frames bus
    assert vis[1][2] == 5                        # majority from the PAST: bus


# ------------------------- anchoring to the lane axis ------------------------- #
def test_lane_axis_y_anclaje_mata_lateral_conserva_longitudinal():
    axis = lane_axis_from_shape([(0.0, 0.0), (100.0, 0.0)])   # straight lane on x
    assert np.isclose(axis.length, 100.0)
    # skewed homographic point: 3 m off the lane, at 40 m depth
    ax, ay = anchor_to_lane(axis, (40.0, 3.0))
    assert np.isclose(ax, 40.0) and np.isclose(ay, 0.0)       # lateral killed, longitudinal preserved
    # and the pos is saturated to the lane range (longitudinal_pos contract)
    ax, _ = anchor_to_lane(axis, (150.0, -2.0))
    assert np.isclose(ax, 100.0)


def test_voto_de_carril_estabiliza_flip_de_frontera():
    trk = WaitingBoxTracker(theta_s=0.0, gap_s=1.0, cross_t_by_track={}, frame_dt=DT)
    for k in range(6):
        vis = trk.update(k * DT, [(1, 0.0, 0.0, 2, 3)])       # 6 frames in c3
    for k in range(6, 8):
        vis = trk.update(k * DT, [(1, 0.0, 0.0, 2, 2)])       # 2 frames of flip to c2
    assert vis[1][3] == 3                                     # the vote from the past rules


def test_carril_none_en_fondo_sin_zona_de_carril():
    trk = WaitingBoxTracker(theta_s=0.0, gap_s=1.0, cross_t_by_track={}, frame_dt=DT)
    vis = trk.update(0.0, [(1, 0.0, 0.0, 2, None)])
    assert vis[1][3] is None                                  # declared fallback: no anchoring


# ------------------------- c2-sur rule (sticky by vote) ------------------------- #
def test_c2sur_sticky_asignado_una_vez_se_mantiene():
    """The assignment (lane=2 once c1-empty holds) is injected ONCE; the later frames
    with None do not erase it — 'keeps c2 while it exists'."""
    trk = WaitingBoxTracker(theta_s=0.0, gap_s=1.0, cross_t_by_track={}, frame_dt=DT)
    assert not trk.has_lane_votes(1)
    vis = trk.update(0.0, [(1, 0.0, 0.0, 2, 2)])          # assignment frame
    assert trk.has_lane_votes(1) and vis[1][3] == 2
    for k in range(1, 30):                                 # afterward: no zone lane
        vis = trk.update(k * DT, [(1, 0.0, 0.0, 2, None)])
    assert vis[1][3] == 2                                  # sticky: still anchored to c2


def test_c2sur_sin_asignacion_queda_crudo():
    """If the condition never holds (always fed None), the box stays sin_carril
    (raw drawing), and has_lane_votes stays False to retry."""
    trk = WaitingBoxTracker(theta_s=0.0, gap_s=1.0, cross_t_by_track={}, frame_dt=DT)
    for k in range(10):
        vis = trk.update(k * DT, [(1, 0.0, 0.0, 2, None)])
    assert vis[1][3] is None and not trk.has_lane_votes(1)


def test_c2sur_hueco_reabre_la_evaluacion():
    """Gap > g kills the box and resets the votes: the new life re-evaluates the condition."""
    trk = WaitingBoxTracker(theta_s=0.0, gap_s=0.5, cross_t_by_track={}, frame_dt=DT)
    trk.update(0.0, [(1, 0.0, 0.0, 2, 2)])
    assert trk.has_lane_votes(1)
    trk.update(1.0, [])                                    # gap 1.0 s > g: dies
    assert not trk.has_lane_votes(1)                       # evaluation reopened

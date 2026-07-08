"""Minimal test for the GT↔shadow matcher (scripts/evaluate_shadow_fidelity.match).

Verifies the design properties over synthetic data (no SUMO, no real pipelines):
strict one-to-one, complete-preferred-over-partial, quarantine of a degenerate track (transit),
duplicate detection, and that an UNCERTAIN-heading partial is not forced by heading.
"""
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "scripts"))
from evaluate_shadow_fidelity import match


def sig(vid, ent, t_in, exit_arm=None, t_out=None, head=None, end_speed=0.0, stability=0.0, traj=None):
    """`traj=(ts,gx,gy)` = ground-contact trail (for the duplicate-by-coexistence criterion).
    By default a trivial far-away trail (does not coexist with anyone)."""
    complete = t_out is not None
    if traj is None:
        ts = np.array([t_in - 1.0, t_in, t_in + 3.0]); gx = np.full(3, 9000.0 + t_in); gy = np.zeros(3)
    else:
        ts, gx, gy = (np.asarray(a, float) for a in traj)
    return dict(vid=vid, ent_arm=ent, t_in=t_in, complete=complete, exit_arm=exit_arm, t_out=t_out,
                transit=(t_out - t_in if complete else None), head=head,
                end_speed=end_speed, stability=stability, _ts=ts, _gx=gx, _gy=gy,
                entry_lane_label="", exit_carril=None)


def gt_df(rows):
    return pd.DataFrame(rows, columns=["event_id", "entry_arm", "entry_lane", "exit_arm",
                                       "exit_lane", "entry_time_s", "exit_time_s"])


def test_complete_preferred_and_duplicate_by_coexistence():
    """1 GT; one complete and one STACKED partial (same trail) -> the complete wins; the partial is
    a duplicate by trajectory COEXISTENCE (not by entry collision). Strict one-to-one."""
    tj = ([9.0, 12.0, 15.0], [500.0, 500.0, 500.0], [300.0, 300.0, 300.0])   # shared trail
    gt = gt_df([[1, "sur", 1, "este", 1, 10.0, 18.0]])
    sigs = {
        1: sig("vComp", "sur", 10.1, exit_arm="este", t_out=18.1, traj=tj),
        2: sig("vPart", "sur", 10.0, head="este", end_speed=3.0, stability=0.9, traj=tj),  # stacked
    }
    r = match(sigs, gt)
    assert r["complete"] == {1: 1}                 # the complete takes ev1
    assert 2 in r["duplicate"]                     # the partial coexists (0 px) -> duplicate
    vals = list({**r["complete"], **r["partial_reliable"], **r["partial_uncertain"]}.values())
    assert len(vals) == len(set(vals))             # no GT with 2 vehicles


def test_close_entry_but_separated_is_not_duplicate():
    """v86/v109 case: enters almost at the same time as a matched one but runs SEPARATE -> it is NOT a
    duplicate (generic ghost). This is the failure that the entry-collision criterion made."""
    gt = gt_df([[1, "sur", 1, "este", 1, 10.0, 18.0]])
    sigs = {
        1: sig("vMatch", "sur", 10.0, exit_arm="este", t_out=18.0,
               traj=([9.0, 12.0, 15.0], [500.0, 500.0, 500.0], [300.0, 300.0, 300.0])),
        2: sig("vDistinct", "sur", 10.1, head="este", end_speed=3.0, stability=0.9,
               traj=([9.0, 12.0, 15.0], [560.0, 560.0, 560.0], [300.0, 300.0, 300.0])),  # 60 px apart
    }
    r = match(sigs, gt)
    assert r["complete"] == {1: 1}
    assert 2 not in r["duplicate"]                 # 60 px > 25 px -> NOT a duplicate
    assert 2 in r["ghost"]                         # it is a generic ghost (different vehicle)


def test_two_movements_match_correctly():
    """2 GT on the same entry approach but different exits; two completes -> each to its own
    by the EXIT (not by the entry, which is almost identical)."""
    gt = gt_df([[1, "norte", 3, "este", 1, 87.0, 92.0],
                [2, "norte", 1, "sur", 1, 87.1, 93.0]])
    sigs = {
        1: sig("vEste", "norte", 87.05, exit_arm="este", t_out=92.1),
        2: sig("vSur", "norte", 87.15, exit_arm="sur", t_out=93.1),
    }
    r = match(sigs, gt)
    assert r["complete"] == {1: 1, 2: 2}


def test_track_degenerate_quarantined():
    """A complete with anomalous transit (>threshold) is quarantined and does NOT match its GT (stays a loss)."""
    gt = gt_df([[1, "sur", 1, "este", 1, 58.0, 66.0]])
    sigs = {1: sig("vDegen", "sur", 58.2, exit_arm="este", t_out=100.0)}  # transit 41.8s
    r = match(sigs, gt)
    assert 1 in r["degenerate"]
    assert r["complete"] == {}
    assert 1 in r["loss"]


def test_ghost_split_real_vs_spurious():
    """A ghost whose ENTRY crosses a GT LOSS (same approach±W_in) = real under-tracked (recall);
    one that crosses no loss = spurious (precision). Crossing by approach+time, NOT lane/class."""
    gt = gt_df([[1, "este", 2, "oeste", 1, 10.0, 16.0],    # vComp matches it
                [2, "este", 2, "norte", 1, 20.0, 26.0]])    # LOSS (nobody matches it)
    sigs = {
        1: sig("vComp", "este", 10.1, exit_arm="oeste", t_out=16.1,
               traj=([9.0, 12.0, 15.0], [500., 500., 500.], [300., 300., 300.])),
        2: sig("vReal", "este", 20.1, exit_arm="sur", t_out=26.1,   # complete, exit≠ev2 → no match; crosses ev2 by entry
               traj=([19., 22., 25.], [700., 700., 700.], [300., 300., 300.])),
        3: sig("vSpur", "sur", 50.0, exit_arm="este", t_out=56.0,   # far from any loss
               traj=([49., 52., 55.], [100., 100., 100.], [100., 100., 100.])),
    }
    r = match(sigs, gt)
    assert 2 in r["ghost_real"] and 2 not in r["ghost_spurious"]      # crosses the ev2 loss (este@20)
    assert 3 in r["ghost_spurious"] and 3 not in r["ghost_real"]      # no loss nearby


def test_uncertain_partial_not_forced_by_heading():
    """UNCERTAIN-heading partial (low speed) is NOT filtered by heading; matches by entry into the residual
    and is labeled partial-uncertain (not partial-reliable)."""
    gt = gt_df([[1, "oeste", 3, "norte", 1, 11.29, 37.9]])
    # stopped partial (low end_speed) with a noisy geometric heading that does NOT point to 'norte'
    sigs = {1: sig("vPolicia", "oeste", 11.30, head="este", end_speed=1.0, stability=0.6)}
    r = match(sigs, gt)
    assert 1 in r["partial_uncertain"]     # matches by entry even though the heading (este) != GT (norte)
    assert 1 not in r["partial_reliable"]
    assert r["partial_uncertain"][1] == 1

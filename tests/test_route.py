"""Tests of the route piece (`digital_shadow/sumo/route.py`).

Pure lookups (lane→turn→edge) + append-only sealed commit + the 3 evidence paths.
Deterministic, real config. pytest or `python -m tests.test_route`.
"""

from __future__ import annotations

from digital_shadow.sumo.lane_map import load_lane_map
from digital_shadow.sumo.route import (
    EV_ARM_ONLY,
    EV_OBSERVADO,
    EV_PREDICHO,
    RouteCommitter,
    load_lane_turn_rule,
    load_turns,
    predict_turn,
    predicted_exit,
)

RULE = load_lane_turn_rule("rusia")
TURNS = load_turns("rusia")
LM = load_lane_map("rusia")


# --------------------------------------------------------------------------- #
# Pure lookups
# --------------------------------------------------------------------------- #
def test_predict_turn_por_carril():
    # oeste SPLIT (grouped→per_lane) and sur ENABLED
    # (arm_only→per_lane); rule re-derived from the GT by derive_lane_turn_rule.py.
    assert predict_turn(RULE, "norte", "2") == "left"
    assert predict_turn(RULE, "oeste", "2") == "through"
    assert predict_turn(RULE, "oeste", "3") == "through"
    assert predict_turn(RULE, "oeste", "1") == "right"


def test_predict_turn_sur_per_lane():
    """Sur ENABLED per_lane: lane→turn from the GT (100 % LOO). The empty label
    (unclassified) still has no prediction."""
    assert predict_turn(RULE, "sur", "1") == "right"
    assert predict_turn(RULE, "sur", "2") == "through"
    assert predict_turn(RULE, "sur", "3") == "left"
    assert predict_turn(RULE, "sur", "") is None


def test_predicted_exit_resuelve_edge():
    turn, exit_arm, exit_edge = predicted_exit(RULE, TURNS, LM, "norte", "2")
    assert (turn, exit_arm, exit_edge) == ("left", "este", "-E3")
    # sur enabled: lane 1 → right → este
    assert predicted_exit(RULE, TURNS, LM, "sur", "1") == ("right", "este", "-E3")


# --------------------------------------------------------------------------- #
# Sealed commit, append-only, 3 paths
# --------------------------------------------------------------------------- #
def test_committer_predicho_y_observado_son_dos_commits_sellados():
    c = RouteCommitter("norte", LM, TURNS, RULE)
    c.commit_origin(10.0, "2")
    snap1 = c.commits()
    c.commit_destination_predicted(10.2, "2")        # predicted: norte/2 → left → este → -E3
    snap2 = c.commits()
    c.commit_destination_observed(15.0, "este")      # observed: NEW commit
    snap3 = c.commits()
    # append-only: each previous snapshot is a prefix of the next (not rewritten)
    assert snap2[: len(snap1)] == snap1
    assert snap3[: len(snap2)] == snap2
    assert len(snap1) == 1 and len(snap3) == 3
    pred = snap3[1]
    assert pred.evidence == EV_PREDICHO and pred.exit_edge == "-E3"
    assert abs(pred.delay - 0.2) < 1e-9                # delay = commit - entry
    assert snap3[2].evidence == EV_OBSERVADO
    assert c.prediction_hit() is True                  # predicted este == observed este


def test_committer_sur_cae_a_arm_only():
    c = RouteCommitter("sur", LM, TURNS, RULE)
    c.commit_origin(20.0, "")
    d = c.commit_destination_predicted(20.1, "")
    assert d.evidence == EV_ARM_ONLY and d.exit_edge is None
    assert c.target_edge() is None                      # no destination → no changeTarget


def test_target_edge_es_el_ultimo_destino():
    c = RouteCommitter("norte", LM, TURNS, RULE)
    c.commit_origin(1.0, "1")
    c.commit_destination_predicted(1.1, "1")            # norte/1 → through → sur → -E5
    assert c.target_edge() == "-E5"
    c.commit_destination_observed(5.0, "este")          # refines to este → -E3
    assert c.target_edge() == "-E3"


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"OK  {name}")
    print("todos los tests pasaron")

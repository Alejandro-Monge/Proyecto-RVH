"""PURE arm/lane ↔ edge/lane lookup of the SUMO net (config-driven).

It answers a single question: given an **arm** (norte/sur/este/oeste), a **frame**
(`entry`/`exit`), and the **GT lane** (1 = the vehicle's right in its travel
direction), on which **edge** and at which SUMO **lane index** does it live? It is the
prerequisite for instantiating a replicated vehicle.

Design (see docs/modules/sumo_lane_map.md):

  * **Runtime reads ONLY config** (`config/sites/<id>/sumo/lane_map.yaml`); the net is
    touched only by the derivation script `scripts/derive_sumo_lane_map.py`.
  * **Pure, stateless lookup:** `target(brazo, marco, carril) -> (edge, index)` is a
    dictionary, idempotent and order-independent. It does not depend on the set of
    events nor on any instant. By construction it gives the same result whether the
    (deferred) validation script calls it or the causal bridge that comes later → it
    does not interfere with the causal boundary.
  * **Entry and exit are resolved SEPARATELY** (it does not require the complete event):
    the origin is queried when committing the entry; the destination, later, when
    committing the exit (possibly after the buffer). This enables the SUMO causal
    pattern (insert with the origin edge and complete the route afterward).

Vocabulary crossing (the trap of the three "entry/exit"; see
`config/sites/rusia/site.yaml` and `docs/site_rusia_geometry.md Section 2bis`):

  * `marco="entry"` = vehicle APPROACHING the intersection = edge `E_n` (approach
    toward J5) = what site.yaml calls "salida" (exit) lanes.
  * `marco="exit"`  = vehicle MOVING AWAY = edge `-E_n` (departure) = what site.yaml
    calls "entrada" (entry) lanes.

  The GT `carril` (1 = the driver's right in their travel direction) corresponds to the
  SUMO lane index by the rule `index = carril − 1` (index 0 = the right lane of the edge
  itself), VERIFIED by geometry in the derivation script; entry and exit each use their
  own edge, which is why the rule holds in both frames even though their lanes 1 fall on
  physically opposite sides.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import Mapping, Optional

import yaml

from digital_shadow.config.site import find_project_root

# Valid frames = GT/event semantics (origin / destination of the vehicle).
ENTRY = "entry"
EXIT = "exit"
_FRAMES = (ENTRY, EXIT)


@dataclass(frozen=True)
class ArmEdges:
    """Edges and carril_GT→lane index mapping of an arm, for entry and exit."""

    entry_edge: str
    entry_lanes: Mapping[int, int]  # carril_GT (1=right) -> SUMO lane index
    exit_edge: str
    exit_lanes: Mapping[int, int]

    def edge(self, marco: str) -> str:
        return self.entry_edge if marco == ENTRY else self.exit_edge

    def lanes(self, marco: str) -> Mapping[int, int]:
        return self.entry_lanes if marco == ENTRY else self.exit_lanes


@dataclass(frozen=True)
class ApproachAxis:
    """Longitudinal axis of an arm's approach edge (from the net), for the pos in SUMO.

    `origin` = point where SUMO measures pos=0 (start of the lane, arm side); `direction`
    = unit vector in the travel direction (toward the central junction); `length` = lane
    length (m). The longitudinal pos of a world point is
    `clip((p − origin)·direction, 0, length)`. It does NOT give the lateral: that is fixed
    by the lane (SUMO centers on the lane).
    """

    origin: tuple[float, float]
    direction: tuple[float, float]
    length: float


@dataclass(frozen=True)
class SumoLaneMap:
    """Arm/lane ↔ edge/lane mapping of a site. Immutable; pure lookup."""

    site_id: str
    central_junction: str
    arms: Mapping[str, ArmEdges]
    # Longitudinal approach axis per arm (position piece). Empty default so as not to break
    # constructors that only use the mapping (contract co-evolution).
    approach_axis: Mapping[str, ApproachAxis] = field(
        default_factory=lambda: MappingProxyType({})
    )
    # (entry_arm, exit_arm) -> internal junction lane (via), for the harness gap filling.
    junction_internal: Mapping[tuple, str] = field(
        default_factory=lambda: MappingProxyType({})
    )

    def internal_lane(self, entry_arm: str, exit_arm: str) -> Optional[str]:
        """Internal junction lane for the movement (None if it does not exist)."""
        return self.junction_internal.get((entry_arm, exit_arm))

    def target(self, brazo: str, marco: str, carril_gt: int) -> tuple[str, int]:
        """(edge_id, lane_index) for the given arm/frame/lane.

        Raises `ValueError`/`KeyError` with a clear message if the arm, the frame, or
        the lane do not exist (e.g. requesting egress lane 2 on norte/sur, which only
        has 1 lane moving away, is an invalid request, not a default).
        """
        if marco not in _FRAMES:
            raise ValueError(
                f"marco {marco!r} inválido; usa {ENTRY!r} (approach, E_n) o "
                f"{EXIT!r} (departure, -E_n)"
            )
        if brazo not in self.arms:
            raise KeyError(
                f"brazo {brazo!r} no está en el mapeo; brazos: {sorted(self.arms)}"
            )
        arm = self.arms[brazo]
        lanes = arm.lanes(marco)
        carril_gt = int(carril_gt)
        if carril_gt not in lanes:
            raise KeyError(
                f"carril_GT {carril_gt} no existe en {brazo}/{marco} "
                f"(edge {arm.edge(marco)}); carriles válidos: {sorted(lanes)}"
            )
        return arm.edge(marco), lanes[carril_gt]

    def lane_id(self, brazo: str, marco: str, carril_gt: int) -> str:
        """SUMO lane id (`<edge>_<index>`, e.g. 'E5_2')."""
        edge, idx = self.target(brazo, marco, carril_gt)
        return f"{edge}_{idx}"


def sumo_target(
    lane_map: SumoLaneMap, brazo: str, marco: str, carril_gt: int
) -> tuple[str, int]:
    """Functional form of the lookup: `(edge_id, lane_index)`.

    Pure: the mapping is passed explicitly (config-driven), with no global state or I/O.
    Equivalent to `lane_map.target(brazo, marco, carril_gt)`.
    """
    return lane_map.target(brazo, marco, carril_gt)


def load_lane_map(
    site_id: str = "rusia", project_root: Optional[Path] = None
) -> SumoLaneMap:
    """Load `config/sites/<site_id>/sumo/lane_map.yaml` (derived from the net)."""
    root = project_root or find_project_root()
    path = root / "config" / "sites" / site_id / "sumo" / "lane_map.yaml"
    if not path.is_file():
        raise FileNotFoundError(
            f"No existe el mapeo de lanes: {path}. Genéralo con "
            f"scripts/derive_sumo_lane_map.py (lee el net)."
        )
    with open(path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)

    arms: dict[str, ArmEdges] = {}
    for arm, spec in data["arms"].items():
        en_edge = spec["entry"].get("edge")
        ex_edge = spec["exit"].get("edge")
        arms[arm] = ArmEdges(
            entry_edge=str(en_edge) if en_edge is not None else None,
            entry_lanes=MappingProxyType(
                {int(k): int(v) for k, v in (spec["entry"].get("lanes") or {}).items()}
            ),
            exit_edge=str(ex_edge) if ex_edge is not None else None,
            exit_lanes=MappingProxyType(
                {int(k): int(v) for k, v in (spec["exit"].get("lanes") or {}).items()}
            ),
        )
    axis: dict[str, ApproachAxis] = {}
    for arm, ax in (data.get("approach_axis") or {}).items():
        axis[arm] = ApproachAxis(
            origin=(float(ax["origin"][0]), float(ax["origin"][1])),
            direction=(float(ax["dir"][0]), float(ax["dir"][1])),
            length=float(ax["length"]),
        )
    internal = {}
    for mv, via in (data.get("junction_internal") or {}).items():
        ea, xa = mv.split("->")
        internal[(ea, xa)] = str(via)
    return SumoLaneMap(
        site_id=str(data["site_id"]),
        central_junction=str(data["central_junction"]),
        arms=MappingProxyType(arms),
        approach_axis=MappingProxyType(axis),
        junction_internal=MappingProxyType(internal),
    )

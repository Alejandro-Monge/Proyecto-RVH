"""DecidedTickView — WS emitter interface (anti-leak layer 1), MESSAGES shadow.

Implements docs/modules/decided_tick_view.md (design FROZEN together with contract
v3.0.0; pin 5e7b43a1 — this module implements it without mutating it; implementation
doc in docs/modules/ws_view_impl.md). The view is built at the post-step coupling
point of causal_player (after the occupancy block, before the attach sub-steps) ONLY
from state already decided for the tick.

STRUCTURAL INVARIANT (what this module guarantees, not promises):
  - the constructor signature has NO parameter for veh, chains, commits,
    GuardedMessageStore or trackers: there is no leak path through the interface;
  - ZERO RETENTION of what is received: the parameters are READ and discarded; the
    view stores exclusively new, immutable objects (str/int/float/tuples/
    frozen dataclasses/MappingProxyType over its own copies). In particular the
    `st` values of last_applied are tr.states[k] from the player's tracker: keeping
    that reference would make trackers[].states reachable — they are extracted field
    by field. Verified by a REAL REACHABILITY test (transitive closure of
    gc.get_referents with bait objects, tests/test_decided_tick_view.py).
  - immutable after construction (frozen dataclass): the emitter reads, never writes.

Eligibility (ghost exclusion, contract Section 3.3): THE SAME rule and iteration as
the player's occupancy layer — it iterates the subscription cache and requires
active, not parked and with applied state (last_applied != None). A vehicle in
staging or park NEVER enters sumo_entities.

In messages box_entities and handoff_entities are EMPTY TUPLES ALWAYS PRESENT
(uniform interface between shadows); in vision (`from_vision_tick`) they are populated
with the vision_box and handoff sources. Both classmethods share the same zero-
retention discipline and the same structural guarantee (no parameter with a future).
"""
from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType

# TraCI subscription constants. Values checked against traci.constants of the real
# environment (/usr/share/sumo/tools/traci/constants.py); the soft check below
# re-checks them on every import where traci exists, without imposing the dependency
# on the pure tests.
VAR_POSITION = 0x42   # post-step position, SUMO network plane [m,m]
VAR_ANGLE = 0x43      # post-step angle from TraCI
try:  # pragma: no cover - only environments with SUMO
    import traci.constants as _tc
    assert VAR_POSITION == _tc.VAR_POSITION and VAR_ANGLE == _tc.VAR_ANGLE, \
        "constantes TraCI divergen del entorno: revisar antes de emitir"
except ImportError:
    pass

# CLOSED vocabulary of contract Section 3.3 (inverse of the player's VTYPE_MAP)
_VCLASS = {"shadow_car": "car", "shadow_bus": "bus", "shadow_truck": "truck"}
_HEADING_SOURCE = "sumo_angle_post_step"   # source convention label; the
                                           # conversion to the contract convention
                                           # (Section 3.6) belongs to the emitter serializer


@dataclass(frozen=True)
class SumoEntity:
    """source=sumo entity of contract Section 3.3 (messages and vision). Frozen."""
    id: str
    source: str
    provenance: str
    pos_plane: tuple            # (x, y) SUMO plane — new floats, post-step
    heading_raw: tuple          # (value, source convention label)
    vclass: str
    generation: int


@dataclass(frozen=True)
class BoxEntity:
    """source=vision_box entity of contract Section 3.3 (VISION ONLY). Frozen. The
    `anchor` (lane|raw_homography, Section 3.7) is the anchoring quality; the client
    MUST draw raw_homography differently."""
    id: str                     # canonical id f"v{root}" (stable box→handoff→veh)
    source: str
    provenance: str             # OBSERVED (box present) | HELD (coasting)
    pos_plane: tuple            # (x, y) plane — anchored position decided by the layer
    heading_raw: tuple          # (lane angle in mathematical rad, "lane_angle")
    anchor: str                 # "lane" | "raw_homography"
    carril: object              # int | None (voted lane; None = background with no zone)
    vclass: str


@dataclass(frozen=True)
class HandoffEntity:
    """source=handoff entity of contract Section 3.3 (VISION ONLY). CLOSED KEYS
    {id, source, last_pos, vclass} + provenance=HELD fixed. No pos_plane (null in the
    gap), no heading, NO prospective field (ETA/duration/progress)."""
    id: str                     # same canonical id f"v{root}" as the box and the veh
    source: str
    provenance: str             # HELD (by definition: only frozen past)
    last_pos: tuple             # (x, y, t): last OBSERVED position of the box
    vclass: str


@dataclass(frozen=True)
class DecidedTickView:
    """DECIDED state of a tick, immutable. The emitter reads from here and only
    from here; the graph reachable from this instance contains no structure with a
    future (blacklist of decided_tick_view.md Section 5)."""
    seq: int
    t_sim: float
    sumo_entities: tuple        # tuple[SumoEntity], ordered by id
    box_entities: tuple         # () in messages — present for uniformity
    handoff_entities: tuple     # () in messages — present for uniformity
    occupancy: MappingProxyType
    guard_counters: MappingProxyType

    @classmethod
    def from_messages_tick(cls, *, seq, t_sim, active, parked, last_applied,
                           applied_ids, subs, generation, assigned_type,
                           occupancy_counts, guard_report):
        """Builds the view of the MESSAGES logical tick from the live feeders of the
        coupling point. All parameters are read and DISCARDED.

        active/parked: sets of ids. last_applied: veh -> st (tracker dict; scalars are
        extracted, nothing is retained). applied_ids: vehs with NEW state this tick
        (the other revealed ones were HELD). subs: getAllSubscriptionResults()
        post-step. generation/assigned_type: dicts of scalars.
        occupancy_counts: {(approach, lane): {"ocupacion", "detenidos"}} already
        computed in-loop. guard_report: dict of live guard counters.
        """
        ents = []
        # SAME iteration source and rule as the player's occupancy: the post-step
        # subscription cache + REVEALED shadow eligibility. sorted():
        # order determinism (byte-for-byte reproducibility of the log).
        for veh in sorted(subs):
            if veh not in active or veh in parked:
                continue                     # staging/park ghost: excluded
            st = last_applied.get(veh)
            if st is None:
                continue                     # no applied state yet: staging
            r = subs[veh]
            pos = r.get(VAR_POSITION)
            ang = r.get(VAR_ANGLE)
            if pos is None or ang is None:
                continue                     # incomplete cache (should not happen: the
                                             # subscription lists both variables)
            # HELD (contract Section 3.5): no new state this tick, or a frozen frame
            # of the interpolation (held_frame). Otherwise, the NATIVE provenance.
            held = (veh not in applied_ids) or bool(st.get("held_frame"))
            ents.append(SumoEntity(
                id=str(veh),
                source="sumo",
                provenance="HELD" if held else str(st["provenance"]),
                pos_plane=(float(pos[0]), float(pos[1])),
                heading_raw=(float(ang), _HEADING_SOURCE),
                vclass=_VCLASS.get(assigned_type.get(veh), "car"),
                generation=int(generation.get(veh, 0)),
            ))
        per_lane: dict = {}
        for (brazo, carril), c in occupancy_counts.items():
            per_lane.setdefault(str(brazo), {})[str(carril)] = MappingProxyType(
                {"ocupacion": int(c["ocupacion"]), "detenidos": int(c["detenidos"])})
        occupancy = MappingProxyType({
            "kind": "sumo_lanes",
            "per_lane": MappingProxyType(
                {b: MappingProxyType(lanes) for b, lanes in per_lane.items()}),
        })
        guard = MappingProxyType({k: (float(v) if isinstance(v, float) else v)
                                  for k, v in dict(guard_report).items()})
        return cls(seq=int(seq), t_sim=float(t_sim),
                   sumo_entities=tuple(ents), box_entities=(),
                   handoff_entities=(), occupancy=occupancy,
                   guard_counters=guard)

    @classmethod
    def from_vision_tick(cls, *, seq, t_sim, sumo_subs, sumo_prov, sumo_vclass,
                         boxes, handoffs, occupancy_pixel, guard_report,
                         sumo_generation=None):
        """VISION shadow: assembles the THREE sources (sumo/vision_box/handoff) from
        ALREADY DECIDED feeders of run_live's coupling point. Everything is READ and
        DISCARDED (zero retention): only new, immutable scalars are retained.

        STRUCTURAL INVARIANT: the signature has NO parameter for veh/chains/wb/
        v["draw"]/inst_t/tid_root → there is no leak path through the interface
        (verified by gc.get_referents with baits, tests/test_vision_view.py). The box
        and handoff feeders arrive as SCALARS already resolved by the caller (canonical
        id f"v{root}", COCO vclass): the view does not touch the identity map.

        sumo_subs: {vid: {VAR_POSITION:(x,y), VAR_ANGLE:a}} (post-step TraCI cache).
        sumo_prov: {vid: provenance} decided this tick (causal_pos branch).
        sumo_vclass: {vid: 'car'|'bus'|'truck'}; sumo_generation: {vid:int} (vision=0).
        boxes: iterable of (id, prov, x, y, heading_rad, anchor, carril, vclass).
        handoffs: iterable of (id, x, y, t, vclass) — HELD, pos_plane=null → last_pos.
        occupancy_pixel: {zone: {lane: count}} RAW (not a metric). guard_report: dict.
        """
        gen = sumo_generation or {}
        ents = []
        for vid in sorted(sumo_subs):
            if vid not in sumo_prov:
                continue                     # present but not decided this tick
            r = sumo_subs[vid]
            pos, ang = r.get(VAR_POSITION), r.get(VAR_ANGLE)
            if pos is None or ang is None:
                continue
            ents.append(SumoEntity(
                id=str(vid), source="sumo", provenance=str(sumo_prov[vid]),
                pos_plane=(float(pos[0]), float(pos[1])),
                heading_raw=(float(ang), _HEADING_SOURCE),
                vclass=str(sumo_vclass.get(vid, "car")),
                generation=int(gen.get(vid, 0))))
        box_ents = [BoxEntity(
            id=str(bid), source="vision_box", provenance=str(prov),
            pos_plane=(float(bx), float(by)),
            heading_raw=(float(bh), "lane_angle"), anchor=str(anchor),
            carril=(None if carril is None else int(carril)), vclass=str(vclass))
            for (bid, prov, bx, by, bh, anchor, carril, vclass)
            in sorted(boxes, key=lambda b: b[0])]
        ho_ents = [HandoffEntity(
            id=str(hid), source="handoff", provenance="HELD",
            last_pos=(float(hx), float(hy), float(ht)), vclass=str(vclass))
            for (hid, hx, hy, ht, vclass) in sorted(handoffs, key=lambda h: h[0])]
        per_zone = {str(z): MappingProxyType({str(k): int(v) for k, v in lanes.items()})
                    for z, lanes in occupancy_pixel.items()}
        occupancy = MappingProxyType({
            "kind": "pixel_zones",
            "semantics": "conteo_crudo",     # RAW, NOT the paper metric (Section 3.4/Section 3.7)
            "per_zone": MappingProxyType(per_zone)})
        guard = MappingProxyType({k: (float(v) if isinstance(v, float) else v)
                                  for k, v in dict(guard_report).items()})
        return cls(seq=int(seq), t_sim=float(t_sim),
                   sumo_entities=tuple(ents), box_entities=tuple(box_ents),
                   handoff_entities=tuple(ho_ents), occupancy=occupancy,
                   guard_counters=guard)

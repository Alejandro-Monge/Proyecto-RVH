"""occupancy_core — approach occupancy rule (SINGLE implementation).

occupancy(lane) = number of vehicles whose lane_id belongs to the approach
lanes of the arm (arms.<arm>.entry of the config lane_map; NEVER from the
net nor hardcoded). stopped(lane) = the same ones with speed < halting
threshold (0.1 m/s, SUMO's native criterion — see
config/sites/rusia/occupancy.yaml). Per arm = sum of its lanes: it is
aggregated downstream (comparator/figures), not recounted here.

A vehicle on an internal lane :J5_* is outside the set BY CONSTRUCTION
(it is crossing, not approaching): no internal lane appears in the map.

Consumers (same function, no copies):
  - causal_player.py: live post-simulationStep hook, per logical tick
    (speed = SUMO's VAR_SPEED, the post-step speed; NOT the message's
    speed_csv).
  - gt2_occupancy.py: offline GT generator over the layer-0 fcd
    (speed = the fcd's speed attribute, which is the same SUMO speed).

Pure function: no TraCI, no I/O, no state.
"""


def load_approach_lanes(lane_map):
    """lane_map (already-parsed YAML dict) -> {lane_id: (brazo, carril_GT)}.

    The lane id is formed as <edge>_<index>, a convention documented in the
    lane_map.yaml itself. carril_GT: 1 = the driver's right in their travel."""
    out = {}
    for brazo, spec in lane_map["arms"].items():
        edge = spec["entry"]["edge"]
        for carril_gt, lane_idx in spec["entry"]["lanes"].items():
            out[f"{edge}_{lane_idx}"] = (brazo, int(carril_gt))
    return out


def occupancy_counts(states, approach_lanes, halting_speed_mps):
    """states: iterable of (vehicle_id, lane_id, speed_mps).

    Returns {(brazo, carril_GT): {"ocupacion": n, "detenidos": n}} with ALL the
    lane_map keys present (explicit zeros: the time series carries one row per
    lane and tick even with an empty lane — a gap in the series would be
    ambiguous). STRICT stop speed < threshold, the same as SUMO's halting."""
    counts = {key: {"ocupacion": 0, "detenidos": 0}
              for key in approach_lanes.values()}
    for _veh, lane_id, speed in states:
        key = approach_lanes.get(lane_id)
        if key is None:
            continue   # internal :J5_*, departure -E_n, staging or outside the network
        counts[key]["ocupacion"] += 1
        if speed < halting_speed_mps:
            counts[key]["detenidos"] += 1
    return counts

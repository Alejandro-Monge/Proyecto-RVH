"""SUMO coupling layer of the digital shadow.

For now only the **static arm/lane ↔ edge/lane mapping** of the net (`lane_map`): the
prerequisite piece for instantiating vehicles (without knowing which edge each one
goes on, there is no SUMO). The causal use (when to instantiate) lives in the replay
layer that comes later; this module is a PURE, stateless lookup so as not to interfere
with that boundary.
"""

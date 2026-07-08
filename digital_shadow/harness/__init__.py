"""Causal harness: orchestrator of the live shadow loop.

Assembles source → causal perception (reconnection + streaming event commit) → position/route →
SUMO stage, frame by frame under a causal constraint, and SEALS the Section 7.2 delay per commit.
The measurement and visualization modes are mounted on top of this core.
"""

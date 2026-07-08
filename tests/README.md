# tests — project tests

Project tests. The Python modules (perception, tracking, position, route,
lane map, zones and occupancy, causality contract, emitter and WebSocket stream
contract) have `pytest` tests in the `test_*.py` files.

The 3D viewer front-end, which has no JS toolchain, is covered by Node checks
(`*.mjs`) over its core logic and by the stream import and integration tests
from the Python side. They run with `pytest` (and `node` for the `.mjs`) from
the repo root.

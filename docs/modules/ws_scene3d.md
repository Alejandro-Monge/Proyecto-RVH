# Module — Static scene of the 3D client (`scripts/derive_scene3d.py`)

Offline script that is run once per site or network, like the calibration: it derives from the
`.net.xml` (with `sumolib`, the same net-reading stack as production, without duplicating
parsing in JS) the scene JSON that the client serves statically and validates against the `hello`.

## Outputs

`scene3d/2` additively adds two presentation overlays derived from config (not from the
net): `stop_lines` and `lane_arrows`. The `net_md5` is still that of the net (the overlays do not alter
it), so the client keeps accepting the scene. `net_scene.js` requires `scene3d/2`.

```json
{
  "schema": "scene3d/2",
  "site": "rusia",
  "net_md5": "<md5 of the EXACT net FILE>",
  "net_path": "<registered relative path (provenance; the pin is the md5)>",
  "bounds": {"xmin", "ymin", "xmax", "ymax"},
  "lanes":          [{"id", "width", "shape": [[x, y], ...]}],
  "internal_lanes": [{"id", "width", "shape": [...]}],
  "junctions":      [{"id", "shape": [...]}],
  "stop_lines":     {"<brazo>": {"edge": [[x,y],[x,y]], "cross": [[x,y],[x,y]], "into": [dx,dy]}},
  "lane_arrows":    [{"lane", "pos": [x, y], "heading", "movement": "right|straight|left"}]
}
```

| Overlay (scene3d/2) | Source | Note |
|---|---|---|
| `stop_lines[arm].edge` | geometric road↔junction edge = end of `entry_edge` + width of the ENTRY | stop line (presentation, NOT measurement — the `cross_t` lives in the pipeline in px/homography). `into` = unit direction toward the junction |
| `stop_lines[arm].cross` | outer edge of the entry → outer edge of the exit (`exit_edge`) = FULL WIDTH of the approach | base of the crosswalk (the whole approach, not only the 3 entry lanes) |
| `lane_arrows` | `lane_map.entry_lanes` (lane→lane_idx) + net shape; lane→movement c1=right/c2=straight/c3=left | official signage; presentation. pos = 7 m before the end of the lane |

The stop-line↔crosswalk offsets and the cosmetics (arrow size, stripe width or gap) are not
in the scene: they are from config (`web3d.yaml → presentation_3d`, served in `/config.json`) and the
client applies them without regenerating the scene.

**scene3d/2 pins:** vision `b9082d37…`, messages `ad1726d2…` (md5 of the artifact). Tests:
`test_derive_scene3d` (schema/2, structure, overlays, rounding, determinism).

| Field | sumolib source | Note |
|---|---|---|
| `net_md5` | md5 of the bytes of the input net (the generator computes it) | what the client compares against `hello.net_md5`; mismatch → REJECTS the stream |
| `bounds` | `net.getBBoxXY()` | initial camera framing |
| `lanes` | normal edges: `lane.getShape()/getWidth()` | RAW SUMO plane — the mapping `(x,y)→(x,0,−y)` belongs to the client, fixed and declared |
| `internal_lanes` | `readNet(withInternal=True)`, `getFunction()=="internal"` | interior of the crossing; separated on purpose (different material; zone of the future vision handoff) |
| `junctions` | `node.getShape()` (nodes with shape) | includes the J5 polygon (the same as the player's crossing logger) |

## Determinism

Coordinates and widths rounded to **3 decimals** (mm, the same quantization as the canonical `pos_plane`);
lists sorted by `id`; `sort_keys` plus compact separators plus `\n`. The same net produces identical
bytes (verified by `cmp`). That the mm rounding does not introduce visible artifacts on the roadway is
confirmed visually in the client; the script guarantees the numerical sanity.

## Motivation for the pin: two nets in the repo

`config/sites/rusia/sumo/intersection_rusia.net.xml` (vision) = `90cc6b86…` and
`reconstruccion_causal/inputs/intersection_rusia.net.xml` (messages) = `c6c84b27…` are different files.
The `hello` of the messages profile pins the second: the scene must be derived from that
net. The rejection control generates a scene from the vision net and verifies
`net_md5 ≠ hello.net_md5`, so the client would reject it; the pin protects exactly the
case of real confusion.

## Verifications

1. **Real generation:** 22 normal lanes, 28 internal, 6 junctions;
   `bounds = (-72.24, -13.83, 167.22, 138.61)` == convBoundary of the net; artifact
   `ws_emitter/web/scene_rusia.json`.
2. **Scene↔hello chain:** `scene.net_md5 == hello.net_md5` of the real log of the message emitter
   (`c6c84b27…`).
3. **Determinism:** `cmp` byte-identical regeneration.
4. **Rejection control:** scene from the vision net (to a temporary directory, not the repo) →
   `90cc6b86… ≠ c6c84b27…` → guaranteed rejection.
5. **Tests 4/4** (`tests/test_derive_scene3d.py`): structure and order of the contract, rounding to
   3 decimals (regex over the bytes), `net_md5` and bounds, determinism.

## Validation regime of the client front

This front is a presentation layer and is validated differently from the server-side pipeline layers:
there is no automated JS suite (the repo has no JS toolchain, and none is faked). The evidence
is visual validation plus the JSON export of WS→render metrics plus the rejection tests
(mutated net_md5 → banner; stream without `end` → "no verdict"; observable coalescing). It is
acceptable here because the client touches neither causality nor paper metrics: the fidelity of the
stream is already proven server-side (36/36). The Python modules of the front (this script, the
webserver) do carry pytest like the rest.

## References

- `docs/modules/ws_net_scene.md`: the client that consumes this scene.
- `docs/modules/ws_payload_contract.md`: the `hello` contract against which `net_md5` is
  validated.
- `docs/modules/sumo_lane_map.md`: the `lane_map` (entry_lanes) from which the arrows derive.

# Module — approach/lane ↔ edge/lane mapping of the SUMO net

Establishes the static correspondence between the shadow's vocabulary (approach + lane of the
ground truth) and that of the SUMO net (edge + lane index). It answers a single question: given an
**approach** (north/south/east/west), a **frame** (`entry`/`exit`) and the **GT lane** (1 =
right of the vehicle in its direction of travel), in which **edge** and which **lane index** of SUMO
does it live? It is the prerequisite for instantiating a replicated vehicle: without knowing which
edge/lane each one goes in, there is no SUMO. It does not position longitudinally (that is done by
the homography→SUMO-coords piece), it does not choose route or turn, and it does not talk to TraCI;
its scope is deliberately minimal.

## Input and output

- **Input:** `(approach, frame, GT_lane)`. `frame ∈ {entry, exit}` (GT/event semantics:
  `entry` = vehicle approaching / origin; `exit` = vehicle departing / destination). `GT_lane`
  integer ≥ 1 with the site convention (1 = driver's right).
- **Output:** `(edge_id, lane_index)`. The SUMO lane id is `<edge>_<index>`
  (`lane_id(...)`).
- **Errors (no silent defaults):** invalid frame → `ValueError`; nonexistent approach or lane →
  `KeyError` with message (for example, requesting egress lane 2 in north/south, which
  only have 1 departing lane, is an invalid request).

## The mapping (verified against the net `intersection_rusia.net.xml`)

**Approach → edge.** Each approach has an **approach** edge (`E_n`, toward the central junction J5) and
a **departure** one (`-E_n`, from J5). The scope of this module is the edge **adjacent to J5**
(the upstream extension of the west —`E0`/`-E0` via J1— corresponds to the route piece).

| Approach | Junction of the approach (position) | Approach `E_n` (frame `entry`) | Departure `-E_n` (frame `exit`) |
|---|---|---|---|
| **south** | J8 (73.9, **−13.8** → min Y) | **E5** (3 lanes) | **−E5** (1 lane) |
| **north** | J6 (44.3, **138.6** → max Y) | **E6** (3 lanes) | **−E6** (1 lane) |
| **east** | J4 (**167.2**, 95.3 → max X) | **E3** (3 lanes) | **−E3** (2 lanes) |
| **west** | J1 (**−5.2**, 61.4 → min X) | **E1** (3 lanes) | **−E1** (2 lanes) |

**GT lane → lane index:** `index = GT_lane − 1`, with index 0 = the rightmost lane of the direction
of travel of that edge. The SUMO convention is not assumed: it is verified by
geometry (see the derivation section). Result:

- **entry (approach):** south `{1:E5_0,2:E5_1,3:E5_2}` · north `{1:E6_0,2:E6_1,3:E6_2}` · east
  `{1:E3_0,2:E3_1,3:E3_2}` · west `{1:E1_0,2:E1_1,3:E1_2}`
- **exit (departure):** south `{1:-E5_0}` · north `{1:-E6_0}` · east `{1:-E3_0,2:-E3_1}` · west
  `{1:-E1_0,2:-E1_1}`

## The three input/output vocabularies

Three vocabularies use the words "entry/exit" with different meanings. Wrongly crossed, they
invert the mapping. The correct crossing:

| Side | GT / event | site.yaml (words inverted vs SUMO) | SUMO edge |
|---|---|---|---|
| origin (approaching J5) | `entry_arm` / `entry_lane` | "exit" (lanes toward J5, 3) | `E_n` (approach) |
| destination (departing) | `exit_arm` / `exit_lane` | "entry" (lanes departing, 1 or 2) | `-E_n` (departure) |

Crossing to keep in mind: **the GT `entry_lane` falls in the lanes that site.yaml calls "exit" and
in the edge `E_n`** (not in "entry", not in `-E_n`). The module uses the GT semantics
(`entry`/`exit`); the site.yaml words describe *sets of lanes off the road*, the GT words describe
*the vehicle's action*.

**Why the `lane−1` rule holds in both frames** even though entry lane 1 and exit lane 1 are on
physically opposite sides: SUMO index 0 is always the right *of the edge itself*; entry uses `E_n`
and exit uses `-E_n`, which travel in opposite directions, so their index 0 falls on physically
opposite sides —exactly the GT asymmetry—. The numbering becomes uniform because each frame uses its
own edge, not because entry and exit are comparable 1=1 (they are not).

## Derivation by geometry

`scripts/derive_sumo_lane_map.py` reads the net and derives **by geometry**, never by the edge
name (the names `E1..E6` **are not cardinal**; deriving by name would give a rotated homography):

- **Approach → edge by coordinates:** the central junction (the `traffic_light`, J5) is located and,
  for each approach (`to=J5`), its approach is classified by the dominant component of
  `(approach_junction − J5)`. In the net (north-up) the approaches align with the cardinals
  (south=−Y, north=+Y, east=+X, west=−X). The derived result is **checked** against the fixed
  physical mapping (control 2). *(Assumption: approaches aligned to cardinals; a site with the X
  rotated 45° would require angular binning; it is a limit of the method, it does not affect Rusia.)*
- **Lane → index by the right-normal of the direction of travel:** for each edge the direction
  of circulation is taken (the SUMO shapes go in that direction), its right-normal `(dy,−dx)` (facing
  north, the right is the east), and the lanes are ordered from right to left. It is verified that
  SUMO index 0 lands on the right → `GT_lane = index + 1`.

## Verifications

The script aborts without writing the yaml if any control does not check out.

1. **Structural:** each `E_n` goes from J_approach to J5 and each `-E_n` from J5 to J_approach; lane
   count = 3 approach; 1 in N/S, 2 in E/W departure (matches `site.yaml: geometry`).
2. **Coordinate anchoring:** the approach→edge derived by position = the fixed physical mapping
   `{south:E5, north:E6, east:E3, west:E1}` (protects against the inherited name).
3. **Lane order by geometry:** in the 8 edges, SUMO index 0 = rightmost lane
   (sequence `[0,1,2]`/`[0,1]`/`[0]`).
4. **`turns.csv` coverage (strong control, one pass):** for the 12 allowed movements,
   the `dir` of the net connection `E_{entry}→-E_{exit}` **reproduces** the `turn` column of
   `turns.csv` (left→l, right→r, through→s). **12/12.** If two approaches were swapped,
   some `dir` would break.
5. **Coherence with GT v2:** `exit_lane` of all north/south egresses = 1 (0/26 violations);
   all lanes of GT v2 fit in the lanes of the net.
6. **Anchor `south→west` / counter-anchor `south→east` (the turn predicts the lane):** `south→west` (left)
   enters via `E5` lane 3 (`E5_2`, the `l` connection leaves from `fromLane=2`) and exits via `-E1` lane 2
   (`-E1_1`, `toLane=1` exists); `south→east` (right) enters via `E5` lane 1 (`E5_0`, `r` connection
   from `fromLane=0`). It ties the GT numbers to the lane-ids through a path independent of the mapping.

## Stateless lookup

The mapping is static, but it is consumed in a causal flow. Five properties make it fit for that
use:

- **Pure and stateless lookup:** `target(...)` is a dictionary, idempotent and order-independent;
  it does not depend on the event set or on any instant. It gives the same result whether called by
  the validation script or by the causal bridge.
- **Entry and exit separately:** the origin is queried when committing the entry (at `t`); the
  destination, later, when committing the exit (possibly after the buffer). It does not require the
  complete event, which enables the causal pattern of SUMO (insert with the origin edge and complete
  the route with `setRoute`/`changeTarget` when the destination is inferred).
- **No temporal logic inside:** the "when" (and the buffer wait = actual delay) lives in the
  causal layer, not here. The batch↔causal boundary passes outside this module.
- **One contract, two consumers:** standalone module; the first consumer is the validation, the
  second the causal bridge, which imports the *same* function. It is not rewritten.
- **Anti-pattern avoided:** a lookup that required entry+exit (or all events) together would fix the
  batch assumption (in causal the exit is not yet known at entry). It is avoided by resolving per
  frame and per vehicle.

## Files

- `scripts/derive_sumo_lane_map.py` — derivation (reads the net) + the controls; writes the yaml with
  `--write` only if everything passes. Records `net_md5` to detect staleness.
- `config/sites/rusia/sumo/lane_map.yaml` — derived config (do not edit by hand; re-derive). The
  runtime reads only this. It **additively** includes `approach_axis` (longitudinal axis per approach:
  origin/dir/length of the net) for the position piece; the approach/lane↔edge/lane mapping is
  independent of that extension (idempotent re-derivation verified by diff).
- `digital_shadow/sumo/lane_map.py` — pure lookup `load_lane_map` + `sumo_target`/`target`/`lane_id`.
- `tests/test_sumo_lane_map.py` — pure contract (synthetic mapping) + check against the real config.

## Limits

- **Longitudinal position** within the lane (where the vehicle appears along the edge):
  corresponds to the homography→SUMO-coords piece (`docs/modules/position.md`).
- **Route construction** (which connection/turn, destination after buffer) and the **upstream
  extension of the west** (`E0`/J1): correspond to the route piece (`docs/modules/route.md`).
- **Causal use** (when to instantiate, move or remove, and the delay sealing): corresponds to the
  causal replay layer.

## References

- `docs/modules/position.md`: the position piece that uses `approach_axis`.
- `docs/modules/route.md`: the route construction and the upstream extension of the west.
- `docs/modules/sumo_stage.md`: the stage that consumes this mapping (`junction_internal`).
- Site config: `config/sites/rusia/sumo/lane_map.yaml`, net
  `config/sites/rusia/sumo/intersection_rusia.net.xml`.
- `scripts/derive_sumo_lane_map.py`, `tests/test_sumo_lane_map.py`.

# Module — Static roadway of the 3D client (`ws_emitter/web/js/net_scene.js`)

Renderer of the static roadway geometry in the 3D client, shadow-agnostic.
It consumes `scene_<site>.json` (contract `scene3d/2`, see `ws_scene3d.md`) and builds the
`THREE.Group` of the roadway. The base is extended with two aesthetic presentation overlays, described
below.

## Scope

The whole module is a presentation demo (PEWASUN 2026), not a result: it feeds no
figure or metric, and the rendered roadway is not a measurement. The canonical declaration is
in `ws_payload_contract.md`.

## Vendoring of three r180

`three@0.180.0` (npm) is pinned for API stability, not by date: the official migration
guide records no changes in WebGLRenderer or OrbitControls between r178 and r185 (the only
deprecation in the range is `PCFSoftShadowMap` in r182, and this client does not use shadow maps);
WebGLRenderer has no announced retirement toward WebGPURenderer; r180 accumulates five releases
of later run-in with the same surface. The paths were verified against the real tarball
of npm, not from memory: `build/three.module.js`, `examples/jsm/controls/OrbitControls.js`
(imports `'three'` bare, which the import map resolves), `LICENSE` MIT.

```
ws_emitter/web/vendor/three/
  three.module.js                   e3a10386281e96895ecedc84238261eb
  three.core.js                     22184b6508211a30dd3ec752297fa999
  addons/controls/OrbitControls.js  c9adb791476ceba9aa02f33205031df7
  LICENSE                           d0c607a5dd111060bd8feb787f5cde4d
  VERSION.md                        (this record + update rule)
```

The r180 build is split into two files: `three.module.js` imports and re-exports from
`./three.core.js` (lines 6-7 of the build, verified in the tarball), so both must be
vendored; if `three.core.js` is missing, `dev_scene.html` goes blank with
`GET /vendor/three/three.core.js → 404`. A smoke test that only checks "N known resources return
200" plus `node --check` validates the list by hand and the syntax, not the resolution of imports at
runtime; that is why `tests/test_web_imports.py` follows the graph (import map plus relative ones,
recursive, plus .json literals) and requires each file to exist. As a negative control, removing
three.core.js makes the test fail naming it. Declared limit: it resolves imports statically
(it does not follow dynamic `import()`; neither three r180 nor these modules use them). The
browser's `favicon.ico → 404` is harmless: it is not a module resource (browsers request it on their
own), and it is declared so it is not confused with a real failure.

Resources the browser actually requests (computed graph plus HTML; all 200):
`dev_scene.html`, `js/net_scene.js`, `scene_rusia.json`, `vendor/three/three.module.js`,
`vendor/three/three.core.js`, `vendor/three/addons/controls/OrbitControls.js` (plus
`favicon.ico`, unrelated).

Import map (in the HTML): `"three" → ./vendor/three/three.module.js`, `"three/addons/" →
./vendor/three/addons/"`. Single copy: the official warning against mixing versions or
origins is satisfied trivially.

## Input and output

```js
loadScene(url, pres) -> Promise<{ netMd5, site, bounds, group }>
```
- Input: `scene_<site>.json` by fetch; validates `schema === "scene3d/2"` (throws otherwise).
  `pres` = `cfg.presentation_3d` from `/config.json` (cosmetics; defaults in `PRES_DEFAULT` if
  absent).
- Output: `group` (static THREE.Group) plus `netMd5` exposed; the comparison against
  `hello.net_md5` is done by `index.html`, not this module.
- Does not do: stream, entities, HUD, read `.net.xml`, compare hello.

## Client geometric mapping

Exported; entities.js reuses it.

```js
sumoToThree(x, y, h=0) -> Vector3(x, h, -y)      // Y-up
headingToRotationY(rad) -> rad                    // contract heading as is
```
Reason for the `−y` (chirality): SUMO is 2D right-handed (+x east, +y north, heading CCW from +x);
Three.js is 3D right-handed Y-up. With `y → −z` the ground plane keeps its orientation and the
contract heading is applied as is as a rotation over +Y. With `(x, 0, y)` the plane
would end up mirrored (every turn would need the inverted sign, the "turns the wrong way" error).
Technical note: for polygons, `THREE.Shape(x, y)` plus `rotateX(−π/2)` produces exactly
`(x, 0, −y)`, the same transformation by another route, verified.

## Construction and presentation constants

- Lanes: flat ribbon centered on the polyline (perpendicular offset ±width/2, direction
  averaged per vertex); `DoubleSide` (winding does not matter, a declared simplification).
- Junctions: `ShapeGeometry` of the polygon.
- `MeshBasicMaterial` materials (no lights or shadows, which is why the r182 deprecation does not
  apply). Uniform junction: lanes, internals and junction take the same asphalt color
  `0x3c3c42`, so the intersection is a continuous surface. The structure is given by the
  markings, not by a color change.
- Anti z-fighting offsets (m): junction 0.0, internals +0.01, lanes +0.02, lane lines
  +0.025, crosswalk +0.026, stop line +0.027, arrows +0.028.

## Aesthetic presentation

The aesthetic presentation extends `net_scene.js` without touching the geometric mapping
(`sumoToThree`/`headingToRotationY` intact, scene not mirrored). Everything is derived from real data
(scene or config), nothing painted by eye:

- **Lane lines:** white marking (`0xc8c8d0`, width 0.2 m) on each lane edge
  (±width/2), derived from `lane.shape` plus `width` of the scene.
- **Stop line, crosswalk and arrows (scene3d/2):**
  - Stop line: white ribbon over `stop_lines[arm].edge` (road↔junction edge). It is presentation,
    not the `cross_t` boundary (that lives in the pipeline, in px and homography).
  - Crosswalk: perpendicular stripes over `stop_lines[arm].cross` (full width of the approach:
    entry plus exit), shifted `into` toward the interior of the junction; stop line and crosswalk do not
    overlap.
  - Direction arrows: local outline (c1=right/c2=straight/c3=left) rotated by heading to
    the lane position (`lane_arrows` of the scene).
- **Cosmetics by config** (`web3d.yaml → presentation_3d`, served in `/config.json`,
  applied by `loadScene(url, pres)` without regenerating the scene): `stop_offset_m`,
  `crosswalk_offset_m`, `crosswalk_stripe_w`, `crosswalk_gap`, `arrow_scale`.

Auto-validation: headless captures (chromium plus swiftshader) plus `test_web_imports` plus
`test_derive_scene3d` (scene3d/2). The 3D vehicle models and their settling are in
`ws_entities.md`.

## Validation scaffolding: dev_scene.html

Labeled as scaffolding in its header (it is not the client; only roadway plus OrbitControls). It is
served with the stdlib one-liner (it is not the final webserver of `--3d`):

```
python3 -m http.server 8300 --directory ws_emitter/web
→ http://localhost:8300/dev_scene.html
```

Smoke: the 5 resources serve HTTP 200 (html, scene json, net_scene.js, three.module.js,
OrbitControls.js); `node --check` of net_scene.js correct.

## Visual validation criteria

1. The roadway corresponds to the Rusia intersection (correct geometry, not mirrored:
   approach orientation as in the SUMO plane); the chirality of the `(x,0,−y)` mapping is validated
   against the real geometry, and the four-approach crossing must be identifiable.
2. Interior of the crossing (internals plus junction) distinguishable by tint.
3. No quantization artifacts on lane edges (mm rounding of the scene), at
   working distance.
4. Smooth orbit and zoom.

## Known limitations

- **Lighting:** not implemented. The reference video is flat and overcast, and `MeshBasic`
  already approximates the intended look.

## References

- `docs/modules/ws_scene3d.md`: the scene generator and the `scene3d/2` contract that this
  module consumes.
- `docs/modules/ws_entities.md`: the dynamic entities and the 3D vehicle models, which
  reuse the geometric mapping.
- `docs/modules/ws_payload_contract.md`: the scope declaration of the presentation demo.

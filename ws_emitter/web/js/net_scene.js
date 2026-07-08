// net_scene — STATIC roadway of the 3D client (contract in docs/modules/ws_net_scene.md).
// Pure geometry renderer: consumes scene_<site>.json (scene3d/1 contract, raw SUMO
// plane) and builds a THREE.Group. It does NOT touch stream/entities/HUD, it does NOT
// read the .net.xml (scripts/derive_scene3d.py already did that), it does NOT compare
// the hello (it exposes netMd5 so main.js does it).

import * as THREE from 'three';

// ---------------------------------------------------------------------------
// SINGLE GEOMETRIC MAPPING of the client: SUMO plane (x, y) -> Three.js (x, h, -y),
// Y-up. The -y is CHIRALITY: SUMO is 2D right-handed (+x east, +y north, contract
// heading CCW from +x) and Three.js is 3D right-handed Y-up; with y -> -z the ground
// plane keeps its orientation and the heading is applied AS-IS as a rotation about +Y.
// With (x, 0, y) the plane would be mirrored and every turn would need an inverted sign
// (silent "turns the wrong way" bug). entities.js MUST reuse these functions — do not
// duplicate the mapping.
export function sumoToThree(x, y, h = 0) {
  return new THREE.Vector3(x, h, -y);
}
export function headingToRotationY(headingRad) {
  // contract heading (rad, 0 = +x, CCW) == rotation about +Y with y -> -z.
  return headingRad;
}

// Declared PRESENTATION constants (they do not come from the contract):
// colors per surface class and minimal anti z-fighting offsets.
// UNIFORM JUNCTION (piece 1, revised): junction AND turn connectors take THE SAME color
// as the roadway → the whole intersection is ONE continuous asphalt surface (before,
// the junction was a darker patch and the connectors were loose strips). The structure
// is given by the lane LINES, white and visible (road markings).
const COLOR = { lane: 0x3c3c42, internal: 0x3c3c42, junction: 0x3c3c42,
                lane_line: 0xc8c8d0, stop: 0xf0f0f4, crosswalk: 0xdedee4,
                arrow: 0xf0f0f4 };
const Y_OFF = { junction: 0.0, internal: 0.01, lane: 0.02, lane_line: 0.025,
                crosswalk: 0.026, stop: 0.027, arrow: 0.028 };
const LANE_LINE_W = 0.2;   // lane marking width (presentation)
const STOP_W = 0.5;        // stop line thickness (presentation)
const CW_DEPTH = 2.6;      // crosswalk depth (along `into`)

// direction arrows (local outline, +x = forward; rotated by heading and translated to
// pos). c1=right, c2=straight, c3=left (official signage, Israel 2026-07-05).
const ARROW = {
  straight: [[-2.4, -0.3], [0.8, -0.3], [0.8, -0.85], [2.4, 0], [0.8, 0.85],
             [0.8, 0.3], [-2.4, 0.3]],
  right: [[-2.4, 0.3], [0.3, 0.3], [0.3, -1.0], [0.9, -1.0], [0.0, -2.2],
          [-0.9, -1.0], [-0.3, -1.0], [-0.3, -0.3], [-2.4, -0.3]],
  left: [[-2.4, -0.3], [0.3, -0.3], [0.3, 1.0], [0.9, 1.0], [0.0, 2.2],
         [-0.9, 1.0], [-0.3, 1.0], [-0.3, 0.3], [-2.4, 0.3]],
};

function ribbonGeometry(shape, width, yOff) {
  // Flat ribbon centered on the polyline (perpendicular offset +-width/2 per vertex,
  // direction averaged between segments). DoubleSide on the material: the winding does
  // not matter (declared presentation simplification).
  const n = shape.length;
  const half = width / 2;
  const pos = [];
  const idx = [];
  for (let i = 0; i < n; i++) {
    const prev = shape[Math.max(0, i - 1)];
    const next = shape[Math.min(n - 1, i + 1)];
    let dx = next[0] - prev[0];
    let dy = next[1] - prev[1];
    const len = Math.hypot(dx, dy) || 1;
    dx /= len; dy /= len;
    const ux = -dy, uy = dx;                       // perpendicular in the SUMO plane
    const [px, py] = shape[i];
    const l = sumoToThree(px + ux * half, py + uy * half, yOff);
    const r = sumoToThree(px - ux * half, py - uy * half, yOff);
    pos.push(l.x, l.y, l.z, r.x, r.y, r.z);
  }
  for (let i = 0; i < n - 1; i++) {
    const a = 2 * i;
    idx.push(a, a + 1, a + 2, a + 1, a + 3, a + 2);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setIndex(idx);
  return g;
}

function offsetShape(shape, offset) {
  // Polyline shifted perpendicularly by `offset` (lane edge). Same averaged
  // perpendicular as ribbonGeometry (in the SUMO plane).
  const n = shape.length;
  const out = [];
  for (let i = 0; i < n; i++) {
    const prev = shape[Math.max(0, i - 1)];
    const next = shape[Math.min(n - 1, i + 1)];
    let dx = next[0] - prev[0];
    let dy = next[1] - prev[1];
    const len = Math.hypot(dx, dy) || 1;
    dx /= len; dy /= len;
    out.push([shape[i][0] - dy * offset, shape[i][1] + dx * offset]);
  }
  return out;
}

function polygonGeometry(shape, yOff) {
  // Junction polygon: THREE.Shape in (x, y) + rotateX(-PI/2) == (x, 0, -y) — exactly
  // the sumoToThree mapping, verified in the module doc.
  const s = new THREE.Shape(shape.map(([x, y]) => new THREE.Vector2(x, y)));
  const g = new THREE.ShapeGeometry(s);
  g.rotateX(-Math.PI / 2);
  g.translate(0, yOff, 0);
  return g;
}

function arrowGeometry(a, yOff, scale) {
  // Direction arrow: LOCAL outline (+x forward) scaled, rotated by heading and
  // translated to pos → WORLD polygon → Shape (same mapping as the junction).
  const local = ARROW[a.movement] || ARROW.straight;
  const c = Math.cos(a.heading), s = Math.sin(a.heading);
  const world = local.map(([x0, y0]) => {
    const x = x0 * scale, y = y0 * scale;
    return new THREE.Vector2(a.pos[0] + x * c - y * s, a.pos[1] + x * s + y * c);
  });
  const g = new THREE.ShapeGeometry(new THREE.Shape(world));
  g.rotateX(-Math.PI / 2);
  g.translate(0, yOff, 0);
  return g;
}

// PRESENTATION parameters (from /config.json = web3d.yaml; reasonable defaults):
// stop↔crosswalk offsets and crosswalk/arrow cosmetics. They can be adjusted without
// regenerating the scene (they live in config, not in the artifact).
const PRES_DEFAULT = {
  stop_offset_m: 0.0,        // stop at the street↔junction edge
  crosswalk_offset_m: 3.0,   // crosswalk shifted toward the INTERIOR of the junction
  crosswalk_stripe_w: 0.7,   // wider stripes
  crosswalk_gap: 1.3,        // more spacing → fewer stripes
  arrow_scale: 0.55,         // smaller arrows
};

export async function loadScene(url, pres) {
  const resp = await fetch(url);
  if (!resp.ok) throw new Error(`scene3d: fetch ${resp.status} en ${url}`);
  const scene = await resp.json();
  if (scene.schema !== 'scene3d/2') {
    throw new Error(`scene3d: schema inesperado ${scene.schema} (se espera scene3d/2)`);
  }
  const P = Object.assign({}, PRES_DEFAULT, pres || {});
  const mk = (color) => new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide });
  const mat = {
    lane: mk(COLOR.lane), internal: mk(COLOR.internal), junction: mk(COLOR.junction),
    lane_line: mk(COLOR.lane_line), stop: mk(COLOR.stop),
    crosswalk: mk(COLOR.crosswalk), arrow: mk(COLOR.arrow),
  };
  const group = new THREE.Group();
  group.name = `calzada_${scene.site}`;
  for (const lane of scene.lanes) {
    group.add(new THREE.Mesh(ribbonGeometry(lane.shape, lane.width, Y_OFF.lane), mat.lane));
    // LANE LINES (piece 1): thin marking on each lane edge (±width/2), derived from
    // lane.shape + width of the scene (not painted by eye).
    for (const s of [lane.width / 2, -lane.width / 2]) {
      group.add(new THREE.Mesh(
        ribbonGeometry(offsetShape(lane.shape, s), LANE_LINE_W, Y_OFF.lane_line),
        mat.lane_line));
    }
  }
  for (const lane of scene.internal_lanes) {
    group.add(new THREE.Mesh(ribbonGeometry(lane.shape, lane.width, Y_OFF.internal), mat.internal));
  }
  for (const j of scene.junctions) {
    if (j.shape.length >= 3) {
      group.add(new THREE.Mesh(polygonGeometry(j.shape, Y_OFF.junction), mat.junction));
    }
  }
  // STOP LINE (street↔junction edge) + CROSSWALK (further in, shifted by `into`).
  // Pure presentation: the stop goes on the entering approach's edge; the crosswalk is
  // offset toward the interior — they do not overlap. The cross_t measurement lives in
  // the pipeline.
  for (const sl of Object.values(scene.stop_lines || {})) {
    if (!sl || !sl.edge) continue;
    const [a, b] = sl.edge, [ix, iy] = sl.into;
    // stop: edge of the ENTRY (+ stop_offset toward the junction)
    const so = P.stop_offset_m;
    group.add(new THREE.Mesh(ribbonGeometry(
      [[a[0] + ix * so, a[1] + iy * so], [b[0] + ix * so, b[1] + iy * so]],
      STOP_W, Y_OFF.stop), mat.stop));
    // crosswalk: over the FULL WIDTH of the approach (sl.cross = entry+exit), shifted
    // crosswalk_offset toward the interior; stripes in the `into` direction
    const [ca0, cb0] = sl.cross || sl.edge;
    const co = P.crosswalk_offset_m;
    const ca = [ca0[0] + ix * co, ca0[1] + iy * co], cb = [cb0[0] + ix * co, cb0[1] + iy * co];
    let dx = cb[0] - ca[0], dy = cb[1] - ca[1];
    const len = Math.hypot(dx, dy) || 1;
    dx /= len; dy /= len;                        // along the edge (roadway width)
    const step = P.crosswalk_stripe_w + P.crosswalk_gap;
    for (let d = 0.4; d < len - 0.4; d += step) {
      const c0 = [ca[0] + dx * d, ca[1] + dy * d];
      group.add(new THREE.Mesh(ribbonGeometry(
        [[c0[0] - ix * CW_DEPTH / 2, c0[1] - iy * CW_DEPTH / 2],
         [c0[0] + ix * CW_DEPTH / 2, c0[1] + iy * CW_DEPTH / 2]],
        P.crosswalk_stripe_w, Y_OFF.crosswalk), mat.crosswalk));
    }
  }
  // DIRECTION ARROWS: c1=right, c2=straight, c3=left (lane_map). Scale from config.
  for (const a of scene.lane_arrows || []) {
    group.add(new THREE.Mesh(arrowGeometry(a, Y_OFF.arrow, P.arrow_scale), mat.arrow));
  }
  return { netMd5: scene.net_md5, site: scene.site, bounds: scene.bounds, group };
}

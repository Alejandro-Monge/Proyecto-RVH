// Verification of the SETTLING of the vehicle 3D model (settling requirement):
// the BASE of the placed model lands on the VISIBLE road surface (Y_OFF.lane of
// net_scene = 0.02), not at Y=0 (under the junction) nor floating. Real measurement: builds
// the model with three, places it as _pose (sumoToThree with h=ROAD_SURFACE_Y), measures the
// bounding box and compares min.y with the surface. Also: nose (max.x) at the reference
// point (pos_plane = bumper).
//   node tests/vehicle_settle_check.mjs
import * as THREE from '../ws_emitter/web/vendor/three/three.module.js';
import { vehicleParts, ROAD_SURFACE_Y } from '../ws_emitter/web/js/entities_geometry.js';

const LANE_Y = 0.02;   // Y_OFF.lane of net_scene: the VISIBLE surface under the vehicle
// tolerance 1e-6 m (0.001 mm): much finer than the mm quantization of pos_plane and
// than any visible artifact, but tolerant of the float noise of BoxGeometry with
// non-exact dims (3.2, 2.6). A real settling mismatch would be cm, not nm.
const EPS = 1e-6;
const DIMS = {
  car: { length_m: 4.5, width_m: 1.8 },
  bus: { length_m: 12.0, width_m: 2.55 },
  truck: { length_m: 7.5, width_m: 2.5 },
};
const REFX = 10, REFY = -20;   // arbitrary reference position (pos_plane in world)

let ok = true;
for (const vclass of ['car', 'bus', 'truck']) {
  const g = new THREE.Group();
  for (const p of vehicleParts(vclass, DIMS[vclass])) {
    const geo = new THREE.BoxGeometry(p.w, p.h, p.dep);
    geo.translate(p.x, p.y, p.z);
    g.add(new THREE.Mesh(geo));
  }
  // _pose: sumoToThree(x, y, ROAD_SURFACE_Y) = (x, ROAD_SURFACE_Y, −y); heading 0
  g.position.set(REFX, ROAD_SURFACE_Y, -REFY);
  g.updateMatrixWorld(true);
  const box = new THREE.Box3().setFromObject(g);
  const baseOk = Math.abs(box.min.y - LANE_Y) < EPS;         // base on the surface
  const noseOk = Math.abs(box.max.x - REFX) < 1e-6;          // nose at the reference point
  const floats = box.min.y > LANE_Y + EPS;
  const sinks = box.min.y < LANE_Y - EPS;
  ok = ok && baseOk && noseOk;
  console.log(
    `${vclass}: base.y=${box.min.y.toFixed(4)} (superficie=${LANE_Y}) ` +
    `${baseOk ? 'ASENTADO' : (floats ? 'FLOTA' : (sinks ? 'HUNDIDO' : 'FALLO'))} | ` +
    `nariz.x=${box.max.x.toFixed(4)} (ref=${REFX}) ${noseOk ? 'OK' : 'FALLO'} | ` +
    `alto=${(box.max.y - box.min.y).toFixed(2)}m`);
}
if (!ok) { console.error('FALLO: el modelo no apoya en la superficie o la nariz no está en pos_plane'); process.exit(1); }
console.log('OK: los 3 modelos apoyan la base en la superficie visible de calzada; nariz en pos_plane');

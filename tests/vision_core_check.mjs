// HEADLESS verification of the reconcile with the THREE vision sources (node >=22,
// orchestrated by tests/test_vision_client.py). Covers: handoff WITHOUT pos_plane does
// not break those that have it, source change (box→handoff→vehicle, same id) => recreate,
// and the transport of anchor/lane from vision_box. The THREE glue is validated by visual inspection.
import assert from 'node:assert/strict';
import { reconcile } from '../ws_emitter/web/js/entities_core.js';

const DIMS = { car: { length_m: 5, width_m: 1.8 },
               bus: { length_m: 12, width_m: 2.5 },
               truck: { length_m: 8, width_m: 2.4 } };
const LAYERS = ['sumo', 'vision_box', 'handoff'];
const step = (...entities) => ({ entities });
let n = 0;
const ok = (cond, msg) => { assert.ok(cond, msg); n++; };

// 1) the three sources are drawn; handoff WITHOUT pos_plane does not break those that have it
let r = reconcile(new Map(), step(
  { id: 'v1', source: 'sumo', provenance: 'INTERPOLATED', pos_plane: [10, 20],
    heading: 0.5, vclass: 'car', generation: 0 },
  { id: 'v2', source: 'vision_box', provenance: 'OBSERVED', pos_plane: [5, 6],
    heading: 1.2, anchor: 'raw_homography', carril: null, vclass: 'bus' },
  { id: 'v3', source: 'handoff', provenance: 'HELD', last_pos: { x: 3, y: 4, t: 0.9 },
    vclass: 'truck' },
), LAYERS, DIMS);
ok(r.create.length === 3 && r.remove.length === 0, 'las tres fuentes crean');
const byId = Object.fromEntries(r.create.map((i) => [i.id, i]));
ok(byId.v1.x === 10 && byId.v1.heading === 0.5, 'sumo: pos_plane + heading');
ok(byId.v2.source === 'vision_box' && byId.v2.anchor === 'raw_homography'
   && byId.v2.carril === null, 'vision_box: anchor + carril');
ok(byId.v3.x === 3 && byId.v3.y === 4 && byId.v3.heading === null,
   'handoff: last_pos, sin heading (no rompió a las que sí traen pos_plane)');

// 2) source change (same canonical id) => recreate; sumo→sumo => move
let idx = r.nextIndex;
r = reconcile(idx, step(
  { id: 'v1', source: 'sumo', provenance: 'OBSERVED', pos_plane: [11, 21],
    heading: 0.6, vclass: 'car', generation: 0 },
  { id: 'v2', source: 'handoff', provenance: 'HELD', last_pos: { x: 5, y: 6, t: 1.0 },
    vclass: 'bus' },
), LAYERS, DIMS);
ok(r.move.map((i) => i.id).includes('v1'), 'sumo→sumo: move');
ok(r.recreate.map((i) => i.id).includes('v2'), 'vision_box→handoff: recreate (geom distinta)');
ok(r.remove.includes('v3'), 'v3 ausente: retira');

// 3) handoff→sumo => recreate (the vehicle takes over from the ghost)
idx = r.nextIndex;
r = reconcile(idx, step(
  { id: 'v2', source: 'sumo', provenance: 'OBSERVED', pos_plane: [7, 8],
    heading: 1.0, vclass: 'bus', generation: 0 },
), LAYERS, DIMS);
ok(r.recreate.map((i) => i.id).includes('v2'), 'handoff→sumo: recreate');

console.log(`OK: ${n} verificaciones del reconcile de visión (tres fuentes)`);

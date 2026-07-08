// HEADLESS verification of the reconciliation core (entities_core.js) — run by
// node (>=22) orchestrated by tests/test_entities_core.py. Hard asserts;
// exit 0 only if everything passes. The THREE glue is validated by visual inspection.
import assert from 'node:assert/strict';
import { reconcile } from '../ws_emitter/web/js/entities_core.js';

const DIMS = { car: { length_m: 5, width_m: 1.8 },
               bus: { length_m: 12, width_m: 2.5 } };
const LAYERS = ['sumo'];
const ent = (id, over = {}) => ({
  id, source: 'sumo', provenance: 'OBSERVED', pos_plane: [1.0, 2.0],
  heading: 0.5, vclass: 'car', generation: 0, ...over });
const step = (...entities) => ({ entities });

let n = 0;
const ok = (cond, msg) => { assert.ok(cond, msg); n++; };

// 1) basic create / move / remove
let r = reconcile(new Map(), step(ent('a'), ent('b', { vclass: 'bus' })), LAYERS, DIMS);
ok(r.create.length === 2 && r.move.length === 0 && r.remove.length === 0, 'crea 2');
let idx = r.nextIndex;
r = reconcile(idx, step(ent('a', { pos_plane: [9, 9] })), LAYERS, DIMS);
ok(r.move.length === 1 && r.remove.length === 1 && r.remove[0] === 'b',
   'mueve a, retira b');
ok(r.nextIndex.size === 1 && r.nextIndex.has('a'), 'indice == dibujables');

// 2) generation -> recreate (another incarnation, no tween)
idx = r.nextIndex;
r = reconcile(idx, step(ent('a', { generation: 1 })), LAYERS, DIMS);
ok(r.recreate.length === 1 && r.create.length === 0 && r.move.length === 0,
   'generation cambia => recreate');

// 3) layers: source outside => counted + listed, does NOT draw, does NOT enter the index
r = reconcile(new Map(), step(ent('a'), ent('x', { source: 'vision_box' })),
              LAYERS, DIMS);
ok(r.ignored.count === 1 && r.ignored.sources.includes('vision_box'),
   'source fuera de layers contado y listado');
ok(!r.nextIndex.has('x') && r.nextIndex.size === 1, 'ignorado no entra al indice');

// 4) unknown vclass: counted, not drawn
r = reconcile(new Map(), step(ent('a', { vclass: 'tractor' })), LAYERS, DIMS);
ok(r.unknownVclass === 1 && r.nextIndex.size === 0, 'vclass desconocido: fuera');

// 5) INVARIANT under long pseudo-random sequences: the simulated mesh
// "scene" follows nextIndex EXACTLY — no orphans, no leaks.
// deterministic PRNG (mulberry32): no Math.random for reproducibility.
function rng(seed) { let a = seed >>> 0; return () => {
  a |= 0; a = (a + 0x6D2B79F5) | 0;
  let t = Math.imul(a ^ (a >>> 15), 1 | a);
  t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
  return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; }
const rand = rng(20260704);
let index = new Map();
const meshIds = new Set();          // simulation of the glue bookkeeping
for (let k = 0; k < 500; k++) {
  const ents = [];
  const nEnts = Math.floor(rand() * 12);
  for (let j = 0; j < nEnts; j++) {
    const id = `veh_${Math.floor(rand() * 15)}`;
    if (ents.some((e) => e.id === id)) continue;
    ents.push(ent(id, {
      generation: rand() < 0.05 ? 1 : 0,
      source: rand() < 0.1 ? 'vision_box' : 'sumo',
      vclass: rand() < 0.1 ? 'tractor' : (rand() < 0.5 ? 'car' : 'bus'),
    }));
  }
  const rr = reconcile(index, step(...ents), LAYERS, DIMS);
  for (const id of rr.remove) { assert.ok(meshIds.has(id)); meshIds.delete(id); }
  for (const info of rr.recreate) assert.ok(meshIds.has(info.id));
  for (const info of rr.create) { assert.ok(!meshIds.has(info.id)); meshIds.add(info.id); }
  for (const info of rr.move) assert.ok(meshIds.has(info.id));
  index = rr.nextIndex;
  assert.deepEqual([...meshIds].sort(), [...index.keys()].sort(),
    `iter ${k}: meshes != indice (huerfano o fuga)`);
}
n++;
console.log(`OK: ${n} verificaciones del core (incl. invariante 500 iteraciones)`);

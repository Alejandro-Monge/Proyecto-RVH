// entities — THREE glue of the vehicle layer (doc: docs/modules/ws_entities.md).
// Consumes the step (takeLatest from stream.js) and applies the entities_core actions
// to meshes. REUSES sumoToThree / headingToRotationY from net_scene — vehicles and
// roadway SHARE the mapping or they diverge (Israel's rule).
//
// HEADING COHERENCE (dynamic counterpart of the chirality validated in static): with
// y -> -z, rotating the local +x by θ about +Y gives (cosθ, 0, −sinθ) =
// east·cosθ + north·sinθ — exactly the contract direction (rad, 0=+x east, CCW). The
// heading is applied AS-IS, with no ad hoc signs; the box's local +x axis is the front
// of the vehicle.
//
// GEOMETRY: pos_plane is the FRONT BUMPER (VAR_POSITION 0x42, verified in step 2.1;
// same convention as the waiting boxes) — the BoxGeometry is translated so the origin
// is the nose and the base rests on the ground. A centered box would be shifted half a
// length.
//
// Heights and colors = declared PRESENTATION CONSTANTS (not contract data). Geometries
// and materials SHARED by (vclass) and (vclass, provenance): removing a mesh is
// scene.remove, without disposing shared resources (≤ tens of vehicles: no pool).

import * as THREE from 'three';
import { sumoToThree, headingToRotationY } from './net_scene.js';
import { reconcile } from './entities_core.js';
import { vehicleParts, ROAD_SURFACE_Y } from './entities_geometry.js';

const BASE_COLOR = { car: 0xffd633, bus: 0x3396ff, truck: 0xff7033 }; // echo of SHADOW_DIMS
const FLAT_H = 0.3;   // height of the flat FOOTPRINT of tail/handoff (slab over the roadway)

// provenance -> SUMO vehicle style (table from the approved design; legend on the
// page). EXTRAPOLATED is a VISION provenance (causal dead-reckoning): it does not
// appear in messages; it is shown tinted/dimmed.
function styledColor(vclass, provenance) {
  const c = new THREE.Color(BASE_COLOR[vclass] ?? 0xffffff);
  let opacity = 1.0;
  switch (provenance) {
    case 'OBSERVED': break;                                   // saturated solid
    case 'INTERPOLATED': c.lerp(new THREE.Color(0x9a9a9a), 0.18); break;
    case 'EXTRAPOLATED': c.lerp(new THREE.Color(0xff8c00), 0.40); opacity = 0.8; break;  // vision
    case 'INFERRED': c.lerp(new THREE.Color(0x7f7fff), 0.35); break;
    case 'UNCERTAIN': opacity = 0.45; break;                  // translucent
    case 'HELD': c.lerp(new THREE.Color(0x555555), 0.55); opacity = 0.7; break;
    case 'REACQUIRED':
    case 'REACQUIRED_SNAP': c.set(0xff4bd8); break;           // accent mark
    default: c.set(0xffffff); break;                          // unknown: white
  }
  return { color: c, opacity };
}

// vision_box -> style BY ANCHOR (Section 3.7). raw_homography MUST be visually
// distinct from lane (validation mechanism for the south, NOT decoration): outline
// (wireframe) + warning tint "RAW anchoring, known lateral error". lane = full
// translucent footprint.
function boxStyle(vclass, anchor) {
  const c = new THREE.Color(BASE_COLOR[vclass] ?? 0xffffff);
  if (anchor === 'raw_homography') {
    return { color: c.lerp(new THREE.Color(0xff3311), 0.5), opacity: 0.95, wireframe: true };
  }
  return { color: c, opacity: 0.5, wireframe: false };
}

export class EntityLayer {
  constructor(scene, hello) {
    this.layers = hello.layers;
    this.dims = hello.vclass_dims;
    this.index = new Map();          // id -> {generation, provenance, vclass}
    this.meshes = new Map();         // id -> Mesh
    this._geo = new Map();           // vclass -> BoxGeometry (shared)
    this._mat = new Map();           // `${vclass}|${prov}` -> Material (shared)
    this.counters = { ignored: 0, unknownVclass: 0, ignoredSources: new Set() };
    this.group = new THREE.Group();
    this.group.name = 'vehiculos';
    scene.add(this.group);
  }

  // flat FOOTPRINT (vision_box/handoff): CENTERED on pos, base on the ground. Shared.
  _flatGeometry(vclass) {
    const key = `flat|${vclass}`;
    if (!this._geo.has(key)) {
      const d = this.dims[vclass];
      const g = new THREE.BoxGeometry(d.length_m, FLAT_H, d.width_m);
      g.translate(0, FLAT_H / 2, 0);
      this._geo.set(key, g);
    }
    return this._geo.get(key);
  }

  // i-th box of the vehicle's procedural 3D MODEL (sumo). Shared by (vclass, i).
  // vehicleParts defines nose at x=0, tail at −L, BASE at y=0.
  _vehicleGeometry(vclass, i) {
    const key = `veh|${vclass}|${i}`;
    if (!this._geo.has(key)) {
      const p = vehicleParts(vclass, this.dims[vclass])[i];
      const g = new THREE.BoxGeometry(p.w, p.h, p.dep);
      g.translate(p.x, p.y, p.z);
      this._geo.set(key, g);
    }
    return this._geo.get(key);
  }

  // material PER SOURCE: sumo = provenance; vision_box = anchor (Section 3.7); handoff =
  // HELD ghost (translucent gray, no heading).
  _material(info) {
    let key, mk;
    if (info.source === 'sumo') {
      key = `sumo|${info.vclass}|${info.provenance}`;
      mk = () => styledColor(info.vclass, info.provenance);
    } else if (info.source === 'vision_box') {
      key = `box|${info.vclass}|${info.anchor}`;
      mk = () => boxStyle(info.vclass, info.anchor);
    } else {                                       // handoff
      key = 'handoff';
      mk = () => ({ color: new THREE.Color(0x888888), opacity: 0.3, wireframe: false });
    }
    if (!this._mat.has(key)) {
      const { color, opacity, wireframe } = mk();
      this._mat.set(key, new THREE.MeshBasicMaterial(
        { color, opacity, transparent: opacity < 1, wireframe: !!wireframe }));
    }
    return this._mat.get(key);
  }

  _pose(obj, info) {
    // SETTLING: the sumo vehicle rests its BASE on the visible roadway surface
    // (ROAD_SURFACE_Y); the flat footprint (tail/handoff) goes to the ground (Y=0). The
    // model's base is at y=0 local → its world Y = the position's Y.
    const y = info.source === 'sumo' ? ROAD_SURFACE_Y : 0;
    obj.position.copy(sumoToThree(info.x, info.y, y));
    obj.rotation.y = info.source === 'handoff' ? 0 : headingToRotationY(info.heading);
    const mat = this._material(info);
    if (obj.isGroup) { for (const m of obj.children) m.material = mat; }  // multi-box model
    else { obj.material = mat; }
  }

  _spawn(info) {
    let obj;
    if (info.source === 'sumo') {
      obj = new THREE.Group();                          // procedural 3D model (multi-box)
      const mat = this._material(info);
      const n = vehicleParts(info.vclass, this.dims[info.vclass]).length;
      for (let i = 0; i < n; i++) {
        obj.add(new THREE.Mesh(this._vehicleGeometry(info.vclass, i), mat));
      }
    } else {
      obj = new THREE.Mesh(this._flatGeometry(info.vclass), this._material(info)); // flat footprint
    }
    this._pose(obj, info);
    this.group.add(obj);
    this.meshes.set(info.id, obj);
  }

  _drop(id) {
    const mesh = this.meshes.get(id);
    if (mesh) {
      this.group.remove(mesh);       // shared geometry/material: no dispose
      this.meshes.delete(id);
    }
  }

  apply(step) {
    const r = reconcile(this.index, step, this.layers, this.dims);
    for (const id of r.remove) this._drop(id);
    for (const info of r.recreate) { this._drop(info.id); this._spawn(info); }
    for (const info of r.create) this._spawn(info);
    for (const info of r.move) this._pose(this.meshes.get(info.id), info);
    this.index = r.nextIndex;
    this.counters.ignored += r.ignored.count;
    for (const s of r.ignored.sources) this.counters.ignoredSources.add(s);
    this.counters.unknownVclass += r.unknownVclass;
    return r;
  }

  size() { return this.meshes.size; }
}

// entities_core — id→entity reconciliation, PURE (no THREE, no DOM): the part of
// entities decidable by code and verified headless in node
// (tests/entities_core_check.mjs, orchestrated by pytest). The THREE glue
// (entities.js) applies these actions to meshes and Israel's eye validates it.
//
// Contract (ws_payload_contract v3.0.0 + approved design):
//  - layers: only sources declared in hello.layers are drawn; the rest are counted
//    and listed (notice), never drawn.
//  - vclass outside hello.vclass_dims: counted and NOT drawn (declared protocol
//    anomaly; no dimensions are invented).
//  - a different generation with the same id = ANOTHER INCARNATION -> recreate
//    (remove + create; no tween — we never interpolate).
//  - SOURCE CHANGE (box→handoff→vehicle, same canonical id Section 3.3) = different
//    geometry -> recreate. The id is stable across source; the drawable changes.
//  - handoff does NOT carry pos_plane or heading (closed keys): position from last_pos,
//    heading null. sumo/vision_box carry pos_plane. The position is resolved PER SOURCE
//    (an entity without pos_plane does not break the ones that have it).
//  - INVARIANT (test assert, not a promise): nextIndex == exactly the DRAWABLE ids of
//    the step — neither orphans nor leaks, in any sequence.

export function reconcile(prevIndex, step, layers, vclassDims) {
  const layerSet = new Set(layers || []);
  const dims = vclassDims || {};
  const create = [], move = [], recreate = [], remove = [];
  const ignoredSources = new Set();
  let ignored = 0, unknownVclass = 0;
  const nextIndex = new Map();

  for (const e of step.entities || []) {
    if (!layerSet.has(e.source)) {
      ignored++;
      ignoredSources.add(String(e.source));
      continue;
    }
    if (!Object.prototype.hasOwnProperty.call(dims, e.vclass)) {
      unknownVclass++;
      continue;
    }
    const generation = e.generation ?? 0;
    // position PER SOURCE: handoff -> last_pos (no pos_plane or heading);
    // sumo/vision_box -> pos_plane + heading.
    let x, y, heading;
    if (e.source === 'handoff') {
      x = e.last_pos.x; y = e.last_pos.y; heading = null;
    } else {
      x = e.pos_plane[0]; y = e.pos_plane[1]; heading = e.heading;
    }
    const info = {
      id: e.id, source: e.source, vclass: e.vclass, generation,
      provenance: e.provenance, x, y, heading,
      anchor: e.anchor ?? null, carril: e.carril ?? null,   // only vision_box
    };
    const prev = prevIndex.get(e.id);
    if (!prev) create.push(info);
    else if (prev.generation !== generation || prev.source !== e.source) recreate.push(info);
    else move.push(info);
    nextIndex.set(e.id, { generation, source: e.source,
                          provenance: e.provenance, vclass: e.vclass });
  }
  for (const id of prevIndex.keys()) {
    if (!nextIndex.has(id)) remove.push(id);
  }
  return { create, move, recreate, remove, nextIndex,
           ignored: { count: ignored, sources: [...ignoredSources].sort() },
           unknownVclass };
}

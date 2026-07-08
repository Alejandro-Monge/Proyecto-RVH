// entities_geometry — PURE geometry of the vehicle 3D model (no THREE, no DOM), so it
// is verifiable in node (settling measurement). entities.js consumes it and builds the
// BoxGeometry. Doc: docs/modules/ws_entities.md.

// Presentation heights of the SUMO vehicle (not contract data).
export const HEIGHT_M = { car: 1.5, bus: 3.2, truck: 2.6 };

// VISIBLE SURFACE of the roadway: the model's BASE (wheels) RESTS HERE, not at Y=0
// (which would be under the junction/lanes). It MUST match Y_OFF.lane of net_scene
// (0.02) — the settling is verified by measuring the placed model's box.
export const ROAD_SURFACE_Y = 0.02;

// low-poly MODEL: boxes with NOSE at x=0 (pos_plane is the bumper), tail at −L, BASE at
// y=0, centered in z. Scales to vclass_dims (length/width). Invariants that the settling
// requires: min(y − h/2) === 0 (base on the plane) and max(x + w/2) === 0 (nose at the
// origin). Fidelity is encoded in the dimensionality (RAISED model = precise, vs flat
// footprint = uncertain).
export function vehicleParts(vclass, d) {
  const L = d.length_m, W = d.width_m, H = HEIGHT_M[vclass] ?? 1.5;
  if (vclass === 'bus') {
    return [{ w: L, h: H, dep: W, x: -L / 2, y: H / 2, z: 0 }];           // long tall box
  }
  if (vclass === 'truck') {
    const cabL = L * 0.30, cargoL = L * 0.66, cabH = H * 0.82;
    return [
      { w: cabL, h: cabH, dep: W, x: -cabL / 2, y: cabH / 2, z: 0 },      // cab (nose)
      { w: cargoL, h: H, dep: W * 0.98,
        x: -(cabL + cargoL / 2 + 0.08), y: H / 2, z: 0 },                 // cargo (rear)
    ];
  }
  const bodyH = H * 0.5, cabH = H * 0.5;                                  // car (default)
  return [
    { w: L, h: bodyH, dep: W, x: -L / 2, y: bodyH / 2, z: 0 },            // body
    { w: L * 0.5, h: cabH, dep: W * 0.9, x: -L * 0.55, y: bodyH + cabH / 2, z: 0 }, // cab
  ];
}

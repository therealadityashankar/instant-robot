// Small rotation helpers, in the convention the rest of the app uses: row-major
// 3×3, columns are the frame's axes.
//
// These live here rather than inside the component because this is the code that
// keeps going wrong in ways that look fine on screen. Two failures so far: a
// frame built from (forward, right, up) that was a mirror rather than a rotation,
// and an aim that jumped 180° whenever noise pushed a folded angle across its
// boundary. Neither showed up as an error — both showed up as the arm behaving
// strangely, hours later. Pure functions with a test are the cheap fix.

/** Row-major 3×3 multiply. */
export function matMul3x3(a: number[] | Float64Array, b: number[] | Float64Array): number[] {
  const out = new Array(9).fill(0);
  for (let r = 0; r < 3; r++)
    for (let c = 0; c < 3; c++)
      out[r * 3 + c] = a[r * 3] * b[c] + a[r * 3 + 1] * b[3 + c] + a[r * 3 + 2] * b[6 + c];
  return out;
}

/** Angle between two rotations, in radians. */
export function rotAngleBetween(a: number[] | Float64Array, b: number[] | Float64Array): number {
  // trace(Aᵀ·B) is the elementwise sum for rotations, and the angle follows.
  let tr = 0;
  for (let i = 0; i < 9; i++) tr += a[i] * b[i];
  return Math.acos(Math.max(-1, Math.min(1, (tr - 1) / 2)));
}

/** Rotate a gripper orientation about its own approach axis (its x column). */
export function rotateAboutApproach(Rd: number[] | Float64Array, angle: number): number[] {
  const c = Math.cos(angle), s = Math.sin(angle);
  // Applied on the right, so it turns the wrist rather than the world.
  return matMul3x3(Rd, [1, 0, 0, 0, c, -s, 0, s, c]);
}

/**
 * Step `from` toward `to` by at most `maxRad`, along the shortest arc.
 *
 * A cap on the turn per pass is the orientation's equivalent of a step cap on
 * position, and exists for the same reason: one wild reading should cost one
 * small wrong move, not a wrist swing that reconfigures the whole arm.
 */
export function slewRotation(
  from: number[] | Float64Array,
  to: number[] | Float64Array,
  maxRad: number,
): number[] {
  const ang = rotAngleBetween(from, to);
  if (ang <= maxRad || ang < 1e-6) return [...to];
  // The relative rotation fromᵀ·to, as an axis and an angle.
  const rel: number[] = [];
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      let s = 0;
      for (let k = 0; k < 3; k++) s += from[k * 3 + i] * to[k * 3 + j];
      rel[i * 3 + j] = s;
    }
  }
  const axis = [rel[7] - rel[5], rel[2] - rel[6], rel[3] - rel[1]];
  let n = Math.hypot(axis[0], axis[1], axis[2]);
  if (n < 1e-9) {
    // A half-turn: the skew part vanishes, but the axis is still well defined —
    // rel + I is 2uuᵀ, so its largest column points along it. Returning `to` here
    // would snap through the full 180°, which is exactly what the cap is for.
    const d = [rel[0] + 1, rel[4] + 1, rel[8] + 1];
    const k = d[0] >= d[1] && d[0] >= d[2] ? 0 : d[1] >= d[2] ? 1 : 2;
    axis[0] = rel[k];
    axis[1] = rel[3 + k];
    axis[2] = rel[6 + k];
    axis[k] += 1;
    n = Math.hypot(axis[0], axis[1], axis[2]);
    if (n < 1e-9) return [...to]; // not a rotation at all; nothing sensible to do
  }
  const u = axis.map((v) => v / n);
  const c = Math.cos(maxRad), s = Math.sin(maxRad), t = 1 - c;
  // Rodrigues, rebuilt at the capped angle, applied in the gripper's own frame.
  const R = [
    t * u[0] * u[0] + c, t * u[0] * u[1] - s * u[2], t * u[0] * u[2] + s * u[1],
    t * u[0] * u[1] + s * u[2], t * u[1] * u[1] + c, t * u[1] * u[2] - s * u[0],
    t * u[0] * u[2] - s * u[1], t * u[1] * u[2] + s * u[0], t * u[2] * u[2] + c,
  ];
  return matMul3x3(from, R);
}

/**
 * Of a grasp aim and its half-turn twin, whichever is nearer `ref`.
 *
 * A grasp with the jaws swapped is the same grasp, so the aim is only defined up
 * to a half-turn about the approach axis. Picking by nearest-to-`ref` keeps the
 * choice continuous between sightings; picking the one nearest zero instead makes
 * the aim flip 180° whenever noise moves the folded angle across its boundary.
 */
export function nearestHalfTurn(aim: number[], ref: number[] | Float64Array): number[] {
  const flipped = rotateAboutApproach(aim, Math.PI);
  return rotAngleBetween(ref, flipped) < rotAngleBetween(ref, aim) ? flipped : aim;
}

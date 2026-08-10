// Geometry for "what would the robot's camera see?" in simulation.
//
// Pulled out of the Simulator so it can be tested directly: whether a tag is
// discoverable is the difference between the explore-and-navigate flow working
// and silently doing nothing, and that's far too easy to get subtly wrong by
// eye. Everything here is plain maths on plain numbers — no MuJoCo, no DOM.

export interface CamPose {
  /** Row-major 3×3 camera→world rotation, OpenCV axes (x right, y down, z fwd). */
  R: number[];
  t: [number, number, number];
}

/** Onboard-camera pose from the robot's world pose and its mount offsets. */
export function baseCameraPose(
  robotX: number,
  robotY: number,
  robotYawDeg: number,
  armLift: number,
  camFwd: number,
  camH: number,
): CamPose {
  const psi = (robotYawDeg * Math.PI) / 180;
  const c = Math.cos(psi), s = Math.sin(psi);
  return {
    R: [s, 0, c, -c, 0, s, 0, -1, 0],
    t: [robotX + camFwd * c, robotY + camFwd * s, armLift + camH],
  };
}

/** A world point expressed in the camera's frame. */
export function toCameraFrame(cam: CamPose, p: number[]): [number, number, number] {
  const d = [p[0] - cam.t[0], p[1] - cam.t[1], p[2] - cam.t[2]];
  const R = cam.R; // world→camera is its transpose, i.e. dot with each column
  return [
    R[0] * d[0] + R[3] * d[1] + R[6] * d[2],
    R[1] * d[0] + R[4] * d[1] + R[7] * d[2],
    R[2] * d[0] + R[5] * d[1] + R[8] * d[2],
  ];
}

export interface Sighting {
  /** Position in the camera frame (m). */
  cam: [number, number, number];
  /** How off-square the face is viewed, radians; 0 is head-on. */
  square: number;
}

/**
 * Is a flat, one-sided tag visible from this camera? Returns the sighting, or
 * null with the reason it failed — the reason is what makes this debuggable
 * rather than a mystery when nothing shows up.
 */
export function seeTag(
  cam: CamPose,
  tagPos: number[],
  tagNormal: number[],
  opts: { hfov: number; vfov: number; maxDist?: number; minDist?: number },
): { hit: Sighting | null; why: string } {
  const c = toCameraFrame(cam, tagPos);
  const [cx, cy, cz] = c;
  const minDist = opts.minDist ?? 0.1;
  const maxDist = opts.maxDist ?? 6;
  if (cz < minDist) return { hit: null, why: `behind or too close (z=${cz.toFixed(3)})` };
  if (cz > maxDist) return { hit: null, why: `too far (z=${cz.toFixed(3)})` };
  const hAng = Math.atan2(cx, cz);
  if (Math.abs(hAng) > opts.hfov) {
    return { hit: null, why: `outside horizontal fov (${((hAng * 180) / Math.PI).toFixed(1)}°)` };
  }
  const vAng = Math.atan2(cy, cz);
  if (Math.abs(vAng) > opts.vfov) {
    return { hit: null, why: `outside vertical fov (${((vAng * 180) / Math.PI).toFixed(1)}°)` };
  }
  // A printed tag can only be read from its front.
  const d = [tagPos[0] - cam.t[0], tagPos[1] - cam.t[1], tagPos[2] - cam.t[2]];
  const facing = tagNormal[0] * d[0] + tagNormal[1] * d[1] + tagNormal[2] * d[2];
  if (facing > 0) return { hit: null, why: `facing away (dot=${facing.toFixed(3)})` };

  const R = cam.R;
  const nz = R[2] * tagNormal[0] + R[5] * tagNormal[1] + R[8] * tagNormal[2];
  const nx = R[0] * tagNormal[0] + R[3] * tagNormal[1] + R[6] * tagNormal[2];
  return { hit: { cam: c, square: Math.atan2(nx, -nz) }, why: 'visible' };
}

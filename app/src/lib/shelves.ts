// A configurable shelf unit for the mobile-robot (LeKiwi) scene: a static frame
// with N stacked drawers, each on a slide joint (pulls open toward the robot to
// a limit) fronted by a grab rod. The whole unit is a mocap body so its pose
// (X/Y/yaw/elevation) can be tuned live; the drawers are jointed children.

export const SHELF = {
  width: 0.24, // X extent (local)
  depth: 0.22, // Y extent (local); opening faces local −Y
  height: 0.11, // per-compartment height (Z)
  openLimit: 0.14, // how far a drawer slides out (m)
  rodRadius: 0.008,
  rodFront: 0.03, // rod distance ahead of the drawer front face (local −Y)
};

const f = (n: number) => n.toFixed(5);

export function shelfJointName(i: number): string {
  return `shelf_slide_${i}`;
}

/** Rod position in the shelf unit's LOCAL frame for drawer `i` at `open` m. */
export function rodLocal(i: number, open: number): [number, number, number] {
  const zc = i * SHELF.height + SHELF.height / 2;
  return [0, -(SHELF.depth / 2) - SHELF.rodFront - open, zc];
}

/**
 * Worldbody XML for a shelf unit with `count` drawers, as a mocap body at the
 * origin (pose set at runtime). Frame geoms + drawer bodies are all in the
 * unit's local frame; the unit origin is the bottom centre.
 */
export function buildShelvesXml(count: number): string {
  const s = SHELF;
  const hw = s.width / 2;
  const hd = s.depth / 2;
  const total = count * s.height;
  const parts: string[] = [];

  const wall = (name: string, x: number, y: number, z: number, sx: number, sy: number, sz: number) =>
    `<geom name="${name}" type="box" pos="${f(x)} ${f(y)} ${f(z)}" size="${f(sx)} ${f(sy)} ${f(sz)}" ` +
    `rgba="0.45 0.38 0.30 1" contype="1" conaffinity="1"/>`;

  parts.push(wall('shelf_back', 0, hd, total / 2, hw, 0.006, total / 2));
  parts.push(wall('shelf_left', -hw, 0, total / 2, 0.006, hd, total / 2));
  parts.push(wall('shelf_right', hw, 0, total / 2, 0.006, hd, total / 2));
  for (let k = 0; k <= count; k++) parts.push(wall(`shelf_plate_${k}`, 0, 0, k * s.height, hw, hd, 0.004));

  for (let i = 0; i < count; i++) {
    const zc = i * s.height + s.height / 2;
    const trayZ = -(s.height / 2) + 0.02;
    const rodY = -hd - s.rodFront;
    const rodX = hw - 0.03;
    parts.push(
      `<body name="shelf_drawer_${i}" pos="0 0 ${f(zc)}">` +
        `<joint name="${shelfJointName(i)}" type="slide" axis="0 -1 0" ` +
        `range="0 ${f(s.openLimit)}" limited="true" damping="8"/>` +
        `<geom name="shelf_tray_${i}" type="box" pos="0 0 ${f(trayZ)}" ` +
        `size="${f(hw - 0.012)} ${f(hd - 0.012)} 0.006" rgba="0.80 0.70 0.55 1" contype="1" conaffinity="1"/>` +
        `<geom name="shelf_rod_${i}" type="capsule" ` +
        `fromto="${f(-rodX)} ${f(rodY)} 0 ${f(rodX)} ${f(rodY)} 0" size="${f(s.rodRadius)}" ` +
        `rgba="0.20 0.20 0.22 1" contype="1" conaffinity="1"/>` +
        `<geom name="shelf_arm_l_${i}" type="box" pos="${f(-rodX)} ${f(rodY / 2 - hd / 2)} 0" ` +
        `size="0.006 ${f(hd / 2 + s.rodFront / 2)} 0.006" rgba="0.20 0.20 0.22 1" contype="1" conaffinity="1"/>` +
        `<geom name="shelf_arm_r_${i}" type="box" pos="${f(rodX)} ${f(rodY / 2 - hd / 2)} 0" ` +
        `size="0.006 ${f(hd / 2 + s.rodFront / 2)} 0.006" rgba="0.20 0.20 0.22 1" contype="1" conaffinity="1"/>` +
        `</body>`,
    );
  }

  return `<body name="shelf_unit" mocap="true" pos="0 0 0">\n      ${parts.join('\n      ')}\n    </body>`;
}

/** Splice the shelf unit into a scene XML before `</worldbody>`. */
export function injectShelves(xml: string, count: number): string {
  if (count <= 0) return xml;
  const idx = xml.lastIndexOf('</worldbody>');
  if (idx < 0) return xml;
  return xml.slice(0, idx) + `  ${buildShelvesXml(count)}\n  ` + xml.slice(idx);
}

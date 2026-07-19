// Board geometry — direct port of the pure-Python helpers in calibrate_board.py.

/** Output panel size for the rectified (bird's-eye) view, in pixels. */
export const OUT_W = 600;
export const OUT_H = 600;

/**
 * Bordered tag IDs (only these are treated as objects on the board).
 * Must match the IDs printed by make_bordered_tags.py. Board border tags are
 * excluded separately (they appear in the tag-centres map).
 */
export const BORDERED_IDS = new Set<number>(
  Array.from({ length: 100 }, (_, i) => 100 + i),
);

/**
 * Bordered-tag margin fraction. From bordered_aruco.py defaults at
 * tag_size_px=200: margin_per_side = 10 + 8 + 6 + 24 = 48, so 48/200.
 * Used to reconstruct the full bordered-tag footprint from inner ArUco corners.
 */
export const MARGIN_FRAC = 48 / 200;

/** Manual position offset (mm) applied to observed interior positions. */
export const DELTA_X = -10;
export const DELTA_Y = 1;

/** Apply the manual offset to an observed interior position. */
export function correctPos(obsX: number, obsY: number): [number, number] {
  return [obsX + DELTA_X, obsY + DELTA_Y];
}

/** Corner labels and order for the 4-corner calibration. */
export const CALIB_CORNER_NAMES = ['TL', 'TR', 'BL', 'BR'] as const;

export type TagCentres = Map<number, [number, number]>;

/**
 * Return {tagId -> (xMm, yMm)} for all border tags.
 * Origin = top-left corner of the square. X -> right, Y -> down.
 */
export function boardTagCentres(
  squareMm: number,
  tagMm: number,
  gapMm: number,
  nOuter: number,
  nInner: number,
): TagCentres {
  const centres: TagCentres = new Map();
  let tagId = 0;

  type Side = 'top' | 'bottom' | 'left' | 'right';

  const sideCentres = (
    insetMm: number,
    n: number,
    side: Side,
  ): Array<[number, number]> => {
    const half = tagMm / 2;
    const margin = insetMm + half;
    const span = squareMm - 2 * margin;
    const positions: Array<[number, number]> = [];
    for (let i = 0; i < n; i++) {
      const t = n > 1 ? i / (n - 1) : 0.5;
      const along = margin + t * span;
      if (side === 'top') {
        positions.push([along, insetMm + half]);
      } else if (side === 'bottom') {
        positions.push([along, squareMm - insetMm - half]);
      } else if (side === 'left') {
        if (i > 0 && i < n - 1) positions.push([insetMm + half, along]);
      } else if (side === 'right') {
        if (i > 0 && i < n - 1) positions.push([squareMm - insetMm - half, along]);
      }
    }
    return positions;
  };

  const sides: Side[] = ['top', 'bottom', 'left', 'right'];

  for (const side of sides) {
    for (const [cx, cy] of sideCentres(0, nOuter, side)) {
      centres.set(tagId, [cx, cy]);
      tagId += 1;
    }
  }

  const innerInset = tagMm + gapMm;
  for (const side of sides) {
    for (const [cx, cy] of sideCentres(innerInset, nInner, side)) {
      centres.set(tagId, [cx, cy]);
      tagId += 1;
    }
  }

  return centres;
}

// Shared runtime state types.

export interface BoardParams {
  camera: number;
  squareMm: number;
  tagMm: number;
  gapMm: number;
  nOuter: number;
  nInner: number;
  blockTag: number;
  blockHeight: number;
  blockW: number;
  blockD: number;
}

export const DEFAULT_PARAMS: BoardParams = {
  camera: 0,
  squareMm: 180,
  tagMm: 16,
  gapMm: 2,
  nOuter: 10,
  nInner: 8,
  blockTag: 101,
  blockHeight: 15,
  blockW: 25,
  blockD: 75,
};

export interface CalibState {
  active: boolean;
  done: boolean;
  step: number;
  /** rows: [trueX, trueY, obsX, obsY, heightMm] */
  data: number[][];
  /** recorded corner target positions in warped px */
  donePx: Array<[number, number]>;
  cornerPositions: Array<[number, number]>;
  blockTag: number;
  blockW: number;
  blockD: number;
}

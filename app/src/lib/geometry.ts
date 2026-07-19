// Pure geometry / fitting helpers ported from calibrate_board.py.

import { MARGIN_FRAC, CALIB_CORNER_NAMES } from './board';

export type Pt = [number, number];
/** ArUco corners in detection order: [TL, TR, BR, BL]. */
export type Corners = [Pt, Pt, Pt, Pt];

/**
 * Compute the 4 corners of the full bordered tag in image-pixel space.
 * The ArUco detector returns the inner bit-grid corners only; the bordered tag
 * extends MARGIN_FRAC * side further on every side.
 * Input/output corner order: [TL, TR, BR, BL].
 */
export function borderedCornersImage(c: Corners): Corners {
  const [tl, tr, br, bl] = c;
  const right: Pt = [tr[0] - tl[0], tr[1] - tl[1]]; // along top edge
  const down: Pt = [bl[0] - tl[0], bl[1] - tl[1]]; // along left edge
  const m = MARGIN_FRAC;
  return [
    [tl[0] - m * right[0] - m * down[0], tl[1] - m * right[1] - m * down[1]],
    [tr[0] + m * right[0] - m * down[0], tr[1] + m * right[1] - m * down[1]],
    [br[0] + m * right[0] + m * down[0], br[1] + m * right[1] + m * down[1]],
    [bl[0] - m * right[0] + m * down[0], bl[1] - m * right[1] + m * down[1]],
  ];
}

export interface LinearFit {
  Sx: number;
  Bx: number;
  Sy: number;
  By: number;
  rms: number;
  /** Per-corner rows: [trueX, trueY, obsX, obsY, heightMm]. */
  measurements: number[][];
}

/** Ordinary least-squares fit of `true = S * obs + B` for one axis. */
function fitAxis(obs: number[], truth: number[]): [number, number] {
  const n = obs.length;
  const meanO = obs.reduce((a, b) => a + b, 0) / n;
  const meanT = truth.reduce((a, b) => a + b, 0) / n;
  let cov = 0;
  let varO = 0;
  for (let i = 0; i < n; i++) {
    cov += (obs[i] - meanO) * (truth[i] - meanT);
    varO += (obs[i] - meanO) ** 2;
  }
  const S = varO === 0 ? 0 : cov / varO;
  const B = meanT - S * meanO;
  return [S, B];
}

/**
 * Fit a per-axis linear correction from 4 corner measurements.
 * `calibData` rows are [trueX, trueY, obsX, obsY, heightMm].
 */
export function fitLinear2d(calibData: number[][]): LinearFit {
  const trueX = calibData.map((r) => r[0]);
  const trueY = calibData.map((r) => r[1]);
  const obsX = calibData.map((r) => r[2]);
  const obsY = calibData.map((r) => r[3]);

  const [Sx, Bx] = fitAxis(obsX, trueX);
  const [Sy, By] = fitAxis(obsY, trueY);

  let sq = 0;
  for (let i = 0; i < calibData.length; i++) {
    const px = Sx * obsX[i] + Bx;
    const py = Sy * obsY[i] + By;
    sq += (px - trueX[i]) ** 2 + (py - trueY[i]) ** 2;
  }
  const rms = Math.sqrt(sq / calibData.length);

  return { Sx, Bx, Sy, By, rms, measurements: calibData };
}

/** Human-readable summary matching the Python console report. */
export function formatFitReport(fit: LinearFit): string {
  const lines: string[] = [];
  lines.push('2-D LINEAR CALIBRATION RESULTS');
  lines.push(
    ['Corner', 'TrueX', 'TrueY', 'ObsX', 'ObsY', 'ErrX', 'ErrY']
      .map((s) => s.padStart(7))
      .join(' '),
  );
  fit.measurements.forEach((row, i) => {
    const [tx, ty, ox, oy] = row;
    const px = fit.Sx * ox + fit.Bx;
    const py = fit.Sy * oy + fit.By;
    lines.push(
      [
        CALIB_CORNER_NAMES[i],
        tx.toFixed(2),
        ty.toFixed(2),
        ox.toFixed(2),
        oy.toFixed(2),
        (px - tx).toFixed(2),
        (py - ty).toFixed(2),
      ]
        .map((s) => s.padStart(7))
        .join(' '),
    );
  });
  lines.push('');
  lines.push(`RMS residual: ${fit.rms.toFixed(2)} mm`);
  lines.push(`CORR_Sx = ${fit.Sx.toFixed(5)}`);
  lines.push(`CORR_Bx = ${fit.Bx.toFixed(3)}`);
  lines.push(`CORR_Sy = ${fit.Sy.toFixed(5)}`);
  lines.push(`CORR_By = ${fit.By.toFixed(3)}`);
  return lines.join('\n');
}

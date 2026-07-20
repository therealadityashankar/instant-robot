// MuJoCo Jacobian-based inverse kinematics — position-only damped least squares,
// the same scheme dm_control's qpos_from_site_pose uses, specialised to a small
// arm. Operates on a live MuJoCo model/data pair (see mujocoSession.ts) so it is
// deterministic and unit-testable headless in Node.
//
// Per iteration: mj_forward → site position error → mj_jacSite → reduce the
// Jacobian to the controlled DOFs → solve (J Jᵀ + λ²I) y = err (a 3×3 system for
// position-only) → dq = Jᵀ y → clamp step, integrate into qpos, clamp to limits.

import type { Mj, MjModel, MjData } from './mujocoSession';

export interface IKOptions {
  /** Site to drive to the target (id or name). */
  site: number | string;
  /** DOF indices the solver may move (defaults to all hinge DOFs of the arm). */
  dofIndices: number[];
  maxIters?: number;
  /** Convergence tolerance on position error, metres. */
  tol?: number;
  /** Damping λ (larger = more stable, slower). */
  damping?: number;
  /** Max per-iteration joint-space step norm, radians. */
  stepClamp?: number;
}

export interface IKResult {
  ok: boolean;
  iters: number;
  /** Final position error, metres. */
  error: number;
  /** Resulting angles for `dofIndices`, in the same order. */
  qpos: number[];
}

function invert3(A: number[][]): number[][] | null {
  const det =
    A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
    A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
    A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
  if (Math.abs(det) < 1e-12) return null;
  const inv = [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
  ];
  inv[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) / det;
  inv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) / det;
  inv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / det;
  inv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) / det;
  inv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / det;
  inv[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) / det;
  inv[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) / det;
  inv[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) / det;
  inv[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) / det;
  return inv;
}

export class IKSolver {
  private mj: Mj;
  private model: MjModel;
  private data: MjData;
  private nv: number;
  private siteId: number;
  private jacp: any;
  private jacr: any;
  /** [lo, hi] per DOF, from actuator ctrlrange (falls back to ±π). */
  private limits: Array<[number, number]>;

  constructor(mj: Mj, model: MjModel, data: MjData, site: number | string) {
    this.mj = mj;
    this.model = model;
    this.data = data;
    this.nv = model.nv;
    if (typeof site === 'number') {
      this.siteId = site;
    } else {
      // Resolve name → id via mj_name2id (mjOBJ_SITE = 6). Stable across builds
      // and allocation-free, unlike the named-accessor method.
      const MJOBJ_SITE = 6;
      this.siteId = mj.mj_name2id(model, MJOBJ_SITE, site);
      if (this.siteId < 0) throw new Error(`Site not found: ${site}`);
    }
    this.jacp = new mj.DoubleBuffer(3 * this.nv);
    this.jacr = new mj.DoubleBuffer(3 * this.nv);

    const cr = Array.from(model.actuator_ctrlrange as ArrayLike<number>);
    this.limits = [];
    for (let i = 0; i < this.nv; i++) {
      const lo = cr[i * 2];
      const hi = cr[i * 2 + 1];
      this.limits.push(lo < hi ? [lo, hi] : [-Math.PI, Math.PI]);
    }
  }

  /** Free the WASM-side Jacobian buffers. */
  dispose() {
    try {
      this.jacp.delete();
      this.jacr.delete();
    } catch {
      /* ignore */
    }
  }

  /**
   * Current world position of the driven site, read from the flat `site_xpos`
   * view (a by-reference typed array — no per-call C++ handle allocation).
   */
  sitePosition(): [number, number, number] {
    const sx = this.data.site_xpos as Float64Array;
    const i = this.siteId * 3;
    return [sx[i], sx[i + 1], sx[i + 2]];
  }

  /**
   * Solve so the driven site reaches `target` (world x,y,z). Mutates the DOFs in
   * `opts.dofIndices` on the live data; other DOFs are held fixed.
   */
  solve(target: [number, number, number], opts: Omit<IKOptions, 'site'>): IKResult {
    const { dofIndices } = opts;
    const maxIters = opts.maxIters ?? 200;
    const tol = opts.tol ?? 1e-4;
    const lambda = opts.damping ?? 0.15;
    const stepClamp = opts.stepClamp ?? 0.2;
    const m = dofIndices.length;
    const nv = this.nv;

    let error = NaN;
    for (let it = 0; it < maxIters; it++) {
      this.mj.mj_forward(this.model, this.data);
      const sx = this.data.site_xpos as Float64Array;
      const si = this.siteId * 3;
      const err = [target[0] - sx[si], target[1] - sx[si + 1], target[2] - sx[si + 2]];
      error = Math.hypot(err[0], err[1], err[2]);
      if (error < tol) {
        return { ok: true, iters: it, error, qpos: this.readDofs(dofIndices) };
      }

      this.mj.mj_jacSite(this.model, this.data, this.jacp, this.jacr, this.siteId);
      const J = this.jacp.GetView() as Float64Array; // 3 × nv, row-major

      // Reduced Jacobian Jr (3 × m) over the controlled DOFs.
      const Jr = [new Array(m), new Array(m), new Array(m)];
      for (let r = 0; r < 3; r++) {
        for (let c = 0; c < m; c++) Jr[r][c] = J[r * nv + dofIndices[c]];
      }

      // A = Jr Jrᵀ + λ²I  (3×3)
      const A = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
      ];
      for (let i = 0; i < 3; i++) {
        for (let j = 0; j < 3; j++) {
          let s = 0;
          for (let k = 0; k < m; k++) s += Jr[i][k] * Jr[j][k];
          A[i][j] = s + (i === j ? lambda * lambda : 0);
        }
      }
      const inv = invert3(A);
      if (!inv) return { ok: false, iters: it, error, qpos: this.readDofs(dofIndices) };

      // y = A⁻¹ err ; dq = Jrᵀ y
      const y = [
        inv[0][0] * err[0] + inv[0][1] * err[1] + inv[0][2] * err[2],
        inv[1][0] * err[0] + inv[1][1] * err[1] + inv[1][2] * err[2],
        inv[2][0] * err[0] + inv[2][1] * err[1] + inv[2][2] * err[2],
      ];
      const dq = new Array(m).fill(0);
      for (let c = 0; c < m; c++) {
        let s = 0;
        for (let r = 0; r < 3; r++) s += Jr[r][c] * y[r];
        dq[c] = s;
      }

      // Clamp step norm and apply, respecting joint limits.
      const dn = Math.hypot(...dq);
      const sc = dn > stepClamp ? stepClamp / dn : 1;
      const q = this.data.qpos;
      for (let c = 0; c < m; c++) {
        const d = dofIndices[c];
        const [lo, hi] = this.limits[d];
        q[d] = Math.max(lo, Math.min(hi, q[d] + dq[c] * sc));
      }
    }
    return { ok: false, iters: maxIters, error, qpos: this.readDofs(dofIndices) };
  }

  private readDofs(dofIndices: number[]): number[] {
    const q = this.data.qpos;
    return dofIndices.map((d) => q[d]);
  }
}

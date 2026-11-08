// 곡가공 전판 해석기 — 고유변형(등가하중) + MITC4 셸 + 밴드 Cholesky
// 브라우저(Web Worker)와 Node 양쪽에서 그대로 동작하는 순수 ES 모듈이다. 외부 의존성 없음.
//
// 물리 모델
//   가열선 1줄이 판에 남기는 영구 변형을 "고유변형 적분량" 세 개로 나타낸다.
//     S_t [mm²]  횡방향 막 고유변형 = -횡수축 × 두께
//     B_t [mm³]  횡방향 굽힘 고유변형 = 각변형(rad) × 두께³ / 12
//     S_l [mm²]  종방향 막 고유변형 = -종수축 × W × 두께 / L
//   이 적분량을 가열선을 따른 선적분으로 셸 요소의 등가 절점력에 바꾸고,
//   탄성 1스텝(선형, 초기 곡면 반영)으로 푼다. 반복 가열은 선형 중첩이다.
//
// 좌표 단위 mm, 응력 MPa. 입력 격자는 판의 윗면이고, 두께는 연직(-z) 방향으로 쌓는다.

export const STEEL = Object.freeze({ E: 206000, nu: 0.3 });
export const STRIP_REF = Object.freeze({ W: 300, L: 400 }); // 3성분을 계측한 스트립의 폭·구간

const DOF = 5;
const GP = 1 / Math.sqrt(3);

// ── 작은 벡터 연산 ─────────────────────────────────────────
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
function cross(a, b, o = [0, 0, 0]) {
  const x = a[1] * b[2] - a[2] * b[1];
  const y = a[2] * b[0] - a[0] * b[2];
  const z = a[0] * b[1] - a[1] * b[0];
  o[0] = x; o[1] = y; o[2] = z;
  return o;
}
function normalize(a) {
  const n = Math.hypot(a[0], a[1], a[2]) || 1;
  a[0] /= n; a[1] /= n; a[2] /= n;
  return a;
}

// ── 격자 보간 (Catmull-Rom 곱) ─────────────────────────────
function catmull(p0, p1, p2, p3, t) {
  const t2 = t * t, t3 = t2 * t;
  return 0.5 * (2 * p1 + (p2 - p0) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2 + (3 * p1 - p0 - 3 * p2 + p3) * t3);
}
function gridAt(pts, nr, nc, i, j, k) {
  // 경계 바깥은 선형 외삽
  const ci = Math.min(Math.max(i, 0), nr - 1), cj = Math.min(Math.max(j, 0), nc - 1);
  let v = pts[ci * nc * 3 + cj * 3 + k];
  if (i !== ci) {
    const a = i < 0 ? 0 : nr - 1, b = i < 0 ? 1 : nr - 2;
    v += (pts[a * nc * 3 + cj * 3 + k] - pts[b * nc * 3 + cj * 3 + k]) * Math.abs(i - ci);
  }
  if (j !== cj) {
    const a = j < 0 ? 0 : nc - 1, b = j < 0 ? 1 : nc - 2;
    v += (pts[ci * nc * 3 + a * 3 + k] - pts[ci * nc * 3 + b * 3 + k]) * Math.abs(j - cj);
  }
  return v;
}

/** [row][col][xyz] 또는 평탄 배열을 받아 m 배로 세분한 평탄 격자를 돌려준다. */
export function refineGrid(points, m) {
  const nr0 = points.length, nc0 = points[0].length;
  const flat = new Float64Array(nr0 * nc0 * 3);
  for (let i = 0; i < nr0; i++) for (let j = 0; j < nc0; j++) for (let k = 0; k < 3; k++) flat[(i * nc0 + j) * 3 + k] = points[i][j][k];
  const nr = (nr0 - 1) * m + 1, nc = (nc0 - 1) * m + 1;
  const X = new Float64Array(nr * nc * 3);
  for (let i = 0; i < nr; i++) {
    const i0 = Math.min(Math.floor(i / m), nr0 - 2), ti = i / m - i0;
    for (let j = 0; j < nc; j++) {
      const j0 = Math.min(Math.floor(j / m), nc0 - 2), tj = j / m - j0;
      for (let k = 0; k < 3; k++) {
        const col = [0, 0, 0, 0];
        for (let a = -1; a <= 2; a++) {
          col[a + 1] = catmull(gridAt(flat, nr0, nc0, i0 + a, j0 - 1, k), gridAt(flat, nr0, nc0, i0 + a, j0, k), gridAt(flat, nr0, nc0, i0 + a, j0 + 1, k), gridAt(flat, nr0, nc0, i0 + a, j0 + 2, k), tj);
        }
        X[(i * nc + j) * 3 + k] = catmull(col[0], col[1], col[2], col[3], ti);
      }
    }
  }
  return { X, nr, nc, m, nr0, nc0 };
}

// ── 밴드 Cholesky ──────────────────────────────────────────
// A[i*(b+1)+k] = a(i, i+k), k=0..b. 제자리에서 윗삼각 인자로 바뀐다.
function bandCholesky(A, n, b) {
  const w = b + 1;
  for (let i = 0; i < n; i++) {
    const ii = i * w;
    const p = A[ii];
    if (!(p > 0)) throw new Error(`강성행렬이 양정치가 아닙니다 (자유도 ${i}, 피벗 ${p}). 격자에 접히거나 겹친 부분이 있는지 확인하세요.`);
    const d = Math.sqrt(p);
    A[ii] = d;
    const kMax = Math.min(b, n - 1 - i);
    for (let k = 1; k <= kMax; k++) A[ii + k] /= d;
    for (let k = 1; k <= kMax; k++) {
      const v = A[ii + k];
      if (v === 0) continue;
      const row = (i + k) * w - k; // A[row + l] = a(i+k, i+l)
      for (let l = k; l <= kMax; l++) A[row + l] -= v * A[ii + l];
    }
  }
}
function bandSolve(A, n, b, x) {
  const w = b + 1;
  for (let i = 0; i < n; i++) {
    const ii = i * w, kMax = Math.min(b, n - 1 - i);
    x[i] /= A[ii];
    const v = x[i];
    if (v !== 0) for (let k = 1; k <= kMax; k++) x[i + k] -= v * A[ii + k];
  }
  for (let i = n - 1; i >= 0; i--) {
    const ii = i * w, kMax = Math.min(b, n - 1 - i);
    let s = x[i];
    for (let k = 1; k <= kMax; k++) s -= A[ii + k] * x[i + k];
    x[i] = s / A[ii];
  }
  return x;
}

// ── MITC4 요소 계산 ────────────────────────────────────────
function shape(r, s, h, hr, hs) {
  h[0] = 0.25 * (1 - r) * (1 - s); h[1] = 0.25 * (1 + r) * (1 - s); h[2] = 0.25 * (1 + r) * (1 + s); h[3] = 0.25 * (1 - r) * (1 + s);
  hr[0] = -0.25 * (1 - s); hr[1] = 0.25 * (1 - s); hr[2] = 0.25 * (1 + s); hr[3] = -0.25 * (1 + s);
  hs[0] = -0.25 * (1 - r); hs[1] = -0.25 * (1 + r); hs[2] = 0.25 * (1 + r); hs[3] = 0.25 * (1 - r);
}

class Mitc4 {
  constructor(a) {
    this.a = a; // t/2
    this.h = new Float64Array(4); this.hr = new Float64Array(4); this.hs = new Float64Array(4);
    this.gr = [0, 0, 0]; this.gs = [0, 0, 0]; this.gz = [0, 0, 0];
    this.node = { x: [], v1: [], v2: [], v3: [] };
    for (let n = 0; n < 4; n++) {
      this.node.x.push([0, 0, 0]); this.node.v1.push([0, 0, 0]); this.node.v2.push([0, 0, 0]); this.node.v3.push([0, 0, 0]);
    }
    this.rowsA = {}; this.rowsB = {};
    for (const set of [this.rowsA, this.rowsB]) for (const key of ['rr', 'ss', 'rs', 'rz', 'sz']) set[key] = new Float64Array(20);
    this.cov = {}; for (const key of ['rr', 'ss', 'rs', 'rz', 'sz']) this.cov[key] = new Float64Array(20);
    this.B = []; for (let i = 0; i < 5; i++) this.B.push(new Float64Array(20));
    this.c = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    this.e1 = [0, 0, 0]; this.e2 = [0, 0, 0]; this.e3 = [0, 0, 0];
    this.detJ = 0;
  }

  // (r,s,z)에서 공변 변형률 행들을 rows 에 채우고 기저벡터를 계산한다.
  covariant(r, s, z, rows) {
    const { h, hr, hs, gr, gs, gz, a, node } = this;
    shape(r, s, h, hr, hs);
    for (let k = 0; k < 3; k++) {
      let vr = 0, vs = 0, vz = 0;
      for (let n = 0; n < 4; n++) {
        const q = node.x[n][k] + z * a * node.v3[n][k];
        vr += hr[n] * q; vs += hs[n] * q; vz += a * h[n] * node.v3[n][k];
      }
      gr[k] = vr; gs[k] = vs; gz[k] = vz;
    }
    const { rr, ss, rs, rz, sz } = rows;
    for (let n = 0; n < 4; n++) {
      const o = 5 * n, pr = hr[n], ps = hs[n], p = h[n];
      for (let k = 0; k < 3; k++) {
        rr[o + k] = gr[k] * pr;
        ss[o + k] = gs[k] * ps;
        rs[o + k] = gr[k] * ps + gs[k] * pr;
        rz[o + k] = gz[k] * pr;
        sz[o + k] = gz[k] * ps;
      }
      const v1 = node.v1[n], v2 = node.v2[n];
      for (let q = 0; q < 2; q++) {
        const phi = q === 0 ? [-v2[0], -v2[1], -v2[2]] : v1;
        const cr = z * a * pr, cs = z * a * ps, cz = a * p;
        const grp = dot(gr, phi), gsp = dot(gs, phi), gzp = dot(gz, phi);
        rr[o + 3 + q] = grp * cr;
        ss[o + 3 + q] = gsp * cs;
        rs[o + 3 + q] = grp * cs + gsp * cr;
        rz[o + 3 + q] = grp * cz + gzp * cr;
        sz[o + 3 + q] = gsp * cz + gzp * cs;
      }
    }
  }

  // (r,s,z)에서 국소 직교좌표 B 행렬 [e11, e22, g12, g13, g23] 을 만든다.
  localB(r, s, z) {
    const { rowsA, rowsB, cov } = this;
    // MITC 전단 묶음점
    this.covariant(0, 1, z, rowsA); const rzP = rowsA.rz.slice();
    this.covariant(0, -1, z, rowsA); const rzM = rowsA.rz.slice();
    this.covariant(1, 0, z, rowsA); const szP = rowsA.sz.slice();
    this.covariant(-1, 0, z, rowsA); const szM = rowsA.sz.slice();
    this.covariant(r, s, z, rowsB);
    for (let i = 0; i < 20; i++) {
      cov.rr[i] = rowsB.rr[i]; cov.ss[i] = rowsB.ss[i]; cov.rs[i] = rowsB.rs[i];
      cov.rz[i] = 0.5 * (1 + s) * rzP[i] + 0.5 * (1 - s) * rzM[i];
      cov.sz[i] = 0.5 * (1 + r) * szP[i] + 0.5 * (1 - r) * szM[i];
    }
    const { gr, gs, gz, e1, e2, e3, c, B } = this;
    const gsXgz = cross(gs, gz), gzXgr = cross(gz, gr), grXgs = cross(gr, gs);
    const J = dot(gr, gsXgz);
    this.detJ = Math.abs(J);
    e3[0] = gz[0]; e3[1] = gz[1]; e3[2] = gz[2]; normalize(e3);
    const q = dot(gr, e3);
    e1[0] = gr[0] - q * e3[0]; e1[1] = gr[1] - q * e3[1]; e1[2] = gr[2] - q * e3[2]; normalize(e1);
    cross(e3, e1, e2);
    const bases = [e1, e2, e3];
    for (let b = 0; b < 3; b++) {
      c[0][b] = dot(gsXgz, bases[b]) / J; // g^r · e_b
      c[1][b] = dot(gzXgr, bases[b]) / J; // g^s · e_b
      c[2][b] = dot(grXgs, bases[b]) / J; // g^z · e_b
    }
    const comp = (a, b, f) => {
      const row = B[f.idx], m = f.m;
      const crr = c[0][a] * c[0][b];
      const css = c[1][a] * c[1][b];
      const crs = 0.5 * (c[0][a] * c[1][b] + c[1][a] * c[0][b]);
      const crz = 0.5 * (c[0][a] * c[2][b] + c[2][a] * c[0][b]);
      const csz = 0.5 * (c[1][a] * c[2][b] + c[2][a] * c[1][b]);
      for (let i = 0; i < 20; i++) row[i] = m * (crr * cov.rr[i] + css * cov.ss[i] + crs * cov.rs[i] + crz * cov.rz[i] + csz * cov.sz[i]);
    };
    comp(0, 0, { idx: 0, m: 1 });
    comp(1, 1, { idx: 1, m: 1 });
    comp(0, 1, { idx: 2, m: 2 });
    comp(0, 2, { idx: 3, m: 2 });
    comp(1, 2, { idx: 4, m: 2 });
    return B;
  }
}

// ── 판 모델 ────────────────────────────────────────────────
export class PlateModel {
  /**
   * @param points  [row][col]=[x,y,z] 윗면 격자
   * @param opt     { thickness, E, nu, refine, shearK }
   */
  constructor(points, opt) {
    this.t = opt.thickness;
    this.E = opt.E ?? STEEL.E;
    this.nu = opt.nu ?? STEEL.nu;
    this.shearK = opt.shearK ?? 5 / 6;
    const g = refineGrid(points, opt.refine ?? 2);
    this.g = g;
    this.nr = g.nr; this.nc = g.nc;
    this.nn = g.nr * g.nc;
    this.neq = this.nn * DOF;
    this.bw = DOF * (g.nc + 2) - 1;
    this.X = new Float64Array(this.nn * 3); // 중립면
    this.Xtop = g.X;
    for (let n = 0; n < this.nn; n++) {
      this.X[3 * n] = g.X[3 * n]; this.X[3 * n + 1] = g.X[3 * n + 1]; this.X[3 * n + 2] = g.X[3 * n + 2] - this.t / 2;
    }
    this.V1 = new Float64Array(this.nn * 3); this.V2 = new Float64Array(this.nn * 3); this.V3 = new Float64Array(this.nn * 3);
    this.#directors();
    this.el = new Mitc4(this.t / 2);
    this.factor = null;
    this.bcDofs = [];
    this._hint = 0;
  }

  id(i, j) { return i * this.nc + j; }

  #directors() {
    const { nr, nc, X, V1, V2, V3 } = this;
    const ti = [0, 0, 0], tj = [0, 0, 0], v3 = [0, 0, 0], v1 = [0, 0, 0], v2 = [0, 0, 0];
    for (let i = 0; i < nr; i++) for (let j = 0; j < nc; j++) {
      const n = this.id(i, j);
      const ip = this.id(Math.min(i + 1, nr - 1), j), im = this.id(Math.max(i - 1, 0), j);
      const jp = this.id(i, Math.min(j + 1, nc - 1)), jm = this.id(i, Math.max(j - 1, 0));
      for (let k = 0; k < 3; k++) { ti[k] = X[3 * ip + k] - X[3 * im + k]; tj[k] = X[3 * jp + k] - X[3 * jm + k]; }
      cross(ti, tj, v3); normalize(v3);
      if (v3[2] < 0) { v3[0] = -v3[0]; v3[1] = -v3[1]; v3[2] = -v3[2]; }
      const ref = Math.abs(v3[1]) > 0.9 ? [1, 0, 0] : [0, 1, 0];
      cross(ref, v3, v1); normalize(v1);
      cross(v3, v1, v2);
      for (let k = 0; k < 3; k++) { V1[3 * n + k] = v1[k]; V2[3 * n + k] = v2[k]; V3[3 * n + k] = v3[k]; }
    }
  }

  #loadElement(i, j) {
    const ids = [this.id(i, j), this.id(i + 1, j), this.id(i + 1, j + 1), this.id(i, j + 1)];
    const { node } = this.el;
    for (let n = 0; n < 4; n++) {
      const q = ids[n];
      for (let k = 0; k < 3; k++) {
        node.x[n][k] = this.X[3 * q + k]; node.v1[n][k] = this.V1[3 * q + k]; node.v2[n][k] = this.V2[3 * q + k]; node.v3[n][k] = this.V3[3 * q + k];
      }
    }
    return ids;
  }

  /** 강성 조립 + 3-2-1 구속 + 분해. onProgress(0~1) 선택. */
  factorize(onProgress) {
    const { neq, bw, nr, nc, E, nu, shearK, el } = this;
    const w = bw + 1;
    const A = new Float64Array(neq * w);
    const D = E / (1 - nu * nu), G = shearK * E / (2 * (1 + nu));
    const a = el.a;
    const DB = [new Float64Array(20), new Float64Array(20), new Float64Array(20), new Float64Array(20), new Float64Array(20)];
    const Ke = new Float64Array(400);
    const gauss = [-GP, GP];
    const totalE = (nr - 1) * (nc - 1);
    let doneE = 0;
    for (let i = 0; i < nr - 1; i++) {
      for (let j = 0; j < nc - 1; j++) {
        const ids = this.#loadElement(i, j);
        Ke.fill(0);
        for (const r of gauss) for (const s of gauss) for (const z of gauss) {
          const B = el.localB(r, s, z);
          const dV = el.detJ;
          for (let c = 0; c < 20; c++) {
            DB[0][c] = D * (B[0][c] + nu * B[1][c]);
            DB[1][c] = D * (nu * B[0][c] + B[1][c]);
            DB[2][c] = D * 0.5 * (1 - nu) * B[2][c];
            DB[3][c] = G * B[3][c];
            DB[4][c] = G * B[4][c];
          }
          for (let p = 0; p < 20; p++) for (let q = p; q < 20; q++) {
            let v = 0;
            for (let m = 0; m < 5; m++) v += B[m][p] * DB[m][q];
            Ke[p * 20 + q] += v * dV;
          }
        }
        for (let p = 0; p < 20; p++) {
          const P = ids[p / 5 | 0] * DOF + p % 5;
          for (let q = p; q < 20; q++) {
            const Q = ids[q / 5 | 0] * DOF + q % 5;
            const lo = Math.min(P, Q), hi = Math.max(P, Q);
            A[lo * w + (hi - lo)] += Ke[p * 20 + q];
          }
        }
        doneE++;
      }
      if (onProgress) onProgress(0.5 * doneE / totalE);
    }
    void a;
    // 대각 스케일 + 3-2-1 정정 구속 (세 모서리)
    const nA = this.id(0, 0), nB = this.id(nr - 1, 0), nC = this.id(0, nc - 1);
    this.bcDofs = [nA * DOF, nA * DOF + 1, nA * DOF + 2, nB * DOF + 1, nB * DOF + 2, nC * DOF + 2];
    for (const d of this.bcDofs) {
      for (let k = 0; k <= bw && d + k < neq; k++) A[d * w + k] = 0;
      for (let k = 1; k <= bw && d - k >= 0; k++) A[(d - k) * w + k] = 0;
      A[d * w] = 1;
    }
    bandCholesky(A, neq, bw);
    if (onProgress) onProgress(1);
    this.factor = A;
    return this;
  }

  /** xy 평면에서 점을 포함하는 요소와 (r,s)를 찾는다. 판 바깥이면 null. */
  locate(x, y) {
    const { nr, nc, X } = this;
    const tryElement = (e) => {
      const i = Math.floor(e / (nc - 1)), j = e % (nc - 1);
      const ids = [this.id(i, j), this.id(i + 1, j), this.id(i + 1, j + 1), this.id(i, j + 1)];
      const px = ids.map((n) => X[3 * n]), py = ids.map((n) => X[3 * n + 1]);
      const bx0 = Math.min(...px), bx1 = Math.max(...px), by0 = Math.min(...py), by1 = Math.max(...py);
      const mx = (bx1 - bx0) * 0.05 + 1e-6, my = (by1 - by0) * 0.05 + 1e-6;
      if (x < bx0 - mx || x > bx1 + mx || y < by0 - my || y > by1 + my) return null;
      let r = 0, s = 0;
      const h = new Float64Array(4), hr = new Float64Array(4), hs = new Float64Array(4);
      for (let it = 0; it < 12; it++) {
        shape(r, s, h, hr, hs);
        let fx = -x, fy = -y, a11 = 0, a12 = 0, a21 = 0, a22 = 0;
        for (let n = 0; n < 4; n++) { fx += h[n] * px[n]; fy += h[n] * py[n]; a11 += hr[n] * px[n]; a12 += hs[n] * px[n]; a21 += hr[n] * py[n]; a22 += hs[n] * py[n]; }
        const det = a11 * a22 - a12 * a21;
        if (Math.abs(det) < 1e-12) return null;
        const dr = (a22 * fx - a12 * fy) / det, ds = (-a21 * fx + a11 * fy) / det;
        r -= dr; s -= ds;
        if (Math.abs(dr) + Math.abs(ds) < 1e-9) break;
      }
      if (Math.abs(r) > 1 + 1e-6 || Math.abs(s) > 1 + 1e-6) return null;
      return { i, j, r: Math.max(-1, Math.min(1, r)), s: Math.max(-1, Math.min(1, s)) };
    };
    const ne = (nr - 1) * (nc - 1);
    const hint = this._hint;
    const span = [0, 1, -1, nc - 1, -(nc - 1), nc - 2, nc, -(nc - 2), -nc];
    for (const d of span) {
      const e = hint + d;
      if (e < 0 || e >= ne) continue;
      const hit = tryElement(e);
      if (hit) { this._hint = e; return hit; }
    }
    for (let e = 0; e < ne; e++) {
      const hit = tryElement(e);
      if (hit) { this._hint = e; return hit; }
    }
    return null;
  }

  /**
   * 가열선 1줄의 단위 하중 벡터. comp = { S_t, B_t, S_l } 또는 { theta_mrad, dtrans_mm, dlong_mm }.
   * 윗면을 가열면으로 본다.
   */
  lineLoad(polyline, comp, out) {
    const { neq, E, nu, t, el } = this;
    const f = out ?? new Float64Array(neq);
    const inv = toInvariants(comp, t);
    const D = E / (1 - nu * nu);
    const q = [0, 0, 0];
    const step = Math.max(10, Math.min(30, 0.2 * this.#meshSize()));
    for (let k = 0; k + 1 < polyline.length; k++) {
      const a = polyline[k], b = polyline[k + 1];
      const dx = b[0] - a[0], dy = b[1] - a[1], dz = b[2] - a[2];
      const len = Math.hypot(dx, dy, dz);
      if (len < 1e-9) continue;
      const count = Math.max(1, Math.ceil(len / step));
      const ds = len / count;
      for (let c = 0; c < count; c++) {
        const u = (c + 0.5) / count;
        const hit = this.locate(a[0] + dx * u, a[1] + dy * u);
        if (!hit) continue;
        const ids = this.#loadElement(hit.i, hit.j);
        for (const g of [-GP, GP]) {
          const B = el.localB(hit.r, hit.s, g);
          const { e1, e2 } = el;
          // 가열선 접선을 국소좌표로 투영
          q[0] = dx / len; q[1] = dy / len; q[2] = dz / len;
          const lx0 = dot(q, e1), ly0 = dot(q, e2);
          const nrm = Math.hypot(lx0, ly0) || 1;
          const lx = lx0 / nrm, ly = ly0 / nrm, tx = -ly, ty = lx;
          // 두께 방향 분포: ε*(z) = S/t + z·B·12/t³ (z = g·t/2)
          const zv = g * t / 2;
          const et = inv.S_t / t + zv * inv.B_t * 12 / (t * t * t);
          const el_ = inv.S_l / t;
          const e11 = el_ * lx * lx + et * tx * tx;
          const e22 = el_ * ly * ly + et * ty * ty;
          const g12 = 2 * (el_ * lx * ly + et * tx * ty);
          const s11 = D * (e11 + nu * e22), s22 = D * (nu * e11 + e22), s12 = D * 0.5 * (1 - nu) * g12;
          const wt = ds * (t / 2);
          for (let p = 0; p < 20; p++) {
            const P = ids[p / 5 | 0] * DOF + p % 5;
            f[P] += wt * (B[0][p] * s11 + B[1][p] * s22 + B[2][p] * s12);
          }
        }
      }
    }
    return f;
  }

  #meshSize() {
    const { nr, nc, X } = this;
    const a = this.id(nr >> 1, nc >> 1), b = this.id((nr >> 1) + 1, nc >> 1), c = this.id(nr >> 1, (nc >> 1) + 1);
    const d1 = Math.hypot(X[3 * a] - X[3 * b], X[3 * a + 1] - X[3 * b + 1]);
    const d2 = Math.hypot(X[3 * a] - X[3 * c], X[3 * a + 1] - X[3 * c + 1]);
    return Math.min(d1, d2);
  }

  /** f 를 복사해 풀이한다. */
  solve(f) {
    if (!this.factor) throw new Error('factorize()를 먼저 호출하세요.');
    const x = Float64Array.from(f);
    for (const d of this.bcDofs) x[d] = 0;
    return bandSolve(this.factor, this.neq, this.bw, x);
  }

  /** 자유도 해를 윗면 원 격자(nr0×nc0×3 평탄 배열) 변위로 바꾼다. */
  topDisplacement(u) {
    const { nr0, nc0, m } = this.g;
    const out = new Float64Array(nr0 * nc0 * 3);
    const a = this.t / 2;
    for (let i = 0; i < nr0; i++) for (let j = 0; j < nc0; j++) {
      const n = this.id(i * m, j * m);
      const al = u[n * DOF + 3], be = u[n * DOF + 4];
      for (let k = 0; k < 3; k++) {
        out[(i * nc0 + j) * 3 + k] = u[n * DOF + k] + a * (-this.V2[3 * n + k] * al + this.V1[3 * n + k] * be);
      }
    }
    return out;
  }

  /** 세분 격자 전체의 윗면 변위 (화면 표시용) */
  topDisplacementFine(u) {
    const out = new Float64Array(this.nn * 3);
    const a = this.t / 2;
    for (let n = 0; n < this.nn; n++) {
      const al = u[n * DOF + 3], be = u[n * DOF + 4];
      for (let k = 0; k < 3; k++) out[3 * n + k] = u[n * DOF + k] + a * (-this.V2[3 * n + k] * al + this.V1[3 * n + k] * be);
    }
    return out;
  }
}

/** 3성분(계측량)을 고유변형 적분량으로 환산한다. */
export function toInvariants(c, t, ref = STRIP_REF) {
  if (c.S_t !== undefined) return { S_t: c.S_t, B_t: c.B_t, S_l: c.S_l };
  return {
    S_t: -c.dtrans_mm * t,
    B_t: (c.theta_mrad / 1000) * t * t * t / 12,
    S_l: -c.dlong_mm * ref.W * t / ref.L,
  };
}

// ── 후처리 ─────────────────────────────────────────────────
function solveSmall(A, b) {
  const n = b.length;
  for (let i = 0; i < n; i++) {
    let piv = i;
    for (let r = i + 1; r < n; r++) if (Math.abs(A[r][i]) > Math.abs(A[piv][i])) piv = r;
    [A[i], A[piv]] = [A[piv], A[i]]; [b[i], b[piv]] = [b[piv], b[i]];
    for (let r = i + 1; r < n; r++) {
      const m = A[r][i] / A[i][i];
      for (let c = i; c < n; c++) A[r][c] -= m * A[i][c];
      b[r] -= m * b[i];
    }
  }
  for (let i = n - 1; i >= 0; i--) {
    for (let c = i + 1; c < n; c++) b[i] -= A[i][c] * b[c];
    b[i] /= A[i][i];
  }
  return b;
}

/** 변위장 U(평탄 3N)에서 강체 6자유도를 최소제곱으로 제거한 복사본을 돌려준다. X 는 기준 좌표. */
export function removeRigid(U, X) {
  const n = U.length / 3;
  let cx = 0, cy = 0, cz = 0;
  for (let i = 0; i < n; i++) { cx += X[3 * i]; cy += X[3 * i + 1]; cz += X[3 * i + 2]; }
  cx /= n; cy /= n; cz /= n;
  const A = Array.from({ length: 6 }, () => new Array(6).fill(0));
  const b = new Array(6).fill(0);
  const row = (x, y, z) => [[1, 0, 0, 0, z, -y], [0, 1, 0, -z, 0, x], [0, 0, 1, y, -x, 0]];
  for (let i = 0; i < n; i++) {
    const R = row(X[3 * i] - cx, X[3 * i + 1] - cy, X[3 * i + 2] - cz);
    for (let k = 0; k < 3; k++) for (let p = 0; p < 6; p++) {
      b[p] += R[k][p] * U[3 * i + k];
      for (let q = 0; q < 6; q++) A[p][q] += R[k][p] * R[k][q];
    }
  }
  const c = solveSmall(A, b);
  const out = new Float64Array(U.length);
  for (let i = 0; i < n; i++) {
    const R = row(X[3 * i] - cx, X[3 * i + 1] - cy, X[3 * i + 2] - cz);
    for (let k = 0; k < 3; k++) {
      let v = 0;
      for (let p = 0; p < 6; p++) v += R[k][p] * c[p];
      out[3 * i + k] = U[3 * i + k] - v;
    }
  }
  return out;
}

/** 정답 대비 R²(평균 기준)와 RMS(mm). */
export function score(pred, truth) {
  const n = truth.length;
  let mean = 0, sse = 0, sst = 0;
  for (let i = 0; i < n; i++) mean += truth[i];
  mean /= n;
  for (let i = 0; i < n; i++) { sse += (pred[i] - truth[i]) ** 2; sst += (truth[i] - mean) ** 2; }
  return { r2: 1 - sse / sst, rms: Math.sqrt(sse / (n / 3)), rmsTruth: Math.sqrt(sst / (n / 3)) };
}

/** [row][col][xyz] → 평탄 배열 */
export function flatten(points) {
  const nr = points.length, nc = points[0].length;
  const out = new Float64Array(nr * nc * 3);
  for (let i = 0; i < nr; i++) for (let j = 0; j < nc; j++) for (let k = 0; k < 3; k++) out[(i * nc + j) * 3 + k] = points[i][j][k];
  return out;
}

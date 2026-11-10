// 가열선 추천 확장 — 주방향을 따라 추적한 곡선 후보(family: 'traced'), 방향 규칙 비교, 정지 규칙, 곡 펴짐 지표
//
// 공개 코드 /studio/planner.js 의 후보 생성(makeCandidates)과 탐욕 선택 절차는 그대로 두고,
// 그 앞단에 추적선 후보를 더한다. 원본 파일은 고치지 않는다. 브라우저(Web Worker)와 Node 양쪽에서 동작하는 순수 ES 모듈이다.
// Node 에서는 '/studio/...' 경로를 공개 저장소로 돌리는 hook 이 필요하다 (test_planner_shin.mjs 의 node_studio_hook.mjs).
//
// 근거 (자세한 내용은 맥시뮬레이터/shin/spec_heating_line_algo.md, 연구조사_2026-11/11_신종계교수_적용계획.md)
//   주방향에 수직으로 긋는 절차 : 김용균·신종계 2002(D06), 박정서 외 2008(D10), 유철호·신종계 2002(C03)
//   위치별 면내·굽힘 큰 쪽 규칙 (D06)
//   입열 문턱 규칙 (N1)          : Kato 외 2025b (Q/h² 문턱 아래면 굽힘 선, 위면 수축 선)
//   시작점·추적 절차의 세부       : 명세 S5 [재구성] — 논문에는 "기준선 양 끝에서 이등분"이라는 설명뿐이고 나머지는 우리가 채웠다
//   곡 펴짐 지표                  : 박정서 외 2008 (방향별 주 굽힘 변형도 제곱합)
//   정지 규칙                     : 김용균·신종계 2002 의 "줄 수를 늘려도 이득이 작을 때"를 값으로 옮긴 것 [제안]
//
// 값에 대해
//   회귀 계수, 스프링 상수 같은 문헌 수치는 넣지 않았다. 기본값은 모두 [제안] 이고 호출할 때 바꿀 수 있다.
//   N1 문턱은 스트립 표에서 구하는 값이라 코드에 넣지 않았다. n1Threshold 에 표를 주거나 opt.n1 으로 준다.
//   이 모듈의 결과는 해석 모델 값이다. 실제 판의 가공 정확도를 뜻하지 않는다.
import { flatten } from '../../studio/solver.js';
import { makeCandidates, boundedFit, zPlaneFree, rms } from '../../studio/planner.js';
import { strains, classifyPlate } from './kinematics.js';

const NU_DEFAULT = 0.3;
const DEG = Math.PI / 180;
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const dot = (a, b) => { let s = 0; for (let i = 0; i < a.length; i++) s += a[i] * b[i]; return s; };

// ── 격자 도구 ─────────────────────────────────────────────
/** 격자 지수 (i, j) 위치의 윗면 좌표 (쌍선형). planner.js 의 surfaceAt 과 같은 식이다. */
function surfaceAt(points, i, j) {
  const nr = points.length, nc = points[0].length;
  const i0 = Math.max(0, Math.min(nr - 2, Math.floor(i))), j0 = Math.max(0, Math.min(nc - 2, Math.floor(j)));
  const u = i - i0, v = j - j0;
  const out = [0, 0, 0];
  for (let k = 0; k < 3; k++) {
    out[k] = (1 - u) * ((1 - v) * points[i0][j0][k] + v * points[i0][j0 + 1][k]) + u * ((1 - v) * points[i0 + 1][j0][k] + v * points[i0 + 1][j0 + 1][k]);
  }
  return out;
}
const toGridObj = (points) => ({ X: flatten(points), nr: points.length, nc: points[0].length });

/** 평판 좌표 x,y 의 격자 지수 미분표 (행 i, 열 j). 가열선은 평판 좌표로 추적하고 격자 지수로 옮긴다. */
function makeChart(points) {
  const nr = points.length, nc = points[0].length, N = nr * nc;
  const xi = new Float64Array(N), xj = new Float64Array(N), yi = new Float64Array(N), yj = new Float64Array(N);
  for (let i = 0; i < nr; i++) for (let j = 0; j < nc; j++) {
    const p = i * nc + j;
    const ip = Math.min(i + 1, nr - 1), im = Math.max(i - 1, 0), jp = Math.min(j + 1, nc - 1), jm = Math.max(j - 1, 0);
    xi[p] = (points[ip][j][0] - points[im][j][0]) / (ip - im);
    yi[p] = (points[ip][j][1] - points[im][j][1]) / (ip - im);
    xj[p] = (points[i][jp][0] - points[i][jm][0]) / (jp - jm);
    yj[p] = (points[i][jp][1] - points[i][jm][1]) / (jp - jm);
  }
  return { nr, nc, xi, xj, yi, yj };
}
/** 평판 좌표의 단위 벡터 (dx, dy)로 1 mm 갈 때의 격자 지수 변화 (di, dj). */
function indexRate(chart, i, j, dx, dy) {
  const { nr, nc, xi, xj, yi, yj } = chart;
  const p = clamp(Math.round(i), 0, nr - 1) * nc + clamp(Math.round(j), 0, nc - 1);
  const det = xi[p] * yj[p] - xj[p] * yi[p];
  return [(yj[p] * dx - xj[p] * dy) / det, (-yi[p] * dx + xi[p] * dy) / det];
}

// ── 전개 대용 (얕은 곡면) [제안] ───────────────────────────
// 전개(명세 S2, unfold)는 아직 없다(P2-1). 목표 곡면을 xy 투영으로만 시작 판과 대응시키면
// 전개 가능 곡면(원통 등)에서도 막 변형률이 ½·(기울기)² 만큼 생겨, 면내 쪽이 지배한다고 잘못 판단하게 된다.
// 그래서 평면응력 판으로 막 에너지를 최소로 하는 (u, v)를 구해 목표 곡면의 재료점을 평면 안에서 옮긴다.
// 이렇게 해서 얻는 막 변형률은 전개 가능 곡면에서는 거의 0, 비전개 곡면에서는 가우스 곡률에 따른 값이 된다.
// unfold 가 생기면 그 결과로 바꾼다.
function bandCholesky(A, n, b) {
  const w = b + 1;
  for (let i = 0; i < n; i++) {
    const ii = i * w, p = A[ii];
    if (!(p > 0)) throw new Error('막 전개 계산에서 강성행렬이 양정치가 아닙니다. 격자에 접히거나 겹친 부분이 있는지 확인하세요.');
    const d = Math.sqrt(p);
    A[ii] = d;
    const kMax = Math.min(b, n - 1 - i);
    for (let k = 1; k <= kMax; k++) A[ii + k] /= d;
    for (let k = 1; k <= kMax; k++) {
      const v = A[ii + k];
      if (v === 0) continue;
      const row = (i + k) * w - k;
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
const GP2 = 1 / Math.sqrt(3);
/**
 * 막 에너지 ∑∫ ½(Bu + e0)ᵀ D (Bu + e0) dA 를 최소로 하는 평면 변위 (u, v)를 구한다.
 * P: 평판 격자 {X, nr, nc} (x, y 를 쓴다), e0: 절점별 막 변형률 {xx, yy, xy} (텐서 성분)
 */
function relaxMembrane(P, e0, nu) {
  const { X, nr, nc } = P, N = nr * nc;
  const neq = 2 * N, bw = 2 * (nc + 1) + 1, W = bw + 1;
  const A = new Float64Array(neq * W), F = new Float64Array(neq);
  const c = 1 / (1 - nu * nu);
  const Dm = [c, c * nu, 0.5 * (1 - nu) * c]; // E=1. 상수 E 는 K, F 양쪽에 곱해져 답에 영향이 없다
  const h = new Float64Array(4), dhr = new Float64Array(4), dhs = new Float64Array(4), dx = new Float64Array(4), dy = new Float64Array(4);
  const shape = (r, s) => {
    h[0] = 0.25 * (1 - r) * (1 - s); h[1] = 0.25 * (1 + r) * (1 - s); h[2] = 0.25 * (1 + r) * (1 + s); h[3] = 0.25 * (1 - r) * (1 + s);
    dhr[0] = -0.25 * (1 - s); dhr[1] = 0.25 * (1 - s); dhr[2] = 0.25 * (1 + s); dhr[3] = -0.25 * (1 + s);
    dhs[0] = -0.25 * (1 - r); dhs[1] = -0.25 * (1 + r); dhs[2] = 0.25 * (1 + r); dhs[3] = 0.25 * (1 - r);
  };
  for (let i = 0; i < nr - 1; i++) for (let j = 0; j < nc - 1; j++) {
    const ids = [i * nc + j, (i + 1) * nc + j, (i + 1) * nc + j + 1, i * nc + j + 1];
    const xs = ids.map((n) => X[3 * n]), ys = ids.map((n) => X[3 * n + 1]);
    for (const r of [-GP2, GP2]) for (const s of [-GP2, GP2]) {
      shape(r, s);
      let a11 = 0, a12 = 0, a21 = 0, a22 = 0;
      for (let n = 0; n < 4; n++) { a11 += dhr[n] * xs[n]; a12 += dhs[n] * xs[n]; a21 += dhr[n] * ys[n]; a22 += dhs[n] * ys[n]; }
      const det = a11 * a22 - a12 * a21;
      for (let n = 0; n < 4; n++) {
        dx[n] = (a22 * dhr[n] - a21 * dhs[n]) / det;
        dy[n] = (-a12 * dhr[n] + a11 * dhs[n]) / det;
      }
      let e0x = 0, e0y = 0, e0xy = 0;
      for (let n = 0; n < 4; n++) { const q = ids[n]; e0x += h[n] * e0.xx[q]; e0y += h[n] * e0.yy[q]; e0xy += h[n] * 2 * e0.xy[q]; }
      const sx = Dm[0] * e0x + Dm[1] * e0y, sy = Dm[1] * e0x + Dm[0] * e0y, sxy = Dm[2] * e0xy;
      for (let a = 0; a < 4; a++) {
        const P0 = 2 * ids[a];
        F[P0] -= (dx[a] * sx + dy[a] * sxy) * det;
        F[P0 + 1] -= (dy[a] * sy + dx[a] * sxy) * det;
        for (let b = a; b < 4; b++) {
          const Q0 = 2 * ids[b];
          const add = (p0, q0, v) => {
            const lo = Math.min(p0, q0), hi = Math.max(p0, q0);
            if (hi - lo <= bw) A[lo * W + (hi - lo)] += v * det;
          };
          // Bᵀ D B
          add(P0, Q0, dx[a] * Dm[0] * dx[b] + dy[a] * Dm[2] * dy[b]);
          add(P0, Q0 + 1, dx[a] * Dm[1] * dy[b] + dy[a] * Dm[2] * dx[b]);
          add(P0 + 1, Q0 + 1, dy[a] * Dm[0] * dy[b] + dx[a] * Dm[2] * dx[b]);
          if (b > a) add(P0 + 1, Q0, dy[a] * Dm[1] * dx[b] + dx[a] * Dm[2] * dy[b]);
        }
      }
    }
  }
  // 강체 운동 3개 고정 (한 절점의 u, v + 먼 절점의 한 성분)
  const a0 = 0, b0 = (nr - 1) * nc;
  const useY = Math.abs(X[3 * b0] - X[3 * a0]) >= Math.abs(X[3 * b0 + 1] - X[3 * a0 + 1]);
  const pins = [2 * a0, 2 * a0 + 1, 2 * b0 + (useY ? 1 : 0)];
  for (const d of pins) {
    for (let k = 0; k <= bw && d + k < neq; k++) A[d * W + k] = 0;
    for (let k = 1; k <= bw && d - k >= 0; k++) A[(d - k) * W + k] = 0;
    A[d * W] = 1; F[d] = 0;
  }
  bandCholesky(A, neq, bw);
  bandSolve(A, neq, bw, F);
  return F; // [u0, v0, u1, v1, ...]
}

// ── 방향장 ───────────────────────────────────────────────
/** 쌍선형으로 방향장을 읽는다. dx,dy 는 기준 좌표의 단위 벡터 (kinematics.js 의 sampleDirField 와 같은 2배각 보간 + anis). */
function sampleField(F, i, j) {
  const { nr, nc, c2, s2, mag, src, anis } = F;
  const i0 = clamp(Math.floor(i), 0, nr - 2), j0 = clamp(Math.floor(j), 0, nc - 2);
  const u = clamp(i - i0, 0, 1), v = clamp(j - j0, 0, 1);
  const p00 = i0 * nc + j0, p01 = p00 + 1, p10 = p00 + nc, p11 = p10 + 1;
  const w00 = (1 - u) * (1 - v), w01 = (1 - u) * v, w10 = u * (1 - v), w11 = u * v;
  const c = w00 * c2[p00] + w01 * c2[p01] + w10 * c2[p10] + w11 * c2[p11];
  const s = w00 * s2[p00] + w01 * s2[p01] + w10 * s2[p10] + w11 * s2[p11];
  const psi = 0.5 * Math.atan2(s, c);
  const near = (u < 0.5 ? (v < 0.5 ? p00 : p01) : (v < 0.5 ? p10 : p11));
  return { dx: Math.cos(psi), dy: Math.sin(psi), mag: w00 * mag[p00] + w01 * mag[p01] + w10 * mag[p10] + w11 * mag[p11], anis: anis ? w00 * anis[p00] + w01 * anis[p01] + w10 * anis[p10] + w11 * anis[p11] : 1, src: src ? src[near] : 1 };
}
const NEW_FIELD = (nr, nc) => ({ nr, nc, c2: new Float64Array(nr * nc), s2: new Float64Array(nr * nc), mag: new Float64Array(nr * nc), src: new Uint8Array(nr * nc), anis: new Float64Array(nr * nc) });
function percentile(a, q) { const s = Float64Array.from(a).sort(); return s.length ? s[clamp(Math.floor(q * s.length), 0, s.length - 1)] : 0; }

/**
 * 방향장 두 벌을 만든다 (명세 S4). 방향의 부호는 없다.
 * 0 번: 가열선이 큰(대수적으로 큰) 주변형·주곡률 방향에 수직 / 1 번: 작은 쪽 방향에 수직 (0 번을 90° 돌린 것)
 * 주곡률의 크기가 비슷한 곳(비틀림의 ±45° 등)에서 크기 순서로 고르면 절점마다 방향이 뒤바뀌므로 대수적 순서로 고정한다.
 * rule 'auto' = 위치별로 면내·굽힘 중 큰 쪽 (D06), 'bending' = 굽힘만, 'membrane' = 면내만
 * mag 는 그 방향에 만들어야 할 변형률 크기 (굽힘은 |κ|·t/2, 면내는 |ε|)
 */
function makeFields(S, rule, t) {
  const { nr, nc } = S, N = nr * nc;
  const pm = S.principal.m, pb = S.principal.b;
  const F = [NEW_FIELD(nr, nc), NEW_FIELD(nr, nc)];
  const tt = t > 0 ? t / 2 : 1;
  const anisOf = (a, b) => Math.abs(a - b) / (Math.abs(a) + Math.abs(b) + 1e-300);
  for (let p = 0; p < N; p++) {
    const e1 = pm.e1[p], e2 = pm.e2[p], k1 = pb.k1[p] * tt, k2 = pb.k2[p] * tt;
    const useM = rule === 'membrane' || (rule === 'auto' && Math.max(Math.abs(e1), Math.abs(e2)) >= Math.max(Math.abs(k1), Math.abs(k2)));
    const phi = useM ? pm.phi1[p] : pb.phi1[p];
    const an = useM ? anisOf(e1, e2) : anisOf(k1, k2);
    for (let f = 0; f < 2; f++) {
      const psi = f === 0 ? phi + Math.PI / 2 : phi;
      F[f].c2[p] = Math.cos(2 * psi); F[f].s2[p] = Math.sin(2 * psi);
      F[f].mag[p] = useM ? Math.abs(f === 0 ? e1 : e2) : Math.abs(f === 0 ? k1 : k2);
      F[f].src[p] = useM ? 0 : 1;
      F[f].anis[p] = an;
    }
  }
  return F;
}

/**
 * 추적선을 만드는 데 필요한 것을 모은다. 내부에서 strains(from, to, flat) 를 부른다.
 * @param points 시작 형상, target 목표 곡면 (같은 격자, [row][col]=[x,y,z])
 * @param opt.thickness 판 두께 mm (필수)
 * @param opt.rule      'auto'(D06) | 'bending' | 'membrane'   방향장 규칙
 * @param opt.window    곡률 창. 1(기본) | 'auto'(잡음에 맞춰 키움) | 정수
 * @param opt.membrane  'min-energy'(기본, 전개 대용) | 'projection'(xy 투영 그대로)
 * @param opt.shrinkOnly 기본 true. 선상가열은 줄이기만 하므로(명세 S2) 전개판을 전체로 키워 최대 주변형이 0 이 되게 한다
 * @param opt.margin    판 끝 여백 비율 (기본 makeCandidates 와 같은 0.05)
 * @param opt.nu        포아송비 (기본 0.3)
 */
export function prepareKinematics(points, target, opt) {
  const t = opt.thickness;
  if (!(t > 0)) throw new Error('prepareKinematics: 판 두께(opt.thickness)가 필요합니다.');
  const rule = opt.rule ?? 'auto';
  const nr = points.length, nc = points[0].length, N = nr * nc;
  const from = toGridObj(points), flat = { X: Float64Array.from(from.X), nr, nc };
  for (let p = 0; p < N; p++) flat.X[3 * p + 2] = 0;
  const win = chooseWindow(points, target, opt);
  const nu = opt.nu ?? NU_DEFAULT;

  // 재료점 대응: 목표 곡면의 x,y 를 막 에너지가 최소가 되게 옮긴다 (전개 대용, [제안])
  const toBase = toGridObj(target);
  let to = toBase;
  let tensionShift = 0;
  if ((opt.membrane ?? 'min-energy') !== 'projection') {
    const base = strains(from, toBase, flat, { use: 'membrane', window: win.window });
    const uv = relaxMembrane(flat, base.eps_m, nu);
    const X2 = Float64Array.from(toBase.X);
    for (let p = 0; p < N; p++) { X2[3 * p] += uv[2 * p]; X2[3 * p + 1] += uv[2 * p + 1]; }
    to = { X: X2, nr, nc };
    if (opt.shrinkOnly !== false) {
      const r1 = strains(from, to, flat, { use: 'membrane', window: win.window });
      let s = 0;
      for (let i = 2; i < nr - 2; i++) for (let j = 2; j < nc - 2; j++) { const p = i * nc + j; if (r1.trusted[p]) s = Math.max(s, r1.principal.m.e1[p]); }
      if (s > 0) {
        let cx = 0, cy = 0;
        for (let p = 0; p < N; p++) { cx += X2[3 * p]; cy += X2[3 * p + 1]; }
        cx /= N; cy /= N;
        const X3 = Float64Array.from(X2);
        for (let p = 0; p < N; p++) { X3[3 * p] = cx + (X2[3 * p] - cx) * (1 - s); X3[3 * p + 1] = cy + (X2[3 * p + 1] - cy) * (1 - s); }
        to = { X: X3, nr, nc };
        tensionShift = s;
      }
    }
  }
  const S = strains(from, to, flat, { use: 'auto', thickness: t, window: win.window });
  const fields = makeFields(S, rule, t);
  const magRef = fields.map((F) => percentile(innerValues(F.mag, S.trusted, nr, nc), 0.99));
  const margin = opt.margin ?? 0.05;
  return {
    nr, nc, points, target, thickness: t, rule, S, fields, magRef, tensionShift,
    spanX: spanOf(points, 0), spanY: spanOf(points, 1),
    chart: makeChart(points), window: win.window, noise: win.noise, margin,
    bounds: { i0: (nr - 1) * margin, i1: (nr - 1) * (1 - margin), j0: (nc - 1) * margin, j1: (nc - 1) * (1 - margin) },
    spacing: spacingOf(points),
  };
}
function spanOf(points, k) {
  let lo = Infinity, hi = -Infinity;
  for (const row of points) for (const q of row) { if (q[k] < lo) lo = q[k]; if (q[k] > hi) hi = q[k]; }
  return hi - lo;
}
function innerValues(a, trusted, nr, nc) {
  const v = [];
  for (let i = 1; i < nr - 1; i++) for (let j = 1; j < nc - 1; j++) { const p = i * nc + j; if (trusted[p]) v.push(a[p]); }
  return v;
}
function spacingOf(points) {
  const nr = points.length, nc = points[0].length;
  const a = points[nr >> 1][nc >> 1], b = points[(nr >> 1) + 1][nc >> 1], c = points[nr >> 1][(nc >> 1) + 1];
  return { dr: Math.hypot(a[0] - b[0], a[1] - b[1]) || 1, dc: Math.hypot(a[0] - c[0], a[1] - c[1]) || 1 };
}
/** 곡률 창. opt.window 가 숫자면 그대로, 'auto' 면 z 차이 곡면의 잡음 비율이 0.1 이하가 되는 가장 작은 창을 쓴다 (kinematics.js 의 신뢰 기준). */
function chooseWindow(points, target, opt) {
  const w = opt.window ?? 1;
  if (w !== 'auto') return { window: w, noise: null };
  const nr = points.length, nc = points[0].length;
  const diff = points.map((row, i) => row.map((q, j) => [q[0], q[1], target[i][j][2] - q[2]]));
  const maxWin = opt.maxWindow ?? Math.max(1, Math.floor(Math.min(nr, nc) / 5));
  let last = null;
  for (let win = 1; win <= maxWin; win++) {
    const c = classifyPlate(diff, { window: win });
    last = { window: win, noise: c.noise };
    if (c.noise.reliable) break;
  }
  return last;
}

/** 시험용: 방향장을 직접 준다. fields = [F0] 또는 [F0, F1] (kinematics.js 의 필드 형식 + anis). */
export function kinFromField(points, fields, opt = {}) {
  const nr = points.length, nc = points[0].length;
  const margin = opt.margin ?? 0.05;
  return {
    nr, nc, points, thickness: opt.thickness ?? 1, rule: 'test', fields, magRef: fields.map((F) => percentile(F.mag, 0.99) || 1),
    spanX: spanOf(points, 0), spanY: spanOf(points, 1),
    chart: makeChart(points), window: 1, noise: null, margin, tensionShift: 0, S: null,
    bounds: { i0: (nr - 1) * margin, i1: (nr - 1) * (1 - margin), j0: (nc - 1) * margin, j1: (nc - 1) * (1 - margin) },
    spacing: spacingOf(points),
  };
}

// ── 추적 (명세 S5) ─────────────────────────────────────────
function clipToBounds(p, pn, B) {
  let t = 1;
  const check = (a, b, lo, hi) => {
    if (b < lo) t = Math.min(t, (lo - a) / (b - a));
    if (b > hi) t = Math.min(t, (hi - a) / (b - a));
  };
  check(p[0], pn[0], B.i0, B.i1);
  check(p[1], pn[1], B.j0, B.j1);
  return [p[0] + (pn[0] - p[0]) * t, p[1] + (pn[1] - p[1]) * t];
}
const inside = (p, B) => p[0] >= B.i0 - 1e-9 && p[0] <= B.i1 + 1e-9 && p[1] >= B.j0 - 1e-9 && p[1] <= B.j1 + 1e-9;

/**
 * 시작점 한 쪽으로 추적한다. 2차 룽게-쿠타, 평판 좌표(mm) 기준 한 걸음 ds.
 * @returns { path:[[i,j],...](시작점 제외), reason:'edge'|'low'|'kink'|'undefined'|'length' , kink:bool }
 */
function traceOneWay(kin, f, p0, d0, opt) {
  const F = kin.fields[f], B = kin.bounds;
  const ds = opt.ds, maxLen = opt.maxLength;
  const cosKink = Math.cos((opt.kinkDeg ?? 30) * DEG);
  const maxTurn = (opt.maxTurnDeg ?? 120) * DEG;
  const magStop = opt.magStop, anisStop = opt.anisStop;
  const path = [];
  let p = p0, d = d0, len = 0, turned = 0;
  const step = (q, dirv, h) => { const r = indexRate(kin.chart, q[0], q[1], dirv[0], dirv[1]); return [q[0] + r[0] * h, q[1] + r[1] * h]; };
  const aligned = (s, ref) => { let v = [s.dx, s.dy]; if (v[0] * ref[0] + v[1] * ref[1] < 0) v = [-v[0], -v[1]]; return v; };
  for (let n = 0; n < 100000; n++) {
    if (len >= maxLen) return { path, reason: 'length' };
    const a = aligned(sampleField(F, p[0], p[1]), d);
    const pm = step(p, a, ds / 2);
    let b = a;
    if (inside(pm, B)) b = aligned(sampleField(F, pm[0], pm[1]), a);
    const pn = step(p, b, ds);
    const cosTurn = clamp(b[0] * d[0] + b[1] * d[1], -1, 1);
    if (cosTurn < cosKink) return { path, reason: 'kink', kink: true };
    turned += Math.acos(cosTurn);
    if (turned > maxTurn) return { path, reason: 'turn' }; // 한 줄이 크게 도는 곳(방향이 정해지지 않는 곳 근처)에서는 끝낸다
    if (!inside(pn, B)) { const c = clipToBounds(p, pn, B); path.push(c); return { path, reason: 'edge' }; }
    const s = sampleField(F, pn[0], pn[1]);
    if (magStop > 0 && s.mag < magStop) return { path, reason: 'low' };
    if (anisStop > 0 && s.anis < anisStop) return { path, reason: 'undefined' };
    path.push(pn); p = pn; d = b; len += ds;
  }
  return { path, reason: 'length' };
}

/**
 * 시작점에서 양방향으로 추적해 한 줄을 만든다 (명세 S5).
 * @param seed { i, j, f } 격자 지수와 방향장 번호 f (0|1). 판 끝 여백 안이어야 한다
 * @param opt  { ds, kinkDeg, demandStop, anisStop, minLength, maxPoints }
 * @returns null(짧거나 시작점이 범위 밖) | { path:[[i,j],...], points:[[x,y,z],...], length, ends:[끝이유, 끝이유], kink, f, deg }
 */
export function traceLine(kin, seed, opt = {}) {
  const f = seed.f ?? 0;
  if (!inside([seed.i, seed.j], kin.bounds)) return null;
  const ds = opt.ds ?? Math.min(kin.spacing.dr, kin.spacing.dc) / 2;
  const magStop = (opt.demandStop ?? 0.05) * kin.magRef[f];
  const rec = { ds, kinkDeg: opt.kinkDeg ?? 30, magStop: opt.useDemand === false ? 0 : magStop, anisStop: opt.anisStop ?? 0.02, maxTurnDeg: opt.maxTurnDeg, maxLength: opt.maxLength ?? 1.5 * Math.max(kin.spanX ?? 0, kin.spanY ?? 0) };
  const s0 = sampleField(kin.fields[f], seed.i, seed.j);
  if (rec.magStop > 0 && s0.mag < rec.magStop) return null;
  const d0 = [s0.dx, s0.dy];
  const fw = traceOneWay(kin, f, [seed.i, seed.j], d0, rec);
  const bw = traceOneWay(kin, f, [seed.i, seed.j], [-d0[0], -d0[1]], rec);
  const path = [...bw.path.slice().reverse(), [seed.i, seed.j], ...fw.path];
  // 평판 위 길이
  const raw = path.map((q) => surfaceAt(kin.points, q[0], q[1]));
  let length = 0;
  for (let k = 1; k < raw.length; k++) length += Math.hypot(raw[k][0] - raw[k - 1][0], raw[k][1] - raw[k - 1][1], raw[k][2] - raw[k - 1][2]);
  if (length < (opt.minLength ?? 0) || path.length < 2) return null;
  const resampled = resamplePath(path, raw, opt.maxPoints ?? 40, opt.spacingMax ?? 100);
  const pts = resampled.map((q) => surfaceAt(kin.points, q[0], q[1]));
  const a = pts[0], b = pts[pts.length - 1];
  return { path: resampled, points: pts, length, ends: [bw.reason, fw.reason], kink: !!(bw.kink || fw.kink), f, deg: Math.round(Math.atan2(b[1] - a[1], b[0] - a[0]) / DEG) };
}
function resamplePath(path, xyz, maxPoints, spacingMax) {
  let len = 0;
  const s = [0];
  for (let k = 1; k < xyz.length; k++) { len += Math.hypot(xyz[k][0] - xyz[k - 1][0], xyz[k][1] - xyz[k - 1][1]); s.push(len); }
  const n = clamp(Math.ceil(len / spacingMax) + 1, 2, maxPoints);
  const out = [];
  let k = 0;
  for (let q = 0; q < n; q++) {
    const target = len * q / (n - 1);
    while (k < s.length - 2 && s[k + 1] < target) k++;
    const seg = s[k + 1] - s[k] || 1;
    const u = clamp((target - s[k]) / seg, 0, 1);
    out.push([path[k][0] + (path[k + 1][0] - path[k][0]) * u, path[k][1] + (path[k + 1][1] - path[k][1]) * u]);
  }
  return out;
}

/** 줄 하나의 각변형 (rad, 크기). seedLines 의 기본 thetaLine 으로 쓴다. */
function unitTheta(comp, t) {
  if (!comp) return 0;
  if (comp.theta_mrad !== undefined) return Math.abs(comp.theta_mrad) / 1000;
  if (comp.B_t !== undefined && t > 0) return Math.abs(comp.B_t) * 12 / (t * t * t);
  return 0;
}

/**
 * 시작점을 정한다 (명세 S5 [재구성]).
 * 판 가운데를 지나 가열선에 수직으로 기준선을 긋고, 기준선을 pitch 간격 구간으로 나눈 뒤
 * 구간의 필요량 Θ = ∫(2|ε|/t) ds 가 thetaLine 을 넘는 동안 이등분한다. 시작점은 구간의 가운데다.
 * 기준선이 닿지 않는 곳은 tracedCandidates 의 빈틈 메우기로 채운다.
 * @param opt { fields, pitch, thetaLine, seedStep, seedPhase, minSeedGap, ds, demandStop }
 * @returns [{ i, j, f, s, density }]
 */
export function seedLines(kin, opt = {}) {
  const fields = opt.fields ?? [0, 1];
  const pitch = opt.pitch ?? 150;
  const minGap = opt.minSeedGap ?? pitch / 4;
  const ds = opt.ds ?? Math.min(kin.spacing.dr, kin.spacing.dc) / 2;
  const t = kin.thickness;
  const seeds = [];
  for (const f of fields) {
    const other = kin.fields.length > 1 ? 1 - f : f;
    const center = [(kin.nr - 1) / 2, (kin.nc - 1) / 2];
    if (!inside(center, kin.bounds)) continue;
    // 기준선: 방향장 f 의 가열선에 수직 (= 다른 방향장을 따라감. 필요량이 작아도 끝까지 간다)
    const refOpt = { ds, useDemand: false, kinkDeg: 45, anisStop: 0, minLength: 0, maxPoints: 2000, spacingMax: ds };
    let ref = null;
    if (kin.fields.length > 1) ref = traceLine(kin, { i: center[0], j: center[1], f: other }, refOpt);
    if (!ref) continue;
    // 기준선 위 위치 s 와 필요량 밀도
    const P = ref.path, Xr = ref.points;
    const s = [0];
    for (let k = 1; k < Xr.length; k++) s.push(s[k - 1] + Math.hypot(Xr[k][0] - Xr[k - 1][0], Xr[k][1] - Xr[k - 1][1]));
    const dens = P.map((q) => sampleField(kin.fields[f], q[0], q[1]).mag * (t > 0 ? 2 / t : 1));
    const Lr = s[s.length - 1];
    const cum = [0];
    for (let k = 1; k < P.length; k++) cum.push(cum[k - 1] + 0.5 * (dens[k] + dens[k - 1]) * (s[k] - s[k - 1]));
    const theta = (a, b) => interp(s, cum, b) - interp(s, cum, a);
    const baseStep = opt.seedStep ?? pitch / 2; // 기본은 pitch 의 절반 간격. 탐욕 선택이 고를 위치가 고정 직선 족과 비슷해지게 한다
    const n0 = Math.max(1, Math.floor(Lr / baseStep + 1e-9));
    const stop = (opt.thetaLine ?? 0);
    const mids = [];
    const split = (a, b) => {
      const th = theta(a, b);
      if (stop > 0 && th > stop && (b - a) / 2 >= minGap) { const m = 0.5 * (a + b); split(a, m); split(m, b); return; }
      mids.push(0.5 * (a + b));
    };
    const phase = opt.seedPhase ?? 0; // 시작점 격자를 간격의 일부만큼 밀어 본다 (시작점 위치 민감도 시험용)
    for (let q = 0; q < n0; q++) { const a = Lr * (q + phase) / n0, b = Lr * (q + 1 + phase) / n0; if (b <= Lr + 1e-9) split(a, b); }
    const magStop = (opt.demandStop ?? 0.05) * kin.magRef[f];
    for (const m of mids) {
      const k = indexAt(s, m);
      const u = (m - s[k]) / ((s[k + 1] - s[k]) || 1);
      const q = [P[k][0] + (P[k + 1][0] - P[k][0]) * u, P[k][1] + (P[k + 1][1] - P[k][1]) * u];
      const seedMag = sampleField(kin.fields[f], q[0], q[1]).mag;
      if (seedMag < magStop) continue;
      seeds.push({ i: q[0], j: q[1], f, s: m, density: seedMag * (t > 0 ? 2 / t : 1) }); // density: 위치의 필요량 밀도 (rad/mm 근사)
    }
  }
  return seeds;
}
function indexAt(s, v) { let k = 0; while (k < s.length - 2 && s[k + 1] < v) k++; return k; }
function interp(s, y, v) { const k = indexAt(s, v); const u = clamp((v - s[k]) / ((s[k + 1] - s[k]) || 1), 0, 1); return y[k] + (y[k + 1] - y[k]) * u; }

/**
 * 추적선 후보를 만든다. 기준선 시작점(seedLines)으로 추적하고, 필요량이 있는데 줄이 닿지 않은 곳(빈틈)을 시작점으로 더한다 [제안].
 * @param kin  prepareKinematics 결과
 * @param comp 윗면 기준 1줄 성분 (thetaLine 기본값에 쓴다)
 * @param opt  { pitch, thetaLine, fields, maxTraced, minLength, fill, ds, demandStop, ... }
 * @returns [{ family:'traced', deg, f, points, path, length, ends, kink, src }]
 */
export function tracedCandidates(kin, comp, opt = {}) {
  const pitch = opt.pitch ?? 150;
  const t = kin.thickness;
  const thetaLine = opt.thetaLine ?? unitTheta(comp, t);
  const fields = opt.fields ?? [0, 1];
  const span = Math.min((kin.nr - 1) * kin.spacing.dr, (kin.nc - 1) * kin.spacing.dc);
  const minLen = opt.minLength ?? Math.max(300, 0.2 * span);
  const traceOpt = { ds: opt.ds, kinkDeg: opt.kinkDeg, demandStop: opt.demandStop, anisStop: opt.anisStop, minLength: minLen, maxPoints: opt.maxPoints, spacingMax: opt.spacingMax };
  const maxTraced = opt.maxTraced ?? 80;
  const seeds = seedLines(kin, { fields, pitch, thetaLine, seedStep: opt.seedStep, seedPhase: opt.seedPhase, minSeedGap: opt.minSeedGap, ds: opt.ds, demandStop: opt.demandStop });
  const out = [];
  const strongest = Math.max(...kin.magRef);
  for (const f of fields) {
    const F = kin.fields[f];
    if (!(kin.magRef[f] > 0) || kin.magRef[f] < 1e-3 * strongest) continue; // 만들 변형이 거의 없는 방향은 뺀다 (실린더 등의 작은 쪽)
    const lines = [];
    const dist = new Float64Array(kin.nr * kin.nc).fill(Infinity);
    const P = kin.points;
    const addLine = (ln) => {
      if (!ln) return false;
      for (const other of lines) if (meanGap(ln.points, other.points) < 0.25 * pitch) return false; // 거의 같은 선은 뺀다
      lines.push(ln);
      for (let i = 0; i < kin.nr; i++) for (let j = 0; j < kin.nc; j++) {
        const p = i * kin.nc + j;
        for (const q of ln.points) { const d = Math.hypot(P[i][j][0] - q[0], P[i][j][1] - q[1]); if (d < dist[p]) dist[p] = d; }
      }
      return true;
    };
    for (const sd of seeds) { if (sd.f === f && lines.length < maxTraced) addLine(traceLine(kin, sd, traceOpt)); }
    // 빈틈 메우기: 필요량이 하한 위인 절점 중 줄에서 pitch 이상 떨어진 곳에서 시작한다
    if (opt.fill !== false) {
      const magStop = (opt.demandStop ?? 0.05) * kin.magRef[f];
      const B = kin.bounds;
      for (let round = 0; round < maxTraced && lines.length < maxTraced; round++) {
        let best = -1, bestMag = 0;
        for (let i = Math.ceil(B.i0); i <= B.i1; i++) for (let j = Math.ceil(B.j0); j <= B.j1; j++) {
          const p = i * kin.nc + j;
          if (dist[p] < pitch || F.mag[p] < magStop) continue;
          if (F.mag[p] > bestMag) { bestMag = F.mag[p]; best = p; }
        }
        if (best < 0) break;
        const i = Math.floor(best / kin.nc), j = best % kin.nc;
        if (!addLine(traceLine(kin, { i, j, f }, traceOpt))) dist[best] = 0; // 이 점에서는 선이 안 만들어지니 다시 고르지 않는다
      }
    }
    for (const ln of lines) {
      const srcCount = [0, 0];
      for (const q of ln.path) srcCount[sampleField(F, q[0], q[1]).src]++;
      out.push({ family: 'traced', deg: ln.deg, f, points: ln.points, path: ln.path, length: ln.length, ends: ln.ends, kink: ln.kink, src: srcCount[0] >= srcCount[1] ? 'membrane' : 'bending' });
    }
  }
  return out;
}
/** 두 선의 평균 거리 (a 의 점마다 b 까지 가장 가까운 거리의 평균, xy). */
function meanGap(a, b) {
  let sum = 0;
  for (const p of a) {
    let best = Infinity;
    for (const q of b) { const d = Math.hypot(p[0] - q[0], p[1] - q[1]); if (d < best) best = d; }
    sum += best;
  }
  return sum / a.length;
}

// ── 면 규칙 (N1) ──────────────────────────────────────────
/** 선의 수직 방향 곡률 변화량의 길이 평균 (부호 있음, 1/mm). κ<0 은 윗면 가열 쪽(끝이 올라감). */
function meanNormalCurvature(kin, ln) {
  const { nr, nc, S } = kin;
  let sum = 0;
  const P = ln.path;
  for (let k = 0; k < P.length; k++) {
    const q = P[k];
    const a = k < P.length - 1 ? P[k + 1] : q, b = k > 0 ? P[k - 1] : q;
    const a3 = surfaceAt(kin.points, a[0], a[1]), b3 = surfaceAt(kin.points, b[0], b[1]);
    let dx = a3[0] - b3[0], dy = a3[1] - b3[1];
    const l = Math.hypot(dx, dy) || 1; dx /= l; dy /= l;
    const ip = clamp(Math.round(q[0]), 0, nr - 1) * nc + clamp(Math.round(q[1]), 0, nc - 1);
    const nx = -dy, ny = dx;
    sum += nx * nx * S.kappa.xx[ip] + 2 * nx * ny * S.kappa.xy[ip] + ny * ny * S.kappa.yy[ip];
  }
  return sum / P.length;
}

/**
 * 스트립 표에서 각변형이 가장 큰 입열 파라미터 Q/h² 를 찾는다 (N1 의 문턱). 표는 호출하는 쪽에서 준다. 저장하지 않는다.
 * @param rows [{ qh2, theta }] 또는 [{ q, h, theta }]
 * @returns { crit, thetaMax, index } crit 은 포물선 보간한 Q/h²
 */
export function n1Threshold(rows) {
  const r = rows.map((x) => ({ qh2: x.qh2 ?? x.q / (x.h * x.h), theta: Math.abs(x.theta) })).filter((x) => Number.isFinite(x.qh2) && Number.isFinite(x.theta)).sort((a, b) => a.qh2 - b.qh2);
  if (r.length < 3) throw new Error('n1Threshold: 표에 조건이 세 개 이상 있어야 합니다.');
  let m = 0;
  for (let k = 1; k < r.length; k++) if (r[k].theta > r[m].theta) m = k;
  if (m === 0 || m === r.length - 1) return { crit: r[m].qh2, thetaMax: r[m].theta, index: m, edge: true };
  // 세 점을 지나는 포물선의 꼭짓점
  const [a, b, c] = [r[m - 1], r[m], r[m + 1]];
  const d1 = (b.theta - a.theta) / (b.qh2 - a.qh2), d2 = (c.theta - b.theta) / (c.qh2 - b.qh2);
  const d3 = (d2 - d1) / (c.qh2 - a.qh2);
  const crit = d3 !== 0 ? 0.5 * (a.qh2 + b.qh2) - d1 / (2 * d3) : b.qh2;
  return { crit: clamp(crit, a.qh2, c.qh2), thetaMax: b.theta, index: m, edge: false };
}
/** Q/h² 가 문턱 아래면 굽힘 선 'bending', 위면 수축 선 'membrane' (N1). */
export function n1Regime(qh2, crit) { return qh2 < crit ? 'bending' : 'membrane'; }

// ── 후보 응답 ─────────────────────────────────────────────
/** 윗면 기준 성분에서 면에 맞는 성분을 만든다. 각변형 항만 부호가 뒤집힌다 (solver_ext.js 머리말의 규약). */
export function compForSide(comp, side) {
  if (side !== 'bottom') return comp;
  const c = { ...comp };
  for (const k of ['theta_mrad', 'B_t', 'theta_l_mrad', 'B_l']) if (c[k] !== undefined) c[k] = -c[k];
  return c;
}

function polyLength(points) {
  let L = 0;
  for (let k = 1; k < points.length; k++) L += Math.hypot(points[k][0] - points[k - 1][0], points[k][1] - points[k - 1][1], points[k][2] - points[k - 1][2]);
  return L;
}

// ── 간격 판정 ─────────────────────────────────────────────
/** planner.js 의 lineGap 과 같은 계산. gapScope:'all' 에서 원본과 결과를 맞출 때 쓴다. */
function lineGapPlanner(a, b) {
  let sum = 0;
  for (const p of a) {
    let best = Infinity;
    for (const q of b) best = Math.min(best, Math.hypot(p[0] - q[0], p[1] - q[1]));
    sum = Math.min(best, sum === 0 ? best : sum);
  }
  return sum;
}
/** 같은 면에서 평행에 가깝고 minGap 보다 가까우면 true. 90° 가깝게 만나는 선은 허용한다 (forming.js 가 면·방향 묶음별로 고르는 것과 같은 뜻). */
function tooClose(a, b, minGap) {
  if (a.side !== b.side) return false;
  let best = Infinity, ia = 0, ib = 0;
  const P = a.points, Q = b.points;
  for (let i = 0; i < P.length; i++) for (let j = 0; j < Q.length; j++) {
    const d = Math.hypot(P[i][0] - Q[j][0], P[i][1] - Q[j][1]);
    if (d < best) { best = d; ia = i; ib = j; }
  }
  if (best >= minGap - 1e-6) return false;
  const dirOf = (poly, k) => {
    const u = poly[Math.max(0, k - 1)], v = poly[Math.min(poly.length - 1, k + 1)];
    const dx = v[0] - u[0], dy = v[1] - u[1];
    const l = Math.hypot(dx, dy) || 1;
    return [dx / l, dy / l];
  };
  const da = dirOf(P, ia), db = dirOf(Q, ib);
  return Math.abs(da[0] * db[0] + da[1] * db[1]) > Math.cos(45 * DEG);
}

// ── 곡 펴짐 지표 ──────────────────────────────────────────
/**
 * 곡 펴짐 지표 (박정서 외 2008): 줄을 더할 때마다 주 굽힘 변형도의 제곱합을 방향별로 기록한다.
 * 방향: 주곡률 방향이 격자 x 쪽에 가까우면 0(종), y 쪽에 가까우면 1(횡).
 * 절대값보다 회차별 추세를 본다. 합이 줄어드는 회차는 앞서 만든 곡이 펴지는 것으로 읽는다.
 * @param points 시작 형상, lines [{ weight, zResp }] 줄 순서대로 (zResp 는 평면 제거한 z 응답, 격자 순서)
 * @param thickness 판 두께 mm (없으면 κ 그대로 합산)
 * @param opt.flat 평판 좌표 (없으면 points 의 x,y)
 * @returns { series:[{n, sum:[S0,S1]}], events:[{n, dir, drop}], final:[S0,S1] }
 */
export function curlSeries(points, lines, thickness, opt = {}) {
  const nr = points.length, nc = points[0].length, N = nr * nc;
  const G0 = toGridObj(points);
  const flat = opt.flat ?? (() => { const X = Float64Array.from(G0.X); for (let p = 0; p < N; p++) X[3 * p + 2] = 0; return { X, nr, nc }; })();
  const tt = thickness > 0 ? thickness / 2 : 1;
  const disp = new Float64Array(N);
  const series = [{ n: 0, sum: [0, 0] }];
  const X = Float64Array.from(G0.X);
  const useOpt = { use: 'bending', window: opt.window ?? 1 };
  for (let n = 0; n < lines.length; n++) {
    const w = lines[n].weight, z = lines[n].zResp;
    for (let p = 0; p < N; p++) { disp[p] += w * z[p]; X[3 * p + 2] = G0.X[3 * p + 2] + disp[p]; }
    const S = strains(G0, { X, nr, nc }, flat, useOpt);
    const b = S.principal.b;
    const sum = [0, 0];
    for (let p = 0; p < N; p++) {
      const k1 = b.k1[p] * tt, k2 = b.k2[p] * tt, phi = b.phi1[p];
      const cls = Math.abs(Math.cos(phi)) >= Math.abs(Math.sin(phi)) ? 0 : 1;
      sum[cls] += k1 * k1;
      sum[1 - cls] += k2 * k2;
    }
    series.push({ n: n + 1, sum });
  }
  const events = [];
  for (let n = 2; n < series.length; n++) for (let d = 0; d < 2; d++) {
    const a = series[n - 1].sum[d], b = series[n].sum[d];
    if (b < a * (1 - 1e-3)) events.push({ n, dir: d, drop: (a - b) / (a || 1) });
  }
  return { series, events, final: series[series.length - 1].sum, note: '방향 0=격자 x(행) 쪽, 1=y(열) 쪽. 해석값이며 회차별 추세만 본다.' };
}

// ── 추천 ──────────────────────────────────────────────────
/**
 * 가열선 추천. recommend()(planner.js) 와 같은 인자와 탐욕 선택을 쓰고, 추적선 후보·면·정지 규칙을 더한다.
 *
 * @param model  factorize 된 PlateModel (solver.js 또는 solver_ext.js)
 * @param points 시작 형상 [row][col]=[x,y,z]
 * @param D      목표 z 편차 (zPlaneFree 결과, 시작 → 목표)
 * @param comp   윗면 가열 1줄 성분 (θ<0). 아랫면은 compForSide 로 뒤집는다
 * @param opt.target    목표 곡면 격자 (없으면 points + D ẑ)
 * @param opt.sides     ['top'](기본) | ['top','bottom']
 * @param opt.fixed     false 면 고정 직선을 뺀다 (기본: 넣음)
 * @param opt.traced    false 면 추적선을 뺀다 (기본: 넣음)
 * @param opt.extraLines 고정 직선 후보에 더할 선 [{ family, deg, points }] (시험용: 길이방향 조밀 후보 등)
 * @param opt.rule      'D06'(기본) | 'N1' | 'bending' | 'membrane'
 * @param opt.n1        rule='N1' 일 때 { qh2, crit } 또는 { regime:'bending'|'membrane' }
 * @param opt.sideRule  'both'(기본, 두 면 다 후보) | 'curvature'(N1, 그 줄의 곡률 부호로 면을 정한다). rule='N1' 이면 기본 'curvature'
 * @param opt.gapScope  'group'(기본) 같은 면에서 가깝고 평행한 줄만 금지 | 'all' planner.js 와 같은 전체 금지 (같은 후보·같은 결과 확인용)
 * @param opt.stop      { alpha } 또는 false. 기본 { alpha:0.2 }. J = RMS/처음 RMS + α·가중 가열량/전체 계획 가열량 이 가장 작은 곳까지만 쓴다
 * @param opt.tolerance 허용 RMS(mm). 주면 그 안에 든 첫 줄에서 끝낸다 (명세 S9)
 * @param opt.poolBest  true 면 후보 묶음(전체·고정만·추적만)으로 각각 탐욕을 돌려 가장 나은 결과를 쓴다. 기본 false (명세의 혼합안 그대로)
 * @param opt.window    곡률 창. 측정 곡면이면 'auto'
 * @param opt.thickness 판 두께 (없으면 model.t)
 * @param opt.maxLines, minGap, wmax, minGain, pitch, angles, margin, families  planner.js 와 같은 뜻
 * @returns { lines, rmsBefore, rmsAfter, candidates, counts, predicted, log, stop, lengthTotal, lengthWeighted, classify, kin, curl, ... }
 */
export function recommendShin(model, points, D, comp, opt = {}, onProgress) {
  const X = flatten(points);
  const maxLines = opt.maxLines ?? 10;
  const minGap = opt.minGap ?? 150;
  const wmax = opt.wmax ?? 20;
  const minGain = opt.minGain ?? 0.002;
  const sides = opt.sides ?? ['top'];
  const t = opt.thickness ?? model.t;
  const rule = opt.rule ?? 'D06';
  const fixedLines = opt.fixed === false ? [] : [...makeCandidates(points, opt), ...(opt.extraLines ?? [])];

  // 방향 규칙 정리
  let fieldRule = 'auto', ruleLabel = rule, regime = null;
  if (rule === 'D06') fieldRule = 'auto';
  else if (rule === 'bending' || rule === 'membrane') fieldRule = rule;
  else if (rule === 'N1') {
    if (!opt.n1) throw new Error("rule='N1' 은 opt.n1 ({ qh2, crit } 또는 { regime }) 이 필요합니다.");
    regime = opt.n1.regime ?? n1Regime(opt.n1.qh2, opt.n1.crit);
    fieldRule = regime;
    ruleLabel = `N1(${regime === 'bending' ? '문턱 아래·굽힘 선' : '문턱 위·수축 선'})`;
  } else throw new Error(`알 수 없는 규칙: ${rule}`);
  const sideRule = opt.sideRule ?? (rule === 'N1' ? 'curvature' : 'both');

  // 추적선
  let kin = null, tracedLines = [], classify = null;
  if (opt.traced !== false) {
    const target = opt.target ?? targetFromD(points, D);
    kin = prepareKinematics(points, target, { ...opt, thickness: t, rule: fieldRule });
    tracedLines = tracedCandidates(kin, comp, opt);
    try { const c = classifyPlate(target, { window: kin.window > 1 ? kin.window : undefined }); classify = { y: c.y, z: c.z, tasks: c.tasks, shapeName: c.shapeName }; } catch { classify = null; }
  }

  // 면 붙이기
  const cands = [];
  for (const ln of fixedLines) for (const side of sides) cands.push({ ...ln, side });
  for (const ln of tracedLines) {
    if (sideRule === 'curvature') {
      const side = meanNormalCurvature(kin, ln) < 0 ? 'top' : 'bottom';
      if (sides.includes(side)) cands.push({ ...ln, side });
    } else for (const side of sides) cands.push({ ...ln, side });
  }
  const comps = { top: comp, bottom: compForSide(comp, 'bottom') };

  // 후보 응답 (recommend 와 같은 계산)
  const R = new Array(cands.length);
  for (let c = 0; c < cands.length; c++) {
    const u = model.solve(model.lineLoad(cands[c].points, comps[cands[c].side]));
    R[c] = zPlaneFree(model.topDisplacement(u), X);
    if (onProgress && c % 8 === 0) onProgress(0.8 * c / cands.length);
  }

  // 탐욕 선택 (recommend 의 절차를 그대로 따른다. 간격 판정만 gapScope 로 바꿀 수 있다)
  const total = dot(D, D) || 1;
  const conflict = (c, p) => (opt.gapScope === 'all' ? lineGapPlanner(cands[c].points, cands[p].points) < minGap : tooClose(cands[c], cands[p], minGap));
  const norms = R.map((r) => dot(r, r) || 1);
  const runGreedy = (pool, done) => {
    const picked = [];
    const steps = []; // 줄을 더한 뒤의 상태
    let residual = Float64Array.from(D);
    for (let k = 0; k < maxLines; k++) {
      let best = -1, bestGain = 0;
      for (const c of pool) {
        if (picked.includes(c)) continue;
        const a = dot(R[c], residual);
        if (a <= 0) continue;
        const gain = a * a / norms[c];
        if (gain <= bestGain) continue;
        if (picked.some((p) => conflict(c, p))) continue;
        best = c; bestGain = gain;
      }
      if (best < 0 || bestGain / total < minGain) break;
      picked.push(best);
      const fit = boundedFit(picked.map((p) => R[p]), D, wmax);
      residual = fit.residual;
      steps.push({ c: best, rms: rms(fit.residual), w: Float64Array.from(fit.w), res: fit.residual });
      if (onProgress) onProgress(done + (1 - done) * (k + 1) / maxLines);
    }
    return { picked, steps };
  };

  // 정지 규칙: J = (RMS / 처음 RMS) + α × (가중 가열량 / 전체 계획의 가중 가열량) 이 가장 작은 곳까지만 쓴다.
  // 줄을 더해서 RMS 가 처음 RMS 의 몇 % 만큼 줄어드는지와, 그 줄이 전체 계획 가열량의 몇 % 를 더하는지를 비교한다.
  // α = 0.2 는 "처음 RMS 의 1% 를 줄이는 데 전체 계획 가열량의 5% 까지 쓴다" 는 뜻이다 [제안].
  // 허용 오차 opt.tolerance(mm)를 주면 RMS 가 그 안에 든 첫 줄에서 끝낸다.
  const stopOpt = opt.stop === false ? null : { alpha: opt.stop?.alpha ?? 0.2 };
  const tolerance = opt.tolerance ?? 0;
  const rms0 = rms(D);
  const evalRun = (run) => {
    const lens = run.picked.map((c) => polyLength(cands[c].points));
    const cost = run.steps.map((st) => { let T = 0; st.w.forEach((w, i) => { T += w * lens[i]; }); return T; });
    const TN = cost.length ? cost[cost.length - 1] : 0;
    let kStar = run.steps.length; // 정지 규칙이 없으면 끝까지
    if (stopOpt && TN > 0 && rms0 > 0) {
      let bestJ = Infinity;
      for (let k = 0; k < run.steps.length; k++) {
        const J = run.steps[k].rms / rms0 + stopOpt.alpha * cost[k] / TN;
        if (J < bestJ - 1e-12) { bestJ = J; kStar = k + 1; }
      }
    }
    if (tolerance > 0) {
      const hit = run.steps.findIndex((st) => st.rms <= tolerance);
      if (hit >= 0 && hit + 1 < kStar) kStar = hit + 1;
    }
    const used = kStar > 0 ? run.steps[kStar - 1] : null;
    return { ...run, lens, cost, TN, kStar, used, rmsKept: used ? used.rms : rms0, costKept: kStar > 0 ? cost[kStar - 1] : 0 };
  };

  // 후보 묶음: 기본은 전체(고정 + 추적) 한 번. opt.poolBest 이면 고정만·추적만도 돌려 가장 나은 쪽을 쓴다.
  const allIdx = cands.map((_, i) => i);
  const fixedIdx = allIdx.filter((i) => cands[i].family !== 'traced');
  const tracedIdx = allIdx.filter((i) => cands[i].family === 'traced');
  const pools = [{ name: '혼합', idx: allIdx }];
  if (opt.poolBest && fixedIdx.length && tracedIdx.length) { pools.push({ name: '고정만', idx: fixedIdx }); pools.push({ name: '추적만', idx: tracedIdx }); }
  const runs = pools.map((p, n) => ({ pool: p.name, ...evalRun(runGreedy(p.idx, 0.8 + 0.2 * n / pools.length)) }));
  let chosen = runs[0];
  for (const r of runs.slice(1)) {
    if (r.rmsKept < chosen.rmsKept - 1e-9 || (Math.abs(r.rmsKept - chosen.rmsKept) <= 1e-9 && r.costKept < chosen.costKept)) chosen = r;
  }
  const { picked, steps, lens, cost, TN, kStar, used: usedSteps, pool } = chosen;
  const log = steps.map((st, k) => {
    const prevRms = k > 0 ? steps[k - 1].rms : rms0;
    const prevT = k > 0 ? cost[k - 1] : 0;
    const ln = cands[st.c];
    return {
      k: k + 1, c: st.c, family: ln.family, deg: ln.deg, side: ln.side, f: ln.f, length: lens[k], weightAtStep: st.w[k], rms: st.rms, rmsFrac: st.rms / (rms0 || 1),
      gainFrac: (prevRms - st.rms) / (rms0 || 1), cost: cost[k], costFrac: TN > 0 ? (cost[k] - prevT) / TN : 0, J: stopOpt ? st.rms / (rms0 || 1) + stopOpt.alpha * (TN > 0 ? cost[k] / TN : 0) : null, kept: k < kStar,
    };
  });
  const lines = [];
  const lineResp = [];
  if (usedSteps) {
    picked.slice(0, kStar).forEach((c, n) => {
      const w = usedSteps.w[n];
      if (w <= 1e-6) return;
      lines.push({ ...stripTrace(cands[c]), weight: w, length: lens[n], thetaLine: unitTheta(comp, t) * w, candidate: c });
      lineResp.push({ weight: w, zResp: R[c], side: cands[c].side });
    });
  }
  // 작업 순서: 면별로 묶는다 (같은 면을 먼저 끝내고 뒤집는다. 순서를 어떻게 정할지는 정해진 답이 없다 — 명세 S9). 곡 펴짐은 이 순서로 계산한다
  const orderIdx = lines.map((_, i) => i).sort((a, b) => (lines[a].side === lines[b].side ? a - b : (lines[a].side === 'top' ? -1 : 1)));
  orderIdx.forEach((idx, order) => { lines[idx].order = order; });
  const curl = lines.length && (opt.curl ?? true) ? curlSeries(points, orderIdx.map((i) => lineResp[i]), t) : null;

  const finalRes = usedSteps ? usedSteps.res : Float64Array.from(D);
  const lengthTotal = lines.reduce((s, ln) => s + ln.length, 0);
  const lengthWeighted = lines.reduce((s, ln) => s + ln.weight * ln.length, 0);
  const fullRms = steps.length ? steps[steps.length - 1].rms : rms0;
  const stop = {
    rule: stopOpt ? 'J = RMS/처음 RMS + α·가중 가열량/전체 계획의 가중 가열량 (가장 작을 때까지만 쓴다)' : '쓰지 않음', alpha: stopOpt?.alpha ?? null, tolerance: tolerance || null,
    pickedAll: steps.length, kept: kStar, rmsAll: fullRms, rmsKept: rms(finalRes), lengthWeightedAll: TN, lengthWeightedKept: cost[kStar - 1] ?? 0,
    reason: steps.length === 0 ? '후보 없음' : (kStar < steps.length ? '이득이 시간 증가보다 작아 끊음' : (steps.length >= maxLines ? '최대 줄 수에 닿음' : '이득이 minGain 미만')),
  };
  const poolInfo = runs.map((r) => ({ pool: r.pool, picked: r.picked.length, kept: r.kStar, rmsKept: r.rmsKept, lengthWeightedKept: r.costKept }));
  return {
    lines, rmsBefore: rms0, rmsAfter: rms(finalRes), candidates: cands.length,
    counts: { fixed: fixedLines.length * sides.length, tracedLines: tracedLines.length, tracedCands: cands.length - fixedLines.length * sides.length },
    predicted: finalRes, log, stop, lengthTotal, lengthWeighted, pool, poolInfo, rule: { name: ruleLabel, fieldRule, regime, sideRule },
    classify, curl,
    kin: kin ? { rule: kin.rule, window: kin.window, noise: kin.noise, tensionShift: kin.tensionShift, dirField: kin.S?.dirField, principal: kin.S?.principal, eps_m: kin.S?.eps_m, kappa: kin.S?.kappa, fields: kin.fields } : null,
  };
}
/** 시작 형상 + z 편차로 목표 곡면을 만든다. */
function targetFromD(points, D) {
  const nr = points.length, nc = points[0].length;
  return points.map((row, i) => row.map((q, j) => [q[0], q[1], q[2] + D[i * nc + j]]));
}
function stripTrace(c) {
  const { path, ...rest } = c;
  return rest;
}

/**
 * 명세 §4 의 이름. input = { points, D 또는 target, comp, ...옵션 }
 * target 을 주면 D 는 zPlaneFree(target - points) 로 계산한다.
 */
export function planKinematic(model, input, onProgress) {
  const { points, comp, ...opt } = input;
  let D = input.D;
  if (!D) {
    if (!input.target) throw new Error('planKinematic: D 또는 target 이 필요합니다.');
    const X = flatten(points), T = flatten(input.target);
    const diff = new Float64Array(X.length);
    for (let i = 0; i < X.length; i++) diff[i] = T[i] - X[i];
    D = zPlaneFree(diff, X);
  }
  return recommendShin(model, points, D, comp, { target: input.target, ...opt }, onProgress);
}

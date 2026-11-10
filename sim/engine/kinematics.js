// 격자 곡면의 기하 계산 — 이산 곡률, 곡판 분류, 막 변형률, 곡률 변화량, 주방향
// 브라우저(Web Worker 포함)와 Node 양쪽에서 그대로 동작하는 순수 ES 모듈이다. 외부 의존성 없음.
//
// 입력 격자는 [row][col] = [x,y,z] (mm) 이고, solver.js / forming.js 와 같은 형식이다.
// 좌표 순서를 바꾸지 않으며, 판 데이터를 저장하거나 밖으로 보내지 않는다.
//
// 근거 (자세한 내용은 맥시뮬레이터/shin/spec_heating_line_algo.md)
//   이산 곡률 식   : 김찬석 외 2014 (Meyer 2002 인용), 명세 S1
//   분류·작업 배정 : 노재규·신종계 2009, 김찬석 외 2014, 명세 S1
//   막 변형률·곡률 : 유철호·신종계 2002, 명세 S3 / 주방향·지배 성분: 김용균·신종계 2002, 명세 S4
//
// 이 모듈의 값은 격자에서 계산한 기하량이다. 측정값이 아니며, 실제 곡판의 가공 정확도를 뜻하지 않는다.

const TWO_PI = 2 * Math.PI;

/** 분류 번호. y 벡터의 순서와 같다: (안장, 볼록, 실린더, 평판) */
export const CLASS = Object.freeze({ SADDLE: 0, CONVEX: 1, CYLINDER: 2, FLAT: 3 });
const SHAPE_NAME = ['안장', '볼록', '실린더', '평판'];
const INVALID = 255;

/*
 * 판정 여유 (전부 [제안]: 논문에는 값이 없고, 5% 잡음 문턱만 논문값이다. 측정 기록은 test_kinematics_result.json)
 *
 * EPS_K = 0.01 : |K| < EPS_K·m² 이면 K=0 (m = 큰 주곡률의 크기). 작은 주곡률이 큰 주곡률의 1% 미만이라는 뜻이다.
 *   - 위: 격자에서 돌린 원통의 이산 오차 |K|/m² 는 (h/R)=0.02 에서 안쪽 최대 1.0e-4, 경계 최대 5.5e-4 였다. 격자에 맞춘 원통은 0.
 *     오차는 (h/R)² 에 비례하므로 여유 0.01 까지 경계 기준 h/R≈0.08, 안쪽 기준 h/R≈0.2 의 여유가 있다.
 *   - 아래: 1% 는 곡률 반경으로 100배 차이다. 예를 들어 반경 5 m 곡면에 반경 500 m 의 곡이 겹친 정도는 실린더로 본다.
 * EPS_H = 0.01 : 큰 주곡률이 판의 기준 곡률(m_ref)의 1% 이하이면 평판으로 본다.
 *   S자처럼 곡률 부호가 바뀌는 선 근처 띠의 넓이는 (2/π)·asin(EPS_H) (값은 0.64%) 로, 5% 문턱보다 작아 평판 성분이 생기지 않는다.
 * NOISE_CUT = 0.05 : 논문값 (노재규·신종계 2009 표, y 성분 5% 미만은 잡음).
 * NOISE_SIGMAS = 3 : 곡률 잡음 표준편차의 3배 안에 드는 값은 0 과 구분할 수 없다고 본다 (정규 분포에서 약 0.3%).
 * MREF_Q = 0.99 : 판의 기준 곡률은 안쪽 절점 큰 주곡률의 99 분위수. 튀는 점 몇 개로 기준이 움직이지 않게 한다.
 * NUM_FLOOR = 1e-9 : 부동소수 한계. 판 대각선 D 에 대해 곡률 1e-9/D (처짐 약 1e-9·D/8 mm) 아래는 0 이다.
 * NOISE_RELIABLE = 0.1 : 잡음 비율 r = κ_noise/m_ref 가 이 값 이하이면 분류를 믿어도 좋다.
 *   합성 시험(구·돌린 원통·안장, R=5 m, h=50·25 mm, 창 1~4)에서 r≤0.1 일 때 창이 들어간 안쪽 절점의 정답률이 99.8% 이상이었고,
 *   0.1<r≤0.15 에서는 약 70%까지 떨어졌다 (test_kinematics_result.json '판정여유_잡음비율'). r 이 크면 window 를 키운다.
 */
const TOL = Object.freeze({ EPS_K: 0.01, EPS_H: 0.01, NOISE_CUT: 0.05, NOISE_SIGMAS: 3, MREF_Q: 0.99, NUM_FLOOR: 1e-9, NOISE_RELIABLE: 0.1 });
const { EPS_K, EPS_H, NOISE_CUT, NOISE_SIGMAS, NUM_FLOOR, NOISE_RELIABLE } = TOL;
/** 판정 여유 값 (읽기 전용, 근거는 파일 앞쪽 주석). 화면 표시와 시험에서 쓴다. */
export { TOL };

// ── 격자 입력 ──────────────────────────────────────────────
/** [row][col]=[x,y,z] 또는 [x,y], 또는 { X, nr, nc } 를 평탄 배열 격자로 바꾼다. */
function toGrid(points, name) {
  if (points && points.X && points.nr && points.nc) return { X: points.X, nr: points.nr, nc: points.nc };
  if (!Array.isArray(points) || !Array.isArray(points[0])) throw new Error(`${name}: 격자는 [행][열]=[x,y,z] 형식이어야 합니다.`);
  const nr = points.length, nc = points[0].length;
  if (nr < 3 || nc < 3) throw new Error(`${name}: 격자가 너무 작습니다 (3×3 이상 필요, 지금 ${nr}×${nc}).`);
  const X = new Float64Array(nr * nc * 3);
  for (let i = 0; i < nr; i++) {
    if (points[i].length !== nc) throw new Error(`${name}: 행마다 열 수가 달라 격자로 볼 수 없습니다 (${i}행).`);
    for (let j = 0; j < nc; j++) {
      const p = points[i][j], q = (i * nc + j) * 3;
      X[q] = p[0]; X[q + 1] = p[1]; X[q + 2] = p.length > 2 ? p[2] : 0;
    }
  }
  return { X, nr, nc };
}
function sameSize(a, b, what) {
  if (a.nr !== b.nr || a.nc !== b.nc) throw new Error(`${what}: 격자 크기가 다릅니다 (${a.nr}×${a.nc} / ${b.nr}×${b.nc}). 같은 재료점을 가리키는 격자여야 합니다.`);
}

// ── 1차원 미분 가중치 ──────────────────────────────────────
/** 작은 연립방정식을 푼다 (부분 피벗 가우스 소거). */
function solveDense(A, b) {
  const n = b.length;
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(A[r][c]) > Math.abs(A[p][c])) p = r;
    [A[c], A[p]] = [A[p], A[c]]; [b[c], b[p]] = [b[p], b[c]];
    for (let r = c + 1; r < n; r++) {
      const f = A[r][c] / A[c][c];
      for (let k = c; k < n; k++) A[r][k] -= f * A[c][k];
      b[r] -= f * b[c];
    }
  }
  for (let c = n - 1; c >= 0; c--) {
    for (let k = c + 1; k < n; k++) b[c] -= A[c][k] * b[k];
    b[c] /= A[c][c];
  }
  return b;
}
/** offsets 위치 값들로 degree 차 다항식을 최소제곱으로 맞출 때, 0 위치의 값·1계·2계 미분을 주는 가중치. */
function polyWeights(offsets, degree) {
  const m = offsets.length, p = degree + 1;
  const V = offsets.map((t) => { const row = [1]; for (let q = 1; q < p; q++) row.push(row[q - 1] * t); return row; });
  const out = [];
  for (const [d, fact] of [[0, 1], [1, 1], [2, 2]]) {
    const B = Array.from({ length: p }, (_, a) => Array.from({ length: p }, (_, b) => { let s = 0; for (let k = 0; k < m; k++) s += V[k][a] * V[k][b]; return s; }));
    const rhs = new Array(p).fill(0);
    if (d < p) rhs[d] = fact;
    const c = solveDense(B, rhs);
    out.push(Float64Array.from(V, (row) => row.reduce((s, v, q) => s + v * c[q], 0)));
  }
  return { s0: out[0], s1: out[1], s2: out[2] };
}
/**
 * 길이 n 격자의 각 위치에서 쓸 미분 가중치.
 * win=1 : 안쪽은 3점 중심 차분, 경계는 4점 한쪽 차분 (2차 정확도)
 * win≥2 : 2·win+1 점 2차 최소제곱 (Savitzky-Golay). 경계에서는 창을 안쪽으로 밀어 쓴다.
 */
function stencil1D(n, win) {
  const w = Math.max(1, Math.min(win, Math.floor((n - 1) / 2)));
  const lo = new Int32Array(n), st = new Array(n);
  const range = (a, b) => { const r = []; for (let k = a; k <= b; k++) r.push(k); return r; };
  const interior = polyWeights(range(-w, w), 2);
  const memo = new Map();
  for (let i = 0; i < n; i++) {
    if (i >= w && i < n - w) { lo[i] = i - w; st[i] = interior; continue; }
    let a, b, degree;
    if (w === 1) {
      const len = Math.min(n, 4);
      a = i < w ? 0 : n - len; b = a + len - 1; degree = len - 1;
    } else {
      // 경계 근처는 창을 안쪽으로 밀어 쓰므로 절점이 창 가운데에 오지 않는다.
      // 차수를 2로 두면 잡음에 강한 대신 곡률 편향이 커지고 (h/R=0.02, 창 2~4, 돌린 원통 |K|/m² 경계 최대 약 0.01~0.03, 시험 기록 '창경계편향'),
      // 3으로 올리면 편향은 줄지만(개발 중 측정 약 0.001~0.003) 잡음 증폭이 커져 r≈0.1 부근의 잡음에서 경계 절점 정답률이 크게 떨어졌다.
      // 그래서 2차로 두고, 분류의 비율 계산에서는 창이 판 안에 다 들어간 절점만 쓴다(trusted).
      const len = 2 * w + 1;
      a = i < w ? 0 : n - len; b = a + len - 1; degree = 2;
    }
    lo[i] = a;
    const key = `${a - i}|${b - i}|${degree}`;
    if (!memo.has(key)) memo.set(key, polyWeights(range(a - i, b - i), degree));
    st[i] = memo.get(key);
  }
  const g0 = Float64Array.from(st, (e) => Math.hypot(...e.s0)), g2 = Float64Array.from(st, (e) => Math.hypot(...e.s2));
  return { lo, st, w, g0, g2 };
}

/** 격자 지수(u=행, v=열)에 대한 1·2계 미분. win≥2 이면 평활 미분이다. */
function derivatives(G, win) {
  const { X, nr, nc } = G, N = nr * nc;
  const R = stencil1D(nr, win), C = stencil1D(nc, win);
  const Xu = new Float64Array(3 * N), Xv = new Float64Array(3 * N), Xuu = new Float64Array(3 * N), Xuv = new Float64Array(3 * N), Xvv = new Float64Array(3 * N);
  for (let i = 0; i < nr; i++) {
    const ri = R.st[i], r0 = R.lo[i], na = ri.s0.length;
    for (let j = 0; j < nc; j++) {
      const cj = C.st[j], c0 = C.lo[j], nb = cj.s0.length, o = (i * nc + j) * 3;
      for (let a = 0; a < na; a++) {
        const r00 = ri.s0[a], r1 = ri.s1[a], r2 = ri.s2[a];
        for (let b = 0; b < nb; b++) {
          const c00 = cj.s0[b], c1 = cj.s1[b], c2 = cj.s2[b], q = ((r0 + a) * nc + c0 + b) * 3;
          for (let k = 0; k < 3; k++) {
            const v = X[q + k];
            Xu[o + k] += r1 * c00 * v;
            Xv[o + k] += r00 * c1 * v;
            Xuu[o + k] += r2 * c00 * v;
            Xuv[o + k] += r1 * c1 * v;
            Xvv[o + k] += r00 * c2 * v;
          }
        }
      }
    }
  }
  return { Xu, Xv, Xuu, Xuv, Xvv, R, C };
}

/** 절점별 곡률 잡음 이득: σ(mm)에 곱하면 그 절점의 2계 미분 잡음 표준편차(1/mm)가 된다. 경계의 한쪽 차분은 안쪽보다 크다. */
function nodeNoiseGain(R, C, hu, hv, N) {
  const g = new Float64Array(N);
  const nr = R.st.length, nc = C.st.length;
  const iu = 1 / (hu * hu), iv = 1 / (hv * hv);
  for (let i = 0; i < nr; i++) for (let j = 0; j < nc; j++) g[i * nc + j] = Math.max(R.g2[i] * C.g0[j] * iu, R.g0[i] * C.g2[j] * iv);
  return g;
}

/** 단위 법선. 판 전체 평균 법선이 +z 쪽을 향하도록 부호를 하나로 맞춘다 (격자 순서에 따라 뒤집히지 않게). */
function unitNormals(Xu, Xv, N) {
  const n = new Float64Array(3 * N);
  let mz = 0;
  for (let p = 0; p < N; p++) {
    const q = 3 * p;
    const nx = Xu[q + 1] * Xv[q + 2] - Xu[q + 2] * Xv[q + 1];
    const ny = Xu[q + 2] * Xv[q] - Xu[q] * Xv[q + 2];
    const nz = Xu[q] * Xv[q + 1] - Xu[q + 1] * Xv[q];
    const len = Math.hypot(nx, ny, nz) || 1;
    n[q] = nx / len; n[q + 1] = ny / len; n[q + 2] = nz / len;
    if (Number.isFinite(nz)) mz += nz;
  }
  if (mz < 0) for (let k = 0; k < 3 * N; k++) n[k] = -n[k];
  return n;
}

// ── 코탄젠트 식 (김찬석 외 2014, Meyer 2002) ───────────────
/**
 * 사각형을 대각선으로 나눈 삼각망에서 각도 합, 혼합 볼로노이 면적, 코탄젠트 합 벡터를 모은다.
 * diag 'a': (i,j)-(i+1,j+1) 대각선, 'b': (i+1,j)-(i,j+1) 대각선
 */
function meshTerms(G, diag) {
  const { X, nr, nc } = G, N = nr * nc;
  const ang = new Float64Array(N), area = new Float64Array(N), lap = new Float64Array(3 * N);
  const tri = (a, b, c) => {
    const A = 3 * a, B = 3 * b, C = 3 * c;
    const abx = X[B] - X[A], aby = X[B + 1] - X[A + 1], abz = X[B + 2] - X[A + 2];
    const acx = X[C] - X[A], acy = X[C + 1] - X[A + 1], acz = X[C + 2] - X[A + 2];
    const bcx = X[C] - X[B], bcy = X[C + 1] - X[B + 1], bcz = X[C + 2] - X[B + 2];
    const cx = aby * acz - abz * acy, cy = abz * acx - abx * acz, cz = abx * acy - aby * acx;
    const dbl = Math.hypot(cx, cy, cz);
    if (!(dbl > 0)) return;
    const dA = abx * acx + aby * acy + abz * acz;
    const dB = -(abx * bcx + aby * bcy + abz * bcz);
    const dC = acx * bcx + acy * bcy + acz * bcz;
    ang[a] += Math.atan2(dbl, dA); ang[b] += Math.atan2(dbl, dB); ang[c] += Math.atan2(dbl, dC);
    const cotA = dA / dbl, cotB = dB / dbl, cotC = dC / dbl, tA = dbl / 2;
    const lab = abx * abx + aby * aby + abz * abz, lac = acx * acx + acy * acy + acz * acz, lbc = bcx * bcx + bcy * bcy + bcz * bcz;
    if (cotA < 0) { area[a] += tA / 2; area[b] += tA / 4; area[c] += tA / 4; }
    else if (cotB < 0) { area[b] += tA / 2; area[a] += tA / 4; area[c] += tA / 4; }
    else if (cotC < 0) { area[c] += tA / 2; area[a] += tA / 4; area[b] += tA / 4; }
    else {
      area[a] += (cotB * lac + cotC * lab) / 8;
      area[b] += (cotA * lbc + cotC * lab) / 8;
      area[c] += (cotA * lbc + cotB * lac) / 8;
    }
    // 변 bc 는 꼭짓점 a 의 코탄젠트로, 양 끝점에 (x_i − x_j) 방향으로 더한다.
    lap[B] -= cotA * bcx; lap[B + 1] -= cotA * bcy; lap[B + 2] -= cotA * bcz;
    lap[C] += cotA * bcx; lap[C + 1] += cotA * bcy; lap[C + 2] += cotA * bcz;
    lap[A] -= cotB * acx; lap[A + 1] -= cotB * acy; lap[A + 2] -= cotB * acz;
    lap[C] += cotB * acx; lap[C + 1] += cotB * acy; lap[C + 2] += cotB * acz;
    lap[A] -= cotC * abx; lap[A + 1] -= cotC * aby; lap[A + 2] -= cotC * abz;
    lap[B] += cotC * abx; lap[B + 1] += cotC * aby; lap[B + 2] += cotC * abz;
  };
  for (let i = 0; i < nr - 1; i++) for (let j = 0; j < nc - 1; j++) {
    const p00 = i * nc + j, p01 = p00 + 1, p10 = p00 + nc, p11 = p10 + 1;
    if (diag === 'a') { tri(p00, p10, p11); tri(p00, p11, p01); }
    else { tri(p00, p10, p01); tri(p10, p11, p01); }
  }
  return { ang, area, lap };
}

// ── 공개 함수 ──────────────────────────────────────────────
/**
 * 격자의 각 절점에서 곡률을 계산한다.
 * @param points [row][col]=[x,y,z]
 * @param opt.method 'meyer'(기본, 안쪽 절점은 명세 S1 식) | 'ff'(제1·제2 기본형식)
 * @param opt.diag   'avg'(기본, 두 대각선 평균) | 'a' | 'b'  (meyer 일 때만)
 * @param opt.window 1(기본) 3점 차분, ≥2 면 (2·window+1)² 창 2차 최소제곱 평활. window≥2 는 ff 로 계산한다
 * @returns { nr, nc, K, H, k1, k2, d1, d2, normal, area, boundary, aniso, method, window, noiseGain }
 *   K 가우스 곡률, H 평균 곡률 (1/mm²,1/mm). k1≥k2 주곡률.
 *   부호 규약: 법선은 판 평균이 +z 인 쪽이다. 가운데가 낮은 그릇 모양이면 H>0, 가운데가 높으면 H<0.
 *   d1, d2 는 k1, k2 방향의 3차원 단위 벡터. 주곡률이 같은 점(구)에서는 방향이 정해지지 않고 aniso 가 0 에 가깝다.
 */
export function curvatures(points, opt = {}) {
  const G = toGrid(points, 'curvatures');
  const { nr, nc } = G, N = nr * nc;
  const win = opt.window ?? 1;
  const method = win > 1 ? 'ff' : (opt.method ?? 'meyer');
  const diag = opt.diag ?? 'avg';
  const D = derivatives(G, win);
  const { Xu, Xv, Xuu, Xuv, Xvv } = D;
  const normal = unitNormals(Xu, Xv, N);
  const K = new Float64Array(N), H = new Float64Array(N), k1 = new Float64Array(N), k2 = new Float64Array(N);
  const d1 = new Float64Array(3 * N), d2 = new Float64Array(3 * N), aniso = new Float64Array(N);
  const boundary = new Uint8Array(N);

  // 코탄젠트 식에 쓰는 항. 면적은 방법과 관계없이 분류에 쓴다.
  const terms = diag === 'avg' ? [meshTerms(G, 'a'), meshTerms(G, 'b')] : [meshTerms(G, diag)];
  const area = new Float64Array(N);
  for (let p = 0; p < N; p++) { let s = 0; for (const t of terms) s += t.area[p]; area[p] = s / terms.length; }

  for (let i = 0; i < nr; i++) for (let j = 0; j < nc; j++) {
    const p = i * nc + j, q = 3 * p;
    const isEdge = i === 0 || j === 0 || i === nr - 1 || j === nc - 1;
    boundary[p] = isEdge ? 1 : 0;
    const E = Xu[q] * Xu[q] + Xu[q + 1] * Xu[q + 1] + Xu[q + 2] * Xu[q + 2];
    const F = Xu[q] * Xv[q] + Xu[q + 1] * Xv[q + 1] + Xu[q + 2] * Xv[q + 2];
    const Gg = Xv[q] * Xv[q] + Xv[q + 1] * Xv[q + 1] + Xv[q + 2] * Xv[q + 2];
    const nx = normal[q], ny = normal[q + 1], nz = normal[q + 2];
    const L = Xuu[q] * nx + Xuu[q + 1] * ny + Xuu[q + 2] * nz;
    const M = Xuv[q] * nx + Xuv[q + 1] * ny + Xuv[q + 2] * nz;
    const Nn = Xvv[q] * nx + Xvv[q + 1] * ny + Xvv[q + 2] * nz;
    const det = E * Gg - F * F;
    const Kf = (L * Nn - M * M) / det, Hf = (E * Nn - 2 * F * M + Gg * L) / (2 * det);
    const discF = Math.sqrt(Math.max(Hf * Hf - Kf, 0));
    const kf1 = Hf + discF;

    let Kv = Kf, Hv = Hf;
    if (method === 'meyer' && !isEdge) {
      let deficit = 0, vx = 0, vy = 0, vz = 0, aSum = 0;
      for (const t of terms) {
        deficit += TWO_PI - t.ang[p];
        vx += t.lap[q]; vy += t.lap[q + 1]; vz += t.lap[q + 2];
        aSum += t.area[p];
      }
      // 각도 합의 반올림 오차(약 1e-15 rad)가 sqrt(H²−K) 로 커져 평판이 곡면처럼 보이지 않게 0 으로 자른다.
      // 1e-13 rad 는 h=50 mm 격자에서 K≈4e-17/mm² (곡률 반경 약 160 km) 에 해당한다.
      Kv = Math.abs(deficit) < 1e-13 * terms.length ? 0 : deficit / aSum;
      // 코탄젠트 합 벡터 v = Σ(cot α + cot β)(x_i − x_j) 의 크기는 4A|H|(A 는 혼합 면적 합)이고, 곡면이 볼록하게 나온 쪽을 가리킨다.
      // 논문 식의 |v|/(2A) 는 평균 곡률의 2배다 (11번 문서 §3.5). 여기서는 법선 성분을 써서 부호까지 잡는다 (명세 S1 [제안]).
      Hv = -(vx * nx + vy * ny + vz * nz) / (4 * aSum);
    }
    const disc = Math.sqrt(Math.max(Hv * Hv - Kv, 0));
    K[p] = Kv; H[p] = Hv; k1[p] = Hv + disc; k2[p] = Hv - disc;
    aniso[p] = 2 * disc / (Math.abs(k1[p]) + Math.abs(k2[p]) + 1e-300);

    // 주방향: (II − k I) 의 행 중 큰 쪽에 수직인 벡터. II·I 는 격자 지수 기준이다.
    const r11 = L - kf1 * E, r12 = M - kf1 * F, r22 = Nn - kf1 * Gg;
    let a, b;
    if (Math.abs(r11) + Math.abs(r12) >= Math.abs(r12) + Math.abs(r22)) { a = -r12; b = r11; } else { a = -r22; b = r12; }
    if (a === 0 && b === 0) { a = 1; b = 0; }
    let dx = a * Xu[q] + b * Xv[q], dy = a * Xu[q + 1] + b * Xv[q + 1], dz = a * Xu[q + 2] + b * Xv[q + 2];
    const dl = Math.hypot(dx, dy, dz) || 1; dx /= dl; dy /= dl; dz /= dl;
    d1[q] = dx; d1[q + 1] = dy; d1[q + 2] = dz;
    d2[q] = ny * dz - nz * dy; d2[q + 1] = nz * dx - nx * dz; d2[q + 2] = nx * dy - ny * dx;
  }
  const { hu, hv } = spacingOf(Xu, Xv, N);
  const noiseGain = nodeNoiseGain(D.R, D.C, hu || 1, hv || 1, N);
  // 평활 창을 쓸 때는 창이 판 밖으로 나가는 띠의 값을 분류 비율에 넣지 않는다 (stencil1D 의 설명 참고).
  const trusted = new Uint8Array(N).fill(1);
  if (win > 1) {
    const wr = D.R.w, wc = D.C.w;
    for (let i = 0; i < nr; i++) for (let j = 0; j < nc; j++) if (i < wr || j < wc || i >= nr - wr || j >= nc - wc) trusted[i * nc + j] = 0;
  }
  return { nr, nc, K, H, k1, k2, d1, d2, normal, area, boundary, trusted, aniso, method, window: win, noiseGain, hu, hv };
}

function spacingOf(Xu, Xv, N) {
  const su = new Float64Array(N), sv = new Float64Array(N);
  for (let p = 0; p < N; p++) {
    const q = 3 * p;
    su[p] = Math.hypot(Xu[q], Xu[q + 1], Xu[q + 2]);
    sv[p] = Math.hypot(Xv[q], Xv[q + 1], Xv[q + 2]);
  }
  return { hu: median(su), hv: median(sv) };
}
function median(a) {
  const s = Float64Array.from(a).sort();
  return s.length ? s[s.length >> 1] : 0;
}
function percentile(sorted, q) {
  if (!sorted.length) return 0;
  return sorted[Math.min(sorted.length - 1, Math.max(0, Math.floor(q * sorted.length)))];
}

/**
 * 격자의 법선 방향 잡음 크기 σ(mm)를 추정한다.
 * 곡면이 매끄러우면 4차 차분 Δ⁴ = x[i-2]−4x[i-1]+6x[i]−4x[i+1]+x[i+2] 은 거의 0 이고,
 * 독립 잡음만 있으면 표준편차가 √70·σ 가 된다. 튀는 값에 흔들리지 않게 중앙값으로 잡는다 (1.4826·median/√70).
 * 판의 실제 형상 안의 굴곡도 일부 잡음으로 읽힐 수 있으므로 상한 추정으로 본다.
 */
export function estimateNoise(points) {
  const G = toGrid(points, 'estimateNoise');
  const { X, nr, nc } = G, N = nr * nc;
  const D = derivatives(G, 1);
  const normal = unitNormals(D.Xu, D.Xv, N);
  const vals = [];
  const add = (a, b, c, d, e, o) => {
    let s = 0;
    for (let k = 0; k < 3; k++) s += (X[a + k] - 4 * X[b + k] + 6 * X[c + k] - 4 * X[d + k] + X[e + k]) * normal[o + k];
    if (Number.isFinite(s)) vals.push(Math.abs(s));
  };
  for (let i = 2; i < nr - 2; i++) for (let j = 0; j < nc; j++) {
    const c = (i * nc + j) * 3, st = nc * 3;
    add(c - 2 * st, c - st, c, c + st, c + 2 * st, c);
  }
  for (let i = 0; i < nr; i++) for (let j = 2; j < nc - 2; j++) {
    const c = (i * nc + j) * 3;
    add(c - 6, c - 3, c, c + 3, c + 6, c);
  }
  if (!vals.length) return { sigma: 0, samples: 0 };
  const sigma = 1.4826 * median(Float64Array.from(vals)) / Math.sqrt(70);
  return { sigma, samples: vals.length };
}

/** 표 4(노재규·신종계 2009)의 작업 배정. */
function assignTasks(z, shape) {
  const [a, b] = z;
  if (a === 0 && b === 0) return shape === CLASS.FLAT ? [] : ['롤'];
  if (a === 1 && b === 0) return ['롤', '선상'];
  if (a === 0 && b === 1) return ['롤', '삼각'];
  return ['롤', '선상', '삼각'];
}

/**
 * 비율 벡터 y=(안장, 볼록, 실린더, 평판)에 5% 잡음 처리를 하고 대표 형상과 작업을 정한다.
 * 분류의 맨 끝 단계만 따로 쓸 수 있게 뺐다 (시험 T2).
 */
export function decideShape(yRaw, cut = 0.05) {
  const y = Array.from(yRaw, (v) => (v < cut ? 0 : v));
  const z = [y[CLASS.SADDLE] > 0 ? 1 : 0, y[CLASS.CONVEX] > 0 ? 1 : 0];
  let shape = 0, best = -1;
  for (let k = 0; k < 4; k++) if (y[k] > best) { best = y[k]; shape = k; }  // 표준 벡터 e_k 까지 거리가 가장 가까운 k = 가장 큰 성분
  // 평판 성분만 남으면 곡가공이 필요 없다 (김찬석 외 2014 표 2, 평판: 없음).
  const flatOnly = shape === CLASS.FLAT && y[CLASS.CYLINDER] === 0 && z[0] === 0 && z[1] === 0;
  return { y, z, shape, shapeName: SHAPE_NAME[shape], tasks: assignTasks(z, flatOnly ? CLASS.FLAT : -1) };
}

/**
 * 곡판을 안장/볼록/실린더/평판으로 분류한다 (명세 S1).
 * @param points [row][col]=[x,y,z]
 * @param opt.epsK      K=0 판정 여유 (기본 EPS_K)
 * @param opt.epsH      평판 판정 여유 (기본 EPS_H)
 * @param opt.noiseCut  y 성분을 0 으로 처리하는 문턱 (기본 0.05, 논문값)
 * @param opt.useArea   true 면 y 를 면적 비율로 계산한다 (기본 false, 논문은 점 개수)
 * @param opt.window    곡률 창 크기. 측정 곡면처럼 잡음이 있으면 2 이상으로 둔다
 * @param opt.noiseSigma 잡음 σ(mm)를 알면 넘긴다. 없으면 격자에서 추정한다
 * @param opt.flatSag   판 대각선 기준 처짐이 이 값(mm) 이하이면 평판으로 본다. 기본 0(쓰지 않음)
 * @param opt.curv      이미 계산한 curvatures 결과를 다시 쓸 때
 * @returns { y, yRaw, z, shape, shapeName, tasks, areaRatio, countRatio, classes, thresholds, noise, curv }
 *   y, yRaw, areaRatio, countRatio 는 (안장, 볼록, 실린더, 평판) 순서. classes 는 절점별 CLASS 값.
 */
export function classifyPlate(points, opt = {}) {
  const c = opt.curv ?? curvatures(points, opt);
  const { nr, nc, K, k1, k2, area, boundary, noiseGain, trusted } = c, N = nr * nc;
  const epsK = opt.epsK ?? EPS_K, epsH = opt.epsH ?? EPS_H, cut = opt.noiseCut ?? NOISE_CUT;
  const G = toGrid(points, 'classifyPlate');
  const est = opt.noiseSigma !== undefined ? { sigma: opt.noiseSigma, samples: 0 } : estimateNoise(G);

  // 판 크기 (경계 상자의 대각선)
  let lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  for (let p = 0; p < N; p++) for (let k = 0; k < 3; k++) {
    const v = G.X[3 * p + k];
    if (v < lo[k]) lo[k] = v;
    if (v > hi[k]) hi[k] = v;
  }
  const diag = Math.hypot(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]) || 1;

  const mAbs = new Float64Array(N);
  for (let p = 0; p < N; p++) mAbs[p] = Math.max(Math.abs(k1[p]), Math.abs(k2[p]));
  // 판의 곡률 크기 기준 m_ref: 안쪽 절점의 상위 분위수. 경계의 한쪽 차분과 평활 창의 띠(trusted=0)는 뺀다.
  const inner = [];
  for (let p = 0; p < N; p++) if (trusted[p] && !boundary[p] && Number.isFinite(mAbs[p])) inner.push(mAbs[p]);
  if (inner.length === 0) for (let p = 0; p < N; p++) if (Number.isFinite(mAbs[p])) inner.push(mAbs[p]);
  const mRef = percentile(Float64Array.from(inner).sort(), TOL.MREF_Q);
  let kappaNoiseIn = 0;
  { const v = []; for (let p = 0; p < N; p++) if (trusted[p] && !boundary[p]) v.push(noiseGain[p]); kappaNoiseIn = est.sigma * (v.length ? median(Float64Array.from(v)) : 0); }
  const flatSagKappa = (opt.flatSag ?? 0) > 0 ? 8 * opt.flatSag / (diag * diag) : 0;
  const tauFlatBase = Math.max(epsH * mRef, flatSagKappa, NUM_FLOOR / diag);

  const classes = new Uint8Array(N).fill(INVALID);
  const count = [0, 0, 0, 0], areaSum = [0, 0, 0, 0];
  let valid = 0, areaAll = 0;
  for (let p = 0; p < N; p++) {
    const m = mAbs[p];
    if (!Number.isFinite(m) || !Number.isFinite(K[p])) continue;
    const small = Math.min(Math.abs(k1[p]), Math.abs(k2[p]));
    const tauN = NOISE_SIGMAS * est.sigma * noiseGain[p];
    let cls;
    if (m <= Math.max(tauFlatBase, tauN)) cls = CLASS.FLAT;
    else if (Math.abs(K[p]) < epsK * m * m || small <= tauN) cls = CLASS.CYLINDER;
    else cls = K[p] < 0 ? CLASS.SADDLE : CLASS.CONVEX;
    classes[p] = cls;
    if (!trusted[p]) continue;       // 평활 창의 띠는 지도에만 쓰고 비율에는 넣지 않는다
    count[cls]++; areaSum[cls] += area[p];
    valid++; areaAll += area[p];
  }
  if (valid === 0) throw new Error('classifyPlate: 곡률을 계산할 수 있는 절점이 없습니다. 격자 값에 빈 칸(NaN)이 많은지 확인하세요.');
  const countRatio = count.map((v) => (valid ? v / valid : 0));
  const areaRatio = areaSum.map((v) => (areaAll > 0 ? v / areaAll : 0));
  const yRaw = opt.useArea ? areaRatio : countRatio;
  const decided = decideShape(yRaw, cut);
  const noiseRatio = mRef > 0 ? kappaNoiseIn / mRef : 0;
  return {
    ...decided, yRaw: yRaw.slice(), areaRatio, countRatio, classes, valid,
    thresholds: { epsK, epsH, cut, tauFlatBase, mRef, diag, noiseSigmas: NOISE_SIGMAS },
    noise: { sigma: est.sigma, kappaNoise: kappaNoiseIn, ratio: noiseRatio, reliable: noiseRatio <= NOISE_RELIABLE },
    curv: c,
  };
}

// ── 막 변형률·곡률 변화량 (유철호·신종계 2002, 명세 S3·S4) ──
function principal2D(xx, yy, xy) {
  const c = 0.5 * (xx + yy), d = 0.5 * (xx - yy), r = Math.hypot(d, xy);
  return { p1: c + r, p2: c - r, phi1: 0.5 * Math.atan2(2 * xy, xx - yy) };
}

/**
 * 시작 형상에서 목표 곡면까지의 막 변형률(Green-Lagrange), 곡률 변화량, 주방향을 절점마다 계산한다.
 *
 * @param from 지금 형상 [row][col]=[x,y,z]. 없으면(null) 평판에서 시작한다
 * @param to   목표 곡면
 * @param flat 전개판 [row][col]=[x,y] (또는 [x,y,z]). 같은 재료점을 가리키는 격자여야 한다
 *             없고 from 도 없으면 목표 곡면을 xy 평면에 투영한 판을 쓴다 (note 에 적는다)
 * @param opt.thickness  판 두께(mm). 면내·굽힘 중 큰 쪽을 고를 때 필요 (use='auto')
 * @param opt.use        'auto'(기본, 명세 S4) | 'membrane' | 'bending'
 * @param opt.window     곡률과 같은 창 크기. 측정 곡면이면 2 이상
 * @returns {
 *   nr, nc, chart,
 *   eps_m: {xx, yy, xy},     막 변형률 (무차원, 텐서 성분. 공학 전단 γ=2·xy)
 *   kappa: {xx, yy, xy},     곡률 변화량 (1/mm) κ_목표−κ_시작. κ_αβ = −n·X,αβ (기울기가 작으면 −w,αβ). 가운데가 낮은 그릇 모양이면 κ 음수
 *   principal: { m:{e1,e2,phi1}, b:{k1,k2,phi1} },  phi1 은 e1(k1) 방향 각 (rad, 기준 좌표 x 축에서)
 *   dirField: { psi, c2, s2, mag, src, anis, d3, nr, nc }  가열선 방향장 (방향의 부호는 없다).
 *      psi 가열선 방향각, c2·s2 2배각, mag 지배 성분 크기(thickness 없이 use='bending' 이면 κ 그대로, 1/mm),
 *      src 0=면내, 1=굽힘, anis 0~1 (0 이면 방향을 정할 수 없음), d3 목표 곡면 위 3차원 단위 벡터
 *   trusted,  창≥2 일 때 창이 판 안에 다 들어간 절점 표시 (곡률과 같은 뜻)
 *   note
 * }
 *   기준 좌표: flat 이 있으면 전개판의 x, y. flat 없이 from 만 있으면 from 곡면 접평면의 국소 좌표
 *   (x 는 행 방향을 접평면에 투영한 방향). flat 도 from 도 없으면 목표를 투영한 xy.
 */
export function strains(from, to, flat, opt = {}) {
  const T = toGrid(to, 'strains(to)');
  const S = from ? toGrid(from, 'strains(from)') : null;
  const P = flat ? toGrid(flat, 'strains(flat)') : null;
  if (S) sameSize(S, T, 'strains(from, to)');
  if (P) sameSize(P, T, 'strains(flat, to)');
  const { nr, nc } = T, N = nr * nc;
  const win = opt.window ?? 1;
  const use = opt.use ?? 'auto';
  const t = opt.thickness;
  if (use === 'auto' && !(t > 0)) throw new Error('strains: 면내·굽힘 중 큰 쪽을 고르려면 opt.thickness(판 두께 mm)가 필요합니다.');

  const Dt = derivatives(T, win), nt = unitNormals(Dt.Xu, Dt.Xv, N);
  const Ds = S ? derivatives(S, win) : null, ns = S ? unitNormals(Ds.Xu, Ds.Xv, N) : null;
  const Dp = P ? derivatives(P, win) : null;
  const chart = P ? 'flat' : S ? 'from' : 'projection';
  const note = chart === 'projection' ? '전개판이 없어 목표 곡면을 xy 평면에 투영한 판을 기준으로 계산했습니다. 막 변형률은 w,α w,β/2 항만 들어갑니다.' : '';

  // from 기준 국소 좌표의 x 축: 판 전체의 평균 행 방향
  let ax = [0, 0, 0];
  if (chart === 'from') {
    for (let p = 0; p < N; p++) for (let k = 0; k < 3; k++) ax[k] += Ds.Xu[3 * p + k];
    const l = Math.hypot(ax[0], ax[1], ax[2]) || 1;
    ax = ax.map((v) => v / l);
  }

  // 창을 쓰면 경계 띠는 값이 치우치므로 표시한다 (curvatures 의 trusted 와 같다)
  const trusted = new Uint8Array(N).fill(1);
  if (win > 1) {
    const wr = Dt.R.w, wc = Dt.C.w;
    for (let i = 0; i < nr; i++) for (let j = 0; j < nc; j++) if (i < wr || j < wc || i >= nr - wr || j >= nc - wc) trusted[i * nc + j] = 0;
  }
  const out = {
    nr, nc, chart, note, trusted,
    eps_m: { xx: new Float64Array(N), yy: new Float64Array(N), xy: new Float64Array(N) },
    kappa: { xx: new Float64Array(N), yy: new Float64Array(N), xy: new Float64Array(N) },
    principal: {
      m: { e1: new Float64Array(N), e2: new Float64Array(N), phi1: new Float64Array(N) },
      b: { k1: new Float64Array(N), k2: new Float64Array(N), phi1: new Float64Array(N) },
    },
    dirField: {
      nr, nc, psi: new Float64Array(N), c2: new Float64Array(N), s2: new Float64Array(N),
      mag: new Float64Array(N), src: new Uint8Array(N), anis: new Float64Array(N), d3: new Float64Array(3 * N),
    },
  };

  const dot3 = (a, i, b, j) => a[i] * b[j] + a[i + 1] * b[j + 1] + a[i + 2] * b[j + 2];
  for (let p = 0; p < N; p++) {
    const q = 3 * p;
    // 기준 좌표 → 격자 지수: J = [[a, b], [c, d]] = ∂(x,y)/∂(u,v)
    let a, b, c, d;
    if (chart === 'flat') {
      a = Dp.Xu[q]; b = Dp.Xv[q]; c = Dp.Xu[q + 1]; d = Dp.Xv[q + 1];
    } else if (chart === 'from') {
      // 접평면 위 단위 벡터 e1(행 방향 투영), e2 = n × e1
      const dn = ax[0] * ns[q] + ax[1] * ns[q + 1] + ax[2] * ns[q + 2];
      let e1x = ax[0] - dn * ns[q], e1y = ax[1] - dn * ns[q + 1], e1z = ax[2] - dn * ns[q + 2];
      const le = Math.hypot(e1x, e1y, e1z) || 1; e1x /= le; e1y /= le; e1z /= le;
      const e2x = ns[q + 1] * e1z - ns[q + 2] * e1y, e2y = ns[q + 2] * e1x - ns[q] * e1z, e2z = ns[q] * e1y - ns[q + 1] * e1x;
      a = Ds.Xu[q] * e1x + Ds.Xu[q + 1] * e1y + Ds.Xu[q + 2] * e1z;
      b = Ds.Xv[q] * e1x + Ds.Xv[q + 1] * e1y + Ds.Xv[q + 2] * e1z;
      c = Ds.Xu[q] * e2x + Ds.Xu[q + 1] * e2y + Ds.Xu[q + 2] * e2z;
      d = Ds.Xv[q] * e2x + Ds.Xv[q + 1] * e2y + Ds.Xv[q + 2] * e2z;
    } else {
      a = Dt.Xu[q]; b = Dt.Xv[q]; c = Dt.Xu[q + 1]; d = Dt.Xv[q + 1];
    }
    const det = a * d - b * c;
    // 격자 지수 → 기준 좌표의 미분 (u_x, u_y, v_x, v_y)
    const ux = d / det, uy = -b / det, vx = -c / det, vy = a / det;

    // 곡면 계량 (u,v 기준). 시작 형상이 없으면 기준면 자신의 계량 (JᵀJ)
    const Et = dot3(Dt.Xu, q, Dt.Xu, q), Ft = dot3(Dt.Xu, q, Dt.Xv, q), Gt = dot3(Dt.Xv, q, Dt.Xv, q);
    let Es, Fs, Gs;
    if (S) { Es = dot3(Ds.Xu, q, Ds.Xu, q); Fs = dot3(Ds.Xu, q, Ds.Xv, q); Gs = dot3(Ds.Xv, q, Ds.Xv, q); }
    else { Es = a * a + c * c; Fs = a * b + c * d; Gs = b * b + d * d; }
    const E11 = 0.5 * (Et - Es), E12 = 0.5 * (Ft - Fs), E22 = 0.5 * (Gt - Gs);

    const exx = ux * ux * E11 + 2 * ux * vx * E12 + vx * vx * E22;
    const eyy = uy * uy * E11 + 2 * uy * vy * E12 + vy * vy * E22;
    const exy = ux * uy * E11 + (ux * vy + vx * uy) * E12 + vx * vy * E22;
    out.eps_m.xx[p] = exx; out.eps_m.yy[p] = eyy; out.eps_m.xy[p] = exy;

    // 제2 기본형식 (u,v 기준). 곡률 변화량 ΔII = II_목표 − II_시작
    const L = dot3(Dt.Xuu, q, nt, q) - (S ? dot3(Ds.Xuu, q, ns, q) : 0);
    const M = dot3(Dt.Xuv, q, nt, q) - (S ? dot3(Ds.Xuv, q, ns, q) : 0);
    const Nn = dot3(Dt.Xvv, q, nt, q) - (S ? dot3(Ds.Xvv, q, ns, q) : 0);
    // κ_αβ = −II_αβ (명세 S3의 κ=−w,αβ 를 곡면 식으로 쓴 것. 기울기가 작으면 같다)
    const kxx = -(ux * ux * L + 2 * ux * vx * M + vx * vx * Nn);
    const kyy = -(uy * uy * L + 2 * uy * vy * M + vy * vy * Nn);
    const kxy = -(ux * uy * L + (ux * vy + vx * uy) * M + vx * vy * Nn);
    out.kappa.xx[p] = kxx; out.kappa.yy[p] = kyy; out.kappa.xy[p] = kxy;

    const pm = principal2D(exx, eyy, exy), pb = principal2D(kxx, kyy, kxy);
    out.principal.m.e1[p] = pm.p1; out.principal.m.e2[p] = pm.p2; out.principal.m.phi1[p] = pm.phi1;
    out.principal.b.k1[p] = pb.p1; out.principal.b.k2[p] = pb.p2; out.principal.b.phi1[p] = pb.phi1;

    // 만들 변형이 큰 주방향과 크기 (명세 S4)
    const bigM = Math.abs(pm.p1) >= Math.abs(pm.p2);
    const sm = Math.max(Math.abs(pm.p1), Math.abs(pm.p2));
    const phiM = bigM ? pm.phi1 : pm.phi1 + Math.PI / 2;
    const anisM = (Math.abs(pm.p1 - pm.p2)) / (Math.abs(pm.p1) + Math.abs(pm.p2) + 1e-300);
    const bigB = Math.abs(pb.p1) >= Math.abs(pb.p2);
    const sbRaw = Math.max(Math.abs(pb.p1), Math.abs(pb.p2));
    const sb = sbRaw * (t > 0 ? t / 2 : 1);
    const phiB = bigB ? pb.phi1 : pb.phi1 + Math.PI / 2;
    const anisB = (Math.abs(pb.p1 - pb.p2)) / (Math.abs(pb.p1) + Math.abs(pb.p2) + 1e-300);
    const useM = use === 'membrane' || (use === 'auto' && sm >= sb);

    // 가열선은 그 주방향에 수직으로 긋는다.
    const psi = (useM ? phiM : phiB) + Math.PI / 2;
    const f = out.dirField;
    f.psi[p] = Math.atan2(Math.sin(psi), Math.cos(psi));
    f.c2[p] = Math.cos(2 * psi); f.s2[p] = Math.sin(2 * psi);
    f.mag[p] = useM ? sm : sb;
    f.src[p] = useM ? 0 : 1;
    f.anis[p] = useM ? anisM : anisB;

    // 화면 바늘용 3차원 방향: 기준 좌표의 (cos ψ, sin ψ) 를 목표 곡면에 옮긴 것
    const cs = Math.cos(psi), sn = Math.sin(psi);
    const du = ux * cs + uy * sn, dv = vx * cs + vy * sn;
    let dx = Dt.Xu[q] * du + Dt.Xv[q] * dv, dy = Dt.Xu[q + 1] * du + Dt.Xv[q + 1] * dv, dz = Dt.Xu[q + 2] * du + Dt.Xv[q + 2] * dv;
    const dl = Math.hypot(dx, dy, dz) || 1;
    f.d3[q] = dx / dl; f.d3[q + 1] = dy / dl; f.d3[q + 2] = dz / dl;
  }
  return out;
}

/**
 * 방향장을 격자 지수 (i, j, 소수 가능) 위치에서 읽는다.
 * 방향에 부호가 없으므로 2배각(cos 2ψ, sin 2ψ)으로 쌍선형 보간한다.
 * @returns { dx, dy, mag, src } dx·dy 는 기준 좌표의 단위 벡터. 앞뒤 정렬은 호출하는 쪽에서 한다.
 */
export function sampleDirField(field, i, j) {
  const { nr, nc, c2, s2, mag, src } = field;
  const i0 = Math.max(0, Math.min(nr - 2, Math.floor(i))), j0 = Math.max(0, Math.min(nc - 2, Math.floor(j)));
  const u = Math.max(0, Math.min(1, i - i0)), v = Math.max(0, Math.min(1, j - j0));
  const p00 = i0 * nc + j0, p01 = p00 + 1, p10 = p00 + nc, p11 = p10 + 1;
  const w00 = (1 - u) * (1 - v), w01 = (1 - u) * v, w10 = u * (1 - v), w11 = u * v;
  const c = w00 * c2[p00] + w01 * c2[p01] + w10 * c2[p10] + w11 * c2[p11];
  const s = w00 * s2[p00] + w01 * s2[p01] + w10 * s2[p10] + w11 * s2[p11];
  const psi = 0.5 * Math.atan2(s, c);
  const near = (u < 0.5 ? (v < 0.5 ? p00 : p01) : (v < 0.5 ? p10 : p11));
  return { dx: Math.cos(psi), dy: Math.sin(psi), mag: w00 * mag[p00] + w01 * mag[p01] + w10 * mag[p10] + w11 * mag[p11], src: src[near] };
}

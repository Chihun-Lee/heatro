// 가열선 추천 — 빠른 정방향 해석을 이용한 후보선 탐욕 선택
// 학습 데이터가 필요 없는 물리 기반 방식이다. 해석 모델의 변형량 수준이 실제와 다를 수 있으므로
// 결과의 세기(w)는 "기준 1줄 대비 필요한 배수"로 해석하고, 작업자가 검토·승인한 뒤에만 사용한다.
import { flatten } from './solver.js';

/** 격자 지수 (i,j) 위치의 윗면 좌표를 쌍선형 보간으로 구한다. */
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

function spacing(points) {
  const nr = points.length, nc = points[0].length;
  const a = points[nr >> 1][nc >> 1], b = points[(nr >> 1) + 1][nc >> 1], c = points[nr >> 1][(nc >> 1) + 1];
  return { dr: Math.hypot(a[0] - b[0], a[1] - b[1]) || 1, dc: Math.hypot(a[0] - c[0], a[1] - c[1]) || 1 };
}

/** 폭방향·사선·길이방향 직선 후보를 만든다. */
export function makeCandidates(points, opt = {}) {
  const nr = points.length, nc = points[0].length;
  const { dr, dc } = spacing(points);
  const pitch = opt.pitch ?? 150; // mm
  const angles = opt.angles ?? [0, 12, -12, 25, -25];
  const margin = opt.margin ?? 0.05;
  const families = opt.families ?? ['across', 'along'];
  const out = [];
  const sample = 25;
  if (families.includes('across')) {
    const j0 = (nc - 1) * margin, j1 = (nc - 1) * (1 - margin), jc = (j0 + j1) / 2;
    const step = pitch / dr;
    for (const deg of angles) {
      const slope = Math.tan(deg * Math.PI / 180) * (dc / dr);
      for (let ic = 0; ic <= nr - 1 + 1e-9; ic += step) {
        const poly = [];
        for (let s = 0; s < sample; s++) {
          const j = j0 + (j1 - j0) * s / (sample - 1);
          const i = ic + slope * (j - jc);
          if (i < 0 || i > nr - 1) continue;
          poly.push(surfaceAt(points, i, j));
        }
        if (poly.length >= sample * 0.6) out.push({ family: 'across', deg, points: poly });
      }
    }
  }
  if (families.includes('along')) {
    const i0 = (nr - 1) * margin, i1 = (nr - 1) * (1 - margin);
    const step = Math.max(pitch, 300) / dc;
    for (let jc = step; jc <= nc - 1 - step + 1e-9; jc += step) {
      const poly = [];
      for (let s = 0; s < sample; s++) poly.push(surfaceAt(points, i0 + (i1 - i0) * s / (sample - 1), jc));
      out.push({ family: 'along', deg: 90, points: poly });
    }
  }
  return out;
}

/** z 성분만 뽑아 평면(강체 기울임) 성분을 제거한다. */
export function zPlaneFree(field, X) {
  const n = X.length / 3;
  let cx = 0, cy = 0;
  for (let i = 0; i < n; i++) { cx += X[3 * i]; cy += X[3 * i + 1]; }
  cx /= n; cy /= n;
  let sxx = 0, syy = 0, sxy = 0, sz = 0, sxz = 0, syz = 0;
  for (let i = 0; i < n; i++) {
    const x = X[3 * i] - cx, y = X[3 * i + 1] - cy, z = field[3 * i + 2];
    sxx += x * x; syy += y * y; sxy += x * y; sz += z; sxz += x * z; syz += y * z;
  }
  const det = sxx * syy - sxy * sxy || 1;
  const a = (sxz * syy - syz * sxy) / det, b = (syz * sxx - sxz * sxy) / det, c = sz / n;
  const out = new Float64Array(n);
  for (let i = 0; i < n; i++) out[i] = field[3 * i + 2] - (c + a * (X[3 * i] - cx) + b * (X[3 * i + 1] - cy));
  return out;
}

const dot = (a, b) => { let s = 0; for (let i = 0; i < a.length; i++) s += a[i] * b[i]; return s; };
export const rms = (a) => Math.sqrt(dot(a, a) / a.length);

/** 0 ≤ w ≤ wmax 범위의 좌표하강 최소제곱. 반환: { w, residual } */
export function boundedFit(R, D, wmax = Infinity, iters = 80) {
  const n = R.length;
  const w = new Float64Array(n);
  const res = Float64Array.from(D);
  const norms = R.map((r) => dot(r, r) || 1);
  for (let it = 0; it < iters; it++) {
    let change = 0;
    for (let c = 0; c < n; c++) {
      const r = R[c];
      const next = Math.min(wmax, Math.max(0, w[c] + dot(r, res) / norms[c]));
      const d = next - w[c];
      if (d !== 0) {
        for (let i = 0; i < res.length; i++) res[i] -= d * r[i];
        w[c] = next;
        change += Math.abs(d);
      }
    }
    if (change < 1e-6) break;
  }
  return { w, residual: res };
}

function lineGap(a, b) {
  let sum = 0;
  for (const p of a) {
    let best = Infinity;
    for (const q of b) best = Math.min(best, Math.hypot(p[0] - q[0], p[1] - q[1]));
    sum = Math.min(best, sum === 0 ? best : sum);
  }
  return sum;
}

/**
 * 후보선 응답을 계산하고 탐욕 선택으로 가열선을 고른다.
 * @param model   factorize 된 PlateModel
 * @param D       목표 z 편차(평면 제거, zPlaneFree 결과)
 * @param comp    기준 1줄 성분
 * @param opt     { maxLines, minGap, wmax, pitch, angles, families, minGain }
 */
export function recommend(model, points, D, comp, opt = {}, onProgress) {
  const X = flatten(points);
  const maxLines = opt.maxLines ?? 10;
  const minGap = opt.minGap ?? 150;
  const wmax = opt.wmax ?? 20;
  const minGain = opt.minGain ?? 0.002;
  const candidates = makeCandidates(points, opt);
  const R = [];
  candidates.forEach((c, idx) => {
    const u = model.solve(model.lineLoad(c.points, comp));
    R.push(zPlaneFree(model.topDisplacement(u), X));
    if (onProgress && idx % 8 === 0) onProgress(0.8 * idx / candidates.length);
  });
  const total = dot(D, D) || 1;
  const picked = [];
  let residual = Float64Array.from(D);
  for (let k = 0; k < maxLines; k++) {
    let best = -1, bestGain = 0;
    for (let c = 0; c < candidates.length; c++) {
      if (picked.includes(c)) continue;
      const a = dot(R[c], residual);
      if (a <= 0) continue;
      const gain = a * a / (dot(R[c], R[c]) || 1);
      if (gain <= bestGain) continue;
      if (picked.some((p) => lineGap(candidates[c].points, candidates[p].points) < minGap)) continue;
      best = c; bestGain = gain;
    }
    if (best < 0 || bestGain / total < minGain) break;
    picked.push(best);
    residual = boundedFit(picked.map((p) => R[p]), D, wmax).residual;
    if (onProgress) onProgress(0.8 + 0.2 * (k + 1) / maxLines);
  }
  const fit = boundedFit(picked.map((p) => R[p]), D, wmax);
  const lines = picked.map((p, n) => ({ ...candidates[p], weight: fit.w[n] })).filter((l) => l.weight > 1e-6);
  return { lines, rmsBefore: rms(D), rmsAfter: rms(fit.residual), candidates: candidates.length, predicted: fit.residual };
}

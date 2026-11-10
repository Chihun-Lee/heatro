// AI 실시간 시뮬레이터: 브라우저 안에서 순수 JS 로 추론한다 (서버 호출 없음).
// 모델 파일은 ai_model/model.json + model.bin 이다.
'use strict';
const qs = new URLSearchParams(location.search);
const MODEL_DIR = qs.get('model') || 'ai_model';
const $ = (id) => document.getElementById(id);
const css = (n) => getComputedStyle(document.documentElement).getPropertyValue(n).trim();

let M = null;       // 모델 메타
let BUF = null;     // model.bin
const INPUT_NAMES = ['thickness', 'speed', 'peak_target', 'r0', 'gap', 'h_water', 'fy', 'water'];
const state = { x: null, timeIndex: 8, playTimer: null, lastPred: null, inferMs: [], drawMs: [] };

// ── 모델 파일 ─────────────────────────────────────────────────────
function blob(name) {
  const b = M.blobs[name];
  const T = b.dtype === 'f4' ? Float32Array : Float64Array;
  return new T(BUF, b.offset, b.length);
}

// ── 입력 → 특징 (train 쪽 lib.py 와 같은 식) ────────────────────────
function featRaw(x) {
  const [t, v, p, r, g, h, f, w] = x;
  return [t, v, p, r, f, w, w * g, w * Math.log(h)];
}
function featPhys(x) {
  const [t, v, p, r, g, h, f, w] = x;
  const KAPPA = M.const.kappa, E = M.const.e_ref;
  const dT = Math.max(p - 20, 1);
  const Pe = v * r / KAPPA;
  const Qt2 = dT * r / (v * t * t);
  const eps = 1.2e-5 * dT * E / f;
  const Fo = KAPPA * (2 * r / v) / (t * t);
  return [Math.log(t), Math.log(v), Math.log(r), Math.log(dT), Math.log(f), Math.log(Pe), Math.log(t / r), Math.log(Qt2), Math.log(eps), Math.log(Fo), w, w * Math.log(g / v), w * Math.log(h)];
}
const FEATS = { raw: featRaw, phys: featPhys };
function scaled(fname, x, scaler) {
  const f = FEATS[fname](x);
  return f.map((v, i) => (v - scaler.m[i]) / scaler.s[i]);
}

// ── 가우시안 과정 ─────────────────────────────────────────────────
function gpKernelVec(gp, z, Z, d, N) {
  const lam = gp.lam, sf2 = gp.sf2, m52 = gp.kernel === 'm52';
  const kv = new Float64Array(N);
  for (let i = 0; i < N; i++) {
    let r2 = 0;
    const off = i * d;
    for (let j = 0; j < d; j++) { const dz = z[j] - Z[off + j]; r2 += dz * dz * lam[j]; }
    if (m52) { const s5r = Math.sqrt(5 * r2); kv[i] = sf2 * (1 + s5r + 5 * r2 / 3) * Math.exp(-s5r); }
    else kv[i] = sf2 * Math.exp(-0.5 * r2);
  }
  return kv;
}
function dot(a, b) { let s = 0; for (let i = 0; i < a.length; i++) s += a[i] * b[i]; return s; }
function gpMean(gp, z, Z, d, N) {
  const kv = gpKernelVec(gp, z, Z, d, N);
  return gp.ym + gp.ys * dot(kv, cachedAlpha(gp));
}
function cachedAlpha(gp) { return gp._alpha || (gp._alpha = blob(gp.alpha)); }
function gpPredict(gp, z, Z, d, N) {
  const kv = gpKernelVec(gp, z, Z, d, N);
  const mean = gp.ym + gp.ys * dot(kv, cachedAlpha(gp));
  const L = gp._L || (gp._L = blob(gp.L));
  const v = new Float64Array(N);
  let vv = 0, off = 0;
  for (let i = 0; i < N; i++) {
    let s = kv[i];
    for (let j = 0; j < i; j++) s -= L[off + j] * v[j];
    v[i] = s / L[off + i];
    vv += v[i] * v[i];
    off += i + 1;
  }
  const varz = Math.max(gp.sf2 + gp.sn2 - vv, 1e-12);
  return { mean, std: Math.sqrt(varz) * gp.ys };
}

// ── MLP (GELU 활성화) ─────────────────────────────────────────────
const ERFC_COF = [-1.3026537197817094, 6.4196979235649026e-1, 1.9476473204185836e-2, -9.561514786808631e-3, -9.46595344482036e-4, 3.66839497852761e-4, 4.2523324806907e-5, -2.0278578112534e-5, -1.624290004647e-6, 1.303655835580e-6, 1.5626441722e-8, -8.5238095915e-8, 6.529054439e-9, 5.059343495e-9, -9.91364156e-10, -2.27365122e-10, 9.6467911e-11, 2.394038e-12, -6.886027e-12, 8.94487e-13, 3.13092e-13, -1.12708e-13, 3.81e-16, 7.106e-15, -1.523e-15, -9.4e-17, 1.21e-16, -2.8e-17];
function erfc(x) {
  const z = Math.abs(x); const t = 2 / (2 + z); const ty = 4 * t - 2;
  let d = 0, dd = 0;
  for (let j = ERFC_COF.length - 1; j > 0; j--) { const tmp = d; d = ty * d - dd + ERFC_COF[j]; dd = tmp; }
  const ans = t * Math.exp(-z * z + 0.5 * (ERFC_COF[0] + ty * d) - dd);
  return x >= 0 ? ans : 2 - ans;
}
const gelu = (x) => 0.5 * x * (2 - erfc(x * Math.SQRT1_2));
function netLayers(spec) {
  if (spec._nets) return spec._nets;
  spec._nets = spec.layers.map((l) => ({ W: blob(l.W), b: blob(l.b), shapeW: M.blobs[l.W].shape, shapeB: M.blobs[l.b].shape }));
  return spec._nets;
}
function runEnsemble(spec, z) {
  // z: 정규화 입력 (d). 각 층 W: [M, out, in], b: [M, out]. 출력 [M, n_out]
  const nets = netLayers(spec), Mm = spec.M;
  const outs = [];
  for (let m = 0; m < Mm; m++) {
    let h = z;
    for (let li = 0; li < nets.length; li++) {
      const { W, b, shapeW } = nets[li];
      const nOut = shapeW[1], nIn = shapeW[2];
      const off = m * nOut * nIn, bo = m * nOut;
      const out = new Array(nOut);
      for (let o = 0; o < nOut; o++) {
        let s = b[bo + o];
        const row = off + o * nIn;
        for (let i = 0; i < nIn; i++) s += W[row + i] * h[i];
        out[o] = li < nets.length - 1 ? gelu(s) : s;
      }
      h = out;
    }
    outs.push(h);
  }
  return outs;
}

// ── 필드 (POD + 회귀, 좌표 모델) ──────────────────────────────────
function fieldPredict(spec, x, withHist) {
  const fam = spec.family;
  if (fam === 'coord') {
    const zc = scaled(spec.feature, x, spec.scaler);
    const co = blob(spec.coord.co);
    const [ny, nz] = spec.coord.shape;
    const P = new Float64Array(ny * nz);
    const nd = zc.length;
    const input = new Array(nd + 2);
    for (let i = 0; i < nd; i++) input[i] = zc[i];
    for (let idx = 0; idx < ny * nz; idx++) {
      input[nd] = co[idx * 2]; input[nd + 1] = co[idx * 2 + 1];
      P[idx] = runEnsemble(spec.coord, input)[0][0] * spec.coord.fs + spec.coord.fm;
    }
    return P;
  }
  const z = scaled(spec.feature, x, spec.scaler);
  const k = spec.pod.k;
  const coef = new Float64Array(k);
  if (fam === 'pod_gp') {
    const ref = spec.gps[0];
    const Zb = M.blobs[ref.Z];
    const Z = blob(ref.Z), N = Zb.shape[0], d = Zb.shape[1];
    for (let c = 0; c < k; c++) coef[c] = gpMean(spec.gps[c], z, Z, d, N);
  } else {
    const outs = runEnsemble(spec.mlp, z);
    for (let c = 0; c < k; c++) {
      let s = 0;
      for (let m = 0; m < outs.length; m++) s += outs[m][c];
      coef[c] = s / outs.length * spec.mlp.asd[c] + spec.mlp.am[c];
    }
  }
  const mean = spec._mean || (spec._mean = blob(spec.pod.mean));
  const Phi = spec._phi || (spec._phi = blob(spec.pod.Phi));
  const D = mean.length;
  const out = new Float64Array(D);
  for (let i = 0; i < D; i++) out[i] = mean[i];
  for (let c = 0; c < k; c++) { const off = c * D; const a = coef[c]; for (let i = 0; i < D; i++) out[i] += a * Phi[off + i]; }
  return out;
}

// ── 스칼라 추론 ───────────────────────────────────────────────────
function predictScalars(x) {
  const out = {};
  // MLP 앙상블(선택된 경우) 한 번만 실행
  let mlpOuts = null;
  for (const s of M.scalars) {
    if (s.family === 'mlp' && mlpOuts === null) {
      const spec = s.mlp;
      const z = scaled(spec.feature, x, spec.scaler);
      mlpOuts = runEnsemble(spec, z);
    }
  }
  for (const s of M.scalars) {
    let val, std;
    if (s.family === 'gp') {
      const gp = s.gp;
      const Zb = M.blobs[gp.Z], Z = blob(gp.Z), N = Zb.shape[0], d = Zb.shape[1];
      const z = scaled(s.feature, x, s.scaler);
      const r = gpPredict(gp, z, Z, d, N);
      if (s.tf.kind === 'log') { val = s.tf.sign * Math.exp(r.mean); std = Math.abs(val) * r.std; }
      else { val = r.mean; std = r.std; }
    } else {
      const spec = s.mlp;
      const vals = mlpOuts.map((o) => o[s.index] * spec.ys[s.index] + spec.ym[s.index]);
      const m = vals.reduce((a, b) => a + b, 0) / vals.length;
      const sd = Math.sqrt(vals.reduce((a, b) => a + (b - m) * (b - m), 0) / vals.length);
      if (s.index === spec.q_idx) { val = Math.exp(m); std = val * sd; }
      else { val = m; std = sd; }
    }
    out[s.name] = { value: val, std: std * s.calib, stdRaw: std };
  }
  return out;
}

function predictAll(x) {
  const t0 = performance.now();
  const sc = predictScalars(x);
  const fields = {};
  for (const name of Object.keys(M.fields)) fields[name] = fieldPredict(M.fields[name], x);
  const hist = fieldPredict(M.hist, x);
  const t1 = performance.now();
  return { sc, fields, hist, inferMs: t1 - t0 };
}

// ── 시간 격자 (lib.py hist_times 와 같은 식) ───────────────────────
function histTimes(v, gap) {
  const hs = M.time_grid.heat_s, wp = M.time_grid.water_pts, cu = M.time_grid.cool_u;
  const sw0 = Math.max(gap - 20, 33);
  const t = [];
  for (const s of hs) t.push(s / v);
  for (let i = 1; i < 5; i++) t.push((hs[hs.length - 1] + (sw0 - hs[hs.length - 1]) * i / 5) / v);
  for (const s of wp) t.push((sw0 + s) / v);
  for (const u of cu) t.push((sw0 + 20) / v + u);
  for (let i = 0; i < t.length; i++) t[i] += 1e-6 * i;
  for (let i = 1; i < t.length; i++) t[i] = Math.max(t[i], t[i - 1]);
  return t;
}

// ── 화면 ──────────────────────────────────────────────────────────
function currentX() {
  return INPUT_NAMES.map((n) => {
    if (n === 'water') return $('in_water').checked ? 1 : 0;
    return parseFloat($('in_' + n).dataset.value);
  });
}
function fmt(v, d = 3) { if (!isFinite(v)) return '-'; const a = Math.abs(v); if (a !== 0 && (a < 0.01 || a >= 1e5)) return v.toExponential(2); return v.toFixed(d); }
function colorHeat(u) {
  const st = [[49, 54, 149], [69, 117, 180], [171, 217, 233], [254, 224, 144], [244, 109, 67], [165, 0, 38]];
  const x = Math.min(0.9999, Math.max(0, u)) * 5, i = Math.floor(x), f = x - i;
  return st[i].map((a, k) => Math.round(a + (st[i + 1][k] - a) * f));
}
function colorDiv(u) {   // -1 ~ 1, 파랑-흰-빨강
  const c = Math.max(-1, Math.min(1, u));
  const t = Math.abs(c);
  const a = c < 0 ? [59, 76, 192] : [180, 4, 38];
  return [Math.round(255 + (a[0] - 255) * t), Math.round(255 + (a[1] - 255) * t), Math.round(255 + (a[2] - 255) * t)];
}
function prepare(cv) {
  const r = cv.getBoundingClientRect(), s = devicePixelRatio || 1;
  if (cv.width !== Math.round(r.width * s)) { cv.width = Math.round(r.width * s); cv.height = Math.round(r.height * s); }
  const c = cv.getContext('2d'); c.setTransform(s, 0, 0, s, 0, 0);
  return { c, w: r.width, h: r.height };
}
// 값 배열 (ny, nz) → 열지도. 격자는 절점 또는 요소 중심. ymaxShow mm 까지만 표시.
function drawMap(cv, vals, ny, nz, opts) {
  const { c, w, h } = prepare(cv);
  c.clearRect(0, 0, w, h);
  const L = 42, B = 36, Tp = 8, R = 70;
  const ww = w - L - R, hh = h - B - Tp;
  const ymax = opts.ymaxShow;
  const nodes = opts.nodes;                // true: 절점 격자(양 끝 포함), false: 요소 중심
  const dy = 150 / (nodes ? ny - 1 : ny);
  // 격자 칸의 실제 위치(mm)를 축과 맞춘다. 절점은 절점을 중심으로, 요소는 요소 폭 그대로 칠하고 ymax 에서 자른다.
  const showN = Math.min(ny, Math.ceil(ymax / dy) + 1);
  const xOf = (y) => L + ww * Math.max(0, Math.min(ymax, y)) / ymax;
  const cellH = hh / nz;
  let lo = Infinity, hi = -Infinity;
  for (let j = 0; j < ny * nz; j++) { if (vals[j] < lo) lo = vals[j]; if (vals[j] > hi) hi = vals[j]; }
  if (opts.fixed) { lo = opts.fixed[0]; hi = opts.fixed[1]; }
  const div = opts.diverge;
  const sc = div ? Math.max(Math.abs(lo), Math.abs(hi), 1e-12) : (hi - lo || 1);
  for (let i = 0; i < showN; i++) {
    const y0 = nodes ? (i - 0.5) * dy : i * dy, y1 = nodes ? (i + 0.5) * dy : (i + 1) * dy;
    const x0 = xOf(y0), x1 = xOf(y1);
    if (x1 <= x0) continue;
    for (let k = 0; k < nz; k++) {
      const v = vals[i * nz + k];
      const col = div ? colorDiv(v / sc) : colorHeat((v - lo) / sc);
      c.fillStyle = `rgb(${col[0]},${col[1]},${col[2]})`;
      c.fillRect(x0, Tp + (nz - 1 - k) * cellH, x1 - x0 + 0.6, cellH + 0.6);
    }
  }
  c.strokeStyle = css('--line'); c.strokeRect(L, Tp, ww, hh);
  c.fillStyle = css('--sub'); c.font = '11px sans-serif';
  c.textAlign = 'center';
  for (let m = 0; m <= ymax; m += 20) { const X = L + ww * m / ymax; c.fillText(m, X, Tp + hh + 13); c.beginPath(); c.moveTo(X, Tp + hh); c.lineTo(X, Tp + hh + 3); c.stroke(); }
  c.fillText('가열선에서의 거리 mm', L + ww / 2, h - 4);
  c.textAlign = 'right'; c.fillText('윗면', L - 4, Tp + 10); c.fillText('아랫면', L - 4, Tp + hh);
  // 범례
  const Lx = w - R + 16, Lh = hh, Lw = 12;
  for (let p = 0; p < 40; p++) {
    const y = Tp + Lh * p / 40;
    const u = 1 - p / 39;
    const col = div ? colorDiv(2 * u - 1) : colorHeat(u);
    c.fillStyle = `rgb(${col[0]},${col[1]},${col[2]})`;
    c.fillRect(Lx, y, Lw, Lh / 40 + 1);
  }
  c.fillStyle = css('--sub'); c.textAlign = 'left'; c.font = '10.5px sans-serif';
  const fmtv = (v) => opts.scale ? (v * opts.scale).toFixed(opts.digits ?? 1) : v.toFixed(0);
  if (div) { c.fillText('+' + fmtv(sc), Lx + Lw + 3, Tp + 9); c.fillText('0', Lx + Lw + 3, Tp + Lh / 2 + 3); c.fillText('-' + fmtv(sc), Lx + Lw + 3, Tp + Lh); }
  else { c.fillText(fmtv(hi), Lx + Lw + 3, Tp + 9); c.fillText(fmtv(lo), Lx + Lw + 3, Tp + Lh); }
  if (opts.unit) c.fillText(opts.unit, Lx - 4, Tp + Lh + 16);
}
function drawTrace(cv, t, T1, T2, idx) {
  const { c, w, h } = prepare(cv);
  c.clearRect(0, 0, w, h);
  const L = 46, B = 36, Tp = 8, R = 14;
  const X0 = Math.min(t[0], -5), X1 = t[t.length - 1];
  const vmax = Math.max(...T1, 50) * 1.05;
  const X = (x) => L + (w - L - R) * (x - X0) / (X1 - X0), Y = (y) => h - B - (h - B - Tp) * y / vmax;
  c.strokeStyle = css('--line'); c.fillStyle = css('--sub'); c.font = '11px sans-serif';
  for (let g = 0; g <= 4; g++) { const y = vmax * g / 4; c.beginPath(); c.moveTo(L, Y(y)); c.lineTo(w - R, Y(y)); c.stroke(); c.textAlign = 'right'; c.fillText(y.toFixed(0), L - 4, Y(y) + 4); }
  // 시간축 눈금
  const span = X1 - X0, stepX = span > 240 ? 60 : span > 100 ? 25 : span > 40 ? 10 : 5;
  c.textAlign = 'center';
  for (let xt = Math.ceil(X0 / stepX) * stepX; xt <= X1 + 1e-9; xt += stepX) { const px = X(xt); c.beginPath(); c.moveTo(px, h - B); c.lineTo(px, h - B + 3); c.stroke(); c.fillText(String(xt), px, h - B + 14); }
  c.fillText('열원 통과 후 시간 (초)', (L + w - R) / 2, h - 4);
  const line = (arr, col, width) => { c.strokeStyle = col; c.lineWidth = width; c.beginPath(); for (let i = 0; i < t.length; i++) { const px = X(t[i]), py = Y(arr[i]); i ? c.lineTo(px, py) : c.moveTo(px, py); } c.stroke(); c.lineWidth = 1; };
  line(T1, css('--series-1'), 2); line(T2, css('--series-2'), 2);
  c.strokeStyle = css('--ink'); c.setLineDash([4, 3]); c.beginPath(); c.moveTo(X(t[idx]), Tp); c.lineTo(X(t[idx]), h - B); c.stroke(); c.setLineDash([]);
  // 범례는 곡선이 낮은 오른쪽 위에 둔다 (왼쪽 위는 온도 봉우리와 겹친다)
  c.textAlign = 'right';
  const lab1 = '윗면 중심', lab2 = '중간 두께 중심';
  const rx = w - R - 2, wid2 = c.measureText(lab2).width;
  c.fillStyle = css('--series-2'); c.fillText(lab2, rx, Tp + 12);
  c.fillStyle = css('--series-1'); c.fillText(lab1, rx - wid2 - 14, Tp + 12);
}

// 시간 프레임: 이벤트 격자 (nt, ny, nz) 에서 시각 idx 의 단면
function frame(arr, idx, ny, nz) {
  const start = idx * ny * nz;
  return arr.subarray(start, start + ny * nz);
}

function render(pred) {
  const t0 = performance.now();
  const { sc, fields, hist } = pred;
  const x = state.x;
  const ny = M.const.ny_nodes, nz = M.const.nz_nodes;
  // 스칼라 카드
  for (const s of M.scalars) {
    const r = sc[s.name];
    const el = $('v_' + s.name);
    if (!el) continue;
    let digits = 3;
    if (s.name === 'theta') digits = 2;
    if (s.name === 'q') digits = 0;
    el.textContent = fmt(r.value, digits);
    const relEl = $('u_' + s.name), barEl = $('b_' + s.name);
    if (relEl) {
      const sigma2 = 2 * r.std;
      const rel = Math.abs(r.value) > 1e-12 ? sigma2 / Math.abs(r.value) : 0;
      relEl.textContent = `±${fmt(sigma2, digits)} ${s.unit} (2σ, ${rel < 10 ? Math.round(rel * 100) + '%' : '100%↑'})`;
      barEl.style.width = Math.min(100, rel * 100) + '%';
      barEl.style.background = rel > 0.5 ? css('--hot') : rel > 0.2 ? css('--warn') : css('--acc');
    }
  }
  // 참고 표시: 고유변형 적분으로 환산한 3성분 (위 카드의 스트립 변위 기준 값과 정의가 다르다)
  const t = x[0];
  const Bt = sc.B_t.value, St = sc.S_t.value, Sl = sc.S_l.value;
  $('der_theta').textContent = fmt(1000 * Bt * 12 / (t * t * t), 2);
  $('der_trans').textContent = fmt(-St / t, 4);
  $('der_long').textContent = fmt(-Sl * 400 / (300 * t), 4);
  // 필드
  const Tmax = fields.Tmax;
  let tmaxHi = -Infinity;
  for (let i = 0; i < Tmax.length; i++) if (Tmax[i] > tmaxHi) tmaxHi = Tmax[i];
  $('v_tmax').textContent = fmt(tmaxHi, 0);
  drawMap($('map_tmax'), Tmax, ny, nz, { nodes: true, ymaxShow: 100, unit: '℃' });
  const nye = M.const.ny_el, nze = M.const.nz_el;
  const scaleEp = 1000;
  drawMap($('map_epy'), fields.epy, nye, nze, { nodes: false, ymaxShow: 100, diverge: true, scale: scaleEp, digits: 2, unit: '×10⁻³' });
  drawMap($('map_epx'), fields.epx, nye, nze, { nodes: false, ymaxShow: 100, diverge: true, scale: scaleEp, digits: 2, unit: '×10⁻³' });
  // 시간
  const nt = M.time_grid.n_t;
  const tm = histTimes(x[1], x[4]);
  const idx = Math.max(0, Math.min(nt - 1, state.timeIndex));
  $('time_val').textContent = `${tm[idx] >= 0 ? '+' : ''}${tm[idx].toFixed(1)} 초`;
  const frameT = frame(hist, idx, ny, nz);
  drawMap($('map_hist'), frameT, ny, nz, { nodes: true, ymaxShow: 100, unit: '℃', fixed: [20, Math.max(tmaxHi, 100)] });
  // 두께 절점이 짝수 개(6개)면 정확히 중간인 절점이 없으므로 가운데 두 절점의 평균을 쓴다 (z 인덱스는 0 이 아랫면, 끝이 윗면)
  const zMidA = Math.floor((nz - 1) / 2), zMidB = Math.ceil((nz - 1) / 2);
  const topCenter = new Array(nt), midCenter = new Array(nt);
  for (let i = 0; i < nt; i++) { topCenter[i] = hist[i * ny * nz + (nz - 1)]; midCenter[i] = 0.5 * (hist[i * ny * nz + zMidA] + hist[i * ny * nz + zMidB]); }
  drawTrace($('trace'), tm, topCenter, midCenter, idx);
  state.drawMs.push(performance.now() - t0);
}

function showTiming() {
  const avg = (a) => a.length ? a.reduce((p, q) => p + q, 0) / a.length : 0;
  const last = (a) => a.length ? a[a.length - 1] : 0;
  const firstDraw = state.drawMs.length <= 1;     // 페이지를 열고 처음 그릴 때는 글꼴·캔버스 준비가 포함되어 조작 중 응답 시간이 아니다
  $('timing').textContent = `추론 ${last(state.inferMs).toFixed(1)} ms (최근 평균 ${avg(state.inferMs.slice(-30)).toFixed(1)} ms) · 화면 그리기 ${last(state.drawMs).toFixed(1)} ms${firstDraw ? ' (처음 그리기, 조작 시작 전)' : ''}`;
  const budget = last(state.inferMs) + last(state.drawMs);
  $('timing').style.color = (budget > 50 && !firstDraw) ? css('--hot') : css('--sub');
}

function checkWarnings() {
  const x = state.x;
  const box = $('warn');
  const msgs = [];
  for (let i = 0; i < M.inputs.length; i++) {
    const inp = M.inputs[i];
    if (inp.kind !== 'num') continue;
    if (x[6 + 1] === 0 && (inp.name === 'gap' || inp.name === 'h_water')) continue;
    const v = x[i];
    if (v < inp.train_min - 1e-9 || v > inp.train_max + 1e-9) msgs.push(`${inp.label}: ${v.toFixed(2)} 은 학습 범위(${inp.train_min}~${inp.train_max}) 밖입니다.`);
  }
  if (x[7] === 0) msgs.push('수냉 없음 조건은 학습 표본의 약 10%뿐입니다. 수냉 있음보다 불확실합니다.');
  // 해석기가 수렴하지 못해 학습에 없는 입력 근처 (두께·목표 최고온도 기준). 정보가 없는 모델 파일이면 아무것도 표시하지 않는다.
  const fails = ((M.metrics || {}).data || {}).failed_inputs || [];
  const near = fails.filter((f) => f.kw && Math.abs(f.kw.thickness - x[0]) <= 1.5 && Math.abs(f.kw.peak_target - x[2]) <= 100);
  if (near.length) msgs.push(`이 조건 근처(두께·목표 최고온도 기준)에서 해석기가 수렴하지 못한 사례가 ${near.length}건 있어 그 입력은 학습에 없습니다.`);
  if (msgs.length) { box.style.display = 'block'; box.innerHTML = '<b>주의</b><br>' + msgs.join('<br>'); }
  else { box.style.display = 'none'; }
}

let pending = false;
function schedule() {
  state.x = currentX();
  updateLabels();
  checkWarnings();
  if (pending) return;
  pending = true;
  requestAnimationFrame(() => {
    pending = false;
    const pred = predictAll(state.x);
    state.lastPred = pred;
    state.inferMs.push(pred.inferMs);
    if (state.inferMs.length > 200) state.inferMs.shift();
    render(pred);
    showTiming();
    // 판 이미지(plate_preview.js)에 지금 조건과 값을 알린다.
    window.aiLatest = { x: state.x, sc: pred.sc };
    window.dispatchEvent(new CustomEvent('ai-prediction', { detail: window.aiLatest }));
  });
}

function updateLabels() {
  for (const inp of M.inputs) {
    if (inp.kind !== 'num') continue;
    const el = $('in_' + inp.name), lab = $('lab_' + inp.name);
    const v = parseFloat(el.dataset.value);
    lab.textContent = inp.name === 'h_water' ? v.toFixed(2) : v.toFixed(inp.step < 0.5 ? 2 : inp.step < 1 ? 1 : 0);
    const r = $('range_' + inp.name);
    r.classList.toggle('out', v < inp.train_min - 1e-9 || v > inp.train_max + 1e-9);
  }
  const off = !$('in_water').checked;
  $('row_gap').style.opacity = off ? 0.45 : 1;
  $('row_h_water').style.opacity = off ? 0.45 : 1;
}

function buildControls() {
  const box = $('controls_inner');
  for (const inp of M.inputs) {
    if (inp.kind !== 'num') continue;
    const row = document.createElement('div');
    row.className = 'ctl'; row.id = 'row_' + inp.name;
    const lo = inp.log ? Math.log(inp.min) : inp.min, hi = inp.log ? Math.log(inp.max) : inp.max;
    const vv = inp.log ? Math.log(inp.value) : inp.value;
    const step = inp.log ? (hi - lo) / 200 : inp.step;
    row.innerHTML = `<label for="in_${inp.name}">${inp.label}</label><span class="val" id="lab_${inp.name}"></span>`
      + `<input type="range" id="in_${inp.name}" min="${lo}" max="${hi}" step="${step}" value="${vv}" data-value="${inp.value}">`
      + `<span class="range" id="range_${inp.name}">학습 범위 ${inp.train_min}~${inp.train_max}</span>`;
    box.appendChild(row);
    const el = row.querySelector('input');
    el.addEventListener('input', () => {
      let v = parseFloat(el.value);
      if (inp.log) v = Math.exp(v);
      el.dataset.value = v;
      schedule();
    });
  }
  $('in_water').addEventListener('change', schedule);
}

function initTimeSlider() {
  const s = $('time_slider');
  s.max = M.time_grid.n_t - 1;
  s.value = state.timeIndex;
  s.addEventListener('input', () => { state.timeIndex = parseInt(s.value, 10); if (state.lastPred) { const t = performance.now(); render(state.lastPred); state.drawMs[state.drawMs.length - 1] = performance.now() - t; showTiming(); } });
  $('play').addEventListener('click', () => {
    if (state.playTimer) { clearInterval(state.playTimer); state.playTimer = null; $('play').textContent = '자동 재생'; return; }
    $('play').textContent = '멈춤';
    state.playTimer = setInterval(() => {
      state.timeIndex = (state.timeIndex + 1) % M.time_grid.n_t;
      s.value = state.timeIndex;
      if (state.lastPred) render(state.lastPred);
    }, 180);
  });
}

// ── 시작 ───────────────────────────────────────────────────────────
async function main() {
  try {
    const meta = await (await fetch(MODEL_DIR + '/model.json', { cache: 'no-store' })).json();
    const bin = await (await fetch(MODEL_DIR + '/' + meta.model_bin, { cache: 'no-store' })).arrayBuffer();
    M = meta; BUF = bin;
  } catch (e) {
    $('warn').style.display = 'block';
    $('warn').textContent = '모델 파일을 불러오지 못했습니다. 잠시 뒤 다시 열어 주세요.';
    return;
  }
  for (const s of M.scalars) { const el = $('card_' + s.name); if (el) el.style.display = ''; }
  buildControls();
  initTimeSlider();
  fillModelInfo();
  schedule();
  // 창 크기가 바뀌면 그림을 새 크기로 다시 그린다 (그대로 두면 그림이 늘어나 글자가 찌그러진다)
  let resizePending = false;
  window.addEventListener('resize', () => {
    if (resizePending) return;
    resizePending = true;
    requestAnimationFrame(() => { resizePending = false; if (state.lastPred) render(state.lastPred); });
  });
}
function shortMetric(a, d) {
  if (!a || a.mean == null) return '-';
  const sh = (v, n) => { const x = Math.abs(v); if (x !== 0 && (x < 1e-3 || x >= 1e5)) return v.toExponential(1); return v.toPrecision(n); };
  return sh(a.mean, d) + (a.std != null ? ' ± ' + sh(a.std, 1) : '');
}
function fillModelInfo() {
  const d = M.data;
  const fams = { gp: 'GP', gp_aug: 'GP+온도표본', mlp: 'MLP', pod_gp: 'POD·GP', pod_mlp: 'POD·MLP', coord: '좌표망' };
  $('info_n').textContent = `가상 해석 표본 ${d.n_D}건 (학습 ${M.created})`;
  const box = $('data_notice');
  const notes = [];
  if (d.n_D < 64) notes.push(`학습 표본이 ${d.n_D}건으로 적습니다. 시연 수준이므로 지표와 값을 인용하지 마세요.`);
  for (const w of M.warnings || []) if (!(d.n_D < 64 && w.startsWith('D_ 표본이'))) notes.push(w);
  if (notes.length) { box.style.display = 'block'; box.innerHTML = notes.join('<br>'); }
  const rows = [];
  const met = M.metrics || {};
  const lab = { theta: '각변형', dtrans: '횡수축', dlong: '종수축', q: '열속 q', S_t: 'S_t', B_t: 'B_t', S_l: 'S_l' };
  for (const s of M.scalars) {
    const m = (met.scalars || {})[s.name];
    if (!m) continue;
    const fmt2 = shortMetric;
    rows.push(`<tr><td>${lab[s.name] || s.name}</td><td>${fmt2(m.r2, 4)}</td><td>${fmt2(m.mae, 3)} ${m.unit}</td><td>${(fams[m.family] || m.family)}</td></tr>`);
  }
  $('info_table').innerHTML = rows.length ? `<table><thead><tr><th>출력</th><th>R²</th><th>절대오차 MAE</th><th>모델</th></tr></thead><tbody>${rows.join('')}</tbody></table>` : '<p class="hint">시험 성능은 표본이 쌓인 뒤 표시됩니다.</p>';
  const tf = [];
  for (const k of ['Tmax', 'epy', 'epx']) {
    const m = (met.fields || {})[k];
    if (!m) continue;
    const name = { Tmax: '최고온도 ℃', epy: '소성변형 yy', epx: '소성변형 xx' }[k];
    const fmt2 = shortMetric;
    tf.push(`<tr><td>${name}</td><td>${fmt2(m.r2, 4)}</td><td>${fmt2(m.mae, 3)}</td><td>${fams[m.family] || m.family}</td></tr>`);
  }
  if (met.thist) { const fmt2 = shortMetric; tf.push(`<tr><td>시간별 단면 온도 ℃</td><td>${fmt2(met.thist.r2, 4)}</td><td>${fmt2(met.thist.mae, 3)}</td><td>${fams[met.thist.family] || met.thist.family}</td></tr>`); }
  $('info_field').innerHTML = tf.length ? `<table><thead><tr><th>필드</th><th>R²</th><th>절대오차 MAE</th><th>모델</th></tr></thead><tbody>${tf.join('')}</tbody></table>` : '';
}
main();

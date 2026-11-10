// 뷰어 계산 패널: 불러온 판에 대해 변형 계산, 가열선 추천, 가열선 라벨을 수행한다.
// 모든 계산은 Web Worker 안에서 하며 파일과 결과는 네트워크로 나가지 않는다.
import * as THREE from 'three';
import { flatten } from '../studio/solver.js';
import { zPlaneFree, rms } from '../studio/planner.js';

const viewer = window.heatroViewer;
const worker = new Worker(new URL('../studio/worker.js', import.meta.url), { type: 'module' });
const overlay = new THREE.Group();
viewer.scene.add(overlay);

let builtFor = null;
let busy = false;
let calibration = null;
let exag = 1;
let last = null;          // 마지막 계산 결과. 높이 배율을 바꾸면 이것으로 다시 그린다
let shownLayers = null;   // 배율 보기로 들어가기 전의 레이어 켜짐 상태
const TRUE_SCALE_LAYERS = ['source', 'target', 'deviation', 'designEdges', 'measureEdges', 'curves'];

const panel = document.createElement('section');
panel.className = 'panel';
panel.id = 'simPanel';
panel.innerHTML = `
  <h2>변형 계산, 가열선 추천</h2>
  <p class="hint">시험 기능입니다. 해석 모델의 값이며 실측 정확도를 보장하지 않습니다. 삼각가열 선은 계산에서 제외합니다.</p>
  <div id="simOut" class="meta-block"></div>
  <div class="row slider-row"><label for="simExag">높이 배율</label><input type="range" id="simExag" min="1" max="60" step="1" value="1"><span id="simExagVal" class="num">×1</span></div>
  <div class="sim-grid">
    <label>각변형 mrad<input id="simTheta" type="number" step="0.1" value="5"></label>
    <label>횡수축 mm<input id="simTrans" type="number" step="0.001" value="0.02"></label>
    <label>종수축 mm<input id="simLong" type="number" step="0.001" value="0.025"></label>
    <label>강도 배수<input id="simGain" type="number" step="0.5" min="0" value="1"></label>
  </div>
  <p class="hint" id="simCalState">예시값 사용 중</p>
  <div class="row">
    <label class="btn-file">보정표 불러오기<input id="simCal" type="file" accept=".json" hidden></label>
    <button id="simRun">변형 계산</button>
  </div>
  <div class="sim-grid">
    <label>최대 줄 수<input id="simMax" type="number" min="1" max="30" value="6"></label>
    <label>최소 간격 mm<input id="simGap" type="number" step="10" value="150"></label>
    <label>줄별 최대 배수<input id="simW" type="number" step="0.5" min="0.5" value="1.5"></label>
    <button id="simRec">가열선 추천</button>
  </div>
  <div class="panelhead"><h2>가열선 라벨</h2><span id="labelCount" class="hint"></span></div>
  <p class="hint">질문: 지금 이 판에 이 선을 쓰겠는가? 연구진 라벨은 숙련공 판단과 구분해 기록합니다.</p>
  <div id="labelList"></div>
  <div class="row"><button id="labelSave">라벨 파일로 저장</button></div>`;
document.getElementById('sidebar').insertBefore(panel, document.getElementById('datasetsPanel'));

const style = document.createElement('style');
style.textContent = `.sim-grid{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin:6px 0}.sim-grid label{display:grid;gap:2px;font-size:12px}.sim-grid input{width:100%}#simOut table{width:100%;border-collapse:collapse;font-size:12px}#simOut td,#simOut th{padding:2px 4px;text-align:right}#simOut td:first-child,#simOut th:first-child{text-align:left}.lab{border-top:1px solid var(--color-rule);padding:4px 0;font-size:12px}.lab select,.lab input{font-size:12px;margin-right:4px}.lab input{width:96%;margin-top:3px}`;
document.head.appendChild(style);

const $ = (id) => document.getElementById(id);
const num = (id) => parseFloat($(id).value) || 0;
const out = (html) => { $('simOut').innerHTML = html; };

function ask(message, doneType) {
  return new Promise((resolve, reject) => {
    worker.onmessage = ({ data }) => {
      if (data.type === 'error') reject(new Error(data.message));
      else if (data.type === doneType) resolve(data);
    };
    worker.postMessage(message);
  });
}

async function guard(label, job) {
  if (busy) return;
  const d = viewer.getPrimary();
  if (!d) { out('먼저 판 데이터를 불러오세요.'); return; }
  busy = true;
  out(`${label} 중...`);
  try { await job(d); } catch (err) { out(`오류: ${err.message}`); } finally { busy = false; }
}

async function ensureModel(d) {
  if (builtFor === d) return;
  await ask({ type: 'build', points: d.sourceGrid.points, thickness: d.thickness, refine: 2 }, 'built');
  builtFor = d;
}

function componentFor(d, curve) {
  if (calibration) {
    const peak = Math.max(...(curve?.paramTemp ?? [0]));
    let best = null, bestScore = Infinity;
    for (const e of calibration.entries) {
      const score = Math.abs(e.thickness_mm - d.thickness) * 10 + Math.abs(e.speed - (curve?.speed ?? e.speed)) + Math.abs(e.peak_c - peak) / 100;
      if (score < bestScore) { bestScore = score; best = e; }
    }
    if (best) return { theta_mrad: best.theta_mrad, dtrans_mm: best.dtrans_mm, dlong_mm: best.dlong_mm };
  }
  return { theta_mrad: -Math.abs(num('simTheta')), dtrans_mm: num('simTrans'), dlong_mm: num('simLong') };
}

function targetGap(d) {
  const X = flatten(d.sourceGrid.points), T = flatten(d.targetGrid.points);
  const diff = new Float64Array(X.length);
  for (let i = 0; i < X.length; i++) diff[i] = T[i] - X[i];
  return { X, D: zPlaneFree(diff, X) };
}

function remaining(D, P, k) {
  const e = new Float64Array(D.length);
  let over = 0;
  for (let i = 0; i < D.length; i++) {
    e[i] = D[i] - k * P[i];
    if (Math.sign(e[i]) !== Math.sign(D[i]) && Math.abs(e[i]) > 3) over++;
  }
  return { rms: rms(e), over: over / D.length };
}

function clearOverlay() {
  while (overlay.children.length) {
    const c = overlay.children.pop();
    c.geometry?.dispose();
    c.material?.dispose();
  }
}

function heat(t) {
  const c = new THREE.Color();
  c.setHSL(0.66 * (1 - Math.min(1, Math.max(0, t))), 0.85, 0.5);
  return c;
}

function gridGeometry(d, pos, col) {
  const nr = d.sourceGrid.points.length, nc = d.sourceGrid.points[0].length;
  const index = [];
  for (let i = 0; i < nr - 1; i++) for (let j = 0; j < nc - 1; j++) {
    const a = i * nc + j, b = a + 1, c = a + nc, e = c + 1;
    index.push(a, c, b, b, c, e);
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  if (col) geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
  geo.setIndex(index);
  geo.computeVertexNormals();
  return geo;
}

function residualColors(residual) {
  const col = new Float32Array(residual.length * 3);
  const top = Math.max(1e-6, ...residual.map(Math.abs));
  residual.forEach((v, i) => {
    const c = heat(Math.abs(v) / top);
    col[3 * i] = c.r; col[3 * i + 1] = c.g; col[3 * i + 2] = c.b;
  });
  return col;
}

function drawResult(d, X, disp, residual) {
  const pos = new Float32Array(X.length);
  for (let i = 0; i < X.length; i++) pos[i] = X[i] + disp[i];
  overlay.add(new THREE.Mesh(gridGeometry(d, pos, residualColors(residual)), new THREE.MeshStandardMaterial({ vertexColors: true, side: THREE.DoubleSide, roughness: 0.8 })));
}

// 높이를 과장한 보기: 굽은 정도가 눈에 보이도록 z 만 배율을 곱한다. 수치는 바꾸지 않는다.
function drawExaggerated({ d, X, P, D, residual, lines }) {
  const n = X.length / 3;
  const pos = new Float32Array(X), tgt = new Float32Array(X);
  for (let i = 0; i < n; i++) {
    pos[3 * i + 2] = (X[3 * i + 2] + P[i]) * exag;
    tgt[3 * i + 2] = (X[3 * i + 2] + D[i]) * exag;
  }
  overlay.add(new THREE.Mesh(gridGeometry(d, pos, residualColors(residual)), new THREE.MeshStandardMaterial({ vertexColors: true, side: THREE.DoubleSide, roughness: 0.8 })));
  overlay.add(new THREE.Mesh(gridGeometry(d, tgt), new THREE.MeshBasicMaterial({ color: 0xf7b83d, wireframe: true, transparent: true, opacity: 0.35 })));
  const nearest = (p) => {
    let best = 0, gap = Infinity;
    for (let i = 0; i < n; i++) {
      const dd = (X[3 * i] - p[0]) ** 2 + (X[3 * i + 1] - p[1]) ** 2;
      if (dd < gap) { gap = dd; best = i; }
    }
    return best;
  };
  for (const line of lines) drawLine(line.points.map((p) => [p[0], p[1], (p[2] + P[nearest(p)]) * exag]), line.color);
}

function drawView() {
  clearOverlay();
  if (!last) return;
  if (exag > 1) drawExaggerated(last);
  else drawResult(last.d, last.X, last.disp, last.residual);
}

function setExag(value) {
  const next = Math.max(1, Math.min(60, Math.round(value) || 1));
  if (next > 1 && exag === 1) {
    shownLayers = {};
    for (const id of TRUE_SCALE_LAYERS) {
      const cb = $('cb-' + id);
      if (!cb) continue;
      shownLayers[id] = cb.checked;
      if (cb.checked) cb.click();
    }
  } else if (next === 1 && exag > 1 && shownLayers) {
    for (const id of TRUE_SCALE_LAYERS) {
      const cb = $('cb-' + id);
      if (cb && cb.checked !== shownLayers[id]) cb.click();
    }
    shownLayers = null;
  }
  exag = next;
  $('simExag').value = String(next);
  $('simExagVal').textContent = `×${next}`;
}

function drawLine(points, color) {
  const v = points.map((p) => new THREE.Vector3(p[0], p[1], p[2] + 14));
  const geo = new THREE.TubeGeometry(new THREE.CatmullRomCurve3(v), Math.max(10, v.length * 2), 16, 6);
  overlay.add(new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color })));
}

function closePairs(curves, limit) {
  let n = 0;
  for (let a = 0; a < curves.length; a++) for (let b = a + 1; b < curves.length; b++) {
    if (curves[a].points.some((p) => curves[b].points.some((q) => Math.hypot(p[0] - q[0], p[1] - q[1]) < limit))) n++;
  }
  return n;
}

$('simCal').addEventListener('change', async (e) => {
  const file = e.target.files[0];
  if (!file) return;
  try {
    const parsed = JSON.parse((await file.text()).replace(/^﻿/, ''));
    if (!Array.isArray(parsed.entries) || !parsed.entries.length) throw new Error('entries 없음');
    calibration = parsed;
    $('simCalState').textContent = `보정표 사용 중 (${parsed.entries.length}개 조건, 이 브라우저 안에서만 사용)`;
  } catch (err) { $('simCalState').textContent = `보정표를 읽지 못했습니다: ${err.message}`; }
});

const runDeformation = () => guard('변형 계산', async (d) => {
  await ensureModel(d);
  const used = d.bendingCurves.filter((c) => c.heatingMethod === 1 && c.points.length >= 2);
  const gain = num('simGain');
  // 가상 예시는 추천 때 정한 줄별 세기를 그대로 쓴다. 실제 파일의 Strength 는 뜻이 정해지지 않아 쓰지 않는다.
  const lines = used.map((c) => ({ points: c.points, comp: componentFor(d, c), scale: gain * (d.deviceCode === 'DEMO' ? (c.strength ?? 1) : 1) }));
  const res = await ask({ type: 'run', lines, id: 1 }, 'result');
  const { X, D } = targetGap(d);
  const P = zPlaneFree(res.disp, X);
  const residual = D.map((v, i) => v - P[i]);
  last = { d, X, disp: res.disp, P, D, residual, lines: used.map((c) => ({ points: c.points, color: 0xf7b83d })) };
  drawView();
  const base = rms(D);
  const rows = [0.5, 1, 1.5, 2].map((k) => {
    const r = remaining(D, P, k);
    return `<tr><td>세기 ×${k}</td><td>${r.rms.toFixed(2)}</td><td>${(r.rms / base).toFixed(3)}</td><td>${(100 * r.over).toFixed(1)}%</td></tr>`;
  }).join('');
  const near = closePairs(used, 50);
  out(`계산 ${used.length}줄, ${res.ms.toFixed(0)} ms, 시작 편차 RMS ${base.toFixed(2)} mm
    <table><tr><th>조건</th><th>남은 RMS mm</th><th>비율</th><th>과가공</th></tr>${rows}</table>
    ${near ? `<p class="hint">50 mm 안으로 가까운 가열선 ${near}쌍: 단순 합 계산의 오차가 커질 수 있습니다.</p>` : ''}
    <p class="hint">세기 표는 가열 세기가 틀렸을 때의 영향을 보여 줍니다. 과가공은 목표를 3 mm 넘게 지나친 격자점 비율입니다.</p>
    <p class="hint">표면 색은 세기 ×1 뒤 남은 편차입니다(파랑 0, 빨강 최대 ${Math.max(...residual.map(Math.abs)).toFixed(2)} mm).${exag > 1 ? ` 높이는 ×${exag}로 과장해 그렸고, 호박색 격자가 목표 곡면입니다.` : ''}</p>`);
});
$('simRun').addEventListener('click', runDeformation);
viewer.runDeformation = async () => {
  while (busy) await new Promise((resolve) => setTimeout(resolve, 30));
  await runDeformation();
};

$('simRec').addEventListener('click', () => guard('가열선 추천', async (d) => {
  await ensureModel(d);
  const first = d.bendingCurves.find((c) => c.heatingMethod === 1);
  const { D } = targetGap(d);
  const options = { maxLines: num('simMax'), minGap: num('simGap'), wmax: num('simW') * Math.max(num('simGain'), 1e-6), pitch: 150, families: ['across'] };
  const comp = componentFor(d, first);
  const res = await ask({ type: 'recommend', D, comp, options }, 'recommended');
  if (exag > 1) {
    const run = await ask({ type: 'run', lines: res.lines.map((l) => ({ points: l.points, comp, scale: l.weight })), id: 2 }, 'result');
    const X = flatten(d.sourceGrid.points);
    const P = zPlaneFree(run.disp, X);
    last = { d, X, disp: run.disp, P, D, residual: D.map((v, i) => v - P[i]), lines: res.lines.map((l) => ({ points: l.points, color: 0x12a150 })) };
    drawView();
  } else {
    last = null;
    clearOverlay();
    res.lines.forEach((l) => drawLine(l.points, 0x12a150));
  }
  const list = res.lines.map((l, i) => `<tr><td>${i + 1}</td><td>${l.deg}°</td><td>${l.weight.toFixed(2)}</td></tr>`).join('');
  out(`후보 ${res.candidates}개 중 ${res.lines.length}줄, ${(res.ms / 1000).toFixed(1)}초<br>
    남은 편차 RMS ${res.rmsBefore.toFixed(2)} → ${res.rmsAfter.toFixed(2)} mm (해석 모델 기준)
    <table><tr><th>순번</th><th>각도</th><th>세기 배수</th></tr>${list}</table>
    <p class="hint">작업자가 검토하는 참고안입니다. 현장 검증 전이며, 1회분을 적용한 뒤 다시 스캔해 계산하는 사용을 권장합니다.</p>`);
}));

// ── 가열선 라벨 ─────────────────────────────────────────
const KEY = 'heatro.viewer.labels.v1';
let labels = {};
try { labels = JSON.parse(localStorage.getItem(KEY) || '{}'); } catch { labels = {}; }
const persist = () => { try { localStorage.setItem(KEY, JSON.stringify(labels)); } catch { /* 파일 저장으로 보관 */ } };
const idOf = (d) => d.legacyId || d.fileName;

function direction(d, c) {
  const g = d.sourceGrid.points, m = g[0].length >> 1;
  const ux = g[g.length - 1][m][0] - g[0][m][0], uy = g[g.length - 1][m][1] - g[0][m][1];
  const a = c.points[0], b = c.points[c.points.length - 1], dx = b[0] - a[0], dy = b[1] - a[1];
  const cos = Math.abs(dx * ux + dy * uy) / ((Math.hypot(ux, uy) || 1) * (Math.hypot(dx, dy) || 1));
  return cos > 0.87 ? '길이방향' : cos < 0.26 ? '폭방향' : '사선';
}

function renderLabels() {
  const d = viewer.getPrimary();
  const box = $('labelList');
  box.innerHTML = '';
  if (!d) { $('labelCount').textContent = ''; return; }
  const store = (labels[idOf(d)] ??= {});
  const count = () => { $('labelCount').textContent = `${d.bendingCurves.filter((c) => store[c.curveId]?.verdict).length} / ${d.bendingCurves.length}`; };
  d.bendingCurves.forEach((c) => {
    const row = document.createElement('div');
    row.className = 'lab';
    const v = store[c.curveId] ?? {};
    const tags = `${c.heatingMethod === 2 ? '삼각가열' : '선상가열'}, ${c.isAutoMaked ? '자동' : '수동'}, ${direction(d, c)}`;
    row.innerHTML = `<b>${String(c.curveId).replace(/[<>&]/g, '')}</b> <span class="hint">${tags}</span><br>
      <select data-k="verdict"><option value="">판단 안 함</option><option>쓴다</option><option>옮긴다</option><option>뺀다</option></select>
      <select data-k="by"><option>연구진</option><option>숙련공</option></select>
      <input data-k="note" placeholder="메모">`;
    row.querySelectorAll('[data-k]').forEach((el) => {
      if (v[el.dataset.k]) el.value = v[el.dataset.k];
      el.addEventListener('change', () => {
        store[c.curveId] = { ...store[c.curveId], by: row.querySelector('[data-k=by]').value, [el.dataset.k]: el.value, at: new Date().toISOString() };
        persist(); count();
      });
    });
    box.appendChild(row);
  });
  count();
}

$('labelSave').addEventListener('click', () => {
  const url = URL.createObjectURL(new Blob([JSON.stringify({ schema: 1, labels }, null, 1)], { type: 'application/json' }));
  const a = Object.assign(document.createElement('a'), { href: url, download: 'curve_labels.json' });
  a.click();
  URL.revokeObjectURL(url);
});

$('simExag').addEventListener('input', (e) => { setExag(Number(e.target.value)); drawView(); });

window.addEventListener('heatro:reload', () => {
  builtFor = null; last = null; clearOverlay(); out(''); renderLabels();
  setExag(viewer.getPrimary()?.deviceCode === 'DEMO' ? 20 : 1);
});
renderLabels();

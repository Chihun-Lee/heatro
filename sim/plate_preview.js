// 시뮬레이터 화면의 판 이미지: 왼쪽 조건의 AI 값으로 판에 가열선을 넣었을 때의 변형을 바로 그린다.
// 판 해석은 곡면 가공 화면과 같은 엔진(engine/worker_ext.js)을 쓴다.
// 줄이 몇 개든 하중을 더해 한 번만 풀므로 슬라이더를 움직여도 바로 갱신된다. 값은 해석값이며 실측이 아니다.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { flatten, removeRigid } from './engine/solver_ext.js';
import { signedComp } from './engine/forming_tools.js';
import { zPlaneFree } from '../studio/planner.js';

const $ = (id) => document.getElementById(id);
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const fix = (v, d) => (Number.isFinite(v) ? v.toFixed(d) : '-');

// 가상 평판. x 는 길이, y 는 폭, 원점은 판의 한 모서리다.
const L = 3000, W = 2000, DX = 50;
const NR = Math.round(L / DX) + 1, NC = Math.round(W / DX) + 1, NV = NR * NC;
const points = Array.from({ length: NR }, (_, i) => Array.from({ length: NC }, (_, j) => [i * DX, j * DX, 0]));
const X = flatten(points);
const MIN_LEN = 80, MAX_LINES = 40, NEAR_MM = 50;

const S = {
  lines: [], nextId: 1, m: null, thickness: null, builtFor: null,
  field: null, exag: 30, mode: null, first: null, drawing: null,
  running: false, again: false, ms: null,
};

function setMsg(text, isErr = false) {
  const m = $('pvMsg');
  m.textContent = text || '';
  m.classList.toggle('err', isErr);
}

// ── 해석 작업자 ─────────────────────────────────────────
const worker = new Worker('engine/worker_ext.js', { type: 'module' });
worker.onerror = (e) => setMsg(`판 해석을 시작하지 못했습니다: ${e.message || ''}`, true);
let chain = Promise.resolve();
function call(message, doneType) {
  const run = () => new Promise((resolve, reject) => {
    worker.onmessage = ({ data }) => {
      if (data.type === 'error') reject(new Error(data.message));
      else if (data.type === doneType) resolve(data);
    };
    worker.postMessage(message);
  });
  const p = chain.then(run, run);
  chain = p.catch(() => {});
  return p;
}

async function solveNow() {
  const key = S.thickness.toFixed(2);
  if (S.builtFor !== key) {
    await call({ type: 'build', points, thickness: S.thickness, refine: 1 }, 'built');
    S.builtFor = key;
  }
  if (!S.lines.length) { S.field = null; S.ms = null; return; }
  const lines = S.lines.map((l) => ({ points: l.points, comp: signedComp(S.m, l.side, null), scale: l.weight }));
  const out = await call({ type: 'run', lines, id: 1 }, 'result');
  const z = zPlaneFree(out.disp, X);
  const free = removeRigid(out.disp, X);
  for (let i = 0; i < z.length; i++) free[3 * i + 2] = z[i];
  S.field = free;
  S.ms = out.ms;
}

// 슬라이더를 끄는 동안 쌓이는 요청은 하나로 합친다.
async function solve() {
  if (!S.m) return;
  if (S.running) { S.again = true; return; }
  S.running = true;
  try {
    do {
      S.again = false;
      await solveNow();
      writeSurface();
    } while (S.again);
    $('pvMsg').classList.contains('err') || setMsg('');
  } catch (err) {
    setMsg(`판 계산 오류: ${err.message}`, true);
  } finally {
    S.running = false;
  }
}

// ── 3D 화면 ─────────────────────────────────────────────
const host = $('pvHost');
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(36, 1, 10, 60000);
camera.up.set(0, 0, 1);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
const canvas = renderer.domElement;
host.prepend(canvas);
const controls = new OrbitControls(camera, canvas);
controls.mouseButtons = { LEFT: THREE.MOUSE.ROTATE, MIDDLE: THREE.MOUSE.PAN, RIGHT: THREE.MOUSE.PAN };
scene.add(new THREE.HemisphereLight(0xffffff, 0xb6bcc6, 1.9));
const sun = new THREE.DirectionalLight(0xffffff, 1.5);
sun.position.set(0.45, -0.7, 1);
scene.add(sun);

let drawPending = false;
function requestDraw() {
  if (drawPending) return;
  drawPending = true;
  requestAnimationFrame(() => { drawPending = false; renderer.render(scene, camera); });
}
controls.addEventListener('change', requestDraw);

const world = new THREE.Group();
world.position.set(-L / 2, -W / 2, 0);
scene.add(world);

const geo = new THREE.BufferGeometry();
{
  const pos = new Float32Array(3 * NV), uv = new Float32Array(2 * NV), index = [];
  for (let i = 0; i < NR; i++) for (let j = 0; j < NC; j++) {
    const v = i * NC + j;
    pos[3 * v] = i * DX; pos[3 * v + 1] = j * DX;
    uv[2 * v] = i / (NR - 1); uv[2 * v + 1] = j / (NC - 1);
    if (i < NR - 1 && j < NC - 1) index.push(v, v + NC, v + 1, v + 1, v + NC, v + NC + 1);
  }
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
  geo.setAttribute('color', new THREE.BufferAttribute(new Float32Array(3 * NV), 3));
  geo.setIndex(index);
}
const mesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ vertexColors: true, side: THREE.DoubleSide, roughness: 0.68, metalness: 0.02, polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1 }));
world.add(mesh);
const axes = new THREE.AxesHelper(500);
axes.position.z = 12;
world.add(axes);

const ground = new THREE.GridHelper(8000, 40);
ground.rotation.x = Math.PI / 2;
ground.position.z = -320;
ground.material.vertexColors = false;
ground.material.transparent = true;
ground.material.opacity = 0.22;
ground.material.depthWrite = false;
scene.add(ground);

const tubes = new THREE.Group();
world.add(tubes);
const topMat = new THREE.MeshBasicMaterial({ color: 0xf2a93b, toneMapped: false });
const bottomMat = new THREE.MeshBasicMaterial({ color: 0x4f9cf0, toneMapped: false });

const PREVIEW_MAX = 500;
const previewGeo = new THREE.BufferGeometry();
previewGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(3 * PREVIEW_MAX), 3));
const preview = new THREE.Line(previewGeo, new THREE.LineBasicMaterial({ color: 0xf2a93b, toneMapped: false }));
preview.visible = false;
preview.frustumCulled = false;
world.add(preview);

// 색: 곡면 가공 화면과 같은 파랑-회색-빨강
const stops = (mid) => [[-1, '#184f95'], [-0.5, '#5598e7'], [0, mid], [0.5, '#ec7f6e'], [1, '#a11d1f']];
let lut = null, viewNow = '';
function buildLut(mid) {
  const st = stops(mid).map(([t, c]) => [t, new THREE.Color(c)]);
  const out = new Float32Array(256 * 3), c = new THREE.Color();
  for (let i = 0; i < 256; i++) {
    const t = -1 + 2 * i / 255;
    let k = 0;
    while (k < st.length - 2 && t > st[k + 1][0]) k++;
    c.copy(st[k][1]).lerp(st[k + 1][1], (t - st[k][0]) / (st[k + 1][0] - st[k][0]));
    out[3 * i] = c.r; out[3 * i + 1] = c.g; out[3 * i + 2] = c.b;
  }
  return out;
}
function applyTheme() {
  const view = getComputedStyle(document.documentElement).getPropertyValue('--view').trim();
  if (!view || view === viewNow) return false;
  viewNow = view;
  const back = new THREE.Color(view);
  scene.background = back;
  const dark = back.r + back.g + back.b < 1.2;
  const mid = dark ? '#737d8a' : '#e6e3da';
  lut = buildLut(mid);
  $('pvLegendBar').style.background = `linear-gradient(90deg, ${stops(mid).map(([, c]) => c).join(', ')})`;
  ground.material.color.set(dark ? 0xdfe6f0 : 0x20262e);
  return true;
}

function deformedAt(x, y, out) {
  const a = clamp(x / DX, 0, NR - 1.0001), b = clamp(y / DX, 0, NC - 1.0001);
  const i = Math.floor(a), j = Math.floor(b), fa = a - i, fb = b - j;
  const p = geo.attributes.position.array;
  const v00 = 3 * (i * NC + j), v01 = v00 + 3, v10 = v00 + 3 * NC, v11 = v10 + 3;
  const at = (k) => (1 - fa) * ((1 - fb) * p[v00 + k] + fb * p[v01 + k]) + fa * ((1 - fb) * p[v10 + k] + fb * p[v11 + k]);
  return out.set(at(0), at(1), at(2));
}

const tmpV = new THREE.Vector3();
function writeLines() {
  for (const c of tubes.children) c.geometry.dispose();
  tubes.clear();
  for (const l of S.lines) {
    const pts = l.points.map(([x, y]) => { deformedAt(x, y, tmpV); return new THREE.Vector3(tmpV.x, tmpV.y, tmpV.z + 10); });
    const tube = new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts, false, 'centripetal'), pts.length * 2, 14, 6, false);
    tubes.add(new THREE.Mesh(tube, l.side === 'bottom' ? bottomMat : topMat));
  }
}

function writeSurface() {
  const pos = geo.attributes.position.array, col = geo.attributes.color.array, F = S.field, ex = S.exag;
  let lo = 0, hi = 0;
  if (F) for (let v = 0; v < NV; v++) { const z = F[3 * v + 2]; if (z < lo) lo = z; if (z > hi) hi = z; }
  const range = Math.max(-lo, hi, 1e-6);
  for (let v = 0; v < NV; v++) {
    for (let k = 0; k < 3; k++) pos[3 * v + k] = X[3 * v + k] + (F ? F[3 * v + k] * ex : 0);
    const u = F ? F[3 * v + 2] / range : 0;
    const q = 3 * Math.round((clamp(u, -1, 1) + 1) * 127.5);
    col[3 * v] = lut[q]; col[3 * v + 1] = lut[q + 1]; col[3 * v + 2] = lut[q + 2];
  }
  geo.attributes.position.needsUpdate = true;
  geo.attributes.color.needsUpdate = true;
  geo.computeVertexNormals();
  geo.computeBoundingSphere();
  writeLines();
  let total = 0;
  for (const l of S.lines) total += l.len;
  $('pvCount').textContent = String(S.lines.length);
  $('pvLength').textContent = fix(total / 1000, 2);
  $('pvRange').textContent = F ? fix(hi - lo, 2) : '0.00';
  $('pvMs').textContent = S.ms == null ? '-' : fix(S.ms, 0);
  $('pvLo').textContent = F ? fix(-range, 1) : '-';
  $('pvHi').textContent = F ? `+${fix(range, 1)}` : '+';
  $('pvScale').textContent = `변형 배율 ×${ex} (모양만 확대) · 바닥 눈금 한 칸 200 mm`;
  requestDraw();
}

function resize() {
  const r = host.getBoundingClientRect();
  if (r.width < 2 || r.height < 2) return;
  renderer.setSize(r.width, r.height);
  camera.aspect = r.width / r.height;
  camera.updateProjectionMatrix();
  renderer.render(scene, camera);
}
new ResizeObserver(resize).observe(host);

function fitCamera(kind) {
  const [az, el] = (kind === 'top' ? [-90, 87] : kind === 'side' ? [-90, 4] : [-60, 28]).map(THREE.MathUtils.degToRad);
  const d = Math.hypot(L, W) * 1.7 * Math.max(1, 1.5 / (camera.aspect || 1.5));
  camera.position.set(d * Math.cos(el) * Math.cos(az), d * Math.cos(el) * Math.sin(az), d * Math.sin(el));
  controls.target.set(0, 0, 0);
  controls.update();
  requestDraw();
  for (const b of document.querySelectorAll('[data-pvcam]')) b.setAttribute('aria-pressed', String(b.dataset.pvcam === kind));
}

// ── 가열선 ──────────────────────────────────────────────
function polyLength(p) {
  let s = 0;
  for (let i = 1; i < p.length; i++) s += Math.hypot(p[i][0] - p[i - 1][0], p[i][1] - p[i - 1][1]);
  return s;
}
function resample(p, n) {
  const total = polyLength(p), out = [];
  let k = 0, walked = 0;
  for (let s = 0; s < n; s++) {
    const want = total * s / (n - 1);
    while (k < p.length - 2 && walked + Math.hypot(p[k + 1][0] - p[k][0], p[k + 1][1] - p[k][1]) < want) {
      walked += Math.hypot(p[k + 1][0] - p[k][0], p[k + 1][1] - p[k][1]);
      k++;
    }
    const seg = Math.hypot(p[k + 1][0] - p[k][0], p[k + 1][1] - p[k][1]) || 1, u = clamp((want - walked) / seg, 0, 1);
    out.push([p[k][0] + u * (p[k + 1][0] - p[k][0]), p[k][1] + u * (p[k + 1][1] - p[k][1]), 0]);
  }
  return out;
}
function setPath(l, poly) {
  l.len = polyLength(poly);
  l.points = resample(poly, clamp(Math.ceil(l.len / 100) + 1, 13, 81));
}
const clampPoint = ([x, y]) => [clamp(x, 0, L), clamp(y, 0, W)];

function addLine(kind, poly) {
  if (S.lines.length >= MAX_LINES) { setMsg(`가열선은 ${MAX_LINES}줄까지 넣을 수 있습니다.`, true); return false; }
  const clean = poly.map(clampPoint);
  if (polyLength(clean) < MIN_LEN) { setMsg('선이 너무 짧습니다. 80 mm 이상으로 그려 주세요.', true); return false; }
  const l = { id: S.nextId++, kind, weight: 1, side: 'top' };
  if (kind === 'straight') { l.a = clean[0]; l.b = clean[clean.length - 1]; }
  setPath(l, clean);
  S.lines.push(l);
  setMsg('');
  linesChanged();
  return true;
}

function nearPairs() {
  let n = 0;
  for (let a = 0; a < S.lines.length; a++) for (let b = a + 1; b < S.lines.length; b++) {
    let close = false;
    for (const p of S.lines[a].points) {
      for (const q of S.lines[b].points) if (Math.hypot(p[0] - q[0], p[1] - q[1]) < NEAR_MM) { close = true; break; }
      if (close) break;
    }
    if (close) n++;
  }
  return n;
}

function field(label, value, step, onChange) {
  const box = document.createElement('label');
  box.textContent = label;
  const inp = document.createElement('input');
  inp.type = 'number';
  inp.step = String(step);
  inp.value = String(value);
  inp.addEventListener('change', () => onChange(parseFloat(inp.value), inp));
  box.append(inp);
  return box;
}

function renderList() {
  const box = $('pvList');
  box.textContent = '';
  $('pvEmpty').hidden = S.lines.length > 0;
  S.lines.forEach((l, idx) => {
    const row = document.createElement('div');
    row.className = 'ln';
    const top = document.createElement('div');
    top.className = 'ln-top';
    const no = document.createElement('span');
    no.className = 'no-text';
    no.textContent = String(idx + 1);
    const name = document.createElement('span');
    name.className = 'name';
    name.textContent = `${l.kind === 'straight' ? '직선' : '자유선'} ${fix(l.len / 1000, 2)} m`;
    const close = document.createElement('button');
    close.className = 'x';
    close.textContent = '×';
    close.setAttribute('aria-label', `${idx + 1}번째 선 지우기`);
    close.addEventListener('click', () => { S.lines = S.lines.filter((q) => q !== l); setMsg(''); linesChanged(); });
    top.append(no, name, close);
    row.append(top);

    const edit = document.createElement('div');
    edit.className = 'ln-edit two';
    edit.append(field('세기 배수', l.weight, 0.1, (v, inp) => {
      if (!Number.isFinite(v) || v < 0) { inp.value = String(l.weight); return; }
      l.weight = clamp(v, 0, 20);
      solve();
    }));
    const sideBox = document.createElement('label');
    sideBox.textContent = '가열 면';
    const side = document.createElement('select');
    for (const [v, t] of [['top', '윗면'], ['bottom', '아랫면']]) {
      const o = document.createElement('option');
      o.value = v; o.textContent = t;
      side.append(o);
    }
    side.value = l.side;
    side.addEventListener('change', () => { l.side = side.value; solve(); });
    sideBox.append(side);
    edit.append(sideBox);
    row.append(edit);

    if (l.kind === 'straight') {
      const xy = document.createElement('div');
      xy.className = 'ln-edit';
      const coord = (label, key, axis, max) => field(label, Math.round(l[key][axis]), 10, (v, inp) => {
        const before = l[key][axis];
        if (!Number.isFinite(v)) { inp.value = String(Math.round(before)); return; }
        l[key][axis] = clamp(v, 0, max);
        if (polyLength([l.a, l.b]) < MIN_LEN) {
          l[key][axis] = before;
          inp.value = String(Math.round(before));
          setMsg('시작점과 끝점은 80 mm 이상 떨어져야 합니다.', true);
          return;
        }
        setMsg('');
        setPath(l, [l.a, l.b]);
        linesChanged();
      });
      xy.append(coord('시작 x', 'a', 0, L), coord('시작 y', 'a', 1, W), coord('끝 x', 'b', 0, L), coord('끝 y', 'b', 1, W));
      row.append(xy);
    }
    box.append(row);
  });
  const near = nearPairs();
  $('pvNear').hidden = !near;
  $('pvNear').textContent = near ? `${NEAR_MM} mm 안으로 가깝거나 만나는 가열선 ${near}쌍이 있습니다. 단순 합 계산이라 오차가 커질 수 있습니다.` : '';
}

function linesChanged() {
  renderList();
  writeSurface();
  solve();
}

// ── 마우스로 그리기 ──────────────────────────────────────
const raycaster = new THREE.Raycaster();
const ndc = new THREE.Vector2();
function pick(ev) {
  const r = canvas.getBoundingClientRect();
  ndc.set(((ev.clientX - r.left) / r.width) * 2 - 1, -((ev.clientY - r.top) / r.height) * 2 + 1);
  raycaster.setFromCamera(ndc, camera);
  const hit = raycaster.intersectObject(mesh, false)[0];
  return hit ? [hit.uv.x * L, hit.uv.y * W] : null;
}
function drawPreview(poly) {
  const arr = previewGeo.attributes.position.array, n = Math.min(poly.length, PREVIEW_MAX);
  for (let i = 0; i < n; i++) {
    deformedAt(poly[i][0], poly[i][1], tmpV);
    arr[3 * i] = tmpV.x; arr[3 * i + 1] = tmpV.y; arr[3 * i + 2] = tmpV.z + 16;
  }
  previewGeo.attributes.position.needsUpdate = true;
  previewGeo.setDrawRange(0, n);
  preview.visible = n > 1;
  requestDraw();
}

const HINTS = {
  straight: '판 위를 두 번 누르면 직선이 생깁니다. 끝내려면 단추를 다시 누르거나 Esc 를 누르세요.',
  free: '판 위를 누른 채로 끌면 그린 대로 선이 생깁니다. 끝내려면 단추를 다시 누르거나 Esc 를 누르세요.',
};
function setMode(mode) {
  S.mode = mode;
  S.first = null;
  S.drawing = null;
  preview.visible = false;
  controls.enabled = mode !== 'free';
  host.classList.toggle('adding', !!mode);
  $('pvStraight').setAttribute('aria-pressed', String(mode === 'straight'));
  $('pvFree').setAttribute('aria-pressed', String(mode === 'free'));
  $('pvHint').textContent = HINTS[mode] || '[직선 그리기]나 [자유선 그리기]로 판 위에 가열선을 넣거나, [좌표로 추가]로 값을 입력하세요.';
  requestDraw();
}

let pressed = null;
canvas.addEventListener('pointerdown', (ev) => {
  pressed = { x: ev.clientX, y: ev.clientY };
  if (S.mode !== 'free' || ev.button !== 0) return;
  // 판 밖에서 끌기 시작해도 판에 닿는 순간부터 선을 잇는다.
  const hit = pick(ev);
  S.drawing = hit ? [hit] : [];
  canvas.setPointerCapture(ev.pointerId);
});
canvas.addEventListener('pointermove', (ev) => {
  if (S.mode === 'free' && S.drawing) {
    const hit = pick(ev);
    if (!hit) return;
    const last = S.drawing[S.drawing.length - 1];
    if ((last && Math.hypot(hit[0] - last[0], hit[1] - last[1]) < 30) || S.drawing.length >= PREVIEW_MAX) return;
    S.drawing.push(hit);
    drawPreview(S.drawing);
  } else if (S.mode === 'straight' && S.first) {
    const hit = pick(ev);
    if (hit) drawPreview([S.first, hit]);
  }
});
canvas.addEventListener('pointerup', (ev) => {
  if (S.mode === 'free') {
    const poly = S.drawing;
    S.drawing = null;
    preview.visible = false;
    requestDraw();
    if (poly && poly.length > 1) addLine('free', poly);
    else if (poly) setMsg('판 위를 누른 채로 끌어 주세요.', true);
    return;
  }
  if (S.mode !== 'straight' || ev.button !== 0 || !pressed) return;
  if (Math.hypot(ev.clientX - pressed.x, ev.clientY - pressed.y) > 5) return;
  const hit = pick(ev);
  if (!hit) return;
  if (!S.first) { S.first = hit; setMsg('끝점을 눌러 주세요.'); return; }
  const a = S.first;
  S.first = null;
  preview.visible = false;
  addLine('straight', [a, hit]);
});

// ── 단추 ────────────────────────────────────────────────
$('pvStraight').addEventListener('click', () => setMode(S.mode === 'straight' ? null : 'straight'));
$('pvFree').addEventListener('click', () => setMode(S.mode === 'free' ? null : 'free'));
window.addEventListener('keydown', (ev) => { if (ev.key === 'Escape' && S.mode) setMode(null); });
$('pvCoord').addEventListener('click', () => {
  const y0 = clamp(300 + 150 * (S.lines.length % 9), 0, W);
  if (addLine('straight', [[500, y0], [2500, y0]])) setMsg('목록의 시작·끝 좌표를 고치면 선이 옮겨집니다.');
});
$('pvClear').addEventListener('click', () => { S.lines = []; setMsg(''); linesChanged(); });
$('pvExag').addEventListener('input', () => {
  S.exag = parseInt($('pvExag').value, 10);
  $('pvExagVal').textContent = `×${S.exag}`;
  writeSurface();
});
$('pvSave').addEventListener('click', () => {
  renderer.render(scene, camera);
  const a = document.createElement('a');
  a.href = canvas.toDataURL('image/png');
  a.download = 'plate_heating_lines.png';
  a.click();
});
for (const b of document.querySelectorAll('[data-pvcam]')) b.addEventListener('click', () => fitCamera(b.dataset.pvcam));

// ── 시작 ────────────────────────────────────────────────
function usePrediction(p) {
  if (!p) return;
  const { x, sc } = p;
  S.thickness = x[0];
  S.m = { theta_mrad: Math.abs(sc.theta.value), dtrans_mm: sc.dtrans.value, dlong_mm: sc.dlong.value };
  if (sc.B_l && Number.isFinite(sc.B_l.value)) S.m.B_l = sc.B_l.value;
  solve();
}
function refreshLook() {
  if (applyTheme()) writeSurface();
  resize();
}
applyTheme();
resize();
fitCamera('iso');
writeSurface();
setMode(null);
addLine('straight', [[1500, 100], [1500, 1900]]);   // 처음부터 결과가 보이도록 넣어 두는 예시 1줄
window.addEventListener('resize', refreshLook);
matchMedia('(prefers-color-scheme: dark)').addEventListener('change', refreshLook);
window.addEventListener('ai-prediction', (ev) => usePrediction(ev.detail));
usePrediction(window.aiLatest);

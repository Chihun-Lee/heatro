// 곡면 가공 시험 화면.
// 판과 목표 곡면을 고르고, 가열선을 더해 가며 곡면이 어떻게 변하는지 재생한다.
// 해석은 engine/solver_ext.js 의 PlateModel(Web Worker engine/worker_ext.js)로 한다. 공개 저장소의 /studio/solver.js 를 복사해 B_l·끝단 구간을 더한 것이고, 그 입력을 주지 않으면 원본과 같은 값이다.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { flatten, refineGrid, removeRigid } from './engine/solver_ext.js';
import { zPlaneFree, rms, boundedFit } from '../studio/planner.js';
import { classifyPlate, CLASS } from './engine/kinematics.js';
import { prepareKinematics, curlSeries } from './engine/planner_shin.js';
import { signedComp, compKey, lineSeconds, formatSeconds, cumulativeLength, endZoneSpans, arcIndex, buildAiBridge, aiPredictComp, aiRangeWarnings, aiModelNotes } from './engine/forming_tools.js';

// ── 상수 ───────────────────────────────────────────────────
const SEGMENTS = 6;        // 줄 하나를 나누는 구간 수. 구간별 응답을 더해 재생 중 상태를 만든다.
const TORCH_SHARE = 0.86;  // 줄 하나의 재생 시간 중 토치가 움직이는 몫. 나머지는 식는 시간.
const LINE_SECONDS = 1.9;  // 배속 1 에서 줄 하나의 재생 시간(초)
const COST_LIMIT = 6e9;    // 밴드 분해 계산량 상한. 넘으면 격자를 거칠게 잡는다.
const ROUNDS = 3;          // 자동 추천에서 (면, 방향) 묶음을 도는 횟수

// a, b 는 판 중앙을 0 으로 한 -0.5~0.5 위치. 값의 절대 크기는 뒤에서 '깊이'로 맞춘다.
const SHAPES = {
  flat: { name: '평판', f: () => 0 },
  pillow: { name: '볼록 (가운데 높음)', f: (a, b) => (1 - 4 * a * a) * (1 - 4 * b * b) },
  bowl: { name: '오목 (가운데 낮음)', f: (a, b) => -(1 - 4 * a * a) * (1 - 4 * b * b) },
  cyl_up: { name: '원통 (양 끝 올라감)', f: (a) => 4 * a * a },
  cyl_down: { name: '원통 (가운데 높음)', f: (a) => -4 * a * a },
  saddle: { name: '안장', f: (a, b) => 4 * a * a - 4 * b * b },
  twist: { name: '비틀림', f: (a, b) => 4 * a * b },
  scurve: { name: 'S자 굴곡', f: (a) => Math.sin(2 * Math.PI * a) },
  edge: { name: '한쪽 끝 굽힘', f: (a) => (a > 0 ? (2 * a) ** 2 : 0) },
};

// ── 작은 도구 ──────────────────────────────────────────────
const $ = (id) => document.getElementById(id);
const num = (id) => { const v = parseFloat($(id).value); return Number.isFinite(v) ? v : 0; };
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const dot = (a, b) => { let s = 0; for (let i = 0; i < a.length; i++) s += a[i] * b[i]; return s; };
const maxAbs = (a) => { let m = 0; for (let i = 0; i < a.length; i++) { const v = Math.abs(a[i]); if (v > m) m = v; } return m; };
const fix = (v, d = 2) => (Number.isFinite(v) ? v.toFixed(d) : '-');
const sleep = (ms) => new Promise((ok) => setTimeout(ok, ms));
const yieldFrame = () => new Promise((ok) => setTimeout(ok, 0));
function el(tag, attrs = {}, ...kids) {
  const n = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === 'class') n.className = v;
    else if (k === 'text') n.textContent = v;
    else n.setAttribute(k, v);
  }
  n.append(...kids);
  return n;
}
function roundStep(x) {
  const e = Math.floor(Math.log10(x)), b = x / 10 ** e;
  return (b < 1.5 ? 1 : b < 3.5 ? 2 : b < 7.5 ? 5 : 10) * 10 ** e;
}
function setMsg(text, isErr = false) { const m = $('msg'); m.textContent = text || ''; m.classList.toggle('err', isErr); }
function setProgress(v) { $('bar').style.width = `${Math.round(clamp(v, 0, 1) * 100)}%`; }

// ── 상태 ───────────────────────────────────────────────────
const S = {
  plate: null,        // 지금 판 (아래 preparePlate 참고)
  lines: [],          // 가열선 목록
  T: 0,               // 재생 위치 (줄 단위, 0~N)
  view: 'after',      // before | after | target
  exag: 10,           // 변형 배율
  playing: false,
  rate: 1,
  busy: false,
  nextId: 1,
  pickCounter: 0,
  table: null,        // 불러온 보정표 (메모리)
  condCache: new Map(),
  cumR: [], cumZ: [], rmsSeries: [], maxSeries: [],
  dark: false,
  colorKind: 'dev',   // dev 목표 차이 | curv 곡률 부호
  showNeedles: false,
  curl: null,         // 곡 펴짐 지표 (curlSeries 결과)
};

// AI 시뮬레이터 모델 (static/ai_model/, static/ai.js 의 추론 코드를 그대로 쓴다)
const AI = { bridge: null, meta: null, dir: null, notes: [], error: null, cache: new Map(), loading: null };

// ── 해석 작업자 (engine/worker_ext.js) ──────────────────────
const worker = new Worker('engine/worker_ext.js', { type: 'module' });
let callChain = Promise.resolve();
let runId = 0;
function call(message, doneType, onProgress) {
  const run = () => new Promise((resolve, reject) => {
    worker.onmessage = ({ data }) => {
      if (data.type === 'error') reject(new Error(data.message));
      else if (data.type === 'progress') { if (onProgress) onProgress(data.value); }
      else if (data.type === doneType) resolve(data);
    };
    worker.postMessage(message);
  });
  const p = callChain.then(run, run);
  callChain = p.catch(() => {});
  return p;
}
worker.onerror = (e) => setMsg(`해석 작업자를 시작하지 못했습니다: ${e.message || ''}`, true);

// ── 판 만들기 ──────────────────────────────────────────────
function solverCost(nr0, nc0, refine) {
  const nr = (nr0 - 1) * refine + 1, nc = (nc0 - 1) * refine + 1;
  const neq = nr * nc * 5, bw = 5 * (nc + 2) - 1;
  return { neq, bw, flops: neq * bw * bw };
}

function shapePoints(L, W, nr, nc, key, depth) {
  const f = (SHAPES[key] ?? SHAPES.flat).f;
  const raw = new Float64Array(nr * nc);
  let lo = Infinity, hi = -Infinity;
  for (let i = 0; i < nr; i++) for (let j = 0; j < nc; j++) {
    const v = f(i / (nr - 1) - 0.5, j / (nc - 1) - 0.5);
    raw[i * nc + j] = v;
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }
  const scale = hi > lo ? depth / (hi - lo) : 0;
  const pts = [];
  for (let i = 0; i < nr; i++) {
    const row = [];
    for (let j = 0; j < nc; j++) row.push([L * i / (nr - 1), W * j / (nc - 1), (raw[i * nc + j] - lo) * scale]);
    pts.push(row);
  }
  return pts;
}

function syntheticPlate() {
  const L = clamp(num('L'), 300, 30000), W = clamp(num('W'), 300, 30000), t = clamp(num('t'), 3, 100);
  let dx = clamp(num('dx'), 10, 500);
  let coarsened = false;
  let nr, nc;
  for (;;) {
    nr = Math.max(5, Math.round(L / dx) + 1);
    nc = Math.max(5, Math.round(W / dx) + 1);
    if (solverCost(nr, nc, 1).flops <= COST_LIMIT) break;
    dx *= 1.2;
    coarsened = true;
  }
  return {
    fromFile: false,
    points: shapePoints(L, W, nr, nc, $('srcShape').value, clamp(num('srcDepth'), 0, 1000)),
    target: shapePoints(L, W, nr, nc, $('tgtShape').value, clamp(num('tgtDepth'), 0, 1000)),
    t, refine: 1, dx, coarsened, name: '합성 판',
  };
}

// 뷰어와 같은 형식의 판 파일을 읽는다. 메모리에서만 쓴다.
function readPoint(s) {
  const v = Array.isArray(s) ? s.slice(0, 3).map(Number) : String(s).split(',').map((x) => parseFloat(x));
  if (v.length < 3 || !v.slice(0, 3).every(Number.isFinite)) throw new Error('좌표가 숫자 세 개가 아닌 점이 있습니다.');
  return [v[0], v[1], v[2]];
}
function readGrid(raw, label) {
  if (!Array.isArray(raw) || raw.length < 3 || !Array.isArray(raw[0]) || raw[0].length < 3) throw new Error(`${label} 격자가 없습니다.`);
  const nc = raw[0].length;
  return raw.map((row) => {
    if (row.length !== nc) throw new Error(`${label} 격자의 행 길이가 일정하지 않습니다.`);
    return row.map(readPoint);
  });
}
function plateFromDoc(doc, fileName) {
  const points = readGrid(doc.SourceGrid?.Points, '시작 형상(SourceGrid)');
  const target = readGrid(doc.TargetGrid?.Points, '목표 형상(TargetGrid)');
  if (points.length !== target.length || points[0].length !== target[0].length) throw new Error('시작 형상과 목표 형상의 격자 크기가 다릅니다.');
  const t = Number(doc.dThickNess ?? doc.SourceGrid?.Thickness);
  if (!(t > 0)) throw new Error('두께(dThickNess)가 없습니다.');
  const refine = solverCost(points.length, points[0].length, 2).flops <= COST_LIMIT ? 2 : 1;
  if (solverCost(points.length, points[0].length, refine).flops > COST_LIMIT * 4) throw new Error('격자가 너무 커서 이 화면에서 풀기 어렵습니다.');
  const allCurves = doc.BendingCurves ?? [];
  const curves = allCurves.filter((c) => c.HeatingMethod === 1 && (c.Points?.length ?? 0) >= 2).map((c, n) => ({
    name: c.CurveId != null ? String(c.CurveId) : `파일 선 ${n + 1}`,
    points: c.Points.map(readPoint),
    speed: Number.isFinite(c.speed) ? c.speed : null,
    peak: Array.isArray(c.ParameterTemp) && c.ParameterTemp.length ? Math.max(...c.ParameterTemp) : null,
  }));
  return { fromFile: true, synthetic: doc.DeviceCode === 'DEMO' || String(doc.LegacyId ?? '').startsWith('demo-'), points, target, t, refine, name: fileName, curves, skippedCurves: allCurves.length - curves.length };
}

// 판에서 파생되는 값들을 만든다. p.m 은 표시 격자를 입력 격자의 몇 배로 쪼갤지.
function preparePlate(p) {
  p.nr = p.points.length;
  p.nc = p.points[0].length;
  p.m = clamp(Math.ceil(Math.sqrt(9000 / (p.nr * p.nc))), 1, 4);
  p.X = flatten(p.points);
  const g = refineGrid(p.points, p.m);
  p.rnr = g.nr;
  p.rnc = g.nc;
  const nv = g.nr * g.nc;
  let cx = 0, cy = 0, cz = 0;
  for (let i = 0; i < nv; i++) { cx += g.X[3 * i]; cy += g.X[3 * i + 1]; cz += g.X[3 * i + 2]; }
  p.center = [cx / nv, cy / nv, cz / nv];
  p.Xr = new Float32Array(3 * nv);
  let xmin = Infinity, xmax = -Infinity, ymin = Infinity, ymax = -Infinity, zmin = Infinity, zmax = -Infinity;
  for (let i = 0; i < nv; i++) {
    const x = g.X[3 * i] - p.center[0], y = g.X[3 * i + 1] - p.center[1], z = g.X[3 * i + 2] - p.center[2];
    p.Xr[3 * i] = x; p.Xr[3 * i + 1] = y; p.Xr[3 * i + 2] = z;
    if (x < xmin) xmin = x; if (x > xmax) xmax = x; if (y < ymin) ymin = y; if (y > ymax) ymax = y; if (z < zmin) zmin = z; if (z > zmax) zmax = z;
  }
  p.bbox = { xmin, xmax, ymin, ymax, zmin, zmax };
  p.diag = Math.hypot(xmax - xmin, ymax - ymin, zmax - zmin);
  p.U = new Float32Array(3 * nv);
  p.zNow = new Float64Array(p.nr * p.nc);
  const mid = (a) => Math.floor(a / 2);
  const dirOf = (a, b) => { const dx = b[0] - a[0], dy = b[1] - a[1], n = Math.hypot(dx, dy) || 1; return [dx / n, dy / n]; };
  p.rowAxis = dirOf(p.points[0][mid(p.nc)], p.points[p.nr - 1][mid(p.nc)]);
  p.colAxis = dirOf(p.points[mid(p.nr)][0], p.points[mid(p.nr)][p.nc - 1]);
  const a = (i, j) => p.points[i][j];
  p.dsRow = Math.hypot(a(mid(p.nr) + 1, mid(p.nc))[0] - a(mid(p.nr), mid(p.nc))[0], a(mid(p.nr) + 1, mid(p.nc))[1] - a(mid(p.nr), mid(p.nc))[1]);
  p.dsCol = Math.hypot(a(mid(p.nr), mid(p.nc) + 1)[0] - a(mid(p.nr), mid(p.nc))[0], a(mid(p.nr), mid(p.nc) + 1)[1] - a(mid(p.nr), mid(p.nc))[1]);
  setTargetFields(p);
}

function upsample3(flat, p) {
  if (p.m === 1) return Float32Array.from(flat);
  const nested = new Array(p.nr);
  for (let i = 0; i < p.nr; i++) {
    const row = new Array(p.nc);
    for (let j = 0; j < p.nc; j++) { const o = (i * p.nc + j) * 3; row[j] = [flat[o], flat[o + 1], flat[o + 2]]; }
    nested[i] = row;
  }
  return Float32Array.from(refineGrid(nested, p.m).X);
}
function upsampleScalar(a, p) {
  const v = new Float64Array(a.length * 3);
  for (let i = 0; i < a.length; i++) v[3 * i + 2] = a[i];
  const r = upsample3(v, p);
  const out = new Float32Array(r.length / 3);
  for (let i = 0; i < out.length; i++) out[i] = r[3 * i + 2];
  return out;
}

function setTargetFields(p) {
  const T = flatten(p.target);
  const diff = new Float64Array(p.X.length);
  for (let i = 0; i < diff.length; i++) diff[i] = T[i] - p.X[i];
  p.D = zPlaneFree(diff, p.X);
  p.Dr = upsampleScalar(p.D, p);
  p.rms0 = rms(p.D);
  p.max0 = maxAbs(p.D);
}

// ── 3D 장면 ────────────────────────────────────────────────
const host = $('view');
function fatal(text) {
  const f = $('fatal');
  f.style.display = 'flex';
  f.textContent = text;
}
let renderer;
try {
  renderer = new THREE.WebGLRenderer({ antialias: true });
} catch (err) {
  fatal('이 브라우저에서 3D 그래픽(WebGL)을 쓸 수 없어 화면을 그릴 수 없습니다.');
  throw err;
}
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
host.prepend(renderer.domElement);
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(32, 1, 10, 100000);
camera.up.set(0, 0, 1);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.14;
controls.mouseButtons = { LEFT: THREE.MOUSE.ROTATE, MIDDLE: THREE.MOUSE.PAN, RIGHT: THREE.MOUSE.PAN };
const hemi = new THREE.HemisphereLight(0xffffff, 0xb6bcc6, 1.9);
const sun = new THREE.DirectionalLight(0xffffff, 1.5);
sun.position.set(0.45, -0.7, 1);
scene.add(hemi, sun);

const surfaceMat = new THREE.MeshStandardMaterial({ vertexColors: true, side: THREE.DoubleSide, roughness: 0.68, metalness: 0.02, polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1 });
const gridMat = new THREE.LineBasicMaterial({ color: 0x20262e, transparent: true, opacity: 0.2, depthWrite: false });
const skirtMat = new THREE.MeshStandardMaterial({ color: 0x626b77, roughness: 0.75, side: THREE.DoubleSide });
const heatMat = new THREE.MeshBasicMaterial({ vertexColors: true, transparent: true, depthWrite: false, side: THREE.DoubleSide, toneMapped: false });
const ribbonMat = new THREE.MeshBasicMaterial({ vertexColors: true, transparent: true, depthWrite: false, side: THREE.DoubleSide, toneMapped: false, polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2 });
const ghostMat = new THREE.MeshBasicMaterial({ color: 0x4f9cf0, transparent: true, opacity: 0.28, depthWrite: false, side: THREE.DoubleSide });
const bandMat = new THREE.MeshBasicMaterial({ color: 0xb02a9e, transparent: true, opacity: 0.55, depthWrite: false, side: THREE.DoubleSide, toneMapped: false, polygonOffset: true, polygonOffsetFactor: -1, polygonOffsetUnits: -1 });
const needleMat = new THREE.LineBasicMaterial({ vertexColors: true, toneMapped: false, transparent: true, opacity: 0.9 });

// 토치: 노즐 + 불꽃. 끝이 판 표면(국소 z=0)에 오도록 만든다.
const torch = new THREE.Group();
const torchBody = new THREE.Mesh(new THREE.CylinderGeometry(1, 0.62, 3.2, 20), new THREE.MeshStandardMaterial({ color: 0x8892a0, metalness: 0.5, roughness: 0.35 }));
torchBody.rotation.x = Math.PI / 2;
torchBody.position.z = 4.1;
const flame = new THREE.Mesh(new THREE.ConeGeometry(0.75, 2.3, 16), new THREE.MeshBasicMaterial({ color: 0xffa12e, transparent: true, opacity: 0.92, toneMapped: false }));
flame.rotation.x = -Math.PI / 2;
flame.position.z = 1.3;
const glow = new THREE.Mesh(new THREE.ConeGeometry(1.1, 3.1, 16), new THREE.MeshBasicMaterial({ color: 0xff6a1c, transparent: true, opacity: 0.25, toneMapped: false, depthWrite: false }));
glow.rotation.x = -Math.PI / 2;
glow.position.z = 1.8;
torch.add(torchBody, flame, glow);
torch.visible = false;
scene.add(torch);

let ribbon = null;
let bandMesh = null;    // 끝단 구간 띠
let needleObj = null;   // 주방향 바늘
let ground = null;
let sceneGroup = new THREE.Group();
scene.add(sceneGroup);

// 색표: 목표 차이는 파랑-회색-빨강, 열은 노랑 쪽 밝은 색이라 구분된다.
const DIV_STOPS = (mid) => [[-1, '#184f95'], [-0.5, '#5598e7'], [0, mid], [0.5, '#ec7f6e'], [1, '#a11d1f']];
const HEAT_STOPS = [[0, '#6a1200'], [0.3, '#d63a00'], [0.6, '#ff8a1f'], [0.85, '#ffd36b'], [1, '#fff4cf']];
let divLUT = null, heatLUT = null;
function buildLUT(stops, n, map = (t) => t) {
  const cols = stops.map(([x, hex]) => [x, new THREE.Color(hex)]);
  const out = new Float32Array(3 * n);
  const lo = stops[0][0], hi = stops[stops.length - 1][0];
  for (let i = 0; i < n; i++) {
    const x = lo + (hi - lo) * map(i / (n - 1));
    let a = 0;
    while (a < cols.length - 2 && x > cols[a + 1][0]) a++;
    const [x0, c0] = cols[a], [x1, c1] = cols[a + 1];
    const u = clamp((x - x0) / (x1 - x0), 0, 1);
    out[3 * i] = c0.r + (c1.r - c0.r) * u;
    out[3 * i + 1] = c0.g + (c1.g - c0.g) * u;
    out[3 * i + 2] = c0.b + (c1.b - c0.b) * u;
  }
  return out;
}
const cssVar = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
function applyTheme() {
  const set = document.documentElement.dataset.theme;
  S.dark = set === 'dark' || (!set && matchMedia('(prefers-color-scheme: dark)').matches);
  const mid = S.dark ? '#737d8a' : '#e6e3da';
  divLUT = buildLUT(DIV_STOPS(mid), 256);
  heatLUT = buildLUT(HEAT_STOPS, 256);
  scene.background = new THREE.Color(cssVar('--view'));
  gridMat.color.set(S.dark ? 0xdfe6f0 : 0x20262e);
  gridMat.opacity = S.dark ? 0.16 : 0.2;
  hemi.intensity = S.dark ? 1.5 : 1.9;
  sun.intensity = S.dark ? 1.25 : 1.5;
  ghostMat.color.set(S.dark ? 0x7ab6ff : 0x1f5fbf);
  bandMat.color.set(S.dark ? 0xf06cd6 : 0xb02a9e);
  buildCurvColors();
  $('legendBar').style.background = `linear-gradient(90deg, ${DIV_STOPS(mid).map(([x, c]) => `${c} ${(x + 1) * 50}%`).join(', ')})`;
  if (ground) rebuildGround();
  paintRibbons();
  if (S.plate?.posAttr) buildNeedles();
  markDirty();
}

function rebuildGround() {
  const p = S.plate;
  if (!p) return;
  if (ground) { scene.remove(ground); ground.geometry.dispose(); ground.material.dispose(); }
  const size0 = Math.max(p.bbox.xmax - p.bbox.xmin, p.bbox.ymax - p.bbox.ymin);
  const step = roundStep(size0 / 10);
  const div = Math.ceil(1.7 * size0 / step);
  const cols = S.dark ? [0x3a4656, 0x273040] : [0xb4bcc8, 0xd0d6df];
  ground = new THREE.GridHelper(step * div, div, cols[0], cols[1]);
  ground.rotation.x = Math.PI / 2;
  ground.material.transparent = true;
  ground.material.opacity = 0.75;
  ground.renderOrder = -1;
  scene.add(ground);
  S.groundStep = step;
  updateScaleNote();
}

function disposeGroup() {
  sceneGroup.traverse((o) => { if (o.geometry) o.geometry.dispose(); });
  scene.remove(sceneGroup);
  sceneGroup = new THREE.Group();
  scene.add(sceneGroup);
}

function buildPlateMeshes(p) {
  disposeGroup();
  const nv = p.rnr * p.rnc;
  p.posAttr = new THREE.BufferAttribute(new Float32Array(3 * nv), 3).setUsage(THREE.DynamicDrawUsage);
  p.colAttr = new THREE.BufferAttribute(new Float32Array(3 * nv), 3).setUsage(THREE.DynamicDrawUsage);
  p.heatAttr = new THREE.BufferAttribute(new Float32Array(4 * nv), 4).setUsage(THREE.DynamicDrawUsage);
  const idx = new Uint32Array(6 * (p.rnr - 1) * (p.rnc - 1));
  let q = 0;
  for (let i = 0; i < p.rnr - 1; i++) for (let j = 0; j < p.rnc - 1; j++) {
    const a = i * p.rnc + j, b = a + 1, c = a + p.rnc, d = c + 1;
    idx[q++] = a; idx[q++] = c; idx[q++] = b; idx[q++] = b; idx[q++] = c; idx[q++] = d;
  }
  const index = new THREE.BufferAttribute(idx, 1);

  const sg = new THREE.BufferGeometry();
  sg.setAttribute('position', p.posAttr);
  sg.setAttribute('color', p.colAttr);
  sg.setIndex(index);
  p.surface = new THREE.Mesh(sg, surfaceMat);
  p.surface.frustumCulled = false;
  sceneGroup.add(p.surface);

  const hg = new THREE.BufferGeometry();
  hg.setAttribute('position', p.posAttr);
  hg.setAttribute('color', p.heatAttr);
  hg.setIndex(index);
  p.heatMesh = new THREE.Mesh(hg, heatMat);
  p.heatMesh.frustumCulled = false;
  p.heatMesh.renderOrder = 2;
  p.heatMesh.visible = false;
  sceneGroup.add(p.heatMesh);

  // 판 위 눈금선 (표면과 같은 좌표를 공유한다)
  const spacing = (p.dsRow + p.dsCol) / 2 / p.m || 25;
  const stepMm = roundStep(p.diag / 12);
  const every = Math.max(1, Math.round(stepMm / spacing));
  const li = [];
  for (let i = 0; i < p.rnr; i += every) for (let j = 0; j < p.rnc - 1; j++) li.push(i * p.rnc + j, i * p.rnc + j + 1);
  for (let j = 0; j < p.rnc; j += every) for (let i = 0; i < p.rnr - 1; i++) li.push(i * p.rnc + j, (i + 1) * p.rnc + j);
  const lg = new THREE.BufferGeometry();
  lg.setAttribute('position', p.posAttr);
  lg.setIndex(li);
  p.gridLines = new THREE.LineSegments(lg, gridMat);
  p.gridLines.frustumCulled = false;
  p.gridLines.renderOrder = 1;
  sceneGroup.add(p.gridLines);

  // 판 두께 옆면
  const ring = [];
  for (let j = 0; j < p.rnc; j++) ring.push(j);
  for (let i = 1; i < p.rnr; i++) ring.push(i * p.rnc + p.rnc - 1);
  for (let j = p.rnc - 2; j >= 0; j--) ring.push((p.rnr - 1) * p.rnc + j);
  for (let i = p.rnr - 2; i > 0; i--) ring.push(i * p.rnc);
  p.ring = ring;
  p.skirtAttr = new THREE.BufferAttribute(new Float32Array(6 * ring.length), 3).setUsage(THREE.DynamicDrawUsage);
  const ski = [];
  for (let r = 0; r < ring.length; r++) {
    const a = 2 * r, b = 2 * r + 1, c = 2 * ((r + 1) % ring.length), d = c + 1;
    ski.push(a, c, b, b, c, d);
  }
  const skg = new THREE.BufferGeometry();
  skg.setAttribute('position', p.skirtAttr);
  skg.setIndex(ski);
  p.skirt = new THREE.Mesh(skg, skirtMat);
  p.skirt.frustumCulled = false;
  sceneGroup.add(p.skirt);

  // 목표 곡면 겹쳐 보기
  p.ghostAttr = new THREE.BufferAttribute(new Float32Array(3 * nv), 3).setUsage(THREE.DynamicDrawUsage);
  const gg = new THREE.BufferGeometry();
  gg.setAttribute('position', p.ghostAttr);
  gg.setIndex(index);
  p.ghost = new THREE.Mesh(gg, ghostMat);
  p.ghost.frustumCulled = false;
  p.ghost.renderOrder = 5;
  p.ghost.visible = false;
  sceneGroup.add(p.ghost);

  // 토치 크기는 판 크기에 맞춘다
  const r = clamp(p.diag * 0.012, 14, 60);
  torch.scale.setScalar(r);
  p.heatWidth = clamp(p.diag * 0.012, 25, 90);
  p.ribbonHalf = clamp(p.diag * 0.0052, 7, 28);
  rebuildGround();
}

// ── 세기 (3성분) ───────────────────────────────────────────
const modeNow = () => $('compMode').value;
const condKey = (t, v, pk) => [t, v, pk].map((x) => (+x).toFixed(3)).join('|');
function magnitudeFor(speed, peak) {
  const mode = modeNow();
  const t = S.plate.t;
  if (mode === 'direct') {
    const m = { theta_mrad: Math.abs(num('dirTheta')), dtrans_mm: num('dirTrans'), dlong_mm: num('dirLong'), source: 'direct' };
    const tl = parseFloat($('dirThetaL').value);
    if (Number.isFinite(tl) && tl !== 0) m.theta_l_mrad = Math.abs(tl);
    return m;
  }
  if (mode === 'table') {
    if (!S.table) return null;
    let best = null, score = Infinity;
    for (const e of S.table.entries) {
      const s = Math.abs(e.thickness_mm - t) * 10 + Math.abs(e.speed - speed) + Math.abs(e.peak_c - peak) / 100;
      if (s < score) { score = s; best = e; }
    }
    if (!best) return null;
    const m = { theta_mrad: Math.abs(best.theta_mrad), dtrans_mm: best.dtrans_mm, dlong_mm: best.dlong_mm, source: 'table', entry: best };
    if (Number.isFinite(best.B_l) && best.B_l !== 0) m.B_l = best.B_l;
    return m;
  }
  if (mode === 'ai') return aiMagnitude(speed, peak);
  return S.condCache.get(condKey(t, speed, peak)) ?? null;
}
function lineCond(l) {
  // 불러온 파일의 조건은 보정표 조회와 AI 계산(브라우저 안)에만 쓴다. 서버로 보내지 않는다.
  const own = modeNow() === 'table' || modeNow() === 'ai' || !S.plate.fromFile;
  return { speed: own && l.speed != null ? l.speed : num('speed'), peak: own && l.peak != null ? l.peak : num('peak') };
}
// 끝단 구간 입력. 길이를 넣지 않으면 끈다 (기본값 없음).
function endZoneNow() {
  const length = parseFloat($('endLen').value);
  if (!(length > 0)) return null;
  const ratio = parseFloat($('endRatio').value);
  return { length_mm: length, ratio: ratio > 0 ? ratio : 0.5, keepTotal: $('endKeep').checked };
}
const compFor = (m, side) => signedComp(m, side, endZoneNow());
function compOf(l) {
  const c = lineCond(l);
  const m = magnitudeFor(c.speed, c.peak);
  if (!m) throw new Error('이 줄의 세기를 정할 수 없습니다. 세기 설정을 확인해 주세요.');
  return compFor(m, l.side);
}
const respKey = (l) => compKey(compOf(l));

// AI 시뮬레이터: ai.js 의 추론 코드와 ai_model/ 의 모델로 줄별 세기를 바로 계산한다. 서버를 부르지 않는다.
const aiInputDefault = (name, fallback) => {
  const inp = (AI.meta?.inputs || []).find((q) => q.name === name);
  return inp && Number.isFinite(inp.value) ? inp.value : fallback;
};
async function loadAiModel() {
  if (AI.bridge) return AI;
  if (AI.loading) return AI.loading;
  AI.loading = (async () => {
    try {
      const fromUrl = new URLSearchParams(location.search).get('model');
      const dirs = [...new Set([fromUrl, 'ai_model', 'ai_model_dev'].filter(Boolean))];
      let meta = null, bin = null, dir = null;
      // 정식 폴더가 아직 비어 있으면 404 로 콘솔을 어지럽히지 않게 폴더 목록을 먼저 본다 (목록을 못 읽으면 그냥 시도한다).
      const hasModel = async (d) => {
        try {
          const r = await fetch(`${d}/`, { cache: 'no-store' });
          return !r.ok || (await r.text()).includes('model.json');
        } catch { return true; }
      };
      for (const d of dirs) {
        if (!(await hasModel(d))) continue;
        const res = await fetch(`${d}/model.json`, { cache: 'no-store' }).catch(() => null);
        if (!res || !res.ok) continue;
        meta = await res.json();
        const b = await fetch(`${d}/${meta.model_bin}`, { cache: 'no-store' });
        if (!b.ok) { meta = null; continue; }
        bin = await b.arrayBuffer();
        dir = d;
        break;
      }
      if (!meta) throw new Error('AI 모델 파일(ai_model/model.json)이 아직 없습니다. 학습이 끝나면 쓸 수 있습니다.');
      const src = await (await fetch('ai.js', { cache: 'no-store' })).text();
      const bridge = buildAiBridge(src);
      bridge.use(meta, bin);
      AI.bridge = bridge; AI.meta = meta; AI.dir = dir;
      AI.notes = aiModelNotes(meta, dir !== 'ai_model');
      AI.error = null;
      $('aiR0').value = aiInputDefault('r0', 15);
      $('aiGap').value = aiInputDefault('gap', 100);
      $('aiH').value = aiInputDefault('h_water', 3);
      $('aiFy').value = aiInputDefault('fy', 235);
      $('aiWater').checked = aiInputDefault('water', 1) !== 0;
    } catch (err) {
      AI.error = err.message;
    } finally {
      AI.loading = null;
    }
    return AI;
  })();
  return AI.loading;
}
function aiMagnitude(speed, peak) {
  if (!AI.bridge) return null;
  const t = S.plate.t;
  const water = $('aiWater').checked ? 1 : 0;
  const x = [t, speed, peak, num('aiR0'), num('aiGap'), num('aiH'), num('aiFy'), water];
  // 입력이 비었거나 0 이하면 모델의 로그 항이 NaN 이 된다. 값을 내지 않고 안내한다.
  AI.inputError = x.slice(0, 7).every((v) => v > 0) ? null : 'AI 입력에 비었거나 0 이하인 값이 있습니다. 속도, 최고온도, 열원 반경, 열원-수냉 거리, 수냉 세기, 항복강도를 확인해 주세요.';
  if (AI.inputError) return null;
  const key = x.join('|');
  let m = AI.cache.get(key);
  if (!m) {
    if (AI.cache.size > 500) AI.cache.clear();
    m = aiPredictComp(AI.bridge, x);
    m.warn = aiRangeWarnings(AI.meta, x);
    AI.cache.set(key, m);
  }
  return m;
}

async function computeCond() { throw new Error('이 공개판에서는 AI, 직접 입력, 보정표로 세기를 정합니다.'); }

async function ensureConditions(lines) {
  if (modeNow() !== 'cond') return;
  for (const l of lines) {
    const c = lineCond(l);
    if (S.condCache.has(condKey(S.plate.t, c.speed, c.peak))) continue;
    if (S.plate.fromFile) throw new Error('불러온 판에서는 [이 조건으로 세기 계산] 버튼을 먼저 눌러 주세요.');
    await computeCond(c.speed, c.peak);
  }
}

async function baseMagnitude() {
  const speed = num('speed'), peak = num('peak');
  let m = magnitudeFor(speed, peak);
  if (m) return m;
  if (modeNow() === 'cond') {
    if (S.plate.fromFile) throw new Error('불러온 판에서는 [이 조건으로 세기 계산] 버튼을 먼저 눌러 주세요.');
    m = await computeCond(speed, peak);
    updateCompView();
    return m;
  }
  if (modeNow() === 'ai') {
    await loadAiModel();
    m = magnitudeFor(speed, peak);
    if (m) { updateCompView(); return m; }
    throw new Error(AI.inputError || AI.error || 'AI 모델을 불러오지 못했습니다.');
  }
  throw new Error('보정표를 먼저 불러와 주세요.');
}

function updateCompView() {
  const mode = modeNow();
  // 속도는 어떤 방식에서든 줄별 가열 시간(길이 ÷ 속도)에 쓴다. 직접 입력에서는 세기 계산에 쓰지 않는다.
  $('speedText').textContent = mode === 'direct' ? '속도 mm/s (시간 표시용)' : '속도 mm/s';
  $('peakLabel').hidden = mode === 'direct';
  $('directBox').hidden = mode !== 'direct';
  $('tableBox').hidden = mode !== 'table';
  $('aiBox').hidden = mode !== 'ai';
  $('compRow').hidden = mode !== 'cond';
  $('compNote').hidden = mode !== 'cond';
  if (mode === 'ai') renderAiStatus();
  if (!S.plate) return;
  const v = $('compView');
  v.textContent = '';
  const m = magnitudeFor(num('speed'), num('peak'));
  if (!m) {
    v.textContent = mode === 'table' ? '보정표를 불러오면 세기가 정해집니다.'
      : mode === 'ai' ? (AI.inputError || AI.error || 'AI 모델을 불러오는 중입니다.')
      : (S.plate.fromFile ? '[이 조건으로 세기 계산]을 눌러 세기를 구해 주세요.' : '아직 세기를 계산하지 않았습니다. 가열선을 계산할 때 자동으로 구합니다.');
    return;
  }
  const b = (x, d) => { const e = document.createElement('b'); e.textContent = fix(x, d); return e; };
  v.append('각변형 ', b(m.theta_mrad, 2), ' mrad · 횡수축 ', b(m.dtrans_mm, 4), ' mm · 종수축 ', b(m.dlong_mm, 4), ' mm');
  if (Number.isFinite(m.theta_l_mrad)) v.append(' · 가열선 방향 각변형 ', b(m.theta_l_mrad, 2), ' mrad');
  else if (Number.isFinite(m.B_l)) v.append(' · 가열선 방향 각변형 ', b(1000 * Math.abs(m.B_l) * 12 / (S.plate.t ** 3), 2), ' mrad');
  const src = el('div', { class: 'hint' });
  if (m.source === 'cond') src.textContent = `빠른 단면 해석으로 구한 값 (표면 최고온도 ${fix(m.peak_c, 0)} ℃). 실측이 아닙니다.`;
  else if (m.source === 'table') src.textContent = `보정표에서 가장 가까운 항목: 두께 ${m.entry.thickness_mm} · 속도 ${m.entry.speed} · 최고온도 ${m.entry.peak_c}`;
  else if (m.source === 'ai') {
    const sigma = `불확실 폭(2σ) 각변형 ±${fix(2 * m.std.theta, 2)} mrad`;
    src.textContent = `AI 근사 모델의 계산값입니다 (계산 ${fix(m.ms, 2)} ms, ${sigma}). 정밀 3D 해석의 근사이며 실측이 아닙니다.`;
  } else src.textContent = '직접 넣은 값입니다. 기본값은 예시이며 어떤 판의 실측도 아닙니다.';
  v.append(src);
  if (m.source === 'ai' && m.warn?.length) v.append(el('div', { class: 'note-warn', text: `주의: ${m.warn.join(' · ')}.` }));
}

function renderAiStatus() {
  const note = $('aiNote'), st = $('aiStatus');
  if (AI.error) { note.hidden = false; note.textContent = AI.error; st.textContent = ''; return; }
  if (!AI.meta) { note.hidden = true; st.textContent = '모델을 불러오는 중입니다.'; return; }
  note.hidden = !AI.notes.length;
  note.textContent = AI.notes.join(' ');
  const d = AI.meta.data || {};
  st.textContent = `가상 해석 표본 ${d.n_D ?? '-'}건으로 학습한 시연용 모델입니다 (${AI.meta.created ?? ''}). 계산은 이 브라우저에서만 하고 서버로 보내지 않습니다.`;
}

// ── 가열선 ─────────────────────────────────────────────────
function polylineLength(pts) {
  let s = 0;
  for (let i = 0; i + 1 < pts.length; i++) s += Math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1], pts[i + 1][2] - pts[i][2]);
  return s;
}
function resample(pts, n) {
  const acc = [0];
  for (let i = 0; i + 1 < pts.length; i++) acc.push(acc[i] + Math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1], pts[i + 1][2] - pts[i][2]));
  const total = acc[acc.length - 1] || 1;
  const out = [];
  let k = 0;
  for (let s = 0; s < n; s++) {
    const d = total * s / (n - 1);
    while (k < pts.length - 2 && acc[k + 1] < d) k++;
    const u = clamp((d - acc[k]) / ((acc[k + 1] - acc[k]) || 1), 0, 1);
    out.push([0, 1, 2].map((c) => pts[k][c] + (pts[k + 1][c] - pts[k][c]) * u));
  }
  return out;
}
const normalizedPoints = (pts) => { const n = clamp(Math.ceil(polylineLength(pts) / 100) + 1, 13, 81); return pts.length === n ? pts : resample(pts, n); };
function surfacePoint(p, u, v) {
  const i0 = clamp(Math.floor(u), 0, p.nr - 2), j0 = clamp(Math.floor(v), 0, p.nc - 2);
  const s = u - i0, t = v - j0, P = p.points;
  return [0, 1, 2].map((k) => (1 - s) * ((1 - t) * P[i0][j0][k] + t * P[i0][j0 + 1][k]) + s * ((1 - t) * P[i0 + 1][j0][k] + t * P[i0 + 1][j0 + 1][k]));
}
function lineGap(a, b) {
  let best = Infinity;
  for (const p of a) for (const q of b) { const d = Math.hypot(p[0] - q[0], p[1] - q[1]); if (d < best) best = d; }
  return best;
}
function lineName(l) {
  if (l.family === 'across') return `폭방향 ${l.deg}°`;
  if (l.family === 'along') return '길이방향';
  if (l.family === 'file') return l.name ?? '파일 선';
  if (l.family === 'traced') return `추적선 ${l.deg}°`;
  return `직접 ${Math.round(l.deg)}°`;
}

function makeLine(fields) {
  const l = { id: S.nextId++, pickOrder: S.pickCounter++, weight: 1, side: 'top', speed: null, peak: null, ...fields };
  prepareLine(l);
  return l;
}

// 줄마다 화면용 자료를 미리 만든다: 중심 기준 좌표, 평면 법선, 가까운 표시 격자점, 열 표시용 거리.
function prepareLine(l) {
  const p = S.plate, pts = l.points, n = pts.length, nv = p.rnr * p.rnc, Xr = p.Xr;
  const c = new Float32Array(3 * n);
  for (let s = 0; s < n; s++) { c[3 * s] = pts[s][0] - p.center[0]; c[3 * s + 1] = pts[s][1] - p.center[1]; c[3 * s + 2] = pts[s][2] - p.center[2]; }
  const nrm = new Float32Array(2 * n);
  for (let s = 0; s < n; s++) {
    const a = Math.max(s - 1, 0), b = Math.min(s + 1, n - 1);
    const tx = c[3 * b] - c[3 * a], ty = c[3 * b + 1] - c[3 * a + 1], len = Math.hypot(tx, ty) || 1;
    nrm[2 * s] = -ty / len;
    nrm[2 * s + 1] = tx / len;
  }
  const near = new Int32Array(n);
  for (let s = 0; s < n; s++) {
    let best = Infinity, who = 0;
    for (let v = 0; v < nv; v++) {
      const dx = Xr[3 * v] - c[3 * s], dy = Xr[3 * v + 1] - c[3 * s + 1], d = dx * dx + dy * dy;
      if (d < best) { best = d; who = v; }
    }
    near[s] = who;
  }
  const acc = new Float64Array(n);
  for (let s = 1; s < n; s++) acc[s] = acc[s - 1] + Math.hypot(c[3 * s] - c[3 * s - 3], c[3 * s + 1] - c[3 * s - 2]);
  const total = acc[n - 1] || 1;
  const sv = new Float32Array(nv), dv = new Float32Array(nv);
  for (let v = 0; v < nv; v++) {
    const x = Xr[3 * v], y = Xr[3 * v + 1];
    let best = Infinity, at = 0;
    for (let s = 0; s + 1 < n; s++) {
      const ax = c[3 * s], ay = c[3 * s + 1], ex = c[3 * s + 3] - ax, ey = c[3 * s + 4] - ay;
      const u = clamp(((x - ax) * ex + (y - ay) * ey) / (ex * ex + ey * ey || 1), 0, 1);
      const dx = x - (ax + u * ex), dy = y - (ay + u * ey), d = dx * dx + dy * dy;
      if (d < best) { best = d; at = acc[s] + u * (acc[s + 1] - acc[s]); }
    }
    sv[v] = at / total;
    dv[v] = Math.sqrt(best);
  }
  l._c = c; l._n = nrm; l._near = near; l._sv = sv; l._dv = dv; l._len = total;
  l._acc3 = cumulativeLength(pts);
  let mx = 0, my = 0;
  for (let s = 0; s < n; s++) { mx += pts[s][0]; my += pts[s][1]; }
  mx /= n; my /= n;
  l._posKey = l.family === 'along' ? mx * p.colAxis[0] + my * p.colAxis[1] : mx * p.rowAxis[0] + my * p.rowAxis[1];
}

function fieldOf(disp) {
  const p = S.plate;
  const z = zPlaneFree(disp, p.X);
  const free = removeRigid(disp, p.X);
  for (let i = 0; i < z.length; i++) free[3 * i + 2] = z[i];
  return { z, render: upsample3(free, p) };
}

// 줄을 SEGMENTS 구간으로 나눠 구간별 응답을 구한다. 구간을 다 더하면 줄 전체 응답과 정확히 같다 (선형 해석).
async function ensureResponses(lines) {
  await ensureConditions(lines);
  const todo = lines.filter((l) => !l._resp || l._resp.key !== respKey(l));
  if (!todo.length) return;
  const p = S.plate, nv = 3 * p.rnr * p.rnc, nn = p.nr * p.nc;
  let done = 0;
  const tick = lines.length > 4;
  for (const l of todo) {
    const comp = compOf(l), key = respKey(l), n = l.points.length;
    const segs = Math.min(SEGMENTS, n - 1);
    const cumR = [new Float32Array(nv)], cumZ = [new Float64Array(nn)];
    for (let s = 0; s < segs; s++) {
      const a = Math.round(s * (n - 1) / segs), b = Math.round((s + 1) * (n - 1) / segs);
      // 끝단 구간은 줄 전체 기준이므로 조각을 잘라 보내지 않고 gate 로 구간만 고른다. 구간을 다 더하면 줄 전체와 같다.
      const out = await call({ type: 'run', lines: [{ points: l.points, comp, scale: 1, gate: { from: a, to: b } }], id: ++runId }, 'result');
      const f = fieldOf(out.disp);
      const R = Float32Array.from(cumR[s]), Z = Float64Array.from(cumZ[s]);
      for (let i = 0; i < nv; i++) R[i] += f.render[i];
      for (let i = 0; i < nn; i++) Z[i] += f.z[i];
      cumR.push(R);
      cumZ.push(Z);
    }
    l._resp = { key, n: segs, cumR, cumZ };
    done++;
    setProgress(done / todo.length);
    if (tick) setMsg(`가열선 해석 ${done} / ${todo.length}`);
    await yieldFrame();
  }
}

function sortLines() {
  if ($('order').value === 'pick') { S.lines.sort((a, b) => a.pickOrder - b.pickOrder); return; }
  const sideRank = (l) => (l.side === 'bottom' ? 1 : 0);
  const famRank = (l) => (l.family === 'along' ? 1 : 0);
  S.lines.sort((a, b) => sideRank(a) - sideRank(b) || famRank(a) - famRank(b) || a._posKey - b._posKey);
}

function rebuildCumulative() {
  const p = S.plate, nv = 3 * p.rnr * p.rnc, nn = p.nr * p.nc, N = S.lines.length;
  // 줄의 세기·면·응답이 바뀌면 재생 위치가 같아도 곡면이 달라지므로 곡률 색 캐시를 비운다.
  curvState = { p: null, key: '', classes: null, at: 0 };
  S.cumR = [new Float32Array(nv)];
  S.cumZ = [new Float64Array(nn)];
  for (let k = 0; k < N; k++) {
    const l = S.lines[k], r = l._resp, w = l.weight;
    const R = Float32Array.from(S.cumR[k]), Z = Float64Array.from(S.cumZ[k]);
    const a = r.cumR[r.n], b = r.cumZ[r.n];
    for (let i = 0; i < nv; i++) R[i] += w * a[i];
    for (let i = 0; i < nn; i++) Z[i] += w * b[i];
    S.cumR.push(R);
    S.cumZ.push(Z);
  }
  S.rmsSeries = [];
  S.maxSeries = [];
  const e = new Float64Array(nn);
  for (let k = 0; k <= N; k++) {
    for (let i = 0; i < nn; i++) e[i] = p.D[i] - S.cumZ[k][i];
    S.rmsSeries.push(rms(e));
    S.maxSeries.push(maxAbs(e));
  }
  updateCurl();
}

// 곡 펴짐 지표(박정서 외 2008): 지금 재생 순서로 줄을 더하면서 방향별 주 굽힘 변형 제곱합을 기록한다. 회차별 추세만 본다.
function updateCurl() {
  const p = S.plate;
  S.curl = null;
  if (!p || S.lines.length < 1 || !S.lines.every((l) => l._resp)) return;
  try {
    S.curl = curlSeries(p.points, S.lines.map((l) => ({ weight: l.weight, zResp: l._resp.cumZ[l._resp.n] })), p.t, { window: p.kinWindow && p.kinWindow > 1 ? p.kinWindow : 1 });
  } catch { S.curl = null; }
}

function afterLinesChanged() {
  sortLines();
  rebuildCumulative();
  S.T = clamp(S.T, 0, S.lines.length);
  rebuildRibbons();
  renderList();
  drawChart();
  syncTimeline();
  syncButtons();
  updateCompView();
  $('viewHint').hidden = S.lines.length > 0;
  markDirty();
  updateReadouts(true);
}

// 자동 추천: 면(윗면/아랫면)과 방향(폭/길이) 묶음마다 recommend 를 호출하고, 마지막에 세기를 다시 맞춘다.
// recommend 한 번은 서로 교차하는 선을 함께 고르지 못하므로 묶음별로 나눠 부른다.
async function autoRecommend() {
  const p = S.plate;
  // 재생 중이면 멈춘다. 새 줄은 응답 계산이 끝난 뒤에 한 번에 바꿔 넣는다 (계산 도중 화면이 빈 응답을 읽지 않게).
  S.playing = false;
  syncTimeline();
  const mode = $('recMode').value;
  if (mode !== 'fixed') { await autoRecommendShin(mode); return; }
  const base = await baseMagnitude();
  const sides = $('sides').value === 'both' ? ['top', 'bottom'] : ['top'];
  const fams = [$('famAcross').checked && 'across', $('famAlong').checked && 'along'].filter(Boolean);
  if (!fams.length) throw new Error('폭방향이나 길이방향 중 하나는 골라 주세요.');
  const maxLines = clamp(Math.round(num('maxLines')), 1, 40);
  const wmax = Math.max(0.1, num('wmax'));
  const minGap = Math.max(0, num('minGap'));
  const pitch = Math.max(20, num('pitch'));
  const D = p.D;
  const E0 = dot(D, D);
  if (E0 < 1e-6) { setMsg('목표와 시작 형상이 같아서 고를 가열선이 없습니다.'); return; }
  const lineZ = async (points, comp) => {
    const r = await call({ type: 'run', lines: [{ points, comp, scale: 1 }], id: ++runId }, 'result');
    return zPlaneFree(r.disp, p.X);
  };
  const R = Float64Array.from(D);
  const share = Math.max(2, Math.ceil(maxLines / (sides.length * fams.length)));
  const stageTotal = ROUNDS * sides.length * fams.length;
  let stageDone = 0, budget = maxLines;
  const candidateCount = new Map();
  const picked = [];
  for (let round = 0; round < ROUNDS && budget > 0; round++) {
    let any = false;
    for (const side of sides) for (const family of fams) {
      stageDone++;
      if (budget <= 0) continue;
      const ER = dot(R, R);
      if (ER < 1e-9 * E0) continue;
      const fraction = (stageDone - 1) / stageTotal;
      setProgress(fraction);
      setMsg(`가열선 찾는 중: ${side === 'top' ? '윗면' : '아랫면'} ${family === 'across' ? '폭방향' : '길이방향'} (${stageDone}/${stageTotal})`);
      const comp = compFor(base, side);
      const res = await call({
        type: 'recommend', D: R, comp,
        options: { maxLines: Math.min(share, budget), minGap, wmax, pitch, families: [family], minGain: Math.max(0.01, 0.002 * E0 / ER) },
      }, 'recommended', (v) => setProgress(fraction + v / stageTotal));
      candidateCount.set(family, res.candidates);
      for (const ln of res.lines) {
        const key = `${side}|${family}|${ln.deg}|${ln.points[0][0].toFixed(1)},${ln.points[0][1].toFixed(1)}`;
        let rec = picked.find((q) => q.key === key);
        if (!rec) {
          if (budget <= 0) break;
          rec = { key, side, family: ln.family, deg: ln.deg, points: ln.points, z: await lineZ(ln.points, comp) };
          picked.push(rec);
          budget--;
        }
        for (let i = 0; i < R.length; i++) R[i] -= ln.weight * rec.z[i];
        any = true;
      }
    }
    if (!any) break;
  }
  if (!picked.length) {
    setMsg($('sides').value === 'top' ? '윗면 가열만으로는 이 목표를 만들 선을 찾지 못했습니다. [양면]으로 바꿔 보세요.' : '이 목표에 맞는 가열선을 찾지 못했습니다.');
    S.lines = [];
    afterLinesChanged();
    return;
  }
  const first = boundedFit(picked.map((q) => q.z), D, wmax, 200);
  const kept = picked.map((q, i) => ({ ...q, w: first.w[i] })).filter((q) => q.w > 0.08).sort((a, b) => b.w - a.w);
  const finalPick = [];
  for (const q of kept) {
    if (finalPick.some((o) => o.side === q.side && o.family === q.family && lineGap(o.points, q.points) < minGap)) continue;
    finalPick.push(q);
  }
  const fit = boundedFit(finalPick.map((q) => q.z), D, wmax, 200);
  finalPick.forEach((q, i) => { q.w = fit.w[i]; });
  const chosen = finalPick.filter((q) => q.w > 0.05);
  setMsg(`가열선 ${chosen.length}줄을 재생할 수 있게 계산합니다.`);
  const next = chosen.map((q) => makeLine({ family: q.family, deg: q.deg, points: q.points, weight: q.w, side: q.side, origin: 'auto' }));
  await ensureResponses(next);
  S.lines = next;
  S.T = 0;
  afterLinesChanged();
  setProgress(0);
  const t = S.rmsSeries;
  const candidates = [...candidateCount.values()].reduce((a, b) => a + b, 0);
  setMsg(`후보 ${candidates}개에서 ${chosen.length}줄을 골랐습니다. 남은 RMS ${fix(t[0])} → ${fix(t[t.length - 1])} mm (해석 모델 값). [재생]을 눌러 보세요.`);
}

// 추적선·혼합 추천: recommendShin 을 작업자에서 부른다. 혼합은 혼합·직선만·추적선만 중 남은 편차가 가장 작은 결과를 쓴다.
async function autoRecommendShin(mode) {
  const p = S.plate;
  const base = await baseMagnitude();
  const sides = $('sides').value === 'both' ? ['top', 'bottom'] : ['top'];
  const fams = [$('famAcross').checked && 'across', $('famAlong').checked && 'along'].filter(Boolean);
  if (mode === 'mixed' && !fams.length) throw new Error('혼합은 직선 후보가 필요합니다. 폭방향이나 길이방향 중 하나는 골라 주세요.');
  if (dot(p.D, p.D) < 1e-6) { setMsg('목표와 시작 형상이 같아서 고를 가열선이 없습니다.'); return; }
  const options = {
    maxLines: clamp(Math.round(num('maxLines')), 1, 40),
    minGap: Math.max(0, num('minGap')),
    wmax: Math.max(0.1, num('wmax')),
    pitch: Math.max(20, num('pitch')),
    families: fams, sides, thickness: p.t,
    fixed: mode === 'mixed', traced: true, poolBest: mode === 'mixed',
    stop: $('stopRule').checked ? undefined : false,
    window: p.fromFile ? 'auto' : 1,
    curl: false,
  };
  setMsg(mode === 'traced' ? '추적선 후보를 만드는 중입니다.' : '직선과 추적선 후보를 만드는 중입니다.');
  const comp = compFor(base, 'top');
  const res = await call({ type: 'recommend_shin', D: p.D, target: p.target, comp, options }, 'recommended_shin', (v) => setProgress(v));
  if (!res.lines.length) {
    setMsg($('sides').value === 'top' ? '윗면 가열만으로는 이 목표를 만들 선을 찾지 못했습니다. [양면]으로 바꿔 보세요.' : '이 목표에 맞는 가열선을 찾지 못했습니다.');
    S.lines = [];
    afterLinesChanged();
    return;
  }
  // 줄 순서는 planner_shin.js 의 작업 순서(면별로 묶음)를 따른다.
  const ordered = res.lines.slice().sort((a, b) => (a.order ?? 0) - (b.order ?? 0));
  setMsg(`가열선 ${ordered.length}줄을 재생할 수 있게 계산합니다.`);
  const next = ordered.map((q) => makeLine({ family: q.family, deg: q.deg, points: q.points, weight: q.weight, side: q.side, origin: 'auto' }));
  await ensureResponses(next);
  S.lines = next;
  S.T = 0;
  afterLinesChanged();
  setProgress(0);
  const r = S.rmsSeries;
  const name = mode === 'traced' ? '추적선' : '혼합';
  const pool = mode === 'mixed' && res.pool && res.pool !== '혼합' ? ` (묶음 중 ${res.pool === '고정만' ? '직선만' : '추적선만'}이 가장 나았음)` : '';
  const cut = res.stop && res.stop.kept < res.stop.pickedAll ? ` 정지 규칙으로 ${res.stop.pickedAll}줄 중 ${res.stop.kept}줄만 썼습니다.` : '';
  setMsg(`${name}: 후보 ${res.candidates}개(추적선 ${res.counts.tracedLines}개)에서 ${next.length}줄을 골랐습니다${pool}. 남은 RMS ${fix(r[0])} → ${fix(r[r.length - 1])} mm (해석 모델 값).${cut} 추적선은 시작점에 따라 결과가 달라집니다.`);
}

// ── 화면 갱신 ──────────────────────────────────────────────
function timeState(T) {
  const N = S.lines.length;
  const k = Math.min(Math.floor(T + 1e-9), N);
  if (k >= N) return { k: N, fp: 0, cool: 0, frac: 0 };
  const frac = T - k;
  const fp = Math.min(1, frac / TORCH_SHARE);
  const cool = frac > TORCH_SHARE ? (frac - TORCH_SHARE) / (1 - TORCH_SHARE) : 0;
  return { k, fp, cool, frac };
}

function computeU() {
  const p = S.plate, U = p.U, nv = p.rnr * p.rnc;
  if (S.view === 'before') { U.fill(0); return; }
  if (S.view === 'target') {
    for (let i = 0; i < nv; i++) { U[3 * i] = 0; U[3 * i + 1] = 0; U[3 * i + 2] = p.Dr[i]; }
    return;
  }
  const { k, fp } = timeState(S.T);
  U.set(S.cumR[k]);
  if (k < S.lines.length && fp > 0) {
    const l = S.lines[k], r = l._resp, w = l.weight;
    const x = fp * r.n, s = Math.min(r.n - 1, Math.floor(x)), u = x - s;
    const a = r.cumR[s], b = r.cumR[s + 1];
    for (let i = 0; i < U.length; i++) U[i] += w * (a[i] + u * (b[i] - a[i]));
  }
}

function colorRange() {
  const v = $('colorRange').value;
  if (v !== 'auto') return parseFloat(v);
  const p = S.plate;
  return p && p.max0 > 0.5 ? p.max0 : 1;
}

function writeSurface() {
  const p = S.plate, pos = p.posAttr.array, X = p.Xr, U = p.U, e = S.exag;
  for (let i = 0; i < pos.length; i++) pos[i] = X[i] + e * U[i];
  p.posAttr.needsUpdate = true;
  p.surface.geometry.computeVertexNormals();
  const ring = p.ring, sp = p.skirtAttr.array;
  for (let r = 0; r < ring.length; r++) {
    const v = ring[r], x = pos[3 * v], y = pos[3 * v + 1], z = pos[3 * v + 2];
    sp[6 * r] = x; sp[6 * r + 1] = y; sp[6 * r + 2] = z;
    sp[6 * r + 3] = x; sp[6 * r + 4] = y; sp[6 * r + 5] = z - p.t;
  }
  p.skirtAttr.needsUpdate = true;
  p.skirt.geometry.computeVertexNormals();
  p.ghost.visible = $('ghost').checked && S.view !== 'target';
  if (p.ghost.visible) {
    const g = p.ghostAttr.array;
    for (let i = 0; i < p.rnr * p.rnc; i++) { g[3 * i] = X[3 * i]; g[3 * i + 1] = X[3 * i + 1]; g[3 * i + 2] = X[3 * i + 2] + e * p.Dr[i]; }
    p.ghostAttr.needsUpdate = true;
  }
  if (ground) ground.position.z = p.bbox.zmin - p.t - e * p.max0 - p.diag * 0.018;
}

const NEUTRAL = new THREE.Color('#aeb6c1');

// ── 곡률 부호 색 (E02 식 1·2) ───────────────────────────────
// 안장 K<0, 볼록 K>0, 실린더 K≈0(H≠0), 평판. 격자 잡음 처리는 kinematics.js 의 판정 여유를 그대로 쓴다.
const CURV_HEX = { light: ['#d9681d', '#2f7fc1', '#4ba152', '#c9ced6'], dark: ['#f59344', '#6ca9e8', '#78c37f', '#6b7482'] };
let curvCol = [];
let curvState = { p: null, key: '', classes: null, at: 0 };
function buildCurvColors() {
  const hex = S.dark ? CURV_HEX.dark : CURV_HEX.light;
  curvCol = hex.map((h) => new THREE.Color(h));
  document.querySelectorAll('[data-curv]').forEach((node) => { node.style.background = hex[+node.dataset.curv]; });
}
// 지금 보는 곡면의 분류 (입력 격자 기준). 재생 중에는 초당 8번까지만 다시 계산한다.
function curvClassesNow() {
  const p = S.plate;
  const key = S.view === 'after' && S.lines.length ? `a${S.T.toFixed(3)}` : S.view;
  const now = performance.now();
  if (curvState.p === p && curvState.key === key) return curvState.classes;
  if (curvState.p === p && curvState.classes && S.playing && now - curvState.at < 120) return curvState.classes;
  const X = Float64Array.from(p.X);
  if (S.view === 'target') for (let i = 0; i < p.D.length; i++) X[3 * i + 2] += p.D[i];
  else if (S.view === 'after') { zAt(S.T, p.zNow); for (let i = 0; i < p.zNow.length; i++) X[3 * i + 2] += p.zNow[i]; }
  let classes = null;
  try { classes = classifyPlate({ X, nr: p.nr, nc: p.nc }, { noiseSigma: p.noiseSigma, window: p.kinWindow > 1 ? p.kinWindow : undefined }).classes; } catch { classes = null; }
  curvState = { p, key, classes, at: now };
  return classes;
}
function writeCurvColors(col) {
  const p = S.plate, classes = curvClassesNow();
  const nv = p.rnr * p.rnc, m = p.m, nc = p.nc, nr = p.nr;
  for (let v = 0; v < nv; v++) {
    const i = Math.min(nr - 1, Math.round(Math.floor(v / p.rnc) / m)), j = Math.min(nc - 1, Math.round((v % p.rnc) / m));
    const c = (classes && curvCol[classes[i * nc + j]]) || NEUTRAL;
    col[3 * v] = c.r; col[3 * v + 1] = c.g; col[3 * v + 2] = c.b;
  }
}

function writeColors() {
  const p = S.plate, col = p.colAttr.array, nv = p.rnr * p.rnc, U = p.U;
  const on = $('devColor').checked, curv = on && S.colorKind === 'curv';
  const showDev = on && !curv && S.view !== 'target';
  $('legend').hidden = !showDev;
  $('legendCurv').hidden = !curv;
  if (curv) {
    $('legendCurvTitle').textContent = `곡률 부호 (${S.view === 'target' ? '목표 곡면' : S.view === 'before' ? '가공 전 곡면' : '지금 곡면'})`;
    writeCurvColors(col);
  } else if (!showDev) {
    for (let i = 0; i < nv; i++) { col[3 * i] = NEUTRAL.r; col[3 * i + 1] = NEUTRAL.g; col[3 * i + 2] = NEUTRAL.b; }
  } else {
    const range = colorRange();
    for (let i = 0; i < nv; i++) {
      const dev = U[3 * i + 2] - p.Dr[i];
      const t = clamp((dev / range + 1) / 2, 0, 1) * 255 | 0;
      col[3 * i] = divLUT[3 * t]; col[3 * i + 1] = divLUT[3 * t + 1]; col[3 * i + 2] = divLUT[3 * t + 2];
    }
  }
  p.colAttr.needsUpdate = true;
}

// 토치가 지나간 자리에 열 색이 남았다가 식는 시각 효과. 온도 계산이 아니다.
function writeHeat() {
  const p = S.plate, heat = p.heatAttr.array, nv = p.rnr * p.rnc;
  const { k, fp, cool } = timeState(S.T);
  const active = S.view === 'after' && k < S.lines.length && fp > 0 && cool < 1;
  p.heatMesh.visible = active;
  if (!active) return;
  const l = S.lines[k], sv = l._sv, dv = l._dv, L = l._len;
  const w0 = p.heatWidth, tail = Math.max(0.6 * L, 900), front = 50;
  const fade = (1 - cool) * (l.side === 'bottom' ? 0.55 : 1);
  for (let v = 0; v < nv; v++) {
    const d = dv[v];
    let H = 0;
    if (d < w0 * 3.2) {
      const dist = (fp - sv[v]) * L;
      if (dist >= 0) {
        const wid = w0 * (1 + 1.3 * dist / tail);
        H = Math.exp(-dist / tail) * Math.exp(-(d * d) / (wid * wid));
      } else {
        H = 0.65 * Math.exp(dist / front) * Math.exp(-(d * d) / (w0 * w0));
      }
    }
    H *= fade;
    const t = clamp(H, 0, 1) * 255 | 0;
    heat[4 * v] = heatLUT[3 * t]; heat[4 * v + 1] = heatLUT[3 * t + 1]; heat[4 * v + 2] = heatLUT[3 * t + 2];
    heat[4 * v + 3] = H < 0.015 ? 0 : clamp(H * 1.6, 0, 0.95);
  }
  p.heatAttr.needsUpdate = true;
}

function rebuildRibbons() {
  if (ribbon) { sceneGroup.remove(ribbon); ribbon.geometry.dispose(); ribbon = null; }
  if (bandMesh) { sceneGroup.remove(bandMesh); bandMesh.geometry.dispose(); bandMesh = null; }
  const lines = S.lines;
  if (!lines.length) return;
  let nv = 0;
  for (const l of lines) nv += 2 * l.points.length;
  const idx = [];
  let base = 0;
  for (const l of lines) {
    const n = l.points.length;
    l._base = base;
    for (let s = 0; s < n - 1; s++) {
      if (l.side === 'bottom' && s % 2) continue;   // 아랫면 선은 끊어서 표시
      const a = base + 2 * s;
      idx.push(a, a + 2, a + 1, a + 1, a + 2, a + 3);
    }
    base += 2 * n;
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(3 * nv), 3).setUsage(THREE.DynamicDrawUsage));
  geo.setAttribute('color', new THREE.BufferAttribute(new Float32Array(4 * nv), 4).setUsage(THREE.DynamicDrawUsage));
  geo.setIndex(idx);
  ribbon = new THREE.Mesh(geo, ribbonMat);
  ribbon.frustumCulled = false;
  ribbon.renderOrder = 3;
  sceneGroup.add(ribbon);
  S.lastPainted = -2;
  paintRibbons();
  buildEndBands();
}

// 끝단 구간 띠: 선 양 끝의 끝단 길이만큼을 넓은 띠로 표시한다. 재생 계산은 그대로이고, 보기 위한 표시다.
function buildEndBands() {
  const ez = endZoneNow();
  if (!ez || !S.lines.length) return;
  const idx = [];
  let pairs = 0;
  for (const l of S.lines) {
    l._bandI = [];
    const spans = endZoneSpans(l.points, ez.length_mm);
    if (!spans) continue;
    for (const sp of spans) {
      const m = clamp(Math.ceil((sp.to - sp.from) / 25), 2, 60);
      for (let q = 0; q <= m; q++) l._bandI.push(arcIndex(l._acc3, sp.from + (sp.to - sp.from) * q / m));
      for (let q = 0; q < m; q++) { const a = 2 * (pairs + q); idx.push(a, a + 2, a + 1, a + 1, a + 2, a + 3); }
      pairs += m + 1;
    }
  }
  if (!pairs) return;
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6 * pairs), 3).setUsage(THREE.DynamicDrawUsage));
  geo.setIndex(idx);
  bandMesh = new THREE.Mesh(geo, bandMat);
  bandMesh.frustumCulled = false;
  bandMesh.renderOrder = 2.5;
  sceneGroup.add(bandMesh);
}

function paintRibbons() {
  if (!ribbon) return;
  const { k } = timeState(S.T);
  S.lastPainted = k;
  const col = ribbon.geometry.attributes.color.array;
  const done = new THREE.Color(S.dark ? '#8fc0ff' : '#1f5fbf');
  const now = new THREE.Color('#ff7a1a');
  const wait = new THREE.Color(S.dark ? '#b7c1cf' : '#68707e');
  S.lines.forEach((l, i) => {
    const c = i < k ? done : i === k ? now : wait;
    const a = i === k ? 1 : i < k ? 0.95 : 0.72;
    for (let s = 0; s < 2 * l.points.length; s++) {
      const o = 4 * (l._base + s);
      col[o] = c.r; col[o + 1] = c.g; col[o + 2] = c.b; col[o + 3] = a;
    }
  });
  ribbon.geometry.attributes.color.needsUpdate = true;
}

function updateRibbons() {
  const p = S.plate, U = p.U, e = S.exag;
  ribbon.visible = S.view !== 'target';
  if (!ribbon.visible) return;
  const pos = ribbon.geometry.attributes.position.array;
  const hw = p.ribbonHalf, lift = 4 * (1 + e / 60);
  let o = 0;
  for (const l of S.lines) {
    const n = l.points.length, c = l._c, nr = l._n, near = l._near;
    if (!l._cur || l._cur.length !== 3 * n) l._cur = new Float32Array(3 * n);
    const cur = l._cur;
    for (let s = 0; s < n; s++) {
      const v = near[s];
      const x = c[3 * s] + e * U[3 * v], y = c[3 * s + 1] + e * U[3 * v + 1], z = c[3 * s + 2] + e * U[3 * v + 2] + lift;
      cur[3 * s] = x; cur[3 * s + 1] = y; cur[3 * s + 2] = z;
      const nx = nr[2 * s] * hw, ny = nr[2 * s + 1] * hw;
      pos[o++] = x + nx; pos[o++] = y + ny; pos[o++] = z;
      pos[o++] = x - nx; pos[o++] = y - ny; pos[o++] = z;
    }
  }
  ribbon.geometry.attributes.position.needsUpdate = true;
  if (bandMesh) {
    bandMesh.visible = ribbon.visible;
    const bp = bandMesh.geometry.attributes.position.array, bw = hw * 2.4;
    let w = 0;
    for (const l of S.lines) {
      if (!l._bandI?.length) continue;
      const n = l.points.length, cur = l._cur, nr = l._n;
      for (const fi of l._bandI) {
        const k = Math.min(n - 2, Math.floor(fi)), u = fi - k;
        const x = cur[3 * k] + u * (cur[3 * k + 3] - cur[3 * k]), y = cur[3 * k + 1] + u * (cur[3 * k + 4] - cur[3 * k + 1]), z = cur[3 * k + 2] + u * (cur[3 * k + 5] - cur[3 * k + 2]) - 1;
        const nx = nr[2 * k] + u * (nr[2 * k + 2] - nr[2 * k]), ny = nr[2 * k + 1] + u * (nr[2 * k + 3] - nr[2 * k + 1]);
        bp[w++] = x + nx * bw; bp[w++] = y + ny * bw; bp[w++] = z;
        bp[w++] = x - nx * bw; bp[w++] = y - ny * bw; bp[w++] = z;
      }
    }
    bandMesh.geometry.attributes.position.needsUpdate = true;
  }
}

// ── 곡판 분류 안내와 주방향 바늘 ────────────────────────────
function refreshShapeInfo() {
  const p = S.plate;
  if (!p) return;
  p.classify = null; p.classifyError = null; p.kin = null; p.kinError = null; p.kinWindow = 1; p.noiseSigma = undefined;
  try {
    // 만들 변형의 방향장 (추적선 추천과 같은 계산: 막 에너지 최소로 전개 대용)
    p.kin = prepareKinematics(p.points, p.target, { thickness: p.t, rule: 'auto', window: p.fromFile ? 'auto' : 1 });
    p.kinWindow = p.kin.window;
  } catch (err) { p.kinError = err.message; }
  try {
    p.classify = classifyPlate(p.target, { window: p.kinWindow > 1 ? p.kinWindow : undefined });
    p.noiseSigma = p.classify.noise.sigma;
  } catch (err) { p.classifyError = err.message; }
  renderClassCard();
  buildNeedles();
  curvState = { p: null, key: '', classes: null, at: 0 };
  markDirty();
}

const TASK_NAME = { 롤: '롤 벤딩', 선상: '선상가열', 삼각: '삼각가열' };
function renderClassCard() {
  const p = S.plate, box = $('classBody');
  box.textContent = '';
  if (!p) return;
  const c = p.classify;
  if (!c) { box.append(el('p', { class: 'hint', text: `이 판은 분류하지 못했습니다.${p.classifyError ? ` (${p.classifyError})` : ''}` })); return; }
  const names = ['안장', '볼록', '실린더', '평판'];
  box.append(el('div', { class: 'class-name', text: c.shapeName === '평판' ? '평판' : `${c.shapeName}형 판` }));
  const bars = el('div', { class: 'class-bars' });
  names.forEach((name, k) => {
    const pct = Math.round(c.yRaw[k] * 100);
    const track = el('div', { class: 'track' });
    const fill = el('i');
    fill.style.width = `${clamp(c.yRaw[k] * 100, 0, 100)}%`;
    track.append(fill);
    const weak = c.yRaw[k] > 0 && c.y[k] === 0;
    bars.append(el('span', { text: name }), track, el('span', { text: `${pct}%${weak ? ' (5% 미만, 0으로 처리)' : ''}` }));
  });
  box.append(bars);
  const chips = el('div', { class: 'task-chips' });
  if (!c.tasks.length) chips.append(el('span', { class: 'chip', text: '가공 없음' }));
  for (const task of c.tasks) {
    const chip = el('span', { class: `chip${task === '삼각' ? ' off' : ''}`, text: TASK_NAME[task] ?? task });
    if (task === '삼각') chip.title = '삼각가열은 이 화면의 자동 추천 범위 밖입니다.';
    chips.append(chip);
  }
  box.append(chips);
  const [saddle, convex] = c.z;
  let tip;
  if (!c.tasks.length) tip = '곡가공이 필요 없는 판입니다.';
  else if (!saddle && !convex) tip = '한 방향으로만 굽은 판입니다. 롤 벤딩으로 만드는 형상이고, 선상가열은 마무리 보조로 봅니다.';
  else if (saddle && !convex) tip = '안장 성분이 있는 판입니다. 선상가열을 쓰는 형상이고, 면내 변형은 판 안쪽에 몰립니다.';
  else if (!saddle && convex) tip = '볼록 성분이 있는 판입니다. 면내 변형이 판 경계부에 몰려 보통 삼각가열을 씁니다. 이 화면의 자동 추천은 선상가열만 다루므로 남는 편차가 크게 나올 수 있습니다.';
  else tip = '안장과 볼록 성분이 섞인 판입니다. 안장 쪽은 선상가열, 볼록 쪽은 삼각가열로 나눠 봅니다. 자동 추천은 선상가열만 다룹니다.';
  box.append(el('p', { class: 'hint', text: tip }));
  if (!c.noise.reliable) box.append(el('p', { class: 'note-warn', text: `곡률 잡음이 커서(잡음 비율 ${c.noise.ratio.toFixed(2)}) 이 분류는 믿기 어렵습니다. 격자를 다시 만들거나 매끄럽게 한 뒤 보세요.` }));
}

// 주방향 바늘: 이 위치에서 가열선을 그을 방향 (만들 변형이 큰 주방향에 수직). 색은 면내·굽힘 중 어느 쪽이 큰지.
const NEEDLE_RGB = { light: { bend: '#20262e', mem: '#c026a3' }, dark: { bend: '#eef1f5', mem: '#f472d4' } };
function buildNeedles() {
  const p = S.plate;
  if (needleObj) { sceneGroup.remove(needleObj); needleObj.geometry.dispose(); needleObj = null; }
  p.needles = [];
  const K = p.kin?.S;
  if (!K) return;
  const f = K.dirField;
  const ds = ((p.dsRow + p.dsCol) / 2) || 1;
  const every = Math.max(1, Math.round((p.diag / 26) / ds));
  const v = [];
  for (let q = 0; q < f.mag.length; q++) if (K.trusted[q]) v.push(f.mag[q]);
  v.sort((a, b) => a - b);
  const magRef = v.length ? v[Math.floor(0.99 * (v.length - 1))] : 0;
  if (!(magRef > 0)) return;
  p.needleLen = 0.72 * every * ds;
  const { nr, nc } = p;
  const first = Math.floor(every / 2);
  for (let i = first; i < nr; i += every) for (let j = first; j < nc; j += every) {
    const q = i * nc + j;
    if (!K.trusted[q] || f.mag[q] < 0.05 * magRef || f.anis[q] < 0.02) continue;
    p.needles.push({ v: i * p.m * p.rnc + j * p.m, c: Math.cos(f.psi[q]), s: Math.sin(f.psi[q]), mem: f.src[q] === 0 });
  }
  if (!p.needles.length) return;
  const pal = S.dark ? NEEDLE_RGB.dark : NEEDLE_RGB.light, bend = new THREE.Color(pal.bend), mem = new THREE.Color(pal.mem);
  const colors = new Float32Array(6 * p.needles.length);
  p.needles.forEach((n, k) => {
    const c = n.mem ? mem : bend;
    for (let e = 0; e < 2; e++) { colors[6 * k + 3 * e] = c.r; colors[6 * k + 3 * e + 1] = c.g; colors[6 * k + 3 * e + 2] = c.b; }
  });
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6 * p.needles.length), 3).setUsage(THREE.DynamicDrawUsage));
  geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  needleObj = new THREE.LineSegments(geo, needleMat);
  needleObj.frustumCulled = false;
  needleObj.renderOrder = 4;
  needleObj.visible = false;
  sceneGroup.add(needleObj);
}
function updateNeedles() {
  const p = S.plate;
  if (!needleObj) return;
  needleObj.visible = S.showNeedles;
  if (!S.showNeedles) return;
  const pos = p.posAttr.array, arr = needleObj.geometry.attributes.position.array, rnr = p.rnr, rnc = p.rnc;
  const half = p.needleLen / 2, lift = 4 * (1 + S.exag / 60);
  p.needles.forEach((n, k) => {
    const v = n.v, i = Math.floor(v / rnc), j = v % rnc;
    const ip = Math.min(i + 1, rnr - 1) * rnc + j, im = Math.max(i - 1, 0) * rnc + j, jp = i * rnc + Math.min(j + 1, rnc - 1), jm = i * rnc + Math.max(j - 1, 0);
    const ax = pos[3 * ip] - pos[3 * im], ay = pos[3 * ip + 1] - pos[3 * im + 1], az = pos[3 * ip + 2] - pos[3 * im + 2];
    const bx = pos[3 * jp] - pos[3 * jm], by = pos[3 * jp + 1] - pos[3 * jm + 1], bz = pos[3 * jp + 2] - pos[3 * jm + 2];
    const det = ax * by - ay * bx || 1;
    const al = (n.c * by - n.s * bx) / det, be = (n.s * ax - n.c * ay) / det;
    let dx = al * ax + be * bx, dy = al * ay + be * by, dz = al * az + be * bz;
    const len = Math.hypot(dx, dy, dz) || 1;
    dx *= half / len; dy *= half / len; dz *= half / len;
    const x = pos[3 * v], y = pos[3 * v + 1], z = pos[3 * v + 2] + lift;
    arr[6 * k] = x - dx; arr[6 * k + 1] = y - dy; arr[6 * k + 2] = z - dz;
    arr[6 * k + 3] = x + dx; arr[6 * k + 4] = y + dy; arr[6 * k + 5] = z + dz;
  });
  needleObj.geometry.attributes.position.needsUpdate = true;
}

function updateTorch() {
  const p = S.plate, U = p.U, e = S.exag;
  const { k, fp, cool } = timeState(S.T);
  if (S.view !== 'after' || k >= S.lines.length || fp <= 0 || fp >= 1 || cool > 0) { torch.visible = false; return; }
  const l = S.lines[k], n = l.points.length, c = l._c, near = l._near;
  const x = fp * (n - 1), s = Math.min(n - 2, Math.floor(x)), u = x - s;
  const at = (q, k3) => c[3 * q + k3] + e * U[3 * near[q] + k3];
  const px = at(s, 0) + u * (at(s + 1, 0) - at(s, 0));
  const py = at(s, 1) + u * (at(s + 1, 1) - at(s, 1));
  let pz = at(s, 2) + u * (at(s + 1, 2) - at(s, 2));
  torch.visible = true;
  if (l.side === 'bottom') { pz -= p.t; torch.rotation.set(Math.PI, 0, 0); } else torch.rotation.set(0, 0, 0);
  torch.position.set(px, py, pz);
  const shake = 0.92 + 0.16 * Math.random();
  flame.scale.set(1, shake, 1);
  glow.scale.set(1, 2 - shake, 1);
}

// 재생 위치 T 에서의 z 변위장 (평면 제거, 입력 격자)
function zAt(T, out) {
  const { k, fp } = timeState(T);
  out.set(S.cumZ[k]);
  if (k < S.lines.length && fp > 0) {
    const l = S.lines[k], r = l._resp, w = l.weight;
    const x = fp * r.n, s = Math.min(r.n - 1, Math.floor(x)), u = x - s;
    const a = r.cumZ[s], b = r.cumZ[s + 1];
    for (let i = 0; i < out.length; i++) out[i] += w * (a[i] + u * (b[i] - a[i]));
  }
}

let lastReadout = 0;
function updateReadouts(force = false) {
  const now = performance.now();
  if (!force && now - lastReadout < 90) return;
  lastReadout = now;
  const p = S.plate;
  if (!p) return;
  let rmsNow, maxNow;
  if (S.view === 'before' || !S.lines.length) { rmsNow = p.rms0; maxNow = p.max0; } else {
    zAt(S.T, p.zNow);
    const e = new Float64Array(p.zNow.length);
    for (let i = 0; i < e.length; i++) e[i] = p.D[i] - p.zNow[i];
    rmsNow = rms(e);
    maxNow = maxAbs(e);
  }
  $('statRms').textContent = fix(rmsNow, 2);
  $('statMax').textContent = fix(maxNow, 2);
  $('statFrom').textContent = fix(p.rms0, 2);
  $('statCut').textContent = p.rms0 > 1e-6 ? `${Math.round((1 - rmsNow / p.rms0) * 100)}%` : '-';
  const r = colorRange();
  $('legLo').textContent = `−${fix(r, r < 2 ? 2 : 1)}`;
  $('legHi').textContent = `+${fix(r, r < 2 ? 2 : 1)} mm`;
  updateChartMarker(rmsNow);
  const N = S.lines.length;
  const { k, fp } = timeState(S.T);
  const doneCount = S.view === 'before' ? 0 : (k >= N ? N : k);
  $('lineCount').textContent = `${Math.min(doneCount, N)} / ${N}줄`;
  const info = $('lineNow');
  if (!N) info.textContent = '';
  else if (S.view === 'before') info.textContent = '가공 전 상태입니다.';
  else if (S.view === 'target') info.textContent = '목표 곡면입니다. 판 위 선은 숨겼습니다.';
  else if (k >= N) info.textContent = `가열선 ${N}줄을 모두 지난 상태입니다.`;
  else if (fp > 0) {
    const sec = lineSeconds(S.lines[k].points, lineCond(S.lines[k]).speed);
    info.textContent = `${k + 1}번째 선: ${lineName(S.lines[k])} · ${S.lines[k].side === 'bottom' ? '아랫면' : '윗면'} · 세기 배수 ${fix(S.lines[k].weight, 2)}${sec != null ? ` · 가열 ${formatSeconds(sec)}` : ''}`;
  }
  else info.textContent = k === 0 ? '아직 가열하지 않은 상태입니다.' : `${k}번째 선까지 지난 상태입니다.`;
  document.querySelectorAll('.ln').forEach((row) => row.classList.toggle('now', +row.dataset.idx === k && k < N && S.view === 'after'));
}

// 줄 목록 (표 대신 쓰는 한 줄 카드들)
function renderList() {
  const box = $('lineList');
  box.textContent = '';
  const lines = S.lines;
  $('listCount').textContent = lines.length ? `${lines.length}줄` : '';
  $('listEmpty').hidden = lines.length > 0;
  // 순수 가열 시간 합계 (길이 ÷ 속도). 세기 배수는 넣지 않은 1회 통과 기준이다.
  let totalSec = 0, known = 0;
  for (const l of lines) { const sec = lineSeconds(l.points, lineCond(l).speed); if (sec != null) { totalSec += sec; known++; } }
  const tt = $('timeTotal');
  tt.hidden = !lines.length || !known;
  if (known) tt.textContent = `순수 가열 시간 합계 ${formatSeconds(totalSec)}${known < lines.length ? ` (속도를 아는 ${known}줄)` : ''} · 줄마다 길이 ÷ 속도이고, 이동·뒤집기 시간은 빠져 있습니다.`;
  const showCond = modeNow() !== 'direct' && !(modeNow() === 'cond' && S.plate?.fromFile);
  lines.forEach((l, idx) => {
    const row = el('div', { class: 'ln', 'data-idx': String(idx) });
    const jump = el('button', { class: 'no', title: '이 선 앞에서부터 보기', text: String(idx + 1) });
    jump.addEventListener('click', () => { S.view = 'after'; syncViewButtons(); setT(idx); });
    const close = el('button', { class: 'x', title: '이 선 지우기', 'aria-label': `${idx + 1}번째 선 지우기`, text: '×' });
    close.addEventListener('click', () => { S.lines = S.lines.filter((q) => q !== l); S.T = clamp(S.T, 0, S.lines.length); afterLinesChanged(); setMsg('선을 지웠습니다.'); });
    const name = el('span', { class: 'name', title: lineName(l), text: lineName(l) });
    const tag = el('span', { class: 'tag', text: l.side === 'bottom' ? '아랫면' : '윗면' });
    row.append(el('div', { class: 'ln-top' }, jump, name, tag, close));

    const edit = el('div', { class: 'ln-edit' });
    const wInput = el('input', { type: 'number', min: '0', step: '0.1', value: fix(l.weight, 2), 'aria-label': '세기 배수' });
    wInput.addEventListener('change', () => { l.weight = clamp(parseFloat(wInput.value) || 0, 0, 20); rebuildCumulative(); drawChart(); renderList(); markDirty(); updateReadouts(true); });
    edit.append(el('label', {}, '배수', wInput));
    const side = el('select', { 'aria-label': '가열 면' });
    side.append(el('option', { value: 'top', text: '윗면' }), el('option', { value: 'bottom', text: '아랫면' }));
    side.value = l.side;
    side.addEventListener('change', () => {
      l.side = side.value;
      guard('면을 바꿔 다시 계산', async () => { await ensureResponses([l]); afterLinesChanged(); setMsg(''); });
    });
    edit.append(el('label', {}, '면', side));
    if (showCond) {
      const makeNum = (key, label, ph) => {
        const inp = el('input', { type: 'number', step: key === 'speed' ? '0.5' : '10', placeholder: ph, 'aria-label': label });
        if (l[key] != null) inp.value = String(l[key]);
        inp.addEventListener('change', () => {
          const v = parseFloat(inp.value);
          l[key] = Number.isFinite(v) && v > 0 ? v : null;
          guard('세기를 바꿔 다시 계산', async () => { await ensureResponses([l]); afterLinesChanged(); setMsg(''); });
        });
        return el('label', {}, label, inp);
      };
      edit.append(makeNum('speed', '속도', String(num('speed'))), makeNum('peak', '최고온도', String(num('peak'))));
    }
    row.append(edit);
    const after = S.rmsSeries[idx + 1];
    const sec = lineSeconds(l.points, lineCond(l).speed);
    const parts = [`이 선까지 더한 뒤 RMS ${fix(after, 2)} mm`];
    const lc = lineCond(l), mag = S.plate ? magnitudeFor(lc.speed, lc.peak) : null;
    if (mag?.source === 'ai') parts.push(`AI 각변형 ${fix(mag.theta_mrad, 2)} mrad (${fix(mag.ms, 1)} ms)`);
    else if (mag?.source === 'table') parts.push(`보정표 각변형 ${fix(mag.theta_mrad, 2)} mrad`);
    if (sec != null) parts.push(`가열 ${formatSeconds(sec)}`);
    row.append(el('div', { class: 'ln-rms', text: parts.join(' · ') }));
    box.append(row);
  });
}

// 남은 RMS 그래프: 세로축 하나, 선 하나.
let chartGeo = null;
function drawChart() {
  drawCurlChart();
  const svg = $('chart');
  const series = S.rmsSeries;
  svg.textContent = '';
  chartGeo = null;
  if (!S.plate || series.length < 2) return;
  const W = 280, H = 132, L = 36, R = 10, T = 8, B = 22;
  const N = series.length - 1;
  const yMax = Math.max(...series, 0.1) * 1.05;
  const step = roundStep(yMax / 3);
  const X = (k) => L + (W - L - R) * k / N;
  const Y = (v) => T + (H - T - B) * (1 - v / yMax);
  const ns = 'http://www.w3.org/2000/svg';
  const add = (tag, attrs, text) => { const e = document.createElementNS(ns, tag); for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, v); if (text != null) e.textContent = text; svg.append(e); return e; };
  for (let v = 0; v <= yMax + 1e-9; v += step) {
    add('line', { x1: L, x2: W - R, y1: Y(v), y2: Y(v), stroke: 'var(--line)', 'stroke-width': 1 });
    add('text', { x: L - 5, y: Y(v) + 3.5, 'text-anchor': 'end' }, fix(v, step < 0.1 ? 2 : step < 1 ? 1 : 0));
  }
  const xTicks = [0, Math.round(N / 2), N].filter((v, i, a) => a.indexOf(v) === i);
  xTicks.forEach((k) => add('text', { x: X(k), y: H - 6, 'text-anchor': 'middle' }, `${k}줄`));
  const d = series.map((v, k) => `${k ? 'L' : 'M'}${X(k).toFixed(1)},${Y(v).toFixed(1)}`).join('');
  add('path', { d, fill: 'none', stroke: 'var(--acc)', 'stroke-width': 2, 'stroke-linejoin': 'round', 'stroke-linecap': 'round' });
  chartGeo = { X, Y, N, svg };
  S.chartMarker = add('circle', { r: 4.5, fill: 'var(--acc)', stroke: 'var(--card)', 'stroke-width': 2, cx: X(0), cy: Y(series[0]) });
}
function updateChartMarker(v) {
  if (!chartGeo || !S.chartMarker) return;
  const { X, Y, N } = chartGeo;
  const x = S.view === 'before' ? 0 : Math.min(S.T, N);
  S.chartMarker.setAttribute('cx', X(x));
  S.chartMarker.setAttribute('cy', Y(v));
  if (curlGeo && S.curlMarker) { const px = curlGeo.X(Math.min(x, curlGeo.N)); S.curlMarker.setAttribute('x1', px); S.curlMarker.setAttribute('x2', px); }
}

// 곡 펴짐 그래프: 줄을 더할 때 방향별 굽힘 변형 제곱합이 어떻게 변하는지 (상대값). 값이 줄어드는 곳이 앞서 만든 곡이 펴지는 구간이다.
let curlGeo = null;
function drawCurlChart() {
  const svg = $('curlChart'), wrap = $('curlWrap'), c = S.curl;
  svg.textContent = '';
  curlGeo = null;
  S.curlMarker = null;
  if (!c || c.series.length < 2) { wrap.hidden = true; return; }
  wrap.hidden = false;
  const W = 280, H = 112, L = 36, R = 10, T = 8, B = 20, N = c.series.length - 1;
  const maxV = Math.max(...c.series.flatMap((d) => d.sum), 1e-30);
  const X = (k) => L + (W - L - R) * k / N, Y = (v) => T + (H - T - B) * (1 - v / maxV);
  const ns = 'http://www.w3.org/2000/svg';
  const add = (tag, attrs, text) => { const e = document.createElementNS(ns, tag); for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, v); if (text != null) e.textContent = text; svg.append(e); return e; };
  for (const g of [0, 0.5, 1]) {
    add('line', { x1: L, x2: W - R, y1: Y(g * maxV), y2: Y(g * maxV), stroke: 'var(--line)', 'stroke-width': 1 });
    add('text', { x: L - 5, y: Y(g * maxV) + 3.5, 'text-anchor': 'end' }, String(g));
  }
  [0, Math.round(N / 2), N].filter((v, i, a) => a.indexOf(v) === i).forEach((k) => add('text', { x: X(k), y: H - 6, 'text-anchor': 'middle' }, `${k}줄`));
  const colors = ['var(--acc)', 'var(--hot)'];
  [0, 1].forEach((d) => add('path', { d: c.series.map((pt, k) => `${k ? 'L' : 'M'}${X(k).toFixed(1)},${Y(pt.sum[d]).toFixed(1)}`).join(''), fill: 'none', stroke: colors[d], 'stroke-width': 2, 'stroke-linejoin': 'round', 'stroke-linecap': 'round', ...(d ? { 'stroke-dasharray': '5 3' } : {}) }));
  S.curlMarker = add('line', { x1: X(0), x2: X(0), y1: T, y2: H - B, stroke: 'var(--sub)', 'stroke-width': 1, 'stroke-dasharray': '3 3' });
  curlGeo = { X, N };
  const events = (c.events || []).filter((e) => e.drop >= 0.01);
  const dirName = ['길이(x)', '폭(y)'];
  let note = events.length
    ? `${events.slice(0, 3).map((e) => `${e.n}번째 줄에서 ${dirName[e.dir]}방향 곡 약 ${Math.round(e.drop * 100)}% 펴짐`).join(' · ')}${events.length > 3 ? ` 외 ${events.length - 3}건` : ''}.`
    : '뚜렷하게 곡이 펴지는 줄은 없습니다.';
  $('curlNote').textContent = `${note} 절대값이 아니라 줄이 늘 때의 추세만 봅니다 (해석값).`;
}
function chartPointer(ev) {
  if (!chartGeo) return null;
  const r = $('chart').getBoundingClientRect();
  const x = (ev.clientX - r.left) / r.width * 280;
  const k = clamp(Math.round((x - 36) / (280 - 36 - 10) * chartGeo.N), 0, chartGeo.N);
  return { k, r, ev };
}
$('chart').addEventListener('mousemove', (ev) => {
  const c = chartPointer(ev);
  if (!c) return;
  const tip = $('chartTip');
  tip.hidden = false;
  tip.textContent = `${c.k}줄 뒤 RMS ${fix(S.rmsSeries[c.k])} mm · 최대 ${fix(S.maxSeries[c.k])}`;
  const wrap = tip.parentElement.getBoundingClientRect();
  tip.style.left = `${clamp(ev.clientX - wrap.left - 60, 0, Math.max(0, wrap.width - tip.offsetWidth))}px`;
  tip.style.top = `${ev.clientY - wrap.top - 34}px`;
});
$('chart').addEventListener('mouseleave', () => { $('chartTip').hidden = true; });
$('chart').addEventListener('click', (ev) => { const c = chartPointer(ev); if (c) { S.view = 'after'; syncViewButtons(); setT(c.k); } });

function updateScaleNote() {
  const note = $('scaleNote');
  note.textContent = `해석 모델 계산값 · 변형 배율 ×${S.exag} (모양만 확대) · 바닥 눈금 한 칸 ${S.groundStep ?? '-'} mm`;
}

// ── 카메라·크기 ────────────────────────────────────────────
function fitCamera(kind) {
  const p = S.plate;
  if (!p) return;
  const R = p.diag * 0.56;
  const tan = Math.tan(THREE.MathUtils.degToRad(camera.fov) / 2);
  const dist = Math.max(R / tan, R / (tan * camera.aspect)) * 1.04;
  const [az, el2] = kind === 'top' ? [-90, 87] : kind === 'side' ? [-90, 4] : [-60, 28];
  const a = THREE.MathUtils.degToRad(az), e = THREE.MathUtils.degToRad(el2);
  camera.position.set(dist * Math.cos(e) * Math.cos(a), dist * Math.cos(e) * Math.sin(a), dist * Math.sin(e));
  camera.near = p.diag / 60;
  camera.far = p.diag * 40;
  camera.updateProjectionMatrix();
  controls.target.set(0, 0, 0);
  controls.update();
  needDraw = true;
}
function resize() {
  const r = host.getBoundingClientRect();
  if (r.width < 2 || r.height < 2) return;
  renderer.setSize(r.width, r.height);
  camera.aspect = r.width / r.height;
  camera.updateProjectionMatrix();
  needDraw = true;
}
new ResizeObserver(resize).observe(host);

// ── 재생 ───────────────────────────────────────────────────
let dirty = true, needDraw = true, lastNow = performance.now();
function markDirty() { dirty = true; }

function setT(t) {
  S.T = clamp(t, 0, S.lines.length);
  syncTimeline();
  markDirty();
  updateReadouts(true);
}
function syncTimeline() {
  const tl = $('timeline');
  tl.max = Math.max(1, S.lines.length);
  tl.value = S.T;
  $('play').textContent = S.playing ? '멈춤' : '재생';
}
function syncButtons() {
  const n = S.lines.length;
  for (const id of ['play', 'first', 'prev', 'next', 'last', 'timeline', 'clear']) $(id).disabled = S.busy || !n;
  $('order').disabled = S.busy || n < 2;
  const synthOnly = !!S.plate?.fromFile;
  for (const id of ['L', 'W', 't', 'dx', 'srcShape', 'srcDepth', 'tgtShape', 'tgtDepth']) $(id).disabled = S.busy || synthOnly;
  $('backRow').hidden = !synthOnly;
}
function syncViewButtons() {
  document.querySelectorAll('[data-view]').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.view === S.view)));
  markDirty();
  updateReadouts(true);
}
function togglePlay() {
  if (!S.lines.length) return;
  if (!S.playing && S.T >= S.lines.length) S.T = 0;
  S.playing = !S.playing;
  if (S.playing && S.view !== 'after') { S.view = 'after'; syncViewButtons(); }
  syncTimeline();
  lastNow = performance.now();
}

function frame(now) {
  requestAnimationFrame(frame);
  const dt = Math.min(0.1, (now - lastNow) / 1000);
  lastNow = now;
  const p = S.plate;
  if (S.playing && S.lines.length) {
    S.T += dt * S.rate / LINE_SECONDS;
    if (S.T >= S.lines.length) { S.T = S.lines.length; S.playing = false; }
    syncTimeline();
    dirty = true;
  }
  if (controls.update()) needDraw = true;
  if (dirty && p && p.surface) {
    dirty = false;
    computeU();
    writeSurface();
    updateNeedles();
    writeColors();
    writeHeat();
    if (ribbon) {
      updateRibbons();
      const { k } = timeState(S.T);
      if (k !== S.lastPainted) paintRibbons();
    }
    updateTorch();
    updateReadouts(!S.playing);
    needDraw = true;
  }
  if (needDraw) {
    renderer.render(scene, camera);
    needDraw = false;
  }
}

// ── 흐름 묶기 ──────────────────────────────────────────────
function setBusy(on, label) {
  S.busy = on;
  document.querySelectorAll('main input, main select, main button').forEach((el2) => { if (!el2.dataset.free) el2.disabled = on; });
  if (on) { setMsg(label); setProgress(0); } else { setProgress(0); syncButtons(); }
}
let pendingRebuild = false;
async function guard(label, job) {
  if (S.busy) return false;
  setBusy(true, label);
  try {
    await job();
    setBusy(false);
    return true;
  } catch (err) {
    setBusy(false);
    setMsg(`오류: ${err.message}`, true);
    return false;
  } finally {
    if (pendingRebuild) { pendingRebuild = false; rebuild(); }
  }
}

async function usePlate(p, startLines) {
  S.playing = false;
  S.T = 0;
  S.lines = [];
  S.plate = p;
  $('synthTag').hidden = p.fromFile && !p.synthetic;       // 기본 판과 가상 표시가 있는 파일만 가상 데이터로 표시한다
  if (p.fromFile) { S.savedT ??= $('t').value; $('t').value = String(p.t); }
  preparePlate(p);
  buildPlateMeshes(p);
  refreshShapeInfo();
  rebuildCumulative();
  $('viewHint').hidden = false;
  S.chartMarker = null;
  await call({ type: 'build', points: p.points, thickness: p.t, refine: p.refine }, 'built', (v) => setProgress(v)).then((out) => {
    p.dof = out.dof;
    const grid = `격자 ${p.nr} × ${p.nc}`;
    const extra = p.fromFile ? (p.refine > 1 ? ` · ${p.refine}배로 잘게 나눠 계산` : '') : ` (간격 ${fix(p.dx, 0)} mm)`;
    let info = `${grid}${extra} · 자유도 ${out.dof.toLocaleString('ko-KR')} · 계산 준비 ${(out.ms / 1000).toFixed(1)}초`;
    if (p.coarsened) info += ' · 계산량 때문에 격자를 키웠습니다';
    if (p.fromFile) info = `불러온 판: ${p.name} · ${info}`;
    $('plateInfo').textContent = info;
  });
  fitCamera('iso');
  if (startLines?.length) {
    S.lines = startLines.map((c) => {
      const pts = normalizedPoints(c.points);
      return makeLine({ family: 'file', name: c.name, deg: 0, points: pts, speed: c.speed, peak: c.peak, origin: 'file' });
    });
    if (magnitudeFor(num('speed'), num('peak'))) await ensureResponses(S.lines);
    else { S.lines = []; setMsg('파일의 가열선을 쓰려면 먼저 세기를 정해 주세요.', true); }
  }
  afterLinesChanged();
  const skipped = p.skippedCurves ? ` 선상가열이 아닌 ${p.skippedCurves}줄은 뺐습니다.` : '';
  setMsg(S.lines.length ? `파일의 가열선 ${S.lines.length}줄을 불러왔습니다.${skipped}` : (skipped ? `이 파일에는 쓸 수 있는 선상가열 선이 없습니다.${skipped}` : ''));
}

function rebuild() {
  if (S.busy) { pendingRebuild = true; return; }
  guard('판 준비 중', async () => { await usePlate(syntheticPlate()); }).then(async () => { await loadAiModel(); updateCompView(); renderList(); $('recommend').click(); });
}

function targetChanged() {
  const p = S.plate;
  if (!p || p.fromFile) return;
  const L = p.points[p.nr - 1][0][0], W = p.points[0][p.nc - 1][1];
  p.target = shapePoints(L, W, p.nr, p.nc, $('tgtShape').value, clamp(num('tgtDepth'), 0, 1000));
  setTargetFields(p);
  refreshShapeInfo();
  rebuildCumulative();
  drawChart();
  renderList();
  markDirty();
  updateReadouts(true);
  setMsg(S.lines.length ? '목표를 바꿨습니다. 지금 선 그대로의 남은 편차를 표시합니다. 새 목표에 맞추려면 자동 추천을 다시 누르세요.' : '');
}

// 2번째 클릭까지 받아 가열선을 하나 만든다.
const manual = { on: false, first: null };
const raycaster = new THREE.Raycaster();
const ndc = new THREE.Vector2();
let pressed = null;
const tri = new THREE.Triangle();
const bary = new THREE.Vector3();
const markerGeo = new THREE.SphereGeometry(1, 16, 12);
const marker = new THREE.Mesh(markerGeo, new THREE.MeshBasicMaterial({ color: 0xff7a1a, toneMapped: false }));
marker.visible = false;
scene.add(marker);
const previewGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
const preview = new THREE.Line(previewGeo, new THREE.LineBasicMaterial({ color: 0xff7a1a, toneMapped: false }));
preview.visible = false;
preview.frustumCulled = false;
scene.add(preview);

function pick(ev) {
  const p = S.plate;
  if (!p || !p.surface) return null;
  const r = renderer.domElement.getBoundingClientRect();
  ndc.set(((ev.clientX - r.left) / r.width) * 2 - 1, -((ev.clientY - r.top) / r.height) * 2 + 1);
  raycaster.setFromCamera(ndc, camera);
  p.surface.geometry.computeBoundingSphere();
  const hits = raycaster.intersectObject(p.surface, false);
  if (!hits.length) return null;
  const h = hits[0];
  tri.setFromAttributeAndIndices(p.posAttr, h.face.a, h.face.b, h.face.c);
  tri.getBarycoord(h.point, bary);
  const ij = (v) => [Math.floor(v / p.rnc) / p.m, (v % p.rnc) / p.m];
  const A = ij(h.face.a), B = ij(h.face.b), C = ij(h.face.c);
  return { u: bary.x * A[0] + bary.y * B[0] + bary.z * C[0], v: bary.x * A[1] + bary.y * B[1] + bary.z * C[1], point: h.point.clone() };
}
function setManual(on) {
  manual.on = on;
  manual.first = null;
  marker.visible = false;
  preview.visible = false;
  $('manual').classList.toggle('on', on);
  $('manual').setAttribute('aria-pressed', String(on));
  $('manualHint').hidden = !on;
  host.classList.toggle('adding', on);
  needDraw = true;
}
function addManual(a, b) {
  const p = S.plate;
  const len = Math.hypot((b.u - a.u) * p.dsRow, (b.v - a.v) * p.dsCol);
  if (len < 80) { setMsg('선이 너무 짧습니다. 조금 더 떨어진 곳을 눌러 주세요.', true); return; }
  const n = clamp(Math.ceil(len / 100) + 1, 13, 61);
  const points = [];
  for (let s = 0; s < n; s++) { const t = s / (n - 1); points.push(surfacePoint(p, a.u + (b.u - a.u) * t, a.v + (b.v - a.v) * t)); }
  const deg = THREE.MathUtils.radToDeg(Math.atan2((b.u - a.u) * p.dsRow, (b.v - a.v) * p.dsCol));
  const l = makeLine({ family: 'manual', deg: ((deg + 90) % 180 + 180) % 180 - 90, points, side: $('manualSide').value, origin: 'manual' });
  guard('가열선 계산 중', async () => {
    await ensureResponses([l]);
    S.lines.push(l);
    afterLinesChanged();
    S.view = 'after';
    syncViewButtons();
    setT(S.lines.length);
    setMsg(`${lineName(l)} 선을 넣었습니다. 슬라이더를 움직이거나 [재생]으로 볼 수 있습니다.`);
  });
}
const canvas = renderer.domElement;
canvas.addEventListener('pointerdown', (ev) => { pressed = { x: ev.clientX, y: ev.clientY }; });
canvas.addEventListener('pointerup', (ev) => {
  if (!manual.on || ev.button !== 0 || !pressed || S.busy) return;
  if (Math.hypot(ev.clientX - pressed.x, ev.clientY - pressed.y) > 5) return;
  const hit = pick(ev);
  if (!hit) return;
  if (!manual.first) {
    manual.first = hit;
    marker.position.copy(hit.point);
    marker.scale.setScalar(clamp(S.plate.diag * 0.006, 8, 30));
    marker.visible = true;
    setMsg('끝점을 눌러 주세요.');
  } else {
    const a = manual.first;
    manual.first = null;
    marker.visible = false;
    preview.visible = false;
    addManual(a, hit);
  }
  needDraw = true;
});
canvas.addEventListener('pointermove', (ev) => {
  if (!manual.on || !manual.first) return;
  const hit = pick(ev);
  if (!hit) return;
  const arr = previewGeo.attributes.position.array;
  arr[0] = marker.position.x; arr[1] = marker.position.y; arr[2] = marker.position.z + 6;
  arr[3] = hit.point.x; arr[4] = hit.point.y; arr[5] = hit.point.z + 6;
  previewGeo.attributes.position.needsUpdate = true;
  preview.visible = true;
  needDraw = true;
});

// ── 입력 연결 ──────────────────────────────────────────────
for (const [id, def] of [['srcShape', 'flat'], ['tgtShape', 'cyl_up']]) {
  const sel = $(id);
  for (const [k, v] of Object.entries(SHAPES)) sel.append(el('option', { value: k, text: v.name }));
  sel.value = def;
}
['L', 'W', 't', 'dx', 'srcShape', 'srcDepth'].forEach((id) => $(id).addEventListener('change', rebuild));
['tgtShape', 'tgtDepth'].forEach((id) => $(id).addEventListener('change', targetChanged));
$('backSynthetic').addEventListener('click', () => {
  if (S.savedT != null) { $('t').value = S.savedT; S.savedT = null; }
  rebuild();
});

$('plateFile').addEventListener('change', async (ev) => {
  const file = ev.target.files[0];
  ev.target.value = '';
  if (!file) return;
  await guard('판 파일 읽는 중', async () => {
    const raw = await file.arrayBuffer();
    let text = new TextDecoder('utf-8').decode(raw);
    if (text.charCodeAt(0) === 0xFEFF) text = text.slice(1);
    let doc;
    try { doc = JSON.parse(text); } catch { throw new Error('파일을 읽지 못했습니다. JSON 형식인지 확인해 주세요.'); }
    const plate = plateFromDoc(doc, file.name);
    const curves = plate.curves;
    delete plate.curves;
    if (modeNow() === 'cond' || (modeNow() === 'table' && !S.table)) { $('compMode').value = 'direct'; updateCompView(); }
    await usePlate(plate, curves);
  });
});

$('tableFile').addEventListener('change', async (ev) => {
  const file = ev.target.files[0];
  ev.target.value = '';
  if (!file) return;
  try {
    let parsed;
    try { parsed = JSON.parse((await file.text()).replace(/^﻿/, '')); } catch { throw new Error('JSON 형식이 아닙니다'); }
    if (!Array.isArray(parsed.entries) || !parsed.entries.length) throw new Error('entries 항목이 없습니다');
    for (const e of parsed.entries) {
      if (![e.thickness_mm, e.speed, e.peak_c, e.theta_mrad, e.dtrans_mm, e.dlong_mm].every(Number.isFinite)) throw new Error('수치가 빠진 항목이 있습니다');
    }
    S.table = parsed;
    $('tableInfo').textContent = `보정표 사용 중: ${parsed.entries.length}개 조건 (이 화면의 메모리에서만 씁니다)`;
    updateCompView();
    scheduleRecompute();
  } catch (err) {
    $('tableInfo').textContent = `보정표를 읽지 못했습니다: ${err.message}`;
  }
});

function scheduleRecompute() {
  updateCompView();
  if (modeNow() === 'table' && !S.table) return;
  if (modeNow() === 'ai' && !AI.bridge) return;
  if (S.lines.length) guard('세기를 바꿔 다시 계산', async () => { await ensureResponses(S.lines); afterLinesChanged(); setMsg(''); });
}
$('compMode').addEventListener('change', async () => {
  if (modeNow() === 'ai' && !AI.bridge) { updateCompView(); await loadAiModel(); }
  updateCompView();
  renderList();
  scheduleRecompute();
});
['dirTheta', 'dirTrans', 'dirLong', 'dirThetaL', 'speed', 'peak', 'aiR0', 'aiGap', 'aiH', 'aiFy', 'aiWater', 'endLen', 'endRatio', 'endKeep'].forEach((id) => $(id).addEventListener('change', scheduleRecompute));
$('compRun').addEventListener('click', () => guard('세기 계산 중', async () => {
  await computeCond(num('speed'), num('peak'));
  updateCompView();
  await ensureResponses(S.lines);
  afterLinesChanged();
  setMsg('세기를 계산했습니다. 해석값이며 실측이 아닙니다.');
}));

$('recommend').addEventListener('click', () => guard('가열선 찾는 중', autoRecommend));
$('manual').addEventListener('click', () => setManual(!manual.on));
$('clear').addEventListener('click', () => { S.lines = []; S.T = 0; S.playing = false; afterLinesChanged(); setMsg('가열선을 모두 지웠습니다.'); });

$('play').addEventListener('click', togglePlay);
// 위치를 옮기면 그 위치의 판이 보이도록 '가공 후' 보기로 돌아온다 (가공 전·목표 보기에서는 위치가 바뀌어도 화면이 그대로라 멈춘 것처럼 보인다).
function showAfter() { if (S.view !== 'after') { S.view = 'after'; syncViewButtons(); } }
$('first').addEventListener('click', () => { S.playing = false; showAfter(); setT(0); });
$('prev').addEventListener('click', () => { S.playing = false; showAfter(); setT(Math.ceil(S.T - 1e-6) - 1); });
$('next').addEventListener('click', () => { S.playing = false; showAfter(); setT(Math.floor(S.T + 1e-6) + 1); });
$('last').addEventListener('click', () => { S.playing = false; showAfter(); setT(S.lines.length); });
$('timeline').addEventListener('input', (ev) => { S.playing = false; showAfter(); setT(parseFloat(ev.target.value)); });
$('rate').addEventListener('change', (ev) => { S.rate = parseFloat(ev.target.value) || 1; });
$('order').addEventListener('change', () => { S.T = 0; afterLinesChanged(); });

document.querySelectorAll('[data-view]').forEach((b) => b.addEventListener('click', () => { S.view = b.dataset.view; syncViewButtons(); }));
document.querySelectorAll('[data-cam]').forEach((b) => b.addEventListener('click', () => fitCamera(b.dataset.cam)));
$('exag').addEventListener('input', (ev) => {
  S.exag = parseInt(ev.target.value, 10);
  $('exagVal').textContent = `×${S.exag}`;
  updateScaleNote();
  markDirty();
});
['colorRange', 'devColor', 'ghost'].forEach((id) => $(id).addEventListener('change', () => { markDirty(); updateReadouts(true); }));
$('colorKind').addEventListener('change', (ev) => { S.colorKind = ev.target.value; curvState = { p: null, key: '', classes: null, at: 0 }; markDirty(); updateReadouts(true); });
$('needles').addEventListener('change', (ev) => { S.showNeedles = ev.target.checked; markDirty(); });
const REC_HINT = {
  fixed: '폭방향·길이방향 직선 후보에서 고릅니다 (기존 방식).',
  traced: '만들 변형이 큰 주방향에 수직으로 곡선을 추적합니다. 전개는 얕은 곡면 가정으로 대신 계산했고, 결과는 시작점에 따라 달라집니다.',
  mixed: '직선과 추적선을 함께 후보로 넣고, 혼합·직선만·추적선만 중 남은 편차가 가장 작은 결과를 씁니다.',
};
function updateRecHint() { $('recHint').textContent = REC_HINT[$('recMode').value] ?? ''; }
$('recMode').addEventListener('change', updateRecHint);
updateRecHint();

window.addEventListener('keydown', (ev) => {
  if (ev.key === 'Escape' && manual.on) setManual(false);
  if (ev.key === ' ' && !/INPUT|SELECT|TEXTAREA|BUTTON/.test(ev.target.tagName)) { ev.preventDefault(); togglePlay(); }
});

// 화면 밝기: 자동 → 밝게 → 어둡게. 판 데이터와 무관한 값만 저장한다.
const THEMES = [['', '테마: 자동'], ['light', '테마: 밝게'], ['dark', '테마: 어둡게']];
let themeIndex = 0;
try { const saved = localStorage.getItem('forming.theme') ?? ''; themeIndex = Math.max(0, THEMES.findIndex(([v]) => v === saved)); } catch { themeIndex = 0; }
function setThemeIndex(i) {
  themeIndex = i;
  const [v, label] = THEMES[i];
  if (v) document.documentElement.dataset.theme = v; else delete document.documentElement.dataset.theme;
  $('themeBtn').textContent = label;
  try { if (v) localStorage.setItem('forming.theme', v); else localStorage.removeItem('forming.theme'); } catch { /* 저장 못 해도 화면은 쓴다 */ }
  applyTheme();
}
$('themeBtn').addEventListener('click', () => setThemeIndex((themeIndex + 1) % THEMES.length));
matchMedia('(prefers-color-scheme: dark)').addEventListener('change', applyTheme);

// 시험용으로 현재 상태를 읽을 수 있게 둔다 (값을 읽기만 한다).
window.formingPage = {
  snapshot: () => ({
    ready: !!S.plate?.dof,
    busy: S.busy,
    lines: S.lines.length,
    T: S.T,
    view: S.view,
    msg: $('msg').textContent,
    statRms: $('statRms').textContent,
    statMax: $('statMax').textContent,
    statCut: $('statCut').textContent,
    rmsSeries: S.rmsSeries.slice(),
    maxSeries: S.maxSeries.slice(),
    plateInfo: $('plateInfo').textContent,
    gl: renderer.getContext().getParameter(renderer.getContext().VERSION),
    compMode: modeNow(), recMode: $('recMode').value,
    compView: $('compView').textContent,
    aiNote: $('aiNote').textContent, aiStatus: $('aiStatus').textContent,
    classLabel: $('classBody').textContent,
    classify: S.plate?.classify ? { y: Array.from(S.plate.classify.y), z: S.plate.classify.z, tasks: S.plate.classify.tasks, shape: S.plate.classify.shapeName, noise: S.plate.classify.noise } : null,
    curl: S.curl ? { series: S.curl.series, events: S.curl.events, note: $('curlNote').textContent } : null,
    timeTotal: $('timeTotal').hidden ? '' : $('timeTotal').textContent,
    needles: S.plate?.needles?.length ?? 0,
    bandVerts: bandMesh ? bandMesh.geometry.attributes.position.count : 0,
    lineFamilies: S.lines.map((l) => l.family),
    listTexts: [...document.querySelectorAll('.ln-rms')].map((n) => n.textContent),
    colorKind: S.colorKind,
    curvLegend: !$('legendCurv').hidden,
    worker: 'engine/worker_ext.js',
  }),
  screenOf: (u, v) => {
    const p = S.plate, i = Math.round(u * (p.rnr - 1)), j = Math.round(v * (p.rnc - 1)), o = (i * p.rnc + j) * 3, a = p.posAttr.array;
    const q = new THREE.Vector3(a[o], a[o + 1], a[o + 2]).project(camera);
    const r = renderer.domElement.getBoundingClientRect();
    return { x: r.left + (q.x + 1) / 2 * r.width, y: r.top + (1 - q.y) / 2 * r.height };
  },
  exportPlan: () => (S.plate.fromFile ? null : {
    t: S.plate.t,
    points: S.plate.points,
    target: S.plate.target,
    lines: S.lines.map((l) => ({ points: l.points, weight: l.weight, side: l.side, comp: compOf(l), family: l.family })),
    rmsEnd: S.rmsSeries[S.lines.length],
  }),
};

// ── 시작 ───────────────────────────────────────────────────
setThemeIndex(themeIndex);
resize();
requestAnimationFrame(frame);
guard('판 준비 중', async () => { await usePlate(syntheticPlate()); }).then(async () => { await loadAiModel(); updateCompView(); renderList(); $('recommend').click(); });

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { CSS2DRenderer, CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';

// ============================================================================
// 상태
// ============================================================================
const DATASETS = [];           // 로드된 모든 데이터셋 (업로드 순서)
let primaryIdx = -1;           // 상세 뷰에 쓰이는 데이터셋 인덱스
const compareIndices = new Set(); // 겹쳐 볼 데이터셋들 (Source 표면만)

const COMPARE_PALETTE = [
  0x00ff88, 0xff00aa, 0xffaa00, 0x00aaff, 0xaa00ff,
  0xffff00, 0x00ffff, 0xff5555, 0x55ff55, 0x5555ff,
];

// 레이어 정의 (primary 데이터셋 기준)
const LAYER_DEFS = [
  { id: 'source',      name: 'Source 표면 (가열 전 스캔)',  color: 0x4ea8ff, opacity: 0.85 },
  { id: 'target',      name: 'Target 표면 (설계 목표)',      color: 0xffb545, opacity: 0.35 },
  { id: 'deviation',   name: 'Deviation Scatter (Src↔Tgt)',  color: 0x00ff88, opacity: 1.0  },
  { id: 'designEdges', name: 'Design Boundary Edges',        color: 0xffb545, opacity: 1.0  },
  { id: 'measureEdges',name: 'Measured Boundary Edges',      color: 0x4ea8ff, opacity: 1.0  },
  { id: 'curves',      name: 'Bending Curves (열 프로파일)', color: 0xff5577, opacity: 1.0  },
  { id: 'directions',  name: '방향 라벨 (AFT/FORE/TOP/BOTTOM)', color: 0x4ea8ff, opacity: 1.0 },
  { id: 'seams',       name: 'Seam 라벨 (S4xx/S9xx)',        color: 0xffb545, opacity: 1.0  },
  { id: 'compare',     name: 'Compare Source 표면들',        color: 0x00ff88, opacity: 0.55 },
  { id: 'grid',        name: '월드 그리드',                   color: 0x666666, opacity: 0.4  },
  { id: 'axes',        name: '좌표축',                        color: 0xffffff, opacity: 1.0  },
];
const LAYER_STATE = {};
LAYER_DEFS.forEach(l => { LAYER_STATE[l.id] = { visible: true, opacity: l.opacity }; });
LAYER_STATE.deviation.visible = false;

// ============================================================================
// Three.js 기본 구성
// ============================================================================
const canvasHost = document.getElementById('canvas-host');
const labelHost = document.getElementById('label-host');
const tooltip = document.getElementById('tooltip');
const emptyState = document.getElementById('emptyState');

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0d12);

const camera = new THREE.PerspectiveCamera(45, 1, 1, 50000);
camera.position.set(4000, 3500, 4500);
camera.up.set(0, 0, 1);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
canvasHost.appendChild(renderer.domElement);

const labelRenderer = new CSS2DRenderer();
labelRenderer.domElement.style.position = 'absolute';
labelRenderer.domElement.style.inset = '0';
labelRenderer.domElement.style.pointerEvents = 'none';
labelHost.appendChild(labelRenderer.domElement);

// CAD 스타일 부드러운 조작
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;          // 작을수록 더 부드러움
controls.rotateSpeed = 0.7;
controls.panSpeed = 0.9;
controls.zoomSpeed = 0.9;
controls.screenSpacePanning = true;     // 화면 평면 기준 pan (CAD 관습)
controls.mouseButtons = {
  LEFT: THREE.MOUSE.ROTATE,
  MIDDLE: THREE.MOUSE.PAN,              // 휠클릭 = 절대 위치 이동(Pan)
  RIGHT: THREE.MOUSE.PAN,
};
controls.touches = {
  ONE: THREE.TOUCH.ROTATE,
  TWO: THREE.TOUCH.DOLLY_PAN,
};

// 카메라 tween (프리셋/사용자 각도 전환을 부드럽게)
const cameraTween = { active: false };
function easeInOutCubic(t) {
  return t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;
}
function animateCameraTo(targetPos, targetLookAt, upVec, duration = 650) {
  cameraTween.active = true;
  cameraTween.startPos = camera.position.clone();
  cameraTween.endPos = targetPos.clone();
  cameraTween.startTarget = controls.target.clone();
  cameraTween.endTarget = targetLookAt.clone();
  cameraTween.startUp = camera.up.clone();
  cameraTween.endUp = (upVec || camera.up).clone();
  cameraTween.startTime = performance.now();
  cameraTween.duration = duration;
}
function updateCameraTween() {
  if (!cameraTween.active) return;
  const k = Math.min(1, (performance.now() - cameraTween.startTime) / cameraTween.duration);
  const t = easeInOutCubic(k);
  camera.position.lerpVectors(cameraTween.startPos, cameraTween.endPos, t);
  controls.target.lerpVectors(cameraTween.startTarget, cameraTween.endTarget, t);
  camera.up.lerpVectors(cameraTween.startUp, cameraTween.endUp, t).normalize();
  if (k >= 1) cameraTween.active = false;
}

scene.add(new THREE.AmbientLight(0xffffff, 0.55));
const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
dirLight.position.set(5000, -5000, 8000);
scene.add(dirLight);
const dirLight2 = new THREE.DirectionalLight(0xffffff, 0.35);
dirLight2.position.set(-5000, 5000, 5000);
scene.add(dirLight2);

const gridHelper = new THREE.GridHelper(10000, 40, 0x666666, 0x333333);
gridHelper.rotation.x = Math.PI / 2;
gridHelper.material.transparent = true;
scene.add(gridHelper);

const axesHelper = new THREE.AxesHelper(2000);
scene.add(axesHelper);

function resize() {
  const rect = canvasHost.getBoundingClientRect();
  renderer.setSize(rect.width, rect.height);
  labelRenderer.setSize(rect.width, rect.height);
  camera.aspect = rect.width / rect.height;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);

const root = new THREE.Group();
scene.add(root);

const layerObjects = {};
LAYER_DEFS.forEach(l => { layerObjects[l.id] = []; });
const curveObjects = new Map();

// Top-View Deviation 상태
const TOP_DEV = {
  source: false,
  target: false,
  post: false,
  threshold: 5,     // mm
  zOffset: 600,     // mm above bbox.max.z
  size: 14,         // px
  group: new THREE.Group(),
};
TOP_DEV.group.name = 'TopDevGroup';
scene.add(TOP_DEV.group);

// ============================================================================
// JSON 파싱 (build_data.py 와 동등)
// ============================================================================
function parsePoint(s) {
  if (s == null) return null;
  if (Array.isArray(s)) return s.slice(0, 3).map(Number);
  const parts = String(s).split(',').map(x => parseFloat(x.trim()));
  return [parts[0], parts[1], parts[2]];
}
function parsePoints1D(seq) { return (seq || []).map(parsePoint); }
function parsePoints2D(grid) { return (grid || []).map(parsePoints1D); }
function parseNamePoints(arr) {
  return (arr || [])
    .filter(it => it && it.textPoint)
    .map(it => ({ point: parsePoint(it.textPoint), text: it.text || '' }));
}

function normalizeDataset(doc, fileName) {
  const src = doc.SourceGrid || {};
  const tgt = doc.TargetGrid || {};
  const curves = (doc.BendingCurves || []).map(c => ({
    curveId: c.CurveId,
    heatingMethod: c.HeatingMethod,
    points: parsePoints1D(c.Points),
    inputPoints: parsePoints1D(c.inputUserPointList),
    strength: c.Strength,
    width: c.Width,
    heatIntensity: c.HeatIntensity,
    split: c.Split,
    weavingRepeat: c.WeavingRepeat,
    thk: c.THK,
    speed: c.speed,
    coolingTime: c.coolingTime,
    paramShape: c.ParameterShape,
    paramTemp: c.ParameterTemp,
    paramStep: c.ParameterStep,
    paramRepeat: c.ParameterRepeat,
    paramCoolingSpeed: c.ParameterCoolingSpeed,
    isAutoMaked: c.isAutoMaked,
  }));
  return {
    fileName,
    legacyId: doc.LegacyId,
    partName: doc.sPartName,
    plateName: doc.sPlateName,
    curveType: doc.CurveType,
    thickness: doc.dThickNess,
    deviceCode: doc.DeviceCode,
    createdAt: doc.CreateDateTime,
    directionKey: doc.DirectionKey,
    completedRatio1: doc.complatedRatio1,
    completedRatio2: doc.complatedRatio2,
    residualNorm: doc.ResidualNorm,
    totalCurveLength: doc.TotalCurveLength,
    curveCount: doc.CurveCount,
    designMirror: doc.designMirror,
    designFlip: doc.designFlip,
    designRotate: doc.designRotate,
    designRotate90: doc.designRotate90,
    sourceGrid: {
      resolution: src.Resolution,
      material: src.Material,
      thickness: src.Thickness,
      centerX: src.CenterX,
      centerY: src.CenterY,
      directionRad: src.DirectionRad,
      points: parsePoints2D(src.Points),
    },
    targetGrid: {
      resolution: tgt.Resolution,
      material: tgt.Material,
      thickness: tgt.Thickness,
      centerX: tgt.CenterX,
      centerY: tgt.CenterY,
      directionRad: tgt.DirectionRad,
      points: parsePoints2D(tgt.Points),
    },
    designBouEdges: (doc.designBouEdges || []).map(parsePoints1D),
    measureBouEdges: (doc.measureBouEdges || []).map(parsePoints1D),
    seamNames: parseNamePoints(doc.seamNames),
    directionNames: parseNamePoints(doc.directionNames),
    bendingCurves: curves,
  };
}

async function readFileAsText(file) {
  const buf = await file.arrayBuffer();
  let text = new TextDecoder('utf-8').decode(buf);
  if (text.charCodeAt(0) === 0xFEFF) text = text.slice(1); // BOM 제거
  return text;
}

async function ingestFiles(fileList) {
  const added = [];
  for (const file of fileList) {
    try {
      const text = await readFileAsText(file);
      const doc = JSON.parse(text);
      const ds = normalizeDataset(doc, file.name);
      DATASETS.push(ds);
      added.push(ds);
    } catch (err) {
      alert(`${file.name} 파싱 실패: ${err.message}`);
    }
  }
  if (added.length) {
    if (primaryIdx < 0) primaryIdx = DATASETS.length - added.length;
    renderDatasetList();
    reloadScene();
    showUIPanels(true);
  }
  document.getElementById('uploadStatus').textContent = `업로드된 데이터셋: ${DATASETS.length}`;
  return added;
}

// ============================================================================
// 유틸: 컬러맵, 기하 생성
// ============================================================================
function viridis(t) {
  const anchors = [
    [0.267, 0.005, 0.329],
    [0.229, 0.322, 0.545],
    [0.127, 0.568, 0.549],
    [0.369, 0.788, 0.384],
    [0.993, 0.906, 0.144],
  ];
  t = Math.max(0, Math.min(1, t));
  const s = t * (anchors.length - 1);
  const i = Math.floor(s);
  const f = s - i;
  if (i >= anchors.length - 1) return anchors[anchors.length - 1];
  const a = anchors[i], b = anchors[i + 1];
  return [a[0]*(1-f)+b[0]*f, a[1]*(1-f)+b[1]*f, a[2]*(1-f)+b[2]*f];
}

function makeSurface(points2d, color, { withColors = false } = {}) {
  if (!points2d || !points2d.length || !points2d[0].length) return null;
  const rows = points2d.length;
  const cols = points2d[0].length;
  const positions = new Float32Array(rows * cols * 3);
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      const p = points2d[r][c];
      const idx = (r * cols + c) * 3;
      positions[idx] = p[0];
      positions[idx + 1] = p[1];
      positions[idx + 2] = p[2];
    }
  }
  const indices = [];
  for (let r = 0; r < rows - 1; r++) {
    for (let c = 0; c < cols - 1; c++) {
      const a = r * cols + c;
      const b = r * cols + (c + 1);
      const d = (r + 1) * cols + c;
      const e = (r + 1) * cols + (c + 1);
      indices.push(a, b, e, a, e, d);
    }
  }
  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geom.setIndex(indices);
  if (withColors) {
    geom.setAttribute('color', new THREE.BufferAttribute(new Float32Array(rows*cols*3), 3));
  }
  geom.computeVertexNormals();
  const mat = new THREE.MeshStandardMaterial({
    color, side: THREE.DoubleSide, transparent: true,
    opacity: 0.75, metalness: 0.1, roughness: 0.85,
    vertexColors: withColors,
  });
  const mesh = new THREE.Mesh(geom, mat);
  mesh.userData = { rows, cols };
  return mesh;
}

// Source 그리드 포인트의 실제 3D 위치에 정사각형 scatter (viridis) 로 편차 표시.
// 면이 아니라 점군이라 Source/Target 표면과 독립적으로 토글 가능.
function makeDeviationScatter(src2d, tgt2d, pointSize = 8) {
  if (!src2d?.length || !tgt2d?.length) return { obj: null, min: 0, max: 0 };
  const rows = Math.min(src2d.length, tgt2d.length);
  const cols = Math.min(src2d[0]?.length || 0, tgt2d[0]?.length || 0);
  if (!rows || !cols) return { obj: null, min: 0, max: 0 };
  const n = rows * cols;
  const pos = new Float32Array(n * 3);
  const col = new Float32Array(n * 3);
  const dists = new Float32Array(n);
  let dMin = Infinity, dMax = -Infinity;
  for (let r = 0; r < rows; r++) for (let c = 0; c < cols; c++) {
    const i = r * cols + c;
    const a = src2d[r][c], b = tgt2d[r][c];
    pos[i*3]     = a[0];
    pos[i*3 + 1] = a[1];
    pos[i*3 + 2] = a[2];   // 곡면 위 (= curve 입력점 높이) 에 그대로 안착
    const dx = a[0]-b[0], dy = a[1]-b[1], dz = a[2]-b[2];
    const d = Math.sqrt(dx*dx + dy*dy + dz*dz);
    dists[i] = d;
    if (d < dMin) dMin = d;
    if (d > dMax) dMax = d;
  }
  const span = Math.max(1e-6, dMax - dMin);
  for (let i = 0; i < n; i++) {
    const t = (dists[i] - dMin) / span;
    const [rr, gg, bb] = viridis(t);
    col[i*3] = rr; col[i*3 + 1] = gg; col[i*3 + 2] = bb;
  }
  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geom.setAttribute('color',    new THREE.BufferAttribute(col, 3));
  // map 없는 PointsMaterial 은 GPU 가 정사각형 점으로 그림 (이미지 참조 스타일)
  const mat = new THREE.PointsMaterial({
    size: pointSize,
    sizeAttenuation: false,
    vertexColors: true,
    transparent: true,
    depthTest: false,        // 표면 뒤로 가려져도 보이게 (독립 시각화)
    depthWrite: false,
  });
  const obj = new THREE.Points(geom, mat);
  obj.renderOrder = 12;
  return { obj, min: dMin, max: dMax };
}

function makePolyline(points, color, { dashed = false } = {}) {
  if (!points || points.length < 2) return null;
  const pts = points.map(p => new THREE.Vector3(p[0], p[1], p[2]));
  const geom = new THREE.BufferGeometry().setFromPoints(pts);
  const mat = dashed
    ? new THREE.LineDashedMaterial({ color, dashSize: 30, gapSize: 20, transparent: true })
    : new THREE.LineBasicMaterial({ color, transparent: true });
  const line = new THREE.Line(geom, mat);
  if (dashed) line.computeLineDistances();
  return line;
}

function makeTube(points, color, radius = 8) {
  if (!points || points.length < 2) return null;
  const pts = points.map(p => new THREE.Vector3(p[0], p[1], p[2]));
  const curve = new THREE.CatmullRomCurve3(pts, false, 'centripetal', 0.3);
  const segs = Math.max(32, Math.min(400, pts.length * 6));
  const geom = new THREE.TubeGeometry(curve, segs, radius, 6, false);
  const mat = new THREE.MeshStandardMaterial({
    color, metalness: 0.2, roughness: 0.6,
    transparent: true, opacity: 1.0,
    emissive: color, emissiveIntensity: 0.35,
  });
  return new THREE.Mesh(geom, mat);
}

// 동그라미 스프라이트 텍스처 (Top-View Deviation 용)
function makeCircleTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 64;
  const ctx = c.getContext('2d');
  // 부드러운 가장자리 + 흰색 윤곽 → vertexColors로 색을 입혀도 또렷
  const grad = ctx.createRadialGradient(32, 32, 0, 32, 32, 30);
  grad.addColorStop(0.0, 'rgba(255,255,255,1)');
  grad.addColorStop(0.55, 'rgba(255,255,255,1)');
  grad.addColorStop(0.78, 'rgba(255,255,255,0.55)');
  grad.addColorStop(0.95, 'rgba(255,255,255,0.05)');
  grad.addColorStop(1.0, 'rgba(255,255,255,0)');
  ctx.fillStyle = grad;
  ctx.beginPath();
  ctx.arc(32, 32, 32, 0, Math.PI * 2);
  ctx.fill();
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}
const CIRCLE_TEX = makeCircleTexture();

// 편차에 따른 색 (blue → red)
const TD_BLUE = new THREE.Color(0x4ea8ff);
const TD_RED  = new THREE.Color(0xff4040);
function deviationColor(dev, threshold, out) {
  const t = Math.min(1, Math.max(0, dev / Math.max(1e-6, threshold)));
  out.copy(TD_BLUE).lerp(TD_RED, t);
  return out;
}

// (src - ref) 3D 거리로 색칠한 점군. ref 가 null 이면 0 으로 간주 (Target self-reference).
function makeDeviationPointCloud(srcPts2d, refPts2d, zPlane, threshold, sizePx) {
  if (!srcPts2d?.length || !srcPts2d[0]?.length) return null;
  const rows = srcPts2d.length;
  const cols = srcPts2d[0].length;
  const n = rows * cols;
  const pos = new Float32Array(n * 3);
  const col = new Float32Array(n * 3);
  const tmp = new THREE.Color();
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      const p = srcPts2d[r][c];
      const i = r * cols + c;
      pos[i * 3]     = p[0];
      pos[i * 3 + 1] = p[1];
      pos[i * 3 + 2] = zPlane;
      let dev = 0;
      if (refPts2d?.[r]?.[c]) {
        const q = refPts2d[r][c];
        const dx = p[0] - q[0], dy = p[1] - q[1], dz = p[2] - q[2];
        dev = Math.sqrt(dx*dx + dy*dy + dz*dz);
      }
      deviationColor(dev, threshold, tmp);
      col[i * 3]     = tmp.r;
      col[i * 3 + 1] = tmp.g;
      col[i * 3 + 2] = tmp.b;
    }
  }
  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geom.setAttribute('color',    new THREE.BufferAttribute(col, 3));
  const mat = new THREE.PointsMaterial({
    size: sizePx,
    sizeAttenuation: false,
    map: CIRCLE_TEX,
    alphaTest: 0.15,
    transparent: true,
    depthWrite: false,
    vertexColors: true,
  });
  const obj = new THREE.Points(geom, mat);
  obj.renderOrder = 20;
  return obj;
}

function makePointsObj(points, color, size = 12) {
  if (!points || !points.length) return null;
  const arr = new Float32Array(points.length * 3);
  points.forEach((p, i) => { arr[i*3] = p[0]; arr[i*3+1] = p[1]; arr[i*3+2] = p[2]; });
  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.BufferAttribute(arr, 3));
  const mat = new THREE.PointsMaterial({
    color, size, sizeAttenuation: false, transparent: true, depthWrite: false,
  });
  return new THREE.Points(geom, mat);
}

function makeLabel(text, point, cls = '') {
  const el = document.createElement('div');
  el.className = `label-tag ${cls}`;
  el.textContent = text;
  const obj = new CSS2DObject(el);
  obj.position.set(point[0], point[1], point[2]);
  return obj;
}

// ============================================================================
// 씬 재구성
// ============================================================================
function disposeObj(obj) {
  obj.traverse(o => {
    if (o.geometry) o.geometry.dispose?.();
    if (o.material) {
      if (Array.isArray(o.material)) o.material.forEach(m => m.dispose?.());
      else o.material.dispose?.();
    }
  });
}

function clearScene() {
  while (root.children.length) {
    const obj = root.children.pop();
    disposeObj(obj);
  }
  while (TOP_DEV.group.children.length) {
    const obj = TOP_DEV.group.children.pop();
    disposeObj(obj);
  }
  labelHost.querySelectorAll('.label-tag').forEach(el => el.remove());
  LAYER_DEFS.forEach(l => { layerObjects[l.id] = []; });
  curveObjects.clear();
}

function rebuildTopDev() {
  while (TOP_DEV.group.children.length) {
    const o = TOP_DEV.group.children.pop();
    disposeObj(o);
  }
  if (primaryIdx < 0 || !DATASETS[primaryIdx]) {
    updateTopDevLegend();
    return;
  }
  const d = DATASETS[primaryIdx];
  // 메쉬 bbox 의 max.z 를 기준으로 평면 결정
  const box = new THREE.Box3().setFromObject(root);
  const baseZ = box.isEmpty() ? 0 : box.max.z;
  const layerSpacing = TOP_DEV.zOffset * 0.45;     // 세 층이 겹치지 않게 약간 띄움

  // 1) Source 편차: |source - target|
  if (TOP_DEV.source) {
    const obj = makeDeviationPointCloud(
      d.sourceGrid.points, d.targetGrid.points,
      baseZ + TOP_DEV.zOffset,
      TOP_DEV.threshold, TOP_DEV.size,
    );
    if (obj) TOP_DEV.group.add(obj);
  }
  // 2) Target 자기 기준 (편차 0 → 전부 파랑, 위치 확인용)
  if (TOP_DEV.target) {
    const obj = makeDeviationPointCloud(
      d.targetGrid.points, null,
      baseZ + TOP_DEV.zOffset + layerSpacing,
      TOP_DEV.threshold, TOP_DEV.size,
    );
    if (obj) TOP_DEV.group.add(obj);
  }
  // 3) 공정후 데이터: 아직 없음 (placeholder)
  // postPoints 가 추후 추가되면 같은 패턴으로 빌드.
  updateTopDevLegend();
}

function updateTopDevLegend() {
  const ticks = document.getElementById('topDevLegendTicks');
  if (!ticks) return;
  const t = TOP_DEV.threshold;
  ticks.innerHTML =
    `<span>0</span><span>${(t/2).toFixed(1)}</span><span>≥ ${t.toFixed(1)} mm</span>`;
}

function reloadScene() {
  clearScene();
  if (primaryIdx < 0 || !DATASETS[primaryIdx]) {
    emptyState.classList.toggle('hidden', false);
    return;
  }
  emptyState.classList.toggle('hidden', true);
  const d = DATASETS[primaryIdx];

  // Source 표면 (단색)
  const srcMesh = makeSurface(d.sourceGrid.points, 0x4ea8ff);
  if (srcMesh) {
    srcMesh.material.opacity = LAYER_STATE.source.opacity;
    srcMesh.material.depthWrite = false;
    srcMesh.renderOrder = 1;
    srcMesh.material.needsUpdate = true;
    root.add(srcMesh);
    layerObjects.source.push(srcMesh);
  }

  // Target 표면
  const tgtMesh = makeSurface(d.targetGrid.points, 0xffb545);
  if (tgtMesh) {
    tgtMesh.material.opacity = LAYER_STATE.target.opacity;
    tgtMesh.material.depthWrite = false;
    tgtMesh.renderOrder = 2;
    root.add(tgtMesh);
    layerObjects.target.push(tgtMesh);
  }

  // Deviation Scatter — Source 그리드 위치에 정사각형 점으로 viridis 색칠
  const devScatter = makeDeviationScatter(d.sourceGrid.points, d.targetGrid.points, 8);
  if (devScatter.obj) {
    root.add(devScatter.obj);
    layerObjects.deviation.push(devScatter.obj);
  }
  const devRange = { min: devScatter.min, max: devScatter.max };
  updateLegend(devRange);

  // Design boundary edges
  (d.designBouEdges || []).forEach(edge => {
    const line = makePolyline(edge, 0xffb545);
    if (line) { line.material.opacity = LAYER_STATE.designEdges.opacity; root.add(line); layerObjects.designEdges.push(line); }
  });

  // Measured boundary edges
  (d.measureBouEdges || []).forEach(edge => {
    const line = makePolyline(edge, 0x4ea8ff, { dashed: true });
    if (line) { line.material.opacity = LAYER_STATE.measureEdges.opacity; root.add(line); layerObjects.measureEdges.push(line); }
  });

  // Bending curves
  const curveColors = [
    0xff5577, 0xff9a3c, 0xffd24c, 0x4cff9a, 0x4ad6ff,
    0x9a6bff, 0xff4cff, 0xc6ff4c, 0xffff88, 0x66ffff,
    0xff88aa, 0x88ff88, 0xaa88ff, 0xffaa66, 0x66aaff,
  ];
  (d.bendingCurves || []).forEach((c, i) => {
    const col = curveColors[i % curveColors.length];
    const tube = makeTube(c.points, col, 8);
    if (tube) {
      tube.material.opacity = LAYER_STATE.curves.opacity;
      tube.renderOrder = 10;
      tube.userData = { curveId: c.curveId, curveIdx: i };
      root.add(tube);
      layerObjects.curves.push(tube);
      curveObjects.set(c.curveId, { line: tube, color: col });
    }
  });

  // Direction / Seam labels
  (d.directionNames || []).forEach(n => {
    if (!n.point) return;
    const lbl = makeLabel(n.text, n.point, 'direction');
    root.add(lbl);
    layerObjects.directions.push(lbl);
  });
  (d.seamNames || []).forEach(n => {
    if (!n.point) return;
    const lbl = makeLabel(n.text, n.point, 'seam');
    root.add(lbl);
    layerObjects.seams.push(lbl);
  });

  // Compare datasets (각 Source 표면만)
  let paletteIdx = 0;
  for (const idx of compareIndices) {
    if (idx === primaryIdx || !DATASETS[idx]) continue;
    const c = DATASETS[idx];
    const col = COMPARE_PALETTE[paletteIdx++ % COMPARE_PALETTE.length];
    const mesh = makeSurface(c.sourceGrid.points, col);
    if (mesh) {
      mesh.material.opacity = LAYER_STATE.compare.opacity;
      mesh.material.depthWrite = false;
      mesh.renderOrder = 3;
      mesh.userData = { compareIdx: idx };
      root.add(mesh);
      layerObjects.compare.push(mesh);
      // 라벨: 파일명
      const pts = c.sourceGrid.points;
      const cr = pts[Math.floor(pts.length/2)];
      const cp = cr[Math.floor(cr.length/2)];
      if (cp) {
        const lbl = makeLabel(`△ ${c.fileName.slice(0, 14)}…`, cp, '');
        lbl.element.style.color = '#' + new THREE.Color(col).getHexString();
        lbl.element.style.borderColor = '#' + new THREE.Color(col).getHexString();
        root.add(lbl);
        layerObjects.compare.push(lbl);
      }
    }
  }

  LAYER_DEFS.forEach(l => setLayerVisible(l.id, LAYER_STATE[l.id].visible));
  LAYER_DEFS.forEach(l => setLayerOpacity(l.id, LAYER_STATE[l.id].opacity));

  renderDatasetMeta(d);
  renderMetrics(d, devRange);
  renderCurvesList(d);
  renderCurveDetail(null);
  rebuildTopDev();
  fitCamera();
}

// 카메라 위치 계산: Z-up 구면 좌표 → 월드 위치
function sphericalToPosition(targetCenter, azDeg, elDeg, distance) {
  // top/bottom 특이점 회피용 클램프 (±89.5°)
  const elClamped = Math.max(-89.5, Math.min(89.5, elDeg));
  const az = THREE.MathUtils.degToRad(azDeg);
  const el = THREE.MathUtils.degToRad(elClamped);
  const dir = new THREE.Vector3(
    Math.cos(el) * Math.cos(az),
    Math.cos(el) * Math.sin(az),
    Math.sin(el),
  );
  return targetCenter.clone().add(dir.multiplyScalar(distance));
}

function getCurrentAngles() {
  // 현재 카메라의 az/el (deg) 계산
  const v = camera.position.clone().sub(controls.target);
  const d = v.length();
  if (d < 1e-6) return { az: 0, el: 35, dist: 1 };
  const el = THREE.MathUtils.radToDeg(Math.asin(v.z / d));
  const az = THREE.MathUtils.radToDeg(Math.atan2(v.y, v.x));
  return { az, el, dist: d };
}

function fitDistanceFromBox(box) {
  if (box.isEmpty()) return 5000;
  const sphere = box.getBoundingSphere(new THREE.Sphere());
  return sphere.radius / Math.sin((camera.fov * Math.PI) / 360) * 1.25;
}

// 프리셋 시점 정의 (Z-up 좌표계)
const VIEW_PRESETS = {
  iso:    { az: 45,  el: 35,  up: [0, 0, 1] },
  top:    { az: 90,  el: 89.5, up: [0, 1, 0] },   // +Z 에서 내려다봄, 화면 위 = +Y
  bottom: { az: 90,  el: -89.5, up: [0, 1, 0] },
  front:  { az: -90, el: 0,   up: [0, 0, 1] },    // -Y 에서 바라봄 (모델의 "앞")
  back:   { az: 90,  el: 0,   up: [0, 0, 1] },
  left:   { az: 180, el: 0,   up: [0, 0, 1] },
  right:  { az: 0,   el: 0,   up: [0, 0, 1] },
};

function applyView(name, { animate = true, fit = false } = {}) {
  const preset = VIEW_PRESETS[name];
  if (!preset) return;
  const box = new THREE.Box3().setFromObject(root);
  if (box.isEmpty()) return;
  const sphere = box.getBoundingSphere(new THREE.Sphere());
  const dist = fit ? fitDistanceFromBox(box) : Math.max(camera.position.distanceTo(controls.target), fitDistanceFromBox(box));
  const pos = sphericalToPosition(sphere.center, preset.az, preset.el, dist);
  const upVec = new THREE.Vector3(preset.up[0], preset.up[1], preset.up[2]);
  if (animate) animateCameraTo(pos, sphere.center, upVec);
  else {
    camera.position.copy(pos);
    controls.target.copy(sphere.center);
    camera.up.copy(upVec);
    controls.update();
  }
  syncAngleInputs(preset.az, preset.el);
}

function applyCustomAngle(azDeg, elDeg, { animate = true, fit = false } = {}) {
  const box = new THREE.Box3().setFromObject(root);
  if (box.isEmpty()) return;
  const sphere = box.getBoundingSphere(new THREE.Sphere());
  const dist = fit ? fitDistanceFromBox(box) : Math.max(camera.position.distanceTo(controls.target), 1);
  const pos = sphericalToPosition(sphere.center, azDeg, elDeg, dist);
  const upVec = Math.abs(elDeg) > 88
    ? new THREE.Vector3(0, 1, 0)
    : new THREE.Vector3(0, 0, 1);
  if (animate) animateCameraTo(pos, sphere.center, upVec);
  else {
    camera.position.copy(pos);
    controls.target.copy(sphere.center);
    camera.up.copy(upVec);
    controls.update();
  }
}

function syncAngleInputs(az, el) {
  const azEl = document.getElementById('azInput');
  const elEl = document.getElementById('elInput');
  if (azEl) azEl.value = Math.round(az);
  if (elEl) elEl.value = Math.round(el);
}

function fitCamera() {
  applyView('iso', { animate: true, fit: true });
}

// ============================================================================
// 레이어 조작
// ============================================================================
function setLayerVisible(id, visible) {
  LAYER_STATE[id].visible = visible;
  (layerObjects[id] || []).forEach(o => { o.visible = visible; });
  if (id === 'grid') gridHelper.visible = visible;
  if (id === 'axes') axesHelper.visible = visible;
}

function setLayerOpacity(id, opacity) {
  LAYER_STATE[id].opacity = opacity;
  (layerObjects[id] || []).forEach(o => {
    if (o.material) {
      o.material.opacity = opacity;
      o.material.transparent = opacity < 1;
    }
    if (o.element) o.element.style.opacity = opacity;
  });
  if (id === 'grid') gridHelper.material.opacity = opacity;
}

// ============================================================================
// 사이드바 렌더링
// ============================================================================
function kv(k, v, hi = false) {
  return `<div class="kv"><span class="k">${k}</span><span class="v${hi ? ' hi' : ''}">${v ?? '—'}</span></div>`;
}
function fmt(v, digits = 2) {
  if (v == null || v === '') return '—';
  if (typeof v === 'number') return v.toFixed(digits);
  return String(v);
}

function renderDatasetMeta(d) {
  document.getElementById('datasetMeta').innerHTML = [
    kv('File', d.fileName),
    kv('LegacyId', d.legacyId),
    kv('Part', d.partName),
    kv('CurveType', d.curveType),
    kv('Plate', d.plateName || '—'),
    kv('Thickness', `${d.thickness} mm`),
    kv('Device', d.deviceCode),
    kv('Created', d.createdAt),
    kv('DirectionKey', d.directionKey),
    kv('Mirror/Flip/Rot/Rot90',
      `${d.designMirror} / ${d.designFlip} / ${d.designRotate} / ${d.designRotate90}`),
  ].join('');
}

function renderMetrics(d, dev) {
  document.getElementById('metricsBlock').innerHTML = [
    kv('Completed Ratio 1', `${fmt(d.completedRatio1)} %`, true),
    kv('Completed Ratio 2', `${fmt(d.completedRatio2)} %`, true),
    kv('Residual Norm', fmt(d.residualNorm), true),
    kv('Curve Count', d.curveCount),
    kv('Total Curve Length', `${fmt(d.totalCurveLength, 1)} mm`),
    kv('Src Grid', `${d.sourceGrid.points.length} × ${d.sourceGrid.points[0]?.length || 0}`),
    kv('Tgt Grid', `${d.targetGrid.points.length} × ${d.targetGrid.points[0]?.length || 0}`),
    kv('Deviation min/max', `${fmt(dev.min)} / ${fmt(dev.max)} mm`),
  ].join('');
}

function renderDatasetList() {
  const host = document.getElementById('datasetList');
  host.innerHTML = '';
  DATASETS.forEach((d, i) => {
    const row = document.createElement('div');
    row.className = 'dataset-item';
    const isPrimary = i === primaryIdx;
    const isCompare = compareIndices.has(i);
    const paletteIdx = [...compareIndices].indexOf(i);
    const compareColor = paletteIdx >= 0
      ? '#' + new THREE.Color(COMPARE_PALETTE[paletteIdx % COMPARE_PALETTE.length]).getHexString()
      : '#444';
    row.innerHTML = `
      <input type="radio" name="primary" ${isPrimary ? 'checked' : ''} title="Primary" />
      <input type="checkbox" ${isCompare ? 'checked' : ''} title="Compare" />
      <span class="name" title="${d.fileName}">${i + 1}. ${d.partName || '?'} · R1=${fmt(d.completedRatio1, 1)}% · ${d.fileName}</span>
      <span class="sw" style="background:${compareColor}"></span>
    `;
    const [rad, cb] = row.querySelectorAll('input');
    rad.addEventListener('change', () => { primaryIdx = i; renderDatasetList(); reloadScene(); });
    cb.addEventListener('change', e => {
      if (e.target.checked) compareIndices.add(i);
      else compareIndices.delete(i);
      renderDatasetList();
      reloadScene();
    });
    host.appendChild(row);
  });
}

function renderCurvesList(d) {
  const el = document.getElementById('curvesList');
  el.innerHTML = '';
  (d.bendingCurves || []).forEach((c, i) => {
    const row = document.createElement('div');
    row.className = 'curve-item';
    row.dataset.curveId = c.curveId;
    const cbId = `cbCurve-${i}`;
    const entry = curveObjects.get(c.curveId);
    const col = entry?.color ?? 0xff5577;
    const hex = '#' + new THREE.Color(col).getHexString();
    row.innerHTML = `
      <input type="checkbox" id="${cbId}" checked />
      <span class="sw" style="background:${hex}"></span>
      <span>
        <span class="id">${c.curveId}</span>
        <span class="meta">M${c.heatingMethod} · ${fmt(c.speed,1)}mm/s · T${(c.paramTemp || []).slice(0,2).map(v => fmt(v,0)).join('/')}</span>
      </span>
      <span class="meta">${(c.points || []).length}p</span>
    `;
    row.querySelector('input').addEventListener('change', e => {
      e.stopPropagation();
      const v = e.target.checked;
      const obj = curveObjects.get(c.curveId);
      if (obj && obj.line) obj.line.visible = v && LAYER_STATE.curves.visible;
    });
    row.addEventListener('click', e => {
      if (e.target.tagName === 'INPUT') return;
      document.querySelectorAll('.curve-item').forEach(r => r.classList.remove('active'));
      row.classList.add('active');
      renderCurveDetail(c);
    });
    el.appendChild(row);
  });
}

function renderCurveDetail(c) {
  const el = document.getElementById('curveDetail');
  if (!c) {
    el.innerHTML = '<em class="hint">곡선을 선택하면 상세 파라미터가 표시됩니다.</em>';
    return;
  }
  el.innerHTML = [
    kv('CurveId', c.curveId),
    kv('HeatingMethod', c.heatingMethod),
    kv('isAutoMade', c.isAutoMaked),
    kv('Speed', `${fmt(c.speed, 2)} mm/s`, true),
    kv('CoolingTime', `${fmt(c.coolingTime, 2)} s`, true),
    kv('Strength', fmt(c.strength)),
    kv('Width', fmt(c.width)),
    kv('HeatIntensity', fmt(c.heatIntensity)),
    kv('Split', fmt(c.split)),
    kv('WeavingRepeat', c.weavingRepeat),
    kv('THK', c.thk),
    kv('ParamTemp (°C)', JSON.stringify(c.paramTemp), true),
    kv('ParamShape', JSON.stringify(c.paramShape)),
    kv('ParamStep', c.paramStep),
    kv('ParamRepeat', JSON.stringify(c.paramRepeat)),
    kv('ParamCoolingSpeed', fmt(c.paramCoolingSpeed)),
    kv('Points(curve)', (c.points || []).length),
    kv('InputPoints(user)', (c.inputPoints || []).length),
  ].join('');
}

function updateLegend({ min, max }) {
  const ticks = document.getElementById('legendTicks');
  if (!ticks) return;
  const mid = (min + max) / 2;
  ticks.innerHTML = `<span>${fmt(min)}</span><span>${fmt(mid)}</span><span>${fmt(max)} (mm)</span>`;
}

function showUIPanels(show) {
  ['metaPanel','metricsPanel','layersPanel','curvesPanel','curveDetailPanel','viewPanel','topDevPanel','legendPanel']
    .forEach(id => {
      const el = document.getElementById(id);
      if (show) el.removeAttribute('hidden');
      else el.setAttribute('hidden', '');
    });
}

// ============================================================================
// 사이드바 초기화
// ============================================================================
function buildLayerUI() {
  const host = document.getElementById('layerList');
  host.innerHTML = '';
  LAYER_DEFS.forEach(l => {
    const row = document.createElement('div');
    row.className = 'layer-item';
    const cbId = `cb-${l.id}`;
    const opId = `op-${l.id}`;
    const hex = '#' + new THREE.Color(l.color).getHexString();
    row.innerHTML = `
      <input type="checkbox" id="${cbId}" />
      <label for="${cbId}" title="${l.name}">
        <span class="sw" style="background:${hex}"></span>
        <span class="name">${l.name}</span>
      </label>
      <input class="opacity" type="range" id="${opId}" min="0" max="1" step="0.01" value="${l.opacity}" />
    `;
    host.appendChild(row);
    const cb = row.querySelector(`#${cbId}`);
    cb.checked = !!LAYER_STATE[l.id].visible;
    cb.addEventListener('change', e => setLayerVisible(l.id, e.target.checked));
    row.querySelector(`#${opId}`).addEventListener('input', e => setLayerOpacity(l.id, parseFloat(e.target.value)));
  });
}

// 뷰/카메라 컨트롤
document.getElementById('resetView').addEventListener('click', () => fitCamera());
document.getElementById('isoView').addEventListener('click', () => applyView('iso', { animate: true, fit: true }));
document.querySelectorAll('#viewPanel button[data-view]').forEach(btn => {
  btn.addEventListener('click', () => applyView(btn.dataset.view, { animate: true }));
});
document.getElementById('goAngle').addEventListener('click', () => {
  const az = parseFloat(document.getElementById('azInput').value);
  const el = parseFloat(document.getElementById('elInput').value);
  if (Number.isNaN(az) || Number.isNaN(el)) return;
  applyCustomAngle(az, el, { animate: true });
});
document.getElementById('syncAngle').addEventListener('click', () => {
  const cur = getCurrentAngles();
  syncAngleInputs(cur.az, cur.el);
});
// Enter 로도 적용
['azInput', 'elInput'].forEach(id => {
  document.getElementById(id).addEventListener('keydown', e => {
    if (e.key === 'Enter') document.getElementById('goAngle').click();
  });
});
document.getElementById('smoothCam').addEventListener('change', e => {
  controls.enableDamping = e.target.checked;
});
document.getElementById('toggleLabels').addEventListener('change', e => {
  labelHost.style.display = e.target.checked ? '' : 'none';
});
document.getElementById('toggleWire').addEventListener('change', e => {
  const v = e.target.checked;
  [...layerObjects.source, ...layerObjects.target, ...layerObjects.compare].forEach(m => {
    if (m.material) { m.material.wireframe = v; m.material.needsUpdate = true; }
  });
});

document.getElementById('layersAll').addEventListener('click', () => {
  LAYER_DEFS.forEach(l => {
    LAYER_STATE[l.id].visible = true;
    const cb = document.getElementById(`cb-${l.id}`);
    if (cb) cb.checked = true;
    setLayerVisible(l.id, true);
  });
});
document.getElementById('layersNone').addEventListener('click', () => {
  LAYER_DEFS.forEach(l => {
    LAYER_STATE[l.id].visible = false;
    const cb = document.getElementById(`cb-${l.id}`);
    if (cb) cb.checked = false;
    setLayerVisible(l.id, false);
  });
});

document.getElementById('curvesAll').addEventListener('click', () => {
  document.querySelectorAll('#curvesList input[type="checkbox"]').forEach(cb => {
    if (!cb.checked) { cb.checked = true; cb.dispatchEvent(new Event('change')); }
  });
});
document.getElementById('curvesNone').addEventListener('click', () => {
  document.querySelectorAll('#curvesList input[type="checkbox"]').forEach(cb => {
    if (cb.checked) { cb.checked = false; cb.dispatchEvent(new Event('change')); }
  });
});

document.getElementById('clearDatasets').addEventListener('click', () => {
  if (!DATASETS.length) return;
  if (!confirm(`데이터셋 ${DATASETS.length}개를 모두 제거할까요?`)) return;
  DATASETS.length = 0;
  primaryIdx = -1;
  compareIndices.clear();
  renderDatasetList();
  reloadScene();
  showUIPanels(false);
  document.getElementById('uploadStatus').textContent = `업로드된 데이터셋: 0`;
});

// 파일 업로드 (input + drag-drop)
const fileInput = document.getElementById('fileInput');
const dropZone = document.getElementById('dropZone');
fileInput.addEventListener('change', e => {
  ingestFiles([...e.target.files]);
  e.target.value = '';
});
['dragenter', 'dragover'].forEach(ev => dropZone.addEventListener(ev, e => {
  e.preventDefault(); e.stopPropagation(); dropZone.classList.add('over');
}));
['dragleave', 'drop'].forEach(ev => dropZone.addEventListener(ev, e => {
  e.preventDefault(); e.stopPropagation(); dropZone.classList.remove('over');
}));
dropZone.addEventListener('drop', e => {
  const files = [...(e.dataTransfer?.files || [])].filter(f => f.name.toLowerCase().endsWith('.json'));
  if (files.length) ingestFiles(files);
});

// 전체 뷰포트로도 드래그 지원
window.addEventListener('dragover', e => { e.preventDefault(); });
window.addEventListener('drop', e => {
  if (e.target.closest('#dropZone')) return;
  e.preventDefault();
  const files = [...(e.dataTransfer?.files || [])].filter(f => f.name.toLowerCase().endsWith('.json'));
  if (files.length) ingestFiles(files);
});

// 곡선 raycaster
const raycaster = new THREE.Raycaster();
raycaster.params.Line.threshold = 30;
raycaster.params.Points.threshold = 30;
const mouse = new THREE.Vector2();
function pickCurveFromEvent(e) {
  const rect = renderer.domElement.getBoundingClientRect();
  mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
  mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(mouse, camera);
  const targets = layerObjects.curves.filter(o => o.visible);
  const hits = raycaster.intersectObjects(targets, false);
  return hits[0]?.object ?? null;
}
renderer.domElement.addEventListener('click', e => {
  const obj = pickCurveFromEvent(e);
  if (!obj || primaryIdx < 0) return;
  const cid = obj.userData.curveId;
  document.querySelectorAll('.curve-item').forEach(r => r.classList.toggle('active', r.dataset.curveId === cid));
  const c = DATASETS[primaryIdx].bendingCurves.find(c => c.curveId === cid);
  if (c) renderCurveDetail(c);
});
renderer.domElement.addEventListener('mousemove', e => {
  const obj = pickCurveFromEvent(e);
  if (obj && primaryIdx >= 0) {
    const c = DATASETS[primaryIdx].bendingCurves.find(c => c.curveId === obj.userData.curveId);
    if (c) {
      tooltip.style.display = 'block';
      tooltip.style.left = `${e.clientX + 12}px`;
      tooltip.style.top = `${e.clientY + 12}px`;
      tooltip.innerHTML =
        `<b>${c.curveId}</b><br>method=${c.heatingMethod}, speed=${c.speed}<br>` +
        `temp=[${(c.paramTemp || []).join(',')}]<br>cool=${c.coolingTime}s`;
      return;
    }
  }
  tooltip.style.display = 'none';
});
renderer.domElement.addEventListener('mouseleave', () => { tooltip.style.display = 'none'; });

// ============================================================================
// Top-View Deviation UI 와이어링
// ============================================================================
function bindTopDevUI() {
  const bindToggle = (cbId, key) => {
    const cb = document.getElementById(cbId);
    if (!cb) return;
    cb.checked = TOP_DEV[key];
    cb.addEventListener('change', e => {
      TOP_DEV[key] = e.target.checked;
      rebuildTopDev();
    });
  };
  bindToggle('topDevSourceCb', 'source');
  bindToggle('topDevTargetCb', 'target');
  bindToggle('topDevPostCb',   'post');   // disabled — 안전하게 노출만

  const bindSlider = (rangeId, valId, key, formatter) => {
    const r = document.getElementById(rangeId);
    const v = document.getElementById(valId);
    if (!r) return;
    r.value = TOP_DEV[key];
    v.textContent = formatter(TOP_DEV[key]);
    r.addEventListener('input', e => {
      const x = parseFloat(e.target.value);
      TOP_DEV[key] = x;
      v.textContent = formatter(x);
      rebuildTopDev();
    });
  };
  bindSlider('topDevThreshold', 'topDevThresholdVal', 'threshold', x => x.toFixed(1));
  bindSlider('topDevZOffset',   'topDevZOffsetVal',   'zOffset',   x => `${x}`);
  bindSlider('topDevSize',      'topDevSizeVal',      'size',      x => `${x}`);

  updateTopDevLegend();
}

// ============================================================================
// 부팅
// ============================================================================
buildLayerUI();
bindTopDevUI();
resize();
showUIPanels(false);

function animate() {
  updateCameraTween();
  controls.update();
  renderer.render(scene, camera);
  labelRenderer.render(scene, camera);
  requestAnimationFrame(animate);
}
animate();

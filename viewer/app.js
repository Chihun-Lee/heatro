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
  { id: 'deviation',   name: 'Deviation Heatmap (Src↔Tgt)',  color: 0x00ff88, opacity: 0.9  },
  { id: 'designEdges', name: 'Design Boundary Edges',        color: 0xffb545, opacity: 1.0  },
  { id: 'measureEdges',name: 'Measured Boundary Edges',      color: 0x4ea8ff, opacity: 1.0  },
  { id: 'curves',      name: 'Bending Curves (열 프로파일)', color: 0xff5577, opacity: 1.0  },
  { id: 'curveInputs', name: 'Curve 입력 점',                color: 0xffffff, opacity: 1.0  },
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

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

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

function applyDeviationColors(mesh, src2d, tgt2d) {
  if (!mesh || !src2d?.length || !tgt2d?.length) return { min: 0, max: 0 };
  const rows = Math.min(src2d.length, tgt2d.length);
  const cols = Math.min(src2d[0].length, tgt2d[0].length);
  const dists = new Float32Array(rows * cols);
  let dMin = Infinity, dMax = -Infinity;
  for (let r = 0; r < rows; r++) for (let c = 0; c < cols; c++) {
    const a = src2d[r][c], b = tgt2d[r][c];
    const dx = a[0]-b[0], dy = a[1]-b[1], dz = a[2]-b[2];
    const d = Math.sqrt(dx*dx + dy*dy + dz*dz);
    dists[r*cols+c] = d;
    if (d < dMin) dMin = d;
    if (d > dMax) dMax = d;
  }
  const colors = mesh.geometry.getAttribute('color');
  if (!colors) return { min: dMin, max: dMax };
  const span = Math.max(1e-6, dMax - dMin);
  for (let i = 0; i < dists.length; i++) {
    const t = (dists[i] - dMin) / span;
    const [r, g, b] = viridis(t);
    colors.array[i*3] = r; colors.array[i*3+1] = g; colors.array[i*3+2] = b;
  }
  colors.needsUpdate = true;
  return { min: dMin, max: dMax };
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
function clearScene() {
  while (root.children.length) {
    const obj = root.children.pop();
    obj.traverse(o => {
      if (o.geometry) o.geometry.dispose?.();
      if (o.material) {
        if (Array.isArray(o.material)) o.material.forEach(m => m.dispose?.());
        else o.material.dispose?.();
      }
    });
  }
  labelHost.querySelectorAll('.label-tag').forEach(el => el.remove());
  LAYER_DEFS.forEach(l => { layerObjects[l.id] = []; });
  curveObjects.clear();
}

function reloadScene() {
  clearScene();
  if (primaryIdx < 0 || !DATASETS[primaryIdx]) {
    emptyState.classList.toggle('hidden', false);
    return;
  }
  emptyState.classList.toggle('hidden', true);
  const d = DATASETS[primaryIdx];

  // Source 표면
  const srcMesh = makeSurface(d.sourceGrid.points, 0x4ea8ff, { withColors: true });
  if (srcMesh) {
    srcMesh.material.opacity = LAYER_STATE.source.opacity;
    srcMesh.material.vertexColors = LAYER_STATE.deviation.visible;
    srcMesh.material.depthWrite = false;
    srcMesh.renderOrder = 1;
    srcMesh.material.needsUpdate = true;
    root.add(srcMesh);
    layerObjects.source.push(srcMesh);
    layerObjects.deviation.push(srcMesh);
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

  // Deviation colormap 미리 계산
  const devRange = srcMesh
    ? applyDeviationColors(srcMesh, d.sourceGrid.points, d.targetGrid.points)
    : { min: 0, max: 0 };
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
    const inpts = makePointsObj(c.inputPoints, 0xffffff, 14);
    if (inpts) {
      inpts.material.opacity = LAYER_STATE.curveInputs.opacity;
      inpts.material.depthTest = false;
      inpts.renderOrder = 11;
      root.add(inpts);
      layerObjects.curveInputs.push(inpts);
      const entry = curveObjects.get(c.curveId) || {};
      entry.dots = inpts;
      curveObjects.set(c.curveId, entry);
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
  fitCamera();
}

function fitCamera() {
  const box = new THREE.Box3().setFromObject(root);
  if (box.isEmpty()) return;
  const sphere = box.getBoundingSphere(new THREE.Sphere());
  const dist = sphere.radius / Math.sin((camera.fov * Math.PI) / 360);
  const dir = new THREE.Vector3(1, -1, 0.9).normalize();
  camera.position.copy(sphere.center.clone().add(dir.multiplyScalar(dist * 1.2)));
  controls.target.copy(sphere.center);
  controls.update();
}

// ============================================================================
// 레이어 조작
// ============================================================================
function setLayerVisible(id, visible) {
  LAYER_STATE[id].visible = visible;
  (layerObjects[id] || []).forEach(o => { o.visible = visible; });
  if (id === 'deviation') {
    layerObjects.source.forEach(m => {
      m.material.vertexColors = visible;
      m.material.needsUpdate = true;
    });
  }
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
      if (obj) {
        if (obj.line) obj.line.visible = v && LAYER_STATE.curves.visible;
        if (obj.dots) obj.dots.visible = v && LAYER_STATE.curveInputs.visible;
      }
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
  ['metaPanel','metricsPanel','layersPanel','curvesPanel','curveDetailPanel','viewPanel','legendPanel']
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

// 뷰/레이어 컨트롤
document.getElementById('resetView').addEventListener('click', fitCamera);
document.getElementById('topView').addEventListener('click', () => {
  const box = new THREE.Box3().setFromObject(root);
  if (box.isEmpty()) return;
  const s = box.getBoundingSphere(new THREE.Sphere());
  camera.position.set(s.center.x, s.center.y, s.center.z + s.radius * 2.2);
  camera.up.set(0, 1, 0);
  controls.target.copy(s.center);
  controls.update();
});
document.getElementById('sideView').addEventListener('click', () => {
  const box = new THREE.Box3().setFromObject(root);
  if (box.isEmpty()) return;
  const s = box.getBoundingSphere(new THREE.Sphere());
  camera.position.set(s.center.x, s.center.y + s.radius * 2.5, s.center.z);
  camera.up.set(0, 0, 1);
  controls.target.copy(s.center);
  controls.update();
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
// 부팅
// ============================================================================
buildLayerUI();
resize();
showUIPanels(false);

function animate() {
  controls.update();
  renderer.render(scene, camera);
  labelRenderer.render(scene, camera);
  requestAnimationFrame(animate);
}
animate();

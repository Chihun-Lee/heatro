// 뷰어에 미리 넣어 두는 가상 판을 만든다. 곡면은 모두 식으로 만든 것이고 실제 판 자료는 쓰지 않는다.
// 실행: node viewer/examples/make_examples.mjs (저장소 맨 위에서)
import { writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { PlateModel, flatten } from '../../studio/solver.js';
import { recommend, zPlaneFree, rms } from '../../studio/planner.js';

const HERE = dirname(fileURLToPath(import.meta.url));
const L = 3000, W = 2000, T = 20, NR = 31, NC = 21;
const COMP = { theta_mrad: -5, dtrans_mm: 0.02, dlong_mm: 0.025 }; // 뷰어 계산 패널의 기본 예시값과 같다
const u = (x) => (x - L / 2) / (L / 2), v = (y) => (y - W / 2) / (W / 2);
const bump = (x, y, cx, cy, s) => Math.exp(-((x - cx) ** 2 + (y - cy) ** 2) / (2 * s * s));

const EXAMPLES = [
  { id: 'cylinder', title: '원통형 판', desc: '평판을 길이방향으로 한 번 굽힌 가장 단순한 경우입니다.',
    src: () => 0, tgt: (x) => 14 * u(x) ** 2, opt: { maxLines: 8, families: ['across'] } },
  { id: 'dome', title: '볼록형 판', desc: '평판을 길이와 폭 두 방향으로 함께 굽힙니다.',
    src: () => 0, tgt: (x, y) => 12 * u(x) ** 2 + 6 * v(y) ** 2, opt: { maxLines: 10, families: ['across', 'along'] } },
  { id: 'saddle', title: '안장형 판', desc: '폭방향은 먼저 롤로 굽혀 둔 판에 길이방향 곡을 더합니다.',
    src: (x, y) => -6 * v(y) ** 2, tgt: (x, y) => 12 * u(x) ** 2 - 6 * v(y) ** 2, opt: { maxLines: 8, families: ['across'] } },
  { id: 'twist', title: '비틀린 판', desc: '비틀림이 섞여 윗면 가열만으로는 다 잡히지 않는 경우입니다.',
    src: () => 0, tgt: (x, y) => 6 * u(x) ** 2 + 7 * u(x) * v(y), opt: { maxLines: 10, families: ['across', 'along'] } },
  { id: 'finish', title: '마무리 단계 판', desc: '거의 다 된 판에 국부 편차만 남은 경우입니다.',
    src: (x) => 14 * u(x) ** 2, tgt: (x, y) => 14 * u(x) ** 2 - 4 * bump(x, y, 900, 700, 420) - 3 * bump(x, y, 2200, 1300, 380), opt: { maxLines: 6, families: ['across', 'along'] } },
];

const grid = (f) => Array.from({ length: NR }, (_, i) => Array.from({ length: NC }, (_, j) => {
  const x = i * L / (NR - 1), y = j * W / (NC - 1);
  return [x, y, f(x, y)];
}));
const fmt = (p) => `${p[0].toFixed(2)}, ${p[1].toFixed(2)}, ${p[2].toFixed(2)}`;
const fmt2 = (g) => g.map((r) => r.map(fmt));
function edges(g) {
  const nr = g.length, nc = g[0].length;
  const a = g[0], b = g.map((r) => r[nc - 1]), c = [...g[nr - 1]].reverse(), d = g.map((r) => r[0]).reverse();
  return [a, b, c, d].map((e) => e.map(fmt));
}

const index = [];
EXAMPLES.forEach((ex, n) => {
  const src = grid(ex.src), tgt = grid(ex.tgt);
  const X = flatten(src), Y = flatten(tgt);
  const diff = new Float64Array(X.length);
  for (let i = 0; i < X.length; i++) diff[i] = Y[i] - X[i];
  const D = zPlaneFree(diff, X);
  const model = new PlateModel(src, { thickness: T, refine: 2 });
  model.factorize();
  const res = recommend(model, src, D, COMP, { minGap: 150, wmax: 2, pitch: 150, ...ex.opt });
  const doc = {
    LegacyId: `demo-${ex.id}`, sPartName: `가상 예시 ${n + 1}`, sPlateName: ex.title, CurveType: 'Demo', dThickNess: T, DeviceCode: 'DEMO',
    CreateDateTime: '2026-11-10T00:00:00', DirectionKey: 'demo', complatedRatio1: null, complatedRatio2: null, ResidualNorm: Number(res.rmsBefore.toFixed(3)),
    TotalCurveLength: Math.round(res.lines.reduce((s, l) => s + l.points.slice(1).reduce((a, p, i) => a + Math.hypot(p[0] - l.points[i][0], p[1] - l.points[i][1]), 0), 0)),
    CurveCount: res.lines.length, designMirror: false, designFlip: false, designRotate: false, designRotate90: false,
    SourceGrid: { Resolution: [NR, NC], Material: 'DEMO', Thickness: T, CenterX: L / 2, CenterY: W / 2, DirectionRad: 0, Points: fmt2(src) },
    TargetGrid: { Resolution: [NR, NC], Material: 'DEMO', Thickness: T, CenterX: L / 2, CenterY: W / 2, DirectionRad: 0, Points: fmt2(tgt) },
    designBouEdges: edges(tgt), measureBouEdges: edges(src),
    seamNames: [], directionNames: [{ text: 'AFT', textPoint: fmt([0, W / 2, 0]) }, { text: 'FORE', textPoint: fmt([L, W / 2, 0]) }, { text: 'TOP', textPoint: fmt([L / 2, W, 0]) }, { text: 'BOTTOM', textPoint: fmt([L / 2, 0, 0]) }],
    BendingCurves: res.lines.map((l, i) => ({
      CurveId: `L${i + 1}`, HeatingMethod: 1, Points: l.points.map(fmt), inputUserPointList: [fmt(l.points[0]), fmt(l.points[l.points.length - 1])],
      Strength: Number(l.weight.toFixed(3)), Width: 0, HeatIntensity: 1, Split: 0, WeavingRepeat: 1, THK: T, speed: 7, coolingTime: 5,
      ParameterShape: [], ParameterTemp: [800, 900], ParameterStep: 0, ParameterRepeat: 1, ParameterCoolingSpeed: 0, isAutoMaked: true,
    })),
  };
  writeFileSync(join(HERE, `${ex.id}.json`), JSON.stringify(doc));
  index.push({ id: ex.id, title: ex.title, desc: ex.desc, file: `${ex.id}.json`, lines: res.lines.length, rmsBefore: Number(res.rmsBefore.toFixed(2)), rmsAfter: Number(res.rmsAfter.toFixed(2)) });
  console.log(ex.id, 'lines', res.lines.length, 'rms', res.rmsBefore.toFixed(2), '->', res.rmsAfter.toFixed(2), 'weights', res.lines.map((l) => l.weight.toFixed(2)).join(' '));
});
writeFileSync(join(HERE, 'index.json'), JSON.stringify(index, null, 1));

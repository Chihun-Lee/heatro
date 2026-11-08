// 해석 전용 Web Worker — 화면을 멈추지 않도록 계산을 분리한다. 데이터는 이 브라우저 밖으로 나가지 않는다.
import { PlateModel } from './solver.js';
import { recommend } from './planner.js';

let model = null;
let points = null;

const post = (msg, transfer) => self.postMessage(msg, transfer ?? []);

self.onmessage = (e) => {
  const m = e.data;
  try {
    if (m.type === 'build') {
      const t0 = performance.now();
      points = m.points;
      model = new PlateModel(points, { thickness: m.thickness, refine: m.refine ?? 2 });
      model.factorize((p) => post({ type: 'progress', task: 'build', value: p }));
      post({ type: 'built', ms: performance.now() - t0, dof: model.neq, token: m.token });
    } else if (m.type === 'run') {
      const t0 = performance.now();
      const f = new Float64Array(model.neq);
      const g = new Float64Array(model.neq);
      for (const line of m.lines) {
        g.fill(0);
        model.lineLoad(line.points, line.comp, g);
        for (let i = 0; i < f.length; i++) f[i] += g[i] * line.scale;
      }
      const u = model.solve(f);
      const disp = model.topDisplacement(u);
      post({ type: 'result', disp, ms: performance.now() - t0, id: m.id }, [disp.buffer]);
    } else if (m.type === 'recommend') {
      const t0 = performance.now();
      const out = recommend(model, points, m.D, m.comp, m.options, (p) => post({ type: 'progress', task: 'recommend', value: p }));
      post({ type: 'recommended', ms: performance.now() - t0, lines: out.lines, rmsBefore: out.rmsBefore, rmsAfter: out.rmsAfter, candidates: out.candidates });
    }
  } catch (err) {
    post({ type: 'error', message: err.message });
  }
};

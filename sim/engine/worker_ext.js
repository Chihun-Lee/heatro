// 곡면 가공 시험 화면의 해석 작업자 (Web Worker).
// 공개 저장소 /studio/worker.js 와 같은 일을 하되 두 가지가 다르다.
//   1) 판 해석기로 solver_ext.js 를 쓴다 (가열선 방향 각변형 B_l, 끝단 구간, 구간 재생용 gate).
//   2) 추적선 추천(recommendShin)을 함께 부를 수 있다.
// 계산은 이 브라우저 안에서만 하고, 데이터는 밖으로 나가지 않는다.
import { PlateModel } from './solver_ext.js';
import { recommend } from '../../studio/planner.js';
import { recommendShin } from './planner_shin.js';

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
        model.lineLoad(line.points, line.comp, g, line.gate);
        for (let i = 0; i < f.length; i++) f[i] += g[i] * line.scale;
      }
      const u = model.solve(f);
      const disp = model.topDisplacement(u);
      post({ type: 'result', disp, ms: performance.now() - t0, id: m.id }, [disp.buffer]);
    } else if (m.type === 'recommend') {
      const t0 = performance.now();
      const out = recommend(model, points, m.D, m.comp, m.options, (p) => post({ type: 'progress', task: 'recommend', value: p }));
      post({ type: 'recommended', ms: performance.now() - t0, lines: out.lines, rmsBefore: out.rmsBefore, rmsAfter: out.rmsAfter, candidates: out.candidates });
    } else if (m.type === 'recommend_shin') {
      const t0 = performance.now();
      const out = recommendShin(model, points, m.D, m.comp, { ...m.options, target: m.target }, (p) => post({ type: 'progress', task: 'recommend', value: p }));
      // 화면에 필요한 것만 돌려준다 (kin, predicted 는 크고 쓰지 않는다)
      post({
        type: 'recommended_shin', ms: performance.now() - t0, lines: out.lines, rmsBefore: out.rmsBefore, rmsAfter: out.rmsAfter,
        candidates: out.candidates, counts: out.counts, stop: out.stop, pool: out.pool, poolInfo: out.poolInfo, rule: out.rule, log: out.log,
        lengthTotal: out.lengthTotal, lengthWeighted: out.lengthWeighted,
      });
    }
  } catch (err) {
    post({ type: 'error', message: err.message });
  }
};

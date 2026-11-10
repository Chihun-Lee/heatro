// 곡면 가공 시험 화면(forming.js)의 계산 도구. DOM 을 쓰지 않아 Node 시험(test_forming_link.mjs)으로 확인할 수 있다.
// 세기 부호·캐시 키, 가열 시간, 끝단 구간, AI 시뮬레이터 모델 연결을 맡는다.

// ── 세기 ─────────────────────────────────────────────────
/**
 * 세기 크기 m 에서 면에 맞는 성분을 만든다. 부호 규약은 solver_ext.js 머리말과 같다.
 * 윗면 가열: theta < 0, B_l < 0. 아랫면 가열: 부호를 뒤집는다. 막 성분(횡수축·종수축)은 면과 무관하다.
 * m.theta_mrad 는 크기로 넣는다. 가열선 방향 각변형은 m.B_l(윗면 가열 기준 부호 있는 값, mm³) 또는 m.theta_l_mrad(크기, mrad) 중 있는 것을 쓴다.
 * endZone 을 주면 그대로 붙인다 (없으면 넣지 않는다. solver_ext.js 는 이때 끝단을 쓰지 않는다).
 */
export function signedComp(m, side, endZone) {
  const th = Math.abs(m.theta_mrad);
  const c = { theta_mrad: side === 'bottom' ? th : -th, dtrans_mm: m.dtrans_mm, dlong_mm: m.dlong_mm };
  if (Number.isFinite(m.B_l) && m.B_l !== 0) c.B_l = side === 'bottom' ? -m.B_l : m.B_l;
  else if (Number.isFinite(m.theta_l_mrad) && m.theta_l_mrad !== 0) c.theta_l_mrad = side === 'bottom' ? Math.abs(m.theta_l_mrad) : -Math.abs(m.theta_l_mrad);
  if (endZone) c.endZone = endZone;
  return c;
}

/** 성분의 캐시 키. B_l, 끝단 구간까지 포함해서 값이 바뀌면 응답을 다시 계산하게 한다. */
export const compKey = (c) => JSON.stringify(c);

// ── 선 길이와 시간 ─────────────────────────────────────────
export function polyLength(points) {
  let s = 0;
  for (let i = 0; i + 1 < points.length; i++) s += Math.hypot(points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1], points[i + 1][2] - points[i][2]);
  return s;
}

/** 순수 가열 시간(초) = 길이 ÷ 이송 속도. 속도를 모르면 null. 이동·뒤집기 시간은 포함하지 않는다. */
export function lineSeconds(points, speed) {
  if (!(speed > 0)) return null;
  return polyLength(points) / speed;
}

export function formatSeconds(s) {
  if (s == null || !Number.isFinite(s)) return '-';
  if (s < 90) return `${Math.round(s)}초`;
  if (s < 3600) { const m = Math.floor(s / 60), r = Math.round(s - 60 * m); return r ? `${m}분 ${r}초` : `${m}분`; }
  const h = Math.floor(s / 3600), m = Math.round((s - 3600 * h) / 60);
  return m ? `${h}시간 ${m}분` : `${h}시간`;
}

// ── 끝단 구간 ──────────────────────────────────────────────
/** 폴리라인의 3차원 누적 길이 배열 */
export function cumulativeLength(points) {
  const a = new Float64Array(points.length);
  for (let i = 1; i < points.length; i++) a[i] = a[i - 1] + Math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1], points[i][2] - points[i - 1][2]);
  return a;
}

/**
 * 끝단 구간의 길이 범위를 돌려준다. [{from, to}, {from, to}] 는 선 길이(mm) 기준 시작 쪽과 끝 쪽 구간.
 * solver_ext.js 와 같이 끝단 길이는 선 길이의 절반을 넘지 못한다. length 가 0 이거나 없으면 null.
 */
export function endZoneSpans(points, length) {
  if (!(length > 0)) return null;
  const L = polyLength(points);
  const Le = Math.min(length, L / 2);
  return [{ from: 0, to: Le }, { from: L - Le, to: L }];
}

/** 선 길이 위치 s(mm)를 폴리라인 지수(소수)로 바꾼다. acc 는 cumulativeLength 결과. */
export function arcIndex(acc, s) {
  const n = acc.length;
  let k = 0;
  while (k < n - 2 && acc[k + 1] < s) k++;
  const span = acc[k + 1] - acc[k];
  return k + (span > 0 ? Math.min(1, Math.max(0, (s - acc[k]) / span)) : 0);
}

// ── AI 시뮬레이터 ───────────────────────────────────────────
const AI_START = '// ── 모델 파일', AI_END = '// ── 시간 격자';

/**
 * static/ai.js 소스에서 추론 부분(모델 파일 ~ 스칼라 추론)을 잘라 실행한다.
 * ai.js 를 복사하지 않으므로 모델 형식이 바뀌어도 ai.js 쪽 고침이 그대로 따라온다.
 * @param src ai.js 전체 텍스트
 * @returns { use(meta, buf), predictScalars(x) }
 */
export function buildAiBridge(src) {
  const a = src.indexOf(AI_START), b = src.indexOf(AI_END);
  if (a < 0 || b < 0 || b < a) throw new Error('ai.js 에서 추론 코드를 찾지 못했습니다. 파일 형식이 바뀌었는지 확인해 주세요.');
  const code = src.slice(a, b);
  // eslint-disable-next-line no-new-func
  const factory = new Function(`'use strict';\nlet M = null, BUF = null;\n${code}\nreturn { use(meta, buf) { M = meta; BUF = buf; }, predictScalars };`);
  return factory();
}

/**
 * 모델 입력 순서(ai.js 의 INPUT_NAMES): [두께, 속도, 목표 최고온도, r0, gap, h_water, fy, water]
 * @returns { theta_mrad, dtrans_mm, dlong_mm, B_l?, std, ms, source:'ai' }
 * 모델의 theta, dtrans, dlong 은 스트립 변위 기준이다. 정밀 3D 결과·보정표와 같은 정의라 그대로 판 해석 입력으로 쓴다.
 * 고유변형 적분(S_t, B_t, S_l)은 정밀 3D 대조용이고 여기서 쓰지 않는다.
 */
export function aiPredictComp(bridge, x) {
  const t0 = performance.now();
  const sc = bridge.predictScalars(x);
  const ms = performance.now() - t0;
  const out = {
    theta_mrad: Math.abs(sc.theta.value), dtrans_mm: sc.dtrans.value, dlong_mm: sc.dlong.value,
    std: { theta: sc.theta.std, dtrans: sc.dtrans.std, dlong: sc.dlong.std },
    ms, source: 'ai',
  };
  if (sc.B_l && Number.isFinite(sc.B_l.value)) out.B_l = sc.B_l.value; // 모델에 B_l 이 있을 때만
  return out;
}

/** 입력이 학습 범위 밖인지 본다. x[7] 이 0 이면(수냉 없음) 수냉 입력은 보지 않는다. ai.js checkWarnings 와 같은 기준이다. */
export function aiRangeWarnings(meta, x) {
  const msgs = [];
  (meta.inputs || []).forEach((inp, i) => {
    if (inp.kind !== 'num') return;
    if (x[7] === 0 && (inp.name === 'gap' || inp.name === 'h_water')) return;
    const v = x[i];
    if (v < inp.train_min - 1e-9 || v > inp.train_max + 1e-9) msgs.push(`${inp.label} ${v.toFixed(2)}은(는) 학습 범위(${inp.train_min}~${inp.train_max}) 밖입니다`);
  });
  if (x[7] === 0) msgs.push('수냉 없음 조건은 학습 표본이 적어 불확실합니다');
  return msgs;
}

/**
 * 화면에 알릴 모델 상태 문구. dev 는 정식 폴더(ai_model) 대신 개발 점검용 폴더를 쓸 때 true.
 * 개발 점검용 모델은 온도 전용 표본으로 만든 합성 데이터이거나, 적은 표본으로 시험한 것이다.
 */
export function aiModelNotes(meta, dev) {
  const notes = [];
  if (dev) notes.push('개발 점검용 임시 모델입니다. 합성 데이터이거나 표본이 아주 적습니다. 값을 판단이나 인용에 쓰지 마세요.');
  const d = meta.data || {};
  if (Number.isFinite(d.n_D) && d.n_D < 64) notes.push(`학습 표본이 ${d.n_D}건으로 적습니다. 지금은 시연 수준입니다.`);
  // 표본 수 경고는 위와 겹치면 빼고, 검증 시드 수 안내(시간 제한 등)는 화면 판단과 무관하니 싣지 않는다
  for (const w of meta.warnings || []) if (!(Number.isFinite(d.n_D) && d.n_D < 64 && w.startsWith('D_ 표본이')) && !w.includes('시드')) notes.push(w);
  return notes;
}

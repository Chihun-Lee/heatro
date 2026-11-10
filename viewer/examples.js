// 처음 열면 가상 판 1건을 계산이 끝난 상태로 보여 준다.
// 예시는 examples/ 의 식으로 만든 곡면이다. 사용자가 올린 파일은 이 브라우저 밖으로 나가지 않는다.
const viewer = window.heatroViewer;
const PREFIX = '가상예시_';
const BASE = new URL('./examples/', import.meta.url);
const select = document.getElementById('exampleSelect');
const desc = document.getElementById('exampleDesc');
const badge = document.getElementById('exampleBadge');

let items = [];
let current = null;
let switching = false;

const isExample = (d) => d.fileName.startsWith(PREFIX);
// 목록의 예시뿐 아니라 사용자가 올린 파일도 가상 자료 표시(DeviceCode DEMO, LegacyId demo-)가 있으면 가상으로 본다
const isSynthetic = (d) => isExample(d) || d.deviceCode === 'DEMO' || String(d.legacyId ?? '').startsWith('demo-');

function showBadge() {
  const d = viewer.getPrimary();
  badge.hidden = !(d && isSynthetic(d));
  if (!badge.hidden) badge.textContent = `가상 데이터: ${d.plateName || d.partName || d.fileName}. 실제 판이 아닙니다.`;
  // 목록의 각 줄에도 표시한다
  document.querySelectorAll('#datasetList .dataset-item .name').forEach((el, i) => {
    const row = viewer.datasets()[i];
    if (row && isSynthetic(row) && !el.querySelector('.tag-synth')) {
      const tag = document.createElement('span');
      tag.className = 'tag-synth';
      tag.textContent = '가상';
      el.prepend(tag);
    }
  });
}

function chooseFromUrl() {
  const wanted = new URLSearchParams(location.search).get('example');
  return items.find((x) => x.id === wanted) ?? items[0];
}

function remember(item) {
  const url = new URL(location.href);
  url.searchParams.set('example', item.id);
  history.replaceState(null, '', url);
}

async function showExample(item) {
  switching = true;
  try {
    current = item;
    select.value = item.id;
    desc.textContent = `${item.desc} 처음 편차 RMS ${item.rmsBefore} mm, 가열선 ${item.lines}줄을 넣은 뒤 ${item.rmsAfter} mm (해석값).`;
    viewer.removeWhere(isExample);
    const text = await (await fetch(new URL(item.file, BASE))).text();
    await viewer.ingest([new File([text], `${PREFIX}${item.title}.json`, { type: 'application/json' })]);
    viewer.setPrimary(viewer.datasets().length - 1);
    showBadge();
    await viewer.runDeformation();
  } finally {
    switching = false;
  }
}

window.addEventListener('heatro:reload', () => {
  showBadge();
  if (!switching && current && !viewer.datasets().length) showExample(current);
});

// 사용자가 자기 파일을 올리면 가상 예시와 섞이지 않도록 뺀다
window.addEventListener('heatro:ingested', (e) => {
  if (e.detail.some(isExample)) return;
  switching = true;
  viewer.removeWhere(isExample);
  switching = false;
  showBadge();
});

select.addEventListener('change', () => {
  const item = items.find((x) => x.id === select.value);
  if (!item) return;
  remember(item);
  showExample(item);
});

items = await (await fetch(new URL('index.json', BASE))).json();
for (const x of items) select.append(new Option(x.title, x.id));
showExample(chooseFromUrl());

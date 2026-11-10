// 프로그램 틀: 도구 화면을 탭으로 묶는다.
// 도구마다 iframe 을 하나씩 두어 요소 이름이 섞이지 않게 하고, 탭을 옮겨도 입력값과 계산 결과가 그대로 남게 한다.
// 각 도구는 처음 열 때 불러온다. 숨긴 탭은 브라우저가 그리기를 멈추므로 계산기를 덜 잡아먹는다.
'use strict';

const THEME_KEY = 'forming.theme';   // 곡면 화면이 쓰는 저장 이름과 같게 둔다
const THEMES = [['', '테마: 자동'], ['light', '테마: 밝게'], ['dark', '테마: 어둡게']];

const stage = document.getElementById('stage');
const note = document.getElementById('stageNote');
const themeBtn = document.getElementById('shellTheme');
const links = [...document.querySelectorAll('.tabs a[data-src]')];
const baseTitle = document.title;
const entries = new Map();           // 탭 이름 → { frame, ready, failed, name }
let current = null;

function readTheme() {
  try {
    const saved = localStorage.getItem(THEME_KEY) ?? '';
    return THEMES.some(([v]) => v === saved) ? saved : '';
  } catch { return ''; }
}
let theme = readTheme();

function applyThemeTo(frame) {
  const doc = frame.contentDocument, win = frame.contentWindow;
  if (!doc || !win) return;
  // 곡면 화면은 3D 장면 색까지 다시 칠해야 하므로 자기 버튼을 거쳐 바꾼다.
  const own = doc.getElementById('themeBtn');
  if (own) {
    const want = THEMES.find(([v]) => v === theme)[1];
    for (let i = 0; i < THEMES.length && own.textContent !== want; i++) own.click();
  } else if (theme) doc.documentElement.dataset.theme = theme;
  else delete doc.documentElement.dataset.theme;
  // 뷰어의 3D 배경은 고정색이라 틀에서 앱의 값으로 맞춘다.
  const back = win.heatroViewer?.scene?.background;
  if (back) {
    const view = win.getComputedStyle(doc.documentElement).getPropertyValue('--view').trim();
    if (view) back.set(view);
  }
  win.dispatchEvent(new Event('resize'));   // 캔버스를 새 색과 크기로 다시 그리게 한다
}

function setTheme(value) {
  theme = value;
  if (theme) document.documentElement.dataset.theme = theme;
  else delete document.documentElement.dataset.theme;
  themeBtn.textContent = THEMES.find(([v]) => v === theme)[1];
  try {
    if (theme) localStorage.setItem(THEME_KEY, theme); else localStorage.removeItem(THEME_KEY);
  } catch { /* 저장 못 해도 화면은 쓴다 */ }
  for (const { frame, ready } of entries.values()) if (ready) applyThemeTo(frame);
}

async function prepare(frame, link) {
  const doc = frame.contentDocument;
  doc.documentElement.classList.add('in-shell');
  if (link.dataset.style && !doc.querySelector('link[data-shell]')) {
    const css = doc.createElement('link');
    css.rel = 'stylesheet';
    css.href = new URL(link.dataset.style, location.href).href;
    css.dataset.shell = '1';
    await new Promise((done) => { css.onload = css.onerror = done; doc.head.append(css); });
  }
  applyThemeTo(frame);
}

function syncNote() {
  const entry = entries.get(current);
  note.hidden = !entry || entry.ready;
  if (entry && !entry.ready) note.textContent = entry.failed ? `${entry.name} 화면 파일을 찾지 못했습니다.` : `${entry.name} 화면을 불러오는 중입니다.`;
}

function createEntry(key, link) {
  const frame = document.createElement('iframe');
  frame.title = link.textContent;
  frame.className = 'is-loading';
  const entry = { frame, ready: false, failed: false, name: link.textContent };
  entries.set(key, entry);
  frame.addEventListener('load', async () => {
    if (entry.failed) return;
    try { await prepare(frame, link); } catch (err) { console.error('화면 준비 실패', err); }
    entry.ready = true;
    frame.classList.remove('is-loading');
    syncNote();
    frame.contentWindow?.dispatchEvent(new Event('resize'));
  });
  stage.append(frame);
  frame.src = link.dataset.src;
  fetch(link.dataset.src, { method: 'HEAD', cache: 'no-store' }).then((res) => {
    if (res.ok) return;
    entry.failed = true;
    entry.ready = false;
    frame.hidden = true;
    syncNote();
  }).catch(() => { /* 확인을 못 해도 화면은 그대로 연다 */ });
  return entry;
}

function openTab(name) {
  const link = links.find((a) => a.hash === `#${name}`) || links[0];
  const key = link.hash.slice(1);
  if (key === current) return;
  current = key;
  for (const a of links) {
    if (a === link) a.setAttribute('aria-current', 'page');
    else a.removeAttribute('aria-current');
  }
  document.title = `${link.textContent} - ${baseTitle}`;
  const entry = entries.get(key) || createEntry(key, link);
  for (const [k, e] of entries) e.frame.hidden = k !== key || e.failed;
  syncNote();
  if (entry.ready) entry.frame.contentWindow?.dispatchEvent(new Event('resize'));
}

themeBtn.addEventListener('click', () => {
  const i = THEMES.findIndex(([v]) => v === theme);
  setTheme(THEMES[(i + 1) % THEMES.length][0]);
});
matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => {
  for (const { frame, ready } of entries.values()) if (ready) applyThemeTo(frame);
});
window.addEventListener('hashchange', () => openTab(location.hash.slice(1)));

themeBtn.textContent = THEMES.find(([v]) => v === theme)[1];
openTab(location.hash.slice(1));

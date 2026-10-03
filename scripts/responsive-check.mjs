// Checks the simulator and the flowchart page at phone, tablet and desktop
// sizes in headless Chrome: nothing scrolls sideways, the floating panels
// never overlap, phones get the settings sheet and finger-sized buttons, and
// the flowcharts open readable on a narrow screen and work with fingers
// (drag, tap, pinch), with a step's details never hiding the step.
//   node scripts/responsive-check.mjs [screenshot-dir]
import { tmpdir } from "node:os";
import { join } from "node:path";
import { openBrowser } from "./lib/browser.mjs";

const OUT = process.argv[2] ?? join(tmpdir(), "lcm-responsive");
const { base, send, js, sleep, shot, until, check, finish, consoleErrors } = await openBrowser(OUT);
await send("Page.enable");
await send("Runtime.enable");

const SIZES = [
  ["phone", 375, 812, 2, true],
  ["phone-sideways", 812, 375, 2, true],
  ["tablet", 768, 1024, 2, true],
  ["laptop", 1280, 720, 1, false],
  ["desktop", 1920, 1080, 1, false],
];
const PANELS = ["toolbar", "settings", "hud", "legend", "tools", "inspector"];

async function size(w, h, dpr, touch) {
  await send("Emulation.setDeviceMetricsOverride", { width: w, height: h, deviceScaleFactor: dpr, mobile: touch });
  await send("Emulation.setTouchEmulationEnabled", { enabled: touch });
}
const layout = () => js(`(() => {
  const shown = (e) => e && !e.hidden && getComputedStyle(e).display !== 'none' && getComputedStyle(e).visibility !== 'hidden';
  const rects = ${JSON.stringify(PANELS)}.map((id) => [id, document.getElementById(id)]).filter(([, e]) => shown(e)).map(([id, e]) => [id, e.getBoundingClientRect()]);
  const overlaps = [];
  for (let a = 0; a < rects.length; a++) for (let b = a + 1; b < rects.length; b++) {
    const [n1, r1] = rects[a], [n2, r2] = rects[b];
    if (r1.left < r2.right - 1 && r2.left < r1.right - 1 && r1.top < r2.bottom - 1 && r2.top < r1.bottom - 1) overlaps.push(n1 + "/" + n2);
  }
  const offscreen = rects.filter(([, r]) => r.left < -1 || r.right > innerWidth + 1 || r.top < -1 || r.bottom > innerHeight + 1).map(([id]) => id);
  return { sideways: document.documentElement.scrollWidth > innerWidth, overlaps, offscreen };
})()`);

for (const [name, w, h, dpr, touch] of SIZES) {
  await size(w, h, dpr, touch);
  await send("Page.navigate", { url: `${base}/` });
  await until("!!window.lcm", 10000);
  await js("window.lcm.ready");
  await sleep(300);
  const l = await layout();
  check(`simulator, ${name}: no sideways scrolling, no overlapping or cut-off panels`, !l.sideways && !l.overlaps.length && !l.offscreen.length, JSON.stringify(l));
  await shot(`sim-${name}`);
}

// Phone specifics.
await size(375, 812, 2, true);
await send("Page.navigate", { url: `${base}/` });
await until("!!window.lcm", 10000);
await js("window.lcm.ready");
await sleep(300);
check("phone: settings start closed, so the swarm is visible", await js("document.getElementById('settings').hidden"));
check("phone: toolbar buttons are finger-sized (40 px)", await js("[...document.querySelectorAll('#toolbar .btn, #tools .btn')].filter((b) => b.offsetParent).every((b) => b.getBoundingClientRect().height >= 39)"));
await js("document.getElementById('toggle_settings').click()");
await sleep(200);
const sheet = await js(`(() => { const r = document.getElementById('settings').getBoundingClientRect(); return { left: r.left, right: r.right, bottom: r.bottom, w: innerWidth, h: innerHeight }; })()`);
check("phone: settings open as a full-width sheet from the bottom", Math.abs(sheet.left) < 1 && Math.abs(sheet.right - sheet.w) < 1 && Math.abs(sheet.bottom - sheet.h) < 1, JSON.stringify(sheet));
await shot("sim-phone-settings");
await js("document.getElementById('close_settings').click()");
check("phone: the sheet's close button closes it", await js("document.getElementById('settings').hidden"));
const view = await js(`(() => { const r = lcm.renderer, f = r.frame; let out = 0;
  for (let i = 0; i < f.flags.length; i++) { const [x, y] = r.toScreen(f.xy[2*i], f.xy[2*i+1]); if (x < 0 || y < 0 || x > r.width || y > r.height) out++; } return out; })()`);
check("phone: the whole swarm fits the screen", view === 0, `${view} robots off screen`);

// Flowchart page.
for (const [name, w, h, dpr, touch] of SIZES) {
  await size(w, h, dpr, touch);
  await send("Page.navigate", { url: `${base}/web/docs.html` });
  await until("!!window.docs", 10000);
  await sleep(300);
  check(`flowcharts, ${name}: no sideways scrolling`, !(await js("document.documentElement.scrollWidth > innerWidth")));
}
await size(375, 812, 2, true);
await send("Page.navigate", { url: `${base}/web/docs.html#look-compute-move` });
await until("!!window.docs", 10000);
await sleep(300);
const zoom = parseInt(await js("document.querySelector('.chart:not([hidden]) .zoom-level').textContent"), 10);
check("phone: a chart opens at a readable zoom (60 % or more)", zoom >= 60, `${zoom}%`);
check("phone: only one 'Open the simulator' button", await js("[...document.querySelectorAll('a')].filter((a) => a.offsetParent && /Open the simulator/.test(a.textContent)).length === 1"));
await js("document.querySelector('.chart:not([hidden]) .tour-start').click()");
await sleep(300);
const detail = await js(`(() => { const r = document.getElementById('detail').getBoundingClientRect(); return { top: r.top, bottom: r.bottom, h: innerHeight }; })()`);
check("phone: a step's details open as a sheet in view", detail.top > 0 && detail.top < detail.h * 0.6 && Math.abs(detail.bottom - detail.h) < 2, JSON.stringify(detail));
await shot("docs-phone-detail");

// Each scenario below starts from a fresh page: the same page with a new "#"
// would only switch charts and keep the earlier state.
const fresh = async (url) => { await send("Page.navigate", { url: "about:blank" }); await sleep(100); await send("Page.navigate", { url }); await until("!!window.docs", 10000); await sleep(300); };

// Flowchart page with fingers: drag from a box, tap right after, pinch.
await size(390, 844, 3, true);
await send("Emulation.setTouchEmulationEnabled", { enabled: true, maxTouchPoints: 5 });
await fresh(`${base}/web/docs.html#look-compute-move`);
const touch = (type, pts) => send("Input.dispatchTouchEvent", { type, touchPoints: pts.map(([x, y], id) => ({ x, y, id })) });
const box = () => js(`(() => { const b = docs.views[docs.chart].box; return { y: b.y, w: b.w }; })()`);
const onScreenNode = () => js(`(() => { const s = document.querySelector('.chart:not([hidden]) .stage').getBoundingClientRect();
  for (const g of document.querySelectorAll('.chart:not([hidden]) .node')) { const r = g.getBoundingClientRect(), x = r.x + r.width / 2, y = r.y + r.height / 2;
    if (x > s.left + 10 && x < s.right - 10 && y > s.top + 140 && y < s.bottom - 80) return [x, y, g.dataset.id]; } return null; })()`);
await js("document.querySelector('.chart:not([hidden]) [data-zoom=\"in\"]').click()");
await js("document.querySelector('.chart:not([hidden]) [data-zoom=\"in\"]').click()");
await sleep(100);
const from = await onScreenNode();
const before = await box();
await touch("touchStart", [from]);
for (let i = 1; i <= 8; i++) await touch("touchMove", [[from[0], from[1] - i * 15]]);
await touch("touchEnd", []);
await sleep(100);
const moved = ((await box()).y - before.y) * (await js("docs.views['look-compute-move'].scale()"));
check("phone: one finger drags the chart, even starting on a box", Math.abs(moved - 120) < 3 && !(await js("document.body.classList.contains('has-detail')")), `${moved.toFixed(1)} px of 120`);
const target = await onScreenNode();
await touch("touchStart", [target]);
await touch("touchEnd", []);
await sleep(300);
check("phone: a tap right after a drag opens the box", await js(`document.querySelector('.chart:not([hidden]) .node.selected')?.dataset.id === ${JSON.stringify(target[2])}`));
const tapped = await js(`(() => { const n = document.querySelector('.chart:not([hidden]) .node.selected').getBoundingClientRect(), s = document.querySelector('.chart:not([hidden]) .stage').getBoundingClientRect(), d = document.getElementById('detail').getBoundingClientRect(); return { node: [n.top, n.bottom], stage: [s.top, s.bottom], detail: d.top }; })()`);
check("phone: the tapped box stays in view above its details", tapped.node[0] >= tapped.stage[0] && tapped.node[1] <= tapped.stage[1] && tapped.node[1] <= tapped.detail, JSON.stringify(tapped));
const w0 = (await box()).w;
const c = await js(`(() => { const r = document.querySelector('.chart:not([hidden]) .stage').getBoundingClientRect(); return [r.x + r.width / 2, r.y + r.height / 2]; })()`);
await touch("touchStart", [[c[0] - 30, c[1]], [c[0] + 30, c[1]]]);
for (let i = 1; i <= 6; i++) await touch("touchMove", [[c[0] - 30 - i * 12, c[1]], [c[0] + 30 + i * 12, c[1]]]);
await touch("touchEnd", []);
await sleep(100);
check("phone: two fingers pinch to zoom", (await box()).w < w0 * 0.6, `visible width ${w0.toFixed(0)} -> ${(await box()).w.toFixed(0)}`);
// While a finger moves, the drawn picture is slid on the GPU; it must line up
// exactly with the sharp redraw when the finger lifts, and never show blank.
await fresh(`${base}/web/docs.html#event-loop`);
const someNode = () => js(`(() => { const r = document.querySelector('.chart:not([hidden]) .node').getBoundingClientRect(); return [r.x, r.y, r.width]; })()`);
const mid = await js(`(() => { const r = document.querySelector('.chart:not([hidden]) .stage').getBoundingClientRect(); return [r.x + r.width / 2, r.y + r.height / 2]; })()`);
await touch("touchStart", [mid]);
for (let i = 1; i <= 6; i++) { await touch("touchMove", [[mid[0] - i * 10, mid[1]]]); await sleep(20); }
await sleep(50);
const sliding = await someNode();
const slid = (await js("document.querySelector('.chart:not([hidden]) svg').style.transform")) !== "";
await touch("touchEnd", []);
await sleep(100);
const settled = await someNode();
check("phone: a drag slides the drawn chart and lands without a jump", slid && sliding.every((v, i) => Math.abs(v - settled[i]) < 1), `${sliding.map((v) => v.toFixed(1))} -> ${settled.map((v) => v.toFixed(1))}`);
await touch("touchStart", [mid]);
for (let i = 1; i <= 30; i++) { await touch("touchMove", [[mid[0] + (i % 2 ? 1 : -1), mid[1] - i * 12]]); await sleep(18); }
await sleep(50);
check("phone: a long drag never uncovers blank space", await js(`(() => { const s = document.querySelector('.chart:not([hidden]) .stage').getBoundingClientRect(), g = document.querySelector('.chart:not([hidden]) svg').getBoundingClientRect(); return g.left <= s.left + 1 && g.top <= s.top + 1 && g.right >= s.right - 1 && g.bottom >= s.bottom - 1; })()`));
await touch("touchEnd", []);
await sleep(100);
await fresh(`${base}/web/docs.html#look-compute-move`);
await js("document.querySelector('.chart:not([hidden]) .about-btn').click()");
check("phone: 'About this chart' shows the introduction and notes in place of the chart", await js("(() => { const s = document.querySelector('.chart:not([hidden])'); return getComputedStyle(s.querySelector('.blurb')).display !== 'none' && getComputedStyle(s.querySelector('.notes')).display !== 'none' && getComputedStyle(s.querySelector('.stage-wrap')).display === 'none'; })()"));
await js("document.querySelector('.chart:not([hidden]) .about-btn').click()");
await sleep(100);
check("phone: 'Back to the chart' brings the chart back", await js("(() => { const r = document.querySelector('.chart:not([hidden]) .stage').getBoundingClientRect(); return r.height > 200; })()"));

// A phone held sideways: the details open beside the chart.
await size(844, 390, 3, true);
await fresh(`${base}/web/docs.html#look-compute-move/comp`);
await sleep(200);
const side = await js(`(() => { const s = document.querySelector('.chart:not([hidden]) .stage').getBoundingClientRect(), d = document.getElementById('detail').getBoundingClientRect(), n = document.querySelector('.chart:not([hidden]) .node.selected').getBoundingClientRect(); return { stageRight: s.right, detailLeft: d.left, nodeIn: n.top >= s.top && n.bottom <= s.bottom && n.right <= s.right }; })()`);
check("sideways: details open beside the chart, the step still in view", side.detailLeft >= side.stageRight - 1 && side.nodeIn, JSON.stringify(side));
await shot("docs-sideways-detail");

check("no errors in the console", consoleErrors.length === 0, consoleErrors.join(" / "));
await finish();

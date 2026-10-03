// Checks the simulator and the flowchart page at phone, tablet and desktop
// sizes in headless Chrome: nothing scrolls sideways, the floating panels
// never overlap, phones get the settings sheet and finger-sized buttons, and
// the flowcharts open readable on a narrow screen.
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

check("no errors in the console", consoleErrors.length === 0, consoleErrors.join(" / "));
await finish();

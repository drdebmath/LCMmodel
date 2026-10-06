// The SqGathering page (web/sqgathering.html) in headless Chrome: every
// preset plays to the same end as `lcm seq`, each multiplicity case shows the
// right observation, active robot and rule, a budget-limited run says
// INCOMPLETE, and the page fits phone, tablet and desktop without sideways
// scrolling.
//   node scripts/sqgathering-check.mjs [screenshot-dir]
//   (CHROME=/path/to/chrome on Windows)
import { readFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { fileURLToPath } from "node:url";
import { openBrowser } from "./lib/browser.mjs";
import { initSync, sq_gathering_run } from "../web/pkg/lcm_wasm.js";

const root = fileURLToPath(new URL("..", import.meta.url));
initSync({ module: readFileSync(join(root, "web/pkg/lcm_wasm_bg.wasm")) });
const OUT = process.argv[2] ?? join(tmpdir(), "lcm-sqgathering");
const { base, send, js, sleep, shot, until, check, finish, consoleErrors } = await openBrowser(OUT);
await send("Page.enable");
await send("Runtime.enable");

const PRESETS = ["k0_no_multiplicity", "k1_one_multiplicity", "kmany_two_multiplicities",
  "nonrigid_collision", "kmany_three_multiplicities_random", "budget_incomplete"];

async function open(hash) {
  await send("Page.navigate", { url: "about:blank" });
  await sleep(50);
  await send("Page.navigate", { url: `${base}/web/sqgathering.html#${hash}` });
  await sleep(200);
  return until("document.getElementById('preset_blurb').textContent.length > 0 && document.getElementById('view').childElementCount > 0", 10000);
}
const panel = () => js(`(() => {
  const $ = (id) => document.getElementById(id);
  return { round: +$("s_round").textContent, robot: $("a_robot").textContent, kappa: $("a_kappa").textContent,
    b: $("a_b").textContent, rule: $("a_rule").querySelector(".rule-key")?.textContent ?? "",
    obs: [...$("obs").children].map((li) => li.textContent), status: $("s_status").className,
    note: $("s_status").textContent, dest: $("algo").querySelector("li.dest")?.dataset.line ?? null };
})()`);
const playToEnd = () => js(`(() => { const b = document.getElementById("step"); let n = 0; while (!b.disabled && n < 100000) { b.click(); n++; } return n; })()`);

await send("Emulation.setDeviceMetricsOverride", { width: 1280, height: 900, deviceScaleFactor: 1, mobile: false });

// Every preset ends where the core says it ends.
for (const name of PRESETS) {
  check(`${name}: page loads`, await open(name));
  const expected = JSON.parse(sq_gathering_run(readFileSync(join(root, "fixtures/sqgathering", `${name}.json`), "utf8"), false)).report;
  await playToEnd();
  const p = await panel();
  const rows = await js("document.getElementById('trace').children.length");
  check(`${name}: ${expected.rounds} rounds, ${expected.status}, like the core`,
    p.round === expected.rounds && rows === expected.rounds && p.status.includes(expected.status), JSON.stringify({ round: p.round, rows, status: p.status }));
}
check("budget-limited run is labelled INCOMPLETE", (await panel()).note.startsWith("INCOMPLETE (budget-limited)"));

// The three multiplicity cases, round by round (hand traces A, B, C).
await open("k0_no_multiplicity@1");
let p = await panel();
check("κ = 0: r0 sees four single points and moves to the closest (line 16)",
  p.robot === "r0" && p.kappa === "0" && p.rule === "k0-move-to-closest" && p.dest === "16"
    && p.obs.length === 4 && p.obs.every((o) => o.startsWith("single")), JSON.stringify(p));
await open("k1_one_multiplicity@2");
p = await panel();
check("κ = 1: r0 on the multiplicity stays (line 3)",
  p.robot === "r0" && p.kappa === "1" && p.b.startsWith("1") && p.rule === "k1-hold-multiplicity" && p.dest === "3", JSON.stringify(p));
await js("document.getElementById('back').click()");
p = await panel();
check("κ = 1: r2 (single) joins the multiplicity (line 7)",
  p.robot === "r2" && p.rule === "k1-join-multiplicity" && p.dest === "7" && p.obs[0].startsWith("multiplicity"), JSON.stringify(p));
await open("kmany_two_multiplicities@2");
p = await panel();
check("κ > 1: r0 on a multiplicity leaves for the midpoint (line 13)",
  p.robot === "r0" && p.kappa === "2" && p.rule === "kmany-leave-multiplicity" && p.dest === "13", JSON.stringify(p));
await js("document.getElementById('back').click()");
p = await panel();
check("κ > 1: r4 (single) waits (line 3)", p.robot === "r4" && p.rule === "kmany-wait" && p.dest === "3", JSON.stringify(p));
check("weak detection: counts are hidden until asked for", !(await js("document.getElementById('obs').textContent.includes('robots')")));
await js("document.getElementById('counts').click()");
check("true counts can be shown, marked as hidden from robots", await js("document.getElementById('obs').textContent.includes('2 robots')"));
await shot("kmany-round1");

// The main simulator: "Sequential ›" in the algorithm menu runs SqGathering.
async function openSim(w, h, touch) {
  await send("Emulation.setDeviceMetricsOverride", { width: w, height: h, deviceScaleFactor: touch ? 2 : 1, mobile: touch });
  await send("Page.navigate", { url: "about:blank" });
  await sleep(50);
  await send("Page.navigate", { url: `${base}/` });
  await until("!!window.lcm", 10000);
  await js("window.lcm.ready");
  await sleep(200);
}
const shown = (id) => js(`!document.getElementById(${JSON.stringify(id)}).hidden`);
await openSim(1280, 800, false);
await js("document.getElementById('algo_button').click()");
check("simulator: the algorithm button opens the menu", await shown("algo_menu"));
check("simulator: the menu lists the asynchronous algorithms", await js(
  "[...document.querySelectorAll('#algo_async [data-key]')].map((b) => b.dataset.key).join() === 'Gathering,SEC'"));
check("simulator: 'Sequential' has a › arrow and its flyout starts closed",
  (await js("!!document.querySelector('#algo_seq_toggle [data-icon=chevron] svg')")) && !(await shown("algo_sub")));
await js("document.getElementById('algo_seq_toggle').click()");
check("simulator: clicking 'Sequential' opens the flyout with SqGathering and its options",
  (await shown("algo_sub")) && (await js("!!document.querySelector('#algo_seq [data-key=SqGathering]') && document.querySelectorAll('[data-seq]').length === 5 && !!document.getElementById('seq_movement') && !!document.getElementById('seq_delta')")));
const flyout = await js(`(() => { const m = document.getElementById("algo_menu").getBoundingClientRect(), f = document.getElementById("algo_sub").getBoundingClientRect(); return { right: f.left >= m.right - 3, onScreen: f.right <= innerWidth }; })()`);
check("simulator: the flyout opens to the right of the menu, on screen", flyout.right && flyout.onScreen, JSON.stringify(flyout));
await shot("sim-menu-sequential");
await js("document.querySelector('#algo_seq [data-key=SqGathering]').click()");
await sleep(300);
check("simulator: picking SqGathering closes the menu and labels the button",
  !(await shown("algo_menu")) && (await js("document.getElementById('algo_label').textContent")) === "Sequential › SqGathering");
check("simulator: the HUD counts rounds and gathered robots", await js(
  "document.getElementById('hud_time_label').textContent === 'Round' && document.getElementById('hud_term_label').textContent === 'Gathered'"));
await js("document.getElementById('playback').value = 'Infinity'; document.getElementById('playback').dispatchEvent(new Event('change'))");
await js("document.getElementById('play').click()");
const ended = await until("window.lcm.state === 'ended'", 60000);
const end = await js(`({ status: document.getElementById("hud_status").textContent, term: document.getElementById("hud_term").textContent, stats: window.lcm.stats })`);
check("simulator: SqGathering runs on the canvas until every robot is gathered",
  ended && end.status === "Ended · gathered" && end.stats.terminated === end.stats.robots, JSON.stringify({ status: end.status, term: end.term, rounds: end.stats.events }));
await shot("sim-sqgathering-gathered");
await js("document.getElementById('algo_button').click()");
check("simulator: reopening the menu shows the flyout with SqGathering ticked",
  (await shown("algo_sub")) && (await js("document.querySelector('#algo_seq [data-key=SqGathering]').getAttribute('aria-checked')")) === "true");
await js(`(() => { const s = document.querySelector('[data-seq="schedule"]'); s.value = "round-robin"; s.dispatchEvent(new Event("change")); })()`);
check("simulator: a sequential option is remembered", await js("JSON.parse(localStorage.getItem('lcm-seq-options')).schedule === 'round-robin'"));
await js(`(() => { const s = document.querySelector('[data-seq="schedule"]'); s.value = "shuffled"; s.dispatchEvent(new Event("change")); })()`);
await js("document.querySelector('#algo_async [data-key=Gathering]').click()");
await sleep(200);
check("simulator: picking from the menu after a run starts the new algorithm at once", await js("window.lcm.state === 'ready'"));
check("simulator: back to Center of Gravity restores Time / Terminated", await js(
  "document.getElementById('algo_label').textContent === 'Center of Gravity' && document.getElementById('hud_time_label').textContent === 'Time'"));
// Real mouse, while a run is playing: the complaint that started this.
await openSim(1500, 800, false);
const centre = (sel) => js(`(() => { const r = document.querySelector(${JSON.stringify(sel)}).getBoundingClientRect(); return [r.left + r.width / 2, r.top + r.height / 2]; })()`);
const mouse = (type, x, y) => send("Input.dispatchMouseEvent", { type, x, y, button: "left", buttons: type === "mousePressed" ? 1 : 0, clickCount: 1 });
const realClick = async (sel) => { const [x, y] = await centre(sel); for (const t of ["mouseMoved", "mousePressed", "mouseReleased"]) await mouse(t, x, y); await sleep(150); };
await realClick("#play");
check("mouse: the default run is playing", await until("window.lcm.state === 'running'", 3000));
await realClick("#algo_button");
const [tx, ty] = await centre("#algo_seq_toggle");
await mouse("mouseMoved", tx, ty); await sleep(150);
check("mouse: pointing at 'Sequential' opens the flyout", await shown("algo_sub"));
const [ix, iy] = await centre("#algo_seq [data-key=SqGathering]");
for (let k = 1; k <= 8; k++) { await mouse("mouseMoved", tx + (ix - tx) * k / 8, ty + (iy - ty) * k / 8); await sleep(15); }
check("mouse: the flyout stays open on the way to SqGathering (no gap)", await shown("algo_sub"));
const gap = await js(`(() => { const m = document.getElementById("algo_menu").getBoundingClientRect(), f = document.getElementById("algo_sub").getBoundingClientRect(); return f.left - m.right; })()`);
check("mouse: the flyout touches the menu", gap <= 0, `${gap}px`);
await realClick("#algo_seq [data-key=SqGathering]");
check("mouse: clicking SqGathering while playing switches at once",
  (await js("document.getElementById('algo_label').textContent")) === "Sequential › SqGathering"
    && (await js("window.lcm.state")) === "ready" && (await js("document.getElementById('hud_time_label').textContent")) === "Round");
// Movement and δ from the flyout.
await realClick("#algo_button");
check("flyout: δ and adversary stops are off while movement is rigid",
  await js("document.getElementById('seq_delta').disabled && document.querySelector('[data-seq=stop]').disabled"));
await js(`(() => { const s = document.getElementById("seq_movement"); s.value = "non-rigid"; s.dispatchEvent(new Event("change")); })()`);
check("flyout: choosing Non-rigid enables δ and adversary stops, and sets the setting",
  await js("!document.getElementById('seq_delta').disabled && !document.querySelector('[data-seq=stop]').disabled && window.lcm.settings.values.rigid_movement === false"));
await js(`(() => { const d = document.getElementById("seq_delta"); d.value = "4"; d.dispatchEvent(new Event("change")); })()`);
check("flyout: δ sets Speed", await js("window.lcm.settings.values.robot_speeds === 4"));
// Every κ start gathers on the main canvas.
await js("document.getElementById('playback').value = 'Infinity'; document.getElementById('playback').dispatchEvent(new Event('change'))");
for (const k of [0, 1, 2, 3]) {
  await js(`(() => { document.getElementById("algo_button").click(); const s = document.querySelector('[data-seq="multiplicities"]'); s.value = "${k}"; s.dispatchEvent(new Event("change")); document.body.click(); })()`);
  await sleep(300);
  const startRings = await js("(() => { const c = window.lcm.renderer.frame.circle; let r = 0; for (let i = 2; i < c.length; i += 3) if (c[i] < 0) r++; return r; })()");
  const start = await js("(() => { const f = window.lcm.renderer.frame; const seen = new Map(); for (let i = 0; i < f.robots; i++) { const key = f.xy[2 * i] + ',' + f.xy[2 * i + 1]; seen.set(key, (seen.get(key) ?? 0) + 1); } return [...seen.values()].filter((c) => c > 1).length; })()");
  await js("document.getElementById('play').click()");
  const done = await until("window.lcm.state === 'ended'", 60000);
  const status = await js("document.getElementById('hud_status').textContent");
  const rings = await js("(() => { const c = window.lcm.renderer.frame.circle; let r = 0; for (let i = 2; i < c.length; i += 3) if (c[i] < 0) r++; return r; })()");
  check(`κ = ${k}: the start has ${k} multiplicity point(s), each drawn with a ring, and the swarm gathers (non-rigid, δ = 4)`,
    start === k && done && status === "Ended · gathered", JSON.stringify({ start, status }));
  if (k > 0) check(`κ = ${k}: one ring per multiplicity at the start`, startRings === k, String(startRings));
  await js("document.getElementById('reset').click()");
  await sleep(200);
}
// Manual κ and size.
await js(`(() => { document.getElementById("algo_button").click();
  const k = document.querySelector('[data-seq="multiplicities"]'); k.value = "6"; k.dispatchEvent(new Event("change"));
  const z = document.querySelector('[data-seq="multiplicity_size"]'); z.value = "5"; z.dispatchEvent(new Event("change")); document.body.click(); })()`);
await sleep(300);
const manual = await js("(() => { const f = window.lcm.renderer.frame; const seen = new Map(); for (let i = 0; i < f.robots; i++) { const key = f.xy[2 * i] + ',' + f.xy[2 * i + 1]; seen.set(key, (seen.get(key) ?? 0) + 1); } return [...seen.values()].filter((c) => c > 1).join(); })()");
check("manual κ = 6 with 5 robots on each", manual === "5,5,5,5,5,5", manual);
await js(`(() => { document.getElementById("algo_button").click(); const z = document.querySelector('[data-seq="multiplicity_size"]'); z.value = "400"; z.dispatchEvent(new Event("change")); document.body.click(); })()`);
await sleep(300);
check("too many robots asked for: a clear message", await js("!document.getElementById('toast').hidden && /need 2400 robots/.test(document.getElementById('toast').textContent)"),
  await js("document.getElementById('toast').textContent"));
await js(`(() => { document.getElementById("algo_button").click(); const z = document.querySelector('[data-seq="multiplicity_size"]'); z.value = ""; z.dispatchEvent(new Event("change")); document.body.click(); })()`);
await js(`(() => { const s = document.getElementById("seq_movement"); s.value = "rigid"; s.dispatchEvent(new Event("change")); const m = document.querySelector('[data-seq="multiplicities"]'); m.value = "0"; m.dispatchEvent(new Event("change")); window.lcm.settings.set("robot_speeds", 1, false); })()`);
await shot("sim-menu-final");

await openSim(375, 812, true);
await js("document.getElementById('algo_button').click(); document.getElementById('algo_seq_toggle').click()");
const phone = await js(`(() => { const f = document.getElementById("algo_sub").getBoundingClientRect(); return { inside: f.left >= 0 && f.right <= innerWidth, sideways: document.documentElement.scrollWidth > innerWidth }; })()`);
check("simulator, phone: the Sequential options open inline, on screen", phone.inside && !phone.sideways, JSON.stringify(phone));
await shot("sim-menu-phone");

// Layout.
for (const [name, w, h, dpr, touch] of [["phone", 375, 812, 2, true], ["tablet", 768, 1024, 2, true], ["desktop", 1440, 900, 1, false]]) {
  await send("Emulation.setDeviceMetricsOverride", { width: w, height: h, deviceScaleFactor: dpr, mobile: touch });
  await open("nonrigid_collision@3");
  await sleep(150);
  const sideways = await js("document.documentElement.scrollWidth > innerWidth");
  check(`${name}: no sideways scrolling`, !sideways, String(await js("document.documentElement.scrollWidth")));
  check(`${name}: page is not scrolled by playback`, (await js("scrollY")) === 0);
  await shot(`sqgathering-${name}`);
}

check("no console errors", consoleErrors.length === 0, consoleErrors.join(" | "));
await finish();

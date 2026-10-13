// SqGathering (Algorithm 8) in headless Chrome, on both pages:
//  - web/sqgathering.html: every preset plays to the same end as the core's
//    trace (`sq_gathering_trace`, the same as `lcm seq`); each multiplicity
//    case shows the right observation, active robot and rule; a turn-limited
//    run says INCOMPLETE; the page fits phone, tablet and desktop.
//  - the simulator: SqGathering in the Algorithm menu switches the scheduler
//    to Sequential, runs to the end, rings stacked points, refuses limited
//    visibility, and a new algorithm picked mid-run starts at once.
//   node scripts/sqgathering-check.mjs [screenshot-dir]   (CHROME=… on Windows)
import { readFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { fileURLToPath } from "node:url";
import { openBrowser } from "./lib/browser.mjs";
import { initSync, sq_gathering_trace } from "../web/pkg/lcm_wasm.js";

const root = fileURLToPath(new URL("..", import.meta.url));
initSync({ module: readFileSync(join(root, "web/pkg/lcm_wasm_bg.wasm")) });
const OUT = process.argv[2] ?? join(tmpdir(), "lcm-sqgathering");
const { base, send, js, sleep, shot, until, check, finish, consoleErrors } = await openBrowser(OUT);
await send("Page.enable");
await send("Runtime.enable");

const PRESETS = ["k0_no_multiplicity", "k1_one_multiplicity", "kmany_two_multiplicities",
  "nonrigid_collision", "kmany_three_multiplicities_random", "budget_incomplete"];

async function navigate(url) {
  await send("Page.navigate", { url: "about:blank" });
  await sleep(50);
  await send("Page.navigate", { url });
}
async function open(hash) {
  await navigate(`${base}/web/sqgathering.html#${hash}`);
  await sleep(200);
  return until("document.getElementById('preset_blurb').textContent.length > 0 && document.getElementById('view').childElementCount > 0", 10000);
}
const panel = () => js(`(() => {
  const $ = (id) => document.getElementById(id);
  return { turn: +$("s_round").textContent, robot: $("a_robot").textContent, kappa: $("a_kappa").textContent,
    b: $("a_b").textContent, rule: $("a_rule").querySelector(".rule-key")?.textContent ?? "",
    obs: [...$("obs").children].map((li) => li.textContent), status: $("s_status").className,
    note: $("s_status").textContent, dest: $("algo").querySelector("li.dest")?.dataset.line ?? null };
})()`);
const playToEnd = () => js(`(() => { const b = document.getElementById("step"); let n = 0; while (!b.disabled && n < 100000) { b.click(); n++; } return n; })()`);

// ── the step-by-step page ────────────────────────────────────────────────
await send("Emulation.setDeviceMetricsOverride", { width: 1280, height: 900, deviceScaleFactor: 1, mobile: false });
for (const name of PRESETS) {
  check(`${name}: page loads`, await open(name));
  const expected = JSON.parse(sq_gathering_trace(readFileSync(join(root, "fixtures/sqgathering", `${name}.json`), "utf8"), false)).report;
  await playToEnd();
  const p = await panel();
  const rows = await js("document.getElementById('trace').children.length");
  const wanted = expected.status === "gathered" ? "gathered" : "incomplete";
  check(`${name}: ${expected.turns} turns, ${expected.status}, like the core`,
    p.turn === expected.turns && rows === expected.turns && p.status.includes(wanted), JSON.stringify({ turn: p.turn, rows, status: p.status }));
}
check("turn-limited run is labelled INCOMPLETE", (await panel()).note.startsWith("INCOMPLETE (budget-limited)"));

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
await shot("step-page-kmany");

// Settings form → a core SimConfig.
await open("k1_one_multiplicity");
await js(`(() => { const $ = (id) => document.getElementById(id);
  $("f_positions").value = "0,0; 0,0; 5,5; -3,7; 9,-2"; $("f_movement").value = "rigid"; $("f_schedule").value = "random";
  $("f_movement").dispatchEvent(new Event("change")); $("apply").click(); })()`);
await sleep(200);
await playToEnd();
p = await panel();
check("settings form: a custom rigid, random-order run gathers", p.status.includes("gathered"), JSON.stringify(p));
await js(`(() => { document.getElementById("f_positions").value = "0,0; nope"; document.getElementById("apply").click(); })()`);
check("settings form: bad coordinates give a message", (await js("document.getElementById('f_error').textContent")).includes("is not x,y"));

// ── the simulator page ───────────────────────────────────────────────────
async function openSim(w, h, touch) {
  await send("Emulation.setDeviceMetricsOverride", { width: w, height: h, deviceScaleFactor: touch ? 2 : 1, mobile: touch });
  await navigate(`${base}/`);
  await until("!!window.lcm", 10000);
  await js("window.lcm.ready");
  await sleep(200);
}
const pick = (key) => js(`(() => { const s = document.getElementById("algorithm"); s.value = ${JSON.stringify(key)}; s.dispatchEvent(new Event("change")); })()`);
await openSim(1280, 800, false);
check("simulator: SqGathering is in the Algorithm menu", await js("[...document.getElementById('algorithm').options].some((o) => o.value === 'SqGathering')"));
check("simulator: the scheduler starts as Async", (await js("window.lcm.settings.values.scheduler")) === "async");
await pick("SqGathering");
await sleep(400);
check("simulator: picking SqGathering switches the scheduler to Sequential and says so",
  (await js("window.lcm.settings.values.scheduler")) === "sequential" && (await js("/Scheduler set to Sequential/.test(document.getElementById('toast').textContent)")));
check("simulator: the run is ready (no refusal)", (await js("window.lcm.state")) === "ready");
await js("document.getElementById('playback').value = 'Infinity'; document.getElementById('playback').dispatchEvent(new Event('change'))");
await js("document.getElementById('play').click()");
const ended = await until("window.lcm.state === 'ended'", 90000);
const s = await js("window.lcm.stats");
check("simulator: SqGathering runs until every robot has gathered and terminated",
  ended && s.terminated === s.robots && s.stop !== "stalled", JSON.stringify({ terminated: s.terminated, robots: s.robots, epochs: s.epochs, stop: s.stop }));
await shot("sim-sqgathering-gathered");

// Stacked starts: rings on the multiplicity points.
await js(`window.lcm.settings.set("multiplicities", 3)`);
await js("document.getElementById('reset').click()");
await sleep(400);
const rings = await js(`(() => { const f = window.lcm.renderer.frame; const seen = new Map(); for (let i = 0; i < f.robots; i++) { const k = f.xy[2*i] + ',' + f.xy[2*i+1]; seen.set(k, (seen.get(k) ?? 0) + 1); } return { stacked: [...seen.values()].filter((c) => c > 1).length, ringing: window.lcm.renderer.options.multiplicities }; })()`);
check("simulator: 3 stacked points, drawn with rings", rings.stacked === 3 && rings.ringing, JSON.stringify(rings));
await shot("sim-sqgathering-stacked");
await js("document.getElementById('play').click()");
check("simulator: the stacked start gathers too", await until("window.lcm.state === 'ended' && window.lcm.stats.terminated === window.lcm.stats.robots", 90000));

// Refusals come back as a message, not a broken page.
await js(`window.lcm.settings.set("unlimited_visibility", false)`);
await js("document.getElementById('reset').click()");
await sleep(400);
check("simulator: limited visibility is refused with the reason",
  await js("/unlimited visibility/.test(document.getElementById('toast').textContent)"), await js("document.getElementById('toast').textContent"));
await js(`window.lcm.settings.set("unlimited_visibility", true); window.lcm.settings.set("multiplicities", 0)`);

// A new algorithm picked while a run is playing starts at once.
await js("document.getElementById('reset').click()");
await sleep(300);
await js("document.getElementById('playback').value = '1'; document.getElementById('playback').dispatchEvent(new Event('change'))");
await js("document.getElementById('play').click()");
await until("window.lcm.state === 'running'", 3000);
await pick("Gathering");
await sleep(400);
check("simulator: switching algorithm mid-run starts the new one at once",
  (await js("window.lcm.state")) === "ready" && (await js("document.getElementById('dirty').hidden")) && !(await js("window.lcm.renderer.options.multiplicities")));

// ── layout ───────────────────────────────────────────────────────────────
for (const [name, w, h, dpr, touch] of [["phone", 375, 812, 2, true], ["tablet", 768, 1024, 2, true], ["desktop", 1440, 900, 1, false]]) {
  await send("Emulation.setDeviceMetricsOverride", { width: w, height: h, deviceScaleFactor: dpr, mobile: touch });
  await open("nonrigid_collision@3");
  await sleep(150);
  check(`step page, ${name}: no sideways scrolling`, !(await js("document.documentElement.scrollWidth > innerWidth")), String(await js("document.documentElement.scrollWidth")));
  check(`step page, ${name}: playback never scrolls the page`, (await js("scrollY")) === 0);
  await shot(`step-page-${name}`);
}

check("no console errors", consoleErrors.length === 0, consoleErrors.join(" | "));
await finish();

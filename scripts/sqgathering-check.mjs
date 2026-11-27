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

// The paper's model is fixed while SqGathering is chosen: limited visibility and the async
// scheduler cannot be picked (the settings are greyed out and a change is ignored).
await js(`window.lcm.settings.set("unlimited_visibility", false); window.lcm.settings.set("scheduler", "async")`);
check("simulator: the paper's model is fixed (unlimited visibility, sequential scheduler, greyed out)",
  await js(`window.lcm.settings.values.unlimited_visibility === true && window.lcm.settings.values.scheduler === "sequential"
    && document.querySelector('[data-key="scheduler"] select').disabled && document.querySelector('[data-key="unlimited_visibility"] input').disabled`),
  await js(`JSON.stringify({ vis: window.lcm.settings.values.unlimited_visibility, scheduler: window.lcm.settings.values.scheduler })`));
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
check("simulator: leaving SqGathering frees the scheduler again", !(await js(`document.querySelector('[data-key="scheduler"] select').disabled`)));

// Sequential Pattern Formation (Algorithms 1 and 7 of the same paper): the model is fixed, robots
// never terminate, and the run stops once nothing moves, with the pattern formed.
await js(`window.lcm.settings.set("scheduler", "async")`);
await pick("SqPatternFormation");
await sleep(400);
check("simulator: picking SqPF fixes the sequential scheduler and non-rigid moves with δ > 0",
  await js(`(() => { const v = window.lcm.settings.values, q = (k, t) => document.querySelector('[data-key="' + k + '"] ' + t);
    return v.scheduler === "sequential" && q("scheduler", "select").disabled && v.rigid_movement === false
      && q("rigid_movement", "input").disabled && v.delta >= 1 && v.unlimited_visibility && v.num_of_faults === 0; })()`),
  await js("JSON.stringify(window.lcm.settings.values)"));
check("simulator: SqPF shows the pattern list", !(await js("document.getElementById('target-pattern').hidden")));
await js(`(() => { const t = document.getElementById("target-pattern"); t.value = "pentagon"; t.dispatchEvent(new Event("change")); })()`);
check("simulator: the robot.py Pattern Formation is not offered next to SqPF",
  !(await js(`[...document.getElementById("algorithm").options].some((o) => o.value === "PatternFormation")`)));
await js(`window.lcm.settings.set("num_of_robots", 7)`);
await js("document.getElementById('reset').click()");
await sleep(400);
check("simulator: SqPF fixes one robot per pattern point (5 for a pentagon) and locks the count",
  await js(`window.lcm.state === "ready" && window.lcm.settings.values.num_of_robots === 5
    && window.lcm.renderer.frame.flags.length === 5 && document.getElementById("set-num_of_robots-n").disabled`),
  await js("String(window.lcm.settings.values.num_of_robots)"));
await js("document.getElementById('playback').value = 'Infinity'; document.getElementById('playback').dispatchEvent(new Event('change'))");
await js("document.getElementById('play').click()");
const formed = await until("window.lcm.state === 'ended'", 90000);
check("simulator: SqPF ends with the pattern formed and says so",
  formed && (await js("/Formed/.test(document.getElementById('hud_status').textContent)")),
  await js("document.getElementById('hud_status').textContent"));
check("simulator: the formed shape is drawn, joining the pattern points, every robot on one",
  await js(`(() => { const f = window.lcm.renderer.frame, s = window.lcm.renderer.patternStats;
    return !!(f.pattern && f.pattern.world && f.pattern.m === 5 && s && s.reached === f.robots); })()`),
  await js("JSON.stringify(window.lcm.renderer.patternStats)"));
await shot("sim-sqpf-formed");

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

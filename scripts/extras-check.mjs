// The extras pages in headless Chrome:
//  - web/ring.html (GatheRRing): every solvable preset gathers when stepped to
//    the end, the unsolvable ones show the warning, and clicking a node adds a
//    robot to the start.
//  - the simulator's examples for Advanced gathering and Mutual visibility:
//    each loads its own start (the robots it lists) and ends as its note says.
//   node scripts/extras-check.mjs [screenshot-dir]   (CHROME=… on Windows)
import { tmpdir } from "node:os";
import { join } from "node:path";
import { openBrowser } from "./lib/browser.mjs";

const OUT = process.argv[2] ?? join(tmpdir(), "lcm-extras");
const { base, send, js, sleep, shot, until, check, finish, consoleErrors } = await openBrowser(OUT);
await send("Page.enable");
await send("Runtime.enable");

async function navigate(url) {
  await send("Page.navigate", { url: "about:blank" });
  await sleep(50);
  await send("Page.navigate", { url });
}

/* ------------------------------------------------------------ ring page */
await navigate(`${base}/web/ring.html`);
check("ring page loads", await until("window.lcmRing && lcmRing.state && document.getElementById('ring').childElementCount > 0", 10000));
const presets = await js(`[...document.getElementById("preset").options].filter((o) => o.value !== "").map((o) => ({ value: o.value, label: o.textContent }))`);
for (const p of presets) {
  const unsolvable = p.label.startsWith("Unsolvable");
  await js(`(() => { const s = document.getElementById("preset"); s.value = "${p.value}"; s.dispatchEvent(new Event("change")); })()`);
  await sleep(50);
  const warned = await js(`!document.getElementById("warning").hidden`);
  check(`ring "${p.label}": ${unsolvable ? "warns it is unsolvable" : "no unsolvable warning"}`, warned === unsolvable);
  if (unsolvable) continue;
  // Step to the end (the page's own Step button logic), at most 10,000 activations.
  const end = await js(`(() => { let k = 0; while (lcmRing.state.status === "running" && k < 10000) { lcmRing.step(); k++; } return { status: lcmRing.state.status, steps: k, rows: document.querySelectorAll("#trace tbody tr").length }; })()`);
  check(`ring "${p.label}": gathers`, end.status === "Gathered" || end.status === "gathered", JSON.stringify(end));
  check(`ring "${p.label}": the trace lists the moves`, end.rows > 0 && end.rows <= 200, JSON.stringify(end));
}
await shot("ring-gathered");
// Clicking a node adds a robot to the start.
await js(`document.getElementById("clear").click()`);
await sleep(50);
const added = await js(`(() => {
  const node = document.querySelector("#ring [data-node='2']");
  if (!node) return null;
  const opts = { bubbles: true, button: 0, pointerId: 1 };
  node.dispatchEvent(new PointerEvent("pointerdown", opts));
  node.dispatchEvent(new PointerEvent("pointerup", opts));
  return lcmRing.state ? lcmRing.state.positions.join(",") : null;
})()`);
check("ring: clicking a node adds a robot there", added === "2", String(added));

// A 60-node ring with 40 robots stays readable: a node number on every few nodes (and every occupied
// one), at most "R7 +3" per crowded node, every robot named in its node's hover title.
const big = await js(`(() => {
  const robots = Array.from({ length: 40 }, (_, k) => (k * 7) % 60).concat([5, 5, 5, 5]);
  lcmRing.setStart({ nodes: 60, robots });
  const labels = document.querySelectorAll("#ring text.label").length;
  const ids = [...document.querySelectorAll("#ring text.ids")].map((t) => t.textContent);
  const titled = [...document.querySelectorAll("#ring [data-node] title")].map((t) => t.textContent).filter((t) => t.includes("R")).join(" ");
  const occupied = new Set(robots).size;
  return { labels, occupied, longest: Math.max(...ids.map((t) => t.length)), named: robots.every((_, r) => titled.split(" ").map((w) => w.replace(",", "")).includes("R" + r)) };
})()`);
check("ring: a 60-node ring writes few node numbers", big.labels <= 24 + big.occupied && big.labels < 60, JSON.stringify(big));
check("ring: crowded nodes stay short, hover names every robot", big.longest <= 7 && big.named, JSON.stringify(big));
await shot("ring-60-nodes");
// The speed slider says its pace; the CSV holds every turn.
const pace = await js(`(() => { const s = document.getElementById("speed"); s.value = "10"; s.dispatchEvent(new Event("input")); return document.getElementById("speed_out").textContent; })()`);
check("ring: the speed slider shows activations per second", pace === "10 / s", pace);
const csv = await js(`(() => { const s = document.getElementById("preset"); s.value = "2"; s.dispatchEvent(new Event("change"));
  while (lcmRing.state.status === "running") lcmRing.step();
  const lines = lcmRing.traceCsv().trim().split("\\n");
  return { header: lines[1], rows: lines.length - 2, turns: lcmRing.trace.length, first: lines[2] }; })()`);
check("ring: the CSV export holds every turn", csv.header === "turn,epoch,robot,from,to,task,rule" && csv.rows === csv.turns && csv.rows > 0, JSON.stringify(csv));

/* ------------------------------------------------- simulator examples */
await navigate(`${base}/`);
check("simulator loads", await until("window.lcm && lcm.state === 'ready'", 15000));

/** Runs every example of `menu` (and `variant`), checking each with `expect(result)`. Runs that never
 * end (`forever(label)`) play for a few seconds instead. */
async function examples(menu, variant, expect, forever = () => false) {
  await js(`(() => { const a = document.getElementById("algorithm"); a.value = "${menu}"; a.dispatchEvent(new Event("change", { bubbles: true })); })()`);
  if (variant) await js(`lcm.settings.set("variant", "${variant}", true)`);
  await sleep(200);
  const count = await js(`document.querySelectorAll("#examples_menu [data-example]").length - 1`);
  check(`${variant ?? menu}: has examples`, count > 0, String(count));
  for (let k = 0; k < count; k++) {
    const label = await js(`document.querySelector("#examples_menu [data-example='${k}'] .title").textContent`);
    await js(`document.querySelector("#examples_menu [data-example='${k}']").click()`);
    await until("lcm.state === 'ready' || lcm.state === 'running' || lcm.state === 'paused'", 5000);
    await js(`(() => { const p = document.getElementById("playback"); p.value = "Infinity"; p.dispatchEvent(new Event("change", { bubbles: true })); })()`);
    if (await js(`lcm.state !== "running"`)) await js(`document.dispatchEvent(new KeyboardEvent("keydown", { key: " ", code: "Space", bubbles: true }))`);
    const ended = forever(label) ? (await sleep(8000), true) : await until("lcm.state === 'ended'", 60000);
    const r = await js(`(() => { const f = lcm.renderer.frame, v = lcm.settings.values;
      return { stop: f.stop, robots: f.robots, terminated: f.terminated, seeing: f.seeing,
        gap: f.robots === 2 ? Math.hypot(f.xy[0] - f.xy[2], f.xy[1] - f.xy[3]) : null,
        listed: v.pattern === "custom" ? v.custom.split("\\n").filter((l) => l.trim()).length : null,
        status: document.getElementById("hud_status").textContent }; })()`);
    const name = `${variant ?? menu} "${label}"`;
    check(`${name}: loads its own start`, r.listed == null || r.listed === r.robots, JSON.stringify(r));
    check(`${name}: ends as described`, ended && expect(r), JSON.stringify(r));
    await shot(`example-${variant ?? menu}-${k}`);
  }
}
const allGathered = (r) => r.stop === "ended" && r.terminated === r.robots;
// A_AD under async: the robots meet and stay together but never terminate (Remark 2).
await examples("advanced", "CompassGathering", (r) => allGathered(r) || (r.stop === "running" && r.gap < 1e-3), (label) => label.includes("async"));
await examples("advanced", "RandomizedGathering", allGathered);
// Crashed robots never terminate: everyone else does, and the status says how many.
await examples("advanced", "CrashGathering", (r) => r.stop === "ended" && (r.terminated === r.robots || (r.terminated > 0 && r.status.includes(`${r.terminated} of ${r.robots}`))));
// Mutual visibility never terminates: the examples stop at a turn limit, with everyone in sight.
await examples("MutualVisibility", null, (r) => r.stop !== "running" && r.seeing === r.robots && r.status.includes(`${r.robots} of ${r.robots} see everyone`));

// The inspector's "If it looked now": the algorithm's own account of the selected robot's next Compute.
await js(`(() => { const a = document.getElementById("algorithm"); a.value = "advanced"; a.dispatchEvent(new Event("change", { bubbles: true })); })()`);
await js(`lcm.settings.set("variant", "CompassGathering", true)`);
await sleep(200);
const runBefore = await js(`lcm.renderer.frame ? lcm.renderer.frame.run : -1`);
await js(`document.querySelector("#examples_menu [data-example='0']").click()`);
await until(`lcm.renderer.frame && lcm.renderer.frame.run !== ${runBefore} && lcm.renderer.frame.robots === 2`, 5000);
if (await js(`lcm.state === "running"`)) await js(`document.dispatchEvent(new KeyboardEvent("keydown", { key: " ", code: "Space", bubbles: true }))`);
await js(`(() => { window.__why = null; lcm.client.on("robot", ({ info }) => { window.__why = info && info.why; }); lcm.client.inspect(0); })()`);
await until("window.__why !== null", 5000);
const why = await js(`window.__why`);
check("inspector explains the robot's rule (compass gathering)", typeof why === "string" && why.includes("fixed compass") && why.includes("By its own compass"), String(why));

check("no console errors", consoleErrors.length === 0, consoleErrors.join(" | "));
await finish();

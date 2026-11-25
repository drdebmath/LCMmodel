// The ring page (web/ring.html, GatheRRing) in headless Chrome: every solvable
// preset gathers when stepped to the end, the unsolvable ones show the warning,
// the paper's Fig. 4 is reported over the bound, clicking a node adds a robot to
// the start, and the page loads web/ring-pkg only (never the simulator's web/pkg).
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

// The paper's Fig. 4 gathers in round 4, over the bound of 3 (a known exception), and the page says so.
const fig4 = await js(`(() => { const s = document.getElementById("preset");
  s.value = String([...s.options].findIndex((o) => o.textContent.startsWith("Paper's Fig. 4")) - 1); s.dispatchEvent(new Event("change"));
  while (lcmRing.state.status === "running") lcmRing.step();
  return { status: lcmRing.state.status, epoch: lcmRing.state.epoch, bound: lcmRing.state.bound, card: document.getElementById("next_card").textContent }; })()`);
check("ring: Fig. 4 gathers in round 4, reported over the paper's bound", fig4.status === "gathered" && fig4.epoch === 4 && fig4.bound === 3 && fig4.card.includes("over the paper's bound"), JSON.stringify(fig4));
check("ring: the notes list the known exceptions", await js(`document.querySelector(".ring-how-body").textContent.includes("Theorem V.1 (n − 3 epochs) is the paper's claim; an exhaustive check found these")`));

// Only the ring package: the simulator's web/pkg is never fetched.
const loaded = await js(`performance.getEntriesByType("resource").map((e) => new URL(e.name).pathname)`);
check("ring: loads web/ring-pkg", loaded.some((p) => p.includes("/web/ring-pkg/lcm_ring_wasm_bg.wasm")), loaded.join(" "));
check("ring: does not load the simulator's web/pkg", !loaded.some((p) => p.includes("/web/pkg/")), loaded.join(" "));

check("no console errors", consoleErrors.length === 0, consoleErrors.join(" | "));
await finish();

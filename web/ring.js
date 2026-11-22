/* Ring page: GatheRRing (crates/lcm-ring) run turn by turn in the browser,
 * through the WasmRing binding of web/pkg. The page owns the start (nodes,
 * robots, order, ties); every change rebuilds the run from it. */
import { icon, renderIcons } from "./icons.js";
import { menuHtml } from "./menu.js";

const $ = (id) => document.getElementById(id);
renderIcons();

const PRESETS = [
  { label: "Three robots, all apart", nodes: 9, robots: [0, 2, 5] },
  { label: "Seven robots, some stacked", nodes: 12, robots: [0, 0, 3, 4, 4, 8, 9] },
  { label: "Ten robots on fifteen nodes", nodes: 15, robots: [0, 1, 3, 4, 6, 8, 9, 11, 12, 14] },
  { label: "6-ring, two islands of two", nodes: 6, robots: [0, 1, 3, 4] },
  { label: "Every node but two", nodes: 10, robots: [0, 1, 2, 4, 5, 6, 7, 8] },
  { label: "Unsolvable: three in a row", nodes: 9, robots: [3, 4, 5] },
  { label: "Unsolvable: a full 5-ring", nodes: 5, robots: [0, 1, 2, 3, 4] },
];

let wasm = null;
let ring = null;      // the WasmRing being played
let start = { nodes: 9, robots: [0, 2, 5] };
let state = null;
let trace = [];
let timer = 0;

$("preset").innerHTML = `<option value="">Your own</option>` + PRESETS.map((p, k) => `<option value="${k}">${p.label}</option>`).join("");

function config() {
  const order = $("order").value.split(/[\s,;]+/).filter(Boolean).map(Number);
  return {
    nodes: start.nodes,
    robots: start.robots,
    order: order.length ? order : null,
    ties: $("ties").value,
    seed: 7,
  };
}

function rebuild() {
  stop();
  trace = [];
  ring?.free();
  ring = null;
  if (!start.robots.length) { state = null; draw(); say("Click nodes to place robots."); return; }
  try {
    ring = new wasm.WasmRing(JSON.stringify(config()));
    state = JSON.parse(ring.state());
    say("");
  } catch (e) {
    state = null;
    say(`<strong>Cannot start:</strong> ${String(e.message ?? e)}`);
  }
  draw();
}

function say(html) { $("ring_status").innerHTML = html; }

function step() {
  if (!ring || state.status !== "running") return false;
  const t = JSON.parse(ring.step());
  if (!t) return false;
  trace.push(t);
  state = JSON.parse(ring.state());
  draw(t);
  if (t.from !== t.to) slide(t.from, t.to);
  return state.status === "running";
}

/* A robot that steps slides along the ring to its new node (skipped at high speed or with reduced motion). */
let sliding = 0;
function slide(from, to) {
  cancelAnimationFrame(sliding);
  const rate = Number($("speed").value), ms = Math.min(380, 700 / rate);
  if (rate > 15 || matchMedia("(prefers-reduced-motion: reduce)").matches) { $("ring").querySelector(".arriving")?.classList.remove("arriving"); return; }
  const n = start.nodes, R = 120, nodeR = Math.max(7, Math.min(16, 300 / n));
  const a0 = -Math.PI / 2 + (2 * Math.PI * from) / n;
  let d = (2 * Math.PI * (((to - from) % n + n) % n)) / n;
  if (d > Math.PI) d -= 2 * Math.PI; // the short way round: one step either side
  const svg = $("ring");
  const ns = "http://www.w3.org/2000/svg";
  const dot = document.createElementNS(ns, "circle");
  dot.setAttribute("class", "robot mover");
  dot.setAttribute("r", String(nodeR - 3));
  dot.setAttribute("pointer-events", "none");
  svg.append(dot);
  const t0 = performance.now();
  const frame = (now) => {
    const k = Math.min(1, (now - t0) / ms), e = k < 0.5 ? 2 * k * k : 1 - (-2 * k + 2) ** 2 / 2; // ease in and out
    const a = a0 + d * e;
    dot.setAttribute("cx", String(R * Math.cos(a)));
    dot.setAttribute("cy", String(R * Math.sin(a)));
    if (k < 1) sliding = requestAnimationFrame(frame);
    else { dot.remove(); svg.querySelector(".arriving")?.classList.remove("arriving"); }
  };
  sliding = requestAnimationFrame(frame);
}

function play() {
  if (timer) { stop(); return; }
  if (!ring || state.status !== "running") return;
  $("play").innerHTML = `${icon("pause")}<span class="label">Pause</span>`;
  const tick = () => { if (step()) timer = setTimeout(tick, 1000 / Number($("speed").value)); else stop(); };
  timer = setTimeout(tick, 0);
}
function stop() {
  clearTimeout(timer);
  timer = 0;
  $("play").innerHTML = `${icon("play")}<span class="label">Play</span>`;
}

const STATUS = { running: "Running", gathered: "Gathered", stalled: "Stuck: a whole round without a move", incomplete: "Stopped at the round limit" };
// The situation a robot sees (the algorithm's task, T1 to T8), and why it moves, in plain words.
const SITUATION = {
  T1: "Several gaps", T2: "One gap of 4 nodes or fewer, or none", T3: "One gap of 5 nodes or more",
  T4: "Two gaps, one a single node", T5: "Two occupied nodes, one empty node between",
  T6: "Three nodes in a row occupied", T7: "Two neighbouring nodes occupied", T8: "Gathered on one node",
};
const situation = (task) => SITUATION[task] ?? task ?? "—";
// The same situations explained in full, for the card about the next move.
const SITUATION_LONG = {
  T1: "The occupied nodes are separated by several empty stretches (gaps). Only robots standing right next to one of the biggest gaps move; everybody else stays. Round by round this shrinks the space the robots spread over.",
  T2: "There is at most one gap and it is short (four nodes or fewer), or there is no empty node at all. A robot with an empty neighbour stays; a robot with robots on both sides heads towards the nearest empty node, which reshapes the swarm towards a single block.",
  T3: "There is exactly one gap and it is long (five nodes or more), so the robots stand in one unbroken block. Robots at the two ends of the block step out into the gap; robots inside the block stay.",
  T4: "There are exactly two gaps: one is a single empty node, the other is longer. A robot that stands next to the longer gap with another robot on its other side steps towards that robot, pulling the group together away from the long gap.",
  T5: "Only two nodes are occupied, with exactly one empty node between them. A robot next to that empty node steps into it, so the robots end up on neighbouring nodes.",
  T6: "The robots occupy exactly three nodes in a row. Robots on the two end nodes step inwards onto the occupied neighbour; robots on the middle node stay, so the three nodes shrink to one.",
  T7: "Only two nodes are occupied and they are neighbours. A robot on either of them steps onto the other one, which finishes the gathering.",
  T8: "Every robot stands on the same node: the robots have gathered, and nobody moves any more.",
};
const why = (rule) => {
  const text = String(rule ?? "").replace(/^m\d+:\s*/, "").replace(/\bholes?\b/g, (w) => (w === "holes" ? "gaps" : "gap"));
  return text.charAt(0).toUpperCase() + text.slice(1);
};

function draw(last = null) {
  const n = start.nodes, R = 120;
  const at = (v) => { const a = -Math.PI / 2 + (2 * Math.PI * v) / n; return [R * Math.cos(a), R * Math.sin(a)]; };
  const positions = state?.positions ?? start.robots;
  const count = new Array(n).fill(0);
  const ids = Array.from({ length: n }, () => []);
  positions.forEach((v, r) => { count[v]++; ids[v].push(r); });
  const biggest = Math.max(0, ...(state?.holes ?? []).map((h) => h[1]));
  const big = new Array(n).fill(false);
  for (const [first, len] of state?.holes ?? []) if (len === biggest) for (let k = 0; k < len; k++) big[(first + k) % n] = true;
  const nodeR = Math.max(7, Math.min(16, 300 / n));
  // Big rings: text shrinks with the nodes, only every few node numbers are written (and every occupied
  // node's), and a crowded node lists its first robot and how many more. Hover a node for all of it.
  const font = Math.max(6.5, Math.min(11, nodeR * 0.85));
  const every = Math.ceil(n / 24);
  let svg = `<defs><marker id="tip" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="currentColor" style="color:var(--sg-active)"/></marker></defs>`;
  svg += `<circle class="edge" r="${R}"/>`;
  for (let v = 0; v < n; v++) {
    const [x, y] = at(v);
    const who = ids[v].length ? `: ${ids[v].map((r) => `R${r}`).join(", ")}` : "";
    svg += `<circle class="node${count[v] ? " occupied" : ""}${big[v] ? " hole-big" : ""}" data-node="${v}" cx="${x}" cy="${y}" r="${nodeR}"><title>Node ${v}${who}</title></circle>`;
    if (v % every === 0 || count[v]) svg += `<text class="label" x="${x * 1.2}" y="${y * 1.2}" font-size="${font}">${v}</text>`;
  }
  const gathered = state?.status === "gathered";
  for (let v = 0; v < n; v++) {
    if (!count[v]) continue;
    const [x, y] = at(v);
    const isNext = state && state.status === "running" && ids[v].includes(state.next);
    // The robot that just stepped here alone appears when its slide arrives (see slide()).
    const arriving = last && last.from !== last.to && last.to === v && count[v] === 1;
    if (isNext) svg += `<circle class="halo" cx="${x}" cy="${y}" r="${nodeR + 4}" pointer-events="none"/>`;
    if (gathered) svg += `<circle class="gather-burst" cx="${x}" cy="${y}" r="${nodeR}" pointer-events="none"/>`;
    svg += `<circle class="robot${isNext ? " next" : ""}${gathered ? " ring-gathered" : ""}${arriving ? " arriving" : ""}" cx="${x}" cy="${y}" r="${nodeR - 3}" pointer-events="none"/>`;
    if (count[v] > 1) svg += `<text class="count" x="${x}" y="${y}" font-size="${font}">${count[v]}</text>`;
    const shown = ids[v].length === 1 ? `R${ids[v][0]}` : `R${ids[v][0]} +${ids[v].length - 1}`; // hover the node for every name
    // Over 30 nodes, names are written only for the robot about to move; hover a node for the others.
    // Written just inside the ring, anchored on the side facing the centre so longer names never reach the node.
    const anchor = x > R * 0.25 ? "end" : x < -R * 0.25 ? "start" : "middle";
    if (n <= 30 || isNext) svg += `<text class="ids" x="${x * 0.84}" y="${y * 0.8}" text-anchor="${anchor}" font-size="${font}">${shown}</text>`;
  }
  // Where the next robot will go.
  if (state && state.status === "running" && (state.move === 1 || state.move === -1)) {
    const from = positions[state.next], to = (from + state.move + n) % n;
    const a0 = -Math.PI / 2 + (2 * Math.PI * from) / n, a1 = -Math.PI / 2 + (2 * Math.PI * to) / n;
    const rr = R + nodeR + 8, sweep = state.move === 1 ? 1 : 0;
    svg += `<path class="arrow" pathLength="1" d="M${rr * Math.cos(a0)},${rr * Math.sin(a0)} A${rr},${rr} 0 0 ${sweep} ${rr * Math.cos(a1)},${rr * Math.sin(a1)}"/>`;
  }
  $("ring").innerHTML = svg;

  const gaps = (state?.holes ?? []).map((h) => h[1]).sort((a, b) => b - a);
  const facts = state ? [
    ["Status", STATUS[state.status] ?? state.status],
    ["Rounds", `${state.epochs} <span class="muted">of at most ${state.bound}</span>`],
    ["Turns taken", `${state.turns} <span class="muted">· ${plural(trace.filter((t) => t.from !== t.to).length, "move")}</span>`],
    ["Robots · occupied nodes", `${positions.length} · ${count.filter((c) => c).length}`],
    ["Gaps (empty stretches)", gaps.length ? gaps.join(", ") : "none"],
  ] : [["Robots", positions.length]];
  $("facts").innerHTML = facts.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join("");
  $("ring_bar").style.width = state ? `${Math.min(100, (100 * state.epochs) / Math.max(1, state.bound))}%` : "0%";
  $("ring_bar").classList.toggle("done", gathered);
  nextCard(positions, gathered);
  const stuck = state?.unsolvable?.replace(/\s*\([^)]*\)\s*$/, "");
  $("warning").hidden = !stuck;
  if (stuck) $("warning").innerHTML = `<strong>No algorithm can gather this start</strong> (${stuck}): taking turns in some order keeps the robots apart for ever. Watch it fail, or change the start.`;
  if (last) say(`<strong>R${last.robot}</strong> ${last.from === last.to ? `stays on node ${last.from}` : `steps ${last.from} → ${last.to}`} · ${why(last.rule)}`);
  else if (gathered) say("<strong>Gathered</strong>: every robot on one node.");
  $("trace").querySelector("tbody").innerHTML = trace.slice(-200).reverse().map((t, k) =>
    `<tr class="${t.from !== t.to ? "moved" : ""}${k === 0 && last ? " fresh" : ""}"><td>${t.turn}</td><td>${t.epoch}</td><td>R${t.robot}</td><td>${t.from === t.to ? `stays ${t.from}` : `${t.from} → ${t.to}`}</td><td>${situation(t.task)}</td><td>${why(t.rule)}</td></tr>`).join("");
  $("turns_count").textContent = trace.length ? `· ${trace.length}${trace.length > 200 ? " (latest 200 shown; CSV has all)" : ""}` : "";
}

const plural = (k, word) => `${k} ${word}${k === 1 ? "" : "s"}`;

/** The card above the facts: whose turn it is, what that robot sees, and what it will do, in full sentences. */
function nextCard(positions, gathered) {
  const el = $("next_card");
  if (!state) { el.innerHTML = `<div class="nc-head">Place robots to begin</div><p>Click nodes of the ring to put robots on them, or pick a preset above.</p>`; return; }
  if (gathered) {
    el.innerHTML = `<div class="nc-head done">Gathered on node ${positions[0]}</div><p>Every robot stands on the same node after ${state.epochs} round${state.epochs === 1 ? "" : "s"} and ${state.turns} turns, within the guaranteed ${state.bound}. Nobody moves any more.</p>`;
    return;
  }
  if (state.status !== "running") {
    el.innerHTML = `<div class="nc-head warn">${STATUS[state.status] ?? state.status}</div><p>${state.status === "stalled" ? "A whole round passed without any robot moving, so nothing can change any more: this start cannot be gathered under this turn order." : "The run stopped at its round limit before gathering."}</p>`;
    return;
  }
  const n = start.nodes, from = positions[state.next];
  const to = state.move === 1 || state.move === -1 ? (from + state.move + n) % n : null;
  const does = to !== null ? `It will step from node ${from} to node ${to}.`
    : state.move === 2 ? `It will step to one of its two neighbours; both are equally good, and the Ties setting decides which.`
    : `It will stay on node ${from}.`;
  el.innerHTML = `<div class="nc-head"><span class="nc-dot"></span>R${state.next}'s turn next <span class="muted">· node ${from}</span></div>`
    + `<p><strong>What it sees.</strong> ${SITUATION_LONG[state.task] ?? situation(state.task)}</p>`
    + `<p><strong>What it does.</strong> ${why(state.rule)}. ${does}</p>`;
}

/* Editing the start: click a node to add a robot, right-click (or long-press) to take one off. */
function editNode(v, add) {
  const robots = [...start.robots];
  if (add) robots.push(v);
  else { const k = robots.lastIndexOf(v); if (k < 0) return; robots.splice(k, 1); }
  start = { nodes: start.nodes, robots };
  $("preset").value = "";
  $("order").value = "";
  rebuild();
}
let press = 0;
$("ring").addEventListener("pointerdown", (e) => {
  const v = e.target.closest?.("[data-node]")?.dataset.node;
  if (v === undefined) return;
  press = setTimeout(() => { press = -1; editNode(Number(v), false); }, 550);
});
$("ring").addEventListener("pointerup", (e) => {
  const v = e.target.closest?.("[data-node]")?.dataset.node;
  const long = press === -1;
  clearTimeout(press);
  press = 0;
  if (v === undefined || long || e.button !== 0) return;
  editNode(Number(v), true);
});
$("ring").addEventListener("contextmenu", (e) => {
  const v = e.target.closest?.("[data-node]")?.dataset.node;
  e.preventDefault();
  if (v !== undefined) editNode(Number(v), false);
});

$("preset").addEventListener("change", () => {
  const p = PRESETS[Number($("preset").value)];
  if (!p) return;
  start = { nodes: p.nodes, robots: [...p.robots] };
  $("nodes").value = p.nodes;
  $("order").value = "";
  rebuild();
});
$("nodes").addEventListener("change", () => {
  const n = Math.max(3, Math.min(60, Number($("nodes").value) || 3));
  $("nodes").value = n;
  start = { nodes: n, robots: start.robots.filter((v) => v < n) };
  $("preset").value = "";
  rebuild();
});
$("order").addEventListener("change", rebuild);
$("ties").addEventListener("change", rebuild);
$("reset").addEventListener("click", rebuild);
$("step").addEventListener("click", () => { stop(); step(); });
$("play").addEventListener("click", play);
function speedLabel() { $("speed_out").textContent = `${Number($("speed").value)} / s`; }
$("speed").addEventListener("input", () => {
  speedLabel();
  if (timer) { stop(); play(); } // the new pace from the next activation
});
speedLabel();
/** Every turn so far (not just the 200 the table shows), one row each. */
function traceCsv() {
  const q = (s) => `"${String(s).replace(/"/g, '""')}"`;
  const head = `# GatheRRing on a ${start.nodes}-ring, start ${start.robots.join(" ")}, choices left open: ${$("ties").value}`;
  return [head, "turn,epoch,robot,from,to,task,rule",
    ...trace.map((t) => [t.turn, t.epoch, t.robot, t.from, t.to, q(t.task), q(t.rule)].join(","))].join("\n") + "\n";
}
$("csv").addEventListener("click", () => {
  const url = URL.createObjectURL(new Blob([traceCsv()], { type: "text/csv" }));
  const a = Object.assign(document.createElement("a"), { href: url, download: `gatherring-${start.nodes}-nodes.csv` });
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
});
$("clear").addEventListener("click", () => { start = { nodes: start.nodes, robots: [] }; $("preset").value = ""; rebuild(); });
window.addEventListener("keydown", (e) => {
  if (e.target.closest?.("input, select, textarea")) return;
  if (e.key === " ") { e.preventDefault(); play(); }
  if (e.key === "ArrowRight") { e.preventDefault(); stop(); step(); }
});

/* Theme, shared with the other pages. */
const THEMES = ["system", "light", "dark"];
function current() { try { return localStorage.getItem("lcm-theme") || "system"; } catch { return "system"; } }
function apply(t) {
  if (t === "system") delete document.documentElement.dataset.theme; else document.documentElement.dataset.theme = t;
  try { localStorage.setItem("lcm-theme", t); } catch {}
  const dark = t === "dark" || (t === "system" && matchMedia("(prefers-color-scheme: dark)").matches);
  $("theme").innerHTML = icon(dark ? "sun" : "moon");
}
$("theme").addEventListener("click", () => apply(THEMES[(THEMES.indexOf(current()) + 1) % THEMES.length]));
apply(current());

/* The core, with the build id so a cached older package is never used. */
(async () => {
  const { id } = await fetch("./pkg/build.json", { cache: "no-store" }).then((r) => (r.ok ? r.json() : {})).catch(() => ({}));
  const suffix = id ? `?v=${encodeURIComponent(id)}` : "";
  const module = await import(`./pkg/lcm_wasm.js${suffix}`);
  await module.default(`./pkg/lcm_wasm_bg.wasm${suffix}`);
  wasm = module;
  // The simulator's menu: picking anything else opens the simulator on it.
  const pick = $("algorithm_switch");
  pick.innerHTML = menuHtml(JSON.parse(module.algorithms()));
  pick.value = "ring";
  pick.addEventListener("change", () => { if (pick.value !== "ring") location.href = `../?algorithm=${encodeURIComponent(pick.value)}`; });
  window.lcmRing = { get state() { return state; }, get trace() { return trace; }, step, rebuild, traceCsv, setStart: (s) => { start = s; $("nodes").value = s.nodes; rebuild(); } };
  $("preset").value = "0";
  $("preset").dispatchEvent(new Event("change"));
})();

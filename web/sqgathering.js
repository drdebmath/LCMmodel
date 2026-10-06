/* SqGathering page: Algorithm 8 under the sequential scheduler, one round at
 * a time. The Rust core (web/pkg, `WasmSqGathering`) plays the rounds; this
 * page only records them for playback and draws them. Presets are the
 * configs in fixtures/sqgathering, the same files `lcm seq` and
 * scripts/check-seq-parity.mjs run. */

import { icon, renderIcons } from "./icons.js";

const PRESETS = [
  { file: "k0_no_multiplicity.json", label: "κ = 0 · no multiplicity",
    blurb: "All points single. The active robot moves to its closest occupied point; round 2 creates the first multiplicity, after which κ = 1. Hand trace A." },
  { file: "k1_one_multiplicity.json", label: "κ = 1 · one multiplicity",
    blurb: "One multiplicity at the origin. Singles walk to it (stopped after δ by the adversary); a robot on it stays. Hand trace B." },
  { file: "kmany_two_multiplicities.json", label: "κ > 1 · two multiplicities",
    blurb: "Two multiplicities. Singles wait; a robot on a multiplicity heads for the midpoint towards its closest point, which breaks the tie. Hand trace C." },
  { file: "nonrigid_collision.json", label: "Non-rigid stop on a robot",
    blurb: "The adversary stops a joining robot exactly on a single robot, creating a second multiplicity (κ = 2) — twice. The algorithm still gathers." },
  { file: "kmany_three_multiplicities_random.json", label: "κ = 3 · random frames",
    blurb: "Three multiplicities, nine robots, a shuffled fair schedule, random stopping points and private frames with random rotation, scale and handedness." },
  { file: "budget_incomplete.json", label: "Budget-limited run",
    blurb: "Only 25 rounds allowed: the run stops before gathering and is reported as INCOMPLETE, not as a result." },
];

const ALGO = [
  "(q, b) ← position of the activated robot;",
  "κ ← number of multiplicities in Q′(t);",
  "destination ← q;",
  "if κ = 1 then",
  "  if b = 0 then",
  "    q_m ← the multiplicity point in Q′(t);",
  "    destination ← q_m;",
  "else",
  "  if κ > 1 then",
  "    if b = 1 then",
  "      q′ ← closest point to q;",
  "      z ← midpoint of qq′;",
  "      destination ← z;",
  "  else",
  "    q′ ← closest point to q;",
  "    destination ← q′;",
];

// Lines executed by each rule, and the line that sets the destination.
const PATHS = {
  "k1-hold-multiplicity": { lines: [1, 2, 3, 4, 5], dest: 3, paperCase: "case 1 (κ = 1)" },
  "k1-join-multiplicity": { lines: [1, 2, 3, 4, 5, 6, 7], dest: 7, paperCase: "case 1 (κ = 1)" },
  "kmany-leave-multiplicity": { lines: [1, 2, 3, 4, 8, 9, 10, 11, 12, 13], dest: 13, paperCase: "case 2 (κ > 1)" },
  "kmany-wait": { lines: [1, 2, 3, 4, 8, 9, 10], dest: 3, paperCase: "case 2 (κ > 1)" },
  "k0-move-to-closest": { lines: [1, 2, 3, 4, 8, 9, 14, 15, 16], dest: 16, paperCase: "case 3 (κ = 0)" },
  "alone": { lines: [1, 2, 3, 4, 8, 9, 14, 15], dest: 3, paperCase: "n = 1" },
};

const $ = (id) => document.getElementById(id);
const SVG = "http://www.w3.org/2000/svg";
const TOL = 1e-9;

let wasm = null;
let config = null;       // the SeqConfig being played
let presetSchedule = null;
let sim = null;
let history = [];        // [{positions: [[x,y]…], round: object|null, raw: string|null}]
let at = 0;              // index into history being shown
let timer = null;
let bounds = null;

// ------------------------------------------------------------------ helpers
function fmt(v) {
  const s = (Math.abs(v) < 5e-13 ? 0 : v).toFixed(3).replace(/\.?0+$/, "");
  return s === "-0" ? "0" : s;
}
const pt = (p) => `(${fmt(p[0])}, ${fmt(p[1])})`;
const same = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]) <= TOL;

function groups(positions) {
  const out = [];
  positions.forEach((p, i) => {
    const g = out.find((o) => same(o.at, p));
    if (g) g.ids.push(i); else out.push({ at: p, ids: [i] });
  });
  return out;
}

function el(name, attrs = {}, parent = null) {
  const node = document.createElementNS(SVG, name);
  for (const [k, v] of Object.entries(attrs)) node.setAttribute(k, v);
  if (parent) parent.appendChild(node);
  return node;
}

function niceStep(span) {
  const raw = span / 8;
  const p = 10 ** Math.floor(Math.log10(raw));
  return [1, 2, 5, 10].map((m) => m * p).find((s) => s >= raw) ?? raw;
}

// ------------------------------------------------------------------- setup
async function loadCore() {
  const { id } = await fetch("./pkg/build.json", { cache: "no-store" })
    .then((r) => (r.ok ? r.json() : {})).catch(() => ({}));
  const suffix = id ? `?v=${encodeURIComponent(id)}` : "";
  const module = await import(`./pkg/lcm_wasm.js${suffix}`);
  await module.default(`./pkg/lcm_wasm_bg.wasm${suffix}`);
  wasm = module;
}

async function loadPreset(file) {
  const res = await fetch(`../fixtures/sqgathering/${file}`, { cache: "no-store" });
  if (!res.ok) throw new Error(`${file}: ${res.status}`);
  const cfg = await res.json();
  presetSchedule = cfg.schedule;
  fillForm(cfg);
  $("preset_blurb").textContent = PRESETS.find((p) => p.file === file)?.blurb ?? "";
  try { window.history.replaceState(null, "", `#${file.replace(/\.json$/, "")}`); } catch {}
  start(cfg);
}

function start(cfg) {
  pause();
  sim?.free();
  sim = new wasm.WasmSqGathering(JSON.stringify(cfg));
  config = cfg;
  history = [{ positions: cfg.positions.map((p) => [p[0], p[1]]), round: null, raw: null }];
  at = 0;
  const xs = cfg.positions.map((p) => p[0]);
  const ys = cfg.positions.map((p) => p[1]);
  // Every move goes to an occupied point or a midpoint, so robots never leave
  // the convex hull of the start: these bounds hold for the whole run.
  bounds = { x0: Math.min(...xs), x1: Math.max(...xs), y0: Math.min(...ys), y1: Math.max(...ys) };
  $("trace").innerHTML = "";
  $("f_error").textContent = "";
  render();
}

// --------------------------------------------------------------- stepping
function finished() { return sim.status() !== "running"; }

function forward() {
  if (at < history.length - 1) { at++; render(); return true; }
  if (finished()) return false;
  const raw = sim.step();
  if (raw === "null") return false;
  const round = JSON.parse(raw);
  const positions = history[history.length - 1].positions.map((p) => p.slice());
  positions[round.robot] = round.stop.slice();
  history.push({ positions, round, raw });
  appendTraceRow(history.length - 1);
  at = history.length - 1;
  render();
  return true;
}

function backward() { if (at > 0) { at--; render(); } }

function play() {
  if (timer) return;
  const rate = Number($("speed").value);
  $("play").innerHTML = `${icon("pause")}<span class="label">Pause</span>`;
  timer = setInterval(() => { if (!forward()) pause(); }, 1000 / rate);
}

function pause() {
  if (timer) clearInterval(timer);
  timer = null;
  $("play").innerHTML = `${icon("play")}<span class="label">Play</span>`;
}

// ----------------------------------------------------------------- drawing
function render() {
  draw();
  panels();
  $("scrub").max = String(history.length - 1 + (finished() ? 0 : 1));
  $("scrub").value = String(at);
  $("scrub_label").textContent = `round ${at}${at === history.length - 1 && finished() ? " (end)" : ""}`;
  $("back").disabled = at === 0;
  $("restart").disabled = at === 0;
  $("step").disabled = at === history.length - 1 && finished();
  for (const tr of $("trace").children) tr.classList.toggle("current", Number(tr.dataset.i) === at);
  // Keep the current row visible by scrolling the table only, never the page.
  const row = $("trace").querySelector("tr.current");
  const wrap = $("trace").closest(".trace-wrap");
  if (row) {
    const head = wrap.querySelector("thead").offsetHeight;
    if (row.offsetTop - head < wrap.scrollTop) wrap.scrollTop = row.offsetTop - head;
    else if (row.offsetTop + row.offsetHeight > wrap.scrollTop + wrap.clientHeight) wrap.scrollTop = row.offsetTop + row.offsetHeight - wrap.clientHeight;
  }
}

function draw() {
  const svg = $("view");
  const { width: W, height: H } = svg.getBoundingClientRect();
  if (!W || !H) return;
  svg.setAttribute("viewBox", `0 0 ${W} ${H}`);
  svg.replaceChildren();
  const pad = 46;
  const spanX = Math.max(bounds.x1 - bounds.x0, 1e-6);
  const spanY = Math.max(bounds.y1 - bounds.y0, 1e-6);
  const scale = Math.min((W - 2 * pad) / spanX, (H - 2 * pad) / spanY, 120);
  const cx = (bounds.x0 + bounds.x1) / 2;
  const cy = (bounds.y0 + bounds.y1) / 2;
  const sx = (x) => W / 2 + (x - cx) * scale;
  const sy = (y) => H / 2 - (y - cy) * scale;

  // grid
  const step = niceStep(Math.max(W, H) / scale);
  const gx0 = Math.floor((cx - W / 2 / scale) / step) * step;
  const gy0 = Math.floor((cy - H / 2 / scale) / step) * step;
  for (let x = gx0; sx(x) <= W; x += step) el("line", { x1: sx(x), y1: 0, x2: sx(x), y2: H, class: Math.abs(x) < step / 1e6 ? "axis-line" : "grid-line" }, svg);
  for (let y = gy0; sy(y) >= 0; y += step) el("line", { x1: 0, y1: sy(y), x2: W, y2: sy(y), class: Math.abs(y) < step / 1e6 ? "axis-line" : "grid-line" }, svg);

  const defs = el("defs", {}, svg);
  const marker = el("marker", { id: "arrow", viewBox: "0 0 10 10", refX: "8.5", refY: "5", markerWidth: "7", markerHeight: "7", orient: "auto-start-reverse" }, defs);
  el("path", { d: "M1,1 L9,5 L1,9 Z", class: "marker-dest" }, marker);

  const { positions, round } = history[at];
  if (round) {
    const [px, py] = [sx(round.position[0]), sy(round.position[1])];
    if (config.movement.kind === "non-rigid") el("circle", { cx: px, cy: py, r: config.movement.delta * scale, class: "delta-ring" }, svg);
    const moved = !same(round.position, round.destination);
    if (moved) {
      el("line", { x1: px, y1: py, x2: sx(round.destination[0]), y2: sy(round.destination[1]), class: "mv-planned", "marker-end": "url(#arrow)" }, svg);
      el("line", { x1: px, y1: py, x2: sx(round.stop[0]), y2: sy(round.stop[1]), class: "mv-done" }, svg);
      el("circle", { cx: px, cy: py, r: 7, class: "pt-ghost" }, svg);
      el("circle", { cx: sx(round.destination[0]), cy: sy(round.destination[1]), r: 11, class: "pt-dest" }, svg);
    }
  }

  const showCounts = $("counts").checked;
  for (const g of groups(positions)) {
    const x = sx(g.at[0]);
    const y = sy(g.at[1]);
    const multi = g.ids.length > 1;
    if (multi) {
      el("circle", { cx: x, cy: y, r: 10, class: "pt-multi-ring" }, svg);
      el("circle", { cx: x, cy: y, r: 6, class: "pt-multi" }, svg);
    } else {
      el("circle", { cx: x, cy: y, r: 5.5, class: "pt-single" }, svg);
    }
    const active = round && g.ids.includes(round.robot);
    if (active) el("circle", { cx: x, cy: y, r: multi ? 14 : 10, class: "pt-active" }, svg);
    const names = g.ids.length > 4 ? `r${g.ids[0]}…r${g.ids[g.ids.length - 1]}` : g.ids.map((i) => `r${i}`).join(",");
    const label = el("text", { x: x + (multi ? 14 : 10), y: y - 8, class: `pt-label${active ? " active" : ""}` }, svg);
    label.textContent = names;
    if (multi && showCounts) {
      const count = el("text", { x: x + (multi ? 14 : 10), y: y + 14, class: "pt-count" }, svg);
      count.textContent = `×${g.ids.length}`;
    }
  }
}

function panels() {
  const { positions, round } = history[at];
  const gs = groups(positions);
  $("s_round").textContent = String(at);
  $("s_epoch").textContent = round ? String(round.epoch) : "—";
  $("s_distinct").textContent = String(gs.length);
  $("s_multi").textContent = String(gs.filter((g) => g.ids.length > 1).length);

  const status = $("s_status");
  const last = at === history.length - 1;
  if (gs.length === 1) {
    status.className = "status gathered";
    status.textContent = `Gathered at ${pt(gs[0].at)} after ${at} rounds. Every robot now sees κ = 1, b = 1 and stays.`;
  } else if (last && sim.status() === "incomplete") {
    status.className = "status incomplete";
    status.textContent = JSON.parse(sim.report()).note;
  } else {
    status.className = "status";
    status.textContent = `Running · budget ${config.max_rounds} rounds · ${config.movement.kind === "rigid" ? "rigid moves" : `non-rigid, δ = ${fmt(config.movement.delta)}`}`;
  }

  $("a_robot").textContent = round ? `r${round.robot}` : "";
  $("a_kappa").textContent = round ? String(round.kappa) : "—";
  $("a_b").innerHTML = round
    ? (round.on_multiplicity ? '<span class="chip multi">1 · multiplicity</span>' : '<span class="chip single">0 · single</span>')
    : "—";
  $("a_from").textContent = round ? pt(round.position) : "—";
  $("a_dest").textContent = round ? pt(round.destination) : "—";
  $("a_stop").textContent = round
    ? `${pt(round.stop)} · ${same(round.position, round.destination) ? "stays" : round.reached ? "reached" : `stopped after ${fmt(round.travelled)}`}`
    : "—";
  const path = round ? PATHS[round.rule] : null;
  $("a_rule").innerHTML = round
    ? `<span class="case">${path?.paperCase ?? ""} · lines ${round.lines}</span><span class="rule-key">${round.rule}</span>${round.description}`
    : at === 0 && !finished() ? "Press → or Play to activate the first robot." : "No round to show.";

  const obs = $("obs");
  obs.innerHTML = "";
  if (!round) {
    obs.innerHTML = '<li class="empty">Nothing yet: no robot has looked.</li>';
  } else {
    const showCounts = $("counts").checked;
    round.observation.forEach((o, k) => {
      const li = document.createElement("li");
      const self = k === round.self_index;
      const kind = o.multiplicity ? '<span class="chip multi">multiplicity</span>' : '<span class="chip single">single</span>';
      li.innerHTML = `${kind}<span class="xy">${pt(o.at)}</span>${self ? '<span class="chip self">self</span>' : ""}${showCounts ? `<span class="count">${o.count} robot${o.count === 1 ? "" : "s"}</span>` : ""}`;
      obs.appendChild(li);
    });
  }

  for (const li of $("algo").children) {
    const n = Number(li.dataset.line);
    li.classList.toggle("hit", Boolean(path?.lines.includes(n)));
    li.classList.toggle("dest", path?.dest === n);
  }
}

function appendTraceRow(i) {
  const r = history[i].round;
  const tr = document.createElement("tr");
  tr.dataset.i = String(i);
  tr.innerHTML = `<td>${r.round}</td><td>${r.epoch}</td><td>r${r.robot}</td><td>${r.kappa}</td><td>${r.on_multiplicity ? 1 : 0}</td>`
    + `<td class="rule">${r.rule}</td><td>${r.lines}</td><td>${pt(r.position)}</td><td>${pt(r.destination)}</td><td>${pt(r.stop)}${r.reached ? "" : " ⏸"}</td>`;
  tr.addEventListener("click", () => { pause(); at = i; render(); });
  $("trace").appendChild(tr);
}

// ---------------------------------------------------------------- settings
function fillForm(cfg) {
  $("f_positions").value = cfg.positions.map((p) => `${p[0]},${p[1]}`).join("; ");
  $("f_schedule").value = "preset";
  $("f_movement").value = cfg.movement.kind;
  $("f_delta").value = String(cfg.movement.delta ?? 1);
  const stop = cfg.movement.stop?.kind ?? "delta";
  $("f_stop").value = stop === "fraction" ? "half" : stop;
  $("f_frames").value = cfg.frames?.kind ?? "global";
  $("f_budget").value = String(cfg.max_rounds ?? 10000);
  $("f_seed").value = String(cfg.schedule.seed ?? cfg.frames?.seed ?? 1);
  syncForm();
}

function syncForm() {
  const rigid = $("f_movement").value === "rigid";
  $("f_delta").disabled = rigid;
  $("f_stop").disabled = rigid;
}

function readForm() {
  const positions = $("f_positions").value.split(";").map((s) => s.trim()).filter(Boolean).map((s) => {
    const xy = s.split(",").map(Number);
    if (xy.length !== 2 || !xy.every(Number.isFinite)) throw new Error(`"${s}" is not x,y`);
    return xy;
  });
  const seed = Math.max(0, Math.floor(Number($("f_seed").value) || 0));
  const schedule = { preset: presetSchedule, "round-robin": { kind: "round-robin" }, shuffled: { kind: "shuffled", seed } }[$("f_schedule").value];
  const stopKind = $("f_stop").value;
  const stop = stopKind === "random" ? { kind: "random", seed } : stopKind === "half" ? { kind: "fraction", value: 0.5 } : { kind: "delta" };
  const movement = $("f_movement").value === "rigid" ? { kind: "rigid" } : { kind: "non-rigid", delta: Number($("f_delta").value), stop };
  const frames = $("f_frames").value === "random" ? { kind: "random", seed } : { kind: "global" };
  return { algorithm: "SqGathering", positions, schedule, movement, frames, max_rounds: Math.max(0, Math.floor(Number($("f_budget").value) || 0)) };
}

function apply() {
  try {
    start(readForm());
  } catch (e) {
    $("f_error").textContent = String(e?.message ?? e);
  }
}

function download() {
  const text = history.slice(1).map((h) => h.raw).join("\n") + "\n";
  const a = document.createElement("a");
  a.href = URL.createObjectURL(new Blob([text], { type: "application/x-ndjson" }));
  a.download = "sqgathering-trace.jsonl";
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

// ------------------------------------------------------------------- theme
const media = matchMedia("(prefers-color-scheme: dark)");
const THEMES = ["system", "light", "dark"];
function theme() { try { return localStorage.getItem("lcm-theme") || "system"; } catch { return "system"; } }
function applyTheme(t) {
  if (t === "system") delete document.documentElement.dataset.theme; else document.documentElement.dataset.theme = t;
  const dark = t === "dark" || (t === "system" && media.matches);
  try { localStorage.setItem("lcm-theme", t); } catch {}
  $("theme").innerHTML = icon(dark ? "sun" : "moon");
  $("theme").title = `Theme: ${t}`;
}

// -------------------------------------------------------------------- wire
function wire() {
  renderIcons();
  $("algo").innerHTML = ALGO.map((line, i) => `<li data-line="${i + 1}">${line}</li>`).join("");
  $("preset").innerHTML = PRESETS.map((p) => `<option value="${p.file}">${p.label}</option>`).join("");
  $("preset").addEventListener("change", () => loadPreset($("preset").value).catch(showError));
  $("step").addEventListener("click", () => { pause(); forward(); });
  $("back").addEventListener("click", () => { pause(); backward(); });
  $("restart").addEventListener("click", () => { pause(); at = 0; render(); });
  $("play").addEventListener("click", () => (timer ? pause() : play()));
  $("speed").addEventListener("change", () => { if (timer) { pause(); play(); } });
  $("scrub").addEventListener("input", () => {
    pause();
    const target = Number($("scrub").value);
    while (history.length - 1 < target && forward());
    at = Math.min(target, history.length - 1);
    render();
  });
  $("counts").addEventListener("change", render);
  $("f_movement").addEventListener("change", syncForm);
  $("apply").addEventListener("click", apply);
  $("download").addEventListener("click", download);
  $("theme").addEventListener("click", () => applyTheme(THEMES[(THEMES.indexOf(theme()) + 1) % THEMES.length]));
  media.addEventListener("change", () => { if (theme() === "system") applyTheme("system"); });
  applyTheme(theme());
  addEventListener("resize", () => { if (config) draw(); });
  addEventListener("keydown", (e) => {
    if (e.target.closest("input, textarea, select")) return;
    if (e.key === "ArrowRight") { pause(); forward(); }
    else if (e.key === "ArrowLeft") { pause(); backward(); }
    else if (e.key === "Home") { pause(); at = 0; render(); }
    else if (e.key === " ") { e.preventDefault(); timer ? pause() : play(); }
    else return;
    e.preventDefault();
  });
}

function showError(e) {
  $("s_status").className = "status incomplete";
  $("s_status").textContent = `Could not load: ${e?.message ?? e}`;
}

/** `#preset` or `#preset@round` opens a preset, optionally at a round. */
function openFromHash() {
  const [name, round] = location.hash.slice(1).split("@");
  const file = PRESETS.some((p) => p.file === `${name}.json`) ? `${name}.json` : PRESETS[0].file;
  $("preset").value = file;
  return loadPreset(file).then(() => {
    for (let k = Number(round) || 0; k > 0 && forward(); k--);
  });
}

wire();
loadCore()
  .then(() => {
    addEventListener("hashchange", () => openFromHash().catch(showError));
    return openFromHash();
  })
  .catch(showError);

/* Simulator page: wires the settings panel, toolbar, HUD, legend, inspector
 * and view tools to the worker (client.js) and the renderer.
 *
 * Run states: loading → ready (new run at t = 0, paused) → running ⇄ paused → ended.
 * Changing a setting while ready rebuilds the run at once, so the canvas
 * previews the new swarm; after the run has started it marks the settings
 * "Reset to apply" instead of throwing the run away. */
import { renderIcons, icon } from "./icons.js";
import { COLORS, LOD, Renderer } from "./renderer.js";
import { SimClient } from "./client.js";
import { Inspector } from "./inspector.js";
import { Overview } from "./overview.js";
import { buildSettings, readConfig } from "./settings.js";
import { parseCoordinates, startOutline } from "./start.js";

const $ = (id) => document.getElementById(id);
renderIcons();

const renderer = new Renderer($("canvas"));
// The overview map; clicking it moves the view (which stops following the swarm).
const overview = new Overview($("overview"), $("overview_canvas"), renderer, { onCenter: (x, y) => renderer.centerOn(x, y) });
const client = new SimClient(new URL("./worker.js", import.meta.url));
let algorithms = [];
let state = "loading";
let dirty = false;
let frame = null;
let restartTimer = 0; // a settings change waiting to rebuild the run (0 = none)
let firstFrame = true;

const settings = buildSettings($("settings_body"), onSettingChange);
renderIcons($("settings_body"));
renderIcons($("coords"));
const inspector = new Inspector({
  card: $("inspector"), tip: $("tip"), renderer, client,
  onCenter: (x, y) => renderer.centerOn(x, y),
});

let resolveReady;
window.lcm = { renderer, client, settings, parseCoordinates, stats: {}, ready: new Promise((r) => { resolveReady = r; }),
  get state() { return state; } };

/* ------------------------------------------------------------ run control */
function config() { return readConfig(settings.values, $("algorithm").value, settings.customPoints); }

let fitPending = false; // fit the view to the new run's first frame

/** Builds a new run from the settings; returns false if they can't make one. */
function newRun({ quiet = false } = {}) {
  clearTimeout(restartTimer);
  restartTimer = 0;
  const problem = settings.problem;
  if (problem) {
    if (!quiet) toast(`Fix the coordinates first: line ${problem[0].line}, ${problem[0].message}`, true);
    return false;
  }
  const c = config();
  const v = settings.values;
  renderer.setOutline(v.open_world ? startOutline(v) : { kind: "rect", w: v.width_bound, h: v.height_bound, rotation: 0 });
  fitPending = true;
  inspector.leave(); // the robot under the pointer belonged to the old run
  renderer.options.visibility = c.visibility_radius;
  renderer.options.allowedRegion = c.algorithm === "GoToCenter";
  inspector.select(-1);
  $("hud_last").hidden = true;
  dirty = false;
  $("dirty").hidden = true;
  client.stop();
  updatePlaybackOptions();
  client.speed(playbackSpeed());
  client.start(c);
  setState("ready");
  return true;
}

/** Frame the new run: its robots, plus the world box in World mode. */
function fitToStart(f) {
  const xy = f.xy, n = f.flags.length, v = settings.values;
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (let i = 0; i < n; i++) {
    const x = f.originX + xy[2 * i], y = f.originY + xy[2 * i + 1];
    if (x < minX) minX = x; if (x > maxX) maxX = x; if (y < minY) minY = y; if (y > maxY) maxY = y;
  }
  if (!v.open_world) {
    minX = Math.min(minX, -v.width_bound / 2); maxX = Math.max(maxX, v.width_bound / 2);
    minY = Math.min(minY, -v.height_bound / 2); maxY = Math.max(maxY, v.height_bound / 2);
  }
  const pad = 20;
  let w = maxX - minX, h = maxY - minY;
  const size = Math.max(w, h, 1);
  w = Math.max(w, size * 0.25) + 2 * pad; // a line still gets some height
  h = Math.max(h, size * 0.25) + 2 * pad;
  const region = { w, h, cx: (minX + maxX) / 2, cy: (minY + maxY) / 2 };
  const old = renderer.world;
  const same = Math.abs(old.w - region.w) < old.w * 0.02 && Math.abs(old.h - region.h) < old.h * 0.02
    && Math.abs(old.cx - region.cx) < old.w * 0.02 && Math.abs(old.cy - region.cy) < old.h * 0.02;
  renderer.setView(region.w, region.h, region.cx, region.cy);
  overview.setStart(f);
  if (!same && !renderer.follow) renderer.resetView(); // a new start shape: show all of it; same shape: keep the user's view
}

/* The playback menu: speeds (simulated time per second), and under the sequential scheduler also
 * "seconds per turn", which gives every walk the same time on screen however short it is. */
const TURN_OPTIONS = [["turn:0.25", "0.25 s/turn"], ["turn:1", "1 s/turn"], ["turn:3", "3 s/turn"]];
function playbackSpeed() {
  const v = $("playback").value;
  return v.startsWith("turn:") ? { turn: Number(v.slice(5)) } : Number(v);
}
function updatePlaybackOptions() {
  const select = $("playback");
  const has = select.querySelector('option[value^="turn:"]') !== null;
  if (settings.values.scheduler === "sequential") {
    if (has) return;
    const group = document.createElement("optgroup");
    group.label = "Per turn";
    group.innerHTML = TURN_OPTIONS.map(([value, label]) => `<option value="${value}">${label}</option>`).join("");
    select.append(group);
    select.classList.add("has-turns");
  } else if (has) {
    select.classList.remove("has-turns");
    const wasTurn = select.value.startsWith("turn:");
    select.querySelector("optgroup")?.remove();
    if (wasTurn) { select.value = "20"; client.speed(20); }
  }
}

function onSettingChange() {
  if (state === "loading") return;
  updatePlaybackOptions();
  if (state === "ready") {
    clearTimeout(restartTimer);
    restartTimer = setTimeout(() => newRun({ quiet: true }), 120); // debounce slider drags and typing
  } else {
    dirty = true;
    $("dirty").hidden = false;
  }
}

function setState(next) {
  state = next;
  const running = next === "running";
  const canRun = next === "ready" || next === "paused" || running;
  $("play").disabled = !canRun;
  $("play").innerHTML = `${icon(running ? "pause" : "play")}<span class="label">${running ? "Pause" : "Play"}</span>`;
  $("play").title = running ? "Pause (Space)" : "Play (Space)";
  $("play").setAttribute("aria-label", running ? "Pause" : "Play");
  $("step_event").disabled = !canRun;
  $("step_time").disabled = !canRun;
  $("reset").disabled = next === "loading";
  $("algorithm").disabled = next === "loading";
  updateHud(true);
}

function play() {
  // A setting was just changed: play the run it describes (not if it is invalid).
  if (restartTimer && !newRun()) return;
  if (state === "ready" || state === "paused") { client.play(); setState("running"); }
  else if (state === "running") { client.pause(); setState("paused"); }
}

function step(unit) {
  if (restartTimer && !newRun()) return;
  if (!(state === "ready" || state === "paused" || state === "running")) return;
  client.step(unit, 1);
  setState("paused");
}

/* ---------------------------------------------------------------- examples */
// Ready-made setups, per algorithm. `set` is applied to the settings in the order written.
const CHAIN = "-80, 10\n0, 0\n95, -5";
const TWO_GROUPS = "-170, -10\n-150, 15\n-135, -20\n-160, 30\n150, -25\n165, 10\n135, 20\n170, -15";
const GTC_SEQUENTIAL = { scheduler: "sequential", turn_gap: "none", rigid_movement: true, robot_speeds: 25 };
const EXAMPLES = {
  GoToCenter: [
    { label: "Three robots in a chain", note: "Limited sight: each robot sees only its neighbour", links: true,
      set: { open_world: true, pattern: "custom", custom: CHAIN, unlimited_visibility: false, visibility_radius: 100, ...GTC_SEQUENTIAL } },
    { label: "A swarm of 40", note: "Limited sight (110), links drawn", links: true,
      set: { open_world: true, pattern: "cloud", num_of_robots: 40, spread: 70, unlimited_visibility: false, visibility_radius: 110, ...GTC_SEQUENTIAL } },
    { label: "Two groups", note: "Out of sight of each other", links: true,
      set: { open_world: true, pattern: "custom", custom: TWO_GROUPS, unlimited_visibility: false, visibility_radius: 100, ...GTC_SEQUENTIAL } },
  ],
};
// Choosing an example overwrites several settings. "None" puts back what they were before the first
// example was chosen (`before`), so trying examples never loses the setup the user had.
let activeExample = -1;
let before = null; // { values, links }
function setLinks(on) {
  renderer.options.links = on;
  $("links").setAttribute("aria-pressed", String(on));
}
function renderExamplesMenu() {
  const list = EXAMPLES[$("algorithm").value] ?? [];
  const items = [{ label: "None", note: "Your own settings" }, ...list];
  $("examples_menu").innerHTML = items.map((e, k) => {
    const on = k - 1 === activeExample;
    return `<button class="btn menu-item" role="menuitemradio" aria-checked="${on}" data-example="${k - 1}">`
      + `<span class="tick" data-icon="check" data-size="16"></span>`
      + `<span class="text"><span class="title">${e.label}</span><span class="note">${e.note}</span></span></button>`;
  }).join("");
  renderIcons($("examples_menu"));
  $("examples").setAttribute("aria-pressed", String(activeExample >= 0));
}
function updateExamples() {
  const list = EXAMPLES[$("algorithm").value];
  activeExample = -1; // another algorithm: the old choice no longer means anything
  before = null;
  openExamples(false);
  $("examples").hidden = !list;
  if (list) renderExamplesMenu();
  updateInsets(); // the toolbar may have wrapped onto another row
}
function openExamples(open) {
  const menu = $("examples_menu"), button = $("examples");
  menu.hidden = !open;
  button.setAttribute("aria-expanded", String(open));
  if (!open) return;
  const r = button.getBoundingClientRect();
  menu.style.top = `${Math.round(r.bottom + 6)}px`;
  menu.style.left = `${Math.max(8, Math.min(Math.round(r.left), innerWidth - menu.offsetWidth - 8))}px`;
  (menu.querySelector("[aria-checked=true]") ?? menu.querySelector("[role=menuitemradio]"))?.focus();
}
function chooseExample(index) {
  const list = EXAMPLES[$("algorithm").value] ?? [];
  if (index < 0) {
    if (before) { setLinks(before.links); settings.apply(before.values); }
    before = null;
  } else if (list[index]) {
    const example = list[index];
    before ??= {
      links: renderer.options.links,
      values: Object.fromEntries([...new Set(list.flatMap((e) => Object.keys(e.set)))].map((key) => [key, settings.values[key]])),
    };
    setLinks(example.links);
    settings.apply(example.set);
  } else return;
  activeExample = index;
  renderExamplesMenu();
}
$("examples").addEventListener("click", (e) => { e.stopPropagation(); openExamples($("examples_menu").hidden); });
$("examples_menu").addEventListener("click", (e) => {
  e.stopPropagation();
  const item = e.target.closest("[data-example]");
  if (!item) return;
  chooseExample(Number(item.dataset.example));
  openExamples(false);
  $("examples").focus();
});
$("examples_menu").addEventListener("keydown", (e) => {
  const items = [...$("examples_menu").querySelectorAll("[role=menuitemradio]")];
  const at = items.indexOf(document.activeElement);
  const to = { ArrowDown: at + 1, ArrowUp: at - 1, Home: 0, End: items.length - 1 }[e.key];
  if (to === undefined) return;
  e.preventDefault();
  items[(to + items.length) % items.length].focus();
});

client.on("ready", (list) => {
  algorithms = list;
  $("algorithm").innerHTML = list.map((a) => `<option value="${a.key}">${a.name}</option>`).join("");
  updateExamples();
  $("loading_text").textContent = "Building the swarm…";
  newRun();
});

/* ------------------------------------------------------- swarm-size trace */
// The worker samples "how far the farthest robot is from the swarm's centre" as the run advances
// and sends the points with each frame (time, size, time, size, ...).
const trace = { points: new Float64Array(0) };
window.lcm.trace = trace;
window.lcm.overview = overview;
function drawTrace() {
  const canvas = $("hud_plot"), ctx = canvas.getContext("2d"), W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);
  const flat = trace.points, count = flat.length / 2;
  if (count < 1) { $("hud_spread").textContent = "—"; return; }
  const times = Array.from({ length: count }, (_, k) => flat[2 * k]);
  const sizes = Array.from({ length: count }, (_, k) => flat[2 * k + 1]);
  $("hud_spread").textContent = fmtSize(sizes[count - 1]);
  if (count < 2) return;
  const ys = sizes.map((v) => Math.log10(Math.max(v, 1e-7)));
  const lo = Math.min(...ys), hi = Math.max(...ys, lo + 0.5), t1 = times[count - 1] || 1;
  const css = getComputedStyle(document.documentElement);
  ctx.strokeStyle = css.getPropertyValue("--accent").trim() || "#2f5bea";
  ctx.lineWidth = 2.5; ctx.lineJoin = "round";
  ctx.beginPath();
  times.forEach((t, k) => { const x = 6 + ((W - 12) * t) / t1, y = H - 8 - ((H - 16) * (ys[k] - lo)) / (hi - lo); k ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
  ctx.stroke();
}
const fmtSize = (v) => (v >= 100 ? v.toFixed(0) : v >= 1 ? v.toFixed(1) : v >= 1e-3 ? v.toFixed(3) : v.toExponential(1));

/* Playing a turn at a time: say, in plain words, whose turn it is and what the algorithm does. */
const RULES = {
  Gathering: "It goes to the average position of all the robots it sees (the centre of gravity).",
  SEC: "It goes to the nearest point of the smallest circle around the robots it sees.",
  GoToCenter: "It heads for the centre of the smallest circle around the robots it sees, but never so far that it could lose sight of a neighbour.",
};
const fmtNumber = (v) => (v === 0 ? "0" : v >= 1e-3 ? String(+v.toPrecision(3)) : v.toExponential(2));
let endNote = false;
function showEndNote(on) {
  endNote = on;
  const el = $("turn_caption");
  el.hidden = !on;
  if (!on) return;
  const points = trace.points, size = points.length ? points[points.length - 1] : 0;
  const zoomed = renderer.zoom > 20 ? ` This view is zoomed in ${Math.round(renderer.zoom).toLocaleString()} times; the grid and the scale bar show the real size.` : "";
  const html = `<div class="cap-head">Finished: every robot has terminated</div><div class="cap-body">Grey robots have terminated. The swarm is ${fmtSize(size)} units across (from its centre to its farthest robot).${zoomed}</div>`;
  if (el.innerHTML !== html) el.innerHTML = html;
}
function updateCaption(f) {
  const el = $("turn_caption"), w = f.walk;
  if (!w) {
    // A run that is over, in a view that is zoomed in on the swarm or played a turn at a time: say so, because
    // grey robots a long way apart on screen can look like a failure when they are a hair apart in fact.
    showEndNote(f.ended && (renderer.follow || typeof playbackSpeed() === "object"));
    return;
  }
  el.hidden = false;
  endNote = false;
  const who = `<strong>Turn ${(f.turns ?? 0).toLocaleString()}</strong> · Robot ${w.robot}`;
  const rule = RULES[$("algorithm").value] ?? "";
  el.innerHTML = w.phase === "decide"
    ? `<div class="cap-head">${who} looks and decides</div><div class="cap-body">${rule || "It looks at the others and picks its destination, the ✕."}</div>`
    : `<div class="cap-head">${who} moves</div><div class="cap-body">It walks ${fmtNumber(w.length)} units to the ✕.${w.cut ? " The adversary stops it short of its destination (non-rigid movement)." : ""}</div>`;
}

client.on("frame", (f) => {
  frame = f;
  renderer.options.walk = f.walk ?? null;
  if (f.trace) trace.points = f.trace;
  updateCaption(f);
  if (fitPending) { fitPending = false; fitToStart(f); }
  renderer.setFrame(f);
  if (firstFrame) { firstFrame = false; $("loading").hidden = true; resolveReady(); }
  if (f.lastEvent) showLastEvent(f.lastEvent);
  window.lcm.frames = (window.lcm.frames ?? 0) + 1;
  if (f.stop !== "running" && state !== "ended" && state !== "loading") {
    client.stopPulling(); // the run is over: nothing new to fetch or draw
    setState("ended");
  }
  inspector.refresh();
  updateHud(state !== "running");
});

client.on("error", (message) => {
  if (/unknown field|unknown variant|missing field/.test(message)) {
    // The core and the page come from different builds (a stale cache).
    $("loading_text").textContent = "The page and the simulator core are from different versions.";
    toast("The simulator files are out of date in your browser. Reload the page (Ctrl + Shift + R).", true);
    if (state === "running") setState("paused");
    return;
  }
  $("loading_text").textContent = "Could not start the simulation.";
  toast(message.includes("pkg") || message.includes("worker") ? `${message} — build the core with ./scripts/build-wasm.sh` : message, true);
  if (state === "running") setState("paused");
});

/* ------------------------------------------------------------------- HUD */
let hudAt = 0, rateFrom = null, rate = 0, draws = 0, fps = 0, fpsAt = performance.now();
let legendAt = 0;
renderer.onDraw = () => {
  draws++;
  updateScale();
  updateZoomRows();
  overview.draw();
  if (endNote) showEndNote(true);  // the view is still settling after the run ends: keep its zoom up to date
  const now = performance.now();
  if (state !== "running" || now - legendAt > 250) { legendAt = now; updateLegend(); }
};

/** The scale bar: the nearest 1, 2 or 5 × 10^k simulation units that makes a bar of about 90 px. */
const SUPERSCRIPT = { "-": "⁻", 0: "⁰", 1: "¹", 2: "²", 3: "³", 4: "⁴", 5: "⁵", 6: "⁶", 7: "⁷", 8: "⁸", 9: "⁹" };
const fmtLength = (v) => {
  if (v >= 1e-3) return String(+v.toPrecision(1));
  const [m, e] = v.toExponential(0).split("e");
  return `${m}×10${[...String(Number(e))].map((c) => SUPERSCRIPT[c]).join("")}`;
};
let scaleShown = "";
function updateScale() {
  const s = renderer.scale;
  if (!(s > 0) || !isFinite(s)) return;
  const want = 90 / s, pow = 10 ** Math.floor(Math.log10(want)), m = want / pow;
  const len = (m >= 5 ? 5 : m >= 2 ? 2 : 1) * pow;
  const px = Math.round(len * s), label = fmtLength(len);
  const key = `${px}|${label}`;
  if (key === scaleShown) return;
  scaleShown = key;
  $("hud_scale_bar").style.width = `${px}px`;
  $("hud_scale").textContent = label;
  $("hud_scale").dataset.value = String(len);
}

/** "1.3×", "230×", "4.5k×", "30.1M×": how far the view is zoomed in, kept short for the panel. */
function fmtZoom(z) {
  if (z < 10) return `${z.toFixed(2)}×`;
  if (z < 1e3) return `${z.toFixed(z < 100 ? 1 : 0)}×`;
  const units = [[1e12, "T"], [1e9, "G"], [1e6, "M"], [1e3, "k"]];
  for (const [size, suffix] of units) if (z >= size) return `${(z / size).toFixed(1)}${suffix}×`;
  return `${z}×`;
}
/** The zoom gauge (logarithmic, 1× to a billion ×) and the distance between the labelled grid lines. */
let zoomShown = "";
function updateZoomRows() {
  const z = renderer.zoom, text = fmtZoom(z), grid = renderer.gridState;
  const key = `${text}|${grid?.spacing ?? ""}`;
  if (key === zoomShown) return;
  zoomShown = key;
  $("hud_zoom").textContent = text;
  $("hud_zoom_fill").style.width = `${Math.max(0, Math.min(1, Math.log10(Math.max(z, 1)) / 9)) * 100}%`;
  $("hud_grid").textContent = grid && renderer.options.grid ? fmtLength(grid.spacing) : "off";
}

function updateHud(force = false) {
  const now = performance.now();
  if (!force && now - hudAt < 200) return;
  hudAt = now;
  if (now - fpsAt >= 1000) { fps = (draws * 1000) / (now - fpsAt); draws = 0; fpsAt = now; }
  const statusText = {
    loading: "Loading…", ready: "Ready", running: "Running", paused: "Paused",
    ended: frame?.ended ? "Ended · all terminated"
      : frame?.stop === "stalled" ? "Stalled · nothing can move any more"
      : frame?.stop === "max-turns" ? "Stopped · turn limit reached" : `Stopped (${frame?.stop})`,
  }[state];
  $("hud_status").textContent = statusText;
  $("hud_status").className = `hud-status${state === "ended" ? " ended" : ""}`;
  if (!frame) return;
  if (rateFrom && state === "running") {
    const dt = (now - rateFrom.wall) / 1000;
    if (dt > 0) rate = 0.6 * rate + 0.4 * ((frame.events - rateFrom.events) / dt);
  } else if (state !== "running") rate = 0;
  rateFrom = { wall: now, events: frame.events };
  $("hud_time").textContent = frame.time.toFixed(2);
  $("hud_term").textContent = `${frame.terminated.toLocaleString()} / ${frame.robots.toLocaleString()}`;
  $("hud_bar").style.width = `${(100 * frame.terminated) / Math.max(1, frame.robots)}%`;
  drawTrace();
  $("hud_epochs_row").hidden = frame.epochs == null;
  if (frame.epochs != null) {
    $("hud_epochs").textContent = frame.epochs.toLocaleString();
    $("hud_turns").textContent = (frame.turns ?? 0).toLocaleString();
  }
  $("hud_events").textContent = (frame.events - frame.ticks).toLocaleString();
  $("hud_ticks").textContent = frame.ticks.toLocaleString();
  $("hud_rate").textContent = state === "running" ? Math.round(rate).toLocaleString() : "—";
  const lod = frame.robots > LOD.outlines ? " · simplified drawing" : "";
  $("hud_perf").textContent = state === "running"
    ? `Draw ${renderer.lastDrawMs.toFixed(1)} ms · ${Math.round(fps)} fps · worker ${Math.round(frame.busy * 100)} %${lod}`
    : `Draw ${renderer.lastDrawMs.toFixed(1)} ms${lod}`;
  window.lcm.stats = {
    time: frame.time, events: frame.events, ticks: frame.ticks, eventsPerSecond: Math.round(rate), robots: frame.robots,
    terminated: frame.terminated, epochs: frame.epochs, fps: Math.round(fps), drawMs: renderer.lastDrawMs, busy: frame.busy, stop: frame.stop, state,
  };
}
setInterval(() => { if (state === "running") updateHud(); }, 250);

function showLastEvent(e) {
  const el = $("hud_last");
  el.hidden = false;
  el.textContent = e.robot >= 0
    ? `t ${e.time.toFixed(3)} · robot ${e.robot} · ${e.kind} → ${e.outcome}`
    : `t ${e.time.toFixed(3)} · ${e.kind ?? "—"} → ${e.outcome}`;
}

function updateLegend() {
  const counts = renderer.counts;
  const items = COLORS.map((c, i) => [c, counts[i]]).filter(([, n]) => n > 0);
  $("legend").innerHTML = items.map(([c, n]) =>
    `<span class="item"><span class="dot" style="background:${c.fill}"></span>${c.label} <span class="count">${n.toLocaleString()}</span></span>`).join("")
    || `<span class="item">No robots</span>`;
}

/* ------------------------------------------------------------- toolbar */
$("play").addEventListener("click", play);
$("step_event").addEventListener("click", () => step("event"));
$("step_time").addEventListener("click", () => step("time"));
$("reset").addEventListener("click", () => newRun());
$("algorithm").addEventListener("change", () => { updateExamples(); onSettingChange(); });
$("playback").addEventListener("change", () => {
  const speed = playbackSpeed();
  client.speed(speed);
  if (typeof speed !== "object") {  // not played a turn at a time any more: drop the turn caption and marks at once
    $("turn_caption").hidden = true;
    renderer.options.walk = null;
    renderer.requestDraw();
  }
});
$("defaults").addEventListener("click", () => settings.reset());
$("toggle_settings").addEventListener("click", toggleSettings);
$("close_settings").addEventListener("click", toggleSettings);
// Phones (and short screens) open on the swarm; settings is one tap away.
if (innerWidth <= 760 || innerHeight <= 520) { $("settings").hidden = true; $("toggle_settings").setAttribute("aria-pressed", "false"); }
function toggleSettings() {
  const panel = $("settings");
  panel.hidden = !panel.hidden;
  $("toggle_settings").setAttribute("aria-pressed", String(!panel.hidden));
  updateInsets();
}

/** Tell the renderer which parts of the window the floating panels cover,
 * and let the CSS stack panels under the toolbar and stats at any width. */
function updateInsets() {
  const visible = (id) => { const el = $(id); return !el.hidden && getComputedStyle(el).display !== "none" ? el.getBoundingClientRect() : null; };
  const root = document.documentElement.style;
  const toolbar = visible("toolbar");
  root.setProperty("--top-h", `${Math.round(toolbar ? toolbar.bottom : 0)}px`);
  const hud = visible("hud");
  root.setProperty("--hud-bottom", `${Math.round(hud ? hud.bottom : 0)}px`);
  const settingsRect = visible("settings"), tools = visible("tools"), legend = visible("legend");
  // The overview map sits just left of the view tools, in the bottom right corner.
  if (tools && tools.height > tools.width) $("overview").style.right = `${Math.round(innerWidth - tools.left + 8)}px`;
  if (innerWidth <= 760) {
    // Phones: the settings sheet floats over the view instead of narrowing it.
    const bottomEdge = Math.min(tools ? tools.top : innerHeight, legend ? legend.top : innerHeight);
    renderer.setInsets({ left: 0, right: 0, top: Math.max(toolbar ? toolbar.bottom : 0, hud ? hud.bottom : 0), bottom: innerHeight - bottomEdge });
    return;
  }
  renderer.setInsets({
    left: settingsRect ? settingsRect.right : 0,
    right: Math.max(hud ? innerWidth - hud.left : 0, tools && tools.height > tools.width ? innerWidth - tools.left : 0),
    top: toolbar ? toolbar.bottom : 0,
    bottom: tools && tools.width >= tools.height ? innerHeight - tools.top : 56,
  });
}

/* ------------------------------------------------------------ view tools */
$("zoom_in").addEventListener("click", () => renderer.zoomAt(1.25));
$("zoom_out").addEventListener("click", () => renderer.zoomAt(0.8));
$("fit").addEventListener("click", () => renderer.resetView());
$("grid").addEventListener("click", toggleGrid);
function toggleOverview() {
  $("overview").hidden = !$("overview").hidden;
  updateInsets();
  renderer.requestDraw();
}
function toggleGrid() {
  renderer.options.grid = !renderer.options.grid;
  $("grid").setAttribute("aria-pressed", String(renderer.options.grid));
  renderer.requestDraw();
}
$("follow").addEventListener("click", toggleFollow);
function toggleFollow() { renderer.setFollow(!renderer.follow); }
renderer.onFollowChange = (on) => $("follow").setAttribute("aria-pressed", String(on));
$("fullscreen").addEventListener("click", toggleFullscreen);
function toggleFullscreen() {
  const doc = document, el = doc.documentElement;
  const active = doc.fullscreenElement || doc.webkitFullscreenElement;
  const request = el.requestFullscreen?.bind(el) ?? el.webkitRequestFullscreen?.bind(el);
  const exit = doc.exitFullscreen?.bind(doc) ?? doc.webkitExitFullscreen?.bind(doc);
  if (!request) { toast("Full screen is not available in this browser", true); return; }
  Promise.resolve(active ? exit() : request()).catch(() => toast("The browser refused full screen", true));
}
function onFullscreenChange() {
  const on = Boolean(document.fullscreenElement || document.webkitFullscreenElement);
  $("fullscreen").innerHTML = icon(on ? "exitFullscreen" : "fullscreen");
  $("fullscreen").setAttribute("aria-pressed", String(on));
  $("fullscreen").title = on ? "Exit full screen (Shift + F or Esc)" : "Full screen (Shift + F)";
  // The window just changed size: re-measure panels and keep the world fitted.
  requestAnimationFrame(() => { renderer.resize(); updateInsets(); });
}
document.addEventListener("fullscreenchange", onFullscreenChange);
document.addEventListener("webkitfullscreenchange", onFullscreenChange);
$("lights").addEventListener("click", toggleLights);
$("links").addEventListener("click", toggleLinks);
function toggleLinks() {
  renderer.options.links = !renderer.options.links;
  $("links").setAttribute("aria-pressed", String(renderer.options.links));
  renderer.requestDraw();
}
function toggleLights() {
  renderer.options.showLights = !renderer.options.showLights;
  $("lights").setAttribute("aria-pressed", String(renderer.options.showLights));
  renderer.requestDraw();
}

$("export").addEventListener("click", (e) => { e.stopPropagation(); openExamples(false); $("export_menu").hidden = !$("export_menu").hidden; });
document.addEventListener("click", () => { $("export_menu").hidden = true; openExamples(false); });
for (const b of document.querySelectorAll("[data-export]")) {
  b.addEventListener("click", () => (b.dataset.export === "png" ? exportPng() : exportCsv()));
}

function fileStem() {
  const c = config();
  const t = frame ? frame.time.toFixed(2) : "0";
  return `lcm-${c.algorithm.toLowerCase()}-${frame?.robots ?? c.num_of_robots}robots-t${t}`;
}
function download(blob, name) {
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}
async function exportPng() {
  const name = algorithms.find((a) => a.key === $("algorithm").value)?.name ?? "";
  const caption = `LCM simulator · ${name} · ${(frame?.robots ?? 0).toLocaleString()} robots · t = ${(frame?.time ?? 0).toFixed(2)} · seed ${settings.values.random_seed}`;
  download(await renderer.toPngBlob(caption), `${fileStem()}.png`);
  toast("Saved the view as PNG");
}
async function exportCsv() {
  const { text } = await client.csv();
  download(new Blob([text], { type: "text/csv" }), `${fileStem()}.csv`);
  toast(`Saved ${(frame?.robots ?? 0).toLocaleString()} robots as CSV`);
}

/* ------------------------------------------------------------------ theme */
const THEMES = ["system", "light", "dark"];
const media = matchMedia("(prefers-color-scheme: dark)");
function currentTheme() { try { return localStorage.getItem("lcm-theme") || "system"; } catch { return "system"; } }
function applyTheme(t) {
  if (t === "system") delete document.documentElement.dataset.theme;
  else document.documentElement.dataset.theme = t;
  const dark = t === "dark" || (t === "system" && media.matches);
  document.documentElement.dataset.resolvedTheme = dark ? "dark" : "light";
  try {
    localStorage.setItem("lcm-theme", t);
    localStorage.setItem("darkMode", String(dark)); // shared with docs.html
  } catch {}
  $("theme").innerHTML = icon(dark ? "sun" : "moon");
  $("theme").title = `Theme: ${t} (T)`;
  renderer.readTheme();
}
function cycleTheme() { applyTheme(THEMES[(THEMES.indexOf(currentTheme()) + 1) % THEMES.length]); }
$("theme").addEventListener("click", cycleTheme);
media.addEventListener("change", () => { if (currentTheme() === "system") applyTheme("system"); });
applyTheme(currentTheme());

/* ---------------------------------------------- canvas: pan, zoom, pick */
const canvas = $("canvas");
let drag = null;
canvas.addEventListener("pointerdown", (e) => {
  drag = { x: e.clientX, y: e.clientY, moved: false };
  canvas.setPointerCapture(e.pointerId);
});
canvas.addEventListener("pointermove", (e) => {
  const rect = canvas.getBoundingClientRect();
  if (drag) {
    const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
    if (drag.moved || Math.hypot(dx, dy) > 3) {
      drag.moved = true;
      canvas.classList.add("panning");
      renderer.panBy(dx, dy);
      drag.x = e.clientX; drag.y = e.clientY;
      inspector.leave();
    }
    return;
  }
  inspector.hover(e.clientX - rect.left, e.clientY - rect.top, e.clientX, e.clientY);
});
canvas.addEventListener("pointerup", (e) => {
  const rect = canvas.getBoundingClientRect();
  if (drag && !drag.moved) {
    const i = renderer.pick(e.clientX - rect.left, e.clientY - rect.top);
    inspector.select(i);
  }
  drag = null;
  canvas.classList.remove("panning");
});
canvas.addEventListener("pointerleave", () => inspector.leave());
canvas.addEventListener("wheel", (e) => {
  e.preventDefault();
  const rect = canvas.getBoundingClientRect();
  renderer.zoomAt(Math.exp(-e.deltaY * 0.0015), e.clientX - rect.left, e.clientY - rect.top);
}, { passive: false });
new ResizeObserver(() => { renderer.resize(); updateInsets(); }).observe($("stage"));
updateInsets();
renderer.resetView();

/* -------------------------------------------------------------- shortcuts */
function openShortcuts(open) { $("shortcuts").hidden = !open; if (open) $("shortcuts").querySelector("button").focus(); }
$("help").addEventListener("click", () => openShortcuts(true));
$("shortcuts").addEventListener("click", (e) => { if (e.target === e.currentTarget || e.target.closest("[data-action=close]")) openShortcuts(false); });

window.addEventListener("keydown", (e) => {
  if (e.key === "Escape") {
    if (!$("shortcuts").hidden) openShortcuts(false);
    else if (!$("export_menu").hidden) $("export_menu").hidden = true;
    else if (!$("examples_menu").hidden) { openExamples(false); $("examples").focus(); }
    else inspector.select(-1);
    return;
  }
  const typing = e.target.closest?.("input, select, textarea");
  if (settings.editor.isOpen || typing || e.ctrlKey || e.metaKey || e.altKey) return;
  // Space/Enter on a focused button already clicks it; don't toggle twice.
  if ((e.key === " " || e.key === "Enter") && e.target.closest?.("button, a, summary")) return;
  const actions = {
    " ": play,
    ArrowRight: () => step(e.shiftKey ? "time" : "event"),
    r: newRun, R: newRun,
    f: () => renderer.resetView(), F: toggleFullscreen,
    z: toggleFollow, Z: toggleFollow,
    g: toggleGrid, G: toggleGrid,
    o: toggleOverview, O: toggleOverview,
    "+": () => renderer.zoomAt(1.25), "=": () => renderer.zoomAt(1.25), "-": () => renderer.zoomAt(0.8),
    l: toggleLights, L: toggleLights,
    v: toggleLinks, V: toggleLinks,
    s: toggleSettings, S: toggleSettings,
    h: toggleClean, H: toggleClean,
    e: exportPng, E: exportCsv,
    t: cycleTheme, T: cycleTheme,
    "?": () => openShortcuts(true),
  };
  const action = actions[e.key];
  if (action) { e.preventDefault(); action(); }
});

function toggleClean() { document.body.classList.toggle("clean"); updateInsets(); }

/* ------------------------------------------------------------------ toast */
let toastTimer = 0;
function toast(message, error = false) {
  const t = $("toast");
  t.textContent = message;
  t.className = error ? "error" : "";
  t.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { t.hidden = true; }, error ? 8000 : 2500);
}

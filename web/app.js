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
import { buildSettings, readConfig } from "./settings.js";
import { parseCoordinates, startOutline } from "./start.js";

const $ = (id) => document.getElementById(id);
renderIcons();

const renderer = new Renderer($("canvas"));
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
  inspector.select(-1);
  $("hud_last").hidden = true;
  dirty = false;
  $("dirty").hidden = true;
  client.stop();
  client.speed(Number($("playback").value));
  client.start(c);
  setState("ready");
  return true;
}

/** Frame the new run: its robots, plus the world box in World mode. */
function fitToStart(f) {
  const xy = f.xy, n = f.flags.length, v = settings.values;
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (let i = 0; i < n; i++) {
    const x = xy[2 * i], y = xy[2 * i + 1];
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
  if (!same) renderer.resetView(); // a new start shape: show all of it; same shape: keep the user's view
}

function onSettingChange() {
  if (state === "loading") return;
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

client.on("ready", (list) => {
  algorithms = list;
  $("algorithm").innerHTML = list.map((a) => `<option value="${a.key}">${a.name}</option>`).join("");
  $("loading_text").textContent = "Building the swarm…";
  newRun();
});

client.on("frame", (f) => {
  frame = f;
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
  const now = performance.now();
  if (state !== "running" || now - legendAt > 250) { legendAt = now; updateLegend(); }
};

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
  $("hud_epochs_row").hidden = frame.epochs == null;
  if (frame.epochs != null) $("hud_epochs").textContent = frame.epochs.toLocaleString();
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
$("algorithm").addEventListener("change", onSettingChange);
$("playback").addEventListener("change", (e) => client.speed(Number(e.target.value)));
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
function toggleLights() {
  renderer.options.showLights = !renderer.options.showLights;
  $("lights").setAttribute("aria-pressed", String(renderer.options.showLights));
  renderer.requestDraw();
}

$("export").addEventListener("click", (e) => { e.stopPropagation(); $("export_menu").hidden = !$("export_menu").hidden; });
document.addEventListener("click", () => { $("export_menu").hidden = true; });
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
    "+": () => renderer.zoomAt(1.25), "=": () => renderer.zoomAt(1.25), "-": () => renderer.zoomAt(0.8),
    l: toggleLights, L: toggleLights,
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

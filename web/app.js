/* Simulator page: wires the settings panel, toolbar, HUD, legend, inspector
 * and view tools to the worker (client.js) and the renderer.
 *
 * Run states: loading → ready (new run at t = 0, paused) → running ⇄ paused → ended.
 * Changing a setting while ready rebuilds the run at once, so the canvas
 * previews the new swarm; after the run has started it marks the settings
 * "Reset to apply" instead of throwing the run away. */
import { renderIcons, icon } from "./icons.js";
import { COLORS, LOD, Renderer, fmtDistance, ulpOf } from "./renderer.js";
import { SimClient } from "./client.js";
import { Inspector } from "./inspector.js";
import { Overview } from "./overview.js";
import { SETTINGS, buildSettings, readConfig } from "./settings.js";
import { EXPLAINED, explain } from "./explain.js";
import { ADVANCED, MENU_NAMES, menuHtml } from "./menu.js";
import { parseCoordinates, startOutline } from "./start.js";
import { Adversary } from "./adversary.js";
import { ScenarioEditor } from "./editor.js";
import { buildEmbedDialog, copyText, linkQuery, pageAddress, readLink, setUpEmbed, whenFirstSeen } from "./embed.js";

const $ = (id) => document.getElementById(id);
renderIcons();
// A run described by the page's link, and the small embed version of the page (embed.js).
const link = readLink(location.search);
const embedded = link.embed;
if (embedded) setUpEmbed(link);

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

// Info cards under a setting: what the choice means, given the algorithm chosen now.
const settings = buildSettings($("settings_body"), onSettingChange, {
  explained: EXPLAINED,
  explain: (key) => {
    const k = algorithmKey(), a = algorithms.find((x) => x.key === k);
    return explain(key, settings.values, a && { key: k, name: a.name, multiplicities: a.multiplicities, sequentialOnly: a.sequential_only });
  },
});
if (Object.keys(link.settings).length) settings.apply(link.settings);
renderIcons($("settings_body"));
renderIcons($("coords"));
const inspector = new Inspector({
  card: $("inspector"), tip: $("tip"), renderer, client,
  onCenter: (x, y) => renderer.centerOn(x, y),
});

const adversary = new Adversary({
  renderer, client, settings, panel: $("adversary_panel"), toast,
  canAct: () => state === "ready" || state === "paused",
  onPlay: () => setState("running"),
});
adversary.onLeave = () => setAdversary(false);
const editor = new ScenarioEditor({
  renderer, settings, panel: $("editor_panel"), toast,
  rebuild: () => newRun({ quiet: true, keepView: true }),
});
editor.onAction = (action) => {
  if (action === "done") setEditor(false);
  else if (action === "save") exportScenario();
  else if (action === "link") copyCaseLink();
};

/** The scenario editor edits the start: it rebuilds the run (paused at t = 0) on every edit. */
function setEditor(on) {
  if (on && adversary.on) setAdversary(false);
  if (on && state !== "ready" && !newRun({ quiet: true, keepView: true })) return;
  if (on && state === "running") { client.pause(); setState("paused"); }
  if (!editor.setOn(on)) return;
  $("edit").setAttribute("aria-pressed", String(on));
  $("canvas").classList.toggle("editing", on);
  if (on && measure.on) setMeasure(false);
  if (on) toast("Scenario editor: click to add robots, drag to move them");
}

/** Playing the adversary needs the sequential or SSYNC scheduler; it starts a new run that waits for each choice. */
function setAdversary(on) {
  if (on && editor.on) setEditor(false);
  if (on && !adversary.supported) {
    settings.set("scheduler", "sequential", false);
    toast("Playing the adversary chooses turns: Scheduler set to Sequential (SSYNC works too)");
  }
  adversary.setOn(on);
  $("adversary").setAttribute("aria-pressed", String(on));
  $("canvas").classList.toggle("adversary", on);
  if (on && measure.on) setMeasure(false);
  if (state !== "loading") newRun({ quiet: true, keepView: true });
}

/* --------------------------------------------------------- algorithm brief */
// The box under the toolbar: what the chosen algorithm does, the robots and model this run uses (settings the
// algorithm fixes carry a lock), and, from the core with the run's first frame, where the settings leave what
// the algorithm was made for. It folds to one line while a run plays; a click opens it again.
const brief = { notes: [], open: true, seen: new Set(), opening: false }; // seen: notes already on screen (only new ones animate)
const esc = (t) => String(t).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);
const SCHEDULER_WORDS = { async: "Async", sequential: "One at a time", ssync: "Some at once · SSYNC", fsync: "All at once · FSYNC", kasync: "k-ASYNC" };
const MULTIPLICITY_WORDS = { none: "Can't see stacks", local: "Knows if its own spot is shared", weak: "Sees stacks", strong: "Counts stacks" };
const NOTE_KINDS = [
  [/^Outside the model:\s*/, "warn", "Outside what it was built for"],
  [/^Experimental:\s*/, "caution", "Untested ground"],
  [/^(Approximation|Simplification):\s*/, "info", "Simplified"],
];
/** The chips: [text, setting key or null]; a key the algorithm locks gets the lock. */
function briefChips(v, key) {
  const a = algorithms.find((x) => x.key === key);
  const n = v.pattern === "custom" ? (settings.customPoints?.length ?? v.num_of_robots) : v.num_of_robots;
  const chips = [[`${n} robot${n === 1 ? "" : "s"}`, "num_of_robots"], [SCHEDULER_WORDS[v.scheduler] ?? v.scheduler, "scheduler"]];
  chips.push([v.unlimited_visibility ? "Sees everyone" : `Sight ${v.visibility_radius}`, "unlimited_visibility"]);
  if (v.obstructed_visibility) chips.push(["Robots block the view", "obstructed_visibility"]);
  if (a?.multiplicities || LOCKS[key]?.multiplicity) chips.push([MULTIPLICITY_WORDS[v.multiplicity], "multiplicity"]);
  if (v.frames === "compass") chips.push([`Compass off by up to ${v.compass_error}° · ${v.compass_static ? "fixed" : "changes each look"}`, "frames"]);
  else if (v.frames !== "global") chips.push([v.frames === "rotated" ? "Own frame per robot" : "Disoriented robots", "frames"]);
  if (v.num_of_faults > 0) chips.push([`${v.num_of_faults} ${v.fault_type} fault${v.num_of_faults === 1 ? "" : "s"}`, "num_of_faults"]);
  return chips;
}
function renderBrief() {
  const el = $("model_notes");
  const key = algorithmKey();
  const a = algorithms.find((x) => x.key === key);
  if (!a) { el.hidden = true; return; }
  if (el.hidden) { el.hidden = false; requestAnimationFrame(updateInsets); } // its head now takes room above the view
  const locks = LOCKS[key] ?? {};
  const notes = brief.notes.map((text) => {
    const [re, kind, tag] = NOTE_KINDS.find(([r]) => r.test(text)) ?? [null, "info", ""];
    const body = re ? text.replace(re, "") : text;
    return { kind, tag, text: body.charAt(0).toUpperCase() + body.slice(1) }; // "Tag. Sentence", not "Tag. sentence"
  });
  const warns = notes.filter((x) => x.kind !== "info").length;
  el.classList.toggle("folded", !brief.open);
  el.innerHTML = `<button class="brief-head" type="button" aria-expanded="${brief.open}">`
    + `<span class="brief-name">${esc(MENU_NAMES[key] ?? a.name)}</span>`
    + (notes.length ? `<span class="brief-count ${warns ? "warn" : ""}">${icon(warns ? "warn" : "info", 13)}${notes.length}</span>` : "")
    + `<span class="chev" data-icon="chevron" data-size="14"></span></button>`
    + `<div class="brief-body"${brief.open ? "" : " hidden"}>`
    + `<p class="brief-rule">${esc(RULES[key] ?? "")}</p>`
    + `<div class="brief-chips">${briefChips(settings.values, key).map(([text, k]) =>
      `<span class="chip-tag${locks[k] ? " locked" : ""}"${locks[k] ? ` title="Fixed for this algorithm: ${esc(locks[k].reason)}"` : ""}>${locks[k] ? icon("lock", 11) : ""}${esc(text)}</span>`).join("")}</div>`
    + (notes.length ? `<ul class="brief-notes">${notes.map((x, i) =>
      `<li class="${x.kind}${brief.seen.has(x.text) ? "" : " fresh"}" style="--i:${i}">${icon(x.kind === "info" ? "info" : "warn", 13)}<span>${x.tag ? `<b>${x.tag}.</b> ` : ""}${esc(x.text)}</span></li>`).join("")}</ul>` : "")
    + `</div>`;
  if (brief.opening) { el.querySelector(".brief-body")?.classList.add("opening"); brief.opening = false; }
  // Remember what is on screen, so only notes a change brings in animate. Folded, they still count as new when it
  // opens; and the moment of no notes while a new run starts is not remembered, or every note would animate again.
  if (brief.open && notes.length) brief.seen = new Set(notes.map((x) => x.text));
  renderIcons(el);
  el.querySelector(".brief-head").addEventListener("click", () => { brief.open = !brief.open; brief.opening = brief.open; renderBrief(); });
}
/** Notes from the core for the current run (empty while a new run starts). */
function showNotes(notes) { brief.notes = notes; renderBrief(); }

/* -------------------------------------------------------------- scenarios */
// A scenario is the algorithm plus every setting, saved as a file. Links made by older versions (#scenario=…) still open.
function scenario() {
  return { format: "lcm-scenario", version: 1, algorithm: algorithmKey(), settings: { ...settings.values } };
}
function applyScenario(sc) {
  if (!sc || typeof sc !== "object" || !sc.settings) { toast("That is not a scenario file", true); return; }
  if (sc.algorithm && algorithms.some((a) => a.key === sc.algorithm)) { selectAlgorithm(sc.algorithm); updateExamples(); }
  const settable = new Set(SETTINGS.map((x) => x.key));
  const known = Object.fromEntries(Object.entries(sc.settings).filter(([k]) => settable.has(k)));
  settings.apply(known);
  newRun();
  toast("Scenario loaded");
}
const fromBase64 = (code) => new TextDecoder().decode(Uint8Array.from(atob(code.replace(/-/g, "+").replace(/_/g, "/")), (c) => c.charCodeAt(0)));
function exportScenario() {
  download(new Blob([JSON.stringify(scenario(), null, 2)], { type: "application/json" }), `${fileStem()}.scenario.json`);
  toast("Saved the scenario");
}
$("scenario_file").addEventListener("change", async (e) => {
  const file = e.target.files?.[0];
  e.target.value = "";
  if (!file) return;
  try { applyScenario(JSON.parse(await file.text())); } catch { toast("Could not read that scenario file", true); }
});

let resolveReady;
window.lcm = { renderer, client, settings, parseCoordinates, adversary, editor, stats: {}, ready: new Promise((r) => { resolveReady = r; }),
  get state() { return state; } };

/* -------------------------------------------------------- algorithm menu */
/** The registry key of the algorithm that runs: the menu's choice, or the advanced variant. */
function algorithmKey() {
  const v = $("algorithm").value;
  return v === "advanced" ? settings.values.variant : v;
}
/** Shows algorithm `key` in the menu (an advanced variant selects "Advanced gathering" and that variant). */
function selectAlgorithm(key) {
  if (ADVANCED.includes(key)) { $("algorithm").value = "advanced"; settings.set("variant", key, false); }
  else $("algorithm").value = key;
}
function buildMenu(list) { $("algorithm").innerHTML = menuHtml(list); }

// What each algorithm fixes, shown locked in the sidebar with the reason (instead of refusing the run).
const LOCKS = {
  CompassGathering: {
    num_of_robots: { value: 2, reason: "this algorithm is for exactly two robots" },
    frames: { value: "compass", reason: "each robot reads its own compass" },
    num_of_faults: { value: 0, reason: "it assumes robots that never fail" },
  },
  RandomizedGathering: {
    num_of_robots: { value: 2, reason: "this algorithm is for exactly two robots" },
    num_of_faults: { value: 0, reason: "it assumes robots that never fail" },
  },
  CrashGathering: {
    scheduler: { value: "ssync", reason: "it works in rounds, some robots at a time" },
    multiplicity: { value: "weak", reason: "it needs weak multiplicity detection" },
    fault_type: { value: "crash", reason: "it tolerates crash faults only" },
  },
  MutualVisibility: {
    obstructed_visibility: { value: true, reason: "the task exists only when robots block the view" },
    unlimited_visibility: { value: true, reason: "only other robots block the view" },
    scheduler: { value: "sequential", reason: "the no-lights algorithm is for a sequential scheduler" },
  },
  SqGathering: {
    num_of_faults: { value: 0, reason: "it assumes robots that never fail" },
  },
};
LOCKS.RandomizedGatheringUniform = LOCKS.RandomizedGathering;

/** Applies the chosen algorithm's locks and fills the algorithm card. */
function applyAlgorithm() {
  const key = algorithmKey();
  settings.setContext({ family: $("algorithm").value === "advanced" ? "advanced" : "plain" });
  settings.freeze(LOCKS[key] ?? {});
  // The sidebar only says what is locked; what the algorithm does is in the brief under the toolbar.
  const card = $("algo_card");
  if (card) {
    const locks = Object.values(LOCKS[key] ?? {});
    card.hidden = !locks.length;
    card.innerHTML = locks.length ? `${icon("lock", 12)}<span>${locks.length} setting${locks.length === 1 ? " is" : "s are"} fixed for this variant, marked with a lock below.</span>` : "";
  }
  renderBrief();
}

/* ------------------------------------------------------------ run control */
function config() {
  const c = readConfig(settings.values, algorithmKey(), settings.customPoints);
  // Playing the adversary: the run waits for each turn (or round) the user chooses.
  if (adversary.on && c.scheduler === "sequential") { c.schedule = []; c.schedule_end = "await"; delete c.rounds; }
  else if (adversary.on && c.scheduler === "ssync") { c.rounds = []; c.schedule_end = "await"; delete c.schedule; }
  return c;
}

let fitPending = false; // fit the view to the new run's first frame

/** Builds a new run from the settings; returns false if they can't make one. */
function newRun({ quiet = false, keepView = false } = {}) {
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
  fitPending = !keepView;
  inspector.leave(); // the robot under the pointer belonged to the old run
  adversary.reset();
  showNotes([]);
  renderer.options.visibility = c.visibility_radius;
  renderer.options.compassError = c.frames === "compass" ? c.compass_error : 0;
  // "Who sees whom" means something only when robots see just a part of the swarm.
  const limitedSight = c.visibility_radius != null;
  $("links").hidden = !limitedSight;
  if (!limitedSight && renderer.options.links) setLinks(false);
  renderer.options.allowedRegion = c.algorithm === "GoToCenter";
  renderer.options.multiplicities = Boolean(algorithms.find((a) => a.key === c.algorithm)?.multiplicities);
  inspector.select(-1);
  $("hud_last").hidden = true;
  dirty = false;
  $("dirty").hidden = true;
  client.stop();
  renderer.clearMotion();
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

function onSettingChange(key) {
  if (state === "loading") return;
  syncAddress();
  if (key === "variant") { onAlgorithmChange(); return; }
  updatePlaybackOptions();
  renderBrief();
  if (state === "ready") {
    clearTimeout(restartTimer);
    restartTimer = setTimeout(() => newRun({ quiet: true }), 120); // debounce slider drags and typing
  } else {
    dirty = true;
    $("dirty").hidden = false;
  }
}

function setState(next) {
  const was = state;
  state = next;
  const running = next === "running";
  // Playing: the brief folds to one line so the robots have the room; a click opens it again.
  if (running && was !== "running" && brief.open) { brief.open = false; renderBrief(); }
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
// SqGathering: two multiplicities of two (robots 0, 1 and 2, 3), and two robots sharing a point with one alone.
const TWO_PAIRS = "-64, 0\n-64, 0\n64, 0\n64, 0";
const PAIR_AND_ONE = "-60, 0\n-60, 0\n60, 0";
const SQ = { open_world: true, pattern: "custom", turn_gap: "none", robot_speeds: 20, unlimited_visibility: true, num_of_faults: 0, frames: "global", schedule_text: "" };
const EXAMPLE_BASE = { max_turns: 0, max_rounds: 0, schedule_text: "", threshold_precision: 5 };
const EXAMPLES = {
  SqGathering: [
    { label: "Rendezvous: two robots", note: "κ = 0: the active robot walks to the other one",
      set: { ...SQ, custom: "-80, 0\n80, 0", scheduler: "sequential", multiplicity: "weak", rigid_movement: true } },
    { label: "Rounds: never gathers", note: "A whole stack moves at once each time: two stacks get ever closer but never merge",
      set: { ...SQ, custom: TWO_PAIRS, scheduler: "ssync", multiplicity: "weak", rigid_movement: true, schedule_text: "0 1 | 2 3", schedule_repeat: "repeat", max_rounds: 16 } },
    { label: "The same start, sequential", note: "One robot at a time: it gathers",
      set: { ...SQ, custom: TWO_PAIRS, scheduler: "sequential", multiplicity: "weak", rigid_movement: true } },
    { label: "Can't see stacks: never gathers", note: "Every robot sees two points and goes to the other: the order 0, 2, 1 keeps them apart",
      set: { ...SQ, custom: PAIR_AND_ONE, scheduler: "sequential", multiplicity: "none", rigid_movement: true, schedule_text: "0, 2, 1", schedule_repeat: "repeat", max_turns: 60 } },
    { label: "Same order, robots see stacks", note: "Now the single robot knows the pair is a stack and joins it",
      set: { ...SQ, custom: PAIR_AND_ONE, scheduler: "sequential", multiplicity: "weak", rigid_movement: true, schedule_text: "0, 2, 1", schedule_repeat: "repeat", max_turns: 60 } },
    { label: "Limited visibility (untested ground)", note: "Experimental: 30 robots that see only 120 units around them; whether the rule always gathers like this is not known",
      set: { open_world: true, pattern: "cloud", num_of_robots: 30, spread: 90, scheduler: "sequential", turn_gap: "none", multiplicity: "weak", unlimited_visibility: false, visibility_radius: 120, rigid_movement: true, schedule_text: "" } },
  ],
  Gathering: [
    { label: "FSYNC: one round", note: "Everyone sees the same snapshot and walks to the same centroid",
      set: { open_world: true, pattern: "cloud", num_of_robots: 20, spread: 120, scheduler: "fsync", turn_gap: "none", rigid_movement: true } },
    { label: "SSYNC, a few robots per round", note: "Each robot is active with chance 0.3",
      set: { open_world: true, pattern: "cloud", num_of_robots: 20, spread: 120, scheduler: "ssync", ssync_probability: 0.3, turn_gap: "none", rigid_movement: true, schedule_text: "" } },
    { label: "Robots block the view", note: "Obstructed visibility, robot radius 4",
      set: { open_world: true, pattern: "line", num_of_robots: 8, size: 300, scheduler: "sequential", turn_gap: "none", obstructed_visibility: true, robot_radius: 4, rigid_movement: true } },
  ],
  CompassGathering: [
    { label: "Static compass, φ = 60°", note: "Always gathers while the error stays below 90°. Red needles show where each robot thinks north is",
      set: { open_world: true, obstructed_visibility: false, unlimited_visibility: true, multiplicity: "weak", pattern: "custom", custom: "-120, 40\n110, -60", frames: "compass", compass_error: 60, compass_static: true, scheduler: "ssync", turn_gap: "none", robot_speeds: 20, rigid_movement: true, num_of_faults: 0, schedule_text: "" } },
    { label: "Dynamic compass, SSYNC, φ = 40°", note: "Robots act in rounds: always gathers while the error stays below 45°",
      set: { open_world: true, obstructed_visibility: false, unlimited_visibility: true, multiplicity: "weak", pattern: "custom", custom: "-120, 40\n110, -60", frames: "compass", compass_error: 40, compass_static: false, scheduler: "ssync", turn_gap: "none", robot_speeds: 20, rigid_movement: true, num_of_faults: 0, schedule_text: "" } },
    { label: "Dynamic compass, async, φ = 25°", note: "Robots act on their own: gathers while the error stays below 30°. Once together they stay put but never stop, so pause when they meet",
      set: { open_world: true, obstructed_visibility: false, unlimited_visibility: true, multiplicity: "weak", pattern: "custom", custom: "-120, 40\n110, -60", frames: "compass", compass_error: 25, compass_static: false, scheduler: "async", robot_speeds: 20, rigid_movement: true, num_of_faults: 0 } },
  ],
  RandomizedGathering: [
    { label: "Mirror images, FSYNC", note: "Both robots see the same picture; only their coins tell them apart",
      set: { open_world: true, obstructed_visibility: false, unlimited_visibility: true, frames: "global", pattern: "custom", custom: "-150, 0\n150, 0", scheduler: "fsync", turn_gap: "none", robot_speeds: 40, rigid_movement: true, num_of_faults: 0 } },
    { label: "Async", note: "Each look tosses a coin: stay, go halfway or go all the way",
      set: { open_world: true, obstructed_visibility: false, unlimited_visibility: true, frames: "global", pattern: "custom", custom: "-150, 0\n150, 0", scheduler: "async", robot_speeds: 40, rigid_movement: true, num_of_faults: 0 } },
  ],
  get RandomizedGatheringUniform() { return this.RandomizedGathering; },
  MutualVisibility: [
    { label: "Twelve robots in a line", note: "Each sees only its neighbours; after one epoch everyone sees everyone",
      set: { open_world: true, frames: "global", unlimited_visibility: true, num_of_faults: 0, pattern: "line", num_of_robots: 12, size: 400, distribution: "even", scheduler: "sequential", turn_gap: "none", obstructed_visibility: true, robot_radius: 0, robot_speeds: 10, max_turns: 60 } },
    { label: "A 5 × 5 grid", note: "Rows, columns and diagonals hide robots; after two epochs all 25 see each other. Precision 10⁻⁹: its moves get smaller than the default ε",
      set: { open_world: true, frames: "global", unlimited_visibility: true, num_of_faults: 0, pattern: "custom", custom: Array.from({ length: 25 }, (_, k) => `${(k % 5) * 60 - 120}, ${Math.floor(k / 5) * 60 - 120}`).join("\n"), scheduler: "sequential", turn_gap: "none", obstructed_visibility: true, robot_radius: 0, robot_speeds: 10, max_turns: 50, threshold_precision: 9 } },
  ],
  CrashGathering: [
    { label: "Ten robots, three crashed", note: "The others gather on one point; crashed robots stay where they are",
      set: { open_world: true, frames: "global", obstructed_visibility: false, unlimited_visibility: true, pattern: "cloud", num_of_robots: 10, spread: 120, num_of_faults: 3, fault_type: "crash", scheduler: "ssync", turn_gap: "none", multiplicity: "weak", robot_speeds: 20, rigid_movement: true, schedule_text: "" } },
    { label: "Robots in line behind each other", note: "A robot with another in its way curves around it",
      set: { open_world: true, frames: "global", obstructed_visibility: false, unlimited_visibility: true, pattern: "custom", custom: "10, 0\n20, 0\n30, 0\n40, 0\n-40, 0\n0, 35", num_of_faults: 0, scheduler: "ssync", turn_gap: "none", multiplicity: "weak", robot_speeds: 20, rigid_movement: true, schedule_text: "" } },
  ],
  Spreading: [
    { label: "Spreading in a box", note: "Robots start bunched and spread to cover the world",
      set: { open_world: false, pattern: "cloud", num_of_robots: 30, spread: 40, width_bound: 400, height_bound: 400, frames: "global" } },
    { label: "Disoriented robots", note: "Each robot samples a grid in its own frame: Spreading depends on the axes",
      set: { open_world: true, pattern: "cloud", num_of_robots: 30, spread: 60, frames: "disoriented" } },
  ],
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
  const list = EXAMPLES[algorithmKey()] ?? [];
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
  const list = EXAMPLES[algorithmKey()];
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
  const list = EXAMPLES[algorithmKey()] ?? [];
  if (index < 0) {
    if (before) { setLinks(before.links); settings.apply(before.values); }
    before = null;
  } else if (list[index]) {
    const example = list[index];
    before ??= {
      links: renderer.options.links,
      values: Object.fromEntries([...new Set([...Object.keys(EXAMPLE_BASE), ...list.flatMap((e) => Object.keys(e.set))])].map((key) => [key, settings.values[key]])),
    };
    setLinks(example.links);
    // Limits and explicit schedules belong to the example that sets them: never carried into the next one.
    settings.apply({ ...EXAMPLE_BASE, ...example.set });
    settings.freeze(LOCKS[algorithmKey()] ?? {});
  } else return;
  activeExample = index;
  renderExamplesMenu();
  // An example is a new experiment, like a new algorithm: start it now, even after a run has ended.
  if (state !== "loading" && state !== "ready") newRun();
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
  buildMenu(list);
  // ?algorithm=KEY (a case link, or the ring page's menu) opens the page on that algorithm; an advanced
  // variant selects "Advanced gathering" and that variant.
  const asked = link.algorithm;
  if (asked && (asked === "advanced" || list.some((a) => a.key === asked))) {
    selectAlgorithm(asked);
    lastMenu = $("algorithm").value;
    if (list.find((a) => a.key === algorithmKey())?.sequential_only) settings.set("scheduler", "sequential", false);
  }
  applyAlgorithm();
  updateExamples();
  updatePlaybackOptions();
  const speed = link.speed === "max" ? "Infinity" : link.speed;
  if (speed && [...$("playback").options].some((o) => o.value === speed)) $("playback").value = speed;
  $("loading_text").textContent = "Building the swarm…";
  if (location.hash.startsWith("#scenario=")) {
    try { applyScenario(JSON.parse(fromBase64(location.hash.slice(10)))); return; } catch { toast("The scenario in this link could not be read", true); }
  }
  newRun();
  if (link.links && !$("links").hidden) setLinks(true); // who sees whom, asked for by the link
  syncAddress(); // a link with values the page could not use shows what it did use
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
  Gathering: "Every time a robot looks, it walks to the average position of all the robots it can see, its centre of gravity. Because everybody aims at nearly the same middle point, the swarm keeps shrinking towards its centre, quickly at first and then more and more slowly; a robot terminates once every robot it sees is within ε of that middle.",
  SEC: "Every time a robot looks, it finds the smallest circle that encloses all the robots it can see and walks to the nearest point on that circle. Robots inside the circle move out to its edge, robots already on it stay, and the run ends when every robot sits on the circle.",
  GoToCenter: "Every time a robot looks, it heads for the centre of the smallest circle around the robots it can see, but it only goes as far as it can without losing sight of any of its neighbours. Groups that can see each other therefore stay connected while they shrink, and with limited sight the swarm closes in without falling apart.",
  SqGathering: "One robot moves at a time, and robots can tell a stack (several robots on one spot) from a single robot. If there is exactly one stack, every robot walks to it and robots already on it stay. If there are several stacks, a robot on a stack steps halfway towards its nearest neighbour, breaking the tie, while robots on their own wait. If there is no stack yet, a robot walks onto its nearest neighbour to create the first one.",
  Spreading: "Every time a robot looks, it walks to the centre of the area that is closer to it than to any other robot (its Voronoi cell). Crowded robots have small cells and move apart, lonely robots have large ones, so the swarm spreads out until the robots cover the space evenly.",
  TwoTask: "The swarm is split into two teams that want different things. Red robots run the smallest-enclosing-circle rule and walk to the edge of the circle around the swarm; blue robots run the centre-of-gravity rule and walk to its middle. Watching them pull against each other shows how two algorithms interact in one swarm.",
  CircleFormation: "A robot that is not on the circle around the swarm first steps onto it. A robot already on the circle slides along it to the point halfway between its two neighbours. Repeated over and over, these small corrections spread the robots out until they stand evenly spaced around the circle.",
  CompassGathering: "Two robots whose compasses may point a little away from true north. Each robot looks at where the other one lies according to its own compass and, depending on that direction, walks straight onto it, steps west, swings around it, or waits. The rules are chosen so that the two robots meet as long as the compass errors stay below a known bound.",
  RandomizedGathering: "Two robots that look exactly alike. Every time a robot looks it tosses a coin and either stays where it is, goes halfway towards the other robot, or goes all the way to it. Randomness is what breaks the symmetry: sooner or later one robot moves while the other stays, and they meet.",
  RandomizedGatheringUniform: "Two robots that look exactly alike. Every time a robot looks it picks at random to go all the way to the other robot, halfway, or a random fraction of the way. Because the two robots almost never choose the same move, the distance between them keeps shrinking until they meet.",
  MutualVisibility: "Robots that stand in line hide each other. Every time a robot looks, it makes a small sideways move chosen so that it never lands in line with two other robots, which gradually uncovers robots that were hidden. The goal is a position in which every robot can see every other robot.",
  CrashGathering: "Robots gather on one spot even though some of them may stop working for good (crash). Each working robot heads for the stack where robots are already gathering or, before there is one, for the centre of the smallest circle around everyone, and curves around any robot standing in its way. Crashed robots simply stay where they are.",
};
const fmtNumber = (v) => (v === 0 ? "0" : v >= 1e-3 ? String(+v.toPrecision(3)) : v.toExponential(2));
let endNote = null;   // "ended", "stalled" or null: which note is up
let stepOfNumbers = 0; // the gap between neighbouring numbers where the swarm is, set from the frame
function showEndNote(kind, f = renderer.frame) {
  endNote = kind;
  const el = $("turn_caption");
  el.hidden = !kind;
  if (!kind) return;
  const points = trace.points, size = points.length ? points[points.length - 1] : 0;
  const zoomed = renderer.zoom > 20 ? ` This view is zoomed in ${fmtZoomWords(renderer.zoom)} times; the grid and the scale bar show the real size.` : "";
  let html;
  if (kind === "stalled" && f?.seeing != null) {
    const all = f.seeing === f.robots;
    html = `<div class="cap-head">${all ? "Mutual visibility reached" : "Stopped before mutual visibility"}</div><div class="cap-body">${f.seeing} of ${f.robots} robots see every other robot. This algorithm never terminates: the run stops once a whole round passes in which every robot's next move is too small to make (shorter than ε).${all ? "" : " Some robots are still hidden behind others: a lower precision (larger ε) or a turn limit changes where it stops."}${zoomed}</div>`;
  } else if (kind === "ended") {
    const left = f ? f.robots - f.terminated : 0; // crashed and Byzantine robots never terminate
    html = `<div class="cap-head">Finished: every ${left ? "working " : ""}robot has terminated</div><div class="cap-body">Grey robots have terminated.${left ? ` The other ${left} are faulty (crashed or Byzantine) and do not terminate.` : ""} The swarm is ${fmtSize(size)} units across (from its centre to its farthest robot).${zoomed}</div>`;
  } else {
    // Stalled: a whole round in which no robot could move or finish. Often the numbers have run out: a robot's
    // destination rounds to the spot where it already stands.
    const p = settings.values.threshold_precision, eps = 10 ** -p;
    const numbers = stepOfNumbers > 0 && eps < 2 * stepOfNumbers
      ? ` Here, the computer's numbers cannot tell two positions apart if they are closer than ${fmtDistance(stepOfNumbers)}, but the precision setting (${p}) asks for ε = ${fmtDistance(eps)}, which is finer. So a robot's destination rounds to where it already stands, and it does not move. Try a precision of ${Math.max(2, Math.floor(-Math.log10(stepOfNumbers * 20)))} or less.`
      : "";
    html = `<div class="cap-head">Stalled: nothing can move any more</div><div class="cap-body">A whole round passed in which no robot could move or finish. The swarm is ${fmtSize(size)} units across.${numbers}${zoomed}</div>`;
  }
  if (el.innerHTML !== html) el.innerHTML = html;
}
function updateCaption(f) {
  const el = $("turn_caption"), w = f.walk;
  if (!w) {
    // A run that is over, in a view that is zoomed in on the swarm or played a turn at a time: say so, because
    // grey robots a long way apart on screen can look like a failure when they are a hair apart in fact.
    stepOfNumbers = ulpOf(Math.max(Math.abs(f.originX ?? 0), Math.abs(f.originY ?? 0)));
    showEndNote(f.stop === "stalled" ? "stalled" : f.ended && (renderer.follow || typeof playbackSpeed() === "object") ? "ended" : null, f);
    return;
  }
  el.hidden = false;
  endNote = null;
  const who = `<strong>Turn ${(f.turns ?? 0).toLocaleString()}</strong> · Robot ${w.robot}`;  // turn pacing is sequential only
  const rule = RULES[algorithmKey()] ?? "";
  el.innerHTML = w.phase === "decide"
    ? `<div class="cap-head">${who} looks and decides</div><div class="cap-body">${rule || "It looks at the others and picks its destination, the ✕."}</div>`
    : `<div class="cap-head">${who} moves</div><div class="cap-body">It walks ${fmtNumber(w.length)} units to the ✕.${w.cut ? " The adversary stops it short of its destination (non-rigid movement)." : ""}</div>`;
}

client.on("frame", (f) => {
  frame = f;
  if (f.notes) showNotes(f.notes);
  // The adversary's schedule ran out: the run waits, paused, for the next choice.
  if (f.awaiting && !f.playing && state === "running") { client.stopPulling(); setState("paused"); }
  renderer.options.walk = f.walk ?? null;
  if (f.trace) trace.points = f.trace;
  updateCaption(f);
  if (fitPending) {
    fitPending = false;
    fitToStart(f);
    // Embedded: start from the fitted view, then keep the swarm centred and in view as it moves, zooming
    // out if it spreads but never in past the start (a gathering stays the size it was, in the middle).
    if (embedded) { renderer.resetView(); renderer.followCap = 1; renderer.setFollow(true); }
  }
  renderer.setFrame(f);
  renderer.keepInView(f);
  if (firstFrame) { firstFrame = false; $("loading").hidden = true; resolveReady(); }
  if (f.lastEvent) showLastEvent(f.lastEvent);
  window.lcm.frames = (window.lcm.frames ?? 0) + 1;
  if (f.stop !== "running" && state !== "ended" && state !== "loading") {
    client.stopPulling(); // the run is over: nothing new to fetch or draw
    setState("ended");
    // An embedded demo asked to loop starts again after a moment (unless the reader has reset it meanwhile).
    if (link.loop) setTimeout(() => { if (state === "ended" && newRun({ quiet: true })) play(); }, 2500);
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
  if (endNote) showEndNote(endNote);  // the view is still settling after the run ends: keep its zoom up to date
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
  if (z >= 1e15) {  // beyond a trillion: 4.1×10¹⁶×
    const [m, e] = z.toExponential(1).split("e");
    return `${m}×10${[...String(Number(e))].map((c) => SUPERSCRIPT[c]).join("")}×`;
  }
  if (z < 10) return `${z.toFixed(2)}×`;
  if (z < 1e3) return `${z.toFixed(z < 100 ? 1 : 0)}×`;
  const units = [[1e12, "T"], [1e9, "B"], [1e6, "M"], [1e3, "k"]];  // thousand, million, billion, trillion
  for (const [size, suffix] of units) if (z >= size) return `${(z / size).toFixed(1)}${suffix}×`;
  return `${z}×`;
}
/** The same in words for a sentence: "173.2 billion", "4.1×10¹⁶". */
function fmtZoomWords(z) {
  if (z >= 1e15) return fmtZoom(z).replace(/×$/, "");
  const units = [[1e12, "trillion"], [1e9, "billion"], [1e6, "million"]];
  for (const [size, word] of units) if (z >= size) return `${(z / size).toFixed(1)} ${word}`;
  return Math.round(z).toLocaleString();
}
/** The zoom gauge (logarithmic, 1× to 10¹⁸ ×) and the distance between the labelled grid lines. */
let zoomShown = "";
function updateZoomRows() {
  const z = renderer.zoom, text = fmtZoom(z), grid = renderer.gridState;
  const key = `${text}|${grid?.spacing ?? ""}`;
  if (key === zoomShown) return;
  zoomShown = key;
  $("hud_zoom").textContent = text;
  $("hud_zoom_fill").style.width = `${Math.max(0, Math.min(1, Math.log10(Math.max(z, 1)) / 18)) * 100}%`;
  $("hud_grid").textContent = !renderer.options.grid ? "off" : !grid ? "—" : grid.valid ? fmtLength(grid.spacing) : "numbers run out";
}

function updateHud(force = false) {
  const now = performance.now();
  if (!force && now - hudAt < 200) return;
  hudAt = now;
  if (now - fpsAt >= 1000) { fps = (draws * 1000) / (now - fpsAt); draws = 0; fpsAt = now; }
  let statusText = {
    loading: "Loading…", ready: "Ready", running: "Running", paused: "Paused",
    ended: frame?.ended ? (frame.terminated < frame.robots ? `Ended · ${frame.terminated} of ${frame.robots} terminated, the rest faulty` : "Ended · all terminated")
      : frame?.seeing != null ? `Stopped · ${frame.seeing} of ${frame.robots} see everyone`
      : frame?.stop === "stalled" ? "Stalled · nothing can move any more"
      : frame?.stop === "max-turns" ? "Stopped · turn limit reached"
      : frame?.stop === "max-rounds" ? "Stopped · round limit reached" : `Stopped (${frame?.stop})`,
  }[state];
  if (state === "paused" && frame?.awaiting) statusText = adversary.mode === "ssync" ? "Your move · pick a round" : "Your move · pick a robot";
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
    $("hud_epochs_row").querySelector(".k").textContent = frame.unit === "round" ? "Rounds · epochs" : "Turns · epochs";
    $("hud_epochs").textContent = frame.epochs.toLocaleString();
    $("hud_turns").textContent = (frame.turns ?? 0).toLocaleString();
  }
  $("hud_events").textContent = (frame.events - frame.ticks).toLocaleString()
    + (frame.deferred ? ` · ${frame.deferred.toLocaleString()} put off` : "");
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
let lastMenu = "Gathering";
$("algorithm").addEventListener("change", () => {
  if ($("algorithm").value === "ring") {
    $("algorithm").value = lastMenu; // back to the old choice if the user returns
    location.href = "web/ring.html";
    return;
  }
  lastMenu = $("algorithm").value;
  onAlgorithmChange();
});
function onAlgorithmChange() {
  brief.open = true; // a new algorithm: say what it does
  applyAlgorithm();
  updateExamples();
  syncAddress();
  // An algorithm that only works under the sequential scheduler (SqGathering)
  // switches it on, rather than failing with the async default.
  const a = algorithms.find((x) => x.key === algorithmKey());
  if (a?.sequential_only && (settings.values.scheduler === "async" || settings.values.scheduler === "kasync")) {
    settings.set("scheduler", "sequential", false);
    toast(`${a.name} runs one robot at a time: Scheduler set to Sequential`);
  }
  // Algorithms for exactly two robots open on their first example rather than refusing the swarm on screen.
  // A new algorithm is a new experiment: start it now, even mid-run.
  if (state === "loading") return;
  newRun({ quiet: true });
}
$("playback").addEventListener("change", () => {
  syncAddress();
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
  // Where the view tools start: the brief, when it sits in the right-hand column, stops above them.
  const toolsBox = visible("tools");
  root.setProperty("--tools-top", `${Math.round(toolsBox ? toolsBox.top : innerHeight)}px`);
  const settingsRect = visible("settings"), tools = visible("tools"), legend = visible("legend"), briefRect = visible("model_notes");
  // The overview map sits just left of the view tools, in the bottom right corner.
  if (tools && tools.height > tools.width) $("overview").style.right = `${Math.round(innerWidth - tools.left + 8)}px`;
  if (embedded) {
    // Embedded: the robots use the whole box below the buttons, above the caption when there is one.
    const caption = visible("turn_caption");
    renderer.setInsets({ left: 0, right: 0, top: toolbar ? toolbar.bottom : 0, bottom: caption ? innerHeight - caption.top + 8 : 8 });
    return;
  }
  if (innerWidth <= 760) {
    // Phones: the settings sheet floats over the view instead of narrowing it.
    const bottomEdge = Math.min(tools ? tools.top : innerHeight, legend ? legend.top : innerHeight);
    renderer.setInsets({ left: 0, right: 0, top: Math.max(toolbar ? toolbar.bottom : 0, hud ? hud.bottom : 0), bottom: innerHeight - bottomEdge });
    return;
  }
  renderer.setInsets({
    left: settingsRect ? settingsRect.right : 0,
    right: Math.max(hud ? innerWidth - hud.left : 0, tools && tools.height > tools.width ? innerWidth - tools.left : 0),
    // The brief's one-line head stays clear of the view (opened, it may cover the top of the view for a while).
    top: Math.max(toolbar ? toolbar.bottom : 0, briefRect ? briefRect.top + 42 : 0),
    bottom: tools && tools.width >= tools.height ? innerHeight - tools.top : 56,
  });
}

/* ------------------------------------------------------------ view tools */
$("zoom_in").addEventListener("click", () => renderer.zoomAt(1.25));
$("zoom_out").addEventListener("click", () => renderer.zoomAt(0.8));
$("hud_zoom_in").addEventListener("click", () => renderer.zoomAt(1.25));
$("hud_zoom_out").addEventListener("click", () => renderer.zoomAt(0.8));
$("fit").addEventListener("click", () => renderer.resetView());
$("grid").addEventListener("click", toggleGrid);
$("trails").addEventListener("click", toggleTrails);
function toggleTrails() {
  renderer.options.trails = !renderer.options.trails;
  $("trails").setAttribute("aria-pressed", String(renderer.options.trails));
  renderer.requestDraw();
}
function toggleOverview() {
  $("overview").hidden = !$("overview").hidden;
  updateInsets();
  renderer.requestDraw();
}
if (!link.grid) { renderer.options.grid = false; $("grid").setAttribute("aria-pressed", "false"); } // grid=0 in the link
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
$("links").addEventListener("click", toggleLinks);
function toggleLinks() {
  if ($("links").hidden) return;  // unlimited sight: everyone sees everyone, so there is nothing to show
  renderer.options.links = !renderer.options.links;
  $("links").setAttribute("aria-pressed", String(renderer.options.links));
  renderer.requestDraw();
}

$("export").addEventListener("click", (e) => { e.stopPropagation(); openExamples(false); $("export_menu").hidden = !$("export_menu").hidden; });
document.addEventListener("click", () => { $("export_menu").hidden = true; openExamples(false); });
// Links to the current case: the full page, or the iframe code for another web page (embed.js).
const caseQuery = (speed = $("playback").value) => linkQuery({ values: settings.values, algorithm: algorithmKey(), playback: speed, points: settings.customPoints });
const embedDialog = buildEmbedDialog({
  query: caseQuery,
  speeds: () => [...$("playback").options].map((o) => [o.value, o.text]),
  speed: () => $("playback").value,
}, toast);
renderIcons($("embed_dialog"));
window.lcm.caseLink = () => `${pageAddress()}?${caseQuery()}`;
// The address bar follows the case on screen, so reloading the page or copying its address keeps the
// case (a link it was opened with does not go stale). An embedded demo leaves its address alone.
let addressTimer = 0;
function syncAddress() {
  if (embedded) return;
  clearTimeout(addressTimer);
  addressTimer = setTimeout(() => {
    const query = caseQuery();
    const plain = query === `algorithm=${algorithms[0]?.key}`; // nothing but the defaults: a clean address
    const url = plain || query.length > 8000 ? pageAddress() : `${pageAddress()}?${query}`; // (thousands of typed points: too long)
    try { history.replaceState(null, "", url + location.hash); } catch {}
  }, 250);
}
window.lcm.embedDialog = embedDialog;
async function copyCaseLink() {
  if (await copyText(window.lcm.caseLink())) toast("Link copied: it opens the simulator with these settings");
  else toast("Could not copy: the browser did not allow it", true);
}
const EXPORTS = { png: () => exportPng(), csv: () => exportCsv(), link: copyCaseLink, embed: () => embedDialog.open(),
  scenario: () => exportScenario(), open: () => $("scenario_file").click() };
for (const b of document.querySelectorAll("[data-export]")) b.addEventListener("click", EXPORTS[b.dataset.export]);
$("edit").addEventListener("click", () => setEditor(!editor.on));
$("adversary").addEventListener("click", () => setAdversary(!adversary.on));

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
  const name = algorithms.find((a) => a.key === algorithmKey())?.name ?? "";
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
function applyTheme(t, save = true) {
  if (t === "system") delete document.documentElement.dataset.theme;
  else document.documentElement.dataset.theme = t;
  const dark = t === "dark" || (t === "system" && media.matches);
  document.documentElement.dataset.resolvedTheme = dark ? "dark" : "light";
  if (save) try {
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
// A theme in the link is for this page only: it does not change the reader's choice for the simulator.
applyTheme(link.theme ?? currentTheme(), !link.theme);

/* ---------------------------------------------- canvas: pan, zoom, pick */
const canvas = $("canvas");
let drag = null;
let editDrag = false; // the scenario editor is dragging a robot
canvas.addEventListener("pointerdown", (e) => {
  const rect = canvas.getBoundingClientRect();
  canvas.setPointerCapture(e.pointerId);
  if (editor.down(e.clientX - rect.left, e.clientY - rect.top, e)) { editDrag = true; return; }
  drag = { x: e.clientX, y: e.clientY, moved: false };
});
canvas.addEventListener("contextmenu", (e) => {
  if (!editor.on) return;
  e.preventDefault();
  const rect = canvas.getBoundingClientRect();
  editor.remove(renderer.pick(e.clientX - rect.left, e.clientY - rect.top, 10));
});
canvas.addEventListener("pointermove", (e) => {
  const rect = canvas.getBoundingClientRect();
  if (editor.move(e.clientX - rect.left, e.clientY - rect.top)) return;
  if (adversary.on && !drag) adversary.move(e.clientX - rect.left, e.clientY - rect.top);
  if (measure.on && !drag) {  // the ruler follows the pointer until its second point is placed
    measureHover(e.clientX - rect.left, e.clientY - rect.top);
    return;
  }
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
  if (editDrag) { editDrag = false; editor.up(); return; }
  if (drag && !drag.moved) {
    const sx = e.clientX - rect.left, sy = e.clientY - rect.top;
    if (editor.on) editor.click(sx, sy, e);
    else if (adversary.on) adversary.click(sx, sy);
    else if (measure.on) measureClick(e.clientX - rect.left, e.clientY - rect.top);
    else inspector.select(renderer.pick(e.clientX - rect.left, e.clientY - rect.top));
  }
  drag = null;
  canvas.classList.remove("panning");
});
canvas.addEventListener("pointerleave", () => { inspector.leave(); if (measure.on && measure.hover) { measure.hover = null; renderer.requestDraw(); } });

/* ------------------------------------------------------------ measuring */
// Click a point (or a robot), then a second one: the distance is written on a ruler between them. A robot is
// picked when the click is near it and the end stays attached to it (the number follows it as it moves); empty
// space is a fixed point in the world. A third click starts over. Esc clears it, and again leaves the tool.
const measure = { on: false, a: null, b: null, hover: null };
function measurePointAt(sx, sy) {
  const i = renderer.pick(sx, sy, 14);
  if (i >= 0) return { robot: i };
  const [x, y] = renderer.toWorld(sx, sy);
  return { x, y };
}
function measureHover(sx, sy) {
  measure.hover = measurePointAt(sx, sy);
  renderer.requestDraw();
}
function measureClick(sx, sy) {
  const p = measurePointAt(sx, sy);
  if (!measure.a || measure.b) { measure.a = p; measure.b = null; } else measure.b = p;
  measure.hover = p;
  renderer.requestDraw();
}
function setMeasure(on) {
  measure.on = on;
  measure.a = measure.b = measure.hover = null;
  $("measure").setAttribute("aria-pressed", String(on));
  canvas.classList.toggle("measuring", on);
  renderer.options.measure = on ? measure : null;
  inspector.leave();
  if (on) toast("Measure: click a point or a robot, then a second one. Esc clears.");
  renderer.requestDraw();
}
function toggleMeasure() { setMeasure(!measure.on); }
$("measure").addEventListener("click", toggleMeasure);
window.lcm.measure = measure;
canvas.addEventListener("wheel", (e) => {
  if (embedded && !e.ctrlKey) return; // inside another page the wheel scrolls that page; Ctrl + wheel zooms
  e.preventDefault();
  const rect = canvas.getBoundingClientRect();
  renderer.zoomAt(Math.exp(-e.deltaY * 0.0015), e.clientX - rect.left, e.clientY - rect.top);
}, { passive: false });
new ResizeObserver(() => { renderer.resize(); updateInsets(); }).observe($("stage"));
// The stats card grows as a run fills it in: whatever sits under it moves down with it.
new ResizeObserver(() => updateInsets()).observe($("hud"));
if (embedded) new ResizeObserver(updateInsets).observe($("turn_caption")); // the caption comes and goes
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
    else if (adversary.on && adversary.chosen.length) adversary.action("clear");
    else if (editor.on) setEditor(false);
    else if (measure.on) { if (measure.a) { measure.a = measure.b = measure.hover = null; renderer.requestDraw(); } else setMeasure(false); }
    else inspector.select(-1);
    return;
  }
  if (embedded && e.key !== " ") return; // embedded: Space plays and pauses, nothing else
  if (!$("embed_dialog").hidden) return; // the embed window is open: keys are for it
  const typing = e.target.closest?.("input, select, textarea");
  if (editor.on && !typing && (e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "z") { e.preventDefault(); editor.undo(); return; }
  if (settings.editor.isOpen || typing || e.ctrlKey || e.metaKey || e.altKey) return;
  if (editor.on && (e.key === "Delete" || e.key === "Backspace")) { e.preventDefault(); editor.remove(editor.hovered); return; }
  if (adversary.on && e.key === "Enter" && !e.target.closest?.("button, a, summary")) { e.preventDefault(); adversary.action("play"); return; }
  // Space/Enter on a focused button already clicks it; don't toggle twice.
  if ((e.key === " " || e.key === "Enter") && e.target.closest?.("button, a, summary")) return;
  const actions = {
    " ": play,
    ArrowRight: () => step(e.shiftKey ? "time" : "event"),
    r: newRun, R: newRun,
    f: () => renderer.resetView(), F: toggleFullscreen,
    z: toggleFollow, Z: toggleFollow,
    g: toggleGrid, G: toggleGrid,
    p: toggleTrails, P: toggleTrails,
    o: toggleOverview, O: toggleOverview,
    m: toggleMeasure, M: toggleMeasure,
    d: () => setEditor(!editor.on), D: () => setEditor(!editor.on),
    a: () => setAdversary(!adversary.on), A: () => setAdversary(!adversary.on),
    "+": () => renderer.zoomAt(1.25), "=": () => renderer.zoomAt(1.25), "-": () => renderer.zoomAt(0.8),
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

/* ------------------------------------------------------------------ embed */
// Embedded in another page: play by itself the first time it is scrolled into view; after that the
// play button is the reader's.
if (link.autoplay) window.lcm.ready.then(() => whenFirstSeen(() => { if (state === "ready") play(); }));

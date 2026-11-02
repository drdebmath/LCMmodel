/* Simulation settings, defined once. The panel is built from this list and
 * `readConfig()` turns it into the core's SimConfig (docs/core-schema.md §2),
 * so adding a setting is one entry here. */
import { ADVANCED_VARIANTS } from "./menu.js";
import { ARRANGEMENTS, ARRANGEMENT_LABELS, DISTRIBUTION_LABELS, PATTERNS, describeStart, distributionsFor, naturalDistribution, parseCoordinates } from "./start.js";

export const GROUPS = [
  // Shown only while "Advanced gathering" is the algorithm, above the tabs: its variant and what it fixes.
  { id: "algorithm", label: "Advanced gathering", open: true, showIf: (v) => v.family === "advanced", card: true },
  { id: "swarm", label: "Swarm", open: true, tab: "robots" },
  { id: "start", label: "Start", open: true, describe: true, tab: "robots" },
  { id: "movement", label: "Movement", open: true, tab: "model" },
  { id: "visibility", label: "Visibility", open: true, tab: "model" },
  { id: "model", label: "Robot model", open: true, tab: "model" },
  { id: "faults", label: "Faults", open: true, tab: "model" },
  { id: "timing", label: "Scheduler", open: true, tab: "timing", describe: (v) => describeTiming(v) },
  { id: "clock", label: "Clock and precision", open: true, tab: "timing" },
];
/** The settings panel's tabs: which robots and where they start; what a robot can do and see; when robots act. */
export const TABS = [["robots", "Robots"], ["model", "Model"], ["timing", "Timing"]];

const SHAPES = ["square", "rectangle", "circle", "polygon", "line"];
const ROTATABLE = ["square", "rectangle", "line", "polygon", "clusters"];
const SIZE_LABEL = { square: "Side length", rectangle: "Width", circle: "Diameter", polygon: "Diameter", line: "Length", clusters: "Distance between groups" };

// kind: "range" (slider + number), "log" (log-scale slider + number), "switch",
// "select" (options: a list, or a function of the values), "number", "text",
// "line" (one line of text, typed in place).
// `key` is the SimConfig field unless `local`; `label` may be a function of the
// values; `note` is a line under the control; `showIf` hides a setting that
// does not apply; `enabledBy` greys it out; `more` puts it under "More options"
// (a function of the values, or true).
export { ADVANCED_VARIANTS }; // defined with the menu (menu.js)

export const SETTINGS = [
  { key: "variant", group: "algorithm", label: "Variant", kind: "select", value: "CompassGathering", local: true,
    options: ADVANCED_VARIANTS,
    hint: "Which advanced gathering algorithm runs. Settings it fixes are locked below, with the reason." },
  { key: "num_of_robots", group: "swarm", label: "Robots", kind: "log", min: 1, max: 10000, value: 500,
    enabledBy: (v) => v.pattern !== "custom",
    hint: "Number of robots. With your own positions it is the number of positions." },
  { key: "random_seed", group: "swarm", label: "Random seed", kind: "number", min: 0, max: 4294967295, value: 12345, dice: true,
    hint: "Same seed and settings give the same run, every time, on every machine." },

  { key: "open_world", group: "start", label: "Open world", kind: "switch", value: true, local: true,
    note: (v) => (v.open_world ? "No border: robots can start anywhere." : "The original world box (robots can still move out of it)."),
    hint: "On: no world box, robots start wherever the pattern puts them. Off: the original 600 × 600 world, as in the original simulator." },
  { key: "pattern", group: "start", label: "Pattern", kind: "select", value: "cloud", local: true,
    options: (v) => PATTERNS.filter(([k]) => k !== "box" || !v.open_world),
    hint: "What shape the robots start in." },
  { key: "arrangement", group: "start", label: "Arrangement", kind: "select", value: "filled", local: true,
    options: (v) => (ARRANGEMENTS[v.pattern] ?? []).map((k) => [k, ARRANGEMENT_LABELS[k]]),
    showIf: (v) => (ARRANGEMENTS[v.pattern] ?? []).length > 1,
    hint: "Inside the shape, only on its edge, or (circles) in a ring with a hole." },
  { key: "distribution", group: "start", label: "Distribution", kind: "select", value: "gaussian", local: true,
    options: (v) => distributionsFor(v.pattern, v.arrangement).map((k) => [k, DISTRIBUTION_LABELS[k]]),
    showIf: (v) => distributionsFor(v.pattern, v.arrangement).length > 1,
    hint: "How the robots are spaced: at random, evenly, evenly with a little randomness, or bunched toward the middle." },
  { key: "size", group: "start", label: (v) => SIZE_LABEL[v.pattern] ?? "Size", kind: "range", min: 20, max: 3000, step: 10, value: 600, local: true,
    showIf: (v) => SHAPES.includes(v.pattern) || v.pattern === "clusters", more: (v) => v.pattern === "clusters",
    hint: "In world units. For a polygon: corner to corner. For groups: the diameter of the circle their centres sit on." },
  { key: "sides", group: "start", label: "Sides", kind: "range", min: 3, max: 12, step: 1, value: 6, local: true,
    showIf: (v) => v.pattern === "polygon" },
  { key: "clusters", group: "start", label: "Number of groups", kind: "range", min: 1, max: 50, step: 1, value: 4, local: true,
    showIf: (v) => v.pattern === "clusters" },
  { key: "spread", group: "start", label: (v) => (v.pattern === "cloud" ? "How spread out" : v.pattern === "clusters" ? "Group size" : "How bunched"), kind: "range", min: 5, max: 1000, step: 5, value: 150, local: true,
    showIf: (v) => v.pattern === "cloud" || v.pattern === "clusters" || (SHAPES.includes(v.pattern) && v.distribution === "gaussian"),
    more: (v) => v.pattern !== "cloud",
    hint: "Standard deviation σ of the bunching: about 86 % of robots end up within 2σ of the centre, 99 % within 3σ. Smaller is tighter. (Groups placed at random: their radius.)" },
  { key: "aspect", group: "start", label: "Height / width", kind: "range", min: 0.1, max: 3, step: 0.05, value: 0.5, local: true,
    showIf: (v) => v.pattern === "rectangle", more: true },
  { key: "inner", group: "start", label: "Hole size", kind: "range", min: 0, max: 0.95, step: 0.05, value: 0.6, local: true,
    showIf: (v) => v.pattern === "circle" && v.arrangement === "ring", more: true, hint: "Width of the hole as a fraction of the circle's." },
  { key: "jitter", group: "start", label: "Messiness", kind: "range", min: 0, max: 1, step: 0.05, value: 0.3, local: true,
    showIf: (v) => v.distribution === "jittered" && SHAPES.includes(v.pattern), more: true,
    hint: "How far each robot is moved from its even spot: 0 = exactly even, 1 = up to half the gap to its neighbour." },
  { key: "rotation", group: "start", label: "Rotation °", kind: "range", min: 0, max: 360, step: 5, value: 0, local: true,
    showIf: (v) => ROTATABLE.includes(v.pattern), more: true },
  { key: "width_bound", group: "start", label: "World width", kind: "range", min: 50, max: 1000, step: 10, value: 600,
    showIf: (v) => !v.open_world, more: (v) => v.pattern !== "box",
    hint: "Size of the world box; with the original pattern robots start at random inside it." },
  { key: "height_bound", group: "start", label: "World height", kind: "range", min: 50, max: 1000, step: 10, value: 600,
    showIf: (v) => !v.open_world, more: (v) => v.pattern !== "box" },
  { key: "multiplicities", group: "start", label: "Stacked points", kind: "range", min: 0, max: 50, step: 1, value: 0, more: true,
    hint: "Start with this many spots where several robots stand on top of each other (multiplicities), made from the start above. 0: none." },
  { key: "multiplicity_size", group: "start", parent: "multiplicities", label: "Robots per stacked point", kind: "range", min: 0, max: 200, step: 1, value: 0, more: true,
    showIf: (v) => v.multiplicities > 0,
    hint: "At least 2. 0 lets the simulator choose: a third of the robots for one stacked point, up to 4 each for several." },
  { key: "custom", group: "start", label: "Positions", kind: "text", local: true,
    value: "# One robot per line: x, y\n-100, -100\n100, -100\n100, 100\n-100, 100",
    showIf: (v) => v.pattern === "custom" },

  { key: "robot_speeds", group: "movement", label: "Speed", kind: "range", min: 0.1, max: 10, step: 0.1, value: 1,
    hint: "Distance a robot covers per unit of simulated time." },
  { key: "rigid_movement", group: "movement", label: "Rigid movement", kind: "switch", value: true,
    hint: "Rigid: a robot always reaches its target. Off: set a minimum move δ below and robots can be stopped on the way. (Without δ, non-rigid robots still arrive, as in the original.)" },
  { key: "delta", group: "movement", parent: "rigid_movement", label: "Minimum move δ", kind: "range", min: 0, max: 500, step: 1, value: 0,
    showIf: (v) => !v.rigid_movement,
    hint: "A robot can be stopped on the way to its target, but never before it has covered δ (or reached the target, if that is nearer). 0: robots always arrive, as in the original." },
  { key: "stop_policy", group: "movement", parent: "rigid_movement", label: "Where it stops", kind: "select", value: "random",
    options: [["delta", "Right after δ"], ["random", "Anywhere after δ, at random"], ["fraction", "After a fraction of the way"]],
    showIf: (v) => !v.rigid_movement && v.delta > 0,
    hint: "Who decides where a stopped robot ends up: the worst case (just δ), a random point, or a fixed fraction of its way." },
  { key: "stop_fraction", group: "movement", parent: "rigid_movement", label: "Fraction of the way", kind: "range", min: 0.05, max: 1, step: 0.05, value: 0.5,
    showIf: (v) => !v.rigid_movement && v.delta > 0 && v.stop_policy === "fraction",
    hint: "How far along its way a stopped robot gets (never less than δ)." },
  { key: "unlimited_visibility", group: "visibility", label: "Unlimited visibility", kind: "switch", value: true, local: true,
    hint: "Every robot sees every other robot." },
  { key: "visibility_radius", group: "visibility", parent: "unlimited_visibility", label: "Radius", kind: "range", min: 10, max: 1000, step: 10, value: 150,
    showIf: (v) => !v.unlimited_visibility, hint: "How far a robot can see." },
  { key: "obstructed_visibility", group: "visibility", label: "Robots block the view", kind: "switch", value: false,
    hint: "A robot does not see a robot hidden behind another one (obstructed visibility). Each Look then costs time proportional to n², so keep the swarm small." },
  { key: "robot_radius", group: "visibility", parent: "obstructed_visibility", label: "Robot radius", kind: "range", min: 0, max: 20, step: 0.5, value: 0,
    showIf: (v) => v.obstructed_visibility,
    hint: "Robots block the view as discs of this radius. 0: points, which block only robots exactly in line behind them." },
  { key: "multiplicity", group: "model", label: "Multiplicity detection", kind: "select", value: "weak",
    options: [["none", "None: a shared point looks like any other"], ["local", "Local: only whether its own point is shared"], ["weak", "Weak: one robot, or more than one"], ["strong", "Strong: how many robots"]],
    hint: "What a robot can tell about a point several robots share. SqGathering needs at least weak detection: without it, one robot at a time can never gather. The ports of the original algorithms see every robot and ignore this." },
  { key: "frames", group: "model", label: "Coordinate frames", kind: "select", value: "global",
    options: [["global", "Global: all robots share one frame"], ["rotated", "Own frame: random rotation and unit"], ["disoriented", "Disoriented: also random handedness"], ["compass", "Compass with an error of up to φ"]],
    hint: "At each Look a robot can get a private coordinate system of its own, with itself at the origin. An algorithm that only uses distances and angles behaves the same; one that uses the axes (Spreading's grid) does not. Compass: the robot's north is off by at most φ." },
  { key: "compass_error", group: "model", parent: "frames", label: "Compass error φ °", kind: "range", min: 0, max: 180, step: 1, value: 30,
    showIf: (v) => v.frames === "compass",
    hint: "The largest angle between a robot's north and the true north. Units and handedness stay shared." },
  { key: "compass_static", group: "model", parent: "frames", label: "Static compass", kind: "switch", value: true,
    showIf: (v) => v.frames === "compass",
    hint: "On: each robot keeps one error for the whole run. Off (dynamic): a new error at every Look." },
  { key: "num_of_faults", group: "faults", label: "Faulty robots", kind: "range", min: 0, max: 1000, step: 1, value: 0,
    hint: "How many robots, chosen at random, have a fault." },
  { key: "fault_type", group: "faults", parent: "num_of_faults", label: "Fault type", kind: "select", value: "crash",
    showIf: (v) => v.num_of_faults > 0,
    options: [["crash", "Crash: stops for good"], ["byzantine", "Byzantine: moves erratically"],
      ["omission", "Omission: skips half its moves"], ["delay", "Delay: 40 % speed"], ["mixed", "Mixed: all four"]] },
  { key: "scheduler", group: "timing", label: "Scheduler", kind: "select", value: "async",
    options: [["async", "Async: robots act whenever they wake"], ["sequential", "Sequential: one robot at a time"],
      ["ssync", "SSYNC: rounds, some robots at once"], ["fsync", "FSYNC: rounds, every robot at once"],
      ["kasync", "k-ASYNC: async, bounded overlap"]],
    hint: "Async: every robot wakes on its own random clock, so many can be moving at once. Sequential: one robot does its whole Look, Compute and Move while every other robot stays still. SSYNC: in each round a set of robots looks at the same instant, then they all move; FSYNC: every robot, every round." },
  { key: "k_async", group: "timing", parent: "scheduler", label: "k (Looks per cycle)", kind: "range", min: 1, max: 20, step: 1, value: 1,
    showIf: (v) => v.scheduler === "kasync",
    hint: "k-ASYNC: while one robot is in a cycle, no other robot starts more than k Looks. A Look that would is put off until a cycle ends." },
  { key: "ssync_probability", group: "timing", parent: "scheduler", label: "Chance to be active", kind: "range", min: 0.05, max: 1, step: 0.05, value: 0.5,
    showIf: (v) => v.scheduler === "ssync",
    hint: "SSYNC: each robot still moving is active in a round with this chance. A round with nobody drawn activates one robot at random." },
  { key: "ssync_fairness", group: "timing", parent: "scheduler", label: "Fairness bound", kind: "range", min: 1, max: 50, step: 1, value: 4,
    showIf: (v) => v.scheduler === "ssync",
    hint: "SSYNC: a robot left out of this many rounds in a row is active in the next one, so every robot gets to act." },
  { key: "activation_order", group: "timing", parent: "scheduler", label: "Turn order", kind: "select", value: "round_robin",
    options: [["round_robin", "Round-robin: 0, 1, 2, …"], ["random", "Random: reshuffled every epoch"]],
    showIf: (v) => v.scheduler === "sequential",
    hint: "Who goes next. Each epoch, every robot still moving gets exactly one turn; crashed and terminated robots are skipped. Random uses the seed, so a run repeats." },
  { key: "turn_gap", group: "timing", parent: "scheduler", label: "Pause between turns", kind: "select", value: "random",
    options: [["random", "Random (rate λ)"], ["none", "None"]],
    showIf: (v) => v.scheduler !== "async" && v.scheduler !== "kasync",
    hint: "Simulated time between one turn (or round) ending and the next one starting. None: at once." },
  { key: "schedule_text", group: "timing", parent: "scheduler", label: (v) => (v.scheduler === "ssync" ? "Rounds" : "Turns"), kind: "line", value: "", local: true,
    showIf: (v) => v.scheduler === "sequential" || v.scheduler === "ssync", more: true,
    hint: "An explicit schedule instead of the order above. Sequential: robots in turn order, e.g. 0, 2, 1 (add @0.5 to stop a robot halfway: non-rigid with δ). SSYNC: the robots of each round, rounds separated by |, e.g. 0 1 | 2 3. Empty: the scheduler's own order. Playing the adversary (toolbar) writes one for you." },
  { key: "schedule_repeat", group: "timing", parent: "scheduler", label: "After the list", kind: "select", value: "repeat", local: true,
    options: (v) => [["repeat", "Play it again"], ["round_robin", v.scheduler === "ssync" ? "Random rounds" : "Round-robin"]],
    showIf: (v) => (v.scheduler === "sequential" || v.scheduler === "ssync") && v.schedule_text.trim() !== "", more: true,
    hint: "What follows the end of the list. Playing it again needs every robot in it, to stay fair." },
  { key: "max_turns", group: "timing", parent: "scheduler", label: "Turn limit", kind: "number", min: 0, max: 1000000000, value: 0,
    showIf: (v) => v.scheduler === "sequential",
    hint: "Stop after this many turns (one robot's whole Look, Compute and Move each). 0: no limit." },
  { key: "max_rounds", group: "timing", parent: "scheduler", label: "Round limit", kind: "number", min: 0, max: 1000000000, value: 0,
    showIf: (v) => v.scheduler === "ssync" || v.scheduler === "fsync",
    hint: "Stop after this many rounds. 0: no limit." },
  { key: "lambda_rate", group: "clock", label: "Activation rate λ", kind: "range", min: 0.1, max: 20, step: 0.1, value: 5,
    showIf: (v) => v.scheduler === "async" || v.scheduler === "kasync" || v.turn_gap === "random",
    note: (v) => `On average every ${+(1 / v.lambda_rate).toFixed(2)} time units`,
    hint: "Async: robots wake up after exponential delays with this rate (mean 1/λ). Sequential: the random pause between turns has this rate." },
  { key: "sampling_rate", group: "clock", label: "Sampling interval", kind: "range", min: 0.05, max: 1, step: 0.05, value: 0.1, more: true,
    hint: "Period of the scheduler's Visualize tick (kept for parity with the original)." },
  { key: "threshold_precision", group: "clock", label: "Precision (10⁻ᵖ)", kind: "range", min: 2, max: 15, step: 1, value: 5,
    note: (v) => `ε = ${Math.pow(10, -v.threshold_precision).toFixed(v.threshold_precision)} world units`,
    hint: "ε = 10^-p: moves shorter than ε count as arrived; robots within ε count as gathered." },
];

const byKey = Object.fromEntries(SETTINGS.map((s) => [s.key, s]));
const LOG_STEPS = 1000;
const toLog = (n, s) => Math.round((LOG_STEPS * Math.log(n / s.min)) / Math.log(s.max / s.min));
const fromLog = (v, s) => Math.round(s.min * Math.pow(s.max / s.min, v / LOG_STEPS));
const decimals = (s) => (String(s.step ?? 1).split(".")[1] ?? "").length;
const LOCK = `<svg class="lock" viewBox="0 0 24 24" width="12" height="12" aria-hidden="true"><rect x="5" y="11" width="14" height="10" rx="2" fill="none" stroke="currentColor" stroke-width="2"/><path d="M8 11V8a4 4 0 0 1 8 0v3" fill="none" stroke="currentColor" stroke-width="2"/></svg>`;
const escapeHtml = (t) => String(t).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);

/** One sentence on how the robots are timed with these settings (under the Scheduler group). */
export function describeTiming(v) {
  const unit = v.scheduler === "sequential" ? "turn" : "round";
  const pause = v.turn_gap === "none" ? `the next ${unit} starts at once` : `a random pause before each ${unit}`;
  const turnLimit = v.max_turns > 0 ? `; stops after ${v.max_turns.toLocaleString()} turns` : "";
  const roundLimit = v.max_rounds > 0 ? `; stops after ${v.max_rounds.toLocaleString()} rounds` : "";
  const listed = v.schedule_text?.trim() ? ", following your list of turns" : "";
  switch (v.scheduler) {
    case "sequential":
      return `One robot at a time${listed || (v.activation_order === "random" ? ", in a new random order each epoch" : ", in the order 0, 1, 2, …")}; ${pause}${turnLimit}.`;
    case "ssync":
      return `Rounds: each robot acts with chance ${v.ssync_probability} and is never left out more than ${v.ssync_fairness} rounds in a row${listed ? ", following your list of rounds" : ""}; ${pause}${roundLimit}.`;
    case "fsync":
      return `Rounds with every robot at once; ${pause}${roundLimit}.`;
    case "kasync":
      return `Robots wake on their own clocks, but no robot takes more than ${v.k_async} snapshot${v.k_async === 1 ? "" : "s"} while another is in the middle of its cycle.`;
    default:
      return `Robots wake on their own clocks, each about every ${+(1 / v.lambda_rate).toFixed(2)} time units, so several can be moving at once.`;
  }
}

/** Builds the settings panel into `root`; `onChange(key, values)` fires on every edit.
 *  `info`: { explain(key) -> card or null, explained: Set of keys } adds the info cards (web/explain.js). */
export function buildSettings(root, onChange, info = null) {
  const values = Object.fromEntries(SETTINGS.map((s) => [s.key, s.value]));
  const fields = {};
  const mores = []; // per group: its "More options" box
  const groupBoxes = []; // per group: its <details>, for groups shown only sometimes
  // Settings the chosen algorithm fixes: key -> { value, reason }. The user's own value is kept in `saved`
  // and put back when the algorithm no longer fixes it.
  let frozen = {};
  const saved = {};
  const summaries = []; // per group with `describe`: [group, <p>] — a sentence saying what its settings give
  let customResult = parseCoordinates(values.custom);
  const optionsOf = (s) => (typeof s.options === "function" ? s.options(values) : s.options);

  // Tabs: a bar of three, one pane each; groups without a tab (the advanced variant) sit above them.
  const tabBar = document.createElement("div");
  tabBar.className = "tabs";
  tabBar.setAttribute("role", "tablist");
  const panes = {};
  let tab = "robots";
  try { tab = localStorage.getItem("lcm-settings-tab") || tab; } catch {}
  if (!TABS.some(([id]) => id === tab)) tab = "robots";
  for (const [id, label] of TABS) {
    tabBar.insertAdjacentHTML("beforeend", `<button type="button" role="tab" class="tab" data-tab="${id}" id="tab-${id}" aria-controls="pane-${id}">${label}<span class="tab-lock" hidden></span></button>`);
    const pane = document.createElement("div");
    pane.className = "tab-pane";
    pane.id = `pane-${id}`;
    pane.dataset.tab = id;
    pane.setAttribute("role", "tabpanel");
    pane.setAttribute("aria-labelledby", `tab-${id}`);
    panes[id] = pane;
  }
  function selectTab(id) {
    tab = id;
    for (const b of tabBar.querySelectorAll(".tab")) b.setAttribute("aria-selected", String(b.dataset.tab === id));
    for (const [pid, pane] of Object.entries(panes)) pane.hidden = pid !== id;
    try { localStorage.setItem("lcm-settings-tab", id); } catch {}
  }
  tabBar.addEventListener("click", (e) => { const b = e.target.closest(".tab"); if (b) selectTab(b.dataset.tab); });
  tabBar.addEventListener("keydown", (e) => {
    const k = TABS.findIndex(([id]) => id === tab), d = e.key === "ArrowRight" ? 1 : e.key === "ArrowLeft" ? -1 : 0;
    if (!d) return;
    e.preventDefault();
    const next = TABS[(k + d + TABS.length) % TABS.length][0];
    selectTab(next);
    tabBar.querySelector(`[data-tab="${next}"]`).focus();
  });
  let tabsPlaced = false;
  const placeTabs = () => { if (!tabsPlaced) { root.append(tabBar, ...Object.values(panes)); tabsPlaced = true; } };

  for (const g of GROUPS) {
    const details = document.createElement("details");
    details.className = "group";
    details.open = g.open;
    details.dataset.group = g.id;
    details.innerHTML = `<summary><span class="chev" data-icon="chevron" data-size="14"></span>${g.label}</summary><div class="group-body"></div>`;
    if (g.card) details.classList.add("algo-group");
    groupBoxes.push({ group: g, details });
    const body = details.querySelector(".group-body");
    const own = SETTINGS.filter((x) => x.group === g.id);
    for (const s of own) {
      const field = document.createElement("div");
      field.className = s.parent ? "field sub-option" : "field";
      field.dataset.key = s.key;
      const id = `set-${s.key}`;
      // The help text shows on hovering the label (dotted underline).
      const text = typeof s.label === "function" ? s.label(values) : s.label;
      const label = (target) => `<label for="${target}"${s.hint ? ` class="has-hint" title="${escapeHtml(s.hint)}"` : ""}>${text}</label>`;
      if (s.kind === "switch") {
        field.innerHTML = `<div class="field-top">${label(id)}<span class="switch"><input id="${id}" type="checkbox"><span></span></span></div>`;
      } else if (s.kind === "select") {
        field.innerHTML = `<div class="field-top">${label(id)}</div><select id="${id}"></select>`;
      } else if (s.kind === "number") {
        field.innerHTML = `<div class="field-top">${label(id)}<input id="${id}" type="number" min="${s.min}" max="${s.max}" step="1">${s.dice ? `<button class="btn small outline" type="button" title="Random seed" aria-label="Random seed" data-icon="dice" data-size="15"></button>` : ""}</div>`;
      } else if (s.kind === "line") {
        field.innerHTML = `<div class="field-top">${label(id)}</div><input id="${id}" type="text" spellcheck="false" autocomplete="off" placeholder="empty: none">`;
      } else if (s.kind === "text") {
        // The editor opens in its own window; the panel shows a summary.
        field.innerHTML = `<div class="field-top">${label(`${id}-edit`)}<button id="${id}-edit" class="btn small outline" type="button">Edit…</button></div><div class="sub" id="${id}-summary"></div>`;
      } else {
        const range = s.kind === "log" ? `min="0" max="${LOG_STEPS}" step="1"` : `min="${s.min}" max="${s.max}" step="${s.step}"`;
        field.innerHTML = `<div class="field-top">${label(`${id}-n`)}<input id="${id}-n" type="number" min="${s.min}" max="${s.max}" step="${s.step ?? 1}"></div><input id="${id}" type="range" ${range} aria-label="${s.label}">`;
      }
      if (info?.explained.has(s.key)) {
        field.querySelector(".field-top label")?.insertAdjacentHTML("afterend",
          `<button type="button" class="info-btn" data-info="${s.key}" aria-label="What does this setting mean?" title="What does this setting mean?" aria-expanded="false">i</button>`);
      }
      if (s.note) field.insertAdjacentHTML("beforeend", `<div class="sub note"></div>`);
      body.append(field);
      fields[s.key] = field;
    }
    if (g.card) {
      // Filled by the page: the variant's guarantee and what it locks.
      const card = document.createElement("div");
      card.className = "algo-card";
      card.id = "algo_card";
      body.append(card);
    }
    if (g.describe) {
      const summary = document.createElement("p");
      summary.className = "start-summary";
      summary.id = `${g.id}-summary`;
      summary.setAttribute("aria-live", "polite");
      body.append(summary);
      summaries.push([g, summary]);
    }
    if (own.some((x) => x.more)) {
      const more = document.createElement("details");
      more.className = "more";
      more.dataset.more = g.id;
      more.innerHTML = `<summary><span class="chev" data-icon="chevron" data-size="12"></span>More options</summary><div class="more-body"></div>`;
      body.append(more);
      mores.push({ group: g.id, details: more, body: more.querySelector(".more-body"), main: body });
    }
    if (g.tab) { placeTabs(); panes[g.tab].append(details); } else root.append(details);
  }
  placeTabs();
  selectTab(tab);
  let built = false; // settings shown while the panel is built do not animate
  /** Marks the tabs holding a setting the chosen algorithm locks. */
  function markTabs() {
    for (const [id] of TABS) {
      const locked = Object.keys(frozen).some((key) => byKey[key] && GROUPS.find((g) => g.id === byKey[key].group)?.tab === id);
      const mark = tabBar.querySelector(`[data-tab="${id}"] .tab-lock`);
      mark.hidden = !locked;
      mark.title = locked ? "Some settings here are fixed by the algorithm" : "";
    }
  }
  const editor = buildCoordinateEditor((text) => set("custom", text));

  function clamp(s, v) {
    if (s.kind === "switch") return Boolean(v);
    if (s.kind === "text" || s.kind === "line") return String(v);
    if (s.kind === "select") { const opts = optionsOf(s); return opts.some(([o]) => o === v) ? v : opts[0]?.[0] ?? s.value; }
    let n = Number(v);
    if (!Number.isFinite(n)) n = s.value;
    n = Math.min(s.max, Math.max(s.min, n));
    return s.kind === "log" || s.kind === "number" ? Math.round(n) : Number(n.toFixed(decimals(s)));
  }

  function show(key) {
    const s = byKey[key], f = fields[key], v = values[key], id = `set-${key}`;
    if (s.kind === "switch") f.querySelector("input").checked = v;
    else if (s.kind === "select") {
      const sel = f.querySelector("select");
      let html = "", heading = null;
      for (const [o, t, g] of optionsOf(s)) {
        if (g !== heading) { html += `${heading ? "</optgroup>" : ""}${g ? `<optgroup label="${g}">` : ""}`; heading = g; }
        html += `<option value="${o}">${t}</option>`;
      }
      if (heading) html += "</optgroup>";
      if (sel.dataset.html !== html) { sel.innerHTML = html; sel.dataset.html = html; }
      sel.value = v;
    } else if (s.kind === "number") f.querySelector(`#${id}`).value = v;
    else if (s.kind === "text") { if (editor.text.value !== v) editor.text.value = v; }
    else if (s.kind === "line") { const input = f.querySelector("input"); if (input.value !== v) input.value = v; }
    else {
      f.querySelector(`#${id}`).value = s.kind === "log" ? toLog(v, s) : v;
      f.querySelector(`#${id}-n`).value = v;
    }
  }

  function showCustomStatus() {
    const { points, errors } = customResult;
    const summary = fields.custom.querySelector(".sub");
    const robots = `${points.length.toLocaleString()} robot${points.length === 1 ? "" : "s"}`;
    if (errors.length) {
      editor.status.className = "sub error";
      editor.status.textContent = errors.slice(0, 3).map((e) => `Line ${e.line}: ${e.message}`).join(" · ") + (errors.length > 3 ? ` · and ${errors.length - 3} more` : "");
      summary.className = "sub error";
      summary.textContent = `${errors.length} problem${errors.length === 1 ? "" : "s"}: open the editor to fix`;
    } else {
      editor.status.className = summary.className = "sub";
      editor.status.textContent = robots;
      summary.textContent = `${robots} loaded`;
    }
  }

  /** Re-derive everything that depends on other settings. */
  function refresh() {
    // Choices that stopped being valid fall back to the first valid one.
    for (const key of ["pattern", "arrangement", "distribution"]) {
      const s = byKey[key], opts = optionsOf(s);
      if (!opts.some(([o]) => o === values[key])) values[key] = opts[0]?.[0] ?? s.value;
      show(key);
    }
    if (values.pattern === "custom") {
      customResult = parseCoordinates(values.custom);
      if (!customResult.errors.length) { values.num_of_robots = customResult.points.length; show("num_of_robots"); }
      showCustomStatus();
    }
    // Main settings stay in the group; the rest go under "More options",
    // which is hidden when nothing in it applies.
    for (const m of mores) {
      let any = false;
      for (const s of SETTINGS.filter((x) => x.group === m.group)) {
        const more = typeof s.more === "function" ? s.more(values) : Boolean(s.more);
        (more ? m.body : m.main).append(fields[s.key]);
        if (more && (!s.showIf || s.showIf(values))) any = true;
      }
      for (const [, summary] of summaries) if (m.main.contains(summary)) m.main.append(summary);
      m.main.append(m.details);
      m.details.hidden = !any;
    }
    for (const s of SETTINGS) {
      if (typeof s.label === "function") fields[s.key].querySelector("label").textContent = s.label(values);
      if (s.note) fields[s.key].querySelector(".note").textContent = s.note(values);
      if (s.showIf) {
        const f = fields[s.key], hide = !s.showIf(values);
        if (f.hidden && !hide && built) { // it appears because of a choice just made: let it fade in
          f.classList.remove("appear");
          void f.offsetWidth;
          f.classList.add("appear");
        }
        f.hidden = hide;
      }
      if (s.key === cardKey && fields[s.key].hidden) closeInfo();
      if (s.enabledBy) {
        const on = s.enabledBy(values);
        fields[s.key].classList.toggle("disabled", !on);
        for (const el of fields[s.key].querySelectorAll("input, select, textarea")) el.disabled = !on;
      }
    }
    // Frozen settings: shown, locked, with the reason.
    for (const s of SETTINGS) {
      const f = fields[s.key], lock = frozen[s.key];
      f.classList.toggle("frozen", Boolean(lock));
      let note = f.querySelector(".lock-note");
      if (lock) {
        for (const el of f.querySelectorAll("input, select, textarea, button")) el.disabled = true;
        if (!note) { note = document.createElement("div"); note.className = "sub lock-note"; f.append(note); }
        note.innerHTML = `${LOCK}<span>${escapeHtml(lock.reason)}</span>`;
      } else note?.remove();
    }
    for (const { group, details } of groupBoxes) if (group.showIf) details.hidden = !group.showIf(values);
    for (const [g, summary] of summaries) {
      summary.textContent = typeof g.describe === "function" ? g.describe(values) : describeStart(values, customResult.points.length);
    }
    fields.num_of_faults.querySelector('input[type="range"]').max = Math.min(1000, values.num_of_robots);
  }

  // Info cards: one at a time, inside the field it explains (so it moves with it into and out of
  // "More options"). Changing a setting opens its card unless the reader turned that off; ⓘ always does.
  let cardKey = null, cardTimer = 0;
  let autoCards = true;
  try { autoCards = localStorage.getItem("lcm-info-auto") !== "0"; } catch {}
  function closeInfo() {
    const card = cardKey && fields[cardKey]?.querySelector(".info-card");
    if (cardKey) fields[cardKey]?.querySelector(".info-btn")?.setAttribute("aria-expanded", "false");
    if (card) {
      card.classList.add("leaving");
      const done = () => card.remove();
      card.addEventListener("animationend", done, { once: true });
      setTimeout(done, 400); // reduced motion: no animation to wait for
    }
    cardKey = null;
  }
  function showInfo(key, { focus = false } = {}) {
    const card = info?.explain(key);
    if (!card || fields[key]?.hidden) { if (cardKey === key) closeInfo(); return; }
    const html = `<div class="info-head"><span class="info-mark" aria-hidden="true">i</span><span class="info-title">${escapeHtml(card.title)}</span>`
      + `<button type="button" class="info-close" aria-label="Close this explanation" title="Close">×</button></div>`
      + card.paras.map((x) => `<p>${x}</p>`).join("")
      + (card.tip ? `<p class="info-tip">${card.tip}</p>` : "")
      + `<label class="info-auto"><input type="checkbox"${autoCards ? " checked" : ""}> Explain a setting whenever I change it</label>`;
    let el = cardKey === key ? fields[key].querySelector(".info-card:not(.leaving)") : null;
    if (!el) {
      closeInfo();
      el = document.createElement("div");
      el.className = "info-card";
      el.setAttribute("role", "note");
      fields[key].append(el);
    }
    el.innerHTML = html; // same setting: the text follows the value without animating again
    cardKey = key;
    fields[key].querySelector(".info-btn")?.setAttribute("aria-expanded", "true");
    el.querySelector(".info-close").addEventListener("click", () => { closeInfo(); fields[key]?.querySelector(".info-btn")?.focus(); });
    el.querySelector(".info-auto input").addEventListener("change", (e) => {
      autoCards = e.target.checked;
      try { localStorage.setItem("lcm-info-auto", autoCards ? "1" : "0"); } catch {}
    });
    if (focus) el.querySelector(".info-close").focus({ preventScroll: true });
    el.scrollIntoView({ block: "nearest", behavior: matchMedia("(prefers-reduced-motion: reduce)").matches ? "auto" : "smooth" });
  }

  function set(key, v, notify = true) {
    const s = byKey[key];
    if (frozen[key]) { show(key); return; } // locked by the algorithm
    const before = values.open_world;
    values[key] = clamp(s, v);
    if (key === "open_world" && values.open_world !== before) {
      // Switching worlds picks that world's natural start.
      values.pattern = values.open_world ? "cloud" : "box";
      show("pattern");
    }
    if (key === "pattern" || key === "open_world") {
      // A new pattern starts filled (a line: on its edge)…
      values.arrangement = (ARRANGEMENTS[values.pattern] ?? [])[0] ?? values.arrangement;
      show("arrangement");
    }
    if (key === "pattern" || key === "open_world" || key === "arrangement") {
      // …with the spacing that suits it (a line evenly spaced, a disk at random, …).
      values.distribution = naturalDistribution(values.pattern, values.arrangement);
      show("distribution");
    }
    if (key === "num_of_robots" && values.num_of_faults > values.num_of_robots) {
      values.num_of_faults = values.num_of_robots;
      show("num_of_faults");
    }
    show(key);
    refresh();
    if (notify) onChange(key, values);
    // A slider fires on every step: explain once it settles.
    if (notify && autoCards && info?.explained.has(key)) {
      clearTimeout(cardTimer);
      cardTimer = setTimeout(() => showInfo(key), byKey[key].kind === "range" || byKey[key].kind === "log" ? 350 : 0);
    } else if (notify && cardKey === key) showInfo(key);
  }

  // ⓘ opens (or closes) the card of its setting.
  root.addEventListener("click", (e) => {
    const btn = e.target.closest(".info-btn");
    if (!btn) return;
    e.preventDefault();
    if (cardKey === btn.dataset.info) closeInfo(); else showInfo(btn.dataset.info, { focus: true });
  });

  for (const s of SETTINGS) {
    const f = fields[s.key], id = `set-${s.key}`;
    if (s.kind === "switch") f.querySelector("input").addEventListener("change", (e) => set(s.key, e.target.checked));
    else if (s.kind === "select") f.querySelector("select").addEventListener("change", (e) => {
      set(s.key, e.target.value);
      if (s.key === "pattern" && e.target.value === "custom") editor.open(values.custom);
    });
    else if (s.kind === "text") f.querySelector("button").addEventListener("click", () => editor.open(values[s.key]));
    else if (s.kind === "line") f.querySelector("input").addEventListener("change", (e) => set(s.key, e.target.value));
    else if (s.kind === "number") {
      f.querySelector(`#${id}`).addEventListener("change", (e) => set(s.key, e.target.value));
      f.querySelector("button")?.addEventListener("click", () => set(s.key, Math.floor(Math.random() * 1e9)));
    } else {
      f.querySelector(`#${id}`).addEventListener("input", (e) => set(s.key, s.kind === "log" ? fromLog(+e.target.value, s) : e.target.value));
      f.querySelector(`#${id}-n`).addEventListener("change", (e) => set(s.key, e.target.value));
    }
  }
  refresh();
  for (const s of SETTINGS) show(s.key);
  refresh();
  built = true;

  return {
    values,
    set,
    editor,
    /** Opens the info card of `key` (web/explain.js), or closes the open one. */
    showInfo,
    closeInfo,
    /** Errors that stop a run from being built (custom coordinates only). */
    get problem() { return values.pattern === "custom" && customResult.errors.length ? customResult.errors : null; },
    get customPoints() { return customResult.points; },
    /** Sets several settings at once (in the order given) and restarts the run once. */
    apply(entries) { for (const [key, value] of Object.entries(entries)) set(key, value, false); onChange(null, values); },
    reset() {
      for (const s of SETTINGS) values[s.key] = s.value;
      for (const [key, lock] of Object.entries(frozen)) values[key] = lock.value;
      refresh(); for (const s of SETTINGS) show(s.key); refresh(); onChange(null, values);
    },
    /** Lock settings for the chosen algorithm (`{ key: { value, reason } }`); earlier locks not named here are lifted and
     * the user's own values come back. Does not restart the run. */
    freeze(locks) {
      for (const key of Object.keys(frozen)) {
        if (!(key in locks) && key in saved) { values[key] = saved[key]; delete saved[key]; }
      }
      for (const [key, lock] of Object.entries(locks)) {
        if (!(key in frozen)) saved[key] = values[key];
        values[key] = lock.value;
      }
      // Two robots from "my own positions" with more points: use a generated start instead.
      if (locks.num_of_robots && values.pattern === "custom" && customResult.points.length !== locks.num_of_robots.value) {
        saved.pattern ??= values.pattern;
        values.pattern = "cloud";
        values.arrangement = (ARRANGEMENTS.cloud ?? [])[0] ?? values.arrangement;
        values.distribution = naturalDistribution("cloud", values.arrangement);
      }
      frozen = locks;
      for (const s of SETTINGS) show(s.key);
      refresh();
      markTabs();
    },
    /** Shows a tab of the panel ("robots", "model" or "timing"). */
    selectTab,
    get frozen() { return frozen; },
    /** Extra values the panel's conditions read (e.g. `family`), then re-draw. */
    setContext(extra) { Object.assign(values, extra); refresh(); },
    refresh,
  };
}

/** The window for typing or pasting custom coordinates; `onInput(text)` fires on every edit. */
function buildCoordinateEditor(onInput) {
  const overlay = document.createElement("div");
  overlay.id = "coords";
  overlay.className = "overlay";
  overlay.hidden = true;
  overlay.innerHTML = `<div class="panel dialog wide" role="dialog" aria-modal="true" aria-labelledby="coords_title">
    <button class="btn small close" data-action="close" aria-label="Close"><span data-icon="close" data-size="15"></span></button>
    <h2 id="coords_title">Custom coordinates</h2>
    <p class="sub">One robot per line as <code>x, y</code> (comma, space, tab or <code>;</code>). Lines starting with <code>#</code> are ignored.
      You can also paste the simulator's CSV export, or JSON <code>[[x, y], …]</code>. The run updates as you type.</p>
    <textarea id="set-custom" rows="14" spellcheck="false" autocomplete="off" aria-label="Coordinates"></textarea>
    <div class="dialog-foot"><span class="sub" id="set-custom-status" aria-live="polite"></span><button class="btn primary" type="button" data-action="close">Done</button></div>
  </div>`;
  document.body.append(overlay);
  const text = overlay.querySelector("textarea");
  const close = () => { overlay.hidden = true; document.getElementById("set-custom-edit")?.focus(); };
  text.addEventListener("input", () => onInput(text.value));
  overlay.addEventListener("click", (e) => { if (e.target === overlay || e.target.closest("[data-action=close]")) close(); });
  overlay.addEventListener("keydown", (e) => { if (e.key === "Escape") { e.stopPropagation(); close(); } });
  return {
    text,
    status: overlay.querySelector("#set-custom-status"),
    get isOpen() { return !overlay.hidden; },
    open(value) { if (text.value !== value) text.value = value; overlay.hidden = false; text.focus(); },
    close,
  };
}

/** SimConfig for the core from the panel's values plus the chosen algorithm. */
export function readConfig(values, algorithm, customPoints = []) {
  const config = { algorithm };
  for (const s of SETTINGS) if (!s.local) config[s.key] = values[s.key];
  config.visibility_radius = values.unlimited_visibility ? null : values.visibility_radius;
  config.delta = !values.rigid_movement && values.delta > 0 ? values.delta : null;
  config.max_turns = values.scheduler === "sequential" && values.max_turns > 0 ? values.max_turns : null;
  config.max_rounds = (values.scheduler === "ssync" || values.scheduler === "fsync") && values.max_rounds > 0 ? values.max_rounds : null;
  Object.assign(config, parseSchedule(values.schedule_text, values.scheduler));
  if (config.schedule || config.rounds) config.schedule_end = values.schedule_repeat;
  config.open_world = values.open_world;
  config.start = null;
  config.initial_positions = null;
  const p = values.pattern;
  if (p === "custom") {
    config.initial_positions = customPoints;
    config.num_of_robots = customPoints.length;
  } else if (p !== "box") {
    config.start = {
      pattern: p, arrangement: values.arrangement, distribution: values.distribution,
      size: values.size, aspect: values.aspect, inner: values.inner, sides: values.sides,
      clusters: values.clusters, spread: values.spread, jitter: values.jitter, rotation: values.rotation,
    };
  }
  return config;
}

/** The explicit schedule typed in Settings → Scheduler, as SimConfig fields: `schedule` (sequential:
 * "0, 2, 1@0.5") or `rounds` (SSYNC: "0 1 | 2 3"). Nothing for an empty line or another scheduler;
 * the core reports robots that do not exist. */
export function parseSchedule(text, scheduler) {
  const words = (part) => part.split(/[\s,;]+/).filter(Boolean);
  if (!text?.trim()) return {};
  if (scheduler === "sequential") {
    return {
      schedule: words(text.replace(/\|/g, " ")).map((w) => {
        const [robot, stop] = w.split("@");
        return stop === undefined ? Number(robot) : { robot: Number(robot), stop: Number(stop) };
      }),
    };
  }
  if (scheduler === "ssync") return { rounds: text.split("|").map((part) => words(part).map(Number)).filter((r) => r.length) };
  return {};
}

/** The other way round: a schedule as the line Settings → Scheduler takes. */
export function formatSchedule(list, scheduler) {
  if (scheduler === "ssync") return list.map((round) => round.join(" ")).join(" | ");
  return list.map((a) => (typeof a === "number" ? String(a) : `${a.robot}@${+a.stop.toFixed(3)}`)).join(", ");
}

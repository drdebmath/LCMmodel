/* Simulation settings, defined once. The panel is built from this list and
 * `readConfig()` turns it into the core's SimConfig (docs/core-schema.md §2),
 * so adding a setting is one entry here. */
import { ARRANGEMENTS, ARRANGEMENT_LABELS, DISTRIBUTION_LABELS, PATTERNS, describeStart, distributionsFor, naturalDistribution, parseCoordinates } from "./start.js";

export const GROUPS = [
  { id: "swarm", label: "Swarm", open: true },
  { id: "start", label: "Start", open: true, describe: true },
  { id: "movement", label: "Movement", open: false },
  { id: "visibility", label: "Visibility", open: true },
  { id: "faults", label: "Faults", open: false },
  { id: "timing", label: "Scheduler", open: false },
];

const SHAPES = ["square", "rectangle", "circle", "polygon", "line"];
const ROTATABLE = ["square", "rectangle", "line", "polygon", "clusters"];
const SIZE_LABEL = { square: "Side length", rectangle: "Width", circle: "Diameter", polygon: "Diameter", line: "Length", clusters: "Distance between groups" };

// kind: "range" (slider + number), "log" (log-scale slider + number), "switch",
// "select" (options: a list, or a function of the values), "number", "text".
// `key` is the SimConfig field unless `local`; `label` may be a function of the
// values; `note` is a line under the control; `showIf` hides a setting that
// does not apply; `enabledBy` greys it out; `more` puts it under "More options"
// (a function of the values, or true).
export const SETTINGS = [
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
  { key: "multiplicity_size", group: "start", label: "Robots per stacked point", kind: "range", min: 0, max: 200, step: 1, value: 0, more: true,
    showIf: (v) => v.multiplicities > 0,
    hint: "At least 2. 0 lets the simulator choose: a third of the robots for one stacked point, up to 4 each for several." },
  { key: "custom", group: "start", label: "Positions", kind: "text", local: true,
    value: "# One robot per line: x, y\n-100, -100\n100, -100\n100, 100\n-100, 100",
    showIf: (v) => v.pattern === "custom" },

  { key: "robot_speeds", group: "movement", label: "Speed", kind: "range", min: 0.1, max: 10, step: 0.1, value: 1,
    hint: "Distance a robot covers per unit of simulated time." },
  { key: "rigid_movement", group: "movement", label: "Rigid movement", kind: "switch", value: true,
    hint: "Rigid: a robot always reaches its target. Off: set a minimum move δ below and robots can be stopped on the way. (Without δ, non-rigid robots still arrive, as in the original.)" },
  { key: "delta", group: "movement", label: "Minimum move δ", kind: "range", min: 0, max: 500, step: 1, value: 0,
    showIf: (v) => !v.rigid_movement,
    hint: "A robot can be stopped on the way to its target, but never before it has covered δ (or reached the target, if that is nearer). 0: robots always arrive, as in the original." },
  { key: "stop_policy", group: "movement", label: "Where it stops", kind: "select", value: "random",
    options: [["delta", "Right after δ"], ["random", "Anywhere after δ, at random"], ["fraction", "After a fraction of the way"]],
    showIf: (v) => !v.rigid_movement && v.delta > 0,
    hint: "Who decides where a stopped robot ends up: the worst case (just δ), a random point, or a fixed fraction of its way." },
  { key: "stop_fraction", group: "movement", label: "Fraction of the way", kind: "range", min: 0.05, max: 1, step: 0.05, value: 0.5,
    showIf: (v) => !v.rigid_movement && v.delta > 0 && v.stop_policy === "fraction",
    hint: "How far along its way a stopped robot gets (never less than δ)." },
  { key: "unlimited_visibility", group: "visibility", label: "Unlimited visibility", kind: "switch", value: true, local: true,
    hint: "Every robot sees every other robot." },
  { key: "visibility_radius", group: "visibility", label: "Radius", kind: "range", min: 10, max: 1000, step: 10, value: 150,
    enabledBy: (v) => !v.unlimited_visibility, hint: "How far a robot can see." },
  { key: "num_of_faults", group: "faults", label: "Faulty robots", kind: "range", min: 0, max: 1000, step: 1, value: 0,
    hint: "How many robots, chosen at random, have a fault." },
  { key: "fault_type", group: "faults", label: "Fault type", kind: "select", value: "crash",
    enabledBy: (v) => v.num_of_faults > 0,
    options: [["crash", "Crash: stops for good"], ["byzantine", "Byzantine: moves erratically"],
      ["omission", "Omission: skips half its moves"], ["delay", "Delay: 40 % speed"], ["mixed", "Mixed: all four"]] },
  { key: "scheduler", group: "timing", label: "Scheduler", kind: "select", value: "async",
    options: [["async", "Async: robots act whenever they wake"], ["sequential", "Sequential: one robot at a time"]],
    hint: "Async: every robot wakes on its own random clock, so many can be moving at once. Sequential: one robot does its whole Look, Compute and Move while every other robot stays still." },
  { key: "activation_order", group: "timing", label: "Turn order", kind: "select", value: "round_robin",
    options: [["round_robin", "Round-robin: 0, 1, 2, …"], ["random", "Random: reshuffled every epoch"]],
    showIf: (v) => v.scheduler === "sequential",
    hint: "Who goes next. Each epoch, every robot still moving gets exactly one turn; crashed and terminated robots are skipped. Random uses the seed, so a run repeats." },
  { key: "turn_gap", group: "timing", label: "Pause between turns", kind: "select", value: "random",
    options: [["random", "Random (rate λ)"], ["none", "None"]],
    showIf: (v) => v.scheduler === "sequential",
    hint: "Simulated time between one robot stopping and the next one looking. None: the next robot looks at once." },
  { key: "max_turns", group: "timing", label: "Turn limit", kind: "number", min: 0, max: 1000000000, value: 0,
    showIf: (v) => v.scheduler === "sequential",
    hint: "Stop after this many turns (one robot's whole Look, Compute and Move each). 0: no limit." },
  { key: "lambda_rate", group: "timing", label: "Activation rate λ", kind: "range", min: 0.1, max: 20, step: 0.1, value: 5,
    enabledBy: (v) => v.scheduler !== "sequential" || v.turn_gap === "random",
    hint: "Async: robots wake up after exponential delays with this rate (mean 1/λ). Sequential: the random pause between turns has this rate." },
  { key: "sampling_rate", group: "timing", label: "Sampling interval", kind: "range", min: 0.05, max: 1, step: 0.05, value: 0.1,
    hint: "Period of the scheduler's Visualize tick (kept for parity with the original)." },
  { key: "threshold_precision", group: "timing", label: "Precision (10⁻ᵖ)", kind: "range", min: 2, max: 15, step: 1, value: 5,
    note: (v) => `ε = ${Math.pow(10, -v.threshold_precision).toFixed(v.threshold_precision)} world units`,
    hint: "ε = 10^-p: moves shorter than ε count as arrived; robots within ε count as gathered." },
];

const byKey = Object.fromEntries(SETTINGS.map((s) => [s.key, s]));
const LOG_STEPS = 1000;
const toLog = (n, s) => Math.round((LOG_STEPS * Math.log(n / s.min)) / Math.log(s.max / s.min));
const fromLog = (v, s) => Math.round(s.min * Math.pow(s.max / s.min, v / LOG_STEPS));
const decimals = (s) => (String(s.step ?? 1).split(".")[1] ?? "").length;
const escapeHtml = (t) => String(t).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);

/** Builds the settings panel into `root`; `onChange(key, values)` fires on every edit. */
export function buildSettings(root, onChange) {
  const values = Object.fromEntries(SETTINGS.map((s) => [s.key, s.value]));
  const fields = {};
  const mores = []; // per group: its "More options" box
  let summary = null; // the sentence saying what the start settings give
  let customResult = parseCoordinates(values.custom);
  const optionsOf = (s) => (typeof s.options === "function" ? s.options(values) : s.options);

  for (const g of GROUPS) {
    const details = document.createElement("details");
    details.className = "group";
    details.open = g.open;
    details.dataset.group = g.id;
    details.innerHTML = `<summary><span class="chev" data-icon="chevron" data-size="14"></span>${g.label}</summary><div class="group-body"></div>`;
    const body = details.querySelector(".group-body");
    const own = SETTINGS.filter((x) => x.group === g.id);
    for (const s of own) {
      const field = document.createElement("div");
      field.className = "field";
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
      } else if (s.kind === "text") {
        // The editor opens in its own window; the panel shows a summary.
        field.innerHTML = `<div class="field-top">${label(`${id}-edit`)}<button id="${id}-edit" class="btn small outline" type="button">Edit…</button></div><div class="sub" id="${id}-summary"></div>`;
      } else {
        const range = s.kind === "log" ? `min="0" max="${LOG_STEPS}" step="1"` : `min="${s.min}" max="${s.max}" step="${s.step}"`;
        field.innerHTML = `<div class="field-top">${label(`${id}-n`)}<input id="${id}-n" type="number" min="${s.min}" max="${s.max}" step="${s.step ?? 1}"></div><input id="${id}" type="range" ${range} aria-label="${s.label}">`;
      }
      if (s.note) field.insertAdjacentHTML("beforeend", `<div class="sub note"></div>`);
      body.append(field);
      fields[s.key] = field;
    }
    if (g.describe) {
      summary = document.createElement("p");
      summary.className = "start-summary";
      summary.id = `${g.id}-summary`;
      summary.setAttribute("aria-live", "polite");
      body.append(summary);
    }
    if (own.some((x) => x.more)) {
      const more = document.createElement("details");
      more.className = "more";
      more.dataset.more = g.id;
      more.innerHTML = `<summary><span class="chev" data-icon="chevron" data-size="12"></span>More options</summary><div class="more-body"></div>`;
      body.append(more);
      mores.push({ group: g.id, details: more, body: more.querySelector(".more-body"), main: body });
    }
    root.append(details);
  }
  const editor = buildCoordinateEditor((text) => set("custom", text));

  function clamp(s, v) {
    if (s.kind === "switch") return Boolean(v);
    if (s.kind === "text") return String(v);
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
      if (summary && m.main.contains(summary)) m.main.append(summary);
      m.main.append(m.details);
      m.details.hidden = !any;
    }
    for (const s of SETTINGS) {
      if (typeof s.label === "function") fields[s.key].querySelector("label").textContent = s.label(values);
      if (s.note) fields[s.key].querySelector(".note").textContent = s.note(values);
      if (s.showIf) fields[s.key].hidden = !s.showIf(values);
      if (s.enabledBy) {
        const on = s.enabledBy(values);
        fields[s.key].classList.toggle("disabled", !on);
        for (const el of fields[s.key].querySelectorAll("input, select, textarea")) el.disabled = !on;
      }
    }
    if (summary) summary.textContent = describeStart(values, customResult.points.length);
    fields.num_of_faults.querySelector('input[type="range"]').max = Math.min(1000, values.num_of_robots);
  }

  function set(key, v, notify = true) {
    const s = byKey[key];
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
  }

  for (const s of SETTINGS) {
    const f = fields[s.key], id = `set-${s.key}`;
    if (s.kind === "switch") f.querySelector("input").addEventListener("change", (e) => set(s.key, e.target.checked));
    else if (s.kind === "select") f.querySelector("select").addEventListener("change", (e) => {
      set(s.key, e.target.value);
      if (s.key === "pattern" && e.target.value === "custom") editor.open(values.custom);
    });
    else if (s.kind === "text") f.querySelector("button").addEventListener("click", () => editor.open(values[s.key]));
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

  return {
    values,
    set,
    editor,
    /** Errors that stop a run from being built (custom coordinates only). */
    get problem() { return values.pattern === "custom" && customResult.errors.length ? customResult.errors : null; },
    get customPoints() { return customResult.points; },
    reset() { for (const s of SETTINGS) values[s.key] = s.value; refresh(); for (const s of SETTINGS) show(s.key); refresh(); onChange(null, values); },
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

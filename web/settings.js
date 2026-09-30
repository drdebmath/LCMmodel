/* Simulation settings, defined once. The panel is built from this list and
 * `readConfig()` turns it into the core's SimConfig (docs/core-schema.md §2),
 * so adding a setting is one entry here. */

export const GROUPS = [
  { id: "swarm", label: "Swarm", open: true },
  { id: "movement", label: "Movement", open: true },
  { id: "visibility", label: "Visibility", open: true },
  { id: "faults", label: "Faults", open: false },
  { id: "world", label: "World", open: false },
  { id: "timing", label: "Scheduler", open: false },
];

// kind: "range" (slider + number), "log" (log-scale slider + number),
// "switch", "select", "number". `key` is the SimConfig field.
export const SETTINGS = [
  { key: "num_of_robots", group: "swarm", label: "Robots", kind: "log", min: 1, max: 10000, value: 500,
    hint: "Number of robots. Large swarms hide outlines and target lines to keep drawing fast." },
  { key: "random_seed", group: "swarm", label: "Random seed", kind: "number", min: 0, max: 4294967295, value: 12345, dice: true,
    hint: "Same seed and settings give the same run, every time, on every machine." },
  { key: "robot_speeds", group: "movement", label: "Speed", kind: "range", min: 0.1, max: 10, step: 0.1, value: 1,
    hint: "Distance a robot covers per unit of simulated time." },
  { key: "rigid_movement", group: "movement", label: "Rigid movement", kind: "switch", value: true,
    hint: "Rigid: a robot always reaches its target. (In the original, non-rigid robots also reach it.)" },
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
  { key: "width_bound", group: "world", label: "Width", kind: "range", min: 50, max: 1000, step: 10, value: 600,
    hint: "World size; robots start uniformly at random inside it." },
  { key: "height_bound", group: "world", label: "Height", kind: "range", min: 50, max: 1000, step: 10, value: 600 },
  { key: "lambda_rate", group: "timing", label: "Activation rate λ", kind: "range", min: 0.1, max: 20, step: 0.1, value: 5,
    hint: "Robots wake up after exponential delays with this rate (mean 1/λ)." },
  { key: "sampling_rate", group: "timing", label: "Sampling interval", kind: "range", min: 0.05, max: 1, step: 0.05, value: 0.1,
    hint: "Period of the scheduler's Visualize tick (kept for parity with the original)." },
  { key: "threshold_precision", group: "timing", label: "Precision (10⁻ᵖ)", kind: "range", min: 2, max: 10, step: 1, value: 5,
    hint: "ε = 10^-p: moves shorter than ε count as arrived; robots within ε count as gathered." },
];

const byKey = Object.fromEntries(SETTINGS.map((s) => [s.key, s]));
const LOG_STEPS = 1000;
const toLog = (n, s) => Math.round((LOG_STEPS * Math.log(n / s.min)) / Math.log(s.max / s.min));
const fromLog = (v, s) => Math.round(s.min * Math.pow(s.max / s.min, v / LOG_STEPS));
const decimals = (s) => (String(s.step ?? 1).split(".")[1] ?? "").length;

/** Builds the settings panel into `root`; `onChange(key, values)` fires on every edit. */
export function buildSettings(root, onChange) {
  const values = Object.fromEntries(SETTINGS.map((s) => [s.key, s.value]));
  const fields = {};

  for (const g of GROUPS) {
    const details = document.createElement("details");
    details.className = "group";
    details.open = g.open;
    details.innerHTML = `<summary><span class="chev" data-icon="chevron" data-size="14"></span>${g.label}</summary><div class="group-body"></div>`;
    const body = details.querySelector(".group-body");
    for (const s of SETTINGS.filter((x) => x.group === g.id)) {
      const field = document.createElement("div");
      field.className = "field";
      const id = `set-${s.key}`;
      const hint = s.hint ? `<span class="hint" title="${s.hint}">?</span>` : "";
      if (s.kind === "switch") {
        field.innerHTML = `<div class="field-top"><label for="${id}">${s.label}</label>${hint}<span class="switch"><input id="${id}" type="checkbox"><span></span></span></div>`;
      } else if (s.kind === "select") {
        field.innerHTML = `<div class="field-top"><label for="${id}">${s.label}</label>${hint}</div><select id="${id}">${s.options.map(([v, t]) => `<option value="${v}">${t}</option>`).join("")}</select>`;
      } else if (s.kind === "number") {
        field.innerHTML = `<div class="field-top"><label for="${id}">${s.label}</label>${hint}<input id="${id}" type="number" min="${s.min}" max="${s.max}" step="1">${s.dice ? `<button class="btn small outline" type="button" title="Random seed" aria-label="Random seed" data-icon="dice" data-size="15"></button>` : ""}</div>`;
      } else {
        const range = s.kind === "log" ? `min="0" max="${LOG_STEPS}" step="1"` : `min="${s.min}" max="${s.max}" step="${s.step}"`;
        field.innerHTML = `<div class="field-top"><label for="${id}-n">${s.label}</label>${hint}<input id="${id}-n" type="number" min="${s.min}" max="${s.max}" step="${s.step ?? 1}"></div><input id="${id}" type="range" ${range} aria-label="${s.label}">`;
      }
      body.append(field);
      fields[s.key] = field;
    }
    root.append(details);
  }

  function clamp(s, v) {
    if (s.kind === "switch") return Boolean(v);
    if (s.kind === "select") return s.options.some(([o]) => o === v) ? v : s.value;
    let n = Number(v);
    if (!Number.isFinite(n)) n = s.value;
    n = Math.min(s.max, Math.max(s.min, n));
    return s.kind === "log" || s.kind === "number" ? Math.round(n) : Number(n.toFixed(decimals(s)));
  }

  function show(key) {
    const s = byKey[key], f = fields[key], v = values[key], id = `set-${key}`;
    if (s.kind === "switch") f.querySelector("input").checked = v;
    else if (s.kind === "select" || s.kind === "number") f.querySelector(`#${id}`).value = v;
    else {
      f.querySelector(`#${id}`).value = s.kind === "log" ? toLog(v, s) : v;
      f.querySelector(`#${id}-n`).value = v;
    }
  }

  function refreshEnabled() {
    for (const s of SETTINGS) {
      if (!s.enabledBy) continue;
      const on = s.enabledBy(values);
      fields[s.key].classList.toggle("disabled", !on);
      for (const el of fields[s.key].querySelectorAll("input, select")) el.disabled = !on;
    }
    fields.num_of_faults.querySelector('input[type="range"]').max = Math.min(1000, values.num_of_robots);
  }

  function set(key, v, notify = true) {
    const s = byKey[key];
    values[key] = clamp(s, v);
    if (key === "num_of_robots" && values.num_of_faults > values.num_of_robots) {
      values.num_of_faults = values.num_of_robots;
      show("num_of_faults");
    }
    show(key);
    refreshEnabled();
    if (notify) onChange(key, values);
  }

  for (const s of SETTINGS) {
    const f = fields[s.key], id = `set-${s.key}`;
    if (s.kind === "switch") f.querySelector("input").addEventListener("change", (e) => set(s.key, e.target.checked));
    else if (s.kind === "select") f.querySelector("select").addEventListener("change", (e) => set(s.key, e.target.value));
    else if (s.kind === "number") {
      f.querySelector(`#${id}`).addEventListener("change", (e) => set(s.key, e.target.value));
      f.querySelector("button")?.addEventListener("click", () => set(s.key, Math.floor(Math.random() * 1e9)));
    } else {
      f.querySelector(`#${id}`).addEventListener("input", (e) => set(s.key, s.kind === "log" ? fromLog(+e.target.value, s) : e.target.value));
      f.querySelector(`#${id}-n`).addEventListener("change", (e) => set(s.key, e.target.value));
    }
    show(s.key);
  }
  refreshEnabled();

  return {
    values,
    set,
    reset() { for (const s of SETTINGS) set(s.key, s.value, false); onChange(null, values); },
  };
}

/** SimConfig for the core from the panel's values plus the chosen algorithm. */
export function readConfig(values, algorithm) {
  const config = { algorithm };
  for (const s of SETTINGS) if (!s.local) config[s.key] = values[s.key];
  config.visibility_radius = values.unlimited_visibility ? null : values.visibility_radius;
  return config;
}

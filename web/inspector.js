/* Robot inspector: a tooltip for the robot under the pointer, and a card
 * with everything the core knows about the selected robot, refreshed while
 * the simulation runs. */
import { COLORS, STATE_NAMES, colorIndex } from "./renderer.js";

const fmt = (v, d = 3) => (v == null ? "—" : Number(v).toLocaleString(undefined, { maximumFractionDigits: d, minimumFractionDigits: d }));
const point = (p) => (p ? `${fmt(p[0], 2)}, ${fmt(p[1], 2)}` : "—");
const FAULTS = { None: "none", Crash: "crash", Byzantine: "Byzantine", Omission: "omission", Delay: "delay" };

export function stateLabel(frame, i) {
  const c = COLORS[colorIndex(frame.flags[i], frame.fault[i], frame.task[i])];
  return { label: c.label, fill: c.fill, state: STATE_NAMES[frame.flags[i] & 3] };
}

export class Inspector {
  constructor({ card, tip, renderer, client, onCenter }) {
    Object.assign(this, { card, tip, renderer, client, onCenter });
    this.lastAsk = 0;
    this.info = null;
    card.querySelector("[data-action=close]").addEventListener("click", () => this.select(-1));
    card.querySelector("[data-action=center]").addEventListener("click", () => {
      if (this.info) this.onCenter(this.info.x, this.info.y);
    });
    client.on("robot", ({ id, info }) => {
      if (id === this.renderer.selected) { this.info = info; this.render(); }
    });
  }

  /** Pointer moved over the canvas (CSS px relative to the canvas). */
  hover(sx, sy, clientX, clientY) {
    const i = this.renderer.pick(sx, sy);
    if (i !== this.renderer.hovered) { this.renderer.hovered = i; this.renderer.requestDraw(); }
    const f = this.renderer.frame;
    if (i < 0 || !f) { this.tip.hidden = true; this.renderer.canvas.style.cursor = ""; return; }
    const s = stateLabel(f, i);
    this.tip.innerHTML = `<span class="chip"><span class="dot" style="background:${s.fill}"></span>Robot ${i} · ${s.label}</span>`;
    this.tip.style.left = `${clientX + 14}px`;
    this.tip.style.top = `${clientY + 14}px`;
    this.tip.hidden = false;
    this.renderer.canvas.style.cursor = "pointer";
  }

  leave() {
    this.tip.hidden = true;
    if (this.renderer.hovered !== -1) { this.renderer.hovered = -1; this.renderer.requestDraw(); }
  }

  select(i) {
    this.renderer.selected = i;
    this.renderer.requestDraw();
    this.info = null;
    this.card.hidden = i < 0;
    if (i >= 0) { this.render(); this.client.inspect(i); }
  }

  /** Called on every new frame: keep the card current, a few times a second. */
  refresh(force = false) {
    const i = this.renderer.selected;
    if (i < 0) return;
    const now = performance.now();
    if (force || now - this.lastAsk > 200) { this.lastAsk = now; this.client.inspect(i); }
  }

  render() {
    const i = this.renderer.selected, f = this.renderer.frame, info = this.info;
    const s = f && i < f.flags.length ? stateLabel(f, i) : null;
    this.card.querySelector("[data-field=title]").textContent = `Robot ${i}`;
    this.card.querySelector("[data-field=status]").innerHTML = s ? `<span class="chip"><span class="dot" style="background:${s.fill}"></span>${s.label}</span>` : "";
    const rows = info ? [
      ["Algorithm", info.algorithm],
      ["Phase", info.state + (info.frozen ? " · frozen" : "") + (info.terminated ? " · terminated" : "")],
      ["Position", point([info.x, info.y])],
      ["Target", point(info.target)],
      ["To target", info.target ? fmt(Math.hypot(info.target[0] - info.x, info.target[1] - info.y), 3) : "—"],
      ["Speed", fmt(info.speed, 2)],
      ["Travelled", fmt(info.travelled, 2)],
      ["Fault", FAULTS[info.fault] ?? info.fault],
      ["Light", info.light ?? "—"],
      ...(info.task ? [["Task", info.task]] : []),
      ...(info.circle ? [["Circle", `(${fmt(info.circle[0], 1)}, ${fmt(info.circle[1], 1)}) r ${fmt(info.circle[2], 1)}`]] : []),
    ] : [["", "Loading…"]];
    this.card.querySelector("dl").innerHTML = rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join("");
  }
}

/* Play the adversary. Under the sequential scheduler you choose which robot takes the next turn
 * and, with non-rigid movement, where it is stopped; under SSYNC you choose the robots of each
 * round. The run waits (schedule_end "await" in the core) until you choose. Every choice is kept,
 * so the schedule can be copied into Settings → Scheduler and replayed exactly. */
import { formatSchedule } from "./settings.js";

const CRASH = 3, TERMINATED = 8;

export class Adversary {
  /** `canAct()`: the run is waiting for a choice; `onPlay()`: a turn or round was just sent. */
  constructor({ renderer, client, settings, panel, toast, canAct, onPlay }) {
    Object.assign(this, { renderer, client, settings, panel, toast, canAct, onPlay });
    this.on = false;
    this.mode = "sequential";
    this.chosen = [];      // the robot (sequential) or robots (SSYNC) picked for the next turn
    this.preview = null;   // { robot, target: [x, y] }: where the picked robot would go
    this.hover = null;     // [x, y]: the stopping point under the pointer, on the path
    this.history = [];     // what has been played, as Settings → Scheduler writes it
    client.on("preview", (d) => {
      if (!this.on || this.chosen[0] !== d.robot) return;
      this.preview = d.target ? { robot: d.robot, target: d.target } : null;
      this.render();
    });
    panel.addEventListener("click", (e) => {
      const action = e.target.closest("[data-adv]")?.dataset.adv;
      if (action) this.action(action);
    });
  }

  get supported() { const s = this.settings.values.scheduler; return s === "sequential" || s === "ssync"; }

  setOn(on) {
    this.on = on;
    this.mode = this.settings.values.scheduler;
    this.panel.hidden = !on;
    this.renderer.options.adversary = on ? this : null;
    this.reset();
  }

  /** A new run: nothing chosen or played yet. */
  reset() { this.chosen = []; this.preview = null; this.hover = null; this.history = []; this.render(); }

  live(i) {
    const f = this.renderer.frame;
    return f && i >= 0 && i < f.flags.length && (f.flags[i] & 3) !== CRASH && !(f.flags[i] & TERMINATED);
  }

  /** Movement settings that let the adversary stop a robot early, and δ. */
  get nonRigid() { const v = this.settings.values; return !v.rigid_movement && v.delta > 0 ? v.delta : 0; }

  /** Where on the previewed path the screen point (sx, sy) is, as a fraction of the way; null when it is not near the path. */
  pathFraction(sx, sy) {
    const p = this.preview, f = this.renderer.frame;
    if (!p || !f) return null;
    const [ax, ay] = this.renderer.toScreen(f.x(p.robot), f.y(p.robot));
    const [bx, by] = this.renderer.toScreen(p.target[0], p.target[1]);
    const dx = bx - ax, dy = by - ay, len2 = dx * dx + dy * dy;
    if (len2 < 1) return null;
    const t = Math.max(0, Math.min(1, ((sx - ax) * dx + (sy - ay) * dy) / len2));
    return Math.hypot(ax + t * dx - sx, ay + t * dy - sy) <= 10 ? t : null;
  }

  /** The fraction the adversary may stop at: never before δ. */
  allowedStop(t) {
    const p = this.preview, f = this.renderer.frame, delta = this.nonRigid;
    const len = Math.hypot(p.target[0] - f.x(p.robot), p.target[1] - f.y(p.robot));
    if (!delta || len <= delta) return null;
    return Math.max(t, delta / len);
  }

  /** Pointer move over the canvas: show where a click would stop the robot. */
  move(sx, sy) {
    if (!this.on || this.mode !== "sequential") return;
    const t = this.pathFraction(sx, sy);
    const stop = t === null || !this.nonRigid ? null : this.allowedStop(t);
    const f = this.renderer.frame, p = this.preview;
    const next = stop === null || stop >= 1 ? null
      : [f.x(p.robot) + stop * (p.target[0] - f.x(p.robot)), f.y(p.robot) + stop * (p.target[1] - f.y(p.robot))];
    if (String(next) !== String(this.hover)) { this.hover = next; this.renderer.requestDraw(); }
  }

  /** A click on the canvas in adversary mode. Always handled (returns true). */
  click(sx, sy) {
    if (!this.canAct()) { this.toast("Wait: the last choice is still being played"); return true; }
    const i = this.renderer.pick(sx, sy, 12);
    if (this.mode === "ssync") {
      if (i < 0) return true;
      if (!this.live(i)) { this.toast(`Robot ${i} has stopped for good and takes no more rounds`); return true; }
      const k = this.chosen.indexOf(i);
      if (k >= 0) this.chosen.splice(k, 1); else this.chosen.push(i);
      this.render();
      return true;
    }
    if (this.chosen.length && this.preview) {
      if (i === this.chosen[0]) { this.play(null); return true; }
      const t = this.pathFraction(sx, sy);
      if (t !== null && i < 0) {
        if (!this.nonRigid) { this.toast("With rigid movement a robot always arrives. Turn rigid movement off and set δ to stop robots on the way."); return true; }
        const stop = this.allowedStop(t);
        this.play(stop === null || stop >= 1 ? null : stop);
        return true;
      }
    }
    if (i >= 0 && !this.live(i)) { this.toast(`Robot ${i} has stopped for good and takes no more turns`); return true; }
    this.chosen = i >= 0 ? [i] : [];
    this.preview = null;
    this.hover = null;
    if (i >= 0) this.client.preview(i);
    this.render();
    return true;
  }

  play(stop) {
    const i = this.chosen[0];
    this.client.turn(i, stop);
    this.history.push(stop === null ? i : { robot: i, stop });
    this.chosen = []; this.preview = null; this.hover = null;
    this.onPlay();
    this.render();
  }

  playRound() {
    if (!this.chosen.length) { this.toast("Click at least one robot for the round"); return; }
    const round = [...this.chosen].sort((a, b) => a - b);
    this.client.round(round);
    this.history.push(round);
    this.chosen = [];
    this.onPlay();
    this.render();
  }

  get scheduleText() { return formatSchedule(this.history, this.mode); }

  action(name) {
    if (name === "play") {
      if (!this.canAct()) return;
      if (this.mode === "ssync") this.playRound();
      else if (this.chosen.length) this.play(null);
      else this.toast("Click the robot that takes the next turn");
    } else if (name === "all") {
      const f = this.renderer.frame;
      this.chosen = f ? [...Array(f.flags.length).keys()].filter((i) => this.live(i)) : [];
      this.render();
    } else if (name === "clear") {
      this.chosen = []; this.preview = null; this.hover = null; this.render();
    } else if (name === "copy") {
      const text = this.scheduleText;
      navigator.clipboard?.writeText(text).then(() => this.toast("Schedule copied"), () => this.toast(text));
    } else if (name === "keep") {
      // Settings → Scheduler → Turns/Rounds: after leaving this mode, Reset replays the same choices.
      this.settings.set("schedule_text", this.scheduleText, false);
      this.toast("Saved as the run's schedule (Settings → Scheduler → More options)");
    } else if (name === "leave") this.onLeave?.();
  }

  render() {
    if (!this.on) { this.renderer.requestDraw(); return; }
    const delta = this.nonRigid, n = this.chosen.length;
    let what;
    if (this.mode === "ssync") {
      what = `Click robots to put them in the next round (<strong>${n}</strong> chosen). They all look at the same instant, then move together. <kbd>Enter</kbd> plays the round.`;
    } else if (!n) {
      what = "Click the robot that takes the next turn. Every other robot stays still while it looks, computes and moves.";
    } else if (!this.preview) {
      what = `Robot ${this.chosen[0]}: working out where it would go…`;
    } else {
      what = `Robot ${this.chosen[0]} would go to the ✕. Click it again, or press <kbd>Enter</kbd>, to let it arrive`
        + (delta ? `, or click on its path to stop it there (it covers at least δ = ${delta}).` : ". (Turn rigid movement off and set δ to stop robots on the way.)");
    }
    const played = this.history.length;
    this.panel.innerHTML = `<div class="cap-head">You are the adversary · ${this.mode === "ssync" ? "SSYNC rounds" : "sequential turns"}</div>
      <div class="cap-body">${what}</div>
      <div class="cap-body adv-history">${played ? `${played} ${this.mode === "ssync" ? "round" : "turn"}${played === 1 ? "" : "s"} played: <code>${this.scheduleText}</code>` : "Nothing played yet."}</div>
      <div class="adv-actions">
        <button class="btn small primary" data-adv="play">${this.mode === "ssync" ? "Play round" : "Move"}</button>
        ${this.mode === "ssync" ? '<button class="btn small outline" data-adv="all">All robots</button>' : ""}
        <button class="btn small outline" data-adv="clear">Clear choice</button>
        <button class="btn small outline" data-adv="copy" ${played ? "" : "disabled"}>Copy schedule</button>
        <button class="btn small outline" data-adv="keep" ${played ? "" : "disabled"}>Keep as schedule</button>
        <button class="btn small outline" data-adv="leave">Leave</button>
      </div>`;
    this.renderer.requestDraw();
  }
}

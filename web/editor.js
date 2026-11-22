/* The scenario editor: build a start by hand, for any algorithm. It starts from the robots on
 * screen; click to add a robot, drag one to move it, Shift + click to stack another robot on it
 * (a multiplicity), Alt + click or right-click to remove it, Ctrl + Z to undo. Every edit is
 * written to Settings → Start → "My own positions" and the run is rebuilt from it, so the
 * scenario is an ordinary setting: it can be saved, shared as a link and replayed. */

const MAX = 2000;
const fmt = (v) => String(+v.toFixed(3));

export class ScenarioEditor {
  /** `rebuild()`: build a new run from the settings, keeping the view. */
  constructor({ renderer, settings, panel, toast, rebuild }) {
    Object.assign(this, { renderer, settings, panel, toast, rebuild });
    this.on = false;
    this.points = [];
    this.undoStack = [];
    this.drag = null;     // { i, to: [x, y] | null }
    this.hovered = -1;
    panel.addEventListener("click", (e) => {
      const action = e.target.closest("[data-edit]")?.dataset.edit;
      if (action === "undo") this.undo();
      else if (action === "clear") this.edit(() => { this.points = []; });
      else if (action) this.onAction?.(action);
    });
  }

  /** Turns the editor on, starting from the robots of `frame`. Returns false if it cannot. */
  setOn(on, frame = this.renderer.frame) {
    if (on && frame && frame.flags.length > MAX) {
      this.toast(`The editor works with up to ${MAX.toLocaleString()} robots; this start has ${frame.flags.length.toLocaleString()}`, true);
      return false;
    }
    this.on = on;
    this.panel.hidden = !on;
    this.renderer.options.editor = on ? this : null;
    this.drag = null;
    if (on) {
      this.points = frame ? Array.from({ length: frame.flags.length }, (_, i) => [frame.x(i), frame.y(i)]) : [];
      this.undoStack = [];
      this.write();
    }
    this.render();
    return true;
  }

  /** Applies `change` to the points as one undoable edit. */
  edit(change) {
    this.undoStack.push(this.points.map((p) => [...p]));
    if (this.undoStack.length > 200) this.undoStack.shift();
    change();
    this.write();
  }

  undo() {
    const last = this.undoStack.pop();
    if (!last) { this.toast("Nothing to undo"); return; }
    this.points = last;
    this.write();
  }

  write() {
    if (!this.points.length) {  // the core needs at least one robot: show an empty world until one is added
      this.renderer.setFrame(null);
      this.render();
      return;
    }
    const text = `# Made in the scenario editor: one robot per line\n${this.points.map(([x, y]) => `${fmt(x)}, ${fmt(y)}`).join("\n")}`;
    this.settings.apply({ pattern: "custom", custom: text, multiplicities: 0 });
    this.rebuild();
    this.render();
  }

  /** Pointer down: a plain press on a robot starts dragging it. Returns true if handled. */
  down(sx, sy, e) {
    if (!this.on || e.button !== 0 || e.shiftKey || e.altKey) return false;
    const i = this.renderer.pick(sx, sy, 10);
    if (i < 0) return false;
    this.drag = { i, to: null, x: sx, y: sy };
    return true;
  }

  move(sx, sy) {
    if (!this.on) return false;
    if (this.drag) {
      if (this.drag.to || Math.hypot(sx - this.drag.x, sy - this.drag.y) > 3) {
        this.drag.to = this.renderer.toWorld(sx, sy);
        this.renderer.requestDraw();
      }
      return true;
    }
    const i = this.renderer.pick(sx, sy, 10);
    if (i !== this.hovered) { this.hovered = i; this.renderer.requestDraw(); }
    return false;
  }

  /** Pointer up after `down` returned true. */
  up() {
    const d = this.drag;
    this.drag = null;
    if (d?.to) this.edit(() => { this.points[d.i] = d.to; });
    this.renderer.requestDraw();
  }

  /** A click that did not drag. Returns true if handled. */
  click(sx, sy, e) {
    if (!this.on) return false;
    const i = this.renderer.pick(sx, sy, 10);
    if (i >= 0 && e.altKey) this.remove(i);
    else if (i >= 0 && e.shiftKey) this.edit(() => { this.points.push([...this.points[i]]); });
    else if (i < 0 && this.points.length >= MAX) this.toast(`At most ${MAX.toLocaleString()} robots in the editor`, true);
    else if (i < 0) { const p = this.renderer.toWorld(sx, sy); this.edit(() => { this.points.push(p); }); }
    return true;
  }

  remove(i) {
    if (i < 0 || i >= this.points.length) return;
    if (this.points.length === 1) { this.toast("A run needs at least one robot"); return; }
    this.edit(() => { this.points.splice(i, 1); });
    this.hovered = -1;
  }

  render() {
    if (!this.on) return;
    const n = this.points.length;
    this.panel.innerHTML = `<div class="cap-head">Scenario editor · ${n.toLocaleString()} robot${n === 1 ? "" : "s"}</div>
      <div class="cap-body">Click: add a robot · Drag: move it · <kbd>Shift</kbd> + click: stack another robot on it · <kbd>Alt</kbd> + click, right-click or <kbd>Delete</kbd>: remove · <kbd>Ctrl</kbd> + <kbd>Z</kbd>: undo.
        The start is kept in Settings → Start → My own positions; robot numbers follow the order you placed them.</div>
      <div class="adv-actions">
        <button class="btn small outline" data-edit="undo" ${this.undoStack.length ? "" : "disabled"}>Undo</button>
        <button class="btn small outline" data-edit="clear">Clear all</button>
        <button class="btn small outline" data-edit="save">Save scenario…</button>
        <button class="btn small outline" data-edit="link">Copy link</button>
        <button class="btn small primary" data-edit="done">Done</button>
      </div>`;
  }
}

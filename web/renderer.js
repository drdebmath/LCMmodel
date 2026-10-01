/* CPU Canvas2D renderer for frames from worker.js (docs/core-schema.md §8).
 *
 * Built for 10,000 robots without a GPU. Measured in headless Chrome with the
 * GPU off (10k robots): one path of rectangles 14.9 ms/frame, fillRect per
 * robot 5.9 ms; 2,000 outlined circles at 2× pixel density: one path of arcs
 * 48 ms, stamping a pre-drawn sprite 14.7 ms. So tiny robots are fillRects,
 * larger ones and light rings are sprites (cached per colour, size and pixel
 * density) stamped at whole-pixel positions. Detail that stops being
 * readable at large counts (outlines, target lines, visibility and SEC
 * circles) is dropped automatically; see LOD. The selected robot is always
 * drawn in full.
 *
 * Drawing is coalesced: `requestDraw()` schedules at most one draw per
 * animation frame however many frames, zooms or hovers arrive.
 */

// Same colours and precedence as the Pyodide page (main.js colorForRobot).
export const COLORS = [
  { key: "active", label: "Active", fill: "#007aff" },
  { key: "move", label: "Moving", fill: "#34c759" },
  { key: "frozen", label: "Frozen", fill: "#ff9500" },
  { key: "terminated", label: "Terminated", fill: "#a0a0a0" },
  { key: "crash", label: "Crashed", fill: "#555555" },
  { key: "byzantine", label: "Byzantine", fill: "#e0115f" },
  { key: "omission", label: "Omission fault", fill: "#00bcd4" },
  { key: "delay", label: "Delay fault", fill: "#9c27b0" },
  { key: "task-red", label: "SEC task", fill: "#dc2626" },
  { key: "task-blue", label: "Gathering task", fill: "#2563eb" },
];
const C = Object.fromEntries(COLORS.map((c, i) => [c.key, i]));
export const LIGHTS = [null, "#3b82f6", "#ef4444", "#22c55e"];

// Level of detail: draw these only up to this many robots.
export const LOD = { outlines: 2000, targets: 2000, visibility: 1000, circles: 1000 };

export const STATE_NAMES = ["Wait", "Look", "Move", "Crash"];
const STATE_MOVE = 2, STATE_CRASH = 3, FROZEN = 4, TERMINATED = 8;

export function colorIndex(flags, fault, task) {
  if ((flags & 3) === STATE_CRASH) return C.crash;
  if (fault === 2) return C.byzantine;
  if (fault === 3) return C.omission;
  if (fault === 4) return C.delay;
  if (task === 1) return C["task-red"];
  if (task === 2) return C["task-blue"];
  if (flags & TERMINATED) return C.terminated;
  if (flags & FROZEN) return C.frozen;
  if ((flags & 3) === STATE_MOVE) return C.move;
  return C.active;
}

export class Renderer {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d", { alpha: false });
    // The region "Fit" frames (world units), and the start-area outline to draw.
    this.world = { w: 600, h: 600, cx: 0, cy: 0 };
    this.outline = null;
    this.zoom = 1;
    this.pan = { x: 0, y: 0 };
    // CSS px covered by floating panels; the world is fitted into what's left.
    this.insets = { left: 0, right: 0, top: 0, bottom: 0 };
    this.options = { showLights: false, visibility: null };
    this.frame = null;
    this.selected = -1;
    this.hovered = -1;
    this.colorOf = new Uint8Array(0);
    this.counts = new Uint32Array(COLORS.length);
    this.lastDrawMs = 0;
    this.onDraw = null;
    this.pending = false;
    this.sprites = new Map();
    this.readTheme();
    this.resize();
  }

  readTheme() {
    const css = getComputedStyle(document.documentElement);
    this.theme = {
      bg: css.getPropertyValue("--canvas").trim() || "#f5f6f8",
      bounds: css.getPropertyValue("--bounds").trim() || "rgba(0,0,0,.15)",
      accent: css.getPropertyValue("--accent").trim() || "#2f5bea",
      text: css.getPropertyValue("--text").trim() || "#111",
      outline: document.documentElement.dataset.resolvedTheme === "dark" ? "rgba(0,0,0,.55)" : "rgba(0,0,0,.7)",
    };
    this.sprites.clear();
    this.requestDraw();
  }

  resize() {
    const rect = this.canvas.getBoundingClientRect();
    this.dpr = window.devicePixelRatio || 1;
    this.width = Math.max(1, rect.width);
    this.height = Math.max(1, rect.height);
    this.canvas.width = Math.floor(this.width * this.dpr);
    this.canvas.height = Math.floor(this.height * this.dpr);
    this.requestDraw();
  }

  requestDraw() {
    if (this.pending) return;
    this.pending = true;
    requestAnimationFrame(() => { this.pending = false; this.draw(); });
  }

  setFrame(frame) { this.frame = frame; this.requestDraw(); }
  /** The region "Fit" frames: w × h world units centred on (cx, cy). */
  setView(w, h, cx = 0, cy = 0) {
    const fitted = this.isFitted();
    this.world = { w: Math.max(1e-6, w), h: Math.max(1e-6, h), cx, cy };
    if (fitted) this.resetView(); else this.requestDraw();
  }
  /** The start-area outline (see startOutline in start.js), or null for none. */
  setOutline(outline) { this.outline = outline; this.requestDraw(); }

  fittedPan() {
    const { left, right, top, bottom } = this.insets;
    const zoom = this.zoom;
    this.zoom = 1;
    const s = this.scale;
    this.zoom = zoom;
    return { x: (left - right) / 2 - this.world.cx * s, y: (top - bottom) / 2 + this.world.cy * s };
  }
  isFitted() {
    const p = this.fittedPan();
    return this.zoom === 1 && Math.abs(this.pan.x - p.x) < 0.5 && Math.abs(this.pan.y - p.y) < 0.5;
  }
  /** Fit the view region into the area the panels leave free. */
  resetView() {
    this.zoom = 1;
    this.pan = this.fittedPan();
    this.requestDraw();
  }

  setInsets(insets) {
    const fitted = this.isFitted();
    this.insets = insets;
    if (fitted) this.resetView(); // keep a fitted view fitted; leave a zoomed one alone
  }

  get scale() {
    const { left, right, top, bottom } = this.insets;
    const w = Math.max(100, this.width - left - right), h = Math.max(100, this.height - top - bottom);
    return Math.min(w / this.world.w, h / this.world.h) * 0.92 * this.zoom;
  }

  toScreen(x, y) {
    const s = this.scale;
    return [this.width / 2 + this.pan.x + x * s, this.height / 2 + this.pan.y - y * s];
  }

  toWorld(sx, sy) {
    const s = this.scale;
    return [(sx - this.width / 2 - this.pan.x) / s, -(sy - this.height / 2 - this.pan.y) / s];
  }

  /** Zoom by `factor`, keeping the world point under (mx, my) (CSS px) still. */
  zoomAt(factor, mx = this.width / 2, my = this.height / 2) {
    const [wx, wy] = this.toWorld(mx, my);
    this.zoom = Math.min(400, Math.max(0.05, this.zoom * factor));
    const s = this.scale;
    this.pan.x = mx - this.width / 2 - wx * s;
    this.pan.y = my - this.height / 2 + wy * s;
    this.requestDraw();
  }

  panBy(dx, dy) { this.pan.x += dx; this.pan.y += dy; this.requestDraw(); }

  centerOn(x, y) {
    const s = this.scale, { left, right, top, bottom } = this.insets;
    this.pan.x = (left - right) / 2 - x * s;
    this.pan.y = (top - bottom) / 2 + y * s;
    this.requestDraw();
  }

  /** Robot radius in CSS px: smaller for big swarms so they stay distinguishable. */
  robotRadius(n) {
    const base = n <= 200 ? 5 : n <= 2000 ? 3 : 1.6;
    return Math.min(9, base * Math.sqrt(this.zoom));
  }

  /** A pre-drawn disc (or ring) of radius r CSS px at pixel density dpr. */
  sprite(fill, r, dpr, { outline = null, ring = 0 } = {}) {
    const key = `${fill}|${r.toFixed(2)}|${dpr}|${outline}|${ring}`;
    let sp = this.sprites.get(key);
    if (sp) return sp;
    const extent = r + (ring ? ring / 2 : 1) + 1;
    const size = Math.ceil(2 * extent * dpr);
    const canvas = document.createElement("canvas");
    canvas.width = canvas.height = size;
    const g = canvas.getContext("2d");
    g.scale(dpr, dpr);
    g.beginPath();
    g.arc(size / dpr / 2, size / dpr / 2, r, 0, 2 * Math.PI);
    if (ring) { g.lineWidth = ring; g.strokeStyle = fill; g.stroke(); }
    else {
      g.fillStyle = fill; g.fill();
      if (outline) { g.lineWidth = 1; g.strokeStyle = outline; g.stroke(); }
    }
    sp = { canvas, half: size / dpr / 2, size: size / dpr };
    if (this.sprites.size > 256) this.sprites.clear();
    this.sprites.set(key, sp);
    return sp;
  }

  /** Index of the robot nearest to screen point (sx, sy) within reach, or -1. */
  pick(sx, sy, reach = 8) {
    const f = this.frame;
    if (!f) return -1;
    const s = this.scale, ox = this.width / 2 + this.pan.x, oy = this.height / 2 + this.pan.y;
    const r = Math.max(reach, this.robotRadius(f.flags.length) + 3);
    let best = -1, bestD = r * r;
    const xy = f.xy;
    for (let i = 0, n = f.flags.length; i < n; i++) {
      const dx = ox + xy[2 * i] * s - sx, dy = oy - xy[2 * i + 1] * s - sy;
      const d = dx * dx + dy * dy;
      if (d < bestD) { bestD = d; best = i; }
    }
    return best;
  }

  draw(ctx = this.ctx, dpr = this.dpr) {
    const t0 = performance.now();
    const frame = this.frame;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = this.theme.bg;
    ctx.fillRect(0, 0, this.width, this.height);

    const s = this.scale;
    const ox = this.width / 2 + this.pan.x;
    const oy = this.height / 2 + this.pan.y;
    const X = (x) => ox + x * s;
    const Y = (y) => oy - y * s;

    // Start area (a drawing guide, not a wall: robots can move past it).
    const o = this.outline;
    if (o) {
      ctx.strokeStyle = this.theme.bounds;
      ctx.lineWidth = 1;
      ctx.setLineDash([5, 5]);
      ctx.beginPath();
      const rot = ((o.rotation ?? 0) * Math.PI) / 180;
      const P = (x, y) => [X(x * Math.cos(rot) - y * Math.sin(rot)), Y(x * Math.sin(rot) + y * Math.cos(rot))];
      if (o.kind === "rect") {
        const corners = [[-o.w / 2, -o.h / 2], [o.w / 2, -o.h / 2], [o.w / 2, o.h / 2], [-o.w / 2, o.h / 2]].map(([x, y]) => P(x, y));
        corners.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
        ctx.closePath();
      } else if (o.kind === "circle" || o.kind === "ring") {
        ctx.moveTo(X(o.r), Y(0)); ctx.arc(X(0), Y(0), o.r * s, 0, 2 * Math.PI);
        if (o.kind === "ring" && o.inner > 0) { ctx.moveTo(X(o.inner), Y(0)); ctx.arc(X(0), Y(0), o.inner * s, 0, 2 * Math.PI); }
      } else if (o.kind === "line") {
        ctx.moveTo(...P(-o.length / 2, 0)); ctx.lineTo(...P(o.length / 2, 0));
      } else if (o.kind === "polygon") {
        for (let i = 0; i <= o.sides; i++) {
          const a = Math.PI / 2 + (2 * Math.PI * i) / o.sides;
          const [x, y] = P(o.r * Math.cos(a), o.r * Math.sin(a));
          if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y);
        }
      }
      ctx.stroke();
      ctx.setLineDash([]);
    }
    if (!frame) { this.lastDrawMs = performance.now() - t0; return; }

    const { xy, flags, fault, task, light, target, circle } = frame;
    const n = flags.length;
    if (this.colorOf.length !== n) this.colorOf = new Uint8Array(n);
    const colorOf = this.colorOf, counts = this.counts;
    counts.fill(0);
    for (let i = 0; i < n; i++) { const c = colorIndex(flags[i], fault[i], task[i]); colorOf[i] = c; counts[c]++; }
    const r = this.robotRadius(n);
    const V = this.options.visibility;

    // Visibility circles (behind everything).
    if (V && n <= LOD.visibility) {
      ctx.strokeStyle = "rgba(100,110,200,.22)";
      ctx.beginPath();
      for (let i = 0; i < n; i++) {
        if ((flags[i] & 3) === STATE_CRASH || flags[i] & TERMINATED) continue;
        const x = X(xy[2 * i]), y = Y(xy[2 * i + 1]);
        ctx.moveTo(x + V * s, y);
        ctx.arc(x, y, V * s, 0, 2 * Math.PI);
      }
      ctx.stroke();
    }

    // Circles the robots computed (SEC and friends).
    if (n <= LOD.circles) {
      ctx.strokeStyle = "rgba(255,150,0,0.55)";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      for (let i = 0; i < n; i++) {
        const cr = circle[3 * i + 2];
        if (!(cr >= 0)) continue;
        const cx = X(circle[3 * i]), cy = Y(circle[3 * i + 1]);
        ctx.moveTo(cx + cr * s, cy);
        ctx.arc(cx, cy, cr * s, 0, 2 * Math.PI);
      }
      ctx.stroke();
      ctx.lineWidth = 1;
    }

    // Dashed line from each moving robot to its target, in the robot's colour.
    if (n <= LOD.targets) {
      ctx.setLineDash([2, 3]);
      for (let c = 0; c < COLORS.length; c++) {
        if (!counts[c]) continue;
        ctx.beginPath();
        let any = false;
        for (let i = 0; i < n; i++) {
          if (colorOf[i] !== c || (flags[i] & 3) !== STATE_MOVE || !(target[2 * i] === target[2 * i])) continue;
          ctx.moveTo(X(xy[2 * i]), Y(xy[2 * i + 1]));
          ctx.lineTo(X(target[2 * i]), Y(target[2 * i + 1]));
          any = true;
        }
        if (any) { ctx.strokeStyle = COLORS[c].fill; ctx.globalAlpha = 0.7; ctx.stroke(); ctx.globalAlpha = 1; }
      }
      ctx.setLineDash([]);
    }

    // Robot bodies, colour by colour: fillRect dots, or stamped sprites.
    const snap = (v) => Math.round(v * dpr) / dpr;
    const outline = n <= LOD.outlines ? this.theme.outline : null;
    for (let c = 0; c < COLORS.length; c++) {
      if (!counts[c]) continue;
      if (r < 2) {
        ctx.fillStyle = COLORS[c].fill;
        const d = 2 * r;
        for (let i = 0; i < n; i++) {
          if (colorOf[i] === c) ctx.fillRect(snap(X(xy[2 * i]) - r), snap(Y(xy[2 * i + 1]) - r), d, d);
        }
      } else {
        const sp = this.sprite(COLORS[c].fill, r, dpr, { outline });
        for (let i = 0; i < n; i++) {
          if (colorOf[i] === c) ctx.drawImage(sp.canvas, snap(X(xy[2 * i]) - sp.half), snap(Y(xy[2 * i + 1]) - sp.half), sp.size, sp.size);
        }
      }
    }

    // Luminous-robot lights: a ring in the light's colour.
    if (this.options.showLights) {
      for (let l = 1; l < LIGHTS.length; l++) {
        const sp = this.sprite(LIGHTS[l], r + 2.5, dpr, { ring: r >= 2 ? 2 : 1 });
        for (let i = 0; i < n; i++) {
          if (light[i] === l) ctx.drawImage(sp.canvas, snap(X(xy[2 * i]) - sp.half), snap(Y(xy[2 * i + 1]) - sp.half), sp.size, sp.size);
        }
      }
    }

    // Hovered and selected robots, drawn in full at any swarm size.
    const mark = (i, ring, strong) => {
      if (i < 0 || i >= n) return;
      const x = X(xy[2 * i]), y = Y(xy[2 * i + 1]);
      ctx.strokeStyle = this.theme.accent;
      if (strong) {
        if (V && !((flags[i] & 3) === STATE_CRASH)) {
          ctx.globalAlpha = 0.35;
          ctx.beginPath(); ctx.arc(x, y, V * s, 0, 2 * Math.PI); ctx.stroke();
          ctx.globalAlpha = 1;
        }
        const cr = circle[3 * i + 2];
        if (cr >= 0) {
          ctx.strokeStyle = "rgba(255,150,0,0.9)"; ctx.lineWidth = 1.5;
          ctx.beginPath(); ctx.arc(X(circle[3 * i]), Y(circle[3 * i + 1]), cr * s, 0, 2 * Math.PI); ctx.stroke();
          ctx.strokeStyle = this.theme.accent;
        }
        if (target[2 * i] === target[2 * i]) {
          const tx = X(target[2 * i]), ty = Y(target[2 * i + 1]);
          ctx.setLineDash([4, 4]); ctx.lineWidth = 1.5;
          ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(tx, ty); ctx.stroke();
          ctx.setLineDash([]);
          ctx.beginPath(); ctx.moveTo(tx - 5, ty - 5); ctx.lineTo(tx + 5, ty + 5); ctx.moveTo(tx + 5, ty - 5); ctx.lineTo(tx - 5, ty + 5); ctx.stroke();
        }
      }
      ctx.lineWidth = strong ? 2.5 : 1.5;
      ctx.beginPath(); ctx.arc(x, y, Math.max(r, 2) + ring, 0, 2 * Math.PI); ctx.stroke();
      ctx.lineWidth = 1;
    };
    if (this.hovered !== this.selected) mark(this.hovered, 4, false);
    mark(this.selected, 5, true);

    this.lastDrawMs = performance.now() - t0;
    this.onDraw?.(this);
  }

  /** PNG of the current view with a caption line, for export. */
  async toPngBlob(caption) {
    const scale = Math.max(2, this.dpr);
    const out = document.createElement("canvas");
    out.width = Math.floor(this.width * scale);
    out.height = Math.floor(this.height * scale);
    const ctx = out.getContext("2d", { alpha: false });
    this.draw(ctx, scale);
    ctx.setTransform(scale, 0, 0, scale, 0, 0);
    ctx.font = "12px system-ui, sans-serif";
    ctx.fillStyle = this.theme.text;
    ctx.globalAlpha = 0.7;
    ctx.fillText(caption, 14, this.height - 14);
    ctx.globalAlpha = 1;
    this.requestDraw(); // the export drew with a different transform
    return new Promise((resolve) => out.toBlob(resolve, "image/png"));
  }
}

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

const SUPERSCRIPT = { "-": "⁻", 0: "⁰", 1: "¹", 2: "²", 3: "³", 4: "⁴", 5: "⁵", 6: "⁶", 7: "⁷", 8: "⁸", 9: "⁹" };
/** The gap between neighbouring doubles near `m` (a size): what positions that big cannot be told apart below. */
export function ulpOf(m) {
  m = Math.abs(m);
  return m > 1e-300 ? 2 ** (Math.floor(Math.log2(m)) - 52) : 5e-324;
}

/** A distance in the simulation's units, to three significant digits: 37.5, 0.0179, 1.79×10⁻⁵. */
export function fmtDistance(v) {
  if (v === 0 || !isFinite(v)) return v === 0 ? "0" : "—";
  if (v >= 1e-3 && v < 1e7) return String(+v.toPrecision(3));
  const [m, e] = v.toExponential(2).split("e");
  return `${m}×10${[...String(Number(e))].map((c) => SUPERSCRIPT[c]).join("")}`;
}

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
export const LOD = { outlines: 2000, targets: 2000, visibility: 1000, circles: 1000, links: 400 };

// How far in the view may go. The real limit is the numbers, not this: positions are doubles, which cannot tell
// places closer than about 2e-16 times their size apart, so a swarm near (100, 0) can only be told apart down to
// about 1.4e-14, while one near (0, 0) can be told apart far smaller. Follow stops where the swarm is one such
// step across (see stepFollow); this only keeps the zoom number finite.
const MAX_ZOOM = 1e36;
// Circles bigger than this many CSS px cannot be seen (a swarm followed in close next to a visibility
// radius, say): skip them instead of asking the canvas for a huge arc.
const HUGE = 1e6;
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
    // The world point drawn at screen (width/2 + pan.x, height/2 + pan.y). It is (0, 0) normally and
    // the swarm's centre while following it, so a tiny swarm far from (0, 0) is positioned as
    // (its centre - anchor) * scale and stays exact however far in the view has zoomed.
    this.anchor = { x: 0, y: 0 };
    this.follow = false;     // zoom in and out with the swarm (see stepFollow)
    this.needAnchor = false;
    this.swarm = null;       // { x, y, extent }: centre and radius of the robots that have not crashed
    this.lastStep = 0;
    this.onFollowChange = null;
    // CSS px covered by floating panels; the world is fitted into what's left.
    this.insets = { left: 0, right: 0, top: 0, bottom: 0 };
    this.options = { showLights: false, visibility: null, links: false, allowedRegion: false, walk: null, grid: true, measure: null };
    this.gridState = null; // what the last grid drawn looked like (for the page and its checks)
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
      grid: document.documentElement.dataset.resolvedTheme === "dark" ? "170,180,210" : "70,80,120",
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

  setFrame(frame) { this.frame = frame; if (this.follow) this.measureSwarm(); this.requestDraw(); }

  /** Centre (the frame's origin, the centroid of the robots that have not crashed) and radius. */
  measureSwarm() {
    const f = this.frame;
    if (!f) { this.swarm = null; return; }
    const { xy, flags } = f;
    let far = 0, any = false;
    for (let i = 0, n = flags.length; i < n; i++) {
      if ((flags[i] & 3) === STATE_CRASH) continue;
      any = true;
      const d = xy[2 * i] * xy[2 * i] + xy[2 * i + 1] * xy[2 * i + 1];
      if (d > far) far = d;
    }
    // While a turn is played one at a time, the walking robot's destination must be in view too.
    const w = this.options.walk;
    if (w && f.target && f.target[2 * w.robot] === f.target[2 * w.robot]) {
      const tx = f.target[2 * w.robot], ty = f.target[2 * w.robot + 1];
      far = Math.max(far, tx * tx + ty * ty);
    }
    this.swarm = any ? { x: f.originX ?? 0, y: f.originY ?? 0, extent: Math.sqrt(far) } : null;
  }

  /** Follow the swarm: the view zooms in as it shrinks (and out if it spreads). Display only. */
  setFollow(on) {
    if (on === this.follow) return;
    this.follow = on;
    if (on) { this.needAnchor = true; this.lastStep = performance.now(); this.measureSwarm(); }
    this.onFollowChange?.(on);
    this.requestDraw();
  }

  /** One animation step of the follow view, called from draw(). */
  stepFollow() {
    const sw = this.swarm;
    if (!sw) return;
    const now = performance.now();
    const dt = Math.min(0.1, Math.max(0.001, (now - this.lastStep) / 1000));
    this.lastStep = now;
    // Playing a turn at a time: the camera stands still while a robot walks, so the only thing that
    // moves on screen is the robot, and it re-frames (quickly) while the next decision is shown.
    const walk = this.options.walk;
    if (walk && walk.phase === "move") return;
    if (this.needAnchor) {
      // Re-anchor on the swarm without moving anything on screen, then ease to the middle.
      const [px, py] = this.toScreen(sw.x, sw.y);
      this.pan = { x: px - this.width / 2, y: py - this.height / 2 };
      this.needAnchor = false;
    }
    const { left, right, top, bottom } = this.insets;
    const k = 1 - Math.exp(-dt / (walk ? Math.max(0.06, walk.settle / 4) : 0.25));
    this.anchor = walk
      ? { x: this.anchor.x + (sw.x - this.anchor.x) * k, y: this.anchor.y + (sw.y - this.anchor.y) * k }
      : { x: sw.x, y: sw.y };
    let gap = 0;
    // A swarm gathered to a point (radius 0) has nothing to zoom to: hold the zoom. Zooming goes down to a
    // swarm that is one step of the numbers across (half a step is the radius of two neighbouring numbers):
    // below that the positions cannot say anything more, however far in the view goes.
    const stepOfNumbers = ulpOf(Math.max(Math.abs(sw.x), Math.abs(sw.y)));
    if (sw.extent > Math.max(1e-30, 0.45 * stepOfNumbers)) {
      const w = Math.max(100, this.width - left - right), h = Math.max(100, this.height - top - bottom);
      const target = (0.92 * Math.min(w, h)) / (2 * sw.extent) / (this.scale / this.zoom);
      const next = Math.exp(Math.log(this.zoom) + (Math.log(target) - Math.log(this.zoom)) * k);
      this.zoom = Math.min(MAX_ZOOM, Math.max(0.05, next));
      gap = Math.abs(Math.log(target / this.zoom));
    }
    const cx = (left - right) / 2, cy = (top - bottom) / 2;
    this.pan.x += (cx - this.pan.x) * k;
    this.pan.y += (cy - this.pan.y) * k;
    const drift = walk ? Math.hypot(sw.x - this.anchor.x, sw.y - this.anchor.y) * this.scale : 0;
    if (gap > 1e-3 || drift > 0.2 || Math.hypot(cx - this.pan.x, cy - this.pan.y) > 0.2) this.requestDraw();
  }
  /** The region "Fit" frames: w × h world units centred on (cx, cy). */
  setView(w, h, cx = 0, cy = 0) {
    const fitted = this.isFitted();
    this.world = { w: Math.max(1e-6, w), h: Math.max(1e-6, h), cx, cy };
    if (fitted && !this.follow) this.fitView(); else this.requestDraw();
  }
  /** The start-area outline (see startOutline in start.js), or null for none. */
  setOutline(outline) { this.outline = outline; this.requestDraw(); }

  fittedPan() {
    const { left, right, top, bottom } = this.insets;
    const zoom = this.zoom;
    this.zoom = 1;
    const s = this.scale;
    this.zoom = zoom;
    return { x: (left - right) / 2 - (this.world.cx - this.anchor.x) * s, y: (top - bottom) / 2 + (this.world.cy - this.anchor.y) * s };
  }
  isFitted() {
    const p = this.fittedPan();
    return this.zoom === 1 && Math.abs(this.pan.x - p.x) < 0.5 && Math.abs(this.pan.y - p.y) < 0.5;
  }
  /** The Fit button: stop following and fit the view region into the area the panels leave free. */
  resetView() {
    this.setFollow(false);
    this.fitView();
  }
  fitView() {
    this.anchor = { x: 0, y: 0 };
    this.zoom = 1;
    this.pan = this.fittedPan();
    this.requestDraw();
  }

  setInsets(insets) {
    const fitted = this.isFitted();
    this.insets = insets;
    if (fitted && !this.follow) this.fitView(); // keep a fitted view fitted; leave a zoomed one alone
  }

  get scale() {
    const { left, right, top, bottom } = this.insets;
    const w = Math.max(100, this.width - left - right), h = Math.max(100, this.height - top - bottom);
    return Math.min(w / this.world.w, h / this.world.h) * 0.92 * this.zoom;
  }

  toScreen(x, y) {
    const s = this.scale;
    return [this.width / 2 + this.pan.x + (x - this.anchor.x) * s, this.height / 2 + this.pan.y - (y - this.anchor.y) * s];
  }

  toWorld(sx, sy) {
    const s = this.scale;
    return [this.anchor.x + (sx - this.width / 2 - this.pan.x) / s, this.anchor.y - (sy - this.height / 2 - this.pan.y) / s];
  }

  /** Zoom by `factor`, keeping the world point under (mx, my) (CSS px) still. */
  zoomAt(factor, mx = this.width / 2, my = this.height / 2) {
    this.setFollow(false); // zooming by hand takes over from following
    const [wx, wy] = this.toWorld(mx, my);
    this.zoom = Math.min(MAX_ZOOM, Math.max(0.05, this.zoom * factor));
    const s = this.scale;
    this.pan.x = mx - this.width / 2 - (wx - this.anchor.x) * s;
    this.pan.y = my - this.height / 2 + (wy - this.anchor.y) * s;
    this.requestDraw();
  }

  /**
   * A measuring point in world coordinates: a robot (`{ robot }`, wherever it is now: the point is attached to
   * it) or a fixed spot (`{ x, y }`). The frame's offsets are added to its f64 origin, so a robot's place is
   * exact to the f32 offset even at a billion times zoom.
   */
  measurePoint(pt) {
    if (!pt) return null;
    if (pt.robot !== undefined) {
      const f = this.frame;
      if (!f || pt.robot >= f.flags.length) return null;
      return { x: (f.originX ?? 0) + f.xy[2 * pt.robot], y: (f.originY ?? 0) + f.xy[2 * pt.robot + 1], robot: pt.robot };
    }
    return { x: pt.x, y: pt.y };
  }

  /** The measurement as it stands now: both points (the second is the pointer until it is placed), and the distance. */
  measureInfo() {
    const m = this.options.measure;
    if (!m || !m.a) return null;
    const a = this.measurePoint(m.a), b = this.measurePoint(m.b ?? m.hover);
    if (!a || !b) return null;
    const dx = b.x - a.x, dy = b.y - a.y;
    return { a, b, dx, dy, d: Math.hypot(dx, dy), done: Boolean(m.b) };
  }

  /** The part of the world the view shows (the area the panels leave free), in world coordinates. */
  viewRect() {
    const s = this.scale, ox = this.width / 2 + this.pan.x, oy = this.height / 2 + this.pan.y;
    const { left, right, top, bottom } = this.insets;
    return {
      x0: this.anchor.x + (left - ox) / s, x1: this.anchor.x + (this.width - right - ox) / s,
      y1: this.anchor.y - (top - oy) / s, y0: this.anchor.y - (this.height - bottom - oy) / s,
    };
  }

  panBy(dx, dy) { this.setFollow(false); this.pan.x += dx; this.pan.y += dy; this.requestDraw(); }

  centerOn(x, y) {
    this.setFollow(false);
    const s = this.scale, { left, right, top, bottom } = this.insets;
    this.pan.x = (left - right) / 2 - (x - this.anchor.x) * s;
    this.pan.y = (top - bottom) / 2 + (y - this.anchor.y) * s;
    this.requestDraw();
  }

  /**
   * A grid behind everything, so zooming (or following a swarm in) can be seen: with no grid, a view
   * zoomed in 30 million times looks the same as one that is not. One layer of lines per power of ten,
   * each faded in as its lines drift apart (10 px to 50 px), so a zoom shows lines sliding apart and
   * finer ones appearing. The layer whose lines are about 80 px apart or more is labelled with their
   * coordinates, along the top and left edges of the free area.
   */
  drawGrid(ctx, s, ox, oy) {
    const W = this.width, H = this.height, { left, right, top } = this.insets;
    const xMin = this.anchor.x - ox / s, xMax = this.anchor.x + (W - ox) / s;
    const yMax = this.anchor.y + oy / s, yMin = this.anchor.y - (H - oy) / s;  // y points up
    const span = Math.max(xMax - xMin, yMax - yMin);
    if (!(span > 0) || !isFinite(span)) { this.gridState = null; return; }
    // Lines closer than 16 steps of the numbers cannot be told apart (or labelled truthfully): leave them out.
    const mag = Math.max(Math.abs(xMin), Math.abs(xMax), Math.abs(yMin), Math.abs(yMax));
    const finest = 16 * ulpOf(mag);
    const eMin = Math.ceil(Math.log10(10 / s)), eMax = Math.floor(Math.log10(span * 2));
    const rgb = this.theme.grid;
    const sx = (x) => ox + (x - this.anchor.x) * s, sy = (y) => oy - (y - this.anchor.y) * s;
    ctx.save();
    ctx.lineWidth = 1;
    // Faint layers: one per power of ten, each fading in as its lines drift apart.
    for (let e = eMax; e >= eMin; e--) {
      const step = 10 ** e, px = step * s;
      const fade = Math.min(1, Math.max(0, (px - 10) / 40));
      if (fade <= 0 || step < finest) continue;
      const k0 = Math.ceil(xMin / step), k1 = Math.floor(xMax / step);
      const j0 = Math.ceil(yMin / step), j1 = Math.floor(yMax / step);
      if (k1 - k0 > 600 || j1 - j0 > 600) continue;
      ctx.strokeStyle = `rgba(${rgb},${(0.12 * fade).toFixed(3)})`;
      ctx.beginPath();
      for (let k = k0; k <= k1; k++) { const x = Math.round(sx(k * step)) + 0.5; ctx.moveTo(x, 0); ctx.lineTo(x, H); }
      for (let j = j0; j <= j1; j++) { const y = Math.round(sy(j * step)) + 0.5; ctx.moveTo(0, y); ctx.lineTo(W, y); }
      ctx.stroke();
    }
    // The labelled lines: 1, 2 or 5 times a power of ten, whichever puts them 90 to 225 px apart. They
    // are stronger, and their spacing steps 1, 2, 5, 10, ... as you zoom, so it can be read off.
    const want = 90 / s, pow = 10 ** Math.floor(Math.log10(want)), mant = want / pow;
    const step = (mant <= 1 ? 1 : mant <= 2 ? 2 : mant <= 5 ? 5 : 10) * pow, px = step * s;
    const kx0 = Math.ceil(xMin / step), kx1 = Math.floor(xMax / step);
    const ky0 = Math.ceil(yMin / step), ky1 = Math.floor(yMax / step);
    const labelled = kx1 - kx0 <= 600 && ky1 - ky0 <= 600 && step >= finest;
    if (labelled) {
      ctx.strokeStyle = `rgba(${rgb},0.26)`;
      ctx.beginPath();
      for (let k = kx0; k <= kx1; k++) { const x = Math.round(sx(k * step)) + 0.5; ctx.moveTo(x, 0); ctx.lineTo(x, H); }
      for (let j = ky0; j <= ky1; j++) { const y = Math.round(sy(j * step)) + 0.5; ctx.moveTo(0, y); ctx.lineTo(W, y); }
      ctx.stroke();
    }
    // The axes through (0, 0), when in view, a little stronger still.
    ctx.strokeStyle = `rgba(${rgb},0.4)`;
    ctx.beginPath();
    if (xMin <= 0 && xMax >= 0) { const x = Math.round(sx(0)) + 0.5; ctx.moveTo(x, 0); ctx.lineTo(x, H); }
    if (yMin <= 0 && yMax >= 0) { const y = Math.round(sy(0)) + 0.5; ctx.moveTo(0, y); ctx.lineTo(W, y); }
    ctx.stroke();
    const decade = Math.floor(Math.log10(step) + 1e-9);
    const text = (v) => {
      if (v === 0) return "0";
      if (decade >= 6 || decade < -15) return v.toExponential(2);
      return v.toFixed(Math.max(0, -decade));
    };
    ctx.font = "11px system-ui, sans-serif";
    ctx.textBaseline = "alphabetic";
    ctx.fillStyle = `rgba(${rgb},0.9)`;
    ctx.lineJoin = "round"; ctx.lineWidth = 3; ctx.strokeStyle = this.theme.bg;
    const label = (str, x, y) => { ctx.strokeText(str, x, y); ctx.fillText(str, x, y); };
    if (labelled) {
      for (let k = kx0; k <= kx1; k++) {
        const x = sx(k * step), str = text(k * step), w = ctx.measureText(str).width;
        if (x - w / 2 < left + 4 || x + w / 2 > W - right - 4) continue;
        label(str, x - w / 2, top + 16);
      }
      for (let j = ky0; j <= ky1; j++) {
        const y = sy(j * step);
        if (y < top + 30 || y > H - 20) continue;
        label(text(j * step), left + 6, y - 3);
      }
    }
    ctx.restore();
    this.gridState = { decade, spacing: step, px, layers: Math.max(0, eMax - eMin + 1), valid: step >= finest };
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
    const fx = ox + ((f.originX ?? 0) - this.anchor.x) * s, fy = oy - ((f.originY ?? 0) - this.anchor.y) * s;
    for (let i = 0, n = f.flags.length; i < n; i++) {
      const dx = fx + xy[2 * i] * s - sx, dy = fy - xy[2 * i + 1] * s - sy;
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

    if (this.follow && frame) this.stepFollow();
    const s = this.scale;
    const ox = this.width / 2 + this.pan.x;
    const oy = this.height / 2 + this.pan.y;
    const WX = (x) => ox + (x - this.anchor.x) * s;
    const WY = (y) => oy - (y - this.anchor.y) * s;

    if (this.options.grid) this.drawGrid(ctx, s, ox, oy);
    else this.gridState = null;

    // Start area (a drawing guide, not a wall: robots can move past it).
    const o = this.outline;
    if (o) {
      ctx.strokeStyle = this.theme.bounds;
      ctx.lineWidth = 1;
      ctx.setLineDash([5, 5]);
      ctx.beginPath();
      const rot = ((o.rotation ?? 0) * Math.PI) / 180;
      const P = (x, y) => [WX(x * Math.cos(rot) - y * Math.sin(rot)), WY(x * Math.sin(rot) + y * Math.cos(rot))];
      if (o.kind === "rect") {
        const corners = [[-o.w / 2, -o.h / 2], [o.w / 2, -o.h / 2], [o.w / 2, o.h / 2], [-o.w / 2, o.h / 2]].map(([x, y]) => P(x, y));
        corners.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
        ctx.closePath();
      } else if (o.kind === "circle" || o.kind === "ring") {
        ctx.moveTo(WX(o.r), WY(0)); ctx.arc(WX(0), WY(0), o.r * s, 0, 2 * Math.PI);
        if (o.kind === "ring" && o.inner > 0) { ctx.moveTo(WX(o.inner), WY(0)); ctx.arc(WX(0), WY(0), o.inner * s, 0, 2 * Math.PI); }
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
    // The frame's numbers are offsets from its origin (see Frame in lcm-core): add the origin
    // here in f64, so a swarm far from (0, 0) and tiny next to that distance stays distinct.
    const fx = ox + ((frame.originX ?? 0) - this.anchor.x) * s, fy = oy - ((frame.originY ?? 0) - this.anchor.y) * s;
    const X = (x) => fx + x * s;
    const Y = (y) => fy - y * s;
    const n = flags.length;
    if (this.colorOf.length !== n) this.colorOf = new Uint8Array(n);
    const colorOf = this.colorOf, counts = this.counts;
    counts.fill(0);
    for (let i = 0; i < n; i++) { const c = colorIndex(flags[i], fault[i], task[i]); colorOf[i] = c; counts[c]++; }
    const r = this.robotRadius(n);
    const V = this.options.visibility;

    // Visibility circles (behind everything); the links replace them when they are shown.
    if (V && n <= LOD.visibility && !this.options.links && V * s < HUGE) {
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

    // Who sees whom: a line between robots within sight of each other; a link that is
    // nearly at the limit is drawn in orange, since it is the one most at risk of breaking.
    if (this.options.links && V && n <= LOD.links) {
      const limit2 = V * V, warn2 = (0.9 * V) * (0.9 * V);
      for (const [style, lo, hi] of [["rgba(90,100,170,.38)", 0, warn2], ["rgba(235,120,40,.8)", warn2, limit2]]) {
        ctx.strokeStyle = style;
        ctx.beginPath();
        for (let i = 0; i < n; i++) {
          if ((flags[i] & 3) === STATE_CRASH) continue;
          const xi = xy[2 * i], yi = xy[2 * i + 1];
          for (let j = i + 1; j < n; j++) {
            if ((flags[j] & 3) === STATE_CRASH) continue;
            const dx = xy[2 * j] - xi, dy = xy[2 * j + 1] - yi, d2 = dx * dx + dy * dy;
            if (d2 > lo && d2 <= hi) { ctx.moveTo(X(xi), Y(yi)); ctx.lineTo(X(xy[2 * j]), Y(xy[2 * j + 1])); }
          }
        }
        ctx.stroke();
      }
    }

    // Circles the robots computed (SEC and friends).
    if (n <= LOD.circles) {
      // With a robot selected, everyone else's circle fades so its own stands out.
      ctx.strokeStyle = this.selected >= 0 || this.options.walk ? "rgba(255,150,0,0.16)" : "rgba(255,150,0,0.55)";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      for (let i = 0; i < n; i++) {
        const cr = circle[3 * i + 2];
        if (!(cr >= 0) || cr * s > HUGE) continue;
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

    // Playing a turn at a time: name the robots, ring the one whose turn it is, and show where the
    // algorithm sends it. While it decides, thin lines show whom it looked at.
    const walk = this.options.walk;
    if (walk && walk.robot < n) {
      const i = walk.robot, rx = X(xy[2 * i]), ry = Y(xy[2 * i + 1]);
      const has = target[2 * i] === target[2 * i];
      const tx = has ? X(target[2 * i]) : rx, ty = has ? Y(target[2 * i + 1]) : ry;
      const accent = this.theme.accent;
      ctx.save();
      // The circle this robot worked out stands out; the other robots' circles are faded (above).
      const cr = circle[3 * i + 2];
      if (cr >= 0 && cr * s < HUGE) {
        ctx.strokeStyle = "rgba(255,150,0,0.95)"; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(X(circle[3 * i]), Y(circle[3 * i + 1]), cr * s, 0, 2 * Math.PI); ctx.stroke();
      }
      if (walk.phase === "decide" && n <= 24) {
        ctx.strokeStyle = "rgba(110,120,190,.55)"; ctx.lineWidth = 1.3; ctx.setLineDash([3, 4]);
        ctx.beginPath();
        for (let j = 0; j < n; j++) {
          if (j === i || (flags[j] & 3) === STATE_CRASH) continue;
          if (V && Math.hypot(xy[2 * j] - xy[2 * i], xy[2 * j + 1] - xy[2 * i + 1]) > V) continue;
          ctx.moveTo(rx, ry); ctx.lineTo(X(xy[2 * j]), Y(xy[2 * j + 1]));
        }
        ctx.stroke();
        ctx.setLineDash([]);
      }
      if (has) {
        ctx.strokeStyle = accent; ctx.lineWidth = 2.2; ctx.lineCap = "round";
        ctx.beginPath(); ctx.moveTo(rx, ry); ctx.lineTo(tx, ty); ctx.stroke();
        ctx.lineWidth = 3.4;
        ctx.beginPath(); ctx.moveTo(tx - 8, ty - 8); ctx.lineTo(tx + 8, ty + 8); ctx.moveTo(tx + 8, ty - 8); ctx.lineTo(tx - 8, ty + 8); ctx.stroke();
      }
      ctx.strokeStyle = accent; ctx.lineWidth = 3;
      ctx.beginPath(); ctx.arc(rx, ry, Math.max(r, 2) + 6, 0, 2 * Math.PI); ctx.stroke();
      if (n <= 12) {
        ctx.font = "600 13px system-ui, sans-serif"; ctx.lineJoin = "round"; ctx.textBaseline = "alphabetic";
        for (let j = 0; j < n; j++) {
          if ((flags[j] & 3) === STATE_CRASH) continue;
          const label = `R${j}`, lx = X(xy[2 * j]) + Math.max(r, 2) + 6, ly = Y(xy[2 * j + 1]) - Math.max(r, 2) - 6;
          ctx.lineWidth = 4; ctx.strokeStyle = this.theme.bg; ctx.strokeText(label, lx, ly);
          ctx.fillStyle = j === i ? accent : this.theme.text; ctx.fillText(label, lx, ly);
        }
      }
      ctx.restore();
    }

    // The measuring tool: a ruler from one point to another, with the distance written on it.
    const measure = this.options.measure;
    if (measure) {
      const hover = measure.hover && measure.hover.robot !== undefined ? this.measurePoint(measure.hover) : null;
      const info = this.measureInfo();
      ctx.save();
      const color = "#c026d3";
      const ring = (p) => { ctx.beginPath(); ctx.arc(WX(p.x), WY(p.y), Math.max(r, 2) + 5, 0, 2 * Math.PI); ctx.stroke(); };
      if (hover && !measure.b) { ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.globalAlpha = 0.6; ring(hover); ctx.globalAlpha = 1; }
      if (info) {
        const ax = WX(info.a.x), ay = WY(info.a.y), bx = WX(info.b.x), by = WY(info.b.y);
        ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2; ctx.lineCap = "round";
        if (!info.done) ctx.setLineDash([6, 5]);
        ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
        ctx.setLineDash([]);
        const len = Math.hypot(bx - ax, by - ay) || 1, nx = -(by - ay) / len, ny = (bx - ax) / len;
        for (const [px, py] of [[ax, ay], [bx, by]]) {  // ruler ends
          ctx.beginPath(); ctx.moveTo(px - nx * 7, py - ny * 7); ctx.lineTo(px + nx * 7, py + ny * 7); ctx.stroke();
        }
        if (info.a.robot !== undefined) ring(info.a);
        else { ctx.beginPath(); ctx.arc(ax, ay, 3.5, 0, 2 * Math.PI); ctx.fill(); }
        if (info.done && info.b.robot !== undefined) ring(info.b);
        else if (info.done) { ctx.beginPath(); ctx.arc(bx, by, 3.5, 0, 2 * Math.PI); ctx.fill(); }
        // The label, beside the middle of the line.
        const name = (p, fallback) => (p.robot !== undefined ? `R${p.robot}` : fallback);
        const title = `${name(info.a, "A")} → ${info.done ? name(info.b, "B") : "…"}  ${fmtDistance(info.d)}`;
        const detail = `dx ${fmtDistance(Math.abs(info.dx))}   dy ${fmtDistance(Math.abs(info.dy))}`;
        ctx.font = "600 13px system-ui, sans-serif";
        const w1 = ctx.measureText(title).width;
        ctx.font = "11px system-ui, sans-serif";
        const w2 = ctx.measureText(detail).width, bw = Math.max(w1, w2) + 16, bh = 40;
        const { left, top } = this.insets;
        let lx = (ax + bx) / 2 + nx * (bh / 2 + 12) - bw / 2, ly = (ay + by) / 2 + ny * (bh / 2 + 12) - bh / 2;
        lx = Math.min(Math.max(lx, left + 6), this.width - bw - 6); ly = Math.min(Math.max(ly, top + 6), this.height - bh - 6);
        ctx.fillStyle = this.theme.bg; ctx.globalAlpha = 0.94;
        ctx.beginPath(); (ctx.roundRect ? ctx.roundRect(lx, ly, bw, bh, 8) : ctx.rect(lx, ly, bw, bh)); ctx.fill();
        ctx.globalAlpha = 1; ctx.strokeStyle = color; ctx.lineWidth = 1.2; ctx.stroke();
        ctx.fillStyle = color; ctx.font = "600 13px system-ui, sans-serif"; ctx.textBaseline = "alphabetic";
        ctx.fillText(title, lx + 8, ly + 17);
        ctx.fillStyle = this.theme.text; ctx.globalAlpha = 0.75; ctx.font = "11px system-ui, sans-serif";
        ctx.fillText(detail, lx + 8, ly + 33);
      }
      ctx.restore();
    }

    // Hovered and selected robots, drawn in full at any swarm size.
    const mark = (i, ring, strong) => {
      if (i < 0 || i >= n) return;
      const x = X(xy[2 * i]), y = Y(xy[2 * i + 1]);
      ctx.strokeStyle = this.theme.accent;
      if (strong) {
        if (V && V * s < HUGE && !((flags[i] & 3) === STATE_CRASH)) {
          ctx.globalAlpha = 0.35;
          ctx.beginPath(); ctx.arc(x, y, V * s, 0, 2 * Math.PI); ctx.stroke();
          ctx.globalAlpha = 1;
        }
        if (this.options.allowedRegion && V && V * s < HUGE && !((flags[i] & 3) === STATE_CRASH)) {
          // Go-To-Center: the destination must stay inside the disc of radius V/2 around the
          // midpoint with every robot it sees; where the discs overlap is the allowed region.
          const xi = xy[2 * i], yi = xy[2 * i + 1];
          ctx.strokeStyle = "rgba(40,160,110,.8)"; ctx.fillStyle = "rgba(40,160,110,.06)"; ctx.lineWidth = 1;
          for (let j = 0; j < n; j++) {
            if (j === i || (flags[j] & 3) === STATE_CRASH) continue;
            const dx = xy[2 * j] - xi, dy = xy[2 * j + 1] - yi;
            if (dx * dx + dy * dy > V * V) continue;
            ctx.beginPath(); ctx.arc(X(xi + dx / 2), Y(yi + dy / 2), (V / 2) * s, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
            ctx.beginPath(); ctx.arc(X(xy[2 * j]), Y(xy[2 * j + 1]), Math.max(r, 2) + 3, 0, 2 * Math.PI); ctx.stroke();
          }
          ctx.strokeStyle = this.theme.accent;
        }
        const cr = circle[3 * i + 2];
        if (cr >= 0 && cr * s < HUGE) {
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

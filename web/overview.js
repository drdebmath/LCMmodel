/* The overview map: a small picture of the whole starting area with a rectangle for the part of it
 * the main view shows. Zoomed far in, the rectangle shrinks to a dot, so how far in the view is can be
 * seen at a glance (the zoom gauge says it in numbers). Clicking the map moves the view there.
 *
 * It draws from the renderer's own numbers (`world`, `viewRect()`, the current frame) in f64 world
 * coordinates, so a view zoomed in a billion times still lands on the right spot of the map. */

const W = 176, H = 132, PAD = 10;

export class Overview {
  constructor(panel, canvas, renderer, { onCenter } = {}) {
    this.panel = panel;
    this.canvas = canvas;
    this.renderer = renderer;
    this.onCenter = onCenter;
    this.start = null;        // Float64Array [x0, y0, x1, y1, …]: where the robots started this run
    this.last = null;         // what the last draw looked like, for the page's checks
    this.map = null;          // { x0, y1, k }: world → map pixels
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.round(W * dpr);
    canvas.height = Math.round(H * dpr);
    canvas.style.width = `${W}px`;
    canvas.style.height = `${H}px`;
    this.dpr = dpr;
    canvas.addEventListener("pointerdown", (e) => this.click(e));
  }

  /** Remember where the robots started (the first frame of a run), for the ghost dots. */
  setStart(frame) {
    const n = frame.flags.length;
    if (n > 5000) { this.start = null; return; }
    const p = new Float64Array(2 * n);
    for (let i = 0; i < n; i++) { p[2 * i] = frame.x(i); p[2 * i + 1] = frame.y(i); }
    this.start = p;
  }

  get visible() { return !this.panel.hidden && getComputedStyle(this.panel).display !== "none"; }

  /** The world region the map shows: the starting area, and the view too if that is bigger. */
  bounds(view) {
    const w = this.renderer.world;
    let x0 = w.cx - w.w / 2, x1 = w.cx + w.w / 2, y0 = w.cy - w.h / 2, y1 = w.cy + w.h / 2;
    x0 = Math.min(x0, view.x0); x1 = Math.max(x1, view.x1); y0 = Math.min(y0, view.y0); y1 = Math.max(y1, view.y1);
    return { x0, x1, y0, y1 };
  }

  draw() {
    if (!this.visible) return;
    const r = this.renderer, ctx = this.canvas.getContext("2d"), theme = r.theme;
    const view = r.viewRect();
    const b = this.bounds(view);
    const k = Math.min((W - 2 * PAD) / (b.x1 - b.x0), (H - 2 * PAD) / (b.y1 - b.y0));
    // Centre the picture in the map.
    const offX = (W - k * (b.x1 - b.x0)) / 2, offY = (H - k * (b.y1 - b.y0)) / 2;
    const mx = (x) => offX + (x - b.x0) * k, my = (y) => H - offY - (y - b.y0) * k;
    this.map = { x0: b.x0, y1: b.y1, k, offX, offY };
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = theme.bg;
    ctx.fillRect(0, 0, W, H);

    // The starting area (what "Fit" shows).
    const w = r.world;
    ctx.strokeStyle = theme.bounds; ctx.lineWidth = 1; ctx.setLineDash([3, 3]);
    ctx.strokeRect(Math.round(mx(w.cx - w.w / 2)) + 0.5, Math.round(my(w.cy + w.h / 2)) + 0.5, Math.round(w.w * k), Math.round(w.h * k));
    ctx.setLineDash([]);

    // Where the robots started (ghosts), then where they are now.
    if (this.start) {
      ctx.fillStyle = "rgba(120,130,150,0.55)";
      const p = this.start;
      for (let i = 0; i < p.length; i += 2) ctx.fillRect(mx(p[i]) - 1, my(p[i + 1]) - 1, 2, 2);
    }
    const f = r.frame;
    if (f) {
      const n = f.flags.length, step = Math.max(1, Math.floor(n / 3000));
      ctx.fillStyle = theme.accent;
      for (let i = 0; i < n; i += step) {
        if ((f.flags[i] & 3) === 3) continue;  // crashed
        ctx.fillRect(mx(f.x(i)) - 1.25, my(f.y(i)) - 1.25, 2.5, 2.5);
      }
    }

    // The part of the world the main view shows. Tiny, it becomes a marked dot.
    const rx0 = mx(view.x0), rx1 = mx(view.x1), ry0 = my(view.y1), ry1 = my(view.y0);
    const rw = rx1 - rx0, rh = ry1 - ry0;
    const tiny = rw < 7 || rh < 7;
    const cx = (rx0 + rx1) / 2, cy = (ry0 + ry1) / 2;
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = "rgba(255,150,0,0.95)"; ctx.fillStyle = "rgba(255,150,0,0.14)";
    if (!tiny) {
      ctx.fillRect(rx0, ry0, rw, rh);
      ctx.strokeRect(Math.round(rx0) + 0.5, Math.round(ry0) + 0.5, Math.round(rw), Math.round(rh));
    } else {
      ctx.beginPath(); ctx.arc(cx, cy, 6, 0, 2 * Math.PI); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(cx - 11, cy); ctx.lineTo(cx - 6, cy); ctx.moveTo(cx + 6, cy); ctx.lineTo(cx + 11, cy);
      ctx.moveTo(cx, cy - 11); ctx.lineTo(cx, cy - 6); ctx.moveTo(cx, cy + 6); ctx.lineTo(cx, cy + 11); ctx.stroke();
      ctx.fillStyle = "rgba(255,150,0,0.95)"; ctx.fillRect(cx - 1.5, cy - 1.5, 3, 3);
    }
    ctx.fillStyle = theme.text; ctx.globalAlpha = 0.6; ctx.font = "10px system-ui, sans-serif"; ctx.textBaseline = "top";
    ctx.fillText("Overview", 6, 5);
    ctx.globalAlpha = 1;
    this.last = { rectW: rw, rectH: rh, tiny, cx, cy, width: W, height: H };
  }

  /** The world point under a click on the map, and the view moves there. */
  click(e) {
    if (!this.map) return;
    const rect = this.canvas.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * W, py = ((e.clientY - rect.top) / rect.height) * H;
    const { x0, y1, k, offX, offY } = this.map;
    // Invert mx / my.
    const wx = x0 + (px - offX) / k, wy = y1 - (py - offY) / k;
    this.onCenter?.(wx, wy);
    e.preventDefault();
  }
}

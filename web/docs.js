/* Flowchart page: chart navigation, pan/zoom, hover focus, the step detail
 * panel with highlighted source, and the guided tour. Chart data comes from
 * the JSON block that scripts/gen-flowcharts.py writes into docs.html.
 * Deep links: docs.html#chart or docs.html#chart/step. */
import { icon, renderIcons } from "./icons.js";

const DATA = JSON.parse(document.getElementById("doc-data").textContent);
const $ = (id) => document.getElementById(id);
const KIND = { entry: "Start", step: "Step", decision: "Question", end: "Finish", out: "Ends this pass" };
renderIcons();

/** How much chart is drawn beyond each edge of the stage, as a fraction of
 *  its size, so a drag can slide the picture without uncovering blank space. */
const MARGIN = 0.5;
/** Below this zoom the step titles stop being readable. */
const MIN_READABLE = 0.6;
let chartId = null;
let selected = null;
const views = {}; // chart id -> view state

/* ------------------------------------------------------------ pan & zoom */
class View {
  constructor(section) {
    this.section = section;
    this.stage = section.querySelector(".stage");
    this.svg = this.stage.querySelector("svg");
    this.w = Number(this.svg.dataset.width);
    this.h = Number(this.svg.dataset.height);
    this.box = { x: 0, y: 0, w: this.w, h: this.h };
    this.drawn = { ...this.box }; // the view box the SVG is actually drawn with
    this.live = false; // a drag, pinch or wheel is under way
    this.frame = 0;
    this.level = section.querySelector(".zoom-level");
    this.measure();
    this.bind();
  }
  /** The stage's size, measured once per layout change rather than on every move. */
  measure() { this.rect = this.stage.getBoundingClientRect(); return this.rect; }
  /**
   * Shows `this.box`. During a gesture the chart is not redrawn: the picture
   * already drawn is slid and scaled on the GPU (a CSS transform), at most once
   * per frame. When the gesture ends, `commit` redraws it sharply in place.
   */
  apply() {
    this.clamp();
    if (!this.live) { this.commit(); return; }
    if (!this.frame) this.frame = requestAnimationFrame(() => { this.frame = 0; if (this.live) this.slide(); });
  }
  slide() {
    const d = this.drawn, b = this.box, { width: W, height: H } = this.rect;
    if (!W || !d.w || !b.w) return;
    const s = W / b.w, k = d.w / b.w;
    const dx = (d.x - b.x) * s, dy = (d.y - b.y) * s;
    // The picture has half a stage of chart drawn beyond each edge. Redraw
    // only when the view would leave it (or the picture would get very
    // stretched); otherwise stretching and sliding it costs no drawing.
    const covered = b.x >= d.x - d.w * MARGIN && b.y >= d.y - d.h * MARGIN
      && b.x + b.w <= d.x + d.w * (1 + MARGIN) && b.y + b.h <= d.y + d.h * (1 + MARGIN);
    if (!covered || k > 3) { this.commit(); return; }
    const ox = W * MARGIN * (1 - k), oy = H * MARGIN * (1 - k);
    this.svg.style.transform = `translate(${dx + ox}px, ${dy + oy}px) scale(${k})`;
  }
  commit() {
    if (this.frame) { cancelAnimationFrame(this.frame); this.frame = 0; }
    const b = this.box;
    // Drawn with a margin of chart around the visible part (see `slide`).
    this.svg.setAttribute("viewBox", `${b.x - b.w * MARGIN} ${b.y - b.h * MARGIN} ${b.w * (1 + 2 * MARGIN)} ${b.h * (1 + 2 * MARGIN)}`);
    this.svg.style.transform = "";
    this.drawn = { ...b };
    const rect = this.rect;
    if (rect.width) this.level.textContent = `${Math.round(Math.min(rect.width / b.w, rect.height / b.h) * 100)}%`;
  }
  /** Start or end a gesture; ending it redraws the chart once, sharply. */
  setLive(on) {
    if (on === this.live) return;
    this.live = on;
    if (on) this.measure(); else { this.clamp(); this.commit(); }
  }
  /** Keep some of the chart in view: never scroll past its edges by more than a margin. */
  clamp() {
    const b = this.box, m = 40;
    const clampAxis = (pos, size, total) => (size >= total + 2 * m ? (total - size) / 2 : Math.min(Math.max(pos, -m), total + m - size));
    b.x = clampAxis(b.x, b.w, this.w);
    b.y = clampAxis(b.y, b.h, this.h);
  }
  /** Show the whole chart, at most at 100 %. */
  fit() {
    const rect = this.measure();
    if (!rect.width) return;
    const scale = Math.min(1, (rect.width - 24) / this.w, (rect.height - 64) / this.h);
    const w = rect.width / scale, h = rect.height / scale;
    this.box = { x: (this.w - w) / 2, y: (this.h - h) / 2 + 20 / scale, w, h };
    this.lastRect = rect;
    this.apply();
  }
  /** Readable start: the chart's full width (at most 100 %), from the top. */
  fitWidth() {
    const rect = this.measure();
    if (!rect.width) return;
    const scale = Math.min(1, (rect.width - 24) / this.w);
    if (scale < MIN_READABLE) { this.startReadable(rect); return; }
    if (this.h * scale <= rect.height - 56) { this.fit(); return; }
    const w = rect.width / scale, h = rect.height / scale;
    this.box = { x: (this.w - w) / 2, y: 0, w, h };
    this.lastRect = rect;
    this.apply();
  }
  /** A narrow screen can't show the whole width readably: start at a readable
   *  zoom, at the top, centred on the chart's first step. */
  startReadable(rect) {
    const first = this.section.querySelector(`.node[data-id="${DATA.charts[this.section.dataset.chart].order[0]}"]`);
    const bb = first ? first.getBBox() : { x: this.w / 2, width: 0 };
    const w = rect.width / MIN_READABLE, h = rect.height / MIN_READABLE;
    this.box = { x: bb.x + bb.width / 2 - w / 2, y: 0, w, h };
    this.lastRect = rect;
    this.apply();
  }
  scale() { return this.rect.width / this.box.w; }
  /** The stage changed size: keep the zoom level and the view's centre. */
  resized() {
    const rect = this.measure();
    if (!rect.width || !this.lastRect) { this.lastRect = rect; this.apply(); return; }
    const s = this.lastRect.width / this.box.w;
    const cx = this.box.x + this.box.w / 2, cy = this.box.y + this.box.h / 2;
    const w = rect.width / s, h = rect.height / s;
    this.box = { x: cx - w / 2, y: cy - h / 2, w, h };
    this.lastRect = rect;
    this.apply();
  }
  zoom(factor, cx, cy) {
    const rect = this.rect;
    const s = this.scale();
    const next = Math.min(3, Math.max(0.2, s * factor));
    const px = cx ?? rect.width / 2, py = cy ?? rect.height / 2;
    const wx = this.box.x + px / s, wy = this.box.y + py / s;
    const w = rect.width / next, h = rect.height / next;
    this.box = { x: wx - px / next, y: wy - py / next, w, h };
    this.apply();
  }
  /** Keep a node in view, centring it if it isn't comfortably visible. */
  reveal(g) {
    const bb = g.getBBox();
    const b = this.box, m = 40 / this.scale();
    if (bb.x > b.x + m && bb.y > b.y + m && bb.x + bb.width < b.x + b.w - m && bb.y + bb.height < b.y + b.h - m - 50 / this.scale()) return;
    this.box = { ...b, x: bb.x + bb.width / 2 - b.w / 2, y: bb.y + bb.height / 2 - b.h / 2 + 20 / this.scale() };
    this.apply();
  }
  bind() {
    // One finger or the mouse drags the chart, from anywhere, boxes included;
    // two fingers pinch to zoom. A press that barely moves is still a click.
    const pointers = new Map();
    let drag = null, pinch = null, moved = false;
    const midpoint = () => {
      const [a, b] = [...pointers.values()];
      return { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2, d: Math.hypot(a.x - b.x, a.y - b.y) };
    };
    this.stage.addEventListener("pointerdown", (e) => {
      pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
      if (pointers.size === 1) { drag = { x: e.clientX, y: e.clientY, start: { x: e.clientX, y: e.clientY } }; moved = false; }
      if (pointers.size === 2) { pinch = midpoint(); drag = null; moved = true; this.setLive(true); }
    });
    this.stage.addEventListener("pointermove", (e) => {
      if (!pointers.has(e.pointerId)) return;
      pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
      if (pinch && pointers.size === 2) {
        const now = midpoint(), rect = this.rect;
        if (pinch.d > 0 && now.d > 0) this.zoom(now.d / pinch.d, now.x - rect.left, now.y - rect.top);
        const s = this.scale();
        this.box.x -= (now.x - pinch.x) / s;
        this.box.y -= (now.y - pinch.y) / s;
        this.apply();
        pinch = now;
        return;
      }
      if (!drag) return;
      if (!moved && Math.hypot(e.clientX - drag.start.x, e.clientY - drag.start.y) < 6) return;
      if (!moved) { moved = true; this.stage.setPointerCapture(e.pointerId); this.stage.classList.add("panning"); this.setLive(true); }
      const s = this.scale();
      this.box.x -= (e.clientX - drag.x) / s;
      this.box.y -= (e.clientY - drag.y) / s;
      drag = { ...drag, x: e.clientX, y: e.clientY };
      this.apply();
    });
    let tapAt = 0;
    const end = (e) => {
      // A finger tap that didn't move opens the box at once. Browsers swallow
      // the click of a tap that comes right after a swipe, so don't wait for it.
      if (e.type === "pointerup" && e.pointerType !== "mouse" && !moved && pointers.size === 1) {
        const g = e.target.closest?.(".node");
        if (g) { tapAt = performance.now(); g.dispatchEvent(new MouseEvent("click", { bubbles: true })); }
      }
      pointers.delete(e.pointerId);
      if (pointers.size < 2) pinch = null;
      if (pointers.size === 0) { drag = null; this.stage.classList.remove("panning"); this.setLive(false); }
    };
    this.stage.addEventListener("pointerup", end);
    this.stage.addEventListener("pointercancel", end);
    // A drag that started on a box must not also open it.
    // ...and the browser's own click for that tap, if it comes, is not a second one.
    this.stage.addEventListener("click", (e) => {
      if (moved || (e.isTrusted && performance.now() - tapAt < 700)) { e.stopPropagation(); e.preventDefault(); moved = false; }
    }, true);
    // Wheel scrolls; Ctrl + wheel (and trackpad pinch, which arrives as it) zooms.
    let wheelEnd = 0;
    this.stage.addEventListener("wheel", (e) => {
      e.preventDefault();
      this.setLive(true);
      clearTimeout(wheelEnd);
      wheelEnd = setTimeout(() => this.setLive(false), 140);
      const rect = this.rect;
      if (e.ctrlKey || e.metaKey) {
        this.zoom(Math.exp(-e.deltaY * 0.01), e.clientX - rect.left, e.clientY - rect.top);
      } else {
        const s = this.scale();
        this.box.x += (e.shiftKey ? e.deltaY : e.deltaX) / s;
        this.box.y += (e.shiftKey ? 0 : e.deltaY) / s;
        this.apply();
      }
    }, { passive: false });
    this.section.querySelector('[data-zoom="in"]').addEventListener("click", () => this.zoom(1.25));
    this.section.querySelector('[data-zoom="out"]').addEventListener("click", () => this.zoom(0.8));
    this.section.querySelector('[data-zoom="fit"]').addEventListener("click", () => this.fit());
  }
}

/* ------------------------------------------------------------ focus */
function neighbours(chart, id) {
  const n = DATA.charts[chart].nodes[id];
  return new Set([id, ...n.in, ...n.out]);
}

function setFocus(section, id) {
  const stage = section.querySelector(".stage");
  stage.classList.toggle("focusing", Boolean(id));
  for (const el of stage.querySelectorAll(".hot")) el.classList.remove("hot");
  for (const edge of stage.querySelectorAll(".edge")) edge.setAttribute("marker-end", edge.getAttribute("marker-end").replace(/-hot\)$/, `-${edge.dataset.tone})`));
  if (!id) return;
  const near = neighbours(section.dataset.chart, id);
  for (const g of stage.querySelectorAll(".node")) if (near.has(g.dataset.id)) g.classList.add("hot");
  for (const el of stage.querySelectorAll(".edge, .chip")) {
    if (el.dataset.from === id || el.dataset.to === id) {
      el.classList.add("hot");
      if (el.classList.contains("edge")) el.setAttribute("marker-end", el.getAttribute("marker-end").replace(/-(plain|yes|no)\)$/, "-hot)"));
    }
  }
}

/* ------------------------------------------------------------ detail panel */
function showDetail(id) {
  const chart = DATA.charts[chartId];
  const n = chart.nodes[id];
  const section = currentSection();
  selected = id;
  for (const g of section.querySelectorAll(".node.selected")) g.classList.remove("selected");
  const g = section.querySelector(`.node[data-id="${id}"]`);
  g.classList.add("selected");
  setFocus(section, id);
  views[chartId].reveal(g);
  // On small screens opening the details shrinks the chart: keep the step in view.
  const opening = !document.body.classList.contains("has-detail");

  $("detail").querySelector(".detail-empty").hidden = true;
  $("detail").querySelector(".detail-body").hidden = false;
  document.body.classList.add("has-detail");
  if (opening) requestAnimationFrame(() => { views[chartId].resized(); views[chartId].reveal(g); });
  const chip = $("d_kind");
  chip.textContent = KIND[n.kind];
  chip.className = `kind-chip k-${n.kind}`;
  const pos = chart.order.indexOf(id);
  $("d_pos").textContent = `${pos + 1} / ${chart.order.length}`;
  $("d_prev").disabled = pos <= 0;
  $("d_next").disabled = pos >= chart.order.length - 1;
  $("d_title").textContent = n.title;
  $("d_sub").textContent = n.sub ?? "";
  $("d_desc").textContent = n.desc || defaultDesc(n);

  const link = (to, label) => `<button class="btn" data-go="${to}">${label ? `<b>${label}</b>&nbsp;` : ""}${chart.nodes[to].title}</button>`;
  const outs = n.out.map((to) => link(to, n.labels[to])).join("");
  const ins = n.in.map((from) => link(from)).join("");
  $("d_links").innerHTML = (ins ? `<span class="dir">from</span>${ins}` : "") + (outs ? `<span class="dir">next</span>${outs}` : "");
  for (const b of $("d_links").querySelectorAll("[data-go]")) b.addEventListener("click", () => showDetail(b.dataset.go));

  const code = n.code;
  $("d_code").hidden = !code;
  if (code) {
    $("d_file").textContent = `${code.path}:${code.line}`;
    $("d_github").href = `${DATA.repo}/${code.path}#L${code.line}`;
    $("d_src").style.setProperty("--start", String(code.line - 1));
    $("d_src").innerHTML = code.html.map((l) => `<span class="line">${l}</span>`).join("");
    $("d_copy").textContent = "Copy";
  }
  history.replaceState(null, "", `#${chartId}/${id}`);
}

function defaultDesc(n) {
  if (n.kind === "decision") return "A question: the arrows leaving this step are its answers.";
  if (n.kind === "end") return "The run or this robot's cycle finishes here.";
  if (n.kind === "out") return "This pass ends here; the robot or scheduler carries on with the next event.";
  return "";
}

function clearDetail() {
  selected = null;
  const section = currentSection();
  for (const g of section.querySelectorAll(".node.selected")) g.classList.remove("selected");
  setFocus(section, null);
  $("detail").querySelector(".detail-empty").hidden = false;
  $("detail").querySelector(".detail-body").hidden = true;
  document.body.classList.remove("has-detail");
  history.replaceState(null, "", `#${chartId}`);
}

function stepTour(delta) {
  const order = DATA.charts[chartId].order;
  const pos = selected ? order.indexOf(selected) : -1;
  const next = Math.min(order.length - 1, Math.max(0, pos + delta));
  showDetail(order[next]);
}

$("d_prev").addEventListener("click", () => stepTour(-1));
$("d_next").addEventListener("click", () => stepTour(1));
$("d_close").addEventListener("click", clearDetail);
$("d_copy").addEventListener("click", async () => {
  const code = DATA.charts[chartId].nodes[selected]?.code;
  if (!code) return;
  try { await navigator.clipboard.writeText(code.source); $("d_copy").textContent = "Copied"; }
  catch { $("d_copy").textContent = "Copy failed"; }
});

/* ------------------------------------------------------------ charts */
const currentSection = () => document.querySelector(`.chart[data-chart="${chartId}"]`);

function showChart(id, step) {
  if (!DATA.charts[id]) id = Object.keys(DATA.charts)[0];
  const changed = id !== chartId;
  chartId = id;
  for (const s of document.querySelectorAll(".chart")) s.hidden = s.dataset.chart !== id;
  for (const a of document.querySelectorAll(".nav-item")) {
    if (a.dataset.chart === id) a.setAttribute("aria-current", "page"); else a.removeAttribute("aria-current");
  }
  const section = currentSection();
  views[id] ??= new View(section);
  if (changed) { views[id].fitWidth(); document.querySelector(".docs-content").scrollTop = 0; }
  if (step && DATA.charts[id].nodes[step]) showDetail(step);
  else if (changed) clearDetail();
}

for (const section of document.querySelectorAll(".chart")) {
  // Small screens hide the introduction and notes behind this button.
  const about = document.createElement("button");
  about.className = "btn outline small about-btn";
  about.type = "button";
  about.textContent = "About this chart";
  about.setAttribute("aria-expanded", "false");
  about.addEventListener("click", () => {
    const open = section.classList.toggle("about-open");
    about.setAttribute("aria-expanded", String(open));
    about.textContent = open ? "Back to the chart" : "About this chart";
    if (!open) requestAnimationFrame(() => views[section.dataset.chart]?.resized());
  });
  section.querySelector(".chart-head").append(about);
  const stage = section.querySelector(".stage");
  for (const g of stage.querySelectorAll(".node")) {
    g.addEventListener("click", () => showDetail(g.dataset.id));
    g.addEventListener("keydown", (e) => { if (e.key === "Enter" || e.key === " ") { e.preventDefault(); showDetail(g.dataset.id); } });
    g.addEventListener("mouseenter", () => setFocus(section, g.dataset.id));
    g.addEventListener("mouseleave", () => setFocus(section, selected));
  }
  section.querySelector(".tour-start").addEventListener("click", () => showDetail(DATA.charts[section.dataset.chart].order[0]));
}
for (const a of document.querySelectorAll(".nav-item")) {
  a.addEventListener("click", (e) => { e.preventDefault(); showChart(a.dataset.chart); history.replaceState(null, "", `#${a.dataset.chart}`); });
}

function fromHash() {
  const [id, step] = location.hash.slice(1).split("/");
  showChart(id, step);
}
window.addEventListener("hashchange", fromHash);
new ResizeObserver(() => { const v = views[chartId]; if (v) v.resized(); }).observe(document.querySelector(".docs-content"));

/* ------------------------------------------------------------ keys & theme */
window.addEventListener("keydown", (e) => {
  if (e.target.closest?.("input, textarea, select") || e.ctrlKey || e.metaKey || e.altKey) return;
  const v = views[chartId];
  if (e.key === "ArrowRight" || e.key === "j") { e.preventDefault(); stepTour(1); }
  else if (e.key === "ArrowLeft" || e.key === "k") { e.preventDefault(); stepTour(-1); }
  else if (e.key === "Escape") clearDetail();
  else if (e.key === "f" || e.key === "F") v.fit();
  else if (e.key === "+" || e.key === "=") v.zoom(1.25);
  else if (e.key === "-") v.zoom(0.8);
  else if (/^[1-9]$/.test(e.key)) { const ids = Object.keys(DATA.charts); if (ids[+e.key - 1]) showChart(ids[+e.key - 1]); }
});

const media = matchMedia("(prefers-color-scheme: dark)");
const THEMES = ["system", "light", "dark"];
function theme() { try { return localStorage.getItem("lcm-theme") || "system"; } catch { return "system"; } }
function applyTheme(t) {
  if (t === "system") delete document.documentElement.dataset.theme; else document.documentElement.dataset.theme = t;
  const dark = t === "dark" || (t === "system" && media.matches);
  try { localStorage.setItem("lcm-theme", t); localStorage.setItem("darkMode", String(dark)); } catch {}
  $("theme").innerHTML = icon(dark ? "sun" : "moon");
  $("theme").title = `Theme: ${t}`;
}
$("theme").addEventListener("click", () => applyTheme(THEMES[(THEMES.indexOf(theme()) + 1) % THEMES.length]));
media.addEventListener("change", () => { if (theme() === "system") applyTheme("system"); });
applyTheme(theme());

fromHash();
window.docs = { DATA, showChart, showDetail, views, get selected() { return selected; }, get chart() { return chartId; } };

/* A run set up from the page's link, and the small "embed" version of the page for putting a demo
 * inside another web page (an <iframe>).
 *
 *   ?algorithm=SqGathering&robots=7&seed=12&scheduler=sequential&speed=turn:1
 *   ?embed&algorithm=GoToCenter&points=-80,10;0,0;95,-5&visibility=100
 *
 * Any setting can be given by its key (docs/core-schema.md, and the list in settings.js), plus a few
 * short names: robots, seed, visibility (a radius; turns unlimited visibility off) and points
 * ("x,y;x,y;…", the robots' exact starting positions); `grid=0` hides the grid, `links=1` draws who
 * sees whom. Embed mode shows only play/pause, reset and the speed menu unless `controls` lists others
 * (play, reset, step, speed); it starts playing the first time it is seen (`autoplay=0`: not until the
 * play button), `loop=1` starts it again after it ends, and `open=0` drops the "Open in simulator" link. */
import { SETTINGS } from "./settings.js";

const ALIASES = { robots: "num_of_robots", seed: "random_seed" };
const CONTROLS = ["play", "reset", "step", "speed"];
const DEFAULT_CONTROLS = ["play", "reset", "speed"];
const KEYS = new Set(SETTINGS.map((s) => s.key));
const kindOf = Object.fromEntries(SETTINGS.map((s) => [s.key, s.kind]));

/** What the link asks for. Settings come back in the settings list's order, which is the order they
 * must be applied in (the pattern resets the arrangement under it, and so on). */
export function readLink(search) {
  const q = new URLSearchParams(search);
  const given = {};
  for (const [name, raw] of q) {
    const key = ALIASES[name] ?? name;
    if (!KEYS.has(key)) continue;
    given[key] = kindOf[key] === "switch" ? !/^(0|false|no|off)$/i.test(raw) : raw;
  }
  if (q.has("visibility")) { given.unlimited_visibility = false; given.visibility_radius = q.get("visibility"); }
  if (q.has("points")) {
    given.pattern = "custom";
    given.custom = q.get("points").split(";").map((p) => p.trim()).filter(Boolean).map((p) => p.replace(",", ", ")).join("\n");
  }
  const settings = {};
  for (const s of SETTINGS) if (s.key in given) settings[s.key] = given[s.key];

  const embed = q.has("embed") && q.get("embed") !== "0";
  const controls = (q.get("controls") ?? DEFAULT_CONTROLS.join(",")).split(",").map((c) => c.trim()).filter((c) => CONTROLS.includes(c));
  return {
    settings,
    algorithm: q.get("algorithm"),
    speed: q.get("speed"),
    theme: ["light", "dark"].includes(q.get("theme")) ? q.get("theme") : null,
    embed,
    controls: controls.length ? controls : DEFAULT_CONTROLS,
    autoplay: embed && q.get("autoplay") !== "0",
    loop: embed && q.get("loop") === "1",       // start again a moment after the run ends
    grid: q.get("grid") !== "0",
    links: q.get("links") === "1",              // who sees whom (limited visibility)
    open: q.get("open") !== "0",                // the "Open in simulator" link
  };
}

/** Embed mode: hide everything but the canvas and the chosen buttons, and link to the full page. */
export function setUpEmbed(link) {
  document.body.classList.add("embed");
  for (const c of CONTROLS) document.body.classList.toggle(`embed-${c}`, link.controls.includes(c));
  if (!link.open) return;
  const full = new URL(location.href);
  for (const k of ["embed", "controls", "autoplay", "loop", "open"]) full.searchParams.delete(k);
  const a = document.createElement("a");
  a.id = "embed_open";
  a.className = "panel";
  a.href = full.href;
  a.target = "_blank";
  a.rel = "noopener";
  a.title = "Open this case in the full simulator";
  a.innerHTML = `<span class="long">Open in simulator </span>↗`;
  document.body.append(a);
  // In a narrow frame with many buttons the link shrinks to its arrow rather than sit under the toolbar,
  // and in a very narrow one it goes (the demo itself still works).
  const toolbar = document.getElementById("toolbar");
  const clash = () => toolbar.getBoundingClientRect().right + 8 > a.getBoundingClientRect().left;
  const fit = () => {
    a.classList.remove("short", "gone");
    if (clash()) a.classList.add("short");
    if (clash()) a.classList.add("gone");
  };
  new ResizeObserver(fit).observe(toolbar);
  new ResizeObserver(fit).observe(document.body);
}

/** Calls `start` once, the first time at least half of the page is on screen. Inside an iframe the
 * browser measures this against the reader's window, so a demo further down an article waits until
 * it is scrolled to. */
export function whenFirstSeen(start) {
  if (!("IntersectionObserver" in window)) { start(); return; }
  const seen = new IntersectionObserver((entries) => {
    if (!entries.some((e) => e.isIntersecting)) return;
    seen.disconnect();
    start();
  }, { threshold: 0.5 });
  seen.observe(document.body);
}

/* ---------------------------------------------------------------- making links */
const SHORT = Object.fromEntries(Object.entries(ALIASES).map(([short, key]) => [key, short]));
const START_SHAPE = ["pattern", "arrangement", "distribution"];

/** The link's query for a run: the algorithm, every setting that differs from its default (with the
 * start's pattern, arrangement and spacing together, since choosing a pattern resets the other two),
 * the exact points of a custom start, and the playback speed. Read back by `readLink`. */
export function linkQuery({ values, algorithm, playback, points }) {
  const custom = values.pattern === "custom";
  const shapeChanged = START_SHAPE.some((k) => values[k] !== SETTINGS.find((s) => s.key === k).value);
  const parts = [["algorithm", algorithm]];
  for (const s of SETTINGS) {
    const v = values[s.key];
    if (s.key === "custom" || (custom && (START_SHAPE.includes(s.key) || s.key === "num_of_robots"))) continue;
    if (s.key === "visibility_radius" && values.unlimited_visibility) continue; // not used
    if (v === s.value && !(shapeChanged && START_SHAPE.includes(s.key))) continue;
    parts.push([SHORT[s.key] ?? s.key, s.kind === "switch" ? (v ? "1" : "0") : String(v)]);
  }
  if (custom) parts.push(["points", points.map(([x, y]) => `${x},${y}`).join(";")]);
  if (playback && playback !== "20") parts.push(["speed", playback === "Infinity" ? "max" : playback]);
  // Commas, semicolons and colons are left as they are, so the link stays readable.
  return parts.map(([k, v]) => `${k}=${encodeURIComponent(v).replace(/%2C/g, ",").replace(/%3B/g, ";").replace(/%3A/g, ":")}`).join("&");
}

/** The page's own address, without any query: where links to a case point. */
export const pageAddress = () => `${location.origin}${location.pathname}`;

/** Copies `text`; falls back to selecting it in `field` (a textarea) when the clipboard is not allowed. */
export async function copyText(text, field) {
  try { await navigator.clipboard.writeText(text); return true; } catch {}
  if (!field) return false;
  field.focus();
  field.select();
  try { return document.execCommand("copy"); } catch { return false; }
}

/** The "Embed in a web page" window: the iframe code for the current run, with the buttons, speed and
 * size to give it (the rest, such as the theme, can be added to the link by hand). `run` describes the current run:
 * `query(speed)` (its link query at that playback speed), `speeds()` ([[value, label], …] of the
 * playback menu) and `speed()` (the current one). `say(message, error)` reports a copy. */
export function buildEmbedDialog(run, say) {
  const overlay = document.createElement("div");
  overlay.id = "embed_dialog";
  overlay.className = "overlay";
  overlay.hidden = true;
  const box = (name, label, on) => `<label class="check"><input type="checkbox" data-control="${name}"${on ? " checked" : ""}> ${label}</label>`;
  overlay.innerHTML = `<div class="panel dialog wide" role="dialog" aria-modal="true" aria-labelledby="embed_title">
    <button class="btn small close" data-action="close" aria-label="Close"><span data-icon="close" data-size="15"></span></button>
    <h2 id="embed_title">Embed in a web page</h2>
    <p class="sub">Paste this into a web page to show this case there, with the current settings. It starts playing when a reader scrolls to it; after that the play button is theirs.</p>
    <div class="embed-options">
      <span class="k">Buttons</span>
      <span class="checks"><label class="check"><input type="checkbox" checked disabled> Play / pause</label>${box("reset", "Reset", true)}${box("speed", "Speed", true)}${box("step", "Step", false)}</span>
      <label class="k" for="embed_speed">Speed</label><span><select id="embed_speed"></select></span>
      <span class="k">Size</span>
      <span class="size"><label>Width <input id="embed_w" type="text" value="100%" size="5" spellcheck="false"></label><label>Height <input id="embed_h" type="text" value="420" size="5" spellcheck="false"></label>
        <span class="presets">${[["100%", "420"], ["500", "500"], ["640", "360"]].map(([w, h]) => `<button class="btn small outline" type="button" data-size="${w}x${h}">${w === "100%" ? "Full width" : `${w} × ${h}`}</button>`).join("")}</span></span>
    </div>
    <textarea id="embed_code" rows="6" readonly spellcheck="false" aria-label="Embed code"></textarea>
    <div class="dialog-foot"><a class="sub" id="embed_try" target="_blank" rel="noopener">Try it on its own ↗</a>
      <span class="buttons"><button class="btn outline" type="button" data-action="copy-link">Copy link</button><button class="btn primary" type="button" data-action="copy-code">Copy code</button></span></div>
  </div>`;
  document.body.append(overlay);
  const $ = (sel) => overlay.querySelector(sel);
  const size = (v, fallback) => (/^\d+(\.\d+)?(px|%)?$/.test(v.trim()) ? v.trim().replace(/px$/, "") : fallback);
  let src = "";
  const update = () => {
    const controls = ["play", ...[...overlay.querySelectorAll("[data-control]")].filter((c) => c.checked).map((c) => c.dataset.control)];
    const standard = controls.join(",") === DEFAULT_CONTROLS.join(",");
    src = `${pageAddress()}?embed${standard ? "" : `&controls=${controls.join(",")}`}&${run.query($("#embed_speed").value)}`;
    const w = size($("#embed_w").value, "100%"), h = size($("#embed_h").value, "420");
    $("#embed_code").value = `<iframe src="${src.replace(/&/g, "&amp;")}"\n  width="${w}" height="${h}" loading="lazy" title="Robot simulation" style="border: 1px solid #ccc; border-radius: 8px"></iframe>`;
    $("#embed_try").href = src;
  };
  const close = () => { overlay.hidden = true; };
  overlay.addEventListener("input", update);
  overlay.addEventListener("change", update);
  overlay.addEventListener("click", async (e) => {
    const preset = e.target.closest("[data-size]");
    if (preset) { [$("#embed_w").value, $("#embed_h").value] = preset.dataset.size.split("x"); update(); }
    else if (e.target === overlay || e.target.closest("[data-action=close]")) close();
    else if (e.target.closest("[data-action=copy-code]")) {
      if (await copyText($("#embed_code").value, $("#embed_code"))) say("Embed code copied: paste it into your page");
      else { $("#embed_code").select(); say("The browser did not allow copying: the code is selected, press Ctrl+C", true); }
    } else if (e.target.closest("[data-action=copy-link]")) {
      if (await copyText(src)) say("Link to the embedded demo copied");
      else say("The browser did not allow copying", true);
    }
  });
  overlay.addEventListener("keydown", (e) => { if (e.key === "Escape") { e.stopPropagation(); close(); } });
  return {
    open() {
      // The speed list and choice follow the page: per-turn speeds appear under the sequential scheduler.
      $("#embed_speed").innerHTML = run.speeds().map(([v, label]) => `<option value="${v}">${label}</option>`).join("");
      $("#embed_speed").value = run.speed();
      update();
      overlay.hidden = false;
      $("[data-action=copy-code]").focus();
    },
    get code() { return $("#embed_code").value; },
    get src() { return src; },
  };
}

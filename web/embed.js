/* A run set up from the page's link, and the small "embed" version of the page for putting a demo
 * inside another web page (an <iframe>).
 *
 *   ?algorithm=SqGathering&robots=7&seed=12&scheduler=sequential&speed=turn:1
 *   ?embed&algorithm=GoToCenter&points=-80,10;0,0;95,-5&visibility=100
 *
 * Any setting can be given by its key (docs/core-schema.md, and the list in settings.js), plus a few
 * short names: robots, seed, visibility (a radius; turns unlimited visibility off) and points
 * ("x,y;x,y;…", the robots' exact starting positions). Embed mode shows only play/pause unless
 * `controls` lists more (play, reset, step, speed); it starts playing the first time it is seen. */
import { SETTINGS } from "./settings.js";

const ALIASES = { robots: "num_of_robots", seed: "random_seed" };
const CONTROLS = ["play", "reset", "step", "speed"];
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
  const controls = (q.get("controls") ?? "play").split(",").map((c) => c.trim()).filter((c) => CONTROLS.includes(c));
  return {
    settings,
    algorithm: q.get("algorithm"),
    speed: q.get("speed"),
    theme: ["light", "dark"].includes(q.get("theme")) ? q.get("theme") : null,
    embed,
    controls: controls.length ? controls : ["play"],
    autoplay: embed && q.get("autoplay") !== "0",
  };
}

/** Embed mode: hide everything but the canvas and the chosen buttons, and link to the full page. */
export function setUpEmbed(link) {
  document.body.classList.add("embed");
  for (const c of CONTROLS) document.body.classList.toggle(`embed-${c}`, link.controls.includes(c));
  const full = new URL(location.href);
  for (const k of ["embed", "controls", "autoplay"]) full.searchParams.delete(k);
  const a = document.createElement("a");
  a.id = "embed_open";
  a.className = "panel";
  a.href = full.href;
  a.target = "_blank";
  a.rel = "noopener";
  a.textContent = "Open in simulator ↗";
  document.body.append(a);
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

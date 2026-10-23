// The simulator set up from its link, and embedded in another page, in headless Chrome:
//  - a link sets the algorithm, settings, exact starting points and speed; the page does not start by itself;
//  - in an <iframe> with ?embed only the chosen buttons show (play/pause by default), with a link
//    to the full page; it starts playing the first time it is scrolled into view, and only then;
//  - the wheel and a finger scroll the article, not the demo; only Space works as a key;
//  - a theme in the link does not change the reader's own choice; the demo fits a phone.
//   node scripts/embed-check.mjs [screenshot-dir]   (CHROME=… on Windows)
import { tmpdir } from "node:os";
import { join } from "node:path";
import { openBrowser } from "./lib/browser.mjs";

const OUT = process.argv[2] ?? join(tmpdir(), "lcm-embed");
const { base, send, js, sleep, shot, until, check, finish, consoleErrors } = await openBrowser(OUT);
await send("Page.enable");
await send("Runtime.enable");

async function navigate(url) {
  await send("Page.navigate", { url: "about:blank" });
  await sleep(50);
  await send("Page.navigate", { url });
}
const size = (width, height, mobile = false) =>
  send("Emulation.setDeviceMetricsOverride", { width, height, deviceScaleFactor: mobile ? 2 : 1, mobile });

// ── a run set up from the link (the full page) ─────────────────────────────
await size(1280, 800);
const CHAIN = "points=-80,10;0,0;95,-5&visibility=100&scheduler=sequential&turn_gap=none&robot_speeds=10";
await navigate(`${base}/?algorithm=GoToCenter&${CHAIN}&speed=turn:1&seed=3`);
await until("!!window.lcm", 10000);
await js("window.lcm.ready");
await sleep(300);
const set = await js(`(() => {
  const v = window.lcm.settings.values, f = window.lcm.renderer.frame;
  const xy = []; for (let i = 0; i < f.flags.length; i++) xy.push([f.originX + f.xy[2 * i], f.originY + f.xy[2 * i + 1]].map((n) => Math.round(n)));
  return { algorithm: document.getElementById("algorithm").value, pattern: v.pattern, unlimited: v.unlimited_visibility, radius: v.visibility_radius,
    scheduler: v.scheduler, gap: v.turn_gap, speed: v.robot_speeds, seed: v.random_seed, playback: document.getElementById("playback").value, xy };
})()`);
check("link: algorithm, exact points, visibility, scheduler, speeds and seed are taken from it",
  set.algorithm === "GoToCenter" && set.pattern === "custom" && !set.unlimited && set.radius === 100 && set.scheduler === "sequential"
  && set.gap === "none" && set.speed === 10 && set.seed === 3 && set.playback === "turn:1"
  && JSON.stringify(set.xy) === JSON.stringify([[-80, 10], [0, 0], [95, -5]]), JSON.stringify(set));
check("link: the full page keeps its panels and does not start by itself",
  (await js("window.lcm.state")) === "ready" && (await js("!document.body.classList.contains('embed') && getComputedStyle(document.getElementById('settings')).display !== 'none'")));

await navigate(`${base}/?algorithm=SqGathering&robots=5&open_world=false`);
await until("!!window.lcm", 10000);
await js("window.lcm.ready");
const sq = await js("({ scheduler: window.lcm.settings.values.scheduler, robots: window.lcm.renderer.frame.flags.length, open: window.lcm.settings.values.open_world, pattern: window.lcm.settings.values.pattern, state: window.lcm.state })");
check("link: SqGathering gets the sequential scheduler it needs; 'false' switches a setting off",
  sq.scheduler === "sequential" && sq.robots === 5 && sq.open === false && sq.pattern === "box" && sq.state === "ready", JSON.stringify(sq));
await navigate(`${base}/?algorithm=NoSuchThing&robots=banana&speed=7`);
await until("!!window.lcm", 10000);
await js("window.lcm.ready");
check("link: unknown or bad values fall back to the defaults",
  (await js("window.lcm.state")) === "ready" && (await js("window.lcm.settings.values.num_of_robots")) === 500 && (await js("document.getElementById('playback').value")) === "20");

// ── embedded in an article ─────────────────────────────────────────────────
await js("localStorage.setItem('lcm-theme', 'light')");
await size(1000, 800);
await navigate(`${base}/web/tests/embed-host.html`);
await until("document.readyState === 'complete'", 10000);
const frameOf = (id) => `document.getElementById(${JSON.stringify(id)}).contentWindow`;
const sqWin = frameOf("demo_sq"), gtcWin = frameOf("demo_gtc");
const stateOf = (w) => js(`${w}?.lcm?.state ?? "none"`);
await sleep(2500);
check("embed: a demo further down the page does not start before it is seen", (await stateOf(sqWin)) !== "running", await stateOf(sqWin));

await js("document.getElementById('demo_sq').scrollIntoView({ block: 'center' })");
const started = await until(`${sqWin}?.lcm?.state === "running"`, 15000);
check("embed: it starts playing once scrolled into view", started, await stateOf(sqWin));
await sleep(1500);
await shot("embed-sq-desktop");
const ui = await js(`(() => {
  const d = ${sqWin}.document, shown = (id) => { const e = d.getElementById(id); return !!e && getComputedStyle(e).display !== "none" && !e.hidden; };
  return { play: shown("play"), others: ["algorithm", "examples", "step_event", "step_time", "reset", "playback", "toggle_settings", "docs_link", "settings", "hud", "tools", "legend", "overview"].filter(shown),
    caption: shown("turn_caption"), open: d.getElementById("embed_open")?.href ?? "" };
})()`);
check("embed: only play/pause shows by default (and the turn caption)", ui.play && !ui.others.length && ui.caption, JSON.stringify(ui));
const open = new URL(ui.open || "http://x/");
check("embed: 'Open in simulator' opens the same case in the full page",
  open.searchParams.get("algorithm") === "SqGathering" && open.searchParams.get("robots") === "6" && !open.searchParams.has("embed"), ui.open);

await js(`${sqWin}.document.getElementById("play").click()`);
check("embed: the play button pauses it", await until(`${sqWin}.lcm.state === "paused"`, 3000));
await js("window.scrollTo(0, 0)");
await sleep(400);
await js("document.getElementById('demo_sq').scrollIntoView({ block: 'center' })");
await sleep(1200);
check("embed: scrolled away and back, it stays paused (it starts by itself only once)", (await stateOf(sqWin)) === "paused", await stateOf(sqWin));

// The wheel over the demo scrolls the article; it does not zoom the robots.
const box = await js("(() => { const r = document.getElementById('demo_sq').getBoundingClientRect(); return { x: r.left + r.width / 2, y: r.top + r.height / 2 }; })()");
const before = await js(`({ y: window.scrollY, zoom: ${sqWin}.lcm.renderer.zoom })`);
await send("Input.dispatchMouseEvent", { type: "mouseWheel", x: box.x, y: box.y, deltaX: 0, deltaY: 300 });
await sleep(500);
const after = await js(`({ y: window.scrollY, zoom: ${sqWin}.lcm.renderer.zoom })`);
check("embed: the mouse wheel over the demo scrolls the article, it does not zoom", after.y > before.y && after.zoom === before.zoom, JSON.stringify({ before, after }));
check("embed: a finger on the demo scrolls the article", (await js(`getComputedStyle(${sqWin}.document.getElementById("canvas")).touchAction`)) !== "none");
const keys = await js(`(() => {
  const w = ${sqWin}, grid = w.lcm.renderer.options.grid, press = (key) => w.dispatchEvent(new w.KeyboardEvent("keydown", { key }));
  press("g"); press("s");
  const same = w.lcm.renderer.options.grid === grid && w.document.body.classList.contains("embed");
  press(" ");
  return { same, state: w.lcm.state };
})()`);
check("embed: Space plays and pauses; the other shortcut keys do nothing", keys.same && keys.state === "running", JSON.stringify(keys));

await js("document.getElementById('demo_gtc').scrollIntoView({ block: 'center' })");
check("embed: the second demo starts when it is scrolled to", await until(`${gtcWin}?.lcm?.state === "running" || ${gtcWin}?.lcm?.state === "ended"`, 15000), await stateOf(gtcWin));
await sleep(1200);
await shot("embed-gtc-desktop");
const gtc = await js(`(() => {
  const d = ${gtcWin}.document, shown = (id) => getComputedStyle(d.getElementById(id)).display !== "none";
  return { shown: ["play", "reset", "playback", "step_event", "algorithm"].filter(shown), dark: d.documentElement.dataset.resolvedTheme, robots: ${gtcWin}.lcm.renderer.frame.flags.length };
})()`);
check("embed: controls=play,reset,speed shows those three, and the link's points and dark theme are used",
  JSON.stringify(gtc.shown) === JSON.stringify(["play", "reset", "playback"]) && gtc.dark === "dark" && gtc.robots === 3, JSON.stringify(gtc));
check("embed: the link's theme does not change the reader's own choice", (await js("localStorage.getItem('lcm-theme')")) === "light");

// One demo per algorithm: each starts when scrolled to, and runs the algorithm and swarm its link asks for.
const DEMOS = [["demo_gathering", "Gathering", 20], ["demo_sec", "SEC", 12], ["demo_circle", "CircleFormation", 10],
  ["demo_gtc", "GoToCenter", 3], ["demo_sq", "SqGathering", 6]];
await js("window.scrollTo(0, 0)");
for (const [id, algorithm, robots] of DEMOS) {
  const w = frameOf(id);
  await js(`document.getElementById(${JSON.stringify(id)}).scrollIntoView({ block: "center" })`);
  const going = await until(`["running", "paused", "ended"].includes(${w}?.lcm?.state)`, 15000);
  await sleep(4000);
  await shot(`embed-${algorithm}`);
  const got = await js(`({ algorithm: ${w}.document.getElementById("algorithm").value, robots: ${w}.lcm.renderer.frame.flags.length, time: ${w}.lcm.stats.time, state: ${w}.lcm.state })`);
  check(`embed: the ${algorithm} demo starts when scrolled to and runs ${robots} robots`,
    going && got.algorithm === algorithm && got.robots === robots && got.time > 0, JSON.stringify(got));
}

// ── on a phone ─────────────────────────────────────────────────────────────
await size(390, 844, true);
await navigate(`${base}/web/tests/embed-host.html`);
await until("document.readyState === 'complete'", 10000);
await js("document.getElementById('demo_sq').scrollIntoView({ block: 'center' })");
await until(`${sqWin}?.lcm?.state === "running"`, 15000);
await sleep(1500);
await shot("embed-sq-phone");
const fit = await js(`(() => {
  const w = ${sqWin}, d = w.document, inside = (id) => { const r = d.getElementById(id).getBoundingClientRect(); return r.left >= 0 && r.top >= 0 && r.right <= w.innerWidth && r.bottom <= w.innerHeight; };
  const t = d.getElementById("toolbar").getBoundingClientRect(), o = d.getElementById("embed_open").getBoundingClientRect();
  return { sideways: d.documentElement.scrollWidth > w.innerWidth, toolbar: inside("toolbar"), open: inside("embed_open"), apart: t.right <= o.left, width: w.innerWidth };
})()`);
check("embed on a phone: fits, with no sideways scrolling and nothing overlapping", !fit.sideways && fit.toolbar && fit.open && fit.apart, JSON.stringify(fit));

check("no console errors", consoleErrors.length === 0, consoleErrors.join(" | "));
await finish();

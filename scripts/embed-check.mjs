// The simulator set up from its link, and embedded in another page, in headless Chrome:
//  - a link sets the algorithm, settings, exact starting points and speed; the page does not start by itself;
//  - in an <iframe> with ?embed only the chosen buttons show (play/pause, reset and speed by default), with a link
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
// A test article: text, then one embedded demo per algorithm further down (as on a lecture page).
// It is written into an empty page of the test server, so the demos are on the same origin and can be inspected.
const DEMO_LINKS = {
  demo_gathering: "algorithm=Gathering&robots=20&seed=1",
  demo_sec: "algorithm=SEC&robots=12&seed=2",
  demo_circle: "algorithm=CircleFormation&robots=10&seed=3",
  demo_gtc: "controls=play,reset,step,speed&algorithm=GoToCenter&points=-80,10;0,0;95,-5&visibility=100&scheduler=sequential&turn_gap=none&robot_speeds=10&speed=turn:1&theme=dark",
  demo_sq: "algorithm=SqGathering&robots=6&seed=7&speed=turn:1",
};
const ARTICLE = `<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<style>body { font: 16px/1.6 system-ui, sans-serif; max-width: 760px; margin: 0 auto; padding: 24px 16px 400px; background: #fff; }
.tall { height: 1400px; } iframe { display: block; width: 100%; height: 420px; border: 1px solid #ccc; border-radius: 8px; margin: 24px 0; }</style></head>
<body><h1>Article</h1><p class="tall">Text of the article…</p>
${Object.entries(DEMO_LINKS).map(([id, q]) => `<iframe id="${id}" title="${id}" loading="lazy" src="${base}/?embed&${q}"></iframe>`).join("\n")}
</body></html>`;
/** An empty page of the test server (same origin as the simulator), with `html` written into it. */
async function openBlank(html = "") {
  await navigate(`${base}/__blank`);
  await until("document.readyState === 'complete'", 10000);
  if (html) await js(`document.open(); document.write(${JSON.stringify(html)}); document.close(); true`);
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

// ── links made by the page ─────────────────────────────────────────────────
// "Copy link to this case" gives a link that opens the same case: every setting, the algorithm, the
// exact custom points and the playback speed come back as they were.
const SETUPS = [
  ["the defaults", null, {}, null],
  ["the original world box", null, { open_world: false }, null],
  ["a rotated line", null, { pattern: "line", num_of_robots: 30, rotation: 45, size: 800 }, "5"],
  ["a ring of robots", "SEC", { pattern: "circle", arrangement: "ring", inner: 0.4, num_of_robots: 40 }, null],
  ["custom points, limited sight, one at a time", "GoToCenter",
    { pattern: "custom", custom: "-80, 10\n0, 0\n95.5, -5.25", unlimited_visibility: false, visibility_radius: 100, scheduler: "sequential", turn_gap: "none" }, "turn:1"],
  ["faults, non-rigid moves, a turn limit", "Gathering",
    { num_of_faults: 5, fault_type: "byzantine", rigid_movement: false, delta: 5, stop_policy: "fraction", stop_fraction: 0.3,
      scheduler: "sequential", activation_order: "random", max_turns: 100, threshold_precision: 8, random_seed: 99 }, "Infinity"],
  ["SqGathering with stacked points", "SqGathering", { multiplicities: 2, num_of_robots: 12 }, "turn:3"],
];
const snapshot = () => js(`(() => ({ values: { ...window.lcm.settings.values, custom: undefined }, points: window.lcm.settings.customPoints,
  algorithm: document.getElementById("algorithm").value, playback: document.getElementById("playback").value }))()`);
for (const [name, algorithm, values, playback] of SETUPS) {
  await navigate(`${base}/`);
  await until("!!window.lcm", 10000);
  await js("window.lcm.ready");
  if (algorithm) await js(`(() => { const s = document.getElementById("algorithm"); s.value = ${JSON.stringify(algorithm)}; s.dispatchEvent(new Event("change")); })()`);
  await js(`window.lcm.settings.apply(${JSON.stringify(values)})`);
  await sleep(300);
  if (playback) await js(`(() => { const s = document.getElementById("playback"); s.value = ${JSON.stringify(playback)}; s.dispatchEvent(new Event("change")); })()`);
  const want = await snapshot();
  const link = await js("window.lcm.caseLink()");
  await navigate(link);
  await until("!!window.lcm", 10000);
  await js("window.lcm.ready");
  const got = await snapshot();
  const diff = Object.keys(want.values).filter((k) => JSON.stringify(want.values[k]) !== JSON.stringify(got.values[k]));
  if (want.values.pattern === "custom" && JSON.stringify(want.points) !== JSON.stringify(got.points)) diff.push("points");
  if (want.algorithm !== got.algorithm) diff.push("algorithm");
  if (want.playback !== got.playback) diff.push("playback");
  check(`copied link, ${name}: opens the same case`, !diff.length && got.algorithm !== "",
    diff.length ? JSON.stringify({ link, diff: diff.map((k) => [k, want.values[k] ?? want[k], got.values[k] ?? got[k]]) }) : link.replace(base, ""));
}

// The address bar follows the case: change the speed and a setting, and it says so; reloading keeps them.
await navigate(`${base}/?algorithm=Gathering&robots=20&seed=1&speed=5`);
await until("!!window.lcm", 10000);
await js("window.lcm.ready");
await js(`(() => { const s = document.getElementById("playback"); s.value = "100"; s.dispatchEvent(new Event("change")); })()`);
await js(`window.lcm.settings.set("num_of_robots", 35)`);
const followed = await until("location.search === '?algorithm=Gathering&robots=35&seed=1&speed=100'", 3000);
check("address bar: follows changes to the speed and the settings", followed, await js("location.search"));
await send("Page.reload");
await sleep(300);
await until("!!window.lcm", 10000);
await js("window.lcm.ready");
check("address bar: reloading the page keeps the case",
  (await js("document.getElementById('playback').value")) === "100" && (await js("window.lcm.settings.values.num_of_robots")) === 35 && (await js("window.lcm.renderer.frame.flags.length")) === 35);
await js(`window.lcm.settings.reset()`);
await js(`(() => { const s = document.getElementById("playback"); s.value = "20"; s.dispatchEvent(new Event("change")); })()`);
check("address bar: back to the defaults, it is the plain address again", await until("location.search === ''", 3000), await js("location.search"));

// "Embed in a web page…": the iframe code for the current case, with the buttons and size chosen.
await navigate(`${base}/?algorithm=GoToCenter&${CHAIN}&speed=turn:1`);
await until("!!window.lcm", 10000);
await js("window.lcm.ready");
await js("document.getElementById('export').click()");
await js("document.querySelector('[data-export=embed]').click()");
await sleep(200);
const dialog = await js(`({ open: !document.getElementById("embed_dialog").hidden, code: window.lcm.embedDialog.code, src: window.lcm.embedDialog.src })`);
const src = new URL(dialog.src);
check("embed window: opens from the Export menu with iframe code for this case",
  dialog.open && dialog.code.startsWith("<iframe src=\"") && dialog.code.includes('width="100%" height="420"')
  && src.searchParams.has("embed") && !src.searchParams.has("controls") && src.searchParams.get("algorithm") === "GoToCenter"
  && src.searchParams.get("points") === "-80,10;0,0;95,-5" && src.searchParams.get("speed") === "turn:1", dialog.code);
await shot("embed-dialog");
await js(`(() => { const d = document.getElementById("embed_dialog");
  d.querySelector("[data-control=step]").click(); d.querySelector("[data-control=speed]").click();
  const w = d.querySelector("#embed_w"), h = d.querySelector("#embed_h");
  w.value = "500"; h.value = "500px"; w.dispatchEvent(new Event("input", { bubbles: true })); })()`);
const changed = await js("({ code: window.lcm.embedDialog.code, src: window.lcm.embedDialog.src })");
check("embed window: the buttons and size chosen go into the code",
  new URL(changed.src).searchParams.get("controls") === "play,reset,step" && changed.code.includes('width="500" height="500"'), changed.code);
await send("Browser.grantPermissions", { origin: base, permissions: ["clipboardReadWrite", "clipboardSanitizedWrite"] });
await send("Emulation.setFocusEmulationEnabled", { enabled: true });
await js("document.querySelector('#embed_dialog [data-action=copy-code]').click()");
const said = await until("/copied/i.test(document.getElementById('toast').textContent) && !document.getElementById('toast').hidden", 3000);
check("embed window: Copy code copies the code and says so", said && (await js("navigator.clipboard.readText()")) === changed.code,
  await js("document.getElementById('toast').textContent"));
await js(`document.getElementById("embed_dialog").dispatchEvent(new KeyboardEvent("keydown", { key: "Escape", bubbles: true }))`);
check("embed window: Esc closes it", await js("document.getElementById('embed_dialog').hidden"));
// The code works: in an iframe it shows the same robots, with the buttons chosen.
const frameSrc = changed.code.match(/src="([^"]+)"/)[1].replace(/&amp;/g, "&");
await js(`document.body.insertAdjacentHTML("beforeend", '<iframe id="made" style="position:fixed;left:0;top:0;width:500px;height:500px;z-index:99" src="${frameSrc}"></iframe>')`);
const made = `document.getElementById("made").contentWindow`;
await until(`${made}?.lcm?.renderer?.frame`, 15000);
const madeState = await js(`(() => { const w = ${made}, d = w.document, f = w.lcm.renderer.frame, shown = (id) => getComputedStyle(d.getElementById(id)).display !== "none";
  const xy = []; for (let i = 0; i < f.flags.length; i++) xy.push([Math.round(f.originX + f.xy[2 * i]), Math.round(f.originY + f.xy[2 * i + 1])]);
  return { algorithm: d.getElementById("algorithm").value, xy, buttons: ["play", "reset", "step_event", "playback"].filter(shown), embed: d.body.classList.contains("embed") }; })()`);
check("embed window: the code works (same robots, chosen buttons)", madeState.embed && madeState.algorithm === "GoToCenter"
  && JSON.stringify(madeState.xy) === JSON.stringify([[-80, 10], [0, 0], [95, -5]]) && JSON.stringify(madeState.buttons) === JSON.stringify(["play", "reset", "step_event"]), JSON.stringify(madeState));
await js(`document.getElementById("made").remove()`);

// The window's speed menu and size presets.
await js("document.getElementById('export').click()");
await js("document.querySelector('[data-export=embed]').click()");
await sleep(200);
const speeds = await js(`[...document.getElementById("embed_speed").options].map((o) => o.value)`);
check("embed window: its speed menu lists the page's speeds, per turn too, and starts at the current one",
  speeds.includes("turn:3") && speeds.includes("Infinity") && (await js(`document.getElementById("embed_speed").value`)) === "turn:1", JSON.stringify(speeds));
await js(`(() => { const d = document.getElementById("embed_dialog"), e = d.querySelector("#embed_speed");
  e.value = "turn:3"; e.dispatchEvent(new Event("change", { bubbles: true })); d.querySelector("[data-size='500x500']").click(); })()`);
const all = await js("({ code: window.lcm.embedDialog.code, src: window.lcm.embedDialog.src })");
const p = new URL(all.src).searchParams;
check("embed window: the speed and a size preset go into the code (and only the case: no theme or other options)",
  p.get("speed") === "turn:3" && all.code.includes('width="500" height="500"') && all.code.includes("border: 1px solid #ccc")
  && !["theme", "autoplay", "loop", "grid", "links", "open"].some((k) => p.has(k)), all.code);
await shot("embed-dialog-options");
await js(`document.getElementById("embed_dialog").dispatchEvent(new KeyboardEvent("keydown", { key: "Escape", bubbles: true }))`);
// Options that can be added to the link by hand still work: theme, no autoplay, no grid, who sees whom, no link.
await js(`document.body.insertAdjacentHTML("beforeend", '<iframe id="made2" style="position:fixed;left:0;top:0;width:500px;height:500px;z-index:99" src="${all.src}&theme=dark&autoplay=0&grid=0&links=1&open=0"></iframe>')`);
const made2 = `document.getElementById("made2").contentWindow`;
await until(`${made2}?.lcm?.renderer?.frame`, 15000);
await sleep(2000);
const opts = await js(`(() => { const w = ${made2}, d = w.document;
  return { state: w.lcm.state, grid: w.lcm.renderer.options.grid, links: w.lcm.renderer.options.links, open: !!d.getElementById("embed_open"),
    dark: d.documentElement.dataset.resolvedTheme, speed: d.getElementById("playback").value }; })()`);
check("embed: options added to the link by hand work (dark, waits for play, no grid, who sees whom, no link, 3 s/turn)",
  opts.state === "ready" && !opts.grid && opts.links && !opts.open && opts.dark === "dark" && opts.speed === "turn:3", JSON.stringify(opts));
await js(`document.getElementById("made2").remove()`);

// "Starts again after a moment": a looping demo plays to its end, then from the beginning again.
await js(`document.body.insertAdjacentHTML("beforeend", '<iframe id="loop" style="position:fixed;left:0;top:0;width:400px;height:300px;z-index:99" src="${base}/?embed&loop=1&algorithm=GoToCenter&${CHAIN}&speed=max"></iframe>')`);
const loopWin = `document.getElementById("loop").contentWindow`;
const loopEnded = await until(`${loopWin}?.lcm?.state === "ended"`, 20000);
// (At Max this run is over in moments, so look for frames of a new run rather than for "running".)
await sleep(300);
const framesAtEnd = await js(`${loopWin}.lcm.frames`);
const loopAgain = loopEnded && await until(`${loopWin}.lcm.frames > ${framesAtEnd} + 1`, 6000);
check("embed: with loop=1 it starts again after it ends", loopEnded && loopAgain, JSON.stringify({ framesAtEnd, now: await js(`${loopWin}.lcm.frames`), state: await js(`${loopWin}.lcm.state`) }));
await js(`document.getElementById("loop").remove()`);

// ── embedded in an article ─────────────────────────────────────────────────
await js("localStorage.setItem('lcm-theme', 'light')");
await size(1000, 800);
await openBlank(ARTICLE);
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
  return { play: shown("play") && shown("reset") && shown("playback"), others: ["algorithm", "examples", "step_event", "step_time", "toggle_settings", "docs_link", "settings", "hud", "tools", "legend", "overview"].filter(shown),
    caption: shown("turn_caption"), open: d.getElementById("embed_open")?.href ?? "" };
})()`);
check("embed: only play/pause, reset and the speed menu show by default (and the turn caption)", ui.play && !ui.others.length && ui.caption, JSON.stringify(ui));
const open = new URL(ui.open || "http://x/");
check("embed: the demo leaves its own address alone (it is not the reader's page)", (await js(`${sqWin}.location.search`)) === "?embed&algorithm=SqGathering&robots=6&seed=7&speed=turn:1", await js(`${sqWin}.location.search`));
check("embed: 'Open in simulator' opens the same case in the full page",
  open.searchParams.get("algorithm") === "SqGathering" && open.searchParams.get("robots") === "6" && !open.searchParams.has("embed"), ui.open);

await js(`${sqWin}.document.getElementById("reset").click()`);
check("embed: reset starts the case again from the beginning", await until(`${sqWin}.lcm.state === "ready" && ${sqWin}.lcm.stats.time === 0`, 3000), await stateOf(sqWin));
await js(`${sqWin}.document.getElementById("play").click()`);
check("embed: the play button plays it", await until(`${sqWin}.lcm.state === "running"`, 3000));
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
  return { shown: ["play", "step_event", "step_time", "reset", "playback", "algorithm"].filter(shown), dark: d.documentElement.dataset.resolvedTheme, robots: ${gtcWin}.lcm.renderer.frame.flags.length };
})()`);
check("embed: controls=play,reset,step,speed shows all four, and the link's points and dark theme are used",
  JSON.stringify(gtc.shown) === JSON.stringify(["play", "step_event", "step_time", "reset", "playback"]) && gtc.dark === "dark" && gtc.robots === 3, JSON.stringify(gtc));
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
  const got = await js(`({ algorithm: ${w}.document.getElementById("algorithm").value, playback: ${w}.document.getElementById("playback").value, robots: ${w}.lcm.renderer.frame.flags.length, time: ${w}.lcm.stats.time, state: ${w}.lcm.state })`);
  check(`embed: the ${algorithm} demo starts when scrolled to and runs ${robots} robots (at 20× unless its link says)`,
    going && got.algorithm === algorithm && got.robots === robots && got.time > 0 && got.playback === (algorithm === "GoToCenter" || algorithm === "SqGathering" ? "turn:1" : "20"), JSON.stringify(got));
}

// Played to the end (fast), every demo still shows all its robots and circles, around the middle of its box.
for (const [id, algorithm] of DEMOS) {
  const w = frameOf(id);
  await js(`document.getElementById(${JSON.stringify(id)}).scrollIntoView({ block: "center" })`);
  await js(`(() => { const d = ${w}.document, s = d.getElementById("playback"); s.value = "Infinity"; s.dispatchEvent(new Event("change"));
    if (${w}.lcm.state !== "running" && ${w}.lcm.state !== "ended") d.getElementById("play").click(); })()`);
  const ended = await until(`${w}.lcm.state === "ended"`, 60000);
  await sleep(2000); // the camera eases into place
  await shot(`embed-end-${algorithm}`);
  const view = await js(`(() => {
    const r = ${w}.lcm.renderer, f = r.frame, v = r.viewRect(), n = f.flags.length;
    let out = 0, sx = 0, sy = 0;
    const inside = (x, y, pad = 0) => x - pad >= v.x0 && x + pad <= v.x1 && y - pad >= v.y0 && y + pad <= v.y1;
    for (let i = 0; i < n; i++) {
      const x = f.originX + f.xy[2 * i], y = f.originY + f.xy[2 * i + 1];
      sx += x; sy += y;
      if (!inside(x, y)) out++;
      const cr = f.circle?.[3 * i + 2];
      if (cr >= 0 && !inside(f.originX + f.circle[3 * i], f.originY + f.circle[3 * i + 1], cr * 0.98)) out++;
    }
    const w = v.x1 - v.x0, h = v.y1 - v.y0;
    return { out, offX: Math.abs(sx / n - (v.x0 + v.x1) / 2) / w, offY: Math.abs(sy / n - (v.y0 + v.y1) / 2) / h };
  })()`);
  check(`embed: the ${algorithm} demo, played to the end, keeps every robot and circle in its box, near the middle`,
    ended && view.out === 0 && view.offX < 0.15 && view.offY < 0.15, JSON.stringify({ ended, ...view }));
}

// ── boxes of different sizes ───────────────────────────────────────────────
// Tiny, short and wide, tall and narrow: the buttons fit, the full-page link never sits under them (it
// shrinks to its arrow, or goes), the caption fits, and every robot is in view while playing and at the end.
await size(1400, 1000);
const SIZE_DEMOS = {
  "Go-To-Center with every button": "controls=play,reset,step,speed&algorithm=GoToCenter&points=-80,10;0,0;95,-5&visibility=100&scheduler=sequential&turn_gap=none&robot_speeds=10&speed=turn:0.25",
  SqGathering: "algorithm=SqGathering&robots=6&seed=7&speed=turn:0.25",
  SEC: "algorithm=SEC&robots=12&seed=2",
};
for (const [name, query] of Object.entries(SIZE_DEMOS)) for (const [bw, bh] of [[240, 180], [320, 240], [500, 500], [900, 220], [280, 560]]) {
  await openBlank("<!doctype html><body style='margin:8px'></body>");
  await js(`document.body.innerHTML = '<iframe id="box" style="width:${bw}px;height:${bh}px" src="${base}/?embed&${query}"></iframe>'`);
  const win = frameOf("box");
  const going = await until(`["running", "ended"].includes(${win}?.lcm?.state)`, 15000);
  await sleep(700);
  const layout = () => js(`(() => {
    const w = ${win}, d = w.document, r = (id) => d.getElementById(id).getBoundingClientRect();
    const inside = (b) => b.left >= -0.5 && b.top >= -0.5 && b.right <= w.innerWidth + 0.5 && b.bottom <= w.innerHeight + 0.5;
    const hit = (a, b) => a.left < b.right && b.left < a.right && a.top < b.bottom && b.top < a.bottom;
    const t = r("toolbar"), o = r("embed_open"), c = r("turn_caption"), link = getComputedStyle(d.getElementById("embed_open")).visibility !== "hidden";
    const R = w.lcm.renderer, f = R.frame, v = R.viewRect();
    let out = 0;
    for (let i = 0; i < f.flags.length; i++) {
      const x = f.originX + f.xy[2 * i], y = f.originY + f.xy[2 * i + 1];
      if (x < v.x0 || x > v.x1 || y < v.y0 || y > v.y1) out++;
    }
    const problems = [];
    if (!inside(t)) problems.push("buttons cut off");
    if (link && (!inside(o) || hit(t, o))) problems.push("link under the buttons");
    if (!d.getElementById("turn_caption").hidden && (!inside(c) || hit(c, t))) problems.push("caption");
    if (d.documentElement.scrollWidth > w.innerWidth) problems.push("sideways scrolling");
    if (out) problems.push(out + " robots out of view");
    return problems;
  })()`);
  const playing = await layout();
  await js(`(() => { const s = ${win}.document.getElementById("playback"); s.value = "Infinity"; s.dispatchEvent(new Event("change")); })()`);
  const ended = await until(`${win}.lcm.state === "ended"`, 60000);
  await sleep(1500);
  const end = await layout();
  await shot(`embed-size-${name.split(" ")[0]}-${bw}x${bh}`);
  check(`embed, ${name}, in a ${bw}×${bh} box: everything fits and stays in view, playing and at the end`,
    going && ended && !playing.length && !end.length, JSON.stringify({ playing, end }));
}

// ── on a phone ─────────────────────────────────────────────────────────────
await size(390, 844, true);
await openBlank(ARTICLE);
const fitOf = (win) => js(`(() => {
  const w = ${win}, d = w.document, inside = (id) => { const r = d.getElementById(id).getBoundingClientRect(); return r.left >= 0 && r.top >= 0 && r.right <= w.innerWidth && r.bottom <= w.innerHeight; };
  const t = d.getElementById("toolbar").getBoundingClientRect(), o = d.getElementById("embed_open").getBoundingClientRect();
  return { sideways: d.documentElement.scrollWidth > w.innerWidth, toolbar: inside("toolbar"), open: inside("embed_open"), apart: t.right <= o.left, width: w.innerWidth };
})()`);
for (const [id, win] of [["demo_sq", sqWin], ["demo_gtc", gtcWin]]) {
  await js(`document.getElementById("${id}").scrollIntoView({ block: "center" })`);
  await until(`${win}?.lcm?.state === "running"`, 15000);
  await sleep(800);
  await shot(`embed-phone-${id}`);
  const fit = await fitOf(win);
  check(`embed on a phone (${id}): fits, with no sideways scrolling and nothing overlapping`, !fit.sideways && fit.toolbar && fit.open && fit.apart, JSON.stringify(fit));
}

check("no console errors", consoleErrors.length === 0, consoleErrors.join(" | "));
await finish();

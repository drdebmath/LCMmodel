// End-to-end check of the simulator page (index.html at the site root) in headless Chrome,
// driven through the DevTools protocol: real clicks, keys and screenshots.
//
//   node scripts/ui-check.mjs [screenshot-dir]
//
// Exits 1 if any check fails (see scripts/lib/browser.mjs for requirements).
import { tmpdir } from "node:os";
import { join } from "node:path";
import { openBrowser } from "./lib/browser.mjs";

const { base, send, js, sleep, shot, key, clickAt, click, until, check, finish, consoleErrors } =
  await openBrowser(process.argv[2] ?? join(tmpdir(), "lcm-ui"));

await send("Page.enable");
await send("Runtime.enable");
await send("Emulation.setDeviceMetricsOverride", { width: 1440, height: 900, deviceScaleFactor: 1, mobile: false });
await send("Emulation.setEmulatedMedia", { features: [{ name: "prefers-color-scheme", value: "light" }] });

const t0 = Date.now();
await send("Page.navigate", { url: `${base}/` });
await until("!!window.lcm", 10000);
await js("window.lcm.ready");
check("page loads and shows the first swarm", true, `${Date.now() - t0} ms to first frame`);
check("starts in the ready state", (await js("lcm.state")) === "ready");
const build = await js("fetch('web/pkg/build.json', { cache: 'no-store' }).then((r) => r.json())");
const loaded = await js("lcm.client.loadedVersion");
check("the core is loaded with this build's id, so a stale cache cannot mix versions", !!build.id && loaded === build.id, `build ${build.id}, worker loaded ${loaded}`);
await sleep(300);
await shot("1-ready-light");
const legend = await js("document.getElementById('legend').textContent");
check("legend lists only colours present, with counts", /Active\s*500/.test(legend) && !/Moving/.test(legend), legend.trim());

// "How it works" lives in the toolbar; fit and full screen are separate buttons.
const docsLink = await js(`(() => { const a = document.getElementById("docs_link"); const r = a.getBoundingClientRect();
  return { inToolbar: !!a.closest("#toolbar"), label: a.textContent.trim(), top: Math.round(r.top) }; })()`);
check("\"How it works\" is a labelled button in the top toolbar", docsLink.inToolbar && docsLink.label === "How it works" && docsLink.top < 40, JSON.stringify(docsLink));
check("the bottom tools no longer hold the flowchart link", !(await js("!!document.querySelector('#tools a[href=\"docs.html\"]')")));
const fsBox = await js(`(() => { const r = document.getElementById("fullscreen").getBoundingClientRect(); return { x: r.x + r.width / 2, y: r.y + r.height / 2 }; })()`);
await clickAt(fsBox.x, fsBox.y);
check("the full-screen button enters full screen", await until("!!document.fullscreenElement", 3000));
check("the button shows it can exit", await until("document.getElementById('fullscreen').getAttribute('aria-pressed') === 'true'", 2000));
await key("F", "KeyF", 70, true);
check("Shift + F leaves full screen", await until("!document.fullscreenElement", 3000));
await js("lcm.renderer.zoomAt(2.5)");
await clickAt(...Object.values(await js(`(() => { const r = document.getElementById("fit").getBoundingClientRect(); return { x: r.x + r.width / 2, y: r.y + r.height / 2 }; })()`)));
check("the fit button brings a zoomed view back to the whole world", (await js("lcm.renderer.zoom")) === 1);
await js("document.activeElement.blur()"); // a focused button takes Space itself, as browsers do

// Stepping.
await click("#step_event");
await until("!document.getElementById('hud_last').hidden");
const last1 = await js("document.getElementById('hud_last').textContent");
check("step one event shows what happened", /t 0\.\d+ · (robot \d+|Visualize)/.test(last1), last1);
check("stepping pauses", (await js("lcm.state")) === "paused");
const tBefore = await js("lcm.renderer.frame.time");
await click("#step_time");
await until(`lcm.renderer.frame.time > ${tBefore} + 0.5`);
const tAfter = await js("lcm.renderer.frame.time");
check("step one time unit advances time by up to 1", tAfter > tBefore && tAfter <= tBefore + 1, `${tBefore.toFixed(3)} → ${tAfter.toFixed(3)}`);
await key("ArrowRight", "ArrowRight", 39);
await sleep(200);
check("→ key steps one event", /robot \d+|Visualize/.test(await js("document.getElementById('hud_last').textContent")));

// Keyboard play / pause.
await key(" ", "Space", 32);
await sleep(600);
check("Space plays", (await js("lcm.state")) === "running");
await key(" ", "Space", 32);
await sleep(200);
const pausedAt = await js("lcm.renderer.frame.time");
await sleep(500);
check("Space pauses and time stays put", (await js("lcm.state")) === "paused" && (await js("lcm.renderer.frame.time")) === pausedAt);

// Inspector: click the robot the renderer draws nearest the middle.
const target = await js(`(() => { const r = lcm.renderer, f = r.frame; let best = -1, bd = 1e18;
  for (let i = 0; i < f.flags.length; i++) { const [x, y] = r.toScreen(f.xy[2*i], f.xy[2*i+1]);
    const d = (x - r.width/2) ** 2 + (y - r.height/2) ** 2; if (d < bd) { bd = d; best = i; } }
  const [x, y] = r.toScreen(f.xy[2*best], f.xy[2*best+1]); const b = r.canvas.getBoundingClientRect();
  return { i: best, x: x + b.left, y: y + b.top }; })()`);
await clickAt(target.x, target.y);
await until("!document.getElementById('inspector').hidden && /Gathering/.test(document.getElementById('inspector').textContent)");
const card = await js("document.getElementById('inspector').innerText");
check("clicking a robot opens the inspector with its details", card.includes(`Robot ${target.i}`) && /Position/.test(card) && /Gathering/.test(card), card.replace(/\n+/g, " | "));
check("the selected robot is highlighted", (await js("lcm.renderer.selected")) === target.i);
await shot("2-inspector-light");
await key("Escape", "Escape", 27);
check("Esc closes the inspector", await until("document.getElementById('inspector').hidden", 1000));

// Settings: live preview when ready, "Reset to apply" once started.
await click("#reset");
await until("lcm.state === 'ready' && lcm.renderer.frame && lcm.renderer.frame.time === 0");
await js("lcm.settings.set('num_of_robots', 120)");
check("changing a setting while ready rebuilds the swarm", await until("lcm.renderer.frame && lcm.renderer.frame.robots === 120", 3000));
await click("#play");
await sleep(400);
await js("lcm.settings.set('num_of_robots', 80)");
check("changing a setting while running asks to reset", !(await js("document.getElementById('dirty').hidden")) && (await js("lcm.renderer.frame.robots")) === 120);
await key("r", "KeyR", 82);
check("R resets with the new settings", await until("lcm.state === 'ready' && lcm.renderer.frame.robots === 80 && document.getElementById('dirty').hidden", 3000));
await js("lcm.settings.set('num_of_faults', 12); lcm.settings.set('fault_type', 'mixed'); lcm.settings.set('unlimited_visibility', false); lcm.settings.set('visibility_radius', 160)");
await until("lcm.renderer.frame && lcm.renderer.options.visibility === 160", 3000);
await click("#lights");
await click("#play");
await sleep(1500);
await shot("3-faults-visibility-lights");
const legend2 = await js("document.getElementById('legend').textContent");
check("fault colours appear in the legend", /Byzantine/.test(legend2) && /Crashed/.test(legend2), legend2.trim());
await click("#lights");

// Start settings: open world by default, the World toggle, pattern / arrangement / distribution, custom positions.
// Earlier checks changed robots, faults and visibility: start from the defaults.
await js("lcm.settings.reset()");
await click("#reset");
await until("lcm.state === 'ready'");
const startDefaults = await js(`({ open: lcm.settings.values.open_world, pattern: lcm.settings.values.pattern,
  outline: lcm.renderer.outline, widthHidden: document.querySelector('[data-key="width_bound"]').hidden })`);
check("opens in an open world: free cloud, no box, no world size", startDefaults.open && startDefaults.pattern === "cloud" && startDefaults.outline === null && startDefaults.widthHidden,
  JSON.stringify(startDefaults));
const onScreen = (await js(`(() => { const r = lcm.renderer, f = r.frame; let out = 0;
  for (let i = 0; i < f.flags.length; i++) { const [x, y] = r.toScreen(f.xy[2*i], f.xy[2*i+1]); if (x < 0 || y < 0 || x > r.width || y > r.height) out++; } return out; })()`));
check("the view fits the whole cloud", onScreen === 0, `${onScreen} robots off screen`);
const visibleStart = () => js(`[...document.querySelectorAll('[data-group="start"] .field')].filter((f) => f.offsetParent && !f.closest('.more-body')).map((f) => f.dataset.key)`);
const cloudFields = await visibleStart();
check("the cloud start shows only the world switch, pattern and spread", JSON.stringify(cloudFields) === JSON.stringify(["open_world", "pattern", "spread"]) &&
  await js("document.querySelector('[data-more=\"start\"]').hidden"), cloudFields.join(", "));
check("no '?' icons; help is on the labels", await js("document.querySelectorAll('#settings .hint').length === 0 && document.querySelectorAll('#settings label.has-hint').length > 5"));
await shot("8-start-cloud");
await js("lcm.settings.set('open_world', false)");
await until("lcm.renderer.outline && lcm.renderer.outline.kind === 'rect'", 3000);
await sleep(300);
const world = await js(`(() => { const f = lcm.renderer.frame; let inBox = true;
  for (let i = 0; i < f.flags.length; i++) if (Math.abs(f.xy[2*i]) > 300.001 || Math.abs(f.xy[2*i+1]) > 300.001) inBox = false;
  return { pattern: lcm.settings.values.pattern, inBox, widthShown: !document.querySelector('[data-key="width_bound"]').hidden }; })()`);
check("World mode: the original box start, with its size settings", world.pattern === "box" && world.inBox && world.widthShown, JSON.stringify(world));
await js("lcm.settings.set('open_world', true); lcm.settings.set('pattern', 'circle'); lcm.settings.set('arrangement', 'edge'); lcm.settings.set('size', 400)");
check("a circle: robots evenly on a circle of radius 200", await until(`(() => { const f = lcm.renderer.frame;
  return f.flags.length === 500 && [...Array(500).keys()].every((i) => Math.abs(Math.hypot(f.xy[2*i], f.xy[2*i+1]) - 200) < 0.01); })()`, 3000));
await sleep(200);
await shot("9-start-ring");
check("a new start clears the hover tooltip of the old run", await js("document.getElementById('tip').hidden"));
const circleFields = await visibleStart();
const circleSummary = await js("document.getElementById('start-summary').textContent");
check("a circle's edge: pattern, arrangement, distribution and diameter, nothing else",
  JSON.stringify(circleFields) === JSON.stringify(["open_world", "pattern", "arrangement", "distribution", "size"]) && await js("lcm.settings.values.distribution === 'even'") &&
  await js("document.querySelector('[data-key=\"size\"] label').textContent === 'Diameter' && document.querySelector('[data-more=\"start\"]').hidden"),
  circleFields.join(", "));
check("one sentence says what the start gives", circleSummary === "500 robots evenly spaced on the edge of a circle 400 wide.", circleSummary);
const arrangements = await js("[...document.querySelectorAll('#set-arrangement option')].map((o) => o.value).join(',')");
check("a circle can be filled, edge only or a ring", arrangements === "filled,edge,ring", arrangements);
// The three combinations added with the three-setting layout.
await js("lcm.settings.set('pattern', 'square'); lcm.settings.set('arrangement', 'edge'); lcm.settings.set('size', 400)");
check("square, edge only: every robot on the square's outline", await until(`(() => { const f = lcm.renderer.frame; if (f.flags.length !== 500) return false;
  for (let i = 0; i < 500; i++) { const x = Math.abs(f.xy[2*i]), y = Math.abs(f.xy[2*i+1]); if (Math.max(x, y) > 200.01 || Math.max(x, y) < 199.99) return false; } return true; })()`, 3000));
await js("lcm.settings.set('pattern', 'rectangle'); lcm.settings.set('arrangement', 'edge'); lcm.settings.set('size', 400); lcm.settings.set('aspect', 0.5)");
check("rectangle, edge only: every robot on the rectangle's outline", await until(`(() => { const f = lcm.renderer.frame; if (f.flags.length !== 500) return false;
  for (let i = 0; i < 500; i++) { const x = Math.abs(f.xy[2*i]) / 200, y = Math.abs(f.xy[2*i+1]) / 100; if (Math.max(x, y) > 1.0001 || Math.max(x, y) < 0.9999) return false; } return true; })()`, 3000));
await js("lcm.settings.set('pattern', 'polygon'); lcm.settings.set('distribution', 'even'); lcm.settings.set('size', 400)");
check("polygon, filled: robots inside the hexagon, reaching near its corners", await until(`(() => { const f = lcm.renderer.frame; if (f.flags.length !== 500) return false;
  let far = 0; for (let i = 0; i < 500; i++) { const r = Math.hypot(f.xy[2*i], f.xy[2*i+1]); if (r > 200.001) return false; far = Math.max(far, r); } return far > 180; })()`, 3000) &&
  await js("lcm.settings.values.arrangement === 'filled'"));
await sleep(200);
await shot("9b-start-polygon-filled");
await js("lcm.settings.set('pattern', 'line'); lcm.settings.set('size', 600)");
check("a line: robots on a line", await until(`(() => { const f = lcm.renderer.frame; return [...Array(f.flags.length).keys()].every((i) => Math.abs(f.xy[2*i+1]) < 1e-3); })()`, 3000));
const lineDistributions = await js("[...document.querySelectorAll('#set-distribution option')].map((o) => o.value)");
check("only distributions that fit the pattern are offered", !lineDistributions.includes("gaussian") && lineDistributions.includes("even"), lineDistributions.join(", "));
await js("lcm.settings.set('pattern', 'clusters'); lcm.settings.set('size', 500); lcm.settings.set('spread', 50)");
await until("lcm.settings.values.pattern === 'clusters' && lcm.renderer.frame.flags.length === 500", 3000);
await sleep(300);
await shot("10-start-clusters");
await js("lcm.settings.set('pattern', 'cloud')");
await until("lcm.settings.values.pattern === 'cloud'", 3000);
await sleep(300);
const exported = (await js("lcm.client.csv().then((r) => r.text)"));
await js("lcm.settings.set('pattern', 'custom')");
await js(`lcm.settings.set('custom', ${JSON.stringify("# my robots\n0, 0\n80 0\n40;69.3")})`);
check("custom coordinates: the robot count is the number of lines", await until("lcm.renderer.frame.flags.length === 3 && lcm.settings.values.num_of_robots === 3", 3000));
check("custom coordinates: the robot-count control is locked", await js("document.getElementById('set-num_of_robots-n').disabled"));
const tri = await js("Array.from(lcm.renderer.frame.xy).map((v) => +v.toFixed(2))");
check("custom coordinates are used exactly", JSON.stringify(tri) === JSON.stringify([0, 0, 80, 0, 40, 69.3]), JSON.stringify(tri));
await js("document.getElementById('set-pattern').dispatchEvent(new Event('change'))");
check("custom coordinates are edited in their own window", await js("!document.getElementById('coords').hidden && document.activeElement.id === 'set-custom'"));
await sleep(200);
await shot("11-start-custom");
await js("document.getElementById('set-custom').dispatchEvent(new KeyboardEvent('keydown', { key: 'Escape', bubbles: true }))");
check("Esc closes the coordinates window; the panel shows a summary", await js("document.getElementById('coords').hidden && /3 robots/.test(document.getElementById('set-custom-summary').textContent)"));
await js(`lcm.settings.set('custom', ${JSON.stringify("1, 2\n3, abc")})`);
await sleep(400);
const err = await js("({ status: document.getElementById('set-custom-status').textContent, robots: lcm.renderer.frame.flags.length })");
check("a typo is reported by line and does not rebuild the run", /Line 2/.test(err.status) && err.robots === 3, JSON.stringify(err));
await js(`lcm.settings.set('custom', ${JSON.stringify("[[5, 5], [-5, -5]]")})`);
check("JSON coordinates work", await until("lcm.renderer.frame.flags.length === 2", 3000));
await js(`lcm.settings.set('custom', ${JSON.stringify(exported)})`);
const roundTrip = await until(`(() => { const f = lcm.renderer.frame; return f.flags.length === 500; })()`, 4000);
const cloudFirst = exported.split("\n")[1].split(",");
const firstNow = await js("Array.from(lcm.renderer.frame.xy.slice(0, 2))");
check("the simulator's own CSV export pastes back as a custom start", roundTrip && Math.abs(firstNow[0] - Number(cloudFirst[1])) < 1e-3 && Math.abs(firstNow[1] - Number(cloudFirst[2])) < 1e-3,
  `500 robots, first at (${firstNow.map((v) => v.toFixed(3)).join(", ")})`);
await js("lcm.settings.reset()");
await until("lcm.settings.values.pattern === 'cloud' && lcm.renderer.frame.flags.length === 500", 3000);

// Play straight after a settings change plays the new settings.
await click("#reset");
await until("lcm.state === 'ready'");
await js("lcm.settings.set('num_of_robots', 90); document.getElementById('play').click()");
await sleep(600);
check("Play right after a settings change runs the new settings", (await js("lcm.renderer.frame.robots")) === 90 && (await js("lcm.state")) === "running",
  `${await js("lcm.renderer.frame.robots")} robots, ${await js("lcm.state")}`);
// A frame from an earlier run (still in flight when a new run starts) is ignored.
await click("#reset");
await until("lcm.state === 'ready'");
const before = await js("lcm.frames");
await js(`(() => { const f = lcm.renderer.frame; lcm.client.receive({ ...f, type: "frame", run: lcm.client.run - 1, stop: "ended", ended: true }); })()`);
await sleep(200);
check("a stale frame from an earlier run cannot end the new run", (await js("lcm.frames")) === before && (await js("lcm.state")) === "ready");

// Runs to the end at Max speed.
await js("lcm.settings.reset()");
await click("#reset");
await until("lcm.state === 'ready'");
await js("lcm.settings.set('num_of_robots', 40)");
await until("lcm.renderer.frame.robots === 40", 3000);
await js("document.getElementById('playback').value = 'Infinity'; document.getElementById('playback').dispatchEvent(new Event('change'))");
await click("#play");
check("a run finishes and says so", await until("lcm.state === 'ended'", 10000), await js("document.getElementById('hud_status').textContent"));
check("play is disabled after the end", await js("document.getElementById('play').disabled"));
await sleep(300);
const framesAtEnd = await js("lcm.frames");
await sleep(1500);
const framesLater = await js("lcm.frames");
check("after the end the page stops fetching and redrawing frames", framesLater === framesAtEnd, `${framesLater - framesAtEnd} frames in 1.5 s after the end`);

// Exports.
const csv = await js("lcm.client.csv().then((r) => r.text)");
const lines = csv.trim().split("\n");
check("CSV export has a header and one exact row per robot", lines.length === 41 && lines[0].startsWith("id,x,y,state") && lines[1].split(",")[1].length > 8, `${lines.length} lines`);
const png = await js("lcm.renderer.toPngBlob('test').then((b) => b.size)");
check("PNG export produces an image", png > 5000, `${png} bytes`);

// Theme: system dark, then manual cycle.
await send("Emulation.setEmulatedMedia", { features: [{ name: "prefers-color-scheme", value: "dark" }] });
await sleep(200);
check("follows the system dark setting", (await js("getComputedStyle(document.body).backgroundColor")) === "rgb(14, 16, 20)");
await js("document.getElementById('playback').value = '20'; document.getElementById('playback').dispatchEvent(new Event('change'))");
await js("lcm.settings.set('num_of_robots', 500)");
await click("#reset");
await until("lcm.renderer.frame.robots === 500", 3000);
await click("#play");
await sleep(1200);
await shot("4-running-dark");
await key("t", "KeyT", 84);
check("T switches to light", (await js("document.documentElement.dataset.theme")) === "light");
await key("t", "KeyT", 84);
check("T switches to dark", (await js("document.documentElement.dataset.theme")) === "dark");
await key("t", "KeyT", 84);
check("T returns to system", (await js("document.documentElement.dataset.theme ?? 'system'")) === "system");

// Panels.
await key("h", "KeyH", 72);
check("H hides the panels", await js("getComputedStyle(document.getElementById('hud')).display === 'none'"));
await shot("5-clean-view");
await key("h", "KeyH", 72);
await key("?", "Slash", 191, true);
check("? opens the shortcut list", !(await js("document.getElementById('shortcuts').hidden")));
await shot("6-shortcuts");
await key("Escape", "Escape", 27);
check("Esc closes it", await js("document.getElementById('shortcuts').hidden"));

// 10,000 robots: drawing at 20x, simulation speed at Max.
await send("Emulation.setEmulatedMedia", { features: [{ name: "prefers-color-scheme", value: "light" }] });
await js("lcm.settings.set('num_of_robots', 10000)");
await click("#reset");
await until("lcm.renderer.frame.robots === 10000", 5000);
await js(`window.__gap = 0; window.__last = performance.now(); setInterval(() => { const n = performance.now(); __gap = Math.max(__gap, n - __last); __last = n; }, 10)`);
await click("#play");
await sleep(5000);
const perf = await js(`(() => { const r = lcm.renderer, t0 = performance.now();
  for (let i = 0; i < 20; i++) { r.draw(); r.ctx.getImageData(0, 0, 1, 1); }
  return { ...lcm.stats, drawWithPaintMs: (performance.now() - t0) / 20, maxPauseMs: __gap }; })()`);
await shot("7-10k-light");
await js("document.getElementById('playback').value = 'Infinity'; document.getElementById('playback').dispatchEvent(new Event('change'))");
await sleep(1000);
// Count events directly over a fixed wall time (the HUD figure is smoothed).
const e0 = await js("lcm.renderer.frame.events"), w0 = Date.now();
await sleep(4000);
const maxSpeed = { eventsPerSecond: Math.round(((await js("lcm.renderer.frame.events")) - e0) / ((Date.now() - w0) / 1000)),
  busy: await js("lcm.renderer.frame.busy"), fps: await js("lcm.stats.fps") };
check("10,000 robots: 55+ fps", perf.fps >= 55, `${perf.fps} fps`);
check("10,000 robots: draw < 16 ms per frame (CPU)", perf.drawWithPaintMs < 16, `${perf.drawWithPaintMs.toFixed(2)} ms`);
check("10,000 robots: main thread never blocked > 50 ms", perf.maxPauseMs < 50, `${perf.maxPauseMs.toFixed(1)} ms`);
check("10,000 robots at Max: simulation keeps the worker busy", maxSpeed.eventsPerSecond > 3000 && maxSpeed.busy > 0.85,
  `${maxSpeed.eventsPerSecond} events/s, worker ${Math.round(maxSpeed.busy * 100)} %, ${maxSpeed.fps} fps`);

check("no errors in the console", consoleErrors.length === 0, consoleErrors.join(" / "));

await finish();

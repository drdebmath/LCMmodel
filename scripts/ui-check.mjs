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
  for (let i = 0; i < f.flags.length; i++) { const [x, y] = r.toScreen(f.x(i), f.y(i));
    const d = (x - r.width/2) ** 2 + (y - r.height/2) ** 2; if (d < bd) { bd = d; best = i; } }
  const [x, y] = r.toScreen(f.x(best), f.y(best)); const b = r.canvas.getBoundingClientRect();
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
await click("#play");
await sleep(1500);
await shot("3-faults-visibility");
const legend2 = await js("document.getElementById('legend').textContent");
check("fault colours appear in the legend", /Byzantine/.test(legend2) && /Crashed/.test(legend2), legend2.trim());

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
  for (let i = 0; i < f.flags.length; i++) { const [x, y] = r.toScreen(f.x(i), f.y(i)); if (x < 0 || y < 0 || x > r.width || y > r.height) out++; } return out; })()`));
check("the view fits the whole cloud", onScreen === 0, `${onScreen} robots off screen`);
const visibleStart = () => js(`[...document.querySelectorAll('[data-group="start"] .field')].filter((f) => f.offsetParent && !f.closest('.more-body')).map((f) => f.dataset.key)`);
// "More options" always holds the stacked-points setting, and nothing else for a simple start.
const moreStart = () => js(`[...document.querySelectorAll('[data-more="start"] .field')].filter((f) => f.offsetParent).map((f) => f.dataset.key)`);
const cloudFields = await visibleStart();
check("the cloud start shows only the world switch, pattern and spread, and More options only the stacked points", JSON.stringify(cloudFields) === JSON.stringify(["open_world", "pattern", "spread"]) &&
  JSON.stringify(await js(`(() => { const d = document.querySelector('[data-more="start"]'); d.open = true; return [...d.querySelectorAll('.field')].filter((f) => f.offsetParent).map((f) => f.dataset.key); })()`)) === JSON.stringify(["multiplicities"]), cloudFields.join(", "));
check("no '?' icons; help is on the labels", await js("document.querySelectorAll('#settings .hint').length === 0 && document.querySelectorAll('#settings label.has-hint').length > 5"));
await shot("8-start-cloud");
await js("lcm.settings.set('open_world', false)");
await until("lcm.renderer.outline && lcm.renderer.outline.kind === 'rect'", 3000);
await sleep(300);
const world = await js(`(() => { const f = lcm.renderer.frame; let inBox = true;
  for (let i = 0; i < f.flags.length; i++) if (Math.abs(f.x(i)) > 300.001 || Math.abs(f.y(i)) > 300.001) inBox = false;
  return { pattern: lcm.settings.values.pattern, inBox, widthShown: !document.querySelector('[data-key="width_bound"]').hidden }; })()`);
check("World mode: the original box start, with its size settings", world.pattern === "box" && world.inBox && world.widthShown, JSON.stringify(world));
await js("lcm.settings.set('open_world', true); lcm.settings.set('pattern', 'circle'); lcm.settings.set('arrangement', 'edge'); lcm.settings.set('size', 400)");
check("a circle: robots evenly on a circle of radius 200", await until(`(() => { const f = lcm.renderer.frame;
  return f.flags.length === 500 && [...Array(500).keys()].every((i) => Math.abs(Math.hypot(f.x(i), f.y(i)) - 200) < 0.01); })()`, 3000));
await sleep(200);
await shot("9-start-ring");
check("a new start clears the hover tooltip of the old run", await js("document.getElementById('tip').hidden"));
const circleFields = await visibleStart();
const circleSummary = await js("document.getElementById('start-summary').textContent");
check("a circle's edge: pattern, arrangement, distribution and diameter, nothing else",
  JSON.stringify(circleFields) === JSON.stringify(["open_world", "pattern", "arrangement", "distribution", "size"]) && await js("lcm.settings.values.distribution === 'even'") &&
  await js("document.querySelector('[data-key=\"size\"] label').textContent === 'Diameter'") && JSON.stringify(await moreStart()) === JSON.stringify(["multiplicities"]),
  circleFields.join(", "));
check("one sentence says what the start gives", circleSummary === "500 robots evenly spaced on the edge of a circle 400 wide.", circleSummary);
const arrangements = await js("[...document.querySelectorAll('#set-arrangement option')].map((o) => o.value).join(',')");
check("a circle can be filled, edge only or a ring", arrangements === "filled,edge,ring", arrangements);
// The three combinations added with the three-setting layout.
await js("lcm.settings.set('pattern', 'square'); lcm.settings.set('arrangement', 'edge'); lcm.settings.set('size', 400)");
check("square, edge only: every robot on the square's outline", await until(`(() => { const f = lcm.renderer.frame; if (f.flags.length !== 500) return false;
  for (let i = 0; i < 500; i++) { const x = Math.abs(f.x(i)), y = Math.abs(f.y(i)); if (Math.max(x, y) > 200.01 || Math.max(x, y) < 199.99) return false; } return true; })()`, 3000));
await js("lcm.settings.set('pattern', 'rectangle'); lcm.settings.set('arrangement', 'edge'); lcm.settings.set('size', 400); lcm.settings.set('aspect', 0.5)");
check("rectangle, edge only: every robot on the rectangle's outline", await until(`(() => { const f = lcm.renderer.frame; if (f.flags.length !== 500) return false;
  for (let i = 0; i < 500; i++) { const x = Math.abs(f.x(i)) / 200, y = Math.abs(f.y(i)) / 100; if (Math.max(x, y) > 1.0001 || Math.max(x, y) < 0.9999) return false; } return true; })()`, 3000));
await js("lcm.settings.set('pattern', 'polygon'); lcm.settings.set('distribution', 'even'); lcm.settings.set('size', 400)");
check("polygon, filled: robots inside the hexagon, reaching near its corners", await until(`(() => { const f = lcm.renderer.frame; if (f.flags.length !== 500) return false;
  let far = 0; for (let i = 0; i < 500; i++) { const r = Math.hypot(f.x(i), f.y(i)); if (r > 200.001) return false; far = Math.max(far, r); } return far > 180; })()`, 3000) &&
  await js("lcm.settings.values.arrangement === 'filled'"));
await sleep(200);
await shot("9b-start-polygon-filled");
await js("lcm.settings.set('pattern', 'line'); lcm.settings.set('size', 600)");
check("a line: robots on a line", await until(`(() => { const f = lcm.renderer.frame; return [...Array(f.flags.length).keys()].every((i) => Math.abs(f.y(i)) < 1e-3); })()`, 3000));
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
const tri = await js("(() => { const f = lcm.renderer.frame; return [...Array(f.flags.length).keys()].flatMap((i) => [f.x(i), f.y(i)]).map((v) => +v.toFixed(2)); })()");
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
const firstNow = await js("(() => { const f = lcm.renderer.frame; return [f.x(0), f.y(0)]; })()");
check("the simulator's own CSV export pastes back as a custom start", roundTrip && Math.abs(firstNow[0] - Number(cloudFirst[1])) < 1e-3 && Math.abs(firstNow[1] - Number(cloudFirst[2])) < 1e-3,
  `500 robots, first at (${firstNow.map((v) => v.toFixed(3)).join(", ")})`);
await js("lcm.settings.reset()");
await until("lcm.settings.values.pattern === 'cloud' && lcm.renderer.frame.flags.length === 500", 3000);

// Precision shows the tolerance it stands for, and goes up to 15.
const epsNote = () => js("document.querySelector('[data-key=\"threshold_precision\"] .note').textContent");
check("precision shows ε in world units", (await epsNote()) === "ε = 0.00001 world units", await epsNote());
await js("lcm.settings.set('threshold_precision', 15)");
check("precision goes up to 15 and the note follows", (await epsNote()) === "ε = 0.000000000000001 world units" && (await js("lcm.settings.values.threshold_precision")) === 15, await epsNote());
await js("lcm.settings.set('threshold_precision', 50)");
check("precision cannot go past 15", (await js("lcm.settings.values.threshold_precision")) === 15);
await js("lcm.settings.set('threshold_precision', 5)");

// The sequential scheduler: one robot at a time, turns counted in epochs.
const shown = (key) => `!document.querySelector('[data-key="${key}"]').hidden`;
check("async is the default; turn order, pause and epochs only show for sequential",
  (await js("lcm.settings.values.scheduler")) === "async" && !(await js(shown("activation_order"))) && !(await js(shown("turn_gap"))) &&
  (await js("document.getElementById('hud_epochs_row').hidden")));
await js("lcm.settings.set('scheduler', 'sequential'); lcm.settings.set('num_of_robots', 30); lcm.settings.set('robot_speeds', 10)");
check("sequential shows the turn order and the pause between turns", (await js(shown("activation_order"))) && (await js(shown("turn_gap"))));
await js("lcm.settings.set('turn_gap', 'none')");
check("with no pause between turns, λ is greyed out", await js("document.getElementById('set-lambda_rate').disabled"));
await js("lcm.settings.set('turn_gap', 'random'); lcm.settings.set('activation_order', 'random')");
check("λ sets the random pause", !(await js("document.getElementById('set-lambda_rate').disabled")));
await until("lcm.state === 'ready' && lcm.renderer.frame.robots === 30", 3000);
await js(`window.__seq = { frames: 0, maxMoving: 0 }; window.__seqTimer = setInterval(() => {
  const f = lcm.renderer.frame; let moving = 0;
  for (let i = 0; i < f.flags.length; i++) if ((f.flags[i] & 3) === 2) moving++;
  __seq.frames++; __seq.maxMoving = Math.max(__seq.maxMoving, moving);
}, 30)`);
await click("#play");
await sleep(3000);
await js("clearInterval(__seqTimer)");
// Then at Max speed: at normal speed a single move takes seconds.
const playback = await js("document.getElementById('playback').value");
const setPlayback = (v) => js(`document.getElementById('playback').value = '${v}'; document.getElementById('playback').dispatchEvent(new Event('change'))`);
await setPlayback("Infinity");
const epochsUp = await until("lcm.stats.epochs >= 2", 8000);
await setPlayback(playback);
const seq = await js("({ ...__seq, epochs: lcm.stats.epochs, row: !document.getElementById('hud_epochs_row').hidden, text: document.getElementById('hud_epochs').textContent })");
check("sequential: the epoch counter shows and climbs", epochsUp && seq.row && Number(seq.text) === seq.epochs, JSON.stringify(seq));
check("sequential: never more than one robot moving at once", seq.frames > 20 && seq.maxMoving <= 1, JSON.stringify(seq));
const hudNum = (id) => js(`Number(document.getElementById('${id}').textContent.replace(/,/g, ''))`);
const [robotEvents, ticks, allEvents] = [await hudNum("hud_events"), await hudNum("hud_ticks"), await js("lcm.renderer.frame.events")];
check("Events are split into robot events and ticks that add up to the total", ticks > 0 && robotEvents > 0 && robotEvents + ticks === allEvents, `${robotEvents} + ${ticks} = ${allEvents}`);
await shot("12-sequential");
await click("#reset");
await js("lcm.settings.reset()");
await until("lcm.state === 'ready' && lcm.settings.values.scheduler === 'async' && lcm.renderer.frame.flags.length === 500", 3000);
check("back on async the epoch counter hides", await js("document.getElementById('hud_epochs_row').hidden"));

// Non-rigid movement: the minimum move δ and where a robot is stopped.
const shownNow = (key) => js(`!document.querySelector('[data-key="${key}"]').hidden`);
check("δ and where it stops are hidden while movement is rigid", !(await shownNow("delta")) && !(await shownNow("stop_policy")));
await js("lcm.settings.set('rigid_movement', false)");
check("non-rigid movement shows δ, but not the stopping rules until δ is set", (await shownNow("delta")) && !(await shownNow("stop_policy")));
await js("lcm.settings.set('delta', 20)");
check("with δ set, the stopping policy shows; the fraction waits for its policy", (await shownNow("stop_policy")) && !(await shownNow("stop_fraction")));
await js("lcm.settings.set('stop_policy', 'fraction')");
check("the fraction shows for the fraction policy", await shownNow("stop_fraction"));
await js("lcm.settings.set('stop_policy', 'random'); lcm.settings.set('scheduler', 'sequential'); lcm.settings.set('num_of_robots', 30); lcm.settings.set('robot_speeds', 10)");
await until("lcm.state === 'ready' && lcm.renderer.frame.robots === 30", 3000);
await setPlayback("Infinity");
await click("#play");
const nonRigidEnds = await until("lcm.state === 'ended'", 20000);
const nonRigid = await js("({ terminated: lcm.stats.terminated, stop: lcm.stats.stop, status: document.getElementById('hud_status').textContent })");
check("a sequential run with non-rigid δ-stops still gathers all robots", nonRigidEnds && nonRigid.terminated === 30 && nonRigid.stop === "ended", JSON.stringify(nonRigid));
await click("#reset");
await js("lcm.settings.reset()");
await until("lcm.state === 'ready' && lcm.settings.values.rigid_movement === true && lcm.renderer.frame.flags.length === 500", 3000);
await setPlayback(playback);

// Starting with robots stacked on a few points, and the turn limit.
await click("#reset");
await until("lcm.state === 'ready'", 3000);
check("the stack size waits until some stacked points are asked for", !(await shownNow("multiplicity_size")) && !(await shownNow("max_turns")));
await js("lcm.settings.set('num_of_robots', 30); lcm.settings.set('multiplicities', 2); lcm.settings.set('multiplicity_size', 3)");
check("with stacked points asked for, the stack size shows", await shownNow("multiplicity_size"));
const stacked = await until(`(() => { const f = lcm.renderer.frame; if (!f || f.flags.length !== 30) return false;
  const seen = new Set(); for (let i = 0; i < 30; i++) seen.add(f.x(i) + "," + f.y(i)); return seen.size === 26; })()`, 4000);
check("two stacked points of 3 robots leave 26 distinct start points", stacked,
  JSON.stringify(await js(`(() => { const f = lcm.renderer.frame; const seen = new Set(); for (let i = 0; i < f.flags.length; i++) seen.add(f.x(i) + "," + f.y(i));
    return { robots: f.flags.length, distinct: seen.size, state: lcm.state, dirty: !document.getElementById("dirty").hidden, values: [lcm.settings.values.multiplicities, lcm.settings.values.multiplicity_size, lcm.settings.values.num_of_robots] }; })()`)));
await js("lcm.settings.set('multiplicities', 0)");
check("the stack size hides again at 0", !(await shownNow("multiplicity_size")));
await js("lcm.settings.set('scheduler', 'sequential'); lcm.settings.set('max_turns', 12); lcm.settings.set('robot_speeds', 10)");
check("the turn limit shows for the sequential scheduler", await shownNow("max_turns"));
await until("lcm.state === 'ready' && lcm.renderer.frame.robots === 30", 3000);
await setPlayback("Infinity");
await click("#play");
const limited = await until("lcm.state === 'ended'", 20000);
const turnLimit = await js("({ stop: lcm.stats.stop, status: document.getElementById('hud_status').textContent })");
check("a run stops at the turn limit and says so", limited && turnLimit.stop === "max-turns" && /turn limit/.test(turnLimit.status), JSON.stringify(turnLimit));
await click("#reset");
await js("lcm.settings.reset()");
await until("lcm.state === 'ready' && lcm.settings.values.max_turns === 0 && lcm.renderer.frame.flags.length === 500", 3000);
await setPlayback(playback);

// Go-To-Center: examples, who-sees-whom links, the allowed region and the swarm-size plot.
await click("#reset");
await js("lcm.settings.reset()");
await until("lcm.state === 'ready' && lcm.renderer.frame.flags.length === 500", 3000);
check("the examples menu is hidden for an algorithm without examples", await js("document.getElementById('examples').hidden"));
check("the allowed region is only drawn for Go-To-Center", (await js("lcm.renderer.options.allowedRegion")) === false);
await js(`(() => { const s = document.getElementById('algorithm'); s.value = 'GoToCenter'; s.dispatchEvent(new Event('change')); })()`);
await sleep(300);
const pickExample = async (i) => { await click("#examples"); await click(`#examples_menu [data-example="${i}"]`); };
check("the examples button is a toolbar button that opens a menu", await js("(() => { const b = document.getElementById('examples'); return b.tagName === 'BUTTON' && !b.hidden && b.getAttribute('aria-haspopup') === 'menu' && b.getAttribute('aria-expanded') === 'false' && document.getElementById('examples_menu').hidden; })()"));
await click("#examples");
const exampleLabels = await js("[...document.querySelectorAll('#examples_menu .title')].map((o) => o.textContent)");
check("Go-To-Center offers None and three ready-made examples", exampleLabels.length === 4 && exampleLabels[0] === "None" && /chain/i.test(exampleLabels[1]), exampleLabels.join(" | "));
check("the open menu sits under its button, on screen, with None marked", await js(`(() => { const m = document.getElementById('examples_menu').getBoundingClientRect(), b = document.getElementById('examples').getBoundingClientRect();
  return document.getElementById('examples').getAttribute('aria-expanded') === 'true' && m.top >= b.bottom && m.left >= 0 && m.right <= innerWidth && m.bottom <= innerHeight
    && document.querySelector('#examples_menu [data-example="-1"]').getAttribute('aria-checked') === 'true' && document.querySelectorAll('#examples_menu [aria-checked=true]').length === 1; })()`));
await shot("examples-menu");
await key("ArrowDown", "ArrowDown", 40);
check("the arrow keys move through the menu", await js("document.activeElement.dataset.example === '0'"));
await key("Escape", "Escape", 27);
check("Escape closes the menu and returns to its button", await js("document.getElementById('examples_menu').hidden && document.activeElement.id === 'examples'"));
await js("lcm.settings.apply({ num_of_robots: 123 })");
await until("lcm.state === 'ready' && lcm.renderer.frame && lcm.renderer.frame.robots === 123", 3000);
await pickExample(0);
check("the chain example sets up three robots with limited sight and the links on", await until("lcm.renderer.frame && lcm.renderer.frame.robots === 3 && lcm.settings.values.visibility_radius === 100 && lcm.settings.values.unlimited_visibility === false", 4000)
  && (await js("document.getElementById('links').getAttribute('aria-pressed')")) === "true" && (await js("lcm.renderer.options.allowedRegion")) === true);
check("the menu closes after a choice, and the choice is marked", await js("document.getElementById('examples_menu').hidden && document.getElementById('examples').getAttribute('aria-pressed') === 'true' && document.querySelector('#examples_menu [data-example=\"0\"]').getAttribute('aria-checked') === 'true'"));
await pickExample(-1);
check("None puts back the settings from before the example", await until("lcm.state === 'ready' && lcm.renderer.frame && lcm.renderer.frame.robots === 123 && lcm.settings.values.pattern === 'cloud' && lcm.settings.values.unlimited_visibility === true", 4000)
  && (await js("document.getElementById('links').getAttribute('aria-pressed')")) === "false" && (await js("document.getElementById('examples').getAttribute('aria-pressed')")) === "false");
await js("lcm.settings.apply({ num_of_robots: 500 })");
await pickExample(0);
await until("lcm.renderer.frame && lcm.renderer.frame.robots === 3", 4000);
const withLinks = await js("lcm.renderer.requestDraw(), new Promise((r) => requestAnimationFrame(() => r(lcm.renderer.canvas.toDataURL())))");
await js("document.getElementById('links').click()");
const withoutLinks = await js("lcm.renderer.requestDraw(), new Promise((r) => requestAnimationFrame(() => r(lcm.renderer.canvas.toDataURL())))");
check("the links button draws and removes the lines between robots", withLinks !== withoutLinks && (await js("document.getElementById('links').getAttribute('aria-pressed')")) === "false");
await js("window.dispatchEvent(new KeyboardEvent('keydown', { key: 'v', bubbles: true }))");
check("the V key toggles the links too", (await js("lcm.renderer.options.links")) === true);
await pickExample(1);
await until("lcm.state === 'ready' && lcm.renderer.frame && lcm.renderer.frame.robots === 40", 4000);
await setPlayback("Infinity");
await click("#play");
const swarmEnds = await until("lcm.state === 'ended'", 30000);
await sleep(300);
const plot = await js(`(() => { const p = lcm.trace.points, n = p.length / 2; const c = document.getElementById('hud_plot'); const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data; let ink = 0; for (let i = 3; i < d.length; i += 4) if (d[i] > 0) ink++; return { samples: n, first: p[1], last: p[2 * n - 1], ink, label: document.getElementById('hud_spread').textContent }; })()`);
check("a 40-robot Go-To-Center swarm gathers, and the plot shows its size falling to about the threshold", swarmEnds && plot.samples >= 40 && plot.last < plot.first / 1000 && plot.last < 2e-5 && plot.ink > 200, JSON.stringify(plot));
await click("#reset");
await js(`(() => { const s = document.getElementById('algorithm'); s.value = 'Gathering'; s.dispatchEvent(new Event('change')); })()`);
await js("lcm.settings.reset()");
await until("lcm.state === 'ready' && lcm.settings.values.pattern === 'cloud' && lcm.renderer.frame.flags.length === 500", 3000);
await setPlayback(playback);
await js("document.getElementById('links').getAttribute('aria-pressed') === 'true' && document.getElementById('links').click()");
check("back on Center of Gravity, the examples menu is hidden again", await js("document.getElementById('examples').hidden && document.getElementById('algorithm').value === 'Gathering'"));

// Follow the swarm: the view zooms in as the swarm shrinks. It is display only.
const TWO_ROBOTS = "lcm.settings.apply({ open_world: true, pattern: 'custom', custom: '-300, 0\\n300, 0', scheduler: 'sequential', turn_gap: 'none' })";
const followRun = async (follow) => {
  await click("#reset");
  await js(TWO_ROBOTS);
  await until("lcm.state === 'ready' && lcm.renderer.frame && lcm.renderer.frame.robots === 2", 4000);
  await js(`(() => { if (lcm.renderer.follow !== ${follow}) document.getElementById('follow').click(); })()`);
  await setPlayback("Infinity");
  await click("#play");
  const ended = await until("lcm.state === 'ended'", 30000);
  await sleep(2200); // the view eases to its final zoom
  return { ended, ...(await js(`(() => { const f = lcm.renderer.frame, r = lcm.renderer;
    const [x0, y0] = r.toScreen(f.x(0), f.y(0)), [x1, y1] = r.toScreen(f.x(1), f.y(1));
    const inside = [x0, y0, x1, y1].every((v, k) => v > 0 && v < (k % 2 ? r.height : r.width));
    return { time: f.time, events: f.events, pos: [f.x(0), f.y(0), f.x(1), f.y(1)], apart: Math.hypot(x0 - x1, y0 - y1), inside, zoom: r.zoom,
      scale: document.getElementById('hud_scale').textContent, pressed: document.getElementById('follow').getAttribute('aria-pressed') }; })()`)) };
};
const still = await followRun(false);
const followed = await followRun(true);
check("both runs reach the same end", still.ended && followed.ended && still.time === followed.time && still.events === followed.events
  && JSON.stringify(still.pos) === JSON.stringify(followed.pos), `${still.time} / ${followed.time}, ${still.events} / ${followed.events} events`);
check("without following, the two gathered robots (about 0.00002 apart) sit on the same pixel", still.apart < 1 && still.zoom === 1, `${still.apart.toFixed(3)} px apart`);
check("following zooms in until the two robots fill the view, both on screen", followed.apart > 250 && followed.inside && followed.zoom > 1e6 && followed.pressed === "true", `${followed.apart.toFixed(0)} px apart, zoom ${followed.zoom.toExponential(1)}`);
check("the scale bar follows the zoom (about 2×10⁻⁶ when following, tens when not)", /×10⁻/.test(followed.scale) && /^\d+$/.test(still.scale), `${still.scale} / ${followed.scale}`);
await shot("follow-the-swarm");
await key("Z", "KeyZ", 90);
check("the Z key turns following off, and the button shows it", await js("!lcm.renderer.follow && document.getElementById('follow').getAttribute('aria-pressed') === 'false'"));
await key("Z", "KeyZ", 90);
check("the Z key turns it on again", await js("lcm.renderer.follow && document.getElementById('follow').getAttribute('aria-pressed') === 'true'"));
await js("lcm.renderer.zoomAt(1.2)");
check("zooming by hand takes over from following", await js("!lcm.renderer.follow && document.getElementById('follow').getAttribute('aria-pressed') === 'false'"));
await js("document.getElementById('follow').click()");
await click("#fit");
check("Fit also stops following and shows the whole world again", await js("!lcm.renderer.follow && lcm.renderer.zoom === 1 && lcm.renderer.anchor.x === 0"));
await click("#reset");
await js("lcm.settings.reset()");
await until("lcm.state === 'ready' && lcm.settings.values.pattern === 'cloud' && lcm.renderer.frame.flags.length === 500", 3000);
await setPlayback(playback);

// Turn-paced playback (sequential scheduler): every walk takes the same time on screen, however
// short it is in simulated time. Display only: the run must end exactly as it does at any speed.
const paceRun = async (speed, { pauseForMs = 0 } = {}) => {
  await click("#reset");
  await js(TWO_ROBOTS);
  await until("lcm.state === 'ready' && lcm.renderer.frame && lcm.renderer.frame.robots === 2", 4000);
  await setPlayback(speed);
  await click("#play");
  const t0 = Date.now(), stamps = [];
  let turns = 0, paused = null;
  while (Date.now() - t0 < 90000) {
    const r = await js("({ s: lcm.state, turns: lcm.renderer.frame.turns, x0: lcm.renderer.frame.x(0) })");
    if (r.turns !== turns) { stamps.push(Date.now() - t0); turns = r.turns; }
    if (pauseForMs && !paused && r.turns >= 3 && r.s === "running") {
      await sleep(150);                      // part-way along a walk
      await click("#play");                  // pause
      await sleep(100);
      const a = await js("lcm.renderer.frame.x(0) + ',' + lcm.renderer.frame.x(1) + ',' + lcm.renderer.frame.time");
      await sleep(pauseForMs);
      const b = await js("lcm.renderer.frame.x(0) + ',' + lcm.renderer.frame.x(1) + ',' + lcm.renderer.frame.time");
      paused = { still: a === b, state: await js("lcm.state") };
      await click("#play");                  // resume
    }
    if (r.s === "ended") break;
    await sleep(20);
  }
  const end = await js(`(() => { const f = lcm.renderer.frame; return { ended: lcm.state === 'ended', time: f.time, events: f.events, turns: f.turns, pos: [f.x(0), f.y(0), f.x(1), f.y(1)] }; })()`);
  const gaps = stamps.slice(1).map((v, k) => v - stamps[k]).sort((a, b) => a - b);
  return { ...end, took: Date.now() - t0, median: gaps[Math.floor(gaps.length / 2)] ?? 0, paused };
};
const sameEnd = (a, b) => a.ended && b.ended && a.time === b.time && a.events === b.events && JSON.stringify(a.pos) === JSON.stringify(b.pos);
const atMax = await paceRun("Infinity");
check("the per-turn speeds are offered under the sequential scheduler ", await js(`(() => { const o = [...document.querySelectorAll('#playback option')].map((x) => x.value);
  return ['turn:0.25', 'turn:1', 'turn:3'].every((v) => o.includes(v)); })()`));
const at100 = await paceRun("100");
const perTurn = await paceRun("turn:0.25");
check("the same run ends the same at Max speed, 100× and 0.25 s per turn (time, events, positions)",
  sameEnd(atMax, at100) && sameEnd(atMax, perTurn), `${atMax.time} / ${at100.time} / ${perTurn.time}; ${atMax.events} / ${at100.events} / ${perTurn.events}`);
check("0.25 s per turn: each turn takes about a quarter of a second on screen", perTurn.median > 170 && perTurn.median < 450, `median ${perTurn.median} ms between turns, ${perTurn.turns} turns in ${(perTurn.took / 1000).toFixed(1)} s`);
check("the status panel shows the turn count (with the epochs) and it matches the run", await js("!document.getElementById('hud_epochs_row').hidden && Number(document.getElementById('hud_turns').textContent.replace(/,/g, '')) === lcm.renderer.frame.turns"));
const paused = await paceRun("turn:0.25", { pauseForMs: 600 });
check("pausing in the middle of a walk freezes it, and the run still ends the same after resuming", paused.paused?.still && paused.paused.state === "paused" && sameEnd(atMax, paused), JSON.stringify(paused.paused));

// Inside one turn the robot glides although the simulation has no event there: slow it right down and look.
await click("#reset");
await js(TWO_ROBOTS);
await until("lcm.state === 'ready' && lcm.renderer.frame && lcm.renderer.frame.robots === 2", 4000);
await setPlayback("turn:3");
await click("#play");
await until("lcm.renderer.frame.turns >= 1", 4000);
const glide = [];
for (let k = 0; k < 5; k++) {
  await sleep(350);
  glide.push(await js("({ x: lcm.renderer.frame.x(0), turns: lcm.renderer.frame.turns, robotEvents: lcm.renderer.frame.events - lcm.renderer.frame.ticks, time: lcm.renderer.frame.time })"));
}
await click("#play");
check("within one 3-second turn the robot glides steadily along its walk", glide.every((g, i) => !i || g.x > glide[i - 1].x) && glide[4].x - glide[0].x > 40
  && glide.every((g) => g.turns === glide[0].turns && g.robotEvents === glide[0].robotEvents), JSON.stringify(glide.map((g) => +g.x.toFixed(1))));

// The view tools: tidy. No lights button; zoom + and − sit in the status panel's Zoom row; "who sees whom" only
// when the robots see just part of the swarm; the rest in two columns, in pairs.
await js("lcm.settings.reset()");
await until("lcm.state === 'ready' && lcm.settings.values.unlimited_visibility === true && lcm.renderer.frame.flags.length === 500", 3000);
await sleep(300);
await key("L", "KeyL", 76);
check("there is no lights button, and L does nothing", await js("!document.getElementById('lights') && lcm.renderer.options.showLights === false"));
const toolsNow = await js(`(() => { const t = document.getElementById('tools'); const shown = [...t.querySelectorAll('button')].filter((b) => getComputedStyle(b).display !== 'none').map((b) => b.id);
  return { shown, columns: getComputedStyle(t).gridTemplateColumns.split(' ').length, linksHidden: document.getElementById('links').hidden }; })()`);
check("the view tools are in pairs, two columns, without + and − (those are in the status panel): follow, fit, grid, measure, export, full screen, theme, help",
  toolsNow.columns === 2 && JSON.stringify(toolsNow.shown) === JSON.stringify(["follow", "fit", "grid", "measure", "export", "fullscreen", "theme", "help"]), JSON.stringify(toolsNow));
check("with unlimited sight the who-sees-whom button is hidden, and V does nothing", toolsNow.linksHidden && (await (async () => { await key("V", "KeyV", 86); return js("lcm.renderer.options.links === false"); })()));
const zoomBefore = await js("(() => { lcm.renderer.resetView(); return lcm.renderer.zoom; })()");
await click("#hud_zoom_in");
const zoomIn = await js("lcm.renderer.zoom");
await click("#hud_zoom_out");
const zoomOut = await js("lcm.renderer.zoom");
check("the + and − in the status panel's Zoom row zoom the view", Math.abs(zoomIn / zoomBefore - 1.25) < 1e-9 && Math.abs(zoomOut / zoomIn - 0.8) < 1e-9, `${zoomBefore} -> ${zoomIn} -> ${zoomOut}`);
check("the zoom buttons in the tools panel are not shown on a screen with room for the status panel", await js("getComputedStyle(document.getElementById('zoom_in')).display === 'none' && getComputedStyle(document.getElementById('hud_zoom_in')).display !== 'none'"));
await js("lcm.settings.set('unlimited_visibility', false); lcm.settings.set('visibility_radius', 150)");
await click("#reset");
await until("lcm.state === 'ready' && lcm.renderer.options.visibility === 150", 4000);
await sleep(200);
check("with a limited sight radius the who-sees-whom button appears, and V toggles it", !(await js("document.getElementById('links').hidden")) && (await (async () => { await key("V", "KeyV", 86); const on = await js("lcm.renderer.options.links === true"); await key("V", "KeyV", 86); return on; })()));
await js("lcm.settings.reset()");
await until("lcm.state === 'ready' && lcm.settings.values.unlimited_visibility === true", 3000);
await sleep(300);

// The grid: behind everything, labelled, and still readable however far the view is zoomed in.
check("a grid is on by default, with a button that shows it", await js("lcm.renderer.options.grid === true && document.getElementById('grid').getAttribute('aria-pressed') === 'true'"));
const gridPixels = () => js(`(() => { const r = lcm.renderer; r.draw(); const g = r.canvas.getContext('2d').getImageData(0, 0, r.canvas.width, r.canvas.height).data; let n = 0; for (let i = 0; i < g.length; i += 4) if (g[i] !== g[0] || g[i + 1] !== g[1] || g[i + 2] !== g[2]) n++; return n; })()`);
const withGrid = await gridPixels();
await key("G", "KeyG", 71);
const withoutGrid = await gridPixels();
check("the G key hides the grid (it draws lines), and again shows it", await js("lcm.renderer.options.grid === false && document.getElementById('grid').getAttribute('aria-pressed') === 'false'") && withGrid > withoutGrid + 500, `${withGrid} vs ${withoutGrid} non-background pixels`);
await key("G", "KeyG", 71);
check("and shows it again", await js("lcm.renderer.options.grid === true"));
await click("#reset");
await js(TWO_ROBOTS);
await until("lcm.state === 'ready' && lcm.renderer.frame && lcm.renderer.frame.robots === 2", 4000);
await js("(() => { if (!lcm.renderer.follow) document.getElementById('follow').click(); })()");
await setPlayback("Infinity");
await click("#play");
await until("lcm.state === 'ended'", 30000);
await sleep(2500);
const deep = await js("({ grid: lcm.renderer.gridState, zoom: lcm.renderer.zoom, end: document.getElementById('turn_caption').innerText, hidden: document.getElementById('turn_caption').hidden })");
check("zoomed in about 30 million times, the grid still has labelled lines 80 px apart or more, at a spacing near the robots' gap", deep.zoom > 1e6 && deep.grid && deep.grid.px >= 80 && deep.grid.px <= 1500 && deep.grid.spacing <= 1e-4 && deep.grid.spacing >= 1e-8, JSON.stringify(deep.grid));
check("a finished run says so, in words: grey means terminated, the real size, and how far it is zoomed in", !deep.hidden && /Finished/.test(deep.end) && /terminated/.test(deep.end) && /units across/.test(deep.end) && /zoomed in [\d,]+ times/.test(deep.end), deep.end);
await click("#reset");
await until("lcm.state === 'ready'", 4000);
await sleep(300);
check("a new run takes the finished note away", await js("document.getElementById('turn_caption').hidden"));

// The numbers are true at every zoom: the scale bar is as long as its label says, the labelled grid lines are
// 1, 2 or 5 times a power of ten and 85 to 230 px apart, and the zoom gauge reads the real zoom.
const zoomRows = [];
for (const f of [1, 2.5, 10, 1e3, 1e6, 1e9]) {
  await js(`(() => { const r = lcm.renderer; r.resetView(); r.zoomAt(${f}); r.draw(); })()`);
  zoomRows.push({ f, ...(await js(`(() => { const r = lcm.renderer, g = r.gridState; return { bar: parseFloat(document.getElementById('hud_scale_bar').style.width), len: Number(document.getElementById('hud_scale').dataset.value), scale: r.scale,
    px: g.px, mant: g.spacing / 10 ** Math.floor(Math.log10(g.spacing) + 1e-9), gauge: document.getElementById('hud_zoom').textContent, fill: parseFloat(document.getElementById('hud_zoom_fill').style.width), gridRow: document.getElementById('hud_grid').textContent }; })()`)) });
}
check("the scale bar is as long as its label says, at every zoom from 1× to a billion ×", zoomRows.every((z) => Math.abs(z.bar - z.len * z.scale) <= 1.5), JSON.stringify(zoomRows.map((z) => [z.f, z.len, +z.bar.toFixed(1), +(z.len * z.scale).toFixed(1)])));
check("the labelled grid lines are 1, 2 or 5 times a power of ten, 85 to 230 px apart, at every zoom", zoomRows.every((z) => z.px >= 85 && z.px <= 230 && [1, 2, 5].some((m) => Math.abs(z.mant - m) < 1e-6)), JSON.stringify(zoomRows.map((z) => [z.f, +z.px.toFixed(0), +z.mant.toFixed(3)])));
check("the zoom gauge reads the real zoom (number and bar), and the grid-lines row names their spacing", zoomRows.every((z) => Math.abs(z.fill - (100 * Math.log10(z.f)) / 9) < 0.6 && /×$/.test(z.gauge) && z.gridRow !== "—" && z.gridRow !== "off"), JSON.stringify(zoomRows.map((z) => [z.f, z.gauge, +z.fill.toFixed(1), z.gridRow])));
await click("#fit");

// The overview map: the starting area, with a rectangle for the part in view; zoomed far in, a marked dot.
await click("#reset");
await js(TWO_ROBOTS);
await until("lcm.state === 'ready' && lcm.renderer.frame && lcm.renderer.frame.robots === 2", 4000);
await js("lcm.renderer.resetView(); lcm.renderer.draw()");
const map0 = await js(`(() => { const o = lcm.overview, c = document.getElementById('overview_canvas'); const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data; let ink = 0; for (let i = 0; i < d.length; i += 4) if (d[i] !== d[0] || d[i + 1] !== d[1] || d[i + 2] !== d[2]) ink++;
  const r = document.getElementById('overview').getBoundingClientRect(); return { shown: o.visible, ink, last: o.last, w: Math.round(r.width), h: Math.round(r.height), onScreen: r.left >= 0 && r.right <= innerWidth && r.top >= 0 && r.bottom <= innerHeight }; })()`);
check("an overview map is drawn in the corner, on screen, with the starting area, the robots and the view", map0.shown && map0.ink > 150 && map0.onScreen && map0.last && !map0.last.tiny, JSON.stringify({ ...map0, last: undefined }));
const sizes = [];
for (const f of [1, 4, 40, 1e3, 1e6]) {
  await js(`(() => { const r = lcm.renderer; r.resetView(); r.zoomAt(${f}); r.draw(); })()`);
  sizes.push({ f, ...(await js("({ w: lcm.overview.last.rectW, tiny: lcm.overview.last.tiny })")) });
}
check("the view rectangle shrinks as the view zooms in, and becomes a marked dot far in", sizes.every((s, i) => !i || s.w < sizes[i - 1].w) && !sizes[0].tiny && sizes[3].tiny && sizes[4].tiny, JSON.stringify(sizes.map((s) => [s.f, +s.w.toFixed(1), s.tiny])));
await js("lcm.renderer.resetView(); lcm.renderer.zoomAt(8); lcm.renderer.draw()");
const mapTarget = [60, -40];
await js(`(() => { const o = lcm.overview, m = o.map, c = document.getElementById('overview_canvas').getBoundingClientRect(); const px = m.offX + (${mapTarget[0]} - m.x0) * m.k, py = o.canvas.height / o.dpr - m.offY - (${mapTarget[1]} - (m.y1 - (o.canvas.height / o.dpr - 2 * m.offY) / m.k)) * m.k;
  const e = new PointerEvent('pointerdown', { clientX: c.left + px * (c.width / 176), clientY: c.top + py * (c.height / 132), bubbles: true }); o.canvas.dispatchEvent(e); })()`);
await sleep(100);
const moved = await js("(() => { const v = lcm.renderer.viewRect(), k = lcm.overview.map.k; return { cx: (v.x0 + v.x1) / 2, cy: (v.y0 + v.y1) / 2, tol: 2.5 / k, follow: lcm.renderer.follow }; })()");
check("clicking the map moves the view to that spot (and stops following)", Math.abs(moved.cx - mapTarget[0]) < moved.tol && Math.abs(moved.cy - mapTarget[1]) < moved.tol && !moved.follow, JSON.stringify({ ...moved, mapTarget }));
await key("O", "KeyO", 79);
const hiddenMap = await js("document.getElementById('overview').hidden");
await key("O", "KeyO", 79);
check("the O key hides the map, and shows it again", hiddenMap && (await js("!document.getElementById('overview').hidden")));
await click("#fit");

// The measuring tool: click a point or a robot, then a second one. A robot end stays attached to it (the
// number follows it); empty space is a fixed point in the world.
const robotAt = (i, dx = 0, dy = 0) => js(`(() => { const r = lcm.renderer, f = r.frame, c = r.canvas.getBoundingClientRect(); const [sx, sy] = r.toScreen(f.x(${i}), f.y(${i})); return [c.left + sx + ${dx}, c.top + sy + ${dy}]; })()`);
const mouse = async ([x, y]) => { await clickAt(x, y); await sleep(60); };
await click("#reset");
await js(TWO_ROBOTS);
await until("lcm.state === 'ready' && lcm.renderer.frame && lcm.renderer.frame.robots === 2", 4000);
await js("lcm.renderer.resetView(); lcm.renderer.draw()");
await key("M", "KeyM", 77);
check("M turns the measuring tool on: the button shows it, the pointer is a crosshair, a hint says what to do", await js("lcm.measure.on && document.getElementById('measure').getAttribute('aria-pressed') === 'true' && document.getElementById('canvas').classList.contains('measuring') && /Measure: click/.test(document.getElementById('toast').textContent)"));
await mouse(await robotAt(0, 4, -3));
await mouse(await robotAt(1, -3, 4));
const robots = await js("(() => { const i = lcm.renderer.measureInfo(); return { d: i.d, a: i.a.robot, b: i.b.robot, done: i.done, selected: lcm.renderer.selected }; })()");
check("clicking near two robots measures between them (600 apart at the start), and does not select a robot", robots.a === 0 && robots.b === 1 && robots.done && Math.abs(robots.d - 600) < 1e-6 && robots.selected === -1, JSON.stringify(robots));
await setPlayback("Infinity");
await click("#play");
await until("lcm.state === 'ended'", 30000);
await sleep(300);
const live = await js("(() => { const i = lcm.renderer.measureInfo(), f = lcm.renderer.frame; return { d: i.d, apart: Math.abs(f.x(1) - f.x(0)), a: i.a.robot, b: i.b.robot }; })()");
check("the ends stay on their robots: after the run the number is their new gap (about 0.00002), to 1e-9", live.a === 0 && live.b === 1 && live.d < 1e-4 && live.d > 0 && Math.abs(live.d - live.apart) < 1e-9 * Math.max(1, live.d), JSON.stringify(live));
await click("#reset");
await js(TWO_ROBOTS);
await until("lcm.state === 'ready' && lcm.renderer.frame && lcm.renderer.frame.robots === 2", 4000);
await js("lcm.renderer.resetView(); lcm.renderer.draw()");
await key("Escape", "Escape", 27);  // clears the measurement from the last run
const emptyA = await js("(() => { const c = lcm.renderer.canvas.getBoundingClientRect(); return [c.left + 400, c.top + 520]; })()");
const emptyB = await js("(() => { const c = lcm.renderer.canvas.getBoundingClientRect(); return [c.left + 700, c.top + 620]; })()");
await mouse(emptyA);
await mouse(emptyB);
const fixedBefore = await js("(() => { const i = lcm.renderer.measureInfo(); return { d: i.d, ar: i.a.robot, br: i.b.robot, ax: i.a.x, ay: i.a.y, scale: lcm.renderer.scale }; })()");
const pxDistance = Math.hypot(emptyB[0] - emptyA[0], emptyB[1] - emptyA[1]);
await js("lcm.renderer.zoomAt(5); lcm.renderer.panBy(-80, 40); lcm.renderer.draw()");
const fixedAfter = await js("(() => { const i = lcm.renderer.measureInfo(); return { d: i.d, ax: i.a.x, ay: i.a.y }; })()");
check("empty space gives fixed points whose distance is the true one, and it stays put when the view zooms and pans",
  fixedBefore.ar === undefined && fixedBefore.br === undefined && Math.abs(fixedBefore.d - pxDistance / fixedBefore.scale) < 1e-9 * fixedBefore.d && fixedAfter.d === fixedBefore.d && fixedAfter.ax === fixedBefore.ax && fixedAfter.ay === fixedBefore.ay,
  JSON.stringify({ fixedBefore, expected: pxDistance / fixedBefore.scale, fixedAfter }));
await js("lcm.renderer.resetView(); lcm.renderer.draw()");
await mouse(await robotAt(0));
const third = await js("(() => { const m = lcm.measure; return { a: m.a?.robot, b: m.b }; })()");
check("a third click starts a new measurement", third.a === 0 && third.b === null, JSON.stringify(third));
await key("Escape", "Escape", 27);
const cleared = await js("lcm.measure.a === null && lcm.measure.on");
await key("Escape", "Escape", 27);
check("Esc clears the measurement, and a second Esc leaves the tool", cleared && (await js("!lcm.measure.on && document.getElementById('measure').getAttribute('aria-pressed') === 'false' && lcm.renderer.options.measure === null")));
await key("M", "KeyM", 77);
await key("M", "KeyM", 77);
check("and M turns it on and off", await js("!lcm.measure.on"));
await setPlayback(playback);

// The turn view: a caption says whose turn it is and why; the camera stands still while the robot walks.
await click("#reset");
await js(TWO_ROBOTS);
await until("lcm.state === 'ready' && lcm.renderer.frame && lcm.renderer.frame.robots === 2", 4000);
await js("(() => { if (!lcm.renderer.follow) document.getElementById('follow').click(); })()");
await setPlayback("turn:3");
await click("#play");
await until("lcm.renderer.frame.turns >= 3", 20000);
const seen = [];
for (let k = 0; k < 16; k++) {
  await sleep(250);
  seen.push(await js(`({ phase: lcm.renderer.options.walk?.phase ?? null, robot: lcm.renderer.options.walk?.robot ?? null, turns: lcm.renderer.frame.turns,
    hidden: document.getElementById('turn_caption').hidden, cap: document.getElementById('turn_caption').innerText, zoom: lcm.renderer.zoom, ax: lcm.renderer.anchor.x })`));
}
await click("#play");
const decide = seen.find((x) => x.phase === "decide");
const move = seen.filter((x) => x.phase === "move");
check("a turn starts with its decision: a caption names the robot and gives the rule in words", !!decide && !decide.hidden && /Turn \d+/.test(decide.cap) && /Robot \d/.test(decide.cap) && /looks and decides/.test(decide.cap) && /average position/.test(decide.cap), decide?.cap);
check("then the walk: the caption says how far it goes", move.length >= 3 && move.every((x) => !x.hidden && /moves/.test(x.cap) && /walks [\d.]+ units/.test(x.cap)), move[0]?.cap);
check("the camera stands still while the robot walks (same zoom, same centre in every frame of a walk)",
  move.length >= 3 && move.every((x, i) => i === 0 || x.turns !== move[i - 1].turns || (x.zoom === move[i - 1].zoom && x.ax === move[i - 1].ax)), JSON.stringify(move.map((x) => [x.turns, +x.zoom.toFixed(4)])));
await setPlayback("20");
await sleep(500);
check("the caption goes away when the run is not played turn by turn", await js("document.getElementById('turn_caption').hidden && !lcm.renderer.options.walk"));

// Late, tiny turns take as long on screen as early ones: play to turn ~19 (the gap has halved that many
// times), then switch to 1 s per turn in mid-run and watch one walk.
await click("#reset");
await js(TWO_ROBOTS);
await until("lcm.state === 'ready' && lcm.renderer.frame && lcm.renderer.frame.robots === 2", 4000);
await setPlayback("turn:0.25");
await click("#play");
await until("lcm.renderer.frame.turns >= 19", 20000);
await setPlayback("turn:1");
const tiny = [];
for (let k = 0; k < 5; k++) { await sleep(150); tiny.push(await js("({ g: Math.abs(lcm.renderer.frame.x(1) - lcm.renderer.frame.x(0)), turns: lcm.renderer.frame.turns, ended: lcm.state === 'ended' })")); }
await click("#play");
const sameTurn = tiny.filter((t, i) => i && t.turns === tiny[i - 1].turns && t.g !== tiny[i - 1].g).length;
check("a late turn, with the robots less than 0.01 apart, still takes its time on screen and glides", tiny[0].g < 0.01 && !tiny[0].ended && sameTurn >= 2, JSON.stringify(tiny));

// Leaving the sequential scheduler takes the per-turn speeds away and falls back to 20×.
await click("#reset");  // a paused run only notes a changed setting; Reset starts the new one, as a user would
await setPlayback("turn:1");
await js("lcm.settings.apply({ scheduler: 'async' })");
await until("lcm.state === 'ready' && lcm.renderer.frame.turns == null && document.getElementById('hud_epochs_row').hidden && !document.querySelector('#playback option[value^=\"turn:\"]')", 4000);
const afterAsync = await js("({ value: document.getElementById('playback').value, turnOptions: document.querySelectorAll('#playback option[value^=\"turn:\"]').length, rowHidden: document.getElementById('hud_epochs_row').hidden })");
check("with the asynchronous scheduler the per-turn speeds go away and playback falls back to 20×", afterAsync.value === "20" && afterAsync.turnOptions === 0 && afterAsync.rowHidden, JSON.stringify(afterAsync));
await click("#reset");
await js("lcm.settings.reset()");
await until("lcm.state === 'ready' && lcm.settings.values.pattern === 'cloud' && lcm.renderer.frame.flags.length === 500", 3000);
await setPlayback(playback);

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

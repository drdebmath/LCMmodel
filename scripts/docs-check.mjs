// End-to-end check of the flowchart page (web/docs.html) in headless Chrome:
// every chart and step, the detail panel and its code, the guided tour,
// links between steps, zoom, deep links, themes, plus screenshots.
//
//   node scripts/docs-check.mjs [screenshot-dir]
//
// Exits 1 if any check fails (see scripts/lib/browser.mjs for requirements).
import { tmpdir } from "node:os";
import { join } from "node:path";
import { openBrowser } from "./lib/browser.mjs";

const { base, send, js, sleep, shot, key, clickAt, until, check, finish, consoleErrors } =
  await openBrowser(process.argv[2] ?? join(tmpdir(), "lcm-docs"));

await send("Page.enable");
await send("Runtime.enable");
await send("Emulation.setDeviceMetricsOverride", { width: 1500, height: 940, deviceScaleFactor: 1, mobile: false });
await send("Emulation.setEmulatedMedia", { features: [{ name: "prefers-color-scheme", value: "light" }] });
await send("Page.navigate", { url: `${base}/web/docs.html` });
check("page loads", await until("!!window.docs", 10000));
await sleep(300);

const charts = await js("Object.keys(docs.DATA.charts)");
check("four charts", charts.length === 4, charts.join(", "));
const simLinks = await js(`[...document.querySelectorAll('a[href="../"]')].filter((a) => a.offsetParent && /simulator/i.test(a.textContent)).map((a) => { const r = a.getBoundingClientRect(); return { x: Math.round(r.x), y: Math.round(r.y) }; })`);
check("one simulator button on a wide screen: the sidebar card (top left), none in the header's top right", simLinks.length === 1
  && simLinks[0].x < 60 && simLinks[0].y < 120, JSON.stringify(simLinks));
await shot("1-event-loop");

// Every step of every chart opens the panel with its own title; code steps show their code.
let steps = 0, withCode = 0;
const problems = [];
for (const id of charts) {
  await js(`docs.showChart(${JSON.stringify(id)})`);
  const order = await js(`docs.DATA.charts[${JSON.stringify(id)}].order`);
  for (const step of order) {
    const r = await js(`(() => { const g = document.querySelector('.chart[data-chart="${id}"] .node[data-id="${step}"]');
      g.dispatchEvent(new MouseEvent("click", { bubbles: true }));
      const n = docs.DATA.charts["${id}"].nodes["${step}"];
      const code = document.getElementById("d_code");
      return { title: document.getElementById("d_title").textContent === n.title, hasCode: !!n.code,
        codeShown: !code.hidden, lines: document.querySelectorAll("#d_src .line").length,
        highlighted: document.querySelectorAll("#d_src .kw, #d_src .fn").length,
        selected: g.classList.contains("selected"), desc: document.getElementById("d_desc").textContent.length }; })()`);
    steps++;
    if (r.hasCode) withCode++;
    if (!r.title || !r.selected || r.desc < 10 || r.hasCode !== r.codeShown || (r.hasCode && (r.lines < 3 || r.highlighted < 1))) {
      problems.push(`${id}/${step} ${JSON.stringify(r)}`);
    }
  }
}
check("every step opens its explanation; code steps show highlighted code", problems.length === 0,
  problems.length ? problems.join(" | ") : `${steps} steps, ${withCode} with code`);

// Tour.
await js("docs.showChart('look-compute-move')");
await js("document.querySelector('.chart[data-chart=\"look-compute-move\"] .tour-start').click()");
check("guided tour starts at step 1", (await js("document.getElementById('d_pos').textContent")) === "1 / 17");
await key("ArrowRight", "ArrowRight", 39);
await key("ArrowRight", "ArrowRight", 39);
check("→ moves the tour forward", (await js("docs.selected")) === "lit" && (await js("document.getElementById('d_pos').textContent")) === "3 / 17");
await key("ArrowLeft", "ArrowLeft", 37);
check("← moves it back", (await js("docs.selected")) === "view");
check("the URL links to the step", (await js("location.hash")) === "#look-compute-move/view");
await sleep(250);
await shot("2-cycle-step-selected");

// Links between steps.
await js("docs.showDetail('byz')");
const links = await js("[...document.querySelectorAll('#d_links [data-go]')].map((b) => b.dataset.go)");
check("a question links to each of its answers", links.includes("byzt") && links.includes("alone"), links.join(", "));
await js("document.querySelector('#d_links [data-go=\"byzt\"]').click()");
check("following a link selects that step", (await js("docs.selected")) === "byzt");

// Hover focus.
const focus = await js(`(() => { const g = document.querySelector('.chart[data-chart="look-compute-move"] .node[data-id="frz"]');
  g.dispatchEvent(new MouseEvent("mouseenter")); const stage = g.closest(".stage");
  return { focusing: stage.classList.contains("focusing"), hot: stage.querySelectorAll(".edge.hot").length }; })()`);
check("hovering a step highlights its connections", focus.focusing && focus.hot === 3, `${focus.hot} connections`);

// Zoom.
await js("docs.showChart('event-loop')");
const z0 = await js("document.querySelector('.chart[data-chart=\"event-loop\"] .zoom-level').textContent");
await js("document.querySelector('.chart[data-chart=\"event-loop\"] [data-zoom=\"in\"]').click()");
const z1 = await js("document.querySelector('.chart[data-chart=\"event-loop\"] .zoom-level').textContent");
await key("f", "KeyF", 70);
const z2 = await js("document.querySelector('.chart[data-chart=\"event-loop\"] .zoom-level').textContent");
check("zoom in and fit", parseInt(z1) > parseInt(z0) && parseInt(z2) <= parseInt(z0), `${z0} → ${z1} → ${z2}`);

// Keyboard chart switching and deep links.
await key("4", "Digit4", 52);
check("number keys switch charts", (await js("docs.chart")) === "browser");
await send("Page.navigate", { url: `${base}/web/docs.html#gathering/c` });
await until("!!window.docs && docs.selected === 'c'", 5000);
check("a deep link opens the chart at that step", (await js("docs.chart")) === "gathering" && (await js("docs.selected")) === "c");
await sleep(250);
await shot("3-gathering-deeplink");

// Dark mode.
await send("Emulation.setEmulatedMedia", { features: [{ name: "prefers-color-scheme", value: "dark" }] });
await send("Page.navigate", { url: `${base}/web/docs.html#browser/draw` });
await until("!!window.docs && docs.selected === 'draw'", 5000);
await sleep(300);
check("follows the system dark theme", (await js("getComputedStyle(document.body).backgroundColor")) === "rgb(14, 16, 20)");
await shot("4-browser-dark");
await send("Page.navigate", { url: `${base}/web/docs.html#look-compute-move/comp` });
await until("!!window.docs && docs.selected === 'comp'", 5000);
await sleep(300);
await shot("5-cycle-dark-code");

await js("document.querySelector('.sim-card').click()");
check("the button opens the simulator at the site root", await until("location.pathname.endsWith('/') && !location.pathname.includes('/web') && !!window.lcm", 8000));

// Addresses: /web/ (the old link) lands on the simulator at the root; /old/ is the original page.
await send("Page.navigate", { url: `${base}/web/#kept` });
check("the old /web/ link lands on the simulator at the root", await until("!!window.lcm && !location.pathname.includes('/web') && location.hash === '#kept'", 8000));
check("no errors in the console", consoleErrors.length === 0, consoleErrors.join(" / "));

// The original page tries Pyodide from a CDN first. Blocking the CDN makes the check
// independent of the internet and tests its fallback, the bundled pyodide/ (found from /old/).
await send("Network.enable");
await send("Network.setBlockedURLs", { urls: ["*cdn.jsdelivr.net*"] });
await send("Page.navigate", { url: `${base}/old/` });
check("/old/ is the original Pyodide simulator and it finishes loading (bundled Pyodide)", await until(
  "document.title === 'Asynchronous Simulator' && typeof pythonRunner !== 'undefined' && !!pythonRunner", 90000));
await finish();

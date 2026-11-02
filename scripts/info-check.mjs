// The settings' info cards (web/explain.js) in headless Chrome: every option
// of every explained setting has a card with a title and at least one
// paragraph; changing a setting opens its card under it, ⓘ opens and closes
// one, only one is open at a time, a card whose setting disappears goes with
// it, turning the cards off is remembered, and the card fits a phone. Also
// the panel's arrangement: the scheduler's sentence, nested sub-options, and
// that only a newly added note in the brief animates.
//   node scripts/info-check.mjs [screenshot-dir]
import { tmpdir } from "node:os";
import { join } from "node:path";
import { openBrowser } from "./lib/browser.mjs";

const OUT = process.argv[2] ?? join(tmpdir(), "lcm-info");
const { base, send, js, sleep, shot, until, check, finish, consoleErrors } = await openBrowser(OUT);
await send("Page.enable");
await send("Runtime.enable");

async function open(width, height, mobile) {
  await send("Emulation.setDeviceMetricsOverride", { width, height, deviceScaleFactor: mobile ? 2 : 1, mobile });
  await send("Page.navigate", { url: `${base}/` });
  await until("!!window.lcm", 10000);
  await js("window.lcm.ready");
  await sleep(300);
}
const change = (key, value) => js(`(() => { const el = document.getElementById('set-${key}');
  if (el.type === 'checkbox') { if (el.checked !== ${JSON.stringify(value)}) el.click(); }
  else { el.value = ${JSON.stringify(value)}; el.dispatchEvent(new Event('change', { bubbles: true })); } })()`);
const openCard = () => js(`(() => { const c = [...document.querySelectorAll('.info-card:not(.leaving)')];
  return c.map((x) => ({ key: x.closest('.field').dataset.key, title: x.querySelector('.info-title').textContent, paras: x.querySelectorAll('p').length })); })()`);

await open(1400, 900, false);
await js("localStorage.removeItem('lcm-info-auto')");

// Every option of every explained setting has a card.
const coverage = await js(`(async () => {
  const { explain, EXPLAINED } = await import('/web/explain.js');
  const { SETTINGS } = await import('/web/settings.js');
  const values = { ...window.lcm.settings.values }, gaps = [];
  for (const s of SETTINGS.filter((x) => EXPLAINED.has(x.key))) {
    const opts = s.kind === 'select' ? (typeof s.options === 'function' ? s.options(values) : s.options).map(([o]) => o)
      : s.kind === 'switch' ? [true, false] : [s.min ?? 0, s.value, s.max ?? s.value];
    for (const o of opts) {
      const c = explain(s.key, { ...values, [s.key]: o }, { key: 'SqGathering', name: 'SqGathering', multiplicities: true });
      if (!c || !c.title || !c.paras?.length || c.paras.some((x) => x.length < 40)) gaps.push(s.key + '=' + o);
    }
  }
  return { keys: EXPLAINED.size, gaps };
})()`);
check("every option of every explained setting has a card with a title and real text", coverage.gaps.length === 0,
  coverage.gaps.length ? coverage.gaps.join(", ") : `${coverage.keys} settings`);
check("each explained setting has an ⓘ button", (await js("document.querySelectorAll('#settings .info-btn').length")) === coverage.keys);

// Changing a setting opens its card, under that setting.
await js("document.querySelector('#tab-model').click()");
await change("multiplicity", "none");
await sleep(500);
let cards = await openCard();
check("changing a setting opens its card under it", cards.length === 1 && cards[0].key === "multiplicity" && cards[0].title === "No multiplicity detection", JSON.stringify(cards));
check("the card says how the setting meets the chosen algorithm", await js("/does not look at stacks/.test(document.querySelector('.info-card .info-tip')?.textContent)"));
await shot("1-card");

// ⓘ opens another card and closes the first; ⓘ again closes it.
await js("document.querySelector('.info-btn[data-info=\"frames\"]').click()");
await sleep(500);
cards = await openCard();
check("ⓘ opens its card; only one card is open", cards.length === 1 && cards[0].key === "frames", JSON.stringify(cards));
await js("document.querySelector('.info-btn[data-info=\"frames\"]').click()");
await sleep(500);
check("ⓘ again closes it", (await openCard()).length === 0);

// A card whose setting is hidden goes with it.
await js("document.querySelector('#tab-model').click()");
await change("frames", "compass");
await sleep(300);
await js("document.querySelector('.info-btn[data-info=\"compass_error\"]').click()");
await sleep(400);
check("a setting shown only for one choice has its card", (await openCard())[0]?.key === "compass_error");
await change("frames", "global");
await sleep(600);
check("its card closes when the setting disappears", !(await openCard()).some((c) => c.key === "compass_error"));

// The text follows a slider without opening a second card.
await js("document.querySelector('#tab-timing').click()");
await change("scheduler", "kasync");
await sleep(400);
await js(`(() => { const r = document.getElementById('set-k_async'); r.value = 3; r.dispatchEvent(new Event('input', { bubbles: true })); r.dispatchEvent(new Event('change', { bubbles: true })); })()`);
await sleep(700);
cards = await openCard();
check("a slider's card follows its value", cards.length === 1 && cards[0].key === "k_async" && cards[0].title.startsWith("k = 3"), JSON.stringify(cards));

// Turning the cards off is remembered; ⓘ still works.
await js("document.querySelector('.info-card .info-auto input').click()");
await open(1400, 900, false);
await js("document.querySelector('#tab-timing').click()");
await change("scheduler", "fsync");
await sleep(500);
check("with the cards turned off, a change opens none (remembered after reload)", (await openCard()).length === 0);
await js("document.querySelector('.info-btn[data-info=\"scheduler\"]').click()");
await sleep(500);
check("…and ⓘ still opens one", (await openCard())[0]?.title.startsWith("Fully synchronous"));
await js("localStorage.removeItem('lcm-info-auto')");

// The arrangement: a sentence under the scheduler, options nested under the choice they belong to.
await open(1400, 900, false);
await js("localStorage.setItem('lcm-info-auto', '0')");
await js("document.querySelector('#tab-timing').click()");
await change("scheduler", "sequential");
await change("turn_gap", "none");
await sleep(300);
check("the scheduler group says in one sentence how robots are timed",
  /^One robot at a time.*the next turn starts at once/.test(await js("document.getElementById('timing-summary').textContent")),
  await js("document.getElementById('timing-summary').textContent"));
check("options of the scheduler are drawn nested under it", await js(`['activation_order', 'turn_gap', 'max_turns'].every((k) =>
  document.querySelector('[data-key="' + k + '"]').classList.contains('sub-option'))`));
check("with no pause between turns, λ is out of the way", await js("document.querySelector('[data-key=\"lambda_rate\"]').hidden"));
await change("scheduler", "kasync");
await sleep(300);
check("a sub-option that appears fades in", await js("document.querySelector('[data-key=\"k_async\"]').classList.contains('appear')"));
// Only a note that a change brings in animates; the others stay still.
await js("document.querySelector('#algorithm').value = 'SqGathering'; document.querySelector('#algorithm').dispatchEvent(new Event('change', { bubbles: true }))");
await sleep(800);
await js("document.querySelector('#tab-model').click()");
await change("multiplicity", "none");
await sleep(900);
const before = await js("[...document.querySelectorAll('#model_notes .brief-notes li')].map((li) => li.className)");
await change("unlimited_visibility", false);
await sleep(900);
const after = await js("[...document.querySelectorAll('#model_notes .brief-notes li')].map((li) => li.className)");
check("a note brought in by a change slides in; notes already shown stay still",
  after.length === before.length + 1 && after.filter((c) => c.includes("fresh")).length === 1, JSON.stringify({ before, after }));
await js("localStorage.removeItem('lcm-info-auto')");

// A phone: the card fits the settings sheet.
await open(390, 844, true);
await js("document.getElementById('toggle_settings')?.click()");
await sleep(300);
await js("document.querySelector('#tab-model').click()");
await change("rigid_movement", false);
await sleep(700);
const fit = await js(`(() => { const c = document.querySelector('.info-card'); if (!c) return null; const b = c.getBoundingClientRect();
  return { inside: b.left >= 0 && b.right <= innerWidth, sideways: document.documentElement.scrollWidth > innerWidth }; })()`);
check("phone: the card fits, nothing scrolls sideways", fit && fit.inside && !fit.sideways, JSON.stringify(fit));
await shot("2-phone");

check("no console errors", consoleErrors.length === 0, consoleErrors.join(" | "));
await finish();

# What's new on these branches

Three branches on the fork, each built on the one before: `extended-models` → `extras` → `ui-clean`
(and `overnight`, small fixes on top). Every feature is a `SimConfig` field, so it works the same in
the browser, in `lcm run` and in tests, and the defaults leave every earlier run unchanged (the
Python parity fixtures still replay event by event). Open the page with `node scripts/serve.mjs`,
then <http://localhost:8000/>.

## Schedulers and robot models (`extended-models`, `extras`)

| Feature | Try it on the page | Details |
|---|---|---|
| **SSYNC and FSYNC** | Settings → Scheduler → SSYNC / FSYNC. Gathering → Examples → *FSYNC: one round* | [extended-models §1](extended-models.md) |
| **k-ASYNC** (at most k Looks of others during one robot's cycle) | Settings → Scheduler → k-ASYNC, then *k (Looks per cycle)* | [extras](extras.md) |
| **Multiplicity detection** none / local / weak / strong | Settings → Robot model · advanced → Multiplicity detection | [extended-models](extended-models.md) |
| **Private frames** (rotated, disoriented) and **unreliable compasses** (error φ, static or changing at every Look) | Settings → Robot model · advanced → Coordinate frames | [extended-models](extended-models.md), [extras](extras.md) |
| **Obstructed visibility** (robots block the view; robot radius) | Settings → Visibility → Robots block the view. Gathering → Examples → *Robots block the view* | [extended-models](extended-models.md) |

## Algorithms

All under **Algorithm** in the toolbar, grouped. Each has **Examples** (toolbar) that load a start and play it.

| Algorithm | Where | What to watch |
|---|---|---|
| **Spreading**, **TwoTask** (ported from Python) | Original simulator | TwoTask never ends by itself (a note says why) |
| **SqGathering** (Algorithm 8) | Sequential gathering | Examples play Lemma 8 (SSYNC never gathers) and Theorem 4; step-by-step view at `/web/sqgathering.html` |
| **Two robots, unreliable compasses** (Izumi et al.: A_SS, A_SD, A_AD) | Advanced gathering… → Variant | Red needles show each robot's north. A_AD at 29° is slow, not wrong ([extras, Findings](extras.md)) |
| **Randomized gathering** (Pattanayak et al., Theorems 12 and 3) | Advanced gathering… → Variant | Only the coins tell mirror-image robots apart |
| **Gathering despite crashes** (Gather_k, weak multiplicity) | Advanced gathering… → Variant | The status says how many terminated; crashed robots stay put |
| **Mutual visibility** (Di Luna et al. §6.7) | Other tasks | The end says how many robots see every other robot |
| **Gathering on a ring** (GatheRRing, Navarra & Piselli) | Other tasks → *Gathering on a ring ↗*, or `/web/ring.html` | Click nodes to place robots; the unsolvable starts are flagged; CSV of every turn. CLI: `lcm ring config.json --trace` with `{"nodes": 9, "robots": [0, 2, 5]}` |

Picking an advanced variant locks the settings its paper fixes (two robots, SSYNC, weak detection…),
each with its reason under a lock icon, and the notes bar under the toolbar says where your settings leave the
paper's model.

## On the page (`extended-models`, `ui-clean`)

| Tool | How |
|---|---|
| **Inspector** | Click a robot. Under *If it looked now*: what it sees and the rule it would apply (SqGathering's κ and b, the compass sector, the crash-gathering destination) |
| **Play the adversary** | Toolbar hand button or `A`, sequential or SSYNC: you pick who moves next (and where a move stops) |
| **Scenario editor** | Toolbar pencil or `D`: place, move, stack and remove robots by hand |
| **Scenarios** | Export menu: save, open, or copy a link that opens the exact scenario |
| **Trails, pulses, halo** | `P` toggles trails; Looks pulse, the robot about to act has a halo, targets are marked |
| **Phone and tablet** | The layout adapts to phone and tablet widths (checked by `responsive-check.mjs`) |
| **Info cards** | The ⓘ next to a setting (scheduler, multiplicity, frames, visibility, faults, δ, …) opens a card that explains in plain words what the choice means for the robots and when to pick it; it opens by itself when you change the setting (switch off in the card). The multiplicity card says what the selected algorithm needs |
| **Settings arrangement** | Options that only matter for one choice sit indented under it and appear with a short animation; the Timing tab starts with one sentence saying how the chosen scheduler runs; λ, sampling rate and precision are grouped under *Clock and precision* |
| **Notes** | The algorithm brief's notes are rewritten as full explanations, tagged *Outside what it was built for*, *Untested ground* or *Simplified*; new notes slide in when a setting changes |
| **Ring page** | A card says whose turn it is, what that robot sees and where it will step; a bar shows rounds used against the most needed; the gaps are listed; *How the robots decide* explains the rules; the next robot has a halo and the step arrow draws itself |

## Checks (GitHub Actions on every push to the fork)

`cargo test --workspace`; native = Wasm bit for bit (`check-wasm-parity.mjs`); flowcharts match the
code; headless Chrome on every page: `ui-check.mjs`, `responsive-check.mjs`, `sqgathering-check.mjs`,
`docs-check.mjs`, `info-check.mjs` (every setting has an info card), `embed-check.mjs`, and
`extras-check.mjs` (the ring page, every Advanced-gathering and Mutual-visibility example, the
inspector); every algorithm under every scheduler, rigid and not (`scheduler-matrix.mjs`).

## Known limits

- The SSYNC/FSYNC work left Gathering about 7% slower than `main` on a 2,000-robot run with identical
  results; the hot code is unchanged, so it looks like code layout. Not fixed.
- Crash gathering's arcs are approximated by straight steps; mutual visibility never terminates (as in
  the paper without lights). Both are stated on the page and in [extras](extras.md).

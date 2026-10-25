# AGENTS.md: contributing to the LCM simulator

For people and coding agents. [`README.md`](README.md) explains what the project is and how the Rust code is laid out; [`docs/core-schema.md`](docs/core-schema.md) is the contract of the simulation core. Read the schema section for whatever you change before changing it.

## Ground rules

- **The core is deterministic.** The same `SimConfig` and seed must give the same run on every machine, native and WebAssembly. Randomness comes only from `lcm_core::Rng`. Never use the system clock, `HashMap` iteration order, threads or any other source of nondeterminism in `lcm-core` or `lcm-algorithms`.
- **Order of operations is part of the behaviour.** Floating-point results and the order of random draws are checked against recorded runs. Don't "tidy" arithmetic, reorder draws or swap a sum for a fold in code that has parity tests; use the helpers in `lcm_core::pyfloat` where they exist.
- **Robots are oblivious.** An algorithm's `compute` may use only the `Look` it is given, the model constants, the `Rng` and the `Scratch` buffers. Nothing in `Scratch` may carry meaning from one look to the next.
- **The core has no I/O.** No DOM, files, threads or GPU in `lcm-core`/`lcm-algorithms`. `lcm-wasm` and `lcm-cli` are thin adapters and hold no simulation logic.
- **`unsafe` is forbidden** (workspace lint). New dependencies need a good reason; the page ships about 350 KB and that matters.
- Keep comments and naming in the style of the file you are editing. Doc comments say *why* and point to the schema section; don't narrate what the code does.

## Setup

Rust (stable), Node ≥ 22, Chrome (for the page checks), and for the WebAssembly build:

```sh
rustup target add wasm32-unknown-unknown
cargo install wasm-bindgen-cli --version 0.2.129 --locked   # must match the wasm-bindgen pin in Cargo.toml
```

## The loop

| You changed | Run |
| --- | --- |
| Rust code | `cargo test --workspace` (and `cargo clippy --workspace --all-targets`; warnings are reported, not enforced, but don't add new ones) |
| Anything that affects the browser build (any Rust in `lcm-core`, `lcm-algorithms`, `lcm-wasm`) | `cargo build --release -p lcm-cli && ./scripts/build-wasm.sh`, then `node scripts/check-wasm-parity.mjs` and `node scripts/check-seq-parity.mjs` |
| A function shown in a flowchart, or a chart definition | `node scripts/gen-flowcharts.mjs` (writes `docs/flowcharts.md` and `web/docs.html`) |
| Files in `web/` | `node scripts/serve.mjs`, then the matching page checks below |

`web/pkg/` is **committed** and CI fails if it doesn't match the source, so rebuild it and commit it with any Rust change that reaches the browser. Never edit `web/pkg/` or the generated `docs/flowcharts.md` / `web/docs.html` by hand.

Page checks (headless Chrome; each takes an optional screenshot directory): `ui-check.mjs` (simulator page; its 10,000-robot frame-rate targets can miss on slow machines), `docs-check.mjs` (flowchart page), `responsive-check.mjs` (phone/tablet/desktop), `sqgathering-check.mjs`, `embed-check.mjs` (links and `<iframe>` embedding). Run the ones for the pages you touched; CI runs all of them.

## Where things go

- **A new setting:** a field in `SimConfig` (`crates/lcm-core/src/config.rs`) with a default and a check in `validate()`; document it in `docs/core-schema.md` (§2); add one entry in `web/settings.js` (the panel and the config are built from that list). If it should be settable from a link, check `web/embed.js`.
- **A new algorithm:** a module in `crates/lcm-algorithms/src/` implementing `lcm_core::Algorithm`, one entry in `REGISTRY` in `lib.rs` (key, display name, builder; refuse unsupported settings with `ConfigError::Unsupported`; add it to `requirements()` if it needs the sequential scheduler or works with multiplicities). Add a test file in `crates/lcm-algorithms/tests/` with hand-worked cases, plus a flowchart in `scripts/gen-flowcharts.mjs` if it should appear on the flowchart page. Then rebuild `web/pkg/`.
- **Scheduler or event-loop behaviour:** `crates/lcm-core/src/sim.rs`, with tests in `crates/lcm-algorithms/tests/` (`sequential*.rs`, `pacing.rs`). Update `docs/core-schema.md` (§5, §9 for deliberate differences).
- **What the page draws:** the core fills a `Frame` (`frame.rs`); `lcm-wasm` exposes it; `web/worker.js` and `web/client.js` move it; `web/renderer.js` draws it. A new per-robot array has to be added at each of those steps.
- **A new page check:** a `scripts/*-check.mjs` using `scripts/lib/browser.mjs`; print a line starting with `FAIL` for each failure and exit non-zero (`scripts/ci-check.sh` turns those into CI annotations), then add it to `.github/workflows/ci.yml`.

## Tests

- **Unit tests** live next to the code; **integration tests** in each crate's `tests/`. Prefer small hand-worked cases (known inputs, destinations worked out by hand) over snapshot comparisons.
- **Parity.** `crates/lcm-algorithms/tests/parity.rs` replays the recorded runs in `fixtures/` (made by the original Python implementation, which is no longer in the repository) and requires identical events, outcomes and positions. Don't edit those fixtures to make a test pass; if a change legitimately alters behaviour, say so in the PR and update `docs/core-schema.md` §9. New algorithms and the sequential scheduler have no recorded originals, so test them against hand-worked cases and brute-force reimplementations instead.
- **Browser parity.** `check-wasm-parity.mjs` runs the fixtures through the WebAssembly build and the native CLI and requires bit-identical results.
- A bug fix comes with a test that fails without it.

## Pull requests

- Branch from `main`; keep a change focused (one feature or fix, plus its tests and docs).
- Before pushing, run the rows of "The loop" that apply. CI runs everything (`.github/workflows/ci.yml`): Rust tests, committed-vs-rebuilt `web/pkg/` parity, flowchart freshness and the Chrome page checks.
- Commit messages: a short imperative summary line, then why if it isn't obvious.
- Update the docs you made stale: `README.md` (user-facing behaviour, layout), `docs/core-schema.md` (core behaviour), the flowcharts (regenerate, don't edit).
- Don't commit build output other than `web/pkg/` and the two generated flowchart files, and nothing from `target/`.

## Useful commands

```sh
cargo run --release -p lcm-cli -- run config.json --trace trace.jsonl   # one run, with a trace
cargo run --release -p lcm-cli -- seq --robots 6 --seed 7 --trace -     # gathering under the sequential scheduler, turn by turn
cargo run --release -p lcm-cli -- bench --robots 10000 --seconds 5       # events per second on this machine
cargo test -p lcm-algorithms --test parity                               # just the recorded-run replays
node scripts/serve.mjs 8080                                              # dev server, caching off
```

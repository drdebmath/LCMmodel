#!/usr/bin/env bash
# Build the browser package: crates/lcm-wasm -> web/pkg/ (lcm_wasm.js + lcm_wasm_bg.wasm).
# Needs: rustup target add wasm32-unknown-unknown
#        cargo install wasm-bindgen-cli --version 0.2.129 --locked
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo"
# simd128: WebAssembly SIMD (Chrome 91+, Firefox 89+, Safari 16.4+).
# remap: keep the build machine's paths out of the shipped binary.
export RUSTFLAGS="${RUSTFLAGS:-} -C target-feature=+simd128 --remap-path-prefix=$repo=/lcm --remap-path-prefix=${CARGO_HOME:-$HOME/.cargo}=/cargo"
cargo build -p lcm-wasm --target wasm32-unknown-unknown --release
wasm-bindgen "${CARGO_TARGET_DIR:-target}/wasm32-unknown-unknown/release/lcm_wasm.wasm" \
  --target web --out-dir web/pkg --no-typescript
# Build id from the package's contents. The page loads the worker and the
# package with ?v=<id>, so a browser can never pair new page code with a
# cached old core (or the other way round).
id=$( (cat web/pkg/lcm_wasm_bg.wasm web/pkg/lcm_wasm.js) | { sha256sum 2>/dev/null || shasum -a 256; } | cut -c1-16)
printf '{"id":"%s"}\n' "$id" > web/pkg/build.json
ls -l web/pkg

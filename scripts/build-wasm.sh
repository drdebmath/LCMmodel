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
wasm-bindgen target/wasm32-unknown-unknown/release/lcm_wasm.wasm \
  --target web --out-dir web/pkg --no-typescript
ls -l web/pkg

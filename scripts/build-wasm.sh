#!/usr/bin/env bash
# Build the browser packages:
#   crates/lcm-wasm      -> web/pkg/      (lcm_wasm.js + lcm_wasm_bg.wasm), the simulator;
#   crates/lcm-ring-wasm -> web/ring-pkg/ (lcm_ring_wasm.js + lcm_ring_wasm_bg.wasm), only web/ring.html.
# Needs: rustup target add wasm32-unknown-unknown
#        cargo install wasm-bindgen-cli --version 0.2.129 --locked
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo"
# simd128: WebAssembly SIMD (Chrome 91+, Firefox 89+, Safari 16.4+).
# remap: keep the build machine's paths out of the shipped binary.
export RUSTFLAGS="${RUSTFLAGS:-} -C target-feature=+simd128 --remap-path-prefix=$repo=/lcm --remap-path-prefix=${CARGO_HOME:-$HOME/.cargo}=/cargo"
# build <crate> <out dir>: the crate's cdylib -> <out dir>/<crate>.js + <crate>_bg.wasm, plus
# <out dir>/build.json. The build id comes from the package's contents: a page loads its package
# (and the simulator its worker) with ?v=<id>, so a browser can never pair new page code with a
# cached old core (or the other way round).
build() {
  local crate="$1" out="$2" name="${1//-/_}"
  cargo build -p "$crate" --target wasm32-unknown-unknown --release
  wasm-bindgen "${CARGO_TARGET_DIR:-target}/wasm32-unknown-unknown/release/$name.wasm" \
    --target web --out-dir "$out" --no-typescript --remove-name-section
  id=$( (cat "$out/${name}_bg.wasm" "$out/$name.js") | { sha256sum 2>/dev/null || shasum -a 256; } | cut -c1-16)
  printf '{"id":"%s"}\n' "$id" > "$out/build.json"
  ls -l "$out"
}
build lcm-wasm web/pkg
build lcm-ring-wasm web/ring-pkg

//! Browser bindings for `lcm-ring` (GatheRRing on a ring of nodes), for
//! `web/ring.html` only.
//!
//! A separate package (`web/ring-pkg/`) so that the simulator page, which
//! loads `web/pkg/`, never downloads the ring code. A thin adapter: it parses
//! the configuration and returns the run as JSON. No ring logic lives here.

use wasm_bindgen::prelude::*;

/// A ring run, stepped by the page one activation at a time.
#[wasm_bindgen]
pub struct WasmRing {
    sim: lcm_ring::RingSim,
}

#[wasm_bindgen]
impl WasmRing {
    /// `config_json` is a `RingConfig`.
    ///
    /// # Errors
    /// Malformed JSON or an invalid ring.
    #[wasm_bindgen(constructor)]
    pub fn new(config_json: &str) -> Result<WasmRing, JsError> {
        let config: lcm_ring::RingConfig = serde_json::from_str(config_json)?;
        let sim = lcm_ring::RingSim::new(&config).map_err(|e| JsError::new(&e))?;
        Ok(Self { sim })
    }

    /// One activation as JSON (`null` once the run is over).
    pub fn step(&mut self) -> String {
        self.sim.step().map_or_else(
            || "null".to_owned(),
            |t| serde_json::to_string(&t).unwrap_or_default(),
        )
    }

    /// The run now, as JSON: positions, status, counts, the task every robot
    /// would take, the next robot and what it would do.
    #[must_use]
    pub fn state(&self) -> String {
        let view = self.sim.view();
        let next = self.sim.next_robot();
        let (task, mv, rule) = lcm_ring::decide(&view, self.sim.positions[next]);
        serde_json::json!({
            "nodes": self.sim.config.nodes,
            "positions": self.sim.positions,
            "status": self.sim.status,
            "epochs": self.sim.epochs,
            "epoch": self.sim.epoch(),
            "turns": self.sim.turns,
            "next": next,
            "task": task,
            "rule": rule,
            "move": match mv {
                lcm_ring::Move::Stay => 0,
                lcm_ring::Move::Step(d) => i32::from(d),
                lcm_ring::Move::Either => 2,
            },
            "holes": view.holes,
            "bound": self.sim.config.nodes.saturating_sub(3),
            "unsolvable": lcm_ring::unsolvable(&self.sim.config),
        })
        .to_string()
    }
}

//! LCM algorithms and the registry that names them.
//!
//! Each algorithm is a port of the matching `Robot._<name>` method pair
//! (compute + terminal check) in `robot.py`, keeping its operation order.
//! Adding an algorithm: implement [`Algorithm`] in a new module and add one
//! line to [`REGISTRY`]. An entry builds its [`Plan`] from the run's
//! [`SimConfig`], so an algorithm can use the seed, the robot count and the
//! other settings (most ignore them).
//!
//! Gathering and SEC are ported; they drive the core's parity tests.

mod gathering;
mod sec;

pub use gathering::Gathering;
pub use sec::Sec;

use lcm_core::{Algorithm, ConfigError, Plan, SimConfig, Simulation};

/// A selectable entry: key (as sent by the UI), display name, plan builder.
pub struct Entry {
    pub key: &'static str,
    pub name: &'static str,
    pub build: fn(&SimConfig) -> Plan,
}

fn one<A: Algorithm + Default + 'static>(_: &SimConfig) -> Plan {
    Plan::single(Box::new(A::default()))
}

pub const REGISTRY: &[Entry] = &[
    Entry {
        key: "Gathering",
        name: "Center of Gravity",
        build: one::<Gathering>,
    },
    Entry {
        key: "SEC",
        name: "Smallest Enclosing Circle",
        build: one::<Sec>,
    },
];

#[must_use]
pub fn plan(key: &str, config: &SimConfig) -> Option<Plan> {
    REGISTRY
        .iter()
        .find(|e| e.key == key)
        .map(|e| (e.build)(config))
}

/// Builds a simulation for `config.algorithm`.
///
/// # Errors
/// An unknown algorithm or an invalid configuration.
pub fn simulation(config: &SimConfig) -> Result<Simulation, ConfigError> {
    let plan = plan(&config.algorithm, config)
        .ok_or_else(|| ConfigError::UnknownAlgorithm(config.algorithm.clone()))?;
    Simulation::new(config, plan)
}

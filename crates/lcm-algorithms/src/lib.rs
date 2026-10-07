//! LCM algorithms and the registry that names them.
//!
//! Each algorithm is a port of the matching `Robot._<name>` method pair
//! (compute + terminal check) in `robot.py`, keeping its operation order.
//! Adding an algorithm: implement [`Algorithm`] in a new module and add one
//! line to [`REGISTRY`]. An entry builds its [`Plan`] from the run's
//! [`SimConfig`], so an algorithm can use the seed, the robot count and the
//! other settings (most ignore them), and it can refuse settings it cannot
//! work with by returning [`ConfigError::Unsupported`].
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
    pub build: fn(&SimConfig) -> Result<Plan, ConfigError>,
}

fn one<A: Algorithm + Default + 'static>(_: &SimConfig) -> Result<Plan, ConfigError> {
    Ok(Plan::single(Box::new(A::default())))
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

/// The plan of algorithm `key` for a run with this `config`.
///
/// # Errors
/// An unknown algorithm, or settings that algorithm cannot work with.
pub fn plan(key: &str, config: &SimConfig) -> Result<Plan, ConfigError> {
    let entry = REGISTRY
        .iter()
        .find(|e| e.key == key)
        .ok_or_else(|| ConfigError::UnknownAlgorithm(key.to_owned()))?;
    (entry.build)(config)
}

/// Builds a simulation for `config.algorithm`.
///
/// # Errors
/// An unknown algorithm, settings that algorithm cannot work with, or an
/// invalid configuration.
pub fn simulation(config: &SimConfig) -> Result<Simulation, ConfigError> {
    let plan = plan(&config.algorithm, config)?;
    Simulation::new(config, plan)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn needs_sequential(config: &SimConfig) -> Result<Plan, ConfigError> {
        if config.scheduler == lcm_core::SchedulerKind::Sequential {
            one::<Gathering>(config)
        } else {
            Err(ConfigError::Unsupported(
                "this algorithm needs the sequential scheduler".to_owned(),
            ))
        }
    }

    #[test]
    fn an_entry_can_refuse_settings_it_cannot_work_with() {
        let entry = Entry {
            key: "Picky",
            name: "Needs the sequential scheduler",
            build: needs_sequential,
        };
        let asynchronous = SimConfig::default();
        let message = (entry.build)(&asynchronous).err().unwrap().to_string();
        assert_eq!(message, "this algorithm needs the sequential scheduler");
        let sequential = SimConfig {
            scheduler: lcm_core::SchedulerKind::Sequential,
            ..SimConfig::default()
        };
        assert!((entry.build)(&sequential).is_ok());
    }
}

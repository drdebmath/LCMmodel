//! LCM algorithms and the registry that names them.
//!
//! Each algorithm is a port of the matching `Robot._<name>` method pair
//! (compute + terminal check) in the original simulator, keeping its operation order.
//! Adding an algorithm: implement [`Algorithm`] in a new module and add one
//! line to [`REGISTRY`]. An entry builds its [`Plan`] from the run's
//! [`SimConfig`], so an algorithm can use the seed, the robot count and the
//! other settings (most ignore them), and it can refuse settings it cannot
//! work with by returning [`ConfigError::Unsupported`].
//!
//! Gathering and SEC are ported; they drive the core's parity tests. Uniform
//! Circle Formation, Pattern Formation and Go-To-Center are ported too. SqGathering
//! (Algorithm 8 of the sequential-schedulers paper) and Sequential Pattern
//! Formation (its Algorithms 1 and 7, SqPF / SqPF′) are new: they refuse any
//! scheduler but the sequential one, and SqPF runs its robots in the paper's model
//! (private coordinate systems, no identities, no multiplicity detection).

mod circle_formation;
mod gathering;
mod go_to_center;
mod pattern_formation;
mod sec;
pub mod sq_gathering;
pub mod sq_pattern_formation;
pub mod sq_trace;

pub use circle_formation::CircleFormation;
pub use gathering::Gathering;
pub use go_to_center::GoToCenter;
pub use pattern_formation::{template as pattern_template, PatternFormation};
pub use sec::Sec;
pub use sq_gathering::SqGathering;
pub use sq_pattern_formation::SqPatternFormation;

use lcm_core::{Algorithm, ConfigError, Plan, Point, SimConfig, Simulation};

/// A distance or angle for an explanation: four significant digits, or
/// scientific notation when tiny.
pub(crate) fn num(x: f64) -> String {
    if x == 0.0 {
        "0".to_owned()
    } else if x.abs() >= 1e-3 && x.abs() < 1e7 {
        let digits = (3 - libm::floor(libm::log10(x.abs())) as i32).max(0);
        let text = format!("{x:.prec$}", prec = usize::try_from(digits).unwrap_or(0));
        if text.contains('.') {
            text.trim_end_matches('0').trim_end_matches('.').to_owned()
        } else {
            text
        }
    } else {
        format!("{x:.2e}")
    }
}

/// A selectable entry: key (as sent by the UI), display name, plan builder.
pub struct Entry {
    pub key: &'static str,
    pub name: &'static str,
    pub build: fn(&SimConfig) -> Result<Plan, ConfigError>,
}

fn one<A: Algorithm + Default + 'static>(_: &SimConfig) -> Result<Plan, ConfigError> {
    Ok(Plan::single(Box::new(A::default())))
}

/// Pattern Formation reads which predefined shape to form from the config, so
/// one algorithm entry covers every shape in [`PATTERNS`].
fn pattern_formation(config: &SimConfig) -> Result<Plan, ConfigError> {
    // This port forms one robot per point (SqPF, below, allows more robots).
    if let Some(points) = &config.target_points {
        if points.len() != config.num_of_robots as usize {
            return Err(ConfigError::Unsupported(format!(
                "target_points must give one point per robot ({}), got {}",
                config.num_of_robots,
                points.len()
            )));
        }
    }
    Ok(Plan::single(Box::new(PatternFormation {
        shape: config.target_pattern,
        custom: config
            .target_points
            .as_ref()
            .map(|pts| pts.iter().map(|p| Point::new(p[0], p[1])).collect()),
    })))
}

/// The predefined Pattern Formation shapes offered to the UI, as
/// `(key, display name)`. The key is the lowercase `PatternShape` name the
/// frontend sends back in `target_pattern`.
pub const PATTERNS: &[(&str, &str)] = &[
    ("star", "Star"),
    ("circle", "Circle"),
    ("square", "Square"),
    ("triangle", "Triangle"),
    ("pentagon", "Pentagon"),
    ("hexagon", "Hexagon"),
    ("line", "Line"),
    ("cross", "Cross"),
    ("heart", "Heart"),
];

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
    Entry {
        key: "CircleFormation",
        name: "Uniform Circle Formation",
        build: one::<CircleFormation>,
    },
    Entry {
        key: "PatternFormation",
        name: "Pattern Formation (robot.py)",
        build: pattern_formation,
    },
    Entry {
        key: "SqPatternFormation",
        name: "Sequential Pattern Formation",
        build: sq_pattern_formation::plan,
    },
    Entry {
        key: "GoToCenter",
        name: "Go To Center",
        build: one::<GoToCenter>,
    },
    Entry {
        key: "SqGathering",
        name: "SqGathering",
        build: sq_gathering::plan,
    },
];

/// What the page should set up before running algorithm `key`, as
/// `(needs the sequential scheduler, works with multiplicity points)`.
#[must_use]
pub fn requirements(key: &str) -> (bool, bool) {
    match key {
        "SqGathering" | "SqPatternFormation" => (true, true),
        _ => (false, false),
    }
}

/// `true` when algorithm `key` runs only with non-rigid movement and a
/// minimum move δ > 0 (the model of the paper it comes from).
#[must_use]
pub fn needs_non_rigid(key: &str) -> bool {
    key == "SqPatternFormation"
}

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

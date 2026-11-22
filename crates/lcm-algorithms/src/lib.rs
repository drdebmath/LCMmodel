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
//! Circle Formation, Go-To-Center, Spreading and TwoTask are ported too.
//! SqGathering (Algorithm 8 of the sequential-schedulers paper) is new: it
//! refuses the asynchronous scheduler.

mod circle_formation;
pub mod compass_gathering;
pub mod crash_gathering;
pub mod mutual_visibility;
pub mod randomized_gathering;
mod gathering;
mod go_to_center;
mod sec;
mod spreading;
pub mod sq_gathering;
pub mod sq_trace;

pub use circle_formation::CircleFormation;
pub use gathering::Gathering;
pub use go_to_center::GoToCenter;
pub use sec::Sec;
pub use spreading::Spreading;
pub use sq_gathering::SqGathering;

use lcm_core::{
    Algorithm, Assignment, ConfigError, Plan, SchedulerKind, SimConfig, Simulation, TaskColor,
};

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

/// TwoTask (the original Python scheduler's draw): each robot draws `u` in ID order; `u < 0.5`
/// runs SEC and shows red, the others run Gathering and show blue.
#[allow(clippy::unnecessary_wraps)]
fn two_task(_: &SimConfig) -> Result<Plan, ConfigError> {
    Ok(Plan {
        algorithms: vec![Box::new(Sec), Box::new(Gathering)],
        assignment: Assignment::Random(vec![
            (0.5, 0, Some(TaskColor::Red)),
            (1.0, 1, Some(TaskColor::Blue)),
        ]),
    })
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
    Entry {
        key: "CircleFormation",
        name: "Uniform Circle Formation",
        build: one::<CircleFormation>,
    },
    Entry {
        key: "GoToCenter",
        name: "Go To Center",
        build: one::<GoToCenter>,
    },
    Entry {
        key: "Spreading",
        name: "Spreading",
        build: one::<Spreading>,
    },
    Entry {
        key: "TwoTask",
        name: "Two Tasks (SEC + Gathering)",
        build: two_task,
    },
    Entry {
        key: "CompassGathering",
        name: "Two robots, unreliable compasses",
        build: compass_gathering::plan,
    },
    Entry {
        key: "RandomizedGathering",
        name: "Randomized gathering (two robots)",
        build: randomized_gathering::plan,
    },
    Entry {
        key: "RandomizedGatheringUniform",
        name: "Randomized gathering, λ ∈ {1, ½, uniform} (two robots)",
        build: randomized_gathering::plan_uniform,
    },
    Entry {
        key: "MutualVisibility",
        name: "Mutual visibility",
        build: mutual_visibility::plan,
    },
    Entry {
        key: "CrashGathering",
        name: "Gathering despite crashes (weak multiplicity)",
        build: crash_gathering::plan,
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
        "SqGathering" | "CrashGathering" => (true, true),
        _ => (false, false),
    }
}

/// Where a run's settings leave the model algorithm `key` was designed or
/// proved for, as sentences the page can show. Empty inside the model.
#[must_use]
pub fn notes(key: &str, config: &SimConfig) -> Vec<String> {
    match key {
        "SqGathering" => sq_gathering::notes(config),
        "CompassGathering" => compass_gathering::notes(config),
        "CrashGathering" => crash_gathering::notes(config),
        "MutualVisibility" => mutual_visibility::notes(config),
        "RandomizedGathering" | "RandomizedGatheringUniform" => randomized_gathering::notes(config),
        "TwoTask" => vec![
            "This run never ends by itself. Half of the robots (red) follow the smallest-enclosing-circle rule and head for the edge of the circle around the swarm, while the other half (blue) follow the centre-of-gravity rule and head for its middle. The two groups keep pulling the swarm in different directions, so the run goes on until it stalls or reaches a limit, exactly as in the original simulator.".to_owned(),
        ],
        "CircleFormation" if config.scheduler == SchedulerKind::Fsync => vec![
            "With every robot moving at once (FSYNC), the circle each robot aims for moves along with the robots, because it is computed from their positions, so some starts never settle: in testing, 4 out of 12 starts with 10 or 30 robots were still moving at the event limit. Under the semi-synchronous, sequential and asynchronous schedulers the robots settle into an evenly spaced circle.".to_owned(),
        ],
        _ => Vec::new(),
    }
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

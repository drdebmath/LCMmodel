//! Simulation input (docs/core-schema.md §2). Field names match the `params()`
//! object `main.js` sends, so existing UI state maps across unchanged.

use crate::geom::Point;
use crate::start::{self, StartSpec};
use core::fmt;

pub const MAX_ROBOTS: u32 = 100_000;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "lowercase"))]
pub enum FaultSelection {
    Crash,
    Byzantine,
    Omission,
    Delay,
    Mixed,
}

/// How robots are activated.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "lowercase"))]
pub enum SchedulerKind {
    /// Every robot on its own exponential clock; cycles overlap (the original).
    #[default]
    Async,
    /// One robot at a time does a whole Look-Compute-Move; the rest stay still.
    Sequential,
}

/// Sequential scheduler: who takes the next turn. Every epoch gives each
/// robot that is neither crashed nor terminated exactly one turn.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum ActivationOrder {
    /// 0, 1, …, n-1, then 0 again.
    #[default]
    RoundRobin,
    /// A fresh shuffle each epoch, drawn from the run's seed.
    Random,
}

/// Sequential scheduler: sim time between one turn ending and the next Look.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "lowercase"))]
pub enum TurnGap {
    /// Exponential with rate `lambda_rate`, like an async activation.
    #[default]
    Random,
    /// The next robot looks the instant the previous one stops.
    None,
}

#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default, deny_unknown_fields))]
pub struct SimConfig {
    pub algorithm: String,
    pub num_of_robots: u32,
    pub initial_positions: Option<Vec<[f64; 2]>>,
    pub robot_speeds: f64,
    pub visibility_radius: Option<f64>,
    pub num_of_faults: u32,
    pub fault_type: FaultSelection,
    pub rigid_movement: bool,
    pub width_bound: Option<f64>,
    pub height_bound: Option<f64>,
    pub lambda_rate: f64,
    pub sampling_rate: f64,
    pub threshold_precision: u8,
    pub random_seed: u64,
    pub multiplicity_detection: bool,
    pub max_events: Option<u64>,
    pub max_time: Option<f64>,
    /// No world box: algorithms get no bounds. Off reproduces the original.
    pub open_world: bool,
    /// Generated start (pattern + arrangement + distribution). `None` keeps the
    /// original start: `initial_positions` if given, else uniform in the box.
    pub start: Option<StartSpec>,
    pub scheduler: SchedulerKind,
    /// Sequential only.
    pub activation_order: ActivationOrder,
    /// Sequential only.
    pub turn_gap: TurnGap,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            algorithm: "Gathering".to_owned(),
            num_of_robots: 5,
            initial_positions: None,
            robot_speeds: 1.0,
            visibility_radius: None,
            num_of_faults: 0,
            fault_type: FaultSelection::Crash,
            rigid_movement: true,
            width_bound: Some(600.0),
            height_bound: Some(600.0),
            lambda_rate: 5.0,
            sampling_rate: 0.1,
            threshold_precision: 5,
            random_seed: 12345,
            multiplicity_detection: false,
            max_events: None,
            max_time: None,
            open_world: false,
            start: None,
            scheduler: SchedulerKind::Async,
            activation_order: ActivationOrder::RoundRobin,
            turn_gap: TurnGap::Random,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum ConfigError {
    UnknownAlgorithm(String),
    RobotCount(u32),
    NotPositive { field: &'static str, value: f64 },
    Precision(u8),
    Position { index: usize },
    Start(String),
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnknownAlgorithm(name) => write!(f, "unknown algorithm {name:?}"),
            Self::RobotCount(n) => write!(f, "num_of_robots must be 1..={MAX_ROBOTS}, got {n}"),
            Self::NotPositive { field, value } => {
                write!(f, "{field} must be a positive finite number, got {value}")
            }
            Self::Precision(p) => write!(f, "threshold_precision must be 1..=15, got {p}"),
            Self::Position { index } => write!(f, "initial_positions[{index}] is not finite"),
            Self::Start(message) => f.write_str(message),
        }
    }
}

impl std::error::Error for ConfigError {}

fn positive(field: &'static str, value: f64) -> Result<(), ConfigError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(ConfigError::NotPositive { field, value })
    }
}

impl SimConfig {
    /// Checks every field; algorithm names are checked by the registry.
    ///
    /// # Errors
    /// The first invalid field.
    pub fn validate(&self) -> Result<(), ConfigError> {
        if self.num_of_robots == 0 || self.num_of_robots > MAX_ROBOTS {
            return Err(ConfigError::RobotCount(self.num_of_robots));
        }
        positive("robot_speeds", self.robot_speeds)?;
        positive("lambda_rate", self.lambda_rate)?;
        positive("sampling_rate", self.sampling_rate)?;
        if let Some(v) = self.visibility_radius {
            positive("visibility_radius", v)?;
        }
        if !(1..=15).contains(&self.threshold_precision) {
            return Err(ConfigError::Precision(self.threshold_precision));
        }
        if let Some(positions) = &self.initial_positions {
            if let Some(index) = positions
                .iter()
                .position(|p| !p[0].is_finite() || !p[1].is_finite())
            {
                return Err(ConfigError::Position { index });
            }
        }
        if let Some(start) = &self.start {
            start.validate().map_err(ConfigError::Start)?;
        }
        Ok(())
    }

    /// World bounds as the robots see them: none in an open world; `0`
    /// means unbounded (`run.py`'s `or None`).
    #[must_use]
    pub fn bounds(&self) -> (Option<f64>, Option<f64>) {
        if self.open_world {
            return (None, None);
        }
        let nonzero = |v: Option<f64>| v.filter(|w| *w != 0.0);
        (nonzero(self.width_bound), nonzero(self.height_bound))
    }

    /// Initial positions, in this order: the given list if it has one entry
    /// per robot; a generated start if `start` is set (`start::generate`);
    /// otherwise uniform in the box from a generator of their own, like `run.py`.
    #[must_use]
    pub fn resolve_positions(&self) -> Vec<Point> {
        let n = self.num_of_robots as usize;
        if let Some(given) = self.initial_positions.as_ref().filter(|p| p.len() == n) {
            return given.iter().map(|p| Point::new(p[0], p[1])).collect();
        }
        if let Some(spec) = &self.start {
            return start::generate(spec, n, self.random_seed);
        }
        let width = self.width_bound.unwrap_or(100.0);
        let height = self.height_bound.unwrap_or(100.0);
        let mut rng = crate::Rng::new(self.random_seed);
        let xs: Vec<f64> = (0..n)
            .map(|_| rng.uniform(-width / 2.0, width / 2.0))
            .collect();
        let ys: Vec<f64> = (0..n)
            .map(|_| rng.uniform(-height / 2.0, height / 2.0))
            .collect();
        xs.into_iter()
            .zip(ys)
            .map(|(x, y)| Point::new(x, y))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::start::{Arrangement, Distribution, Pattern};

    fn ring() -> StartSpec {
        StartSpec {
            pattern: Pattern::Circle,
            arrangement: Arrangement::Edge,
            distribution: Distribution::Even,
            ..StartSpec::default()
        }
    }

    #[test]
    fn open_world_has_no_bounds() {
        let world = SimConfig::default();
        assert_eq!(world.bounds(), (Some(600.0), Some(600.0)));
        let open = SimConfig {
            open_world: true,
            ..SimConfig::default()
        };
        assert_eq!(open.bounds(), (None, None));
    }

    #[test]
    fn explicit_positions_win_then_generated_then_the_original_box() {
        let explicit = SimConfig {
            num_of_robots: 2,
            initial_positions: Some(vec![[1.0, 2.0], [3.0, 4.0]]),
            start: Some(ring()),
            ..SimConfig::default()
        };
        assert_eq!(
            explicit.resolve_positions(),
            vec![Point::new(1.0, 2.0), Point::new(3.0, 4.0)]
        );
        let generated = SimConfig {
            num_of_robots: 4,
            start: Some(ring()),
            ..SimConfig::default()
        };
        assert_eq!(
            generated.resolve_positions(),
            start::generate(&ring(), 4, generated.random_seed)
        );
        let original = SimConfig {
            num_of_robots: 4,
            ..SimConfig::default()
        };
        assert!(original
            .resolve_positions()
            .iter()
            .all(|p| p.x.abs() <= 300.0 && p.y.abs() <= 300.0));
        assert_ne!(original.resolve_positions(), generated.resolve_positions());
    }

    #[test]
    fn an_unsupported_start_is_rejected() {
        let bad = SimConfig {
            start: Some(StartSpec {
                pattern: Pattern::Line,
                arrangement: Arrangement::Edge,
                distribution: Distribution::Gaussian,
                ..StartSpec::default()
            }),
            ..SimConfig::default()
        };
        assert!(matches!(bad.validate(), Err(ConfigError::Start(_))));
    }
}

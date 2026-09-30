//! Simulation input (docs/core-schema.md §2). Field names match the `params()`
//! object `main.js` sends, so existing UI state maps across unchanged.

use crate::geom::Point;
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
        Ok(())
    }

    /// World bounds as the robots see them: `0` means unbounded (`run.py`'s `or None`).
    #[must_use]
    pub fn bounds(&self) -> (Option<f64>, Option<f64>) {
        let nonzero = |v: Option<f64>| v.filter(|w| *w != 0.0);
        (nonzero(self.width_bound), nonzero(self.height_bound))
    }

    /// Initial positions: the given list if it has one entry per robot, otherwise
    /// drawn from a generator of their own, like `run.py`.
    #[must_use]
    pub fn resolve_positions(&self) -> Vec<Point> {
        let n = self.num_of_robots as usize;
        if let Some(given) = self.initial_positions.as_ref().filter(|p| p.len() == n) {
            return given.iter().map(|p| Point::new(p[0], p[1])).collect();
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

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
    /// Semi-synchronous rounds: each round a non-empty subset of the robots
    /// looks at the same instant, computes, and moves; the round ends once
    /// every one of them has stopped.
    Ssync,
    /// Fully synchronous rounds: every live robot is active in every round.
    Fsync,
    /// Bounded asynchrony (k-Async, Kirkpatrick et al. 2024): as async, but
    /// while one robot is in a cycle (its Look until the end of its move) no
    /// other robot starts more than `k_async` Looks; a Look that would is
    /// put off until that cycle ends.
    #[cfg_attr(feature = "serde", serde(rename = "kasync"))]
    KAsync,
}

impl SchedulerKind {
    /// Rounds of simultaneous activations (SSYNC or FSYNC).
    #[must_use]
    pub const fn rounds(self) -> bool {
        matches!(self, Self::Ssync | Self::Fsync)
    }
}

/// What a robot can tell about a point several robots share. Algorithms read
/// it through [`crate::Look::occupied`]; those that read the raw view (the
/// ports of the Python algorithms) are unaffected.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "lowercase"))]
pub enum MultiplicityModel {
    /// Only the set of occupied points: a shared point looks like any other.
    None,
    /// One bit per occupied point: one robot, or more than one.
    #[default]
    Weak,
    /// The number of robots on each occupied point.
    Strong,
    /// Local weak detection (Kamei et al. 2011): a robot knows whether its
    /// own point holds more than one robot, and nothing about other points.
    Local,
}

/// The coordinate system each robot computes in.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "lowercase"))]
pub enum Frames {
    /// Everyone shares the global frame (the original simulator).
    #[default]
    Global,
    /// At each Look the robot gets a frame of its own: itself at the origin,
    /// a random rotation and unit length, the common handedness.
    Rotated,
    /// As `Rotated`, and the handedness is random too (no chirality).
    Disoriented,
    /// An unreliable compass (Izumi et al. 2012): the robot's axes are the
    /// global ones turned by a deviation of at most `compass_error` degrees
    /// either way; unit length and handedness are shared. The deviation is
    /// drawn once per robot (`compass_static`) or at every Look.
    Compass,
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

/// Non-rigid movement: where the adversary stops a robot whose destination
/// is farther than `delta`. It never stops one before `delta`.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "lowercase"))]
pub enum StopPolicy {
    /// After exactly `delta`: the least progress the model allows.
    Delta,
    /// Anywhere between `delta` and the destination, at random.
    #[default]
    Random,
    /// After `stop_fraction` of the way (never less than `delta`).
    Fraction,
}

/// One turn of an explicit sequential schedule: a robot, optionally with the
/// fraction of the way the adversary lets it travel (non-rigid only).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(untagged))]
pub enum Activation {
    Robot(usize),
    Stopped { robot: usize, stop: f64 },
}

impl Activation {
    #[must_use]
    pub const fn robot(self) -> usize {
        match self {
            Self::Robot(r) | Self::Stopped { robot: r, .. } => r,
        }
    }

    #[must_use]
    pub const fn stop(self) -> Option<f64> {
        match self {
            Self::Robot(_) => None,
            Self::Stopped { stop, .. } => Some(stop),
        }
    }
}

/// What an explicit schedule does once its list is used up.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum ScheduleEnd {
    /// Carry on 0, 1, 2, … (always fair).
    #[default]
    RoundRobin,
    /// Play the list again (it must name every robot, to stay fair).
    Repeat,
    /// Wait for the next activation to be added while the run goes on (an
    /// adversary choosing turns live): `advance` returns `AwaitingSchedule`.
    Await,
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
    /// Display only: draw how many robots share each point.
    pub multiplicity_detection: bool,
    /// What robots can tell about shared points (`none`, `weak`, `strong`).
    pub multiplicity: MultiplicityModel,
    /// Global coordinates, or a private frame per robot and Look.
    pub frames: Frames,
    /// `frames: "compass"`: the largest deviation of a robot's compass from
    /// the global north, in degrees (0 to 180).
    pub compass_error: f64,
    /// `frames: "compass"`: each robot keeps one deviation for the whole run
    /// (static compass); off, it gets a new one at every Look (dynamic).
    pub compass_static: bool,
    /// A robot does not see robots hidden behind another robot.
    pub obstructed_visibility: bool,
    /// Robots are discs of this radius when they block the view (0: points,
    /// which block a view only when exactly in line, to within ε).
    pub robot_radius: f64,
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
    /// Non-rigid movement: a robot always covers at least this far (or reaches
    /// its destination if that is nearer). `None`: the original behaviour, where
    /// a non-rigid robot still reaches its destination. Needs rigid_movement off.
    pub delta: Option<f64>,
    /// Where a non-rigid robot is stopped once its destination is farther than `delta`.
    pub stop_policy: StopPolicy,
    /// The fraction of the way for `stop_policy` `fraction`, in (0, 1].
    pub stop_fraction: f64,
    /// Sequential only: the turns, in order, instead of `activation_order`.
    pub schedule: Option<Vec<Activation>>,
    /// What follows the end of `schedule` (sequential) or `rounds` (SSYNC).
    pub schedule_end: ScheduleEnd,
    /// SSYNC: chance that a live robot is active in a round. A round with
    /// nobody drawn activates one robot at random, so rounds are never empty.
    pub ssync_probability: f64,
    /// SSYNC: a robot left out of this many rounds in a row is activated in
    /// the next one, so the scheduler stays fair.
    pub ssync_fairness: u32,
    /// SSYNC: the robots of each round, in order, instead of random draws.
    pub rounds: Option<Vec<Vec<usize>>>,
    /// SSYNC and FSYNC: stop after this many rounds. `None`: no limit.
    pub max_rounds: Option<u64>,
    /// k-Async: the most Looks any robot may start while another robot is in
    /// one cycle (at least 1).
    pub k_async: u32,
    /// Sequential only: stop after this many turns (`advance` then returns
    /// `MaxTurns`). `None`: no limit.
    pub max_turns: Option<u64>,
    /// Start with this many multiplicity points (several robots on one spot),
    /// made from the start positions. 0: none.
    pub multiplicities: u32,
    /// Robots on each starting multiplicity (at least 2); 0 chooses a size:
    /// a third of the robots for one multiplicity, up to 4 each for several.
    pub multiplicity_size: u32,
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
            multiplicity: MultiplicityModel::Weak,
            frames: Frames::Global,
            compass_error: 0.0,
            compass_static: true,
            obstructed_visibility: false,
            robot_radius: 0.0,
            max_events: None,
            max_time: None,
            open_world: false,
            start: None,
            scheduler: SchedulerKind::Async,
            activation_order: ActivationOrder::RoundRobin,
            turn_gap: TurnGap::Random,
            delta: None,
            stop_policy: StopPolicy::Random,
            stop_fraction: 0.5,
            schedule: None,
            schedule_end: ScheduleEnd::RoundRobin,
            ssync_probability: 0.5,
            ssync_fairness: 4,
            rounds: None,
            max_rounds: None,
            k_async: 1,
            max_turns: None,
            multiplicities: 0,
            multiplicity_size: 0,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum ConfigError {
    UnknownAlgorithm(String),
    RobotCount(u32),
    NotPositive {
        field: &'static str,
        value: f64,
    },
    Precision(u8),
    Position {
        index: usize,
    },
    Start(String),
    /// A sequential-scheduler setting that cannot work as given.
    Schedule(String),
    /// The chosen algorithm cannot run with these settings (for example, it
    /// needs the sequential scheduler).
    Unsupported(String),
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
            Self::Start(message) | Self::Schedule(message) | Self::Unsupported(message) => {
                f.write_str(message)
            }
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
        if let Some(delta) = self.delta {
            positive("delta", delta)?;
        }
        if !(self.stop_fraction > 0.0 && self.stop_fraction <= 1.0) {
            return Err(ConfigError::Schedule(format!(
                "stop_fraction must be in (0, 1], got {}",
                self.stop_fraction
            )));
        }
        start::check_multiplicities(
            self.num_of_robots as usize,
            self.multiplicities as usize,
            self.multiplicity_size as usize,
        )
        .map_err(ConfigError::Start)?;
        if let Some(turns) = self.max_turns {
            if turns == 0 {
                return Err(ConfigError::Schedule(
                    "max_turns must be at least 1".to_owned(),
                ));
            }
            if self.scheduler != SchedulerKind::Sequential {
                return Err(ConfigError::Schedule(
                    "max_turns needs scheduler \"sequential\"".to_owned(),
                ));
            }
        }
        if !(self.compass_error.is_finite() && (0.0..=180.0).contains(&self.compass_error)) {
            return Err(ConfigError::Schedule(format!(
                "compass_error must be between 0 and 180 degrees, got {}",
                self.compass_error
            )));
        }
        if !(self.robot_radius.is_finite() && self.robot_radius >= 0.0) {
            return Err(ConfigError::NotPositive {
                field: "robot_radius",
                value: self.robot_radius,
            });
        }
        if !(self.ssync_probability > 0.0 && self.ssync_probability <= 1.0) {
            return Err(ConfigError::Schedule(format!(
                "ssync_probability must be in (0, 1], got {}",
                self.ssync_probability
            )));
        }
        if self.k_async == 0 {
            return Err(ConfigError::Schedule(
                "k_async must be at least 1".to_owned(),
            ));
        }
        if self.ssync_fairness == 0 {
            return Err(ConfigError::Schedule(
                "ssync_fairness must be at least 1".to_owned(),
            ));
        }
        if let Some(rounds) = self.max_rounds {
            if rounds == 0 {
                return Err(ConfigError::Schedule(
                    "max_rounds must be at least 1".to_owned(),
                ));
            }
            if !self.scheduler.rounds() {
                return Err(ConfigError::Schedule(
                    "max_rounds needs scheduler \"ssync\" or \"fsync\"".to_owned(),
                ));
            }
        }
        self.validate_schedule()?;
        self.validate_rounds()
    }

    /// Explicit SSYNC rounds: non-empty sets of robots that exist.
    fn validate_rounds(&self) -> Result<(), ConfigError> {
        let fail = |message: String| Err(ConfigError::Schedule(message));
        let Some(list) = &self.rounds else {
            if self.schedule_end == ScheduleEnd::Await && self.scheduler == SchedulerKind::Ssync {
                return fail(
                    "schedule_end \"await\" needs rounds (the list may start empty)".to_owned(),
                );
            }
            return Ok(());
        };
        if self.scheduler != SchedulerKind::Ssync {
            return fail("rounds needs scheduler \"ssync\"".to_owned());
        }
        if list.is_empty() && self.schedule_end != ScheduleEnd::Await {
            return fail("rounds must name at least one round".to_owned());
        }
        let n = self.robot_count();
        let mut named = vec![false; n];
        for (k, round) in list.iter().enumerate() {
            if round.is_empty() {
                return fail(format!("rounds[{k}] activates nobody"));
            }
            for &r in round {
                if r >= n {
                    return fail(format!(
                        "rounds[{k}] activates robot {r} but there are only {n} robots"
                    ));
                }
                named[r] = true;
            }
        }
        if self.schedule_end == ScheduleEnd::Repeat {
            if let Some(r) = named.iter().position(|x| !x) {
                return fail(format!(
                    "repeating rounds must name every robot to be fair; robot {r} is never activated"
                ));
            }
        }
        Ok(())
    }

    /// The number of robots the run will have.
    fn robot_count(&self) -> usize {
        if self
            .initial_positions
            .as_ref()
            .is_some_and(|p| p.len() == self.num_of_robots as usize)
        {
            self.num_of_robots as usize
        } else {
            self.resolve_positions().len()
        }
    }

    /// An explicit schedule needs the sequential scheduler, robots that exist,
    /// and, to name a stopping point, non-rigid movement with a `delta`.
    fn validate_schedule(&self) -> Result<(), ConfigError> {
        let fail = |message: String| Err(ConfigError::Schedule(message));
        let Some(list) = &self.schedule else {
            if self.schedule_end == ScheduleEnd::Await
                && self.scheduler == SchedulerKind::Sequential
            {
                return fail(
                    "schedule_end \"await\" needs a schedule (the list may start empty)".to_owned(),
                );
            }
            return Ok(());
        };
        if self.scheduler != SchedulerKind::Sequential {
            return fail("schedule needs scheduler \"sequential\"".to_owned());
        }
        if list.is_empty() && self.schedule_end != ScheduleEnd::Await {
            return fail("schedule must name at least one turn".to_owned());
        }
        let n = self.robot_count();
        for (k, a) in list.iter().enumerate() {
            if a.robot() >= n {
                return fail(format!(
                    "schedule[{k}] activates robot {} but there are only {n} robots",
                    a.robot()
                ));
            }
            if let Some(stop) = a.stop() {
                if !(stop > 0.0 && stop <= 1.0) {
                    return fail(format!("schedule[{k}] stop must be in (0, 1], got {stop}"));
                }
                if self.rigid_movement || self.delta.is_none() {
                    return fail(format!(
                        "schedule[{k}] names a stopping point: turn rigid movement off and set delta"
                    ));
                }
            }
        }
        if self.schedule_end == ScheduleEnd::Repeat {
            let mut named = vec![false; n];
            for a in list {
                named[a.robot()] = true;
            }
            if let Some(r) = named.iter().position(|x| !x) {
                return fail(format!(
                    "a repeating schedule must name every robot to be fair; robot {r} is never activated"
                ));
            }
        }
        Ok(())
    }

    /// World bounds as the robots see them: none in an open world; `0`
    /// means unbounded (the original simulator's `or None`).
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
    /// otherwise uniform in the box from a generator of their own, like the original simulator.
    /// Then, if `multiplicities` is set, robots are stacked onto that many points.
    #[must_use]
    pub fn resolve_positions(&self) -> Vec<Point> {
        let mut points = self.base_positions();
        if self.multiplicities > 0 {
            // `validate` has checked that this fits.
            let _ = start::stack_multiplicities(
                &mut points,
                self.multiplicities as usize,
                self.multiplicity_size as usize,
                self.random_seed,
            );
        }
        points
    }

    fn base_positions(&self) -> Vec<Point> {
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

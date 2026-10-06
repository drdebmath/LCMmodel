//! A fair **sequential** scheduler (SEQ, Section 2.2 of the paper) for
//! [`crate::sq_gathering`]: exactly one robot is activated per round and
//! completes its whole Look-Compute-Move cycle in that round.
//!
//! This is deliberately separate from `lcm_core::Simulation`, whose
//! continuous-time asynchronous event loop is what the generic `Gathering`
//! demo runs on. Here time is rounds, activations follow a schedule, and
//! moves are non-rigid: the adversary may stop a robot anywhere after it has
//! travelled δ (it always arrives if the destination is within δ).
//!
//! Everything is deterministic for a given [`SeqConfig`], natively and in
//! WebAssembly (only `libm` and basic f64 arithmetic).

use crate::sq_gathering::{decide_in_frame, observe, occupancy, Decision, LocalFrame};
use lcm_core::geom::{dist, interpolate};
use lcm_core::{Point, Rng, SimConfig};
use serde::{Deserialize, Serialize};
use std::fmt;

/// The algorithms this scheduler can run.
pub const SEQ_ALGORITHMS: &[&str] = &["SqGathering"];

/// One activation of an explicit schedule: a robot index, optionally with
/// the fraction of the way the adversary lets it travel (non-rigid only).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
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
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Fallback {
    /// Continue round-robin from robot 0 (always fair).
    #[default]
    RoundRobin,
    /// Play the list again (fair only if it names every robot; checked).
    Repeat,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case")]
pub enum ScheduleSpec {
    /// 0, 1, …, n−1, 0, 1, …
    RoundRobin,
    /// A fresh random permutation of the robots every n rounds.
    Shuffled { seed: u64 },
    /// The given activations, then `then`.
    Explicit {
        order: Vec<Activation>,
        #[serde(default)]
        then: Fallback,
    },
}

/// Where the adversary stops a robot whose destination is farther than δ.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case")]
pub enum StopPolicy {
    /// After exactly δ: the slowest progress the model allows.
    Delta,
    /// Uniformly between δ and the destination.
    Random { seed: u64 },
    /// After `value` of the way (never less than δ).
    Fraction { value: f64 },
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case")]
pub enum MovementSpec {
    Rigid,
    NonRigid { delta: f64, stop: StopPolicy },
}

/// The robots' private coordinate systems.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case")]
pub enum FrameSpec {
    /// Every robot uses the global axes (centred on itself).
    Global,
    /// Each robot gets its own rotation, unit length and handedness.
    Random { seed: u64 },
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SeqConfig {
    pub algorithm: String,
    pub positions: Vec<[f64; 2]>,
    pub schedule: ScheduleSpec,
    /// The movement limit: rigid, or non-rigid with δ.
    pub movement: MovementSpec,
    pub frames: FrameSpec,
    /// Round budget. A run that reaches it without gathering is incomplete.
    pub max_rounds: u64,
    /// Points closer than this are one location.
    pub coincidence: f64,
}

impl Default for SeqConfig {
    fn default() -> Self {
        Self {
            algorithm: "SqGathering".to_owned(),
            positions: Vec::new(),
            schedule: ScheduleSpec::RoundRobin,
            movement: MovementSpec::NonRigid {
                delta: 1.0,
                stop: StopPolicy::Delta,
            },
            frames: FrameSpec::Global,
            max_rounds: 10_000,
            coincidence: crate::sq_gathering::DEFAULT_COINCIDENCE,
        }
    }
}

/// Makes `k` multiplicity points of `size` robots each: picks `k` robots at
/// random (from `seed`) as anchors and moves `size - 1` others onto each.
/// The rest stay single. `size` 0 picks one: a third of the robots for a
/// single multiplicity, 4 when there are several (robots leaving one of
/// several multiplicities halve their distance each time, so large ones
/// stall at the floating-point limit, see `Status::Stalled`).
///
/// # Errors
/// `size` 1, or more robots asked for than there are.
pub fn stack_multiplicities(
    positions: &mut [[f64; 2]],
    k: usize,
    size: usize,
    seed: u64,
) -> Result<(), SeqError> {
    if k == 0 {
        return Ok(());
    }
    let n = positions.len();
    let size = match size {
        0 if k == 1 => (n / 3).max(2),
        0 => 4.min(n / k).max(2),
        1 => return fail("a multiplicity needs at least 2 robots"),
        s => s,
    };
    if k * size > n {
        return fail(format!(
            "{k} multiplicities of {size} robots need {} robots, there are {n}",
            k * size
        ));
    }
    let mut order: Vec<usize> = (0..n).collect();
    Rng::new(seed ^ 0x4D55_4C54).shuffle(&mut order);
    let per = size - 1;
    for j in 0..k {
        let anchor = positions[order[j]];
        for t in 0..per {
            positions[order[k + j * per + t]] = anchor;
        }
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SeqError(pub String);

impl fmt::Display for SeqError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for SeqError {}

fn fail<T>(message: impl Into<String>) -> Result<T, SeqError> {
    Err(SeqError(message.into()))
}

/// The sequential options the main simulator's "Sequential ›" menu offers.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SeqOptions {
    /// `"shuffled"` (a fresh random order each epoch) or `"round-robin"`.
    pub schedule: String,
    /// Non-rigid stops: `"random"` (between δ and the destination),
    /// `"delta"` (exactly δ) or `"half"` (halfway, never less than δ).
    pub stop: String,
    /// `"random"` (own rotation, scale and handedness) or `"global"`.
    pub frames: String,
    /// κ at the start: 0 keeps the generated start; k > 0 stacks robots of
    /// it onto k of its points, making k multiplicity points.
    pub multiplicities: u32,
    /// Robots on each starting multiplicity (2 or more); 0 picks a size:
    /// a third of the robots for κ = 1, 4 robots each for κ > 1.
    pub multiplicity_size: u32,
}

impl Default for SeqOptions {
    fn default() -> Self {
        Self {
            schedule: "shuffled".to_owned(),
            stop: "random".to_owned(),
            frames: "random".to_owned(),
            multiplicities: 0,
            multiplicity_size: 0,
        }
    }
}

impl SeqConfig {
    /// The sequential run the main simulator page starts when a sequential
    /// algorithm is picked from its menu: the page's start positions and
    /// seed; non-rigid moves with δ = the speed setting unless movement is
    /// rigid; schedule, stopping points and frames from `options`. The event
    /// limit, if set, is the round budget.
    ///
    /// # Errors
    /// An unknown option, or settings Algorithm 8 does not model: limited
    /// visibility or faults.
    pub fn from_sim(config: &SimConfig, options: &SeqOptions) -> Result<Self, SeqError> {
        config.validate().map_err(|e| SeqError(e.to_string()))?;
        if config.visibility_radius.is_some() {
            return fail("SqGathering assumes unlimited visibility: turn on Unlimited visibility");
        }
        if config.num_of_faults > 0 {
            return fail("SqGathering assumes no faulty robots: set Faulty robots to 0");
        }
        let seed = config.random_seed;
        let stop = match options.stop.as_str() {
            "random" => StopPolicy::Random {
                seed: seed ^ 0x5157_4741,
            },
            "delta" => StopPolicy::Delta,
            "half" => StopPolicy::Fraction { value: 0.5 },
            other => return fail(format!("unknown stop option {other:?}")),
        };
        let movement = if config.rigid_movement {
            MovementSpec::Rigid
        } else {
            MovementSpec::NonRigid {
                delta: config.robot_speeds,
                stop,
            }
        };
        let schedule = match options.schedule.as_str() {
            "shuffled" => ScheduleSpec::Shuffled { seed },
            "round-robin" => ScheduleSpec::RoundRobin,
            other => return fail(format!("unknown schedule option {other:?}")),
        };
        let frames = match options.frames.as_str() {
            "random" => FrameSpec::Random {
                seed: seed.wrapping_add(1),
            },
            "global" => FrameSpec::Global,
            other => return fail(format!("unknown frames option {other:?}")),
        };
        let mut positions: Vec<[f64; 2]> = config
            .resolve_positions()
            .iter()
            .map(|p| [p.x, p.y])
            .collect();
        stack_multiplicities(
            &mut positions,
            options.multiplicities as usize,
            options.multiplicity_size as usize,
            seed,
        )?;
        Ok(Self {
            algorithm: config.algorithm.clone(),
            positions,
            schedule,
            movement,
            frames,
            max_rounds: config.max_events.unwrap_or(u64::MAX),
            ..Self::default()
        })
    }

    /// # Errors
    /// The first invalid field.
    pub fn validate(&self) -> Result<(), SeqError> {
        if !SEQ_ALGORITHMS.contains(&self.algorithm.as_str()) {
            return fail(format!(
                "unknown sequential algorithm {:?} (available: {})",
                self.algorithm,
                SEQ_ALGORITHMS.join(", ")
            ));
        }
        let n = self.positions.len();
        if n == 0 {
            return fail("positions must list at least one robot");
        }
        if let Some(i) = self
            .positions
            .iter()
            .position(|p| !p[0].is_finite() || !p[1].is_finite())
        {
            return fail(format!("positions[{i}] is not finite"));
        }
        if !(self.coincidence.is_finite() && self.coincidence > 0.0) {
            return fail("coincidence must be a positive number");
        }
        if let MovementSpec::NonRigid { delta, stop } = self.movement {
            if !(delta.is_finite() && delta > 0.0) {
                return fail(format!("delta must be a positive number, got {delta}"));
            }
            if let StopPolicy::Fraction { value } = stop {
                if !(value > 0.0 && value <= 1.0) {
                    return fail(format!("stop fraction must be in (0, 1], got {value}"));
                }
            }
        }
        if let ScheduleSpec::Explicit { order, then } = &self.schedule {
            for (k, a) in order.iter().enumerate() {
                if a.robot() >= n {
                    return fail(format!(
                        "schedule[{k}] activates robot {} but there are only {n} robots",
                        a.robot()
                    ));
                }
                if let Some(s) = a.stop() {
                    if !(0.0..=1.0).contains(&s) {
                        return fail(format!("schedule[{k}] stop must be in [0, 1], got {s}"));
                    }
                }
            }
            if *then == Fallback::Repeat {
                let mut named = vec![false; n];
                for a in order {
                    named[a.robot()] = true;
                }
                if let Some(r) = named.iter().position(|x| !x) {
                    return fail(format!(
                        "a repeating schedule must activate every robot to be fair; robot {r} is never activated"
                    ));
                }
            }
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Status {
    Running,
    /// All robots on one point (they all stay there from now on).
    Gathered,
    /// The round budget ran out first. Says nothing about the algorithm.
    Incomplete,
    /// A robot leaving a multiplicity could not get away from it: its
    /// midpoint was closer than `coincidence`. Each robot leaving the same
    /// multiplicity halves the distance (its closest point is the robot
    /// that left before it), so large multiplicities under κ > 1 exhaust
    /// floating-point precision. In exact arithmetic the run would go on.
    Stalled,
}

/// One point of the active robot's observation, for the trace.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SeenPoint {
    pub at: [f64; 2],
    /// What the robot sees (weak detection): `true` for 2 or more robots.
    pub multiplicity: bool,
    /// The real number of robots there. The robot cannot see this.
    pub count: usize,
}

/// Everything that happened in one round.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Round {
    pub round: u64,
    pub epoch: u64,
    pub robot: usize,
    pub position: [f64; 2],
    /// Q′(t) in global coordinates, own point at `self_index`.
    pub observation: Vec<SeenPoint>,
    pub self_index: usize,
    pub kappa: usize,
    pub on_multiplicity: bool,
    pub rule: String,
    pub lines: String,
    pub description: String,
    pub destination: [f64; 2],
    /// Where the robot actually ended the round.
    pub stop: [f64; 2],
    pub reached: bool,
    pub travelled: f64,
    pub distinct_after: usize,
    pub multiplicities_after: usize,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Report {
    pub algorithm: String,
    pub status: Status,
    pub rounds: u64,
    pub epochs_completed: u64,
    pub gathered_at: Option<[f64; 2]>,
    pub distinct: usize,
    pub multiplicities: usize,
    pub positions: Vec<[f64; 2]>,
    pub note: String,
}

const fn xy(p: Point) -> [f64; 2] {
    [p.x, p.y]
}

pub struct SqGatheringRun {
    config: SeqConfig,
    positions: Vec<Point>,
    frames: Vec<(f64, f64, bool)>,
    round: u64,
    epoch: u64,
    seen: Vec<bool>,
    unseen: usize,
    cursor: usize,
    order: Vec<usize>,
    schedule_rng: Rng,
    stop_rng: Rng,
    status: Status,
}

impl SqGatheringRun {
    /// # Errors
    /// An invalid configuration.
    pub fn new(config: SeqConfig) -> Result<Self, SeqError> {
        config.validate()?;
        let positions: Vec<Point> = config
            .positions
            .iter()
            .map(|p| Point::new(p[0], p[1]))
            .collect();
        let n = positions.len();
        let frames = match config.frames {
            FrameSpec::Global => vec![(0.0, 1.0, false); n],
            FrameSpec::Random { seed } => {
                let mut rng = Rng::new(seed);
                (0..n)
                    .map(|_| {
                        let angle = rng.uniform(0.0, core::f64::consts::TAU);
                        let scale = rng.uniform(0.25, 4.0);
                        (angle, scale, rng.random() < 0.5)
                    })
                    .collect()
            }
        };
        let schedule_rng = match config.schedule {
            ScheduleSpec::Shuffled { seed } => Rng::new(seed),
            _ => Rng::new(0),
        };
        let stop_rng = match config.movement {
            MovementSpec::NonRigid {
                stop: StopPolicy::Random { seed },
                ..
            } => Rng::new(seed),
            _ => Rng::new(0),
        };
        let mut run = Self {
            positions,
            frames,
            round: 0,
            epoch: 1,
            seen: vec![false; n],
            unseen: n,
            cursor: 0,
            order: Vec::new(),
            schedule_rng,
            stop_rng,
            status: Status::Running,
            config,
        };
        run.update_status();
        Ok(run)
    }

    #[must_use]
    pub fn config(&self) -> &SeqConfig {
        &self.config
    }

    #[must_use]
    pub fn positions(&self) -> &[Point] {
        &self.positions
    }

    #[must_use]
    pub const fn status(&self) -> Status {
        self.status
    }

    #[must_use]
    pub const fn round(&self) -> u64 {
        self.round
    }

    /// The private frame robot `i` would use right now.
    #[must_use]
    pub fn frame(&self, i: usize) -> LocalFrame {
        let (angle, scale, reflect) = self.frames[i];
        LocalFrame {
            origin: self.positions[i],
            angle,
            scale,
            reflect,
        }
    }

    /// Distinct points and multiplicity points now.
    #[must_use]
    pub fn census(&self) -> (usize, usize) {
        let (_, counts, _) = occupancy(&self.positions, self.config.coincidence);
        (counts.len(), counts.iter().filter(|c| **c >= 2).count())
    }

    /// What robot `i` would decide if it were activated now (no side effects).
    #[must_use]
    pub fn preview(&self, i: usize) -> (crate::sq_gathering::Observation, Decision) {
        let obs = observe(&self.positions, i, self.config.coincidence);
        let decision = decide_in_frame(&obs, &self.frame(i));
        (obs, decision)
    }

    fn update_status(&mut self) {
        if self.census().0 == 1 {
            self.status = Status::Gathered;
        } else if self.round >= self.config.max_rounds {
            self.status = Status::Incomplete;
        }
    }

    fn next_activation(&mut self) -> (usize, Option<f64>) {
        let n = self.positions.len();
        let k = self.cursor;
        self.cursor += 1;
        match &self.config.schedule {
            ScheduleSpec::RoundRobin => (k % n, None),
            ScheduleSpec::Shuffled { .. } => {
                if k.is_multiple_of(n) {
                    self.order = (0..n).collect();
                    self.schedule_rng.shuffle(&mut self.order);
                }
                (self.order[k % n], None)
            }
            ScheduleSpec::Explicit { order, then } => {
                if k < order.len() {
                    (order[k].robot(), order[k].stop())
                } else {
                    match then {
                        Fallback::Repeat => {
                            let a = order[k % order.len()];
                            (a.robot(), a.stop())
                        }
                        Fallback::RoundRobin => ((k - order.len()) % n, None),
                    }
                }
            }
        }
    }

    /// Where the robot ends up: the destination if it is reached, otherwise
    /// the adversary's stopping point (never before δ).
    fn travel(&mut self, from: Point, to: Point, stop: Option<f64>) -> (Point, bool, f64) {
        let length = dist(from, to);
        if length == 0.0 {
            return (from, true, 0.0);
        }
        let covered = match self.config.movement {
            MovementSpec::Rigid => length,
            MovementSpec::NonRigid { delta, .. } if length <= delta => length,
            MovementSpec::NonRigid {
                delta,
                stop: policy,
            } => {
                let wanted = match (stop, policy) {
                    (Some(f), _) | (None, StopPolicy::Fraction { value: f }) => f * length,
                    (None, StopPolicy::Delta) => delta,
                    (None, StopPolicy::Random { .. }) => self.stop_rng.uniform(delta, length),
                };
                wanted.clamp(delta, length)
            }
        };
        if covered >= length {
            (to, true, length)
        } else {
            (interpolate(from, to, covered / length), false, covered)
        }
    }

    /// Plays one round. `None` once gathered or out of budget.
    pub fn step(&mut self) -> Option<Round> {
        if self.status != Status::Running {
            return None;
        }
        let (i, stop) = self.next_activation();
        let tolerance = self.config.coincidence;
        let (_, counts, _) = occupancy(&self.positions, tolerance);
        let (obs, decision) = self.preview(i);
        let from = self.positions[i];
        let destination = decision.destination(&obs);
        let (end, reached, travelled) = self.travel(from, destination, stop);
        self.positions[i] = end;
        self.round += 1;
        let epoch = self.epoch;
        if !self.seen[i] {
            self.seen[i] = true;
            self.unseen -= 1;
            if self.unseen == 0 {
                self.epoch += 1;
                self.seen.fill(false);
                self.unseen = self.positions.len();
            }
        }
        let (distinct_after, multiplicities_after) = self.census();
        self.update_status();
        if decision.rule == crate::sq_gathering::Rule::LeaveMultiplicity
            && dist(from, end) <= tolerance
            && self.status == Status::Running
        {
            self.status = Status::Stalled;
        }
        Some(Round {
            round: self.round,
            epoch,
            robot: i,
            position: xy(from),
            observation: obs
                .points
                .iter()
                .zip(&counts)
                .map(|(o, &count)| SeenPoint {
                    at: xy(o.at),
                    multiplicity: o.multiplicity,
                    count,
                })
                .collect(),
            self_index: obs.me,
            kappa: decision.kappa,
            on_multiplicity: decision.on_multiplicity,
            rule: decision.rule.key().to_owned(),
            lines: decision.rule.lines().to_owned(),
            description: decision.rule.description().to_owned(),
            destination: xy(destination),
            stop: xy(end),
            reached,
            travelled,
            distinct_after,
            multiplicities_after,
        })
    }

    /// Plays rounds until gathered or out of budget, calling `each` per round.
    pub fn run_with(&mut self, mut each: impl FnMut(&Round)) -> Report {
        while let Some(round) = self.step() {
            each(&round);
        }
        self.report()
    }

    pub fn run(&mut self) -> Report {
        self.run_with(|_| {})
    }

    #[must_use]
    pub fn report(&self) -> Report {
        let (distinct, multiplicities) = self.census();
        let note = match self.status {
            Status::Gathered => format!("gathered after {} rounds", self.round),
            Status::Running => format!("still running after {} rounds", self.round),
            Status::Stalled => format!(
                "STALLED (floating-point limit) after {} rounds: a robot leaving a                  multiplicity moved less than the coincidence tolerance ({}) and stayed on it.                  Robots leaving one multiplicity halve their distance each time, so large                  multiplicities under κ > 1 run out of precision. Not a result of the algorithm                  (exact arithmetic would continue); use smaller multiplicities.",
                self.round, self.config.coincidence
            ),
            Status::Incomplete => format!(
                "INCOMPLETE (budget-limited): the round budget of {} rounds ran out before \
                 gathering; {distinct} distinct points and {multiplicities} multiplicities \
                 remain. This is a budget limit, not a result of the algorithm.",
                self.config.max_rounds
            ),
        };
        Report {
            algorithm: self.config.algorithm.clone(),
            status: self.status,
            rounds: self.round,
            epochs_completed: self.epoch - 1,
            gathered_at: (self.status == Status::Gathered).then(|| xy(self.positions[0])),
            distinct,
            multiplicities,
            positions: self.positions.iter().copied().map(xy).collect(),
            note,
        }
    }
}

//! The event loop: Python `Scheduler.handle_event` and `Robot.look/move/wait`
//! (docs/core-schema.md §5), plus the schedulers the original lacks:
//! sequential (one robot per turn) and SSYNC/FSYNC rounds.

use crate::algorithm::{Algorithm, Assignment, Look, Model, OverlayUpdate, Plan, Scratch, View};
use crate::config::{
    Activation, ActivationOrder, ConfigError, FaultSelection, Frames, ScheduleEnd, SchedulerKind,
    SimConfig, StopPolicy, TurnGap,
};
use crate::event::{Event, EventKind, EventQueue};
use crate::geom::{dist, interpolate, Circle, Point};
use crate::local::LocalFrame;
use crate::pyfloat::pow10_neg;
use crate::rng::Rng;
use crate::robots::{FaultKind, Light, RobotState, Robots};

/// What handling one event did. Mirrors `handle_event`'s exit codes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StepOutcome {
    Ignored,
    Moved,
    Frozen,
    Terminated,
    Crashed,
    Visualize,
    Ended,
}

impl StepOutcome {
    #[must_use]
    pub const fn code(self) -> i32 {
        match self {
            Self::Ignored => 0,
            Self::Moved => 2,
            Self::Frozen => 3,
            Self::Terminated => 4,
            Self::Crashed => 5,
            Self::Visualize => 99,
            Self::Ended => -1,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct StepInfo {
    pub time: f64,
    pub robot: i32,
    pub kind: Option<EventKind>,
    pub outcome: StepOutcome,
}

/// How much `advance` may do before returning.
#[derive(Clone, Copy, Debug)]
pub struct Budget {
    pub max_events: u64,
    /// Stop before the first event later than this.
    pub until_time: f64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StopReason {
    /// `max_events` of the budget handled.
    Budget,
    /// The next event is after `until_time`.
    UntilTime,
    /// Global termination or an empty queue.
    Ended,
    /// The configured `max_events` limit.
    MaxEvents,
    /// The configured `max_time` limit.
    MaxTime,
    /// Sequential scheduler: a whole epoch passed in which no robot moved or
    /// terminated, so nothing ever will (see [`Simulation::stalled`]).
    Stalled,
    /// Sequential scheduler: the configured `max_turns` have been taken.
    MaxTurns,
    /// SSYNC/FSYNC: the configured `max_rounds` have been played.
    MaxRounds,
    /// A schedule ending in `await` has run out: add a turn
    /// ([`Simulation::push_turn`]) or a round ([`Simulation::push_round`]).
    AwaitingSchedule,
}

/// An explicit schedule being played, and which robots have had a turn in
/// the current epoch.
struct Schedule {
    list: Vec<Activation>,
    end: ScheduleEnd,
    cursor: usize,
    seen: Vec<bool>,
    unseen: usize,
    /// The robot whose turn is under way.
    in_turn: Option<usize>,
}

/// Sequential scheduler state: the current epoch's order and how far into it
/// the turns have got.
struct Turns {
    order: ActivationOrder,
    gap: TurnGap,
    epoch: Vec<u32>,
    next: usize,
    epochs_completed: u64,
    schedule: Option<Schedule>,
    /// The stopping point (fraction of the way) the schedule names for the
    /// robot now taking its turn.
    stop: Option<f64>,
    taken: u64,
    /// A robot has moved or terminated since the epoch began.
    progress: bool,
}

/// SSYNC/FSYNC state: the robots of the round under way and how many of
/// them have not finished their move yet.
struct Rounds {
    kind: SchedulerKind,
    gap: TurnGap,
    probability: f64,
    fairness: u32,
    /// Rounds in a row each robot has been left out of (SSYNC).
    idle: Vec<u32>,
    list: Option<Vec<Vec<usize>>>,
    end: ScheduleEnd,
    cursor: usize,
    members: Vec<usize>,
    in_round: Vec<bool>,
    /// Members that have looked in phase one but not finished their Look.
    decided: Vec<bool>,
    pending: usize,
    taken: u64,
    max: Option<u64>,
    epochs_completed: u64,
    seen: Vec<bool>,
    unseen: usize,
    progress: bool,
}

/// k-Async state: which robots are in a cycle and since when, each robot's
/// most recent Look times, and the robots whose Look was put off.
struct Bounded {
    k: usize,
    /// Start time of every cycle under way, ordered: `(time bits, robot)`.
    /// Times are never negative, so their bits sort as the times do.
    active: std::collections::BTreeSet<(u64, usize)>,
    started: Vec<Option<f64>>,
    /// Each robot's last `k` Look times, oldest first.
    recent: Vec<std::collections::VecDeque<f64>>,
    deferred: Vec<usize>,
    waiting: Vec<bool>,
    /// Looks put off so far (for the page and tests).
    deferrals: u64,
}

/// Robot flags as they were when a round began, so every robot of the round
/// sees the same snapshot.
#[derive(Default)]
struct RoundSnapshot {
    state: Vec<RobotState>,
    terminated: Vec<bool>,
    frozen: Vec<bool>,
}

pub struct Simulation {
    model: Model,
    rigid: bool,
    lambda: f64,
    sampling: f64,
    multiplicity: bool,
    max_events: Option<u64>,
    max_time: Option<f64>,
    delta: Option<f64>,
    stop: StopPolicy,
    stop_fraction: f64,
    /// Without faults or randomized robots, a silent epoch means nothing will
    /// ever change.
    stall_check: bool,
    stalled: bool,
    max_turns: Option<u64>,
    turn_limit: bool,
    algorithms: Vec<Box<dyn Algorithm>>,
    robots: Robots,
    queue: EventQueue,
    rng: Rng,
    now: f64,
    ended: bool,
    events: u64,
    ticks: u64,
    view: View,
    scratch: Scratch,
    /// `None` unless the scheduler is sequential.
    turns: Option<Turns>,
    /// `None` unless the scheduler is SSYNC or FSYNC.
    rounds: Option<Rounds>,
    /// `None` unless the scheduler is k-Async.
    bounded: Option<Bounded>,
    snapshot: Option<RoundSnapshot>,
    round_limit: bool,
    /// An `await` schedule has run out and waits for the next activation.
    awaiting: bool,
    frames: Frames,
    /// Draws the robots' private frames, apart from the run's generator so
    /// turning frames on changes nothing else about a run.
    frame_rng: Rng,
    /// `frames: "compass"`: each robot's current compass deviation (radians),
    /// drawn once (static) or at each Look (dynamic).
    compass: Vec<f64>,
    compass_error: f64,
    compass_static: bool,
    obstructed: bool,
    robot_radius: f64,
    positions: Vec<Point>,
}

const ALL_FAULTS: [FaultKind; 4] = [
    FaultKind::Crash,
    FaultKind::Byzantine,
    FaultKind::Omission,
    FaultKind::Delay,
];

impl Simulation {
    /// Builds robots, faults and the initial event queue, drawing random
    /// numbers in the order of `Scheduler.__init__`.
    ///
    /// # Errors
    /// An invalid configuration, or a plan with no algorithms.
    pub fn new(config: &SimConfig, plan: Plan) -> Result<Self, ConfigError> {
        config.validate()?;
        if plan.algorithms.is_empty() {
            return Err(ConfigError::UnknownAlgorithm(config.algorithm.clone()));
        }
        let positions = config.resolve_positions();
        let n = positions.len();
        let (width_bound, height_bound) = config.bounds();
        let precision = config.threshold_precision;
        let model = Model {
            precision,
            eps: pow10_neg(precision),
            visibility: config.visibility_radius.unwrap_or(f64::INFINITY),
            width_bound,
            height_bound,
            multiplicity: config.multiplicity,
        };
        let mut rng = Rng::new(config.random_seed);
        let mut robots = Robots::new(&positions, config.robot_speeds);

        if let Assignment::Random(choices) = &plan.assignment {
            for i in 0..n {
                let u = rng.random();
                let &(_, algo, task) = choices
                    .iter()
                    .find(|(cumulative, _, _)| u < *cumulative)
                    .unwrap_or(&choices[choices.len() - 1]);
                robots.algo[i] = u8::try_from(algo).unwrap_or(0);
                robots.task[i] = task;
            }
        }

        if config.num_of_faults > 0 {
            let k = (config.num_of_faults as usize).min(n);
            for (j, i) in rng.choice(n, k).into_iter().enumerate() {
                let fault = match config.fault_type {
                    FaultSelection::Crash => FaultKind::Crash,
                    FaultSelection::Byzantine => FaultKind::Byzantine,
                    FaultSelection::Omission => FaultKind::Omission,
                    FaultSelection::Delay => FaultKind::Delay,
                    FaultSelection::Mixed => ALL_FAULTS[j % ALL_FAULTS.len()],
                };
                robots.fault[i] = fault;
                match fault {
                    FaultKind::Crash => robots.state[i] = RobotState::Crash,
                    FaultKind::Delay => robots.speed[i] *= 0.4,
                    _ => {}
                }
            }
        }

        let sequential = config.scheduler == SchedulerKind::Sequential;
        let in_rounds = config.scheduler.rounds();
        let live = (0..n)
            .filter(|&i| robots.state[i] != RobotState::Crash)
            .count();
        let mut queue = EventQueue::default();
        for i in (0..n).filter(|_| !sequential && !in_rounds) {
            let time = rng.exponential(1.0 / config.lambda_rate).max(0.0);
            let kind = if robots.state[i] == RobotState::Crash {
                EventKind::Crash
            } else {
                EventKind::Look
            };
            queue.push(Event {
                time,
                robot: robot_id(i),
                kind,
            });
        }
        queue.push(Event {
            time: config.sampling_rate,
            robot: -1,
            kind: EventKind::Visualize,
        });

        let mut sim = Self {
            model,
            rigid: config.rigid_movement,
            lambda: config.lambda_rate,
            sampling: config.sampling_rate,
            multiplicity: config.multiplicity_detection,
            max_events: config.max_events,
            max_time: config.max_time,
            delta: config.delta,
            stop: config.stop_policy,
            stop_fraction: config.stop_fraction,
            // A silent epoch proves a stall only for deterministic robots.
            stall_check: config.num_of_faults == 0
                && !plan.algorithms.iter().any(|a| a.randomized()),
            stalled: false,
            max_turns: config.max_turns,
            turn_limit: false,
            algorithms: plan.algorithms,
            robots,
            queue,
            rng,
            now: 0.0,
            ended: false,
            events: 0,
            ticks: 0,
            view: View::default(),
            scratch: Scratch::default(),
            turns: sequential.then(|| Turns {
                order: config.activation_order,
                gap: config.turn_gap,
                epoch: Vec::with_capacity(n),
                next: 0,
                epochs_completed: 0,
                schedule: config.schedule.as_ref().map(|list| Schedule {
                    list: list.clone(),
                    end: config.schedule_end,
                    cursor: 0,
                    seen: vec![false; n],
                    unseen: live,
                    in_turn: None,
                }),
                stop: None,
                taken: 0,
                progress: false,
            }),
            rounds: in_rounds.then(|| Rounds {
                kind: config.scheduler,
                gap: config.turn_gap,
                probability: config.ssync_probability,
                fairness: config.ssync_fairness,
                idle: vec![0; n],
                list: config.rounds.clone(),
                end: config.schedule_end,
                cursor: 0,
                members: Vec::new(),
                in_round: vec![false; n],
                decided: vec![false; n],
                pending: 0,
                taken: 0,
                max: config.max_rounds,
                epochs_completed: 0,
                seen: vec![false; n],
                unseen: live,
                progress: false,
            }),
            bounded: (config.scheduler == SchedulerKind::KAsync).then(|| Bounded {
                k: config.k_async as usize,
                active: std::collections::BTreeSet::new(),
                started: vec![None; n],
                recent: vec![std::collections::VecDeque::new(); n],
                deferred: Vec::new(),
                waiting: vec![false; n],
                deferrals: 0,
            }),
            snapshot: None,
            round_limit: false,
            awaiting: false,
            frames: config.frames,
            frame_rng: Rng::new(config.random_seed ^ 0x9E37_79B9_7F4A_7C15),
            compass: Vec::new(),
            compass_error: config.compass_error.to_radians(),
            compass_static: config.compass_static,
            obstructed: config.obstructed_visibility,
            robot_radius: config.robot_radius,
            positions: Vec::new(),
        };
        if sim.frames == Frames::Compass {
            let phi = sim.compass_error;
            sim.compass = (0..n).map(|_| sim.frame_rng.uniform(-phi, phi)).collect();
        }
        if sequential {
            sim.next_turn(0.0);
        }
        if in_rounds {
            sim.next_round(0.0);
        }
        Ok(sim)
    }

    #[must_use]
    pub fn time(&self) -> f64 {
        self.now
    }

    #[must_use]
    pub fn ended(&self) -> bool {
        self.ended
    }

    /// Events handled so far (including visualize and ignored events).
    #[must_use]
    pub fn event_count(&self) -> u64 {
        self.events
    }

    #[must_use]
    pub fn sequential(&self) -> bool {
        self.turns.is_some()
    }

    /// `frames: "compass"`: each robot's compass deviation from the global
    /// north, in radians (the last one drawn, for a dynamic compass). Empty
    /// otherwise.
    #[must_use]
    pub fn compass_deviations(&self) -> &[f64] {
        &self.compass
    }

    /// SSYNC or FSYNC: robots act in rounds of simultaneous activations.
    #[must_use]
    pub fn in_rounds(&self) -> bool {
        self.rounds.is_some()
    }

    /// SSYNC/FSYNC: rounds begun so far. 0 for the other schedulers.
    #[must_use]
    pub fn round_count(&self) -> u64 {
        self.rounds.as_ref().map_or(0, |r| r.taken)
    }

    /// SSYNC/FSYNC: the robots active in the round under way.
    #[must_use]
    pub fn round_members(&self) -> &[usize] {
        self.rounds.as_ref().map_or(&[], |r| r.members.as_slice())
    }

    /// A schedule ending in `await` has run out: the run waits for
    /// [`Self::push_turn`] or [`Self::push_round`].
    #[must_use]
    pub fn awaiting_schedule(&self) -> bool {
        self.awaiting
    }

    /// Sequential `await` schedule: adds a turn for `robot`, stopped after
    /// `stop` of the way if given (non-rigid with a `delta`). A turn for a
    /// robot that has crashed or terminated is skipped.
    ///
    /// # Errors
    /// Not a sequential run with a schedule, or no such robot.
    pub fn push_turn(&mut self, robot: usize, stop: Option<f64>) -> Result<(), ConfigError> {
        if robot >= self.robots.len() {
            return Err(ConfigError::Schedule(format!("there is no robot {robot}")));
        }
        if let Some(stop) = stop {
            if !(stop > 0.0 && stop <= 1.0) || self.rigid || self.delta.is_none() {
                return Err(ConfigError::Schedule(
                    "a stopping point needs non-rigid movement with a delta, and a fraction in (0, 1]"
                        .to_owned(),
                ));
            }
        }
        let Some(schedule) = self.turns.as_mut().and_then(|t| t.schedule.as_mut()) else {
            return Err(ConfigError::Schedule(
                "push_turn needs the sequential scheduler with a schedule".to_owned(),
            ));
        };
        schedule.list.push(match stop {
            Some(stop) => Activation::Stopped { robot, stop },
            None => Activation::Robot(robot),
        });
        if self.awaiting {
            self.awaiting = false;
            self.next_turn(self.now);
        }
        Ok(())
    }

    /// SSYNC `await` rounds: adds a round activating `robots` at once.
    ///
    /// # Errors
    /// Not an SSYNC run with rounds, an empty round, or no such robot.
    pub fn push_round(&mut self, robots: Vec<usize>) -> Result<(), ConfigError> {
        let n = self.robots.len();
        if robots.is_empty() || robots.iter().any(|&r| r >= n) {
            return Err(ConfigError::Schedule(
                "a round needs at least one robot, each of them existing".to_owned(),
            ));
        }
        let Some(list) = self.rounds.as_mut().and_then(|r| r.list.as_mut()) else {
            return Err(ConfigError::Schedule(
                "push_round needs the SSYNC scheduler with rounds".to_owned(),
            ));
        };
        list.push(robots);
        if self.awaiting {
            self.awaiting = false;
            self.next_round(self.now);
        }
        Ok(())
    }

    /// What robot `i` would decide if it looked now, without changing the
    /// run: its destination in global coordinates (`None` if it would not
    /// compute, e.g. crashed, terminated or Byzantine). For an adversary
    /// choosing turns. Uses a copy of the generators.
    #[must_use]
    pub fn preview_target(&mut self, i: usize) -> Option<Point> {
        let r = &self.robots;
        if i >= r.len()
            || r.state[i] == RobotState::Crash
            || r.terminated[i]
            || r.fault[i] == FaultKind::Byzantine
        {
            return None;
        }
        let me_index = self.build_view(i, self.now);
        let position = self.robots.pos[i];
        let mut rng = self.rng.clone();
        let mut frame_rng = self.frame_rng.clone();
        let (target, _, _) = self.compute_in_frame(i, me_index, position, &mut rng, &mut frame_rng);
        Some(target)
    }

    /// Why robot `i` would do what it does if it looked now, in the words of
    /// its algorithm (`None` if it would not compute, or the algorithm gives
    /// no explanation). Changes nothing about the run.
    pub fn explain(&mut self, i: usize) -> Option<String> {
        let r = &self.robots;
        if i >= r.len()
            || r.state[i] == RobotState::Crash
            || r.terminated[i]
            || r.fault[i] == FaultKind::Byzantine
        {
            return None;
        }
        let me_index = self.build_view(i, self.now);
        let position = self.robots.pos[i];
        let mut frame_rng = self.frame_rng.clone();
        let compass = self.compass.get(i).copied();
        let (text, _) = self.in_frame(i, me_index, position, &mut frame_rng, |a, look, scratch| {
            a.explain(look, scratch)
        });
        // A dynamic compass drew a deviation for this preview only.
        if let Some(c) = compass {
            self.compass[i] = c;
        }
        text
    }

    /// Visualize ticks among the events handled: they belong to no robot.
    #[must_use]
    pub fn tick_count(&self) -> u64 {
        self.ticks
    }

    /// Sequential scheduler: a whole epoch passed without any robot moving or
    /// terminating (only checked without faults), so the run can never change.
    /// `advance` then returns [`StopReason::Stalled`].
    #[must_use]
    pub fn stalled(&self) -> bool {
        self.stalled
    }

    /// Sequential scheduler: `max_turns` turns have been taken, so no more
    /// will be. `advance` then returns [`StopReason::MaxTurns`].
    #[must_use]
    pub fn turn_limit_reached(&self) -> bool {
        self.turn_limit
    }

    /// Sequential scheduler: turns taken so far (one per Look). 0 for async.
    #[must_use]
    pub fn turn_count(&self) -> u64 {
        self.turns.as_ref().map_or(0, |t| t.taken)
    }

    /// Sequential and SSYNC/FSYNC schedulers: epochs in which every live
    /// robot has been active. Always 0 for async.
    #[must_use]
    pub fn epochs_completed(&self) -> u64 {
        match (&self.turns, &self.rounds) {
            (Some(t), _) => t.epochs_completed,
            (None, Some(r)) => r.epochs_completed,
            (None, None) => 0,
        }
    }

    #[must_use]
    pub fn robots(&self) -> &Robots {
        &self.robots
    }

    #[must_use]
    pub fn model(&self) -> &Model {
        &self.model
    }

    #[must_use]
    pub fn multiplicity_detection(&self) -> bool {
        self.multiplicity
    }

    /// Registry key of the algorithm robot `i` runs.
    #[must_use]
    pub fn algorithm_key(&self, i: usize) -> &'static str {
        self.algorithms[usize::from(self.robots.algo[i])].key()
    }

    #[must_use]
    pub fn position_at(&self, i: usize, t: f64) -> Point {
        self.robots.position_at(i, t, self.model.eps)
    }

    /// The time of the next event, if there is one. Read-only, like `arrival_time`.
    #[must_use]
    pub fn next_event_time(&self) -> Option<f64> {
        self.queue.peek().map(|e| e.time)
    }

    /// When robot `i` arrives, if it is on a move right now: the time its move started plus its
    /// length over its speed, the same sum `begin_move` schedules the arrival event with. A reader
    /// of the simulation (the page paces a turn by it); nothing in the run depends on it.
    #[must_use]
    pub fn arrival_time(&self, i: usize) -> Option<f64> {
        let r = &self.robots;
        let (RobotState::Move, Some(start), Some(goal)) = (r.state[i], r.start_time[i], r.goal(i))
        else {
            return None;
        };
        let duration = if r.speed[i] > 1e-9 {
            dist(r.start_pos[i], goal) / r.speed[i]
        } else {
            0.0
        };
        Some(start + duration.max(0.0))
    }

    /// The reason the run cannot go on, if it cannot: ended, stalled, out of turns, or at a
    /// configured limit. The same checks, in the same order, as `advance` makes before each event.
    fn limit_reached(&self) -> Option<StopReason> {
        if self.ended {
            Some(StopReason::Ended)
        } else if self.stalled {
            Some(StopReason::Stalled)
        } else if self.turn_limit {
            Some(StopReason::MaxTurns)
        } else if self.round_limit {
            Some(StopReason::MaxRounds)
        } else if self.awaiting {
            Some(StopReason::AwaitingSchedule)
        } else if self.max_time.is_some_and(|limit| self.now > limit) {
            Some(StopReason::MaxTime)
        } else if self.max_events.is_some_and(|limit| self.events >= limit) {
            Some(StopReason::MaxEvents)
        } else {
            None
        }
    }

    /// Handles one event, unless the run has ended or hit a limit.
    ///
    /// # Errors
    /// The reason it cannot go on, as `advance` would report it.
    pub fn step_checked(&mut self) -> Result<StepInfo, StopReason> {
        match self.limit_reached() {
            Some(reason) => Err(reason),
            None => Ok(self.step()),
        }
    }

    /// Handles events until the budget, a limit or the end is reached.
    pub fn advance(&mut self, budget: Budget) -> StopReason {
        let mut handled = 0;
        loop {
            if let Some(reason) = self.limit_reached() {
                return reason;
            }
            if handled >= budget.max_events {
                return StopReason::Budget;
            }
            if self
                .queue
                .peek()
                .is_some_and(|e| e.time > budget.until_time)
            {
                return StopReason::UntilTime;
            }
            self.step();
            handled += 1;
        }
    }

    /// Handles the next event.
    pub fn step(&mut self) -> StepInfo {
        let ended = StepInfo {
            time: self.now,
            robot: -1,
            kind: None,
            outcome: StepOutcome::Ended,
        };
        if self.ended || self.stalled || self.turn_limit || self.round_limit || self.awaiting {
            return ended;
        }
        let Some(event) = self.queue.pop() else {
            self.ended = true;
            return ended;
        };
        self.events += 1;
        if event.kind == EventKind::Visualize {
            self.ticks += 1;
        }
        let info = |outcome| StepInfo {
            time: event.time,
            robot: event.robot,
            kind: Some(event.kind),
            outcome,
        };
        if event.time < self.now {
            return StepInfo {
                time: self.now,
                ..info(StepOutcome::Ignored)
            };
        }
        let t = event.time;
        self.now = t;

        if event.robot < 0 {
            if event.kind == EventKind::Visualize {
                self.queue.push(Event {
                    time: t + self.sampling,
                    robot: -1,
                    kind: EventKind::Visualize,
                });
                return info(StepOutcome::Visualize);
            }
            return info(StepOutcome::Ignored);
        }
        let i = usize::try_from(event.robot).unwrap_or(usize::MAX);
        if i >= self.robots.len() {
            return info(StepOutcome::Ignored);
        }
        if self.robots.state[i] == RobotState::Crash && event.kind != EventKind::Crash {
            self.schedule_activation(t, i);
            return info(StepOutcome::Ignored);
        }
        // A round's member that terminated when it computed still has to
        // finish its Look, which ends its part of the round.
        if self.robots.terminated[i] && !self.rounds.as_ref().is_some_and(|r| r.decided[i]) {
            return info(StepOutcome::Ignored);
        }

        let outcome = match event.kind {
            EventKind::Look => {
                if let Some(turns) = self.turns.as_mut() {
                    turns.taken += 1;
                }
                if self.rounds.is_some() {
                    self.round_look(i, t)
                } else if self.bounded.is_some() && !self.may_look(i, t) {
                    StepOutcome::Ignored
                } else {
                    self.look(i, t)
                }
            }
            EventKind::Wait => {
                self.wait(i, t);
                self.schedule_activation(t, i);
                StepOutcome::Frozen
            }
            EventKind::Crash => {
                self.robots.fault[i] = FaultKind::Crash;
                self.robots.state[i] = RobotState::Crash;
                StepOutcome::Crashed
            }
            EventKind::Visualize => StepOutcome::Ignored,
        };

        if matches!(outcome, StepOutcome::Moved | StepOutcome::Terminated) {
            if let Some(turns) = self.turns.as_mut() {
                turns.progress = true;
            }
            if let Some(rounds) = self.rounds.as_mut() {
                rounds.progress = true;
            }
        }
        if self.globally_terminated() {
            self.ended = true;
            return info(StepOutcome::Ended);
        }
        info(outcome)
    }

    /// The snapshot robot `i` takes at `t`, filtered by its visibility (and,
    /// with obstructed visibility, by the robots in the way). In a round,
    /// flags are those from the start of the round.
    fn build_view(&mut self, i: usize, t: f64) -> usize {
        let me = self.robots.pos[i];
        let visibility = self.model.visibility;
        let unlimited = visibility == f64::INFINITY;
        let eps = self.model.eps;
        if self.snapshot.is_none() && !self.obstructed {
            // The common case (async and sequential runs): the original loop, kept tight.
            self.view.clear();
            let mut me_index = 0;
            for j in 0..self.robots.len() {
                let p = self.robots.position_at(j, t, eps);
                if unlimited || dist(me, p) <= visibility {
                    if j == i {
                        me_index = self.view.len();
                    }
                    let r = &self.robots;
                    self.view
                        .push(robot_id_u32(j), p, r.state[j], r.terminated[j], r.frozen[j]);
                }
            }
            return me_index;
        }
        if self.obstructed {
            self.positions.clear();
            for j in 0..self.robots.len() {
                self.positions.push(self.robots.position_at(j, t, eps));
            }
        }
        self.view.clear();
        let mut me_index = 0;
        for j in 0..self.robots.len() {
            let p = self.robots.position_at(j, t, eps);
            if !(unlimited || dist(me, p) <= visibility) {
                continue;
            }
            if j != i && self.obstructed && self.hidden(i, me, j, p) {
                continue;
            }
            if j == i {
                me_index = self.view.len();
            }
            let (state, terminated, frozen) = match &self.snapshot {
                Some(s) => (s.state[j], s.terminated[j], s.frozen[j]),
                None => {
                    let r = &self.robots;
                    (r.state[j], r.terminated[j], r.frozen[j])
                }
            };
            self.view
                .push(robot_id_u32(j), p, state, terminated, frozen);
        }
        me_index
    }

    /// Obstructed visibility: some other robot stands on the segment from
    /// robot `i` at `me` to robot `j` at `p`, strictly between them (within
    /// the robot radius, or ε for point robots). Robots sharing `me` or `p`
    /// do not block. Uses `self.positions`.
    fn hidden(&self, i: usize, me: Point, j: usize, p: Point) -> bool {
        let block = self.robot_radius.max(self.model.eps);
        let (dx, dy) = (p.x - me.x, p.y - me.y);
        let length2 = dx * dx + dy * dy;
        if length2 <= block * block {
            return false;
        }
        self.positions.iter().enumerate().any(|(k, &q)| {
            if k == i || k == j || dist(q, me) <= block || dist(q, p) <= block {
                return false;
            }
            let s = ((q.x - me.x) * dx + (q.y - me.y) * dy) / length2;
            if s <= 0.0 || s >= 1.0 {
                return false;
            }
            let foot = Point::new(me.x + s * dx, me.y + s * dy);
            dist(q, foot) <= block
        })
    }

    /// Runs `f` on robot `i`'s Look of the view just built, in its own frame
    /// when frames are on; returns its result and that frame (`None` for
    /// global coordinates).
    fn in_frame<R>(
        &mut self,
        i: usize,
        me_index: usize,
        position: Point,
        frame_rng: &mut Rng,
        f: impl FnOnce(&dyn Algorithm, &Look<'_>, &mut Scratch) -> R,
    ) -> (R, Option<LocalFrame>) {
        let algorithm = self.algorithms[usize::from(self.robots.algo[i])].as_ref();
        if self.frames == Frames::Global {
            let look = Look {
                me: robot_id_u32(i),
                me_index,
                position,
                view: &self.view,
                model: &self.model,
            };
            return (f(algorithm, &look, &mut self.scratch), None);
        }
        let frame = if self.frames == Frames::Compass {
            if !self.compass_static {
                let phi = self.compass_error;
                self.compass[i] = frame_rng.uniform(-phi, phi);
            }
            // Axes turned by the deviation: a point due north of the robot
            // reads as `deviation` off its own north.
            LocalFrame {
                origin: position,
                angle: -self.compass[i],
                scale: 1.0,
                reflect: false,
            }
        } else {
            LocalFrame::random(position, frame_rng, self.frames == Frames::Rotated)
        };
        for p in &mut self.view.pos {
            *p = frame.to_local(*p);
        }
        let model = Model {
            eps: self.model.eps * frame.scale,
            visibility: self.model.visibility * frame.scale,
            // The world box is drawn in global coordinates, unknown to a
            // disoriented robot.
            width_bound: None,
            height_bound: None,
            ..self.model
        };
        let look = Look {
            me: robot_id_u32(i),
            me_index,
            position: Point::new(0.0, 0.0),
            view: &self.view,
            model: &model,
        };
        (f(algorithm, &look, &mut self.scratch), Some(frame))
    }

    /// Robot `i`'s Compute on the view just built, in its own frame when
    /// frames are on: the destination and circle come back in global
    /// coordinates. Returns `(target, terminate, overlay)`.
    fn compute_in_frame(
        &mut self,
        i: usize,
        me_index: usize,
        position: Point,
        rng: &mut Rng,
        frame_rng: &mut Rng,
    ) -> (Point, bool, OverlayUpdate) {
        let (d, frame) = self.in_frame(i, me_index, position, frame_rng, |a, look, scratch| {
            a.compute(look, rng, scratch)
        });
        let Some(frame) = frame else {
            return (d.target, d.terminate, d.overlay);
        };
        let overlay = match d.overlay {
            OverlayUpdate::Set(c) => OverlayUpdate::Set(Circle::new(
                frame.to_global(c.center),
                c.radius / frame.scale,
            )),
            other => other,
        };
        (frame.to_global(d.target), d.terminate, overlay)
    }

    /// `Robot.look` followed by the LOOK branch of `handle_event`.
    fn look(&mut self, i: usize, t: f64) -> StepOutcome {
        self.decide(i, t);
        self.finish_look(i, t)
    }

    /// SSYNC/FSYNC: the first Look of a round lets every member look at the
    /// same snapshot and compute; then each member's own Look event starts
    /// its move.
    fn round_look(&mut self, i: usize, t: f64) -> StepOutcome {
        let decided = self.rounds.as_ref().is_some_and(|r| r.decided[i]);
        if !decided {
            let members = self
                .rounds
                .as_ref()
                .map(|r| r.members.clone())
                .unwrap_or_default();
            let r = &self.robots;
            self.snapshot = Some(RoundSnapshot {
                state: r.state.clone(),
                terminated: r.terminated.clone(),
                frozen: r.frozen.clone(),
            });
            for &m in &members {
                self.decide(m, t);
            }
            self.snapshot = None;
            if let Some(rounds) = self.rounds.as_mut() {
                for &m in &members {
                    rounds.decided[m] = true;
                }
            }
        }
        if let Some(rounds) = self.rounds.as_mut() {
            rounds.decided[i] = false;
        }
        self.finish_look(i, t)
    }

    /// Look and Compute: sets robot `i`'s target and flags.
    fn decide(&mut self, i: usize, t: f64) {
        let me_index = self.build_view(i, t);
        let r = &mut self.robots;
        r.state[i] = RobotState::Look;
        r.set_light(i, Light::Blue, t);
        let position = r.pos[i];

        if r.fault[i] == FaultKind::Byzantine {
            let reach = self
                .view
                .id
                .iter()
                .zip(&self.view.pos)
                .filter(|(id, _)| **id as usize != i)
                .map(|(_, p)| dist(position, *p))
                .fold(None, |best: Option<f64>, d| {
                    Some(best.map_or(d, |b| if d > b { d } else { b }))
                })
                .map_or(10.0, |d| d * 0.5);
            let angle = self.rng.uniform(0.0, 2.0 * std::f64::consts::PI);
            r.target[i] = Some(Point::new(
                position.x + reach * libm::cos(angle),
                position.y + reach * libm::sin(angle),
            ));
            r.frozen[i] = false;
            r.terminated[i] = false;
        } else {
            let active = (0..self.view.len())
                .filter(|&k| !self.view.terminated[k] && self.view.state[k] != RobotState::Crash)
                .count();
            // The original simulator's rule: a robot that sees no other active
            // robot is done. Algorithms whose finished robots still matter
            // (a meeting point to walk to) opt out.
            let alone_stops = self.algorithms[usize::from(r.algo[i])].stops_when_alone();
            if active <= 1 && alone_stops {
                r.frozen[i] = true;
                r.terminated[i] = true;
                self.wait(i, t);
            } else {
                let (target, terminate, overlay) = if self.frames == Frames::Global {
                    // The common case, as the original computes it.
                    let look = Look {
                        me: robot_id_u32(i),
                        me_index,
                        position,
                        view: &self.view,
                        model: &self.model,
                    };
                    let algorithm = &self.algorithms[usize::from(self.robots.algo[i])];
                    let d = algorithm.compute(&look, &mut self.rng, &mut self.scratch);
                    (d.target, d.terminate, d.overlay)
                } else {
                    let mut rng = std::mem::replace(&mut self.rng, Rng::new(0));
                    let mut frame_rng = std::mem::replace(&mut self.frame_rng, Rng::new(0));
                    let out =
                        self.compute_in_frame(i, me_index, position, &mut rng, &mut frame_rng);
                    self.rng = rng;
                    self.frame_rng = frame_rng;
                    out
                };
                let r = &mut self.robots;
                r.target[i] = Some(target);
                match overlay {
                    OverlayUpdate::Keep => {}
                    OverlayUpdate::Set(c) => r.overlay[i] = Some(c),
                    OverlayUpdate::Clear => r.overlay[i] = None,
                }
                if terminate {
                    r.terminated[i] = true;
                }
                if r.terminated[i] {
                    self.wait(i, t);
                } else {
                    let omit = r.fault[i] == FaultKind::Omission && self.rng.random() < 0.5;
                    if omit || dist(target, position) < self.model.eps {
                        self.robots.frozen[i] = true;
                        self.wait(i, t);
                    } else {
                        self.robots.frozen[i] = false;
                    }
                }
            }
        }
    }

    /// The rest of the LOOK branch once robot `i` has decided: terminate,
    /// stay, or start the move.
    fn finish_look(&mut self, i: usize, t: f64) -> StepOutcome {
        let r = &self.robots;
        if r.state[i] == RobotState::Crash {
            return StepOutcome::Crashed;
        }
        if r.terminated[i] {
            if self.turns.is_some() {
                self.next_turn(t);
            }
            if self.rounds.is_some() {
                self.member_done(t, i);
            }
            if self.bounded.is_some() {
                self.cycle_done(t, i);
            }
            return StepOutcome::Terminated;
        }
        if r.frozen[i] {
            self.schedule_activation(t, i);
            return StepOutcome::Frozen;
        }
        self.begin_move(i, t)
    }

    /// `Robot.move` and the WAIT scheduling that follows it.
    fn begin_move(&mut self, i: usize, t: f64) -> StepOutcome {
        let Some(destination) = self.robots.target[i] else {
            self.robots.state[i] = RobotState::Wait;
            self.schedule_activation(t, i);
            return StepOutcome::Frozen;
        };
        let goal = self.limited_target(i, destination);
        let r = &mut self.robots;
        r.stop[i] = (goal != destination).then_some(goal);
        r.state[i] = RobotState::Move;
        r.set_light(i, Light::Red, t);
        r.start_time[i] = Some(t);
        r.start_pos[i] = r.pos[i];
        let distance = dist(r.start_pos[i], goal);
        let duration = if r.speed[i] > 1e-9 {
            distance / r.speed[i]
        } else {
            0.0
        };
        let arrival = t + duration.max(0.0);
        if arrival <= t {
            // Sub-tick move: finish it now rather than schedule a WAIT at the same instant.
            self.wait(i, t);
            self.schedule_activation(t, i);
            StepOutcome::Frozen
        } else {
            self.queue.push(Event {
                time: arrival,
                robot: robot_id(i),
                kind: EventKind::Wait,
            });
            StepOutcome::Moved
        }
    }

    /// Non-rigid movement with a `delta`: where the adversary stops robot `i`
    /// on its way to `destination`. A robot always covers at least `delta`
    /// (or reaches the destination if that is nearer); a schedule can name the
    /// fraction of the way for one turn. Without a `delta`, or with rigid
    /// movement, the robot reaches its destination (the original behaviour).
    fn limited_target(&mut self, i: usize, destination: Point) -> Point {
        let named = self.turns.as_mut().and_then(|turns| turns.stop.take());
        let Some(delta) = self.delta.filter(|_| !self.rigid) else {
            return destination;
        };
        let from = self.robots.pos[i];
        let length = dist(from, destination);
        if length <= delta {
            return destination;
        }
        let wanted = match (named, self.stop) {
            (Some(fraction), _) => fraction * length,
            (None, StopPolicy::Delta) => delta,
            (None, StopPolicy::Random) => self.rng.uniform(delta, length),
            (None, StopPolicy::Fraction) => self.stop_fraction * length,
        };
        let covered = wanted.clamp(delta, length);
        if covered >= length {
            destination
        } else {
            interpolate(from, destination, covered / length)
        }
    }

    /// `Robot.wait`.
    fn wait(&mut self, i: usize, t: f64) {
        let eps = self.model.eps;
        let r = &mut self.robots;
        let moving = r.state[i] == RobotState::Move;
        let snap_to_target = match (moving, r.start_time[i], r.goal(i)) {
            (true, Some(start), Some(target))
                if self.rigid || self.delta.is_some() || t <= start + 1e-12 =>
            {
                Some(target)
            }
            _ => None,
        };
        let end = snap_to_target.unwrap_or_else(|| r.position_at(i, t, eps));
        if moving && r.start_time[i].is_some() {
            r.travelled[i] += dist(r.start_pos[i], end);
        }
        r.pos[i] = end;
        r.start_time[i] = None;
        r.stop[i] = None;
        r.state[i] = RobotState::Wait;
        r.set_light(i, Light::Green, t);
    }

    /// `Scheduler.generate_event`: the random delay is drawn even when a
    /// terminated robot gets no new activation.
    /// Under the sequential scheduler, robot `i`'s turn is over instead.
    fn schedule_activation(&mut self, previous: f64, i: usize) {
        if self.turns.is_some() {
            self.next_turn(previous);
            return;
        }
        if self.rounds.is_some() {
            self.member_done(previous, i);
            return;
        }
        if self.bounded.is_some() {
            self.cycle_done(previous, i);
        }
        let delay = self.rng.exponential(1.0 / self.lambda);
        let time = previous + delay.max(1e-9);
        let kind = if self.robots.state[i] == RobotState::Crash {
            EventKind::Crash
        } else if self.robots.terminated[i] {
            return;
        } else {
            EventKind::Look
        };
        self.queue.push(Event {
            time,
            robot: robot_id(i),
            kind,
        });
    }

    /// Sequential scheduler: queues the Look of the next robot to take a turn:
    /// the next live robot of the epoch (or of the explicit schedule), starting
    /// a new epoch when this one runs out. Queues nothing if no robot is left
    /// to move, or if the finished epoch changed nothing (the run is stalled).
    fn next_turn(&mut self, previous: f64) {
        let Some(turns) = self.turns.as_mut() else {
            return;
        };
        let r = &self.robots;
        let live = |i: usize| r.state[i] != RobotState::Crash && !r.terminated[i];
        turns.stop = None;
        let robot = if let Some(s) = turns.schedule.as_mut() {
            // An epoch ends when every robot that was live at its start has had a turn.
            if let Some(done) = s.in_turn.take() {
                if !s.seen[done] {
                    s.seen[done] = true;
                    s.unseen -= 1;
                }
                if s.unseen == 0 {
                    turns.epochs_completed += 1;
                    if !turns.progress && self.stall_check {
                        self.stalled = true;
                        return;
                    }
                    turns.progress = false;
                    s.seen.fill(false);
                    s.unseen = (0..r.len()).filter(|&i| live(i)).count();
                }
            }
            let (n, len) = (r.len(), s.list.len());
            let mut skipped = 0;
            let picked = loop {
                let k = s.cursor;
                if k >= len && s.end == ScheduleEnd::Await {
                    self.awaiting = true;
                    break None;
                }
                s.cursor += 1;
                let (robot, stop) = if k < len {
                    (s.list[k].robot(), s.list[k].stop())
                } else if s.end == ScheduleEnd::Repeat {
                    (s.list[k % len].robot(), s.list[k % len].stop())
                } else {
                    ((k - len) % n, None)
                };
                if live(robot) {
                    break Some((robot, stop));
                }
                skipped += 1;
                if skipped > len + n {
                    break None;
                }
            };
            let Some((robot, stop)) = picked else {
                return;
            };
            s.in_turn = Some(robot);
            turns.stop = stop;
            robot
        } else {
            let mut fresh = false;
            loop {
                if let Some(&i) = turns.epoch.get(turns.next) {
                    turns.next += 1;
                    if live(i as usize) {
                        break i as usize;
                    }
                    continue;
                }
                if fresh {
                    return;
                }
                if !turns.epoch.is_empty() {
                    turns.epochs_completed += 1;
                    if !turns.progress && self.stall_check {
                        self.stalled = true;
                        return;
                    }
                    turns.progress = false;
                }
                turns.epoch.clear();
                turns.epoch.extend((0..r.len()).map(robot_id_u32));
                if turns.order == ActivationOrder::Random {
                    self.rng.shuffle(&mut turns.epoch);
                }
                turns.next = 0;
                fresh = true;
            }
        };
        if self.max_turns.is_some_and(|max| turns.taken >= max) {
            self.turn_limit = true;
            return;
        }
        let time = match turns.gap {
            TurnGap::Random => previous + self.rng.exponential(1.0 / self.lambda).max(1e-9),
            TurnGap::None => previous,
        };
        self.queue.push(Event {
            time,
            robot: robot_id(robot),
            kind: EventKind::Look,
        });
    }

    /// k-Async: whether robot `i` may start a cycle at `t`. It may not if it
    /// has already started `k` Looks since the oldest cycle still under way
    /// began (counting since the oldest start covers every later one); then
    /// it waits for a cycle to end. Otherwise its cycle starts now.
    fn may_look(&mut self, i: usize, t: f64) -> bool {
        let Some(b) = self.bounded.as_mut() else {
            return true;
        };
        let oldest = b
            .active
            .iter()
            .find(|(_, j)| *j != i)
            .map(|(bits, _)| f64::from_bits(*bits));
        let recent = &b.recent[i];
        if let Some(since) = oldest {
            if recent.len() >= b.k && recent.front().is_some_and(|&first| first >= since) {
                if !b.waiting[i] {
                    b.waiting[i] = true;
                    b.deferred.push(i);
                }
                b.deferrals += 1;
                return false;
            }
        }
        let recent = &mut b.recent[i];
        recent.push_back(t);
        if recent.len() > b.k {
            recent.pop_front();
        }
        b.started[i] = Some(t);
        b.active.insert((t.max(0.0).to_bits(), i));
        true
    }

    /// k-Async: robot `i`'s cycle is over. Robots that were put off look
    /// again now (each checks the bound afresh).
    fn cycle_done(&mut self, t: f64, i: usize) {
        let Some(b) = self.bounded.as_mut() else {
            return;
        };
        if let Some(start) = b.started[i].take() {
            b.active.remove(&(start.max(0.0).to_bits(), i));
        }
        for j in std::mem::take(&mut b.deferred) {
            b.waiting[j] = false;
            self.queue.push(Event {
                time: t,
                robot: robot_id(j),
                kind: EventKind::Look,
            });
        }
    }

    /// How many robots see every other robot right now, with the run's
    /// visibility radius and obstruction (mutual visibility is reached when
    /// all of them do). Costs a Look per robot.
    pub fn seeing_everyone(&mut self) -> usize {
        let (n, t) = (self.robots.len(), self.now);
        let mut count = 0;
        for i in 0..n {
            self.build_view(i, t);
            if self.view.len() == n {
                count += 1;
            }
        }
        count
    }

    /// k-Async: Looks put off so far because they would have broken the bound.
    #[must_use]
    pub fn deferred_looks(&self) -> u64 {
        self.bounded.as_ref().map_or(0, |b| b.deferrals)
    }

    /// k-Async: robots in a cycle right now.
    #[must_use]
    pub fn active_cycles(&self) -> usize {
        self.bounded.as_ref().map_or(0, |b| b.active.len())
    }

    /// SSYNC/FSYNC: robot `i` has finished its part of the round; the next
    /// round begins once every member has.
    fn member_done(&mut self, t: f64, i: usize) {
        let Some(rounds) = self.rounds.as_mut() else {
            return;
        };
        if i >= rounds.in_round.len() || !rounds.in_round[i] {
            return;
        }
        rounds.in_round[i] = false;
        rounds.pending -= 1;
        if rounds.pending == 0 {
            self.next_round(t);
        }
    }

    /// SSYNC/FSYNC: closes the round just played (an epoch ends once every
    /// robot live at its start has been active), picks the next round's
    /// robots and queues their Looks, all at the same instant.
    fn next_round(&mut self, previous: f64) {
        let Some(rounds) = self.rounds.as_mut() else {
            return;
        };
        let r = &self.robots;
        let live = |i: usize| r.state[i] != RobotState::Crash && !r.terminated[i];
        let finished = std::mem::take(&mut rounds.members);
        if !finished.is_empty() {
            for &m in &finished {
                if !rounds.seen[m] {
                    rounds.seen[m] = true;
                    rounds.unseen = rounds.unseen.saturating_sub(1);
                }
            }
            // Robots that crashed or terminated during the epoch owe no turn.
            rounds.unseen = (0..r.len()).filter(|&i| live(i) && !rounds.seen[i]).count();
            if rounds.unseen == 0 {
                rounds.epochs_completed += 1;
                if !rounds.progress && self.stall_check {
                    self.stalled = true;
                    return;
                }
                rounds.progress = false;
                rounds.seen.fill(false);
                rounds.unseen = (0..r.len()).filter(|&i| live(i)).count();
            }
        }
        let alive: Vec<usize> = (0..r.len()).filter(|&i| live(i)).collect();
        if alive.is_empty() {
            return;
        }
        let mut members: Vec<usize> = Vec::new();
        let mut explicit = false;
        if let Some(list) = rounds.list.as_ref() {
            let len = list.len();
            let mut skipped = 0;
            loop {
                let k = rounds.cursor;
                if k >= len {
                    match rounds.end {
                        ScheduleEnd::Await => {
                            self.awaiting = true;
                            return;
                        }
                        ScheduleEnd::RoundRobin => break,
                        ScheduleEnd::Repeat => {}
                    }
                }
                rounds.cursor += 1;
                members = list[k % len].iter().copied().filter(|&m| live(m)).collect();
                members.sort_unstable();
                members.dedup();
                if !members.is_empty() {
                    explicit = true;
                    break;
                }
                skipped += 1;
                if skipped > len {
                    break;
                }
            }
        }
        if !explicit {
            members = match rounds.kind {
                SchedulerKind::Fsync => alive.clone(),
                _ => {
                    let mut chosen: Vec<usize> = alive
                        .iter()
                        .copied()
                        .filter(|&m| {
                            rounds.idle[m] >= rounds.fairness
                                || self.rng.random() < rounds.probability
                        })
                        .collect();
                    if chosen.is_empty() {
                        chosen.push(alive[self.rng.index(alive.len())]);
                    }
                    chosen
                }
            };
        }
        for &m in &alive {
            rounds.idle[m] = rounds.idle[m].saturating_add(1);
        }
        for &m in &members {
            rounds.idle[m] = 0;
        }
        if rounds.max.is_some_and(|max| rounds.taken >= max) {
            self.round_limit = true;
            return;
        }
        rounds.taken += 1;
        let time = match rounds.gap {
            TurnGap::Random => previous + self.rng.exponential(1.0 / self.lambda).max(1e-9),
            TurnGap::None => previous,
        };
        for &m in &members {
            rounds.in_round[m] = true;
            rounds.decided[m] = false;
            self.queue.push(Event {
                time,
                robot: robot_id(m),
                kind: EventKind::Look,
            });
        }
        rounds.pending = members.len();
        rounds.members = members;
    }

    /// `Scheduler._check_global_termination`: every robot that is neither
    /// crashed nor Byzantine has terminated (vacuously true if none are left).
    fn globally_terminated(&self) -> bool {
        let r = &self.robots;
        (0..r.len())
            .filter(|&i| r.state[i] != RobotState::Crash && r.fault[i] != FaultKind::Byzantine)
            .all(|i| r.terminated[i])
    }
}

fn robot_id(i: usize) -> i32 {
    i32::try_from(i).unwrap_or(i32::MAX)
}

fn robot_id_u32(i: usize) -> u32 {
    u32::try_from(i).unwrap_or(u32::MAX)
}

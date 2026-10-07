//! The event loop: Python `Scheduler.handle_event` and `Robot.look/move/wait`
//! (docs/core-schema.md §5), plus a sequential scheduler the original lacks.

use crate::algorithm::{Algorithm, Assignment, Look, Model, OverlayUpdate, Plan, Scratch, View};
use crate::config::{
    Activation, ActivationOrder, ConfigError, FaultSelection, ScheduleEnd, SchedulerKind,
    SimConfig, StopPolicy, TurnGap,
};
use crate::event::{Event, EventKind, EventQueue};
use crate::geom::{dist, interpolate, Point};
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
    /// Without faults, a silent epoch means nothing will ever change.
    fault_free: bool,
    stalled: bool,
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
    /// `None` for the async scheduler.
    turns: Option<Turns>,
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
        let live = (0..n)
            .filter(|&i| robots.state[i] != RobotState::Crash)
            .count();
        let mut queue = EventQueue::default();
        for i in (0..n).filter(|_| !sequential) {
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
            fault_free: config.num_of_faults == 0,
            stalled: false,
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
        };
        if sequential {
            sim.next_turn(0.0);
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

    /// Sequential scheduler: turns taken so far (one per Look). 0 for async.
    #[must_use]
    pub fn turn_count(&self) -> u64 {
        self.turns.as_ref().map_or(0, |t| t.taken)
    }

    /// Sequential scheduler: epochs in which every live robot has had its
    /// turn. Always 0 for async.
    #[must_use]
    pub fn epochs_completed(&self) -> u64 {
        self.turns.as_ref().map_or(0, |t| t.epochs_completed)
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

    /// Handles events until the budget, a limit or the end is reached.
    pub fn advance(&mut self, budget: Budget) -> StopReason {
        let mut handled = 0;
        loop {
            if self.ended {
                return StopReason::Ended;
            }
            if self.stalled {
                return StopReason::Stalled;
            }
            if self.max_time.is_some_and(|limit| self.now > limit) {
                return StopReason::MaxTime;
            }
            if self.max_events.is_some_and(|limit| self.events >= limit) {
                return StopReason::MaxEvents;
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
        if self.ended || self.stalled {
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
        if self.robots.terminated[i] {
            return info(StepOutcome::Ignored);
        }

        let outcome = match event.kind {
            EventKind::Look => {
                if let Some(turns) = self.turns.as_mut() {
                    turns.taken += 1;
                }
                self.look(i, t)
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
        }
        if self.globally_terminated() {
            self.ended = true;
            return info(StepOutcome::Ended);
        }
        info(outcome)
    }

    /// The snapshot robot `i` takes at `t`, filtered by its visibility.
    fn build_view(&mut self, i: usize, t: f64) -> usize {
        let me = self.robots.pos[i];
        let visibility = self.model.visibility;
        let unlimited = visibility == f64::INFINITY;
        self.view.clear();
        let mut me_index = 0;
        for j in 0..self.robots.len() {
            let p = self.robots.position_at(j, t, self.model.eps);
            if unlimited || dist(me, p) <= visibility {
                if j == i {
                    me_index = self.view.len();
                }
                let r = &self.robots;
                self.view
                    .push(robot_id_u32(j), p, r.state[j], r.terminated[j], r.frozen[j]);
            }
        }
        me_index
    }

    /// `Robot.look` followed by the LOOK branch of `handle_event`.
    fn look(&mut self, i: usize, t: f64) -> StepOutcome {
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
            if active <= 1 {
                r.frozen[i] = true;
                r.terminated[i] = true;
                self.wait(i, t);
            } else {
                let look = Look {
                    me: robot_id_u32(i),
                    me_index,
                    position,
                    view: &self.view,
                    model: &self.model,
                };
                let algorithm = &self.algorithms[usize::from(self.robots.algo[i])];
                let decision = algorithm.compute(&look, &mut self.rng, &mut self.scratch);
                let r = &mut self.robots;
                r.target[i] = Some(decision.target);
                match decision.overlay {
                    OverlayUpdate::Keep => {}
                    OverlayUpdate::Set(c) => r.overlay[i] = Some(c),
                    OverlayUpdate::Clear => r.overlay[i] = None,
                }
                if decision.terminate {
                    r.terminated[i] = true;
                }
                if r.terminated[i] {
                    self.wait(i, t);
                } else {
                    let omit = r.fault[i] == FaultKind::Omission && self.rng.random() < 0.5;
                    if omit || dist(decision.target, position) < self.model.eps {
                        self.robots.frozen[i] = true;
                        self.wait(i, t);
                    } else {
                        self.robots.frozen[i] = false;
                    }
                }
            }
        }

        let r = &self.robots;
        if r.state[i] == RobotState::Crash {
            return StepOutcome::Crashed;
        }
        if r.terminated[i] {
            if self.turns.is_some() {
                self.next_turn(t);
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
        let target = self.limited_target(i, destination);
        let r = &mut self.robots;
        r.target[i] = Some(target);
        r.state[i] = RobotState::Move;
        r.set_light(i, Light::Red, t);
        r.start_time[i] = Some(t);
        r.start_pos[i] = r.pos[i];
        let distance = dist(r.start_pos[i], target);
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
        let snap_to_target = match (moving, r.start_time[i], r.target[i]) {
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
                    if !turns.progress && self.fault_free {
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
                    if !turns.progress && self.fault_free {
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

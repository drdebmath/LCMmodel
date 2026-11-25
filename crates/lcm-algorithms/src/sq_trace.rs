//! A turn-by-turn trace of a [`SqGathering`](crate::SqGathering) run on the
//! core's sequential scheduler, for the CLI (`lcm seq`) and the step-by-step
//! page: who was active, what it observed under weak multiplicity detection,
//! which branch of Algorithm 8 fired, where it was heading and where the
//! adversary stopped it.
//!
//! The run itself is an ordinary `lcm_core::Simulation`; the tracer only
//! watches it. Before each Look every robot is still (one turn at a time), so
//! the observation recomputed here is exactly the snapshot the robot used,
//! and the destination it reports is the target the core recorded.

use crate::sq_gathering::{decide, observe_with, occupancy, Observation};
use lcm_core::geom::dist;
use lcm_core::{
    Budget, ConfigError, EventKind, Point, RobotState, SimConfig, Simulation, StopReason,
};
use serde::{Deserialize, Serialize};

/// How a traced run stands.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Status {
    Running,
    /// Every robot saw the single gathering point and terminated.
    Gathered,
    /// A turn, event or time limit ended the run first. Says nothing about
    /// the algorithm.
    Incomplete,
    /// A whole epoch in which no robot moved or terminated (the core's
    /// `Stalled`). For SqGathering this happens when robots leaving one of
    /// several multiplicities get closer than ε to it: each leaver halves the
    /// distance of the one before, so large multiplicities run out of
    /// precision. In exact arithmetic the run would go on.
    Stalled,
}

/// One point of the active robot's observation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SeenPoint {
    pub at: [f64; 2],
    /// What the robot sees (weak detection): `true` for 2 or more robots.
    pub multiplicity: bool,
    /// The real number of robots there. The robot cannot see this.
    pub count: usize,
}

/// One turn: one robot's whole Look-Compute-Move.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Round {
    /// Turns taken so far, this one included (the core's `turn_count`).
    pub turn: u64,
    /// The epoch this turn belongs to, from 1.
    pub epoch: u64,
    pub robot: usize,
    pub position: [f64; 2],
    /// Q′(t), own point at `self_index`.
    pub observation: Vec<SeenPoint>,
    pub self_index: usize,
    pub kappa: usize,
    pub on_multiplicity: bool,
    pub rule: String,
    pub lines: String,
    pub description: String,
    pub destination: [f64; 2],
    /// Where the robot ends the turn: the destination, or where the
    /// adversary stopped it (never before δ).
    pub stop: [f64; 2],
    pub reached: bool,
    /// The robot saw a single occupied point and terminated.
    pub terminated: bool,
    pub distinct_after: usize,
    pub multiplicities_after: usize,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Report {
    pub algorithm: String,
    pub status: Status,
    /// The core's stop reason (`Ended`, `Stalled`, `MaxTurns`, …).
    pub stopped: String,
    pub turns: u64,
    pub epochs_completed: u64,
    /// The turn after which all robots stood on one point.
    pub gathered_after: Option<u64>,
    pub gathered_at: Option<[f64; 2]>,
    pub distinct: usize,
    pub multiplicities: usize,
    pub positions: Vec<[f64; 2]>,
    pub note: String,
}

const fn xy(p: Point) -> [f64; 2] {
    [p.x, p.y]
}

/// Watches a SqGathering simulation turn by turn.
pub struct Tracer {
    sim: Simulation,
    eps: f64,
    stopped: Option<StopReason>,
    gathered_after: Option<u64>,
}

impl Tracer {
    /// # Errors
    /// Another algorithm than SqGathering, or a configuration the core or
    /// SqGathering refuses (SqGathering needs `scheduler: "sequential"`).
    pub fn new(config: &SimConfig) -> Result<Self, ConfigError> {
        if config.algorithm != "SqGathering" {
            return Err(ConfigError::Unsupported(format!(
                "the SqGathering trace needs algorithm \"SqGathering\", got {:?}",
                config.algorithm
            )));
        }
        if config.scheduler != lcm_core::SchedulerKind::Sequential {
            return Err(ConfigError::Unsupported(
                "the SqGathering trace follows one robot per turn: it needs scheduler \"sequential\""
                    .to_owned(),
            ));
        }
        let sim = crate::simulation(config)?;
        let eps = sim.model().eps;
        let mut tracer = Self {
            sim,
            eps,
            stopped: None,
            gathered_after: None,
        };
        if tracer.census().0 == 1 {
            tracer.gathered_after = Some(0);
        }
        Ok(tracer)
    }

    #[must_use]
    pub fn simulation(&self) -> &Simulation {
        &self.sim
    }

    /// Distinct points and multiplicity points now.
    #[must_use]
    pub fn census(&self) -> (usize, usize) {
        let (_, counts, _) = occupancy(&self.sim.robots().pos, self.eps);
        (counts.len(), counts.iter().filter(|c| **c >= 2).count())
    }

    #[must_use]
    pub fn status(&self) -> Status {
        match self.stopped {
            None => Status::Running,
            Some(StopReason::Ended) => Status::Gathered,
            Some(StopReason::Stalled) => Status::Stalled,
            Some(_) => Status::Incomplete,
        }
    }

    /// Handles events until the next turn has been taken and returns it;
    /// `None` once the run has stopped.
    pub fn next_round(&mut self) -> Option<Round> {
        loop {
            if self.stopped.is_some() {
                return None;
            }
            // A zero budget handles nothing: it only reports a limit or the end.
            let reason = self.sim.advance(Budget {
                max_events: 0,
                until_time: f64::INFINITY,
            });
            if reason != StopReason::Budget {
                self.stopped = Some(reason);
                return None;
            }
            let before = self.sim.robots().pos.clone();
            let epoch = self.sim.epochs_completed() + 1;
            let info = self.sim.step();
            if info.kind != Some(EventKind::Look) || info.robot < 0 {
                continue;
            }
            let i = usize::try_from(info.robot).unwrap_or(usize::MAX);
            return Some(self.record(&before, i, epoch));
        }
    }

    /// Q′(t) of robot `i`: the robots it can see, as its multiplicity
    /// detection lets it perceive them (in global coordinates).
    fn observation(&self, before: &[Point], i: usize) -> Observation {
        let model = self.sim.model();
        let me = before[i];
        let mut seen = Vec::with_capacity(before.len());
        let mut me_index = 0;
        for (j, &p) in before.iter().enumerate() {
            if dist(me, p) <= model.visibility {
                if j == i {
                    me_index = seen.len();
                }
                seen.push(p);
            }
        }
        observe_with(&seen, me_index, self.eps, model.multiplicity)
    }

    fn record(&mut self, before: &[Point], i: usize, epoch: u64) -> Round {
        let eps = self.eps;
        let (_, counts, _) = occupancy(before, eps);
        let obs = self.observation(before, i);
        let decision = decide(&obs);
        let destination = decision.destination(&obs);
        let r = self.sim.robots();
        let end = if r.state[i] == RobotState::Move {
            r.goal(i).unwrap_or(destination)
        } else {
            r.pos[i]
        };
        let mut after = before.to_vec();
        after[i] = end;
        let (_, counts_after, _) = occupancy(&after, eps);
        let turn = self.sim.turn_count();
        if counts_after.len() == 1 && self.gathered_after.is_none() {
            self.gathered_after = Some(turn);
        }
        Round {
            turn,
            epoch,
            robot: i,
            position: xy(before[i]),
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
            reached: dist(end, destination) < eps,
            terminated: r.terminated[i],
            distinct_after: counts_after.len(),
            multiplicities_after: counts_after.iter().filter(|c| **c >= 2).count(),
        }
    }

    /// Plays every remaining turn, calling `each` per turn.
    pub fn run_with(&mut self, mut each: impl FnMut(&Round)) -> Report {
        while let Some(round) = self.next_round() {
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
        let status = self.status();
        let turns = self.sim.turn_count();
        let stopped = self
            .stopped
            .map_or_else(|| "Running".to_owned(), |r| format!("{r:?}"));
        let note = match status {
            Status::Gathered => format!(
                "gathered after {} turns; the run ended after {turns} turns, once every robot \
                 had seen the single point and terminated",
                self.gathered_after.unwrap_or(turns)
            ),
            Status::Running => format!("still running after {turns} turns"),
            Status::Incomplete => format!(
                "INCOMPLETE (budget-limited): stopped by {stopped} after {turns} turns, before \
                 gathering; {distinct} distinct points and {multiplicities} multiplicities \
                 remain. This is a budget limit, not a result of the algorithm."
            ),
            Status::Stalled => format!(
                "STALLED (precision limit) after {turns} turns: a whole epoch passed in which no \
                 robot could move more than ε. Robots leaving one of several multiplicities \
                 halve their distance each time, so large multiplicities run out of precision. \
                 Not a result of the algorithm (exact arithmetic would continue); use smaller \
                 multiplicities or a higher precision."
            ),
        };
        let positions: Vec<[f64; 2]> = self.sim.robots().pos.iter().copied().map(xy).collect();
        Report {
            algorithm: "SqGathering".to_owned(),
            status,
            stopped,
            turns,
            epochs_completed: self.sim.epochs_completed(),
            gathered_after: self.gathered_after,
            gathered_at: (status == Status::Gathered).then(|| positions[0]),
            distinct,
            multiplicities,
            positions,
            note,
        }
    }
}

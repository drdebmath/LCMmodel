//! Algorithm 8, `SqGathering` (Section 6 of "Universal pattern formation by
//! oblivious robots under sequential schedulers", arXiv:2412.10733v3).
//!
//! Robots are oblivious, disoriented and have **weak multiplicity
//! detection**: a robot sees the set Q′(t) of occupied points, each with one
//! bit saying whether it holds more than one robot, but never how many.
//! With κ the number of multiplicity points in Q′(t), the active robot at q:
//!
//! 1. κ = 1: stays if it is on the multiplicity, else moves to it;
//! 2. κ > 1: if on a multiplicity, moves to the midpoint of q and the closest
//!    occupied point q′ (a point no other robot occupies), else stays;
//! 3. κ = 0: moves to the closest point occupied by another robot.
//!
//! [`SqGathering`] is the rule as an [`Algorithm`] of the core: it runs under
//! the core's sequential scheduler (`scheduler: "sequential"`), which gives
//! one robot a whole Look-Compute-Move per turn and stops non-rigid moves
//! after δ. Its registry entry ([`plan`]) refuses the asynchronous scheduler,
//! limited visibility and faults, which the paper's model does not have. It
//! is *not* the generic centre-of-gravity `Gathering`.
//!
//! Weak multiplicity detection is part of the robots' model here: the robot
//! turns its snapshot into Q′(t) itself ([`observe`]), keeping one bit per
//! occupied point. Points closer than ε (`10^-threshold_precision`) are one.

use lcm_core::geom::dist;
use lcm_core::occupancy::perceive;
pub use lcm_core::LocalFrame;
use lcm_core::{
    Algorithm, ConfigError, Decision as CoreDecision, Look, MultiplicityModel, OverlayUpdate, Plan,
    Point, Rng, SchedulerKind, Scratch, SimConfig,
};

/// One element of Q′(t): an occupied point and its multiplicity bit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Observed {
    pub at: Point,
    /// `b = 1`: two or more robots are here. The count is not visible.
    pub multiplicity: bool,
}

/// The snapshot one robot takes: Q′(t), and which of its points is its own.
#[derive(Clone, Debug, PartialEq)]
pub struct Observation {
    pub points: Vec<Observed>,
    /// Index in `points` of the observing robot's own location.
    pub me: usize,
}

/// Points closer than this are one location when no ε is given (tests).
pub const DEFAULT_COINCIDENCE: f64 = 1e-9;

pub use lcm_core::occupancy::occupancy;

/// What robot `i` observes under weak multiplicity detection, in global
/// coordinates. Two robots sharing a point and five sharing a point give the
/// same observation.
#[must_use]
pub fn observe(positions: &[Point], i: usize, tolerance: f64) -> Observation {
    observe_with(positions, i, tolerance, MultiplicityModel::Weak)
}

/// What robot `i` observes under `model`. Strong detection gives the same
/// bits as weak (the rule never needs the count); without detection every
/// bit is 0, so the robot always takes the κ = 0 branch (Theorem 4).
#[must_use]
pub fn observe_with(
    positions: &[Point],
    i: usize,
    tolerance: f64,
    model: MultiplicityModel,
) -> Observation {
    let (points, me) = perceive(positions, i, tolerance, model);
    Observation {
        points: points
            .iter()
            .map(|o| Observed {
                at: o.at,
                multiplicity: o.multiple,
            })
            .collect(),
        me,
    }
}

/// Which branch of Algorithm 8 fired.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Rule {
    /// κ = 0 and no other robot exists (n = 1). Not in the paper: nothing to do.
    Alone,
    /// κ = 0: move to the closest occupied point (lines 14–16).
    MoveToClosest,
    /// κ = 1, b = 0: move to the multiplicity point (lines 4–7).
    JoinMultiplicity,
    /// κ = 1, b = 1: stay (lines 3–5).
    HoldMultiplicity,
    /// κ > 1, b = 1: move to the midpoint towards the closest point (lines 9–13).
    LeaveMultiplicity,
    /// κ > 1, b = 0: stay (lines 3, 9–10).
    WaitForSeparation,
}

impl Rule {
    pub const ALL: [Self; 6] = [
        Self::MoveToClosest,
        Self::JoinMultiplicity,
        Self::HoldMultiplicity,
        Self::LeaveMultiplicity,
        Self::WaitForSeparation,
        Self::Alone,
    ];

    /// Stable key used in traces, the CLI and the web page.
    #[must_use]
    pub const fn key(self) -> &'static str {
        match self {
            Self::Alone => "alone",
            Self::MoveToClosest => "k0-move-to-closest",
            Self::JoinMultiplicity => "k1-join-multiplicity",
            Self::HoldMultiplicity => "k1-hold-multiplicity",
            Self::LeaveMultiplicity => "kmany-leave-multiplicity",
            Self::WaitForSeparation => "kmany-wait",
        }
    }

    /// Lines of Algorithm 8 that decide this branch.
    #[must_use]
    pub const fn lines(self) -> &'static str {
        match self {
            Self::Alone => "3",
            Self::MoveToClosest => "14-16",
            Self::JoinMultiplicity => "4-7",
            Self::HoldMultiplicity => "3-5",
            Self::LeaveMultiplicity => "9-13",
            Self::WaitForSeparation => "3, 9-10",
        }
    }

    #[must_use]
    pub const fn description(self) -> &'static str {
        match self {
            Self::Alone => "No other robot exists: stay",
            Self::MoveToClosest => "κ = 0: move to the closest point occupied by another robot",
            Self::JoinMultiplicity => "κ = 1, b = 0: move to the multiplicity point",
            Self::HoldMultiplicity => "κ = 1, b = 1: on the multiplicity, do not move",
            Self::LeaveMultiplicity => {
                "κ > 1, b = 1: move to the midpoint of q and the closest point q′"
            }
            Self::WaitForSeparation => "κ > 1, b = 0: not on a multiplicity, do not move",
        }
    }
}

/// Where the robot goes, by reference to its observation, so the global
/// destination is exact whatever local frame the decision was taken in.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Action {
    Stay,
    /// Move onto observed point `j`.
    MoveTo(usize),
    /// Move to the midpoint of the own point and observed point `j`.
    Midpoint(usize),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Decision {
    /// Number of multiplicity points in Q′(t).
    pub kappa: usize,
    /// `b` of the robot's own point.
    pub on_multiplicity: bool,
    pub rule: Rule,
    pub action: Action,
}

impl Decision {
    /// The destination in the coordinates of `obs`.
    #[must_use]
    pub fn destination(&self, obs: &Observation) -> Point {
        let q = obs.points[obs.me].at;
        match self.action {
            Action::Stay => q,
            Action::MoveTo(j) => obs.points[j].at,
            Action::Midpoint(j) => {
                let r = obs.points[j].at;
                Point::new(0.5 * (q.x + r.x), 0.5 * (q.y + r.y))
            }
        }
    }
}

/// The occupied point closest to the robot's own. Equally close points are
/// told apart by the robot's own coordinates (x, then y): any deterministic
/// choice is allowed, and a disoriented robot's choice depends on its frame.
fn closest(obs: &Observation) -> Option<usize> {
    let q = obs.points[obs.me].at;
    let mut best: Option<(usize, f64)> = None;
    for (j, o) in obs.points.iter().enumerate() {
        if j == obs.me {
            continue;
        }
        let d = dist(q, o.at);
        best = match best {
            None => Some((j, d)),
            Some((k, bd)) => {
                let tie = (d - bd).abs() <= 1e-12 * bd.max(1e-300);
                let at = o.at;
                let other = obs.points[k].at;
                if (!tie && d < bd) || (tie && (at.x, at.y) < (other.x, other.y)) {
                    Some((j, d))
                } else {
                    Some((k, bd))
                }
            }
        };
    }
    best.map(|(j, _)| j)
}

/// Algorithm 8 on one observation.
#[must_use]
pub fn decide(obs: &Observation) -> Decision {
    let kappa = obs.points.iter().filter(|o| o.multiplicity).count();
    let b = obs.points[obs.me].multiplicity;
    let (rule, action) = match kappa {
        1 if b => (Rule::HoldMultiplicity, Action::Stay),
        1 => {
            let m = obs
                .points
                .iter()
                .position(|o| o.multiplicity)
                .unwrap_or(obs.me);
            (Rule::JoinMultiplicity, Action::MoveTo(m))
        }
        0 => match closest(obs) {
            Some(j) => (Rule::MoveToClosest, Action::MoveTo(j)),
            None => (Rule::Alone, Action::Stay),
        },
        _ if b => match closest(obs) {
            Some(j) => (Rule::LeaveMultiplicity, Action::Midpoint(j)),
            // κ > 1 means at least two points, so there is always one.
            None => (Rule::Alone, Action::Stay),
        },
        _ => (Rule::WaitForSeparation, Action::Stay),
    };
    Decision {
        kappa,
        on_multiplicity: b,
        rule,
        action,
    }
}

/// The observation as a robot with this frame perceives it (same point order).
#[must_use]
pub fn localize(frame: &LocalFrame, obs: &Observation) -> Observation {
    Observation {
        points: obs
            .points
            .iter()
            .map(|o| Observed {
                at: frame.to_local(o.at),
                multiplicity: o.multiplicity,
            })
            .collect(),
        me: obs.me,
    }
}

/// Algorithm 8 computed in the robot's own frame. The action refers to
/// observed points, so it applies unchanged to the global observation.
#[must_use]
pub fn decide_in_frame(obs: &Observation, frame: &LocalFrame) -> Decision {
    decide(&localize(frame, obs))
}

/// Algorithm 8 as an algorithm of the core. Stateless: robots are oblivious.
#[derive(Clone, Copy, Debug, Default)]
pub struct SqGathering;

impl Algorithm for SqGathering {
    fn key(&self) -> &'static str {
        "SqGathering"
    }

    /// One robot's Compute: Q′(t) from its snapshot, then the rule. The
    /// robot terminates when it sees a single occupied point: everyone is
    /// gathered there, and every robot activated from now on would stay.
    fn compute(&self, look: &Look<'_>, _: &mut Rng, _: &mut Scratch) -> CoreDecision {
        let obs = observe_with(
            &look.view.pos,
            look.me_index,
            look.model.eps,
            look.model.multiplicity,
        );
        let decision = decide(&obs);
        CoreDecision {
            target: decision.destination(&obs),
            terminate: obs.points.len() == 1,
            overlay: OverlayUpdate::Keep,
        }
    }

    fn explain(&self, look: &Look<'_>, _: &mut Scratch) -> Option<String> {
        let obs = observe_with(
            &look.view.pos,
            look.me_index,
            look.model.eps,
            look.model.multiplicity,
        );
        let d = decide(&obs);
        let points = obs.points.len();
        let mut text = format!(
            "It sees {points} occupied point{}, {} of them stack{} (κ = {}); it stands {} (b = {}).",
            if points == 1 { "" } else { "s" },
            d.kappa,
            if d.kappa == 1 { "" } else { "s" },
            d.kappa,
            if d.on_multiplicity { "on a stack" } else { "alone" },
            u8::from(d.on_multiplicity),
        );
        // The rule's description without its "κ = 1, b = 0:" condition, already said above.
        let rule = d.rule.description();
        let rule = rule.split_once(": ").map_or(rule, |(_, what)| what);
        text.push_str(&format!(" So it does this: {rule}."));
        let to = dist(obs.points[obs.me].at, d.destination(&obs));
        if to > 0.0 {
            text.push_str(&format!(" It moves {} units.", crate::num(to)));
        }
        if points == 1 {
            text.push_str(" Only one occupied point: everyone is gathered, so it terminates.");
        }
        Some(text)
    }
}

/// The registry entry's plan builder. Theorem 5 holds under any sequential
/// scheduler with weak multiplicity detection; the asynchronous scheduler and
/// faulty robots are refused. SSYNC/FSYNC, no multiplicity detection and
/// limited visibility are allowed to show what the paper says about them
/// (see [`notes`]).
///
/// # Errors
/// The asynchronous scheduler, or faulty robots.
pub fn plan(config: &SimConfig) -> Result<Plan, ConfigError> {
    let refuse = |message: &str| Err(ConfigError::Unsupported(message.to_owned()));
    if matches!(
        config.scheduler,
        SchedulerKind::Async | SchedulerKind::KAsync
    ) {
        return refuse(
            "SqGathering does not run under this scheduler. Pick the sequential scheduler (Settings → Timing → Scheduler → Sequential), where one robot moves at a time and every robot sees an exact, up-to-date picture. When robots wake up on their own clocks, a robot can act on a picture that is already out of date, and with more than one robot active at once gathering can no longer be guaranteed, even for robots that can see stacks. SSYNC and FSYNC can be chosen as well, to watch what goes wrong when robots act in rounds.",
        );
    }
    if config.num_of_faults > 0 {
        return refuse("SqGathering is designed for robots that never fail: a faulty robot that crashes or misbehaves can keep the others from ever agreeing on one spot. Set Faulty robots to 0 (Settings → Model → Faults).");
    }
    Ok(Plan::single(Box::new(SqGathering)))
}

/// Where a run's settings leave the model of Theorem 5, and what the paper
/// says happens there. Empty inside the model.
#[must_use]
pub fn notes(config: &SimConfig) -> Vec<String> {
    let mut notes = Vec::new();
    match config.scheduler {
        SchedulerKind::Ssync => notes.push(
            "Outside the model: SqGathering is designed for one robot at a time, and here robots act in rounds, several at once. When every robot on a stack moves at the same moment, the stack moves as a whole instead of breaking up, so two stacks can keep stepping towards each other without ever merging. No rule can rule this out for every possible choice of who acts in each round, so expect some runs that never gather."
                .to_owned(),
        ),
        SchedulerKind::Fsync => notes.push(
            "Outside the model: SqGathering is designed for one robot at a time, and here every robot moves in every round. Robots that see the same picture make the same move together, so the outcome depends on the start: some swarms gather, others fall into a pattern that repeats or stops. Rules made for all-at-once timing, such as walking to the centre of the smallest circle around the swarm, handle this case better."
                .to_owned(),
        ),
        _ => {}
    }
    if config.multiplicity == MultiplicityModel::Local {
        notes.push(
            "Outside the model: with local detection a robot knows whether its own spot is a stack, but it cannot recognise stacks anywhere else. A robot on a stack therefore always stays where it is, and a robot on its own walks to its nearest neighbour, so once two stacks exist they wait for each other forever and never merge. Switch Multiplicity detection to Weak to give the robots what SqGathering expects."
                .to_owned(),
        );
    }
    if config.multiplicity == MultiplicityModel::None {
        notes.push(
            "Outside the model: these robots cannot see stacks at all, so every occupied spot looks like a single robot. Each robot simply walks to its nearest neighbour, and once only two occupied spots are left the scheduler can keep sending robots back and forth between them forever. Gathering cannot be guaranteed this way, even one robot at a time; switch Multiplicity detection to Weak to see the algorithm as it is meant to run."
                .to_owned(),
        );
    }
    if config.visibility_radius.is_some() {
        notes.push(
            "Experimental: SqGathering is designed for robots that see the whole swarm. Here each robot applies the same rule to the robots within its sight radius, which is a natural way to extend it, but nobody knows whether it always gathers this way: it may work, or it may split the swarm into groups that never find each other again. Watch the run to see which happens."
                .to_owned(),
        );
    }
    if config.obstructed_visibility {
        notes.push(
            "Experimental: SqGathering is designed for robots that never hide one another. When robots block the view, a robot can miss a stack or a neighbour standing behind another robot and decide on an incomplete picture. Whether the rule still always gathers is not known, so treat this run as an experiment."
                .to_owned(),
        );
    }
    notes
}

#[cfg(test)]
mod tests {
    use super::*;

    fn obs(points: &[(f64, f64, bool)], me: usize) -> Observation {
        Observation {
            points: points
                .iter()
                .map(|&(x, y, m)| Observed {
                    at: Point::new(x, y),
                    multiplicity: m,
                })
                .collect(),
            me,
        }
    }

    #[test]
    fn kappa_zero_moves_to_the_closest_point() {
        let o = obs(
            &[(0.0, 0.0, false), (5.0, 0.0, false), (0.0, 3.0, false)],
            0,
        );
        let d = decide(&o);
        assert_eq!((d.kappa, d.rule), (0, Rule::MoveToClosest));
        assert_eq!(d.destination(&o), Point::new(0.0, 3.0));
    }

    #[test]
    fn kappa_one_singleton_joins_and_member_holds() {
        let o = obs(&[(1.0, 1.0, true), (4.0, 5.0, false), (9.0, 9.0, false)], 2);
        let d = decide(&o);
        assert_eq!(
            (d.kappa, d.on_multiplicity, d.rule),
            (1, false, Rule::JoinMultiplicity)
        );
        assert_eq!(d.destination(&o), Point::new(1.0, 1.0));
        let o = Observation { me: 0, ..o };
        let d = decide(&o);
        assert_eq!((d.rule, d.action), (Rule::HoldMultiplicity, Action::Stay));
    }

    #[test]
    fn kappa_many_member_leaves_to_the_midpoint_singleton_waits() {
        let o = obs(&[(0.0, 0.0, true), (6.0, 0.0, true), (0.0, 8.0, false)], 0);
        let d = decide(&o);
        assert_eq!((d.kappa, d.rule), (2, Rule::LeaveMultiplicity));
        assert_eq!(d.destination(&o), Point::new(3.0, 0.0));
        let o = Observation { me: 2, ..o };
        assert_eq!(decide(&o).rule, Rule::WaitForSeparation);
        assert_eq!(decide(&o).destination(&o), Point::new(0.0, 8.0));
    }

    #[test]
    fn a_lone_robot_and_a_gathered_swarm_stay() {
        let lone = obs(&[(2.0, 2.0, false)], 0);
        assert_eq!(decide(&lone).rule, Rule::Alone);
        let gathered = obs(&[(2.0, 2.0, true)], 0);
        assert_eq!(decide(&gathered).rule, Rule::HoldMultiplicity);
    }

    #[test]
    fn weak_detection_hides_the_count() {
        let p = |x, y| Point::new(x, y);
        let two = [p(0.0, 0.0), p(0.0, 0.0), p(3.0, 0.0)];
        let five = [p(0.0, 0.0); 4]
            .into_iter()
            .chain([p(3.0, 0.0)])
            .collect::<Vec<_>>();
        assert_eq!(
            observe(&two, 2, DEFAULT_COINCIDENCE),
            observe(&five, 4, DEFAULT_COINCIDENCE)
        );
    }

    #[test]
    fn ties_are_broken_in_the_robots_own_frame() {
        // (0,0) sees (1,0) and (-1,0) at the same distance.
        let o = obs(
            &[(0.0, 0.0, false), (1.0, 0.0, false), (-1.0, 0.0, false)],
            0,
        );
        assert_eq!(decide(&o).action, Action::MoveTo(2));
        let mirrored = LocalFrame {
            angle: core::f64::consts::PI,
            ..LocalFrame::global(Point::new(0.0, 0.0))
        };
        assert_eq!(decide_in_frame(&o, &mirrored).action, Action::MoveTo(1));
    }

    #[test]
    fn frames_round_trip() {
        let f = LocalFrame {
            origin: Point::new(3.0, -2.0),
            angle: 1.234,
            scale: 0.37,
            reflect: true,
        };
        let p = Point::new(-7.5, 11.25);
        let back = f.to_global(f.to_local(p));
        assert!(dist(p, back) < 1e-12);
        assert!(dist(f.to_local(f.origin), Point::new(0.0, 0.0)) < 1e-15);
    }
}

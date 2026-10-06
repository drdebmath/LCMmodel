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
//! This module is the pure decision rule. It is *not* the generic
//! centre-of-gravity `Gathering` of the async simulator; the sequential
//! scheduler that drives it lives in [`crate::seq`].

use lcm_core::geom::dist;
use lcm_core::Point;

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

/// Points closer than this are one location (Q groups co-located robots).
pub const DEFAULT_COINCIDENCE: f64 = 1e-9;

/// Distinct locations of `positions` (first-seen order) and how many robots
/// stand on each. Robot `i` stands on `groups[owner[i]]`.
#[must_use]
#[allow(clippy::cast_possible_truncation)]
pub fn occupancy(positions: &[Point], tolerance: f64) -> (Vec<Point>, Vec<usize>, Vec<usize>) {
    // Groups are bucketed in cells of side `tolerance`, so a point only has to
    // be compared with the groups in its own and the 8 neighbouring cells.
    // Taking the lowest matching index keeps the first-seen grouping of a
    // plain linear scan, in O(n) instead of O(n²).
    let cell = |v: f64| libm::floor(v / tolerance) as i64;
    let mut buckets: std::collections::HashMap<(i64, i64), Vec<usize>> =
        std::collections::HashMap::new();
    let mut groups: Vec<Point> = Vec::new();
    let mut counts: Vec<usize> = Vec::new();
    let mut owner = Vec::with_capacity(positions.len());
    for &p in positions {
        let (cx, cy) = (cell(p.x), cell(p.y));
        let mut found: Option<usize> = None;
        for dx in -1..=1_i64 {
            for dy in -1..=1_i64 {
                let key = (cx.saturating_add(dx), cy.saturating_add(dy));
                for &g in buckets.get(&key).map_or(&[][..], Vec::as_slice) {
                    if dist(p, groups[g]) <= tolerance && found.is_none_or(|f| g < f) {
                        found = Some(g);
                    }
                }
            }
        }
        if let Some(g) = found {
            counts[g] += 1;
            owner.push(g);
        } else {
            buckets.entry((cx, cy)).or_default().push(groups.len());
            owner.push(groups.len());
            groups.push(p);
            counts.push(1);
        }
    }
    (groups, counts, owner)
}

/// What robot `i` observes under weak multiplicity detection, in global
/// coordinates. Two robots sharing a point and five sharing a point give the
/// same observation.
#[must_use]
pub fn observe(positions: &[Point], i: usize, tolerance: f64) -> Observation {
    let (groups, counts, owner) = occupancy(positions, tolerance);
    Observation {
        points: groups
            .iter()
            .zip(&counts)
            .map(|(&at, &c)| Observed {
                at,
                multiplicity: c >= 2,
            })
            .collect(),
        me: owner[i],
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

/// A robot's private coordinate system: itself at the origin, its own
/// rotation, unit length and handedness (no shared chirality).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LocalFrame {
    pub origin: Point,
    pub angle: f64,
    pub scale: f64,
    pub reflect: bool,
}

impl LocalFrame {
    #[must_use]
    pub const fn global(origin: Point) -> Self {
        Self {
            origin,
            angle: 0.0,
            scale: 1.0,
            reflect: false,
        }
    }

    #[must_use]
    pub fn to_local(&self, p: Point) -> Point {
        let (s, c) = (libm::sin(self.angle), libm::cos(self.angle));
        let dx = p.x - self.origin.x;
        let dy = if self.reflect {
            self.origin.y - p.y
        } else {
            p.y - self.origin.y
        };
        Point::new(
            self.scale * (c * dx - s * dy),
            self.scale * (s * dx + c * dy),
        )
    }

    #[must_use]
    pub fn to_global(&self, p: Point) -> Point {
        let (s, c) = (libm::sin(self.angle), libm::cos(self.angle));
        let (x, y) = (p.x / self.scale, p.y / self.scale);
        let dx = c * x + s * y;
        let dy = -s * x + c * y;
        let dy = if self.reflect { -dy } else { dy };
        Point::new(self.origin.x + dx, self.origin.y + dy)
    }

    /// The observation as this robot perceives it (same point order).
    #[must_use]
    pub fn localize(&self, obs: &Observation) -> Observation {
        Observation {
            points: obs
                .points
                .iter()
                .map(|o| Observed {
                    at: self.to_local(o.at),
                    multiplicity: o.multiplicity,
                })
                .collect(),
            me: obs.me,
        }
    }
}

/// Algorithm 8 computed in the robot's own frame. The action refers to
/// observed points, so it applies unchanged to the global observation.
#[must_use]
pub fn decide_in_frame(obs: &Observation, frame: &LocalFrame) -> Decision {
    decide(&frame.localize(obs))
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

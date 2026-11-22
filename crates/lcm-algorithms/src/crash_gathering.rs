//! Gathering with weak multiplicity detection despite crash faults:
//! Pattanayak, Mondal, Ramesh H., Mandal, "Fault-tolerant gathering of mobile
//! robots with weak multiplicity detection" (ICDCN 2017, arXiv:1608.02432,
//! Algorithm 3 `Gather_k`; journal version JPDC 123, 2019, not checked).
//!
//! SSYNC, unlimited visibility, no common direction or chirality, weak
//! multiplicity detection, up to n - 1 robots crashed. From a legal start (at
//! most one multiplicity point) the robots that work gather (Theorem 4.13):
//!
//! - if a multiplicity point m exists the destination p* is m, else the
//!   centre O of the smallest enclosing circle of all robots;
//! - a robot with a robot-free path to p* moves straight there;
//! - otherwise it moves along a circular arc G through itself and p*, tangent
//!   at p* to the bisector of the smallest robot-free sector next to it, so
//!   that no two paths meet before p* and no second multiplicity appears.
//!
//! **Approximation.** The core moves robots in straight lines. A blocked robot
//! here moves to the point halfway along its arc G (a chord of G); from there
//! its path is usually free and it goes straight. Robots on one ray share the
//! tangent line, so their circles are nested and meet only at p*; the paper's
//! recursive tangent (`FindTangent` with the minimum step S) is not used.
//! Crashes happen at the start in this simulator (the paper allows any time).
//!
//! A robot on the multiplicity point stays there for good (it stays the only
//! multiplicity), so it terminates; the run ends once every working robot has.

use crate::sec::welzl_recur;
use core::f64::consts::PI;
use lcm_core::geom::dist;
use lcm_core::{
    Algorithm, Circle, ConfigError, Decision, Look, MultiplicityModel, OverlayUpdate, Plan, Point,
    Rng, SchedulerKind, Scratch, SimConfig,
};

#[derive(Clone, Copy, Debug, Default)]
pub struct CrashGathering;

/// The smallest enclosing circle of `points` (unique, so the order of the
/// randomized Welzl recursion does not matter; a fixed generator keeps the
/// run's own random draws untouched).
fn enclosing_circle(points: &[Point], precision: u8, scratch: &mut Scratch) -> Circle {
    scratch.points.clear();
    scratch.points.extend_from_slice(points);
    scratch.points_b.clear();
    scratch.floats.clear();
    let n = scratch.points.len();
    welzl_recur(
        &mut scratch.points,
        &mut scratch.points_b,
        n,
        &mut Rng::new(0),
        precision,
        &mut scratch.floats,
    )
}

/// Whether some other robot stands on the segment from `p` to `q`, strictly
/// before `q` (robots at `q` are the destination, not in the way).
fn blocked(p: Point, q: Point, others: &[Point], eps: f64) -> bool {
    let (dx, dy) = (q.x - p.x, q.y - p.y);
    let len2 = dx * dx + dy * dy;
    if len2 <= eps * eps {
        return false;
    }
    others.iter().any(|&r| {
        if dist(r, p) <= eps || dist(r, q) <= eps {
            return false;
        }
        let s = ((r.x - p.x) * dx + (r.y - p.y) * dy) / len2;
        s > 0.0 && s < 1.0 && dist(r, Point::new(p.x + s * dx, p.y + s * dy)) <= eps
    })
}

/// The halfway point of the arc a blocked robot at `p` takes to `c`: the
/// circle through `p` and `c` tangent at `c` to the bisector of the smallest
/// robot-free sector (around `c`) next to `p`'s ray.
#[must_use]
pub fn arc_step(p: Point, c: Point, others: &[Point], eps: f64) -> Point {
    let angle = |q: Point| libm::atan2(q.y - c.y, q.x - c.x).rem_euclid(2.0 * PI);
    let mine = angle(p);
    let (mut ccw, mut cw) = (2.0 * PI, 2.0 * PI);
    for &q in others {
        if dist(q, c) <= eps {
            continue;
        }
        let a = (angle(q) - mine).rem_euclid(2.0 * PI);
        if a <= 1e-12 || a >= 2.0 * PI - 1e-12 {
            continue; // on p's own ray
        }
        ccw = ccw.min(a);
        cw = cw.min(2.0 * PI - a);
    }
    // The smaller of the two sectors next to p's ray, and its bisector.
    let heading = if ccw <= cw {
        mine + ccw / 2.0
    } else {
        mine - cw / 2.0
    };
    let (tx, ty) = (libm::cos(heading), libm::sin(heading));
    // The circle's centre is on the normal to the tangent at c.
    let (nx, ny) = (-ty, tx);
    let (vx, vy) = (c.x - p.x, c.y - p.y);
    let along = nx * vx + ny * vy;
    if along.abs() <= 1e-15 {
        return Point::new((p.x + c.x) / 2.0, (p.y + c.y) / 2.0);
    }
    let t = -(vx * vx + vy * vy) / (2.0 * along);
    let o = Point::new(c.x + t * nx, c.y + t * ny);
    // Halfway along the shorter arc from p to c.
    let a0 = libm::atan2(p.y - o.y, p.x - o.x);
    let a1 = libm::atan2(c.y - o.y, c.x - o.x);
    let mut d = a1 - a0;
    while d > PI {
        d -= 2.0 * PI;
    }
    while d < -PI {
        d += 2.0 * PI;
    }
    let r = t.abs();
    let m = a0 + d / 2.0;
    Point::new(o.x + r * libm::cos(m), o.y + r * libm::sin(m))
}

impl CrashGathering {
    /// Adds to `text` that a robot stands in the way to `target`, if one does.
    fn blocked_note(
        self,
        me: Point,
        target: Point,
        occupied: &[lcm_core::occupancy::Occupied],
        mine: usize,
        eps: f64,
        text: &mut String,
    ) {
        let others: Vec<Point> = occupied
            .iter()
            .enumerate()
            .filter(|&(k, _)| k != mine)
            .map(|(_, o)| o.at)
            .collect();
        if blocked(me, target, &others, eps) {
            text.push_str(
                " A robot stands in its way, so it curves around it instead of going straight.",
            );
        }
    }
}

impl Algorithm for CrashGathering {
    fn key(&self) -> &'static str {
        "CrashGathering"
    }

    /// The last working robot must still walk to the robots that have
    /// gathered (and terminated) on the multiplicity point.
    fn stops_when_alone(&self) -> bool {
        false
    }

    fn compute(&self, look: &Look<'_>, _: &mut Rng, scratch: &mut Scratch) -> Decision {
        let eps = look.model.eps;
        let me = look.position;
        let (occupied, mine) = look.occupied();
        let multiplicities: Vec<Point> = occupied
            .iter()
            .filter(|o| o.multiple)
            .map(|o| o.at)
            .collect();
        let (target, overlay) = if let Some(&m) = multiplicities.first() {
            if occupied[mine].multiple {
                // On the multiplicity point: it stays the only one, so stay for good.
                return Decision {
                    target: me,
                    terminate: true,
                    overlay: OverlayUpdate::Clear,
                };
            }
            (m, OverlayUpdate::Clear)
        } else {
            let points: Vec<Point> = occupied.iter().map(|o| o.at).collect();
            let sec = enclosing_circle(&points, look.model.precision, scratch);
            (sec.center, OverlayUpdate::Set(sec))
        };
        if dist(me, target) <= eps {
            return Decision {
                target: me,
                terminate: false,
                overlay,
            };
        }
        let others: Vec<Point> = occupied
            .iter()
            .enumerate()
            .filter(|&(k, _)| k != mine)
            .map(|(_, o)| o.at)
            .collect();
        let goal = if blocked(me, target, &others, eps) {
            arc_step(me, target, &others, eps)
        } else {
            target
        };
        Decision {
            target: goal,
            terminate: false,
            overlay,
        }
    }

    fn explain(&self, look: &Look<'_>, scratch: &mut Scratch) -> Option<String> {
        let eps = look.model.eps;
        let me = look.position;
        let (occupied, mine) = look.occupied();
        let n = occupied.len();
        let seen = format!(
            "It sees {n} occupied point{}",
            if n == 1 { "" } else { "s" }
        );
        let multiplicity = occupied.iter().find(|o| o.multiple).map(|o| o.at);
        let target = match multiplicity {
            Some(_) if occupied[mine].multiple => {
                return Some(format!(
                    "{seen}, and stands on the multiplicity point, the meeting point: it stays there for good (terminates)."
                ))
            }
            Some(m) => {
                let mut text = format!("{seen}, one of them a multiplicity: it heads for it, {} units away.", crate::num(dist(me, m)));
                if dist(me, m) > eps {
                    self.blocked_note(me, m, &occupied, mine, eps, &mut text);
                }
                return Some(text);
            }
            None => {
                let points: Vec<Point> = occupied.iter().map(|o| o.at).collect();
                enclosing_circle(&points, look.model.precision, scratch)
            }
        };
        let to = dist(me, target.center);
        let mut text = format!(
            "{seen}, none of them a multiplicity: it heads for the centre of their smallest enclosing circle (radius {}), {} units away.",
            crate::num(target.radius),
            crate::num(to)
        );
        if to <= eps {
            text.push_str(" It is already there, so it stays.");
        } else {
            self.blocked_note(me, target.center, &occupied, mine, eps, &mut text);
        }
        Some(text)
    }
}

/// # Errors
/// The asynchronous schedulers (the paper's `Gather_k` is for SSYNC; its
/// ASYNC algorithm is different), or a model without weak detection.
pub fn plan(config: &SimConfig) -> Result<Plan, ConfigError> {
    if matches!(
        config.scheduler,
        SchedulerKind::Async | SchedulerKind::KAsync
    ) {
        return Err(ConfigError::Unsupported(
            "Gathering despite crashes works in rounds: pick Sequential, SSYNC or FSYNC as the scheduler (Settings → Timing → Scheduler). It needs every robot that acts to see the swarm at the same moment, which robots on their own clocks cannot guarantee.".to_owned(),
        ));
    }
    if matches!(
        config.multiplicity,
        MultiplicityModel::None | MultiplicityModel::Local
    ) {
        return Err(ConfigError::Unsupported(
            "Gathering despite crashes needs robots that can see stacks: set Multiplicity detection to Weak or Strong (Settings → Model → Robot model). The working robots find each other by heading for the stack, so they must be able to tell a stack from a single robot.".to_owned(),
        ));
    }
    Ok(Plan::single(Box::new(CrashGathering)))
}

#[must_use]
pub fn notes(config: &SimConfig) -> Vec<String> {
    let mut notes = vec!["Simplification: when another robot stands on a robot's straight path to its target, the exact rule makes it curve around that robot. Here the curve is replaced by one short sideways step, after which the robot goes straight on: it gets past in the same way, only along a less smooth path.".to_owned()];
    if config.num_of_faults > 0 && config.fault_type != lcm_core::FaultSelection::Crash {
        notes.push("Outside the model: this algorithm tolerates robots that stop working for good (crash faults): it simply gathers the working robots and walks around the crashed ones. Byzantine, omission and delay faults behave differently, and they can mislead the working robots or keep them waiting.".to_owned());
    }
    if config.multiplicities > 1 {
        notes.push("Outside the model: this run starts with two or more stacks. When robots act in rounds, the robots of a stack can all move together every time, so a stack may never split, and two such stacks can then never be merged by any rule: this start cannot always be gathered.".to_owned());
    }
    if config.visibility_radius.is_some() {
        notes.push(
            "Outside the model: this algorithm is designed for robots that see the whole swarm. With a limited sight radius, robots may head for different stacks or different centres and never meet.".to_owned(),
        );
    }
    notes
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_robot_behind_another_is_blocked_and_curves_off_the_ray() {
        let c = Point::new(0.0, 0.0);
        let front = Point::new(5.0, 0.0);
        let back = Point::new(10.0, 0.0);
        let others = [front, Point::new(0.0, 7.0), Point::new(-6.0, -6.0)];
        assert!(blocked(back, c, &others, 1e-9));
        let s = arc_step(back, c, &others, 1e-9);
        assert!(s.y.abs() > 1e-6, "leaves the ray: {s:?}");
        assert!(dist(s, c) < dist(back, c), "gets closer: {s:?}");
        assert!(!blocked(s, c, &others, 1e-9), "then the way is clear");
    }
}

//! Gathering with weak multiplicity detection despite crash faults:
//! Pattanayak, Mondal, Ramesh H., Mandal, "Fault-tolerant gathering of mobile
//! robots with weak multiplicity detection" (ICDCN 2017, arXiv:1608.02432,
//! Algorithm 3 `Gather_k`; journal version JPDC 123, 2019, not checked).
//!
//! SSYNC, unlimited visibility, no common direction or chirality, weak
//! multiplicity detection, up to n - 1 robots crashed. From a legal start (at
//! most one multiplicity point) the robots that work gather (Theorem 2, from
//! Lemmas 4 and 5: there is never a second multiplicity point):
//!
//! - if a multiplicity point m exists the destination p* is m, else the
//!   centre O of the smallest enclosing circle of all robots;
//! - a robot with a robot-free path to p* moves straight there;
//! - otherwise it moves along a circular arc G through itself and p*, tangent
//!   at p* to the bisector of the smallest robot-free sector next to it, so
//!   that no two paths meet before p* and no second multiplicity appears.
//!
//! **Width of a robot.** A robot counts as standing on a path, or on another
//! robot's ray from p*, when it is within `w = max(robot_radius, ε)` of it:
//! the core's blocking width (docs/core-schema.md §2, `robot_radius`), and
//! never less than ε, the distance at which two robots are one point for
//! multiplicity detection. The same `w` decides that a path is blocked and which robots
//! lie on the blocked robot's own ray when its sector is chosen. With two
//! tolerances (ε and 1e-12 rad) a robot just off the ray blocked the path but
//! was then taken as the edge of the free sector, and the arc landed on it
//! (review of #17, fix B1, case 2).
//!
//! **Approximation.** The core moves robots in straight lines. A blocked robot
//! here moves to the point halfway along its arc G (a chord of G); from there
//! its path is usually free and it goes straight. The chord stays inside the
//! robot-free sector, so it meets no robot; should a robot still be within
//! `w` of the chord or its end (one just off the ray), the step is halved
//! until it is clear. Robots on one ray share the tangent line, so their
//! circles are nested and meet only at p*; the paper's recursive tangent
//! (`FindTangent` with the minimum step S) is not used.
//!
//! **Every other robot on the ray.** When every robot not on p* lies on the
//! blocked robot's own ray, no robot bounds a sector: the free sector is the
//! whole plane and has no bisector, and the paper's `FindTangent` has no
//! answer either. The rule here takes the tangent at p* perpendicular to the
//! ray, so G is the circle with the segment from the robot to p* as its
//! diameter, and the robot steps off the line at 45 degrees, to the point of
//! G above the segment's midpoint. Its path leaves the line at once, so it
//! never passes through the robot in front. (The old fallback went to the
//! plain midpoint, straight through that robot, and the robots on the line
//! leapfrogged each other for ever: review of #17, fix B1, case 1.) All such
//! circles pass through p* tangent to the same line, so two of them meet only
//! at p*, whichever side each robot's own frame picks.
//!
//! Crashes happen at the start in this simulator (the paper allows any time).
//! A robot on the multiplicity point stays there for good (it stays the only
//! multiplicity), so it terminates; the run ends once every working robot has.

use crate::sec::welzl_recur;
use core::f64::consts::{FRAC_PI_2, PI};
use lcm_core::geom::dist;
use lcm_core::pyfloat::pow10_neg;
use lcm_core::{
    Algorithm, Circle, ConfigError, Decision, Look, MultiplicityModel, OverlayUpdate, Plan, Point,
    Rng, SchedulerKind, Scratch, SimConfig,
};

#[derive(Clone, Copy, Debug, Default)]
pub struct CrashGathering {
    /// The robots' radius in units of ε (`robot_radius / ε`), so that a robot
    /// scales it to its own unit length as it does ε.
    pub radius: f64,
}

impl CrashGathering {
    /// `max(robot_radius, ε)` in the robot's own unit: a robot within this of
    /// a path is in the way.
    fn width(self, look: &Look<'_>) -> f64 {
        look.model.eps * self.radius.max(1.0)
    }
}

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

/// Whether some other robot stands within `w` of the segment from `p` to
/// `q`, strictly before `q` (robots at `q` are the destination, not in the
/// way, and robots on `p` are `p`'s own point).
fn blocked(p: Point, q: Point, others: &[Point], w: f64) -> bool {
    let (dx, dy) = (q.x - p.x, q.y - p.y);
    let len2 = dx * dx + dy * dy;
    if len2 <= w * w {
        return false;
    }
    others.iter().any(|&r| {
        if dist(r, p) <= w || dist(r, q) <= w {
            return false;
        }
        let s = ((r.x - p.x) * dx + (r.y - p.y) * dy) / len2;
        s > 0.0 && s < 1.0 && dist(r, Point::new(p.x + s * dx, p.y + s * dy)) <= w
    })
}

/// Whether the straight move from `p` to `s` passes or ends within `w` of
/// another robot (robots on `p` itself aside).
fn step_hits(p: Point, s: Point, others: &[Point], w: f64) -> bool {
    let (dx, dy) = (s.x - p.x, s.y - p.y);
    let len2 = dx * dx + dy * dy;
    others.iter().any(|&r| {
        if dist(r, p) <= w {
            return false;
        }
        let t = if len2 > 0.0 {
            (((r.x - p.x) * dx + (r.y - p.y) * dy) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        dist(r, Point::new(p.x + t * dx, p.y + t * dy)) <= w
    })
}

/// The halfway point of the arc a blocked robot at `p` takes to `c`: the
/// circle through `p` and `c` tangent at `c` to the bisector of the smallest
/// robot-free sector (around `c`) next to `p`'s ray. Robots within `w` of
/// `p`'s ray are on it. With no robot off the ray the tangent is
/// perpendicular to the ray (see the module notes). The step is halved while
/// it would pass or land within `w` of a robot.
#[must_use]
pub fn arc_step(p: Point, c: Point, others: &[Point], w: f64) -> Point {
    let angle = |q: Point| libm::atan2(q.y - c.y, q.x - c.x).rem_euclid(2.0 * PI);
    let mine = angle(p);
    let r = dist(p, c);
    let (ux, uy) = ((p.x - c.x) / r, (p.y - c.y) / r);
    let (mut ccw, mut cw) = (2.0 * PI, 2.0 * PI);
    for &q in others {
        if dist(q, c) <= w {
            continue;
        }
        let (qx, qy) = (q.x - c.x, q.y - c.y);
        if qx * ux + qy * uy > 0.0 && (qx * uy - qy * ux).abs() <= w {
            continue; // on p's own ray
        }
        let a = (angle(q) - mine).rem_euclid(2.0 * PI);
        ccw = ccw.min(a);
        cw = cw.min(2.0 * PI - a);
    }
    // The smaller of the two sectors next to p's ray, and its bisector; with
    // no robot off the ray, the perpendicular.
    let heading = if ccw >= 2.0 * PI {
        mine + FRAC_PI_2
    } else if ccw <= cw {
        mine + ccw / 2.0
    } else {
        mine - cw / 2.0
    };
    let (tx, ty) = (libm::cos(heading), libm::sin(heading));
    // The circle's centre is on the normal to the tangent at c.
    let (nx, ny) = (-ty, tx);
    let (vx, vy) = (c.x - p.x, c.y - p.y);
    let along = nx * vx + ny * vy;
    let mut s = if along.abs() <= 1e-15 {
        // Tangent along the ray: only for a sector of a full turn, which the
        // rule above has replaced by the perpendicular. Kept as a guard.
        Point::new((p.x + c.x) / 2.0, (p.y + c.y) / 2.0)
    } else {
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
        let radius = t.abs();
        let m = a0 + d / 2.0;
        Point::new(o.x + radius * libm::cos(m), o.y + radius * libm::sin(m))
    };
    for _ in 0..64 {
        if !step_hits(p, s, others, w) {
            return s;
        }
        s = Point::new((p.x + s.x) / 2.0, (p.y + s.y) / 2.0);
    }
    // Hemmed in within `w` on every side: wait for the others to move.
    p
}

impl CrashGathering {
    /// Adds to `text` that a robot stands in the way to `target`, if one does.
    fn blocked_note(
        self,
        me: Point,
        target: Point,
        occupied: &[lcm_core::occupancy::Occupied],
        mine: usize,
        w: f64,
        text: &mut String,
    ) {
        let others: Vec<Point> = occupied
            .iter()
            .enumerate()
            .filter(|&(k, _)| k != mine)
            .map(|(_, o)| o.at)
            .collect();
        if blocked(me, target, &others, w) {
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
        let goal = if blocked(me, target, &others, self.width(look)) {
            arc_step(me, target, &others, self.width(look))
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
                    self.blocked_note(me, m, &occupied, mine, self.width(look), &mut text);
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
            self.blocked_note(
                me,
                target.center,
                &occupied,
                mine,
                self.width(look),
                &mut text,
            );
        }
        Some(text)
    }
}

/// The robots' radius reaches the algorithm in units of ε, the one length
/// every robot knows in its own unit (docs/core-schema.md §7.1).
///
/// # Errors
/// The asynchronous schedulers (the paper's `Gather_k` is for SSYNC; its
/// ASYNC algorithm is different), or a model without weak detection.
pub fn plan(config: &SimConfig) -> Result<Plan, ConfigError> {
    if matches!(
        config.scheduler,
        SchedulerKind::Async | SchedulerKind::KAsync
    ) {
        return Err(ConfigError::Unsupported(
            "Gathering despite crashes works in rounds: pick Sequential, SSYNC or FSYNC as the scheduler. It needs every robot that acts to see the swarm at the same moment, which robots on their own clocks cannot guarantee.".to_owned(),
        ));
    }
    if matches!(
        config.multiplicity,
        MultiplicityModel::None | MultiplicityModel::Local
    ) {
        return Err(ConfigError::Unsupported(
            "Gathering despite crashes needs robots that can see stacks: set Multiplicity detection to Weak or Strong. The working robots find each other by heading for the stack, so they must be able to tell a stack from a single robot.".to_owned(),
        ));
    }
    Ok(Plan::single(Box::new(CrashGathering {
        radius: config.robot_radius / pow10_neg(config.threshold_precision),
    })))
}

/// Where the settings leave the paper's model, one sentence each.
#[must_use]
pub fn notes(config: &SimConfig) -> Vec<String> {
    let mut notes = vec![
        "Simplification: crashes happen only at the start of the run (the paper's robots can crash at any time), and a robot curves around a robot in its way by one short step off its straight path instead of along the paper's arc.".to_owned(),
    ];
    if config.num_of_faults > 0 && config.fault_type != lcm_core::FaultSelection::Crash {
        notes.push("Outside the model: the paper tolerates crash faults only; Byzantine, omission and delay faults can mislead or stall the working robots.".to_owned());
    }
    if config.multiplicities > 1 {
        notes.push("Outside the model: the paper needs a start with at most one stack, since two stacks whose robots always move together can never be merged.".to_owned());
    }
    if config.visibility_radius.is_some() {
        notes.push(
            "Outside the model: with a limited sight radius robots may head for different stacks or centres and never meet.".to_owned(),
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

    /// Review of #17, fix B1, case 1: nobody off the ray, so no sector. The
    /// robot steps to the point of the circle on its segment to c as
    /// diameter, off the line, never through the robot in front.
    #[test]
    fn with_everyone_on_the_ray_the_robot_steps_off_the_line() {
        let c = Point::new(30.0, 0.0);
        let others = [Point::new(10.0, 0.0), Point::new(20.0, 0.0)];
        let s = arc_step(Point::new(0.0, 0.0), c, &others, 1e-5);
        assert!(
            (s.x - 15.0).abs() < 1e-9 && (s.y.abs() - 15.0).abs() < 1e-9,
            "{s:?}"
        );
        assert!(!step_hits(Point::new(0.0, 0.0), s, &others, 1e-5));
    }

    /// Review of #17, fix B1, case 2: a robot 8e-6 off the ray (ε = 1e-5)
    /// blocks the path and is on the ray for the choice of sector, so the arc
    /// does not hug it.
    #[test]
    fn a_robot_within_the_width_of_the_ray_is_on_it() {
        let c = Point::new(0.0, 0.0);
        let p = Point::new(10.0, 0.0);
        let near = Point::new(5.0, 8e-6);
        let others = [near, Point::new(0.0, 7.0)];
        assert!(blocked(p, c, &others, 1e-5));
        let s = arc_step(p, c, &others, 1e-5);
        assert!(dist(s, near) > 1.0, "{s:?}");
        assert!(!step_hits(p, s, &others, 1e-5));
    }
}

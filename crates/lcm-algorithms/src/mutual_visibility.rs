//! Mutual visibility with obstructed visibility, no lights: Di Luna,
//! Flocchini, Gan Chaudhuri, Poloni, Santoro, Viglietta, "Mutual visibility by
//! luminous robots without collisions", Information and Computation 254,
//! 2017 (arXiv:1503.04347v2, Section 6.7).
//!
//! Robots see each other only if no robot stands on the segment between
//! them. Under a sequential scheduler: "When a robot is activated, it just
//! moves by a small amount, without crossing or landing on any line that
//! passes through two robots that it can currently see (including itself)."
//! Once every robot (except possibly two) has moved, all see each other, and
//! the rule keeps it so. The algorithm never terminates: robots keep making
//! small moves (the paper terminates only with lights or with its SSYNC
//! algorithm). A robot that stopped on seeing all n others could be hidden
//! later by a robot that could not see it, so stopping is not added here.
//!
//! The paper leaves "a small amount" open. Here the robot moves along the
//! middle of the widest angle between the directions to the robots it sees
//! (so it leaves every line through itself and another robot), by half its
//! distance to the band of width ε around the nearest line through two other
//! visible robots (the core counts a robot that close as blocking; a band it
//! stands in it only leaves), and never more than a quarter of the way to
//! its nearest neighbour. The directions and their opposites make a set
//! symmetric under a half turn, so the widest gap always has a twin pointing
//! the other way; the first in the robot's own angular order is taken, so the
//! heading depends on its frame (any small move is allowed by the paper).

use core::f64::consts::PI;
use lcm_core::{
    Algorithm, ConfigError, Decision, Look, OverlayUpdate, Plan, Point, Rng, SchedulerKind,
    Scratch, SimConfig,
};

#[derive(Clone, Copy, Debug, Default)]
pub struct MutualVisibility;

/// Distance from `o` to the line through `a` and `b`.
fn line_distance(o: Point, a: Point, b: Point) -> f64 {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let len = libm::hypot(dx, dy);
    if len == 0.0 {
        return libm::hypot(o.x - a.x, o.y - a.y);
    }
    ((o.x - a.x) * dy - (o.y - a.y) * dx).abs() / len
}

/// The step a robot at `me` takes among the visible `others`.
#[must_use]
pub fn step(me: Point, others: &[Point], eps: f64) -> Point {
    if others.is_empty() {
        return me;
    }
    // Directions to every visible robot, and their opposites: moving along
    // any of them would keep the robot on a line through itself and another.
    let mut angles: Vec<f64> = others
        .iter()
        .flat_map(|p| {
            let a = libm::atan2(p.y - me.y, p.x - me.x);
            [a, a + PI]
        })
        .map(|a| a.rem_euclid(2.0 * PI))
        .collect();
    angles.sort_by(f64::total_cmp);
    let mut best = (
        angles[0] + 2.0 * PI - angles[angles.len() - 1],
        angles[angles.len() - 1],
    );
    for w in angles.windows(2) {
        if w[1] - w[0] > best.0 {
            best = (w[1] - w[0], w[0]);
        }
    }
    let heading = best.1 + best.0 / 2.0;
    let mut room = f64::INFINITY;
    for (k, &a) in others.iter().enumerate() {
        room = room.min(libm::hypot(a.x - me.x, a.y - me.y) / 4.0);
        for &b in &others[k + 1..] {
            // The core counts a robot within ε of a segment as blocking it,
            // so each line is a band of width ε. A band the robot already
            // stands in (it blocks those two) it only leaves: the heading is
            // never along it. Any other it must not enter.
            let d = line_distance(me, a, b);
            if d > eps {
                room = room.min((d - eps) / 2.0);
            }
        }
    }
    Point::new(
        me.x + room * libm::cos(heading),
        me.y + room * libm::sin(heading),
    )
}

impl Algorithm for MutualVisibility {
    fn key(&self) -> &'static str {
        "MutualVisibility"
    }

    fn compute(&self, look: &Look<'_>, _: &mut Rng, scratch: &mut Scratch) -> Decision {
        let view = look.view;
        let me = look.position;
        let others = &mut scratch.points;
        others.clear();
        others.extend(
            (0..view.len())
                .filter(|&k| k != look.me_index)
                .map(|k| view.pos[k]),
        );
        Decision {
            target: step(me, others, look.model.eps),
            terminate: false,
            overlay: OverlayUpdate::Keep,
        }
    }
}

/// # Errors
/// Never; the model notes say where a run leaves the paper's setting.
#[allow(clippy::unnecessary_wraps)]
pub fn plan(_: &SimConfig) -> Result<Plan, ConfigError> {
    Ok(Plan::single(Box::new(MutualVisibility)))
}

#[must_use]
pub fn notes(config: &SimConfig) -> Vec<String> {
    let mut notes = vec![
        "The robots never stop by themselves. Even once every robot sees every other one they keep making tiny adjustments, because a robot cannot be sure that nobody is still hidden, and those adjustments are chosen so that everybody stays in sight. The run ends when the moves become too small to make (shorter than ε), or at a turn limit if you set one.".to_owned(),
    ];
    if !config.obstructed_visibility {
        notes.push("Turn on Settings → Model → Visibility → “Robots block the view” to see what this algorithm is for. With transparent robots every robot already sees every other one from the very start, so there is nothing for the algorithm to fix and the robots barely move.".to_owned());
    }
    if config.scheduler != SchedulerKind::Sequential {
        notes.push("Outside the model: this algorithm is designed for one robot at a time. When several robots move together, two of them can end up in line with a third without either noticing, which hides robots from each other again; the run may take much longer, or stop with some robots still hidden.".to_owned());
    }
    if config.visibility_radius.is_some() {
        notes.push("Outside the model: this algorithm is designed for robots that can see arbitrarily far, where only other robots block the view. With a limited sight radius, robots that are far apart never see each other however they move, so complete mutual visibility may be out of reach.".to_owned());
    }
    notes
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_step_leaves_the_line_and_stays_short_of_other_lines() {
        let me = Point::new(0.0, 0.0);
        let others = [
            Point::new(5.0, 0.0),
            Point::new(-5.0, 0.0),
            Point::new(0.0, 8.0),
        ];
        let s = step(me, &others, 1e-9);
        // Off the line through the two robots on the x axis.
        assert!(s.y.abs() > 1e-9);
        for (k, &a) in others.iter().enumerate() {
            for &b in &others[k + 1..] {
                // Same side of every line through two others as before.
                let side = |p: Point| (p.x - a.x) * (b.y - a.y) - (p.y - a.y) * (b.x - a.x);
                assert!(side(me) * side(s) > 0.0 || side(me) == 0.0);
            }
        }
    }
}

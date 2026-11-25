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
//! distance to the band of half-width `w = max(robot_radius, ε)` around the
//! nearest line through two other visible robots (the core counts a robot
//! that close to a segment as blocking it, docs/core-schema.md §2,
//! `robot_radius`), and never more than a quarter of the way to its nearest
//! neighbour. With fat robots a band of ε let the step end within the
//! robot's radius of a line, hiding two robots from each other (review of
//! #17, fix B5). The robots' radius reaches the algorithm in units of ε, the
//! one length every robot knows in its own unit.
//!
//! **Fat robots, known gap.** The paper's robots are points, for which
//! keeping off every line through two others is enough. A robot of radius r
//! also hides the robot behind it from a robot whose line of sight passes
//! within r of it: when robot k stands between the mover and robot j, k is
//! closer to the line from the mover to j than the mover is to the line
//! through k and j (by similar triangles), so the band above does not keep
//! k out of the mover's new line of sight to j. A random check finds such
//! moves; avoiding them would need a cone behind each robot, not a band, and
//! is not done here (`notes` says so when `robot_radius` > 0). The directions and their opposites make a set
//! symmetric under a half turn, so the widest gap always has a twin pointing
//! the other way; the first in the robot's own angular order is taken, so the
//! heading depends on its frame (any small move is allowed by the paper).

use core::f64::consts::PI;
use lcm_core::pyfloat::pow10_neg;
use lcm_core::{
    Algorithm, ConfigError, Decision, Look, OverlayUpdate, Plan, Point, Rng, SchedulerKind,
    Scratch, SimConfig,
};

#[derive(Clone, Copy, Debug, Default)]
pub struct MutualVisibility {
    /// The robots' radius in units of ε (`robot_radius / ε`).
    pub radius: f64,
}

/// Distance from `o` to the line through `a` and `b`.
fn line_distance(o: Point, a: Point, b: Point) -> f64 {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let len = libm::hypot(dx, dy);
    if len == 0.0 {
        return libm::hypot(o.x - a.x, o.y - a.y);
    }
    ((o.x - a.x) * dy - (o.y - a.y) * dx).abs() / len
}

/// The step a robot at `me` takes among the visible `others`, keeping out of
/// the band of half-width `w` around every line through two of them.
#[must_use]
pub fn step(me: Point, others: &[Point], w: f64) -> Point {
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
            // The core counts a robot within w of a segment as blocking it,
            // so each line is a band of half-width w, which the step must
            // not enter. A band the robot already stands in sets no limit:
            // the heading is never exactly along its line, so the robot
            // drifts out of it, but it can move some way inside it. That
            // only hides a and b if the robot stands between them, and then
            // it already hides them.
            let d = line_distance(me, a, b);
            if d > w {
                room = room.min((d - w) / 2.0);
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
            target: step(me, others, look.model.eps * self.radius.max(1.0)),
            terminate: false,
            overlay: OverlayUpdate::Keep,
        }
    }
}

/// # Errors
/// Never; the model notes say where a run leaves the paper's setting.
#[allow(clippy::unnecessary_wraps)]
pub fn plan(config: &SimConfig) -> Result<Plan, ConfigError> {
    Ok(Plan::single(Box::new(MutualVisibility {
        radius: config.robot_radius / pow10_neg(config.threshold_precision),
    })))
}

/// Where the settings leave the paper's model, one sentence each.
#[must_use]
pub fn notes(config: &SimConfig) -> Vec<String> {
    let mut notes = vec![
        "The robots never stop by themselves: they keep making tiny moves that keep everyone in sight, until the moves are shorter than ε or a turn limit is reached.".to_owned(),
    ];
    if !config.obstructed_visibility {
        notes.push("Turn on obstructed visibility (robots block the view) to see what this algorithm is for: with transparent robots everyone already sees everyone.".to_owned());
    }
    if config.scheduler != SchedulerKind::Sequential {
        notes.push("Outside the model: the paper's rule is for one robot at a time, and robots moving together can line up and hide each other again.".to_owned());
    }
    if config.robot_radius > 0.0 {
        notes.push("Outside the model: the paper's robots are points, and a fat robot's move can still bring another robot into its own line of sight to a third.".to_owned());
    }
    if config.visibility_radius.is_some() {
        notes.push("Outside the model: with a limited sight radius, robots that are far apart never see each other however they move.".to_owned());
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

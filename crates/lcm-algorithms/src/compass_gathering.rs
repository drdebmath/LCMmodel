//! Gathering two robots with unreliable compasses: Izumi, Souissi, Katayama,
//! Inuzuka, Défago, Wada, Yamashita, "The gathering problem for two oblivious
//! robots with unreliable compasses", SIAM J. Comput. 41(1), 2012
//! (arXiv:1111.1492, Sections 3-5).
//!
//! Each robot's x-axis deviates from the global one by at most φ (a
//! φ-compass); robots share chirality; φ is known to the algorithm. With `p`
//! the other robot's position in the robot's own frame and `a = arg p` in
//! [0, 2π):
//!
//! - **A_SS** (static compass, SSYNC or ASYNC, correct iff φ < π/2):
//!   `p = 0` stay · `0 < a ≤ π` go to `p` · `π < a ≤ 3π/2 + φ` go to
//!   `(-|p|, 0)` · otherwise stay.
//! - **A_SD** (dynamic compass, SSYNC, correct iff φ < π/4):
//!   `π/2 + φ < a ≤ 3π/2 - φ` go to `p` · `-π/2 + φ < a ≤ π/2 - φ` stay ·
//!   otherwise go to `p` turned by `π/2 + φ`.
//! - **A_AD** (dynamic compass, ASYNC, sufficient for φ < π/6):
//!   `2π/3 + φ ≤ a < 3π/2` go to `p` · `3π/2 ≤ a` or `a ≤ π/3 - φ` stay ·
//!   otherwise go to `p` turned by `2π/3 + 2φ`.
//!
//! Runs take the compass from the `frames` setting: `Compass` gives each
//! robot a deviation of at most φ (`compass_error`), static or redrawn at
//! every Look; `Global` is an exact compass (φ = 0, the static rules).
//! `Rotated` and `Disoriented` are refused: they turn each robot's axes by an
//! unbounded random angle at every Look, and `Disoriented` also flips
//! handedness, so the paper's bounded deviation and shared chirality fail
//! (review of #17, fix B2).
//!
//! The paper's robots never terminate (gathering is "eventually co-located
//! forever"). Here a robot that finds the other on its own point terminates
//! under the sequential and synchronous schedulers, where that is safe; under
//! async it only stays (Remark 2 of the paper shows terminating is wrong there).

use core::f64::consts::{FRAC_PI_2, PI};
use lcm_core::{
    Algorithm, ConfigError, Decision, Frames, Look, OverlayUpdate, Plan, Point, Rng, SchedulerKind,
    Scratch, SimConfig,
};

/// Which rule set of the paper a run uses.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Rules {
    StaticCompass,
    DynamicSynchronous,
    DynamicAsynchronous,
}

#[derive(Clone, Copy, Debug)]
pub struct CompassGathering {
    pub rules: Rules,
    /// The bound φ on the compass error, in radians, known to the robots.
    pub phi: f64,
    /// Terminate once gathered (not under async, see the module notes).
    pub terminate: bool,
}

/// `ρ_ω(p)`: `p` turned counter-clockwise by `ω` about the robot.
fn turn(p: Point, w: f64) -> Point {
    let (s, c) = (libm::sin(w), libm::cos(w));
    Point::new(c * p.x - s * p.y, s * p.x + c * p.y)
}

/// What a rule set does when the other robot lies at angle `a` in [0, 2π)
/// in the robot's own frame.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Action {
    /// Stay where it is.
    Stay,
    /// Go to the other robot.
    Go,
    /// A_SS: go west by the distance to the other robot.
    West,
    /// A_SD / A_AD: go to the other robot's position turned about itself.
    Turn,
}

/// The sector of `rules` that angle `a` (in [0, 2π)) falls in.
#[must_use]
pub fn action(rules: Rules, phi: f64, a: f64) -> Action {
    match rules {
        Rules::StaticCompass => {
            if a > 0.0 && a <= PI {
                Action::Go
            } else if a > PI && a <= 3.0 * FRAC_PI_2 + phi {
                Action::West
            } else {
                Action::Stay
            }
        }
        Rules::DynamicSynchronous => {
            if a > FRAC_PI_2 + phi && a <= 3.0 * FRAC_PI_2 - phi {
                Action::Go
            } else if a <= FRAC_PI_2 - phi || a > 3.0 * FRAC_PI_2 + phi {
                // (-π/2 + φ, π/2 - φ], written on [0, 2π).
                Action::Stay
            } else {
                Action::Turn
            }
        }
        Rules::DynamicAsynchronous => {
            if a >= 2.0 * PI / 3.0 + phi && a < 3.0 * FRAC_PI_2 {
                Action::Go
            } else if a >= 3.0 * FRAC_PI_2 || a <= PI / 3.0 - phi {
                Action::Stay
            } else {
                Action::Turn
            }
        }
    }
}

/// The rule's destination for a robot at the origin seeing the other at `p`.
#[must_use]
pub fn destination(rules: Rules, phi: f64, p: Point) -> Point {
    let mut a = libm::atan2(p.y, p.x);
    if a < 0.0 {
        a += 2.0 * PI;
    }
    match action(rules, phi, a) {
        Action::Stay => Point::new(0.0, 0.0),
        Action::Go => p,
        Action::West => Point::new(-libm::hypot(p.x, p.y), 0.0),
        Action::Turn => match rules {
            Rules::DynamicAsynchronous => turn(p, 2.0 * PI / 3.0 + 2.0 * phi),
            _ => turn(p, FRAC_PI_2 + phi),
        },
    }
}

impl Algorithm for CompassGathering {
    fn key(&self) -> &'static str {
        "CompassGathering"
    }

    fn compute(&self, look: &Look<'_>, _: &mut Rng, _: &mut Scratch) -> Decision {
        let me = look.position;
        let other = (0..look.view.len())
            .find(|&k| k != look.me_index)
            .map(|k| look.view.pos[k]);
        let Some(q) = other else {
            return Decision {
                target: me,
                terminate: false,
                overlay: OverlayUpdate::Keep,
            };
        };
        let p = Point::new(q.x - me.x, q.y - me.y);
        if libm::hypot(p.x, p.y) <= look.model.eps {
            return Decision {
                target: me,
                terminate: self.terminate,
                overlay: OverlayUpdate::Keep,
            };
        }
        let d = destination(self.rules, self.phi, p);
        Decision {
            target: Point::new(me.x + d.x, me.y + d.y),
            terminate: false,
            overlay: OverlayUpdate::Keep,
        }
    }

    fn explain(&self, look: &Look<'_>, _: &mut Scratch) -> Option<String> {
        let me = look.position;
        let q = (0..look.view.len())
            .find(|&k| k != look.me_index)
            .map(|k| look.view.pos[k])?;
        let p = Point::new(q.x - me.x, q.y - me.y);
        let gap = libm::hypot(p.x, p.y);
        let name = match self.rules {
            Rules::StaticCompass => "fixed compass",
            Rules::DynamicSynchronous => "changing compass, robots in rounds",
            Rules::DynamicAsynchronous => "changing compass, robots on their own",
        };
        if gap <= look.model.eps {
            return Some(format!(
                "The other robot is on its point: they have gathered. {}",
                if self.terminate {
                    "It terminates."
                } else {
                    "Acting on its own, it stays without stopping."
                }
            ));
        }
        let bearing = libm::atan2(p.y, p.x).to_degrees().rem_euclid(360.0);
        let d = destination(self.rules, self.phi, p);
        let action = if d == p {
            "it moves onto the other robot".to_owned()
        } else if d.x == 0.0 && d.y == 0.0 {
            "it waits".to_owned()
        } else if self.rules == Rules::StaticCompass {
            format!("it steps west by the gap ({} units)", crate::num(gap))
        } else {
            let turn = libm::atan2(d.y, d.x) - libm::atan2(p.y, p.x);
            format!(
                "it moves to the other robot's position turned {}° counter-clockwise about itself",
                crate::num(turn.to_degrees().rem_euclid(360.0))
            )
        };
        Some(format!(
            "By its own compass the other robot is {} units away, {}° counter-clockwise from its east. With the rules for a {name} (error up to {}°), at that bearing {action}.",
            crate::num(gap),
            crate::num(bearing),
            crate::num(self.phi.to_degrees()),
        ))
    }
}

/// The registry entry: two robots; the rule set follows the compass and the
/// scheduler (static: A_SS; dynamic: A_SD when robots never overlap, A_AD
/// otherwise).
///
/// # Errors
/// Not exactly two robots, faulty robots, or rotated or disoriented frames.
pub fn plan(config: &SimConfig) -> Result<Plan, ConfigError> {
    let refuse = |m: &str| Err(ConfigError::Unsupported(m.to_owned()));
    let n = config
        .initial_positions
        .as_ref()
        .filter(|p| p.len() == config.num_of_robots as usize)
        .map_or(config.num_of_robots as usize, Vec::len);
    if n != 2 {
        return refuse(
            "Compass gathering is for exactly two robots: its rules decide what to do from where the one other robot lies by the robot's own compass. Set Robots to 2.",
        );
    }
    if config.num_of_faults > 0 {
        return refuse("Compass gathering is designed for two robots that never fail: with only two robots, a faulty one leaves nobody to gather with. Set Faulty robots to 0.");
    }
    if matches!(config.frames, Frames::Rotated | Frames::Disoriented) {
        return refuse("Compass gathering needs robots whose compasses are off by a bounded angle and agree on clockwise: set Robots' axes to Shared (exact compasses) or to Unreliable compass. Rotated and disoriented frames turn each robot's axes at random at every look (and disoriented ones also mirror them), so the rules lose their meaning.");
    }
    let overlapping = matches!(
        config.scheduler,
        SchedulerKind::Async | SchedulerKind::KAsync
    );
    let rules = if config.compass_static || config.frames != Frames::Compass {
        Rules::StaticCompass
    } else if overlapping {
        Rules::DynamicAsynchronous
    } else {
        Rules::DynamicSynchronous
    };
    let phi = if config.frames == Frames::Compass {
        config.compass_error.to_radians()
    } else {
        0.0
    };
    Ok(Plan::single(Box::new(CompassGathering {
        rules,
        phi,
        terminate: !overlapping,
    })))
}

/// Where the settings leave the paper's guarantees, one sentence each.
#[must_use]
pub fn notes(config: &SimConfig) -> Vec<String> {
    let mut notes = Vec::new();
    if config.frames != Frames::Compass {
        notes.push(
            "The compasses are exact here (Robots' axes: Shared), so the robots use the fixed-compass rules with no error; choose Unreliable compass to test the rules.".to_owned(),
        );
        return notes;
    }
    let phi = config.compass_error;
    let overlapping = matches!(
        config.scheduler,
        SchedulerKind::Async | SchedulerKind::KAsync
    );
    let (name, bound) = if config.compass_static {
        ("fixed compasses", 90.0)
    } else if overlapping {
        ("changing compasses on robots with their own clocks", 30.0)
    } else {
        ("changing compasses on robots acting in rounds", 45.0)
    };
    if phi >= bound {
        notes.push(format!(
            "Outside the model: the rules for {name} are proved only for compass errors below {bound}°, and this run allows {phi}°."
        ));
    }
    if overlapping {
        notes.push("With robots on their own clocks the two robots never terminate, even once together, since one cannot know the other has stopped moving (Remark 2 of the paper).".to_owned());
    }
    notes
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(deg: f64) -> Point {
        let a = deg.to_radians();
        Point::new(libm::cos(a), libm::sin(a))
    }

    #[test]
    fn static_rules_by_sector() {
        let phi = 30f64.to_radians();
        // North-ish (0 < a ≤ π): go to the other robot.
        assert_eq!(destination(Rules::StaticCompass, phi, at(90.0)), at(90.0));
        // South-west (π < a ≤ 3π/2 + φ): go west by |p|.
        let d = destination(Rules::StaticCompass, phi, at(250.0));
        assert!((d.x + 1.0).abs() < 1e-12 && d.y.abs() < 1e-12);
        // Just east of south beyond 3π/2 + φ: stay.
        assert_eq!(
            destination(Rules::StaticCompass, phi, at(330.0)),
            Point::new(0.0, 0.0)
        );
    }

    #[test]
    fn dynamic_synchronous_rules_by_sector() {
        let phi = 20f64.to_radians();
        assert_eq!(
            destination(Rules::DynamicSynchronous, phi, at(180.0)),
            at(180.0)
        );
        assert_eq!(
            destination(Rules::DynamicSynchronous, phi, at(0.0)),
            Point::new(0.0, 0.0)
        );
        let r = destination(Rules::DynamicSynchronous, phi, at(90.0));
        let want = at(90.0 + 90.0 + 20.0);
        assert!((r.x - want.x).abs() < 1e-12 && (r.y - want.y).abs() < 1e-12);
    }

    #[test]
    fn dynamic_asynchronous_rules_by_sector() {
        let phi = 10f64.to_radians();
        assert_eq!(
            destination(Rules::DynamicAsynchronous, phi, at(200.0)),
            at(200.0)
        );
        assert_eq!(
            destination(Rules::DynamicAsynchronous, phi, at(300.0)),
            Point::new(0.0, 0.0)
        );
        assert_eq!(
            destination(Rules::DynamicAsynchronous, phi, at(30.0)),
            Point::new(0.0, 0.0)
        );
        let r = destination(Rules::DynamicAsynchronous, phi, at(90.0));
        let want = at(90.0 + 120.0 + 20.0);
        assert!((r.x - want.x).abs() < 1e-12 && (r.y - want.y).abs() < 1e-12);
    }
}

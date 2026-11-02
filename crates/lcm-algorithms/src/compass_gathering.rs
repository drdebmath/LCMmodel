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
//! The paper's robots never terminate (gathering is "eventually co-located
//! forever"). Here a robot that finds the other on its own point terminates
//! under the sequential and synchronous schedulers, where that is safe; under
//! async it only stays (Remark 2 of the paper shows terminating is wrong there).

use core::f64::consts::{FRAC_PI_2, PI};
use lcm_core::{
    Algorithm, ConfigError, Decision, Look, OverlayUpdate, Plan, Point, Rng, SchedulerKind,
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

/// The rule's destination for a robot at the origin seeing the other at `p`.
#[must_use]
pub fn destination(rules: Rules, phi: f64, p: Point) -> Point {
    let stay = Point::new(0.0, 0.0);
    let mut a = libm::atan2(p.y, p.x);
    if a < 0.0 {
        a += 2.0 * PI;
    }
    match rules {
        Rules::StaticCompass => {
            if a > 0.0 && a <= PI {
                p
            } else if a > PI && a <= 3.0 * FRAC_PI_2 + phi {
                Point::new(-libm::hypot(p.x, p.y), 0.0)
            } else {
                stay
            }
        }
        Rules::DynamicSynchronous => {
            if a > FRAC_PI_2 + phi && a <= 3.0 * FRAC_PI_2 - phi {
                p
            } else if a <= FRAC_PI_2 - phi || a > 3.0 * FRAC_PI_2 + phi {
                // (-π/2 + φ, π/2 - φ], written on [0, 2π).
                stay
            } else {
                turn(p, FRAC_PI_2 + phi)
            }
        }
        Rules::DynamicAsynchronous => {
            if a >= 2.0 * PI / 3.0 + phi && a < 3.0 * FRAC_PI_2 {
                p
            } else if a >= 3.0 * FRAC_PI_2 || a <= PI / 3.0 - phi {
                stay
            } else {
                turn(p, 2.0 * PI / 3.0 + 2.0 * phi)
            }
        }
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
/// Not exactly two robots, or faulty robots.
pub fn plan(config: &SimConfig) -> Result<Plan, ConfigError> {
    let refuse = |m: &str| Err(ConfigError::Unsupported(m.to_owned()));
    let n = config
        .initial_positions
        .as_ref()
        .filter(|p| p.len() == config.num_of_robots as usize)
        .map_or(config.num_of_robots as usize, Vec::len);
    if n != 2 {
        return refuse(
            "Compass gathering is for exactly two robots: its rules decide what to do from where the one other robot lies by the robot's own compass. Set Robots to 2 (Settings → Robots → Swarm).",
        );
    }
    if config.num_of_faults > 0 {
        return refuse("Compass gathering is designed for two robots that never fail: with only two robots, a faulty one leaves nobody to gather with. Set Faulty robots to 0 (Settings → Model → Faults).");
    }
    let overlapping = matches!(
        config.scheduler,
        SchedulerKind::Async | SchedulerKind::KAsync
    );
    let rules = if config.compass_static || config.frames != lcm_core::Frames::Compass {
        Rules::StaticCompass
    } else if overlapping {
        Rules::DynamicAsynchronous
    } else {
        Rules::DynamicSynchronous
    };
    let phi = if config.frames == lcm_core::Frames::Compass {
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

/// Where the settings leave the paper's guarantees.
#[must_use]
pub fn notes(config: &SimConfig) -> Vec<String> {
    let mut notes = Vec::new();
    let phi = config.compass_error;
    if config.frames != lcm_core::Frames::Compass {
        notes.push(
            "The robots' compasses are exact in this run, because Coordinate frames is not set to “Compass with an error”. Both robots agree on north, so they gather easily; set the frames to Compass (Settings → Model → Robot model) to see how the rules cope with compasses that point the wrong way.".to_owned(),
        );
        return notes;
    }
    let overlapping = matches!(
        config.scheduler,
        SchedulerKind::Async | SchedulerKind::KAsync
    );
    let (name, bound, tight) = if config.compass_static {
        ("The rules for fixed compasses", 90.0, true)
    } else if overlapping {
        (
            "The rules for changing compasses, with robots acting on their own clocks,",
            30.0,
            false,
        )
    } else {
        (
            "The rules for changing compasses, with robots acting in rounds,",
            45.0,
            true,
        )
    };
    if phi >= bound {
        notes.push(format!(
            "Outside the model: {name} only guarantee that the two robots meet while every compass error stays below {bound}°{}. This run allows errors of up to {phi}°, so the robots may circle each other for a very long time, or never meet.",
            if tight {
                ", and no rules can guarantee more than that"
            } else {
                "; whether some other rules could do better is not known"
            }
        ));
    } else {
        notes.push(format!(
            "{name} guarantee that the two robots meet as long as every compass error stays below {bound}°, and this run allows at most {phi}°, so they will meet. Close to that bound it can take a very long time, because each correction overshoots a little."
        ));
    }
    if overlapping {
        notes.push("With robots acting on their own clocks, the two robots never terminate, even after they have met: a robot cannot be sure that the other has finished moving, so stopping could be a mistake. They stay together, and the run goes on until a limit.".to_owned());
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

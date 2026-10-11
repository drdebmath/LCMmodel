//! Go-To-Center: `Robot._go_to_center` / `_gtc_terminal` in `robot.py`.
//!
//! Ando, Oasa, Suzuki and Yamashita, "Distributed memoryless point convergence
//! algorithm for mobile robots with limited visibility" (IEEE Trans. Robotics
//! and Automation, 1999). A robot heads for the centre of the smallest circle
//! around the robots it sees, but never so far that it could lose sight of a
//! neighbour: for each visible neighbour `j`, its destination must stay inside
//! the disc of radius V/2 around the midpoint of the robot and `j`, and the
//! step is the farthest point of the way to the centre that is inside them all.
//!
//! The smallest circle is the randomized Welzl routine of `_sec_welzl_coords`,
//! called for every group of two or more robots as the Python does: it
//! shuffles the points and draws a random index at each step, so those random
//! numbers must be drawn in the same order for runs to match event by event.

use crate::sec::welzl_recur;
use lcm_core::{Algorithm, Decision, Look, OverlayUpdate, Point, Rng, RobotState, Scratch};

#[derive(Clone, Copy, Debug, Default)]
pub struct GoToCenter;

impl Algorithm for GoToCenter {
    fn key(&self) -> &'static str {
        "GoToCenter"
    }

    fn compute(&self, look: &Look<'_>, rng: &mut Rng, scratch: &mut Scratch) -> Decision {
        let eps = look.model.eps;
        let me = look.position;
        let view = look.view;

        scratch.points.clear();
        for (p, s) in view.pos.iter().zip(&view.state) {
            if *s != RobotState::Crash {
                scratch.points.push(*p);
            }
        }
        let n = scratch.points.len();
        if n <= 1 {
            // Nobody else to go towards; the Python returns `[None]` and leaves
            // the displayed circle alone, and `_gtc_terminal(None)` is true.
            return Decision {
                target: me,
                terminate: true,
                overlay: OverlayUpdate::Keep,
            };
        }

        // `_sec_welzl_coords`: shuffle a copy, then the randomized recursion.
        rng.shuffle(&mut scratch.points);
        scratch.points_b.clear();
        scratch.floats.clear();
        let sec = welzl_recur(
            &mut scratch.points,
            &mut scratch.points_b,
            n,
            rng,
            look.model.precision,
            &mut scratch.floats,
        );
        if sec.radius < 0.0 {
            return Decision {
                target: me,
                terminate: true,
                overlay: OverlayUpdate::Set(sec),
            };
        }
        // `_gtc_terminal`: converged once the visible robots have collapsed to a point.
        let terminate = sec.radius < eps;
        let overlay = OverlayUpdate::Set(sec);

        let gx = sec.center.x - me.x;
        let gy = sec.center.y - me.y;
        let goal_dist = libm::hypot(gx, gy);
        if goal_dist < eps {
            // Already at the centre.
            return Decision {
                target: me,
                terminate,
                overlay,
            };
        }

        let ux = gx / goal_dist;
        let uy = gy / goal_dist;
        let mut step = goal_dist;

        let visibility = look.model.visibility;
        if visibility != f64::INFINITY {
            let big_r = visibility / 2.0;
            for (p, s) in view.pos.iter().zip(&view.state) {
                if *s == RobotState::Crash {
                    continue;
                }
                let jx = p.x - me.x;
                let jy = p.y - me.y;
                let d = libm::hypot(jx, jy);
                if d <= 1e-12 || d > visibility {
                    continue; // itself, or not actually visible
                }
                // theta: the angle between the way to the centre and the way to `j`.
                let cos_t = ((ux * jx + uy * jy) / d).clamp(-1.0, 1.0);
                let sin_t = (1.0 - cos_t * cos_t).max(0.0).sqrt();
                let r0 = d / 2.0;
                // The farthest point of the ray that stays inside B(midpoint_ij, V/2).
                let across = r0 * sin_t;
                let limit_j = r0 * cos_t + (big_r * big_r - across * across).max(0.0).sqrt();
                step = step.min(limit_j);
            }
        }

        step = 0.0_f64.max(goal_dist.min(step));
        Decision {
            target: Point::new(me.x + ux * step, me.y + uy * step),
            terminate,
            overlay,
        }
    }
}

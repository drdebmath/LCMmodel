//! Center of gravity: `Robot._midpoint` / `_midpoint_terminal`.

use lcm_core::geom::dist;
use lcm_core::{Algorithm, Decision, Look, OverlayUpdate, Point, Rng, RobotState, Scratch};

#[derive(Clone, Copy, Debug, Default)]
pub struct Gathering;

impl Algorithm for Gathering {
    fn key(&self) -> &'static str {
        "Gathering"
    }

    #[allow(clippy::cast_precision_loss)]
    fn compute(&self, look: &Look<'_>, _: &mut Rng, _: &mut Scratch) -> Decision {
        let view = look.view;
        // The Python average includes crashed robots; its termination test does not.
        let (mut x, mut y) = (0.0, 0.0);
        for p in &view.pos {
            x += p.x;
            y += p.y;
        }
        let n = view.len() as f64;
        let target = Point::new(x / n, y / n);
        let terminate = view
            .pos
            .iter()
            .zip(&view.state)
            .filter(|(_, s)| **s != RobotState::Crash)
            .all(|(p, _)| dist(*p, target) <= look.model.eps);
        Decision {
            target,
            terminate,
            overlay: OverlayUpdate::Keep,
        }
    }

    #[allow(clippy::cast_precision_loss)]
    fn explain(&self, look: &Look<'_>, _: &mut Scratch) -> Option<String> {
        let view = look.view;
        let n = view.len();
        let (x, y) = view
            .pos
            .iter()
            .fold((0.0, 0.0), |(x, y), p| (x + p.x, y + p.y));
        let target = Point::new(x / n as f64, y / n as f64);
        Some(format!(
            "It sees {n} robot{} (itself included) and heads for their centre of gravity, {} units away.",
            if n == 1 { "" } else { "s" },
            crate::num(dist(look.position, target))
        ))
    }
}

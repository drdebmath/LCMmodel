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
}

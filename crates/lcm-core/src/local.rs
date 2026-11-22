//! Robots' private coordinate systems (disorientation).

use crate::geom::Point;
use crate::rng::Rng;

/// A robot's private coordinate system: itself at the origin, its own
/// rotation, unit length and handedness (no shared chirality).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LocalFrame {
    pub origin: Point,
    /// Radians the local axes are turned from the global ones.
    pub angle: f64,
    /// Local units per global unit.
    pub scale: f64,
    /// The local y axis points the other way (opposite handedness).
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

    /// A frame at `origin` with a random rotation and a unit length between
    /// 1/2 and 2 (log-uniform), mirrored at random when `chirality` is off.
    pub fn random(origin: Point, rng: &mut Rng, chirality: bool) -> Self {
        let angle = rng.uniform(0.0, 2.0 * std::f64::consts::PI);
        let scale = libm::exp2(rng.uniform(-1.0, 1.0));
        let reflect = !chirality && rng.random() < 0.5;
        Self {
            origin,
            angle,
            scale,
            reflect,
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
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::dist;

    #[test]
    fn a_random_frame_round_trips_and_keeps_its_origin_at_zero() {
        let mut rng = Rng::new(7);
        for _ in 0..100 {
            let f = LocalFrame::random(Point::new(3.0, -4.0), &mut rng, false);
            assert!((0.5..=2.0).contains(&f.scale));
            let p = Point::new(12.5, 7.25);
            assert!(dist(f.to_global(f.to_local(p)), p) < 1e-12);
            assert!(dist(f.to_local(f.origin), Point::new(0.0, 0.0)) < 1e-15);
            let d = dist(f.to_local(p), f.to_local(f.origin));
            assert!((d - f.scale * dist(p, f.origin)).abs() < 1e-9);
        }
        let mut rng = Rng::new(7);
        assert!((0..100).all(|_| !LocalFrame::random(Point::default(), &mut rng, true).reflect));
    }
}

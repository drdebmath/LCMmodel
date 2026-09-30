//! Plane geometry the core needs, with the exact arithmetic of `robot.py`.
//! Algorithm-specific geometry lives with the algorithms.

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    #[must_use]
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
}

/// A circle an algorithm computed, kept per robot for display (`robot.sec`).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Circle {
    pub center: Point,
    pub radius: f64,
}

impl Circle {
    #[must_use]
    pub const fn new(center: Point, radius: f64) -> Self {
        Self { center, radius }
    }
}

/// `math.dist(a, b)`.
#[must_use]
pub fn dist(a: Point, b: Point) -> f64 {
    libm::hypot(a.x - b.x, a.y - b.y)
}

/// `Robot._interpolate`: clamp `t` to [0, 1].
#[must_use]
pub fn interpolate(start: Point, end: Point, t: f64) -> Point {
    let t = t.clamp(0.0, 1.0);
    Point::new(
        start.x + t * (end.x - start.x),
        start.y + t * (end.y - start.y),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interpolation_is_clamped() {
        let (a, b) = (Point::new(0.0, 0.0), Point::new(10.0, -4.0));
        assert_eq!(interpolate(a, b, 0.5), Point::new(5.0, -2.0));
        assert_eq!(interpolate(a, b, 2.0), b);
        assert_eq!(interpolate(a, b, -1.0), a);
        assert_eq!(dist(a, Point::new(3.0, 4.0)), 5.0);
    }
}

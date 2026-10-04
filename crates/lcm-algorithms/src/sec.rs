//! Smallest Enclosing Circle: `Robot._smallest_enclosing_circle` / `_sec_terminal`.
//! Welzl 1991, mirroring `robot.py` exactly — including `generator.integers(0, n)`
//! at each recursive step and `round(dist, precision)` containment checks.

use lcm_core::geom::{dist, Circle, Point};
use lcm_core::{Algorithm, Decision, Look, OverlayUpdate, Rng, RobotState, Scratch};

#[derive(Clone, Copy, Debug, Default)]
pub struct Sec;

impl Algorithm for Sec {
    fn key(&self) -> &'static str {
        "SEC"
    }

    fn compute(&self, look: &Look<'_>, rng: &mut Rng, scratch: &mut Scratch) -> Decision {
        let eps = look.model.eps;
        let precision = look.model.precision;

        // Exclude crashed robots — mirrors Python's visible_robots_details filter.
        scratch.points.clear();
        for (p, s) in look.view.pos.iter().zip(&look.view.state) {
            if *s != RobotState::Crash {
                scratch.points.push(*p);
            }
        }

        let n = scratch.points.len();
        let sec = match n {
            0 => None,
            1 => Some(Circle::new(scratch.points[0], 0.0)),
            2 => Some(circle_from_two(scratch.points[0], scratch.points[1])),
            3 => sec_3pts(&scratch.points.clone(), precision),
            _ => {
                // Fast path A: all points collinear (Line+Edge pattern, or any degenerate
                // convergence where robots collapse onto a single diameter).
                // Fails after checking ~3 points for non-collinear configs — near-zero overhead.
                if let Some(c) = collinear_sec(&scratch.points, eps) {
                    return Decision {
                        target: closest_on_circle(c, look.position, eps),
                        terminate: scratch.points.iter().all(|p| (dist(*p, c.center) - c.radius).abs() < eps),
                        overlay: OverlayUpdate::Set(c),
                    };
                }

                // Fast path B: all points concyclic (Circle+Edge pattern terminates at step 0;
                // also fires for any configuration in its final convergence steps).
                // Fails after checking ~4 points for non-circle configs — near-zero overhead.
                if let Some(c) = concyclic_sec(&scratch.points, precision) {
                    return Decision {
                        target: closest_on_circle(c, look.position, eps),
                        terminate: scratch.points.iter().all(|p| (dist(*p, c.center) - c.radius).abs() < eps),
                        overlay: OverlayUpdate::Set(c),
                    };
                }

                // General path: Welzl (randomized, iterative, WASM-safe).
                rng.shuffle(&mut scratch.points);
                let n = scratch.points.len();
                scratch.points_b.clear();
                // Reuse scratch.floats as the call-stack storage (avoids a heap alloc per LOOK).
                scratch.floats.clear();
                let c = welzl_recur(
                    &mut scratch.points,
                    &mut scratch.points_b,
                    n,
                    rng,
                    precision,
                    &mut scratch.floats,
                );
                Some(c)
            }
        };

        let (target, overlay) = match sec {
            None => (look.position, OverlayUpdate::Clear),
            Some(c) if n == 1 => {
                // For n=1 Python moves the robot directly to that one point, not onto a circle.
                (scratch.points[0], OverlayUpdate::Set(c))
            }
            Some(c) => (
                closest_on_circle(c, look.position, eps),
                OverlayUpdate::Set(c),
            ),
        };

        // Termination: every non-crashed robot is on the circle boundary.
        // Uses _is_point_on_circle: abs(dist - radius) < eps.
        let terminate = sec.map_or(false, |c| {
            if n == 0 {
                return true;
            }
            scratch.points.iter().all(|p| (dist(*p, c.center) - c.radius).abs() < eps)
        });

        Decision {
            target,
            terminate,
            overlay,
        }
    }
}

// ── Fast path A: collinear SEC ────────────────────────────────────────────────
//
// Handles the Line+Edge pattern exactly, and any convergence where robots have
// collapsed onto a single diameter. The SEC of n collinear points is the circle
// whose diameter is the segment from the leftmost to rightmost point.
//
// Complexity: O(n) — checks every point against the line defined by pts[0..1].
// Fails fast: the first off-line point returns None in O(1) for non-Line configs.
fn collinear_sec(pts: &[Point], eps: f64) -> Option<Circle> {
    let a = pts[0];
    // Find a second distinct point to define the line direction.
    let b = pts.iter().copied().skip(1).find(|p| {
        libm::hypot(p.x - a.x, p.y - a.y) > eps
    });
    let b = match b {
        Some(p) => p,
        // All points coincide — not a collinear-line case (handled by n=1 arm).
        None => return None,
    };
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let len = libm::hypot(dx, dy);
    // Verify every point lies on the line a→b (signed distance < eps).
    for p in pts {
        let dist_to_line = ((p.x - a.x) * dy - (p.y - a.y) * dx).abs() / len;
        if dist_to_line > eps {
            return None; // fast fail
        }
    }
    // All collinear: SEC diameter = segment from min-projection to max-projection.
    let proj = |p: &Point| (p.x - a.x) * dx + (p.y - a.y) * dy;
    let min_p = pts.iter().min_by(|p, q| proj(p).partial_cmp(&proj(q)).unwrap()).unwrap();
    let max_p = pts.iter().max_by(|p, q| proj(p).partial_cmp(&proj(q)).unwrap()).unwrap();
    Some(circle_from_two(*min_p, *max_p))
}

// ── Fast path B: concyclic SEC ────────────────────────────────────────────────
//
// Handles the Circle+Edge pattern (all robots already on SEC → terminate at step 0)
// and any configuration in its final convergence steps (robots approaching the SEC
// boundary). The SEC of n concyclic points is exactly the common circle.
//
// Complexity: O(n) — picks 3 non-collinear points, computes their circumcircle,
// then verifies all n points lie on it (with Python-style precision rounding).
// Fails fast: the first off-circle point returns None; for random clouds this
// happens at point ~4, making the check near-free for non-circle configs.
fn concyclic_sec(pts: &[Point], precision: u8) -> Option<Circle> {
    let a = pts[0];
    let b = pts[1];
    // Find a third point that is not collinear with a and b.
    let c = pts[2..].iter().copied().find(|p| {
        let cross = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
        cross.abs() > 1e-6
    });
    let c = c?; // None means all points so far are collinear → not concyclic
    let candidate = circle_from_three(a, b, c);
    // Degenerate circumcircle (near-collinear triple) — not a valid concyclic circle.
    if !candidate.radius.is_finite() || candidate.radius < 1e-9 {
        return None;
    }
    // Every point must be exactly on the circle (same rounded distance as radius).
    let factor = 10.0_f64.powi(precision as i32);
    let r_rounded = (candidate.radius * factor).round();
    if pts.iter().all(|p| {
        (dist(*p, candidate.center) * factor).round() == r_rounded
    }) {
        Some(candidate)
    } else {
        None
    }
}

// ── n=3 direct path (mirrors _smallest_enclosing_circle's loop for 3 robots) ─

fn sec_3pts(pts: &[Point], precision: u8) -> Option<Circle> {
    // Try each pair as a diameter first.
    for i in 0..3 {
        for j in (i + 1)..3 {
            let c = circle_from_two(pts[i], pts[j]);
            if pts.iter().all(|p| contains_rounded(c, *p, precision)) {
                return Some(c);
            }
        }
    }
    // No diameter circle covers all — use circumscribed circle.
    let (a, b, c) = (pts[0], pts[1], pts[2]);
    if is_collinear(a, b, c) {
        Some(largest_diameter(a, b, c))
    } else {
        Some(circle_from_three(a, b, c))
    }
}

// ── Welzl — iterative with explicit stack ────────────────────────────────────
//
// The recursive formulation overflows the WASM call stack for large n (≥ ~3 000).
// This iterative version uses `scratch.floats` (passed in as `stack_buf`) as the
// call stack, avoiding any per-call heap allocation.
//
// Frame encoding in `stack_buf` (4 f64 values per frame, pushed/popped as a block):
//   [0.0, n_before_f64, pt.x, pt.y]  → AfterFirst
//   [1.0, 0.0,          pt.x, pt.y]  → AfterSecond
//
// The exact same `rng.index(n)` draws happen in the same order as the recursive
// version, so parity tests against Python fixtures continue to pass.

fn welzl_recur(
    pts: &mut [Point],
    r: &mut Vec<Point>,
    n: usize,
    rng: &mut Rng,
    precision: u8,
    stack_buf: &mut Vec<f64>,
) -> Circle {
    let mut n = n;
    let mut descending = true;
    let mut c = Circle::new(Point::new(0.0, 0.0), 0.0);

    loop {
        if descending {
            if n == 0 || r.len() == 3 {
                c = min_circle(r, precision);
                descending = false;
            } else {
                let idx = rng.index(n); // mirrors Python's generator.integers(0, n)
                pts.swap(idx, n - 1);
                let pt = pts[n - 1];
                // Push AfterFirst frame: [0.0, n_before, pt.x, pt.y]
                stack_buf.push(0.0);
                stack_buf.push(n as f64);
                stack_buf.push(pt.x);
                stack_buf.push(pt.y);
                n -= 1;
            }
        } else {
            let len = stack_buf.len();
            if len == 0 {
                return c;
            }
            let pt_y = stack_buf[len - 1];
            let pt_x = stack_buf[len - 2];
            let n_before_or_zero = stack_buf[len - 3];
            let discriminant = stack_buf[len - 4];
            stack_buf.truncate(len - 4);

            let pt = Point::new(pt_x, pt_y);

            if discriminant == 0.0 {
                // AfterFirst: check if c covers pt.
                let n_before = n_before_or_zero as usize;
                if contains_rounded(c, pt, precision) {
                    // c covers pt — propagate c upward (stay unwinding).
                } else {
                    // Needs second call with R ∪ {pt}.
                    r.push(pt);
                    // Push AfterSecond frame: [1.0, 0.0, pt.x, pt.y]
                    stack_buf.push(1.0);
                    stack_buf.push(0.0);
                    stack_buf.push(pt.x);
                    stack_buf.push(pt.y);
                    n = n_before - 1;
                    descending = true;
                }
            } else {
                // AfterSecond: pop pt from r and propagate c upward.
                r.pop();
            }
        }
    }
}

// ── min_circle (mirrors Python _min_circle) ───────────────────────────────────

fn min_circle(pts: &[Point], precision: u8) -> Circle {
    match pts.len() {
        0 => Circle::new(Point::new(0.0, 0.0), 0.0),
        1 => Circle::new(pts[0], 0.0),
        2 => circle_from_two(pts[0], pts[1]),
        _ => {
            // Try each consecutive pair as a diameter.
            for i in 0..3 {
                let c = circle_from_two(pts[i], pts[(i + 1) % 3]);
                if contains_rounded(c, pts[(i + 2) % 3], precision) {
                    return c;
                }
            }
            // All three points are needed.
            let (a, b, c) = (pts[0], pts[1], pts[2]);
            if is_collinear(a, b, c) {
                largest_diameter(a, b, c)
            } else {
                circle_from_three(a, b, c)
            }
        }
    }
}

// ── geometry helpers ──────────────────────────────────────────────────────────

/// Containment check with Python-style rounding: `round(dist, p) <= round(r, p)`.
fn contains_rounded(c: Circle, p: Point, precision: u8) -> bool {
    let factor = 10.0_f64.powi(precision as i32);
    let d_rounded = (dist(p, c.center) * factor).round();
    let r_rounded = (c.radius * factor).round();
    d_rounded <= r_rounded
}

fn circle_from_two(a: Point, b: Point) -> Circle {
    Circle::new(
        Point::new((a.x + b.x) * 0.5, (a.y + b.y) * 0.5),
        dist(a, b) * 0.5,
    )
}

fn circle_from_three(a: Point, b: Point, c: Point) -> Circle {
    // Mirrors Python's _circle_from_three exactly.
    let big_a = b.x - a.x;
    let big_b = b.y - a.y;
    let big_c = c.x - a.x;
    let big_d = c.y - a.y;
    let big_e = big_a * (a.x + b.x) + big_b * (a.y + b.y);
    let big_f = big_c * (a.x + c.x) + big_d * (a.y + c.y);
    let big_g = 2.0 * (big_a * (c.y - b.y) - big_b * (c.x - b.x));
    if big_g.abs() < 1e-9 {
        return largest_diameter(a, b, c);
    }
    let cx = (big_d * big_e - big_b * big_f) / big_g;
    let cy = (big_a * big_f - big_c * big_e) / big_g;
    let center = Point::new(cx, cy);
    Circle::new(center, dist(center, a))
}

fn largest_diameter(a: Point, b: Point, c: Point) -> Circle {
    let dab = dist(a, b);
    let dac = dist(a, c);
    let dbc = dist(b, c);
    if dab >= dac && dab >= dbc {
        circle_from_two(a, b)
    } else if dac >= dbc {
        circle_from_two(a, c)
    } else {
        circle_from_two(b, c)
    }
}

fn is_collinear(a: Point, b: Point, c: Point) -> bool {
    (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y)).abs() < 1e-9
}

fn closest_on_circle(c: Circle, p: Point, eps: f64) -> Point {
    // Mirrors Python's _closest_point_on_circle.
    let vx = p.x - c.center.x;
    let vy = p.y - c.center.y;
    let d = libm::hypot(vx, vy);
    if d < 1e-9 {
        return Point::new(c.center.x + c.radius, c.center.y);
    }
    let _ = eps; // not used — Python also doesn't use eps here
    let s = c.radius / d;
    Point::new(c.center.x + vx * s, c.center.y + vy * s)
}

// ── Unit tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::{FRAC_1_SQRT_2, SQRT_2, TAU};

    fn pt(x: f64, y: f64) -> Point {
        Point::new(x, y)
    }

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    fn pt_close(a: Point, b: Point) -> bool {
        close(a.x, b.x) && close(a.y, b.y)
    }

    // ── circle_from_two ──────────────────────────────────────────────────────

    #[test]
    fn circle_from_two_axis_aligned() {
        let c = circle_from_two(pt(-3.0, 0.0), pt(3.0, 0.0));
        assert!(pt_close(c.center, pt(0.0, 0.0)), "center {:?}", c.center);
        assert!(close(c.radius, 3.0), "radius {}", c.radius);
    }

    #[test]
    fn circle_from_two_diagonal() {
        let c = circle_from_two(pt(0.0, 0.0), pt(4.0, 4.0));
        assert!(pt_close(c.center, pt(2.0, 2.0)));
        assert!(close(c.radius, SQRT_2 * 2.0));
    }

    // ── circle_from_three ────────────────────────────────────────────────────

    #[test]
    fn circle_from_three_unit_circle() {
        // Three points known to lie on the unit circle.
        let c = circle_from_three(pt(1.0, 0.0), pt(-1.0, 0.0), pt(0.0, 1.0));
        assert!(pt_close(c.center, pt(0.0, 0.0)));
        assert!(close(c.radius, 1.0));
    }

    #[test]
    fn circle_from_three_equilateral_triangle_at_radius_two() {
        let r = 2.0_f64;
        let a = pt(r, 0.0);
        let b = pt(r * libm::cos(TAU / 3.0), r * libm::sin(TAU / 3.0));
        let c_pt = pt(r * libm::cos(2.0 * TAU / 3.0), r * libm::sin(2.0 * TAU / 3.0));
        let c = circle_from_three(a, b, c_pt);
        assert!(pt_close(c.center, pt(0.0, 0.0)));
        assert!(close(c.radius, r));
    }

    #[test]
    fn circle_from_three_collinear_falls_back_to_largest_diameter() {
        // G ≈ 0 for collinear inputs triggers the largest-diameter fallback.
        let c = circle_from_three(pt(-5.0, 0.0), pt(0.0, 0.0), pt(3.0, 0.0));
        // Farthest pair: (-5,0)→(3,0), diameter=8.
        assert!(pt_close(c.center, pt(-1.0, 0.0)));
        assert!(close(c.radius, 4.0));
    }

    // ── contains_rounded ─────────────────────────────────────────────────────

    #[test]
    fn contains_rounded_on_boundary_is_inside() {
        let c = Circle::new(pt(0.0, 0.0), 1.0);
        assert!(contains_rounded(c, pt(1.0, 0.0), 5));
    }

    #[test]
    fn contains_rounded_just_inside_rounds_to_boundary() {
        let c = Circle::new(pt(0.0, 0.0), 1.0);
        // Distance 0.999999 rounds to 1.00000 at precision=5, which equals radius.
        assert!(contains_rounded(c, pt(0.999_999, 0.0), 5));
    }

    #[test]
    fn contains_rounded_outside_is_rejected() {
        let c = Circle::new(pt(0.0, 0.0), 1.0);
        // Distance 1.000_015 rounds to 1.00002 > 1.00000 at precision=5.
        assert!(!contains_rounded(c, pt(1.000_015, 0.0), 5));
    }

    #[test]
    fn contains_rounded_precision_controls_tolerance() {
        let c = Circle::new(pt(0.0, 0.0), 1.0);
        let p = pt(1.000_005, 0.0);
        // At precision=5: rounds to 1.00001 > 1.00000 → outside.
        assert!(!contains_rounded(c, p, 5));
        // At precision=3: rounds to 1.0 == 1.0 → inside.
        assert!(contains_rounded(c, p, 3));
    }

    // ── closest_on_circle ────────────────────────────────────────────────────

    #[test]
    fn closest_on_circle_outside_projects_inward() {
        let c = Circle::new(pt(0.0, 0.0), 3.0);
        let t = closest_on_circle(c, pt(5.0, 0.0), 1e-9);
        assert!(pt_close(t, pt(3.0, 0.0)));
    }

    #[test]
    fn closest_on_circle_inside_projects_outward() {
        let c = Circle::new(pt(0.0, 0.0), 3.0);
        let t = closest_on_circle(c, pt(1.0, 0.0), 1e-9);
        assert!(pt_close(t, pt(3.0, 0.0)));
    }

    #[test]
    fn closest_on_circle_at_center_defaults_to_right() {
        // Robot exactly at center: degenerate, defaults to (center.x + r, center.y).
        let c = Circle::new(pt(2.0, 3.0), 4.0);
        let t = closest_on_circle(c, pt(2.0, 3.0), 1e-9);
        assert!(pt_close(t, pt(6.0, 3.0)));
    }

    #[test]
    fn closest_on_circle_diagonal_direction() {
        let c = Circle::new(pt(0.0, 0.0), 1.0);
        let t = closest_on_circle(c, pt(2.0, 2.0), 1e-9);
        // Direction (1,1)/√2 scaled to radius 1.
        assert!(pt_close(t, pt(FRAC_1_SQRT_2, FRAC_1_SQRT_2)));
    }

    // ── is_collinear ─────────────────────────────────────────────────────────

    #[test]
    fn is_collinear_horizontal_line() {
        assert!(is_collinear(pt(-2.0, 0.0), pt(0.0, 0.0), pt(3.0, 0.0)));
    }

    #[test]
    fn is_collinear_diagonal_line() {
        assert!(is_collinear(pt(-1.0, -1.0), pt(0.0, 0.0), pt(2.0, 2.0)));
    }

    #[test]
    fn is_collinear_false_when_off_line() {
        assert!(!is_collinear(pt(0.0, 0.0), pt(1.0, 0.0), pt(0.0, 1.0)));
    }

    // ── largest_diameter ─────────────────────────────────────────────────────

    #[test]
    fn largest_diameter_picks_the_farthest_pair() {
        // Distances: ab=3, ac=5, bc=2. Pair a-c wins.
        let c = largest_diameter(pt(0.0, 0.0), pt(3.0, 0.0), pt(5.0, 0.0));
        assert!(pt_close(c.center, pt(2.5, 0.0)));
        assert!(close(c.radius, 2.5));
    }

    // ── min_circle ───────────────────────────────────────────────────────────

    #[test]
    fn min_circle_zero_points_is_origin() {
        let c = min_circle(&[], 5);
        assert!(pt_close(c.center, pt(0.0, 0.0)));
        assert!(close(c.radius, 0.0));
    }

    #[test]
    fn min_circle_one_point_is_degenerate() {
        let c = min_circle(&[pt(7.0, -3.0)], 5);
        assert!(pt_close(c.center, pt(7.0, -3.0)));
        assert!(close(c.radius, 0.0));
    }

    #[test]
    fn min_circle_two_points_is_diameter() {
        let c = min_circle(&[pt(-2.0, 0.0), pt(2.0, 0.0)], 5);
        assert!(pt_close(c.center, pt(0.0, 0.0)));
        assert!(close(c.radius, 2.0));
    }

    #[test]
    fn min_circle_three_points_diameter_dominates() {
        // (0,0)→(6,0) as diameter covers (3,0); no circumcircle needed.
        let c = min_circle(&[pt(0.0, 0.0), pt(6.0, 0.0), pt(3.0, 0.0)], 5);
        assert!(pt_close(c.center, pt(3.0, 0.0)));
        assert!(close(c.radius, 3.0));
    }

    #[test]
    fn min_circle_three_points_circumcircle() {
        // Three points on the unit circle — no single diameter covers the third.
        let c = min_circle(&[pt(1.0, 0.0), pt(-1.0, 0.0), pt(0.0, 1.0)], 5);
        assert!(pt_close(c.center, pt(0.0, 0.0)));
        assert!(close(c.radius, 1.0));
    }

    #[test]
    fn min_circle_three_collinear_is_largest_diameter() {
        let c = min_circle(&[pt(-4.0, 0.0), pt(0.0, 0.0), pt(2.0, 0.0)], 5);
        // Farthest pair: (-4,0)→(2,0).
        assert!(pt_close(c.center, pt(-1.0, 0.0)));
        assert!(close(c.radius, 3.0));
    }

    // ── sec_3pts ─────────────────────────────────────────────────────────────

    #[test]
    fn sec_3pts_diameter_when_third_inside_pair() {
        // (1,0) lies inside the diameter circle of (-3,0)→(3,0).
        let c = sec_3pts(&[pt(-3.0, 0.0), pt(3.0, 0.0), pt(1.0, 0.0)], 5).unwrap();
        assert!(pt_close(c.center, pt(0.0, 0.0)));
        assert!(close(c.radius, 3.0));
    }

    #[test]
    fn sec_3pts_circumcircle_when_all_outside_any_diameter() {
        let c = sec_3pts(&[pt(1.0, 0.0), pt(-1.0, 0.0), pt(0.0, 1.0)], 5).unwrap();
        assert!(pt_close(c.center, pt(0.0, 0.0)));
        assert!(close(c.radius, 1.0));
    }

    #[test]
    fn sec_3pts_collinear_is_largest_diameter() {
        let c = sec_3pts(&[pt(-5.0, 0.0), pt(0.0, 0.0), pt(3.0, 0.0)], 5).unwrap();
        assert!(pt_close(c.center, pt(-1.0, 0.0)));
        assert!(close(c.radius, 4.0));
    }

    // ── collinear_sec ────────────────────────────────────────────────────────

    #[test]
    fn collinear_sec_horizontal_line() {
        let pts = vec![pt(-3.0, 0.0), pt(0.0, 0.0), pt(2.0, 0.0), pt(-1.0, 0.0)];
        let c = collinear_sec(&pts, 1e-9).unwrap();
        // Farthest pair: (-3,0)→(2,0).
        assert!(pt_close(c.center, pt(-0.5, 0.0)));
        assert!(close(c.radius, 2.5));
    }

    #[test]
    fn collinear_sec_diagonal_line() {
        let pts = vec![pt(-1.0, -1.0), pt(0.0, 0.0), pt(1.0, 1.0)];
        let c = collinear_sec(&pts, 1e-9).unwrap();
        assert!(pt_close(c.center, pt(0.0, 0.0)));
        assert!(close(c.radius, SQRT_2));
    }

    #[test]
    fn collinear_sec_none_for_non_collinear() {
        // Third point is off the line — must return None quickly.
        let pts = vec![pt(0.0, 0.0), pt(1.0, 0.0), pt(0.0, 1.0), pt(2.0, 0.0)];
        assert!(collinear_sec(&pts, 1e-9).is_none());
    }

    #[test]
    fn collinear_sec_none_when_all_coincident() {
        // No second distinct point exists — cannot define a line.
        let pts = vec![pt(5.0, 5.0), pt(5.0, 5.0), pt(5.0, 5.0), pt(5.0, 5.0)];
        assert!(collinear_sec(&pts, 1e-9).is_none());
    }

    #[test]
    fn collinear_sec_single_outlier_causes_rejection() {
        // Four on line, one off — must fail.
        let pts = vec![
            pt(-2.0, 0.0), pt(-1.0, 0.0), pt(0.0, 0.0),
            pt(1.0, 0.0),  pt(0.5, 1.0),  // this one is off-line
        ];
        assert!(collinear_sec(&pts, 1e-9).is_none());
    }

    // ── concyclic_sec ────────────────────────────────────────────────────────

    #[test]
    fn concyclic_sec_four_cardinal_points_on_unit_circle() {
        let pts = vec![pt(1.0, 0.0), pt(0.0, 1.0), pt(-1.0, 0.0), pt(0.0, -1.0)];
        let c = concyclic_sec(&pts, 5).unwrap();
        assert!(pt_close(c.center, pt(0.0, 0.0)));
        assert!(close(c.radius, 1.0));
    }

    #[test]
    fn concyclic_sec_eight_evenly_spaced_mirrors_circle_edge_start() {
        // Exactly the Circle+Edge+Even pattern: 8 robots at radius 300.
        let r = 300.0_f64;
        let pts: Vec<Point> = (0..8)
            .map(|k| {
                let a = TAU * k as f64 / 8.0;
                pt(r * libm::cos(a), r * libm::sin(a))
            })
            .collect();
        let c = concyclic_sec(&pts, 5).unwrap();
        assert!(pt_close(c.center, pt(0.0, 0.0)));
        assert!(close(c.radius, r));
    }

    #[test]
    fn concyclic_sec_none_for_non_concyclic() {
        // Last point is at a different radius — not on the common circle.
        let pts = vec![pt(1.0, 0.0), pt(0.0, 1.0), pt(-1.0, 0.0), pt(0.0, -2.0)];
        assert!(concyclic_sec(&pts, 5).is_none());
    }

    #[test]
    fn concyclic_sec_none_for_collinear_input() {
        // All collinear — no non-collinear triple can be found.
        let pts = vec![pt(-1.0, 0.0), pt(0.0, 0.0), pt(1.0, 0.0), pt(2.0, 0.0)];
        assert!(concyclic_sec(&pts, 5).is_none());
    }

    #[test]
    fn concyclic_sec_none_when_one_point_is_interior() {
        // Three on unit circle, one inside — not concyclic.
        let pts = vec![pt(1.0, 0.0), pt(-1.0, 0.0), pt(0.0, 1.0), pt(0.0, 0.0)];
        assert!(concyclic_sec(&pts, 5).is_none());
    }
}

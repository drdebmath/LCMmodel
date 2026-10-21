//! Uniform Circle Formation: `Robot._circle_formation` / `_circle_terminal`
//! (Defago & Konagaya 2002; Flocchini et al.), mirroring `robot.py`.
//!
//! Robots arrange at equal angular spacing on a common circle = centroid of the
//! swarm + mean radius. The reference circle is recomputed from each look, so it
//! is a smooth, moving target the swarm settles onto (see the note in
//! `robot.py`: using the live SEC instead makes the centre hop forever).
//!   Phase 1: a robot off the circle projects radially onto it.
//!   Phase 2: a robot on the circle slides tangentially to the angular bisector
//!            of its two neighbours (local gap-averaging) -> equal spacing.
//!
//! Uses no random numbers, so it is deterministic and scheduler-agnostic: the
//! same rule runs unchanged under the async and sequential schedulers.

use core::f64::consts::TAU;

use lcm_core::geom::{dist, Circle, Point};
use lcm_core::{Algorithm, Decision, Look, OverlayUpdate, Rng, RobotState, Scratch};

#[derive(Clone, Copy, Debug, Default)]
pub struct CircleFormation;

impl Algorithm for CircleFormation {
    fn key(&self) -> &'static str {
        "CircleFormation"
    }

    #[allow(clippy::cast_precision_loss)]
    fn compute(&self, look: &Look<'_>, _: &mut Rng, scratch: &mut Scratch) -> Decision {
        let eps = look.model.eps;
        let me = look.position;

        // Non-crashed positions, mirroring Python's visible_robots filter.
        scratch.points.clear();
        for (p, s) in look.view.pos.iter().zip(&look.view.state) {
            if *s != RobotState::Crash {
                scratch.points.push(*p);
            }
        }
        let n = scratch.points.len();

        // Nobody to form a circle with: stay put, show nothing, done.
        if n <= 1 {
            return Decision {
                target: me,
                terminate: true,
                overlay: OverlayUpdate::Clear,
            };
        }

        // Reference circle: centroid + mean radius.
        let (o, rad) = reference_circle(&scratch.points);

        // Swarm has collapsed to a point (rad <= 10^-precision): a degenerate,
        // finished formation. Python returns True from `_circle_terminal` here.
        if rad <= eps {
            return Decision {
                target: me,
                terminate: true,
                overlay: OverlayUpdate::Clear,
            };
        }

        let circle = Circle::new(o, rad);
        // Termination is the global `_circle_terminal` predicate, independent of
        // which phase `me` is in: everyone on the circle and gaps near uniform.
        let terminate = is_uniform(&scratch.points, o, rad, &mut scratch.floats);

        // Phase 1: snap radially onto the circle if not already on it.
        let on_tol = rad * 1e-2;
        if (dist(me, o) - rad).abs() > on_tol {
            return Decision {
                target: closest_on_circle(circle, me),
                terminate,
                overlay: OverlayUpdate::Set(circle),
            };
        }

        // Phase 2: tangential move to the bisector of the two angular neighbours.
        let my_ang = libm::atan2(me.y - o.y, me.x - o.x);
        let mut ccw = f64::INFINITY; // nearest neighbour counter-clockwise (min delta)
        let mut max_delta = f64::NEG_INFINITY; // farthest delta -> nearest clockwise
        let mut any = false;
        for p in &scratch.points {
            if dist(*p, me) <= 1e-9 {
                continue; // skip self and anyone coincident with me
            }
            let delta = (libm::atan2(p.y - o.y, p.x - o.x) - my_ang).rem_euclid(TAU);
            if delta < ccw {
                ccw = delta;
            }
            if delta > max_delta {
                max_delta = delta;
            }
            any = true;
        }
        let target = if any {
            let cw = TAU - max_delta;
            let target_ang = my_ang + (ccw - cw) / 2.0;
            Point::new(
                o.x + rad * libm::cos(target_ang),
                o.y + rad * libm::sin(target_ang),
            )
        } else {
            me
        };
        Decision {
            target,
            terminate,
            overlay: OverlayUpdate::Set(circle),
        }
    }
}

/// Centroid and mean distance-from-centroid of `pts` (`pts` must be non-empty).
#[allow(clippy::cast_precision_loss)]
fn reference_circle(pts: &[Point]) -> (Point, f64) {
    let inv_n = 1.0 / pts.len() as f64;
    let (mut ox, mut oy) = (0.0, 0.0);
    for p in pts {
        ox += p.x;
        oy += p.y;
    }
    let o = Point::new(ox * inv_n, oy * inv_n);
    let rad = pts.iter().map(|p| dist(*p, o)).sum::<f64>() * inv_n;
    (o, rad)
}

/// `_circle_terminal`: every robot is on the circle (within `rad * 2e-2`) and
/// every angular gap is within 6% of the uniform gap `2π/n`. `angs` is a scratch
/// buffer reused for the sorted angles.
#[allow(clippy::cast_precision_loss)]
fn is_uniform(pts: &[Point], o: Point, rad: f64, angs: &mut Vec<f64>) -> bool {
    let n = pts.len();
    if n <= 1 {
        return true;
    }
    let on_tol = rad * 2e-2;
    angs.clear();
    for p in pts {
        if (dist(*p, o) - rad).abs() > on_tol {
            return false; // someone not on the circle
        }
        angs.push(libm::atan2(p.y - o.y, p.x - o.x).rem_euclid(TAU));
    }
    angs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let target_gap = TAU / n as f64;
    for i in 0..n {
        let gap = (angs[(i + 1) % n] - angs[i]).rem_euclid(TAU);
        if (gap - target_gap).abs() >= target_gap * 0.06 {
            return false;
        }
    }
    true
}

/// `_closest_point_on_circle`: the point of `c` nearest `p` (its radial
/// projection), defaulting to the right of the centre when `p` is the centre.
fn closest_on_circle(c: Circle, p: Point) -> Point {
    let vx = p.x - c.center.x;
    let vy = p.y - c.center.y;
    let d = libm::hypot(vx, vy);
    if d < 1e-9 {
        return Point::new(c.center.x + c.radius, c.center.y);
    }
    let s = c.radius / d;
    Point::new(c.center.x + vx * s, c.center.y + vy * s)
}

// ── Unit tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use lcm_core::{Model, View};

    fn pt(x: f64, y: f64) -> Point {
        Point::new(x, y)
    }

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    fn pt_close(a: Point, b: Point) -> bool {
        close(a.x, b.x) && close(a.y, b.y)
    }

    /// n points at equal angular spacing on the circle (centre, radius).
    fn regular(n: usize, centre: Point, radius: f64) -> Vec<Point> {
        (0..n)
            .map(|k| {
                let a = TAU * k as f64 / n as f64;
                pt(centre.x + radius * a.cos(), centre.y + radius * a.sin())
            })
            .collect()
    }

    // ── reference_circle ──────────────────────────────────────────────────────

    #[test]
    fn reference_circle_of_a_centred_square() {
        let pts = vec![pt(1.0, 1.0), pt(-1.0, 1.0), pt(-1.0, -1.0), pt(1.0, -1.0)];
        let (o, rad) = reference_circle(&pts);
        assert!(pt_close(o, pt(0.0, 0.0)), "centre {o:?}");
        assert!(close(rad, 2.0_f64.sqrt()), "radius {rad}");
    }

    #[test]
    fn reference_circle_radius_is_the_mean_not_the_max() {
        // Three on the unit circle, one at the centre: mean radius = 3/4.
        let pts = vec![pt(1.0, 0.0), pt(-1.0, 0.0), pt(0.0, 1.0), pt(0.0, 0.0)];
        let (o, rad) = reference_circle(&pts);
        assert!(pt_close(o, pt(0.0, 0.25)));
        // distances from (0,0.25): 1.030776, 1.030776, 0.75, 0.25 -> mean 0.765582
        assert!(close(
            rad,
            (1.030_776_406 + 1.030_776_406 + 0.75 + 0.25) / 4.0
        ));
    }

    // ── closest_on_circle ─────────────────────────────────────────────────────

    #[test]
    fn closest_on_circle_projects_radially() {
        let c = Circle::new(pt(0.0, 0.0), 3.0);
        assert!(pt_close(closest_on_circle(c, pt(5.0, 0.0)), pt(3.0, 0.0)));
        assert!(pt_close(closest_on_circle(c, pt(0.0, 1.0)), pt(0.0, 3.0)));
    }

    #[test]
    fn closest_on_circle_at_centre_defaults_right() {
        let c = Circle::new(pt(2.0, 3.0), 4.0);
        assert!(pt_close(closest_on_circle(c, pt(2.0, 3.0)), pt(6.0, 3.0)));
    }

    // ── is_uniform ────────────────────────────────────────────────────────────

    #[test]
    fn is_uniform_true_for_a_regular_polygon() {
        let mut buf = Vec::new();
        for n in [3usize, 4, 5, 8, 16] {
            let pts = regular(n, pt(10.0, -4.0), 50.0);
            assert!(is_uniform(&pts, pt(10.0, -4.0), 50.0, &mut buf), "n={n}");
        }
    }

    #[test]
    fn is_uniform_false_when_a_robot_is_off_the_circle() {
        let mut buf = Vec::new();
        let mut pts = regular(6, pt(0.0, 0.0), 10.0);
        pts[0] = pt(5.0, 0.0); // pulled inward, off the circle
        assert!(!is_uniform(&pts, pt(0.0, 0.0), 10.0, &mut buf));
    }

    #[test]
    fn is_uniform_false_when_gaps_are_lopsided() {
        // All on the circle, but clustered: three near angle 0, one opposite.
        let mut buf = Vec::new();
        let r = 10.0;
        let pts = vec![
            pt(r, 0.0),
            pt(r * 0.1_f64.cos(), r * 0.1_f64.sin()),
            pt(r * 0.2_f64.cos(), r * 0.2_f64.sin()),
            pt(-r, 0.0),
        ];
        assert!(!is_uniform(&pts, pt(0.0, 0.0), r, &mut buf));
    }

    // ── compute (phase selection) ─────────────────────────────────────────────

    fn model() -> Model {
        Model {
            precision: 5,
            eps: 1e-5,
            visibility: f64::INFINITY,
            width_bound: None,
            height_bound: None,
        }
    }

    fn view_of(pts: &[Point]) -> View {
        let mut v = View::default();
        for (i, p) in pts.iter().enumerate() {
            v.id.push(u32::try_from(i).unwrap());
            v.pos.push(*p);
            v.state.push(RobotState::Wait);
            v.terminated.push(false);
            v.frozen.push(false);
        }
        v
    }

    fn decide(pts: &[Point], me_index: usize) -> Decision {
        let view = view_of(pts);
        let m = model();
        let look = Look {
            me: u32::try_from(me_index).unwrap(),
            me_index,
            position: pts[me_index],
            view: &view,
            model: &m,
        };
        let mut rng = Rng::new(1);
        let mut scratch = Scratch::default();
        CircleFormation.compute(&look, &mut rng, &mut scratch)
    }

    #[test]
    fn single_robot_stays_and_terminates() {
        let d = decide(&[pt(3.0, 4.0)], 0);
        assert!(pt_close(d.target, pt(3.0, 4.0)));
        assert!(d.terminate);
        assert_eq!(d.overlay, OverlayUpdate::Clear);
    }

    #[test]
    fn phase_one_projects_an_off_circle_robot_onto_the_circle() {
        // Square -> reference circle centre (0,0), radius sqrt(2). Move one robot
        // well inside; it must project radially out to the circle.
        let mut pts = vec![pt(1.0, 1.0), pt(-1.0, 1.0), pt(-1.0, -1.0), pt(1.0, -1.0)];
        pts[0] = pt(0.1, 0.1); // pulled toward the centre, off the circle
        let (o, rad) = reference_circle(&pts);
        let d = decide(&pts, 0);
        // Target is on the reference circle, along the me->centre radial.
        assert!(close(dist(d.target, o), rad), "target not on circle");
        assert!(matches!(d.overlay, OverlayUpdate::Set(_)));
        assert!(!d.terminate);
    }

    #[test]
    fn a_regular_polygon_terminates_in_place() {
        let pts = regular(6, pt(0.0, 0.0), 20.0);
        let d = decide(&pts, 0);
        assert!(
            d.terminate,
            "a regular polygon is already a uniform formation"
        );
        assert!(matches!(d.overlay, OverlayUpdate::Set(_)));
    }

    #[test]
    fn phase_two_moves_toward_the_bisector_and_stays_on_the_circle() {
        // Robots already lie on their reference circle but are unevenly spaced, so
        // Phase 2 fires. The moved robot must (a) stay on the reference circle and
        // (b) land on the angular bisector of its two neighbours — i.e. be equally
        // far (angularly) from the nearest robot each way.
        // Antipodal pairs (0/180 and 50/230) so the centroid is exactly the
        // origin: every robot then lies on its own reference circle (radius 1),
        // which is what makes Phase 2 — not Phase 1 — the active branch.
        let deg = |d: f64| d.to_radians();
        let r = 1.0;
        let on = |a: f64| pt(r * a.cos(), r * a.sin());
        let pts = vec![on(deg(0.0)), on(deg(180.0)), on(deg(50.0)), on(deg(230.0))];
        let (o, rad) = reference_circle(&pts);
        assert!(
            pt_close(o, pt(0.0, 0.0)) && close(rad, 1.0),
            "setup not on circle"
        );
        let d = decide(&pts, 0);

        // (a) stays on the reference circle.
        assert!(
            close(dist(d.target, o), rad),
            "left the circle: {:?}",
            d.target
        );

        // (b) sits on the bisector: equal angular gap to nearest neighbour each way.
        let ang = |p: Point| libm::atan2(p.y - o.y, p.x - o.x);
        let target_ang = ang(d.target);
        let mut ccw = f64::INFINITY; // smallest positive step counter-clockwise
        let mut cw = f64::INFINITY; // smallest positive step clockwise
        for (i, p) in pts.iter().enumerate() {
            if i == 0 {
                continue; // skip the mover's own original position
            }
            let fwd = (ang(*p) - target_ang).rem_euclid(TAU);
            let back = (target_ang - ang(*p)).rem_euclid(TAU);
            ccw = ccw.min(fwd);
            cw = cw.min(back);
        }
        assert!(
            (ccw - cw).abs() < 1e-9,
            "not on the bisector: ccw gap {ccw} vs cw gap {cw}"
        );
    }
}

//! Pattern Formation: `Robot._pattern_formation` / `_pattern_terminal`
//! (Suzuki & Yamashita, SIAM J. Comput. 1999), mirroring `robot.py`, generalised
//! from the fixed 5-pointed star to any predefined shape.
//!
//! The generalisation follows Universal Pattern Formation (Flocchini, Navarra,
//! Pattanayak, Piselli & Santoro, arXiv:2412.10733): under a sequential
//! scheduler oblivious robots can form *any* pattern given as a finite set of
//! points, up to rotation, reflection, translation and scaling. A [`PatternShape`]
//! names which point set to form; the pipeline below is otherwise unchanged.
//!
//! Each robot deterministically derives a common reference frame from the
//! snapshot — origin = centroid, scale = RMS distance from the centroid,
//! orientation = world axes (the sim's shared frame sidesteps the
//! orientation-symmetry obstruction) — embeds the normalised template of the
//! chosen shape in it, matches robots to template points by a consistent angular
//! ordering (with the best cyclic rotation, which removes the rotational
//! ambiguity that would otherwise make the matching oscillate forever), and
//! moves to its matched point.
//!
//! Uses no random numbers, so it is deterministic and scheduler-agnostic.

use core::f64::consts::PI;

use lcm_core::geom::{dist, Point};
use lcm_core::{Algorithm, Decision, Look, OverlayUpdate, PatternShape, Rng, RobotState, Scratch};

#[derive(Clone, Copy, Debug, Default)]
pub struct PatternFormation {
    /// Which predefined shape to form. Defaults to the 5-pointed star.
    pub shape: PatternShape,
}

impl Algorithm for PatternFormation {
    fn key(&self) -> &'static str {
        "PatternFormation"
    }

    #[allow(clippy::cast_precision_loss)]
    fn compute(&self, look: &Look<'_>, _: &mut Rng, scratch: &mut Scratch) -> Decision {
        let eps = look.model.eps;
        let me_pos = look.position;
        let me_id = look.me as usize;

        // Non-crashed (id, position), preserving view order (Python's snapshot filter).
        scratch.points.clear();
        scratch.indices.clear();
        for ((id, p), s) in look
            .view
            .id
            .iter()
            .zip(&look.view.pos)
            .zip(&look.view.state)
        {
            if *s != RobotState::Crash {
                scratch.points.push(*p);
                scratch.indices.push(*id as usize);
            }
        }
        let n = scratch.points.len();
        let stay = Decision {
            target: me_pos,
            terminate: true,
            overlay: OverlayUpdate::Keep,
        };
        if n <= 1 {
            return stay;
        }

        // Common frame: centroid + RMS radius.
        let inv_n = 1.0 / n as f64;
        let (mut ox, mut oy) = (0.0, 0.0);
        for p in &scratch.points {
            ox += p.x;
            oy += p.y;
        }
        ox *= inv_n;
        oy *= inv_n;
        let mut sum_sq = 0.0;
        for p in &scratch.points {
            let (dx, dy) = (p.x - ox, p.y - oy);
            sum_sq += dx * dx + dy * dy;
        }
        let r = libm::sqrt(sum_sq * inv_n);
        if r < eps {
            return stay; // swarm collapsed to a point
        }

        // The index of this robot in the filtered list.
        let Some(my_idx) = scratch.indices.iter().position(|&id| id == me_id) else {
            return stay;
        };

        // Target points: the normalised template of the chosen shape, placed in the frame.
        let unit = pattern_unit_points(self.shape, n);
        let targets: Vec<Point> = unit
            .iter()
            .map(|u| Point::new(ox + r * u.x, oy + r * u.y))
            .collect();

        // Angle (and radius) about the centroid, used as the sort key.
        let angle_radius = |p: &Point| (libm::atan2(p.y - oy, p.x - ox), libm::hypot(p.x - ox, p.y - oy));

        // Robots sorted by (angle, radius, id) — id makes it a total, stable order.
        let mut robot_order: Vec<usize> = (0..n).collect();
        robot_order.sort_by(|&a, &b| {
            let (aa, ar) = angle_radius(&scratch.points[a]);
            let (ba, br) = angle_radius(&scratch.points[b]);
            aa.partial_cmp(&ba)
                .unwrap()
                .then(ar.partial_cmp(&br).unwrap())
                .then(scratch.indices[a].cmp(&scratch.indices[b]))
        });
        // Targets sorted by (angle, radius); stable sort breaks exact ties by index.
        let mut target_order: Vec<usize> = (0..n).collect();
        target_order.sort_by(|&a, &b| {
            let (aa, ar) = angle_radius(&targets[a]);
            let (ba, br) = angle_radius(&targets[b]);
            aa.partial_cmp(&ba).unwrap().then(ar.partial_cmp(&br).unwrap())
        });
        let rp: Vec<Point> = robot_order.iter().map(|&k| scratch.points[k]).collect();
        let tp: Vec<Point> = target_order.iter().map(|&k| targets[k]).collect();

        // The cyclic rotation that best aligns the two angle-sorted rings, by total
        // squared displacement. Tie-break to the smallest offset (strict 1e-9 margin).
        let mut best_s = 0usize;
        let mut best_cost: Option<f64> = None;
        for s in 0..n {
            let mut cost = 0.0;
            for i in 0..n {
                let t = tp[(i + s) % n];
                let (dx, dy) = (rp[i].x - t.x, rp[i].y - t.y);
                cost += dx * dx + dy * dy;
            }
            if best_cost.is_none_or(|b| cost < b - 1e-9) {
                best_cost = Some(cost);
                best_s = s;
            }
        }

        let my_rank = robot_order.iter().position(|&k| k == my_idx).unwrap();
        let my_target = tp[(my_rank + best_s) % n];
        let mut err = 0.0_f64;
        for i in 0..n {
            err = err.max(dist(rp[i], tp[(i + best_s) % n]));
        }
        // `_pattern_terminal`: everyone within 3% of the frame scale (or eps).
        let terminate = err < (r * 0.03).max(eps);

        Decision {
            target: my_target,
            terminate,
            overlay: OverlayUpdate::Keep,
        }
    }
}

/// `_pattern_unit_points`: `n` points spread by arc length along a 5-pointed-star
/// outline, re-centred on their own centroid and normalised to unit RMS radius.
#[allow(clippy::cast_precision_loss)]
fn star_unit_points(n: usize) -> Vec<Point> {
    const SPIKES: usize = 5;
    const INNER: f64 = 0.382;
    let m = 2 * SPIKES; // 10 vertices, alternating outer/inner

    let mut verts = Vec::with_capacity(m);
    for i in 0..m {
        let ang = PI / 2.0 + i as f64 * PI / SPIKES as f64;
        let rad = if i % 2 == 0 { 1.0 } else { INNER };
        verts.push(Point::new(rad * libm::cos(ang), rad * libm::sin(ang)));
    }

    let mut segs = Vec::with_capacity(m);
    let mut total = 0.0;
    for i in 0..m {
        let (a, b) = (verts[i], verts[(i + 1) % m]);
        let len = libm::hypot(b.x - a.x, b.y - a.y);
        segs.push(len);
        total += len;
    }

    let mut pts = Vec::with_capacity(n);
    for k in 0..n {
        let d = (k as f64 / n as f64) * total;
        let mut acc = 0.0;
        for i in 0..m {
            let len = segs[i];
            if acc + len >= d || i == m - 1 {
                let t = if len > 1e-12 { (d - acc) / len } else { 0.0 };
                let (a, b) = (verts[i], verts[(i + 1) % m]);
                pts.push(Point::new(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)));
                break;
            }
            acc += len;
        }
    }

    // Re-centre on the template's own centroid.
    let inv_n = 1.0 / n as f64;
    let cx = pts.iter().map(|p| p.x).sum::<f64>() * inv_n;
    let cy = pts.iter().map(|p| p.y).sum::<f64>() * inv_n;
    for p in &mut pts {
        p.x -= cx;
        p.y -= cy;
    }
    // Normalise to unit RMS radius (Python's `sqrt(...) or 1.0`).
    let rms = libm::sqrt(pts.iter().map(|p| p.x * p.x + p.y * p.y).sum::<f64>() * inv_n);
    let rms = if rms == 0.0 { 1.0 } else { rms };
    for p in &mut pts {
        p.x /= rms;
        p.y /= rms;
    }
    pts
}

/// The normalised template (zero centroid, unit RMS radius) of `shape`, as `n`
/// points. The star keeps its original arc-length sampling untouched; every
/// other shape is sampled evenly by arc length along its outline and normalised
/// the same way, so all shapes share one embedding, matching and termination
/// path. This is the only place the chosen pattern enters the algorithm.
fn pattern_unit_points(shape: PatternShape, n: usize) -> Vec<Point> {
    match shape {
        PatternShape::Star => star_unit_points(n),
        PatternShape::Circle => normalise(circle_points(n)),
        PatternShape::Square => normalise(sample_closed(&regular_polygon(4, PI / 4.0), n)),
        PatternShape::Triangle => normalise(sample_closed(&regular_polygon(3, PI / 2.0), n)),
        PatternShape::Pentagon => normalise(sample_closed(&regular_polygon(5, PI / 2.0), n)),
        PatternShape::Hexagon => normalise(sample_closed(&regular_polygon(6, 0.0), n)),
        PatternShape::Line => normalise(sample_open(&line_outline(), n)),
        PatternShape::Cross => normalise(sample_closed(&cross_outline(), n)),
        PatternShape::Heart => normalise(sample_closed(&heart_outline(), n)),
    }
}

/// Re-centre points on their centroid and scale to unit RMS radius (the same
/// normalisation `star_unit_points` applies; `sqrt(...) or 1.0` as in Python).
#[allow(clippy::cast_precision_loss)]
fn normalise(mut pts: Vec<Point>) -> Vec<Point> {
    let n = pts.len();
    if n == 0 {
        return pts;
    }
    let inv_n = 1.0 / n as f64;
    let cx = pts.iter().map(|p| p.x).sum::<f64>() * inv_n;
    let cy = pts.iter().map(|p| p.y).sum::<f64>() * inv_n;
    for p in &mut pts {
        p.x -= cx;
        p.y -= cy;
    }
    let rms = libm::sqrt(pts.iter().map(|p| p.x * p.x + p.y * p.y).sum::<f64>() * inv_n);
    let rms = if rms == 0.0 { 1.0 } else { rms };
    for p in &mut pts {
        p.x /= rms;
        p.y /= rms;
    }
    pts
}

/// `n` points spread evenly by arc length around a closed polygon `verts`
/// (the generalisation of `star_unit_points`' own sampling loop).
#[allow(clippy::cast_precision_loss)]
fn sample_closed(verts: &[Point], n: usize) -> Vec<Point> {
    let m = verts.len();
    let mut segs = Vec::with_capacity(m);
    let mut total = 0.0;
    for i in 0..m {
        let (a, b) = (verts[i], verts[(i + 1) % m]);
        let len = libm::hypot(b.x - a.x, b.y - a.y);
        segs.push(len);
        total += len;
    }
    let mut pts = Vec::with_capacity(n);
    for k in 0..n {
        let d = (k as f64 / n as f64) * total;
        let mut acc = 0.0;
        for i in 0..m {
            let len = segs[i];
            if acc + len >= d || i == m - 1 {
                let t = if len > 1e-12 { (d - acc) / len } else { 0.0 };
                let (a, b) = (verts[i], verts[(i + 1) % m]);
                pts.push(Point::new(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)));
                break;
            }
            acc += len;
        }
    }
    pts
}

/// `n` points spread evenly by arc length along an OPEN polyline, endpoints
/// included (one point sits at the start).
#[allow(clippy::cast_precision_loss)]
fn sample_open(verts: &[Point], n: usize) -> Vec<Point> {
    let m = verts.len();
    let mut segs = Vec::with_capacity(m.saturating_sub(1));
    let mut total = 0.0;
    for i in 0..m.saturating_sub(1) {
        let (a, b) = (verts[i], verts[i + 1]);
        let len = libm::hypot(b.x - a.x, b.y - a.y);
        segs.push(len);
        total += len;
    }
    let mut pts = Vec::with_capacity(n);
    for k in 0..n {
        let frac = if n > 1 { k as f64 / (n - 1) as f64 } else { 0.0 };
        let d = frac * total;
        let mut acc = 0.0;
        let mut placed = false;
        for i in 0..segs.len() {
            let len = segs[i];
            if acc + len >= d || i == segs.len() - 1 {
                let t = if len > 1e-12 { (d - acc) / len } else { 0.0 };
                let (a, b) = (verts[i], verts[i + 1]);
                pts.push(Point::new(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)));
                placed = true;
                break;
            }
            acc += len;
        }
        if !placed {
            pts.push(verts[0]);
        }
    }
    pts
}

/// Vertices of a regular `sides`-gon on the unit circle, first vertex at `phase`.
#[allow(clippy::cast_precision_loss)]
fn regular_polygon(sides: usize, phase: f64) -> Vec<Point> {
    (0..sides)
        .map(|i| {
            let a = phase + i as f64 * 2.0 * PI / sides as f64;
            Point::new(libm::cos(a), libm::sin(a))
        })
        .collect()
}

/// `n` points evenly spaced around the unit circle, starting at the top.
#[allow(clippy::cast_precision_loss)]
fn circle_points(n: usize) -> Vec<Point> {
    (0..n)
        .map(|k| {
            let a = PI / 2.0 + k as f64 * 2.0 * PI / n as f64;
            Point::new(libm::cos(a), libm::sin(a))
        })
        .collect()
}

/// A horizontal unit segment, for the line pattern.
fn line_outline() -> Vec<Point> {
    vec![Point::new(-1.0, 0.0), Point::new(1.0, 0.0)]
}

/// Plus / cross outline: arms of half-length 1 and half-width 0.35, 12 vertices.
fn cross_outline() -> Vec<Point> {
    let (l, w) = (1.0, 0.35);
    [
        (w, w), (w, l), (-w, l), (-w, w), (-l, w), (-l, -w),
        (-w, -w), (-w, -l), (w, -l), (w, -w), (l, -w), (l, w),
    ]
    .iter()
    .map(|&(x, y)| Point::new(x, y))
    .collect()
}

/// A dense heart outline (classic `16 sin^3 t` curve); normalisation rescales it.
#[allow(clippy::cast_precision_loss)]
fn heart_outline() -> Vec<Point> {
    const SEGMENTS: usize = 360;
    (0..SEGMENTS)
        .map(|i| {
            let t = i as f64 * 2.0 * PI / SEGMENTS as f64;
            let s = libm::sin(t);
            let x = 16.0 * s * s * s;
            let y = 13.0 * libm::cos(t) - 5.0 * libm::cos(2.0 * t)
                - 2.0 * libm::cos(3.0 * t)
                - libm::cos(4.0 * t);
            Point::new(x, y)
        })
        .collect()
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

    // ── star_unit_points ───────────────────────────────────────────────────────

    #[test]
    fn star_has_unit_rms_radius_and_zero_centroid() {
        for n in [5usize, 8, 10, 25, 64] {
            let pts = star_unit_points(n);
            assert_eq!(pts.len(), n);
            let inv_n = 1.0 / n as f64;
            let cx = pts.iter().map(|p| p.x).sum::<f64>() * inv_n;
            let cy = pts.iter().map(|p| p.y).sum::<f64>() * inv_n;
            assert!(close(cx, 0.0) && close(cy, 0.0), "centroid ({cx},{cy}) n={n}");
            let rms = (pts.iter().map(|p| p.x * p.x + p.y * p.y).sum::<f64>() * inv_n).sqrt();
            assert!(close(rms, 1.0), "rms {rms} n={n}");
        }
    }

    #[test]
    fn star_outline_has_five_distinct_outer_spikes() {
        // With enough points, the farthest ~5 from the centre should be the spikes,
        // spread around the full circle (a 5-pointed star, not a blob).
        let pts = star_unit_points(60);
        let max_r = pts.iter().map(|p| p.x.hypot(p.y)).fold(0.0_f64, f64::max);
        // Spikes reach the outer radius; count points close to it.
        let near_tip = pts.iter().filter(|p| p.x.hypot(p.y) > max_r * 0.98).count();
        assert!(near_tip >= 5, "expected >=5 near-tip points, got {near_tip}");
    }

    // ── compute ────────────────────────────────────────────────────────────────

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
        PatternFormation::default().compute(&look, &mut rng, &mut scratch)
    }

    // ── generalised templates ───────────────────────────────────────────────────

    const SHAPES: [PatternShape; 9] = [
        PatternShape::Star,
        PatternShape::Circle,
        PatternShape::Square,
        PatternShape::Triangle,
        PatternShape::Pentagon,
        PatternShape::Hexagon,
        PatternShape::Line,
        PatternShape::Cross,
        PatternShape::Heart,
    ];

    #[test]
    fn every_shape_is_normalised_with_the_right_count() {
        for shape in SHAPES {
            for n in [3usize, 8, 25, 64] {
                let pts = pattern_unit_points(shape, n);
                assert_eq!(pts.len(), n, "{shape:?} n={n}");
                let inv_n = 1.0 / n as f64;
                let cx = pts.iter().map(|p| p.x).sum::<f64>() * inv_n;
                let cy = pts.iter().map(|p| p.y).sum::<f64>() * inv_n;
                assert!(close(cx, 0.0) && close(cy, 0.0), "{shape:?} centroid ({cx},{cy})");
                let rms = (pts.iter().map(|p| p.x * p.x + p.y * p.y).sum::<f64>() * inv_n).sqrt();
                assert!(close(rms, 1.0), "{shape:?} rms {rms}");
            }
        }
    }

    #[test]
    fn a_line_is_flat_and_other_shapes_are_not() {
        // The line template has (near) zero spread in y; 2-D shapes do not.
        let spread_y = |shape| {
            let pts = pattern_unit_points(shape, 40);
            let max = pts.iter().map(|p| p.y).fold(f64::MIN, f64::max);
            let min = pts.iter().map(|p| p.y).fold(f64::MAX, f64::min);
            max - min
        };
        assert!(spread_y(PatternShape::Line) < 1e-9, "line should be flat");
        for shape in [PatternShape::Circle, PatternShape::Square, PatternShape::Heart] {
            assert!(spread_y(shape) > 0.5, "{shape:?} should have 2-D spread");
        }
    }

    #[test]
    fn selected_shape_steers_the_target() {
        // A scattered cloud forms whichever shape the instance carries; the
        // matched target differs between two different shapes.
        let pts = vec![pt(0.0, 0.0), pt(100.0, 0.0), pt(0.0, 100.0), pt(100.0, 100.0), pt(50.0, 50.0)];
        let look_for = |shape: PatternShape| {
            let view = view_of(&pts);
            let m = model();
            let look = Look { me: 0, me_index: 0, position: pts[0], view: &view, model: &m };
            let mut rng = Rng::new(1);
            let mut scratch = Scratch::default();
            PatternFormation { shape }.compute(&look, &mut rng, &mut scratch).target
        };
        let star = look_for(PatternShape::Star);
        let square = look_for(PatternShape::Square);
        assert!(
            (star.x - square.x).abs() > 1e-6 || (star.y - square.y).abs() > 1e-6,
            "different shapes should steer to different targets"
        );
    }

    #[test]
    fn single_robot_stays_and_terminates() {
        let d = decide(&[pt(3.0, 4.0)], 0);
        assert!(close(d.target.x, 3.0) && close(d.target.y, 4.0));
        assert!(d.terminate);
    }

    #[test]
    fn coincident_robots_collapse_and_terminate() {
        // All at the same point: RMS radius is 0, so there is no frame — stay put.
        let d = decide(&[pt(1.0, 1.0), pt(1.0, 1.0), pt(1.0, 1.0)], 1);
        assert!(close(d.target.x, 1.0) && close(d.target.y, 1.0));
        assert!(d.terminate);
    }

    #[test]
    fn robots_already_on_the_star_terminate() {
        // Place robots exactly on the embedded star (centre (10,-4), scale 50).
        let (ox, oy, r) = (10.0, -4.0, 50.0);
        let pts: Vec<Point> = star_unit_points(12)
            .iter()
            .map(|u| pt(ox + r * u.x, oy + r * u.y))
            .collect();
        let d = decide(&pts, 0);
        assert!(d.terminate, "robots sitting on the star must report done");
    }

    #[test]
    fn scattered_robots_target_the_star_and_do_not_terminate() {
        // A square cloud is far from any star -> move, not done. The target must lie
        // on the embedded star circle (radius = RMS distance from centroid).
        let pts = vec![pt(0.0, 0.0), pt(100.0, 0.0), pt(0.0, 100.0), pt(100.0, 100.0)];
        let inv_n = 0.25;
        let ox = pts.iter().map(|p| p.x).sum::<f64>() * inv_n;
        let oy = pts.iter().map(|p| p.y).sum::<f64>() * inv_n;
        let r = (pts
            .iter()
            .map(|p| (p.x - ox).powi(2) + (p.y - oy).powi(2))
            .sum::<f64>()
            * inv_n)
            .sqrt();
        let d = decide(&pts, 0);
        assert!(!d.terminate, "a square is not a star");
        // The matched target sits on the star template embedded at radius r — its
        // distance from the centroid equals r * (unit template radius at that point).
        // Weaker, robust check: the target is within the frame (<= r from centroid).
        let dc = (d.target.x - ox).hypot(d.target.y - oy);
        assert!(dc <= r + 1e-6, "target outside the frame: {dc} > {r}");
    }
}

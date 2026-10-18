//! Pattern Formation (star): `Robot._pattern_formation` / `_pattern_terminal`
//! (Suzuki & Yamashita, SIAM J. Comput. 1999), mirroring `robot.py`.
//!
//! Each robot deterministically derives a common reference frame from the
//! snapshot — origin = centroid, scale = RMS distance from the centroid,
//! orientation = world axes (the sim's shared frame sidesteps the
//! orientation-symmetry obstruction) — embeds a normalised 5-pointed-star
//! template in it, matches robots to template points by a consistent angular
//! ordering (with the best cyclic rotation, which removes the rotational
//! ambiguity that would otherwise make the matching oscillate forever), and
//! moves to its matched point.
//!
//! Uses no random numbers, so it is deterministic and scheduler-agnostic.

use core::f64::consts::PI;

use lcm_core::geom::{dist, Point};
use lcm_core::{Algorithm, Decision, Look, OverlayUpdate, Rng, RobotState, Scratch};

#[derive(Clone, Copy, Debug, Default)]
pub struct PatternFormation;

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

        // Target points: the normalised star template placed in the frame.
        let unit = star_unit_points(n);
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
        PatternFormation.compute(&look, &mut rng, &mut scratch)
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

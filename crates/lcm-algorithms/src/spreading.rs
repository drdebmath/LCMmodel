//! Spreading: `Robot._spreading` / `_spreading_terminal`.
//!
//! Each robot moves to the centroid of its Voronoi cell, approximated by
//! sampling the region on a 32 × 32 grid and giving each sample to its
//! nearest robot. Iterating converges to a centroidal Voronoi tessellation:
//! uniform coverage of the region. The region is the world box when the
//! robots know it, else the swarm's bounding box padded by 15 %. A robot
//! whose cell got no sample (it sits on, or right next to, another robot)
//! steps away from its nearest neighbour by one grid cell.

use lcm_core::geom::dist;
use lcm_core::{Algorithm, Decision, Look, OverlayUpdate, Point, Rng, RobotState, Scratch};

const GRID: usize = 32;

#[derive(Clone, Copy, Debug, Default)]
pub struct Spreading;

/// `Robot._spread_region`: `(xmin, xmax, ymin, ymax)`.
fn region(look: &Look<'_>, positions: &[Point]) -> (f64, f64, f64, f64) {
    if let (Some(w), Some(h)) = (look.model.width_bound, look.model.height_bound) {
        return (-w / 2.0, w / 2.0, -h / 2.0, h / 2.0);
    }
    let fold = |f: fn(f64, f64) -> f64, get: fn(&Point) -> f64| {
        positions.iter().map(get).reduce(f).unwrap_or(0.0)
    };
    // Python's max/min keep the first of equal values; for floats that only
    // matters for -0.0, which every use below treats like 0.0.
    let (x0, x1) = (fold(f64::min, |p| p.x), fold(f64::max, |p| p.x));
    let (y0, y1) = (fold(f64::min, |p| p.y), fold(f64::max, |p| p.y));
    let span = (x1 - x0).max(y1 - y0).max(1.0);
    let pad = 0.15 * span;
    (x0 - pad, x1 + pad, y0 - pad, y1 + pad)
}

impl Algorithm for Spreading {
    fn key(&self) -> &'static str {
        "Spreading"
    }

    #[allow(clippy::cast_precision_loss)]
    fn compute(&self, look: &Look<'_>, _: &mut Rng, scratch: &mut Scratch) -> Decision {
        let stay = Decision {
            target: look.position,
            terminate: false,
            overlay: OverlayUpdate::Keep,
        };
        let view = look.view;
        let positions = &mut scratch.points;
        positions.clear();
        let mut mine = None;
        for k in 0..view.len() {
            if view.state[k] == RobotState::Crash {
                continue;
            }
            if k == look.me_index {
                mine = Some(positions.len());
            }
            positions.push(view.pos[k]);
        }
        if positions.len() <= 1 {
            return stay;
        }
        let Some(my_idx) = mine else {
            return stay;
        };
        let me = positions[my_idx];

        let (xmin, xmax, ymin, ymax) = region(look, positions);
        let g = GRID as f64;
        let (sx, sy) = ((xmax - xmin) / g, (ymax - ymin) / g);
        let tol = 0.004 * libm::hypot(xmax - xmin, ymax - ymin);
        let (mut ax, mut ay, mut cnt) = (0.0, 0.0, 0usize);
        for i in 0..GRID {
            let px = xmin + (i as f64 + 0.5) * sx;
            for j in 0..GRID {
                let py = ymin + (j as f64 + 0.5) * sy;
                let (mut best, mut bestd) = (0, f64::INFINITY);
                for (k, p) in positions.iter().enumerate() {
                    let dd = (p.x - px) * (p.x - px) + (p.y - py) * (p.y - py);
                    if dd < bestd {
                        bestd = dd;
                        best = k;
                    }
                }
                if best == my_idx {
                    ax += px;
                    ay += py;
                    cnt += 1;
                }
            }
        }

        let target = if cnt > 0 {
            Point::new(ax / cnt as f64, ay / cnt as f64)
        } else {
            // Starved cell: step away from the nearest neighbour.
            let mut nearest: Option<(Point, f64)> = None;
            for (k, p) in positions.iter().enumerate() {
                if k == my_idx {
                    continue;
                }
                let d2 = (p.x - me.x) * (p.x - me.x) + (p.y - me.y) * (p.y - me.y);
                if nearest.is_none_or(|(_, best)| d2 < best) {
                    nearest = Some((*p, d2));
                }
            }
            let Some((n, _)) = nearest else {
                return stay;
            };
            let (mut dx, mut dy) = (me.x - n.x, me.y - n.y);
            let mut d = libm::hypot(dx, dy);
            if d < 1e-9 {
                // Exactly coincident: an id-based nudge.
                let a = f64::from(look.me % 12) * (std::f64::consts::PI / 6.0);
                (dx, dy, d) = (libm::cos(a), libm::sin(a), 1.0);
            }
            let step = sx.max(sy);
            Point::new(me.x + dx / d * step, me.y + dy / d * step)
        };
        Decision {
            target,
            terminate: dist(target, look.position) < tol,
            overlay: OverlayUpdate::Keep,
        }
    }
}

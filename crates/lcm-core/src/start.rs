//! Generated starting positions (docs/core-schema.md §4.3).
//!
//! Three choices, each answering one question:
//! - **pattern**: what overall shape (a square, a circle, clusters, a free cloud…);
//! - **arrangement**: inside the shape, on its edge, or in a ring;
//! - **distribution**: how robots are spaced there.
//!
//! Positions are centred on the origin, then rotated. Random draws come from a
//! generator of their own, seeded from the run's seed, so a given spec and seed
//! always give the same start, and the start is independent of the activation
//! draws. Starts made without a spec (the original square, or explicit
//! coordinates) never come through here.

use crate::geom::Point;
use crate::rng::Rng;
use core::f64::consts::{PI, TAU};

/// What overall shape the robots start in.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "lowercase"))]
pub enum Pattern {
    /// The open plane, no shape: a Gaussian cloud.
    Cloud,
    /// Side `size`.
    Square,
    /// Width `size`, height `size × aspect`.
    Rectangle,
    /// Diameter `size`.
    Circle,
    /// Regular polygon with `sides` sides, corners on a circle of diameter `size`.
    Polygon,
    /// Length `size`.
    Line,
    /// `clusters` groups, centred on a circle of diameter `size`.
    Clusters,
}

/// Where in the pattern: inside it, on its edge, or (circles) in a ring.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "lowercase"))]
pub enum Arrangement {
    /// The whole inside of the shape.
    Filled,
    /// Only the outline.
    Edge,
    /// A band between the outline and a hole of `inner` × the radius (circles).
    Ring,
}

/// How robots are spaced.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "lowercase"))]
pub enum Distribution {
    /// Every spot equally likely.
    Uniform,
    /// Exact, evenly spaced positions (a grid or sunflower pattern in areas).
    Even,
    /// Even positions moved by up to `jitter` × half the spacing.
    Jittered,
    /// Normal with standard deviation `spread`, centred (per cluster for clusters).
    Gaussian,
}

#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default, deny_unknown_fields))]
pub struct StartSpec {
    pub pattern: Pattern,
    pub arrangement: Arrangement,
    pub distribution: Distribution,
    /// Side, diameter or length of the shape, in world units.
    pub size: f64,
    /// Rectangle height / width.
    pub aspect: f64,
    /// Ring hole radius as a fraction of the outer radius, in [0, 0.95].
    pub inner: f64,
    pub sides: u32,
    pub clusters: u32,
    /// Standard deviation of Gaussian placement (and cluster radius when uniform).
    pub spread: f64,
    /// Jitter as a fraction of half the spacing, in [0, 1].
    pub jitter: f64,
    /// Rotation about the origin, in degrees.
    pub rotation: f64,
}

impl Default for StartSpec {
    fn default() -> Self {
        Self {
            pattern: Pattern::Cloud,
            arrangement: Arrangement::Filled,
            distribution: Distribution::Gaussian,
            size: 600.0,
            aspect: 0.5,
            inner: 0.6,
            sides: 6,
            clusters: 4,
            spread: 150.0,
            jitter: 0.3,
            rotation: 0.0,
        }
    }
}

/// Arrangements each pattern supports (patterns without an inside/edge
/// choice take `Filled`, except the line, which is all edge).
#[must_use]
pub fn arrangements(pattern: Pattern) -> &'static [Arrangement] {
    use Arrangement::{Edge, Filled, Ring};
    match pattern {
        Pattern::Cloud | Pattern::Clusters => &[Filled],
        Pattern::Square | Pattern::Rectangle | Pattern::Polygon => &[Filled, Edge],
        Pattern::Circle => &[Filled, Edge, Ring],
        Pattern::Line => &[Edge],
    }
}

/// Distributions each pattern and arrangement support; empty if the
/// arrangement does not fit the pattern.
#[must_use]
pub fn distributions(pattern: Pattern, arrangement: Arrangement) -> &'static [Distribution] {
    use Distribution::{Even, Gaussian, Jittered, Uniform};
    if !arrangements(pattern).contains(&arrangement) {
        return &[];
    }
    match (pattern, arrangement) {
        (Pattern::Cloud, _) => &[Gaussian],
        (Pattern::Clusters, _) => &[Gaussian, Uniform],
        (_, Arrangement::Filled) => &[Uniform, Even, Jittered, Gaussian],
        _ => &[Uniform, Even, Jittered],
    }
}

impl StartSpec {
    /// # Errors
    /// A parameter out of range, or a combination the pattern does not support.
    pub fn validate(&self) -> Result<(), String> {
        let positive = |name: &str, v: f64| {
            if v.is_finite() && v > 0.0 {
                Ok(())
            } else {
                Err(format!("start.{name} must be a positive number, got {v}"))
            }
        };
        positive("size", self.size)?;
        positive("aspect", self.aspect)?;
        positive("spread", self.spread)?;
        if !(0.0..=0.95).contains(&self.inner) {
            return Err(format!(
                "start.inner must be in [0, 0.95], got {}",
                self.inner
            ));
        }
        if !(0.0..=1.0).contains(&self.jitter) {
            return Err(format!(
                "start.jitter must be in [0, 1], got {}",
                self.jitter
            ));
        }
        if !(3..=64).contains(&self.sides) {
            return Err(format!("start.sides must be 3..=64, got {}", self.sides));
        }
        if self.clusters == 0 {
            return Err("start.clusters must be at least 1".to_owned());
        }
        if !self.rotation.is_finite() {
            return Err("start.rotation must be finite".to_owned());
        }
        if !arrangements(self.pattern).contains(&self.arrangement) {
            return Err(format!(
                "a {:?} pattern cannot be arranged as {:?}",
                self.pattern, self.arrangement
            ));
        }
        if !distributions(self.pattern, self.arrangement).contains(&self.distribution) {
            return Err(format!(
                "{:?} placement is not available for a {:?} {:?} start",
                self.distribution, self.arrangement, self.pattern
            ));
        }
        Ok(())
    }
}

/// Seeds the start generator apart from the activation generator (which uses
/// the plain seed), so start positions and activation times are independent.
const START_SALT: u64 = 0xA076_1D64_78BD_642F;
/// Golden angle, for the sunflower (Vogel) pattern that spreads points evenly over a disk.
const GOLDEN_ANGLE: f64 = 2.399_963_229_728_653;

/// Standard normal pair (Box–Muller).
fn normal_pair(rng: &mut Rng) -> (f64, f64) {
    let u1 = 1.0 - rng.random(); // (0, 1], so the logarithm is finite
    let u2 = rng.random();
    let r = (-2.0 * libm::log(u1)).sqrt();
    (r * libm::cos(TAU * u2), r * libm::sin(TAU * u2))
}

fn gaussian(rng: &mut Rng, sigma: f64) -> Point {
    let (a, b) = normal_pair(rng);
    Point::new(a * sigma, b * sigma)
}

/// A Gaussian point accepted only inside `inside`; after many misses, a uniform point instead.
fn gaussian_within(
    rng: &mut Rng,
    sigma: f64,
    inside: impl Fn(Point) -> bool,
    fallback: impl Fn(&mut Rng) -> Point,
) -> Point {
    for _ in 0..64 {
        let p = gaussian(rng, sigma);
        if inside(p) {
            return p;
        }
    }
    fallback(rng)
}

fn uniform_disk(rng: &mut Rng, r_in: f64, r_out: f64) -> Point {
    let r = (r_in * r_in + rng.random() * (r_out * r_out - r_in * r_in)).sqrt();
    let a = TAU * rng.random();
    Point::new(r * libm::cos(a), r * libm::sin(a))
}

/// Grid cell centres for n points in a w × h rectangle, row by row from the
/// top; a part-filled last row is centred.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]
fn grid(n: usize, w: f64, h: f64) -> (Vec<Point>, f64, f64) {
    let cols = ((n as f64 * w / h).sqrt().ceil() as usize).clamp(1, n.max(1));
    let rows = n.div_ceil(cols).max(1);
    let (sx, sy) = (w / cols as f64, h / rows as f64);
    let mut out = Vec::with_capacity(n);
    for k in 0..n {
        let (r, c) = (k / cols, k % cols);
        let in_row = if r == rows - 1 { n - r * cols } else { cols };
        let shift = (cols - in_row) as f64 * sx / 2.0;
        out.push(Point::new(
            -w / 2.0 + (c as f64 + 0.5) * sx + shift,
            h / 2.0 - (r as f64 + 0.5) * sy,
        ));
    }
    (out, sx, sy)
}

/// Sunflower pattern over the annulus r_in..r_out: even area coverage.
#[allow(clippy::cast_precision_loss)]
fn sunflower(n: usize, r_in: f64, r_out: f64) -> Vec<Point> {
    (0..n)
        .map(|k| {
            let f = (k as f64 + 0.5) / n as f64;
            let r = (r_in * r_in + f * (r_out * r_out - r_in * r_in)).sqrt();
            let a = k as f64 * GOLDEN_ANGLE;
            Point::new(r * libm::cos(a), r * libm::sin(a))
        })
        .collect()
}

fn nudge(rng: &mut Rng, p: Point, dx: f64, dy: f64) -> Point {
    Point::new(p.x + rng.uniform(-dx, dx), p.y + rng.uniform(-dy, dy))
}

/// A point at arc length `s` along the closed outline through `verts`.
fn along(verts: &[Point], perimeter: f64, s: f64) -> Point {
    let mut s = s.rem_euclid(perimeter);
    for i in 0..verts.len() {
        let (a, b) = (verts[i], verts[(i + 1) % verts.len()]);
        let len = libm::hypot(b.x - a.x, b.y - a.y);
        if s <= len || i == verts.len() - 1 {
            let t = if len > 0.0 { (s / len).min(1.0) } else { 0.0 };
            return Point::new(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y));
        }
        s -= len;
    }
    verts[0]
}

/// Corners of the regular polygon of `spec`, counter-clockwise, the first at the top.
#[allow(clippy::cast_precision_loss)]
fn polygon_corners(sides: u32, half: f64) -> Vec<Point> {
    (0..sides)
        .map(|i| {
            let a = PI / 2.0 + TAU * f64::from(i) / f64::from(sides);
            Point::new(half * libm::cos(a), half * libm::sin(a))
        })
        .collect()
}

/// `n` points on the closed outline through `verts`, spaced as `d` says.
#[allow(clippy::cast_precision_loss)]
fn on_outline(rng: &mut Rng, verts: &[Point], d: Distribution, j: f64, n: usize) -> Vec<Point> {
    let perimeter: f64 = (0..verts.len())
        .map(|i| {
            let (a, b) = (verts[i], verts[(i + 1) % verts.len()]);
            libm::hypot(b.x - a.x, b.y - a.y)
        })
        .sum();
    let nf = n.max(1) as f64;
    (0..n)
        .map(|k| {
            let s = match d {
                Distribution::Even => perimeter * k as f64 / nf,
                Distribution::Jittered => perimeter * (k as f64 + rng.uniform(-0.5, 0.5) * j) / nf,
                _ => perimeter * rng.random(),
            };
            along(verts, perimeter, s)
        })
        .collect()
}

/// The point of a convex polygon (corners counter-clockwise around the origin)
/// for (f, u) in [0, 1)²: the polygon is cut into equal triangles at the centre,
/// `u` picks the triangle and the spot along its outer side, `√f` how far out.
/// Uniform (f, u) give uniform points over the area.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]
fn in_polygon(verts: &[Point], f: f64, u: f64) -> Point {
    let k = verts.len();
    let x = u * k as f64;
    let i = (x as usize).min(k - 1);
    let t = x - i as f64;
    let (a, b) = (verts[i], verts[(i + 1) % k]);
    let r = f.sqrt();
    Point::new(r * (a.x + t * (b.x - a.x)), r * (a.y + t * (b.y - a.y)))
}

fn inside_polygon(verts: &[Point], p: Point) -> bool {
    (0..verts.len()).all(|i| {
        let (a, b) = (verts[i], verts[(i + 1) % verts.len()]);
        (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x) >= -1e-12
    })
}

/// Fractional part of k / golden ratio: with (k + ½) / n it makes a Fibonacci
/// lattice, an even spread of n points over the unit square.
#[allow(clippy::cast_precision_loss)]
fn golden_fraction(k: usize) -> f64 {
    (k as f64 * 0.618_033_988_749_894_9).fract()
}

/// Starting positions for `n` robots. `spec` must have passed `validate`.
#[must_use]
#[allow(clippy::cast_precision_loss, clippy::too_many_lines)]
pub fn generate(spec: &StartSpec, n: usize, seed: u64) -> Vec<Point> {
    use Arrangement::{Edge, Filled, Ring};
    use Distribution::{Even, Gaussian, Jittered, Uniform};
    let mut rng = Rng::new(seed ^ START_SALT);
    let rng = &mut rng;
    let nf = n.max(1) as f64;
    let half = spec.size / 2.0;
    let j = spec.jitter;
    let d = spec.distribution;

    let mut points: Vec<Point> = match (spec.pattern, spec.arrangement) {
        (Pattern::Cloud, _) => (0..n).map(|_| gaussian(rng, spec.spread)).collect(),
        (Pattern::Square | Pattern::Rectangle, arrangement) => {
            let (w, h) = if spec.pattern == Pattern::Square {
                (spec.size, spec.size)
            } else {
                (spec.size, spec.size * spec.aspect)
            };
            if arrangement == Edge {
                let corners = [
                    Point::new(-w / 2.0, h / 2.0),
                    Point::new(w / 2.0, h / 2.0),
                    Point::new(w / 2.0, -h / 2.0),
                    Point::new(-w / 2.0, -h / 2.0),
                ];
                on_outline(rng, &corners, d, j, n)
            } else {
                let inside = |p: Point| p.x.abs() <= w / 2.0 && p.y.abs() <= h / 2.0;
                let box_point = |rng: &mut Rng| {
                    Point::new(
                        rng.uniform(-w / 2.0, w / 2.0),
                        rng.uniform(-h / 2.0, h / 2.0),
                    )
                };
                match d {
                    Uniform => (0..n).map(|_| box_point(rng)).collect(),
                    Even => grid(n, w, h).0,
                    Jittered => {
                        let (cells, sx, sy) = grid(n, w, h);
                        cells
                            .into_iter()
                            .map(|p| nudge(rng, p, j * sx / 2.0, j * sy / 2.0))
                            .collect()
                    }
                    Gaussian => (0..n)
                        .map(|_| gaussian_within(rng, spec.spread, inside, box_point))
                        .collect(),
                }
            }
        }
        (Pattern::Circle, Edge) => (0..n)
            .map(|k| {
                let a = match d {
                    Even => TAU * k as f64 / nf,
                    Jittered => TAU * (k as f64 + rng.uniform(-0.5, 0.5) * j) / nf,
                    _ => TAU * rng.random(),
                };
                Point::new(half * libm::cos(a), half * libm::sin(a))
            })
            .collect(),
        (Pattern::Circle, Filled | Ring) => {
            let r_in = if spec.arrangement == Ring {
                spec.inner * half
            } else {
                0.0
            };
            let spacing = (PI * (half * half - r_in * r_in) / nf).sqrt();
            match d {
                Uniform => (0..n).map(|_| uniform_disk(rng, r_in, half)).collect(),
                Even => sunflower(n, r_in, half),
                Jittered => sunflower(n, r_in, half)
                    .into_iter()
                    .map(|p| nudge(rng, p, j * spacing / 2.0, j * spacing / 2.0))
                    .collect(),
                Gaussian => (0..n)
                    .map(|_| {
                        gaussian_within(
                            rng,
                            spec.spread,
                            |p| libm::hypot(p.x, p.y) <= half,
                            |rng| uniform_disk(rng, 0.0, half),
                        )
                    })
                    .collect(),
            }
        }
        (Pattern::Polygon, Edge) => on_outline(rng, &polygon_corners(spec.sides, half), d, j, n),
        (Pattern::Polygon, _) => {
            let corners = polygon_corners(spec.sides, half);
            let area =
                f64::from(spec.sides) / 2.0 * half * half * libm::sin(TAU / f64::from(spec.sides));
            let spacing = (area / nf).sqrt();
            let lattice =
                |k: usize| in_polygon(&corners, (k as f64 + 0.5) / nf, golden_fraction(k));
            match d {
                Uniform => (0..n)
                    .map(|_| {
                        let (f, u) = (rng.random(), rng.random());
                        in_polygon(&corners, f, u)
                    })
                    .collect(),
                Even => (0..n).map(lattice).collect(),
                Jittered => (0..n)
                    .map(|k| nudge(rng, lattice(k), j * spacing / 2.0, j * spacing / 2.0))
                    .collect(),
                Gaussian => (0..n)
                    .map(|_| {
                        gaussian_within(
                            rng,
                            spec.spread,
                            |p| inside_polygon(&corners, p),
                            |rng| {
                                let (f, u) = (rng.random(), rng.random());
                                in_polygon(&corners, f, u)
                            },
                        )
                    })
                    .collect(),
            }
        }
        (Pattern::Line, _) => {
            let spacing = spec.size / (nf - 1.0).max(1.0);
            let even = |k: usize| {
                if n == 1 {
                    0.0
                } else {
                    -half + k as f64 * spacing
                }
            };
            (0..n)
                .map(|k| match d {
                    Even => Point::new(even(k), 0.0),
                    Jittered => {
                        Point::new(even(k) + rng.uniform(-1.0, 1.0) * j * spacing / 2.0, 0.0)
                    }
                    _ => Point::new(rng.uniform(-half, half), 0.0),
                })
                .collect()
        }
        (Pattern::Clusters, _) => {
            let c = spec.clusters as usize;
            let center = |m: usize| {
                if c == 1 {
                    Point::new(0.0, 0.0)
                } else {
                    let a = TAU * m as f64 / c as f64;
                    Point::new(half * libm::cos(a), half * libm::sin(a))
                }
            };
            (0..n)
                .map(|i| {
                    let o = center(i % c);
                    let p = if d == Uniform {
                        uniform_disk(rng, 0.0, spec.spread)
                    } else {
                        gaussian(rng, spec.spread)
                    };
                    Point::new(o.x + p.x, o.y + p.y)
                })
                .collect()
        }
    };

    if spec.pattern != Pattern::Cloud && spec.rotation != 0.0 {
        let a = spec.rotation.to_radians();
        let (s, c) = (libm::sin(a), libm::cos(a));
        for p in &mut points {
            *p = Point::new(p.x * c - p.y * s, p.x * s + p.y * c);
        }
    }
    points
}

/// How many robots stand on each multiplicity: `size` if given (at least 2),
/// else a third of the robots for one multiplicity, or up to 4 each for several.
fn multiplicity_size(n: usize, k: usize, size: usize) -> Result<usize, String> {
    let size = match size {
        0 if k == 1 => (n / 3).max(2),
        0 => 4.min(n / k).max(2),
        1 => return Err("a multiplicity needs at least 2 robots".to_owned()),
        s => s,
    };
    if k * size > n {
        return Err(format!(
            "{k} multiplicities of {size} robots need {} robots, there are {n}",
            k * size
        ));
    }
    Ok(size)
}

/// Checks that `k` multiplicities of `size` robots (0: chosen for you) fit
/// among `n` robots.
///
/// # Errors
/// A size of 1, or more robots asked for than there are.
pub fn check_multiplicities(n: usize, k: usize, size: usize) -> Result<(), String> {
    if k == 0 {
        return Ok(());
    }
    multiplicity_size(n, k, size).map(|_| ())
}

/// Makes `k` multiplicity points of `size` robots each: picks `k` robots at
/// random (from `seed`, with a stream of their own) as anchors and moves
/// `size - 1` other robots onto each. The rest stay where they were.
///
/// # Errors
/// See [`check_multiplicities`].
pub fn stack_multiplicities(
    points: &mut [Point],
    k: usize,
    size: usize,
    seed: u64,
) -> Result<(), String> {
    if k == 0 {
        return Ok(());
    }
    let n = points.len();
    let size = multiplicity_size(n, k, size)?;
    let mut order: Vec<usize> = (0..n).collect();
    Rng::new(seed ^ 0x4D55_4C54).shuffle(&mut order);
    let per = size - 1;
    for j in 0..k {
        let anchor = points[order[j]];
        for t in 0..per {
            points[order[k + j * per + t]] = anchor;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const PATTERNS: [Pattern; 7] = [
        Pattern::Cloud,
        Pattern::Square,
        Pattern::Rectangle,
        Pattern::Circle,
        Pattern::Polygon,
        Pattern::Line,
        Pattern::Clusters,
    ];
    const ARRANGEMENTS: [Arrangement; 3] =
        [Arrangement::Filled, Arrangement::Edge, Arrangement::Ring];
    const ALL: [Distribution; 4] = [
        Distribution::Uniform,
        Distribution::Even,
        Distribution::Jittered,
        Distribution::Gaussian,
    ];

    fn spec(p: Pattern, a: Arrangement, d: Distribution) -> StartSpec {
        StartSpec {
            pattern: p,
            arrangement: a,
            distribution: d,
            ..StartSpec::default()
        }
    }

    /// Distance from p to the closed outline through `verts`.
    fn outline_gap(verts: &[Point], p: Point) -> f64 {
        let k = verts.len();
        (0..k)
            .map(|i| {
                let (a, b) = (verts[i], verts[(i + 1) % k]);
                let (dx, dy) = (b.x - a.x, b.y - a.y);
                let t =
                    (((p.x - a.x) * dx + (p.y - a.y) * dy) / (dx * dx + dy * dy)).clamp(0.0, 1.0);
                libm::hypot(p.x - a.x - t * dx, p.y - a.y - t * dy)
            })
            .fold(f64::INFINITY, f64::min)
    }

    fn rect(w: f64, h: f64) -> Vec<Point> {
        vec![
            Point::new(-w / 2.0, h / 2.0),
            Point::new(w / 2.0, h / 2.0),
            Point::new(w / 2.0, -h / 2.0),
            Point::new(-w / 2.0, -h / 2.0),
        ]
    }

    #[test]
    fn every_supported_combination_places_n_finite_robots_in_its_shape() {
        let mut supported = 0;
        for p in PATTERNS {
            for a in ARRANGEMENTS {
                for d in ALL {
                    let s = spec(p, a, d);
                    assert_eq!(
                        s.validate().is_ok(),
                        distributions(p, a).contains(&d),
                        "{p:?} {a:?} {d:?}"
                    );
                    if s.validate().is_err() {
                        continue;
                    }
                    supported += 1;
                    let half = s.size / 2.0;
                    let corners = polygon_corners(s.sides, half);
                    for n in [1, 2, 7, 500] {
                        let pts = generate(&s, n, 42);
                        assert_eq!(pts.len(), n, "{p:?} {a:?} {d:?} n={n}");
                        // jittered points may move up to half a spacing past the even ones
                        let slack = if d == Distribution::Jittered {
                            half
                        } else {
                            1e-9
                        };
                        for q in &pts {
                            assert!(q.x.is_finite() && q.y.is_finite(), "{p:?} {a:?} {d:?}");
                            let r = libm::hypot(q.x, q.y);
                            let ok = match (p, a) {
                                (Pattern::Square, Arrangement::Filled) => {
                                    q.x.abs() <= half + slack && q.y.abs() <= half + slack
                                }
                                (Pattern::Rectangle, Arrangement::Filled) => {
                                    q.x.abs() <= half + slack
                                        && q.y.abs() <= half * s.aspect + slack
                                }
                                (Pattern::Square, Arrangement::Edge) => {
                                    outline_gap(&rect(s.size, s.size), *q) < 1e-9
                                }
                                (Pattern::Rectangle, Arrangement::Edge) => {
                                    outline_gap(&rect(s.size, s.size * s.aspect), *q) < 1e-9
                                }
                                (Pattern::Circle, Arrangement::Filled) => r <= half + slack,
                                (Pattern::Circle, Arrangement::Ring) => {
                                    d == Distribution::Jittered
                                        || (r >= s.inner * half - 1e-9 && r <= half + 1e-9)
                                }
                                (Pattern::Circle, Arrangement::Edge) => (r - half).abs() < 1e-9,
                                (Pattern::Polygon, Arrangement::Edge) => {
                                    outline_gap(&corners, *q) < 1e-9
                                }
                                (Pattern::Polygon, Arrangement::Filled) => {
                                    d == Distribution::Jittered || inside_polygon(&corners, *q)
                                }
                                (Pattern::Line, _) => q.y == 0.0 && q.x.abs() <= half + slack,
                                _ => true,
                            };
                            assert!(ok, "{p:?} {a:?} {d:?}: {q:?} is outside the shape");
                        }
                    }
                }
            }
        }
        // 4 filled shapes × 4, 5 edges (with the line) × 3, ring × 3, cloud 1, clusters 2
        assert_eq!(supported, 16 + 15 + 3 + 1 + 2);
    }

    #[test]
    fn same_seed_same_start_and_even_ignores_the_seed() {
        for p in PATTERNS {
            for &a in arrangements(p) {
                for &d in distributions(p, a) {
                    let s = spec(p, a, d);
                    assert_eq!(
                        generate(&s, 300, 7),
                        generate(&s, 300, 7),
                        "{p:?} {a:?} {d:?}"
                    );
                    if d == Distribution::Even {
                        assert_eq!(
                            generate(&s, 300, 7),
                            generate(&s, 300, 8),
                            "{p:?} {a:?} even depends on the seed"
                        );
                    } else {
                        assert_ne!(
                            generate(&s, 300, 7),
                            generate(&s, 300, 8),
                            "{p:?} {a:?} {d:?} ignores the seed"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn even_placements_are_exact() {
        use Arrangement::{Edge, Filled};
        use Distribution::Even;
        let line = generate(&spec(Pattern::Line, Edge, Even), 5, 0);
        assert_eq!(
            line.iter().map(|p| p.x).collect::<Vec<_>>(),
            vec![-300.0, -150.0, 0.0, 150.0, 300.0]
        );
        let circle = generate(&spec(Pattern::Circle, Edge, Even), 4, 0);
        assert!((circle[1].x).abs() < 1e-9 && (circle[1].y - 300.0).abs() < 1e-9);
        let grid = generate(&spec(Pattern::Square, Filled, Even), 9, 0);
        assert_eq!(grid[0], Point::new(-200.0, 200.0));
        assert_eq!(grid[4], Point::new(0.0, 0.0));
        // 8 robots on a square's edge: the 4 corners and the 4 midpoints, from the top-left
        let edge = generate(&spec(Pattern::Square, Edge, Even), 8, 0);
        let expect = [
            (-300.0, 300.0),
            (0.0, 300.0),
            (300.0, 300.0),
            (300.0, 0.0),
            (300.0, -300.0),
            (0.0, -300.0),
            (-300.0, -300.0),
            (-300.0, 0.0),
        ];
        for (p, (x, y)) in edge.iter().zip(expect) {
            assert!(
                (p.x - x).abs() < 1e-9 && (p.y - y).abs() < 1e-9,
                "{p:?} vs ({x}, {y})"
            );
        }
    }

    #[test]
    #[allow(clippy::cast_precision_loss)]
    fn a_filled_polygon_is_covered_evenly() {
        // Even and random placements both put each of the polygon's centre
        // triangles' share of robots in it, and a hexagon's inner half (by
        // radius) holds a quarter of them.
        for d in [Distribution::Even, Distribution::Uniform] {
            let s = spec(Pattern::Polygon, Arrangement::Filled, d);
            let pts = generate(&s, 6000, 5);
            let corners = polygon_corners(s.sides, s.size / 2.0);
            let small: Vec<Point> = corners
                .iter()
                .map(|c| Point::new(c.x / 2.0, c.y / 2.0))
                .collect();
            let inner = pts.iter().filter(|p| inside_polygon(&small, **p)).count() as f64 / 6000.0;
            assert!((inner - 0.25).abs() < 0.02, "{d:?}: inner share {inner}");
            let mut per = [0usize; 6];
            for p in &pts {
                let a = (libm::atan2(p.y, p.x) - PI / 2.0).rem_euclid(TAU);
                per[((a / (TAU / 6.0)) as usize).min(5)] += 1;
            }
            for c in per {
                assert!(
                    (c as f64 / 1000.0 - 1.0).abs() < 0.08,
                    "{d:?}: sector counts {per:?}"
                );
            }
        }
    }

    #[test]
    #[allow(clippy::cast_precision_loss)]
    fn free_cloud_has_the_requested_spread() {
        let s = spec(Pattern::Cloud, Arrangement::Filled, Distribution::Gaussian);
        let pts = generate(&s, 40_000, 3);
        let n = pts.len() as f64;
        let (mx, my) = (
            pts.iter().map(|p| p.x).sum::<f64>() / n,
            pts.iter().map(|p| p.y).sum::<f64>() / n,
        );
        let sd = (pts.iter().map(|p| p.x * p.x + p.y * p.y).sum::<f64>() / (2.0 * n)).sqrt();
        assert!(mx.abs() < 3.0 && my.abs() < 3.0, "mean {mx} {my}");
        assert!((sd - 150.0).abs() < 3.0, "spread {sd}");
    }

    #[test]
    fn rotation_turns_the_shape() {
        let s = StartSpec {
            rotation: 90.0,
            ..spec(Pattern::Line, Arrangement::Edge, Distribution::Even)
        };
        let pts = generate(&s, 3, 0);
        assert!(pts[0].x.abs() < 1e-9 && (pts[0].y + 300.0).abs() < 1e-9);
    }

    #[test]
    fn mismatched_choices_are_rejected_with_a_clear_reason() {
        let ring_square = spec(Pattern::Square, Arrangement::Ring, Distribution::Even);
        assert!(ring_square
            .validate()
            .unwrap_err()
            .contains("cannot be arranged"));
        let gaussian_edge = spec(Pattern::Circle, Arrangement::Edge, Distribution::Gaussian);
        assert!(gaussian_edge
            .validate()
            .unwrap_err()
            .contains("not available"));
    }
}

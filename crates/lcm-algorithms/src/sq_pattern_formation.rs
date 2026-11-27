//! Sequential Pattern Formation: Algorithm **SqPF** (Algorithm 1 with
//! Procedures 2–6, for |P̂| ≥ 5) and Algorithm **SqPF′** (Algorithm 7, for
//! 1 < |P̂| < 5) of "Universal Pattern Formation by Oblivious Robots Under
//! Sequential Schedulers" (Flocchini, Navarra, Pattanayak, Piselli & Santoro,
//! arXiv:2412.10733v3, Sections 2–5). docs/sqpf.md maps every line of the
//! paper's pseudocode to the code below and lists where the paper leaves a
//! choice open.
//!
//! **The robots are the paper's (Section 2.1).** [`SqPatternFormation::compute`]
//! is the robot's body; [`destination`] is its built-in algorithm:
//!
//! - *private coordinate system*: every robot has its own [`Compass`] (direction
//!   of its x-axis, unit of length and handedness, drawn once per robot from the
//!   run's seed) and perceives itself at the origin. No robot sees the world axes.
//! - *Look*: the snapshot is the **set** Q of distinct occupied points in that
//!   private frame ([`snapshot`]). There are no identities, no states, no lights
//!   and no multiplicity detection: two robots on one point look like one point.
//! - *oblivious*: [`destination`] is a pure function of (P̂, Q). Nothing is kept
//!   between activations.
//! - *silent*, *identical*: every robot runs the same function on its own
//!   snapshot; robots never terminate, they only stop moving (the UPF predicate
//!   asks the configuration to stay unchanged, Section 2.3).
//! - non-rigid movement with δ > 0 and the sequential scheduler 𝒮𝒬 are the
//!   core's (`rigid_movement: false`, `delta`, `scheduler: "sequential"`); the
//!   registry entry ([`plan`]) refuses anything else.

use core::cmp::Ordering;
use core::f64::consts::TAU;
use std::collections::BTreeMap;

use lcm_core::geom::dist;
use lcm_core::{
    Algorithm, ConfigError, Decision, Look, OverlayUpdate, PatternShape, Plan, Point, Rng,
    SchedulerKind, Scratch, SimConfig,
};

use crate::sq_gathering::LocalFrame;

/// Registry key.
pub const KEY: &str = "SqPatternFormation";

/// Positions closer than this fraction of the SEC radius ρ are the same point,
/// and so are positions closer than the simulator's minimum move (its `eps`):
/// a robot cannot move by less, so a robot that near a point is on it
/// (docs/sqpf.md, choice 9).
const REL_TOL: f64 = 1e-9;
/// Directions closer than this (radians) are the same radius (co-radial).
const ANG_TOL: f64 = 1e-9;

// ── plane geometry ────────────────────────────────────────────────────────────

fn sub(a: Point, b: Point) -> Point {
    Point::new(a.x - b.x, a.y - b.y)
}

fn norm(v: Point) -> f64 {
    libm::hypot(v.x, v.y)
}

fn angle_of(v: Point) -> f64 {
    libm::atan2(v.y, v.x)
}

fn polar(o: Point, r: f64, a: f64) -> Point {
    Point::new(o.x + r * libm::cos(a), o.y + r * libm::sin(a))
}

/// The angle swept from direction `from` to direction `to` turning in
/// direction `dir` (+1: counterclockwise in the robot's frame, −1: clockwise),
/// in [0, 2π).
fn sweep(from: f64, to: f64, dir: f64) -> f64 {
    let s = (dir * (to - from)).rem_euclid(TAU);
    if s > TAU - ANG_TOL {
        0.0
    } else {
        s
    }
}

fn cmp_tol(a: f64, b: f64, tol: f64) -> Ordering {
    if a < b - tol {
        Ordering::Less
    } else if a > b + tol {
        Ordering::Greater
    } else {
        Ordering::Equal
    }
}

fn circle2(a: Point, b: Point) -> (Point, f64) {
    (
        Point::new(0.5 * (a.x + b.x), 0.5 * (a.y + b.y)),
        0.5 * dist(a, b),
    )
}

fn circle3(a: Point, b: Point, c: Point) -> (Point, f64) {
    let (bx, by) = (b.x - a.x, b.y - a.y);
    let (cx, cy) = (c.x - a.x, c.y - a.y);
    let (b2, c2) = (bx * bx + by * by, cx * cx + cy * cy);
    let d = 2.0 * (bx * cy - by * cx);
    if d.abs() <= 1e-14 * (b2 + c2) {
        // Collinear: the circle on the farthest pair.
        let pairs = [circle2(a, b), circle2(a, c), circle2(b, c)];
        return pairs
            .into_iter()
            .fold(pairs[0], |m, x| if x.1 > m.1 { x } else { m });
    }
    let ux = (cy * b2 - by * c2) / d;
    let uy = (bx * c2 - cx * b2) / d;
    (Point::new(a.x + ux, a.y + uy), libm::hypot(ux, uy))
}

/// SEC(S), the smallest enclosing circle (Section 2.3), as (centre, radius).
/// Incremental Welzl; the circle is unique, so the order of `pts` does not
/// change it.
#[must_use]
pub fn sec(pts: &[Point]) -> (Point, f64) {
    let inside = |c: (Point, f64), p: Point| dist(c.0, p) <= c.1 * (1.0 + 1e-12) + 1e-300;
    let mut c = (pts[0], 0.0);
    for i in 1..pts.len() {
        if inside(c, pts[i]) {
            continue;
        }
        c = (pts[i], 0.0);
        for j in 0..i {
            if inside(c, pts[j]) {
                continue;
            }
            c = circle2(pts[i], pts[j]);
            for m in 0..j {
                if !inside(c, pts[m]) {
                    c = circle3(pts[i], pts[j], pts[m]);
                }
            }
        }
    }
    c
}

// ── the pattern P̂ ─────────────────────────────────────────────────────────────

/// The input pattern P̂ (Section 2.3): a set of k ≥ 2 distinct points, kept
/// with the centre of SEC(P̂) at the origin and radius 1, plus what every
/// robot derives from it alone.
#[derive(Clone, Debug, PartialEq)]
pub struct Pattern {
    /// P̂, normalised.
    pub points: Vec<Point>,
    /// Every realisation of μ̂, the lexicographically smallest pattern sequence
    /// (Section 3.3): (index of p̂ᵢ, the first pattern point on SEC(P̂) in the
    /// cyclic order ψ(P̂) it induces; direction of that order, ±1).
    pub anchors: Vec<(usize, f64)>,
    /// Pairs of pattern points at the maximum distance (Algorithm 7).
    pub max_pairs: Vec<(usize, usize)>,
}

impl Pattern {
    /// # Errors
    /// Fewer than two points, a point that is not finite, or two equal points.
    pub fn new(given: &[Point]) -> Result<Self, String> {
        if given.len() < 2 {
            return Err(format!(
                "SqPF forms patterns of at least 2 points (UPF*, Section 2.3); got {}. \
                 A single point is Gathering: use SqGathering",
                given.len()
            ));
        }
        if given.iter().any(|p| !p.x.is_finite() || !p.y.is_finite()) {
            return Err("pattern points must be finite numbers".to_owned());
        }
        let (c, r) = sec(given);
        let points: Vec<Point> = given
            .iter()
            .map(|p| Point::new((p.x - c.x) / r, (p.y - c.y) / r))
            .collect();
        for i in 0..points.len() {
            for j in 0..i {
                if dist(points[i], points[j]) <= 1e-9 {
                    return Err(format!(
                        "pattern points {j} and {i} are the same point: a pattern is a set of \
                         distinct points (Section 2.3)"
                    ));
                }
            }
        }
        let anchors = mu_hat(&points);
        let mut max = 0.0_f64;
        for i in 0..points.len() {
            for j in 0..i {
                max = max.max(dist(points[i], points[j]));
            }
        }
        let mut max_pairs = Vec::new();
        for i in 0..points.len() {
            for j in i + 1..points.len() {
                if dist(points[i], points[j]) >= max - 1e-9 {
                    max_pairs.push((i, j));
                }
            }
        }
        Ok(Self {
            points,
            anchors,
            max_pairs,
        })
    }

    /// k = |P̂|.
    #[must_use]
    pub fn len(&self) -> usize {
        self.points.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// τ(P̂): P̂ scaled so that SEC(P̂) has radius ρ, centred on O, reflected
    /// if `mirror`, and rotated so that pattern point `anchor` lies on `at`.
    fn place(&self, anchor: usize, mirror: bool, at: Point, o: Point, rho: f64) -> Vec<Point> {
        let m = |p: Point| if mirror { Point::new(p.x, -p.y) } else { p };
        let phi = angle_of(sub(at, o)) - angle_of(m(self.points[anchor]));
        let (s, c) = (libm::sin(phi), libm::cos(phi));
        self.points
            .iter()
            .map(|&p| {
                let p = m(p);
                Point::new(o.x + rho * (c * p.x - s * p.y), o.y + rho * (s * p.x + c * p.y))
            })
            .collect()
    }
}

/// μ̂ (Section 3.3): the lexicographically smallest of the clockwise and
/// counterclockwise pattern sequences α⁽ⁱ⁾(P̂), β⁽ⁱ⁾(P̂), sequences of triples
/// (θ(p̂ᵢ, p̂ᵢ₊₁), |Op̂ᵢ|, |Op̂ᵢ₊₁|) around the centre O of SEC(P̂). Returns
/// every realisation as (p̂ᵢ, direction): p̂ᵢ is the first point on SEC(P̂)
/// in the cyclic order ψ(P̂) the realisation starts.
///
/// ψ(P̂) lists the radii in angular order and, on one radius, the co-radial
/// points from the centre outwards (θ = 0 between them), in both directions.
/// A point at O has no angle and is left out of the sequence.
fn mu_hat(pts: &[Point]) -> Vec<(usize, f64)> {
    let idx: Vec<usize> = (0..pts.len()).filter(|&i| norm(pts[i]) > REL_TOL).collect();
    // Radii (rays) in counterclockwise order, each with its points by distance.
    let mut by_angle = idx.clone();
    by_angle.sort_by(|&a, &b| {
        angle_of(pts[a])
            .partial_cmp(&angle_of(pts[b]))
            .unwrap_or(Ordering::Equal)
            .then(norm(pts[a]).partial_cmp(&norm(pts[b])).unwrap_or(Ordering::Equal))
    });
    let mut rays: Vec<(f64, Vec<usize>)> = Vec::new();
    for i in by_angle {
        let a = angle_of(pts[i]);
        match rays.last_mut() {
            Some((start, members)) if a - *start <= ANG_TOL => members.push(i),
            _ => rays.push((a, vec![i])),
        }
    }
    if rays.len() > 1 {
        let first = rays[0].0;
        let last = rays[rays.len() - 1].0;
        if first + TAU - last <= ANG_TOL {
            let (_, mut tail) = rays.pop().unwrap_or_default();
            tail.extend(rays[0].1.iter().copied());
            rays[0].1 = tail;
        }
    }
    for (_, members) in &mut rays {
        members.sort_by(|&a, &b| norm(pts[a]).partial_cmp(&norm(pts[b])).unwrap_or(Ordering::Equal));
    }
    let m = rays.len();
    let mut ray_of = vec![0usize; pts.len()];
    for (r, (_, members)) in rays.iter().enumerate() {
        for &i in members {
            ray_of[i] = r;
        }
    }

    let mut best: Option<Vec<(f64, f64, f64)>> = None;
    let mut found: Vec<(usize, f64)> = Vec::new();
    for dir in [1.0, -1.0] {
        let listing: Vec<usize> = (0..m)
            .map(|t| if dir > 0.0 { t } else { (m - t) % m })
            .flat_map(|r| rays[r].1.iter().copied())
            .collect();
        let kk = listing.len();
        let triple = |t: usize| {
            let (a, b) = (listing[t % kk], listing[(t + 1) % kk]);
            let gap = if ray_of[a] == ray_of[b] && m > 1 {
                0.0
            } else {
                sweep(rays[ray_of[a]].0, rays[ray_of[b]].0, dir)
            };
            (gap, norm(pts[a]), norm(pts[b]))
        };
        for s in 0..kk {
            let seq: Vec<(f64, f64, f64)> = (0..kk).map(|t| triple(s + t)).collect();
            let order = match &best {
                None => Ordering::Less,
                Some(b) => lex(&seq, b),
            };
            let anchor = (0..kk)
                .map(|t| listing[(s + t) % kk])
                .find(|&i| norm(pts[i]) >= 1.0 - 1e-9)
                .unwrap_or(listing[s]);
            match order {
                Ordering::Less => {
                    best = Some(seq);
                    found.clear();
                    found.push((anchor, dir));
                }
                Ordering::Equal => found.push((anchor, dir)),
                Ordering::Greater => {}
            }
        }
    }
    found
}

fn lex(a: &[(f64, f64, f64)], b: &[(f64, f64, f64)]) -> Ordering {
    for (x, y) in a.iter().zip(b) {
        let o = cmp_tol(x.0, y.0, ANG_TOL)
            .then(cmp_tol(x.1, y.1, 1e-9))
            .then(cmp_tol(x.2, y.2, 1e-9));
        if o != Ordering::Equal {
            return o;
        }
    }
    Ordering::Equal
}

// ── finding points fast ───────────────────────────────────────────────────────

/// A set of points sorted by x, as indices into the caller's slice (nothing is
/// copied), answering "is a point within `tol` of p?" in O(log n) instead of
/// a scan of every point. `dist(a, b) ≤ tol` implies |a.x − b.x| ≤ tol, so
/// only the points in the x-window [p.x − 2·tol, p.x + 2·tol] can match (the
/// factor 2 absorbs rounding); each of them is then tested with the same
/// `dist(·, ·) ≤ tol` as a full scan, so the answer is exactly the scan's.
struct PointIndex<'a> {
    pts: &'a [Point],
    by_x: Vec<u32>,
}

impl<'a> PointIndex<'a> {
    #[allow(clippy::cast_possible_truncation)]
    fn new(pts: &'a [Point]) -> Self {
        let mut by_x: Vec<u32> = (0..pts.len() as u32).collect();
        by_x.sort_unstable_by(|&a, &b| pts[a as usize].x.total_cmp(&pts[b as usize].x));
        Self { pts, by_x }
    }

    /// Some point of the set within `tol` of `p` (`dist ≤ tol`).
    fn any_within(&self, p: Point, tol: f64) -> bool {
        let (lo, hi) = (p.x - 2.0 * tol, p.x + 2.0 * tol);
        let start = self.by_x.partition_point(|&i| self.pts[i as usize].x < lo);
        self.by_x[start..]
            .iter()
            .map(|&i| self.pts[i as usize])
            .take_while(|x| x.x <= hi)
            .any(|x| dist(x, p) <= tol)
    }
}

/// Ordered key of an x-coordinate for [`snapshot`]'s tree (−0 and +0 are one key).
fn x_key(x: f64) -> u64 {
    let b = (x + 0.0).to_bits();
    if b >> 63 == 1 {
        !b
    } else {
        b | (1 << 63)
    }
}

// ── what one robot sees ───────────────────────────────────────────────────────

/// The Look phase (Section 2.1): the set Q of distinct points occupied by
/// robots, "expressed within a private coordinate system" of which the robot
/// is the origin. Positions closer than 10⁻⁹ of the snapshot's size, or closer
/// than `min_move` (world units: the simulator's minimum move), are one point
/// (no multiplicity detection: the count is not kept). Sorted by the robot's
/// own coordinates, so nothing of the robots' identities survives.
#[must_use]
pub fn snapshot(world: &[Point], frame: &LocalFrame, min_move: f64) -> Vec<Point> {
    let local: Vec<Point> = world.iter().map(|&p| frame.to_local(p)).collect();
    let span = local.iter().map(|&p| norm(p)).fold(0.0, f64::max);
    let tol = (REL_TOL * span).max(min_move * frame.scale);
    // The first robot seen on a point stands for it, as in a scan of the points
    // kept so far; the tree keeps them by x, so each look is O(log n), not O(n).
    let mut q: Vec<Point> = Vec::with_capacity(local.len() + 1);
    let mut kept: BTreeMap<(u64, u32), ()> = BTreeMap::new();
    #[allow(clippy::cast_possible_truncation)]
    for p in core::iter::once(Point::new(0.0, 0.0)).chain(local) {
        let (lo, hi) = (x_key(p.x - 2.0 * tol), x_key(p.x + 2.0 * tol));
        let taken = kept
            .range((lo, 0)..=(hi, u32::MAX))
            .any(|(&(_, i), ())| dist(q[i as usize], p) <= tol);
        if !taken {
            kept.insert((x_key(p.x), q.len() as u32), ());
            q.push(p);
        }
    }
    q.sort_by(|a, b| {
        a.x.partial_cmp(&b.x)
            .unwrap_or(Ordering::Equal)
            .then(a.y.partial_cmp(&b.y).unwrap_or(Ordering::Equal))
    });
    q
}

/// Which line of the paper's pseudocode decided the move.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Step {
    /// Algorithm 1 l. 2–3 / Algorithm 7 l. 3–4: |Q| < |P̂|, Procedure Separate.
    Separate,
    /// Procedure Last l. 4–5: Q ∈ 𝓑(P̂), the pattern is formed; stay.
    Formed,
    /// Procedure Last l. 7–8: the "last one" case; move toward p̂ₖ.
    Finalize,
    /// Procedure Last, "last one" case, robot not on Op̂ₖ: stay (Lemma 2).
    FinalizeWait,
    /// Procedure Leader l. 2–3: responsible for SEC(Q), or no free path to O.
    LeaderHold,
    /// Procedure Leader l. 4–5: defines θ₁ and has no co-radial robot.
    LeaderKeepAngle,
    /// Procedure Leader l. 6–7: move toward O.
    LeaderToCenter,
    /// Procedure Leader l. 9–17: the robot at O moves to u′.
    LeaderToU,
    /// Procedure Leader l. 18–19: a robot is at O and it is not this one.
    LeaderWait,
    /// Procedure Occupy l. 19–21: the walker, co-radial with pₗ, moves to it.
    OccupyMove,
    /// Procedure Occupy l. 22–23: the walker moves to O first (radial detour).
    OccupyToCenter,
    /// Procedure Occupy l. 24–25: not the walker.
    OccupyWait,
    /// Algorithm 7 l. 7–10: on a non-unique maximum pair, step away one unit.
    PrimeStretch,
    /// Algorithm 7 l. 13–17: |Q| > |P̂|, move to the closer of q₁, q₂.
    PrimeEqualize,
    /// Algorithm 7 l. 18–26: |Q| = |P̂|, move to the pattern point pᵢ.
    PrimeOccupy,
    /// Algorithm 7 l. 2: destination ← q, no other line applies.
    PrimeStay,
    /// Not in Algorithm 7, see [`sqpf_prime`]: Q ∈ 𝓑(P̂), stay.
    PrimeFormed,
}

impl Step {
    /// Lines of the paper that decide this step.
    #[must_use]
    pub const fn lines(self) -> &'static str {
        match self {
            Self::Separate => "Alg. 1 l. 2-3, Proc. 2 (Alg. 7 l. 3-4)",
            Self::Formed => "Proc. 3 l. 4-5",
            Self::Finalize => "Proc. 3 l. 7-8",
            Self::FinalizeWait => "Proc. 3 l. 7 (Lemma 2: the others stay)",
            Self::LeaderHold => "Proc. 5 l. 2-3",
            Self::LeaderKeepAngle => "Proc. 5 l. 4-5",
            Self::LeaderToCenter => "Proc. 5 l. 6-7",
            Self::LeaderToU => "Proc. 5 l. 9-17",
            Self::LeaderWait => "Proc. 5 l. 18-19",
            Self::OccupyMove => "Proc. 6 l. 19-21",
            Self::OccupyToCenter => "Proc. 6 l. 22-23",
            Self::OccupyWait => "Proc. 6 l. 24-25",
            Self::PrimeStretch => "Alg. 7 l. 6-10",
            Self::PrimeEqualize => "Alg. 7 l. 12-17",
            Self::PrimeOccupy => "Alg. 7 l. 18-26",
            Self::PrimeStay => "Alg. 7 l. 2",
            Self::PrimeFormed => "added: formed pattern stays (Sec. 2.3 predicate)",
        }
    }
}

/// One outermost point per radius of S′ (Section 3.3).
#[derive(Clone, Copy, Debug)]
struct Ray {
    at: Point,
    robot: bool,
    angle: f64,
}

/// A leader angular sequence λ (Section 3.3): θ₁ is spanned by `s_in`
/// (inside SEC) and `q_j` (on SEC); turning from `s_in` to `q_j` through θ₁
/// is "clockwise", which is direction `dir` in the robot's own frame.
#[derive(Clone, Copy, Debug)]
struct LeaderSeq {
    s_in: Point,
    q_j: Point,
    dir: f64,
}

/// The joint configuration Γ (Procedure Overlap).
struct Gamma {
    placed: Vec<Point>,
    lam: LeaderSeq,
}

/// One activation's geometry: Q, O, ρ, all in the robot's own frame.
struct Scene<'a> {
    q: Point,
    pts: &'a [Point],
    o: Point,
    rho: f64,
    tol: f64,
    /// Q sorted by x, for [`Scene::occupied`].
    index: PointIndex<'a>,
}

impl<'a> Scene<'a> {
    fn new(q: Point, pts: &'a [Point], o: Point, rho: f64, tol: f64) -> Self {
        Self {
            q,
            pts,
            o,
            rho,
            tol,
            index: PointIndex::new(pts),
        }
    }

    fn at(&self, a: Point, b: Point) -> bool {
        dist(a, b) <= self.tol
    }

    fn on_sec(&self, p: Point) -> bool {
        dist(p, self.o) >= self.rho - self.tol
    }

    fn at_center(&self, p: Point) -> bool {
        dist(p, self.o) <= self.tol
    }

    fn occupied(&self, p: Point) -> bool {
        self.index.any_within(p, self.tol)
    }

    /// The robot positions not located at a point of `placed` (`placed` indexed).
    fn off(&self, placed: &PointIndex) -> Vec<Point> {
        self.pts
            .iter()
            .copied()
            .filter(|&x| !placed.any_within(x, self.tol))
            .collect()
    }

    /// Two points on the same radius of SEC (footnote 2); O is on every radius.
    fn coradial(&self, a: Point, b: Point) -> bool {
        if self.at_center(a) || self.at_center(b) {
            return true;
        }
        let (va, vb) = (sub(a, self.o), sub(b, self.o));
        let cross = va.x * vb.y - va.y * vb.x;
        let dot = va.x * vb.x + va.y * vb.y;
        dot > 0.0 && cross.abs() <= ANG_TOL * norm(va) * norm(vb)
    }

    fn on_segment(&self, p: Point, a: Point, b: Point) -> bool {
        let ab = sub(b, a);
        let len2 = ab.x * ab.x + ab.y * ab.y;
        if len2 == 0.0 {
            return self.at(p, a);
        }
        let ap = sub(p, a);
        let t = ((ap.x * ab.x + ap.y * ab.y) / len2).clamp(0.0, 1.0);
        self.at(p, Point::new(a.x + t * ab.x, a.y + t * ab.y))
    }

    /// Robot positions on segment ab, other than a and b themselves.
    fn robots_on(&self, a: Point, b: Point) -> Vec<Point> {
        self.pts
            .iter()
            .copied()
            .filter(|&p| !self.at(p, a) && !self.at(p, b) && self.on_segment(p, a, b))
            .collect()
    }

    /// Robot positions on the radial detour from `a` to `b` (Section 3.3),
    /// other than `a` and `b`.
    fn robots_on_detour(&self, a: Point, b: Point) -> Vec<Point> {
        if self.coradial(a, b) {
            return self.robots_on(a, b);
        }
        let mut v = self.robots_on(a, self.o);
        for p in self.robots_on(self.o, b) {
            if !v.iter().any(|&x| self.at(x, p)) {
                v.push(p);
            }
        }
        if self.occupied(self.o) && !self.at(a, self.o) && !self.at(b, self.o) {
            v.push(self.o);
        }
        v
    }

    /// `p` is responsible for SEC(Q): without it, the SEC would change.
    fn responsible(&self, p: Point) -> bool {
        if !self.on_sec(p) {
            return false;
        }
        let others: Vec<Point> = self.pts.iter().copied().filter(|&x| !self.at(x, p)).collect();
        if others.is_empty() {
            return true;
        }
        let (c, r) = sec(&others);
        r < self.rho - self.tol || dist(c, self.o) > self.tol
    }

    /// Radiangular distance from `a` to `b` (Section 3.3), clockwise = `dir`.
    fn radiangular(&self, a: Point, b: Point, dir: f64) -> (f64, f64) {
        if self.coradial(a, b) {
            (0.0, dist(a, b))
        } else {
            (
                sweep(angle_of(sub(a, self.o)), angle_of(sub(b, self.o)), dir),
                dist(self.o, a) + dist(self.o, b),
            )
        }
    }

    /// S′ (Section 3.3): co-radial points reduced to the most external one;
    /// points at O have no radius. `items` are (point, is a robot position).
    fn rays(&self, items: &[(Point, bool)]) -> Vec<Ray> {
        let mut v: Vec<(f64, f64, Point, bool)> = items
            .iter()
            .filter(|(p, _)| !self.at_center(*p))
            .map(|&(p, robot)| (angle_of(sub(p, self.o)), dist(p, self.o), p, robot))
            .collect();
        v.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal));
        let mut groups: Vec<Vec<(f64, f64, Point, bool)>> = Vec::new();
        for e in v {
            match groups.last_mut() {
                Some(g) if e.0 - g[0].0 <= ANG_TOL => g.push(e),
                _ => groups.push(vec![e]),
            }
        }
        if groups.len() > 1 {
            let first = groups[0][0].0;
            let last = groups[groups.len() - 1][0].0;
            if first + TAU - last <= ANG_TOL {
                let tail = groups.pop().unwrap_or_default();
                groups[0].extend(tail);
            }
        }
        groups
            .into_iter()
            .map(|g| {
                let rmax = g.iter().map(|e| e.1).fold(0.0, f64::max);
                let outer: Vec<_> = g.iter().filter(|e| e.1 >= rmax - self.tol).collect();
                let pick = outer.iter().find(|e| e.3).unwrap_or(&outer[0]);
                Ray {
                    at: pick.2,
                    robot: outer.iter().any(|e| e.3),
                    angle: g[0].0,
                }
            })
            .collect()
    }

    /// The angular sequence θ(S′): gap i goes from ray i to ray i+1, counterclockwise.
    fn gaps(rays: &[Ray]) -> Vec<f64> {
        let m = rays.len();
        if m < 2 {
            return vec![TAU];
        }
        (0..m)
            .map(|i| sweep(rays[i].angle, rays[(i + 1) % m].angle, 1.0))
            .collect()
    }

    /// λ: a leader angular sequence of S′, if it admits one (Section 3.3).
    fn leader_seq(&self, rays: &[Ray]) -> Option<LeaderSeq> {
        let m = rays.len();
        if m <= 2 {
            return None;
        }
        let g = Self::gaps(rays);
        let (i, &min) = g
            .iter()
            .enumerate()
            .min_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(Ordering::Equal))?;
        if g.iter().enumerate().any(|(j, &x)| j != i && x <= min + ANG_TOL) {
            return None; // (i) θ₁ is not the unique smallest angle
        }
        let (a, b) = (rays[i], rays[(i + 1) % m]);
        if !(a.robot && b.robot) {
            return None; // (ii) both points must be robot positions
        }
        match (self.on_sec(a.at), self.on_sec(b.at)) {
            (false, true) => Some(LeaderSeq { s_in: a.at, q_j: b.at, dir: 1.0 }),
            (true, false) => Some(LeaderSeq { s_in: b.at, q_j: a.at, dir: -1.0 }),
            _ => None, // (ii) one inside SEC(Q), the other on it
        }
    }

    fn robot_items(&self) -> Vec<(Point, bool)> {
        self.pts.iter().map(|&p| (p, true)).collect()
    }

    fn gamma_items(&self, placed: &[Point]) -> Vec<(Point, bool)> {
        let mut items = self.robot_items();
        items.extend(placed.iter().map(|&p| (p, false)));
        items
    }

    /// The ranking of the pattern points (Section 4.1.6), from p₁ (at A) to
    /// pₖ, with "clockwise" = `dir`.
    fn ranking(&self, placed: &[Point], a: Point, dir: f64) -> Option<Vec<usize>> {
        let o = self.o;
        // Each point's distance from O and clockwise angle from A, computed once
        // (not inside the sort's comparisons); the orders are the same.
        let radius: Vec<f64> = placed.iter().map(|&p| dist(p, o)).collect();
        let r = |i: usize| radius[i];
        let mut idx: Vec<usize> = (0..placed.len()).collect();
        idx.sort_by(|&x, &y| r(y).partial_cmp(&r(x)).unwrap_or(Ordering::Equal));
        // Cir₁, Cir₂, …, Cir_σ: concentric circles in decreasing order of radius.
        let mut circles: Vec<Vec<usize>> = Vec::new();
        for i in idx {
            match circles.last_mut() {
                Some(c) if (r(c[0]) - r(i)).abs() <= self.tol => c.push(i),
                _ => circles.push(vec![i]),
            }
        }
        let a_ang = angle_of(sub(a, o));
        let cw_of: Vec<f64> = placed
            .iter()
            .map(|&p| {
                if self.at_center(p) {
                    0.0
                } else {
                    sweep(a_ang, angle_of(sub(p, o)), dir)
                }
            })
            .collect();
        let cw = |i: usize| cw_of[i];
        let by_cw = |x: &usize, y: &usize| cw(*x).partial_cmp(&cw(*y)).unwrap_or(Ordering::Equal);

        let c1 = &circles[0];
        let p1 = *c1.iter().find(|&&i| self.at(placed[i], a))?;
        let mut order = vec![p1];
        let mut rest: Vec<usize> = c1.iter().copied().filter(|&i| i != p1).collect();
        let anti = Point::new(2.0 * o.x - a.x, 2.0 * o.y - a.y);
        if let Some(pos) = rest.iter().position(|&i| self.at(placed[i], anti)) {
            order.push(rest.remove(pos)); // p₂ is the antipodal point p₁′
        } else {
            // p₂, p₃: the closest to p₁′ going from p₁′ to p₁, counterclockwise and clockwise.
            let anti_ang = angle_of(sub(anti, o));
            for turn in [-dir, dir] {
                if let Some((pos, _)) = rest.iter().enumerate().min_by(|x, y| {
                    let ax = sweep(anti_ang, angle_of(sub(placed[*x.1], o)), turn);
                    let ay = sweep(anti_ang, angle_of(sub(placed[*y.1], o)), turn);
                    ax.partial_cmp(&ay).unwrap_or(Ordering::Equal)
                }) {
                    order.push(rest.remove(pos));
                }
            }
        }
        rest.sort_by(by_cw);
        order.extend(rest);
        for c in &circles[1..] {
            let mut c = c.clone();
            c.sort_by(by_cw);
            order.extend(c);
        }
        Some(order)
    }

    fn same_set(&self, placed: &[Point]) -> bool {
        placed.len() == self.pts.len()
            && placed.iter().all(|&p| self.occupied(p))
            && {
                let index = PointIndex::new(placed);
                self.pts.iter().all(|&x| index.any_within(x, self.tol))
            }
    }
}

// ── Algorithm 1 (SqPF) and its procedures ──────────────────────────────────────

/// Procedure 2, Separate: a point at one unit along the robot's own x-axis,
/// or the midpoint towards the closest robot on that segment.
fn separate(pts: &[Point], q: Point, min_move: f64) -> Point {
    let span = pts.iter().map(|&p| dist(p, q)).fold(0.0, f64::max);
    let scale = span.max(1.0);
    let sc = Scene::new(q, pts, q, scale, (REL_TOL * scale).max(min_move));
    let q1 = Point::new(q.x + 1.0, q.y); // l. 1: q′, one unit along the x-axis
    let on = sc.robots_on(q, q1);
    let on: Vec<Point> = on
        .into_iter()
        .chain(pts.iter().copied().filter(|&p| sc.at(p, q1)))
        .collect();
    match on
        .iter()
        .min_by(|a, b| dist(**a, q).partial_cmp(&dist(**b, q)).unwrap_or(Ordering::Equal))
    {
        None => q1,                                                    // l. 2-3
        Some(&q2) => Point::new(0.5 * (q.x + q2.x), 0.5 * (q.y + q2.y)), // l. 5-6
    }
}

/// Procedure 3, Last.
fn last(pat: &Pattern, sc: &Scene) -> Option<(Point, Step)> {
    let on_sec: Vec<Point> = sc.pts.iter().copied().filter(|&p| sc.on_sec(p)).collect();
    // "the case when SEC(Q) contains more than k robot positions is not handled by this procedure"
    if on_sec.len() > pat.len() {
        return None;
    }
    // l. 2: the ≤ 2k transformations overlapping p̂ᵢ with a robot on SEC(Q).
    let mut taus: Vec<(Vec<Point>, Point, f64)> = Vec::new();
    for &qi in &on_sec {
        for &(anchor, d) in &pat.anchors {
            for mirror in [false, true] {
                let placed = pat.place(anchor, mirror, qi, sc.o, sc.rho);
                taus.push((placed, qi, if mirror { -d } else { d }));
            }
        }
    }
    // l. 4-5: Q ∈ 𝓑(P̂): do not move.
    if taus.iter().any(|(placed, _, _)| sc.same_set(placed)) {
        return Some((sc.q, Step::Formed));
    }
    // l. 7-8: the "last one" cases. The configuration is a special case when
    // some τ leaves only p̂ₖ empty (or none) with every other robot on Op̂ₖ;
    // then the robot on Op̂ₖ moves toward p̂ₖ and every other robot stays.
    // Line 7 also asks q ∈ Op̂ₖ, so read literally a robot elsewhere would get
    // null and leave its pattern point through Overlap/Leader, undoing the
    // Finalization; the text ("Once a robot located at Op̂ₖ is activated, it
    // moves toward p̂ₖ"), Fig. 1 (Special case? → Last → END) and the proof of
    // Lemma 2 keep the others still. See docs/sqpf.md, choice 3.
    for (placed, a, dir) in &taus {
        // P̂ \ Q may hold p̂ₖ at most: stop at the second empty pattern point
        // (the ranking is needed only when at most one is empty).
        let mut empty = placed.iter().copied().filter(|&p| !sc.occupied(p));
        let missing = empty.next();
        if empty.next().is_some() {
            continue;
        }
        let Some(rank) = sc.ranking(placed, *a, *dir) else {
            continue;
        };
        let pk = placed[rank[rank.len() - 1]]; // l. 3: lowest priority pattern point
        if missing.is_some_and(|m| !sc.at(m, pk)) {
            continue;
        }
        let extra = sc.off(&PointIndex::new(placed));
        let on_seg: Vec<Point> = sc
            .pts
            .iter()
            .copied()
            .filter(|&x| sc.on_segment(x, sc.o, pk) && !sc.at(x, pk))
            .collect();
        let same = extra.len() == on_seg.len()
            && extra.iter().all(|&x| on_seg.iter().any(|&y| sc.at(x, y)));
        if same {
            if sc.on_segment(sc.q, sc.o, pk) {
                return Some((pk, Step::Finalize));
            }
            return Some((sc.q, Step::FinalizeWait));
        }
    }
    None
}

/// Procedure 4, Overlap.
fn overlap(pat: &Pattern, sc: &Scene) -> Option<Gamma> {
    let lam = sc.leader_seq(&sc.rays(&sc.robot_items()))?; // l. 2
    // l. 1, 3-4: p̂ᵢ on q_j, the order of μ̂ following λ(Q)'s clockwise direction.
    let (anchor, d) = pat.anchors[0];
    let mirror = (d > 0.0) != (lam.dir > 0.0);
    Some(Gamma {
        placed: pat.place(anchor, mirror, lam.q_j, sc.o, sc.rho),
        lam,
    })
}

/// Procedure 5, Leader.
fn leader(sc: &Scene, gamma: Option<&Gamma>) -> (Point, Step) {
    let q = sc.q;
    if !sc.occupied(sc.o) {
        // l. 2-3
        if sc.responsible(q) || !sc.robots_on(sc.o, q).is_empty() {
            return (q, Step::LeaderHold);
        }
        // l. 4-5
        if let Some(g) = gamma {
            let defines = sc.at(q, g.lam.s_in) || sc.at(q, g.lam.q_j);
            let coradial = sc.pts.iter().any(|&p| !sc.at(p, q) && sc.coradial(p, q));
            if defines && !coradial {
                return (q, Step::LeaderKeepAngle);
            }
        }
        return (sc.o, Step::LeaderToCenter); // l. 6-7
    }
    if sc.at_center(q) {
        let (qj, xi, side) = match gamma {
            // l. 10-12: any robot on SEC(Q); ξ the smallest angle of S′(Q).
            None => {
                let qj = sc.pts.iter().copied().find(|&p| sc.on_sec(p)).unwrap_or(q);
                let rays = sc.rays(&sc.robot_items());
                let xi = Scene::gaps(&rays).into_iter().fold(TAU, f64::min);
                (qj, xi, 1.0)
            }
            // l. 13-15: q_j of θ₁; ξ the smallest angle of S′(Γ). u′ goes on the side
            // that keeps Γ's clockwise direction (see docs/sqpf.md, choice 5).
            Some(g) => {
                let rays = sc.rays(&sc.gamma_items(&g.placed));
                let xi = Scene::gaps(&rays).into_iter().fold(TAU, f64::min);
                (g.lam.q_j, xi, -g.lam.dir)
            }
        };
        // l. 16-17: |Ou′| = ρ/2 and ∠u′Oq_j = ξ/3.
        let a = angle_of(sub(qj, sc.o)) + side * xi / 3.0;
        return (polar(sc.o, 0.5 * sc.rho, a), Step::LeaderToU);
    }
    (q, Step::LeaderWait) // l. 18-19
}

/// Procedure 6, Occupy (with the walker of Section 4.1.7).
fn occupy(sc: &Scene, g: &Gamma) -> (Point, Step) {
    let (a, dir) = (g.lam.q_j, g.lam.dir);
    let placed = &g.placed;
    let Some(rank) = sc.ranking(placed, a, dir) else {
        return (sc.q, Step::OccupyWait);
    };
    // l. 1-5
    let pk = placed[rank[rank.len() - 1]];
    let pl = rank
        .iter()
        .map(|&i| placed[i])
        .find(|&p| !sc.occupied(p))
        .unwrap_or(pk);
    // Free robots: not located at a pattern point.
    let free = sc.off(&PointIndex::new(placed));
    // Priority (Section 4.1.7): radiangular distance from p₁ = A.
    // Each robot's radiangular distance is computed once, not in every comparison.
    let higher = |(tx, dx): (f64, f64), (ty, dy): (f64, f64)| -> Ordering {
        match cmp_tol(tx, ty, ANG_TOL) {
            Ordering::Equal if tx <= ANG_TOL => cmp_tol(dx, dy, sc.tol),
            Ordering::Equal => cmp_tol(dy, dx, sc.tol),
            o => o,
        }
    };
    // l. 6: the highest priority free robot not defining SEC(Q(t)).
    let Some((u, _)) = free
        .iter()
        .copied()
        .filter(|&x| !sc.responsible(x))
        .map(|x| (x, sc.radiangular(a, x, dir)))
        .min_by(|x, y| higher(x.1, y.1))
    else {
        return (sc.q, Step::OccupyWait);
    };
    let detour = sc.robots_on_detour(u, pl);
    // ρₗ: the radius of SEC through pₗ.
    let rim = if sc.at_center(pl) {
        sc.o
    } else {
        polar(sc.o, sc.rho, angle_of(sub(pl, sc.o)))
    };
    let on_radius = |x: Point| sc.at(x, sc.o) || sc.on_segment(x, sc.o, rim);
    let radius_others: Vec<Point> = free
        .iter()
        .copied()
        .filter(|&x| !sc.at(x, u) && !sc.at(x, pl) && on_radius(x))
        .collect();
    let w = if detour.is_empty() && radius_others.is_empty() {
        u // l. 7-8
    } else if radius_others.is_empty() {
        // l. 9-11: the robot on the detour with the smallest radiangular distance from pₗ.
        detour
            .iter()
            .copied()
            .min_by(|x, y| {
                let (tx, dx) = sc.radiangular(pl, *x, dir);
                let (ty, dy) = sc.radiangular(pl, *y, dir);
                cmp_tol(tx, ty, ANG_TOL).then(cmp_tol(dx, dy, sc.tol))
            })
            .unwrap_or(u)
    } else {
        // l. 13-18: along ρₗ from the SEC towards O, the first free robot v.
        let v = free
            .iter()
            .copied()
            .filter(|&x| !sc.at(x, pl) && on_radius(x))
            .max_by(|x, y| {
                dist(*x, sc.o)
                    .partial_cmp(&dist(*y, sc.o))
                    .unwrap_or(Ordering::Equal)
            })
            .unwrap_or(u);
        let between = sc.robots_on(v, pl);
        between
            .iter()
            .copied()
            .min_by(|x, y| dist(*x, pl).partial_cmp(&dist(*y, pl)).unwrap_or(Ordering::Equal))
            .unwrap_or(v)
    };
    // l. 19-25
    if sc.at(sc.q, w) {
        if sc.coradial(sc.q, pl) {
            (pl, Step::OccupyMove)
        } else {
            (sc.o, Step::OccupyToCenter)
        }
    } else {
        (sc.q, Step::OccupyWait)
    }
}

/// Algorithm 1, SqPF (|P̂| ≥ 5), after the |Q| < k test.
fn sqpf(pat: &Pattern, sc: &Scene) -> (Point, Step) {
    // l. 5: P̂ is scaled to ρ inside `Pattern::place`.
    if let Some(res) = last(pat, sc) {
        return res; // l. 6-8
    }
    match overlap(pat, sc) {
        None => leader(sc, None), // l. 10-12
        Some(g) => {
            let has = sc.leader_seq(&sc.rays(&sc.gamma_items(&g.placed))).is_some();
            if has {
                occupy(sc, &g) // l. 14-15
            } else {
                leader(sc, Some(&g)) // l. 16-17
            }
        }
    }
}

// ── Algorithm 7 (SqPF′) ───────────────────────────────────────────────────────

/// The similarity mapping pattern points `pa`, `pb` onto `x`, `y`, reflected if `mirror`.
fn map_pair(pat: &Pattern, pa: usize, pb: usize, x: Point, y: Point, mirror: bool) -> Vec<Point> {
    let m = |p: Point| if mirror { Point::new(p.x, -p.y) } else { p };
    let (a, b) = (m(pat.points[pa]), m(pat.points[pb]));
    let s = dist(x, y) / dist(a, b);
    let phi = angle_of(sub(y, x)) - angle_of(sub(b, a));
    let (sn, cs) = (libm::sin(phi), libm::cos(phi));
    pat.points
        .iter()
        .map(|&p| {
            let d = sub(m(p), a);
            Point::new(x.x + s * (cs * d.x - sn * d.y), x.y + s * (sn * d.x + cs * d.y))
        })
        .collect()
}

/// Algorithm 7, SqPF′ (1 < |P̂| < 5), after the |Q| < k test.
///
/// One addition to the paper's pseudocode: when Q is already similar to P̂ the
/// robot stays. Algorithm 7 has no such test, and a pattern with two or three
/// pairs at the maximum distance (a square, a rectangle, an equilateral
/// triangle) has no unique maximum pair once formed, so line 7 would move it
/// again, against the predicate of Section 2.3 ("and to no longer move").
fn sqpf_prime(pat: &Pattern, sc: &Scene) -> (Point, Step) {
    let q = sc.q;
    let k = pat.len();
    if sc.pts.len() == k {
        for (i, &x) in sc.pts.iter().enumerate() {
            for (j, &y) in sc.pts.iter().enumerate() {
                for mirror in [false, true] {
                    if i != j && sc.same_set(&map_pair(pat, 0, 1, x, y, mirror)) {
                        return (q, Step::PrimeFormed);
                    }
                }
            }
        }
    }
    // l. 6: the pairs at the maximum distance in Q.
    let mut dmax = 0.0_f64;
    for (i, &x) in sc.pts.iter().enumerate() {
        for &y in &sc.pts[..i] {
            dmax = dmax.max(dist(x, y));
        }
    }
    let mut pairs = Vec::new();
    for (i, &x) in sc.pts.iter().enumerate() {
        for &y in &sc.pts[i + 1..] {
            if dist(x, y) >= dmax - sc.tol {
                pairs.push((x, y));
            }
        }
    }
    if pairs.len() != 1 {
        // l. 7-10
        if pairs.iter().any(|&(x, y)| sc.at(q, x) || sc.at(q, y)) {
            let far = sc
                .pts
                .iter()
                .copied()
                .find(|&p| dist(p, q) >= dmax - sc.tol)
                .unwrap_or(q);
            let away = sub(q, far);
            let l = norm(away);
            return (
                Point::new(q.x + away.x / l, q.y + away.y / l),
                Step::PrimeStretch,
            );
        }
        return (q, Step::PrimeStay);
    }
    let (q1, q2) = pairs[0]; // l. 12
    if sc.at(q, q1) || sc.at(q, q2) {
        return (q, Step::PrimeStay);
    }
    if sc.pts.len() > k {
        // l. 13-17
        let target = if dist(q, q2) < dist(q, q1) - sc.tol { q2 } else { q1 };
        if sc.robots_on(q, target).is_empty() {
            return (target, Step::PrimeEqualize);
        }
        return (q, Step::PrimeStay);
    }
    // l. 18-26: |Q| = |P̂|. The overlap with the smallest distance deviation.
    let others: Vec<Point> = sc
        .pts
        .iter()
        .copied()
        .filter(|&p| !sc.at(p, q1) && !sc.at(p, q2))
        .collect();
    // l. 20-23: every overlap, with its distance deviation.
    let mut overlaps: Vec<(f64, Vec<Point>)> = Vec::new();
    for &(pa, pb) in &pat.max_pairs {
        for (x, y) in [(q1, q2), (q2, q1)] {
            for mirror in [false, true] {
                let placed = map_pair(pat, pa, pb, x, y, mirror);
                let rest: Vec<Point> = (0..k)
                    .filter(|&i| i != pa && i != pb)
                    .map(|i| placed[i])
                    .collect();
                let dev = match (rest.len(), others.len()) {
                    (1, 1) => dist(rest[0], others[0]),
                    (2, 2) => (dist(rest[0], others[0]) + dist(rest[1], others[1]))
                        .min(dist(rest[0], others[1]) + dist(rest[1], others[0])),
                    _ => 0.0,
                };
                overlaps.push((dev, placed));
            }
        }
    }
    let smallest = overlaps.iter().map(|o| o.0).fold(f64::INFINITY, f64::min);
    // "If there are two overlaps of the pattern that have the same smallest
    // distance deviation, then r can consider one of them indiscriminately"
    // (Section 5.1). r considers one in which it has a move, if there is one:
    // were each robot to pick one in which another robot moves, a symmetric Q
    // would never change. See docs/sqpf.md, choice 6.
    for (_, placed) in overlaps.iter().filter(|o| o.0 <= smallest + sc.tol) {
        let free: Vec<Point> = sc
            .pts
            .iter()
            .copied()
            .filter(|&x| !placed.iter().any(|&p| sc.at(x, p)))
            .collect();
        // l. 24: an unoccupied pattern point whose closest free robot is q.
        let mine = placed
            .iter()
            .copied()
            .filter(|&p| !sc.occupied(p))
            .filter(|&p| {
                free.iter()
                    .copied()
                    .min_by(|x, y| dist(*x, p).partial_cmp(&dist(*y, p)).unwrap_or(Ordering::Equal))
                    .is_some_and(|c| sc.at(c, q))
            })
            .min_by(|x, y| dist(*x, q).partial_cmp(&dist(*y, q)).unwrap_or(Ordering::Equal));
        if let Some(pi) = mine {
            if sc.robots_on(q, pi).is_empty() {
                return (pi, Step::PrimeOccupy); // l. 25-26
            }
        }
    }
    (q, Step::PrimeStay)
}

/// The robot's Compute phase: SqPF (|P̂| ≥ 5) or SqPF′ (|P̂| < 5) on the
/// snapshot `q_pts` (the set Q in the robot's own frame, the robot at the
/// origin, as [`snapshot`] builds it). Returns the destination in the same frame.
/// `min_move` is the simulator's minimum move in the robot's own unit: points
/// closer than it (or than 10⁻⁹·ρ) are the same point.
///
/// # Panics
/// If `q_pts` does not contain the origin.
#[must_use]
pub fn destination(pat: &Pattern, q_pts: &[Point], min_move: f64) -> (Point, Step) {
    let q = *q_pts
        .iter()
        .find(|p| p.x == 0.0 && p.y == 0.0)
        .expect("the robot is at the origin of its own snapshot");
    if q_pts.len() < pat.len() {
        return (separate(q_pts, q, min_move), Step::Separate); // Alg. 1 l. 2-3 / Alg. 7 l. 3-4
    }
    let (o, rho) = sec(q_pts);
    let sc = Scene::new(q, q_pts, o, rho, (REL_TOL * rho).max(min_move));
    if pat.len() >= 5 {
        sqpf(pat, &sc)
    } else {
        sqpf_prime(pat, &sc)
    }
}

/// `true` when the robots at `world` form P̂: the set of occupied points is
/// similar to it (Q ∈ 𝓑(P̂), Section 2.3). `min_move` is the simulator's
/// minimum move, the tolerance the robots themselves use. For reports and the
/// page; robots never call it.
#[must_use]
pub fn is_formed(pat: &Pattern, world: &[Point], min_move: f64) -> bool {
    formed_placement(pat, world, min_move).is_some()
}

/// When the robots at `world` form P̂: the pattern points where they are,
/// in world coordinates and in the order P̂ was given (so the page can join
/// them into the shape's outline). `None` while the pattern is not formed.
/// `min_move` as for [`is_formed`]. For reports and the page; robots never
/// call it.
#[must_use]
pub fn formed_placement(pat: &Pattern, world: &[Point], min_move: f64) -> Option<Vec<Point>> {
    let &first = world.first()?;
    let frame = LocalFrame::global(first);
    let q_pts = snapshot(world, &frame, min_move);
    if q_pts.len() != pat.len() {
        return None;
    }
    let (o, rho) = sec(&q_pts);
    // A little wider than the robots' own tolerance, so the page never calls
    // "not formed" a configuration the robots stopped on as formed.
    let sc = Scene::new(q_pts[0], &q_pts, o, rho, (1e-7 * rho).max(2.0 * min_move));
    // A pair (x, y) can carry p̂₀p̂₁ only at the scale of SEC(Q): when every
    // point of the placement is within tol of Q, the radii of the two SECs
    // differ by at most tol, so | |xy| − ρ·|p̂₀p̂₁| | ≤ tol·|p̂₀p̂₁| ≤ 2·tol
    // (P̂ has SEC radius 1). Pairs farther off are skipped without building a
    // placement; the first pair that matches is the same as before.
    let chord = rho * dist(pat.points[0], pat.points[1]);
    let slack = 10.0 * sc.tol;
    // The same test on squared lengths, a little wider, first: no square root
    // for the pairs that are clearly off.
    let near2 = (chord - 2.0 * slack).max(0.0).powi(2);
    let far2 = (chord + 2.0 * slack).powi(2);
    for &x in sc.pts {
        for &y in sc.pts {
            let (dx, dy) = (x.x - y.x, x.y - y.y);
            let d2 = dx * dx + dy * dy;
            if d2 < near2 || d2 > far2 || sc.at(x, y) || (dist(x, y) - chord).abs() > slack {
                continue;
            }
            for mirror in [false, true] {
                let placed = map_pair(pat, 0, 1, x, y, mirror);
                if sc.same_set(&placed) {
                    return Some(placed.into_iter().map(|p| frame.to_global(p)).collect());
                }
            }
        }
    }
    None
}

// ── the robots ────────────────────────────────────────────────────────────────

/// A robot's private coordinate system, apart from its origin (always the
/// robot itself): the direction of its x-axis, its unit of length and its
/// handedness. Fixed for the robot, unknown to every other robot.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Compass {
    pub angle: f64,
    /// World units per local unit is `1 / scale`.
    pub scale: f64,
    pub mirrored: bool,
}

impl Compass {
    /// The robot's frame when it stands at `origin`.
    #[must_use]
    pub const fn frame(self, origin: Point) -> LocalFrame {
        LocalFrame {
            origin,
            angle: self.angle,
            scale: self.scale,
            reflect: self.mirrored,
        }
    }
}

/// Mixed into the run's seed to draw the compasses, so they never share draws
/// with the run's own random stream.
const COMPASS_SEED: u64 = 0x5EED_C0A5_5E5F_0001;

/// `n` compasses drawn from `seed`: any direction, a unit between 0.5 and 2
/// world units, either handedness.
#[must_use]
pub fn compasses(n: usize, seed: u64) -> Vec<Compass> {
    let mut rng = Rng::new(seed ^ COMPASS_SEED);
    (0..n)
        .map(|_| Compass {
            angle: rng.uniform(0.0, TAU),
            scale: rng.uniform(0.5, 2.0),
            mirrored: rng.random() < 0.5,
        })
        .collect()
}

/// SqPF / SqPF′ as an algorithm of the core.
#[derive(Clone, Debug)]
pub struct SqPatternFormation {
    pub pattern: Pattern,
    /// One per robot, by robot index: the simulator's model of each robot's
    /// body. The algorithm itself ([`destination`]) never sees an index.
    pub compasses: Vec<Compass>,
}

impl Algorithm for SqPatternFormation {
    fn key(&self) -> &'static str {
        KEY
    }

    /// Look in the robot's own frame, Compute with [`destination`], and hand
    /// the destination back to the simulator in world coordinates for Move.
    /// Never terminates: robots stay put once formed, as the paper's do.
    fn compute(&self, look: &Look<'_>, _: &mut Rng, _: &mut Scratch) -> Decision {
        let compass = self.compasses[look.me as usize % self.compasses.len()];
        let frame = compass.frame(look.position);
        // The simulator drops moves shorter than its eps: the robot treats
        // points that close as one (in its own unit for Compute).
        let min_move = look.model.eps;
        let q_pts = snapshot(&look.view.pos, &frame, min_move);
        let (dest, _) = destination(&self.pattern, &q_pts, min_move * compass.scale);
        let target = if dest.x == 0.0 && dest.y == 0.0 {
            look.position
        } else {
            frame.to_global(dest)
        };
        Decision {
            target,
            terminate: false,
            overlay: OverlayUpdate::Keep,
        }
    }
}

/// The predefined shapes as point sets, the same corners the page sends
/// (`SHAPE_POINTS` in web/app.js): used when a config names a shape but no points.
#[must_use]
#[allow(clippy::cast_precision_loss)]
pub fn shape_points(shape: PatternShape) -> Vec<Point> {
    use core::f64::consts::PI;
    const S: f64 = 100.0;
    let ring = |k: usize, phase: f64, rad: &dyn Fn(usize) -> f64| -> Vec<Point> {
        (0..k)
            .map(|i| {
                let a = phase + TAU * i as f64 / k as f64;
                Point::new(S * rad(i) * libm::cos(a), S * rad(i) * libm::sin(a))
            })
            .collect()
    };
    let one = |_: usize| 1.0;
    match shape {
        PatternShape::Star => ring(10, PI / 2.0, &|i| if i % 2 == 1 { 0.382 } else { 1.0 }),
        PatternShape::Circle => ring(12, PI / 2.0, &one),
        PatternShape::Square => ring(4, PI / 4.0, &one),
        PatternShape::Triangle => ring(3, PI / 2.0, &one),
        PatternShape::Pentagon => ring(5, PI / 2.0, &one),
        PatternShape::Hexagon => ring(6, 0.0, &one),
        PatternShape::Line => vec![Point::new(-S, 0.0), Point::new(S, 0.0)],
        PatternShape::Cross => {
            let (l, w) = (S, 0.35 * S);
            [
                (w, w), (w, l), (-w, l), (-w, w), (-l, w), (-l, -w),
                (-w, -w), (-w, -l), (w, -l), (w, -w), (l, -w), (l, w),
            ]
            .iter()
            .map(|&(x, y)| Point::new(x, y))
            .collect()
        }
        PatternShape::Heart => (0..12)
            .map(|i| {
                let t = TAU * i as f64 / 12.0;
                let s = libm::sin(t);
                Point::new(
                    S * s * s * s,
                    S / 16.0
                        * (13.0 * libm::cos(t)
                            - 5.0 * libm::cos(2.0 * t)
                            - 2.0 * libm::cos(3.0 * t)
                            - libm::cos(4.0 * t)),
                )
            })
            .collect(),
    }
}

/// The pattern a config asks for: `target_points` if given, else the corners
/// of `target_pattern`.
///
/// # Errors
/// A pattern the paper does not cover (see [`Pattern::new`]).
pub fn pattern_of(config: &SimConfig) -> Result<Pattern, String> {
    let given: Vec<Point> = match &config.target_points {
        Some(p) => p.iter().map(|p| Point::new(p[0], p[1])).collect(),
        None => shape_points(config.target_pattern),
    };
    Pattern::new(&given)
}

/// The registry entry: SqPF runs only in the paper's model.
///
/// # Errors
/// The asynchronous scheduler, rigid movement or no δ, limited visibility,
/// faulty robots, an unusable pattern, or fewer robots than pattern points.
pub fn plan(config: &SimConfig) -> Result<Plan, ConfigError> {
    let refuse = |message: String| Err(ConfigError::Unsupported(message));
    if config.scheduler != SchedulerKind::Sequential {
        return refuse(
            "Sequential Pattern Formation needs the sequential scheduler (Settings → Scheduler → \
             Sequential): UPF is unsolvable under FSYNC, hence under SSYNC and ASYNC \
             (Theorem 1 of the paper)"
                .to_owned(),
        );
    }
    if config.rigid_movement || config.delta.is_none() {
        return refuse(
            "Sequential Pattern Formation uses the paper's non-rigid movement (Section 2.1: \"In \
             this work we consider only non-rigid movements\"): turn Rigid movement off and set a \
             minimum move δ > 0"
                .to_owned(),
        );
    }
    if config.visibility_radius.is_some() {
        return refuse(
            "Sequential Pattern Formation assumes every robot sees all the others: turn on \
             Unlimited visibility"
                .to_owned(),
        );
    }
    if config.num_of_faults > 0 {
        return refuse(
            "Sequential Pattern Formation assumes no faulty robots: set Faulty robots to 0".to_owned(),
        );
    }
    let pattern = pattern_of(config).map_err(ConfigError::Unsupported)?;
    let n = config.num_of_robots as usize;
    if n < pattern.len() {
        return refuse(format!(
            "a pattern of {} points needs at least {} robots (the trivial assumption |R| ≥ |P̂|, \
             Section 1.2); got {n}",
            pattern.len(),
            pattern.len()
        ));
    }
    Ok(Plan::single(Box::new(SqPatternFormation {
        pattern,
        compasses: compasses(n, config.random_seed),
    })))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: f64, y: f64) -> Point {
        Point::new(x, y)
    }

    #[test]
    fn sec_of_a_triangle_and_a_pair() {
        let (c, r) = sec(&[p(0.0, 0.0), p(2.0, 0.0), p(1.0, 0.1)]);
        assert!((r - 1.0).abs() < 1e-12 && dist(c, p(1.0, 0.0)) < 1e-12);
        let (c, r) = sec(&[p(0.0, 0.0), p(4.0, 0.0), p(2.0, 3.0)]);
        assert!((dist(c, p(0.0, 0.0)) - r).abs() < 1e-9 && (dist(c, p(2.0, 3.0)) - r).abs() < 1e-9);
    }

    #[test]
    fn mu_hat_is_independent_of_the_pattern_frame() {
        // The same pattern given rotated, reflected and scaled has a placement
        // that is the same set of points.
        let base = vec![p(0.0, 1.0), p(1.0, 0.2), p(0.3, -0.9), p(-0.8, -0.5), p(-0.6, 0.6), p(0.1, 0.3)];
        let other: Vec<Point> = base.iter().map(|q| p(3.0 * q.y + 5.0, 3.0 * q.x - 2.0)).collect();
        let (a, b) = (Pattern::new(&base).unwrap(), Pattern::new(&other).unwrap());
        let at = p(10.0, 0.0);
        let o = p(0.0, 0.0);
        let pa = a.place(a.anchors[0].0, a.anchors[0].1 < 0.0, at, o, 10.0);
        let pb = b.place(b.anchors[0].0, b.anchors[0].1 < 0.0, at, o, 10.0);
        for x in &pa {
            assert!(pb.iter().any(|y| dist(*x, *y) < 1e-9), "{x:?} missing");
        }
    }

    #[test]
    fn a_pattern_of_one_point_or_with_a_repeat_is_refused() {
        assert!(Pattern::new(&[p(1.0, 1.0)]).is_err());
        assert!(Pattern::new(&[p(0.0, 0.0), p(1.0, 0.0), p(0.0, 0.0)]).is_err());
    }

    #[test]
    fn separate_goes_one_unit_along_the_own_x_axis_or_halfway_to_the_blocker() {
        let o = p(0.0, 0.0);
        assert_eq!(separate(&[o, p(5.0, 5.0)], o, 0.0), p(1.0, 0.0));
        assert_eq!(separate(&[o, p(0.5, 0.0), p(5.0, 5.0)], o, 0.0), p(0.25, 0.0));
    }
}


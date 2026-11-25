//! Theorem V.1 of Navarra and Piselli: from every start outside the
//! unsolvable set, GatheRRing gathers the robots in at most n - 3 epochs,
//! whatever the round-robin order.
//!
//! Theorem V.1 (n - 3 epochs) is the paper's claim; an exhaustive check found
//! these exceptions (review guide for PR #17, section 6, D2), asserted below
//! as known exceptions so that any change in behaviour shows up:
//! - n = 5, k = 4, stacked starts on three nodes that are not consecutive:
//!   they never gather (T1/m1 swaps two robots forever; `unsolvable` does not
//!   list them, since Theorem III.3 covers k >= 5 only);
//! - n = 6 (k = 5..9) and n = 8 (k = 8): some starts take n - 2 epochs,
//!   among them the paper's Fig. 4 example;
//! - n = 3, k = 2: gathering takes one epoch, over a bound of 0.
//!
//! The algorithm is kept as the paper gives it.

use lcm_ring::{decide, unsolvable, Move, RingConfig, RingSim, Status, View};

/// The epoch in which a run gathered, counting a partial one (guide D1).
fn gathered_in(sim: &RingSim) -> u64 {
    assert_eq!(sim.status, Status::Gathered);
    sim.epoch()
}

/// The paper's Fig. 4 example (n = 6, robots on 0, 5, 3, 2, 0) gathers
/// partway through epoch 4 = n - 2. Comparing completed epochs (3) with the
/// bound hid that (guide D1); it is a known exception to the bound (guide D2).
#[test]
fn the_epoch_of_gathering_counts_a_partial_epoch() {
    let config = RingConfig {
        nodes: 6,
        robots: vec![0, 5, 3, 2, 0],
        ..RingConfig::default()
    };
    assert!(unsolvable(&config).is_none());
    let mut sim = RingSim::new(&config).unwrap();
    let turns = sim.run();
    assert_eq!(sim.epochs, 3, "completed epochs");
    assert_eq!(gathered_in(&sim), 4, "the epoch it gathers in");
    assert_eq!(turns.last().unwrap().epoch, 4);
    // Every tie choice and order: still at most n - 2 (5 robots, 20 turns).
    assert_eq!(worst(6, &[0, 5, 3, 2, 0]), Some(17));
}

/// A run that gathers at the end of an epoch is not counted twice, and a
/// start that is already gathered took no epoch.
#[test]
fn the_epoch_of_gathering_on_a_boundary() {
    let at_start = RingSim::new(&RingConfig {
        nodes: 7,
        robots: vec![3, 3, 3],
        ..RingConfig::default()
    })
    .unwrap();
    assert_eq!(gathered_in(&at_start), 0);
    // Two robots on neighbouring nodes: the first steps onto the second, and
    // the epoch ends with the second robot's turn.
    let mut sim = RingSim::new(&RingConfig {
        nodes: 7,
        robots: vec![2, 3],
        ..RingConfig::default()
    })
    .unwrap();
    sim.run();
    assert_eq!((sim.turns, sim.epochs, gathered_in(&sim)), (1, 0, 1));
}

/// The guide's n = 5, k = 4 example (D2): robots on 0, 3, 1, 0 under the
/// order 0, 1, 2, 3 never gather, and no tie choice is involved.
#[test]
fn a_stacked_start_on_a_5_ring_never_gathers() {
    let config = RingConfig {
        nodes: 5,
        robots: vec![0, 3, 1, 0],
        max_epochs: Some(100),
        ..RingConfig::default()
    };
    assert!(unsolvable(&config).is_none());
    let mut sim = RingSim::new(&config).unwrap();
    let turns = sim.run();
    assert_eq!(sim.status, Status::Incomplete);
    assert!(turns
        .iter()
        .filter(|t| t.from != t.to)
        .all(|t| t.rule == "m1: n-2 occupied, one empty neighbour: to the occupied neighbour"));
    assert_eq!(worst(5, &[0, 3, 1, 0]), None);
}

/// The worst case over every tie choice from one start under the order
/// 0, 1, â€¦, k - 1: `Some(turns)` to gather, or `None` if some choices never
/// gather (the robots are back where they were at the start of an earlier
/// epoch).
///
/// Robots are anonymous and `decide` sees only the occupied nodes, so the run
/// from a start under some order is the run from that start relabelled by
/// the order under 0, 1, â€¦: all starts under 0, 1, â€¦ cover every start under
/// every order.
fn worst(n: usize, start: &[usize]) -> Option<u64> {
    fn view(n: usize, pos: &[usize]) -> View {
        let mut occ = vec![false; n];
        for &v in pos {
            occ[v] = true;
        }
        View::new(occ)
    }
    fn go(
        n: usize,
        mut pos: Vec<usize>,
        mut next: usize,
        mut turn: u64,
        mut seen: Vec<Vec<usize>>,
    ) -> Option<u64> {
        let k = pos.len();
        let mut occupied = view(n, &pos);
        loop {
            if pos.iter().all(|&v| v == pos[0]) {
                return Some(turn);
            }
            if next == 0 {
                if seen.contains(&pos) {
                    return None;
                }
                seen.push(pos.clone());
            }
            let from = pos[next];
            let to = |d: i8| {
                if d == 1 {
                    (from + 1) % n
                } else {
                    (from + n - 1) % n
                }
            };
            match decide(&occupied, from).1 {
                Move::Stay => {}
                Move::Step(d) => {
                    pos[next] = to(d);
                    occupied = view(n, &pos);
                }
                Move::Either => {
                    let mut most = 0;
                    for d in [1, -1] {
                        let mut p = pos.clone();
                        p[next] = to(d);
                        most = most.max(go(n, p, (next + 1) % k, turn + 1, seen.clone())?);
                    }
                    return Some(most);
                }
            }
            next = (next + 1) % k;
            turn += 1;
        }
    }
    go(n, start.to_vec(), 0, 0, Vec::new())
}

/// One (n, k): every start with robot 0 on node 0 (the ring has no labels,
/// so rotations add nothing) outside the unsolvable set, stacked or not.
#[derive(Debug, Default)]
struct Census {
    starts: u64,
    /// The worst epoch of gathering, counting a partial one, over all starts
    /// and choices that gather.
    worst: u64,
    /// Starts whose worst case is over n - 3 epochs.
    over: u64,
    /// Starts from which some choices never gather.
    never: u64,
}

fn census(n: usize, k: usize) -> Census {
    let bound = (n as u64).saturating_sub(3);
    let mut c = Census::default();
    let mut pos = vec![0; k];
    loop {
        let config = RingConfig {
            nodes: n,
            robots: pos.clone(),
            ..RingConfig::default()
        };
        if unsolvable(&config).is_none() {
            c.starts += 1;
            match worst(n, &pos) {
                Some(turns) => {
                    let epoch = turns.div_ceil(k as u64);
                    c.worst = c.worst.max(epoch);
                    c.over += u64::from(epoch > bound);
                }
                None => {
                    // Known exception (guide section 6, D2): only n = 5,
                    // k = 4, stacked on three non-consecutive nodes.
                    let mut nodes = pos.clone();
                    nodes.sort_unstable();
                    nodes.dedup();
                    let empty: Vec<usize> = (0..n).filter(|v| !nodes.contains(v)).collect();
                    let holes_apart = empty.len() == 2
                        && empty[1] - empty[0] != 1
                        && empty[0] + n - empty[1] != 1;
                    assert!(
                        n == 5 && k == 4 && nodes.len() == 3 && holes_apart,
                        "n {n} k {k}: {pos:?} never gathers"
                    );
                    c.never += 1;
                }
            }
        }
        // The next start: robots 1..k count up in base n.
        let mut r = k - 1;
        loop {
            if r == 0 {
                return c;
            }
            pos[r] += 1;
            if pos[r] < n {
                break;
            }
            pos[r] = 0;
            r -= 1;
        }
    }
}

/// Every (n, k) with 3 <= n <= 12, k >= 2 and n^(k-1) <= `limit` (every
/// start, stacked ones included, every round-robin order, every tie choice):
/// the bound n - 3 holds except for the known exceptions, whose counts are
/// pinned. Counts are of starts with robot 0 on node 0 under the order
/// 0, 1, â€¦, k - 1, i.e. of (start, order) pairs up to rotation.
fn exhaustive(limit: u64) {
    // Known exceptions (guide for PR #17, section 6, D2): starts over n - 3
    // epochs, all of them gathering in epoch n - 2.
    let over = |n: usize, k: usize| -> u64 {
        match (n, k) {
            // n = 3, k = 2: one epoch, over a bound of 0.
            (3, 2) => 2,
            // n = 6, k = 5..9, among them the paper's Fig. 4 example.
            (6, 5) => 56,
            (6, 6) => 1_040,
            (6, 7) => 9_328,
            (6, 8) => 75_376,
            (6, 9) => 533_352,
            (8, 8) => 768,
            _ => 0,
        }
    };
    // Known exception (guide D2): n = 5, k = 4 stacked starts never gather.
    let never = |n: usize, k: usize| -> u64 {
        if (n, k) == (5, 4) {
            8
        } else {
            0
        }
    };
    let results = std::thread::scope(|s| {
        let threads: Vec<_> = (3..=12usize)
            .map(|n| {
                s.spawn(move || {
                    let mut rows = Vec::new();
                    let mut k = 2;
                    while (n as u64).pow(k as u32 - 1) <= limit {
                        rows.push((n, k, census(n, k)));
                        k += 1;
                    }
                    rows
                })
            })
            .collect();
        threads
            .into_iter()
            .flat_map(|t| t.join().unwrap())
            .collect::<Vec<_>>()
    });
    for (n, k, c) in &results {
        let (n, k) = (*n, *k);
        let bound = (n as u64).saturating_sub(3);
        println!("n {n:>2} k {k:>2}: {c:?}");
        assert!(c.worst <= bound + 1, "n {n} k {k}: {c:?}");
        assert_eq!(c.over, over(n, k), "n {n} k {k}: {c:?}");
        assert_eq!(c.never, never(n, k), "n {n} k {k}: {c:?}");
    }
    assert!(results.iter().any(|(n, k, _)| (*n, *k) == (6, 5)));
}

/// The exhaustive check for n^(k-1) <= 300,000 (n = 3..12; on a 6-ring k up
/// to 8): about 10 s in a debug build.
#[test]
fn every_small_start_gathers_within_n_minus_3_epochs_or_is_a_known_exception() {
    exhaustive(300_000);
}

/// The guide's full check, n^(k-1) <= 3,000,000 (adds among others n = 6,
/// k = 9 and n = 8, k = 8): about 30 s in a release build, minutes in debug.
/// `cargo test --release -p lcm-ring --test gathering -- --ignored`
#[test]
#[ignore = "slow in debug; run with --release -- --ignored"]
fn every_start_up_to_three_million_gathers_within_n_minus_3_epochs_or_is_a_known_exception() {
    exhaustive(3_000_000);
}

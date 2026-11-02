//! Randomized gathering of two asynchronous robots: Pattanayak, Augustine,
//! Mandal, "Randomized gathering of asynchronous mobile robots", Theoretical
//! Computer Science 858, 2021 (arXiv:1909.10004, Sections 2-4).
//!
//! Both robots move on the line joining them (the paper's λ-class, Lemma 1):
//! at each Look a robot that sees the other at distance δ moves λδ towards
//! it, with λ drawn afresh:
//!
//! - **Theorem 12** (the default): λ ∈ {0, 1/2, 1}, each with probability
//!   1/3. Against an oblivious adversary whose wait-plus-compute time per
//!   cycle exceeds some τ > 0, the expected number of Looks is at most
//!   18 (log2(δ/τ) + 1).
//! - **Theorem 3**: λ = 1, λ = 1/2, or λ uniform in (0, 1), each with
//!   probability 1/3: gathering with positive probability when computing
//!   takes no time.
//!
//! A robot that sees the other on its own point has gathered and stops. The
//! paper's extension to more robots needs co-located robots to merge into one
//! (Section 5), which this simulator does not do, so the algorithm is for
//! two robots.

use lcm_core::{
    Algorithm, ConfigError, Decision, Look, OverlayUpdate, Plan, Point, Rng, Scratch, SimConfig,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Choice {
    /// λ ∈ {0, 1/2, 1} (Theorem 12).
    ZeroHalfOne,
    /// λ ∈ {1, 1/2, U(0, 1)} (Theorem 3).
    OneHalfUniform,
}

#[derive(Clone, Copy, Debug)]
pub struct RandomizedGathering {
    pub choice: Choice,
}

impl RandomizedGathering {
    /// One draw of λ from the run's generator.
    pub fn lambda(&self, rng: &mut Rng) -> f64 {
        let k = rng.index(3);
        match (self.choice, k) {
            (_, 1) => 0.5,
            (Choice::ZeroHalfOne, 0) => 0.0,
            (Choice::ZeroHalfOne, _) | (Choice::OneHalfUniform, 0) => 1.0,
            (Choice::OneHalfUniform, _) => rng.random(),
        }
    }
}

impl Algorithm for RandomizedGathering {
    fn key(&self) -> &'static str {
        "RandomizedGathering"
    }

    fn randomized(&self) -> bool {
        true
    }

    fn compute(&self, look: &Look<'_>, rng: &mut Rng, _: &mut Scratch) -> Decision {
        let me = look.position;
        let other = (0..look.view.len())
            .find(|&k| k != look.me_index)
            .map(|k| look.view.pos[k]);
        let Some(q) = other else {
            return Decision {
                target: me,
                terminate: false,
                overlay: OverlayUpdate::Keep,
            };
        };
        if libm::hypot(q.x - me.x, q.y - me.y) <= look.model.eps {
            return Decision {
                target: me,
                terminate: true,
                overlay: OverlayUpdate::Keep,
            };
        }
        let l = self.lambda(rng);
        Decision {
            target: Point::new(me.x + l * (q.x - me.x), me.y + l * (q.y - me.y)),
            terminate: false,
            overlay: OverlayUpdate::Keep,
        }
    }

    fn explain(&self, look: &Look<'_>, _: &mut Scratch) -> Option<String> {
        let me = look.position;
        let q = (0..look.view.len())
            .find(|&k| k != look.me_index)
            .map(|k| look.view.pos[k])?;
        let gap = libm::hypot(q.x - me.x, q.y - me.y);
        if gap <= look.model.eps {
            return Some(
                "The other robot is on its point: they have gathered, so it terminates.".to_owned(),
            );
        }
        let coin = match self.choice {
            Choice::ZeroHalfOne => "stay, go halfway or go all the way (λ ∈ {0, ½, 1})",
            Choice::OneHalfUniform => {
                "go all the way, halfway or a uniformly random fraction of the way (λ ∈ {1, ½, U(0, 1)})"
            }
        };
        Some(format!(
            "The other robot is {} units away. At its Look it tosses a three-sided coin: {coin}.",
            crate::num(gap)
        ))
    }
}

fn two_robots(config: &SimConfig, name: &str) -> Result<(), ConfigError> {
    let n = config
        .initial_positions
        .as_ref()
        .filter(|p| p.len() == config.num_of_robots as usize)
        .map_or(config.num_of_robots as usize, Vec::len);
    if n == 2 {
        Ok(())
    } else {
        Err(ConfigError::Unsupported(format!(
            "{name} is for exactly two robots: with more, robots that meet would have to recognise each other and merge into one, which these rules do not do. Set Robots to 2 (Settings → Robots → Swarm)."
        )))
    }
}

/// Theorem 12's rule.
///
/// # Errors
/// Not exactly two robots.
pub fn plan(config: &SimConfig) -> Result<Plan, ConfigError> {
    two_robots(config, "Randomized gathering")?;
    Ok(Plan::single(Box::new(RandomizedGathering {
        choice: Choice::ZeroHalfOne,
    })))
}

/// Theorem 3's rule.
///
/// # Errors
/// Not exactly two robots.
pub fn plan_uniform(config: &SimConfig) -> Result<Plan, ConfigError> {
    two_robots(config, "Randomized gathering")?;
    Ok(Plan::single(Box::new(RandomizedGathering {
        choice: Choice::OneHalfUniform,
    })))
}

/// Where the settings leave the paper's model.
#[must_use]
pub fn notes(config: &SimConfig) -> Vec<String> {
    let mut notes = Vec::new();
    if !config.rigid_movement {
        notes.push("Outside the model: these rules are designed for robots that always reach the point they chose. When a robot can be stopped on the way, its coin toss no longer has the effect the rules count on, so meeting may take much longer, or the two robots may keep missing each other.".to_owned());
    }
    if config.num_of_faults > 0 {
        notes.push("Outside the model: these rules are designed for two robots that never fail and move at the same speed. A crashed, erratic or forgetful robot, or one slowed down by a delay fault, breaks the balance between the two robots that the coin tosses rely on.".to_owned());
    }
    notes
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lambda_takes_the_three_values_about_equally() {
        let rule = RandomizedGathering {
            choice: Choice::ZeroHalfOne,
        };
        let mut rng = Rng::new(3);
        let mut counts = [0usize; 3];
        for _ in 0..30_000 {
            let l = rule.lambda(&mut rng);
            counts[if l == 0.0 {
                0
            } else if l == 0.5 {
                1
            } else {
                2
            }] += 1;
        }
        assert!(
            counts.iter().all(|&c| (9_000..11_000).contains(&c)),
            "{counts:?}"
        );
    }

    #[test]
    fn the_uniform_variant_stays_within_zero_and_one() {
        let rule = RandomizedGathering {
            choice: Choice::OneHalfUniform,
        };
        let mut rng = Rng::new(5);
        assert!((0..10_000).all(|_| (0.0..=1.0).contains(&rule.lambda(&mut rng))));
    }
}

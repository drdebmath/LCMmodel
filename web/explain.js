/* Info cards for the settings panel: one or two short sentences per choice,
 * what it does and the one thing to watch for, and how it meets the chosen
 * algorithm. The panel shows the card under the setting that was just changed
 * (or when its ⓘ button is pressed). Longer explanations live in
 * docs/core-schema.md §2 and docs/models.md.
 *
 * explain(key, values, algo) returns { title, paras: [html…], tip?: html }
 * or null when the setting has no card. `algo` is { key, name, multiplicities,
 * sequentialOnly } for the chosen algorithm. */

const MULTIPLICITY_ALGOS = new Set(["SqGathering", "CrashGathering"]);

const p = (...parts) => parts.join(" ");

const CARDS = {
  scheduler: (v) => ({
    async: {
      title: "Asynchronous scheduler",
      paras: [p("Every robot wakes on its own random clock, so many robots look and move at once.",
        "Watch for robots deciding on snapshots that are already out of date when they move.")],
    },
    sequential: {
      title: "Sequential scheduler: one robot at a time",
      paras: [p("One robot looks, decides and moves while all the others stand still; each epoch gives every working robot one turn.",
        "Pick a “s/turn” speed to watch each turn: the look, then the walk.")],
    },
    ssync: {
      title: "Semi-synchronous scheduler (SSYNC): rounds, some robots at once",
      paras: [p("In each round a chosen group looks at the same instant, then all of them move together.",
        "Watch for robots of one round that cannot see where the others are going.")],
    },
    fsync: {
      title: "Fully synchronous scheduler (FSYNC): rounds, every robot at once",
      paras: [p("Every working robot looks and moves in every round, in lock-step.",
        "Watch for symmetry: robots that see the same picture make the same move.")],
    },
    kasync: {
      title: "k-asynchronous scheduler (k-ASYNC): asynchronous, with a bound",
      paras: [p("Asynchronous, but while one robot is in a cycle no other robot takes more than k snapshots.",
        "With k = 1 robots overlap little; a large k is plain asynchronous.")],
    },
  }[v.scheduler]),

  k_async: (v) => ({
    title: `k = ${v.k_async}: at most ${v.k_async} snapshot${v.k_async === 1 ? "" : "s"} per cycle`,
    paras: [p(`While one robot is between its snapshot and the end of its move, no other robot takes more than ${v.k_async}.`,
      "Larger values come closer to plain asynchronous timing.")],
  }),

  ssync_probability: (v) => ({
    title: `Each robot is active in a round with chance ${v.ssync_probability}`,
    paras: [p("Small values give rounds of few robots; 1 makes every round a full (FSYNC) round.",
      "A round with nobody drawn activates one robot at random.")],
  }),

  ssync_fairness: (v) => ({
    title: `Fairness bound: never left out more than ${v.ssync_fairness} rounds`,
    paras: [p(`A robot left out of ${v.ssync_fairness} rounds in a row is active in the next one, so every robot acts again soon.`)],
  }),

  activation_order: (v) => ({
    round_robin: {
      title: "Turn order: round-robin",
      paras: [p("Robots take turns in the order 0, 1, 2, … every epoch, skipping crashed and terminated ones.")],
    },
    random: {
      title: "Turn order: a new random order every epoch",
      paras: [p("The robots are shuffled at the start of every epoch; each still gets one turn, and the seed fixes the shuffle.")],
    },
  }[v.activation_order]),

  turn_gap: (v) => ({
    random: {
      title: "A random pause between turns",
      paras: [p("A random pause (rate λ) passes before the next turn or round.",
        "It only stretches the clock; the outcome does not change.")],
    },
    none: {
      title: "No pause between turns",
      paras: [p("The next turn or round starts the instant the last one ends, so the clock only runs while robots move.")],
    },
  }[v.turn_gap]),

  multiplicity: (v, algo) => {
    const uses = algo && MULTIPLICITY_ALGOS.has(algo.key);
    const card = {
      none: {
        title: "No multiplicity detection",
        paras: [p("A spot with several robots looks like a spot with one, even the robot's own spot.",
          "Some tasks become impossible: a robot cannot tell a finished gathering from an unfinished one.")],
      },
      local: {
        title: "Local multiplicity detection",
        paras: [p("A robot can tell whether its own spot is shared, but sees every other spot as just occupied.")],
      },
      weak: {
        title: "Weak multiplicity detection",
        paras: [p("A robot can tell one robot on a spot from more than one, but cannot count them.")],
      },
      strong: {
        title: "Strong multiplicity detection",
        paras: [p("A robot sees exactly how many robots stand on each occupied spot.")],
      },
    }[v.multiplicity];
    if (!card) return null;
    if (uses) {
      card.tip = v.multiplicity === "weak" || v.multiplicity === "strong"
        ? `${algo.name} is built around seeing stacks, and this gives its robots what they need.`
        : `${algo.name} needs to see stacks; the brief at the top notes what goes wrong without.`;
    } else if (algo) {
      card.tip = `${algo.name} does not look at stacks, so this does not change its runs.`;
    }
    return card;
  },

  frames: (v) => ({
    global: {
      title: "Shared axes",
      paras: [p("Every robot uses the same north, the same unit of length and the same clockwise.")],
    },
    rotated: {
      title: "Own axes: each robot has its own rotation and unit",
      paras: [p("At every look a robot gets new axes of its own, turned and scaled at random, with itself at (0, 0).",
        "Algorithms that use only distances and angles behave the same; Spreading's grid does not.")],
    },
    disoriented: {
      title: "Disoriented robots: own axes, and no common clockwise",
      paras: [p("As with own axes, and the axes may also be mirrored at each look.",
        "Only distances and the swarm's shape up to a mirror image are shared.")],
    },
    compass: {
      title: "Unreliable compasses",
      paras: [p("Each robot's north is off by up to the compass error; unit and clockwise are shared.",
        "A compass that changes at every look is harder than a fixed one.")],
    },
  }[v.frames]),

  compass_error: (v) => ({
    title: `Compass error up to ${v.compass_error}°`,
    paras: [p(`A robot's north may be off by up to ${v.compass_error}° either way.`,
      "Compass gathering only promises to gather below its bound, and the brief warns when a run is past it.")],
  }),

  compass_static: (v) => (v.compass_static ? {
    title: "Fixed compasses",
    paras: [p("Each robot draws its error once and keeps it, so its north is always wrong the same way.")],
  } : {
    title: "Changing compasses",
    paras: [p("A new error is drawn at every look, so two decisions of one robot can use different norths.",
      "Rules then need a smaller error bound.")],
  }),

  rigid_movement: (v) => (v.rigid_movement ? {
    title: "Rigid movement: robots always arrive",
    paras: [p("A robot always reaches the destination it computed, in the same cycle.")],
  } : {
    title: "Non-rigid movement: robots can be stopped on the way",
    paras: [p("A robot can be stopped before its destination, but never before it has covered the minimum move δ.",
      "With δ = 0 robots still arrive, as in the original simulator.")],
  }),

  delta: (v) => ({
    title: v.delta > 0 ? `Minimum move δ = ${v.delta}` : "Minimum move δ = 0: robots always arrive",
    paras: [v.delta > 0
      ? p(`A robot can be stopped anywhere after ${v.delta} units; a nearer destination is always reached.`)
      : p("With δ = 0 non-rigid robots still reach their destination; raise δ to let robots be stopped.")],
  }),

  stop_policy: (v) => ({
    delta: {
      title: "Stopped right after δ",
      paras: [p("Every stoppable robot stops right after δ: the worst case, with the least progress allowed.")],
    },
    random: {
      title: "Stopped at a random point",
      paras: [p("A stoppable robot ends somewhere between δ and its destination, at random from the seed.")],
    },
    fraction: {
      title: "Stopped after a fixed fraction of the way",
      paras: [p("A stoppable robot covers the same fraction of its way every time, but never less than δ.")],
    },
  }[v.stop_policy]),

  unlimited_visibility: (v) => (v.unlimited_visibility ? {
    title: "Unlimited visibility",
    paras: [p("Every robot sees every other robot, however far away (unless robots block the view).")],
  } : {
    title: "Limited visibility",
    paras: [p("A robot sees only the robots within the radius below.",
      "Algorithms made for seeing everyone may split the swarm into groups that never meet.")],
  }),

  visibility_radius: (v) => ({
    title: `Robots see ${v.visibility_radius} units around them`,
    paras: [p("Press V to draw who sees whom.",
      "If the robots are not one connected network at the start, no algorithm can gather them all.")],
  }),

  obstructed_visibility: (v) => (v.obstructed_visibility ? {
    title: "Robots block the view",
    paras: [p("A robot between two others hides them from each other.",
      "Each look then costs time proportional to n², so keep the swarm to a few hundred.")],
  } : {
    title: "Transparent robots",
    paras: [p("Robots never hide each other, even when others stand in between.")],
  }),

  robot_radius: (v) => ({
    title: v.robot_radius > 0 ? `Robots block the view as discs of radius ${v.robot_radius}` : "Robots block the view as points",
    paras: [v.robot_radius > 0
      ? p("Robots nearly in line also hide each other; larger discs make the swarm hard to see across.")
      : p("A point hides only robots exactly in line behind it, which is common in lines and grids.")],
  }),

  num_of_faults: (v) => ({
    title: v.num_of_faults > 0 ? `${v.num_of_faults} faulty robot${v.num_of_faults === 1 ? "" : "s"}` : "No faulty robots",
    paras: [v.num_of_faults > 0
      ? p(`${v.num_of_faults} robots, chosen from the seed, have the fault below and their own colour in the legend.`,
        "A run ends once every working robot has terminated.")
      : p("Every robot works correctly; raise this to make some robots faulty.")],
  }),

  fault_type: (v) => ({
    crash: { title: "Crash faults", paras: [p("A crashed robot stops for good but stays visible where it is.")] },
    byzantine: { title: "Byzantine faults", paras: [p("A Byzantine robot moves to random points, and the others cannot tell it apart.")] },
    omission: { title: "Omission faults", paras: [p("An omission-faulty robot looks and decides as usual but skips about half its moves.")] },
    delay: { title: "Delay faults", paras: [p("A delayed robot works correctly but moves at 40 % of the normal speed.")] },
    mixed: { title: "Mixed faults", paras: [p("The faulty robots are split between crash, Byzantine, omission and delay faults.")] },
  }[v.fault_type]),

  multiplicities: (v) => ({
    title: v.multiplicities > 0 ? `Start with ${v.multiplicities} stacked point${v.multiplicities === 1 ? "" : "s"}` : "No stacked points at the start",
    paras: [v.multiplicities > 0
      ? p(`Robots are moved on top of each other to make ${v.multiplicities} stacks, drawn with an orange ring.`,
        "Large stacks can run out of numerical precision; a few robots each is plenty.")
      : p("Every robot starts on a spot of its own; raise this to start with stacks.")],
  }),

  multiplicity_size: (v) => ({
    title: v.multiplicity_size > 0 ? `${v.multiplicity_size} robots on each stacked point` : "Stack size chosen for you",
    paras: [v.multiplicity_size > 0
      ? p(`Each stacked point gets ${v.multiplicity_size} robots; there must be enough robots for all of them.`)
      : p("The simulator picks: a third of the robots for one stack, up to 4 each for several.")],
  }),

  threshold_precision: (v) => ({
    title: `Precision ε = 10⁻${v.threshold_precision}`,
    paras: [p("Positions closer than ε count as one point, and moves shorter than ε as no move.",
      "Some algorithms need a smaller ε (higher number) near the end of a run.")],
  }),

  lambda_rate: (v) => ({
    title: `Activation rate λ = ${v.lambda_rate}`,
    paras: [p(`Robots wake after random delays averaging 1/λ = ${(1 / v.lambda_rate).toFixed(2)} time units.`,
      "It changes the clock and the overlap, not how fast robots walk.")],
  }),

  open_world: (v) => (v.open_world ? {
    title: "Open world",
    paras: [p("There is no border: the start can be anywhere and as large as you like.")],
  } : {
    title: "The original world box",
    paras: [p("Robots start inside the World width × World height box; they can still move outside it.")],
  }),
};

/** The card for setting `key` with the current `values`, or null. */
export function explain(key, values, algo) {
  const make = CARDS[key];
  if (!make) return null;
  const card = make(values, algo);
  return card && card.title ? card : null;
}

/** Settings that have a card (they get an ⓘ button). */
export const EXPLAINED = new Set(Object.keys(CARDS));

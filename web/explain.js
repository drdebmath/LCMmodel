/* Info cards for the settings panel: what a choice means, what a run will
 * show because of it, and how it meets the algorithm that is chosen. The
 * panel shows the card under the setting that was just changed (or when its
 * ⓘ button is pressed). Plain words; each card stands on its own.
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
      paras: [
        p("Every robot runs on its own clock. It wakes up after a random delay, takes a snapshot of where the other robots are,",
          "decides where to go, and walks there; then it goes back to sleep and the next delay starts. Nobody coordinates these",
          "wake-ups, so at any moment some robots are looking, some are computing and some are half-way through a move."),
        p("This is the most general way robots can be timed, and the hardest one for an algorithm: a robot may take its snapshot",
          "while another robot is still moving, so it decides on positions that are already out of date by the time it moves."),
        p("What you will see: many robots moving at the same time, starting and stopping irregularly. The Activation rate λ",
          "sets how often robots wake up on average (a higher λ means shorter naps)."),
      ],
    },
    sequential: {
      title: "Sequential scheduler: one robot at a time",
      paras: [
        p("Only one robot is active at any moment. It takes its snapshot, decides, and walks all the way to its destination",
          "(or until it is stopped, if movement is non-rigid) while every other robot stands still. Only then does the next robot",
          "get its turn. Because nothing moves while a robot looks, every snapshot is exact and up to date."),
        p("Turns are grouped into epochs: in each epoch every robot that is still working gets exactly one turn. Turn order",
          "decides who goes next (0, 1, 2, … or a fresh random order each epoch), and an explicit list of turns can replace it.",
          "If a whole epoch passes in which no robot moves or terminates, nothing can ever change again, and the run stops as stalled."),
        p("What you will see: one robot moving at a time. Choose a speed such as “1 s/turn” in the speed menu to watch each",
          "turn in two beats: the robot looks (with lines to what it sees and a cross on its destination), then it walks there."),
      ],
    },
    ssync: {
      title: "Semi-synchronous scheduler (SSYNC): rounds, some robots at once",
      paras: [
        p("Time is divided into rounds. In each round a group of robots is chosen; they all take their snapshot at the same",
          "instant, all decide, and all move at the same time, while the robots that were not chosen stand still. The next round",
          "starts only when every chosen robot has finished moving."),
        p("Because the robots of one round look at the same moment, none of them sees where the others of its round are going:",
          "two robots can swap places, or step towards each other and miss. Chance to be active sets how likely each robot is to",
          "be chosen in a round, and the Fairness bound makes sure no robot is left out for too long."),
        p("What you will see: bursts of simultaneous moves, a round at a time."),
      ],
    },
    fsync: {
      title: "Fully synchronous scheduler (FSYNC): rounds, every robot at once",
      paras: [
        p("Time is divided into rounds and every working robot is active in every round: all robots look at the same instant,",
          "all decide, and all move together. This is the most orderly timing there is, and many problems become easier with it."),
        p("The catch is symmetry: robots that see the same picture make the same choice at the same time, so a symmetric",
          "swarm tends to stay symmetric. Two robots at the same spot can never be told apart again."),
        p("What you will see: the whole swarm moving in lock-step, one round at a time."),
      ],
    },
    kasync: {
      title: "k-asynchronous scheduler (k-ASYNC): asynchronous, with a bound",
      paras: [
        p("Robots wake on their own clocks as in the asynchronous scheduler, but with one restriction: while a robot is in the",
          "middle of a cycle, no other robot may start more than k snapshots. A snapshot that would break the bound waits until",
          "the robot's cycle is over."),
        p("This sits between the sequential scheduler and the unrestricted asynchronous one: with k = 1 robots overlap very",
          "little, and as k grows the timing approaches full asynchrony. Set k just below."),
      ],
    },
  }[v.scheduler]),

  k_async: (v) => ({
    title: `k = ${v.k_async}: at most ${v.k_async} snapshot${v.k_async === 1 ? "" : "s"} per cycle`,
    paras: [p(`While one robot is between its snapshot and the end of its move, no other robot may take more than ${v.k_async}`,
      "snapshots. Smaller values keep robots from reacting many times to a robot that has not finished moving; larger values",
      "let the timing become more and more like the unrestricted asynchronous scheduler.")],
  }),

  ssync_probability: (v) => ({
    title: `Each robot is active in a round with chance ${v.ssync_probability}`,
    paras: [p("At the start of each round every robot that is still working is chosen with this chance, independently of the",
      "others. A small value gives rounds with few robots, close to one robot at a time; a value of 1 makes every round a full",
      "round, like the fully synchronous scheduler. A round in which nobody was drawn activates one robot at random.")],
  }),

  ssync_fairness: (v) => ({
    title: `Fairness bound: never left out more than ${v.ssync_fairness} rounds`,
    paras: [p(`A robot that has been left out of ${v.ssync_fairness} rounds in a row is certainly active in the next one.`,
      "This keeps the random choice fair: every robot is guaranteed to act again within a bounded number of rounds, which",
      "many algorithms need in order to finish.")],
  }),

  activation_order: (v) => ({
    round_robin: {
      title: "Turn order: round-robin",
      paras: [p("Robots take their turns in the order of their numbers, 0, 1, 2, …, and start again from 0 at the next",
        "epoch. Robots that have crashed or terminated are skipped. The order is completely predictable, which makes runs",
        "easy to follow and to repeat.")],
    },
    random: {
      title: "Turn order: a new random order every epoch",
      paras: [p("At the start of every epoch the robots are shuffled, and they take their turns in that order. Every robot",
        "still gets exactly one turn per epoch. The shuffle comes from the random seed, so the same seed gives the same",
        "sequence of turns every time.")],
    },
  }[v.activation_order]),

  turn_gap: (v) => ({
    random: {
      title: "A random pause between turns",
      paras: [p("After a turn (or a round) ends, a random amount of simulated time passes before the next one starts, drawn",
        "with the Activation rate λ. This only stretches the clock; the order of events and the outcome do not change.")],
    },
    none: {
      title: "No pause between turns",
      paras: [p("The next turn (or round) starts at the very instant the previous one ends. Simulated time then only advances",
        "while robots move, which makes the clock a direct measure of how far robots have walked.")],
    },
  }[v.turn_gap]),

  multiplicity: (v, algo) => {
    const uses = algo && MULTIPLICITY_ALGOS.has(algo.key);
    const card = {
      none: {
        title: "No multiplicity detection",
        paras: [
          p("When several robots stand on exactly the same spot (a multiplicity, or stack), a robot cannot tell. It sees the",
            "spot as one occupied point, exactly like a spot with a single robot on it. Even its own spot looks the same whether",
            "it is alone there or not."),
          p("With this little information some tasks become impossible: a robot can never know whether the other robots have",
            "already met, so it cannot tell a finished gathering from an unfinished one."),
        ],
      },
      local: {
        title: "Local multiplicity detection",
        paras: [
          p("A robot can tell whether the spot it stands on is shared with other robots, but for every other spot it only",
            "sees that it is occupied, not whether one robot or several are there."),
          p("This is enough to know “I am part of a stack” but not to find stacks elsewhere, so a robot cannot walk towards the",
            "biggest group, only react to its own situation."),
        ],
      },
      weak: {
        title: "Weak multiplicity detection",
        paras: [
          p("For every occupied spot a robot can tell one thing more: whether one robot stands there, or more than one. It",
            "cannot count them: a spot with two robots and a spot with fifty look the same."),
          p("That single bit is often exactly what a gathering algorithm needs: once a stack exists, everybody can see it and",
            "walk to it, and a robot on a stack knows it is already where the others are heading."),
        ],
      },
      strong: {
        title: "Strong multiplicity detection",
        paras: [
          p("A robot sees exactly how many robots stand on each occupied spot. This is the most information a robot can have",
            "about stacks, and it lets an algorithm compare groups by size, for example to head for the largest one."),
        ],
      },
    }[v.multiplicity];
    if (!card) return null;
    if (uses) {
      card.tip = v.multiplicity === "weak" || v.multiplicity === "strong"
        ? `${algo.name} is built around seeing stacks, and this setting gives its robots what they need.`
        : `${algo.name} is built around seeing stacks. With this setting its robots cannot see them, so the run shows what goes wrong: the page notes it in the brief at the top.`;
    } else if (algo) {
      card.tip = `${algo.name} does not look at stacks, so this setting does not change its runs; it matters for algorithms that gather robots on one spot.`;
    }
    return card;
  },

  frames: (v) => ({
    global: {
      title: "Shared coordinate frame",
      paras: [p("Every robot measures positions with the same axes: the same north, the same unit of length and the same",
        "notion of clockwise. Robots effectively agree on a common map, which makes many decisions simpler.")],
    },
    rotated: {
      title: "Private frames: each robot has its own rotation and unit",
      paras: [
        p("At every snapshot a robot gets a coordinate system of its own: itself at the origin, its axes turned by a random",
          "angle and its unit of length scaled at random. Robots still agree on clockwise."),
        p("An algorithm that only uses distances and angles between robots behaves exactly as before, because those do not",
          "depend on the frame. One that relies on the axes, such as Spreading's grid, behaves differently: try it."),
      ],
    },
    disoriented: {
      title: "Disoriented robots: private frames, and no common clockwise",
      paras: [p("As with private frames, every snapshot comes in the robot's own rotated and scaled coordinates, and in",
        "addition the frame may be mirrored, so robots do not even agree on which way is clockwise. Algorithms must then rely",
        "only on what every robot can see the same way: distances, and the shape of the swarm up to a mirror image.")],
    },
    compass: {
      title: "Compasses with an error",
      paras: [
        p("Every robot has a compass, but it may point up to φ degrees away from true north (set φ just below). Robots agree",
          "on the unit of length and on clockwise; only their sense of direction is off."),
        p("Static compass keeps one error per robot for the whole run; turning it off draws a new error at every snapshot,",
          "which is harder for an algorithm because a robot's north keeps drifting."),
      ],
    },
  }[v.frames]),

  compass_error: (v) => ({
    title: `Compass error up to ${v.compass_error}°`,
    paras: [p(`A robot's north may be off by any angle up to ${v.compass_error}° in either direction.`,
      "Two-robot gathering with compasses stays reliable only while the error is small enough; past that bound the robots",
      "may circle each other for a very long time or never meet, and the brief at the top warns when the run is past it.")],
  }),

  compass_static: (v) => (v.compass_static ? {
    title: "Static compasses",
    paras: [p("Each robot draws its compass error once, at the start, and keeps it for the whole run. Its north is wrong,",
      "but always wrong in the same way, so its decisions are consistent from one snapshot to the next.")],
  } : {
    title: "Changing compasses",
    paras: [p("A new error is drawn at every snapshot, so a robot's north wobbles from one look to the next within ±φ.",
      "Rules that are safe for a fixed compass need a smaller error bound here, because two consecutive decisions may be",
      "based on different norths.")],
  }),

  rigid_movement: (v) => (v.rigid_movement ? {
    title: "Rigid movement: robots always arrive",
    paras: [p("A robot that decides to go somewhere always gets there in the same cycle. Nothing can stop it on the way,",
      "so the destination an algorithm computes is exactly where the robot ends up.")],
  } : {
    title: "Non-rigid movement: robots can be stopped on the way",
    paras: [
      p("A robot may be stopped before it reaches its destination, by an adversary that chooses where. It is never stopped",
        "too early, though: it always covers at least the minimum move δ, or reaches its destination if that is nearer."),
      p("Set δ just below; with δ = 0 robots still arrive, as in the original simulator. An algorithm that works with",
        "non-rigid movement must cope with robots that end their move somewhere it did not plan."),
    ],
  }),

  delta: (v) => ({
    title: v.delta > 0 ? `Minimum move δ = ${v.delta}` : "Minimum move δ = 0: robots always arrive",
    paras: [v.delta > 0
      ? p(`A robot heading somewhere farther than ${v.delta} units can be stopped anywhere after it has covered ${v.delta}`,
        "units; a destination closer than that is always reached. A small δ lets the adversary slow robots down a lot; it",
        "can never stop them completely, so they keep making progress.")
      : p("With δ = 0 non-rigid robots still reach their destination, exactly as in the original simulator. Raise δ to let",
        "the adversary stop robots on the way.")],
  }),

  stop_policy: (v) => ({
    delta: {
      title: "Stopped right after δ",
      paras: [p("Every robot that can be stopped is stopped as early as allowed: right after it has covered δ. This is the",
        "worst case for an algorithm, because robots make the least progress the rules permit.")],
    },
    random: {
      title: "Stopped at a random point",
      paras: [p("A robot that can be stopped ends its move somewhere between δ and its destination, at random (from the",
        "seed, so runs repeat). This shows typical behaviour rather than the worst case.")],
    },
    fraction: {
      title: "Stopped after a fixed fraction of the way",
      paras: [p("A robot that can be stopped covers the same fraction of its way every time (set it just below), but never",
        "less than δ. Useful for seeing how an algorithm behaves when every move falls short by the same proportion.")],
    },
  }[v.stop_policy]),

  unlimited_visibility: (v) => (v.unlimited_visibility ? {
    title: "Unlimited visibility",
    paras: [p("Every robot sees every other robot, however far away. Each snapshot is a complete picture of the swarm",
      "(unless robots block each other's view, below).")],
  } : {
    title: "Limited visibility",
    paras: [
      p("A robot only sees the robots within the Radius set below. It knows nothing about robots farther away, so it can",
        "lose track of parts of the swarm if it moves away from them."),
      p("Algorithms made for robots that see everyone may split the swarm into groups that never meet again; algorithms",
        "made for limited visibility move carefully so that robots that see each other keep seeing each other."),
    ],
  }),

  visibility_radius: (v) => ({
    title: `Robots see ${v.visibility_radius} units around them`,
    paras: [p("Press V (or “Show who sees whom” in the view tools) to draw a line between every two robots that see each other. If the",
      "robots do not form one connected network at the start, no algorithm can bring all of them together.")],
  }),

  obstructed_visibility: (v) => (v.obstructed_visibility ? {
    title: "Robots block the view",
    paras: [
      p("Robots are no longer transparent: a robot standing exactly between two others hides them from each other. With a",
        "Robot radius above 0, robots block the view as small discs instead of points."),
      p("Each snapshot now has to check every pair of robots for something in the way, which takes time proportional to",
        "the square of the number of robots: keep the swarm small (a few hundred robots at most)."),
    ],
  } : {
    title: "Transparent robots",
    paras: [p("Robots never hide each other: a robot sees every robot within its visibility range, even when others stand",
      "in between.")],
  }),

  robot_radius: (v) => ({
    title: v.robot_radius > 0 ? `Robots block the view as discs of radius ${v.robot_radius}` : "Robots block the view as points",
    paras: [v.robot_radius > 0
      ? p("A robot hides whatever lies behind it within its disc, so robots that are nearly (not exactly) in line also hide",
        "each other. Larger discs make the swarm much harder to see across.")
      : p("A point only hides robots exactly in line behind it, which rarely happens with positions chosen at random; it",
        "happens often in lines and grids.")],
  }),

  num_of_faults: (v) => ({
    title: v.num_of_faults > 0 ? `${v.num_of_faults} faulty robot${v.num_of_faults === 1 ? "" : "s"}` : "No faulty robots",
    paras: [v.num_of_faults > 0
      ? p(`${v.num_of_faults} robots, chosen at random from the seed, have the fault chosen below. Faulty robots are drawn in`,
        "their own colours (see the legend), and a run counts as finished once every working robot has terminated.")
      : p("Every robot works correctly: it looks, decides and moves exactly as its algorithm says, every time it is",
        "activated. Raise this to make some robots faulty and watch how the algorithm copes (choose the kind of fault below).")],
  }),

  fault_type: (v) => ({
    crash: { title: "Crash faults", paras: [p("A crashed robot stops for good: it never looks or moves again, but it stays where it",
      "is, and the other robots still see it. Algorithms that tolerate crashes must gather the working robots without waiting",
      "for the crashed ones.")] },
    byzantine: { title: "Byzantine faults", paras: [p("A Byzantine robot ignores the algorithm and moves erratically, to random points.",
      "The working robots cannot tell it apart from the others, so it can mislead them.")] },
    omission: { title: "Omission faults", paras: [p("An omission-faulty robot skips about half of its moves: it looks and decides as",
      "usual, but often stays where it is instead of moving.")] },
    delay: { title: "Delay faults", paras: [p("A delayed robot works correctly but moves at 40 % of the normal speed, so its moves",
      "take longer and other robots see it half-way more often.")] },
    mixed: { title: "Mixed faults", paras: [p("The faulty robots are split between the four kinds above: crash, Byzantine,",
      "omission and delay.")] },
  }[v.fault_type]),

  multiplicities: (v) => ({
    title: v.multiplicities > 0 ? `Start with ${v.multiplicities} stacked point${v.multiplicities === 1 ? "" : "s"}` : "No stacked points at the start",
    paras: [v.multiplicities > 0
      ? p(`The start positions are made as usual, then robots are moved on top of each other to make ${v.multiplicities}`,
        "spots where several robots stand together (multiplicities, or stacks). Stacked spots are drawn with an orange ring.",
        "Algorithms that see stacks treat them differently from single robots: try changing Multiplicity detection.")
      : p("Every robot starts on a spot of its own. Raise this to start with some robots stacked on top of each other.")],
    tip: v.multiplicities > 1
      ? "With several stacks, robots leaving one stack land closer and closer to it each time; very large stacks can run out of numerical precision. A few robots per stack is plenty."
      : undefined,
  }),

  multiplicity_size: (v) => ({
    title: v.multiplicity_size > 0 ? `${v.multiplicity_size} robots on each stacked point` : "Stack size chosen for you",
    paras: [v.multiplicity_size > 0
      ? p(`Each stacked point gets ${v.multiplicity_size} robots (at least 2). There must be enough robots for all of them.`)
      : p("With 0 the simulator picks a size: a third of the robots for a single stacked point, up to 4 robots each for",
        "several.")],
  }),

  threshold_precision: (v) => ({
    title: `Precision ε = 10⁻${v.threshold_precision}`,
    paras: [p("Two positions closer than ε count as the same point, and a move shorter than ε counts as no move at all.",
      "A smaller ε (higher number) lets robots make finer moves, which some algorithms need near the end of a run; a larger",
      "ε stops runs earlier, as soon as the remaining moves become too small to matter.")],
  }),

  lambda_rate: (v) => ({
    title: `Activation rate λ = ${v.lambda_rate}`,
    paras: [p(`Robots wake up after random delays with an average of 1/λ = ${(1 / v.lambda_rate).toFixed(2)} time units.`,
      "This only changes the speed of the simulated clock and how much robots overlap; it is not a speed of movement.")],
  }),

  open_world: (v) => (v.open_world ? {
    title: "Open world",
    paras: [p("There is no border: the start pattern can be anywhere and as large as you like, and robots can walk anywhere.")],
  } : {
    title: "The original world box",
    paras: [p("Robots start inside a box (World width × World height), as in the original simulator. The box only matters",
      "for where robots start; they can still move outside it.")],
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

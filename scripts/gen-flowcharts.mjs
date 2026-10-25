#!/usr/bin/env node
/* Generate the flowcharts from one set of chart definitions:

  docs/flowcharts.md   Mermaid charts (GitHub renders them), each followed by
                       the source of every function the chart names
  web/docs.html        the same charts as themed SVG in an interactive page
                       (web/docs.css, web/docs.js): every step explains itself,
                       steps that stand for a function show its source

Function source is copied out of the code, so a function shown is the
function that runs. A chart naming a function that no longer exists is an
error. Re-run after changing any function a chart names:

    node scripts/gen-flowcharts.mjs           # write both files
    node scripts/gen-flowcharts.mjs --check   # fail if either is out of date
*/
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const ROOT = path.dirname(path.dirname(fileURLToPath(import.meta.url)));
const REPO_URL = "https://github.com/drdebmath/LCMmodel/blob/main";

const SIM = "crates/lcm-core/src/sim.rs";
const GATHERING = "crates/lcm-algorithms/src/gathering.rs";
const GTC = "crates/lcm-algorithms/src/go_to_center.rs";
const SEC = "crates/lcm-algorithms/src/sec.rs";
const WASM = "crates/lcm-wasm/src/lib.rs";
const WORKER = "web/worker.js";
const CLIENT = "web/client.js";
const RENDERER = "web/renderer.js";
const SQ = "crates/lcm-algorithms/src/sq_gathering.rs";

// ------------------------------------------------------------- helpers ----

const len = (s) => [...s].length;

/** Escape for HTML text (quote=false) or text and attributes (default). */
function esc(s, quote = true) {
  s = s.replaceAll("&", "&amp;").replaceAll("<", "&lt;").replaceAll(">", "&gt;");
  return quote ? s.replaceAll('"', "&quot;").replaceAll("'", "&#x27;") : s;
}

/** x with d (0 or 1) decimals, rounding exact ties to even (printf-style;
 *  toFixed rounds them up). Only a multiple of 1/2 (d = 0) or 1/4 (d = 1) can
 *  be a tie in binary. */
function fmt(x, d = 1) {
  const k = d === 0 ? 2 : 4;
  if (Number.isInteger(x * k) && !Number.isInteger((x * k) / 2)) {
    const t = x * 10 ** d;
    const lo = Math.floor(t);
    return ((lo % 2 === 0 ? lo : lo + 1) / 10 ** d).toFixed(d);
  }
  return x.toFixed(d);
}

// ---------------------------------------------------------------- charts ----
// Nodes sit on a hand-placed grid (col, row) so a drawing stays stable when a
// node is added. Kinds: entry (where the chart starts), step, decision, end
// (a finish), out (another way this pass ends). `func` = (file, function) when
// the step stands for a function. Nodes are listed in reading order, which is
// also the order of the guided tour.
//
// Edge routes: "auto" (straight when aligned; otherwise out of the source's
// side, across, and into the target's top or bottom), "elbow" (down out of
// the source, then across into the target's side), ("around", col) (out of
// the source's side, along a vertical line at `col`, into the target's side).
//
// Lanes are background bands grouping rows ("rows") or columns ("cols").


const node = (id, title, col, row, kind = "step", func = null, sub = null, desc = "") =>
  ({ id, title, col, row, kind, func, sub, desc });

const edge = (from, to, label = null, route = "auto") => ({ from, to, label, route });

const CHARTS = [
    {
        "id": "event-loop",
        "title": "The event loop",
        "short": "How the scheduler takes and handles events",
        "blurb": (
            "The scheduler is a queue of timed events: each robot's next LOOK or WAIT, a crash, " +
            "and a periodic Visualize tick. advance() handles them one at a time, in time order, " +
            "until its budget, a limit or the end. This is Scheduler.handle_event in " +
            "the original scheduler, ported line by line."
        ),
        "notes": [
            "Ties in time break exactly as in Python's heap of (time, id, state) tuples: lower " +
            "robot id first, then Crash < Look < Visualize < Wait.",
            "Visualize ticks change nothing; they are kept so event counts and limits match the Python.",
            "The run ends when every robot that is neither crashed nor Byzantine has terminated.",
            "Under the sequential scheduler robots have no clocks of their own: only one Look is ever " +
            "in the queue, and the next one is queued when a turn ends (chart 5).",
        ],
        "lanes": ["rows", [["Run loop", 0, 1], ["Take the next event", 2, 6],
                           ["Handle it", 7, 8], ["After each robot event", 9, 10]]],
        "nodes": [
            node("adv", "Run the simulation", 1, 0, "entry", [SIM, "advance"], "advance(budget)",
                 "The worker calls this with a budget: how many events it may handle and the latest " +
                 "simulated time it may reach. It handles events one after another until a stop " +
                 "condition holds."),
            node("lim", "Time to stop?", 1, 1, "decision", null, "ended · limit · budget · time",
                 "Checked before every event, in this order: has the run ended; has max_time passed " +
                 "or max_events been handled (the same checks as the original simulator); is this call's event " +
                 "budget used up; is the next event later than the time this call may reach (this " +
                 "is how playback speed is paced)?"),
            node("ret", "Return why it stopped", 2, 0, "out", null, "StopReason",
                 "Ended, MaxTime, MaxEvents, Budget or UntilTime. The worker uses this to pace " +
                 "playback and to notice the end of the run."),
            node("pop", "Take the earliest event", 1, 2, "step", [SIM, "step"], "step()",
                 "Events wait in a priority queue ordered by (time, robot id, kind), exactly like " +
                 "Python's heap of tuples, so ties break the same way."),
            node("emp", "Queue empty?", 1, 3, "decision", null, null,
                 "Does not happen in practice, because the Visualize tick always reschedules " +
                 "itself; handled like the Python: an empty queue ends the run."),
            node("end1", "Ended", 2, 3, "end", null, "queue empty",
                 "The run is over; advance() reports Ended."),
            node("old", "Earlier than the clock?", 1, 4, "decision", null, null,
                 "Time never goes backwards. Python skips such an event without touching the " +
                 "clock, and so does the port."),
            node("ign", "Skip the event", 0, 4, "out", null, "clock unchanged",
                 "The event is dropped and the clock stays where it was; the loop takes the next event."),
            node("vis", "Visualize tick?", 1, 5, "decision", null, null,
                 "A periodic event every sampling_rate time units. It changes no robot; it is " +
                 "kept so event counts match the Python."),
            node("vis2", "Schedule the next tick", 0, 5, "step", null, "t + sampling_rate",
                 "The tick puts itself back in the queue one sampling interval later; no robot changes."),
            node("dead", "Robot out of play?", 1, 6, "decision", null, "terminated · or crashed, not a crash event",
                 "Yes if the robot has terminated, or if it has crashed and this is not its crash " +
                 "event. A crashed robot's own crash event is not skipped: it goes on to be " +
                 "recorded."),
            node("ign2", "Skip the event", 0, 6, "out", [SIM, "schedule_activation"], "crashed ones rescheduled",
                 "A crashed robot is rescheduled after a random delay and otherwise ignored; a " +
                 "terminated robot is simply ignored and never scheduled again. Both exactly as in " +
                 "the Python."),
            node("kind", "Which kind of event?", 1, 7, "decision", null, null,
                 "A robot event is a LOOK (activate), a WAIT (a move ends) or a CRASH (a crash fault takes effect)."),
            node("look", "Look, compute, move", 0, 8, "step", [SIM, "look"], "look() · chart 2",
                 "The robot's activation: snapshot, compute a target, start moving. Chart 2 shows " +
                 "every branch."),
            node("wait", "Finish the move", 1, 8, "step", [SIM, "wait"], "wait() · then next LOOK",
                 "The WAIT event fires at the arrival time: the robot is placed on its target, " +
                 "its light turns green and its next activation is scheduled after a random " +
                 "exponential delay."),
            node("crash", "Record the crash", 2, 8, "step", null, "crash fault",
                 "Crash-faulty robots are crashed from the start (set when the run is built), so " +
                 "they never move; their single crash event only records it, as in the Python."),
            node("glob", "Every live robot terminated?", 1, 9, "decision", [SIM, "globally_terminated"],
                 "globally_terminated()",
                 "Crashed and Byzantine robots don't count: the run ends once all the others " +
                 "have terminated."),
            node("end2", "Ended", 1, 10, "end", null, "all terminated",
                 "advance() returns Ended and the page shows that every robot terminated."),
        ],
        "edges": [
            edge("adv", "lim"),
            edge("lim", "ret", "yes"),
            edge("lim", "pop", "no"),
            edge("pop", "emp"),
            edge("emp", "end1", "yes"),
            edge("emp", "old", "no"),
            edge("old", "ign", "yes"),
            edge("old", "vis", "no"),
            edge("vis", "vis2", "yes"),
            edge("vis", "dead", "no"),
            edge("dead", "ign2", "yes"),
            edge("dead", "kind", "no"),
            edge("kind", "look", "Look"),
            edge("kind", "wait", "Wait"),
            edge("kind", "crash", "Crash"),
            edge("look", "glob", null, "elbow"),
            edge("wait", "glob"),
            edge("crash", "glob", null, "elbow"),
            edge("glob", "end2", "yes"),
            edge("glob", "lim", "no · next event", ["around", 2.62]),
            edge("ign", "lim", null, ["around", -0.66]),
            edge("vis2", "lim", null, ["around", -0.66]),
            edge("ign2", "lim", null, ["around", -0.66]),
        ],
    },
    {
        "id": "look-compute-move",
        "title": "One Look–Compute–Move cycle",
        "short": "Everything a robot does when it activates",
        "blurb": (
            "The robot takes a snapshot (LOOK), its algorithm computes a target (COMPUTE), and it " +
            "moves there (MOVE); a WAIT event at the arrival time ends the move. Faults, freezing " +
            "and termination all branch off this path. This is Robot.look, Robot.move and " +
            "Robot.wait in the original simulator."
        ),
        "notes": [
            "The snapshot is taken at the exact LOOK time: moving robots are seen at their " +
            "interpolated position, never a stale one.",
            "A robot freezes (stays put this cycle) when its target is within ε = 10^-precision of " +
            "where it is, or when an omission fault skips the move (probability ½).",
            "Rigid movement always reaches the target. Non-rigid movement does too (finding #3 in " +
            "report.md) unless a minimum move δ is set: then the robot may be stopped on the way, " +
            "never before it has covered δ, and the WAIT event is scheduled at the stopping point (chart 5).",
        ],
        "lanes": ["rows", [["LOOK", 0, 4], ["COMPUTE", 5, 7], ["MOVE", 8, 11]]],
        "nodes": [
            node("start", "Robot activates", 1, 0, "entry", [SIM, "look"], "LOOK event · look()",
                 "The robot's LOOK event comes up in the queue. Everything on this chart happens " +
                 "at that single instant, except the final WAIT."),
            node("view", "Take a snapshot", 1, 1, "step", [SIM, "build_view"], "build_view()",
                 "Every robot's position at this exact time (moving robots are interpolated along " +
                 "their path), keeping those within the visibility radius. The robot sees itself too."),
            node("lit", "Light turns blue", 1, 2, "step", null, "state = Look",
                 "Lights change at most once every 0.5 time units, as in the Python, so a light " +
                 "can lag behind the robot's phase."),
            node("byz", "Byzantine robot?", 1, 3, "decision", null, null,
                 "A Byzantine robot ignores its algorithm entirely."),
            node("byzt", "Pick an erratic target", 2, 3, "step", null, "random direction",
                 "It moves in a random direction by half the distance to the farthest robot it " +
                 "sees (10 if it sees none), and never terminates."),
            node("alone", "Only itself still active?", 1, 4, "decision", null, null,
                 "If every other visible robot has crashed or terminated, or none is visible, the " +
                 "robot terminates. With limited visibility an isolated robot stops: the " +
                 "original's behaviour."),
            node("term1", "Terminate", 0, 4, "end", null, "nobody left to see",
                 "The robot is marked terminated (and frozen) and is never activated again."),
            node("comp", "Algorithm computes a target", 1, 5, "step", [GATHERING, "compute"],
                 "compute() · chart 3",
                 "The algorithm sees only the snapshot and returns a target, whether to " +
                 "terminate, and optionally a circle to display. Robots are oblivious: nothing is " +
                 "remembered between activations."),
            node("tdec", "Algorithm says terminate?", 1, 6, "decision", null, null,
                 "Each algorithm has its own test; for Gathering, every live visible robot is within ε of the average."),
            node("term2", "Terminate", 0, 6, "end", null, "for good",
                 "The robot stops permanently and is drawn grey."),
            node("frz", "Skip, or already there?", 1, 7, "decision", null, "omission ½ · target within ε",
                 "An omission-faulty robot skips the move half the time. Any robot whose target is " +
                 "closer than ε = 10^-precision stays put."),
            node("frozen", "Stay put, look again later", 0, 7, "out", [SIM, "schedule_activation"], "frozen",
                 "The robot is marked frozen and its next activation is scheduled after a random " +
                 "exponential delay."),
            node("mv", "Start moving", 1, 8, "step", [SIM, "begin_move"], "begin_move() · light red",
                 "The move starts now at the robot's speed (40 % for a delay fault); its position " +
                 "is interpolated until it arrives. With non-rigid movement and a minimum move δ, " +
                 "the robot walks only as far as the point where the adversary stops it (chart 5); its " +
                 "target still shows what the algorithm computed."),
            node("sub", "Arrival later than now?", 1, 9, "decision", null, null,
                 "A tiny move can be shorter than the clock can resolve. As in the Python fix for " +
                 "the endless-simulation bug, it is finished immediately instead of scheduling a " +
                 "WAIT at the same instant."),
            node("subt", "Finish the tiny move now", 0, 9, "out", null, "look again later",
                 "The robot is placed on its target at once and its next activation is scheduled."),
            node("sched", "Schedule WAIT at arrival", 1, 10, "step", null, "t + distance / speed",
                 "A WAIT event goes in the queue for the moment the robot reaches its target; other robots see it moving until then."),
            node("arr", "Arrive", 1, 11, "end", [SIM, "wait"], "wait() · light green · next LOOK",
                 "At the arrival time the robot is placed exactly on its target, its light turns " +
                 "green, and its next LOOK is scheduled."),
        ],
        "edges": [
            edge("start", "view"),
            edge("view", "lit"),
            edge("lit", "byz"),
            edge("byz", "byzt", "yes"),
            edge("byz", "alone", "no"),
            edge("byzt", "mv", null, "elbow"),
            edge("alone", "term1", "yes"),
            edge("alone", "comp", "no"),
            edge("comp", "tdec"),
            edge("tdec", "term2", "yes"),
            edge("tdec", "frz", "no"),
            edge("frz", "frozen", "yes"),
            edge("frz", "mv", "no"),
            edge("mv", "sub"),
            edge("sub", "subt", "no"),
            edge("sub", "sched", "yes"),
            edge("sched", "arr", "at arrival"),
        ],
    },
    {
        "id": "gathering",
        "title": "Gathering (center of gravity)",
        "short": "Each robot goes to the average of what it sees",
        "blurb": (
            "Each robot moves to the average position of the robots it sees. Repeated " +
            "asynchronously, the swarm converges to a single point. This is Robot._midpoint and " +
            "Robot._midpoint_terminal in the original simulator."
        ),
        "notes": [
            "The average includes crashed robots, but the termination test ignores them: the " +
            "original's behaviour, kept on purpose (finding #4 in report.md).",
            "The sum is a plain left-to-right loop, as in the Python, so results match to the last digits.",
            "Only guaranteed to gather with unlimited visibility. With a limited radius the swarm can stop " +
            "in several separate groups, all marked terminated: a robot that sees nobody terminates (the " +
            "original's rule, chart 2), and moving to the average can break visibility links. Algorithms " +
            "built for limited visibility, such as Go-To-Center (Ando et al.), limit the step length to keep them.",
        ],
        "lanes": ["rows", [["Target", 0, 1], ["Termination", 2, 3]]],
        "nodes": [
            node("a", "Compute", 1, 0, "entry", [GATHERING, "compute"], "Gathering::compute()",
                 "Called from the COMPUTE phase of chart 2 with the robot's snapshot."),
            node("b", "Average of visible robots", 1, 1, "step", null, "crashed ones included",
                 "Sum of every visible robot's position, divided by how many there are."),
            node("c", "All live robots within ε?", 1, 2, "decision", null, "ε = 10^-precision",
                 "Every visible robot that has not crashed must be within ε of the average."),
            node("d", "Terminate", 0, 3, "end", null, "gathered",
                 "The robot has gathered with everyone it can see and stops for good."),
            node("e", "Move to the average", 2, 3, "out", null, "target = average",
                 "The average becomes the robot's target; chart 2 takes over (freeze check, then the move)."),
        ],
        "edges": [
            edge("a", "b"),
            edge("b", "c"),
            edge("c", "d", "yes"),
            edge("c", "e", "no"),
        ],
    },
    {
        "id": "browser",
        "title": "In the browser",
        "short": "Page, worker and Rust core working together",
        "blurb": (
            "The page never runs the simulation itself. A Web Worker owns the Rust core and " +
            "simulates in short slices at the chosen playback speed; the page asks for the latest " +
            "frame whenever it is ready to draw, so drawing never slows the simulation."
        ),
        "notes": [
            "Frames travel as typed arrays that are transferred, not copied or turned into JSON; the " +
            "page hands each frame's arrays back to be refilled, so playback allocates nothing per frame.",
            "The worker computes in ~6 ms slices and yields between them, so messages (frame requests, " +
            "pause, speed changes) get through immediately.",
        ],
        "lanes": ["cols", [["Page · main thread", 0, 0], ["Worker thread", 1, 1], ["Rust core · WebAssembly", 2, 2]]],
        "nodes": [
            node("raf", "Every screen frame", 0, 0, "entry", [CLIENT, "startPulling"], "startPulling()",
                 "While playing, the page runs one loop per screen refresh."),
            node("ask", "Ask for the latest frame", 0, 1, "step", [CLIENT, "requestFrame"], "requestFrame()",
                 "At most one request is in flight. The previous frame's arrays go back to the worker " +
                 "to be refilled."),
            node("slice", "Simulate for ~6 ms", 1, 0, "entry", [WORKER, "slice"], "slice()",
                 "Runs continuously while playing, pacing itself to the playback speed, and yields " +
                 "between slices so messages get through."),
            node("adv", "Handle events", 2, 0, "step", [WASM, "advance"], "advance() · chart 1",
                 "The core handles a batch of events; the batch size adapts so one call takes about " +
                 "1 ms."),
            node("post", "Send the frame", 1, 2, "step", [WORKER, "postFrame"], "postFrame()",
                 "Answers a frame request with the current state and transfers the arrays to the page."),
            node("fill", "Fill the arrays", 2, 2, "step", [WASM, "fill_frame"], "fill_frame()",
                 "Positions, state flags, lights, faults, targets and circles, written straight into " +
                 "the recycled arrays."),
            node("draw", "Draw the swarm", 0, 3, "step", [RENDERER, "draw"], "draw() · CPU Canvas2D",
                 "Tiny robots are drawn as rectangles, larger ones as pre-drawn sprites per colour; at " +
                 "most one draw per screen frame. 10,000 robots take about 4–6 ms without a GPU."),
            node("stats", "Update stats and legend", 0, 4, "end", null, "a few times a second",
                 "Time, events per second, terminated count and the legend counts, refreshed at most every 200 ms so the page stays light."),
        ],
        "edges": [
            edge("raf", "ask"),
            edge("ask", "post", "frame request"),
            edge("slice", "adv", "loop"),
            edge("post", "fill"),
            edge("post", "draw", "transfer"),
            edge("draw", "stats"),
        ],
    },
    {
        "id": "sequential",
        "title": "The sequential scheduler",
        "short": "One robot at a time, turn by turn",
        "blurb": (
            "Under the sequential scheduler only one robot's Look is ever waiting in the queue. When " +
            "its turn ends, next_turn() decides who goes next, and the others stay still until then. " +
            "This is the sequential scheduler of Section 2.2 of \"Universal pattern formation by " +
            "oblivious robots under sequential schedulers\" (arXiv:2412.10733); it is new in the " +
            "Rust core, not in the Python original."
        ),
        "notes": [
            "Round-robin and the shuffle each epoch both give every robot that is still active " +
            "exactly one turn per epoch. An explicit list of turns ends its epoch once every robot " +
            "has had at least one, as in the paper.",
            "Crashed and terminated robots are skipped. The paper has no crashes; this is the " +
            "simulator's own rule.",
            "Stalled is not in the paper either. Robots are deterministic and remember nothing, so " +
            "if a whole epoch changes nothing, every later epoch is the same and the run can never " +
            "end. It is not checked with faults, because an omission fault skips moves at random.",
            "A run can also end at a turn limit (max_turns): the last turn's move still finishes. It " +
            "says nothing about the algorithm, only that the budget ran out.",
            "With non-rigid movement, a robot whose destination is within δ always arrives; " +
            "otherwise the adversary stops it somewhere at least δ along its way. δ is not shown to the " +
            "algorithms: in the paper, the robots do not know it.",
        ],
        "lanes": ["rows", [["A turn ends", 0, 3], ["Who goes next", 4, 6], ["The turn", 7, 9]]],
        "nodes": [
            node("t_end", "A robot's turn ends", 1, 0, "entry", [SIM, "next_turn"], "next_turn()",
                 "A turn ends when the robot arrives, decides to stay put, or terminates. Nobody else " +
                 "has moved in the meantime, so the next robot sees every robot where it stands."),
            node("t_over", "Epoch over?", 1, 1, "decision", null, "every live robot has had a turn",
                 "An epoch ends as soon as every robot that is still active has taken a turn. For " +
                 "round-robin and the shuffle, that is when the epoch's list runs out."),
            node("t_prog", "Anything moved or terminated?", 1, 2, "decision", null, "this epoch · no faults",
                 "Did any robot move (by at least ε) or terminate since the epoch began? Without faults, " +
                 "robots are deterministic, so an epoch in which nothing changed would repeat forever."),
            node("t_stall", "Stalled", 2, 2, "end", null, "the run stops",
                 "advance() returns Stalled; the page shows that nothing can move any more. This also " +
                 "ends a run with no pause between turns, where simulated time would otherwise stand still."),
            node("t_new", "Start the next epoch", 1, 3, "step", null, "epochs + 1 · reshuffle if random",
                 "The epoch counter goes up. With a random order, the robots are shuffled again from " +
                 "the run's seed; with round-robin they go 0, 1, 2, … again."),
            node("t_pick", "Pick the next live robot", 1, 4, "step", null, "listed turn or next in the epoch",
                 "The next turn of the explicit schedule if there is one (then round-robin or a repeat), " +
                 "otherwise the next robot of the epoch. Crashed and terminated robots are skipped."),
            node("t_gap", "Wait out the pause", 1, 5, "step", null, "random (rate λ) or none",
                 "Simulated time between one turn ending and the next Look: random with rate λ, or none, " +
                 "when the next robot looks at the very instant the last one stopped."),
            node("t_look", "Queue its Look", 1, 6, "step", null, "the only Look in the queue",
                 "The robot's Look event goes into the queue, with the stopping point the schedule " +
                 "names for this turn, if any."),
            node("t_run", "It looks, computes and moves", 1, 7, "step", null, "look() · chart 2",
                 "The turn itself: snapshot, compute a target, start moving. Chart 2 shows every branch."),
            node("t_lim", "Non-rigid, and farther than δ?", 1, 8, "decision", [SIM, "limited_target"],
                 "limited_target()",
                 "Only with rigid movement off and a minimum move δ set. A destination within δ is " +
                 "always reached; a farther one may be cut short by the adversary."),
            node("t_stop", "Stop it on the way", 0, 8, "step", null, "never before δ",
                 "The adversary picks the stopping point, never before δ along the way: just after δ, " +
                 "anywhere between δ and the destination at random, a set fraction of the way, or the " +
                 "fraction this turn of the schedule names."),
            node("t_arr", "Arrive, and the turn ends", 1, 9, "end", null, "next turn",
                 "The robot stands at its destination, or at the point where it was stopped. It does not " +
                 "schedule its own next Look: control goes back to next_turn()."),
        ],
        "edges": [
            edge("t_end", "t_over"),
            edge("t_over", "t_prog", "yes"),
            edge("t_over", "t_pick", "no", ["around", -0.66]),
            edge("t_prog", "t_stall", "no"),
            edge("t_prog", "t_new", "yes"),
            edge("t_new", "t_pick"),
            edge("t_pick", "t_gap"),
            edge("t_gap", "t_look"),
            edge("t_look", "t_run"),
            edge("t_run", "t_lim"),
            edge("t_lim", "t_stop", "yes"),
            edge("t_lim", "t_arr", "no"),
            edge("t_stop", "t_arr", null, "elbow"),
            edge("t_arr", "t_end", "next turn", ["around", 2.62]),
        ],
    },
    {
        "id": "sqgathering",
        "title": "SqGathering (Algorithm 8)",
        "short": "Gathering with weak multiplicity detection, one robot per turn",
        "blurb": (
            "Algorithm 8 of \"Universal pattern formation by oblivious robots under sequential " +
            "schedulers\" (arXiv:2412.10733, Section 6). The active robot sees each occupied point " +
            "only as single or multiplicity, counts the multiplicities κ, and follows one of three " +
            "rules. It runs only under the sequential scheduler of chart 5: the registry refuses the " +
            "async one."
        ),
        "notes": [
            "Weak multiplicity detection: one bit per point, never a count. Two robots on a point and " +
            "five robots on a point look the same.",
            "Why the sequential scheduler: without multiplicity detection Gathering is unsolvable " +
            "under any sequential scheduler (Theorem 4); with more than one robot active at once it is " +
            "unsolvable even with detection (Lemma 8). So SqGathering refuses the async scheduler, " +
            "limited visibility and faults.",
            "In the paper robots never terminate. Here a robot that sees a single occupied point " +
            "terminates: everyone is there, and every robot activated later would stay, so it changes " +
            "nothing and lets the run end.",
            "Points closer than ε (10^-precision) are one point. Equally close points are told apart " +
            "by their coordinates; any fixed choice is allowed.",
            "A move cut short after δ can stop on a single robot and make a second multiplicity; the " +
            "κ > 1 rule then separates them again.",
            "With several large multiplicities, each robot leaving one halves the distance of the " +
            "robot that left before it. After enough of them it is closer than ε, cannot leave, and " +
            "the run ends Stalled (chart 5): a precision limit, not a flaw of the algorithm.",
        ],
        "lanes": ["rows", [["From the snapshot", 0, 1], ["Gathered?", 2, 2], ["Algorithm 8", 3, 5]]],
        "nodes": [
            node("q_comp", "Compute", 2, 0, "entry", [SQ, "compute"], "SqGathering::compute()",
                 "Called from the COMPUTE phase of chart 2 with the robot's snapshot, during its turn " +
                 "of the sequential scheduler (chart 5)."),
            node("q_obs", "See Q′(t)", 2, 1, "step", [SQ, "observe"], "observe() · points within ε merge",
                 "Robots closer than ε are grouped into points. Each point keeps one bit: single, or " +
                 "multiplicity (two or more robots). The robot also knows which point is its own."),
            node("q_one", "One occupied point?", 2, 2, "decision", null, "everyone stands here",
                 "If the robot sees a single point, every robot is on it (visibility is unlimited): the " +
                 "swarm has gathered. Then κ = 1 and b = 1, so Algorithm 8 would keep it there anyway."),
            node("q_term", "Terminate", 0, 2, "end", null, "gathered",
                 "The robot stops for good and gets no more turns. Once every robot has had its turn and " +
                 "terminated, the run ends (chart 1). The paper's robots never stop; terminating here " +
                 "changes nothing, because every robot activated later would stay put."),
            node("q_kappa", "How many multiplicities κ?", 2, 3, "decision", [SQ, "decide"], "decide() · lines 1-3",
                 "Count the points marked multiplicity, and look at b, the bit of the robot's own point. " +
                 "Lines 1-3 of Algorithm 8."),
            node("q_k0", "Move to the closest point", 0, 4, "out", [SQ, "closest"], "κ = 0 · lines 14-16",
                 "No multiplicity yet: go to the nearest occupied point to create one there. Nothing lies " +
                 "between the robot and that point, so on the way it never meets a third robot. The move " +
                 "goes on in chart 2; the adversary may stop it after δ (chart 5)."),
            node("q_k1", "On the multiplicity?", 2, 4, "decision", null, "κ = 1 · b = 1? · lines 4-5",
                 "With exactly one multiplicity, it is the gathering point. b tells the robot whether its " +
                 "own point is that multiplicity."),
            node("q_km", "On a multiplicity?", 4, 4, "decision", null, "κ > 1 · b = 1? · lines 9-10",
                 "Several multiplicities: robots must break all but one of them up. Only robots standing " +
                 "on one (b = 1) act."),
            node("q_hold", "Stay on it", 1, 5, "out", null, "κ = 1, b = 1 · lines 3-5",
                 "The robot is on the gathering point: its target is where it stands. Chart 2 sees a " +
                 "target within ε and freezes it; its turn ends (chart 5)."),
            node("q_join", "Move to the multiplicity", 2, 5, "out", null, "κ = 1, b = 0 · lines 4-7",
                 "Head for the gathering point. The adversary may stop it after δ (chart 5); it carries on " +
                 "in a later turn. If it is stopped exactly on a single robot, that spot becomes a second " +
                 "multiplicity, κ > 1, and the robots sort it out (hand trace D)."),
            node("q_wait", "Wait", 3, 5, "out", null, "κ > 1, b = 0 · lines 3, 9-10",
                 "A single robot waits while the multiplicities sort themselves out: its target is where it " +
                 "stands, so chart 2 freezes it and its turn ends."),
            node("q_leave", "Move to the midpoint", 4, 5, "out", [SQ, "destination"],
                 "κ > 1, b = 1 · lines 9-13",
                 "Leave the multiplicity for the midpoint between its point q and the closest occupied " +
                 "point q′. Nothing occupied is within half that distance, so the robot lands on a free " +
                 "spot wherever it is stopped. Once only one multiplicity is left, κ = 1."),
        ],
        "edges": [
            edge("q_comp", "q_obs"),
            edge("q_obs", "q_one"),
            edge("q_one", "q_term", "yes"),
            edge("q_one", "q_kappa", "no"),
            edge("q_kappa", "q_k0", "κ = 0"),
            edge("q_kappa", "q_k1", "κ = 1"),
            edge("q_kappa", "q_km", "κ > 1"),
            edge("q_k1", "q_hold", "yes"),
            edge("q_k1", "q_join", "no"),
            edge("q_km", "q_wait", "no"),
            edge("q_km", "q_leave", "yes"),
        ],
    },
    {
        "id": "go-to-center",
        "title": "Go-To-Center",
        "short": "Head for the centre without losing sight of anyone",
        "blurb": (
            "Each robot heads for the centre of the smallest circle around the robots it sees, but never so " +
            "far that it could lose sight of a neighbour. This is Robot._go_to_center and Robot._gtc_terminal in " +
            "the original simulator, from Ando, Oasa, Suzuki and Yamashita (1999). With limited visibility it keeps a " +
            "swarm that starts connected together, which simple centre-of-gravity gathering does not."
        ),
        "notes": [
            "The limit: for each visible neighbour j, the destination must stay inside the disc of radius V/2 around " +
            "the midpoint of the robot and j. Both robots then stay within V of each other whatever else happens. " +
            "The step is the farthest point of the way to the centre that is inside all of these discs.",
            "A robot stopped part-way by non-rigid movement is still on that segment, inside every disc, so no " +
            "link is lost either. Robots that start within sight of each other stay within sight in all the runs " +
            "tested, under the sequential and the asynchronous scheduler.",
            "The smallest circle is the same randomized routine as SEC's, run for every group of two or more " +
            "robots, so it draws the same random numbers as the Python and the runs match event by event.",
            "A robot is done when the circle around what it sees has a radius below ε. Under the sequential " +
            "scheduler this can deadlock: the robots that define the circle have stopped, the rest are " +
            "within ε of its centre so they do not move, and the radius is just above ε, so nobody finishes. " +
            "About 7 in 100 sequential runs of 8 robots do this at the default precision; the swarm is then within " +
            "0.00002 of one point and the core ends the run as Stalled. The original Python, which only ran " +
            "asynchronously, finished all 139 of the same starts.",
        ],
        "lanes": ["rows", [["Who is there", 0, 1], ["The centre", 2, 3], ["How far", 4, 6]]],
        "nodes": [
            node("g_a", "Compute", 1, 0, "entry", [GTC, "compute"], "GoToCenter::compute()",
                 "Called from the COMPUTE phase of chart 2 with the robot's snapshot."),
            node("g_b", "Sees another robot?", 1, 1, "decision", null, "crashed ones do not count",
                 "The robots in the snapshot that have not crashed, itself included. Alone, there is nobody to go to."),
            node("g_c", "Stay, and done", 0, 1, "end", null, "nobody to go to",
                 "The robot stays where it is and is finished. The circle it last showed is left alone."),
            node("g_d", "Smallest circle of what it sees", 1, 2, "step", [SEC, "welzl_recur"], "randomized, as in the Python",
                 "Shuffles the visible points with the run's random numbers, then Welzl's recursion. The same " +
                 "routine as SEC's, so the random numbers are drawn in the same order as the original."),
            node("g_e", "Already at the centre?", 1, 3, "decision", null, "within ε",
                 "The distance from the robot to the circle's centre is below ε = 10^-precision."),
            node("g_f", "Stay put", 0, 3, "out", null, "done if the circle is a point",
                 "The robot does not move. It is finished only if the circle's radius is below ε."),
            node("g_g", "Limited visibility?", 1, 4, "decision", null, null,
                 "With unlimited visibility every robot sees all the others and nothing limits the step."),
            node("g_h", "Go all the way to the centre", 2, 4, "step", null, "target = the centre",
                 "The centre of the smallest circle becomes the target."),
            node("g_i", "Limit the step for each neighbour", 1, 5, "step", null, "inside every V/2 disc",
                 "For each visible neighbour, the farthest point on the way to the centre that is inside the disc of " +
                 "radius V/2 around the midpoint with it. The step is the smallest of these limits."),
            node("g_j", "Move there", 1, 6, "end", null, "chart 2 takes over",
                 "The destination is the target; chart 2 (freeze check, then the move) takes over. The robot " +
                 "terminates if the circle's radius is below ε."),
        ],
        "edges": [
            edge("g_a", "g_b"),
            edge("g_b", "g_c", "no"),
            edge("g_b", "g_d", "yes"),
            edge("g_d", "g_e"),
            edge("g_e", "g_f", "yes"),
            edge("g_e", "g_g", "no"),
            edge("g_g", "g_h", "no"),
            edge("g_g", "g_i", "yes"),
            edge("g_i", "g_j"),
            edge("g_h", "g_j", null, "elbow"),
        ],
    },
]
// -------------------------------------------------------- source extraction --

const reEscape = (s) => s.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
const lstrip = (s) => s.replace(/^\s+/, "");

/** Return [first line number, source] of function `name` in `file`, with the
 *  doc comments above it. Relies on the formatter putting a function's closing
 *  brace at the indentation of its first line. */
function extract(file, name) {
  const lines = fs.readFileSync(path.join(ROOT, file), "utf-8").split("\n");
  const start = file.endsWith(".rs")
    ? new RegExp(`^\\s*(pub(\\([a-z]+\\))? )?(const )?fn ${reEscape(name)}\\b`)
    : new RegExp(`^\\s*(export )?(async )?(function )?${reEscape(name)}\\s*\\(.*\\)\\s*\\{\\s*$`);
  for (let i = 0; i < lines.length; i++) {
    if (!start.test(lines[i])) continue;
    const indent = lines[i].slice(0, lines[i].length - lstrip(lines[i]).length);
    let first = i;
    while (first > 0 && /^\s*(\/\/\/|#\[)/.test(lines[first - 1])) first--;
    for (let j = i; j < lines.length; j++) {
      if (lines[j] === indent + "}") {
        const body = lines.slice(first, j + 1).map((l) => (l.startsWith(indent) ? l.slice(indent.length) : l));
        return [first + 1, body.join("\n")];
      }
    }
    break;
  }
  console.error(`gen-flowcharts: function \`${name}\` not found in ${file}`);
  process.exit(1);
}

// ------------------------------------------------------ syntax highlighting --

const KEYWORDS = {
  rust: new Set(("as break const continue crate dyn else enum fn for if impl in let loop match mod move mut pub " +
    "ref return self Self static struct super trait type unsafe use where while true false").split(" ")),
  js: new Set(("async await break case catch class const continue default delete do else export extends false " +
    "finally for from function if import in instanceof let new null of return static super switch " +
    "this throw true try typeof undefined var void while").split(" ")),
};
const TOKEN = new RegExp([
  String.raw`(?<comment>//[^\n]*|/\*[\s\S]*?\*/)`,
  String.raw`(?<string>"(?:\\.|[^"\\])*"|\`(?:\\.|[^\`\\])*\`|'(?:\\.|[^'\\\n])'|(?<=[\s(,=:])'(?:\\.|[^'\\\n])*')`,
  String.raw`(?<attr>#\[[^\]\n]*\])`,
  String.raw`(?<number>\b\d[\d_]*(?:\.\d[\d_]*)?(?:[eE][+-]?\d+)?(?:f64|f32|u8|u16|u32|u64|usize|i32|i64)?\b)`,
  String.raw`(?<macro>\b[a-z_][a-z0-9_]*!)`,
  String.raw`(?<ident>\b[A-Za-z_][A-Za-z0-9_]*\b)`,
].join("|"), "g");
const TOKEN_CLASS = { comment: "com", string: "str", attr: "attr", number: "num", macro: "fn" };

/** Source as HTML lines with token classes (kw, ty, fn, str, num, com, attr). */
function highlight(src, lang) {
  const tokens = [];
  let pos = 0;
  for (const m of src.matchAll(TOKEN)) {
    if (m.index > pos) tokens.push([null, src.slice(pos, m.index)]);
    const text = m[0];
    const kind = Object.keys(m.groups).find((k) => m.groups[k] !== undefined);
    let cls = TOKEN_CLASS[kind] ?? null;
    const end = m.index + text.length;
    if (kind === "ident") {
      if (KEYWORDS[lang].has(text) || (lang === "rust" && ["Some", "None", "Ok", "Err"].includes(text))) {
        cls = "kw";
      } else if (/[A-Z]/.test(text[0])) {
        cls = "ty";
      } else if (src.slice(end, end + 1) === "(") {
        cls = "fn";
      }
    }
    tokens.push([cls, text]);
    pos = end;
  }
  tokens.push([null, src.slice(pos)]);
  const lines = [];
  let current = [];
  for (const [cls, text] of tokens) {
    text.split("\n").forEach((part, k) => {
      if (k) {
        lines.push(current.join(""));
        current = [];
      }
      if (part) {
        const e = esc(part, false);
        current.push(cls ? `<span class="${cls}">${e}</span>` : e);
      }
    });
  }
  lines.push(current.join(""));
  return lines;
}

// ---------------------------------------------------------------- layout ----

const COL = 280, ROW = 94;
const CARD_MIN = 176, CARD_MAX = 252;
let LEFT_PAD = 0.78;   // columns of room left of column 0; set per chart by place()
const TOP_PAD = 30;    // room above row 0 for column-lane labels
const CHAR = 6.9;      // average px per character of a 13 px semibold title
const SUB_CHAR = 6.6;  // 11.5 px monospace

function wrap(text, limit) {
  const words = text.split(/\s+/).filter(Boolean);
  const lines = [];
  let cur = "";
  for (const w of words) {
    if (cur && len(cur) + 1 + len(w) > limit) {
      lines.push(cur);
      cur = w;
    } else {
      cur = `${cur} ${w}`.trim();
    }
  }
  if (cur) lines.push(cur);
  return lines;
}

const xcol = (col) => 40 + (col + LEFT_PAD) * COL;
const yrow = (row, top) => top + row * ROW + ROW / 2;
const isBus = (e) => Array.isArray(e.route);

/** Room left of column 0: enough for a route bus there, else a margin. */
function leftPad(chart) {
  const buses = chart.edges.filter((e) => isBus(e) && e.route[1] < 0).map((e) => e.route[1]);
  return buses.length ? -Math.min(...buses) + 0.5 : 0.56;
}

function place(chart) {
  LEFT_PAD = leftPad(chart);
  const top = chart.lanes[0] === "cols" ? TOP_PAD : 12;
  const placed = {};
  for (const n of chart.nodes) {
    const title = wrap(n.title, 26);
    const sub = n.sub !== null ? n.sub : n.func ? `${n.func[1]}()` : null;
    const textW = Math.max(...title.map((t) => len(t) * CHAR), ...(sub ? [len(sub) * SUB_CHAR] : []));
    // 40 px icon column + 18 px right padding, and room for the </> badge.
    const w = Math.min(CARD_MAX + (n.func ? 28 : 0), Math.max(CARD_MIN, textW + 58 + (n.func ? 30 : 0)));
    const h = 22 + 17 * title.length + (sub ? 17 : 0);
    placed[n.id] = { ...n, sub, lines: title, cx: xcol(n.col), cy: yrow(n.row, top), hw: w / 2, hh: Math.max(26, h / 2) };
  }
  return [placed, top];
}

/** SVG path through `points` with rounded corners. */
function rounded(points, r = 10) {
  const f = (x, y) => `${fmt(x)},${fmt(y)}`;
  if (points.length < 3) return "M" + points.map(([x, y]) => f(x, y)).join(" L");
  const d = [`M${f(...points[0])}`];
  for (let i = 0; i + 2 < points.length; i++) {
    const [x0, y0] = points[i], [x1, y1] = points[i + 1], [x2, y2] = points[i + 2];
    const l1 = Math.max(1e-6, Math.abs(x1 - x0) + Math.abs(y1 - y0));
    const l2 = Math.max(1e-6, Math.abs(x2 - x1) + Math.abs(y2 - y1));
    const rr = Math.min(r, l1 / 2, l2 / 2);
    const ax = x1 - (x1 - x0) / l1 * rr, ay = y1 - (y1 - y0) / l1 * rr;
    const bx = x1 + (x2 - x1) / l2 * rr, by = y1 + (y2 - y1) / l2 * rr;
    d.push(`L${f(ax, ay)} Q${f(x1, y1)} ${f(bx, by)}`);
  }
  d.push(`L${f(...points[points.length - 1])}`);
  return d.join(" ");
}

/** Polyline from card a to card b, and where its label goes. */
function route(a, b, how) {
  if (how === "elbow") {
    const x1 = a.cx, y1 = a.cy + a.hh;
    const x2 = b.cx + (a.cx > b.cx ? b.hw : -b.hw);
    return [[[x1, y1], [x1, b.cy], [x2, b.cy]], [x1, (y1 + b.cy) / 2]];
  }
  if (Array.isArray(how)) {
    const bx = xcol(how[1]);
    const s = bx > a.cx ? 1 : -1;
    const pts = [[a.cx + s * a.hw, a.cy], [bx, a.cy], [bx, b.cy], [b.cx + s * b.hw, b.cy]];
    return [pts, [bx, (a.cy + b.cy) / 2]];
  }
  if (Math.abs(a.cx - b.cx) < 1) {            // same column
    const s = b.cy > a.cy ? 1 : -1;
    const y1 = a.cy + s * a.hh, y2 = b.cy - s * b.hh;
    return [[[a.cx, y1], [b.cx, y2]], [a.cx, (y1 + y2) / 2]];
  }
  if (Math.abs(a.cy - b.cy) < 1) {            // same row
    const s = b.cx > a.cx ? 1 : -1;
    const x1 = a.cx + s * a.hw, x2 = b.cx - s * b.hw;
    return [[[x1, a.cy], [x2, b.cy]], [(x1 + x2) / 2, a.cy]];
  }
  const s = b.cx > a.cx ? 1 : -1;             // out of the side, across, into top/bottom
  const v = b.cy > a.cy ? 1 : -1;
  const x1 = a.cx + s * a.hw;
  const y2 = b.cy - v * b.hh;
  return [[[x1, a.cy], [b.cx, a.cy], [b.cx, y2]], [(x1 + b.cx) / 2, a.cy]];
}

function tone(label) {
  if (!label) return "plain";
  const head = label.split(/\s+/).filter(Boolean)[0].toLowerCase();
  return head === "yes" ? "yes" : head === "no" ? "no" : "plain";
}

// Each kind's icon is a small copy of its classic flowchart shape (the same
// shapes the Mermaid version uses): oval = start or finish, rectangle = step,
// diamond = question, slanted box = another way this pass ends.
const OVAL = '<rect x="-7.5" y="-4.5" width="15" height="9" rx="4.5" class="ic-bg"/>';
const ICONS = {
  entry: OVAL,
  step: '<rect x="-7" y="-4.8" width="14" height="9.6" rx="1" class="ic-bg"/>',
  decision: '<path d="M0,-7.2 L7.2,0 L0,7.2 L-7.2,0 Z" class="ic-bg"/>',
  end: OVAL,
  out: '<path d="M-4.3,-4.8 H8 L4.3,4.8 H-8 Z" class="ic-bg"/>',
};

function svgChart(chart) {
  const [P, top] = place(chart);
  const maxCol = Math.max(...chart.nodes.map((n) => n.col));
  const maxRow = Math.max(...chart.nodes.map((n) => n.row));
  const rightBus = Math.max(...chart.edges.filter(isBus).map((e) => e.route[1]), maxCol);
  const width = Math.trunc(xcol(Math.max(maxCol, rightBus) + 0.5) + 40);
  const height = Math.trunc(top + (maxRow + 1) * ROW + 24);
  const out = [`<svg viewBox="0 0 ${width} ${height}" data-width="${width}" data-height="${height}" ` +
    `role="img" aria-label="${esc(chart.title)} flowchart">`];
  out.push("<defs>" + ["plain", "yes", "no", "hot"].map((t) =>
    `<marker id="ah-${chart.id}-${t}" viewBox="0 0 10 10" refX="8.5" refY="5" markerWidth="6.5" ` +
    `markerHeight="6.5" orient="auto-start-reverse"><path d="M1,1 L9,5 L1,9 Z" class="ah ah-${t}"/></marker>`).join("") + "</defs>");

  const [axis, bands] = chart.lanes;
  out.push('<g class="lanes">');
  bands.forEach(([label, a, b], k) => {
    const alt = k % 2 ? " alt" : "";
    if (axis === "rows") {
      const y0 = top + a * ROW + 4, y1 = top + (b + 1) * ROW - 4;
      out.push(`<rect class="lane${alt}" x="12" y="${y0}" width="${width - 24}" height="${y1 - y0}" rx="14"/>`);
      out.push(`<text class="lane-label" x="28" y="${y0 + 20}">${esc(label.toUpperCase())}</text>`);
    } else {
      const x0 = xcol(a) - COL / 2 + 8, x1 = xcol(b) + COL / 2 - 8;
      out.push(`<rect class="lane${alt}" x="${fmt(x0, 0)}" y="8" width="${fmt(x1 - x0, 0)}" height="${height - 16}" rx="14"/>`);
      out.push(`<text class="lane-label" x="${fmt(x0 + 16, 0)}" y="28">${esc(label.toUpperCase())}</text>`);
    }
  });
  out.push("</g>");

  out.push('<g class="edges">');
  const chips = [];
  for (const e of chart.edges) {
    const a = P[e.from], b = P[e.to];
    const [pts, [lx, ly]] = route(a, b, e.route);
    const t = tone(e.label);
    out.push(`<path class="edge e-${t}" data-from="${e.from}" data-to="${e.to}" d="${rounded(pts)}" ` +
      `marker-end="url(#ah-${chart.id}-${t})" data-tone="${t}"/>`);
    if (e.label) {
      const w = 7 * len(e.label) + 16;
      chips.push(`<g class="chip c-${t}" data-from="${e.from}" data-to="${e.to}">` +
        `<rect x="${fmt(lx - w / 2)}" y="${fmt(ly - 10)}" width="${w}" height="20" rx="10"/>` +
        `<text x="${fmt(lx)}" y="${fmt(ly + 0.5)}" dominant-baseline="middle">${esc(e.label)}</text></g>`);
    }
  }
  out.push("</g>");

  out.push('<g class="nodes">');
  chart.nodes.forEach((node0, i) => {
    const step = i + 1;
    const p = P[node0.id];
    const { cx: x, cy: y, hw, hh } = p;
    const label = p.lines.join(" ");
    out.push(`<g class="node k-${p.kind}${p.func ? " has-code" : ""}" data-id="${p.id}" ` +
      `tabindex="0" role="button" aria-label="Step ${step}: ${esc(label)}">`);
    out.push(`<rect class="card" x="${fmt(x - hw)}" y="${fmt(y - hh)}" width="${fmt(2 * hw)}" height="${fmt(2 * hh)}" rx="12"/>`);
    out.push(`<rect class="bar" x="${fmt(x - hw)}" y="${fmt(y - hh + 10)}" width="4" height="${fmt(2 * hh - 20)}" rx="2"/>`);
    out.push(`<g class="icon" transform="translate(${fmt(x - hw + 22)},${fmt(y)})">${ICONS[p.kind]}</g>`);
    const tx = x - hw + 40;
    const lines = p.lines.length + (p.sub ? 1 : 0);
    const ty = y - (lines - 1) * 8.5;
    p.lines.forEach((line, k) => {
      out.push(`<text class="title" x="${fmt(tx)}" y="${fmt(ty + 17 * k)}" dominant-baseline="middle">${esc(line)}</text>`);
    });
    if (p.sub) {
      out.push(`<text class="sub" x="${fmt(tx)}" y="${fmt(ty + 17 * p.lines.length)}" dominant-baseline="middle">${esc(p.sub)}</text>`);
    }
    if (p.func) {
      out.push(`<g class="code-tag" transform="translate(${fmt(x + hw - 18)},${fmt(y - hh + 13)})"><rect x="-12" y="-8" width="24" height="16" rx="5"/>` +
        `<text x="0" y="0.5" dominant-baseline="middle">&lt;/&gt;</text></g>`);
    }
    out.push("</g>");
  });
  out.push("</g>");
  out.push('<g class="chips">' + chips.join("") + "</g>");
  out.push("</svg>");
  return out.join("\n");
}

// ---------------------------------------------------------------- outputs ---

function functionsOf(chart) {
  const seen = new Set(), out = [];
  for (const n of chart.nodes) {
    if (n.func && !seen.has(n.func.join("\0"))) {
      seen.add(n.func.join("\0"));
      out.push(n.func);
    }
  }
  return out;
}

const mermaidText = (text) => text.replaceAll('"', "#quot;").replaceAll("<", "&lt;").replaceAll(">", "&gt;");

function mermaid(chart) {
  const [P] = place(chart);
  const shapes = { entry: ["([", "])"], end: ["([", "])"], out: ["[/", "/]"], step: ["[", "]"], decision: ["{", "}"] };
  const out = ["```mermaid", "flowchart TD",
    "    classDef k_entry fill:#e8eefd,stroke:#2f5bea,color:#111,stroke-width:1.5px",
    "    classDef k_step fill:#ffffff,stroke:#9aa1ad,color:#111",
    "    classDef k_decision fill:#fff4dc,stroke:#d18b00,color:#111",
    "    classDef k_end fill:#e3f6e8,stroke:#1f9d55,color:#111,stroke-width:1.5px",
    "    classDef k_out fill:#f1f2f5,stroke:#9aa1ad,color:#333"];
  const [axis, bands] = chart.lanes;
  bands.forEach(([label, a, b], k) => {
    const members = chart.nodes.filter((n) => {
      const v = axis === "rows" ? n.row : n.col;
      return a <= v && v <= b;
    });
    if (!members.length) return;
    out.push(`    subgraph lane${k}["${mermaidText(label)}"]`);
    for (const n of members) {
      const [s, e] = shapes[n.kind];
      const sub = P[n.id].sub;
      const text = mermaidText(n.title) + (sub ? `<br/><small>${mermaidText(sub)}</small>` : "");
      out.push(`        ${n.id}${s}"${text}"${e}`);
    }
    out.push("    end");
  });
  for (const n of chart.nodes) out.push(`    class ${n.id} k_${n.kind}`);
  for (const e of chart.edges) {
    const arrow = e.label ? ` -- "${mermaidText(e.label)}" --> ` : " --> ";
    out.push(`    ${e.from}${arrow}${e.to}`);
  }
  out.push("```");
  return out.join("\n");
}

function markdown() {
  const out = [
    "# Flowcharts",
    "",
    "How the LCM simulator runs, as flowcharts, each followed by the source of every function",
    "it names. The interactive version is [`web/docs.html`](../web/docs.html): every step",
    "explains itself, steps marked `</>` open their code, and a guided tour walks through each chart.",
    "",
    "> Generated by [`scripts/gen-flowcharts.mjs`](../scripts/gen-flowcharts.mjs). The code blocks",
    "> are copied from the source files, so edits made here are overwritten. Re-run the script",
    "> after changing any function a chart names.",
    "",
    "## Contents",
    "",
  ];
  CHARTS.forEach((c, i) => {
    const anchor = `${i + 1}. ${c.title}`.toLowerCase().replace(/[^a-z0-9 -]/g, "").replaceAll(" ", "-");
    out.push(`${i + 1}. [${c.title}](#${anchor}) — ${c.short}`);
  });
  CHARTS.forEach((c, i) => {
    out.push("", "---", "", `## ${i + 1}. ${c.title}`, "", c.blurb, "", mermaid(c), "");
    out.push(...c.notes.map((note) => `- ${note}`));
    const described = c.nodes.filter((n) => n.desc);
    if (described.length) {
      out.push("", "| Step | What happens |", "| --- | --- |");
      out.push(...described.map((n) => `| ${n.title} | ${n.desc} |`));
    }
    for (const [file, name] of functionsOf(c)) {
      const [line, src] = extract(file, name);
      const lang = file.endsWith(".rs") ? "rust" : "js";
      out.push("", `\`${name}\` — [\`${file}:${line}\`](../${file}#L${line})`, "", "```" + lang, src, "```");
    }
  });
  return out.join("\n") + "\n";
}

const PAGE = `<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>How it works · LCM Simulator</title>
  <meta name="description" content="Interactive flowcharts of the LCM simulator's Rust core, with the source of every step.">
  <!-- Generated by scripts/gen-flowcharts.mjs; edits here are overwritten. -->
  <script>
    (() => {
      let t = "system";
      try { t = localStorage.getItem("lcm-theme") || "system"; } catch {}
      if (t !== "system") document.documentElement.dataset.theme = t;
    })();
  </script>
  <link rel="stylesheet" href="styles.css">
  <link rel="stylesheet" href="docs.css">
</head>
<body class="docs">
  <header class="docs-bar">
    <a class="btn" href="../" title="Back to the simulator"><span data-icon="logo" data-size="20"></span><span class="brand-name">LCM</span></a>
    <span class="crumb">How it works</span>
    <span class="spacer"></span>
    <button id="theme" class="btn" title="Theme" aria-label="Theme"><span data-icon="moon"></span></button>
    <a class="btn outline open-sim" href="../"><span data-icon="play" data-size="14"></span>Open the simulator</a>
  </header>
  <div class="docs-main">
    <nav class="docs-nav" aria-label="Charts">
      <a class="sim-card" href="../">
        <span class="sim-card-icon" data-icon="play" data-size="16"></span>
        <span><span class="sim-card-title">Open the simulator</span><span class="sim-card-sub">Run the swarm these charts describe</span></span>
      </a>
      <div class="nav-title">Flowcharts</div>
      __NAV__
      <div class="nav-title">Legend</div>
      <ul class="legend-list">
        <li><svg width="18" height="18" viewBox="-9 -9 18 18" class="k-entry">__IC_ENTRY__</svg>Oval: where the chart starts</li>
        <li><svg width="18" height="18" viewBox="-9 -9 18 18" class="k-step">__IC_STEP__</svg>Rectangle: a step</li>
        <li><svg width="18" height="18" viewBox="-9 -9 18 18" class="k-decision">__IC_DECISION__</svg>Diamond: a question with branches</li>
        <li><svg width="18" height="18" viewBox="-9 -9 18 18" class="k-end">__IC_END__</svg>Oval: the run or cycle finishes</li>
        <li><svg width="18" height="18" viewBox="-9 -9 18 18" class="k-out">__IC_OUT__</svg>Slanted box: another way this pass ends</li>
        <li><span class="tag">&lt;/&gt;</span>Opens the function's code</li>
      </ul>
      <p class="nav-foot">Generated from the source: the code shown is the code that runs.</p>
    </nav>
    <main class="docs-content">
__CHARTS__
    </main>
    <aside id="detail" class="docs-detail" aria-live="polite">
      <div class="detail-empty">
        <div class="detail-empty-title">Click any step</div>
        <p>to see what it does and, for steps marked <span class="tag">&lt;/&gt;</span>, the code behind it. Or take the guided tour.</p>
      </div>
      <div class="detail-body" hidden>
        <div class="detail-head">
          <span id="d_kind" class="kind-chip"></span>
          <span class="spacer"></span>
          <div class="tour-ctl">
            <button id="d_prev" class="btn small" title="Previous step (←)" aria-label="Previous step"><span data-icon="chevron" data-size="14" class="flip"></span></button>
            <span id="d_pos" class="num"></span>
            <button id="d_next" class="btn small" title="Next step (→)" aria-label="Next step"><span data-icon="chevron" data-size="14"></span></button>
          </div>
          <button id="d_close" class="btn small" title="Close (Esc)" aria-label="Close"><span data-icon="close" data-size="14"></span></button>
        </div>
        <h2 id="d_title"></h2>
        <div id="d_sub" class="detail-sub"></div>
        <p id="d_desc"></p>
        <div id="d_links" class="detail-links"></div>
        <section id="d_code" hidden>
          <div class="code-head"><span id="d_file"></span><span class="spacer"></span>
            <button id="d_copy" class="btn small outline">Copy</button>
            <a id="d_github" class="btn small outline" target="_blank" rel="noopener">GitHub</a></div>
          <pre class="code"><code id="d_src"></code></pre>
        </section>
      </div>
    </aside>
  </div>
  <script id="doc-data" type="application/json">__DATA__</script>
  <script type="module" src="docs.js"></script>
</body>
</html>
`;

function page() {
  const nav = [], sections = [], data = { repo: REPO_URL, charts: {} };
  CHARTS.forEach((c, i) => {
    const k = i + 1;
    nav.push(`<a class="nav-item" href="#${c.id}" data-chart="${c.id}"><span class="nav-num">${k}</span>` +
      `<span><span class="nav-name">${esc(c.title)}</span><span class="nav-short">${esc(c.short)}</span></span></a>`);
    const notes = c.notes.map((n) => `<li>${esc(n)}</li>`).join("");
    sections.push(
      `<section class="chart" data-chart="${c.id}" hidden>` +
      `<div class="chart-head"><div><div class="eyebrow">Chart ${k} of ${CHARTS.length}</div><h1>${esc(c.title)}</h1>` +
      `<p class="blurb">${esc(c.blurb)}</p></div></div>` +
      `<div class="stage-wrap"><div class="stage" tabindex="0" aria-label="Chart canvas: drag or scroll to move, Ctrl + scroll to zoom">${svgChart(c)}</div>` +
      `<div class="stage-tools">` +
      `<button class="btn primary small tour-start"><span data-icon="play" data-size="13"></span>Guided tour</button>` +
      `<span class="stage-hint">Click a step for details · drag or scroll to move · Ctrl + scroll to zoom</span>` +
      `<button class="btn small" data-zoom="out" title="Zoom out (−)" aria-label="Zoom out"><span data-icon="minus" data-size="15"></span></button>` +
      `<span class="zoom-level num">100%</span>` +
      `<button class="btn small" data-zoom="in" title="Zoom in (+)" aria-label="Zoom in"><span data-icon="plus" data-size="15"></span></button>` +
      `<button class="btn small" data-zoom="fit" title="Fit (F)" aria-label="Fit"><span data-icon="fit" data-size="15"></span></button>` +
      `</div></div>` +
      `<details class="notes" open><summary>Notes</summary><ul>${notes}</ul></details></section>`
    );
    const [P] = place(c);
    const chart = { title: c.title, order: c.nodes.map((n) => n.id), nodes: {} };
    for (const n of c.nodes) {
      const labels = {};
      for (const e of c.edges) if (e.from === n.id && e.label) labels[e.to] = e.label;
      const entry = {
        title: n.title, sub: P[n.id].sub, kind: n.kind, desc: n.desc,
        out: c.edges.filter((e) => e.from === n.id).map((e) => e.to),
        in: c.edges.filter((e) => e.to === n.id).map((e) => e.from),
        labels,
      };
      if (n.func) {
        const [file, name] = n.func;
        const [line, src] = extract(file, name);
        entry.code = { name, path: file, line, source: src, html: highlight(src, file.endsWith(".rs") ? "rust" : "js") };
      }
      chart.nodes[n.id] = entry;
    }
    data.charts[c.id] = chart;
  });
  let text = PAGE;
  for (const k of Object.keys(ICONS)) text = text.replaceAll(`__IC_${k.toUpperCase()}__`, () => ICONS[k]);
  const json = JSON.stringify(data).replaceAll("</", "<\\/");
  return text.replace("__NAV__", () => nav.join("\n      ")).replace("__CHARTS__", () => sections.join("\n"))
    .replace("__DATA__", () => json);
}

function main() {
  const outputs = { "docs/flowcharts.md": markdown(), "web/docs.html": page() };
  const check = process.argv.includes("--check");
  const stale = [];
  for (const [file, text] of Object.entries(outputs)) {
    const full = path.join(ROOT, file);
    const current = fs.existsSync(full) ? fs.readFileSync(full, "utf-8") : null;
    if (check) {
      if (current !== text) stale.push(file);
    } else {
      fs.writeFileSync(full, text);
      console.log(`wrote ${file}`);
    }
  }
  if (stale.length) {
    console.log("out of date: " + stale.join(", ") + " (run node scripts/gen-flowcharts.mjs)");
    process.exit(1);
  }
  if (check) console.log("flowcharts are up to date");
}

main();

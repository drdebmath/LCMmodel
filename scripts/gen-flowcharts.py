#!/usr/bin/env python3
"""Generate the flowcharts from one set of chart definitions:

  docs/flowcharts.md   Mermaid charts (GitHub renders them), each followed by
                       the source of every function the chart names
  web/docs.html        the same charts as themed SVG in an interactive page
                       (web/docs.css, web/docs.js): every step explains itself,
                       steps that stand for a function show its source

Function source is copied out of the code, so a function shown is the
function that runs. A chart naming a function that no longer exists is an
error. Re-run after changing any function a chart names:

    python3 scripts/gen-flowcharts.py           # write both files
    python3 scripts/gen-flowcharts.py --check   # fail if either is out of date
"""
import html
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_URL = "https://github.com/imstillsamarth/LCMmodel/blob/rust-core"

SIM = "crates/lcm-core/src/sim.rs"
GATHERING = "crates/lcm-algorithms/src/gathering.rs"
WASM = "crates/lcm-wasm/src/lib.rs"
WORKER = "web/worker.js"
CLIENT = "web/client.js"
RENDERER = "web/renderer.js"

# ---------------------------------------------------------------- charts ----
# Nodes sit on a hand-placed grid (col, row) so a drawing stays stable when a
# node is added. Kinds: entry (where the chart starts), step, decision, end
# (a finish), out (another way this pass ends). `func` = (file, function) when
# the step stands for a function. Nodes are listed in reading order, which is
# also the order of the guided tour.
#
# Edge routes: "auto" (straight when aligned; otherwise out of the source's
# side, across, and into the target's top or bottom), "elbow" (down out of
# the source, then across into the target's side), ("around", col) (out of
# the source's side, along a vertical line at `col`, into the target's side).
#
# Lanes are background bands grouping rows ("rows") or columns ("cols").


def node(id, title, col, row, kind="step", func=None, sub=None, desc=""):
    return {"id": id, "title": title, "col": col, "row": row, "kind": kind,
            "func": func, "sub": sub, "desc": desc}


def edge(a, b, label=None, route="auto"):
    return {"from": a, "to": b, "label": label, "route": route}


CHARTS = [
    {
        "id": "event-loop",
        "title": "The event loop",
        "short": "How the scheduler takes and handles events",
        "blurb": (
            "The scheduler is a queue of timed events: each robot's next LOOK or WAIT, a crash, "
            "and a periodic Visualize tick. advance() handles them one at a time, in time order, "
            "until its budget, a limit or the end. This is Scheduler.handle_event in "
            "scheduler.py, ported line by line."
        ),
        "notes": [
            "Ties in time break exactly as in Python's heap of (time, id, state) tuples: lower "
            "robot id first, then Crash < Look < Visualize < Wait.",
            "Visualize ticks change nothing; they are kept so event counts and limits match the Python.",
            "The run ends when every robot that is neither crashed nor Byzantine has terminated.",
        ],
        "lanes": ("rows", [("Run loop", 0, 1), ("Take the next event", 2, 6),
                           ("Handle it", 7, 8), ("After each robot event", 9, 10)]),
        "nodes": [
            node("adv", "Run the simulation", 1, 0, "entry", (SIM, "advance"), "advance(budget)",
                 "The worker calls this with a budget: how many events it may handle and the latest "
                 "simulated time it may reach. It handles events one after another until a stop "
                 "condition holds."),
            node("lim", "Time to stop?", 1, 1, "decision", None, "ended · limit · budget · time",
                 "Checked before every event, in this order: has the run ended; has max_time passed "
                 "or max_events been handled (the same checks as run.py); is this call's event "
                 "budget used up; is the next event later than the time this call may reach (this "
                 "is how playback speed is paced)?"),
            node("ret", "Return why it stopped", 2, 0, "out", None, "StopReason",
                 "Ended, MaxTime, MaxEvents, Budget or UntilTime. The worker uses this to pace "
                 "playback and to notice the end of the run."),
            node("pop", "Take the earliest event", 1, 2, "step", (SIM, "step"), "step()",
                 "Events wait in a priority queue ordered by (time, robot id, kind), exactly like "
                 "Python's heap of tuples, so ties break the same way."),
            node("emp", "Queue empty?", 1, 3, "decision", None, None,
                 "Does not happen in practice, because the Visualize tick always reschedules "
                 "itself; handled like the Python: an empty queue ends the run."),
            node("end1", "Ended", 2, 3, "end", None, "queue empty",
                 "The run is over; advance() reports Ended."),
            node("old", "Earlier than the clock?", 1, 4, "decision", None, None,
                 "Time never goes backwards. Python skips such an event without touching the "
                 "clock, and so does the port."),
            node("ign", "Skip the event", 0, 4, "out", None, "clock unchanged",
                 "The event is dropped and the clock stays where it was; the loop takes the next event."),
            node("vis", "Visualize tick?", 1, 5, "decision", None, None,
                 "A periodic event every sampling_rate time units. It changes no robot; it is "
                 "kept so event counts match the Python."),
            node("vis2", "Schedule the next tick", 0, 5, "step", None, "t + sampling_rate",
                 "The tick puts itself back in the queue one sampling interval later; no robot changes."),
            node("dead", "Robot out of play?", 1, 6, "decision", None, "terminated · or crashed, not a crash event",
                 "Yes if the robot has terminated, or if it has crashed and this is not its crash "
                 "event. A crashed robot's own crash event is not skipped: it goes on to be "
                 "recorded."),
            node("ign2", "Skip the event", 0, 6, "out", (SIM, "schedule_activation"), "crashed ones rescheduled",
                 "A crashed robot is rescheduled after a random delay and otherwise ignored; a "
                 "terminated robot is simply ignored and never scheduled again. Both exactly as in "
                 "the Python."),
            node("kind", "Which kind of event?", 1, 7, "decision", None, None,
                 "A robot event is a LOOK (activate), a WAIT (a move ends) or a CRASH (a crash fault takes effect)."),
            node("look", "Look, compute, move", 0, 8, "step", (SIM, "look"), "look() · chart 2",
                 "The robot's activation: snapshot, compute a target, start moving. Chart 2 shows "
                 "every branch."),
            node("wait", "Finish the move", 1, 8, "step", (SIM, "wait"), "wait() · then next LOOK",
                 "The WAIT event fires at the arrival time: the robot is placed on its target, "
                 "its light turns green and its next activation is scheduled after a random "
                 "exponential delay."),
            node("crash", "Record the crash", 2, 8, "step", None, "crash fault",
                 "Crash-faulty robots are crashed from the start (set when the run is built), so "
                 "they never move; their single crash event only records it, as in the Python."),
            node("glob", "Every live robot terminated?", 1, 9, "decision", (SIM, "globally_terminated"),
                 "globally_terminated()",
                 "Crashed and Byzantine robots don't count: the run ends once all the others "
                 "have terminated."),
            node("end2", "Ended", 1, 10, "end", None, "all terminated",
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
            edge("look", "glob", None, "elbow"),
            edge("wait", "glob"),
            edge("crash", "glob", None, "elbow"),
            edge("glob", "end2", "yes"),
            edge("glob", "lim", "no · next event", ("around", 2.62)),
            edge("ign", "lim", None, ("around", -0.66)),
            edge("vis2", "lim", None, ("around", -0.66)),
            edge("ign2", "lim", None, ("around", -0.66)),
        ],
    },
    {
        "id": "look-compute-move",
        "title": "One Look–Compute–Move cycle",
        "short": "Everything a robot does when it activates",
        "blurb": (
            "The robot takes a snapshot (LOOK), its algorithm computes a target (COMPUTE), and it "
            "moves there (MOVE); a WAIT event at the arrival time ends the move. Faults, freezing "
            "and termination all branch off this path. This is Robot.look, Robot.move and "
            "Robot.wait in robot.py."
        ),
        "notes": [
            "The snapshot is taken at the exact LOOK time: moving robots are seen at their "
            "interpolated position, never a stale one.",
            "A robot freezes (stays put this cycle) when its target is within ε = 10^-precision of "
            "where it is, or when an omission fault skips the move (probability ½).",
            "Rigid and non-rigid movement both reach the target: the WAIT event is scheduled at the "
            "full-distance arrival time (finding #3 in report.md).",
        ],
        "lanes": ("rows", [("LOOK", 0, 4), ("COMPUTE", 5, 7), ("MOVE", 8, 11)]),
        "nodes": [
            node("start", "Robot activates", 1, 0, "entry", (SIM, "look"), "LOOK event · look()",
                 "The robot's LOOK event comes up in the queue. Everything on this chart happens "
                 "at that single instant, except the final WAIT."),
            node("view", "Take a snapshot", 1, 1, "step", (SIM, "build_view"), "build_view()",
                 "Every robot's position at this exact time (moving robots are interpolated along "
                 "their path), keeping those within the visibility radius. The robot sees itself too."),
            node("lit", "Light turns blue", 1, 2, "step", None, "state = Look",
                 "Lights change at most once every 0.5 time units, as in the Python, so a light "
                 "can lag behind the robot's phase."),
            node("byz", "Byzantine robot?", 1, 3, "decision", None, None,
                 "A Byzantine robot ignores its algorithm entirely."),
            node("byzt", "Pick an erratic target", 2, 3, "step", None, "random direction",
                 "It moves in a random direction by half the distance to the farthest robot it "
                 "sees (10 if it sees none), and never terminates."),
            node("alone", "Only itself still active?", 1, 4, "decision", None, None,
                 "If every other visible robot has crashed or terminated, or none is visible, the "
                 "robot terminates. With limited visibility an isolated robot stops: the "
                 "original's behaviour."),
            node("term1", "Terminate", 0, 4, "end", None, "nobody left to see",
                 "The robot is marked terminated (and frozen) and is never activated again."),
            node("comp", "Algorithm computes a target", 1, 5, "step", (GATHERING, "compute"),
                 "compute() · chart 3",
                 "The algorithm sees only the snapshot and returns a target, whether to "
                 "terminate, and optionally a circle to display. Robots are oblivious: nothing is "
                 "remembered between activations."),
            node("tdec", "Algorithm says terminate?", 1, 6, "decision", None, None,
                 "Each algorithm has its own test; for Gathering, every live visible robot is within ε of the average."),
            node("term2", "Terminate", 0, 6, "end", None, "for good",
                 "The robot stops permanently and is drawn grey."),
            node("frz", "Skip, or already there?", 1, 7, "decision", None, "omission ½ · target within ε",
                 "An omission-faulty robot skips the move half the time. Any robot whose target is "
                 "closer than ε = 10^-precision stays put."),
            node("frozen", "Stay put, look again later", 0, 7, "out", (SIM, "schedule_activation"), "frozen",
                 "The robot is marked frozen and its next activation is scheduled after a random "
                 "exponential delay."),
            node("mv", "Start moving", 1, 8, "step", (SIM, "begin_move"), "begin_move() · light red",
                 "The move starts now at the robot's speed (40 % for a delay fault); its position "
                 "is interpolated until it arrives."),
            node("sub", "Arrival later than now?", 1, 9, "decision", None, None,
                 "A tiny move can be shorter than the clock can resolve. As in the Python fix for "
                 "the endless-simulation bug, it is finished immediately instead of scheduling a "
                 "WAIT at the same instant."),
            node("subt", "Finish the tiny move now", 0, 9, "out", None, "look again later",
                 "The robot is placed on its target at once and its next activation is scheduled."),
            node("sched", "Schedule WAIT at arrival", 1, 10, "step", None, "t + distance / speed",
                 "A WAIT event goes in the queue for the moment the robot reaches its target; other robots see it moving until then."),
            node("arr", "Arrive", 1, 11, "end", (SIM, "wait"), "wait() · light green · next LOOK",
                 "At the arrival time the robot is placed exactly on its target, its light turns "
                 "green, and its next LOOK is scheduled."),
        ],
        "edges": [
            edge("start", "view"),
            edge("view", "lit"),
            edge("lit", "byz"),
            edge("byz", "byzt", "yes"),
            edge("byz", "alone", "no"),
            edge("byzt", "mv", None, "elbow"),
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
        "short": "The one algorithm ported so far",
        "blurb": (
            "Each robot moves to the average position of the robots it sees. Repeated "
            "asynchronously, the swarm converges to a single point. This is Robot._midpoint and "
            "Robot._midpoint_terminal in robot.py."
        ),
        "notes": [
            "The average includes crashed robots, but the termination test ignores them: the "
            "original's behaviour, kept on purpose (finding #4 in report.md).",
            "The sum is a plain left-to-right loop, as in the Python, so results match to the last digits.",
        ],
        "lanes": ("rows", [("Target", 0, 1), ("Termination", 2, 3)]),
        "nodes": [
            node("a", "Compute", 1, 0, "entry", (GATHERING, "compute"), "Gathering::compute()",
                 "Called from the COMPUTE phase of chart 2 with the robot's snapshot."),
            node("b", "Average of visible robots", 1, 1, "step", None, "crashed ones included",
                 "Sum of every visible robot's position, divided by how many there are."),
            node("c", "All live robots within ε?", 1, 2, "decision", None, "ε = 10^-precision",
                 "Every visible robot that has not crashed must be within ε of the average."),
            node("d", "Terminate", 0, 3, "end", None, "gathered",
                 "The robot has gathered with everyone it can see and stops for good."),
            node("e", "Move to the average", 2, 3, "out", None, "target = average",
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
            "The page never runs the simulation itself. A Web Worker owns the Rust core and "
            "simulates in short slices at the chosen playback speed; the page asks for the latest "
            "frame whenever it is ready to draw, so drawing never slows the simulation."
        ),
        "notes": [
            "Frames travel as typed arrays that are transferred, not copied or turned into JSON; the "
            "page hands each frame's arrays back to be refilled, so playback allocates nothing per frame.",
            "The worker computes in ~6 ms slices and yields between them, so messages (frame requests, "
            "pause, speed changes) get through immediately.",
        ],
        "lanes": ("cols", [("Page · main thread", 0, 0), ("Worker thread", 1, 1), ("Rust core · WebAssembly", 2, 2)]),
        "nodes": [
            node("raf", "Every screen frame", 0, 0, "entry", (CLIENT, "startPulling"), "startPulling()",
                 "While playing, the page runs one loop per screen refresh."),
            node("ask", "Ask for the latest frame", 0, 1, "step", (CLIENT, "requestFrame"), "requestFrame()",
                 "At most one request is in flight. The previous frame's arrays go back to the worker "
                 "to be refilled."),
            node("slice", "Simulate for ~6 ms", 1, 0, "entry", (WORKER, "slice"), "slice()",
                 "Runs continuously while playing, pacing itself to the playback speed, and yields "
                 "between slices so messages get through."),
            node("adv", "Handle events", 2, 0, "step", (WASM, "advance"), "advance() · chart 1",
                 "The core handles a batch of events; the batch size adapts so one call takes about "
                 "1 ms."),
            node("post", "Send the frame", 1, 2, "step", (WORKER, "postFrame"), "postFrame()",
                 "Answers a frame request with the current state and transfers the arrays to the page."),
            node("fill", "Fill the arrays", 2, 2, "step", (WASM, "fill_frame"), "fill_frame()",
                 "Positions, state flags, lights, faults, targets and circles, written straight into "
                 "the recycled arrays."),
            node("draw", "Draw the swarm", 0, 3, "step", (RENDERER, "draw"), "draw() · CPU Canvas2D",
                 "Tiny robots are drawn as rectangles, larger ones as pre-drawn sprites per colour; at "
                 "most one draw per screen frame. 10,000 robots take about 4–6 ms without a GPU."),
            node("stats", "Update stats and legend", 0, 4, "end", None, "a few times a second",
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
]

# -------------------------------------------------------- source extraction --


def extract(path, name):
    """Return (first line number, source) of function `name` in `path`, with the
    doc comments above it. Relies on the formatter putting a function's closing
    brace at the indentation of its first line."""
    lines = open(os.path.join(ROOT, path), encoding="utf-8").read().split("\n")
    if path.endswith(".rs"):
        start = re.compile(rf"^\s*(pub(\([a-z]+\))? )?(const )?fn {re.escape(name)}\b")
    else:
        start = re.compile(rf"^\s*(export )?(async )?(function )?{re.escape(name)}\s*\(.*\)\s*\{{\s*$")
    for i, line in enumerate(lines):
        if not start.match(line):
            continue
        indent = line[: len(line) - len(line.lstrip())]
        first = i
        while first > 0 and re.match(r"^\s*(///|#\[)", lines[first - 1]):
            first -= 1
        for j in range(i, len(lines)):
            if lines[j] == indent + "}":
                body = [l[len(indent):] if l.startswith(indent) else l for l in lines[first : j + 1]]
                return first + 1, "\n".join(body)
        break
    raise SystemExit(f"gen-flowcharts: function `{name}` not found in {path}")


# ------------------------------------------------------ syntax highlighting --

KEYWORDS = {
    "rust": set("as break const continue crate dyn else enum fn for if impl in let loop match mod move mut pub "
                "ref return self Self static struct super trait type unsafe use where while true false".split()),
    "js": set("async await break case catch class const continue default delete do else export extends false "
              "finally for from function if import in instanceof let new null of return static super switch "
              "this throw true try typeof undefined var void while".split()),
}
TOKEN = re.compile(r"""
    (?P<comment>//[^\n]*|/\*[\s\S]*?\*/)
  | (?P<string>"(?:\\.|[^"\\])*"|`(?:\\.|[^`\\])*`|'(?:\\.|[^'\\\n])'|(?<=[\s(,=:])'(?:\\.|[^'\\\n])*')
  | (?P<attr>\#\[[^\]\n]*\])
  | (?P<number>\b\d[\d_]*(?:\.\d[\d_]*)?(?:[eE][+-]?\d+)?(?:f64|f32|u8|u16|u32|u64|usize|i32|i64)?\b)
  | (?P<macro>\b[a-z_][a-z0-9_]*!)
  | (?P<ident>\b[A-Za-z_][A-Za-z0-9_]*\b)
""", re.VERBOSE)


def highlight(src, lang):
    """Source as HTML lines with token classes (kw, ty, fn, str, num, com, attr)."""
    tokens, pos = [], 0
    for m in TOKEN.finditer(src):
        if m.start() > pos:
            tokens.append((None, src[pos:m.start()]))
        kind, text = m.lastgroup, m.group()
        cls = {"comment": "com", "string": "str", "attr": "attr", "number": "num", "macro": "fn"}.get(kind)
        if kind == "ident":
            if text in KEYWORDS[lang] or (lang == "rust" and text in ("Some", "None", "Ok", "Err")):
                cls = "kw"
            elif text[0].isupper():
                cls = "ty"
            elif src[m.end():m.end() + 1] == "(":
                cls = "fn"
        tokens.append((cls, text))
        pos = m.end()
    tokens.append((None, src[pos:]))
    lines, current = [], []
    for cls, text in tokens:
        parts = text.split("\n")
        for k, part in enumerate(parts):
            if k:
                lines.append("".join(current))
                current = []
            if part:
                esc = html.escape(part, quote=False)
                current.append(f'<span class="{cls}">{esc}</span>' if cls else esc)
    lines.append("".join(current))
    return lines


# ---------------------------------------------------------------- layout ----

COL, ROW = 280, 94
CARD_MIN, CARD_MAX = 176, 252
LEFT_PAD = 0.78   # columns of room left of column 0; set per chart by place()
TOP_PAD = 30      # room above row 0 for column-lane labels
CHAR = 6.9        # average px per character of a 13 px semibold title
SUB_CHAR = 6.6    # 11.5 px monospace


def wrap(text, limit):
    words, lines, cur = text.split(), [], ""
    for w in words:
        if cur and len(cur) + 1 + len(w) > limit:
            lines.append(cur)
            cur = w
        else:
            cur = f"{cur} {w}".strip()
    if cur:
        lines.append(cur)
    return lines


def xcol(col):
    return 40 + (col + LEFT_PAD) * COL


def yrow(row, top):
    return top + row * ROW + ROW / 2


def left_pad(chart):
    """Room left of column 0: enough for a route bus there, else a margin."""
    buses = [e["route"][1] for e in chart["edges"] if isinstance(e["route"], tuple) and e["route"][1] < 0]
    return (-min(buses) + 0.5) if buses else 0.56


def place(chart):
    global LEFT_PAD
    LEFT_PAD = left_pad(chart)
    top = TOP_PAD if chart["lanes"][0] == "cols" else 12
    placed = {}
    for n in chart["nodes"]:
        title = wrap(n["title"], 26)
        sub = n["sub"] if n["sub"] is not None else (f"{n['func'][1]}()" if n["func"] else None)
        text_w = max([len(t) * CHAR for t in title] + ([len(sub) * SUB_CHAR] if sub else []))
        # 40 px icon column + 18 px right padding, and room for the </> badge.
        w = min(CARD_MAX + (28 if n["func"] else 0), max(CARD_MIN, text_w + 58 + (30 if n["func"] else 0)))
        h = 22 + 17 * len(title) + (17 if sub else 0)
        placed[n["id"]] = {**n, "sub": sub, "lines": title,
                           "cx": xcol(n["col"]), "cy": yrow(n["row"], top), "hw": w / 2, "hh": max(26, h / 2)}
    return placed, top


def rounded(points, r=10):
    """SVG path through `points` with rounded corners."""
    if len(points) < 3:
        return "M" + " L".join(f"{x:.1f},{y:.1f}" for x, y in points)
    d = [f"M{points[0][0]:.1f},{points[0][1]:.1f}"]
    for (x0, y0), (x1, y1), (x2, y2) in zip(points, points[1:], points[2:]):
        l1 = max(1e-6, abs(x1 - x0) + abs(y1 - y0))
        l2 = max(1e-6, abs(x2 - x1) + abs(y2 - y1))
        rr = min(r, l1 / 2, l2 / 2)
        ax, ay = x1 - (x1 - x0) / l1 * rr, y1 - (y1 - y0) / l1 * rr
        bx, by = x1 + (x2 - x1) / l2 * rr, y1 + (y2 - y1) / l2 * rr
        d.append(f"L{ax:.1f},{ay:.1f} Q{x1:.1f},{y1:.1f} {bx:.1f},{by:.1f}")
    d.append(f"L{points[-1][0]:.1f},{points[-1][1]:.1f}")
    return " ".join(d)


def route(a, b, how):
    """Polyline from card a to card b, and where its label goes."""
    if how == "elbow":
        x1, y1 = a["cx"], a["cy"] + a["hh"]
        x2 = b["cx"] + (b["hw"] if a["cx"] > b["cx"] else -b["hw"])
        return [(x1, y1), (x1, b["cy"]), (x2, b["cy"])], (x1, (y1 + b["cy"]) / 2)
    if isinstance(how, tuple):
        bx = xcol(how[1])
        s = 1 if bx > a["cx"] else -1
        pts = [(a["cx"] + s * a["hw"], a["cy"]), (bx, a["cy"]), (bx, b["cy"]), (b["cx"] + s * b["hw"], b["cy"])]
        return pts, (bx, (a["cy"] + b["cy"]) / 2)
    if abs(a["cx"] - b["cx"]) < 1:            # same column
        s = 1 if b["cy"] > a["cy"] else -1
        y1, y2 = a["cy"] + s * a["hh"], b["cy"] - s * b["hh"]
        return [(a["cx"], y1), (b["cx"], y2)], (a["cx"], (y1 + y2) / 2)
    if abs(a["cy"] - b["cy"]) < 1:            # same row
        s = 1 if b["cx"] > a["cx"] else -1
        x1, x2 = a["cx"] + s * a["hw"], b["cx"] - s * b["hw"]
        return [(x1, a["cy"]), (x2, b["cy"])], ((x1 + x2) / 2, a["cy"])
    s = 1 if b["cx"] > a["cx"] else -1         # out of the side, across, into top/bottom
    v = 1 if b["cy"] > a["cy"] else -1
    x1 = a["cx"] + s * a["hw"]
    y2 = b["cy"] - v * b["hh"]
    return [(x1, a["cy"]), (b["cx"], a["cy"]), (b["cx"], y2)], ((x1 + b["cx"]) / 2, a["cy"])


def tone(label):
    if not label:
        return "plain"
    head = label.split()[0].lower()
    return "yes" if head == "yes" else "no" if head == "no" else "plain"


ICONS = {
    "entry": '<circle cx="0" cy="0" r="7" class="ic-bg"/><path d="M-2.2,-3.6 L3.8,0 L-2.2,3.6 Z" class="ic-fg"/>',
    "step": '<circle cx="0" cy="0" r="7" class="ic-bg"/><circle cx="0" cy="0" r="2.6" class="ic-fg"/>',
    "decision": '<rect x="-5.3" y="-5.3" width="10.6" height="10.6" rx="2" transform="rotate(45)" class="ic-bg"/>'
                '<text x="0" y="0.5" class="ic-q" dominant-baseline="middle">?</text>',
    "end": '<circle cx="0" cy="0" r="7" class="ic-bg"/><path d="M-3.2,0.2 L-0.9,2.6 L3.4,-2.4" class="ic-line"/>',
    "out": '<circle cx="0" cy="0" r="7" class="ic-bg"/><path d="M-3,0 H3 M0.6,-2.6 L3,0 L0.6,2.6" class="ic-line"/>',
}


def svg_chart(chart):
    P, top = place(chart)
    max_col = max(n["col"] for n in chart["nodes"])
    max_row = max(n["row"] for n in chart["nodes"])
    right_bus = max([e["route"][1] for e in chart["edges"] if isinstance(e["route"], tuple)] + [max_col])
    width = int(xcol(max(max_col, right_bus) + 0.5) + 40)
    height = int(top + (max_row + 1) * ROW + 24)
    out = [f'<svg viewBox="0 0 {width} {height}" data-width="{width}" data-height="{height}" '
           f'role="img" aria-label="{html.escape(chart["title"])} flowchart">']
    out.append("<defs>" + "".join(
        f'<marker id="ah-{chart["id"]}-{t}" viewBox="0 0 10 10" refX="8.5" refY="5" markerWidth="6.5" '
        f'markerHeight="6.5" orient="auto-start-reverse"><path d="M1,1 L9,5 L1,9 Z" class="ah ah-{t}"/></marker>'
        for t in ("plain", "yes", "no", "hot")) + "</defs>")

    axis, bands = chart["lanes"]
    out.append('<g class="lanes">')
    for k, (label, a, b) in enumerate(bands):
        if axis == "rows":
            y0, y1 = top + a * ROW + 4, top + (b + 1) * ROW - 4
            out.append(f'<rect class="lane{" alt" if k % 2 else ""}" x="12" y="{y0}" width="{width - 24}" height="{y1 - y0}" rx="14"/>')
            out.append(f'<text class="lane-label" x="28" y="{y0 + 20}">{html.escape(label.upper())}</text>')
        else:
            x0, x1 = xcol(a) - COL / 2 + 8, xcol(b) + COL / 2 - 8
            out.append(f'<rect class="lane{" alt" if k % 2 else ""}" x="{x0:.0f}" y="8" width="{x1 - x0:.0f}" height="{height - 16}" rx="14"/>')
            out.append(f'<text class="lane-label" x="{x0 + 16:.0f}" y="28">{html.escape(label.upper())}</text>')
    out.append("</g>")

    out.append('<g class="edges">')
    chips = []
    for e in chart["edges"]:
        a, b = P[e["from"]], P[e["to"]]
        pts, (lx, ly) = route(a, b, e["route"])
        t = tone(e["label"])
        out.append(f'<path class="edge e-{t}" data-from="{e["from"]}" data-to="{e["to"]}" d="{rounded(pts)}" '
                   f'marker-end="url(#ah-{chart["id"]}-{t})" data-tone="{t}"/>')
        if e["label"]:
            w = 7 * len(e["label"]) + 16
            chips.append(f'<g class="chip c-{t}" data-from="{e["from"]}" data-to="{e["to"]}">'
                         f'<rect x="{lx - w / 2:.1f}" y="{ly - 10:.1f}" width="{w}" height="20" rx="10"/>'
                         f'<text x="{lx:.1f}" y="{ly + 0.5:.1f}" dominant-baseline="middle">{html.escape(e["label"])}</text></g>')
    out.append("</g>")

    out.append('<g class="nodes">')
    for step, p in enumerate(chart["nodes"], 1):
        p = P[p["id"]]
        x, y, hw, hh = p["cx"], p["cy"], p["hw"], p["hh"]
        label = " ".join(p["lines"])
        out.append(f'<g class="node k-{p["kind"]}{" has-code" if p["func"] else ""}" data-id="{p["id"]}" '
                   f'tabindex="0" role="button" aria-label="Step {step}: {html.escape(label)}">')
        out.append(f'<rect class="card" x="{x - hw:.1f}" y="{y - hh:.1f}" width="{2 * hw:.1f}" height="{2 * hh:.1f}" rx="12"/>')
        out.append(f'<rect class="bar" x="{x - hw:.1f}" y="{y - hh + 10:.1f}" width="4" height="{2 * hh - 20:.1f}" rx="2"/>')
        out.append(f'<g class="icon" transform="translate({x - hw + 22:.1f},{y:.1f})">{ICONS[p["kind"]]}</g>')
        tx = x - hw + 40
        lines = len(p["lines"]) + (1 if p["sub"] else 0)
        ty = y - (lines - 1) * 8.5
        for k, line in enumerate(p["lines"]):
            out.append(f'<text class="title" x="{tx:.1f}" y="{ty + 17 * k:.1f}" dominant-baseline="middle">{html.escape(line)}</text>')
        if p["sub"]:
            out.append(f'<text class="sub" x="{tx:.1f}" y="{ty + 17 * len(p["lines"]):.1f}" dominant-baseline="middle">{html.escape(p["sub"])}</text>')
        if p["func"]:
            out.append(f'<g class="code-tag" transform="translate({x + hw - 18:.1f},{y - hh + 13:.1f})"><rect x="-12" y="-8" width="24" height="16" rx="5"/>'
                       f'<text x="0" y="0.5" dominant-baseline="middle">&lt;/&gt;</text></g>')
        out.append("</g>")
    out.append("</g>")
    out.append('<g class="chips">' + "".join(chips) + "</g>")
    out.append("</svg>")
    return "\n".join(out)


# ---------------------------------------------------------------- outputs ---


def functions_of(chart):
    seen, out = set(), []
    for n in chart["nodes"]:
        if n["func"] and n["func"] not in seen:
            seen.add(n["func"])
            out.append(n["func"])
    return out


def mermaid_text(text):
    return text.replace('"', "#quot;").replace("<", "&lt;").replace(">", "&gt;")


def mermaid(chart):
    P, _ = place(chart)
    shapes = {"entry": ("([", "])"), "end": ("([", "])"), "out": ("[/", "/]"), "step": ("[", "]"), "decision": ("{", "}")}
    out = ["```mermaid", "flowchart TD",
           "    classDef k_entry fill:#e8eefd,stroke:#2f5bea,color:#111,stroke-width:1.5px",
           "    classDef k_step fill:#ffffff,stroke:#9aa1ad,color:#111",
           "    classDef k_decision fill:#fff4dc,stroke:#d18b00,color:#111",
           "    classDef k_end fill:#e3f6e8,stroke:#1f9d55,color:#111,stroke-width:1.5px",
           "    classDef k_out fill:#f1f2f5,stroke:#9aa1ad,color:#333"]
    axis, bands = chart["lanes"]
    placed = set()
    for k, (label, a, b) in enumerate(bands):
        members = [n for n in chart["nodes"] if a <= (n["row"] if axis == "rows" else n["col"]) <= b]
        if not members:
            continue
        out.append(f'    subgraph lane{k}["{mermaid_text(label)}"]')
        for n in members:
            s, e = shapes[n["kind"]]
            text = mermaid_text(n["title"]) + (f"<br/><small>{mermaid_text(P[n['id']]['sub'])}</small>" if P[n["id"]]["sub"] else "")
            out.append(f'        {n["id"]}{s}"{text}"{e}')
            placed.add(n["id"])
        out.append("    end")
    for n in chart["nodes"]:
        out.append(f"    class {n['id']} k_{n['kind']}")
    for e in chart["edges"]:
        arrow = f' -- "{mermaid_text(e["label"])}" --> ' if e["label"] else " --> "
        out.append(f"    {e['from']}{arrow}{e['to']}")
    out.append("```")
    return "\n".join(out)


def markdown():
    out = [
        "# Flowcharts",
        "",
        "How the LCM simulator runs, as flowcharts, each followed by the source of every function",
        "it names. The interactive version is [`web/docs.html`](../web/docs.html): every step",
        "explains itself, steps marked `</>` open their code, and a guided tour walks through each chart.",
        "",
        "> Generated by [`scripts/gen-flowcharts.py`](../scripts/gen-flowcharts.py). The code blocks",
        "> are copied from the source files, so edits made here are overwritten. Re-run the script",
        "> after changing any function a chart names.",
        "",
        "## Contents",
        "",
    ]
    for k, c in enumerate(CHARTS, 1):
        anchor = re.sub(r"[^a-z0-9 -]", "", f"{k}. {c['title']}".lower()).replace(" ", "-")
        out.append(f"{k}. [{c['title']}](#{anchor}) — {c['short']}")
    for k, c in enumerate(CHARTS, 1):
        out += ["", "---", "", f"## {k}. {c['title']}", "", c["blurb"], "", mermaid(c), ""]
        out += [f"- {note}" for note in c["notes"]]
        described = [n for n in c["nodes"] if n["desc"]]
        if described:
            out += ["", "| Step | What happens |", "| --- | --- |"]
            out += [f"| {n['title']} | {n['desc']} |" for n in described]
        for path, name in functions_of(c):
            line, src = extract(path, name)
            lang = "rust" if path.endswith(".rs") else "js"
            out += ["", f"`{name}` — [`{path}:{line}`](../{path}#L{line})", "", f"```{lang}", src, "```"]
    return "\n".join(out) + "\n"


PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>How it works · LCM Simulator</title>
  <meta name="description" content="Interactive flowcharts of the LCM simulator's Rust core, with the source of every step.">
  <!-- Generated by scripts/gen-flowcharts.py; edits here are overwritten. -->
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
    <a class="btn" href="index.html" title="Back to the simulator"><span data-icon="logo" data-size="20"></span><span class="brand-name">LCM</span></a>
    <span class="crumb">How it works</span>
    <span class="spacer"></span>
    <button id="theme" class="btn" title="Theme" aria-label="Theme"><span data-icon="moon"></span></button>
    <a class="btn outline" href="index.html"><span data-icon="play" data-size="14"></span>Open the simulator</a>
  </header>
  <div class="docs-main">
    <nav class="docs-nav" aria-label="Charts">
      <a class="sim-card" href="index.html">
        <span class="sim-card-icon" data-icon="play" data-size="16"></span>
        <span><span class="sim-card-title">Open the simulator</span><span class="sim-card-sub">Run the swarm these charts describe</span></span>
      </a>
      <div class="nav-title">Flowcharts</div>
      __NAV__
      <div class="nav-title">Legend</div>
      <ul class="legend-list">
        <li><svg width="18" height="18" viewBox="-9 -9 18 18" class="k-entry">__IC_ENTRY__</svg>Where the chart starts</li>
        <li><svg width="18" height="18" viewBox="-9 -9 18 18" class="k-step">__IC_STEP__</svg>A step</li>
        <li><svg width="18" height="18" viewBox="-9 -9 18 18" class="k-decision">__IC_DECISION__</svg>A question with branches</li>
        <li><svg width="18" height="18" viewBox="-9 -9 18 18" class="k-end">__IC_END__</svg>The run or cycle finishes</li>
        <li><svg width="18" height="18" viewBox="-9 -9 18 18" class="k-out">__IC_OUT__</svg>Another way this pass ends</li>
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
"""


def page():
    nav, sections, data = [], [], {"repo": REPO_URL, "charts": {}}
    for k, c in enumerate(CHARTS, 1):
        nav.append(f'<a class="nav-item" href="#{c["id"]}" data-chart="{c["id"]}"><span class="nav-num">{k}</span>'
                   f'<span><span class="nav-name">{html.escape(c["title"])}</span><span class="nav-short">{html.escape(c["short"])}</span></span></a>')
        notes = "".join(f"<li>{html.escape(n)}</li>" for n in c["notes"])
        sections.append(
            f'<section class="chart" data-chart="{c["id"]}" hidden>'
            f'<div class="chart-head"><div><div class="eyebrow">Chart {k} of {len(CHARTS)}</div><h1>{html.escape(c["title"])}</h1>'
            f'<p class="blurb">{html.escape(c["blurb"])}</p></div></div>'
            f'<div class="stage-wrap"><div class="stage" tabindex="0" aria-label="Chart canvas: drag or scroll to move, Ctrl + scroll to zoom">{svg_chart(c)}</div>'
            f'<div class="stage-tools">'
            f'<button class="btn primary small tour-start"><span data-icon="play" data-size="13"></span>Guided tour</button>'
            f'<span class="stage-hint">Click a step for details · drag or scroll to move · Ctrl + scroll to zoom</span>'
            f'<button class="btn small" data-zoom="out" title="Zoom out (−)" aria-label="Zoom out"><span data-icon="minus" data-size="15"></span></button>'
            f'<span class="zoom-level num">100%</span>'
            f'<button class="btn small" data-zoom="in" title="Zoom in (+)" aria-label="Zoom in"><span data-icon="plus" data-size="15"></span></button>'
            f'<button class="btn small" data-zoom="fit" title="Fit (F)" aria-label="Fit"><span data-icon="fit" data-size="15"></span></button>'
            f'</div></div>'
            f'<details class="notes" open><summary>Notes</summary><ul>{notes}</ul></details></section>'
        )
        P, _ = place(c)
        chart = {"title": c["title"], "order": [n["id"] for n in c["nodes"]], "nodes": {}}
        for n in c["nodes"]:
            entry = {"title": n["title"], "sub": P[n["id"]]["sub"], "kind": n["kind"], "desc": n["desc"],
                     "out": [e["to"] for e in c["edges"] if e["from"] == n["id"]],
                     "in": [e["from"] for e in c["edges"] if e["to"] == n["id"]],
                     "labels": {e["to"]: e["label"] for e in c["edges"] if e["from"] == n["id"] and e["label"]}}
            if n["func"]:
                path, name = n["func"]
                line, src = extract(path, name)
                entry["code"] = {"name": name, "path": path, "line": line, "source": src,
                                 "html": highlight(src, "rust" if path.endswith(".rs") else "js")}
            chart["nodes"][n["id"]] = entry
        data["charts"][c["id"]] = chart
    icon = lambda k: ICONS[k]
    text = PAGE
    for k in ICONS:
        text = text.replace(f"__IC_{k.upper()}__", icon(k))
    return (text.replace("__NAV__", "\n      ".join(nav)).replace("__CHARTS__", "\n".join(sections))
            .replace("__DATA__", json.dumps(data, ensure_ascii=False, separators=(",", ":")).replace("</", "<\\/")))


def main():
    outputs = {"docs/flowcharts.md": markdown(), "web/docs.html": page()}
    check = "--check" in sys.argv
    stale = []
    for path, text in outputs.items():
        full = os.path.join(ROOT, path)
        current = open(full, encoding="utf-8").read() if os.path.exists(full) else None
        if check:
            if current != text:
                stale.append(path)
        else:
            with open(full, "w", encoding="utf-8") as f:
                f.write(text)
            print(f"wrote {path}")
    if stale:
        print("out of date: " + ", ".join(stale) + " (run python3 scripts/gen-flowcharts.py)")
        sys.exit(1)
    if check:
        print("flowcharts are up to date")


if __name__ == "__main__":
    main()

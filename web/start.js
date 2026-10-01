/* Starting positions on the page side. Three choices, each answering one
 * question: the pattern (what shape), the arrangement (inside it or on its
 * edge) and the distribution (how robots are spaced). Which combinations
 * exist mirrors `arrangements` / `distributions` in crates/lcm-core/src/start.rs.
 * Also: the parser for custom coordinates, the outline of the start area and
 * the one-line summary of the start. */

// [key, label]. "box" (the original) and "custom" never reach the core's start generator.
export const PATTERNS = [
  ["box", "Original: random in the world box"],
  ["cloud", "Cloud (no shape)"],
  ["square", "Square"],
  ["rectangle", "Rectangle"],
  ["circle", "Circle"],
  ["polygon", "Polygon"],
  ["line", "Line"],
  ["clusters", "Groups (clusters)"],
  ["custom", "My own positions…"],
];

export const ARRANGEMENT_LABELS = { filled: "Filled (inside the shape)", edge: "Edge only", ring: "Ring (with a hole)" };

export const DISTRIBUTION_LABELS = {
  uniform: "Random",
  even: "Evenly spaced",
  jittered: "Evenly spaced, a bit messy",
  gaussian: "Bunched in the middle",
};

/** Arrangements per pattern; must match `arrangements()` in start.rs. */
export const ARRANGEMENTS = {
  box: [], custom: [], cloud: ["filled"], clusters: ["filled"],
  square: ["filled", "edge"], rectangle: ["filled", "edge"], polygon: ["filled", "edge"],
  circle: ["filled", "edge", "ring"], line: ["edge"],
};

/** Distributions per pattern and arrangement; must match `distributions()` in start.rs. */
export function distributionsFor(pattern, arrangement) {
  if (!(ARRANGEMENTS[pattern] ?? []).includes(arrangement)) return [];
  if (pattern === "cloud") return ["gaussian"];
  if (pattern === "clusters") return ["gaussian", "uniform"];
  return arrangement === "filled" ? ["uniform", "even", "jittered", "gaussian"] : ["uniform", "even", "jittered"];
}

/** The spacing a pattern and arrangement start with when picked. */
export function naturalDistribution(pattern, arrangement) {
  if (pattern === "cloud" || pattern === "clusters") return "gaussian";
  if (pattern === "box") return "uniform";
  if (arrangement === "filled") return pattern === "square" || pattern === "rectangle" ? "even" : "uniform";
  return arrangement === "ring" ? "uniform" : "even";
}

const n = (v) => Number(v).toLocaleString();

/** The shape in words, e.g. "a circle 400 wide". */
function shapeWords(v) {
  switch (v.pattern) {
    case "square": return `a square with sides of ${n(v.size)}`;
    case "rectangle": return `a ${n(v.size)} × ${n(Math.round(v.size * v.aspect))} rectangle`;
    case "circle": return `a circle ${n(v.size)} wide`;
    case "polygon": return `a ${v.sides}-sided polygon ${n(v.size)} wide`;
    default: return "";
  }
}

/** One sentence saying what the start settings give, e.g.
 *  "500 robots evenly spaced on the edge of a circle 400 wide." */
export function describeStart(v, customCount) {
  const robots = (k) => `${n(k)} robot${k === 1 ? "" : "s"}`;
  const spacing = { uniform: "placed randomly", even: "evenly spaced", jittered: "roughly evenly spaced", gaussian: "bunched toward the middle" }[v.distribution];
  switch (v.pattern) {
    case "custom": return `${robots(customCount)} at the positions you typed.`;
    case "box": return `${robots(v.num_of_robots)} placed randomly in the ${n(v.width_bound)} × ${n(v.height_bound)} world box, as in the original simulator.`;
    case "cloud": return `${robots(v.num_of_robots)} in a cloud with no edge: most within ${n(2 * v.spread)} of the centre, a few further out.`;
    case "clusters": return `${robots(v.num_of_robots)} in ${v.clusters} group${v.clusters === 1 ? "" : "s"}, ${v.distribution === "uniform" ? `each a disk of radius ${n(v.spread)}` : `each bunched within about ${n(2 * v.spread)} of its centre`}.`;
    case "line": return `${robots(v.num_of_robots)} ${spacing} along a line ${n(v.size)} long.`;
    default: {
      const where = v.arrangement === "edge" ? `on the edge of ${shapeWords(v)}`
        : v.arrangement === "ring" ? `in a ring ${n(v.size)} wide with a hole ${n(Math.round(v.size * v.inner))} wide`
        : `inside ${shapeWords(v)}`;
      return `${robots(v.num_of_robots)} ${spacing} ${where}.`;
    }
  }
}

export const MAX_CUSTOM = 10000;

/**
 * Robot positions from typed or pasted text. Accepted:
 *   - one robot per line: "x, y", "x y", "x;y" or tab-separated
 *   - blank lines and "# comments" are ignored
 *   - a header row naming x and y columns, e.g. the simulator's own CSV export
 *   - a JSON list: [[x, y], …] or [{"x": …, "y": …}, …]
 * Returns { points: [[x, y], …], errors: [{ line, message }] }.
 */
export function parseCoordinates(text) {
  const points = [], errors = [];
  const trimmed = text.trim();
  if (!trimmed) return { points, errors: [{ line: 1, message: "Enter at least one robot as x, y" }] };

  if (trimmed.startsWith("[")) {
    let data;
    try { data = JSON.parse(trimmed); } catch (e) { return { points, errors: [{ line: 1, message: `Not valid JSON: ${e.message}` }] }; }
    if (!Array.isArray(data)) return { points, errors: [{ line: 1, message: "JSON must be a list of points" }] };
    data.forEach((p, i) => {
      const x = Array.isArray(p) ? p[0] : p?.x, y = Array.isArray(p) ? p[1] : p?.y;
      if (Number.isFinite(x) && Number.isFinite(y)) points.push([x, y]);
      else errors.push({ line: i + 1, message: `point ${i + 1} needs two numbers` });
    });
  } else {
    let columns = null; // [x index, y index] from a header row
    text.split(/\r?\n/).forEach((raw, i) => {
      const line = raw.replace(/#.*/, "").trim();
      if (!line) return;
      const cells = (line.includes(",") ? line.split(",") : line.includes(";") ? line.split(";") : line.split(/\s+/)).map((c) => c.trim());
      const isNumber = (c) => c !== "" && Number.isFinite(Number(c));
      if (!columns && points.length === 0 && cells.some((c) => c !== "" && !isNumber(c))) {
        const lower = cells.map((c) => c.toLowerCase());
        const xi = lower.indexOf("x"), yi = lower.indexOf("y");
        if (xi >= 0 && yi >= 0) { columns = [xi, yi]; return; }
      }
      const [xs, ys] = columns ? [cells[columns[0]], cells[columns[1]]] : cells.filter((c) => c !== "");
      if (cells.filter((c) => c !== "").length < 2 && !columns) {
        errors.push({ line: i + 1, message: "needs two numbers: x, y" });
      } else if (!isNumber(xs ?? "") || !isNumber(ys ?? "")) {
        const bad = !isNumber(xs ?? "") ? xs : ys;
        errors.push({ line: i + 1, message: bad ? `“${bad}” is not a number` : "missing x or y" });
      } else {
        points.push([Number(xs), Number(ys)]);
      }
    });
  }
  if (points.length > MAX_CUSTOM) errors.push({ line: MAX_CUSTOM + 1, message: `at most ${MAX_CUSTOM.toLocaleString()} robots` });
  if (!points.length && !errors.length) errors.push({ line: 1, message: "Enter at least one robot as x, y" });
  return { points, errors };
}

/** Pairs of robots placed on exactly the same spot (a warning, not an error). */
export function duplicates(points) {
  const seen = new Set();
  let count = 0;
  for (const [x, y] of points) {
    const key = `${x},${y}`;
    if (seen.has(key)) count++; else seen.add(key);
  }
  return count;
}

/** The outline the renderer draws for the start area, or null for none. */
export function startOutline(values) {
  const p = values.pattern, half = values.size / 2, rotation = values.rotation ?? 0;
  if (!values.open_world && p === "box") return { kind: "rect", w: values.width_bound, h: values.height_bound, rotation: 0 };
  switch (p) {
    case "square": return { kind: "rect", w: values.size, h: values.size, rotation };
    case "rectangle": return { kind: "rect", w: values.size, h: values.size * values.aspect, rotation };
    case "circle": return values.arrangement === "ring" ? { kind: "ring", r: half, inner: values.inner * half } : { kind: "circle", r: half };
    case "line": return { kind: "line", length: values.size, rotation };
    case "polygon": return { kind: "polygon", r: half, sides: values.sides, rotation };
    default: return null; // cloud, clusters and custom positions have no fixed shape
  }
}

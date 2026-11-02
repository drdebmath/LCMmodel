/* The Algorithm menu, shared by the simulator and the ring page so both show the same choices.
 * "Advanced gathering" is one entry: its variant is chosen in the simulator's sidebar.
 * "Gathering on a ring" runs on its own page (nodes, not points). */

export const MENU = [
  ["Original simulator", ["Gathering", "SEC", "CircleFormation", "GoToCenter", "Spreading", "TwoTask"]],
  ["Sequential gathering", ["SqGathering"]],
  ["Advanced gathering", ["advanced"]],
  ["Other tasks", ["MutualVisibility", "ring"]],
];
export const MENU_NAMES = { advanced: "Advanced gathering…", ring: "Gathering on a ring" };

/** The algorithms behind "Advanced gathering", as [registry key, label]. */
export const ADVANCED_VARIANTS = [
  ["CompassGathering", "Unreliable compasses · 2 robots"],
  ["RandomizedGathering", "Randomized, λ ∈ {0, ½, 1} · 2 robots"],
  ["RandomizedGatheringUniform", "Randomized, λ ∈ {1, ½, uniform} · 2 robots"],
  ["CrashGathering", "Despite crash faults · weak multiplicity"],
];
export const ADVANCED = ADVANCED_VARIANTS.map(([key]) => key);

/** The menu's <optgroup>s for the core's algorithm list (`[{ key, name }]`). Anything the core knows that the
 * menu does not place yet still shows up, under "More". */
export function menuHtml(list) {
  const known = new Set(list.map((a) => a.key));
  const name = (key) => MENU_NAMES[key] ?? list.find((a) => a.key === key)?.name ?? key;
  const listed = new Set(MENU.flatMap(([, keys]) => keys));
  const groups = MENU.map(([label, keys]) => [label, keys.filter((k) => k === "advanced" || k === "ring" || known.has(k))]);
  const rest = list.map((a) => a.key).filter((k) => !listed.has(k) && !ADVANCED.includes(k));
  if (rest.length) groups.push(["More", rest]);
  return groups.filter(([, keys]) => keys.length)
    .map(([label, keys]) => `<optgroup label="${label}">${keys.map((k) => `<option value="${k}">${name(k)}</option>`).join("")}</optgroup>`).join("");
}

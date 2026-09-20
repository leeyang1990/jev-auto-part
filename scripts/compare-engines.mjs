import { isParked as parked } from "../public/parking-goal.js";
// Runs the same scenario through both engines and prints the gap between them.
// The engines receive identical perception, identical candidates and identical
// tolerances; only the interface differs, and that difference is the experiment.

const BASE_URL = process.env.TEST_BASE_URL || "http://127.0.0.1:4173";
const MAX_MOVES = Number(process.env.TEST_MAX_MOVES || 30);
const ONLY = process.argv[2] || null;

const scenarios = {
  "offset-bay": { start: { x: 2.65, y: 0.25, heading: 180 }, target: { x: 0, y: -2.45, heading: 90 } },
  "tight-corner": { start: { x: 4.05, y: 0.65, heading: -145 }, target: { x: 0, y: -2.45, heading: 90 } },
  "reverse-entry": { start: { x: 0.22, y: 0.35, heading: 88 }, target: { x: 0, y: -2.45, heading: 90 } },
  "left-offset": { start: { x: -2.65, y: 0.25, heading: 0 }, target: { x: 0, y: -2.45, heading: 90 } },
  "forward-entry": { start: { x: 0.18, y: 1.15, heading: -88 }, target: { x: 0, y: -2.45, heading: -90 } },
  "wide-angle": { start: { x: -4, y: 2.15, heading: 24 }, target: { x: 0, y: -2.45, heading: 90 } },
  custom: { start: { x: 2.65, y: 0.25, heading: 180 }, target: { x: 0, y: -2.45, heading: 90 } },
};

function distance(a, b) { return Math.hypot(b.x - a.x, b.y - a.y); }
function angleError(a, b) { let value = (b - a) % 360; if (value > 180) value -= 360; if (value < -180) value += 360; return Math.abs(value); }
function round(value) { return Math.round(Number(value) * 1000) / 1000; }

async function run(engine, scenario) {
  const { start, target } = scenarios[scenario];
  let pose = { ...start };
  const history = [];
  const totals = { path: 0, blocked: 0, recoveries: 0, latencies: [], minClearance: Infinity, crashes: 0, interface: null, moves: 0, parked: false };
  for (let move = 1; move <= MAX_MOVES; move += 1) {
    const started = performance.now();
    const response = await fetch(BASE_URL + "/api/decision", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ engine, scenario, pose, target, history: history.slice(-10), reset: move === 1 }),
    });
    const data = await response.json();
    if (!response.ok) throw new Error(engine + " " + scenario + " decision " + move + " failed: " + (data.error || response.status));
    const latency = performance.now() - started;
    totals.latencies.push(latency);
    totals.interface = data.decisionInterface || totals.interface;
    totals.path += Number(data.pathM) || 0;
    totals.moves = move;
    if (data.blocked) totals.blocked += 1;
    if (data.recovery?.active) totals.recoveries += 1;
    if (data.actualCollision) totals.crashes += 1;
    if (Number.isFinite(Number(data.clearance))) totals.minClearance = Math.min(totals.minClearance, Number(data.clearance));
    pose = data.projection;
    history.push({
      action: data.action,
      requestedAction: data.requestedAction,
      distance: data.projectedDistance,
      navigationDistance: data.projectedNavigationDistance,
      navigationStage: data.navigation?.stageId || null,
      angleError: data.projectedAngleError,
      clearance: data.clearance,
      blocked: Boolean(data.blocked),
      pose,
      control: data.control,
    });
    if (parked(pose, target)) { totals.parked = true; break; }
  }
  const average = totals.latencies.reduce((sum, value) => sum + value, 0) / Math.max(1, totals.latencies.length);
  return {
    engine, scenario,
    outcome: totals.parked ? "parked" : "unfinished",
    moves: totals.moves,
    distanceM: round(distance(pose, targets(scenario))),
    angleDeg: round(angleError(pose.heading, targets(scenario).heading)),
    pathM: round(totals.path),
    minClearanceM: round(totals.minClearance),
    blocked: totals.blocked,
    recoveries: totals.recoveries,
    crashes: totals.crashes,
    averageDecisionMs: Math.round(average),
    interface: totals.interface,
  };
}
function targets(scenario) { return scenarios[scenario].target; }

const rows = [];
for (const scenario of Object.keys(scenarios)) {
  if (ONLY && scenario !== ONLY) continue;
  rows.push(await run("jev", scenario));
  rows.push(await run("llm", scenario));
}

const header = ["scenario", "engine", "outcome", "moves", "pathM", "minClear", "blocked", "recoveries", "crashes", "avgMs"];
console.log(header.join("\t"));
for (const row of rows) {
  console.log([row.scenario, row.engine, row.outcome, row.moves, row.pathM, row.minClearanceM, row.blocked, row.recoveries, row.crashes, row.averageDecisionMs].join("\t"));
}
console.log("");
for (const scenario of new Set(rows.map((row) => row.scenario))) {
  const jev = rows.find((row) => row.scenario === scenario && row.engine === "jev");
  const llm = rows.find((row) => row.scenario === scenario && row.engine === "llm");
  console.log(scenario + ": jev " + jev.outcome + " in " + jev.moves + " moves vs llm " + llm.outcome + " in " + llm.moves + " moves; decision time " + jev.averageDecisionMs + "ms vs " + llm.averageDecisionMs + "ms");
}
const incomparable = rows.filter((row) => row.interface && !row.interface.includes("typed") && row.engine === "jev").length
  + rows.filter((row) => row.interface && !row.interface.includes("semantic") && row.engine === "llm").length;
console.log(incomparable === 0 ? "interface check: jev used typed questions, llm used the semantic briefing" : "interface check: WARNING, an engine used the wrong interface");
console.log(JSON.stringify(rows, null, 1));

const BASE_URL = process.env.TEST_BASE_URL || "http://127.0.0.1:4173";
const MAX_MOVES = Number(process.env.TEST_MAX_MOVES || 30);
const ENGINE = process.argv[2] || "llm";
const SCENARIO = process.argv[3] || "reverse-entry";
// TEST_NO_HISTORY=1 sends an empty history on every call. That isolates whether
// progress memory is what lets the model break out of a repeated maneuver.
const NO_HISTORY = process.env.TEST_NO_HISTORY === "1";

const scenarios = {
  "offset-bay": { start: { x: 2.65, y: 0.25, heading: 180 }, target: { x: 0, y: -2.45, heading: 90 } },
  "tight-corner": { start: { x: 4.05, y: 0.65, heading: -145 }, target: { x: 0, y: -2.45, heading: 90 } },
  "reverse-entry": { start: { x: 0.22, y: 0.35, heading: 88 }, target: { x: 0, y: -2.45, heading: 90 } },
  "left-offset": { start: { x: -2.65, y: 0.25, heading: 0 }, target: { x: 0, y: -2.45, heading: 90 } },
  "forward-entry": { start: { x: 0.18, y: 1.15, heading: -88 }, target: { x: 0, y: -2.45, heading: -90 } },
  "wide-angle": { start: { x: -4, y: 2.15, heading: 24 }, target: { x: 0, y: -2.45, heading: 90 } },
  custom: { start: { x: 2.65, y: 0.25, heading: 180 }, target: { x: 0, y: -2.45, heading: 90 } },
};

if (!Object.hasOwn(scenarios, SCENARIO)) throw new Error(`Unknown scenario: ${SCENARIO}`);
if (!new Set(["jev", "llm"]).has(ENGINE)) throw new Error(`Unknown engine: ${ENGINE}`);

const { start, target } = scenarios[SCENARIO];
let pose = { ...start };
const history = [];
const totals = { path: 0, interventions: 0, dangerous: 0, minClearance: Infinity };

for (let move = 1; move <= MAX_MOVES; move += 1) {
  const started = performance.now();
  const response = await fetch(`${BASE_URL}/api/decision`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ engine: ENGINE, scenario: SCENARIO, pose, target, history: NO_HISTORY ? [] : history.slice(-10), reset: move === 1, runId: `e2e-${process.pid}` }),
  });
  const data = await response.json();
  if (!response.ok) throw new Error(`Decision ${move} failed: ${data.error || response.status}`);
  if (data.actualCollision) {
    console.error("COLLISION_DIAGNOSTIC", JSON.stringify({
      move, requested: data.requestedAction, requestedPlan: data.requestedPlan,
      control: data.control, projection: data.projection, clearance: data.clearance,
      perceivedCollision: data.perceivedCollision, perceivedConflict: data.perceivedConflict,
      perceptionDebug: data.perceptionDebug, tracks: data.scene?.tracks,
    }));
    throw new Error(`Move ${move} required a pre-motion emergency stop for ${data.collisionWith} at ${data.collisionAtS}s`);
  }
  if (!Array.isArray(data.candidateLines) || data.candidateLines.length === 0) {
    throw new Error(`Move ${move} returned no evaluated prediction lines`);
  }
  const requestedLine = data.candidateLines.find((line) => line.id === data.requestedAction);
  if (!requestedLine) {
    console.error("PREDICTION_LINE_DIAGNOSTIC", JSON.stringify({
      move, requestedAction: data.requestedAction, action: data.action, blocked: data.blocked,
      requestedControl: data.requestedControl, requestedPlan: data.requestedPlan,
      candidateLineIds: data.candidateLines.map((line) => line.id),
    }));
    throw new Error(`Move ${move} omitted the selected prediction line`);
  }
  if (!data.blocked) {
    const samples = requestedLine.path || [];
    const end = samples.at(-1);
    if (!end || Math.hypot(Number(end[0]) - Number(data.projection.x), Number(end[1]) - Number(data.projection.y)) > 0.03) {
      throw new Error(`Move ${move} displayed a prediction line that differs from its executable rollout`);
    }
  }
  if (data.blocked && (data.chosenAction !== null || data.action !== "stop" || !requestedLine.blocked || requestedLine.chosen)) {
    throw new Error(`Move ${move} did not represent a blocked choice as an unchosen stopped trajectory`);
  }
  if (data.proposedBy != null) throw new Error(`Move ${move} exposed forbidden planner proposal: ${data.proposedBy}`);
  if (data.action !== "stop" && data.segments !== 1) throw new Error(`Move ${move} did not return a one-step control`);
  if (Array.isArray(data.requestedControl?.travelDistance) || !Number.isFinite(Number(data.requestedControl?.travelDistance))) {
    throw new Error(`Move ${move} returned invalid selected control metadata`);
  }
  if (data.requestedAction !== "stop" && (!Number.isFinite(Number(data.requestedControl?.targetSpeed)) || Number(data.requestedControl.targetSpeed) <= 0)) {
    throw new Error(`Move ${move} returned no target-speed control`);
  }
  if (!data.blocked && Number(data.segments) !== data.segmentsDetail?.length) throw new Error(`Move ${move} returned inconsistent trajectory segments`);
  // A hold is always allowed: when the model asks for something its own map
  // rejects, the car holds and the loop continues. What must never happen is an
  // executed motion the map considered unsafe.
  // Legal means: no contact at the end of the path, and either comfortably clear
  // or no worse than the pose it started from (the escape rule).
  const escapes = Number(data.startClearance) < 0.08 && Number(data.clearance) >= Number(data.startClearance) - 0.02;
  if (!data.blocked && (data.perceivedCollision || (Number(data.clearance) < 0.08 && !escapes))) {
    throw new Error(`Unsafe command was returned for execution on move ${move}`);
  }
  if (data.blocked && data.action !== "stop") throw new Error(`Move ${move} blocked but executed ${data.action}`);

  const control = data.control || {};
  pose = data.projection;
  totals.path += Number(control.travelDistance) || 0;
  totals.interventions += data.blocked ? 1 : 0;
  totals.dangerous += data.blocked ? 1 : 0;
  totals.minClearance = Math.min(totals.minClearance, Number(data.clearance));
  history.push({
    action: data.action,
    requestedAction: data.requestedAction,
    distance: data.projectedDistance,
    navigationDistance: data.projectedNavigationDistance,
    navigationStage: data.navigation?.stageId || null,
    navigationStageIndex: data.navigation?.stageIndex ?? 0,
    angleError: data.projectedAngleError,
    clearance: data.clearance,
    blocked: Boolean(data.blocked),
    pose,
    control,
  });

  const line = {
    move,
    latencyMs: Math.round(performance.now() - started),
    requested: data.requestedAction,
    requestedPlan: data.requestedPlan,
    executed: data.action,
    gear: control.gear,
    steerDeg: control.steerDeg,
    travelM: control.travelDistance,
    distanceM: round(distance(pose, target)),
    angleDeg: round(angleError(pose.heading, target.heading)),
    clearanceM: round(data.clearance),
    blocked: Boolean(data.blocked),
    source: data.decisionSource,
    navigationStage: data.navigation?.stageId,
    navigationDistanceM: round(data.projectedNavigationDistance),
    recovery: Boolean(data.recovery?.active),
    oscillating: Boolean(data.historyAnalysis?.oscillating),
    perceivedConflict: data.perceivedConflict,
    unknownFraction: data.unknownFraction,
    mappedCells: data.scene?.cells?.length ?? null,
    tracked: (data.scene?.tracks || []).map((track) => track.id + (track.moving ? "*" : "")),
  };
  console.log(JSON.stringify(line));

  if (isParked(pose, target)) {
    console.log("RESULT", JSON.stringify({
      engine: ENGINE,
      scenario: SCENARIO,
      result: "parked",
      moves: move,
      pose,
      pathM: round(totals.path),
      minClearanceM: round(totals.minClearance),
      blocked: totals.interventions,
      dangerous: totals.dangerous,
    }));
    process.exit(0);
  }
}

console.log("RESULT", JSON.stringify({
  engine: ENGINE,
  scenario: SCENARIO,
  result: "move-cap",
  moves: MAX_MOVES,
  pose,
  distanceM: round(distance(pose, target)),
  angleDeg: round(angleError(pose.heading, target.heading)),
  pathM: round(totals.path),
  minClearanceM: round(totals.minClearance),
  blocked: totals.interventions,
  dangerous: totals.dangerous,
}));
process.exitCode = 2;

function distance(a, b) { return Math.hypot(b.x - a.x, b.y - a.y); }
function angleError(a, b) { let value = (b - a) % 360; if (value > 180) value -= 360; if (value < -180) value += 360; return Math.abs(value); }
function isParked(a, b) { return distance(a, b) <= 0.28 && angleError(a.heading, b.heading) <= 7; }
function round(value) { return Math.round(Number(value) * 1000) / 1000; }

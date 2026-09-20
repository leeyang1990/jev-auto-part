const BASE_URL = process.env.TEST_BASE_URL || "http://127.0.0.1:4173";
const ENGINE = process.argv[2] || "jev";
const SCENARIO = process.argv[3] || "reverse-entry";

const scenarios = {
  "offset-bay": { start: { x: 2.65, y: 0.25, heading: 180 }, target: { x: 0, y: -2.45, heading: 90 } },
  "tight-corner": { start: { x: 4.05, y: 0.65, heading: -145 }, target: { x: 0, y: -2.45, heading: 90 } },
  "reverse-entry": { start: { x: 0.22, y: 0.35, heading: 88 }, target: { x: 0, y: -2.45, heading: 90 } },
};
const { start, target } = scenarios[SCENARIO];
const started = performance.now();
const response = await fetch(`${BASE_URL}/api/decision`, {
  method: "POST",
  headers: { "Content-Type": "application/json" },
  body: JSON.stringify({ engine: ENGINE, scenario: SCENARIO, pose: start, target, history: [] }),
});
const data = await response.json();
console.log(JSON.stringify({
  status: response.status,
  wallMs: Math.round(performance.now() - started),
  model: data.model,
  decisionSource: data.decisionSource,
  motion: data.motion,
  action: data.action,
  control: data.control,
  projectedDistance: data.projectedDistance,
  projectedAngleError: data.projectedAngleError,
  clearance: data.clearance,
  confidence: data.confidence,
  recoveryActive: data.recovery?.active,
  usage: data.usage,
  rationale: data.rationale,
  error: data.error,
  raw: data.error ? data.raw : undefined,
}, null, 1));

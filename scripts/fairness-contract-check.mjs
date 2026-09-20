import assert from "node:assert/strict";
import { semanticBriefing } from "../briefing.js";
import { ParkingSessionStore } from "../parking/session-store.js";
import { prepareRequest } from "../parking/jev-request.js";
import { stopAvailability } from "../parking/policy.js";

const store = new ParkingSessionStore();
const pose = { x: 0.42, y: -2.05, heading: 88 };
const target = { x: 0, y: -2.45, heading: 90 };
const history = [];

const build = (engine) => store.perceive({
  engine, scenario: "reverse-entry", pose, target, history, reset: true,
  runId: "fairness-contract",
});
const jev = build("jev");
const llm = build("llm");

const summarize = (context) => Object.fromEntries(Object.entries(context.candidates).map(([id, item]) => [id, {
  path: item.path,
  plan: item.plan,
  clearance: item.clearance,
  parkedAfter: item.parkedAfter,
  parkingToleranceRatio: item.parkingToleranceRatio,
} ]));
assert.deepEqual(summarize(jev), summarize(llm),
  "engine identity must not change generated, smoothed or collision-checked trajectories");

const prepared = prepareRequest({
  scenario: "reverse-entry", pose, target, navigation: jev.navigation,
  history: jev.stageHistory, analysis: jev.analysis, candidates: jev.candidates,
  stop: stopAvailability(jev.candidates, pose, target), recovery: jev.recovery,
  environment: jev.environment,
});
const briefing = semanticBriefing({
  scenario: "reverse-entry", pose, target, navigation: llm.navigation,
  analysis: llm.analysis, recovery: llm.recovery, candidates: llm.candidates,
  environment: llm.environment,
});
assert.deepEqual(Object.keys(prepared.eligible), briefing.order,
  "Jev and LLM must receive the same admitted IDs in the same order");
assert.ok(briefing.order.length > 0, "fairness check must exercise moving choices");
for (const item of Object.values(prepared.eligible)) {
  assert.equal(item.path, item.horizonPath,
    "display, collision checking and execution must use the same trajectory object");
  assert.equal(item.segments, 1, "every model choice must be one immediate control");
}
console.log("fairness-contract-check: passed", JSON.stringify({ choices: briefing.order.length }));

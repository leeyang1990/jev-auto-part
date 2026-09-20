import assert from "node:assert/strict";
import { semanticBriefing } from "../briefing.js";
import { prepareRequest } from "../parking/jev-request.js";
import { ParkingSessionStore } from "../parking/session-store.js";
import { decisionCandidates, stopAvailability } from "../parking/policy.js";

const pose = { x: 2.65, y: 0.25, heading: 180 };
const target = { x: 0, y: -2.45, heading: 90 };
const store = new ParkingSessionStore();
const context = store.perceive({ engine: "jev", scenario: "offset-bay", pose, target, history: [], reset: true, runId: "shared-check" });
const candidates = decisionCandidates(context.candidates, context.recovery);
const prepared = prepareRequest({
  scenario: "offset-bay", pose, target, navigation: context.navigation, history: context.stageHistory,
  analysis: context.analysis, candidates: context.candidates, stop: stopAvailability(context.candidates, pose, target),
  recovery: context.recovery, environment: context.environment,
});
const briefing = semanticBriefing({
  scenario: "offset-bay", task: "test", pose, target, navigation: context.navigation, analysis: context.analysis,
  recovery: context.recovery, candidates: context.candidates, environment: context.environment,
});

assert.deepEqual(Object.keys(prepared.eligible), Object.keys(candidates));
assert.deepEqual(briefing.order, Object.keys(candidates));
assert.ok(Object.values(candidates).every((item) => item.segments === 1));
assert.ok(Object.values(candidates).every((item) => item.control.steeringProfile.length === 1));
assert.ok(Object.values(candidates).every((item) => item.path === item.horizonPath));
assert.ok(Object.values(candidates).some((item) => item.control.direction < 0));
assert.ok(Object.values(candidates).some((item) => item.control.direction > 0));
assert.ok(Object.values(candidates).some((item) => item.control.steerDeg < 0));
assert.ok(Object.values(candidates).some((item) => item.control.steerDeg > 0));

const finishPose = { x: 0, y: -2.14, heading: 90 };
const finishContext = store.perceive({
  engine: "jev", scenario: "reverse-entry", pose: finishPose, target,
  history: [], reset: true, runId: "shared-finish-check",
});
const finishCandidates = decisionCandidates(finishContext.candidates, finishContext.recovery);
assert.ok(Object.values(finishCandidates).some((item) => item.parkedAfter));
console.log("shared-control-check: passed", JSON.stringify({ choices: Object.keys(candidates).length, segments: 1 }));

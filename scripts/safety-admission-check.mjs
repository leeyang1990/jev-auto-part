import assert from "node:assert/strict";
import { semanticBriefing } from "../briefing.js";
import { createControlCandidates } from "../parking/controller.js";
import { prepareRequest } from "../parking/jev-request.js";
import { movingCandidates, stopAvailability } from "../parking/policy.js";
import { navigationState } from "../parking/navigation.js";

const pose = { x: 0, y: 0, heading: 90 };
const target = { x: 0, y: -2.45, heading: 90 };
const baseControl = { gear: "reverse", direction: -1, steerDeg: 0, targetSpeed: 0.22, validityS: 1, travelDistance: 0.22, duration: 1 };
const candidate = (action, overrides = {}) => ({
  action, pose: { x: 0, y: -0.22, heading: 90 }, path: [pose, { x: 0, y: -0.22, heading: 90 }],
  distance: 2.23, angleError: 0, clearance: 0.5, startClearance: 0.5, collision: false, eligible: true,
  control: { ...baseControl }, plan: "R0@0.22:1.00s", navigationProgress: 0.22, navigationDistance: 2.23,
  navigationLateralError: 0, navigationHeadingError: 0, parkingToleranceRatio: 8, ...overrides,
});
const candidates = {
  safe: candidate("safe"),
  colliding: candidate("colliding", { collision: true, clearance: -0.1 }),
  perceived: candidate("perceived", { perceivedTrackCollision: true }),
  stop: { action: "stop", pose, path: [pose], collision: false, eligible: true, clearance: 0.5, control: { gear: "park", duration: 0 } },
};
const analysis = { recentRequestCounts: {}, recentBlockedCounts: {}, recoveryActive: false, recoveryReasons: [], repeatedActions: [], gearChanges: 0, stagnantMoves: 0, blockedMoves: 0, bestDistanceM: null };
const navigation = { stageId: "park", stageIndex: 0, stageCount: 1, label: "Park", finalStage: true, remainingM: 2.45, lateralErrorM: 0, headingErrorDeg: 0, goal: target };

assert.deepEqual(Object.keys(movingCandidates(candidates)), ["safe"]);
const prepared = prepareRequest({ scenario: "reverse-entry", pose, target, navigation, history: [], analysis, candidates, stop: stopAvailability(candidates, pose, target), recovery: { active: false } });
assert.deepEqual(Object.keys(prepared.eligible), ["safe"]);
assert.deepEqual(Object.values(prepared.aliases), ["safe"]);
const briefing = semanticBriefing({ scenario: "reverse-entry", pose, target, navigation, analysis, recovery: { active: false }, candidates });
assert.deepEqual(briefing.order, ["safe"]);

const rollouts = createControlCandidates({
  pose: { x: 0, y: -2.1, heading: 90 }, target, navigation,
  world: { clearanceAt: () => 1 },
});
const movingRollouts = Object.values(rollouts).filter((item) => item.action !== "stop");
assert.ok(movingRollouts.length > 2);
assert.ok(movingRollouts.every((item) => item.segments === 1 && item.path.length > 3));
assert.ok(movingRollouts.some((item) => item.control.steeringProfile.some((segment) => segment.targetSpeed === 0.08)));
assert.ok(movingRollouts.every((item) => item.control.steeringProfile.every((segment) => segment.targetSpeed >= 0.08 && segment.targetSpeed <= 0.62)));
const recoveryRollouts = createControlCandidates({
  pose: { x: 0.49, y: -2.56, heading: 90.48 }, target, navigation, recovery: { active: true },
  world: { clearanceAt: () => 1 },
});
const recoveryMoving = Object.values(recoveryRollouts).filter((item) => item.action !== "stop");
assert.ok(recoveryMoving.every((item) => item.segmentsDetail.every((segment) =>
  segment.duration >= 0.2 && segment.duration <= 5.25)),
"recovery controls remain bounded while moving far enough to leave a repeated pose");
assert.equal(recoveryMoving.some((item) => item.segmentsDetail.some((segment) => segment.targetSpeed === 0.08)), true,
  "final-stage recovery must retain low-speed trim trajectories");

const offGate = navigationState("wide-angle", { x: -4.0, y: 0.0, heading: 90 }, target, 0);
assert.equal(offGate.stageId, "reach_aisle");
assert.ok(offGate.remainingM > 0.65, "lateral route error must remain visible to the model");
const atGateFromSide = navigationState("wide-angle", { x: -2.7, y: 0.75, heading: -15 }, target, 0);
assert.equal(atGateFromSide.stageId, "approach_bay", "reaching a bounded route endpoint must advance the stage without crossing an infinite plane");

console.log("Safety admission and fine-control checks passed");

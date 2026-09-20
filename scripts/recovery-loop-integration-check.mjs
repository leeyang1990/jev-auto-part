import assert from "node:assert/strict";
import { ParkingSessionStore } from "../parking/session-store.js";
import { annotateCandidateRecovery, decisionCandidates } from "../parking/policy.js";

const store = new ParkingSessionStore();
const target = { x: 0, y: -2.45, heading: 90 };
const poses = [
  { x: 0.50, y: -0.10, heading: 90 },
  { x: 0.22, y: -0.12, heading: 94 },
  { x: 0.49, y: -0.09, heading: 91 },
  { x: 0.23, y: -0.11, heading: 95 },
];
const history = poses.map((pose, index) => ({
  pose, action: index % 2 ? "reverse_z0_v22" : "forward_z0_v22",
  requestedAction: index % 2 ? "reverse_z0_v22" : "forward_z0_v22",
  distance: Math.hypot(pose.x - target.x, pose.y - target.y),
  navigationDistance: Math.hypot(pose.x - target.x, pose.y - target.y),
  navigationStage: "park", angleError: Math.abs(pose.heading - target.heading),
  clearance: 0.5, blocked: false,
  control: { gear: index % 2 ? "reverse" : "forward", steerDeg: 0, targetSpeed: 0.22, travelDistance: 0.22 },
}));
const context = store.perceive({ engine: "jev", scenario: "reverse-entry", pose: poses.at(-1), target, history, reset: true, runId: "loop-check" });
assert.equal(context.analysis.stateCycleDetected, true);
assert.equal(context.recovery.active, true);
const remembered = Object.values(context.candidates)
  .filter((item) => item.action !== "stop" && !item.collision && !item.perceivedTrackCollision)
  .slice(0, 2);
context.recovery.visited_poses.push(...remembered.map((item) => item.pose));
// Re-annotate after extending the persistent episode memory with two endpoints.
annotateCandidateRecovery(context.candidates, context.recovery, poses.at(-1));
const allLegal = Object.values(context.candidates).filter((item) => item.action !== "stop" && !item.collision && !item.perceivedTrackCollision);
const choices = Object.values(decisionCandidates(context.candidates, context.recovery));
assert.ok(choices.length > 0);
assert.ok(choices.length <= allLegal.length);
assert.ok(choices.length < allLegal.length, "known recovery endpoints should be removed when novel controls remain");
assert.ok(choices.every((item) => !item.returnsToRecentState && !item.returnsToRecoveryState && !item.undoesPreviousMove)
  || choices.every((item) => Number(item.recoveryObjectiveGain) > 0.015)
  || choices.length === allLegal.length,
"recovery exposes novel controls, objectively improving aspiration controls, or every safe fallback");

// If all legal endpoints are tabu, the shared aspiration rule admits every
// objectively improving rollout and no regressing one. It does not pick a
// winner, and is therefore identical for Jev and LLM.
const synthetic = {
  improve_a: { action: "improve_a", clearance: 0.4, collision: false, returnsToRecentState: true, recoveryObjectiveGain: 0.2, control: { travelDistance: 0.4 } },
  improve_b: { action: "improve_b", clearance: 0.4, collision: false, returnsToRecoveryState: true, recoveryObjectiveGain: 0.08, control: { travelDistance: 0.4 } },
  regress: { action: "regress", clearance: 0.4, collision: false, undoesPreviousMove: true, recoveryObjectiveGain: -0.1, control: { travelDistance: 0.4 } },
};
assert.deepEqual(Object.keys(decisionCandidates(synthetic, { active: true })), ["improve_a", "improve_b"]);
const terminalChoices = decisionCandidates({
  finish_a: { action: "finish_a", clearance: 0.4, collision: false, returnsToRecentState: true, parkedAfter: true },
  finish_b: { action: "finish_b", clearance: 0.4, collision: false, returnsToRecoveryState: true, parkedAfter: true },
  wander: { action: "wander", clearance: 0.4, collision: false, recoveryObjectiveGain: 0.2 },
}, { active: true });
assert.deepEqual(Object.keys(terminalChoices), ["finish_a", "finish_b"],
  "recovery must expose every safe terminal trajectory and no non-terminal trajectory");

const antiDither = decisionCandidates({
  micro: { action: "micro", clearance: 0.4, collision: false, pose: { x: 0.01, y: 0, heading: 1 }, path: [{ x: 0, y: 0, heading: 0 }], control: { travelDistance: 0.05 } },
  short: { action: "short", clearance: 0.4, collision: false, pose: { x: 0.2, y: 0, heading: 14 }, path: [{ x: 0, y: 0, heading: 0 }], control: { travelDistance: 0.3 } },
  escape_left: { action: "escape_left", clearance: 0.4, collision: false, pose: { x: 0.3, y: 0.3, heading: 15 }, path: [{ x: 0, y: 0, heading: 0 }], control: { travelDistance: 0.45 } },
  escape_right: { action: "escape_right", clearance: 0.4, collision: false, pose: { x: -0.3, y: 0.3, heading: -15 }, path: [{ x: 0, y: 0, heading: 0 }], control: { travelDistance: 0.45 } },
}, { active: true });
assert.deepEqual(Object.keys(antiDither), ["escape_left", "escape_right"],
  "recovery must retain every effective novel maneuver while removing tiny dithering");
console.log("recovery-loop-integration-check: passed", JSON.stringify({ legal: allLegal.length, admitted: choices.length, reasons: context.recovery.reasons }));

import assert from "node:assert/strict";
import { analyzeHistory, annotateCandidateHistory, annotateCandidateRecovery, decisionCandidates } from "../parking/policy.js";

const entry = (x, y, heading, gear = "forward", distance = 3) => ({
  pose: { x, y, heading }, control: { gear }, distance, navigationDistance: distance,
});

const twoStateLoop = [
  entry(0, 0, 0, "forward"), entry(0.42, 0.02, 4, "reverse"),
  entry(0.03, 0.01, 1, "forward"), entry(0.4, 0.03, 5, "reverse"),
];
const loopAnalysis = analyzeHistory(twoStateLoop);
assert.equal(loopAnalysis.stateCycleDetected, true);
assert.equal(loopAnalysis.stateCycleLength, 2);
assert.equal(loopAnalysis.recoveryActive, true);

const sameStatesDifferentGears = [
  entry(0, 0, 0, "forward"), entry(0.42, 0, 4, "forward"),
  entry(0.02, 0, 1, "reverse"), entry(0.4, 0.01, 5, "reverse"),
];
assert.equal(analyzeHistory(sameStatesDifferentGears).stateCycleDetected, true);

const progressing = [0, 0.3, 0.6, 0.9, 1.2, 1.5].map((x, index) =>
  entry(x, 0, index * 2, "forward", 4 - x));
assert.equal(analyzeHistory(progressing).stateCycleDetected, false);
assert.equal(analyzeHistory(progressing).oscillating, false);

const candidates = {
  back: { action: "back", pose: { x: 0.03, y: 0.01, heading: 1 } },
  novel: { action: "novel", pose: { x: 1.1, y: 0.5, heading: 30 } },
  stop: { action: "stop", pose: { x: 0, y: 0, heading: 0 } },
};
annotateCandidateHistory(candidates, twoStateLoop, twoStateLoop.at(-1).pose);
assert.equal(candidates.back.returnsToRecentState, true);
assert.equal(candidates.back.undoesPreviousMove, true);
assert.equal(candidates.novel.returnsToRecentState, false);
for (const candidate of Object.values(candidates)) {
  candidate.clearance = 1; candidate.startClearance = 1; candidate.collision = false;
}
assert.deepEqual(Object.keys(decisionCandidates(candidates, { active: true })), ["novel"]);
assert.deepEqual(Object.keys(decisionCandidates(candidates, { active: false })), ["back", "novel"]);

annotateCandidateRecovery(candidates, {
  active: true, anchor_pose: { x: 0, y: 0, heading: 0 },
  visited_poses: [{ x: 0, y: 0, heading: 0 }, { x: 1.08, y: 0.5, heading: 30 }],
  baseline_objective: 3, final_stage: false,
}, { x: 0.1, y: 0, heading: 2 });
assert.equal(candidates.back.exitsRecoveryBasin, false);
assert.equal(candidates.novel.exitsRecoveryBasin, true);
assert.equal(candidates.novel.returnsToRecoveryState, true);
assert.ok(candidates.novel.recoveryEscapeGainM > candidates.back.recoveryEscapeGainM);
assert.deepEqual(Object.keys(decisionCandidates(candidates, { active: true })), ["back", "novel"], "tabu relaxes only when every legal control revisits recovery memory");

console.log("history-loop-check: passed");

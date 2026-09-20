import assert from "node:assert/strict";
import { isParked, parkingAssessment, PARKING_BAY, PARKING_RULE, VEHICLE } from "../public/parking-goal.js";
import { isParked as serverIsParked } from "../parking/policy.js";
import { ParkingSessionStore } from "../parking/session-store.js";
import { prepareRequest } from "../parking/jev-request.js";
import { stopAvailability } from "../parking/policy.js";

assert.equal(isParked, serverIsParked, "browser and server must use the same predicate");
const target = { x: 0, y: -2.45, heading: 90 };
assert.ok(isParked({ ...target, x: 0.4 }, target), "off-centre but safely contained is complete");
assert.ok(isParked({ ...target, heading: 98 }, target), "8-degree alignment no longer causes needless corrections");
assert.ok(!isParked({ ...target, y: -2.20 }, target), "centre is close but bumper crosses the line");
assert.ok(!isParked({ ...target, x: 0.58 }, target), "body inside paint must still leave the margin");
assert.ok(!isParked({ ...target, heading: 103 }, target), "excessive heading error is incomplete");
assert.ok(!isParked({ ...target, heading: -90 }, target), "wrong required facing direction is incomplete");
assert.ok(!isParked({ ...target, x: NaN }, target));
const limit = (PARKING_BAY.length - PARKING_BAY.lineWidth - VEHICLE.length) / 2 - PARKING_RULE.edgeMarginM;
assert.ok(isParked({ ...target, y: target.y + limit }, target));
assert.ok(!isParked({ ...target, y: target.y + limit + 0.001 }, target));

// Independent four-corner oracle across rotated bays, offsets and headings.
for (const heading of [-179, -90, 0, 90, 179]) {
  const bay = { x: 1.3, y: -0.7, heading };
  for (const x of [-0.6, -0.3, 0, 0.3, 0.6]) for (const y of [-0.25, 0, 0.25]) {
    for (const delta of [-13, -8, 0, 8, 13]) {
      const r = heading * Math.PI / 180;
      const pose = { x: bay.x + x * Math.cos(r) - y * Math.sin(r), y: bay.y + x * Math.sin(r) + y * Math.cos(r), heading: heading + delta };
      const a = pose.heading * Math.PI / 180;
      const corners = [-1, 1].flatMap(f => [-1, 1].map(s => ({
        x: pose.x + f * VEHICLE.length / 2 * Math.cos(a) - s * VEHICLE.width / 2 * Math.sin(a),
        y: pose.y + f * VEHICLE.length / 2 * Math.sin(a) + s * VEHICLE.width / 2 * Math.cos(a),
      })));
      const inside = corners.every(c => {
        const dx = c.x - bay.x, dy = c.y - bay.y;
        return Math.abs(dx * Math.cos(r) + dy * Math.sin(r)) <= (PARKING_BAY.length - PARKING_BAY.lineWidth) / 2 - PARKING_RULE.edgeMarginM
          && Math.abs(-dx * Math.sin(r) + dy * Math.cos(r)) <= (PARKING_BAY.width - PARKING_BAY.lineWidth) / 2 - PARKING_RULE.edgeMarginM;
      });
      assert.equal(isParked(pose, bay), inside && Math.abs(delta) <= PARKING_RULE.headingDeg);
    }
  }
}

const store = new ParkingSessionStore();
for (const engine of ["jev", "llm"]) {
  const pose = { ...target, x: 0.4 };
  const context = store.perceive({ engine, scenario: "reverse-entry", pose, target, history: [], reset: true });
  const prepared = prepareRequest({ ...context, scenario: "reverse-entry", pose, target, history: [], stop: stopAvailability(context.candidates, pose, target) });
  assert.equal(prepared.fixed.motion.choice, "stop", "already parked must hold without another model call");
  assert.deepEqual(prepared.eligible, {});
  assert.ok(parkingAssessment(pose, target).toleranceRatio <= 1);
}
console.log("parking-goal-check: passed (geometry boundaries, rotated bays, shared completion and hold)");

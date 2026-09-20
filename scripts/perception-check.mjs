import {
  createPerception, perceive, perceivedWorld, detectedSummary, actualCollision,
  worldAt, occupiedCellList, groundTruthClearanceAt, SENSOR, setWorldProvider,
} from "../perception.js";
import { distance } from "../parking-decision.js";

const failures = [];
function check(name, condition, detail) {
  console.log((condition ? "PASS  " : "FAIL  ") + name + (detail === undefined ? "" : "  " + JSON.stringify(detail)));
  if (!condition) failures.push(name);
}

// --- Phase 1: static world, drive the aisle -------------------------------
const sweep = [
  { x: 2.65, y: 0.25, heading: 180 }, { x: 1.6, y: 0.3, heading: 180 },
  { x: 0.6, y: 0.4, heading: 90 }, { x: 0.1, y: -0.5, heading: 90 },
  { x: 0.0, y: -1.4, heading: 90 }, { x: 0.0, y: -2.2, heading: 90 },
  { x: 0.1, y: -1.2, heading: 270 }, { x: 0.3, y: 0.4, heading: 0 },
  { x: 1.4, y: 0.5, heading: 0 }, { x: 2.4, y: 0.4, heading: 180 },
  { x: 1.2, y: -0.6, heading: 180 }, { x: 0.2, y: -1.6, heading: 90 },
];
const staticState = createPerception();
let readings = 0;
for (const pose of sweep) readings += perceive(staticState, pose, 1.1);
const summary = detectedSummary(staticState);
const vehicles = summary.tracks.filter((track) => track.type === "vehicle");
check("world contains only the two parked cars", worldAt(0).length === 2, worldAt(0).map((o) => o.id));
check("both parked cars detected as vehicles", vehicles.length === 2, vehicles.map((v) => [v.id, v.x, v.y]));
for (const parked of worldAt(staticState.timeS)) {
  const match = vehicles.map((vehicle) => ({ vehicle, gap: distance(vehicle, parked) })).sort((a, b) => a.gap - b.gap)[0];
  check("detected " + parked.id + " within 0.25 m", match && match.gap < 0.25, match ? { gap: Math.round(match.gap * 1000) / 1000 } : null);
}
const world = perceivedWorld(staticState);
const probe = { x: -0.2, y: -0.9, heading: 90 };
const perceived = world.clearanceAt(probe);
const truth = groundTruthClearanceAt(probe, staticState.timeS);
check("perceived clearance tracks ground truth within 0.25 m", Math.abs(perceived - truth) < 0.25,
  { perceived: Math.round(perceived * 1000) / 1000, truth: Math.round(truth * 1000) / 1000 });
check("unknownFraction is bounded", world.unknownFractionAt(probe) >= 0 && world.unknownFractionAt(probe) <= 1, world.unknownFractionAt(probe));
check("no track is reported as moving in a static world", summary.tracks.every((track) => Math.hypot(...track.velocityMps) < 0.2));

// Regression: a partial view used to split a parked car into wheel-sized
// clusters. The rollout could then pass through the unobserved half even
// though instance-level range returns all belonged to the same vehicle.
const fragmentedState = createPerception();
const fragmentedPose = { x: 1.77, y: -0.42, heading: 90.32 };
for (let step = 0; step < 5; step += 1) perceive(fragmentedState, fragmentedPose, step ? 0.35 : 0);
const fragmentedSummary = detectedSummary(fragmentedState);
const fragmentedRight = fragmentedSummary.tracks.find((item) => item.id === "parked_car_right");
check("partial parked-car returns fuse into one vehicle-sized detection",
  fragmentedRight && Math.max(fragmentedRight.lengthM, fragmentedRight.widthM) >= 0.85
    && Math.min(fragmentedRight.lengthM, fragmentedRight.widthM) >= 0.5,
  fragmentedRight ? { length: fragmentedRight.lengthM, width: fragmentedRight.widthM, x: fragmentedRight.x, y: fragmentedRight.y } : null);
const throughRightCar = [];
for (let index = 0; index <= 16; index += 1) throughRightCar.push({ x: 1.77, y: -0.42 - index * 0.1, heading: 90 });
check("perceived rollout rejects a path into the detected parked car",
  throughRightCar.some((pose) => perceivedWorld(fragmentedState).clearanceAt(pose) < 0),
  { minimum: Math.min(...throughRightCar.map((pose) => perceivedWorld(fragmentedState).clearanceAt(pose))) });

const bayPath = [];
for (let index = 0; index <= 10; index += 1) bayPath.push({ x: 0.0, y: -0.4 - index * 0.22, heading: 270 });
check("ground truth clears the empty bay", actualCollision({ x: 0.0, y: -0.4, heading: 270 }, bayPath, 2.2, staticState.timeS) === null);
const crashPath = [];
for (let index = 0; index <= 14; index += 1) crashPath.push({ x: 1.0 + index * 0.0813, y: -index * 0.1828, heading: -65.8 });
check("ground truth flags a real collision into a parked car", Boolean(actualCollision({ x: 1.0, y: 0.0, heading: -65.8 }, crashPath, 2.8, staticState.timeS)));
const crash = actualCollision({ x: 1.0, y: 0.0, heading: -65.8 }, crashPath, 2.8, staticState.timeS);
check("collision reports contact and requires a pre-motion stop",
  Number.isInteger(crash?.pathIndex) && crash.safePathIndex === 0,
  crash);
check("pre-motion emergency stop prevents rendered overlap",
  actualCollision({ x: 1.0, y: 0.0, heading: -65.8 }, [crashPath[0]], 0, staticState.timeS) === null,
  { renderedSamples: 1, contactSample: crash.pathIndex });
const tunnellingPath = [crashPath[0], crashPath.at(-1)];
check("swept collision check cannot tunnel between sparse path samples",
  Boolean(actualCollision(tunnellingPath[0], tunnellingPath, 2.8, staticState.timeS)));

// Regression for the right-side-car contact shown in the operator screenshot.
// The boxes still have a visible 4 cm gap, but that is inside the experiment's
// 8 cm safety margin and therefore must be stopped before animation.
const rightCarNearMiss = [
  { x: 1.24, y: -0.75, heading: 90 },
  { x: 1.24, y: -1.15, heading: 90 },
  { x: 1.24, y: -1.55, heading: 90 },
];
const nearMiss = actualCollision(rightCarNearMiss[0], rightCarNearMiss, 1.2, staticState.timeS);
check("right parked-car safety margin stops motion before visual contact",
  nearMiss?.objectId === "parked_car_right" && nearMiss.clearanceM >= 0 && nearMiss.clearanceM < 0.08,
  nearMiss);

check("sensor budget stays cheap", readings < sweep.length * (SENSOR.ring.rays + SENSOR.ultrasonic.count), { readings });
check("occupancy map is not empty", occupiedCellList(staticState).length > 100, occupiedCellList(staticState).length);

// --- Phase 2: inject a mover, confirm tracking and prediction -------------
// Nothing in the demo declares this object; it exists only to exercise the
// tracker and the forecast, the same way a real detector would feed it.
const mover = { id: "test_mover", type: "pedestrian", length: 0.5, width: 0.5 };
const from = { x: 3.7, y: 2.9 }, to = { x: 0.5, y: -0.3 }, speed = 0.85, startsAt = 6;
setWorldProvider((timeS) => {
  const dx = to.x - from.x, dy = to.y - from.y;
  const span = Math.hypot(dx, dy);
  const travelled = (timeS - startsAt) * speed;
  const u = Math.max(0, Math.min(1, travelled / span));
  const heading = Math.atan2(dy, dx) * 180 / Math.PI;
  const moving = travelled > 0 && travelled < span;
  return [{
    ...mover, heading,
    x: from.x + dx * u, y: from.y + dy * u,
    vx: moving ? Math.cos(heading * Math.PI / 180) * speed : 0,
    vy: moving ? Math.sin(heading * Math.PI / 180) * speed : 0,
  }];
});
const movingState = createPerception();
const sightings = [];
for (let step = 0; step < 13; step += 1) {
  perceive(movingState, { x: -0.9, y: -0.6, heading: 90 }, 1.1);
  const truthNow = worldAt(movingState.timeS).find((object) => object.id === "test_mover");
  const seen = detectedSummary(movingState).tracks.map((track) => ({ track, gap: distance(track, truthNow) })).sort((a, b) => a.gap - b.gap)[0];
  sightings.push({ gap: seen ? Math.round(seen.gap * 1000) / 1000 : null, speed: seen ? Math.round(Math.hypot(...seen.track.velocityMps) * 100) / 100 : null });
}
check("injected mover tracked on most sightings", sightings.filter((entry) => entry.gap !== null && entry.gap < 0.6).length >= 8, sightings.filter((e) => e.gap !== null).length);
const peakSpeed = Math.max(...sightings.map((entry) => entry.speed || 0));
check("mover velocity reaches the true 0.85 m/s", Math.abs(peakSpeed - 0.85) < 0.25, { peakSpeed });

const predictedState = createPerception();
for (let step = 0; step < 10; step += 1) perceive(predictedState, { x: -0.9, y: -0.6, heading: 90 }, 1.1);
const nowTruth = worldAt(predictedState.timeS).find((object) => object.id === "test_mover");
const laterTruth = worldAt(predictedState.timeS + 1).find((object) => object.id === "test_mover");
const track = detectedSummary(predictedState).tracks.map((item) => ({ item, gap: distance(item, nowTruth) })).sort((a, b) => a.gap - b.gap)[0];
if (track) {
  const predicted = track.item.predicted.find((item) => item.inS === 1);
  check("+1 s prediction is closer than standing still",
    Boolean(predicted) && distance(predicted, laterTruth) < distance(track.item, laterTruth),
    predicted ? { predictedGap: Math.round(distance(predicted, laterTruth) * 1000) / 1000, holdGap: Math.round(distance(track.item, laterTruth) * 1000) / 1000 } : null);
} else {
  check("+1 s prediction is closer than standing still", false, "no track at the prediction step");
}
setWorldProvider(null);

console.log(failures.length ? "FAILURES: " + JSON.stringify(failures) : "ALL PERCEPTION CHECKS PASSED");
process.exitCode = failures.length ? 1 : 0;

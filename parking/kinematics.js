import {
  CONTROL_HORIZON_S, LOT_BOUNDS, MIN_CONTROL_SPEED_MPS, OBSTACLES, SAFETY_MARGIN, SPEED_OPTIONS, TRAJECTORY, VEHICLE,
} from "./config.js";
import { clamp, distance, angleError, normalizeAngle, round, roundPose } from "./math.js";

function rectangleCorners(rect) {
  const angle = rect.heading * Math.PI / 180;
  const forward = { x: Math.cos(angle), y: Math.sin(angle) };
  const side = { x: -Math.sin(angle), y: Math.cos(angle) };
  const halfLength = rect.length / 2, halfWidth = rect.width / 2;
  return [[halfLength, halfWidth], [halfLength, -halfWidth], [-halfLength, halfWidth], [-halfLength, -halfWidth]]
    .map(([a, b]) => ({ x: rect.x + forward.x * a + side.x * b, y: rect.y + forward.y * a + side.y * b }));
}

function rectangleSeparation(a, b) {
  const ac = rectangleCorners(a), bc = rectangleCorners(b);
  const axes = [a.heading, a.heading + 90, b.heading, b.heading + 90]
    .map((degrees) => ({ x: Math.cos(degrees * Math.PI / 180), y: Math.sin(degrees * Math.PI / 180) }));
  let largestGap = -Infinity;
  for (const axis of axes) {
    const ap = ac.map((point) => point.x * axis.x + point.y * axis.y);
    const bp = bc.map((point) => point.x * axis.x + point.y * axis.y);
    largestGap = Math.max(largestGap, Math.max(Math.min(...bp) - Math.max(...ap), Math.min(...ap) - Math.max(...bp)));
  }
  return largestGap;
}

export function clearanceAt(pose) {
  const active = { ...pose, length: VEHICLE.length, width: VEHICLE.width };
  const corners = rectangleCorners(active);
  let clearance = Math.min(...corners.map((point) => Math.min(
    point.x - LOT_BOUNDS.minX, LOT_BOUNDS.maxX - point.x,
    point.y - LOT_BOUNDS.minY, LOT_BOUNDS.maxY - point.y,
  )));
  for (const obstacle of OBSTACLES) clearance = Math.min(clearance, rectangleSeparation(active, obstacle));
  return clamp(clearance, -0.75, 3);
}

export function groundTruthWorld() {
  return { kind: "ground-truth", clearanceAt(pose) { return clearanceAt(pose); } };
}

const DEFAULT_WORLD = groundTruthWorld();
function worldOf(world) { return world || DEFAULT_WORLD; }

// A candidate is one bicycle-model rollout. The model chooses among rollouts;
// this module only computes motion and body clearance for a chosen control.
export function simulateControl(action, pose, target, direction, steerDeg, targetSpeed, duration = CONTROL_HORIZON_S, world = null) {
  const speedMagnitude = clamp(targetSpeed, MIN_CONTROL_SPEED_MPS, SPEED_OPTIONS.at(-1));
  const speed = direction * speedMagnitude;
  const travelDistance = speedMagnitude * duration;
  const steps = Math.max(3, Math.ceil(duration / 0.035));
  const dt = duration / steps;
  const steer = steerDeg * Math.PI / 180;
  let next = { ...pose };
  const path = [roundPose(next)];
  const activeWorld = worldOf(world);
  let clearance = activeWorld.clearanceAt(next);
  const startClearance = clearance;
  let unknownFraction = activeWorld.unknownFractionAt ? activeWorld.unknownFractionAt(next) : undefined;
  for (let index = 0; index < steps; index += 1) {
    const theta = next.heading * Math.PI / 180;
    next = {
      x: next.x + speed * Math.cos(theta) * dt,
      y: next.y + speed * Math.sin(theta) * dt,
      heading: normalizeAngle(next.heading + (speed / VEHICLE.wheelbase) * Math.tan(steer) * dt * 180 / Math.PI),
    };
    clearance = Math.min(clearance, activeWorld.clearanceAt(next));
    if (activeWorld.unknownFractionAt) unknownFraction = Math.max(unknownFraction, activeWorld.unknownFractionAt(next));
    path.push(roundPose(next));
  }
  const gear = direction < 0 ? "reverse" : "forward";
  return {
    action,
    label: `${gear === "reverse" ? "Reverse" : "Forward"}, steer ${steerDeg} degrees at ${speedMagnitude.toFixed(2)} m/s for ${duration.toFixed(2)} seconds`,
    pose: roundPose(next), path, distance: distance(next, target), angleError: angleError(next.heading, target.heading),
    clearance, startClearance, collision: clearance < 0, eligible: clearance >= SAFETY_MARGIN, unknownFraction,
    horizonPath: path, horizonDuration: round(duration), horizonClearance: clearance, horizonCollision: clearance < 0,
    control: {
      gear, direction, steerDeg, targetSpeed: speedMagnitude, validityS: round(duration),
      travelDistance: round(travelDistance), speed, duration: round(duration),
    },
  };
}

export function simulateCommand(action, pose, target, direction, steerDeg, travelDistance, world = null) {
  const speedMagnitude = clamp(0.16 + travelDistance * 0.65, 0.16, SPEED_OPTIONS.at(-1));
  return simulateControl(action, pose, target, direction, steerDeg, speedMagnitude, travelDistance / speedMagnitude, world);
}

// Roll a complete trajectory through the same bicycle model used by the
// one-control compatibility API. Steering moves toward each segment command at
// a physical rate limit, so segment boundaries become continuous curves rather
// than instantaneous direction changes. The returned path is both what the
// models inspect and what the browser executes.
export function simulateTrajectory(action, pose, target, segments, world = null) {
  const activeWorld = worldOf(world);
  let next = { ...pose };
  let actualSteerDeg = Number(segments[0]?.steerDeg) || 0;
  let clearance = activeWorld.clearanceAt(next);
  const startClearance = clearance;
  let unknownFraction = activeWorld.unknownFractionAt ? activeWorld.unknownFractionAt(next) : undefined;
  const path = [roundPose(next)];
  const details = [];
  let totalDuration = 0;
  let totalDistance = 0;

  for (const requested of segments) {
    const direction = requested.direction < 0 ? -1 : 1;
    const targetSpeed = clamp(Number(requested.targetSpeed), MIN_CONTROL_SPEED_MPS, SPEED_OPTIONS.at(-1));
    const duration = Math.max(0.2, Number(requested.duration) || CONTROL_HORIZON_S);
    const targetSteerDeg = clamp(Number(requested.steerDeg) || 0, -VEHICLE.maxSteerDeg, VEHICLE.maxSteerDeg);
    const steps = Math.max(4, Math.ceil(duration / 0.035));
    const dt = duration / steps;
    const startIndex = path.length - 1;
    const startSteerDeg = actualSteerDeg;
    for (let index = 0; index < steps; index += 1) {
      const maxStep = TRAJECTORY.steeringRateDegPerS * dt;
      actualSteerDeg += clamp(targetSteerDeg - actualSteerDeg, -maxStep, maxStep);
      const speed = direction * targetSpeed;
      const theta = next.heading * Math.PI / 180;
      const steer = actualSteerDeg * Math.PI / 180;
      next = {
        x: next.x + speed * Math.cos(theta) * dt,
        y: next.y + speed * Math.sin(theta) * dt,
        heading: normalizeAngle(next.heading + (speed / VEHICLE.wheelbase) * Math.tan(steer) * dt * 180 / Math.PI),
      };
      clearance = Math.min(clearance, activeWorld.clearanceAt(next));
      if (activeWorld.unknownFractionAt) unknownFraction = Math.max(unknownFraction, activeWorld.unknownFractionAt(next));
      path.push(roundPose(next));
    }
    const travelDistance = targetSpeed * duration;
    details.push({
      direction, steerDeg: round(targetSteerDeg), startSteerDeg: round(startSteerDeg), endSteerDeg: round(actualSteerDeg),
      targetSpeed: round(targetSpeed), duration: round(duration), travelDistance: round(travelDistance),
      startPathIndex: startIndex, endPathIndex: path.length - 1,
    });
    totalDuration += duration;
    totalDistance += travelDistance;
  }

  const first = details[0] || { direction: 0, steerDeg: 0, targetSpeed: 0 };
  const gear = first.direction < 0 ? "reverse" : "forward";
  return {
    action, label: `${details.length}-segment ${gear} trajectory`, pose: roundPose(next), path,
    distance: distance(next, target), angleError: angleError(next.heading, target.heading),
    clearance, startClearance, collision: clearance < 0, eligible: clearance >= SAFETY_MARGIN, unknownFraction,
    horizonPath: path, horizonDuration: round(totalDuration), horizonClearance: clearance, horizonCollision: clearance < 0,
    segments: details.length, segmentsDetail: details,
    control: {
      gear, direction: first.direction, steerDeg: first.steerDeg, targetSpeed: first.targetSpeed,
      validityS: round(totalDuration), travelDistance: round(totalDistance),
      speed: first.direction * first.targetSpeed, duration: round(totalDuration),
      steeringProfile: details.map((item) => ({
        direction: item.direction, steerDeg: item.steerDeg, targetSpeed: item.targetSpeed, duration: item.duration,
      })),
    },
  };
}

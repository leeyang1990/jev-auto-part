import {
  CONTROL_HORIZON_S, FINE_CONTROL_HORIZON_S, MICRO_CONTROL_HORIZON_S,
  FINE_SPEED_OPTIONS, RECOVERY_CONTROL_HORIZON_S, SPEED_OPTIONS, STEERING_OPTIONS, TRAJECTORY,
} from "./config.js";
import { angleError, distance } from "./math.js";
import { navigationMeasurement } from "./navigation.js";
import { isParked, parkingAssessment } from "../public/parking-goal.js";
import { simulateTrajectory } from "./kinematics.js";

// Build one-step, model-selectable controls. Both engines receive this exact
// same lattice; the controller measures candidates but never picks one.
export function createControlCandidates({ pose, target, world, navigation, recovery = null, simulate = simulateTrajectory }) {
  const rollouts = controlsFor(pose, target, navigation, recovery).map((segment) => {
    const candidate = simulate("control", pose, target, [segment], world);
    const measurement = navigationMeasurement(navigation, candidate.pose);
    candidate.parkingToleranceRatio = parkingAssessment(candidate.pose, target).toleranceRatio;
    return { ...candidate, parkedAfter: isParked(candidate.pose, target), searchCost: searchCost(candidate, measurement, navigation) };
  });
  const candidates = Object.fromEntries(diverseCandidates(rollouts, TRAJECTORY.candidateLimit, recovery).map((candidate, index) => {
    const action = `control_${String(index).padStart(2, "0")}`;
    candidate.action = action;
    return [action, candidate];
  }));
  const parked = isParked(pose, target);
  candidates.stop = {
    action: "stop", label: "Stop and hold position", pose: { ...pose }, path: [{ ...pose }],
    distance: distance(pose, target), angleError: angleError(pose.heading, target.heading),
    clearance: world.clearanceAt(pose), collision: false, eligible: true, segments: 0,
    segmentsDetail: [], plan: "P", parked,
    control: { gear: "park", direction: 0, steerDeg: 0, targetSpeed: 0, validityS: 0, travelDistance: 0, speed: 0, duration: 0, steeringProfile: [] },
  };
  return candidates;
}

function controlsFor(pose, target, navigation, recovery) {
  const remaining = distance(pose, target);
  const finalStage = Boolean(navigation?.finalStage);
  const speeds = finalStage && remaining < 0.55
    ? [...FINE_SPEED_OPTIONS, ...SPEED_OPTIONS]
    : remaining > 1.2 ? SPEED_OPTIONS.slice(1) : SPEED_OPTIONS;
  const controls = [];
  for (const direction of [-1, 1]) for (const steerDeg of STEERING_OPTIONS) for (const targetSpeed of speeds) {
    for (const duration of controlDurations(finalStage, remaining, targetSpeed, recovery)) {
      controls.push({ direction, steerDeg, targetSpeed, duration });
    }
  }
  return controls;
}

function diverseCandidates(candidates, limit, recovery) {
  const ranked = candidates.slice().sort((a, b) => a.searchCost - b.searchCost);
  const picked = [], keys = new Set();
  const terminal = ranked.find((candidate) => candidate.parkedAfter);
  if (terminal) add(terminal);
  for (const direction of [-1, 1]) {
    const directional = ranked.filter((item) => item.control.direction === direction);
    for (const steerDeg of STEERING_OPTIONS) add(directional.find((item) => item.control.steerDeg === steerDeg));
  }
  if (recovery?.active) {
    for (const direction of [-1, 1]) for (const turn of [-1, 1]) {
      add(ranked.find((item) => item.control.direction === direction
        && Math.sign(item.control.steerDeg) === turn
        && item.control.travelDistance >= TRAJECTORY.recoveryMinDistanceM));
    }
  }
  for (const item of ranked) add(item);
  return picked.slice(0, limit);

  function add(item) {
    if (!item || picked.length >= limit) return;
    const key = [item.control.direction, item.control.steerDeg, item.control.targetSpeed, item.control.duration].join(":");
    if (keys.has(key)) return;
    keys.add(key);
    picked.push(item);
  }
}

function searchCost(candidate, measurement, navigation) {
  const objective = navigation?.finalStage
    ? candidate.parkingToleranceRatio
    : measurement.remainingM + Math.abs(measurement.lateralErrorM) * 0.7 + measurement.headingErrorDeg * 0.012;
  const unsafe = candidate.collision ? 1000 : Math.max(0, 0.2 - candidate.clearance) * 8;
  return objective + unsafe;
}

function controlDurations(finalStage, remaining, targetSpeed, recovery) {
  if (!finalStage || remaining >= 0.85) return [CONTROL_HORIZON_S];
  if (remaining >= 0.55) return [FINE_CONTROL_HORIZON_S, CONTROL_HORIZON_S];
  if (recovery?.active) {
    const minimumDuration = TRAJECTORY.recoveryMinDistanceM / targetSpeed;
    return [...new Set([
      Math.max(RECOVERY_CONTROL_HORIZON_S, minimumDuration),
      Math.max(CONTROL_HORIZON_S, minimumDuration),
    ])];
  }
  const desiredTravelM = Math.min(0.32, Math.max(0.08, remaining - 0.126));
  const adaptive = Math.max(MICRO_CONTROL_HORIZON_S, Math.min(CONTROL_HORIZON_S, desiredTravelM / targetSpeed));
  return targetSpeed <= Math.max(...FINE_SPEED_OPTIONS)
    ? [...new Set([MICRO_CONTROL_HORIZON_S, FINE_CONTROL_HORIZON_S, adaptive, CONTROL_HORIZON_S])]
    : [...new Set([FINE_CONTROL_HORIZON_S, adaptive, CONTROL_HORIZON_S])];
}

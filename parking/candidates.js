import { PARK_TOLERANCE, TRAJECTORY } from "./config.js";
import { createControlCandidates } from "./controller.js";
import { groundTruthWorld } from "./kinematics.js";
import { navigationMeasurement } from "./navigation.js";
import { roundPose } from "./math.js";
import { isParked, planString } from "./policy.js";

export function createCandidates(pose, target, world = null, navigation = null, recovery = null) {
  const candidates = createControlCandidates({
    pose, target, world: world || groundTruthWorld(), navigation, recovery,
  });
  for (const candidate of Object.values(candidates)) {
    candidate.pose = roundPose(candidate.pose);
    candidate.path = candidate.path.map(roundPose);
    if (Array.isArray(candidate.horizonPath)) candidate.horizonPath = candidate.path;
    addNavigationMeasurements(candidate, pose, navigation);
    if (candidate.action === "stop") continue;
    candidate.parkedAfter = isParked(candidate.pose, target);
    candidate.parkingToleranceRatio = Math.max(
      candidate.distance / PARK_TOLERANCE.distanceM,
      candidate.angleError / PARK_TOLERANCE.headingDeg,
      Math.abs(candidate.targetLateralError) / TRAJECTORY.terminalLateralToleranceM,
    );
    candidate.plan = planString(candidate.segmentsDetail);
  }
  return candidates;
}

function addNavigationMeasurements(candidate, pose, navigation) {
  if (!navigation) return candidate;
  const before = navigationMeasurement(navigation, pose);
  const after = navigationMeasurement(navigation, candidate.pose);
  candidate.navigationDistance = after.remainingM;
  candidate.navigationLateralError = after.lateralErrorM;
  candidate.navigationHeadingError = after.headingErrorDeg;
  candidate.navigationProgress = navigation.finalStage ? before.remainingM - after.remainingM : after.progressM - before.progressM;
  if (navigation.finalStage) {
    const heading = Number(navigation.goal.heading) * Math.PI / 180;
    const errorX = candidate.pose.x - navigation.goal.x;
    const errorY = candidate.pose.y - navigation.goal.y;
    candidate.targetLongitudinalError = errorX * Math.cos(heading) + errorY * Math.sin(heading);
    candidate.targetLateralError = -errorX * Math.sin(heading) + errorY * Math.cos(heading);
  }
  return candidate;
}

import { CONTROL_HORIZON_S, SAFETY_MARGIN, TRAJECTORY } from "./config.js";
import { angleError, distance, round } from "./math.js";

import { isParked, parkingAssessment } from "../public/parking-goal.js";
export { isParked };

export function movingCandidates(candidates) {
  return Object.fromEntries(Object.entries(candidates)
    .filter(([action, item]) => action !== "stop" && isLegal(item)));
}

// Recovery uses a short-term tabu set over physical states. It removes only
// controls already known to close the current loop; it does not score, rank,
// or choose among the remaining controls. If every safe control is tabu, the
// constraint is relaxed so the car is never deadlocked by stale history.
export function decisionCandidates(candidates, recovery = null) {
  const legal = movingCandidates(candidates);
  if (!recovery?.active) return legal;
  const entries = Object.entries(legal);
  const terminal = Object.fromEntries(entries.filter(([, item]) => item.parkedAfter));
  if (Object.keys(terminal).length) return terminal;
  const novel = Object.fromEntries(entries
    .filter(([, item]) => !item.returnsToRecentState
      && !item.returnsToRecoveryState
      && !item.undoesPreviousMove
      && isEffectiveRecoveryTrajectory(item)));
  if (Object.keys(novel).length) return novel;

  // If every endpoint is tabu, prefer trajectories that have already been
  // simulated to improve the episode objective. This is an industry-standard
  // aspiration rule for tabu search: it relaxes memory when a move beats the
  // recovery baseline. Both engines receive the same filtered table and still
  // choose the trajectory themselves. If no improving trajectory exists, fall
  // back to every safe choice so history can never deadlock the vehicle.
  const improving = Object.fromEntries(entries.filter(([, item]) =>
    Number.isFinite(Number(item.recoveryObjectiveGain))
      && Number(item.recoveryObjectiveGain) > 0.015));
  return Object.keys(improving).length ? improving : legal;
}

// Recovery must offer a maneuver that can actually leave the repeated basin.
// This shared admission rule removes sub-resolution dithering only after a loop
// has been observed. It does not prescribe a direction, curvature, or winner.
function isEffectiveRecoveryTrajectory(candidate) {
  // Synthetic/unit callers that predate trajectory metadata still participate
  // in tabu filtering. Runtime candidates always carry a path and control.
  if (!candidate.control && !Array.isArray(candidate.path)) return true;
  const distanceM = Number(candidate.control?.travelDistance) || 0;
  const path = candidate.path || [];
  const startHeading = Number(path[0]?.heading);
  const endHeading = Number(candidate.pose?.heading);
  const headingChange = Number.isFinite(startHeading) && Number.isFinite(endHeading)
    ? angleError(startHeading, endHeading)
    : 0;
  // A large heading change over only a few centimetres is still a dither: it
  // cannot create the lateral room a non-holonomic vehicle needs. Requiring
  // meaningful translation keeps recovery trajectories physically useful; the
  // model still chooses direction, curvature, speed and the complete sequence.
  return distanceM >= TRAJECTORY.recoveryMinDistanceM;
}

export function isLegal(candidate) {
  if (!candidate) return false;
  if (candidate.action === "stop") return true;
  if (candidate.perceivedTrackCollision || candidate.collision) return false;
  if (candidate.clearance >= SAFETY_MARGIN) return true;
  return typeof candidate.startClearance === "number" && candidate.clearance >= candidate.startClearance - 0.02;
}

export function stopAvailability(candidates, pose, target) {
  const eligible = movingCandidates(candidates);
  const reasons = [];
  if (isParked(pose, target)) reasons.push("parked");
  if (!Object.keys(eligible).length) reasons.push("no_eligible_path");
  return { available: reasons.length > 0, reasons, eligibleCount: Object.keys(eligible).length };
}

export function planString(segments) {
  return segments.map((segment) => (segment.direction < 0 ? "R" : "F")
    + signedSteer(segment.steerDeg)
    + "@" + Number(segment.targetSpeed ?? Math.abs(segment.speed) ?? 0).toFixed(2)
    + ":" + Number(segment.duration ?? CONTROL_HORIZON_S).toFixed(2) + "s").join("; " );
}

function signedSteer(steerDeg) {
  const value = Math.round(Number(steerDeg) || 0);
  return value > 0 ? "+" + value : String(value);
}

export function analyzeHistory(history, finalStage = false, target = null) {
  const entries = history.slice(-10);
  const distances = entries.map((item) => Number(item.navigationDistance ?? item.distance)).filter(Number.isFinite);
  const toleranceRatios = entries.map((item) => target && validPose(item.pose)
    ? parkingAssessment(item.pose, target).toleranceRatio : NaN).filter(Number.isFinite);
  const progressValues = finalStage && toleranceRatios.length === entries.length ? toleranceRatios : distances;
  const gears = entries.map((item) => item.control?.gear).filter((gear) => gear === "forward" || gear === "reverse");
  let gearChanges = 0;
  for (let index = 1; index < gears.length; index += 1) if (gears[index] !== gears[index - 1]) gearChanges += 1;
  const lastFour = gears.slice(-4);
  const gearOscillation = lastFour.length === 4 && lastFour[0] === lastFour[2]
    && lastFour[1] === lastFour[3] && lastFour[0] !== lastFour[1];
  const poseEntries = entries.filter((item) => validPose(item.pose));
  const stateCycle = detectStateCycle(poseEntries);
  const oscillating = gearOscillation || stateCycle.detected;
  let stagnantMoves = 0;
  if (progressValues.length > 1) {
    let rollingBest = progressValues[0];
    const meaningfulGain = finalStage ? 0.06 : 0.04;
    for (let index = 1; index < progressValues.length; index += 1) {
      if (progressValues[index] < rollingBest - meaningfulGain) { rollingBest = progressValues[index]; stagnantMoves = 0; }
      else stagnantMoves += 1;
    }
  }
  const counts = {}, blockedCounts = {};
  for (const item of entries.slice(-6)) {
    const command = item.requestedAction || item.action;
    counts[command] = (counts[command] || 0) + 1;
    if (item.blocked) blockedCounts[command] = (blockedCounts[command] || 0) + 1;
  }
  const repeatedActions = Object.keys(counts).filter((name) => counts[name] >= 2);
  const blockedMoves = entries.slice(-6).filter((item) => item.blocked).length;
  const recentDistances = entries.slice(-6).map((item) => Number(item.navigationDistance ?? item.distance)).filter(Number.isFinite);
  const recentBestDistanceM = recentDistances.length ? Math.min(...recentDistances) : null;
  const latestDistanceM = recentDistances.length ? recentDistances.at(-1) : null;
  const plateauM = recentBestDistanceM == null || latestDistanceM == null ? null : latestDistanceM - recentBestDistanceM;
  const recentRatios = toleranceRatios.slice(-6);
  const recentBestToleranceRatio = recentRatios.length ? Math.min(...recentRatios) : null;
  const recoveryActive = oscillating || stagnantMoves >= 4 || blockedMoves >= 2;
  return {
    gearChanges, gearOscillation, oscillating, stateCycleDetected: stateCycle.detected,
    stateCycleLength: stateCycle.length, stateCyclePositionErrorM: stateCycle.positionErrorM,
    stateCycleHeadingErrorDeg: stateCycle.headingErrorDeg,
    stagnantMoves, blockedMoves, repeatedActions, recoveryActive,
    recentRequestCounts: counts, recentBlockedCounts: blockedCounts,
    recoveryReasons: [
      gearOscillation ? "gear oscillation" : null,
      stateCycle.detected ? `returned to a recent pose (${stateCycle.length}-move state loop)` : null,
      stagnantMoves >= 4 ? `no progress for ${stagnantMoves} moves` : null,
      blockedMoves >= 2 ? "repeated blocked commands" : null,
    ].filter(Boolean),
    bestDistanceM: distances.length ? round(Math.min(...distances)) : null,
    recentBestDistanceM: recentBestDistanceM == null ? null : round(recentBestDistanceM),
    recentBestToleranceRatio: recentBestToleranceRatio == null ? null : round(recentBestToleranceRatio),
    latestDistanceM: latestDistanceM == null ? null : round(latestDistanceM),
    plateauM: plateauM == null ? null : round(plateauM),
  };
}

// Add memory facts to every rollout without ranking the choices. Safety
// admission is applied later and independently of these history annotations.
export function annotateCandidateHistory(candidates, history, currentPose) {
  const poses = history.filter((item) => validPose(item.pose)).map((item) => item.pose);
  while (poses.length && samePose(poses.at(-1), currentPose, 0.04, 3)) poses.pop();
  const recent = poses.slice(-8);

  for (const candidate of Object.values(candidates)) {
    if (candidate.action === "stop" || !validPose(candidate.pose)) continue;
    let nearest = null;
    recent.forEach((pose, index) => {
      const positionM = distance(candidate.pose, pose);
      const headingDeg = angleError(candidate.pose.heading, pose.heading);
      // Position is primary, with heading breaking ties between nearby poses.
      const separation = positionM + headingDeg / 90;
      if (!nearest || separation < nearest.separation) {
        nearest = { positionM, headingDeg, separation, stepsAgo: recent.length - index };
      }
    });
    candidate.nearestRecentPoseDistanceM = nearest?.positionM ?? null;
    candidate.nearestRecentPoseHeadingDeltaDeg = nearest?.headingDeg ?? null;
    candidate.nearestRecentPoseStepsAgo = nearest?.stepsAgo ?? null;
    candidate.returnsToRecentState = Boolean(nearest
      && nearest.positionM <= 0.18 && nearest.headingDeg <= 10);
    candidate.undoesPreviousMove = Boolean(nearest && nearest.stepsAgo === 1
      && nearest.positionM <= 0.18 && nearest.headingDeg <= 10);
  }
  return candidates;
}

// Recovery is an episode, not a one-frame warning. These are measurements
// against the pose where recovery began; they never remove or select a control.
export function annotateCandidateRecovery(candidates, recovery, pose) {
  const anchor = recovery?.anchor_pose;
  if (!recovery?.active || !validPose(anchor)) return candidates;
  const visited = Array.isArray(recovery.visited_poses)
    ? recovery.visited_poses.filter(validPose)
    : [anchor];
  const currentPositionM = distance(pose, anchor);
  const currentHeadingDeg = angleError(pose.heading, anchor.heading);
  for (const candidate of Object.values(candidates)) {
    if (candidate.action === "stop" || !validPose(candidate.pose)) continue;
    const positionM = distance(candidate.pose, anchor);
    const headingDeg = angleError(candidate.pose.heading, anchor.heading);
    candidate.recoveryAnchorDistanceM = positionM;
    candidate.recoveryAnchorHeadingDeltaDeg = headingDeg;
    candidate.recoveryEscapeGainM = positionM - currentPositionM;
    candidate.recoveryHeadingEscapeGainDeg = headingDeg - currentHeadingDeg;
    candidate.exitsRecoveryBasin = positionM >= 0.4 || headingDeg >= 22;
    const nearestVisited = nearestPose(candidate.pose, visited);
    candidate.nearestRecoveryPoseDistanceM = nearestVisited?.positionM ?? null;
    candidate.nearestRecoveryPoseHeadingDeltaDeg = nearestVisited?.headingDeg ?? null;
    candidate.returnsToRecoveryState = Boolean(nearestVisited
      && nearestVisited.positionM <= 0.16 && nearestVisited.headingDeg <= 10);
    const objective = recoveryObjective(candidate, recovery.final_stage);
    const baseline = Number(recovery.baseline_objective);
    candidate.recoveryObjective = objective;
    candidate.recoveryObjectiveGain = Number.isFinite(objective) && Number.isFinite(baseline)
      ? baseline - objective
      : null;
  }
  return candidates;
}

function nearestPose(pose, visited) {
  let nearest = null;
  for (const prior of visited) {
    const positionM = distance(pose, prior);
    const headingDeg = angleError(pose.heading, prior.heading);
    const separation = positionM + headingDeg / 90;
    if (!nearest || separation < nearest.separation) nearest = { positionM, headingDeg, separation };
  }
  return nearest;
}

function recoveryObjective(candidate, finalStage) {
  if (finalStage) return Number(candidate.parkingToleranceRatio);
  return Number(candidate.navigationDistance);
}

function detectStateCycle(entries) {
  // Confirm both states of a two-step loop (A,B,A,B), or all three states of
  // a three-step loop. This catches different commands that reach the same
  // physical states and avoids flagging an isolated close pass.
  for (const length of [2, 3]) {
    if (entries.length < length * 2) continue;
    const comparisons = [];
    let matches = true;
    for (let offset = 0; offset < length; offset += 1) {
      const current = entries.at(-1 - offset).pose;
      const prior = entries.at(-1 - offset - length).pose;
      const positionM = distance(current, prior);
      const headingDeg = angleError(current.heading, prior.heading);
      comparisons.push({ positionM, headingDeg });
      if (positionM > 0.22 || headingDeg > 12) matches = false;
    }
    if (matches) {
      return {
        detected: true, length,
        positionErrorM: round(Math.max(...comparisons.map((item) => item.positionM))),
        headingErrorDeg: round(Math.max(...comparisons.map((item) => item.headingDeg))),
      };
    }
  }
  return { detected: false, length: null, positionErrorM: null, headingErrorDeg: null };
}

function validPose(pose) {
  return pose && Number.isFinite(Number(pose.x)) && Number.isFinite(Number(pose.y))
    && Number.isFinite(Number(pose.heading));
}

function samePose(a, b, positionM, headingDeg) {
  return validPose(a) && validPose(b) && distance(a, b) <= positionM
    && angleError(a.heading, b.heading) <= headingDeg;
}

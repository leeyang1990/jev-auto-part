// Semantic briefing.
//
// The same facts are delivered to the two engines in the form each one is built
// for. Jev gets typed questions over a factored candidate table, which is the
// structured interface it was trained on. The language model gets this: the
// situation written out in prose, with the options listed as sentences, so that
// the decision is made in language rather than by reading a matrix.
//
// Nothing about the information changes. Same perception, same candidate set,
// same tolerances, same safety margin. Only the representation differs.

import { PARKING_BAY, PARKING_RULE } from "./public/parking-goal.js";
import { distance, angleError, round } from "./parking/math.js";
import { decisionCandidates, isParked, isLegal, planString } from "./parking/policy.js";

function metres(value) { return round(value).toFixed(2) + " m"; }
function degrees(value) { return round(value).toFixed(1) + " deg"; }

function describePose(pose) {
  const heading = ((pose.heading % 360) + 360) % 360;
  let facing;
  if (heading >= 45 && heading < 135) facing = "nose pointing into the bays (north)";
  else if (heading >= 135 && heading < 225) facing = "nose pointing left along the aisle (west)";
  else if (heading >= 225 && heading < 315) facing = "nose pointing out of the bays (south)";
  else facing = "nose pointing right along the aisle (east)";
  return "at x=" + round(pose.x).toFixed(2) + ", y=" + round(pose.y).toFixed(2) + ", " + facing + " (" + degrees(heading) + ")";
}

function describeTrack(track) {
  const [width, length] = track.size_m;
  const [vx, vy] = track.velocity_mps;
  const pieces = [
    track.type === "vehicle" ? "a vehicle" : "a person-sized object",
    "at (" + round(track.pose.x).toFixed(2) + ", " + round(track.pose.y).toFixed(2) + ")",
    "about " + metres(length) + " long by " + metres(width) + " wide",
  ];
  const speed = Math.hypot(vx, vy);
  if (speed > 0.25) {
    const bearing = Math.atan2(vy, vx) * 180 / Math.PI;
    pieces.push("moving at " + speed.toFixed(2) + " m/s toward " + degrees(bearing));
    const soon = track.predicted && track.predicted[0];
    if (soon) pieces.push("forecast in one second at (" + soon.x.toFixed(2) + ", " + soon.y.toFixed(2) + ") give or take " + metres(soon.uncertaintyM));
  } else {
    pieces.push("not moving");
  }
  const seen = Number(track.last_seen_s_ago) || 0;
  pieces.push("seen " + (seen <= 0.01 ? "just now" : metres(seen) + " ago") + ", confidence " + Number(track.confidence).toFixed(2));
  return pieces.join(", ");
}

function describeOption(ordinal, candidate, analysis, pose, target, finalStage) {
  const legal = isLegal(candidate);
  const parts = [
    String(ordinal).padStart(2, " ") + ") " + (candidate.plan || planString(candidate.segmentsDetail || [])),
    "nearest object along the way is " + metres(candidate.clearance) + " away",
    "makes " + metres(candidate.navigationProgress) + " progress on the current route stage",
    "has " + metres(candidate.navigationDistance) + " of route remaining, lateral route error " + metres(candidate.navigationLateralError) + ", and " + degrees(candidate.navigationHeadingError) + " heading error to the route",
    "covers " + metres(candidate.control?.travelDistance || 0),
  ];
  if (finalStage) {
    parts.push("ends " + metres(candidate.distance) + " from the bay centre and " + degrees(candidate.angleError) + " off its heading");
    parts.push("signed bay-axis error: " + metres(candidate.targetLongitudinalError)
      + " longitudinal and " + metres(candidate.targetLateralError) + " lateral");
    parts.push("changes target distance by " + metres(candidate.distance - distance(pose, target)));
    parts.push("changes target heading error by " + degrees(candidate.angleError - angleError(pose.heading, target.heading)));
    if (candidate.parkedAfter) parts.push("FINISHES THE TASK: inside body-containment and heading requirements");
    else if (Number.isFinite(candidate.parkingToleranceRatio)) parts.push("worst normalized parking error is " + round(candidate.parkingToleranceRatio).toFixed(2) + " times its allowed tolerance");
  }
  const requested = analysis.recentRequestCounts?.[candidate.action] || 0;
  const blocked = analysis.recentBlockedCounts?.[candidate.action] || 0;
  if (requested) parts.push("requested " + requested + " time(s) in the last six decisions");
  if (blocked) parts.push("BLOCKED " + blocked + " time(s) recently; do not repeat it from this pose");
  if (analysis.recoveryActive) {
    if (finalStage && analysis.recentBestToleranceRatio !== null && Number.isFinite(candidate.parkingToleranceRatio)) {
      const improvement = analysis.recentBestToleranceRatio - candidate.parkingToleranceRatio;
      parts.push(improvement > 0.06
        ? "improves the best combined position-and-heading error from the last six decisions by " + round(improvement).toFixed(2) + " tolerance units"
        : "does not improve the best combined position-and-heading error from the last six decisions");
    } else if (!finalStage && analysis.recentBestDistanceM !== null) {
      const improvement = analysis.recentBestDistanceM - candidate.navigationDistance;
      parts.push(improvement > 0.04
        ? "improves the best stage distance from the last six decisions by " + metres(improvement)
        : "does not improve the best stage distance from the last six decisions");
    }
    if (candidate.nearestRecentPoseDistanceM != null) {
      parts.push("nearest previously visited pose is " + metres(candidate.nearestRecentPoseDistanceM)
        + " away with " + degrees(candidate.nearestRecentPoseHeadingDeltaDeg)
        + " heading difference, visited " + candidate.nearestRecentPoseStepsAgo + " move(s) ago");
    }
    if (candidate.returnsToRecentState) parts.push("RETURNS TO A RECENT STATE");
    if (candidate.undoesPreviousMove) parts.push("UNDOES THE PREVIOUS MOVE");
    if (Number.isFinite(candidate.recoveryEscapeGainM)) parts.push("moves " + metres(candidate.recoveryEscapeGainM) + " farther from the recovery anchor");
    if (Number.isFinite(candidate.recoveryHeadingEscapeGainDeg)) parts.push("changes heading away from the recovery anchor by " + degrees(candidate.recoveryHeadingEscapeGainDeg));
    if (candidate.exitsRecoveryBasin) parts.push("EXITS THE RECOVERY BASIN");
    if (candidate.returnsToRecoveryState) parts.push("RETURNS TO A STATE VISITED DURING THIS RECOVERY");
    if (Number.isFinite(candidate.recoveryObjectiveGain)) parts.push("recovery objective improves by " + round(candidate.recoveryObjectiveGain).toFixed(2) + " normalized units");
  }
  if (typeof candidate.unknownFraction === "number" && candidate.unknownFraction > 0) parts.push(Math.round(candidate.unknownFraction * 100) + "% of it runs through space never observed");
  if (candidate.predictedConflict) parts.push("forecast to conflict with " + candidate.predictedConflict + " after " + metres(candidate.predictedConflictInS));
  parts.push("total trajectory duration " + Number(candidate.control?.validityS || candidate.control?.duration || 0).toFixed(2) + " s");
  if (!legal) parts.push("REJECTED: this one touches something");
  return parts.join("; ");
}

export function semanticBriefing({ scenario, task, pose, target, navigation, analysis, recovery, candidates, environment }) {
  // Use the same safety-admitted choice set as Jev. Unsafe exploratory paths
  // are still rendered in the UI, but neither engine may request them.
  const ordered = Object.entries(isParked(pose, target) ? {} : decisionCandidates(candidates, recovery));
  const order = ordered.map(([id]) => id);
  const bayside = distance(pose, target);

  const lines = [];
  lines.push("You are the driver of a small car parking in a lot. Choose the next immediate control.");
  lines.push("");
  lines.push("THE TASK");
  lines.push(task || "Park in the centre bay between two parked vehicles.");
  lines.push("The bay you must end up in is centred at x=" + round(target.x).toFixed(2) + ", y=" + round(target.y).toFixed(2) + " with its heading at " + degrees(target.heading) + ". Success means holding the entire vehicle inside the " + metres(PARKING_BAY.width) + " wide by " + metres(PARKING_BAY.length) + " long bay, with at least " + metres(PARKING_RULE.edgeMarginM) + " from the inner edges of its " + metres(PARKING_BAY.lineWidth) + " thick painted lines and heading error at most " + degrees(PARKING_RULE.headingDeg) + ". Exact centering is not required; stop adjusting once these conditions hold.");
  if (navigation) {
    if (navigation.finalStage) {
      lines.push("Current navigation stage " + (navigation.stageIndex + 1) + " of " + navigation.stageCount + ": " + navigation.label + ". This is the final parking stage.");
    } else {
      const routeText = navigation.route.map((point) => "(" + point.x.toFixed(2) + ", " + point.y.toFixed(2) + ")").join(" to ");
      lines.push("Current navigation stage " + (navigation.stageIndex + 1) + " of " + navigation.stageCount + ": " + navigation.label + ". The directed route corridor runs " + routeText + ". You have " + metres(navigation.remainingM) + " left along it, lateral error " + metres(navigation.lateralErrorM) + ", and heading error " + degrees(navigation.headingErrorDeg) + ". Cross its end gate before attempting final bay alignment; the route only measures progress and does not select your control.");
    }
  }
  lines.push("");
  lines.push("WHERE YOU ARE");
  lines.push("You are " + describePose(pose) + ", and the bay centre is " + metres(bayside) + " away at a bearing of " + degrees(Math.atan2(target.y - pose.y, target.x - pose.x) * 180 / Math.PI) + ". Your heading is " + degrees(angleError(pose.heading, target.heading)) + " off the bay heading. Your car is " + metres(1.75) + " long and " + metres(0.82) + " wide, and it steers through the rear axle, so it cannot move sideways.");
  lines.push("");
  lines.push("WHAT YOUR OWN SENSORS REPORT");
  if (!environment || !environment.tracked_objects || !environment.tracked_objects.length) {
    lines.push("Nothing. You have not detected any object yet. That is not the same as an empty lot: an area you have not swept is unknown, and you should prefer manoeuvres that stay in space you have already observed.");
  } else {
    lines.push("You currently track " + environment.tracked_objects.length + " object(s):");
    for (const track of environment.tracked_objects) lines.push("- " + track.id + ": " + describeTrack(track));
    lines.push("These are detections with noise and gaps, not truth. Objects can be missed, partly seen, or mis-sized, and the forecast gets less certain the further ahead you look.");
  }
  if (environment && environment.occupancy_grid) {
    lines.push("Your map currently holds " + environment.occupancy_grid.occupied_cells + " occupied cells of " + environment.occupancy_grid.cell_m + " m each. Space that is not on the map has not been observed; treating it as free is a guess.");
  }
  lines.push("");
  lines.push("WHAT YOUR LAST FEW MOVES DID");
  if (analysis.gearChanges !== undefined) {
    lines.push("Across your recent moves you changed gear direction " + analysis.gearChanges + " time(s).");
  }
  if (analysis.oscillating) lines.push("You are currently alternating direction without gaining anything.");
  if (analysis.stateCycleDetected) lines.push("Your position and heading form a repeated " + analysis.stateCycleLength + "-move state loop; changing command names alone will not escape it.");
  if (analysis.stagnantMoves >= 4) lines.push("You have made no real progress for " + analysis.stagnantMoves + " moves.");
  if (analysis.blockedMoves >= 2) lines.push("Two or more of your recent choices were refused because your own map said they would collide, so the car simply held position.");
  if (analysis.repeatedActions && analysis.repeatedActions.length) lines.push("You have repeated these choices: " + analysis.repeatedActions.join(", ") + ".");
  if (analysis.bestDistanceM !== null) lines.push("The closest you have been to the bay so far is " + metres(analysis.bestDistanceM) + ".");
  if (!analysis.gearChanges && !analysis.oscillating && !analysis.stagnantMoves && !analysis.blockedMoves) lines.push("Nothing has gone wrong yet.");
  if (recovery && recovery.active) {
    lines.push("");
    lines.push("RECOVERY NEEDED");
    const recoveryTarget = navigation?.finalStage
      ? "Prefer a safe option that improves the best combined position-and-heading error reached in the last six decisions."
      : "Prefer a safe option that beats the best stage distance reached in the last six decisions.";
    lines.push("Reasons: " + (recovery.reasons || []).join(", ") + ". This recovery episode has lasted " + (recovery.episode_decisions || 0) + " decision(s). Its anchor pose is (" + recovery.anchor_pose.x.toFixed(2) + ", " + recovery.anchor_pose.y.toFixed(2) + ", " + recovery.anchor_pose.heading.toFixed(1) + " deg), and recovery stays active until the car leaves that basin or materially improves the objective. No recovery control will be selected for you. " + recoveryTarget + " Reject options marked as returning to a recent state or undoing the previous move. If no option improves the objective, keep choosing positive escape gain until an option exits the recovery basin.");
    lines.push("Controls whose rollout returns to a recently visited physical state are temporarily absent from this choice set. This short-term tabu constraint prevents a known loop; it does not choose an escape control for you.");
  }
  lines.push("");
  lines.push("YOUR OPTIONS RIGHT NOW");
  lines.push("Each line is one immediate control that passed collision and lot-boundary safety admission. Gear, steering angle, target speed and duration are fixed by your choice. Both engines receive controls from the same shared generator. Before the final stage, prefer positive route progress while keeping lateral and heading error controlled. The displayed line is the same body-checked path that executes; a fresh decision replaces it when the control ends.");
  ordered.forEach(([, candidate], index) => lines.push(describeOption(index + 1, candidate, analysis, pose, target, navigation?.finalStage)));

  return { text: lines.join("\n"), order };
}

export function replyInstruction() {
  return [
    "ANSWER",
    "Reply with one JSON object and nothing else:",
    '{"choice": <the number of the option you pick>, "why": "<one or two sentences, in your own words, explaining the decision>", "confidence": <0 to 1>}',
    "Pick an option that does not touch anything. Stopping still is not an option unless you are already inside the bay.",
  ].join("\n");
}

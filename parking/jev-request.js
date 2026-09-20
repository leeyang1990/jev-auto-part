import { CONTROL_HORIZON_S, LOT_BOUNDS, OBSTACLES, SAFETY_MARGIN, VEHICLE } from "./config.js";
import { angleError, clamp, distance, round, roundPose } from "./math.js";
import { PARKING_BAY, PARKING_RULE } from "../public/parking-goal.js";
import { scenarioTasks } from "../scenarios.js";
import { decisionCandidates, isParked, planString } from "./policy.js";

function factorTable(columns, rows) {
  const values = Object.values(rows);
  if (values.length < 3) return { columns, rows };
  const shared = {}, varying = [];
  columns.forEach((column, index) => {
    const first = values[0][index] ?? null;
    if (values.every((row) => Object.is(row[index] ?? null, first))) shared[column] = first;
    else varying.push(index);
  });
  if (!Object.keys(shared).length) return { columns, rows };
  const project = (row) => varying.map((index) => row[index] ?? null);
  return {
    shared,
    columns: varying.map((index) => columns[index]),
    rows: Array.isArray(rows) ? rows.map(project) : Object.fromEntries(Object.entries(rows).map(([id, row]) => [id, project(row)])),
  };
}

export function prepareRequest({ scenario, pose, target, navigation, history, analysis, candidates, stop, recovery, environment = null }) {
  const recovering = Boolean(recovery?.active);
  // Match JevPilot's safety boundary: all paths remain visible in the operator
  // overlay, while only collision-free controls enter the model's choice set.
  // This layer does not score, rank, or select among the admitted controls.
  const eligible = isParked(pose, target) ? {} : decisionCandidates(candidates, recovery);
  const aliases = Object.fromEntries(Object.keys(eligible).map((id, index) => [`v${index}`, id]));
  const vectors = Object.fromEntries(Object.entries(aliases).map(([alias, original]) => [alias, candidates[original]]));

  const columnDefs = [
    ["plan", (item) => item.plan || planString(item.segmentsDetail || [])],
    ["target_speed_mps", (item) => round(item.control?.targetSpeed || 0)],
    ["validity_s", (item) => round(item.control?.validityS || item.control?.duration || 0)],
    ["trajectory_distance_m", (item) => round(item.control?.travelDistance || 0)],
    ["route_progress_m", (item) => round(item.navigationProgress)],
    ["route_remaining_m", (item) => round(item.navigationDistance)],
    ["route_error_m", (item) => round(Math.abs(item.navigationLateralError))],
    ["heading_error_deg", (item) => round(item.navigationHeadingError)],
    ["clearance_m", (item) => round(item.clearance)],
    ["recent_request_count", (item) => analysis.recentRequestCounts?.[item.action] || 0],
    ["recent_block_count", (item) => analysis.recentBlockedCounts?.[item.action] || 0],
  ];
  if (navigation?.finalStage) {
    columnDefs.push(
      ["distance_to_target_m", (item) => round(item.distance)],
      ["target_heading_error_deg", (item) => round(item.angleError)],
      ["target_longitudinal_error_m", (item) => round(item.targetLongitudinalError)],
      ["target_lateral_error_m", (item) => round(item.targetLateralError)],
      ["parked_after_command", (item) => Boolean(item.parkedAfter)],
      ["parking_tolerance_ratio", (item) => round(item.parkingToleranceRatio)],
      ["target_distance_delta_m", (item) => round(item.distance - distance(pose, target))],
      ["target_heading_error_delta_deg", (item) => round(item.angleError - angleError(pose.heading, target.heading))],
    );
  }
  if (recovering) {
    columnDefs.push(
      ["recovery_improves_recent_best", (item) => navigation?.finalStage
        ? (analysis.recentBestToleranceRatio == null ? null : round(analysis.recentBestToleranceRatio - item.parkingToleranceRatio))
        : (analysis.recentBestDistanceM == null ? null : round(analysis.recentBestDistanceM - item.navigationDistance))],
      ["recovery_escape_distance_m", (item) => round(Math.hypot(item.pose.x - pose.x, item.pose.y - pose.y))],
      ["nearest_recent_pose_distance_m", (item) => item.nearestRecentPoseDistanceM == null ? null : round(item.nearestRecentPoseDistanceM)],
      ["nearest_recent_pose_heading_delta_deg", (item) => item.nearestRecentPoseHeadingDeltaDeg == null ? null : round(item.nearestRecentPoseHeadingDeltaDeg)],
      ["nearest_recent_pose_steps_ago", (item) => item.nearestRecentPoseStepsAgo],
      ["returns_to_recent_state", (item) => Boolean(item.returnsToRecentState)],
      ["undoes_previous_move", (item) => Boolean(item.undoesPreviousMove)],
      ["returns_to_recovery_state", (item) => Boolean(item.returnsToRecoveryState)],
      ["recovery_objective_gain", (item) => round(item.recoveryObjectiveGain)],
      ["recovery_escape_gain_m", (item) => round(item.recoveryEscapeGainM)],
      ["recovery_heading_escape_gain_deg", (item) => round(item.recoveryHeadingEscapeGainDeg)],
      ["exits_recovery_basin", (item) => Boolean(item.exitsRecoveryBasin)],
    );
  }
  if (Object.values(vectors).some((item) => typeof item.unknownFraction === "number")) {
    columnDefs.push(["unknown_fraction", (item) => (typeof item.unknownFraction === "number" ? round(item.unknownFraction) : null)]);
  }
  if (Object.values(vectors).some((item) => item.predictedConflict)) {
    columnDefs.push(
      ["predicted_conflict", (item) => item.predictedConflict || null],
      ["conflict_in_s", (item) => (item.predictedConflictInS == null ? null : round(item.predictedConflictInS))],
    );
  }
  if (Object.values(vectors).some((item) => item.perceivedTrackCollision)) {
    columnDefs.push(
      ["command_track_collision", (item) => Boolean(item.perceivedTrackCollision)],
      ["command_collision_with", (item) => item.perceivedTrackCollisionWith || null],
    );
  }
  if (Object.values(vectors).some((item) => typeof item.horizonClearance === "number")) {
    columnDefs.push(
      ["horizon_clearance_m", (item) => (typeof item.horizonClearance === "number" ? round(item.horizonClearance) : null)],
      ["horizon_collision", (item) => Boolean(item.horizonCollision)],
      ["horizon_safe_distance_m", (item) => (typeof item.horizonSafeDistance === "number" ? round(item.horizonSafeDistance) : null)],
    );
  }
  columnDefs.push(["end_x", (item) => round(item.pose.x)], ["end_y", (item) => round(item.pose.y)]);
  const columns = columnDefs.map(([name]) => name);
  const rows = Object.fromEntries(Object.entries(vectors).map(([alias, item]) => [alias, columnDefs.map(([, read]) => read(item))]));

  const state = {
    protocol: "shared-perception-v3",
    units: "m, deg; pose=[x, y, heading]; heading 90 points into the bay",
    task: scenarioTasks[scenario] || scenarioTasks["offset-bay"],
    policy: [
      "Park the car inside the target bay without touching the two parked vehicles or the lot boundary.",
      "Completion requires the entire vehicle body inside the bay paint inner edges with the specified edge margin and heading tolerance. Exact centering is not required. Once parked_now is true, hold position; do not improve centering further.",
      "Choose one immediate control. Gear, steering angle, target speed and duration are all part of your choice.",
      "Every listed control was rolled out with the real vehicle body, so the numbers are measurements, not estimates.",
      "plan contains F or R gear, steering angle, target speed in m/s and duration.",
      "The environment below was measured by your own sensors, not handed to you. Occupancy is what has been mapped; unknown_fraction is how much of that command runs through space you have never observed, and predicted_conflict names an object that is forecast to reach that path, with conflict_in_s giving when.",
      "The displayed prediction line is the exact collision-tested path that will execute before the next decision.",
      "Every command in the choice table passed the same collision and lot-boundary safety admission. Other exploratory rollouts may remain visible to the operator but cannot be executed.",
      "Navigation supplies a directed route corridor and measures it; it never chooses a control. Before the final stage, prioritize positive route_progress_m, reduce route_remaining_m, and keep route_error_m and heading_error_deg controlled. Candidate target-distance scores are intentionally deferred until the route crosses its entry gate. In the final stage, parked_after_command=true is the best outcome because it satisfies body-containment and heading requirements at once. If none is true, minimize parking_tolerance_ratio rather than optimizing distance at the expense of heading.",
      "Use progress_analysis and recent_decisions. If recovery.active is true you have been repeating or stalling: change the control instead of repeating a failed command. A state loop means the car returned to the same position and heading, even if the command names differed.",
      "During recovery, controls that return to a recently visited physical state, including states visited earlier in the current recovery episode, are temporarily removed from both engines' shared choice set. This is a short-term tabu constraint, not a selected escape action: you still choose freely among all remaining safe, novel controls.",
      "Reverse is allowed and is often required to open up the angle. Choose stop only when parked_now is true.",
    ].join(" "),
    scenario,
    ego: { pose: roundPose(pose), geometry: VEHICLE },
    target: { pose: roundPose(target), bay: PARKING_BAY, completion: PARKING_RULE },
    navigation: navigation ? {
      stage: navigation.stageId, stage_number: navigation.stageIndex + 1, stages_total: navigation.stageCount,
      instruction: navigation.label, final_stage: navigation.finalStage,
      route: navigation.route, route_progress_m: navigation.progressM, route_remaining_m: navigation.remainingM,
      route_lateral_error_m: navigation.lateralErrorM, route_heading_deg: navigation.routeHeadingDeg,
      route_heading_error_deg: navigation.headingErrorDeg, corridor_half_width_m: navigation.corridorHalfWidthM,
      final_bay_pose: roundPose(target),
    } : null,
    // Either what the sensors measured, or, when no perception layer is wired
    // up, the simulator's own object list marked as such.
    perceived_environment: environment || {
      source: "simulator ground truth (no perception layer attached)",
      ground_truth_available: true,
      obstacles: OBSTACLES.map((item) => ({ id: item.id, type: item.type, x: item.x, y: item.y, length: item.length, width: item.width, heading_deg: item.heading })),
      lot_bounds: LOT_BOUNDS,
      safety_margin_m: SAFETY_MARGIN,
    },
    parked_now: isParked(pose, target),
    progress_analysis: analysis,
    recent_decisions: history.slice(-8).map((item) => ({
      requested_command: item.requestedAction || item.action, executed_command: item.action,
      distance_to_target_m: round(item.distance), heading_error_deg: round(item.angleError),
      route_remaining_m: item.navigationDistance == null ? null : round(item.navigationDistance),
      clearance_m: item.clearance == null ? null : round(item.clearance), blocked: Boolean(item.blocked),
      pose: item.pose ? roundPose(item.pose) : null,
    })),
  };
  if (recovering) state.recovery = recovery;
  if (!stop.available) state.all_candidates_move = true;
  else state.stop_reasons = stop.reasons;

  state.candidates = {
    horizon_segments: 1,
    ...factorTable(columns, rows),
  };

  const questions = {};
  const fixed = {};
  const hasDrive = Object.keys(vectors).length > 0;
  const motions = hasDrive ? (stop.available ? { drive: null, stop: null } : { drive: null }) : { stop: null };
  if (Object.keys(motions).length === 1) {
    const onlyMotion = Object.keys(motions)[0];
    fixed.motion = { type: "choice", choice: onlyMotion, probabilities: { [onlyMotion]: 1 } };
  }
  else {
    questions.motion = {
      type: "choice",
      instructions: "drive continues the maneuver; stop means hold this exact position now. Prefer drive unless parked_now is true.",
      criteria: motions,
    };
  }
  const vectorChoices = Object.fromEntries(Object.keys(vectors).map((alias) => [alias, null]));
  if (!Object.keys(vectorChoices).length && fixed.motion?.choice !== "stop") throw new Error("No safe command is available from this pose");
  if (Object.keys(vectorChoices).length > 0) {
    questions.vector = {
      type: "choice",
      instructions: [
        "Assuming drive, choose the immediate control that makes the most useful progress through the current navigation stage. Before the final stage, prefer positive route_progress_m and low route_remaining_m while controlling route_error_m and heading_error_deg. The route is measurement only; you choose the control.",
        recovering
          ? (navigation?.finalStage
            ? "You are in a persistent final-stage recovery episode. Choose parked_after_command=true whenever available. Otherwise prefer positive recovery_objective_gain. If no row improves, choose a novel row with positive recovery_escape_gain_m or recovery_heading_escape_gain_deg; after leaving the basin, turn that space into objective progress. Reject returns_to_recent_state, returns_to_recovery_state and undoes_previous_move. Do not reverse the previous maneuver unless the predicted endpoint has positive objective gain."
            : "You are in a persistent recovery episode. Prefer positive recovery_objective_gain. If no row improves, choose a novel row with positive recovery_escape_gain_m or recovery_heading_escape_gain_deg; after leaving the basin, turn that space into route progress. Reject returns_to_recent_state, returns_to_recovery_state and undoes_previous_move. Do not reverse the previous maneuver unless the predicted endpoint has positive objective gain.")
          : navigation?.finalStage
            ? "This is the final stage: choose parked_after_command=true whenever available. Otherwise minimize parking_tolerance_ratio while keeping clearance_m comfortable; use signed target_longitudinal_error_m and target_lateral_error_m to correct the actual pose rather than alternating indistinguishable radial-distance moves."
            : "Complete the current approach stage before optimizing final bay alignment. Keep clearance_m comfortable.",
      ].join(" "),
      criteria: vectorChoices,
    };
  }
  return { request: { model: null, state, questions }, fixed, aliases, eligible };
}

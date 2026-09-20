import { parkingAssessment } from "../public/parking-goal.js";
import { createCandidates } from "./candidates.js";
import { distance } from "./math.js";
import { navigationState } from "./navigation.js";
import { analyzeHistory, annotateCandidateHistory, annotateCandidateRecovery } from "./policy.js";
import { SENSOR, annotateCandidates, createPerception, perceive, perceivedEnvironment, perceivedWorld, sceneSnapshot } from "../perception.js";

export class ParkingSessionStore {
  constructor() { this.sessions = new Map(); }

  perceive({ engine, scenario, pose, target, history, reset = false, runId = null }) {
    const normalizedRunId = runId == null ? null : String(runId);
    const key = engine + ":" + scenario + ":" + (normalizedRunId ?? "default");
    let session = this.sessions.get(key);
    const changedRun = normalizedRunId !== null && session?.runId !== normalizedRunId;
    const jumped = !session || !session.expectedPose || distance(pose, session.expectedPose) > 0.05 || reset || changedRun;
    if (jumped) session = { state: createPerception(), expectedPose: null, lastDurationS: 0, navigationStage: 0 };
    session.runId = normalizedRunId;
    perceive(session.state, pose, session.expectedPose ? session.lastDurationS + SENSOR.decisionLatencyS : 0);
    this.sessions.set(key, session);

    const navigation = navigationState(scenario, pose, target, session.navigationStage);
    session.navigationStage = navigation.stageIndex;
    const stageHistory = history.filter((item) => item.navigationStage === navigation.stageId);
    const analysis = analyzeHistory(stageHistory, navigation.finalStage, target);
    const recovery = updateRecoveryEpisode(session, analysis, pose, navigation);
    const candidates = createCandidates(pose, target, perceivedWorld(session.state), navigation, recovery);
    annotateCandidates(session.state, candidates);
    annotateCandidateHistory(candidates, stageHistory, pose);
    annotateCandidateRecovery(candidates, recovery, pose);
    return {
      session, navigation, stageHistory, analysis, recovery, candidates,
      environment: perceivedEnvironment(session.state), scene: sceneSnapshot(session.state),
    };
  }

  commit(session, pose, durationS) {
    session.expectedPose = pose;
    session.lastDurationS = durationS;
  }
}

function updateRecoveryEpisode(session, analysis, pose, navigation) {
  const objective = navigation.finalStage
    ? parkingAssessment(pose, navigation.goal).toleranceRatio
    : navigation.remainingM;
  let episode = session.recoveryEpisode;
  if (episode && episode.stageId !== navigation.stageId) episode = null;
  let ended = false;

  if (episode) {
    episode.decisions += 1;
    episode.bestObjective = Math.min(episode.bestObjective, objective);
    rememberPose(episode, pose);
    const escapedPose = distance(pose, episode.anchorPose) >= 0.4
      || headingDifference(pose.heading, episode.anchorPose.heading) >= 22;
    const improvement = episode.baselineObjective - objective;
    const materiallyImproved = improvement >= (navigation.finalStage ? 0.2 : 0.18);
    const escapedWithoutRegression = escapedPose
      && improvement >= (navigation.finalStage ? 0.04 : 0.06);
    episode.exitEvidence = materiallyImproved || escapedWithoutRegression
      ? episode.exitEvidence + 1
      : 0;
    // One lucky move is not enough to declare a loop resolved. Requiring two
    // consecutive observations adds hysteresis without choosing either move.
    if (episode.exitEvidence >= 2) {
      episode = null;
      ended = true;
    }
  }
  if (!episode && !ended && analysis.recoveryActive) {
    episode = {
      stageId: navigation.stageId, anchorPose: { ...pose }, baselineObjective: objective,
      bestObjective: objective, decisions: 0, reasons: [...analysis.recoveryReasons],
      visitedPoses: [{ ...pose }], exitEvidence: 0,
    };
  }
  session.recoveryEpisode = episode;
  return episode ? {
    active: true, reasons: episode.reasons, repeated_commands: analysis.repeatedActions,
    episode_decisions: episode.decisions, anchor_pose: { ...episode.anchorPose },
    visited_poses: episode.visitedPoses.map((item) => ({ ...item })),
    final_stage: Boolean(navigation.finalStage), exit_evidence: episode.exitEvidence,
    baseline_objective: episode.baselineObjective, best_objective: episode.bestObjective,
    exit_condition: "show progress for two consecutive decisions, including leaving the basin without regressing or materially improving the objective",
    note: "The model must choose the recovery control. This memory only keeps the escape objective stable across decisions.",
  } : {
    active: false, reasons: [], repeated_commands: analysis.repeatedActions,
    note: "No recovery episode is active.",
  };
}

function rememberPose(episode, pose) {
  const last = episode.visitedPoses.at(-1);
  if (!last || distance(last, pose) >= 0.08 || headingDifference(last.heading, pose.heading) >= 5) {
    episode.visitedPoses.push({ ...pose });
    if (episode.visitedPoses.length > 32) episode.visitedPoses.shift();
  }
}

function headingDifference(a, b) {
  let value = (Number(a) - Number(b)) % 360;
  if (value > 180) value -= 360;
  if (value < -180) value += 360;
  return Math.abs(value);
}

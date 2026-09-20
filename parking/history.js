import { round, roundPose } from "./math.js";
import { sanitizePose } from "./pose.js";

export function sanitizeHistory(history) {
  if (!Array.isArray(history)) return [];
  return history.slice(-12).map((item) => ({
    action: String(item.action || ""),
    requestedAction: String(item.requestedAction || item.action || ""),
    distance: round(Number(item.distance) || 0),
    navigationDistance: item.navigationDistance == null ? null : round(Number(item.navigationDistance) || 0),
    navigationStage: item.navigationStage ? String(item.navigationStage) : null,
    angleError: round(Number(item.angleError) || 0),
    clearance: item.clearance == null ? null : round(Number(item.clearance) || 0),
    blocked: Boolean(item.blocked),
    control: item.control && typeof item.control === "object" ? {
      gear: String(item.control.gear || ""), steerDeg: Number(item.control.steerDeg) || 0,
      targetSpeed: Number(item.control.targetSpeed) || 0,
      validityS: Number(item.control.validityS) || Number(item.control.duration) || 0,
      travelDistance: Number(item.control.travelDistance) || 0,
    } : null,
    pose: item.pose ? roundPose(sanitizePose(item.pose)) : null,
  }));
}

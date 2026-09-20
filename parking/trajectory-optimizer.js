import { clamp, round } from "./math.js";
import { VEHICLE } from "./config.js";

// A small, deterministic trajectory optimizer shared by both engines. It only
// regularizes adjacent controls; it has no scene IDs, target poses, or preferred
// parking path and therefore cannot manufacture an answer for either model.
export function smoothControlSequence(segments) {
  if (!Array.isArray(segments) || segments.length < 2) return (segments || []).map(copy);
  const source = segments.map(copy);
  const filtered = source.map((segment, index) => {
    if (index === 0 || segment.direction !== source[index - 1].direction) return { ...segment };
    const previous = source[index - 1];
    const next = source[index + 1]?.direction === segment.direction ? source[index + 1] : segment;
    // Preserve the selected first control and smooth future steering with a
    // local [1,2,1] filter. This avoids sharp curvature changes while retaining
    // the candidate's intended shape.
    // A true reversal in steering is an S-curve and must remain a reversal.
    // Averaging it through zero erases the lateral authority of the trajectory.
    const reversesSteering = previous.steerDeg * segment.steerDeg < 0;
    const steerDeg = reversesSteering
      ? segment.steerDeg
      : (previous.steerDeg + 2 * segment.steerDeg + next.steerDeg) / 4;
    return { ...segment, steerDeg: round(clamp(steerDeg, -VEHICLE.maxSteerDeg, VEHICLE.maxSteerDeg)) };
  });
  // Enforce actuator continuity after filtering. A gear change represents a
  // stop, so a new run may start at its requested steering and speed; within a
  // run both steering and speed are slew-limited using segment duration.
  for (let index = 1; index < filtered.length; index += 1) {
    const segment = filtered[index], previous = filtered[index - 1];
    if (segment.direction !== previous.direction) continue;
    const maxSteerDelta = Math.max(6, 72 * Math.min(segment.duration, previous.duration));
    segment.steerDeg = round(previous.steerDeg
      + clamp(segment.steerDeg - previous.steerDeg, -maxSteerDelta, maxSteerDelta));
    const maxSpeedDelta = 0.55 * Math.min(segment.duration, previous.duration);
    segment.targetSpeed = round(Math.max(0, previous.targetSpeed
      + clamp(segment.targetSpeed - previous.targetSpeed, -maxSpeedDelta, maxSpeedDelta)));
  }
  return filtered;
}

export function continuityMetrics(segments) {
  let steeringVariationDeg = 0;
  let speedVariationMps = 0;
  let gearChanges = 0;
  for (let index = 1; index < segments.length; index += 1) {
    steeringVariationDeg += Math.abs(segments[index].steerDeg - segments[index - 1].steerDeg);
    speedVariationMps += Math.abs(segments[index].targetSpeed - segments[index - 1].targetSpeed);
    if (segments[index].direction !== segments[index - 1].direction) gearChanges += 1;
  }
  return { steeringVariationDeg: round(steeringVariationDeg), speedVariationMps: round(speedVariationMps), gearChanges };
}

function copy(segment) {
  return {
    direction: segment.direction < 0 ? -1 : 1,
    steerDeg: Number(segment.steerDeg) || 0,
    targetSpeed: Number(segment.targetSpeed) || 0,
    duration: Number(segment.duration) || 0,
  };
}

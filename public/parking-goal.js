// Shared by the renderer, browser loop, server and comparison tests.
export const VEHICLE = { wheelbase: 1.05, length: 1.75, width: 0.82, maxSteerDeg: 32 };
export const PARKING_BAY = { width: 2.04, length: 2.2, lineWidth: 0.055 };
export const PARKING_RULE = { edgeMarginM: 0.05, headingDeg: 12 };

export function parkingAssessment(pose, target) {
  if (![pose?.x, pose?.y, pose?.heading, target?.x, target?.y, target?.heading].every(Number.isFinite)) {
    return { parked: false, toleranceRatio: Infinity };
  }
  const heading = target.heading * Math.PI / 180;
  const dx = pose.x - target.x, dy = pose.y - target.y;
  const longitudinal = dx * Math.cos(heading) + dy * Math.sin(heading);
  const lateral = -dx * Math.sin(heading) + dy * Math.cos(heading);
  const delta = ((pose.heading - target.heading + 180) % 360 + 360) % 360 - 180;
  const angle = delta * Math.PI / 180;
  // Extreme projections of all four body corners onto the bay axes.
  const lengthExtent = Math.abs(longitudinal) + Math.abs(Math.cos(angle)) * VEHICLE.length / 2
    + Math.abs(Math.sin(angle)) * VEHICLE.width / 2;
  const widthExtent = Math.abs(lateral) + Math.abs(Math.sin(angle)) * VEHICLE.length / 2
    + Math.abs(Math.cos(angle)) * VEHICLE.width / 2;
  const innerHalfLength = (PARKING_BAY.length - PARKING_BAY.lineWidth) / 2;
  const innerHalfWidth = (PARKING_BAY.width - PARKING_BAY.lineWidth) / 2;
  const toleranceRatio = Math.max(
    lengthExtent / (innerHalfLength - PARKING_RULE.edgeMarginM),
    widthExtent / (innerHalfWidth - PARKING_RULE.edgeMarginM),
    Math.abs(delta) / PARKING_RULE.headingDeg,
  );
  return {
    parked: toleranceRatio <= 1 + 1e-9, toleranceRatio,
    edgeClearanceM: Math.min(innerHalfLength - lengthExtent, innerHalfWidth - widthExtent),
    headingErrorDeg: Math.abs(delta), longitudinal, lateral,
  };
}

// Evaluated at a control endpoint; the caller finishes the animation and holds.
export function isParked(pose, target) { return parkingAssessment(pose, target).parked; }

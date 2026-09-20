export function clamp(value, min, max) {
  const number = Number(value);
  return Math.min(max, Math.max(min, Number.isFinite(number) ? number : min));
}

export function round(value) { return Math.round(Number(value) * 100) / 100; }

export function normalizeAngle(value) {
  let angle = Number(value) % 360;
  if (angle > 180) angle -= 360;
  if (angle < -180) angle += 360;
  return angle;
}

export function roundPose(pose) {
  return { x: round(pose.x), y: round(pose.y), heading: round(normalizeAngle(pose.heading)) };
}

export function distance(a, b) {
  return Math.hypot(Number(b.x) - Number(a.x), Number(b.y) - Number(a.y));
}

export function angleError(a, b) {
  let value = (Number(b) - Number(a)) % 360;
  if (value > 180) value -= 360;
  if (value < -180) value += 360;
  return Math.abs(value);
}

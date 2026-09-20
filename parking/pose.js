import { clamp, normalizeAngle } from "./math.js";

export function sanitizePose(value = {}) {
  return {
    x: clamp(Number(value.x) || 0, -5.3, 5.3),
    y: clamp(Number(value.y) || 0, -4, 4),
    heading: normalizeAngle(Number(value.heading) || 0),
  };
}

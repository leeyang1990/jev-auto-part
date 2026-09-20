import { SAFETY_MARGIN } from "./config.js";
import { isLegal } from "./policy.js";
import { round } from "./math.js";

function samplePath(path, maxPoints = 61) {
  if (!Array.isArray(path) || path.length === 0) return [];
  if (path.length <= maxPoints) return path.map((point) => [round(point.x), round(point.y), round(point.heading)]);
  return Array.from({ length: maxPoints }, (_, index) => {
    const at = path[Math.round((index / (maxPoints - 1)) * (path.length - 1))];
    return [round(at.x), round(at.y), round(at.heading)];
  });
}

export function candidateOverlay(candidates, chosenAction, probabilities, blockedAction = null) {
  const entries = Object.entries(candidates).filter(([, item]) => Array.isArray(item.path) && item.path.length >= 2);
  const selectedHold = Object.entries(candidates).find(([id, item]) =>
    (id === chosenAction || id === blockedAction) && Array.isArray(item.path) && item.path.length === 1);
  const score = (item) => item.distance + item.angleError * 0.02;
  const legal = entries.filter(([, item]) => !item.collision && item.clearance >= SAFETY_MARGIN);
  const rejected = entries.filter(([, item]) => item.collision || item.clearance < SAFETY_MARGIN || item.horizonCollision);
  const conflict = entries.filter(([, item]) => item.predictedConflict);
  const picked = new Map();
  const add = (entry) => { if (entry && picked.size < 12 && !picked.has(entry[0])) picked.set(entry[0], entry[1]); };
  add(selectedHold);
  add(entries.find(([id]) => id === chosenAction));
  add(entries.find(([id]) => id === blockedAction));
  [...legal].sort((a, b) => score(a[1]) - score(b[1])).slice(0, 3).forEach(add);
  for (const steer of [-32, -20, -10, 0, 10, 20, 32]) {
    add(legal.filter(([, item]) => item.control.steerDeg === steer).sort((a, b) => score(a[1]) - score(b[1]))[0]);
    if (picked.size >= 10) break;
  }
  conflict.slice(0, 1).forEach(add);
  rejected.slice(0, 1).forEach(add);
  return [...picked.entries()].map(([id, item]) => ({
    id,
    gear: item.control?.gear === "reverse" ? "reverse" : item.control?.gear === "forward" ? "forward" : "hold",
    segments: (item.segmentsDetail || []).map((segment) => [segment.direction, segment.steerDeg, round(segment.travelDistance)]),
    displaySegments: (item.segmentsDetail || []).map((segment) => [segment.direction, segment.steerDeg, round(segment.travelDistance)]),
    path: samplePath(item.path.length === 1 ? [item.path[0], item.path[0]] : item.path), legal: isLegal(item),
    horizonDanger: Boolean(item.horizonCollision || item.horizonClearance < SAFETY_MARGIN),
    horizonClearance: item.horizonClearance ?? null, horizonSafeDistance: item.horizonSafeDistance ?? null,
    conflict: item.predictedConflict || null, blocked: id === blockedAction, proposedBy: null,
    chosen: id === chosenAction, plan: item.plan || null,
    likelihood: probabilities && Number.isFinite(Number(probabilities[id])) ? round(probabilities[id]) : null,
  }));
}

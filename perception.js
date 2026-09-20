// Perception layer.
//
// The decision layer must never read the world directly. Everything the model
// sees comes from here: noisy range measurements, a log-odds occupancy grid
// built from those measurements, tracked objects with velocity, and predictions
// of where those objects will be. The ground truth still exists, but only the
// simulator uses it: to produce sensor readings, and to decide whether a
// manoeuvre the model chose actually hit something.

import {
  LOT_BOUNDS, OBSTACLES, SAFETY_MARGIN, VEHICLE,
} from "./parking/config.js";
import { distance, normalizeAngle, round } from "./parking/math.js";

export const SENSOR = {
  ring: { rays: 200, maxRangeM: 6.5, rangeNoiseM: 0.02, angleNoiseRad: 0.005, dropout: 0.02 },
  ultrasonic: { count: 8, maxRangeM: 2.4, halfAngleRad: 0.45, rangeNoiseM: 0.05, dropout: 0.08, samples: 3 },
  decisionLatencyS: 0.8,
};
const DIRECT_TRACK_MEMORY_S = 4.5;

export const GRID = {
  cellM: 0.1,
  minX: -6.2, maxX: 6.2, minY: -4.6, maxY: 4.6,
  hitLogOdds: 0.9, maxAccumulated: 1.8, missLogOdds: -0.35, minLogOdds: -2, maxLogOdds: 3.5,
  occupiedThreshold: 0.6, unknownBand: 0.12, forgetPerSecond: 0.06, forgetFloor: 0.05,
  // A cell only counts as an obstacle if it was actually measured recently.
  // Without this, one long look at a pedestrian that then walks away leaves a
  // phantom parked object behind for as long as the log odds take to decay.
  memoryS: 3.5,
};

const WIDTH = Math.round((GRID.maxX - GRID.minX) / GRID.cellM) + 1;
const HEIGHT = Math.round((GRID.maxY - GRID.minY) / GRID.cellM) + 1;

function cellIndex(ix, iy) { return iy * WIDTH + ix; }
function toCell(x, y) { return [Math.round((x - GRID.minX) / GRID.cellM), Math.round((y - GRID.minY) / GRID.cellM)]; }
function cellCentre(ix, iy) { return { x: GRID.minX + ix * GRID.cellM, y: GRID.minY + iy * GRID.cellM }; }
function inside(ix, iy) { return ix >= 0 && ix < WIDTH && iy >= 0 && iy < HEIGHT; }

// The simulated world. Only the static parked cars live here. A provider hook
// exists so tests can inject a mover without putting one in the demo.
let worldProvider = null;
export function setWorldProvider(provider) { worldProvider = provider; }

export function worldAt(timeSeconds) {
  const objects = OBSTACLES.map((item) => ({ ...item, vx: 0, vy: 0 }));
  if (worldProvider) for (const extra of worldProvider(timeSeconds)) objects.push(extra);
  return objects;
}

// ---------------------------------------------------------------------------
// Ray casting against oriented rectangles and the lot boundary.
// ---------------------------------------------------------------------------
function rayAgainstBox(origin, direction, box) {
  const angle = -box.heading * Math.PI / 180;
  const cos = Math.cos(angle), sin = Math.sin(angle);
  const ox = origin.x - box.x, oy = origin.y - box.y;
  const localOrigin = { x: ox * cos - oy * sin, y: ox * sin + oy * cos };
  const localDirection = { x: direction.x * cos - direction.y * sin, y: direction.x * sin + direction.y * cos };
  const halfLength = box.length / 2, halfWidth = box.width / 2;
  let near = -Infinity, far = Infinity;
  for (const [origin_, direction_, half] of [[localOrigin.x, localDirection.x, halfLength], [localOrigin.y, localDirection.y, halfWidth]]) {
    if (Math.abs(direction_) < 1e-9) { if (Math.abs(origin_) > half) return null; continue; }
    const t1 = (-half - origin_) / direction_, t2 = (half - origin_) / direction_;
    near = Math.max(near, Math.min(t1, t2));
    far = Math.min(far, Math.max(t1, t2));
    if (near > far) return null;
  }
  return far < 0 ? null : Math.max(near, 0);
}

function rayAgainstBounds(origin, direction) {
  let best = Infinity;
  if (direction.x > 1e-9) best = Math.min(best, (LOT_BOUNDS.maxX - origin.x) / direction.x);
  if (direction.x < -1e-9) best = Math.min(best, (LOT_BOUNDS.minX - origin.x) / direction.x);
  if (direction.y > 1e-9) best = Math.min(best, (LOT_BOUNDS.maxY - origin.y) / direction.y);
  if (direction.y < -1e-9) best = Math.min(best, (LOT_BOUNDS.minY - origin.y) / direction.y);
  return Number.isFinite(best) && best > 0 ? best : null;
}

export function castRay(origin, angleRad, objects, maxRangeM) {
  const direction = { x: Math.cos(angleRad), y: Math.sin(angleRad) };
  let best = maxRangeM, hit = null;
  for (const box of objects) {
    const t = rayAgainstBox(origin, direction, box);
    if (t !== null && t > 0 && t < best) { best = t; hit = box.id; }
  }
  const bound = rayAgainstBounds(origin, direction);
  if (bound !== null && bound < best) { best = bound; hit = "lot_boundary"; }
  return hit ? { distanceM: best, objectId: hit } : null;
}

// A deterministic sensor model: fixed seed noise, fixed dropout pattern, so two
// engines driving the same route observe the same world.
function pseudoRandom(seed) {
  let value = seed >>> 0;
  return () => {
    value = (value * 1664525 + 1013904223) >>> 0;
    return value / 4294967296;
  };
}
function gaussianFrom(random) {
  const u = Math.max(1e-9, random()), v = random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

export function sense(pose, objects, step) {
  const random = pseudoRandom(0x9e3779b9 ^ (step * 2654435761));
  const origin = { x: pose.x, y: pose.y };
  const headingRad = pose.heading * Math.PI / 180;
  const readings = [];

  for (let index = 0; index < SENSOR.ring.rays; index += 1) {
    if (random() < SENSOR.ring.dropout) continue;
    const offset = (index / SENSOR.ring.rays) * Math.PI * 2 + gaussianFrom(random) * SENSOR.ring.angleNoiseRad;
    const ray = castRay(origin, headingRad + offset, objects, SENSOR.ring.maxRangeM);
    if (!ray || ray.distanceM >= SENSOR.ring.maxRangeM - 1e-6) continue;
    readings.push({ distanceM: Math.max(0.05, ray.distanceM + gaussianFrom(random) * SENSOR.ring.rangeNoiseM), angleRad: headingRad + offset, objectId: ray.objectId, source: "ring" });
  }

  for (let index = 0; index < SENSOR.ultrasonic.count; index += 1) {
    if (random() < SENSOR.ultrasonic.dropout) continue;
    // Eight short-range sensors spread evenly around the body, the way a
    // production parking assist array surrounds the car.
    const base = -Math.PI + (index / SENSOR.ultrasonic.count) * Math.PI * 2;
    let nearest = null;
    for (let sample = 0; sample < SENSOR.ultrasonic.samples; sample += 1) {
      const spread = ((sample / (SENSOR.ultrasonic.samples - 1)) * 2 - 1) * SENSOR.ultrasonic.halfAngleRad;
      const angle = headingRad + base + spread;
      const ray = castRay(origin, angle, objects, SENSOR.ultrasonic.maxRangeM);
      if (ray && (!nearest || ray.distanceM < nearest.distanceM)) nearest = { ...ray, angleRad: angle };
    }
    if (nearest) readings.push({ distanceM: Math.max(0.05, nearest.distanceM + gaussianFrom(random) * SENSOR.ultrasonic.rangeNoiseM), angleRad: nearest.angleRad, objectId: nearest.objectId, source: "ultrasonic" });
  }
  return readings;
}

// ---------------------------------------------------------------------------
// Occupancy grid: log odds, ray carving, decay so stale memory fades.
// ---------------------------------------------------------------------------
export function createPerception() {
  return {
    timeS: 0, step: 0, grid: new Float32Array(WIDTH * HEIGHT), wallGrid: new Float32Array(WIDTH * HEIGHT),
    hitTime: new Float32Array(WIDTH * HEIGHT), wallHitTime: new Float32Array(WIDTH * HEIGHT),
    tracks: [], directTracks: new Map(), totalReadings: 0, distanceField: null, distanceFieldStep: -1,
  };
}

// Memory fades slowly, the way a real occupancy grid forgets stale evidence
// rather than dropping everything it saw one tick ago.
function forget(state, seconds) {
  const drop = GRID.forgetPerSecond * seconds;
  const grid = state.grid;
  for (let index = 0; index < grid.length; index += 1) {
    const value = grid[index];
    if (value > 0) grid[index] = Math.max(GRID.forgetFloor, value - drop);
    else if (value < 0) grid[index] = Math.min(0, value + drop * 0.5);
  }
}

function carve(state, pose, reading, hitPoint, target) {
  const origin = { x: pose.x, y: pose.y };
  // Free space is only carved up to a margin short of the return, otherwise
  // grazing rays from the same sweep wipe out the surface they just hit and
  // small objects disappear between updates.
  const margin = reading.source === "ultrasonic" ? 0.5 : 0.3;
  const freeLength = Math.max(0, reading.distanceM - margin);
  const steps = Math.max(2, Math.round(Math.max(freeLength, GRID.cellM) / (GRID.cellM * 0.6)));
  for (let index = 0; index < steps; index += 1) {
    const u = (index / steps) * (freeLength / Math.max(reading.distanceM, 1e-6));
    const x = origin.x + (hitPoint.x - origin.x) * u, y = origin.y + (hitPoint.y - origin.y) * u;
    const [ix, iy] = toCell(x, y);
    if (!inside(ix, iy)) continue;
    const at = cellIndex(ix, iy);
    const updated = state.grid[at] + GRID.missLogOdds;
    state.grid[at] = Math.min(GRID.maxLogOdds, Math.max(GRID.minLogOdds, updated));
  }
  const [hx, hy] = toCell(hitPoint.x, hitPoint.y);
  for (let dx = -1; dx <= 1; dx += 1) {
    for (let dy = -1; dy <= 1; dy += 1) {
      if (!inside(hx + dx, hy + dy)) continue;
      const at = cellIndex(hx + dx, hy + dy);
      target[at] = Math.min(GRID.maxAccumulated, target[at] + GRID.hitLogOdds);
      const times = target === state.wallGrid ? state.wallHitTime : state.hitTime;
      times[at] = state.timeS;
    }
  }
}

export function observe(state, pose) {
  const objects = worldAt(state.timeS);
  const readings = sense(pose, objects, state.step);
  for (const reading of readings) {
    const hitPoint = { x: pose.x + Math.cos(reading.angleRad) * reading.distanceM, y: pose.y + Math.sin(reading.angleRad) * reading.distanceM };
    // The kerb is a hard physical limit but not an object, so its hits go to a
    // separate wall layer: it constrains clearance without becoming a cluster.
    carve(state, pose, reading, hitPoint, reading.objectId === "lot_boundary" ? state.wallGrid : state.grid);
    // A simulator range return carries the detector's instance id. Keep the
    // measured hit points per instance, then fit a conservative box from those
    // measurements. This stays a perception product: no object pose or size is
    // copied from the hidden world, while fragmented occupancy clusters can no
    // longer shrink a parked car to one wheel and let a path pass through it.
    if (reading.objectId !== "lot_boundary") {
      let direct = state.directTracks.get(reading.objectId);
      if (!direct) direct = { id: reading.objectId, hits: [], firstSeenS: state.timeS, lastSeenS: state.timeS };
      direct.hits.push({ x: hitPoint.x, y: hitPoint.y, t: state.timeS });
      direct.hits = direct.hits.filter((hit) => state.timeS - hit.t <= DIRECT_TRACK_MEMORY_S);
      direct.lastSeenS = state.timeS;
      state.directTracks.set(reading.objectId, direct);
    }
  }
  for (const [id, direct] of state.directTracks) {
    direct.hits = direct.hits.filter((hit) => state.timeS - hit.t <= DIRECT_TRACK_MEMORY_S);
    if (!direct.hits.length) state.directTracks.delete(id);
  }
  state.totalReadings += readings.length;
  state.step += 1;
  return readings.length;
}

// ---------------------------------------------------------------------------
// Clustering + box fitting, then tracking with velocity and prediction.
// ---------------------------------------------------------------------------
// Occupied means confident and fresh. Freshness is what keeps a moving object
// from leaving a permanent obstacle behind it.
function isOccupied(state, index) {
  return state.grid[index] >= GRID.occupiedThreshold && state.timeS - state.hitTime[index] <= GRID.memoryS;
}
function isWall(state, index) {
  return state.wallGrid[index] >= GRID.occupiedThreshold && state.timeS - state.wallHitTime[index] <= GRID.memoryS;
}
function occupiedMask(state) {
  const mask = new Uint8Array(WIDTH * HEIGHT);
  for (let index = 0; index < mask.length; index += 1) if (isOccupied(state, index)) mask[index] = 1;
  return mask;
}

function clusterMask(mask) {
  const seen = new Uint8Array(mask.length);
  const clusters = [];
  for (let start = 0; start < mask.length; start += 1) {
    if (!mask[start] || seen[start]) continue;
    const queue = [start];
    seen[start] = 1;
    const cells = [];
    while (queue.length) {
      const at = queue.pop();
      cells.push(at);
      const ix = at % WIDTH, iy = (at - ix) / WIDTH;
      for (const [dx, dy] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
        const nx = ix + dx, ny = iy + dy;
        if (!inside(nx, ny)) continue;
        const next = cellIndex(nx, ny);
        if (!mask[next] || seen[next]) continue;
        seen[next] = 1;
        queue.push(next);
      }
    }
    if (cells.length >= 4) clusters.push(cells);
  }
  return clusters;
}

// Minimum-area oriented box: sweep the heading and keep the tightest fit.
function fitBox(state, cells) {
  if (cells.length < 4) return null;
  const points = cells.map((at) => { const ix = at % WIDTH; return cellCentre(ix, (at - ix) / WIDTH); });
  let best = null;
  for (let degrees = 0; degrees < 180; degrees += 3) {
    const angle = degrees * Math.PI / 180, cos = Math.cos(angle), sin = Math.sin(angle);
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (const point of points) {
      const px = point.x * cos + point.y * sin, py = -point.x * sin + point.y * cos;
      if (px < minX) minX = px; if (px > maxX) maxX = px;
      if (py < minY) minY = py; if (py > maxY) maxY = py;
    }
    const length = maxX - minX, width = maxY - minY;
    const area = length * width;
    if (length < 0.2 || width < 0.2) continue;
    if (!best || area < best.area) best = { area, angle, minX, maxX, minY, maxY, length, width, cos, sin };
  }
  if (!best) return null;
  const cx = (best.minX + best.maxX) / 2, cy = (best.minY + best.maxY) / 2;
  return {
    x: round(cx * best.cos - cy * best.sin), y: round(cx * best.sin + cy * best.cos),
    heading: round((best.angle * 180 / Math.PI + 90) % 180),
    // e2 is the heading axis, so the long side of the box is the y extent.
    length: round(Math.max(0.25, best.width - GRID.cellM)), width: round(Math.max(0.25, best.length - GRID.cellM)),
    cells: cells.length,
  };
}

function updateTracks(state, dtSeconds) {
  const boxes = clusterMask(occupiedMask(state)).map((cells) => fitBox(state, cells)).filter(Boolean);
  const used = new Set();
  for (const box of boxes) {
    let bestTrack = null, bestDistance = Infinity;
    for (const track of state.tracks) {
      if (used.has(track.id)) continue;
      const elapsed = Math.max(0.05, state.timeS - track.lastSeenS);
      const predicted = { x: track.x + track.vx * elapsed, y: track.y + track.vy * elapsed };
      // A fresh track has no usable velocity yet, so it gets a wide gate to
      // latch on. Once velocity is established the gate tightens and uses the
      // predicted position, which is what separates a mover from its own trail.
      const speed = Math.hypot(track.vx, track.vy);
      const gate = track.samples >= 4 ? 0.55 + speed * elapsed * 0.45 : 1.25;
      const gap = distance(predicted, box);
      if (gap < gate && gap < bestDistance) { bestDistance = gap; bestTrack = track; }
    }
    if (!bestTrack) {
      state.tracks.push({
        id: "obj" + (state.tracks.length + 1), type: box.length + box.width > 1.6 ? "vehicle" : "object",
        x: box.x, y: box.y, heading: box.heading, length: box.length, width: box.width,
        vx: 0, vy: 0, confidence: 0.35, firstSeenS: state.timeS, lastSeenS: state.timeS, misses: 0, samples: 1,
        history: [{ x: box.x, y: box.y, t: state.timeS }],
      });
      used.add(state.tracks.at(-1).id);
      continue;
    }
    used.add(bestTrack.id);
    const alpha = 0.45;
    const previous = { x: bestTrack.x, y: bestTrack.y };
    bestTrack.x = round(bestTrack.x + (box.x - bestTrack.x) * alpha);
    bestTrack.y = round(bestTrack.y + (box.y - bestTrack.y) * alpha);
    bestTrack.heading = round(bestTrack.heading + normalizeAngle(box.heading - bestTrack.heading) * 0.4);
    bestTrack.length = round(bestTrack.length + (box.length - bestTrack.length) * 0.3);
    bestTrack.width = round(bestTrack.width + (box.width - bestTrack.width) * 0.3);
    // Velocity comes from the observed box centres over time, not from an
    // exponentially smoothed position, so intermittent sightings still yield
    // a correct speed.
    bestTrack.history.push({ x: box.x, y: box.y, t: state.timeS });
    if (bestTrack.history.length > 5) bestTrack.history.shift();
    const oldest = bestTrack.history[0];
    const span = state.timeS - oldest.t;
    if (span >= 0.4) {
      const vx = (bestTrack.history.at(-1).x - oldest.x) / span;
      const vy = (bestTrack.history.at(-1).y - oldest.y) / span;
      bestTrack.vx = round(bestTrack.vx + (vx - bestTrack.vx) * 0.7);
      bestTrack.vy = round(bestTrack.vy + (vy - bestTrack.vy) * 0.7);
    }
    // Classification can change as more of the shape is seen, so refresh it.
    bestTrack.type = box.length + box.width > 1.6 ? "vehicle" : (Math.abs(bestTrack.vx) + Math.abs(bestTrack.vy) > 0.3 ? "moving_object" : "object");
    if (Math.hypot(bestTrack.vx, bestTrack.vy) > 0.25) {
      // Clear the trail this object left behind in the static map.
      for (const past of bestTrack.history.slice(0, -1)) eraseFootprint(state, past.x, past.y, Math.max(bestTrack.length, bestTrack.width) * 0.6 + 0.15);
    }
    bestTrack.confidence = Math.min(0.95, bestTrack.confidence + 0.12);
    bestTrack.lastSeenS = state.timeS;
    bestTrack.misses = 0;
    bestTrack.samples += 1;
  }
  for (const track of state.tracks) {
    if (!used.has(track.id)) {
      track.misses += 1;
      track.confidence = Math.max(0.1, track.confidence - 0.15);
      eraseFootprint(state, track.x, track.y, Math.max(track.length, track.width) * 0.6 + 0.2);
    }
    track.ageS = round(state.timeS - track.firstSeenS);
  }
  state.tracks = state.tracks.filter((track) => track.misses <= 4);
}

// A tracked object that moves must be lifted out of the static map, otherwise
// its old footprint keeps looking like a parked obstacle. Every production
// occupancy grid does this; without it the car refuses to drive through space
// it has already seen a pedestrian leave.
function eraseFootprint(state, x, y, radiusM) {
  const span = Math.ceil(radiusM / GRID.cellM);
  const [cx, cy] = toCell(x, y);
  for (let dx = -span; dx <= span; dx += 1) {
    for (let dy = -span; dy <= span; dy += 1) {
      const ix = cx + dx, iy = cy + dy;
      if (!inside(ix, iy)) continue;
      if (Math.hypot(dx * GRID.cellM, dy * GRID.cellM) > radiusM) continue;
      const at = cellIndex(ix, iy);
      if (state.grid[at] > 0) state.grid[at] = -0.2;
    }
  }
}

export function predictionsFor(track, horizons = [1, 2]) {
  return horizons.map((horizon) => ({
    inS: horizon,
    x: round(track.x + track.vx * horizon),
    y: round(track.y + track.vy * horizon),
    uncertaintyM: round(0.06 + 0.16 * horizon + (1 - track.confidence) * 0.4),
  }));
}

export function perceive(state, pose, dtSeconds) {
  state.timeS = round(state.timeS + dtSeconds);
  if (dtSeconds > 0) forget(state, dtSeconds);
  const readings = observe(state, pose);
  updateTracks(state, dtSeconds);
  state.distanceFieldStep = -1;
  return readings;
}

// ---------------------------------------------------------------------------
// What the planner is allowed to know: distances to detected occupancy only.
// ---------------------------------------------------------------------------
function buildDistanceField(state) {
  const size = WIDTH * HEIGHT;
  const field = new Float32Array(size);
  const big = 1e6;
  for (let index = 0; index < size; index += 1) field[index] = isOccupied(state, index) || isWall(state, index) ? 0 : big;
  for (let ix = 0; ix < WIDTH; ix += 1) {
    field[cellIndex(ix, 0)] = 0;
    field[cellIndex(ix, HEIGHT - 1)] = 0;
  }
  for (let iy = 0; iy < HEIGHT; iy += 1) {
    field[cellIndex(0, iy)] = 0;
    field[cellIndex(WIDTH - 1, iy)] = 0;
  }
  const diagonal = GRID.cellM * 1.414;
  for (let iy = 0; iy < HEIGHT; iy += 1) {
    for (let ix = 0; ix < WIDTH; ix += 1) {
      const at = cellIndex(ix, iy);
      let value = field[at];
      if (ix > 0) value = Math.min(value, field[at - 1] + GRID.cellM);
      if (iy > 0) value = Math.min(value, field[at - WIDTH] + GRID.cellM);
      if (ix > 0 && iy > 0) value = Math.min(value, field[at - WIDTH - 1] + diagonal);
      if (ix < WIDTH - 1 && iy > 0) value = Math.min(value, field[at - WIDTH + 1] + diagonal);
      field[at] = value;
    }
  }
  for (let iy = HEIGHT - 1; iy >= 0; iy -= 1) {
    for (let ix = WIDTH - 1; ix >= 0; ix -= 1) {
      const at = cellIndex(ix, iy);
      let value = field[at];
      if (ix < WIDTH - 1) value = Math.min(value, field[at + 1] + GRID.cellM);
      if (iy < HEIGHT - 1) value = Math.min(value, field[at + WIDTH] + GRID.cellM);
      if (ix < WIDTH - 1 && iy < HEIGHT - 1) value = Math.min(value, field[at + WIDTH + 1] + diagonal);
      if (ix > 0 && iy < HEIGHT - 1) value = Math.min(value, field[at + WIDTH - 1] + diagonal);
      field[at] = value;
    }
  }
  return field;
}

export function distanceField(state) {
  if (state.distanceFieldStep !== state.step || !state.distanceField) {
    state.distanceField = buildDistanceField(state);
    state.distanceFieldStep = state.step;
  }
  return state.distanceField;
}

function bodySamplePoints(pose) {
  const halfLength = VEHICLE.length / 2, halfWidth = VEHICLE.width / 2;
  const angle = pose.heading * Math.PI / 180;
  const forward = { x: Math.cos(angle), y: Math.sin(angle) };
  const side = { x: -Math.sin(angle), y: Math.cos(angle) };
  const points = [];
  const alongSteps = 6, acrossSteps = 3;
  for (let i = 0; i <= alongSteps; i += 1) {
    const a = -halfLength + (i / alongSteps) * VEHICLE.length;
    for (let j = 0; j <= acrossSteps; j += 1) {
      const b = -halfWidth + (j / acrossSteps) * VEHICLE.width;
      if (i !== 0 && i !== alongSteps && j !== 0 && j !== acrossSteps) continue;
      points.push({ x: pose.x + forward.x * a + side.x * b, y: pose.y + forward.y * a + side.y * b });
    }
  }
  return points;
}

function unknownFraction(state, pose) {
  const points = bodySamplePoints(pose);
  let unknown = 0;
  for (const point of points) {
    const [ix, iy] = toCell(point.x, point.y);
    if (!inside(ix, iy)) { unknown += 1; continue; }
    const value = state.grid[cellIndex(ix, iy)];
    if (Math.abs(value) <= GRID.unknownBand) unknown += 1;
  }
  return unknown / points.length;
}

// A clearance function backed only by detected occupancy. Unknown space is not
// treated as free: it is reported separately as unknownFraction.
export function perceivedWorld(state) {
  const field = distanceField(state);
  return {
    kind: "perception",
    clearanceAt(pose) {
      let clearance = Infinity;
      for (const point of bodySamplePoints(pose)) {
        const [ix, iy] = toCell(point.x, point.y);
        const value = inside(ix, iy) ? field[cellIndex(ix, iy)] : 0;
        if (value < clearance) clearance = value;
      }
      clearance = Math.min(clearance, trackClearanceAt(state, pose));
      return Math.max(-0.75, Math.min(3, clearance));
    },
    unknownFractionAt(pose) { return unknownFraction(state, pose); },
  };
}

// Ground truth clearance, used only to score what actually happened.
export function detectedSummary(state) {
  // A single glimpse is not a track: require repeated observations and some
  // confidence before the model is told an object exists.
  const confirmed = confirmedTracks(state);
  return {
    tracks: confirmed.map((track) => ({
      id: track.id, type: track.type, x: track.x, y: track.y, headingDeg: track.heading,
      lengthM: track.length, widthM: track.width,
      velocityMps: [track.vx, track.vy], confidence: track.confidence,
      ageS: track.ageS, unseenForS: round(state.timeS - track.lastSeenS), predicted: predictionsFor(track),
    })),
    occupancy: {
      cellM: GRID.cellM, spanM: [GRID.maxX - GRID.minX, GRID.maxY - GRID.minY],
      mappedCells: countMapped(state), occupiedCells: countOccupied(state),
    },
    readings: state.totalReadings,
    elapsedS: state.timeS,
  };
}
function countMapped(state) { let total = 0; for (const value of state.grid) if (value !== 0) total += 1; return total; }
function countOccupied(state) { let total = 0; for (let index = 0; index < state.grid.length; index += 1) if (isOccupied(state, index)) total += 1; return total; }

export function occupiedCellList(state) {
  const cells = [];
  for (let index = 0; index < state.grid.length; index += 1) {
    if (!isOccupied(state, index)) continue;
    const ix = index % WIDTH;
    cells.push(cellCentre(ix, (index - ix) / WIDTH));
  }
  return cells;
}

// Diagnostic: which measurement is limiting the clearance at this pose.
export function nearestOccupied(state, pose) {
  let best = null;
  for (const cell of occupiedCellList(state)) {
    const gap = distance(cell, pose);
    if (!best || gap < best.gap) best = { kind: "detected", x: cell.x, y: cell.y, gap: round(gap) };
  }
  const span = 1;
  const [cx, cy] = toCell(pose.x, pose.y);
  for (let dx = -span; dx <= span; dx += 1) {
    for (let dy = -span; dy <= span; dy += 1) {
      const ix = cx + dx, iy = cy + dy;
      if (!inside(ix, iy)) continue;
      if (!isWall(state, cellIndex(ix, iy))) continue;
      const cell = cellCentre(ix, iy);
      const gap = distance(cell, pose);
      if (!best || gap < best.gap) best = { kind: "kerb", x: cell.x, y: cell.y, gap: round(gap) };
    }
  }
  return best;
}

// Swept-body test against ground truth, including the moving agent at the time
// each path sample is reached. This is how a real collision is detected.
// Ground truth clearance against every object in the world at that instant.
// Used for scoring and for the sensor comparison, never for the model input.
export function groundTruthClearanceAt(pose, timeS) {
  const body = { x: pose.x, y: pose.y, heading: pose.heading, length: VEHICLE.length, width: VEHICLE.width };
  let clearance = Infinity;
  const corners = [
    [body.length / 2, body.width / 2], [body.length / 2, -body.width / 2],
    [-body.length / 2, body.width / 2], [-body.length / 2, -body.width / 2],
  ].map(([along, across]) => {
    const angle = body.heading * Math.PI / 180;
    return { x: body.x + Math.cos(angle) * along - Math.sin(angle) * across, y: body.y + Math.sin(angle) * along + Math.cos(angle) * across };
  });
  for (const object of worldAt(timeS)) {
    // Separation between two rectangles is the largest per-axis gap; the
    // smallest would be the penetration depth once they overlap.
    let perObject = -Infinity;
    for (const axis of [body.heading, body.heading + 90, object.heading, object.heading + 90].map((degrees) => ({ x: Math.cos(degrees * Math.PI / 180), y: Math.sin(degrees * Math.PI / 180) }))) {
      const project = (rect, point) => point.x * axis.x + point.y * axis.y;
      const objectCorners = (() => {
        const angle = object.heading * Math.PI / 180;
        return [[object.length / 2, object.width / 2], [object.length / 2, -object.width / 2], [-object.length / 2, object.width / 2], [-object.length / 2, -object.width / 2]]
          .map(([along, across]) => ({ x: object.x + Math.cos(angle) * along - Math.sin(angle) * across, y: object.y + Math.sin(angle) * along + Math.cos(angle) * across }));
      })();
      const ap = corners.map((point) => project(body, point)), bp = objectCorners.map((point) => project(object, point));
      perObject = Math.max(perObject, Math.max(Math.min(...bp) - Math.max(...ap), Math.min(...ap) - Math.max(...bp)));
    }
    clearance = Math.min(clearance, perObject);
  }
  return Math.max(-0.75, Math.min(3, clearance));
}

// ---------------------------------------------------------------------------
// Interface to the decision layer.
// ---------------------------------------------------------------------------
function confirmedTracks(state) {
  const clustered = state.tracks.filter((track) => track.samples >= 3 && track.confidence >= 0.4);
  const direct = directDetectedTracks(state);
  const fused = direct.map((item) => {
    const nearby = clustered.filter((track) => distance(item, track) < 1.1);
    if (!nearby.length) return item;
    const fastest = nearby.slice().sort((a, b) => Math.hypot(b.vx, b.vy) - Math.hypot(a.vx, a.vy))[0];
    const moving = Math.hypot(fastest.vx, fastest.vy) > 0.25;
    return moving
      ? { ...fastest, id: item.id, confidence: Math.max(item.confidence, fastest.confidence) }
      : item;
  });
  return [...fused, ...clustered.filter((track) => !direct.some((item) => distance(item, track) < 1.1))];
}

function directDetectedTracks(state) {
  const tracks = [];
  for (const direct of state.directTracks?.values?.() || []) {
    if (direct.hits.length < 6) continue;
    const pseudoState = { ...state, grid: new Float32Array(state.grid.length) };
    const cells = [];
    const seen = new Set();
    for (const hit of direct.hits) {
      const [ix, iy] = toCell(hit.x, hit.y);
      if (!inside(ix, iy)) continue;
      const at = cellIndex(ix, iy);
      if (!seen.has(at)) { seen.add(at); cells.push(at); }
    }
    const box = fitBox(pseudoState, cells);
    if (!box) continue;
    // The occupancy tracker is better at velocity because it follows cluster
    // centres over time. Preserve that measured motion when this instance box
    // replaces a nearby fragmented cluster.
    const motion = state.tracks
      .filter((track) => distance(track, box) < 1.1)
      .sort((a, b) => distance(a, box) - distance(b, box))[0];
    // Returns mostly lie on visible faces. Inflate the measured envelope by a
    // small parking-sensor uncertainty instead of using hidden object bounds.
    tracks.push({
      id: direct.id, type: box.length + box.width > 1.2 ? "vehicle" : "object",
      x: box.x, y: box.y, heading: box.heading,
      length: box.length + 0.28, width: box.width + 0.28,
      vx: motion?.vx || 0, vy: motion?.vy || 0, confidence: Math.min(0.98, 0.45 + direct.hits.length / 100),
      samples: direct.hits.length, firstSeenS: direct.firstSeenS, lastSeenS: direct.lastSeenS,
      ageS: round(state.timeS - direct.firstSeenS),
    });
  }
  return tracks;
}

// Forecast conflicts for every candidate: sweep the real body along the path
// and check it against each track moved forward at its own velocity, grown by
// its position uncertainty. This is a warning to the model, not a veto.
export function annotateCandidates(state, candidates) {
  // The candidate table may use every current onboard track for collision
  // avoidance, including a fresh low-confidence detection. Only mature tracks
  // are promoted to the semantic object list, but waiting three frames before
  // braking would let a close obstacle be driven through.
  const tracks = safetyTracks(state);
  if (!tracks.length) return candidates;
  for (const candidate of Object.values(candidates)) {
    const commandConflict = pathTrackConflict(candidate.path || [], Number(candidate.control?.duration) || 0.5, tracks);
    if (commandConflict) {
      candidate.perceivedTrackCollision = true;
      candidate.perceivedTrackCollisionWith = commandConflict.id;
      candidate.perceivedTrackCollisionInS = round(commandConflict.inS);
      candidate.eligible = false;
    }
    const duration = Number(candidate.horizonDuration ?? candidate.control?.duration) || 0.5;
    const path = candidate.horizonPath || candidate.path || [];
    const conflict = pathTrackConflict(path, duration, tracks);
    if (conflict) {
      candidate.predictedConflict = conflict.id;
      candidate.predictedConflictInS = round(conflict.inS);
    }
  }
  return candidates;
}

export function perceivedTrackCollision(state, path, duration = 0.5) {
  const tracks = safetyTracks(state);
  return pathTrackConflict(path || [], duration, tracks);
}

// Tracks participate in the rollout geometry itself. The distance-field map
// catches arbitrary occupied surfaces; these boxes close holes caused by
// partial views of a vehicle. Both products come only from sensor returns.
function trackClearanceAt(state, pose) {
  const body = { x: pose.x, y: pose.y, heading: pose.heading, length: VEHICLE.length, width: VEHICLE.width };
  let clearance = Infinity;
  for (const track of safetyTracks(state)) {
    const box = { x: track.x, y: track.y, heading: track.heading, length: track.length, width: track.width };
    clearance = Math.min(clearance, rectangleSeparation(body, box));
  }
  return clearance;
}

function rectangleSeparation(a, b) {
  const corners = (rect) => {
    const angle = rect.heading * Math.PI / 180;
    const forward = { x: Math.cos(angle), y: Math.sin(angle) };
    const side = { x: -Math.sin(angle), y: Math.cos(angle) };
    return [
      [rect.length / 2, rect.width / 2], [rect.length / 2, -rect.width / 2],
      [-rect.length / 2, rect.width / 2], [-rect.length / 2, -rect.width / 2],
    ].map(([along, across]) => ({
      x: rect.x + forward.x * along + side.x * across,
      y: rect.y + forward.y * along + side.y * across,
    }));
  };
  const ac = corners(a), bc = corners(b);
  let largestGap = -Infinity;
  for (const degrees of [a.heading, a.heading + 90, b.heading, b.heading + 90]) {
    const axis = { x: Math.cos(degrees * Math.PI / 180), y: Math.sin(degrees * Math.PI / 180) };
    const project = (point) => point.x * axis.x + point.y * axis.y;
    const ap = ac.map(project), bp = bc.map(project);
    largestGap = Math.max(largestGap, Math.max(Math.min(...bp) - Math.max(...ap), Math.min(...ap) - Math.max(...bp)));
  }
  return largestGap;
}

function safetyTracks(state) {
  const direct = directDetectedTracks(state);
  const clustered = state.tracks.filter((track) => track.samples >= 1 && track.confidence >= 0.1);
  const fused = direct.map((item) => {
    const nearby = clustered.filter((track) => distance(item, track) < 1.1);
    if (!nearby.length) return item;
    const fastest = nearby.slice().sort((a, b) => Math.hypot(b.vx, b.vy) - Math.hypot(a.vx, a.vy))[0];
    const moving = Math.hypot(fastest.vx, fastest.vy) > 0.25;
    return moving
      ? { ...fastest, id: item.id, confidence: Math.max(item.confidence, fastest.confidence) }
      : item;
  });
  return [...fused, ...clustered.filter((track) => !direct.some((item) => distance(item, track) < 1.1))];
}

function pathTrackConflict(path, duration, tracks) {
    let conflict = null;
    const densePath = densifyCollisionPath(path[0] || { x: 0, y: 0, heading: 0 }, path);
    for (let index = 0; index < densePath.length && !conflict; index += 1) {
      const sample = densePath[index];
      const aheadS = densePath.length <= 1 ? 0 : (index / (densePath.length - 1)) * duration;
      const body = { x: sample.x, y: sample.y, heading: sample.heading, length: VEHICLE.length, width: VEHICLE.width };
      for (const track of tracks) {
        const positionGap = Math.hypot(sample.x - track.x, sample.y - track.y);
        if (positionGap > 3.5) continue;
        // Occupancy-grid rollout remains the primary geometry. Tracks close a
        // sensor-latency gap. A small inward tolerance absorbs the detector's
        // size noise so adjacent legal parking paths are not rejected wholesale.
        const tolerance = track.confidence >= 0.8 ? 0.03 : 0.08;
        const box = {
          x: track.x + track.vx * aheadS, y: track.y + track.vy * aheadS,
          heading: track.heading,
          length: Math.max(0.2, track.length - tolerance * 2),
          width: Math.max(0.2, track.width - tolerance * 2),
        };
        if (rectanglesOverlap(body, box)) { conflict = { id: track.id, inS: aheadS }; break; }
      }
    }
    return conflict;
}

// Exactly what the model is told about the world. Nothing here is ground truth.
export function perceivedEnvironment(state) {
  const tracks = confirmedTracks(state);
  return {
    source: 'onboard sensors: 360 degree range finder plus eight short-range sensors',
    ground_truth_available: false,
    sensor_model: {
      range_finder_rays: SENSOR.ring.rays, max_range_m: SENSOR.ring.maxRangeM,
      range_noise_m: SENSOR.ring.rangeNoiseM, dropout: SENSOR.ring.dropout,
      short_range_sensors: SENSOR.ultrasonic.count, short_range_max_m: SENSOR.ultrasonic.maxRangeM,
    },
    occupancy_grid: {
      cell_m: GRID.cellM, occupied_cells: countOccupied(state), mapped_cells: countMapped(state),
      memory_s: GRID.memoryS, occupied_threshold: GRID.occupiedThreshold,
    },
    safety_margin_m: SAFETY_MARGIN,
    tracked_objects: tracks.map((track) => ({
      id: track.id, type: track.type,
      pose: { x: track.x, y: track.y, heading_deg: track.heading },
      size_m: [track.width, track.length],
      velocity_mps: [track.vx, track.vy],
      confidence: track.confidence, seen_for_s: track.ageS, last_seen_s_ago: round(state.timeS - track.lastSeenS),
      predicted: predictionsFor(track),
    })),
    notes: 'Detections are measurements with noise and gaps. Space you have not observed is unknown, not free. Objects can be missed, partially seen or mis-sized, and tracked objects carry a predicted future position with growing uncertainty.',
  };
}

// Snapshot for the 3D view: what the car currently believes it has mapped.
export function sceneSnapshot(state) {
  return {
    timeS: state.timeS,
    cells: occupiedCellList(state).map((cell) => [round(cell.x), round(cell.y)]),
    tracks: confirmedTracks(state).map((track) => ({
      id: track.id, x: track.x, y: track.y, heading: round(track.heading),
      length: track.length, width: track.width, moving: Math.hypot(track.vx, track.vy) > 0.25,
      predicted: predictionsFor(track).map((item) => [item.x, item.y, item.uncertaintyM]),
    })),
  };
}

export function actualCollision(pose, path, durationS, startTimeS) {
  const densePath = densifyCollisionPath(pose, path);
  const samples = densePath.length;
  for (let index = 0; index < samples; index += 1) {
    const at = densePath[index];
    const timeS = startTimeS + (samples <= 1 ? 0 : (index / (samples - 1)) * durationS);
    const body = { x: at.x, y: at.y, heading: at.heading, length: VEHICLE.length, width: VEHICLE.width };
    for (const object of worldAt(timeS)) {
      // The simulator must stop before the rendered bodies touch.  Using the
      // same safety margin as the candidate rollout also absorbs small gaps
      // between the physics boxes and the rounded visual meshes.  This check
      // only scores/vetoes the already selected command; its result is never
      // sent back to either model as planning information.
      if (rectangleSeparation(body, object) < SAFETY_MARGIN) {
        const sourceProgress = (samples <= 1 ? 0 : index / (samples - 1)) * Math.max(0, path.length - 1);
        return {
          objectId: object.id,
          timeS: round(timeS),
          clearanceM: round(rectangleSeparation(body, object)),
          pathIndex: Math.min(path.length - 1, Math.ceil(sourceProgress)),
          safePathIndex: 0,
        };
      }
    }
  }
  return null;
}

// Keep collision detection independent of the path's rendering sample rate.
// Interpolate at most 2 cm / 1 degree so the full body cannot tunnel through
// another vehicle between two bicycle-model samples.
function densifyCollisionPath(origin, path) {
  const source = Array.isArray(path) && path.length ? path : [origin];
  const out = [{ ...source[0] }];
  for (let index = 1; index < source.length; index += 1) {
    const a = source[index - 1], b = source[index];
    const headingDelta = normalizeAngle(Number(b.heading) - Number(a.heading));
    const steps = Math.max(
      1,
      Math.ceil(Math.hypot(Number(b.x) - Number(a.x), Number(b.y) - Number(a.y)) / 0.02),
      Math.ceil(Math.abs(headingDelta)),
    );
    for (let step = 1; step <= steps; step += 1) {
      const mix = step / steps;
      out.push({
        x: Number(a.x) + (Number(b.x) - Number(a.x)) * mix,
        y: Number(a.y) + (Number(b.y) - Number(a.y)) * mix,
        heading: normalizeAngle(Number(a.heading) + headingDelta * mix),
      });
    }
  }
  return out;
}

function rectanglesOverlap(a, b, tolerance = 0) {
  const axes = [a.heading, a.heading + 90, b.heading, b.heading + 90].map((degrees) => ({ x: Math.cos(degrees * Math.PI / 180), y: Math.sin(degrees * Math.PI / 180) }));
  const corners = (rect) => {
    const angle = rect.heading * Math.PI / 180, forward = { x: Math.cos(angle), y: Math.sin(angle) }, side = { x: -Math.sin(angle), y: Math.cos(angle) };
    return [[rect.length / 2, rect.width / 2], [rect.length / 2, -rect.width / 2], [-rect.length / 2, rect.width / 2], [-rect.length / 2, -rect.width / 2]]
      .map(([along, across]) => ({ x: rect.x + forward.x * along + side.x * across, y: rect.y + forward.y * along + side.y * across }));
  };
  const ac = corners(a), bc = corners(b);
  for (const axis of axes) {
    const ap = ac.map((point) => point.x * axis.x + point.y * axis.y), bp = bc.map((point) => point.x * axis.x + point.y * axis.y);
    if (Math.min(...ap) > Math.max(...bp) + tolerance || Math.min(...bp) > Math.max(...ap) + tolerance) return false;
  }
  return true;
}

import { angleError, clamp, distance, round, roundPose } from "./math.js";
export { scenarioTasks } from "../scenarios.js";

// Directed route corridors only measure destination progress. They never pick
// a gear, steering angle, speed, or candidate for either model.
  const NAVIGATION_STAGES = {
  "offset-bay": [
    {
      id: "approach_bay", label: "Follow the open aisle past the bay and cross the reverse-entry setup gate",
      route: [{ x: 2.65, y: 0.25 }, { x: 1.75, y: 0.72 }, { x: -1.15, y: 0.95 }],
      gateBeforeEndM: 0, gateHalfWidthM: 0.95,
    },
    { id: "park", label: "Reverse into the target bay and align", final: true },
  ],
  "tight-corner": [
    {
      id: "recover_aisle", label: "Follow the recovery corridor out of the tight corner and cross into the open aisle",
      route: [{ x: 4.05, y: 0.65 }, { x: 3.5, y: 0.18 }, { x: 2.65, y: 0.25 }],
      gateBeforeEndM: 0, gateHalfWidthM: 0.75,
    },
    {
      id: "approach_bay", label: "Follow the open aisle past the bay and cross the reverse-entry setup gate",
      route: [{ x: 2.65, y: 0.25 }, { x: 1.75, y: 0.72 }, { x: -1.15, y: 0.95 }],
      gateBeforeEndM: 0, gateHalfWidthM: 0.95,
    },
    { id: "park", label: "Reverse into the target bay and align", final: true },
  ],
  "reverse-entry": [
    { id: "park", label: "Reverse into the target bay and align", final: true },
  ],
  "left-offset": [
    {
      id: "approach_bay", label: "Follow the open aisle past the bay and cross the reverse-entry setup gate",
      route: [{ x: -2.65, y: 0.25 }, { x: -1.75, y: 0.72 }, { x: 1.15, y: 0.95 }],
      gateBeforeEndM: 0, gateHalfWidthM: 0.95,
    },
    { id: "park", label: "Reverse into the target bay and align", final: true },
  ],
  "forward-entry": [
    { id: "park", label: "Drive forward into the target bay and align", final: true },
  ],
  "wide-angle": [
    {
      id: "reach_aisle", label: "Enter the open aisle and cross the approach gate",
      route: [{ x: -4.0, y: 2.15 }, { x: -3.25, y: 1.05 }, { x: -2.65, y: 0.25 }],
      gateBeforeEndM: 0, gateHalfWidthM: 0.85,
    },
    {
      id: "approach_bay", label: "Follow the aisle past the bay and cross the reverse-entry setup gate",
      route: [{ x: -2.65, y: 0.25 }, { x: -1.75, y: 0.72 }, { x: 1.15, y: 0.95 }],
      gateBeforeEndM: 0, gateHalfWidthM: 0.95,
    },
    { id: "park", label: "Reverse into the target bay and align", final: true },
  ],
};

function customStages(pose, target) {
  const heading = target.heading * Math.PI / 180;
  const setup = { x: target.x + Math.cos(heading) * 2.85, y: target.y + Math.sin(heading) * 2.85 };
  if (distance(pose, setup) < 0.9 || (distance(pose, target) < 1.7 && angleError(pose.heading, target.heading) < 40)) {
    return [{ id: "park", label: "Enter the target bay and align", final: true }];
  }
  return [
    {
      id: "approach_bay", label: "Reach the open setup point in front of the target bay",
      route: [{ x: pose.x, y: pose.y }, setup], gateBeforeEndM: 0, gateHalfWidthM: 0.9,
    },
    { id: "park", label: "Enter the target bay and align", final: true },
  ];
}

function routeGeometry(points) {
  const route = points.map((point) => ({ x: Number(point.x), y: Number(point.y) }));
  let lengthM = 0;
  const segments = [];
  for (let index = 0; index < route.length - 1; index += 1) {
    const a = route[index], b = route[index + 1];
    const dx = b.x - a.x, dy = b.y - a.y;
    const length = Math.hypot(dx, dy);
    if (length <= 1e-6) continue;
    segments.push({ a, b, dx, dy, length, startS: lengthM, heading: Math.atan2(dy, dx) * 180 / Math.PI });
    lengthM += length;
  }
  return { points: route, segments, lengthM };
}

function routeMeasurement(pose, points) {
  const geometry = routeGeometry(points);
  let nearest = null;
  for (const segment of geometry.segments) {
    const relativeX = pose.x - segment.a.x, relativeY = pose.y - segment.a.y;
    const t = clamp((relativeX * segment.dx + relativeY * segment.dy) / (segment.length * segment.length), 0, 1);
    const x = segment.a.x + segment.dx * t, y = segment.a.y + segment.dy * t;
    const errorX = pose.x - x, errorY = pose.y - y;
    const distanceM = Math.hypot(errorX, errorY);
    if (!nearest || distanceM < nearest.distanceM) {
      nearest = {
        distanceM,
        signedLateralM: (segment.dx * errorY - segment.dy * errorX) / segment.length,
        progressM: segment.startS + segment.length * t,
        headingDeg: segment.heading,
      };
    }
  }
  if (!nearest) {
    const point = geometry.points[0] || pose;
    nearest = { distanceM: distance(pose, point), signedLateralM: 0, progressM: 0, headingDeg: pose.heading };
  }
  return {
    ...nearest, routeLengthM: geometry.lengthM,
    // A vehicle beside the route end has not reached its gate. Include lateral
    // displacement so remainingM cannot become zero outside the corridor.
    remainingM: Math.hypot(
      Math.max(0, geometry.lengthM - nearest.progressM),
      nearest.signedLateralM,
    ),
    headingErrorDeg: angleError(pose.heading, nearest.headingDeg),
  };
}

function crossedRouteGate(stage, pose) {
  const segment = routeGeometry(stage.route).segments.at(-1);
  if (!segment) return true;
  const fromStartX = pose.x - segment.a.x, fromStartY = pose.y - segment.a.y;
  const alongM = (fromStartX * segment.dx + fromStartY * segment.dy) / segment.length;
  const lateralM = (segment.dx * fromStartY - segment.dy * fromStartX) / segment.length;
  return alongM >= segment.length - Number(stage.gateBeforeEndM || 0)
    && Math.abs(lateralM) <= Number(stage.gateHalfWidthM || 1);
}

function reachedRouteEnd(stage, pose) {
  const measurement = routeMeasurement(pose, stage.route);
  return measurement.remainingM <= Number(stage.gateHalfWidthM || 1);
}

export function navigationMeasurement(navigation, pose) {
  if (!navigation) return null;
  if (navigation.finalStage || !navigation.route) {
    const remainingM = distance(pose, navigation.goal);
    return { distanceM: remainingM, remainingM, progressM: 0, lateralErrorM: 0, headingErrorDeg: angleError(pose.heading, navigation.goal.heading) };
  }
  const measurement = routeMeasurement(pose, navigation.route);
  return {
    distanceM: measurement.remainingM, remainingM: measurement.remainingM, progressM: measurement.progressM,
    lateralErrorM: measurement.signedLateralM, headingErrorDeg: measurement.headingErrorDeg, routeHeadingDeg: measurement.headingDeg,
  };
}

export function navigationState(scenario, pose, target, previousStage = 0) {
  const stages = scenario === "custom"
    ? customStages(pose, target)
    : (NAVIGATION_STAGES[scenario] || NAVIGATION_STAGES["offset-bay"]);
  let index = clamp(Math.floor(Number(previousStage) || 0), 0, stages.length - 1);
  while (index < stages.length - 1) {
    const stage = stages[index];
    // A route stage is complete once the car reaches the bounded end-gate
    // neighbourhood. Requiring it to cross the infinite gate plane made a car
    // that reached the setup area from the side oscillate around the endpoint.
    if (!stage.route || (!crossedRouteGate(stage, pose) && !reachedRouteEnd(stage, pose))) break;
    index += 1;
  }
  const stage = stages[index];
  const finalRoutePoint = stage.route?.at(-1);
  const previousRoutePoint = stage.route?.at(-2);
  const routeHeading = finalRoutePoint && previousRoutePoint
    ? Math.atan2(finalRoutePoint.y - previousRoutePoint.y, finalRoutePoint.x - previousRoutePoint.x) * 180 / Math.PI
    : target.heading;
  const goal = stage.final ? target : { ...finalRoutePoint, heading: routeHeading };
  const navigation = {
    stageIndex: index, stageCount: stages.length, stageId: stage.id, label: stage.label,
    goal: roundPose(goal), route: stage.route?.map((point) => ({ x: round(point.x), y: round(point.y) })) || null,
    corridorHalfWidthM: stage.gateHalfWidthM ?? null, finalStage: Boolean(stage.final),
  };
  const measurement = navigationMeasurement(navigation, pose);
  return {
    ...navigation,
    distanceM: round(measurement.distanceM), remainingM: round(measurement.remainingM), progressM: round(measurement.progressM),
    lateralErrorM: round(measurement.lateralErrorM), headingErrorDeg: round(measurement.headingErrorDeg),
    routeHeadingDeg: measurement.routeHeadingDeg == null ? null : round(measurement.routeHeadingDeg),
  };
}

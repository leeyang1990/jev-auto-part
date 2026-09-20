export { VEHICLE } from "../public/parking-goal.js";

export const OBSTACLES = [
  { id: "parked_car_left", type: "parked_vehicle", x: -2.05, y: -2.45, heading: 90, length: 1.75, width: 0.82 },
  { id: "parked_car_right", type: "parked_vehicle", x: 2.1, y: -2.45, heading: 90, length: 1.75, width: 0.82 },
];

export const LOT_BOUNDS = { minX: -5.18, maxX: 5.18, minY: -3.82, maxY: 3.82 };
export const SAFETY_MARGIN = 0.08;
export const STEERING_OPTIONS = [-32, -20, -10, 0, 10, 20, 32];
export const SPEED_OPTIONS = [0.22, 0.42, 0.62];
export const FINE_SPEED_OPTIONS = [0.08, 0.12];
export const MIN_CONTROL_SPEED_MPS = 0.05;
// The browser requests the next model decision while the current control is
// executing. A one-second approach window raises the closed-loop frequency
// while still leaving enough motion in each rollout for the model to distinguish
// useful progress. Near the bay, retain the validated fine-control windows.
export const CONTROL_HORIZON_S = 1.0;
export const FINE_CONTROL_HORIZON_S = 0.6;
export const MICRO_CONTROL_HORIZON_S = 0.3;
export const RECOVERY_CONTROL_HORIZON_S = 0.9;
// Shared trajectory generation. Both engines receive the same bounded set of
// multi-segment trajectories. These values control search breadth only; they
// never select a trajectory for either model.
export const TRAJECTORY = {
  candidateLimit: 18,
  steeringRateDegPerS: 72,
  accelerationMps2: 0.55,
  // During recovery, keep enough physical motion to leave a repeated pose.
  recoveryMinDistanceM: 0.42,
  recoveryMaxDistanceM: 1.35,
  recoveryMinHeadingChangeDeg: 10,
};
export const CANDIDATE_COUNT = TRAJECTORY.candidateLimit + 1;

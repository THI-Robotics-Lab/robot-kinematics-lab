import { analytic2R, residual, type IKResult } from "./ik";
import {
  fromXYZRPY,
  getPosition,
  solveLinear,
  type Mat4,
  type Vec3,
} from "./math";
import {
  analyze,
  ANGULAR_LENGTH,
  projectNullspace,
  taskJacobian,
} from "./analysis";
import type { Robot } from "./models";

export interface CartesianPath {
  p0: Vec3;
  direction: Vec3;
  length: number;
}
export const DEMO_SAMPLE_TIME = 0.02; // seconds, shared by playback and numerical tests
export const lecturePath: CartesianPath = {
  p0: [0.35, 0.2, 0],
  direction: [0.8, 0.6, 0],
  length: 0.4,
};
export function pathTarget(path: CartesianPath, s: number): Mat4 {
  if (
    !Number.isFinite(s) ||
    s < 0 ||
    s > path.length ||
    Math.abs(Math.hypot(...path.direction) - 1) > 1e-12
  )
    throw new Error("Invalid path coordinate or non-unit direction");
  return fromXYZRPY(
    path.p0.map((x, i) => x + path.direction[i] * s) as Vec3,
    [0, 0, 0],
  );
}
export function pathStep(
  robot: Robot,
  path: CartesianPath,
  s: number,
  seed: number[],
  branch: "positive" | "negative",
): IKResult {
  return analytic2R(robot, pathTarget(path, s), seed, branch);
}

export const REDUNDANCY_POSITION_BOUND = 5e-5; // m, explicit demonstration acceptance bounds
export const REDUNDANCY_ROTATION_BOUND = 5e-4; // rad
export interface RedundancyStep {
  q: number[];
  accepted: boolean;
  reason: string;
  positionError: number;
  orientationError: number;
}
/** A small null-space predictor plus full-pose Newton correction. No solver
 * restarts are allowed: a failed step is rejected rather than jumping branches.
 * This maintains local continuity; it does not claim exact pose preservation.
 */
export function redundancyStep(
  robot: Robot,
  q: number[],
  reference: Mat4,
  dt: number,
  direction = 1,
): RedundancyStep {
  const finish = (next: number[], accepted: boolean, reason: string) => ({
    q: next,
    accepted,
    reason,
    ...residual(robot, next, reference),
  });
  if (
    !Number.isFinite(dt) ||
    dt <= 0 ||
    dt > 0.05 ||
    Math.abs(direction) > 1 ||
    !Number.isFinite(direction)
  )
    throw new Error("Invalid null-space time step or direction");
  if (robot.joints.length <= 6)
    return finish(q, false, "No redundant full-pose degree of freedom.");
  if (
    !q.every((x, i) => x >= robot.joints[i].lower && x <= robot.joints[i].upper)
  )
    return finish(q, false, "Initial configuration lies outside joint limits.");
  const J = taskJacobian(robot, q, "pose"),
    info = analyze(robot, q, "pose");
  if (!info.converged || info.rank < 6)
    return finish(
      q,
      false,
      "Full-pose Jacobian is numerically rank deficient.",
    );
  if (Math.abs(direction) < 1e-6) {
    const e = residual(robot, q, reference);
    const accepted =
      e.positionError <= REDUNDANCY_POSITION_BOUND &&
      e.orientationError <= REDUNDANCY_ROTATION_BOUND;
    return finish(
      q,
      accepted,
      accepted
        ? "Turning point: instantaneous null-space speed is zero."
        : "Stopped: reference pose error exceeds the acceptance bounds.",
    );
  }
  // Drive q1 through the null space; a periodic direction reverses it in the UI.
  const g = q.map((_, i) => (i === 0 ? direction * 0.45 : 0));
  const velocity = projectNullspace(J, g);
  if (Math.hypot(...velocity) < 1e-8)
    return finish(q, false, "Selected null-space direction has vanished.");
  for (const scale of [1, 0.5, 0.25, 0.125]) {
    let trial = q.map((x, i) => x + dt * scale * velocity[i]);
    for (let iteration = 0; iteration < 8; iteration++) {
      const e = residual(robot, trial, reference);
      if (e.positionError < 1e-8 && e.orientationError < 1e-7) break;
      const A = taskJacobian(robot, trial, "pose");
      const error = [...e.dp, ...e.dr.map((x) => x * ANGULAR_LENGTH)];
      const normal = A.map((row, i) =>
        A.map(
          (other, j) =>
            row.reduce((s, x, k) => s + x * other[k], 0) +
            (i === j ? 1e-10 : 0),
        ),
      );
      const v = solveLinear(normal, error);
      if (!v) break;
      trial = trial.map(
        (x, i) => x + A.reduce((s, row, k) => s + row[i] * v[k], 0),
      );
    }
    const inLimits = trial.every(
      (x, i) =>
        Number.isFinite(x) &&
        x >= robot.joints[i].lower &&
        x <= robot.joints[i].upper,
    );
    const continuous =
      Math.max(...trial.map((x, i) => Math.abs(x - q[i]))) <= 0.03;
    const e = residual(robot, trial, reference);
    if (
      inLimits &&
      continuous &&
      e.positionError <= REDUNDANCY_POSITION_BOUND &&
      e.orientationError <= REDUNDANCY_ROTATION_BOUND
    )
      return finish(
        trial,
        true,
        "Local null-space step with full-pose correction.",
      );
  }
  return finish(
    q,
    false,
    "Stopped: joint limit, continuity or pose-error bound would be exceeded.",
  );
}

export function pathPoints(path: CartesianPath, count = 60): Vec3[] {
  return Array.from({ length: count + 1 }, (_, i) =>
    getPosition(pathTarget(path, (path.length * i) / count)),
  );
}

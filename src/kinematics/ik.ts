import {
  getPosition,
  orientationError,
  solveLinear,
  vlen,
  vsub,
  wrapAngle,
  type Mat4,
} from "./math";
import { clampJoints, forward, jacobian, reachBound } from "./fk";
import type { Robot } from "./models";

export type Task = "position" | "pose";
export type IKStatus =
  | "converged"
  | "unreachable"
  | "joint-limits"
  | "not-converged"
  | "unsupported";
export interface IKResult {
  q: number[];
  status: IKStatus;
  iterations: number;
  attempts: number;
  positionError: number;
  orientationError: number;
}
export const POSITION_TOLERANCE = 1e-4; // m
export const ROTATION_TOLERANCE = 1e-3; // rad
const ROTATION_WEIGHT = 0.35; // m/rad: explicit characteristic length

export function residual(robot: Robot, q: number[], target: Mat4) {
  const current = forward(robot, q).pose;
  const dp = vsub(getPosition(target), getPosition(current));
  const dr = orientationError(current, target);
  return { dp, dr, positionError: vlen(dp), orientationError: vlen(dr) };
}

/** Two exact elbow branches, with nearest-seed selection or explicit branch. */
export function analytic2R(
  robot: Robot,
  target: Mat4,
  seed: number[],
  elbow: "nearest" | "positive" | "negative" = "nearest",
): IKResult {
  if (!robot.planarLengths)
    throw new Error("Analytic solver requires planar 2R");
  if (target.length !== 16 || target.some((x) => !Number.isFinite(x)))
    throw new Error("Invalid target transform");
  const [l1, l2] = robot.planarLengths;
  const [x, y, z] = getPosition(target);
  const q0 = clampJoints(robot, seed);
  const result = (q: number[], status: IKStatus): IKResult => ({
    q,
    status,
    iterations: 0,
    attempts: 1,
    ...residual(robot, q, target),
  });
  const c = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);
  if (Math.abs(z) > POSITION_TOLERANCE || c < -1 - 1e-10 || c > 1 + 1e-10)
    return result(q0, "unreachable");
  const angle = Math.acos(Math.max(-1, Math.min(1, c)));
  const signs =
    elbow === "nearest" ? [1, -1] : elbow === "positive" ? [1] : [-1];
  const candidates = signs
    .map((sign) => {
      const q2 = sign * angle;
      return [
        wrapAngle(
          Math.atan2(y, x) -
            Math.atan2(l2 * Math.sin(q2), l1 + l2 * Math.cos(q2)),
        ),
        q2,
      ];
    })
    .filter((q) =>
      q.every(
        (x, i) =>
          x >= robot.joints[i].lower - 1e-10 &&
          x <= robot.joints[i].upper + 1e-10,
      ),
    );
  if (!candidates.length) return result(q0, "joint-limits");
  const distance = (q: number[]) =>
    q.reduce((s, x, i) => s + wrapAngle(x - q0[i]) ** 2, 0);
  candidates.sort((a, b) => distance(a) - distance(b));
  return result(clampJoints(robot, candidates[0]), "converged");
}

/** Bounded deterministic multi-start DLS with line search and joint projection.
 * No global reachability claim follows from a local solver's failure.
 */
export function inverse(
  robot: Robot,
  target: Mat4,
  seed: number[],
  task: Task = "position",
): IKResult {
  if (target.length !== 16 || target.some((x) => !Number.isFinite(x)))
    throw new Error("Invalid target transform");
  const start = clampJoints(robot, seed);
  const finish = (
    q: number[],
    status: IKStatus,
    iterations: number,
    attempts: number,
  ): IKResult => ({
    q,
    status,
    iterations,
    attempts,
    ...residual(robot, q, target),
  });
  if (robot.planarLengths)
    return task === "pose"
      ? finish(start, "unsupported", 0, 0)
      : analytic2R(robot, target, start);
  if (vlen(getPosition(target)) > reachBound(robot) + POSITION_TOLERANCE)
    return finish(start, "unreachable", 0, 0);

  const errorVector = (q: number[]) => {
    const e = residual(robot, q, target);
    return task === "position"
      ? e.dp
      : [...e.dp, ...e.dr.map((x) => x * ROTATION_WEIGHT)];
  };
  const cost = (q: number[]) => errorVector(q).reduce((s, x) => s + x * x, 0);
  let best = start,
    bestCost = cost(start),
    total = 0;
  for (let attempt = 0; attempt < 5; attempt++) {
    // Home + deterministic alternate seeds also escape exactly singular starts.
    let q =
      attempt === 0
        ? start.slice()
        : clampJoints(
            robot,
            robot.home.map((x, i) =>
              attempt === 1 ? x : x + 0.65 * Math.sin((attempt + 1) * (i + 2)),
            ),
          );
    let damping = 0.015;
    for (let iteration = 0; iteration < 180; iteration++) {
      const e = residual(robot, q, target);
      const score = cost(q);
      if (score < bestCost) {
        best = q.slice();
        bestCost = score;
      }
      if (
        e.positionError <= POSITION_TOLERANCE &&
        (task === "position" || e.orientationError <= ROTATION_TOLERANCE)
      ) {
        return finish(q, "converged", total, attempt + 1);
      }
      total++;
      const J = jacobian(robot, q)
        .slice(0, task === "position" ? 3 : 6)
        .map((row, i) => row.map((x) => x * (i < 3 ? 1 : ROTATION_WEIGHT)));
      const A = J.map((row, r) =>
        J.map(
          (other, c) =>
            row.reduce((s, x, i) => s + x * other[i], 0) +
            (r === c ? damping * damping : 0),
        ),
      );
      const v = solveLinear(A, errorVector(q));
      if (!v) break;
      const dq = q.map((_, c) => J.reduce((s, row, r) => s + row[c] * v[r], 0));
      const stepScale = Math.min(
        1,
        0.25 / Math.max(1e-12, ...dq.map(Math.abs)),
      );
      let accepted = false;
      for (const alpha of [1, 0.5, 0.25, 0.125, 0.0625]) {
        const trial = clampJoints(
          robot,
          q.map((x, i) => x + alpha * stepScale * dq[i]),
        );
        if (cost(trial) < score - 1e-16) {
          q = trial;
          accepted = true;
          break;
        }
      }
      if (accepted) damping = Math.max(0.002, damping * 0.8);
      else {
        damping *= 3;
        if (damping > 3) break;
      }
    }
    if (cost(q) < bestCost) {
      best = q.slice();
      bestCost = cost(q);
    }
    // The final allowed step may itself have crossed the tolerance threshold.
    const end = residual(robot, q, target);
    if (
      end.positionError <= POSITION_TOLERANCE &&
      (task === "position" || end.orientationError <= ROTATION_TOLERANCE)
    ) return finish(q, "converged", total, attempt + 1);
  }
  return finish(best, "not-converged", total, 5);
}

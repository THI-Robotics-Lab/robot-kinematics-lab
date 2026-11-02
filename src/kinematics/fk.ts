import {
  fromAxisAngle,
  getPosition,
  identity,
  multiply,
  transformDir,
  vcross,
  vsub,
  vlen,
  type Mat4,
} from "./math";
import type { Robot } from "./models";

export function clampJoints(robot: Robot, q: number[]): number[] {
  if (q.length !== robot.joints.length || q.some((x) => !Number.isFinite(x)))
    throw new Error("Invalid joint vector");
  return q.map((x, i) =>
    Math.max(robot.joints[i].lower, Math.min(robot.joints[i].upper, x)),
  );
}

// FK intentionally accepts out-of-limit finite angles for mathematical evaluation.
// UI and IK enforce physical bounds through clampJoints.
export function forward(robot: Robot, q: number[]) {
  if (q.length !== robot.joints.length || q.some((x) => !Number.isFinite(x)))
    throw new Error("Invalid joint vector");
  let pose = identity();
  const pivots: Mat4[] = [],
    frames: Mat4[] = [];
  for (const [i, joint] of robot.joints.entries()) {
    const pivot = multiply(pose, joint.origin);
    pivots.push(pivot);
    pose = multiply(
      multiply(pivot, fromAxisAngle([0, 0, 1], q[i])),
      joint.after,
    );
    frames.push(pose);
  }
  return { pose: multiply(pose, robot.tool), pivots, frames };
}

/** Geometric Jacobian in base/world coordinates: [linear m/rad; angular rad/rad]. */
export function jacobian(robot: Robot, q: number[]): number[][] {
  const { pose, pivots } = forward(robot, q);
  const p = getPosition(pose);
  const columns = pivots.map((t) => {
    const axis = transformDir(t, [0, 0, 1]);
    return [...vcross(axis, vsub(p, getPosition(t))), ...axis];
  });
  return Array.from({ length: 6 }, (_, r) => columns.map((c) => c[r]));
}

/** Conservative sphere bound from triangle inequality; outside implies unreachable. */
export function reachBound(robot: Robot): number {
  return robot.joints.reduce(
    (s, j) => s + vlen(getPosition(j.origin)) + vlen(getPosition(j.after)),
    vlen(getPosition(robot.tool)),
  );
}

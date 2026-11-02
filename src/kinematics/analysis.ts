import { jacobian } from "./fk";
import type { Robot } from "./models";
import type { Task } from "./ik";

export const ANGULAR_LENGTH = 0.35; // m/rad, same convention as spatial IK

/** Small one-sided Jacobi SVD of J^T, avoiding loss of small singular values
 * from explicitly forming J J^T. Columns of left are task-space directions;
 * right contains the corresponding joint-space directions (one per value).
 * Intended for these small teaching Jacobians, not a general LAPACK replacement.
 */
export function spectrum(J: number[][]) {
  const m = J.length,
    n = J[0].length;
  const columns = J.map((row) => row.slice());
  const directions = Array.from({ length: m }, (_, i) =>
    Array.from({ length: m }, (_, j) => Number(i === j)),
  );
  let converged = false;
  for (let sweep = 0; sweep < 80; sweep++) {
    let changed = false;
    for (let p = 0; p < m; p++)
      for (let q = p + 1; q < m; q++) {
        const a = columns[p],
          b = columns[q];
        const aa = a.reduce((s, x) => s + x * x, 0),
          bb = b.reduce((s, x) => s + x * x, 0);
        const ab = a.reduce((s, x, i) => s + x * b[i], 0);
        if (
          Math.abs(ab) <=
          Number.EPSILON * Math.sqrt(aa * bb) * Math.max(m, n)
        )
          continue;
        const tau = (bb - aa) / (2 * ab);
        const t = (tau >= 0 ? 1 : -1) / (Math.abs(tau) + Math.hypot(1, tau));
        const c = 1 / Math.hypot(1, t),
          s = c * t;
        for (let i = 0; i < n; i++) {
          const x = a[i],
            y = b[i];
          a[i] = c * x - s * y;
          b[i] = s * x + c * y;
        }
        for (let i = 0; i < m; i++) {
          const x = directions[p][i],
            y = directions[q][i];
          directions[p][i] = c * x - s * y;
          directions[q][i] = s * x + c * y;
        }
        changed = true;
      }
    if (!changed) {
      converged = true;
      break;
    }
  }
  const order = columns
    .map((col, i) => ({ value: Math.hypot(...col), i }))
    .sort((a, b) => b.value - a.value);
  const values = order.map((x) => x.value);
  const tolerance = Number.EPSILON * Math.max(m, n) * values[0];
  const rank = values.filter((x) => x > tolerance).length;
  return {
    values,
    rank,
    tolerance,
    converged,
    left: order.map(({ i }) => directions[i]),
    right: order.map(({ i, value }) =>
      columns[i].map((x) => (value > tolerance ? x / value : 0)),
    ),
    ratio: values[0] > 0 ? values[m - 1] / values[0] : 0,
    manipulability: values.reduce((s, x) => s * x, 1),
  };
}

export function taskJacobian(robot: Robot, q: number[], task: Task) {
  const J = jacobian(robot, q);
  if (robot.planarLengths) return J.slice(0, 2);
  return task === "position"
    ? J.slice(0, 3)
    : J.map((row, i) => row.map((x) => x * (i < 3 ? 1 : ANGULAR_LENGTH)));
}

export function analyze(robot: Robot, q: number[], task: Task) {
  const J = taskJacobian(robot, q, task);
  const svd = spectrum(J);
  return { J, ...svd, dimensions: J.length, nullity: q.length - svd.rank };
}

/** Orthogonal joint-rate projection (I - J^+ J) g via the SVD directions. */
export function projectNullspace(J: number[][], g: number[]) {
  const svd = spectrum(J);
  const projected = g.slice();
  svd.right.forEach((v, i) => {
    if (svd.values[i] <= svd.tolerance) return;
    const dot = v.reduce((s, x, k) => s + x * g[k], 0);
    v.forEach((x, k) => {
      projected[k] -= x * dot;
    });
  });
  return projected;
}

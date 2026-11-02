import { describe, expect, it } from "vitest";
import {
  analyze,
  projectNullspace,
  spectrum,
  taskJacobian,
} from "../src/kinematics/analysis";
import { clampJoints, forward, jacobian } from "../src/kinematics/fk";
import {
  DEMO_SAMPLE_TIME,
  pathStep,
  lecturePath,
  pathTarget,
  redundancyStep,
  REDUNDANCY_POSITION_BOUND,
  REDUNDANCY_ROTATION_BOUND,
} from "../src/kinematics/motion";
import { panda, planar, six } from "../src/kinematics/models";
import { getPosition } from "../src/kinematics/math";

describe("Task-specific Jacobian spectra", () => {
  it("resolves known singular values, including small ones without squaring them away", () => {
    const root = Math.SQRT1_2;
    const result = spectrum([
      [3 * root, 3 * root, 0],
      [-1e-10 * root, 1e-10 * root, 0],
    ]);
    expect(result.values[0]).toBeCloseTo(3, 12);
    expect(result.values[1] / 1e-10).toBeCloseTo(1, 12);
    expect(result.rank).toBe(2);
  });
  it("reconstructs the Jacobian from its singular directions", () => {
    for (const robot of [planar, six, panda])
      for (const task of ["position", "pose"] as const) {
        const J = taskJacobian(robot, robot.home, task),
          svd = spectrum(J);
        expect(svd.converged).toBe(true);
        for (let r = 0; r < J.length; r++)
          for (let c = 0; c < robot.home.length; c++) {
            const recovered = svd.values.reduce(
              (s, sigma, i) => s + sigma * svd.left[i][r] * svd.right[i][c],
              0,
            );
            expect(recovered).toBeCloseTo(J[r][c], 11);
          }
      }
  });
  it("uses XY for planar analysis and detects folded/extended rank loss", () => {
    for (const q2 of [0, Math.PI, -Math.PI])
      expect(analyze(planar, [0.3, q2], "position").rank).toBe(1);
    const a = analyze(planar, planar.home, "position");
    expect(a.dimensions).toBe(2);
    expect(a.rank).toBe(2);
    expect(a.manipulability).toBeCloseTo(
      Math.abs(0.6 * 0.4 * Math.sin(planar.home[1])),
      12,
    );
    expect(a.ratio).toBeGreaterThan(0);
  });
  it("distinguishes positional and weighted full-pose tasks and redundancy", () => {
    for (const robot of [six, panda]) {
      const p = analyze(robot, robot.home, "position"),
        full = analyze(robot, robot.home, "pose");
      expect(p.rank).toBe(3);
      expect(full.rank).toBe(6);
      expect(full.nullity).toBe(robot.home.length - 6);
      const raw = jacobian(robot, robot.home);
      full.J.slice(3).forEach((row, r) =>
        row.forEach((x, c) => expect(x).toBeCloseTo(0.35 * raw[r + 3][c], 12)),
      );
      expect(p.values).toHaveLength(3);
      expect(full.values).toHaveLength(6);
    }
  });
  it("null-space projection annihilates the full pose Jacobian, not just position", () => {
    const J = taskJacobian(panda, panda.home, "pose");
    const velocity = projectNullspace(J, [1, 0, 0, 0, 0, 0, 0]);
    expect(Math.hypot(...velocity)).toBeGreaterThan(0.05);
    J.forEach((row) =>
      expect(
        Math.abs(row.reduce((s, x, i) => s + x * velocity[i], 0)),
      ).toBeLessThan(1e-11),
    );
  });
  it("reports spatial singularity without classifying a small finite singular value as zero", () => {
    expect(analyze(six, [0, 0, 0, 0, 0, 0], "pose").rank).toBeLessThan(6);
    const close = analyze(planar, [0.2, 1e-8], "position");
    expect(close.rank).toBe(2);
    expect(close.ratio).toBeLessThan(1e-8);
  });
});

describe("Cartesian path demonstration", () => {
  it("keeps both branches feasible, FK-consistent and continuous forwards/backwards", () => {
    for (const branch of ["positive", "negative"] as const) {
      let q = pathStep(planar, lecturePath, 0, planar.home, branch).q;
      for (const direction of [1, -1])
        for (let i = 0; i <= 200; i++) {
          const s =
            lecturePath.length * (direction === 1 ? i / 200 : 1 - i / 200);
          const result = pathStep(planar, lecturePath, s, q, branch);
          expect(result.status).toBe("converged");
          expect(result.positionError).toBeLessThan(1e-12);
          expect(
            Math.max(...result.q.map((x, k) => Math.abs(x - q[k]))),
          ).toBeLessThan(0.03);
          expect(
            result.q[1] * (branch === "positive" ? 1 : -1),
          ).toBeGreaterThan(0);
          getPosition(forward(planar, result.q).pose).forEach((x, k) =>
            expect(x).toBeCloseTo(
              lecturePath.p0[k] + lecturePath.direction[k] * s,
              12,
            ),
          );
          q = result.q;
        }
    }
  });
  it("flags an infeasible path sample and keeps the last valid joints", () => {
    const path = {
      ...lecturePath,
      p0: [1.1, 0, 0] as [number, number, number],
    };
    const result = pathStep(planar, path, 0.1, planar.home, "positive");
    expect(result.status).toBe("unreachable");
    expect(result.q).toEqual(planar.home);
    const limited = {
      ...planar,
      joints: planar.joints.map((j) => ({ ...j, lower: 0, upper: 0.1 })),
    };
    const excluded = pathStep(limited, lecturePath, 0.2, [0, 0], "positive");
    expect(excluded.status).toBe("joint-limits");
    expect(excluded.q).toEqual([0, 0]);
    expect(() => pathTarget(lecturePath, -0.1)).toThrow();
    expect(() => pathTarget(lecturePath, 0.5)).toThrow();
  });
});

describe("Continuous Panda full-pose redundancy demonstration", () => {
  it("moves joints continuously while keeping actual pose errors within bounds", () => {
    const reference = forward(panda, panda.home).pose;
    let q = panda.home.slice(),
      excursion = 0;
    const dt = DEMO_SAMPLE_TIME;
    for (let i = 0; i < 1200; i++) {
      const step = redundancyStep(
        panda,
        q,
        reference,
        dt,
        Math.cos((2 * Math.PI * i * dt) / 12),
      );
      expect(step.accepted, step.reason).toBe(true);
      expect(step.positionError).toBeLessThanOrEqual(REDUNDANCY_POSITION_BOUND);
      expect(step.orientationError).toBeLessThanOrEqual(
        REDUNDANCY_ROTATION_BOUND,
      );
      expect(
        Math.max(...step.q.map((x, k) => Math.abs(x - q[k]))),
      ).toBeLessThan(0.03);
      expect(clampJoints(panda, step.q)).toEqual(step.q);
      excursion = Math.max(
        excursion,
        Math.hypot(...step.q.map((x, k) => x - panda.home[k])),
      );
      q = step.q;
    }
    expect(excursion).toBeGreaterThan(0.08);
  });
  it("rejects rather than clips a step at an active joint limit", () => {
    const locked = {
      ...panda,
      joints: panda.joints.map((j, i) =>
        i === 0 ? { ...j, lower: 0, upper: 0 } : j,
      ),
    };
    const step = redundancyStep(
      locked,
      locked.home,
      forward(locked, locked.home).pose,
      1 / 60,
    );
    expect(step.accepted).toBe(false);
    expect(step.q).toEqual(locked.home);
  });
});

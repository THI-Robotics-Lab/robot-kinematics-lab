import { describe, expect, it } from "vitest";
import {
  clampJoints,
  forward,
  jacobian,
  reachBound,
} from "../src/kinematics/fk";
import {
  analytic2R,
  inverse,
  POSITION_TOLERANCE,
  residual,
  ROTATION_TOLERANCE,
} from "../src/kinematics/ik";
import {
  fromRPY,
  fromXYZRPY,
  getPosition,
  identity,
  orientationError,
  vlen,
  type Vec3,
} from "../src/kinematics/math";
import {
  panda,
  planar,
  robots,
  six,
  type Robot,
} from "../src/kinematics/models";

function near(actual: number[], expected: number[], tolerance = 1e-10) {
  actual.forEach((x, i) =>
    expect(Math.abs(x - expected[i])).toBeLessThanOrEqual(tolerance),
  );
}
function withinLimits(robot: Robot, q: number[]) {
  q.forEach((x, i) => {
    expect(Number.isFinite(x)).toBe(true);
    expect(x).toBeGreaterThanOrEqual(robot.joints[i].lower);
    expect(x).toBeLessThanOrEqual(robot.joints[i].upper);
  });
}

describe("Independent forward-kinematics checks", () => {
  it("matches the 2R closed form over a grid including folded and extended arms", () => {
    for (const q1 of [-Math.PI, -1, 0, 0.9, Math.PI])
      for (const q2 of [-Math.PI, -0.8, 0, 1.3, Math.PI]) {
        const { pose } = forward(planar, [q1, q2]);
        near(getPosition(pose), [
          0.6 * Math.cos(q1) + 0.4 * Math.cos(q1 + q2),
          0.6 * Math.sin(q1) + 0.4 * Math.sin(q1 + q2),
          0,
        ]);
        near(pose.slice(0, 3), [Math.cos(q1 + q2), Math.sin(q1 + q2), 0]);
      }
  });
  it("matches the generic 6R zero-angle DH fixture", () => {
    // q2's +pi/2 offset turns the wrist d5 offset toward +X.
    near(
      getPosition(forward(six, [0, 0, 0, 0, 0, 0]).pose),
      [0.1, -0.15, 0.85],
    );
  });
  it("matches Panda URDF zero-angle flange fixture (q4=0 is outside operational bounds)", () => {
    const pose = forward(panda, [0, 0, 0, 0, 0, 0, 0]).pose;
    near(getPosition(pose), [0.088, 0, 0.926]);
    near([pose[0], pose[5], pose[10]], [1, -1, -1]);
  });
  it("matches Panda ready-pose flange position", () => {
    near(
      getPosition(forward(panda, panda.home).pose),
      [0.306890566592941, 0, 0.590282052302839],
      1e-10,
    );
  });
  it.each(robots)(
    "$name produces rigid transforms and an independent geometric Jacobian",
    (robot) => {
      const q = robot.home,
        h = 1e-6;
      const pose = forward(robot, q).pose,
        J = jacobian(robot, q);
      for (let a = 0; a < 3; a++)
        for (let b = 0; b < 3; b++) {
          const dot = [0, 1, 2].reduce(
            (s, r) => s + pose[a * 4 + r] * pose[b * 4 + r],
            0,
          );
          expect(dot).toBeCloseTo(a === b ? 1 : 0, 12);
        }
      for (let i = 0; i < q.length; i++) {
        const plus = q.slice(),
          minus = q.slice();
        plus[i] += h;
        minus[i] -= h;
        const tp = forward(robot, plus).pose,
          tm = forward(robot, minus).pose;
        const pp = getPosition(tp),
          pm = getPosition(tm);
        const angular = orientationError(tm, tp);
        const numeric = [
          ...pp.map((x, j) => (x - pm[j]) / (2 * h)),
          ...angular.map((x) => x / (2 * h)),
        ];
        near(
          J.map((row) => row[i]),
          numeric,
          2e-8,
        );
      }
    },
  );
});

describe("Analytic 2R inverse kinematics", () => {
  it("recovers both elbow branches with exact FK consistency", () => {
    for (const q of [
      [0.4, 1.1],
      [-2, -0.7],
      [Math.PI, 0],
      [0, Math.PI],
      [1, -Math.PI],
    ]) {
      const target = forward(planar, q).pose;
      for (const elbow of ["positive", "negative"] as const) {
        const result = analytic2R(planar, target, [0, 0], elbow);
        expect(result.status).toBe("converged");
        expect(result.positionError).toBeLessThan(1e-9);
        withinLimits(planar, result.q);
        expect(
          result.q[1] * (elbow === "positive" ? 1 : -1),
        ).toBeGreaterThanOrEqual(-1e-12);
      }
    }
  });
  it.each<Vec3>([
    [1.01, 0, 0],
    [0, 0, 0],
    [0.19, 0, 0],
    [0.5, 0, 0.01],
  ])("rejects unreachable target %j", (x, y, z) => {
    const result = inverse(
      planar,
      fromXYZRPY([x, y, z], [0, 0, 0]),
      planar.home,
    );
    expect(result.status).toBe("unreachable");
    near(result.q, planar.home);
  });
  it("detects a branch excluded by limits without clipping it into a false solution", () => {
    const limited = {
      ...planar,
      joints: planar.joints.map((j) => ({ ...j, lower: 0, upper: 0.1 })),
    };
    expect(
      analytic2R(limited, forward(planar, [0.5, 1]).pose, [0, 0]).status,
    ).toBe("joint-limits");
  });
  it("does not offer a general full-pose task for two joints", () => {
    expect(inverse(planar, identity(), planar.home, "pose").status).toBe(
      "unsupported",
    );
  });
  it("exposes the extended and folded planar singularities", () => {
    for (const q2 of [0, Math.PI, -Math.PI]) {
      const J = jacobian(planar, [0.4, q2]);
      const determinant = J[0][0] * J[1][1] - J[0][1] * J[1][0];
      expect(Math.abs(determinant)).toBeLessThan(1e-12);
      expect(
        inverse(planar, forward(planar, [0.4, q2]).pose, [0, 0]).status,
      ).toBe("converged");
    }
  });
});

describe("Spatial DLS inverse kinematics", () => {
  for (const robot of [six, panda]) {
    for (const task of ["position", "pose"] as const) {
      it(`${robot.name}: ${task} task converges for a representative FK-generated corpus`, () => {
        for (let sample = 0; sample < 10; sample++) {
          const goal = clampJoints(
            robot,
            robot.home.map(
              (x, i) => x + 0.6 * Math.sin((sample + 1) * (i + 2)),
            ),
          );
          const target = forward(robot, goal).pose;
          const result = inverse(robot, target, robot.home, task);
          expect(
            result.status,
            `sample ${sample}, error ${result.positionError}, rotation ${result.orientationError}`,
          ).toBe("converged");
          expect(result.positionError).toBeLessThanOrEqual(POSITION_TOLERANCE);
          if (task === "pose")
            expect(result.orientationError).toBeLessThanOrEqual(
              ROTATION_TOLERANCE,
            );
          withinLimits(robot, result.q);
          const measured = residual(robot, result.q, target);
          expect(result.positionError).toBe(measured.positionError);
          expect(result.orientationError).toBe(measured.orientationError);
        }
      });
    }
    it(`${robot.name}: rejects a provably unreachable target`, () => {
      const target = fromXYZRPY([reachBound(robot) + 1, 0, 0], [0, 0, 0]);
      const result = inverse(robot, target, robot.home, "pose");
      expect(result.status).toBe("unreachable");
      expect(result.iterations).toBe(0);
      withinLimits(robot, result.q);
    });
    it(`${robot.name}: singular or limit-bound starts remain finite`, () => {
      const q = clampJoints(
        robot,
        robot.joints.map(() => 0),
      );
      const result = inverse(robot, forward(robot, robot.home).pose, q, "pose");
      withinLimits(robot, result.q);
      expect(Number.isFinite(result.positionError)).toBe(true);
      expect(Number.isFinite(result.orientationError)).toBe(true);
      if (result.status === "converged")
        expect(result.positionError).toBeLessThanOrEqual(POSITION_TOLERANCE);
    });
    it(`${robot.name}: resolves a 180-degree wrist orientation error`, () => {
      const goal = robot.home.slice();
      goal[goal.length - 1] += robot.id === "six" ? Math.PI : -Math.PI;
      withinLimits(robot, goal);
      const target = forward(robot, goal).pose;
      expect(residual(robot, robot.home, target).orientationError).toBeCloseTo(
        Math.PI,
        10,
      );
      const result = inverse(robot, target, robot.home, "pose");
      expect(result.status).toBe("converged");
      expect(result.orientationError).toBeLessThanOrEqual(ROTATION_TOLERANCE);
      withinLimits(robot, result.q);
    });
  }
  it("distinguishes position-only success from an infeasible full-pose constraint", () => {
    const locked = {
      ...six,
      joints: six.joints.map((j, i) => ({
        ...j,
        lower: six.home[i],
        upper: six.home[i],
      })),
    };
    const target = forward(locked, locked.home).pose.slice();
    const rotation = fromRPY(1, 0.2, -0.4);
    for (let c = 0; c < 3; c++)
      for (let r = 0; r < 3; r++) target[c * 4 + r] = rotation[c * 4 + r];
    expect(inverse(locked, target, locked.home, "position").status).toBe(
      "converged",
    );
    const result = inverse(locked, target, locked.home, "pose");
    expect(result.status).toBe("not-converged");
    expect(result.orientationError).toBeGreaterThan(0.1);
    expect(result.positionError).toBeLessThan(1e-12);
  });
});

describe("Limits and orientation edge cases", () => {
  it.each(robots)("$name clamps seed and output to bounds", (robot) => {
    const originalHome = robot.home.slice();
    const q = clampJoints(
      robot,
      robot.joints.map((_, i) => (i % 2 ? -100 : 100)),
    );
    withinLimits(robot, q);
    const result = inverse(robot, forward(robot, robot.home).pose, q);
    withinLimits(robot, result.q);
    near(robot.home, originalHome);
  });
  it("preserves exactly pi orientation errors instead of reporting false success", () => {
    for (const rpy of [
      [Math.PI, 0, 0],
      [0, Math.PI, 0],
      [0, 0, Math.PI],
    ] as Vec3[]) {
      expect(vlen(orientationError(identity(), fromRPY(...rpy)))).toBeCloseTo(
        Math.PI,
        12,
      );
    }
    expect(
      vlen(orientationError(identity(), fromRPY(0, 0, 2 * Math.PI))),
    ).toBeLessThan(1e-12);
  });
  it("rejects malformed joint and target inputs", () => {
    expect(() => forward(planar, [0])).toThrow();
    expect(() => clampJoints(planar, [NaN, 0])).toThrow();
    expect(() => inverse(panda, [NaN], panda.home)).toThrow();
    expect(() => analytic2R(planar, [NaN], planar.home)).toThrow();
  });
});

import { fromXYZRPY, identity, type Mat4, type Vec3 } from "./math";

export interface Joint {
  name: string;
  origin: Mat4;
  after: Mat4;
  lower: number;
  upper: number;
}
export interface Robot {
  id: "planar" | "six" | "panda";
  name: string;
  description: string;
  joints: Joint[];
  tool: Mat4;
  home: number[];
  planarLengths?: [number, number];
}
const pi = Math.PI;
const joint = (
  name: string,
  xyz: Vec3,
  rpy: Vec3,
  lower: number,
  upper: number,
): Joint => ({
  name,
  origin: fromXYZRPY(xyz, rpy),
  after: identity(),
  lower,
  upper,
});

// Standard DH: Rz(theta + q) Tz(d) Tx(a) Rx(alpha).
// The fixed origin commutes with Rz(q); after contains Tx(a) Rx(alpha).
function dh(
  a: number,
  alpha: number,
  d: number,
  theta: number,
  lo: number,
  hi: number,
  i: number,
): Joint {
  return {
    ...joint(`q${i + 1}`, [0, 0, d], [0, 0, theta], lo, hi),
    after: fromXYZRPY([a, 0, 0], [alpha, 0, 0]),
  };
}

export const planar: Robot = {
  id: "planar",
  name: "Planar 2R",
  description: "Two revolute joints · exact geometric IK",
  joints: [dh(0.6, 0, 0, 0, -pi, pi, 0), dh(0.4, 0, 0, 0, -pi, pi, 1)],
  tool: identity(),
  home: [pi / 6, pi / 3],
  planarLengths: [0.6, 0.4],
};

// Adapted from JINX WELDING_DH_CONFIG, MIT; see public/licenses/JINX.txt.
// A generic teaching arm, not a calibrated commercial robot.
export const six: Robot = {
  id: "six",
  name: "Spatial 6R",
  description: "Generic DH teaching arm · numerical IK",
  joints: [
    dh(0, pi / 2, 0.25, 0, -pi, pi, 0),
    dh(0.35, 0, 0, pi / 2, -pi / 2, pi / 2, 1),
    dh(0.25, 0, 0, 0, -pi * 0.8, pi * 0.8, 2),
    dh(0, pi / 2, 0.1, 0, -pi, pi, 3),
    dh(0, -pi / 2, 0.1, 0, -pi, pi, 4),
    dh(0, 0, 0.05, 0, -pi, pi, 5),
  ],
  tool: identity(),
  home: [0.3, -0.6, -0.9, 0.4, 0.8, -0.2],
};

// Numerical joint origins and bounds from Franka's franka_description.
// Apache-2.0, Franka Robotics GmbH. Re-expressed as data, no meshes or ROS code.
// Tip = panda_link8 (flange), no hand/TCP offset; see SOURCES.md.
export const panda: Robot = {
  id: "panda",
  name: "Franka Panda 7R",
  description: "Seven revolute joints · redundant numerical IK",
  joints: [
    joint("q1", [0, 0, 0.333], [0, 0, 0], -2.8973, 2.8973),
    joint("q2", [0, 0, 0], [-pi / 2, 0, 0], -1.7628, 1.7628),
    joint("q3", [0, -0.316, 0], [pi / 2, 0, 0], -2.8973, 2.8973),
    joint("q4", [0.0825, 0, 0], [pi / 2, 0, 0], -3.0718, -0.0698),
    joint("q5", [-0.0825, 0.384, 0], [-pi / 2, 0, 0], -2.8973, 2.8973),
    joint("q6", [0, 0, 0], [pi / 2, 0, 0], -0.0175, 3.7525),
    joint("q7", [0.088, 0, 0], [pi / 2, 0, 0], -2.8973, 2.8973),
  ],
  tool: fromXYZRPY([0, 0, 0.107], [0, 0, 0]),
  home: [0, -pi / 4, 0, (-3 * pi) / 4, 0, pi / 2, pi / 4],
};

export const robots = [planar, six, panda];

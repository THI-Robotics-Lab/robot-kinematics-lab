// Adapted from Kinematica (MIT), copyright (c) 2026 piranut-op.
// Changes: robust quaternion logarithm at pi; see public/licenses/Kinematica.txt.
/**
 * mathUtils.ts — dependency-free linear algebra for kinematics.
 *
 * Conventions
 * -----------
 * - `Mat4` is a length-16 array in **column-major** order, identical to
 *   THREE.Matrix4.elements, so a result can be fed straight into
 *   `new THREE.Matrix4().fromArray(m)` with no transpose.
 *   Element (row r, col c) lives at index `c * 4 + r`.
 * - Rotations from URDF `rpy` use the URDF/ROS convention: fixed-axis XYZ,
 *   i.e. R = Rz(yaw) · Ry(pitch) · Rx(roll).
 * - Right-handed coordinate system throughout.
 */

export type Vec3 = [number, number, number];
export type Quat = [number, number, number, number]; // x, y, z, w
export type Mat4 = number[]; // length 16, column-major

// ---------------------------------------------------------------------------
// Vec3
// ---------------------------------------------------------------------------

export const vsub = (a: Vec3, b: Vec3): Vec3 => [
  a[0] - b[0],
  a[1] - b[1],
  a[2] - b[2],
];
export const vadd = (a: Vec3, b: Vec3): Vec3 => [
  a[0] + b[0],
  a[1] + b[1],
  a[2] + b[2],
];
export const vscale = (a: Vec3, s: number): Vec3 => [
  a[0] * s,
  a[1] * s,
  a[2] * s,
];
export const vdot = (a: Vec3, b: Vec3): number =>
  a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const vcross = (a: Vec3, b: Vec3): Vec3 => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
export const vlen = (a: Vec3): number => Math.sqrt(vdot(a, a));
export function vnormalize(a: Vec3): Vec3 {
  const l = vlen(a);
  return l > 1e-12 ? [a[0] / l, a[1] / l, a[2] / l] : [0, 0, 0];
}

// ---------------------------------------------------------------------------
// Mat4 (column-major)
// ---------------------------------------------------------------------------

export function identity(): Mat4 {
  return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
}

/** C = A · B (both column-major). */
export function multiply(a: Mat4, b: Mat4): Mat4 {
  const c = new Array<number>(16);
  for (let col = 0; col < 4; col++) {
    for (let row = 0; row < 4; row++) {
      let sum = 0;
      for (let k = 0; k < 4; k++) sum += a[k * 4 + row] * b[col * 4 + k];
      c[col * 4 + row] = sum;
    }
  }
  return c;
}

/** Compose a list of transforms left-to-right: M = m0 · m1 · … · mN. */
export function multiplyMany(mats: Mat4[]): Mat4 {
  let out = identity();
  for (const m of mats) out = multiply(out, m);
  return out;
}

export function fromTranslation(x: number, y: number, z: number): Mat4 {
  const m = identity();
  m[12] = x;
  m[13] = y;
  m[14] = z;
  return m;
}

/** Build a homogeneous transform from a 3×3 rotation (row-major rows) + translation. */
function fromRot3(r: number[][], t: Vec3 = [0, 0, 0]): Mat4 {
  return [
    r[0][0],
    r[1][0],
    r[2][0],
    0,
    r[0][1],
    r[1][1],
    r[2][1],
    0,
    r[0][2],
    r[1][2],
    r[2][2],
    0,
    t[0],
    t[1],
    t[2],
    1,
  ];
}

/** URDF rpy → rotation matrix. Fixed-axis XYZ: R = Rz(yaw)·Ry(pitch)·Rx(roll). */
export function fromRPY(roll: number, pitch: number, yaw: number): Mat4 {
  const cr = Math.cos(roll),
    sr = Math.sin(roll);
  const cp = Math.cos(pitch),
    sp = Math.sin(pitch);
  const cy = Math.cos(yaw),
    sy = Math.sin(yaw);
  const r = [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr],
  ];
  return fromRot3(r);
}

/** URDF `<origin xyz rpy>` → homogeneous transform (translate after rotate). */
export function fromXYZRPY(xyz: Vec3, rpy: Vec3): Mat4 {
  const rot = fromRPY(rpy[0], rpy[1], rpy[2]);
  rot[12] = xyz[0];
  rot[13] = xyz[1];
  rot[14] = xyz[2];
  return rot;
}

/** Rotation about an arbitrary unit-ish axis by `angle` (Rodrigues). */
export function fromAxisAngle(axis: Vec3, angle: number): Mat4 {
  const [x, y, z] = vnormalize(axis);
  const c = Math.cos(angle),
    s = Math.sin(angle),
    t = 1 - c;
  const r = [
    [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
    [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
    [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
  ];
  return fromRot3(r);
}

/** Translation along an axis (prismatic joint). */
export function fromAxisTranslation(axis: Vec3, d: number): Mat4 {
  const [x, y, z] = vnormalize(axis);
  return fromTranslation(x * d, y * d, z * d);
}

export function getPosition(m: Mat4): Vec3 {
  return [m[12], m[13], m[14]];
}

/** Extract the 3×3 rotation as row-major rows. */
export function getRot3(m: Mat4): number[][] {
  return [
    [m[0], m[4], m[8]],
    [m[1], m[5], m[9]],
    [m[2], m[6], m[10]],
  ];
}

export function transformPoint(m: Mat4, p: Vec3): Vec3 {
  return [
    m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12],
    m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13],
    m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14],
  ];
}

/** Rotate a direction vector (ignores translation). */
export function transformDir(m: Mat4, p: Vec3): Vec3 {
  return [
    m[0] * p[0] + m[4] * p[1] + m[8] * p[2],
    m[1] * p[0] + m[5] * p[1] + m[9] * p[2],
    m[2] * p[0] + m[6] * p[1] + m[10] * p[2],
  ];
}

// ---------------------------------------------------------------------------
// Orientation conversions
// ---------------------------------------------------------------------------

/** Rotation matrix → URDF rpy (roll, pitch, yaw), inverse of fromRPY. */
export function matrixToRPY(m: Mat4): Vec3 {
  const r = getRot3(m);
  const sy = Math.hypot(r[0][0], r[1][0]);
  if (sy > 1e-6) {
    return [
      Math.atan2(r[2][1], r[2][2]),
      Math.atan2(-r[2][0], sy),
      Math.atan2(r[1][0], r[0][0]),
    ];
  }
  // Gimbal lock: pitch ≈ ±90°
  return [Math.atan2(-r[1][2], r[1][1]), Math.atan2(-r[2][0], sy), 0];
}

/** Rotation matrix → quaternion [x, y, z, w]. */
export function matrixToQuat(m: Mat4): Quat {
  const r = getRot3(m);
  const trace = r[0][0] + r[1][1] + r[2][2];
  let x: number, y: number, z: number, w: number;
  if (trace > 0) {
    const s = 0.5 / Math.sqrt(trace + 1);
    w = 0.25 / s;
    x = (r[2][1] - r[1][2]) * s;
    y = (r[0][2] - r[2][0]) * s;
    z = (r[1][0] - r[0][1]) * s;
  } else if (r[0][0] > r[1][1] && r[0][0] > r[2][2]) {
    const s = 2 * Math.sqrt(1 + r[0][0] - r[1][1] - r[2][2]);
    w = (r[2][1] - r[1][2]) / s;
    x = 0.25 * s;
    y = (r[0][1] + r[1][0]) / s;
    z = (r[0][2] + r[2][0]) / s;
  } else if (r[1][1] > r[2][2]) {
    const s = 2 * Math.sqrt(1 + r[1][1] - r[0][0] - r[2][2]);
    w = (r[0][2] - r[2][0]) / s;
    x = (r[0][1] + r[1][0]) / s;
    y = 0.25 * s;
    z = (r[1][2] + r[2][1]) / s;
  } else {
    const s = 2 * Math.sqrt(1 + r[2][2] - r[0][0] - r[1][1]);
    w = (r[1][0] - r[0][1]) / s;
    x = (r[0][2] + r[2][0]) / s;
    y = (r[1][2] + r[2][1]) / s;
    z = 0.25 * s;
  }
  return [x, y, z, w];
}

/**
 * Orientation error as a rotation vector (axis · angle), expressed in the
 * world frame, that rotates `current` onto `target`: log(R_target · R_currentᵀ).
 * This is the standard angular term for a 6-DOF IK error vector.
 */
export function orientationError(current: Mat4, target: Mat4): Vec3 {
  // q_error = q_target * conjugate(q_current). Choose the shortest rotation.
  const [ax, ay, az, aw] = matrixToQuat(target);
  const [bx, by, bz, bw] = matrixToQuat(current);
  let x = -aw * bx + ax * bw - ay * bz + az * by;
  let y = -aw * by + ax * bz + ay * bw - az * bx;
  let z = -aw * bz - ax * by + ay * bx + az * bw;
  let w = aw * bw + ax * bx + ay * by + az * bz;
  if (w < 0) {
    x = -x;
    y = -y;
    z = -z;
    w = -w;
  }
  const length = Math.hypot(x, y, z);
  const scale = length < 1e-12 ? 2 : (2 * Math.atan2(length, w)) / length;
  return [x * scale, y * scale, z * scale];
}

// ---------------------------------------------------------------------------
// Small dense linear solver — used by the damped-least-squares IK step.
// ---------------------------------------------------------------------------

/**
 * Solve A·x = b for a square n×n system via Gaussian elimination with partial
 * pivoting. `A` is row-major (array of rows). Returns null if singular.
 */
export function solveLinear(A: number[][], b: number[]): number[] | null {
  const n = b.length;
  // Augmented copy
  const M = A.map((row, i) => [...row, b[i]]);
  for (let col = 0; col < n; col++) {
    // pivot
    let pivot = col;
    let best = Math.abs(M[col][col]);
    for (let r = col + 1; r < n; r++) {
      const v = Math.abs(M[r][col]);
      if (v > best) {
        best = v;
        pivot = r;
      }
    }
    if (best < 1e-12) return null;
    if (pivot !== col) {
      const tmp = M[col];
      M[col] = M[pivot];
      M[pivot] = tmp;
    }
    // eliminate
    const pv = M[col][col];
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const factor = M[r][col] / pv;
      if (factor === 0) continue;
      for (let c = col; c <= n; c++) M[r][c] -= factor * M[col][c];
    }
  }
  const x = new Array<number>(n);
  for (let i = 0; i < n; i++) x[i] = M[i][n] / M[i][i];
  return x;
}

/** Shortest signed angular distance, wrapping to (−π, π]. */
export function wrapAngle(a: number): number {
  let x = a % (2 * Math.PI);
  if (x > Math.PI) x -= 2 * Math.PI;
  if (x <= -Math.PI) x += 2 * Math.PI;
  return x;
}

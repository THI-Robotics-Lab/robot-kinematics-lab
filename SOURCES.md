# Source evaluation and provenance

## Iteration 2 mathematical references

The new analysis and demonstration modules are original TypeScript additions;
the previously adapted FK/IK/model code and its notices are preserved. No new
runtime dependency, remote asset or library source is introduced.

- [Modern Robotics, Chapter 5.4 — Manipulability](https://modernrobotics.northwestern.edu/nu-gm-book-resource/5-4-manipulability/)
  motivates the joint-rate ball mapped by a task Jacobian into a velocity
  ellipse/ellipsoid. Linear and angular rows use different physical units;
  the app explicitly selects the task and states its 0.35 m/rad angular length.
- [LAPACK DGESVJ documentation](https://www.netlib.org/lapack/explore-html/d1/d5e/dgesvj_8f_source.html)
  describes the one-sided Jacobi approach to singular value decomposition.
  This app implements a small cyclic Jacobi algorithm for its own matrices;
  it does not copy LAPACK code or claim LAPACK's robustness or performance.

Null-space projection is evaluated from the nonzero right singular directions
as `(I - J^+ J) g`. The Panda demonstration uses this full-pose projection plus
local correction, with explicit error, continuity and position-limit checks.
The Cartesian path is authored teaching data, tested on both exact 2R branches.

Inspected upstream source, licenses, package manifests and kinematics modules
on 2026-10-30 before implementation. Only the explicitly listed portions were
reused; upstream applications are not dependencies or runtime services.

## Kinematica

- Repository: https://github.com/piranut-op/kinematica
- Revision: `257cdf0c7ed61cc61dcfc5a5873098f84a29012c`
- License: MIT, copyright (c) 2026 piranut-op.
- Inspected `src/kinematics/{mathUtils,fk,ik}.ts`, model/parser structure,
  README, package manifest and third-party notices.
- Suitable parts: a pure TypeScript matrix/vector layer, column-major matrices
  compatible with Three.js, fixed-axis URDF RPY conventions, small dense solve.
  `mathUtils.ts` is adapted as `src/kinematics/math.ts` with its license retained.
- Changed its orientation residual to a quaternion logarithm: the upstream
  near-zero-skew branch also returns near zero at a rotation of π. Here π is a
  real angular error, covered by regression tests.
- The full app depends on React, R3F/drei, URDF loader, Zustand, Radix/shadcn,
  Tailwind, animation, ZIP handling and remote fonts. Those are useful for a
  general user-uploaded URDF viewer but unnecessary for three fixed teaching
  models. Its generic IK also relies on the URDF/parser layer and finite
  differences; adapting that whole stack would obscure the explicit geometric
  Jacobian used in this lecture demonstrator.
- Decision: reuse the math implementation and conventions with attribution;
  retain a smaller explicit model/solver/view boundary. No upstream fonts,
  uploads, assets or UI dependencies are copied.

## JINX (Joint Inverse N-dimensional eXplorer)

- Repository: https://github.com/chandansaipavanpadala/jinx
- Revision: `d11ad062b6a9b436f3b275b5bd503b0e8018fd2f`
- License: MIT, copyright (c) 2026 Chandan Sai Pavan Padala.
- Inspected `src/math/KinematicsNDOF.js`: DH transforms, FK, geometric Jacobian,
  position-only DLS, numerical helpers and `WELDING_DH_CONFIG`; also manifest,
  license and README.
- Reused `WELDING_DH_CONFIG` geometry, offsets and joint bounds in `models.ts`,
  expressed through explicit origin/rotation/post transforms. The 6R model is
  nominal teaching geometry, not a commercial robot or spherical-wrist claim.
- The file header calls the table “Modified DH”, but its actual `dhMatrix`
  implements **standard DH** (`Rz Tz Tx Rx`). This demo explicitly follows that
  implementation, documents the convention, and checks a zero-angle fixture.
- The supplied `ik_dls` uses only positional rows. Its non-converged return also
  carries the previous iteration's error alongside the final position. The new
  solver recomputes residuals from the returned joints, distinguishes task
  types, bounds steps and reports convergence separately from reachability.
- Dependencies include Three.js, Chart.js and Puppeteer, with Vite tooling;
  the application also has login, persistence and task simulation layers.
  None is needed here beyond Three.js/Vite. No JINX login or simulation assets
  are included. Its complete IK engine is not vendored.

Both MIT licenses permit adaptation and redistribution with their notices.
This demo includes the full license texts. No uninspected upstream dependency
tree or remotely loaded robot asset is pulled into the runtime.

## Franka Panda 7R model

- Repository: https://github.com/frankarobotics/franka_ros
- Revision: `ddd2fffd9de44b02ad15b4bbb2bfa2cec4d60d98`
- Geometry: [`franka_description/robots/common/franka_arm.xacro`](https://github.com/frankarobotics/franka_ros/blob/ddd2fffd9de44b02ad15b4bbb2bfa2cec4d60d98/franka_description/robots/common/franka_arm.xacro).
- Bounds: [`franka_description/robots/panda/joint_limits.yaml`](https://github.com/frankarobotics/franka_ros/blob/ddd2fffd9de44b02ad15b4bbb2bfa2cec4d60d98/franka_description/robots/panda/joint_limits.yaml).
- License declaration: [`franka_description/package.xml`](https://github.com/frankarobotics/franka_ros/blob/ddd2fffd9de44b02ad15b4bbb2bfa2cec4d60d98/franka_description/package.xml)
  declares Apache 2.0; upstream root LICENSE supplies its text. The root NOTICE
  (copyright 2023 Franka Robotics GmbH) is preserved as `public/licenses/Franka-NOTICE.txt`.
  The numeric transforms and position bounds have been adapted
  into TypeScript data; no ROS implementation or manufacturer meshes ship.
- All seven axes are local Z. Origins retain the upstream metre/radian values.
  The terminal fixed translation is 0.107 m along joint-7 Z to `panda_link8`.
  There is no hand rotation, gripper or TCP offset.
- Fixture at all-zero mathematical angles: flange position `[0.088, 0, 0.926]`
  m, rotation `diag(1, -1, -1)`. q₄ = 0 is outside its operational limit, so
  this fixture checks geometry only. The UI uses the in-limit ready pose
  `[0, −π/4, 0, −3π/4, 0, π/2, π/4]`.

## Planar 2R

Explicit teaching geometry: L₁ = 0.60 m, L₂ = 0.40 m, both joints ±π, XY plane.
The standard sine/cosine FK and cosine-law IK are implemented here and checked
against each other and independent closed-form fixtures, including both elbow
branches and the inner/outer singular boundaries.

## Runtime and tooling

The locally bundled `public/thi-logo.svg` is the official THI institutional
logo from [thi.de](https://www.thi.de/_assets/a92153751098915699a1afa17e77f864/vite-output/logo/thi-full.svg).
It identifies Technische Hochschule Ingolstadt and is not part of the lab's
MIT-licensed source code.
`public/robotics-lab.png` is the user-supplied image used for the lab-link
thumbnail; it is not part of the lab's MIT-licensed source code.

Runtime: React, React DOM (including scheduler) and Three.js, all MIT. Orbit and
transform controls come from Three.js's locally bundled addons. Vite, React
plugin, Vitest, TypeScript, Playwright and type packages are development tools.
The package manifest pins direct versions and `package-lock.json` pins the
resolved tree. The build copies `public/` notices into `dist/`. There are no
remotely loaded images, font files, URDF fetches, textures or CDN imports.

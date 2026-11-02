# Release validation

The first public release was checked on Ubuntu 24.04 with Node.js 22.16.0 and
npm 10.9.2. Dependency installation uses `npm ci`; the production build uses
Vite with relative asset paths, including the local THI logo and lab thumbnail.
The browser suite uses headless Chromium and software WebGL.

The public snapshot passed `npm ci --offline --no-audit`, **41 numerical
tests**, the production build and **18 browser tests**. A separate browser
check served the build under `/robot-kinematics-lab/` with external requests
blocked: the app and both local header images loaded successfully, with no
external runtime requests or page errors.

The numerical suite covers FK/IK consistency, joint limits, unreachable and
singular configurations, Jacobians and the guided demonstrations. Browser checks
cover the robot models, camera, compact joint controls, IK manipulation, wrist
orientation marker, demonstrations, laptop/mobile layouts and offline runtime
request guards. Run `npm test`, `npm run build` and `npm run test:browser` to
reproduce the checks. GitHub Actions runs them before deploying to Pages.

Remaining limitations are documented in [technical guide](TECHNICAL.md). In particular,
spatial IK is local and seed-dependent, schematic robots do not simulate
collisions or dynamics, and Firefox, Safari, physical GPUs and projector
hardware have not been separately validated. Vite reports a large JavaScript
chunk warning; the application remains a locally bundled static site.

## Numerical coverage

The 31 prototype checks and ten Iteration 2 checks are preserved. They cover:

- Known singular values down to 1e-10 and reconstruction from singular vectors.
- Planar XY rank, extended/folded singularities and the exact manipulability
  identity; a small finite singular value remains distinguishable from zero.
- Spatial position/full-pose ranks, angular scaling, numerical spatial rank
  loss, and full-pose null-space annihilation.
- Both planar elbow trajectories forwards/backwards across **804 path samples**,
  with FK/IK agreement and adjacent-joint continuity checks.
- Unreachable and joint-limit-excluded path samples, preserving the last valid
  joint vector, plus out-of-domain path coordinate rejection.
- **1,200 Panda predictor/corrector updates at 0.02 s** (24 simulated seconds),
  spanning periodic reversals. Every accepted configuration remains in bounds,
  every joint step stays below 0.03 rad, and position/angular errors remain
  below 0.05 mm / 0.0005 rad relative to the original full-pose reference.
  The test also verifies a nontrivial joint excursion, not merely a static pose.
- Rejection of a null-space step at an active joint limit, without clipping the
  candidate into a false pose-preserving solution.

The positive/negative line path, singular configurations and nominal Panda
reference are bounded teaching examples. These checks are not a proof of global
IK convergence, general posture planning or hardware safety.

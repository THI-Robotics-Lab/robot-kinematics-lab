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

Remaining limitations are documented in [README.md](README.md). In particular,
spatial IK is local and seed-dependent, schematic robots do not simulate
collisions or dynamics, and Firefox, Safari, physical GPUs and projector
hardware have not been separately validated. Vite reports a large JavaScript
chunk warning; the application remains a locally bundled static site.

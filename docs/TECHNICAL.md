# Technical guide

## Teaching modes and projection

The lab opens with the Franka Panda 7R at its home joint configuration, with
the target set to its home pose. Select a model, then one of three modes:

- **Forward Kinematics** shows joint sliders, numeric joint angles and the actual
  end-effector pose. Move the joints and toggle **Joint frames** or **Joint labels**.
- **Inverse Kinematics** shows the Cartesian target, task selection and measured
  IK errors. Drag the target axes and release, or enter values and **Solve IK**.
  Choose **Position** to leave orientation free, or **Full pose** on spatial
  models. The planar model offers positive/negative/nearest elbow branches.
- **Kinematic Analysis** shows task-specific singular values, rank, nullity and
  manipulability. Change joints to observe how available task velocities vary.

**Advanced information** exposes the geometric matrix, joint seeds in IK, and
additional pose/joint readouts. Changing mode or robot leaves a guided demo and
pauses playback. Changing an ordinary joint/target invalidates prior IK status.

**Presentation mode** is enabled initially. It enlarges controls, numbers and the viewport and removes
secondary prose. Model, mode and demo switches stay accessible. Joint controls
remain in the viewport. In the Panda demo, advanced analysis details
are condensed until **Advanced information** is enabled. **Enter fullscreen**
uses the browser Fullscreen API when available; presentation layout also works
without fullscreen. Escape exits native fullscreen. Ordinary buttons, sliders,
number fields and selectors support keyboard operation.

Orbit by left-dragging the background, pan by right-dragging, zoom with the
wheel. The compact **Joint controls** in the lower-left of the workspace let
you adjust angles with sliders or numeric fields in any mode while viewing the
robot; click their heading to collapse or expand them. **Home** resets the joint
configuration. Joint controls are disabled during Demos C and D, where
the demonstrations determine the joint configuration. On Planar 2R, **Top View** looks down the Z axis onto XY, with X to the
right and Y up, and fits the complete reachable disk. Demo A and Demo D start
with this view. Orbit controls remain available; **Reset view** restores the
original oblique camera. Selecting another robot restores its own default view.
Top-view framing adapts to viewport resizing until the camera is moved manually.

Joint labels `q1`, `q2`, etc. are anchored by leader lines to the same FK pivots
used to draw the robot. Screen-space offsets separate coincident spatial joints;
they do not change the model. Focusing or changing a joint slider/number field
highlights that joint and its label in gold until focus leaves the control.
Labels grow in Presentation Mode and can be hidden with **Joint labels**.

**Home pose** restores joints during
manual exploration; guided playback uses **Reset demo**. Numeric entries clamp
to slider bounds; their display normalizes on blur/Enter so signed decimals can
be typed naturally. The wireframe is the requested target, the solid orange tip
is the actual flange, and the dashed error line connects them.
The two-finger shape is a schematic orientation marker attached to that flange;
its orange fingertip helps distinguish rotations. It is not an actuated gripper,
and its geometry does not change FK, IK or the reported end-effector pose.

All poses use a right-handed base frame, **Z up**, metres and radians internally.
Joint and RPY controls use degrees. Axis colours are X red, Y green, Z blue.
Panda's tip is its flange (`panda_link8`), without a gripper TCP offset.
Geometry remains the original schematic of link/joint origins.

## Guided demonstrations

Choose any **Demo A–D** directly. Each restores a known model/task/seed, pauses
previous playback and keeps its teaching formulas beside the robot.

| Demo                          | Lecture sequence                                                                                                                                                                                                                                                                                                                                   |
| ----------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **A — Planar FK/IK**          | Joint-space controls → two exact elbow solutions for `[0.6, 0.2, 0]` m → an extended singular configuration. Dashed circles show the reachable annulus. The last step opens the velocity ellipse and rank analysis.                                                                                                                                |
| **B — Position vs full pose** | A 6R wrist rotation of 1.1 rad changes the target orientation without changing its position. Both comparison buttons solve from the same home seed. Position-only succeeds while leaving an angular difference; full-pose IK removes it. Requested/actual frames and both residuals are shown.                                                     |
| **C — Panda redundancy**      | Play/pause a smooth, periodically reversing null-space predictor with full-pose correction. The reference flange pose stays fixed approximately. Joint angles, displacement from home and actual position/angular errors update throughout. Reset restores the reference/home.                                                                     |
| **D — Cartesian line**        | Drag scalar `s`, or play/pause a constant-velocity traversal. Adjust `u` from 0.02 to 0.20 m/s. Green dashes/endpoints show the desired path, orange shows the FK-reconstructed trajectory, and IK joint angles/Cartesian reconstruction error remain visible. Playback stops at the endpoint. Reset restores `s=0` and the positive elbow branch. |

See the [application overview](../README.md) for a current screenshot.

### Cartesian motion and the MPC connection

The built-in path is

```text
p(s) = p0 + e s
p0 = [0.35, 0.20, 0] m, e = [0.8, 0.6, 0], ||e|| = 1
0 <= s <= 0.40 m
s[k+1] = s[k] + Ts u[k], Ts = 0.02 s
```

The complete segment stays inside the nonsingular planar workspace. An explicit
positive or negative elbow branch is retained for the whole traversal; the
branch selector is disabled during playback. Changing branch while paused is
an explicit configuration change, not a claim of a continuous transition
between branches. Sliding `s` can jump to another valid sample; playback uses
small successive steps. Each sample uses the existing analytic IK and is
checked for feasibility/limits before updating joints. A failed sample pauses
playback and retains the last valid configuration. The FK-reconstructed trajectory retains
at most 600 samples; resetting clears it.

`u` is Cartesian velocity along the unit direction `e`, not a joint velocity or
a force. The scalar integrator is the model used in the subsequent linear MPC
lecture. Demo D computes **IK followed by FK** at each path sample: the solid
orange point is the **FK-computed position**, the orange curve is the
**FK-reconstructed trajectory**, and the residual against the desired point is
the **Cartesian reconstruction error**. This is geometric reconstruction, not
physical motion or closed-loop tracking. No MPC controller or robot dynamics
is implemented. Fixed-step browser timers may run slower than
wall time under load; hidden tabs pause rather than catching up with a jump.

### Panda null-space motion and error bounds

The full-pose weighted Jacobian is `Jtilde = [Jv; 0.35 Jw]`. The predictor uses

```text
qdot = (I - Jtilde^+ Jtilde) g(t)
g(t) = 0.45 cos(2 pi t / 12) e1 rad/s
```

The SVD supplies an orthogonal joint-rate null-space projector. Each 0.02 s
predictor step is followed by up to eight local full-pose Newton corrections
with small damping. Step sizes can be halved, but there are **no global solver
restarts** that could jump to a disconnected solution. The accepted step must
stay inside joint limits, change no joint by more than 0.03 rad, and leave at
most **0.05 mm** position error and **0.0005 rad** (about 0.029°) angular error
relative to the original reference. These are demonstration acceptance bounds,
not singularity thresholds or a claim of exact pose preservation. Actual errors
are displayed in scientific notation. Rank loss, a vanished null direction,
or rejected correction pauses the motion with a reason. This is a controlled
local example of redundancy, not a general posture planner.

## Offline operation

`npm run build` creates a complete static `dist/`, including JS, CSS and license
notices. Copy **the entire directory** to an offline machine or local static
host. Relative asset paths (`base: './'`) also allow deployment below a path
such as `/demos/robot-kinematics-lab/`. Serve it over HTTP with any static server;
for example, `npm run preview` on a machine with the development dependencies
installed. A local static file server is delivery infrastructure, not an
application backend. No external connection is needed while using the app.

Opening `dist/index.html` through `file://` is not supported because browsers
restrict JavaScript modules there. There is no service worker or offline cache
installation; remote-host reloads still require that host. For an offline
lecture, carry and serve the local `dist/`. Initial dependency installation
and optional Chromium download require network access (or prepared caches).

## Architecture

| File                             | Responsibility                                                                                               |
| -------------------------------- | ------------------------------------------------------------------------------------------------------------ |
| `src/kinematics/math.ts`         | Dependency-free transforms, vectors, rotation log and small dense solve; adapted from Kinematica.            |
| `src/kinematics/models.ts`       | Explicit joint origins, post-joint transforms, bounds, home configurations and fixed tool offsets.           |
| `src/kinematics/fk.ts`           | Pure FK, joint projection, conservative reach bound, 6×N world-frame geometric Jacobian.                     |
| `src/kinematics/ik.ts`           | Analytic 2R branches and bounded multi-start damped least-squares spatial IK; explicit status and residuals. |
| `src/kinematics/analysis.ts`     | Task Jacobians, small Jacobi SVD, numerical rank and null-space projection.                                  |
| `src/kinematics/motion.ts`       | Feasible Cartesian samples and bounded Panda predictor/corrector steps.                                      |
| `src/teaching/`                  | Mode/demo controls, formulas and task-analysis readouts.                                                     |
| `tests/teaching.test.ts`         | SVD checks, task-specific rank, path continuity, null-space errors and bounds.                               |
| `tests/browser/teaching.spec.ts` | Modes, demos, projection/fullscreen, keyboard interaction and screenshots.                                   |
| `src/App.tsx`                    | React state, teaching controls, solver dispatch and readouts.                                                |
| `src/Viewport.tsx`               | Three.js geometry, orbit/target controls; consumes the same FK as the solver.                                |
| `tests/kinematics.test.ts`       | Independent FK fixtures, Jacobian differences, IK corpus, limits and edge cases.                             |
| `tests/browser/lab.spec.ts`      | Production UI, model switching, target dragging, mobile layout and external-request checks.                  |

Data flows from joint/target controls into React state; pure numerical functions
produce FK, solver results and Jacobians. Three.js is a view of that state.
Changing joints, targets, elbow branch or task invalidates the previous solve
status. Residuals always describe the displayed joint configuration. A failed
numerical solve displays its best candidate and its actual remaining error.

Each revolute joint has

```text
T_parent,child(qᵢ) = T_origin,i Rz(qᵢ) T_after,i
T₀ₑ(q) = ∏ᵢ T_parent,child(qᵢ) T_tool
Jᵢ = [zᵢ × (pₑ − pᵢ); zᵢ]
```

`pᵢ,zᵢ` are taken immediately before the joint rotation. The 2R and generic 6R
use **standard DH**, with `T_origin = Tz(d) Rz(theta_offset)` and
`T_after = Tx(a) Rx(alpha)`. Panda uses its URDF origin transforms directly,
avoiding a standard/modified DH convention conversion.

### Solvers and interpretation

2R uses the cosine law, both signs of q₂ and a two-argument atan formula for q₁.
Targets outside the reachable annulus or XY plane are rejected. Candidate
branches are checked against limits rather than silently clipped into an
incorrect solution. The default branch is nearest to the current seed.

Spatial IK uses the geometric Jacobian and
`Δq = Jᵀ (JJᵀ + λ²I)⁻¹ e`, solved with partial-pivot Gaussian elimination.
Position-only tasks use three rows. Full-pose tasks use six rows with the
world-frame angular residual `log(R_target R_currentᵀ)`. A quaternion log
preserves errors at exactly 180°, unlike a skew-only approximation.
The explicit characteristic length **0.35 m/rad** weights angular residuals
and angular Jacobian rows, rather than treating metres and radians as the
same unit. Displayed errors remain unweighted.

The solver projects every seed/step into limits, caps steps at 0.25 rad, uses
backtracking and adaptive damping, and tries at most five deterministic seeds
with 180 iterations each. It stops at **0.1 mm** and, for full pose, **0.001 rad**
(about 0.057°). Success requires both applicable tolerances. `iterations` counts
attempted DLS steps across seeds; closed-form 2R reports zero iterative steps.

An outside-sphere target is provably unreachable by the triangle inequality.
Inside that conservative bound, local solver failure means **reachability
undetermined**, including potential joint-limit or singularity problems. It is
not a global infeasibility proof. FK accepts finite out-of-limit angles for
mathematical tests; UI and IK explicitly enforce limits through `clampJoints`.

### Task-specific analysis and velocity geometry

Analysis uses `Jxy` (2×2) for the planar robot, `Jv` (3×N) for spatial position,
and `[Jv; 0.35 Jw]` (6×N) for full pose. The one-sided Jacobi SVD operates on the
transpose, avoiding an explicit Gram matrix that would square the condition
number and lose small singular values. It is a small teaching implementation,
not an industrial numerical library. If its sweep limit is reached the UI
reports that the spectrum is not certified.

The UI displays all singular values, `sigma_min / sigma_max`, the joint-space
nullity and `product(sigma_i)`. Numerical rank uses the relative floating-point
criterion `epsilon * max(m,n) * sigma_max`. It describes numerical rank only;
there is **no arbitrary near-singular cutoff**. Watch the ratio continuously.
The product has units `(m/rad)^m`; comparisons require the same task, geometry
and angular characteristic length. Full-pose spectra depend on the chosen
0.35 m/rad scaling. Residual errors are always displayed in physical units.

For position tasks, the velocity ellipse/ellipsoid maps the unit joint-rate
ball `||qdot||_2 <= 1 rad/s` through the task Jacobian. Principal radii equal its
singular values. The viewport uses a fixed **0.35 s** displacement scale, not
per-frame normalization, so collapse at rank loss remains visible. In the
planar view, a joint-rate direction slider maps a unit-circle vector through
`v=Jxy qdot`; the purple arrow and ellipse share that same scale. Spatial
position ellipsoids are shown as three principal great circles. The full-pose
ellipsoid is six-dimensional and is not misrepresented as a 3D position shape.

No velocity ellipsoid accounts for active joint limits; it represents the
unconstrained local joint-rate ball. Playback separately enforces actual joint
position bounds. Existing FK, IK, model data and original numerical tests are
preserved. Future MPC work can use the scalar path coordinate and reference
generator without changing these kinematics.

## Sources, licenses and limitations

See [SOURCES.md](../SOURCES.md) for the pinned Kinematica/JINX evaluation, reuse
decision and Panda provenance, and [THIRD_PARTY_NOTICES](../public/THIRD_PARTY_NOTICES.txt)
for redistribution terms. The demo-specific source is MIT; the adapted Franka
model data retains Apache-2.0 notices. Runtime dependency licenses ship in `dist`.

- Spatial IK is a local numerical method, with finite deterministic restarts;
  it can fail on reachable poses and can select different redundant solutions.
- The redundancy example is local to the chosen reference and does not optimize
  posture globally. Manual IK can still choose different redundant solutions.
  There are no physical joint velocity/acceleration limits or collision guarantees.
- No collision detection, self-collision checks, dynamics, contact, ROS or MPC.
  Schematic links may intersect. Joint limits are position bounds only.
- The 6R is a generic JINX teaching model; Panda uses published nominal geometry
  and limits, without calibration, hand geometry or gripper TCP.
- Numerical work and rendering share the UI thread. General IK runs on release
  or button press; bounded demo corrections run during playback. Slow machines
  may play below the nominal 50 Hz rate; this is not real-time robot control.
- RPY readouts have coordinate singularities at pitch ±90°; IK itself uses
  rotation vectors. Redundancy and rank depend on the chosen task and pose.
- WebGL is required for 3D. Numerical controls/readouts remain available if a
  renderer cannot be created. Keyboard/numeric alternatives accompany dragging.

The lab is self-contained in this directory and is separate from the course's
MPC notebooks and projects.

import { useEffect, useMemo, useRef, useState } from "react";
import { Viewport } from "./Viewport";
import { clampJoints, forward, jacobian } from "./kinematics/fk";
import {
  analytic2R,
  inverse,
  residual,
  type IKResult,
  type Task,
} from "./kinematics/ik";
import {
  fromXYZRPY,
  getPosition,
  matrixToRPY,
  type Mat4,
  type Vec3,
} from "./kinematics/math";
import { panda, planar, six, robots, type Robot } from "./kinematics/models";
import { analyze } from "./kinematics/analysis";
import {
  DEMO_SAMPLE_TIME,
  lecturePath,
  pathPoints,
  pathStep,
  pathTarget,
  redundancyStep,
} from "./kinematics/motion";
import { AnalysisPanel } from "./teaching/AnalysisPanel";
import { DemoPanel, demos, type Demo, type Mode } from "./teaching/DemoPanel";

const degrees = (x: number) => (x * 180) / Math.PI;
const radians = (x: number) => (x * Math.PI) / 180;
const fixed = (x: number, n = 3) =>
  (Math.abs(x) < 0.5 * 10 ** -n ? 0 : x).toFixed(n);
const statusText = {
  converged: "Converged",
  unreachable: "Unreachable target",
  "joint-limits": "Blocked by joint limits",
  "not-converged": "No convergence",
  unsupported: "Unsupported task",
};

function NumberField({
  label,
  value,
  step = 0.01,
  disabled = false,
  onChange,
  onFocus,
  onBlur,
}: {
  label: string;
  value: number;
  step?: number;
  disabled?: boolean;
  onChange: (value: number) => void;
  onFocus?: () => void;
  onBlur?: () => void;
}) {
  const display = (x: number) => String(Number(x.toFixed(5)));
  const [draft, setDraft] = useState(() => display(value));
  const [editing, setEditing] = useState(false);
  // Preserve intermediate keyboard entries such as '-' and '0.' while typing.
  // Valid values update FK live; blur normalizes the displayed, clamped value.
  return (
    <label className="number-field">
      <span>{label}</span>
      <input
        type="number"
        aria-label={label}
        value={editing ? draft : display(value)}
        step={step}
        disabled={disabled}
        onFocus={() => {
          onFocus?.();
          setDraft(display(value));
          setEditing(true);
        }}
        onBlur={() => {
          setEditing(false);
          onBlur?.();
        }}
        onKeyDown={(e) => {
          if (e.key === "Enter") e.currentTarget.blur();
        }}
        onChange={(e) => {
          setDraft(e.target.value);
          if (e.target.value !== "" && Number.isFinite(e.target.valueAsNumber))
            onChange(e.target.valueAsNumber);
        }}
      />
    </label>
  );
}

export function App() {
  const [robot, setRobot] = useState<Robot>(planar);
  const [q, setQ] = useState(planar.home);
  const [target, setTarget] = useState<Mat4>(
    () => forward(planar, planar.home).pose,
  );
  const [task, setTask] = useState<Task>("position");
  const [result, setResult] = useState<IKResult | null>(null);
  const [elbow, setElbow] = useState<"nearest" | "positive" | "negative">(
    "nearest",
  );
  const [frames, setFrames] = useState(false);
  const [tool, setTool] = useState<"translate" | "rotate">("translate");
  const [resetView, setResetView] = useState(0);
  const [cameraView, setCameraView] = useState<"default" | "top">("default");
  const [jointLabels, setJointLabels] = useState(true);
  const [workspaceJointsOpen, setWorkspaceJointsOpen] = useState(true);
  const [activeJoint, setActiveJoint] = useState<number | null>(null);
  const [mode, setMode] = useState<Mode>("fk");
  const [demo, setDemo] = useState<Demo>(null);
  const [advanced, setAdvanced] = useState(false);
  const [presentation, setPresentation] = useState(false);
  const [fullscreen, setFullscreen] = useState(false);
  const [fullscreenMessage, setFullscreenMessage] = useState("");
  const [s, setS] = useState(0);
  const [speed, setSpeed] = useState(0.06);
  const [branch, setBranch] = useState<"positive" | "negative">("positive");
  const [playing, setPlaying] = useState(false);
  const [phase, setPhase] = useState(0);
  const [motionMessage, setMotionMessage] = useState("Ready.");
  const [trail, setTrail] = useState<Vec3[]>([]);
  const [rateAngle, setRateAngle] = useState(0);
  const [ellipsoid, setEllipsoid] = useState(true);
  const analysis = useMemo(() => analyze(robot, q, task), [robot, q, task]);
  const desiredPath = useMemo(() => pathPoints(lecturePath), []);
  const latestMotion = useRef({ q, s, phase, speed, branch, target });
  latestMotion.current = { q, s, phase, speed, branch, target };
  const fk = useMemo(() => forward(robot, q), [robot, q]);
  const p = getPosition(fk.pose),
    rpy = matrixToRPY(fk.pose);
  const targetPosition = getPosition(target),
    targetRPY = matrixToRPY(target);
  const error = residual(robot, q, target);
  const J = useMemo(() => jacobian(robot, q), [robot, q]);
  const planarDeterminant = robot.planarLengths
    ? robot.planarLengths[0] * robot.planarLengths[1] * Math.sin(q[1])
    : null;

  const selectRobot = (next: Robot) => {
    setCameraView("default");
    setActiveJoint(null);
    setPlaying(false);
    setDemo(null);
    setTrail([]);
    setMotionMessage("Ready.");
    setRobot(next);
    setQ(next.home.slice());
    setTarget(forward(next, next.home).pose);
    setTask("position");
    setTool("translate");
    setResult(null);
    setElbow("nearest");
  };
  const solve = (pose = target) => {
    const next = robot.planarLengths
      ? analytic2R(robot, pose, q, elbow)
      : inverse(robot, pose, q, task);
    setQ(next.q);
    setResult(next);
  };
  const changeTarget = (pose: Mat4, solveOnRelease = false) => {
    setTarget(pose);
    setResult(null);
    if (solveOnRelease) solve(pose);
  };
  const changeCoordinate = (index: number, value: number, rotation = false) => {
    const xyz = [...targetPosition] as Vec3,
      angles = [...targetRPY] as Vec3;
    if (rotation) angles[index] = radians(value);
    else xyz[index] = value;
    changeTarget(fromXYZRPY(xyz, angles));
  };
  const changeJoint = (index: number, value: number) => {
    setActiveJoint(index);
    setPlaying(false);
    const next = q.slice();
    next[index] = radians(value);
    setQ(clampJoints(robot, next));
    setResult(null);
  };
  const home = () => {
    setActiveJoint(null);
    setPlaying(false);
    setQ(robot.home.slice());
    setResult(null);
  };

  const changeMode = (next: Mode) => {
    setActiveJoint(null);
    setMode(next);
    setPlaying(false);
    setDemo(null);
    setTrail([]);
  };
  const stepA = (step: number) => {
    setPlaying(false);
    setRobot(planar);
    setDemo("A");
    setTask("position");
    setTool("translate");
    setFrames(false);
    setTrail([]);
    if (step === 1 || step === 2) {
      const pose = fromXYZRPY([0.6, 0.2, 0], [0, 0, 0]);
      const chosen = step === 1 ? "positive" : "negative";
      const next = analytic2R(planar, pose, planar.home, chosen);
      setQ(next.q);
      setTarget(pose);
      setResult(next);
      setElbow(chosen);
      setMode("ik");
    } else {
      const next = step === 3 ? [0.3, 0] : planar.home.slice();
      setQ(next);
      setTarget(forward(planar, next).pose);
      setResult(null);
      setMode(step === 3 ? "analysis" : "fk");
    }
  };
  const compareB = (nextTask: Task) => {
    const goal = six.home.slice();
    goal[5] += 1.1;
    const pose = forward(six, goal).pose;
    const next = inverse(six, pose, six.home, nextTask);
    setPlaying(false);
    setRobot(six);
    setDemo("B");
    setMode("ik");
    setTask(nextTask);
    setFrames(true);
    setTool("translate");
    setQ(next.q);
    setTarget(pose);
    setResult(next);
    setTrail([]);
  };
  const movePath = (nextS: number, nextBranch = branch) => {
    setPlaying(false);
    const next = pathStep(planar, lecturePath, nextS, q, nextBranch);
    setResult(next);
    if (next.status !== "converged") {
      setMotionMessage(
        `Stopped: ${statusText[next.status]}. Last valid configuration retained.`,
      );
      return;
    }
    setS(nextS);
    setBranch(nextBranch);
    setQ(next.q);
    setTarget(pathTarget(lecturePath, nextS));
    setTrail((points) => [
      ...points.slice(-599),
      getPosition(forward(planar, next.q).pose),
    ]);
    setMotionMessage("Valid path sample · continuous branch.");
  };
  const startDemo = (next: Exclude<Demo, null>) => {
    setActiveJoint(null);
    setCameraView(next === "A" || next === "D" ? "top" : "default");
    setAdvanced(false);
    setPlaying(false);
    setPhase(0);
    setS(0);
    setResetView((x) => x + 1);
    if (next === "A") {
      stepA(0);
      return;
    }
    if (next === "B") {
      compareB("position");
      return;
    }
    const model = next === "C" ? panda : planar;
    const pose =
      next === "C"
        ? forward(panda, panda.home).pose
        : pathTarget(lecturePath, 0);
    const joints =
      next === "C"
        ? panda.home.slice()
        : pathStep(planar, lecturePath, 0, planar.home, "positive").q;
    setRobot(model);
    setQ(joints);
    setTarget(pose);
    setDemo(next);
    setMode(next === "C" ? "analysis" : "ik");
    setTask(next === "C" ? "pose" : "position");
    setFrames(next === "C");
    setBranch("positive");
    setTool("translate");
    setResult(null);
    setMotionMessage("Ready. Reference and joints reset.");
    setTrail([getPosition(forward(model, joints).pose)]);
  };
  useEffect(() => {
    if (!playing || (demo !== "C" && demo !== "D")) return;
    // Fixed-step kinematic playback. Hidden tabs pause instead of accumulating
    // a large jump; speed is a path velocity, not a dynamics integration input.
    const Ts = DEMO_SAMPLE_TIME;
    const timer = window.setInterval(() => {
      const current = latestMotion.current;
      if (demo === "D") {
        const nextS = Math.min(
          lecturePath.length,
          current.s + Ts * current.speed,
        );
        const next = pathStep(
          planar,
          lecturePath,
          nextS,
          current.q,
          current.branch,
        );
        if (next.status !== "converged") {
          setPlaying(false);
          setMotionMessage(`Stopped: ${statusText[next.status]}.`);
          return;
        }
        setQ(next.q);
        setS(nextS);
        setTarget(pathTarget(lecturePath, nextS));
        setResult(next);
        setTrail((points) => [
          ...points.slice(-599),
          getPosition(forward(planar, next.q).pose),
        ]);
        if (nextS >= lecturePath.length) {
          setPlaying(false);
          setMotionMessage("End of feasible path.");
        }
      } else {
        const next = redundancyStep(
          panda,
          current.q,
          current.target,
          Ts,
          Math.cos((2 * Math.PI * current.phase) / 12),
        );
        setMotionMessage(next.reason);
        if (!next.accepted) {
          setPlaying(false);
          return;
        }
        setQ(next.q);
        setPhase(current.phase + Ts);
        setResult(null);
      }
    }, Ts * 1000);
    const pauseHidden = () => {
      if (document.hidden) {
        setPlaying(false);
        setMotionMessage("Paused while the tab was hidden.");
      }
    };
    document.addEventListener("visibilitychange", pauseHidden);
    return () => {
      clearInterval(timer);
      document.removeEventListener("visibilitychange", pauseHidden);
    };
  }, [playing, demo]);
  useEffect(() => {
    const listener = () => setFullscreen(Boolean(document.fullscreenElement));
    document.addEventListener("fullscreenchange", listener);
    return () => document.removeEventListener("fullscreenchange", listener);
  }, []);
  const toggleFullscreen = async () => {
    try {
      setFullscreenMessage("");
      if (document.fullscreenElement) await document.exitFullscreen();
      else {
        setPresentation(true);
        await document.documentElement.requestFullscreen();
      }
    } catch {
      setFullscreenMessage(
        "Fullscreen could not start. Presentation layout remains available.",
      );
    }
  };

  return (
    <div
      className={`app ${presentation ? "presentation" : ""} ${advanced ? "advanced-open" : ""}`}
      data-mode={mode}
      data-demo={demo ?? ""}
    >
      <header className="topbar">
        <div className="brand-mark">
          <img src="./thi-logo.svg" alt="Technische Hochschule Ingolstadt" />
        </div>
        <div className="brand">
          <strong>Robot Kinematics Lab</strong>
        </div>
        <a
          className="robotics-lab-link"
          href="https://www.thi.de/elektro-und-informationstechnik/labore/robotik-labor/"
          target="_blank"
          rel="noopener noreferrer"
        >
          <img src="./robotics-lab.png" alt="" />
          THI Robotics Lab
        </a>
        <div className="presentation-actions">
          <button
            aria-pressed={presentation}
            onClick={() => setPresentation((x) => !x)}
          >
            Presentation mode
          </button>
          <button
            disabled={!document.fullscreenEnabled}
            onClick={toggleFullscreen}
          >
            {fullscreen ? "Exit fullscreen" : "Enter fullscreen"}
          </button>
        </div>
      </header>
      <main>
        <div className="intro">
          <div>
            <p className="eyebrow">ROBOTICS / INTERACTIVE LABORATORY</p>
            <h1>From joint space to task space.</h1>
            <p>
              Move a joint. Place a target. Explore the geometry that connects
              them.
            </p>
          </div>
          <div className="lesson-tag">
            01
            <span>
              Forward & inverse
              <br />
              kinematics
            </span>
          </div>
        </div>

        <nav className="model-tabs" aria-label="Robot model">
          {robots.map((model, i) => (
            <button
              key={model.id}
              aria-pressed={robot.id === model.id}
              onClick={() => selectRobot(model)}
            >
              <span className="model-index">0{i + 1}</span>
              <span>
                <strong>{model.name}</strong>
                <small>
                  {model.id === "planar"
                    ? "Analytic foundation"
                    : model.id === "six"
                      ? "Position & orientation"
                      : "Redundancy & joint limits"}
                </small>
              </span>
              <span className="dof">{model.joints.length} DoF</span>
            </button>
          ))}
        </nav>

        <div className="teaching-toolbar">
          <nav className="mode-tabs" aria-label="Interaction mode">
            {(
              [
                ["fk", "Forward Kinematics"],
                ["ik", "Inverse Kinematics"],
                ["analysis", "Kinematic Analysis"],
              ] as const
            ).map(([id, label]) => (
              <button
                key={id}
                aria-pressed={mode === id}
                onClick={() => changeMode(id)}
              >
                {label}
              </button>
            ))}
          </nav>
          <label className="check-row">
            <input
              type="checkbox"
              checked={advanced}
              onChange={(e) => setAdvanced(e.target.checked)}
            />{" "}
            Advanced information
          </label>
        </div>
        <nav className="demo-tabs" aria-label="Guided demonstrations">
          <span>GUIDED DEMOS</span>
          {demos.map((d) => (
            <button
              key={d.id}
              aria-pressed={demo === d.id}
              onClick={() => startDemo(d.id)}
            >
              Demo {d.id} · {d.title}
            </button>
          ))}
        </nav>
        {fullscreenMessage && (
          <p className="fullscreen-message">{fullscreenMessage}</p>
        )}
        <p className="mode-annotation">
          {demo === "D"
            ? "p(s) = p₀ + e s · s[k+1] = s[k] + Ts u[k] · IK/FK reconstruction"
            : demo === "C"
              ? "Full-pose redundancy · q̇ ∈ null J̃ · Reference flange pose fixed approximately"
              : mode === "fk"
                ? "Forward kinematics · q → T₀ₑ(q) · Move the joints and observe the flange frame"
                : mode === "ik"
                  ? "Inverse kinematics · target pose → joint configuration · Inspect the residuals"
                  : "Velocity kinematics · v = J(q) q̇ · Singular values describe local task motion"}
        </p>

        <div className="lab-grid">
          <div className="workspace-column">
            <section className="scene-card" aria-label="Robot workspace">
              <div className="card-heading">
                <div>
                  <span className="section-index">A</span>
                  <h2>Robot workspace</h2>
                </div>
                <div className="view-actions">
                  <label>
                    <input
                      type="checkbox"
                      checked={frames}
                      onChange={(e) => setFrames(e.target.checked)}
                    />{" "}
                    Joint frames
                  </label>
                  <label>
                    <input
                      type="checkbox"
                      checked={jointLabels}
                      onChange={(e) => setJointLabels(e.target.checked)}
                    />{" "}
                    Joint labels
                  </label>
                  {robot.id === "planar" && (
                    <button
                      className="camera-button"
                      onClick={() => {
                        setCameraView("top");
                        setResetView((x) => x + 1);
                      }}
                    >
                      Top View
                    </button>
                  )}
                  <button
                    className="text-button"
                    onClick={() => {
                      setCameraView("default");
                      setResetView((x) => x + 1);
                    }}
                  >
                    Reset view ↗
                  </button>
                </div>
              </div>
              <div className="scene-wrap">
                <Viewport
                  key={robot.id}
                  robot={robot}
                  q={q}
                  target={target}
                  frames={frames}
                  task={task}
                  tool={tool}
                  onTarget={changeTarget}
                  resetView={resetView}
                  cameraView={cameraView}
                  labels={jointLabels}
                  activeJoint={activeJoint}
                  showTarget={mode === "ik" || demo === "C"}
                  targetInteractive={mode === "ik" && demo !== "D"}
                  analysisTask={
                    mode === "analysis" && task === "position" && ellipsoid
                      ? "position"
                      : undefined
                  }
                  velocityAngle={
                    mode === "analysis" && robot.id === "planar"
                      ? rateAngle
                      : undefined
                  }
                  path={demo === "D" ? desiredPath : undefined}
                  trail={demo === "D" ? trail : undefined}
                />
                <div className="scene-caption">
                  <span className="dot blue" />
                  {robot.name}
                  <span className="caption-secondary">
                    {robot.id === "planar"
                      ? "XY plane · L₁ = 0.60 m · L₂ = 0.40 m"
                      : robot.id === "panda"
                        ? "Flange frame: panda_link8"
                        : "Standard DH · generic teaching model"}
                  </span>
                </div>
                <div className="axis-legend">
                  <span className="axis-x">X</span>
                  <span className="axis-y">Y</span>
                  <span className="axis-z">Z</span>
                  <span>base frame · metres</span>
                </div>
                <section className="workspace-joints" aria-label="Workspace joint controls">
                  <div className="workspace-joints-heading">
                    <button
                      type="button"
                      aria-expanded={workspaceJointsOpen}
                      aria-controls="workspace-joint-list"
                      onClick={() => setWorkspaceJointsOpen((open) => !open)}
                    >
                      Joint controls (°) {workspaceJointsOpen ? "−" : "+"}
                    </button>
                    {workspaceJointsOpen && (
                      <button type="button" className="workspace-home" disabled={demo === "C" || demo === "D"} onClick={home}>
                        Home ↺
                      </button>
                    )}
                  </div>
                  <div id="workspace-joint-list" hidden={!workspaceJointsOpen}>
                    {robot.joints.map((joint, i) => (
                      <div className="workspace-joint" data-active={activeJoint === i} key={`${robot.id}-${i}`}>
                        <label htmlFor={`workspace-joint-${i}`}>q{i + 1}</label>
                        <input
                          id={`workspace-joint-${i}`}
                          aria-label={`Joint ${i + 1} slider`}
                          type="range"
                          disabled={demo === "C" || demo === "D"}
                          min={degrees(joint.lower)}
                          max={degrees(joint.upper)}
                          step={0.1}
                          value={degrees(q[i])}
                          onFocus={() => setActiveJoint(i)}
                          onPointerDown={() => setActiveJoint(i)}
                          onBlur={() => setActiveJoint(null)}
                          onChange={(e) => changeJoint(i, e.target.valueAsNumber)}
                        />
                        <NumberField
                          label={`Joint ${i + 1} (deg)`}
                          disabled={demo === "C" || demo === "D"}
                          value={degrees(q[i])}
                          step={1}
                          onChange={(v) => changeJoint(i, v)}
                          onFocus={() => setActiveJoint(i)}
                          onBlur={() => setActiveJoint(null)}
                        />
                      </div>
                    ))}
                  </div>
                </section>
                {mode === "ik" && demo !== "D" && (
                  <div
                    className="gizmo-tools"
                    aria-label="Target manipulation mode"
                  >
                    <button
                      aria-pressed={tool === "translate"}
                      onClick={() => setTool("translate")}
                    >
                      ↔ Translate
                    </button>
                    <button
                      disabled={task !== "pose"}
                      aria-pressed={tool === "rotate"}
                      onClick={() => setTool("rotate")}
                    >
                      ↻ Rotate
                    </button>
                  </div>
                )}
              </div>
              <div className="scene-footer">
                <span>
                  <span className="dot orange" />{" "}
                  {demo === "D"
                    ? "Green: desired path · orange: FK-reconstructed trajectory"
                    : mode === "analysis" && task === "position"
                      ? "Purple: local velocity mapping · displacement scale 0.35 s"
                      : mode === "fk"
                        ? "Solid tip: actual flange · RGB arrows: coordinate axes"
                        : "Orange wireframe: target · solid tip: actual"}
                </span>
                <span>Drag to orbit · scroll to zoom · right-drag to pan</span>
              </div>
            </section>

          </div>
          <aside className="mode-panel">
            <DemoPanel
              demo={demo}
              stepA={stepA}
              compareB={compareB}
              playing={playing}
              onPlay={() => {
                setPlaying(true);
                setMotionMessage("Running within checked bounds.");
              }}
              onPause={() => setPlaying(false)}
              onReset={() => {
                if (demo) startDemo(demo);
              }}
              s={s}
              onS={(value) => movePath(value)}
              speed={speed}
              onSpeed={setSpeed}
              branch={branch}
              onBranch={(value) => movePath(s, value)}
              positionError={error.positionError}
              rotationError={error.orientationError}
              excursion={Math.hypot(...q.map((x, i) => x - robot.home[i]))}
              message={motionMessage}
              joints={q}
            />
            {mode === "analysis" && (
              <AnalysisPanel
                robot={robot}
                q={q}
                task={task}
                onTask={(next) => {
                  setTask(next);
                  setResult(null);
                  setPlaying(false);
                  if (demo === "C") setDemo(null);
                }}
                angle={rateAngle}
                onAngle={setRateAngle}
                ellipsoid={ellipsoid}
                onEllipsoid={setEllipsoid}
              />
            )}
            {mode === "fk" && !demo && (
              <section className="panel fk-guidance">
                <div className="card-heading">
                  <h2>Joint space → Cartesian space</h2>
                </div>
                <div className="panel-body">
                  <p>
                    Move a slider or type a joint angle in the workspace. The pose and
                    frame update from the same FK calculation.
                  </p>
                  <p className="formula">T₀ₑ(q) = ∏ᵢ Tᵢ(qᵢ) Ttool</p>
                  <p>
                    Switch to Inverse Kinematics to move a Cartesian target, or
                    Kinematic Analysis to inspect available velocities.
                  </p>
                </div>
              </section>
            )}
            {mode === "ik" && demo !== "D" && (
              <section
                className="target-card panel"
                aria-labelledby="target-heading"
              >
                <div className="card-heading">
                  <div>
                    <span className="section-index">B</span>
                    <h2 id="target-heading">Inverse kinematics</h2>
                  </div>
                  <span className="tiny-label">TARGET → q</span>
                </div>
                <div className="panel-body">
                  <p className="panel-hint">
                    Drag the target axes and release to solve, or enter
                    coordinates below.
                  </p>
                  <div className="segmented" aria-label="IK task">
                    <button
                      aria-pressed={task === "position"}
                      onClick={() => {
                        setTask("position");
                        setTool("translate");
                        setResult(null);
                      }}
                    >
                      Position
                    </button>
                    <button
                      disabled={robot.id === "planar"}
                      aria-pressed={task === "pose"}
                      onClick={() => {
                        setTask("pose");
                        setResult(null);
                      }}
                    >
                      Full pose
                    </button>
                  </div>
                  <p className="task-description">
                    {task === "position"
                      ? "Match XYZ. End-effector orientation is free."
                      : "Match XYZ and orientation. Rotation is solved on SO(3)."}
                  </p>
                  <div className="field-title">
                    POSITION <span>m · base frame</span>
                  </div>
                  <div className="coordinate-fields">
                    {["X", "Y", "Z"].map((label, i) => (
                      <NumberField
                        key={label}
                        label={`Target ${label} (m)`}
                        value={targetPosition[i]}
                        disabled={robot.id === "planar" && i === 2}
                        onChange={(v) => changeCoordinate(i, v)}
                      />
                    ))}
                  </div>
                  {task === "pose" && (
                    <>
                      <div className="field-title">
                        ORIENTATION <span>degrees · fixed-axis RPY</span>
                      </div>
                      <div className="coordinate-fields">
                        {["Roll", "Pitch", "Yaw"].map((label, i) => (
                          <NumberField
                            key={label}
                            label={`Target ${label} (deg)`}
                            value={degrees(targetRPY[i])}
                            step={1}
                            onChange={(v) => changeCoordinate(i, v, true)}
                          />
                        ))}
                      </div>
                    </>
                  )}
                  {robot.id === "planar" && (
                    <label className="branch-select">
                      Elbow branch
                      <select
                        aria-label="Elbow branch"
                        value={elbow}
                        onChange={(e) => {
                          setElbow(e.target.value as typeof elbow);
                          setResult(null);
                        }}
                      >
                        <option value="nearest">Nearest to current q</option>
                        <option value="positive">Positive q₂</option>
                        <option value="negative">Negative q₂</option>
                      </select>
                    </label>
                  )}
                  <div className="solve-actions">
                    <button className="primary-button" onClick={() => solve()}>
                      Solve IK <span>→</span>
                    </button>
                    <button
                      className="secondary-button"
                      onClick={() => changeTarget(fk.pose.slice())}
                    >
                      Target ← actual
                    </button>
                  </div>
                  <div
                    className={`solver-status ${result ? (result.status === "converged" ? "success" : "warning") : ""}`}
                    role="status"
                    aria-live="polite"
                  >
                    <strong>
                      {result ? statusText[result.status] : "Ready to solve"}
                    </strong>
                    <p>
                      {!result
                        ? "Edit the target, then solve from the current joints."
                        : result.status === "converged"
                          ? robot.id === "planar"
                            ? "Exact geometric solution within joint limits."
                            : `Damped least squares · ${result.iterations} iterations · ${result.attempts} attempt(s)`
                          : result.status === "not-converged"
                            ? "Local solver stopped. Reachability is undetermined; try another joint seed or position-only IK."
                            : result.status === "unreachable"
                              ? robot.id === "planar"
                                ? "Target lies outside the planar reachable annulus."
                                : "Target lies outside a conservative reach sphere."
                              : "The requested branch has no solution within the joint limits."}
                    </p>
                  </div>
                  <div className="error-readout">
                    <div>
                      <span>Position error</span>
                      <strong data-testid="position-error">
                        {fixed(error.positionError * 1000, 2)} <small>mm</small>
                      </strong>
                    </div>
                    <div>
                      <span>
                        {task === "pose" ? "Rotation error" : "Rotation (free)"}
                      </span>
                      <strong>
                        {task === "pose"
                          ? `${fixed(degrees(error.orientationError), 2)}°`
                          : "—"}
                      </strong>
                    </div>
                  </div>
                  <p className="tolerance">
                    Tolerance: 0.1 mm{task === "pose" ? " · 0.057°" : ""}.
                    Errors always describe the displayed robot.
                  </p>
                </div>
              </section>
            )}

            {(mode !== "analysis" || advanced) && (
              <section
                className="pose-card panel"
                aria-labelledby="pose-heading"
              >
                <div className="card-heading">
                  <div>
                    <span className="section-index">D</span>
                    <h2 id="pose-heading">End-effector pose</h2>
                  </div>
                  <span className="tiny-label">
                    {demo === "D" ? "FK-COMPUTED" : "ACTUAL"}
                  </span>
                </div>
                <div className="panel-body">
                  <div className="pose-values">
                    {demo === "D" && (
                      <p className="fk-position-label">FK-computed position</p>
                    )}
                    {["x", "y", "z"].map((axis, i) => (
                      <div key={axis}>
                        <span>
                          {axis} <small>m</small>
                        </span>
                        <strong data-testid={`actual-${axis}`}>
                          {fixed(p[i], 4)}
                        </strong>
                      </div>
                    ))}
                  </div>
                  <div className="rpy-values">
                    {["Roll", "Pitch", "Yaw"].map((axis, i) => (
                      <span key={axis}>
                        {axis}
                        <strong>{fixed(degrees(rpy[i]), 1)}°</strong>
                      </span>
                    ))}
                  </div>
                  <details>
                    <summary>Homogeneous transform T₀ₑ</summary>
                    <div
                      className="matrix"
                      aria-label="End effector homogeneous matrix"
                    >
                      {Array.from({ length: 4 }, (_, row) => (
                        <div key={row}>
                          {Array.from({ length: 4 }, (_, col) => (
                            <span key={col}>
                              {fixed(fk.pose[col * 4 + row])}
                            </span>
                          ))}
                        </div>
                      ))}
                    </div>
                  </details>
                  <p className="tolerance">
                    Right-handed frame, Z up. R = Rz(yaw) Ry(pitch) Rx(roll).
                    RPY is a readout; it has coordinate singularities.
                  </p>
                </div>
              </section>
            )}
            {planarDeterminant !== null && (
              <div className={`singularity-note panel ${analysis.rank < 2 ? "near-singular" : ""}`}>
                <span>det J<sub>xy</sub> = L₁L₂ sin q₂</span>
                <strong>{fixed(planarDeterminant, 4)} m²</strong>
                <span>{analysis.rank < 2 ? "Singular: one Cartesian direction is lost." : "Set q₂ = 0° or ±180° to observe a singularity."}</span>
              </div>
            )}
          </aside>
        </div>

        <section className="teaching-card">
          <div>
            <p className="eyebrow">THE GEOMETRY BEHIND THE MOTION</p>
            <h2>
              {robot.id === "planar"
                ? "Two angles. Two Cartesian coordinates."
                : robot.id === "panda"
                  ? "More joints than pose constraints."
                  : "A position is only half a pose."}
            </h2>
            <p>
              {robot.id === "planar"
                ? "The same point usually admits two elbow configurations. At a fully extended or folded arm they meet, and the planar Jacobian loses rank."
                : robot.id === "panda"
                  ? "A full pose constrains six task coordinates, while Panda has seven joints. Different joint configurations can share a flange pose. This prototype chooses a local, seed-dependent solution."
                  : "Position-only IK uses three task coordinates and leaves orientation free. Full-pose IK adds three angular constraints and may fail even where the position is reachable."}
            </p>
          </div>
          <div className="equation-box">
            {robot.id === "planar" ? (
              <>
                <p>x = L₁ cos q₁ + L₂ cos(q₁ + q₂)</p>
                <p>y = L₁ sin q₁ + L₂ sin(q₁ + q₂)</p>
                <small>Reachable radius: 0.20 m ≤ r ≤ 1.00 m</small>
              </>
            ) : (
              <>
                <p>Δq = Jᵀ(JJᵀ + λ²I)⁻¹e</p>
                <small>
                  World-frame geometric Jacobian · bounded steps
                  <br />
                  Joint-limit projection · damped least squares
                </small>
              </>
            )}
          </div>
        </section>
        {advanced && (
          <details className="jacobian-card">
            <summary>
              Inspect the geometric Jacobian J(q) · 6 × {q.length}
            </summary>
            <p>
              Rows: vₓ, vᵧ, v_z (m/rad), then ωₓ, ωᵧ, ω_z (rad/rad). The planar
              task uses only the first two rows. Spatial position IK uses the
              first three; full-pose IK uses all six with angular weight 0.35
              m/rad.
            </p>
            <div className="jacobian-matrix">
              {J.map((row, i) => (
                <div key={i}>
                  <span>{["vₓ", "vᵧ", "v_z", "ωₓ", "ωᵧ", "ω_z"][i]}</span>
                  {row.map((x, j) => (
                    <code key={j}>{fixed(x, 4)}</code>
                  ))}
                </div>
              ))}
            </div>
          </details>
        )}
        <footer>
          <span>
            THI Robot Kinematics Lab{" "}
            <small>v1.0 · Kinematics only · Primitive geometry</small>
          </span>
          <span>
            Local assets · No account or network required{" "}
            <a href="./THIRD_PARTY_NOTICES.txt">Source & license notices ↗</a>
          </span>
        </footer>
      </main>
    </div>
  );
}

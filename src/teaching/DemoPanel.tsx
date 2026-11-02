import { lecturePath } from "../kinematics/motion";
import type { Task } from "../kinematics/ik";

export type Demo = "A" | "B" | "C" | "D" | null;
export type Mode = "fk" | "ik" | "analysis";
export const demos = [
  { id: "A", title: "2R · FK & IK" },
  { id: "B", title: "6R · Position vs pose" },
  { id: "C", title: "Panda · Redundancy" },
  { id: "D", title: "2R · Cartesian path" },
] as const;

export function DemoPanel({
  demo,
  stepA,
  compareB,
  playing,
  onPlay,
  onPause,
  onReset,
  s,
  onS,
  speed,
  onSpeed,
  branch,
  onBranch,
  positionError,
  rotationError,
  excursion,
  message,
  joints,
}: {
  demo: Demo;
  stepA: (step: number) => void;
  compareB: (task: Task) => void;
  playing: boolean;
  onPlay: () => void;
  onPause: () => void;
  onReset: () => void;
  s: number;
  onS: (s: number) => void;
  speed: number;
  onSpeed: (speed: number) => void;
  branch: "positive" | "negative";
  onBranch: (branch: "positive" | "negative") => void;
  positionError: number;
  rotationError: number;
  excursion: number;
  message: string;
  joints: number[];
}) {
  if (!demo) return null;
  return (
    <section className="demo-panel panel" aria-label={`Demo ${demo}`}>
      <div className="card-heading">
        <h2>
          Demo {demo} · {demos.find((d) => d.id === demo)!.title}
        </h2>
      </div>
      <div className="panel-body">
        {demo === "A" && (
          <>
            <p>
              Start in joint space, compare both exact elbow branches at the
              same Cartesian target, then inspect rank loss.
            </p>
            <div className="demo-steps">
              {[
                "Joint space",
                "Positive elbow",
                "Negative elbow",
                "Singular configuration",
              ].map((label, i) => (
                <button key={label} onClick={() => stepA(i)}>
                  {i + 1} · {label}
                </button>
              ))}
            </div>
            <p className="formula">
              p = f(q) · cos q₂ = (x² + y² − L₁² − L₂²)/(2L₁L₂)
            </p>
          </>
        )}
        {demo === "B" && (
          <>
            <p>
              The target has the same position as the home flange, with 1.1 rad
              of wrist rotation. Compare two tasks from the same seed.
            </p>
            <div className="demo-steps">
              <button onClick={() => compareB("position")}>
                Compare position only
              </button>
              <button onClick={() => compareB("pose")}>
                Compare complete pose
              </button>
            </div>
            <p className="formula">e = Δp versus e = [Δp; 0.35 log(Rt Rᵀ)]</p>
            <p>Frame arrows show the requested and actual orientations.</p>
          </>
        )}
        {demo === "C" && (
          <>
            <p>
              Watch the joints move along a full-pose null direction. A local
              correction keeps the flange close to its reference pose.
            </p>
            <p className="formula">
              q̇ = (I − J̃⁺J̃) g(t)
              <br />
              g(t) = 0.45 cos(2πt/12) e₁ rad/s
            </p>
            <p className="analysis-explanation">
              50 Hz kinematic updates; pose preservation is approximate. The
              reference stays fixed.
            </p>
          </>
        )}
        {demo === "D" && (
          <>
            <p>
              Each desired Cartesian point is mapped to joint angles by IK, then
              back to an FK-computed position. The orange trajectory is
              reconstructed from FK; no physical motion or tracking controller
              is simulated.
            </p>
            <p className="formula">
              p(s) = [0.35, 0.20, 0] + [0.8, 0.6, 0] s<br />
              ‖e‖ = 1 · 0 ≤ s ≤ 0.40 m
            </p>
            <label className="path-label" htmlFor="path-s">
              Path coordinate s{" "}
              <strong data-testid="path-coordinate">{s.toFixed(3)} m</strong>
            </label>
            <input
              id="path-s"
              aria-label="Path coordinate s"
              type="range"
              min={0}
              max={lecturePath.length}
              step={0.001}
              value={s}
              onChange={(e) => onS(e.target.valueAsNumber)}
            />
            <label className="branch-select">
              Continuous elbow branch
              <select
                aria-label="Path elbow branch"
                value={branch}
                disabled={playing}
                onChange={(e) => onBranch(e.target.value as typeof branch)}
              >
                <option value="positive">Positive q₂</option>
                <option value="negative">Negative q₂</option>
              </select>
            </label>
            <label className="path-label" htmlFor="path-speed">
              Cartesian speed u <strong>{speed.toFixed(2)} m/s</strong>
            </label>
            <input
              id="path-speed"
              aria-label="Cartesian playback speed"
              type="range"
              min={0.02}
              max={0.2}
              step={0.01}
              value={speed}
              onChange={(e) => onSpeed(e.target.valueAsNumber)}
            />
            <p className="formula">
              s[k+1] = s[k] + Ts u[k]
              <br />
              Ts = 0.02 s · u is velocity along the line
            </p>
            <details className="mpc-connection">
              <summary>Connection to linear MPC</summary>
              <p className="analysis-explanation">
                s is the scalar state and u the Cartesian velocity input. This
                is IK/FK reconstruction playback; no dynamics or tracking
                controller is simulated.
              </p>
            </details>
          </>
        )}
        {(demo === "C" || demo === "D") && (
          <div
            className="demo-joint-values"
            aria-label={
              demo === "D"
                ? "IK joint configuration"
                : "Actual joint configuration"
            }
          >
            {joints.map((q, i) => (
              <span key={i}>
                q{i + 1}
                <strong>{((q * 180) / Math.PI).toFixed(1)}°</strong>
              </span>
            ))}
          </div>
        )}
        {(demo === "C" || demo === "D") && (
          <>
            <div className="playback-controls">
              <button
                className="primary-button"
                disabled={playing || (demo === "D" && s >= lecturePath.length)}
                onClick={onPlay}
              >
                Play
              </button>
              <button
                className="secondary-button"
                disabled={!playing}
                onClick={onPause}
              >
                Pause
              </button>
              <button className="secondary-button" onClick={onReset}>
                Reset demo
              </button>
            </div>
            <p className="motion-message" role="status">
              {playing ? "Playing · " : "Paused · "}
              {message}
            </p>
          </>
        )}
        {demo !== "A" && (
          <div className="demo-errors">
            <span>
              {demo === "D"
                ? "Cartesian reconstruction error"
                : "Actual position error"}
              <strong data-testid="demo-position-error">
                {demo === "C"
                  ? (positionError * 1000).toExponential(2)
                  : (positionError * 1000).toFixed(5)}{" "}
                mm
              </strong>
            </span>
            {demo !== "D" && (
              <span>
                Orientation difference
                {demo === "B" ? " (free in position task)" : ""}
                <strong data-testid="demo-rotation-error">
                  {demo === "C"
                    ? ((rotationError * 180) / Math.PI).toExponential(2)
                    : ((rotationError * 180) / Math.PI).toFixed(5)}
                  °
                </strong>
              </span>
            )}
          </div>
        )}
        {demo === "C" && (
          <>
            <p data-testid="joint-excursion">
              Joint displacement from home: {excursion.toFixed(3)} rad (2-norm)
            </p>
            <p className="analysis-explanation">
              Each accepted step stays inside joint limits and within 0.05 mm /
              0.029° of the reference. Motion pauses on a rejected step; actual
              errors above are never hidden.
            </p>
          </>
        )}
      </div>
    </section>
  );
}

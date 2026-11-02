import { analyze } from "../kinematics/analysis";
import type { Robot } from "../kinematics/models";
import type { Task } from "../kinematics/ik";

export function AnalysisPanel({
  robot,
  q,
  task,
  onTask,
  angle,
  onAngle,
  ellipsoid,
  onEllipsoid,
}: {
  robot: Robot;
  q: number[];
  task: Task;
  onTask: (task: Task) => void;
  angle: number;
  onAngle: (value: number) => void;
  ellipsoid: boolean;
  onEllipsoid: (value: boolean) => void;
}) {
  const info = analyze(robot, q, task);
  const rate = [Math.cos(angle), Math.sin(angle)];
  const velocity = info.J.map((row) =>
    row.reduce((s, x, i) => s + x * (rate[i] ?? 0), 0),
  );
  const scientific = (x: number) => x.toExponential(3);
  return (
    <section className="analysis-panel panel" aria-label="Kinematic analysis">
      <div className="card-heading">
        <h2>Kinematic analysis</h2>
        <span className="tiny-label">q̇ → v</span>
      </div>
      <div className="panel-body">
        {!robot.planarLengths && (
          <div className="segmented" aria-label="Analysis task">
            <button
              aria-pressed={task === "position"}
              onClick={() => onTask("position")}
            >
              Position task
            </button>
            <button
              aria-pressed={task === "pose"}
              onClick={() => onTask("pose")}
            >
              Full-pose task
            </button>
          </div>
        )}
        <p className="panel-hint">
          {robot.planarLengths
            ? "Planar Jxy maps two joint rates to XY velocity."
            : task === "position"
              ? "Jv maps joint rates to XYZ velocity; orientation is free."
              : "J̃ = [Jv; 0.35 Jω]. Angular velocity uses a characteristic length of 0.35 m/rad."}
        </p>
        <div className="analysis-metrics">
          <div>
            <span>Numerical rank</span>
            <strong data-testid="jacobian-rank">
              {info.rank} / {info.dimensions}
            </strong>
          </div>
          <div>
            <span>Joint nullity</span>
            <strong>{info.nullity}</strong>
          </div>
          <div>
            <span>σmin / σmax</span>
            <strong data-testid="singular-ratio">
              {scientific(info.ratio)}
            </strong>
          </div>
        </div>
        <p
          className={
            info.rank < info.dimensions ? "rank-warning" : "rank-summary"
          }
          data-testid="rank-description"
        >
          {!info.converged
            ? "SVD sweep limit reached; spectrum is not certified."
            : info.rank < info.dimensions
              ? "Rank deficient: at least one task velocity direction is lost."
              : "Full row rank at numerical precision. Watch σmin decrease as the robot approaches a singularity."}
        </p>
        <div className="analysis-detail">
          <div className="spectrum" aria-label="Singular values">
            {info.values.map((value, i) => (
              <div key={i}>
                <span>σ{i + 1}</span>
                <meter
                  aria-label={`Singular value ${i + 1}`}
                  min={0}
                  max={info.values[0] || 1}
                  value={value}
                />
                <code>{scientific(value)}</code>
              </div>
            ))}
          </div>
          <p className="analysis-explanation">
            Singular values: m/rad. The ratio is dimensionless. Numerical rank
            uses ε · max(m,n) · σmax = {scientific(info.tolerance)}; no
            arbitrary “near singular” cutoff is used.
          </p>
          <div className="manipulability">
            <span>∏σᵢ · task manipulability</span>
            <strong>{scientific(info.manipulability)}</strong>
            <small>
              Units (m/rad)<sup>{info.dimensions}</sup>. Compare only the same
              task and scaling.
            </small>
          </div>
          {task === "position" && (
            <label className="check-row">
              <input
                type="checkbox"
                checked={ellipsoid}
                onChange={(e) => onEllipsoid(e.target.checked)}
              />{" "}
              Show velocity {robot.planarLengths ? "ellipse" : "ellipsoid"}
            </label>
          )}
          {task === "pose" ? (
            <p className="analysis-explanation">
              The full-pose ellipsoid is six-dimensional. Switch to Position
              task to see a spatial velocity ellipsoid.
            </p>
          ) : (
            <p className="analysis-explanation">
              Unit joint-rate ball ‖q̇‖₂ ≤ 1 rad/s. Viewport displacements =
              velocity × 0.35 s; axes keep their true relative lengths.
            </p>
          )}
          {robot.planarLengths && (
            <div className="velocity-control">
              <label htmlFor="rate-direction">
                Joint-rate direction: {Math.round((angle * 180) / Math.PI)}°
              </label>
              <input
                id="rate-direction"
                aria-label="Joint-rate direction"
                type="range"
                min={0}
                max={360}
                step={1}
                value={(angle * 180) / Math.PI}
                onChange={(e) =>
                  onAngle((e.target.valueAsNumber * Math.PI) / 180)
                }
              />
              <div className="rate-mapping">
                <svg
                  viewBox="0 0 100 100"
                  role="img"
                  aria-label="Unit circle of joint rates"
                >
                  <circle cx="50" cy="50" r="35" />
                  <path d="M10 50H90M50 10V90" />
                  <line
                    x1="50"
                    y1="50"
                    x2={50 + 35 * rate[0]}
                    y2={50 - 35 * rate[1]}
                  />
                  <circle
                    className="rate-dot"
                    cx={50 + 35 * rate[0]}
                    cy={50 - 35 * rate[1]}
                    r="3"
                  />
                </svg>
                <p>
                  q̇ = [{rate.map((x) => x.toFixed(2)).join(", ")}] rad/s
                  <br />v = [{velocity.map((x) => x.toFixed(3)).join(", ")}] m/s
                  <br />
                  <strong>v = Jxy(q) q̇</strong>
                </p>
              </div>
              <p className="analysis-explanation">
                The purple arrow is the mapped velocity at the tip. At rank
                loss, the ellipse collapses to a line.
              </p>
            </div>
          )}
        </div>
      </div>
    </section>
  );
}

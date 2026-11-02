import { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { TransformControls } from "three/addons/controls/TransformControls.js";
import { forward } from "./kinematics/fk";
import { getPosition, type Mat4 } from "./kinematics/math";
import type { Robot } from "./kinematics/models";
import { analyze } from "./kinematics/analysis";
import { jacobian } from "./kinematics/fk";

interface Props {
  robot: Robot;
  q: number[];
  target: Mat4;
  frames: boolean;
  task: "position" | "pose";
  tool: "translate" | "rotate";
  onTarget: (pose: Mat4, solve: boolean) => void;
  resetView: number;
  cameraView: "default" | "top";
  labels: boolean;
  activeJoint: number | null;
  showTarget?: boolean;
  targetInteractive?: boolean;
  analysisTask?: "position";
  velocityAngle?: number;
  path?: [number, number, number][];
  trail?: [number, number, number][];
}

// This view consumes the same pure FK as the readouts, tests and IK solver.
// There is no second, visual-only kinematic chain.
export function Viewport(props: Props) {
  const mount = useRef<HTMLDivElement>(null);
  const latest = useRef(props);
  latest.current = props;
  const update = useRef<(() => void) | null>(null);
  const reset = useRef<(() => void) | null>(null);
  const [error, setError] = useState("");
  const [cameraDescription, setCameraDescription] = useState("Default view");
  const labelNodes = useRef<(HTMLSpanElement | null)[]>([]);
  const leaderNodes = useRef<(SVGLineElement | null)[]>([]);

  useEffect(() => {
    const host = mount.current!;
    let renderer: THREE.WebGLRenderer;
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true });
    } catch {
      setError(
        "WebGL is unavailable. Enable hardware acceleration to use the 3D view. The numerical controls remain available.",
      );
      return;
    }
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setClearColor("#edf2f2");
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    host.appendChild(renderer.domElement);
    renderer.domElement.setAttribute(
      "aria-label",
      "Interactive robot view. Drag the background to orbit; drag the target axes to set an IK target.",
    );
    renderer.domElement.setAttribute("role", "img");
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(38, 1, 0.01, 60);
    camera.up.set(0, 0, 1);
    let topAligned = false;
    const createOrbit = () => {
      const controls = new OrbitControls(camera, renderer.domElement);
      controls.enableDamping = true;
      controls.minDistance = 0.4;
      controls.maxDistance = 9;
      controls.addEventListener("start", () => {
        topAligned = false;
        setCameraDescription("Orbit view");
      });
      return controls;
    };
    let orbit = createOrbit();
    const fitTop = () => {
      // Frame the complete planar reachable disk, including on narrow screens.
      const radius = latest.current.robot.planarLengths!.reduce(
        (a, b) => a + b,
        0,
      );
      const distance =
        (1.15 * radius) /
        (Math.tan(THREE.MathUtils.degToRad(camera.fov / 2)) *
          Math.min(1, camera.aspect));
      camera.position.set(0, 0, distance);
      orbit.target.set(0, 0, 0);
    };
    const setCamera = () => {
      const planar = latest.current.robot.id === "planar";
      topAligned = planar && latest.current.cameraView === "top";
      // Recreate only the controls on a preset change: OrbitControls caches its
      // up-axis. This also clears residual damping before an exact top view.
      orbit.dispose();
      camera.up.set(0, topAligned ? 1 : 0, topAligned ? 0 : 1);
      camera.position.set(
        ...((planar ? [0.65, -1.25, 2.7] : [1.25, -1.5, 1.35]) as [
          number,
          number,
          number,
        ]),
      );
      orbit = createOrbit();
      orbit.target.set(
        ...((planar ? [0.25, 0.15, 0] : [0, 0, 0.5]) as [
          number,
          number,
          number,
        ]),
      );
      if (topAligned) fitTop();
      orbit.update();
      setCameraDescription(
        topAligned ? "Top view · X → · Y ↑" : "Default view",
      );
    };
    reset.current = setCamera;
    setCamera();
    scene.add(new THREE.HemisphereLight("#ffffff", "#869aa4", 2.4));
    const light = new THREE.DirectionalLight("#ffffff", 3.1);
    light.position.set(3, -2, 5);
    scene.add(light);
    const grid = new THREE.GridHelper(3.6, 36, "#a8bac0", "#d3dfe0");
    grid.rotation.x = Math.PI / 2;
    grid.position.z = -0.045;
    scene.add(grid);
    const baseAxes = new THREE.AxesHelper(0.28);
    scene.add(baseAxes);
    const base = new THREE.Mesh(
      new THREE.CylinderGeometry(0.12, 0.15, 0.045, 48),
      new THREE.MeshStandardMaterial({ color: "#203e50", roughness: 0.5 }),
    );
    base.rotation.x = Math.PI / 2;
    base.position.z = -0.023;
    scene.add(base);

    const robotGroup = new THREE.Group();
    scene.add(robotGroup);
    const jointMaterial = new THREE.MeshStandardMaterial({
      color: "#167d9a",
      metalness: 0.35,
      roughness: 0.3,
    });
    const selectedMaterial = new THREE.MeshStandardMaterial({
      color: "#e9a132",
      metalness: 0.2,
      roughness: 0.3,
    });
    const linkMaterial = new THREE.MeshStandardMaterial({
      color: "#fcfaf2",
      metalness: 0.15,
      roughness: 0.4,
    });
    const tipMaterial = new THREE.MeshStandardMaterial({
      color: "#d97b30",
      metalness: 0.2,
      roughness: 0.3,
    });
    const sphereGeometry = new THREE.SphereGeometry(0.045, 24, 16);
    const linkGeometry = new THREE.CylinderGeometry(0.031, 0.031, 1, 20);
    const joints = props.robot.joints.map(() => {
      const object = new THREE.Mesh(sphereGeometry, jointMaterial);
      robotGroup.add(object);
      return object;
    });
    // Include intermediate frame origins to show nonzero DH link offsets.
    const links = Array.from(
      { length: props.robot.joints.length * 2 + 1 },
      () => {
        const object = new THREE.Mesh(linkGeometry, linkMaterial);
        robotGroup.add(object);
        return object;
      },
    );
    const tip = new THREE.Mesh(
      new THREE.SphereGeometry(0.04, 20, 16),
      tipMaterial,
    );
    scene.add(tip);
    // Schematic orientation cue, rigidly attached to the FK flange pose.
    // Local +Z is the spatial tool direction; the planar sketch points along
    // local +X so that its two fingers are visible in the XY top view.
    const gripper = new THREE.Group();
    gripper.matrixAutoUpdate = false;
    const gripperShape = new THREE.Group();
    gripperShape.scale.setScalar(0.58);
    if (props.robot.planarLengths)
      gripperShape.quaternion.setFromAxisAngle(
        new THREE.Vector3(1, 1, 1).normalize(),
        (2 * Math.PI) / 3,
      );
    const gripperMaterial = new THREE.MeshStandardMaterial({
      color: "#254e60",
      metalness: 0.25,
      roughness: 0.4,
    });
    const fingerMarker = new THREE.MeshStandardMaterial({
      color: "#e4953b",
      metalness: 0.15,
      roughness: 0.4,
    });
    const part = (
      size: [number, number, number],
      position: [number, number, number],
      material: THREE.Material,
    ) => {
      const mesh = new THREE.Mesh(new THREE.BoxGeometry(...size), material);
      mesh.position.set(...position);
      gripperShape.add(mesh);
    };
    part([0.17, 0.045, 0.038], [0, 0, 0.028], gripperMaterial);
    part([0.032, 0.04, 0.14], [-0.069, 0, 0.102], gripperMaterial);
    part([0.032, 0.04, 0.14], [0.069, 0, 0.102], gripperMaterial);
    part([0.038, 0.046, 0.025], [0.069, 0, 0.17], fingerMarker);
    gripper.add(gripperShape);
    scene.add(gripper);
    const eeAxes = new THREE.AxesHelper(0.16);
    scene.add(eeAxes);
    const jointAxes = joints.map(() => {
      const axis = new THREE.AxesHelper(0.11);
      scene.add(axis);
      return axis;
    });
    if (props.robot.planarLengths) {
      for (const radius of [0.2, 1]) {
        const points = Array.from(
          { length: 129 },
          (_, i) =>
            new THREE.Vector3(
              radius * Math.cos((i * Math.PI) / 64),
              radius * Math.sin((i * Math.PI) / 64),
              -0.005,
            ),
        );
        const line = new THREE.LineLoop(
          new THREE.BufferGeometry().setFromPoints(points),
          new THREE.LineDashedMaterial({
            color: "#80a6ae",
            dashSize: 0.025,
            gapSize: 0.018,
          }),
        );
        line.computeLineDistances();
        scene.add(line);
      }
    }
    const target = new THREE.Group();
    const targetSphere = new THREE.Mesh(
      new THREE.SphereGeometry(0.055, 24, 16),
      new THREE.MeshBasicMaterial({
        color: "#d97b30",
        wireframe: true,
        transparent: true,
        opacity: 0.65,
      }),
    );
    target.add(targetSphere, new THREE.AxesHelper(0.2));
    scene.add(target);
    const gizmo = new TransformControls(camera, renderer.domElement);
    gizmo.setSize(0.85);
    gizmo.setSpace("world");
    gizmo.attach(target);
    scene.add(gizmo.getHelper());
    let dragging = false;
    gizmo.addEventListener("dragging-changed", (event) => {
      dragging = Boolean(event.value);
      orbit.enabled = !dragging;
      if (!dragging) {
        target.updateMatrix();
        latest.current.onTarget(target.matrix.toArray(), true);
      }
    });
    gizmo.addEventListener("objectChange", () => {
      if (!dragging) return;
      target.updateMatrix();
      latest.current.onTarget(target.matrix.toArray(), false);
    });
    const errorLine = new THREE.Line(
      new THREE.BufferGeometry(),
      new THREE.LineDashedMaterial({
        color: "#cf7a3c",
        dashSize: 0.02,
        gapSize: 0.012,
      }),
    );
    scene.add(errorLine);
    const velocityShape = new THREE.Group();
    scene.add(velocityShape);
    const shapeMaterial = new THREE.LineBasicMaterial({
      color: "#8e60bb",
      transparent: true,
      opacity: 0.7,
    });
    // Parametric great circles remain well-defined even when an axis collapses.
    const shapeLines = Array.from({ length: 3 }, () => {
      const line = new THREE.LineLoop(
        new THREE.BufferGeometry(),
        shapeMaterial,
      );
      velocityShape.add(line);
      return line;
    });
    const arrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(),
      0.1,
      "#8b46b1",
      0.035,
      0.018,
    );
    scene.add(arrow);
    const pathLine = new THREE.Line(
      new THREE.BufferGeometry(),
      new THREE.LineDashedMaterial({
        color: "#167b49",
        dashSize: 0.018,
        gapSize: 0.012,
        depthTest: false,
      }),
    );
    const trailLine = new THREE.Line(
      new THREE.BufferGeometry(),
      new THREE.LineBasicMaterial({ color: "#d97b30" }),
    );
    const pathEnds = new THREE.Points(
      new THREE.BufferGeometry(),
      new THREE.PointsMaterial({
        color: "#167b49",
        size: 0.025,
        depthTest: false,
      }),
    );
    scene.add(pathLine, trailLine, pathEnds);

    update.current = () => {
      const p = latest.current;
      const fk = forward(p.robot, p.q);
      const path = [new THREE.Vector3()];
      fk.pivots.forEach((pivot, i) => {
        joints[i].position.fromArray(getPosition(pivot));
        joints[i].material =
          p.activeJoint === i ? selectedMaterial : jointMaterial;
        joints[i].scale.setScalar(p.activeJoint === i ? 1.25 : 1);
        path.push(new THREE.Vector3().fromArray(getPosition(pivot)));
        path.push(new THREE.Vector3().fromArray(getPosition(fk.frames[i])));
        jointAxes[i].matrix.fromArray(pivot);
        jointAxes[i].matrixAutoUpdate = false;
        jointAxes[i].visible = p.frames;
      });
      path.push(new THREE.Vector3().fromArray(getPosition(fk.pose)));
      links.forEach((link, i) => {
        const a = path[i],
          b = path[i + 1];
        const direction = b.clone().sub(a);
        link.visible = direction.length() > 1e-6;
        link.position.copy(a).add(b).multiplyScalar(0.5);
        link.scale.y = direction.length();
        if (link.visible)
          link.quaternion.setFromUnitVectors(
            new THREE.Vector3(0, 1, 0),
            direction.normalize(),
          );
      });
      tip.position.fromArray(getPosition(fk.pose));
      gripper.matrix.fromArray(fk.pose);
      eeAxes.matrix.fromArray(fk.pose);
      eeAxes.matrixAutoUpdate = false;
      if (!dragging)
        new THREE.Matrix4()
          .fromArray(p.target)
          .decompose(target.position, target.quaternion, target.scale);
      gizmo.setMode(p.tool);
      gizmo.showZ = p.robot.id !== "planar";
      target.visible = p.showTarget ?? true;
      gizmo.enabled =
        (p.targetInteractive ?? true) &&
        (p.tool === "translate" || p.task === "pose");
      gizmo.getHelper().visible = p.targetInteractive ?? true;
      errorLine.visible = p.showTarget ?? true;
      velocityShape.visible = Boolean(p.analysisTask);
      if (p.analysisTask) {
        const info = analyze(p.robot, p.q, "position");
        velocityShape.position.copy(tip.position);
        shapeLines.forEach((line, circle) => {
          line.visible = info.dimensions === 3 || circle === 0;
          if (!line.visible) return;
          const a = circle === 0 ? 0 : circle === 1 ? 0 : 1;
          const b = circle === 0 ? 1 : 2;
          const points = Array.from({ length: 96 }, (_, i) => {
            const theta = (i * 2 * Math.PI) / 96;
            const coordinates = [0, 0, 0];
            for (let r = 0; r < info.dimensions; r++)
              coordinates[r] =
                0.35 *
                (info.left[a][r] * info.values[a] * Math.cos(theta) +
                  info.left[b][r] * info.values[b] * Math.sin(theta));
            return new THREE.Vector3().fromArray(coordinates);
          });
          line.geometry.dispose();
          line.geometry = new THREE.BufferGeometry().setFromPoints(points);
        });
      }
      arrow.visible = p.velocityAngle !== undefined;
      if (p.velocityAngle !== undefined) {
        const J = jacobian(p.robot, p.q);
        const v = new THREE.Vector3(
          ...(J.slice(0, 3).map(
            (row) =>
              0.35 *
              (row[0] * Math.cos(p.velocityAngle!) +
                row[1] * Math.sin(p.velocityAngle!)),
          ) as [number, number, number]),
        );
        arrow.position.copy(tip.position);
        arrow.visible = v.length() > 1e-12;
        if (arrow.visible) {
          arrow.setLength(v.length(), Math.min(0.035, v.length() / 3), 0.018);
          arrow.setDirection(v.normalize());
        }
      }
      for (const [line, points] of [
        [pathLine, p.path],
        [trailLine, p.trail],
      ] as const) {
        line.visible = Boolean(points && points.length > 1);
        line.geometry.dispose();
        line.geometry = new THREE.BufferGeometry().setFromPoints(
          (points ?? []).map((point) => new THREE.Vector3().fromArray(point)),
        );
        line.computeLineDistances();
      }
      pathEnds.visible = Boolean(p.path?.length);
      pathEnds.geometry.dispose();
      pathEnds.geometry = new THREE.BufferGeometry().setFromPoints(
        p.path?.length
          ? [p.path[0], p.path[p.path.length - 1]].map((point) =>
              new THREE.Vector3().fromArray(point),
            )
          : [],
      );
      errorLine.geometry.dispose();
      errorLine.geometry = new THREE.BufferGeometry().setFromPoints([
        tip.position.clone(),
        new THREE.Vector3().fromArray(getPosition(p.target)),
      ]);
      errorLine.computeLineDistances();
    };
    update.current();
    const resize = new ResizeObserver(() => {
      const { width, height } = host.getBoundingClientRect();
      renderer.setSize(width, height);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      if (topAligned) {
        fitTop();
        orbit.update();
      }
    });
    resize.observe(host);
    let raf = 0;
    const render = () => {
      orbit.update();
      if (latest.current.labels) {
        const { width, height } = host.getBoundingClientRect();
        const occupied: {
          x: number;
          y: number;
          width: number;
          height: number;
        }[] = [];
        // Prioritize the selected label; offset coincident wrist/joint origins
        // in screen space, keeping every leader anchored to the FK pivot.
        const order = joints
          .map((_, i) => i)
          .sort(
            (a, b) =>
              Number(b === latest.current.activeJoint) -
              Number(a === latest.current.activeJoint),
          );
        for (const i of order) {
          const node = labelNodes.current[i],
            leader = leaderNodes.current[i];
          if (!node || !leader) continue;
          const projected = joints[i].position.clone().project(camera);
          const x = ((projected.x + 1) * width) / 2,
            y = ((1 - projected.y) * height) / 2;
          const visible =
            projected.z >= -1 &&
            projected.z <= 1 &&
            x >= 0 &&
            x <= width &&
            y >= 0 &&
            y <= height;
          node.style.visibility = leader.style.visibility = visible
            ? "visible"
            : "hidden";
          if (!visible) continue;
          const w = node.offsetWidth,
            h = node.offsetHeight;
          let box = { x: x + 14, y: y - h - 12, width: w, height: h };
          const offsets = [1, -1, 2, -2, 3, -3].flatMap((row) => [
            [14, row * (h + 8) - h / 2],
            [-w - 14, row * (h + 8) - h / 2],
          ]);
          for (const [dx, dy] of offsets) {
            box = {
              x: Math.max(8, Math.min(width - w - 8, x + dx)),
              y: Math.max(72, Math.min(height - h - 40, y + dy)),
              width: w,
              height: h,
            };
            if (
              !occupied.some(
                (other) =>
                  box.x < other.x + other.width + 5 &&
                  box.x + w + 5 > other.x &&
                  box.y < other.y + other.height + 5 &&
                  box.y + h + 5 > other.y,
              )
            )
              break;
          }
          occupied.push(box);
          node.style.transform = `translate(${box.x}px, ${box.y}px)`;
          leader.setAttribute("x1", String(x));
          leader.setAttribute("y1", String(y));
          leader.setAttribute("x2", String(box.x + w / 2));
          leader.setAttribute("y2", String(box.y + h / 2));
        }
      }
      renderer.render(scene, camera);
      raf = requestAnimationFrame(render);
    };
    render();
    return () => {
      update.current = null;
      reset.current = null;
      cancelAnimationFrame(raf);
      resize.disconnect();
      gizmo.dispose();
      orbit.dispose();
      const geometries = new Set<THREE.BufferGeometry>(),
        materials = new Set<THREE.Material>();
      scene.traverse((object) => {
        if (
          object instanceof THREE.Mesh ||
          object instanceof THREE.Line ||
          object instanceof THREE.Points
        ) {
          geometries.add(object.geometry);
          const values = Array.isArray(object.material)
            ? object.material
            : [object.material];
          values.forEach((m) => materials.add(m));
        }
      });
      geometries.forEach((g) => g.dispose());
      materials.add(selectedMaterial);
      materials.forEach((m) => m.dispose());
      renderer.dispose();
      renderer.forceContextLoss();
      renderer.domElement.remove();
    };
  }, []);
  useEffect(() => {
    update.current?.();
  }, [
    props.q,
    props.target,
    props.frames,
    props.task,
    props.tool,
    props.showTarget,
    props.targetInteractive,
    props.analysisTask,
    props.velocityAngle,
    props.path,
    props.trail,
    props.activeJoint,
  ]);
  useEffect(() => {
    reset.current?.();
  }, [props.resetView, props.cameraView]);
  return (
    <div className="viewport" ref={mount}>
      <div className="camera-description">{cameraDescription}</div>
      <div
        className="joint-label-layer"
        role="group"
        aria-label="Viewport joint labels"
        hidden={!props.labels}
      >
        <svg aria-hidden="true">
          {props.robot.joints.map((_, i) => (
            <line
              key={i}
              ref={(node) => {
                leaderNodes.current[i] = node;
              }}
              data-joint={i + 1}
              data-active={props.activeJoint === i}
            />
          ))}
        </svg>
        {props.robot.joints.map((_, i) => (
          <span
            key={i}
            ref={(node) => {
              labelNodes.current[i] = node;
            }}
            className="joint-label"
            data-testid={`joint-label-${i + 1}`}
            data-active={props.activeJoint === i}
            title={`q${i + 1}: revolute joint ${i + 1}`}
          >
            q{i + 1}
          </span>
        ))}
      </div>
      {error && (
        <p className="webgl-error" role="alert">
          {error}
        </p>
      )}
    </div>
  );
}

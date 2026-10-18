import * as T from "three";
import { flightGround } from "./ground";
import { ThermalColumn } from "./thermal";
import { FlightTrail } from "./trail";
import type { Weather } from "@microagu/contracts/weather";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { ModelAssets } from "../assets/models";
import { ManagedRenderer, disposeTree } from "./resources";
import type { RenderPose } from "@microagu/contracts/pose";
import {
  canopyGeometry,
  canopyMaterial,
  poseCanopyBrakes,
  updateSuspension,
} from "@microagu/rendering/aircraft/paraglider";
import {
  rigPayload,
  poseMechanisms,
  type Mechanisms,
} from "@microagu/rendering/aircraft/mechanisms";
import { position, orientation } from "@microagu/rendering/coordinates/ned";

import { RotorPhase } from "@microagu/rendering/aircraft/mechanisms";

export class FlightView {
  private scene = new T.Scene();
  private thermal = new ThermalColumn();
  private trail = new FlightTrail();
  private renderer = new ManagedRenderer({
    antialias: true,
    logarithmicDepthBuffer: true,
  });
  private camera = new T.PerspectiveCamera(48, 1, 0.1, 5000);
  private orbit: OrbitControls;
  private payload = new T.Group();
  private canopy = new T.Group();
  private lines = new T.LineSegments(
    new T.BufferGeometry(),
    new T.LineBasicMaterial({ color: 0xd6ddd0 }),
  );
  private route = new T.Line(
    new T.BufferGeometry(),
    new T.LineBasicMaterial({
      color: 0xb7ee75,
      transparent: true,
      opacity: 0.75,
    }),
  );
  private mechanisms: Mechanisms | null = null;
  private phase = 0;
  private controller = new AbortController();
  private assets = new ModelAssets(this.controller.signal);
  private resize: ResizeObserver;
  private rotor = new RotorPhase();
  private lastTarget = new T.Vector3();
  private mode = 0;
  private first = true;
  constructor(host: HTMLElement, log: (s: string) => void) {
    host.append(this.renderer.domElement);
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.outputColorSpace = T.SRGBColorSpace;
    this.scene.background = new T.Color("#91b4bf");
    this.scene.fog = new T.Fog("#91b4bf", 700, 3500);
    this.scene.add(new T.HemisphereLight(0xe1f2ff, 0x5d6742, 2.6));
    const sun = new T.DirectionalLight(0xffedc4, 3);
    sun.position.set(-100, 300, 100);
    this.scene.add(sun);
    this.scene.add(flightGround());
    this.scene.add(this.thermal);
    this.scene.add(this.trail);
    // Deterministic distant tree line, leaving the launch area open.
    const treeGeo = new T.ConeGeometry(7, 25, 6),
      treeMat = new T.MeshStandardMaterial({ color: 0x435f50 });
    for (let i = 0; i < 100; i++) {
      const angle = i * 2.39996,
        r = 450 + (i % 13) * 34;
      const tree = new T.Mesh(treeGeo, treeMat);
      tree.position.set(Math.sin(angle) * r, 12, Math.cos(angle) * r);
      this.scene.add(tree);
    }
    this.payload.add(
      new T.Mesh(
        new T.BoxGeometry(0.18, 0.18, 0.25),
        new T.MeshStandardMaterial({ color: 0xdfddd2 }),
      ),
    );
    this.canopy.add(new T.Mesh(canopyGeometry(), canopyMaterial()));
    this.scene.add(this.payload, this.canopy, this.lines, this.route);
    this.assets
      .load(new URL("models/microagu-payload-tl.glb", document.baseURI).href)
      .then((g) => {
        disposeTree(this.payload);
        this.payload.add(g.scene);
        this.mechanisms = rigPayload(g.scene);
      })
      .catch((e) => log(`CAD model could not load: ${e.message}`));
    this.orbit = new OrbitControls(this.camera, this.renderer.domElement);
    this.orbit.enableDamping = true;
    this.orbit.maxDistance = 1000;
    this.orbit.minDistance = 1;
    this.resize = new ResizeObserver(() => {
      this.renderer.setSize(host.clientWidth, host.clientHeight);
      this.camera.aspect = host.clientWidth / host.clientHeight;
      this.camera.updateProjectionMatrix();
    });
    this.resize.observe(host);
  }
  dispose() {
    this.controller.abort();
    this.resize.disconnect();
    this.orbit.dispose();
    this.assets.dispose();
    disposeTree(this.scene);
    this.renderer.dispose();
  }
  cycle() {
    this.mode = (this.mode + 1) % 3;
    this.first = true;
    return ["chase", "orbit", "ground"][this.mode];
  }
  routePoints(pointsNED: number[][]) {
    const points = pointsNED.map(position);
    this.route.geometry.dispose();
    this.route.geometry = new T.BufferGeometry().setFromPoints(points);
  }
  weather(weather: Weather) {
    this.thermal.configure(weather);
  }
  showTrail(show: boolean) {
    this.trail.visible = show;
  }
  render(f: RenderPose, dt: number) {
    if (this.controller.signal.aborted) return;
    this.thermal.update(f.time_s);
    this.trail.record(f.time_s, f.payload.position);
    {
      this.payload.position.copy(position(f.payload.position));
      this.payload.quaternion.copy(orientation(f.payload.quaternion));
      this.canopy.visible = this.lines.visible = f.canopy !== null;
      if (f.canopy) {
        this.canopy.position.copy(position(f.canopy.position));
        this.canopy.quaternion.copy(orientation(f.canopy.quaternion));
      }
      // Illustrative pressure fill only; the solver has no cloth shape state.
      this.canopy.scale.set(1, 1, 0.15 + 0.85 * (f.inflation ?? 1));
      const brakes = f.brake_mm;
      this.phase = this.rotor.advance(f.time_s, f.rpm);
      if (this.mechanisms)
        poseMechanisms(this.mechanisms, this.phase, brakes, [0, 0]);
      poseCanopyBrakes(this.canopy, brakes);
      updateSuspension(
        this.lines,
        this.payload,
        this.canopy,
        {},
        0.066,
        undefined,
        brakes,
      );
    }
    const target = this.payload.position.clone().add(new T.Vector3(0, 0.7, 0));
    if (this.first) {
      this.camera.position
        .copy(target)
        .add(
          this.mode === 2
            ? new T.Vector3(30, 4 - target.y, 25)
            : new T.Vector3(5, 3.5, 9),
        );
      this.orbit.target.copy(target);
      this.lastTarget.copy(target);
      this.first = false;
    }
    if (this.mode === 0) {
      const forward = new T.Vector3(1, 0, 0).applyQuaternion(
        this.payload.quaternion,
      );
      // The operator flips the carriage during handling; keep the launch
      // camera on the same side until the canopy is overhead.
      const fill = Math.max(0, Math.min(1, ((f.inflation ?? 1) - 0.8) / 0.2));
      const blend = fill * fill * (3 - 2 * fill);
      const yaw = Math.atan2(forward.x, -forward.z) * blend;
      forward.set(Math.sin(yaw), 0, -Math.cos(yaw));
      forward.y = 0;
      const desired = forward
        .normalize()
        .multiplyScalar(-9)
        .add(new T.Vector3(0, 3, 0));
      // Translate with the same sampled aircraft pose; only damp the offset.
      this.camera.position.add(target.clone().sub(this.lastTarget));
      this.camera.position.lerp(
        target.clone().add(desired),
        1 - Math.exp(-dt * 3),
      );
      this.orbit.target.copy(target);
    } else if (this.mode === 1) {
      this.camera.position.add(target.clone().sub(this.lastTarget));
      this.orbit.target.copy(target);
    } else this.orbit.target.copy(target);
    this.lastTarget.copy(target);
    this.orbit.enabled = this.mode === 1;
    if (this.mode === 1) this.orbit.update();
    else this.camera.lookAt(target);
    this.renderer.render(this.scene, this.camera);
  }
}

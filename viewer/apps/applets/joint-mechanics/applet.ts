import * as T from "three";
import { registerApplet } from "@microagu/applet-ui/shell/mount";
import {
  ManagedRenderer,
  ManagedScene,
} from "@microagu/rendering/scene/resources";
import { definition } from "./definition";
import data from "./loads.json";
import "./style.css";
const el = (id: string) => document.getElementById(id)!;
const input = (id: string) => el(id) as HTMLInputElement;
const deg = (n: number) => (n * 180) / Math.PI;
const number = (n: number, digits = 3) =>
  (Math.abs(n) < 0.5 * 10 ** -digits ? 0 : n).toFixed(digits);
const spring = "#be5c2f",
  damper = "#7957a4",
  payload = "#397abd",
  canopy = "#237f78";
const defs = `<defs><marker id="arrow" markerWidth="6" markerHeight="6" refX="5" refY="3" orient="auto"><path d="M0 0L6 3L0 6Z" fill="context-stroke"/></marker></defs>`;
export const applet = registerApplet(definition, (scope) => {
  let axis = 2,
    phase = 45,
    playing = !matchMedia("(prefers-reduced-motion: reduce)").matches,
    twisting = false,
    twistPhase = 0,
    top = false;
  function playLabel() {
    el("play").textContent = playing ? "Pause motion" : "Play motion";
    el("twist-play").textContent = twisting ? "Pause twist" : "Animate twist";
  }
  function drawMechanism() {
    const a = data.axes[axis],
      s = a.samples[Math.round(phase) % 360];
    el("parameters").textContent =
      `${a.name} · stiffness ${number(a.stiffness, 5)} N·m/rad · damping ${number(a.damping, 5)} N·m·s/rad`;
    const cx = 240,
      cy = 87,
      r = 62,
      q = s.angle;
    const x = cx + r * Math.sin(q),
      y = cy - r * Math.cos(q),
      right = 410 + 70 * q;
    const coil = Array.from(
      { length: 13 },
      (_, i) =>
        `${130 + ((right - 190) * i) / 12},${190 + (i === 0 || i === 12 ? 0 : i % 2 ? 10 : -10)}`,
    ).join(" ");
    el("mechanism").innerHTML =
      `<svg viewBox="0 0 480 300" role="img" aria-label="Relative rotation about a shared attachment, with a parallel torsional spring and damper"><text x="18" y="24" fill="${payload}" font-size="17">Payload reference</text><text x="285" y="24" fill="${canopy}" font-size="17">Canopy attitude</text><circle cx="${cx}" cy="${cy}" r="${r}" fill="none" stroke="#dde7ec"/><path d="M${cx} ${cy + 32}V${cy - r}" stroke="${payload}" stroke-width="5"/><path d="M${cx - (x - cx) * 0.5} ${cy + (cy - y) * 0.5}L${x} ${y}" stroke="${canopy}" stroke-width="5"/><circle cx="${cx}" cy="${cy}" r="6" fill="#213c4a"/><text x="18" y="90" font-size="17" fill="#213c4a">Angle ${number(deg(q), 1)}°</text><text x="18" y="116" font-size="17" fill="#213c4a">Rate ${number(deg(s.rate), 1)}°/s</text><text x="285" y="112" font-size="15" fill="#536975">Shared point</text><text x="18" y="158" font-size="16" fill="#536975">Angular-domain equivalent · two parallel paths</text><path d="M80 190V254M80 190H130M80 244H190M${right} 190V254M${right - 60} 190H${right}" stroke="#6c8490" stroke-width="2" fill="none"/><polyline points="${coil}" fill="none" stroke="${spring}" stroke-width="3"/><text x="162" y="178" fill="${spring}" font-size="16">Torsional spring</text><path d="M190 229V259M190 229H280M190 259H280M${235 + 70 * q} 230V258M${235 + 70 * q} 244H${right}" stroke="${damper}" stroke-width="3" fill="none"/><text x="178" y="285" fill="${damper}" font-size="16">Rotary viscous damper</text><circle cx="80" cy="218" r="5" fill="${payload}"/><circle cx="${right}" cy="218" r="5" fill="${canopy}"/></svg>`;
    el("torques").innerHTML =
      `<table><tbody><tr><th style="color:${spring}">Spring</th><td>${number(s.spring, 5)} N·m</td></tr><tr><th style="color:${damper}">Damper</th><td>${number(s.damper, 5)} N·m</td></tr><tr><th>Combined</th><td>${number(s.total, 5)} N·m</td></tr><tr><th>Damper power, both bodies</th><td>${number(s.damper * s.rate, 5)} W</td></tr></tbody></table>`;
    el("phase-note").textContent =
      Math.abs(s.rate) < 0.015
        ? "At the turning point: relative speed is nearly zero, so damping nearly vanishes."
        : Math.abs(q) < 0.025
          ? "Passing alignment: spring torque nearly vanishes, while damping remains."
          : s.angle * s.rate < 0
            ? "Returning toward alignment: spring and damper act in opposite directions."
            : "Moving away from alignment: spring and damper both oppose that motion.";
    el("axis-note").textContent =
      axis === 1
        ? "This preset has zero explicit pitch-joint stiffness. Pitch damping remains; aerodynamic, inertial and constraint coupling can still produce pitch dynamics."
        : axis === 0
          ? "Roll damping here is small. These are effective fitted coefficients, not measurements of a physical metal spring or shock absorber."
          : "This preset uses the linear yaw spring. The geometry construction below explains its physical motivation and the optional nonlinear law.";
    input("phase").value = String(Math.round(phase));
  }
  const renderer = scope.own(
    new ManagedRenderer({ antialias: true, alpha: true }),
  );
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  el("geometry").prepend(renderer.domElement);
  const scene = scope.own(new ManagedScene());
  const camera = new T.OrthographicCamera(-0.65, 0.65, 0.3, -1.05, 0.01, 20);
  const upper = [new T.Vector3(-data.b, 0, 0), new T.Vector3(data.b, 0, 0)];
  function line(points: T.Vector3[], color: string, dashed = false) {
    const g = new T.BufferGeometry().setFromPoints(points);
    const l = new T.Line(
      g,
      dashed
        ? new T.LineDashedMaterial({ color, dashSize: 0.025, gapSize: 0.015 })
        : new T.LineBasicMaterial({ color }),
    );
    l.computeLineDistances();
    scene.add(l);
    return l;
  }
  line(upper, canopy);
  // Faint canopy outline provides context without pretending that the upper bar is full span.
  const outline = Array.from({ length: 41 }, (_, i) => {
    const t = (i / 40 - 0.5) * 2;
    return new T.Vector3(t * 0.65, 0.16 - 0.13 * t * t, 0);
  });
  const outlineLine = line(outline, "#a9c9be");
  const lowerBar = line([new T.Vector3(), new T.Vector3()], payload);
  const ropes = upper.map((p) => line([p, p.clone()], "#638190"));
  const dots = [...upper, ...upper].map((p, i) => {
    const m = new T.Mesh(
      new T.SphereGeometry(0.013, 12, 8),
      new T.MeshBasicMaterial({ color: i < 2 ? canopy : payload }),
    );
    m.position.copy(p);
    scene.add(m);
    return m;
  });
  const body = new T.Mesh(
    new T.BoxGeometry(0.11, 0.065, 0.11),
    new T.MeshBasicMaterial({
      color: payload,
      transparent: true,
      opacity: 0.6,
    }),
  );
  scene.add(body);
  const reference = line(
    [
      new T.Vector3(-0.18, -data.height, 0),
      new T.Vector3(0.18, -data.height, 0),
    ],
    "#9bb2bd",
    true,
  );
  void reference;
  let bottom = [new T.Vector3(), new T.Vector3()];
  function renderGeometry() {
    const box = el("geometry"),
      aspect = box.clientWidth / box.clientHeight;
    renderer.setSize(box.clientWidth, box.clientHeight, false);
    const h = 0.66,
      w = top ? 0.43 : Math.max(0.7, h * aspect);
    outlineLine.visible = !top;
    camera.left = -w;
    camera.right = w;
    camera.top = w / aspect;
    camera.bottom = -w / aspect;
    camera.position.set(
      ...((top ? [0, 3, 0.001] : [1.4, 1.1, 2.6]) as [number, number, number]),
    );
    camera.lookAt(0, -0.35, 0);
    camera.updateProjectionMatrix();
    camera.updateMatrixWorld();
    renderer.render(scene, camera);
    const items: [string, T.Vector3][] = [
      ["Effective canopy supports", new T.Vector3(0, 0.04, 0)],
      [
        "Payload riser attachments",
        bottom[0]
          .clone()
          .add(bottom[1])
          .multiplyScalar(0.5)
          .add(new T.Vector3(0, -0.1, 0)),
      ],
    ];
    el("labels").innerHTML = items
      .map(([label, position]) => {
        const p = position.project(camera);
        return `<span style="left:${(p.x + 1) * 50}%;top:${top ? (label.startsWith("Effective") ? 22 : 78) : (1 - p.y) * 50}%">${label}</span>`;
      })
      .join("");
  }
  function yawUpdate() {
    const angle = Number(input("yaw").value),
      s = data.yaw[angle + 60],
      psi = (angle * Math.PI) / 180;
    bottom = [-1, 1].map(
      (sign) =>
        new T.Vector3(
          sign * data.a * Math.cos(psi),
          -s.height,
          -sign * data.a * Math.sin(psi),
        ),
    );
    lowerBar.geometry.setFromPoints(bottom);
    bottom.forEach((p, i) => {
      ropes[i].geometry.setFromPoints([upper[i], p]);
      dots[i + 2].position.copy(p);
    });
    body.position.set(0, -s.height - 0.04, 0);
    body.rotation.y = psi;
    el("yaw-value").textContent = `${angle}°`;
    el("geometry-values").innerHTML =
      `<table><tbody><tr><th>Geometric payload rise</th><td>${number((data.height - s.height) * 1000, 2)} mm</td></tr><tr><th>Geometry torque on canopy</th><td>${number(s.torque)} N·m</td></tr><tr><th>Fitted linear torque</th><td>${number(s.linearTorque)} N·m</td></tr></tbody></table>`;
    const X = (d: number) => 250 + d * 3.2,
      Y = (t: number) => 112 - t * 130;
    const curve = (field: "torque" | "linearTorque") =>
      data.yaw.map((v) => `${X(v.degrees)},${Y(v[field])}`).join(" ");
    el("curve").innerHTML =
      `<svg viewBox="0 0 500 240" role="img" aria-label="Restoring torque versus relative yaw, comparing geometry and the fitted linear spring">${defs}<text x="20" y="24" fill="#536975" font-size="15">Canopy torque · N·m</text><path d="M55 112H450M250 40V190" stroke="#a6b9c3"/><text x="48" y="207" font-size="14" fill="#536975">−60°</text><text x="242" y="207" font-size="14" fill="#536975">0°</text><text x="426" y="207" font-size="14" fill="#536975">60°</text><text x="18" y="55" font-size="14" fill="#536975">+0.5</text><text x="18" y="180" font-size="14" fill="#536975">−0.5</text><polyline points="${curve("linearTorque")}" stroke="${payload}" fill="none" stroke-width="2.5" stroke-dasharray="6 4"/><polyline points="${curve("torque")}" stroke="${spring}" fill="none" stroke-width="3"/><circle cx="${X(angle)}" cy="${Y(s.torque)}" r="5" fill="${spring}"/><circle cx="${X(angle)}" cy="${Y(s.linearTorque)}" r="4" fill="${payload}"/><text x="190" y="234" font-size="14" fill="#536975">Relative yaw</text></svg>`;
    renderGeometry();
  }
  el("dimensions").innerHTML =
    `<svg viewBox="0 0 500 255" role="img" aria-label="Effective upper load centers are within the canopy arc, with 647 mm upper spacing, 132 mm lower spacing, and 850 mm vertical separation">${defs}<path d="M60 100Q250 -10 440 100" fill="none" stroke="#b5d2c9" stroke-width="10"/><path d="M185 51H315" stroke="${canopy}" stroke-width="6"/><circle cx="185" cy="51" r="6" fill="${canopy}"/><circle cx="315" cy="51" r="6" fill="${canopy}"/><path d="M237 185H263" stroke="${payload}" stroke-width="7"/><path d="M185 51L237 185M315 51L263 185" stroke="#6c8490" stroke-width="2"/><path d="M435 55V185" stroke="#213c4a" marker-end="url(#arrow)"/><text x="72" y="22" fill="${canopy}" font-size="17">Effective upper spacing: ${(2 * data.b * 1000).toFixed(0)} mm (2b)</text><text x="90" y="220" fill="${payload}" font-size="17">Lower spacing: ${(2 * data.a * 1000).toFixed(0)} mm (2a)</text><text x="442" y="115" fill="#213c4a" font-size="16">H</text><text x="442" y="138" fill="#213c4a" font-size="15">${data.height * 1000}</text><text x="442" y="159" fill="#213c4a" font-size="15">mm</text><text x="22" y="135" fill="#536975" font-size="14">Canopy arc</text><text x="22" y="155" fill="#536975" font-size="14">is schematic</text><text x="146" y="247" fill="#536975" font-size="14">Aligned, loaded configuration</text></svg>`;
  el("calibration").textContent =
    `At payload mass ${data.payloadMass.toFixed(3)} kg, these dimensions give small-angle yaw stiffness ${number(data.geometryStiffness)} N·m/rad. The preset instead uses ${number(data.axes[2].stiffness)} N·m/rad (geometry mode ${data.geometryEnabled ? "on" : "off"}). Its effective upper halfspacing ${number(data.b * 1000, 1)} mm was fitted; geometry and calibration are not independent measurements.`;
  document.querySelectorAll<HTMLButtonElement>("[data-axis]").forEach((b) =>
    scope.listen(b, "click", () => {
      axis = Number(b.dataset.axis);
      document
        .querySelectorAll<HTMLButtonElement>("[data-axis]")
        .forEach((x) => x.setAttribute("aria-pressed", String(x === b)));
      drawMechanism();
    }),
  );
  scope.listen(el("play"), "click", () => {
    playing = !playing;
    playLabel();
  });
  scope.listen(input("phase"), "input", () => {
    playing = false;
    phase = Number(input("phase").value);
    playLabel();
    drawMechanism();
  });
  scope.listen(input("yaw"), "input", () => {
    twisting = false;
    playLabel();
    yawUpdate();
  });
  scope.listen(el("twist-play"), "click", () => {
    twisting = !twisting;
    twistPhase = Math.asin(Number(input("yaw").value) / 60);
    playLabel();
  });
  document.querySelectorAll<HTMLButtonElement>("[data-view]").forEach((b) =>
    scope.listen(b, "click", () => {
      top = b.dataset.view === "top";
      el("geometry").style.height = top ? "240px" : "";
      document
        .querySelectorAll<HTMLButtonElement>("[data-view]")
        .forEach((x) => x.setAttribute("aria-pressed", String(x === b)));
      renderGeometry();
    }),
  );
  scope.observer(new ResizeObserver(renderGeometry)).observe(el("geometry"));
  let last = 0;
  function tick(now: number) {
    const dt = last ? Math.min((now - last) / 1000, 0.05) : 0;
    last = now;
    if (playing) {
      phase = (phase + (dt * 360) / data.period) % 360;
      drawMechanism();
    }
    if (twisting) {
      twistPhase += (dt * Math.PI) / 4;
      input("yaw").value = String(Math.round(60 * Math.sin(twistPhase)));
      yawUpdate();
    }
    scope.requestFrame(tick);
  }
  playLabel();
  drawMechanism();
  yawUpdate();
  scope.requestFrame(tick);
});

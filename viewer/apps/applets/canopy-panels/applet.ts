import * as T from "three";
import { registerApplet } from "@microagu/applet-ui/shell/mount";
import {
  ManagedRenderer,
  ManagedScene,
  disposeTree,
} from "@microagu/rendering/scene/resources";
import { brakeOffset } from "@microagu/rendering/aircraft/paraglider";
import { definition } from "./definition";
import data from "./loads.json";
import "./style.css";

const colors = ["#98bddb", "#b8d5e8", "#e4d8b7", "#c6ddc0", "#93c6aa"];
const cv = (v: number[]) => new T.Vector3(v[1], -v[2], -v[0]);
const fmt = (v: number[]) =>
  v.map((n) => (Math.abs(n) < 0.005 ? 0 : n).toFixed(2)).join(" / ");
const degrees = (v: number) =>
  Math.abs(v) < 1e-9 ? "0.0" : ((v * 180) / Math.PI).toFixed(1);
const el = (id: string) => document.getElementById(id)!;
export const applet = registerApplet(definition, (scope) => {
  const renderer = scope.own(
    new ManagedRenderer({ antialias: true, alpha: true }),
  );
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  el("view").prepend(renderer.domElement);
  const scene = scope.own(new ManagedScene());
  const group = new T.Group();
  scene.add(group);
  const camera = new T.OrthographicCamera(-1.6, 1.6, 1.1, -1.1, 0.1, 20);
  let caseIndex = 0,
    selected = 2,
    front = false,
    rear = false;
  function arrow(
    vector: number[],
    origin: number[],
    color: string,
    scale: number,
  ) {
    const v = cv(vector);
    if (v.length() < 1e-8) return;
    const length = v.length() * scale;
    group.add(
      new T.ArrowHelper(
        v.normalize(),
        cv(origin),
        length,
        color,
        Math.min(0.09, length * 0.22),
        Math.min(0.045, length * 0.11),
      ),
    );
  }
  function render() {
    const box = el("view");
    const aspect = box.clientWidth / box.clientHeight;
    renderer.setSize(box.clientWidth, box.clientHeight, false);
    const halfWidth = Math.max(1.3, aspect * 0.72),
      halfHeight = halfWidth / aspect;
    camera.left = -halfWidth;
    camera.right = halfWidth;
    camera.top = halfHeight;
    camera.bottom = -halfHeight;
    camera.position.set(
      ...((front ? [0, 0.22, -5] : rear ? [0, 0.8, 5] : [-2.3, 1.8, -4.5]) as [
        number,
        number,
        number,
      ]),
    );
    camera.lookAt(0, 0.08, 0);
    camera.updateProjectionMatrix();
    camera.updateMatrixWorld();
    renderer.render(scene, camera);
    el("labels").innerHTML = data.cases[caseIndex].panels
      .map((p, i) => {
        const point = cv(p.position);
        point.z -= data.chord * 0.62;
        point.project(camera);
        return `<span style="left:${(point.x + 1) * 50}%;top:${(1 - point.y) * 50}%">${i + 1}</span>`;
      })
      .join("");
  }
  function update() {
    disposeTree(group);
    const current = data.cases[caseIndex],
      radius = data.arcSpan / (2 * data.halfArc);
    const centerOffset =
      radius * (1 - Math.cos(-0.8 * data.halfArc)) -
      current.panels[0].position[2];
    current.panels.forEach((p, i) => {
      const vertices: number[] = [];
      const shape = {
        arcSpan: data.arcSpan,
        halfArc: data.halfArc,
        chord: data.chord,
      };
      const brakes = [current.left_m * 1000, current.right_m * 1000];
      const point = (t: number, u: number, deformed = true) => {
        const offset = deformed
          ? brakeOffset(t / data.halfArc, u, shape, brakes)
          : new T.Vector3();
        return cv([
          (0.5 - u) * data.chord + offset.x,
          radius * Math.sin(t) + offset.y,
          radius * (1 - Math.cos(t)) - centerOffset + offset.z,
        ]);
      };
      const start = (-1 + i * 0.4) * data.halfArc,
        end = start + 0.4 * data.halfArc;
      for (let j = 0; j < 12; j++) {
        const a = start + ((end - start) * j) / 12,
          b = start + ((end - start) * (j + 1)) / 12;
        for (let k = 0; k < 20; k++) {
          const u = k / 20,
            v = (k + 1) / 20;
          for (const p of [
            point(a, u),
            point(a, v),
            point(b, v),
            point(a, u),
            point(b, v),
            point(b, u),
          ])
            vertices.push(p.x, p.y, p.z);
        }
      }
      for (const deformed of [false, true]) {
        const edge = new T.Line(
          new T.BufferGeometry().setFromPoints(
            Array.from({ length: 25 }, (_, j) =>
              point(start + ((end - start) * j) / 24, 1, deformed),
            ),
          ),
          deformed
            ? new T.LineBasicMaterial({ color: 0xb98224 })
            : new T.LineDashedMaterial({
                color: 0x748c99,
                dashSize: 0.025,
                gapSize: 0.018,
              }),
        );
        edge.computeLineDistances();
        group.add(edge);
      }
      const geometry = new T.BufferGeometry();
      geometry.setAttribute(
        "position",
        new T.Float32BufferAttribute(vertices, 3),
      );
      const mesh = new T.Mesh(
        geometry,
        new T.MeshBasicMaterial({
          color: colors[i],
          side: T.DoubleSide,
          transparent: true,
          opacity: i === selected ? 0.95 : 0.6,
        }),
      );
      group.add(mesh);
      const edge = new T.LineSegments(
        new T.EdgesGeometry(geometry, 15),
        new T.LineBasicMaterial({ color: 0x5b7886 }),
      );
      group.add(edge);
      const center = cv(p.position);
      const dot = new T.Mesh(
        new T.SphereGeometry(0.022, 10, 8),
        new T.MeshBasicMaterial({ color: 0x213c4a }),
      );
      dot.position.copy(center);
      group.add(dot);
      const line = new T.Line(
        new T.BufferGeometry().setFromPoints([new T.Vector3(), center]),
        new T.LineDashedMaterial({
          color: 0x637e8b,
          dashSize: 0.035,
          gapSize: 0.025,
        }),
      );
      line.computeLineDistances();
      group.add(line);
      arrow(p.lift, p.position, "#237f68", 0.06);
      arrow(p.drag, p.position, "#c05228", 0.06);
      // Incoming airflow, transformed from the panel frame back to canopy FRD.
      const [x, y, z] = p.localVelocity,
        c = Math.cos(p.inclination),
        s = Math.sin(p.inclination);
      arrow(
        [-x, -(c * y - s * z), -(s * y + c * z)],
        [p.position[0] + 0.55, p.position[1], p.position[2]],
        "#397abd",
        0.06,
      );
    });
    const cg = new T.Mesh(
      new T.SphereGeometry(0.035, 12, 8),
      new T.MeshBasicMaterial({ color: 0x142b38 }),
    );
    group.add(cg);
    el("condition").textContent =
      `${current.label}: left ${(current.left_m * 1000).toFixed(0)} mm · right ${(current.right_m * 1000).toFixed(0)} mm effective brake pull. Both cases use identical airspeed and attitude.`;
    const tilt = data.brakeTiltRadPerM * (current.right_m - current.left_m);
    el("tilt-status").textContent =
      data.brakeTiltRadPerM === 0
        ? "Brake-induced inclination change: 0°. The optional roll-axis tilt term is disabled in this preset; braking changes lift and drag coefficients. The surface illustrates trailing-edge deformation separately. The angles below come from canopy curvature."
        : `Brake-induced inclination change: ${degrees(tilt)}° on every panel. The inset shows effective aerodynamic orientations; the surface separately illustrates trailing-edge deformation.`;
    el("front").innerHTML =
      `<svg viewBox="0 0 600 112" role="img" aria-label="Front view of five local aerodynamic inclinations"><text x="10" y="18" fill="#536975" font-size="13">Front view · effective aerodynamic inclination</text>${current.panels
        .map((p, i) => {
          const x = 300 - p.position[1] * 225,
            y = 48 + p.position[2] * 130;
          const dx = -Math.cos(p.inclination) * 32,
            dy = Math.sin(p.inclination) * 32;
          return `<line x1="${x - dx}" y1="${y - dy}" x2="${x + dx}" y2="${y + dy}" stroke="${colors[i]}" stroke-width="12"/><text x="${x}" y="${y + 32}" text-anchor="middle" font-size="13" fill="#213c4a">${i + 1} · ${degrees(p.inclination)}°</text>`;
        })
        .join("")}</svg>`;
    const p = current.panels[selected];
    const profile = (deformed: boolean) => {
      const pts: string[] = [];
      for (const upper of [true, false])
        for (let j = 0; j <= 40; j++) {
          const u = upper ? j / 40 : 1 - j / 40;
          const offset = deformed
            ? brakeOffset(0, u, { halfArc: 0 }, [
                p.brake_m * 1000,
                p.brake_m * 1000,
              ])
            : new T.Vector3();
          const x = 300 - u * 230 + (offset.x * 230) / data.chord;
          const z =
            (upper ? -0.05 : 0.006) * data.chord * Math.sin(Math.PI * u) +
            offset.z;
          pts.push(`${x},${120 + (z * 230) / data.chord}`);
        }
      return `M${pts.join("L")}Z`;
    };
    el("local").innerHTML =
      `<svg viewBox="0 0 380 210" role="img" aria-label="Local panel side view with computed lift, drag and incoming airflow"><defs><marker id="tip" markerWidth="6" markerHeight="6" refX="5" refY="3" orient="auto-start-reverse"><path d="M0 0L6 3L0 6Z" fill="context-stroke"/></marker></defs><path d="${profile(false)}" fill="none" stroke="#748c99" stroke-dasharray="5 4"/><path d="${profile(true)}" fill="${colors[selected]}" fill-opacity=".7" stroke="#536975"/><text x="12" y="195" font-size="12" fill="#536975">Dashed: neutral profile · deformation illustrative</text><circle cx="185" cy="115" r="4" fill="#213c4a"/><text x="260" y="145" font-size="13" fill="#536975">Forward →</text>${[
        [
          -p.localVelocity[0],
          -p.localVelocity[2],
          "#397abd",
          14,
          280,
          65,
          "Airflow",
        ],
        [
          p.lift[0],
          -Math.sin(p.inclination) * p.lift[1] +
            Math.cos(p.inclination) * p.lift[2],
          "#237f68",
          9,
          185,
          115,
          "Lift",
        ],
        [
          p.drag[0],
          -Math.sin(p.inclination) * p.drag[1] +
            Math.cos(p.inclination) * p.drag[2],
          "#c05228",
          9,
          185,
          115,
          "Drag",
        ],
      ]
        .map((v) => {
          const [x, z, color, scale, ox, oy, label] = v as [
            number,
            number,
            string,
            number,
            number,
            number,
            string,
          ];
          const ex = ox + x * scale,
            ey = oy + z * scale;
          return `<line x1="${ox}" y1="${oy}" x2="${ex}" y2="${ey}" stroke="${color}" stroke-width="3" marker-end="url(#tip)"/><text x="${ex + 8}" y="${ey - 8}" fill="${color}" font-size="13">${label}</text>`;
        })
        .join("")}</svg>`;
    el("panel-info").textContent =
      `Panel ${selected + 1} · area ${(data.area / 5).toFixed(3)} m² · brake ${(p.brake_m * 1000).toFixed(0)} mm · local angle of attack ${degrees(p.alpha)}° · lift ${Math.hypot(...p.lift).toFixed(2)} N · drag ${Math.hypot(...p.drag).toFixed(2)} N.`;
    el("brake-map").textContent =
      `Brake mapping: panels 1–2 left (${current.left_m * 1000} mm), panel 3 average (${(current.left_m + current.right_m) * 500} mm), panels 4–5 right (${current.right_m * 1000} mm).`;
    el("totals").innerHTML =
      `<table><thead><tr><th>Contribution</th><th>Components</th></tr></thead><tbody><tr><th>Sum of panel forces · N</th><td>${fmt(current.force)}</td></tr><tr><th>Panel moment arms · N·m</th><td>${fmt(current.panelMoment)}</td></tr><tr><th>Additional aero · N·m</th><td>${fmt(current.additionalMoment)}</td></tr><tr><th>Total aero moment · N·m</th><td>${fmt(current.totalMoment)}</td></tr></tbody></table>`;
    document
      .querySelectorAll<HTMLButtonElement>("[data-case]")
      .forEach((b) =>
        b.setAttribute(
          "aria-pressed",
          String(Number(b.dataset.case) === caseIndex),
        ),
      );
    document
      .querySelectorAll<HTMLButtonElement>("[data-panel]")
      .forEach((b) =>
        b.setAttribute(
          "aria-pressed",
          String(Number(b.dataset.panel) === selected),
        ),
      );
    render();
  }
  el("panels").innerHTML = colors
    .map(
      (_, i) =>
        `<button data-panel="${i}" aria-label="Inspect panel ${i + 1}">${i + 1}</button>`,
    )
    .join("");
  document.querySelectorAll<HTMLButtonElement>("[data-case]").forEach((b) =>
    scope.listen(b, "click", () => {
      caseIndex = Number(b.dataset.case);
      update();
    }),
  );
  document.querySelectorAll<HTMLButtonElement>("[data-panel]").forEach((b) =>
    scope.listen(b, "click", () => {
      selected = Number(b.dataset.panel);
      update();
    }),
  );
  document.querySelectorAll<HTMLButtonElement>("[data-view]").forEach((b) =>
    scope.listen(b, "click", () => {
      front = b.dataset.view === "front";
      rear = b.dataset.view === "rear";
      document
        .querySelectorAll<HTMLButtonElement>("[data-view]")
        .forEach((x) => x.setAttribute("aria-pressed", String(x === b)));
      render();
    }),
  );
  scope.observer(new ResizeObserver(render)).observe(el("view"));
  update();
});

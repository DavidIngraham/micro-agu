import { registerApplet } from "@microagu/applet-ui/shell/mount";
import { definition } from "./definition";
import data from "./data.json";
import "./style.css";
const el = (id: string) => document.getElementById(id)!;
const outputs = ["Throttle", "Common brake", "Differential brake"];
const colors = ["#287d79", "#c1683a", "#7b60a1"];
export const applet = registerApplet(definition, (scope) => {
  scope.defer(() => {
    el("content").replaceChildren();
    el("controls").replaceChildren();
  });
  const view = new URLSearchParams(location.search).get("view") || "coupling";
  const setup = (title: string, intro: string, note: string) => {
    el("title").textContent = title;
    el("intro").textContent = intro;
    el("note").textContent = note;
    el("eyebrow").textContent = "INSIDE COORDINATED CONTROL";
  };
  function controls(html: string) {
    el("controls").innerHTML = html;
  }
  function plot(
    rows: number[][],
    series: { col: number; label: string; color: string }[],
    time: number,
  ) {
    const canvas = el("plot") as HTMLCanvasElement,
      c = canvas.getContext("2d")!;
    canvas.width = 800;
    canvas.height = series.length * 145;
    c.clearRect(0, 0, 800, canvas.height);
    const end = rows.at(-1)![0],
      start = rows[0][0];
    series.forEach((s, j) => {
      const top = j * 145 + 30,
        ys =
          view === "replay"
            ? [...data.replay.lqi, ...data.replay.baseline].map((r) =>
                Number(r[s.col]),
              )
            : view === "observer" && s.col < 4
              ? rows.flatMap((r) => r.slice(1, 4))
              : view === "coupling"
                ? data.coupling.flatMap((rs) => rs.map((r) => r[s.col]))
                : rows.map((r) => r[s.col]),
        max = Math.max(0.01, ...ys.map(Math.abs)),
        x = (t: number) => 70 + ((t - start) / (end - start)) * 700,
        y = (v: number) => top + 48 - (v / max) * 40;
      c.fillStyle = "#405e69";
      c.font = "17px system-ui";
      c.fillText(s.label, 16, j * 145 + 20);
      c.font = "13px system-ui";
      c.fillText("±" + max.toFixed(2), 8, top + 50);
      c.strokeStyle = "#cad8dc";
      c.lineWidth = 1;
      c.beginPath();
      c.moveTo(70, y(0));
      c.lineTo(770, y(0));
      c.stroke();
      c.strokeStyle = s.color;
      c.lineWidth = 2;
      c.beginPath();
      rows.forEach((r, i) =>
        i ? c.lineTo(x(r[0]), y(r[s.col])) : c.moveTo(x(r[0]), y(r[s.col])),
      );
      c.stroke();
      c.strokeStyle = "#203e49";
      c.setLineDash([3, 4]);
      c.beginPath();
      c.moveTo(x(time), top);
      c.lineTo(x(time), top + 96);
      c.stroke();
      c.setLineDash([]);
      c.fillStyle = "#405e69";
      c.fillText(start.toFixed(0) + " s", 70, top + 115);
      c.fillText(end.toFixed(0) + " s", 735, top + 115);
    });
  }
  function slider(max: number) {
    return `<label>Time <output id="time-value"></output><input id="time" type="range" min="0" max="${max}" step="1" value="0"></label>`;
  }
  if (view === "coupling" || view === "observer" || view === "replay") {
    let channel = 0;
    const obs = view === "observer",
      replay = view === "replay";
    setup(
      obs
        ? "Predict, compare, correct"
        : replay
          ? "Tracking and internal motion"
          : "One input, several responses",
      obs
        ? "The canopy starts 5° away from its modeled relative pitch equilibrium. The observer starts unaware of that displacement. Scrub to watch the estimate catch up."
        : replay
          ? "A matched longitudinal validation case, with identical commands and specified weather/model variation. Switch controllers and scrub through the maneuver."
          : "Select a one-percentage-point actuator pulse from 1.0 to 1.5 seconds. Each input changes several state variables, even though it is only one command.",
      obs
        ? "Local linear-model demonstration with the deployed fixed observer and deterministic synthetic measurement noise. Truth is used only for comparison. This is not a nonlinear SITL replay or a measurement of real observer accuracy."
        : replay
          ? "Recorded nonlinear ArduPilot SITL, held-out case 0. Separate recorded trajectories share the same test specification; they need not occupy identical locations. No observer state is reconstructed from these recordings."
          : "Local linear-model response around the release design trim; open loop, no controller or actuator limits. Each trace has its own labeled scale. Vertical velocity here is the body-axis up component, not earth-frame climb. Small-signal illustration, not a flight-performance prediction.",
    );
    const getRows = (): number[][] =>
      obs
        ? data.observer
        : replay
          ? (channel === 0 ? data.replay.lqi : data.replay.baseline).map(
              (r) => r.slice(0, 4) as number[],
            )
          : data.coupling[channel];
    controls(
      `${obs ? '<label><input id="truth" type="checkbox" checked> Show simulator truth</label>' : `<label>${replay ? "Controller" : "Input"} <select id="channel">${(replay ? ["LQI", "PG + TECS"] : outputs).map((s, i) => `<option value="${i}">${s}</option>`).join("")}</select></label>`}${slider(getRows().length - 1)}`,
    );
    el("content").innerHTML =
      '<canvas id="plot" role="img" aria-label="Time traces with a scrubbable cursor"></canvas><div id="readout"></div>';
    function draw() {
      const rows = getRows(),
        i = Math.min(
          Number((el("time") as HTMLInputElement).value),
          rows.length - 1,
        ),
        r = rows[i];
      (el("time") as HTMLInputElement).max = String(rows.length - 1);
      el("time-value").textContent = r[0].toFixed(1) + " s";
      const series = obs
        ? [
            ...((el("truth") as HTMLInputElement).checked
              ? [
                  {
                    col: 1,
                    label: "Relative pitch truth · deg",
                    color: colors[0],
                  },
                ]
              : []),
            {
              col: 2,
              label: "Predicted relative pitch · deg",
              color: "#a38966",
            },
            {
              col: 3,
              label: "Corrected relative pitch · deg",
              color: colors[2],
            },
            {
              col: 4,
              label: "Payload pitch measurement residual · deg",
              color: colors[1],
            },
          ]
        : replay
          ? [
              { col: 1, label: "Altitude error · m", color: colors[0] },
              {
                col: 2,
                label: "Relative pitch rate · deg/s",
                color: colors[2],
              },
              { col: 3, label: "Throttle · %", color: colors[1] },
            ]
          : [
              {
                col: 1,
                label: "Forward air velocity change · m/s",
                color: colors[0],
              },
              {
                col: 2,
                label: "Body-axis up velocity change · m/s",
                color: colors[0],
              },
              {
                col: 3,
                label: "Payload yaw-rate change · deg/s",
                color: colors[1],
              },
              {
                col: 4,
                label: "Relative pitch change · deg",
                color: colors[2],
              },
            ];
      plot(rows, series, r[0]);
      el("readout").innerHTML = series
        .map(
          (s) =>
            `<span>${s.label}: <strong>${r[s.col].toFixed(3)}</strong></span>`,
        )
        .join("");
      el("detail").textContent = obs
        ? "Prediction → compare predicted payload measurements with available measurements → apply fixed correction gains to all estimated states."
        : replay
          ? "Test phase: " +
            (channel === 0 ? data.replay.lqi : data.replay.baseline)[i][4]
          : r[0] >= 1 && r[0] < 1.5
            ? outputs[channel] + " pulse is active."
            : "The pulse is off; the local aircraft model continues moving.";
    }
    scope.listen(el("time"), "input", draw);
    if (obs) scope.listen(el("truth"), "change", draw);
    else
      scope.listen(el("channel"), "change", () => {
        channel = Number((el("channel") as HTMLSelectElement).value);
        draw();
      });
    draw();
  } else if (view === "gains" || view === "motion") {
    const motion = view === "motion";
    let selected = 3,
      row = 0;
    setup(
      motion ? "Same angle. Different motion." : "Read the feedback gains",
      motion
        ? "Both examples have +5° relative pitch. Choose whether that angle is increasing or decreasing. The feedback contributions change because motion matters as well as position."
        : "Each cell shows the command change caused by one positive example deviation, with every other deviation held at zero. Select a cell to inspect its coefficient and vary that state.",
      "Values use the verified deployed gain matrix, including the feedback minus sign and state/output scaling. These are unconstrained feedback contributions, not applied servo positions. Example deviations are teaching scales, not measured standard deviations. Rotor and canopy rates are model states; canopy rates are expressed in the payload frame.",
    );
    controls(
      motion
        ? '<label>Motion <select id="direction"><option value="1">Moving away from alignment</option><option value="-1">Returning toward alignment</option></select></label>'
        : '<label>Deviation multiplier <output id="factor-value">1×</output><input id="factor" type="range" min="-2" max="2" step="0.05" value="1"></label>',
    );
    if (!motion) {
      el("content").innerHTML =
        '<div class="heatmap" id="heatmap"></div><div id="bars"></div>';
      const max = Math.max(
        ...data.physicalGain.flatMap((r) =>
          r.map((k, j) => Math.abs(k * data.scales[j])),
        ),
      );
      el("heatmap").innerHTML =
        '<div class="heat-row heading"><span>Positive example deviation</span>' +
        outputs.map((o) => `<span>${o}</span>`).join("") +
        "</div>" +
        data.names
          .map(
            (name, j) =>
              `<div class="heat-row"><span>${name}<small>+${data.scales[j]} ${data.units[j]}</small></span>${outputs
                .map((_, i) => {
                  const v = data.physicalGain[i][j] * data.scales[j];
                  return `<button data-j="${j}" data-i="${i}" style="background:${v < 0 ? `rgba(193,104,58,${0.08 + (0.72 * Math.abs(v)) / max})` : `rgba(40,125,121,${0.08 + (0.72 * Math.abs(v)) / max})`}" aria-label="${name} to ${outputs[i]}, ${v.toFixed(3)} percentage points">${v >= 0 ? "+" : ""}${v.toFixed(2)}</button>`;
                })
                .join("")}</div>`,
          )
          .join("");
      scope.listen(el("heatmap"), "click", (e) => {
        const b = (e.target as HTMLElement).closest("button");
        if (b) {
          selected = Number(b.dataset.j);
          row = Number(b.dataset.i);
          draw();
        }
      });
    } else
      el("content").innerHTML =
        '<div id="motion-picture"></div><div id="bars"></div>';
    function draw() {
      const factor = motion
        ? Number((el("direction") as HTMLSelectElement).value)
        : Number((el("factor") as HTMLInputElement).value);
      let values: number[];
      if (motion) {
        values = outputs.map(
          (_, i) =>
            data.physicalGain[i][3] * 5 +
            data.physicalGain[i][12] * 10 * factor,
        );
        el("motion-picture").innerHTML =
          `<svg viewBox="0 0 520 180" role="img" aria-label="Schematic canopy and payload at a five-degree relative angle"><path d="M155 65 Q260 -15 365 65" fill="#c5dfdb" stroke="#287d79" stroke-width="4"/><path d="M175 60L270 145L345 60" fill="none" stroke="#78909a" stroke-width="2"/><rect x="251" y="130" width="38" height="30" rx="8" fill="#417ba6"/><text x="15" y="30" font-size="18">Relative pitch +5°</text><text x="15" y="105" font-size="18">${factor > 0 ? "↻ Away" : "↺ Returning"} · ${factor * 10}°/s</text></svg>`;
        el("detail").textContent =
          "Prescribed states: payload pitch rate zero; canopy pitch rate " +
          factor * 10 +
          "°/s in the payload frame. All other deviations and integral terms are zero. No trajectory is being predicted.";
      } else {
        const deviation = data.scales[selected] * factor;
        values = outputs.map(
          (_, i) => data.physicalGain[i][selected] * deviation,
        );
        el("factor-value").textContent = factor.toFixed(2) + "×";
        el("detail").textContent =
          `${data.names[selected]} → ${outputs[row]}: stored normalized K = ${data.rawK[row][selected].toPrecision(6)}. Physical command coefficient = ${data.physicalGain[row][selected].toPrecision(5)} percentage points per ${data.units[selected]}. At ${deviation.toFixed(2)} ${data.units[selected]}, contribution = ${values[row].toFixed(3)} percentage points.`;
        el("heatmap")
          .querySelectorAll("button")
          .forEach((b) =>
            b.setAttribute(
              "aria-pressed",
              String(
                Number(b.dataset.j) === selected && Number(b.dataset.i) === row,
              ),
            ),
          );
      }
      el("bars").innerHTML = outputs
        .map(
          (o, i) =>
            `<div class="contribution"><strong>${o}</strong><span>${values[i] >= 0 ? "+" : ""}${values[i].toFixed(3)} percentage points</span><div class="bar-axis"><i style="left:${values[i] < 0 ? 50 - Math.min(50, Math.abs(values[i])) : 50}%;width:${Math.min(50, Math.abs(values[i]))}%;background:${values[i] < 0 ? colors[1] : colors[0]}"></i></div>${motion ? `<small>Angle ${(data.physicalGain[i][3] * 5).toFixed(3)} + rate ${(data.physicalGain[i][12] * 10 * factor).toFixed(3)}</small>` : ""}</div>`,
        )
        .join("");
    }
    scope.listen(
      el(motion ? "direction" : "factor"),
      motion ? "change" : "input",
      draw,
    );
    draw();
  } else {
    const online = view === "online";
    const blocks = online
      ? [
          [
            "Commands & trim",
            "Mode commands become filtered climb and turn references, an airspeed target and the matching state/control equilibrium.",
          ],
          [
            "Sensors → ArduPilot EKF",
            "Payload attitude, body rates, height, velocity and wind estimates supply nine normalized measurement channels. Synthetic airspeed is retained; no canopy truth or pitot is supplied.",
          ],
          [
            "Canopy observer",
            "Predict 16 states from the previous estimate and actual applied controls. Correct with nine measurement residuals using fixed gains. The model is recentered around the turn equilibrium.",
          ],
          [
            "LQI feedback + integrators",
            "Subtract the reference state, combine 16 state deviations and three accumulated errors, and add equilibrium/feedforward commands. Height, turn-rate and speed errors drive the integrators.",
          ],
          [
            "Engagement & allocation",
            "A two-second engagement blend, travel/slew limits and servo calibration turn the request into achievable controls. Actuator-specific anti-windup feeds the integral update.",
          ],
          [
            "Aircraft → sensors",
            "Applied throttle and brakes change aircraft motion. Sensors close the physical feedback loop. Applied controls return to the observer; requested minus applied controls return to anti-windup.",
          ],
        ]
      : [
          [
            "Measure and calibrate",
            "Use the as-built 132 mm geometry, mass/inertia and flight-log evidence to constrain the nonlinear model. Preserve uncertainty in parameters the logs do not identify.",
          ],
          [
            "Trim and linearize",
            "Find an equilibrium and perturb the nonlinear model to obtain the discrete 20 ms plant. Normalize states and inputs; make a steady-turn trim map for the reference envelope.",
          ],
          [
            "Define sensing and tracking",
            "Select the nine available measurement channels and synthesize a fixed observer. Augment the 16-state feedback plant with height, turn-rate and speed-error integrals.",
          ],
          [
            "Choose weights → solve gains",
            "Solve the discrete Riccati problem for state and actuator costs. Current selected weights penalize pitch-rate motion and common-brake effort while retaining softer speed tracking.",
          ],
          [
            "Evaluate complete nonlinear SITL",
            "Include the EKF, observer, filters, allocation, actuator slew, anti-windup, voltage compensation and mode transitions. Use training cases to choose weights.",
          ],
          [
            "Freeze → validate → export",
            "Freeze the candidate before held-out disturbances and parameter variations. Hard recovery/authority gates precede performance comparisons. Export gains, parameters, source revision and runtime hashes. Failed checks return to design; held-out results must not become an unreported tuning set.",
          ],
        ];
    setup(
      online ? "The online feedback loop" : "The offline design process",
      online
        ? "Select a block to follow the information moving through the controller. These steps repeat while LQI is eligible and active."
        : "The model and test campaign turn engineering priorities into a reproducible controller. This happens before flight; the Riccati equation is not solved onboard.",
      online
        ? "Main sequence is schematic: references also enter the feedback block directly. The observer/control update is 50 Hz. MANUAL, FBWA, assisted launch and landing keep their designated baseline behavior."
        : "Validation gates screen finite simulator trajectories. Passing them does not prove nonlinear robustness, collapse resistance or flight safety.",
    );
    el("content").innerHTML =
      (online
        ? '<div class="reference-branch"><button data-block="0">Commands & trim</button><span> ↘ feeds LQI reference and feedforward</span></div>'
        : "") +
      '<div class="flow">' +
      blocks
        .map((b, i) =>
          online && i === 0
            ? ""
            : `<button data-block="${i}"><small>${i + 1}</small>${b[0]}</button>${i < blocks.length - 1 ? '<span class="arrow" aria-hidden="true">↓</span>' : ""}`,
        )
        .join("") +
      "</div>" +
      (online
        ? '<div class="return-path">↺ Aircraft measurements → EKF → observer<br>↺ Applied controls → observer prediction<br>↺ Allocation residual → integral anti-windup<br>↳ References & trim → LQI feedback</div>'
        : '<div class="return-path">↺ Failed acceptance checks → revise model, design or command envelope → repeat validation</div>');
    function select(i: number) {
      el("detail").textContent = blocks[i][1];
      el("content")
        .querySelectorAll("button")
        .forEach((b) =>
          b.setAttribute("aria-pressed", String(i === Number(b.dataset.block))),
        );
    }
    scope.listen(el("content"), "click", (e) => {
      const b = (e.target as HTMLElement).closest("button");
      if (b) select(Number(b.dataset.block));
    });
    select(0);
  }
});

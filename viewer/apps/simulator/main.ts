import { ensureIsolation } from "@microagu/sim-browser/session/isolation";
import {
  joystick,
  constrainStick,
} from "@microagu/applet-ui/controls/joystick";
import { registerApplet } from "@microagu/applet-ui/shell/mount";
import { TruthBuffer } from "@microagu/playback/live-buffer";
import { truthPose } from "@microagu/playback/pose";
import { HOME, exampleMission, landingMission } from "./missions/default";
import { defaultWeather, windTruthLabel } from "./weather";
import { FlightPlots } from "./flight-plots";
const offset = (lat: number, lon: number) => homeOffset(lat, lon, HOME);
import { definition } from "./definition";
import "./styles.css";
import {
  altitudeLever,
  manualPower,
  pilotSteering,
} from "@microagu/sim-browser/session/pilot-input";
import { MODES } from "@microagu/contracts/telemetry";
import { SimulatorClient } from "@microagu/sim-browser/worker/client";
import { FlightView } from "@microagu/rendering/scene/flight-view";
import {
  parseMission,
  offset as homeOffset,
  type Waypoint,
} from "@microagu/contracts/mission";
export const applet = registerApplet(definition, async (scope) => {
  const el = <T extends HTMLElement = HTMLElement>(id: string) =>
    document.getElementById(id) as T;

  const sim = scope.own(new SimulatorClient(HOME, document.baseURI));

  let ready = false,
    operation = false,
    launched = false,
    uploaded = false,
    items: Waypoint[] = [];

  let roll = 0,
    pitch = 0,
    throttle = 0;

  const logs: string[] = [];

  function log(text: string) {
    logs.push(text);
    if (logs.length > 120) logs.shift();
    el("log").textContent = logs.join("\n");
    el("log").scrollTop = el("log").scrollHeight;
  }

  function status(text: string) {
    el("status").textContent = text;
  }

  const view = scope.own(new FlightView(el("scene"), log));
  const truth = new TruthBuffer();
  const plots = new FlightPlots(el("flight-plots"));
  el("flight-plots").hidden = true;
  el("plot-toggle").setAttribute("aria-pressed", "false");
  scope.property(el("plot-toggle"), "onclick", () => {
    el("flight-plots").hidden = !el("flight-plots").hidden;
    el("plot-toggle").setAttribute(
      "aria-pressed",
      String(!el("flight-plots").hidden),
    );
    plots.draw();
  });
  let lastPresentation = performance.now();
  function present(now: number) {
    const dt = Math.min(0.1, Math.max(0, (now - lastPresentation) / 1000));
    lastPresentation = now;
    const frame = truth.advance(dt);
    if (frame) view.render(truthPose(frame), dt);
    scope.requestFrame(present);
  }
  scope.requestFrame(present);

  sim.onLog = log;

  sim.onTruth = (frame) => {
    if (ready && frame.released) plots.record(frame.time_s, sim.state);
    truth.push(frame, performance.now() / 1000);
    paintWind(frame.wind_ned_m_s);
    const phase = [
      "Ground hold",
      "Assisted inflation",
      "Free flight",
      "Ground contact",
    ][frame.launch_phase ?? 2];
    if (operation && frame.launch_stage) status(frame.launch_stage + "…");
    el("joint-truth").textContent =
      `${phase} · ${Math.round((frame.inflation ?? 1) * 100)}% inflation · ${frame.time_s.toFixed(1)} s · Canopy / payload joint: ${frame.joint_rotvec_rad.map((v) => ((v * 180) / Math.PI).toFixed(1)).join(" / ")}° · ${(frame.rpm ?? 0).toFixed(0)} rpm`;
  };

  sim.onError = (text) => {
    ready = false;
    status(text);
    log(text);
    el("connection").textContent = "Simulation stopped";
    buttons();
  };

  const landingActive = () =>
    !!sim.state.landing && sim.state.landing.phase !== "aborted";

  function buttons() {
    for (const id of ["mode", "apply-wind", "controller"])
      (el(id) as HTMLButtonElement).disabled =
        !ready || operation || !!sim.state.simulationEnded;
    el<HTMLSelectElement>("controller").disabled =
      !ready || operation || landingActive();
    el<HTMLButtonElement>("launch").disabled = !ready && !launched;
    el("launch").textContent = launched ? "Reset simulation" : "Launch";
    el<HTMLInputElement>("throttle").disabled =
      !ready ||
      operation ||
      landingActive() ||
      !(manualPower(sim.state.mode) || altitudeLever(sim.state.mode));
    el<HTMLButtonElement>("altitude-open").disabled =
      !ready || operation || landingActive();
    pilotShape();
    el<HTMLButtonElement>("upload").disabled =
      !ready || operation || !items.length || !!sim.state.simulationEnded;
    el<HTMLButtonElement>("execute").disabled =
      !ready ||
      operation ||
      !uploaded ||
      !launched ||
      !!sim.state.simulationEnded;
    el<HTMLButtonElement>("example").disabled = operation;
    el<HTMLButtonElement>("landing-example").disabled = operation;
    el<HTMLInputElement>("mission-file").disabled = operation;
  }

  async function action(fn: () => Promise<void>) {
    if (operation) return;
    operation = true;
    buttons();
    try {
      await fn();
    } catch (e) {
      if (scope.disposed) return;
      const text = e instanceof Error ? e.message : String(e);
      status(text);
      log(text);
    } finally {
      operation = false;
      if (!scope.disposed) buttons();
    }
  }

  function controls() {
    const input = constrainStick(
      roll,
      pitch,
      sim.state.mode === MODES.MANUAL ? "semicircle" : "line",
    );
    [roll, pitch] = input;
    el("knob").style.transform = `translate(${roll * 52}px,${pitch * 52}px)`;
    paintThrottle();
    if (ready) sim.controls(roll, pitch, throttle);
  }
  function paintThrottle() {
    const automatic =
      landingActive() ||
      (!manualPower(sim.state.mode) && !altitudeLever(sim.state.mode));
    const percent = automatic
      ? Math.max(
          0,
          Math.min(
            100,
            Math.round(((sim.state.servos[0] ?? 1000) - 1000) / 10),
          ),
        )
      : Math.round(throttle * 100);
    el<HTMLInputElement>("throttle").value = String(percent);
    el("throttle-value").textContent =
      !landingActive() && altitudeLever(sim.state.mode)
        ? percent === 50
          ? "HOLD"
          : `${percent > 50 ? "↑" : "↓"} ${Math.abs(percent - 50) * 2}%`
        : `${percent}%`;
  }

  sim.onUpdate = () => {
    el<HTMLSelectElement>("controller").value = sim.state.controllerSelected;
    el("controller-status").textContent =
      landingActive() && sim.state.landing!.phase !== "approach"
        ? sim.state.landing!.phase === "landed"
          ? "Power off"
          : "Bounded flare"
        : sim.state.controllerActive
          ? "LQI active"
          : sim.state.controllerSelected === "lqi"
            ? "Baseline · LQI waiting"
            : "Baseline";
    const s = sim.state;
    paintThrottle();
    if (s.simulationEnded) el("connection").textContent = "Landing complete";
    el("mode-label").textContent =
      Object.keys(MODES).find((n) => MODES[n] === s.mode) ?? String(s.mode);
    el("armed").textContent = s.simulationEnded
      ? "PAUSED"
      : s.armed
        ? "ARMED"
        : "DISARMED";
    el("alt").textContent = s.alt.toFixed(1);
    el("speed").textContent = s.speed.toFixed(1);
    el("climb").textContent = s.climb.toFixed(1);
    el("heading").textContent =
      `${String(Math.round(((s.yaw * 180) / Math.PI + 360) % 360)).padStart(3, "0")}°`;
    el("mission-current").textContent = uploaded
      ? `ITEM ${s.mission} / ${items.length}`
      : "NO MISSION";
    el("alt-target").textContent =
      landingActive() && s.landing!.agl !== null
        ? `AGL ${s.landing!.agl!.toFixed(1)} m · EKF`
        : [MODES.FBWB, MODES.CRUISE, MODES.AUTO, MODES.LOITER].includes(
              s.mode,
            ) && s.targetAlt !== null
          ? `TARGET ${s.targetAlt.toFixed(0)} m`
          : "MANUAL HEIGHT";
    el<HTMLSelectElement>("mode").value =
      Object.keys(MODES).find((n) => MODES[n] === s.mode) ?? "";
    if (s.landing) status(s.landing.message);
    view.setLandingTarget(landingActive() ? s.landing!.target : null);
    buttons();
    drawMap();
  };

  scope.property(el("launch"), "onclick", () => {
    if (launched) {
      void applet.mount().catch(log);
      return;
    }
    void action(async () => {
      launched = true;
      buttons();
      await sim.launch(el<HTMLSelectElement>("mode").value || "CRUISE");
      throttle = altitudeLever(sim.state.mode) ? 0.5 : sim.trimThrottle;
      controls();
      status("Launch complete.");
    });
  });
  let lastMode = -1;
  function pilotShape() {
    const manual = sim.state.mode === MODES.MANUAL;
    el("stick").dataset.shape = manual ? "semicircle" : "line";
    const active =
      ready && !operation && !landingActive() && pilotSteering(sim.state.mode);
    el("stick").setAttribute("aria-disabled", String(!active));
    el("stick").setAttribute(
      "aria-label",
      manual
        ? "Steer left/right; pull down for both brakes"
        : "Steer left or right",
    );
    if (lastMode !== sim.state.mode) {
      lastMode = sim.state.mode;
      keys.clear();
      roll = pitch = 0;
      throttle = altitudeLever(sim.state.mode) ? 0.5 : sim.trimThrottle;
      if (!operation) controls();
      else paintThrottle();
      el("knob").style.transform = "translate(0,0)";
    }
    const height = !landingActive() && altitudeLever(sim.state.mode);
    el("throttle-label").textContent = height ? "ALTITUDE" : "THROTTLE";
    el("throttle").setAttribute(
      "aria-label",
      height
        ? "Climb or descend; release to hold altitude"
        : "Throttle percent",
    );
    el("throttle").dataset.sprung = String(height);
  }
  scope.property(el<HTMLSelectElement>("mode"), "onchange", () => {
    const next = el<HTMLSelectElement>("mode").value;
    void action(async () => {
      roll = pitch = 0;
      controls();
      await sim.mode(next);
      status(`${next} confirmed by ArduPlane.`);
    });
  });
  function closeAltitude() {
    el("altitude-form").hidden = true;
    el("altitude-open").setAttribute("aria-expanded", "false");
  }
  scope.property(el("altitude-open"), "onclick", () => {
    el("altitude-form").hidden = false;
    el("altitude-open").setAttribute("aria-expanded", "true");
    el<HTMLInputElement>("altitude-command").value = String(
      Math.round(sim.state.targetAlt ?? Math.max(5, sim.state.alt)),
    );
    el("altitude-help").textContent =
      sim.state.mode === MODES.AUTO
        ? "Changes this mission leg; later waypoints keep their planned heights."
        : [MODES.MANUAL, MODES.FBWA].includes(sim.state.mode)
          ? "Setting an altitude switches to CRUISE with automatic throttle."
          : "ArduPlane will climb or descend to this height.";
    el<HTMLInputElement>("altitude-command").focus();
    el<HTMLInputElement>("altitude-command").select();
  });
  scope.property(el("altitude-close"), "onclick", closeAltitude);
  scope.listen(el("altitude-form"), "keydown", (e) => {
    if (e.key === "Escape") closeAltitude();
  });
  scope.listen(el("altitude-form"), "submit", (e) => {
    e.preventDefault();
    const metres = Number(el<HTMLInputElement>("altitude-command").value);
    void action(async () => {
      if (!Number.isFinite(metres) || metres < 5 || metres > 300)
        throw Error("Enter 5–300 m above home.");
      if ([MODES.MANUAL, MODES.FBWA].includes(sim.state.mode))
        await sim.mode("CRUISE");
      await sim.altitude(metres);
      closeAltitude();
      status(`Altitude target ${metres} m confirmed by ArduPlane.`);
    });
  });

  scope.property(el<HTMLInputElement>("throttle"), "oninput", (e) => {
    throttle = Number((e.target as HTMLInputElement).value) / 100;
    controls();
  });

  const centerLever = () => {
    if (altitudeLever(sim.state.mode)) {
      throttle = 0.5;
      controls();
    }
  };
  for (const event of [
    "pointerup",
    "pointercancel",
    "lostpointercapture",
    "blur",
  ] as const)
    scope.listen(el("throttle"), event, centerLever);
  scope.listen(el("throttle"), "pointerdown", (event) => {
    el("throttle").setPointerCapture(event.pointerId);
  });
  scope.listen(el("throttle"), "keyup", centerLever);

  const windValues = () =>
    ["wind-speed", "wind-direction", "wind-up"].map((id) =>
      Number(el<HTMLInputElement>(id).value),
    );

  const weatherValues = () => ({
    turbulenceModel: el<HTMLSelectElement>("turbulence-model").value as
      "dryden" | "legacy",
    gustLength: Number(el<HTMLInputElement>("gust-length").value),
    verticalRatio: Number(el<HTMLInputElement>("vertical-ratio").value),
    turbulence: Number(el<HTMLInputElement>("turbulence").value),
    thermalStrength: Number(el<HTMLInputElement>("thermal-strength").value),
    thermalRadius: Number(el<HTMLInputElement>("thermal-radius").value),
    thermalNorth: Number(el<HTMLInputElement>("thermal-north").value),
    thermalEast: Number(el<HTMLInputElement>("thermal-east").value),
  });
  scope.property(el("thermal-here"), "onclick", () => {
    const p = offset(sim.state.lat, sim.state.lon);
    el<HTMLInputElement>("thermal-north").value = String(
      Math.round(Math.max(-1000, Math.min(1000, p.north))),
    );
    el<HTMLInputElement>("thermal-east").value = String(
      Math.round(Math.max(-1000, Math.min(1000, p.east))),
    );
  });
  function windLabels() {
    const weather = weatherValues();
    el("gust-length-value").textContent = `${weather.gustLength} m`;
    el("vertical-ratio-value").textContent =
      `${Math.round(weather.verticalRatio * 100)}%`;
    for (const id of ["gust-length", "vertical-ratio"])
      el<HTMLInputElement>(id).disabled = weather.turbulenceModel !== "dryden";
    el("turbulence-value").textContent = `${weather.turbulence.toFixed(1)} m/s`;
    el("thermal-strength-value").textContent =
      `${weather.thermalStrength.toFixed(1)} m/s`;
    el("thermal-radius-value").textContent = `${weather.thermalRadius} m`;
    const [speed, bearing, up] = windValues();
    el("wind-speed-value").textContent = `${speed.toFixed(1)} m/s`;
    el("wind-direction-value").textContent =
      `${bearing}° · ${["N", "NE", "E", "SE", "S", "SW", "W", "NW"][Math.round(bearing / 45) % 8]}`;
    el("wind-up-value").textContent =
      `${up > 0 ? "+" : ""}${up.toFixed(1)} m/s`;
  }

  for (const id of [
    "wind-speed",
    "wind-direction",
    "wind-up",
    "turbulence",
    "turbulence-model",
    "gust-length",
    "vertical-ratio",
    "thermal-strength",
    "thermal-radius",
  ])
    scope.property(el(id), "oninput", windLabels);

  async function applyWeather() {
    const [speed, bearing, up] = windValues();
    const weather = weatherValues();
    await sim.wind(speed, bearing, up, weather);
    view.weather(weather);
    el("wind-status").textContent =
      speed === 0 &&
      up === 0 &&
      weather.turbulence === 0 &&
      weather.thermalStrength === 0
        ? "Calm"
        : `Applied: ${speed.toFixed(1)} m/s from ${bearing}°, vertical ${up.toFixed(1)} m/s · gusts ${weather.turbulence.toFixed(1)} m/s · thermal ${weather.thermalStrength.toFixed(1)} m/s`;
  }
  scope.property(el("apply-wind"), "onclick", () => void action(applyWeather));

  const stick = joystick(
    scope,
    el("stick"),
    () =>
      ready && !operation && !landingActive() && pilotSteering(sim.state.mode),
    (x, y) => {
      roll = x;
      pitch = y;
      controls();
    },
    () => (sim.state.mode === MODES.MANUAL ? "semicircle" : "line"),
  );
  const release = () => stick.reset();
  const keys = new Set<string>();

  scope.listen(window, "keydown", (e) => {
    if (
      !ready ||
      operation ||
      landingActive() ||
      !pilotSteering(sim.state.mode) ||
      e.target instanceof HTMLInputElement ||
      e.target instanceof HTMLSelectElement ||
      ![
        "ArrowLeft",
        "ArrowRight",
        "ArrowUp",
        "ArrowDown",
        "w",
        "s",
        "W",
        "S",
      ].includes(e.key)
    )
      return;
    e.preventDefault();
    keys.add(e.key.toLowerCase());
  });

  scope.listen(window, "keyup", (e) => keys.delete(e.key.toLowerCase()));

  scope.interval(() => {
    if (!keys.size || !ready || operation || landingActive()) return;
    roll = (keys.has("arrowright") ? 1 : 0) - (keys.has("arrowleft") ? 1 : 0);
    pitch = sim.state.mode === MODES.MANUAL && keys.has("arrowdown") ? 1 : 0;
    if ([MODES.MANUAL, MODES.FBWA].includes(sim.state.mode))
      throttle = Math.max(
        0,
        Math.min(
          1,
          throttle + (keys.has("w") ? 0.01 : 0) - (keys.has("s") ? 0.01 : 0),
        ),
      );
    if (altitudeLever(sim.state.mode))
      throttle = 0.5 + (keys.has("w") ? 0.5 : 0) - (keys.has("s") ? 0.5 : 0);
    controls();
  }, 50);

  scope.listen(window, "keyup", () => {
    if (![...keys].some((k) => k.startsWith("arrow"))) release();
    if (!keys.has("w") && !keys.has("s")) centerLever();
  });

  function lostFocus() {
    keys.clear();
    release();
    centerLever();
  }

  scope.listen(window, "blur", lostFocus);

  scope.listen(document, "visibilitychange", () => {
    if (document.hidden) {
      lostFocus();
      log(
        "Tab hidden: controls centered. Simulation may pause with browser throttling.",
      );
    }
  });

  function setMission(next: Waypoint[], label: string) {
    items = next;
    uploaded = false;
    view.routePoints(
      items
        .filter((w) => w.command !== 20)
        .map((w) => {
          const p = offset(w.lat, w.lon);
          return [p.north, p.east, -(w.frame === 3 ? w.alt : w.alt - HOME.alt)];
        }),
    );
    el("mission-info").textContent =
      `${label} · ${items.length} items. Upload before flying.`;
    buttons();
    drawMap();
  }

  scope.property(el("landing-example"), "onclick", () =>
    setMission(
      landingMission(),
      "Native AUTO landing · 60 m approach, touchdown at home",
    ),
  );
  scope.property(el("example"), "onclick", () =>
    setMission(exampleMission(), "Trout Lake circuit · 60 m, ends in loiter"),
  );

  scope.property(
    el<HTMLInputElement>("mission-file"),
    "onchange",
    async (e) => {
      const file = (e.target as HTMLInputElement).files?.[0];
      if (!file) return;
      uploaded = false;
      buttons();
      try {
        if (file.size > 1e6)
          throw Error("Mission file must be smaller than 1 MB.");
        setMission(parseMission(await file.text()), file.name);
      } catch (e) {
        status(String(e));
      } finally {
        (e.target as HTMLInputElement).value = "";
      }
    },
  );

  scope.property(
    el("upload"),
    "onclick",
    () =>
      void action(async () => {
        uploaded = false;
        await sim.upload(items, (n) =>
          status(`Uploading mission item ${n} / ${items.length}…`),
        );
        uploaded = true;
        el("mission-info").textContent =
          `${items.length} items uploaded and accepted by ArduPlane.`;
        status(
          launched
            ? "Mission accepted. Select Fly mission to enter AUTO."
            : "Mission accepted. Start flight, then select Fly mission.",
        );
      }),
  );

  scope.property(
    el("execute"),
    "onclick",
    () =>
      void action(async () => {
        await sim.execute();
        status("AUTO confirmed. ArduPlane is flying the uploaded mission.");
      }),
  );

  scope.property(
    el("camera"),
    "onclick",
    () => (el("camera").textContent = `View: ${view.cycle()}`),
  );

  scope.property(
    el("fullscreen"),
    "onclick",
    () =>
      void (
        document.fullscreenElement
          ? document.exitFullscreen()
          : document.documentElement.requestFullscreen()
      ).catch((e) => status(e.message)),
  );
  let trailVisible = false;
  scope.property(el<HTMLSelectElement>("controller"), "onchange", () => {
    const scheme = el<HTMLSelectElement>("controller").value as
      "baseline" | "lqi";
    void action(async () => {
      await sim.controller(scheme);
    });
  });
  el("trail-toggle").setAttribute("aria-pressed", "false");
  scope.property(el("trail-toggle"), "onclick", () => {
    trailVisible = !trailVisible;
    view.showTrail(trailVisible);
    el("trail-toggle").setAttribute("aria-pressed", String(trailVisible));
  });

  function setPanel(open: boolean) {
    el("panel").hidden = !open;
    document.body.classList.toggle("panel-closed", !open);
    el("panel-toggle").setAttribute("aria-expanded", String(open));
  }

  scope.property(el("panel-toggle"), "onclick", () =>
    setPanel(el("panel").hidden),
  );

  function drawMap() {
    const canvas = el<HTMLCanvasElement>("map"),
      ctx = canvas.getContext("2d")!;
    const w = canvas.clientWidth || 270,
      h = 135;
    canvas.width = w * devicePixelRatio;
    canvas.height = h * devicePixelRatio;
    ctx.scale(devicePixelRatio, devicePixelRatio);
    ctx.clearRect(0, 0, w, h);
    const craft = offset(sim.state.lat, sim.state.lon),
      points = items
        .filter((w) => w.command !== 20)
        .map((w) => offset(w.lat, w.lon));
    const target = landingActive() ? sim.state.landing!.target : null;
    const all = [
      { east: 0, north: 0 },
      craft,
      ...points,
      ...(target ? [target] : []),
    ];
    const minE = Math.min(...all.map((p) => p.east)) - 40,
      maxE = Math.max(...all.map((p) => p.east)) + 40,
      minN = Math.min(...all.map((p) => p.north)) - 40,
      maxN = Math.max(...all.map((p) => p.north)) + 40;
    const scale = Math.min((w - 28) / (maxE - minE), (h - 24) / (maxN - minN));
    const x = (e: number) => w / 2 + (e - (maxE + minE) / 2) * scale,
      y = (n: number) => h / 2 - (n - (maxN + minN) / 2) * scale;
    ctx.strokeStyle = "#b7ee75";
    ctx.lineWidth = 1.4;
    ctx.beginPath();
    points.forEach((p, i) =>
      i ? ctx.lineTo(x(p.east), y(p.north)) : ctx.moveTo(x(p.east), y(p.north)),
    );
    ctx.stroke();
    ctx.font = "9px system-ui";
    points.forEach((p, i) => {
      ctx.fillStyle = "#b7ee75";
      ctx.beginPath();
      ctx.arc(x(p.east), y(p.north), 3, 0, 7);
      ctx.fill();
      ctx.fillText(String(i + 1), x(p.east) + 5, y(p.north) - 4);
    });
    if (target) {
      ctx.strokeStyle = "#ffbf70";
      ctx.beginPath();
      ctx.arc(x(target.east), y(target.north), 6, 0, Math.PI * 2);
      ctx.stroke();
      ctx.fillStyle = "#ffbf70";
      ctx.fillText("LAND", x(target.east) + 8, y(target.north));
    }
    ctx.fillStyle = "#91aeb9";
    ctx.fillText("H", x(0) - 3, y(0) + 3);
    ctx.fillText("N ↑", 9, 15);
    ctx.save();
    ctx.translate(x(craft.east), y(craft.north));
    ctx.rotate(sim.state.yaw);
    ctx.fillStyle = "#fff";
    ctx.beginPath();
    ctx.moveTo(0, -7);
    ctx.lineTo(5, 5);
    ctx.lineTo(0, 2);
    ctx.lineTo(-5, 5);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  async function boot() {
    if (!crossOriginIsolated)
      status("Enabling shared memory for the simulator…");
    if (!(await ensureIsolation(document.baseURI, scope.signal))) return;
    status("Starting ArduPlane and initializing sensors…");
    operation = true;
    buttons();
    await sim.start();
    await sim.controller("lqi");
    // Warm-up ran faster than real time while held upright. Start a fresh
    // presentation clock before the real-time launch sequence.
    truth.clear();
    await applyWeather();
    operation = false;
    ready = true;
    buttons();
    el("connection").textContent = "SITL online";
    await action(async () => {
      setMission(exampleMission(), "Trout Lake circuit · 60 m, ends in loiter");
      status("Uploading the default mission…");
      await sim.upload(items, (n) =>
        status(`Uploading mission item ${n} / ${items.length}…`),
      );
      uploaded = true;
      el("mission-info").textContent =
        `${items.length} items uploaded and accepted by ArduPlane.`;
      status("Starting the default AUTO mission…");
      launched = true;
      buttons();
      await sim.launch("AUTO");
      throttle = altitudeLever(sim.state.mode) ? 0.5 : sim.trimThrottle;
      controls();
      await sim.execute();
      status("AUTO confirmed. Flying the default circuit.");
      if (innerWidth <= 720) setPanel(false);
    });
  }

  function paintWind(wind?: number[]) {
    const valid = wind?.length === 3 && wind.every(Number.isFinite);
    const [north, east, down] = valid ? wind! : [0, 0, 0];
    const speed = Math.hypot(north, east);
    const from =
      (Math.round((Math.atan2(-east, -north) * 180) / Math.PI) + 360) % 360;
    const label = windTruthLabel(wind);
    el("wind-truth").setAttribute("aria-label", label);
    el("wind-truth").title =
      `${label}. Direction arrow points downwind; bearing is where wind comes from. Vertical speed in m/s.`;
    el("wind-truth-speed").textContent = valid ? speed.toFixed(1) : "—";
    el("wind-truth-bearing").textContent =
      valid && speed >= 0.05 ? `${from}° from` : "—";
    el("wind-direction-arrow").style.transform = `rotate(${from + 180}deg)`;
    el("wind-direction-arrow").style.opacity =
      valid && speed >= 0.05 ? "1" : "0.3";
    el("wind-truth-vertical").textContent = valid
      ? Math.abs(down).toFixed(1)
      : "—";
    el("wind-vertical-arrow").style.transform =
      `rotate(${down > 0 ? 180 : 0}deg)`;
    el("wind-vertical-arrow").style.opacity =
      valid && Math.abs(down) >= 0.05 ? "1" : "0.3";
  }

  // Reset restores the same scenario, including values left in the DOM.
  for (const [id, value] of Object.entries({
    "wind-speed": 0,
    "wind-direction": 0,
    "wind-up": 0,
    turbulence: defaultWeather.turbulence,
    "turbulence-model": defaultWeather.turbulenceModel,
    "gust-length": defaultWeather.gustLength,
    "vertical-ratio": defaultWeather.verticalRatio,
    "thermal-strength": defaultWeather.thermalStrength,
    "thermal-radius": defaultWeather.thermalRadius,
    "thermal-north": defaultWeather.thermalNorth,
    "thermal-east": defaultWeather.thermalEast,
  }))
    el<HTMLInputElement>(id).value = String(value);
  windLabels();
  paintWind();
  setPanel(false);
  drawMap();

  void boot().catch((e) => {
    if (scope.disposed) return;
    status(e.message);
    log(e.stack ?? e.message);
    el("connection").textContent = "Unable to start";
  });

  scope.interval(() => {
    if (ready && performance.now() - sim.lastReceipt > 5000) {
      el("connection").textContent = "Telemetry delayed";
    } else if (ready) el("connection").textContent = "SITL online";
  }, 1000);
});

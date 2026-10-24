import { landingHeight } from "./landing-height";
import { landingTarget } from "./landing-target";
import { RuntimeAssets } from "../assets/runtime";
import { delay } from "./delay";
import { connectPhysicsSocket } from "../ardupilot/socket-physics";
import { altitudeLever, pilotChannels } from "./pilot-input";
import {
  MAVLink,
  type Message,
  type Values,
} from "@microagu/sim-browser/ardupilot/mavlink";
import { type Home, type Waypoint } from "@microagu/contracts/mission";
import {
  Physics,
  type PhysicsConfig,
} from "@microagu/sim-browser/physics/physics";
import type { Frame } from "@microagu/contracts/recordings";
import {
  MODES,
  initialTelemetry,
  type Telemetry,
} from "@microagu/contracts/telemetry";
export class Simulator {
  private lifetime = new AbortController();
  private started = false;
  private disposed = false;
  dispose(reason: unknown = Error("Simulator disposed")) {
    if (this.disposed) return;
    this.disposed = true;
    this.stopped = true;
    this.lifetime.abort(reason);
    this.timers.forEach(clearInterval);
    this.timers = [];
    this.listeners.clear();
    this.disconnectPhysics?.();
    this.physics?.dispose();
    // Emscripten installs throwing getters for unexported runtime names.
    const pthread =
      this.module &&
      Object.getOwnPropertyDescriptor(this.module, "PThread")?.value;
    pthread?.terminateAllThreads();
  }
  constructor(
    readonly home: Home,
    private readonly warmupSpeed = 4,
  ) {
    if (!Number.isFinite(warmupSpeed) || warmupSpeed < 1 || warmupSpeed > 8)
      throw new Error("Warm-up speed must be between 1× and 8×");
    this.state = initialTelemetry(home);
  }
  state: Telemetry;
  onUpdate = () => {};
  onLog = (text: string) => {};
  onTruth = (frame: Frame) => {};
  /** Wall-clock render throttling; set to zero for simulation-time research sampling. */
  truthIntervalMs = 40;
  onError = (text: string) => {};
  physics!: Physics;
  private disconnectPhysics?: () => void;
  private lastTruth = 0;
  private stopped = false;
  private mav = new MAVLink();
  private module: any;
  private read!: (port: number, ptr: number, length: number) => number;
  private write!: (port: number, ptr: number, length: number) => number;
  private rx = 0;
  private tx = 0;
  private listeners = new Set<(m: Message) => void>();
  private queue: Uint8Array[] = [];
  private timers: ReturnType<typeof setInterval>[] = [];
  private channels = [1500, 1500, 1000, 1500, 1500, 1500, 1000, 1800];
  private busy = false;
  private lastStatus = "";
  private gainFileConfirmed = false;
  private missionItems: Waypoint[] = [];
  private nativeLandingPhase = 0;
  private landingGroundSince: number | null = null;
  private lastSensors = "";
  async start(
    url: string,
    config?: PhysicsConfig,
    physicsBase = new URL(".", url).href,
    gainFile?: Uint8Array,
  ) {
    if (this.started) throw Error("Session already started");
    this.started = true;
    this.lifetime.signal.throwIfAborted();
    if (!crossOriginIsolated)
      throw Error(
        "This browser has not enabled shared memory. Reload once after the simulator service worker activates, or use a browser supporting cross-origin isolation.",
      );
    this.physics = await Physics.load(physicsBase, config, this.home.alt);
    this.physics.groundStart(true);
    const assets = url.startsWith("file:")
      ? null
      : await RuntimeAssets.load(new URL(".", url).href);
    if (assets) {
      await assets.bytes("arduplane.js");
      url = assets.url("arduplane.js");
    }
    if (!gainFile && assets?.has("lqi-gains.bin"))
      gainFile = new Uint8Array(await assets.bytes("lqi-gains.bin"));
    const wasmBinary = assets
      ? await assets.bytes("arduplane.wasm")
      : undefined;
    this.lifetime.signal.throwIfAborted();
    const { default: create } = await import(/* @vite-ignore */ url);
    const defaults =
      this.physics.config.defaults +
      (this.physics.config.lidar
        ? `
RNGFND1_TYPE 100
RNGFND1_ORIENT 25
RNGFND1_MIN ${this.physics.config.lidar.min_m}
RNGFND1_MAX ${this.physics.config.lidar.max_m}
RC1_DZ 0
RC2_DZ 0
`
        : "") +
      "\nCP_TYPE 3\nRC6_OPTION 109\nRTL_AUTOLAND 2\n" +
      (gainFile ? "\nCP3_FILE 1\n" : "") +
      // Browser hand launch: release upright at 4.5 m/s; AP owns throttle and climb.
      "\nTKOFF_ALT 15\nTKOFF_DIST 200\nTKOFF_LVL_ALT 5\nTKOFF_LVL_PITCH 5\nTKOFF_THR_MINACC 0\nTKOFF_THR_MINSPD 2\nTKOFF_THR_DELAY 0\nTKOFF_THR_MAX 60\n" +
      `\nFLTMODE_CH 0\nLOG_DISARMED 0\nLOG_BACKEND_TYPE 0\nFS_GCS_ENABL 0\nFS_LONG_ACTN 0\nRC_OVERRIDE_TIME 1\nSIM_SPEEDUP ${this.warmupSpeed}\nSTICK_MIXING 4\nFBWB_ELEV_REV 0\n`;
    this.module = await create({
      ...(wasmBinary ? { wasmBinary } : {}),
      arguments: [
        "--model",
        "JSON:127.0.0.1",
        "--rate",
        "200",
        "--speedup",
        String(this.warmupSpeed),
        "--home",
        `${this.home.lat},${this.home.lon},${this.home.alt},0`,
        "--serial0",
        "wasm",
        "--serial1",
        "none",
        "--serial2",
        "none",
        "--defaults",
        "@ROMFS/default_params/paraglider.parm,/browser.parm",
        "--wipe",
      ],
      ...(url.startsWith("file:")
        ? {}
        : {
            locateFile: (name: string) =>
              assets?.url(name) ?? new URL(name, url).href,
            mainScriptUrlOrBlob: url,
          }),
      preRun: [
        (m: any) => {
          m.FS.writeFile("/browser.parm", defaults);
          if (gainFile) m.FS.writeFile("/lqi-gains.bin", gainFile);
          this.disconnectPhysics = connectPhysicsSocket(m, (packet) => {
            if (this.state.simulationEnded) return this.lastSensors;
            const sensors = this.physics.respond(packet);
            this.lastSensors = sensors;
            if (performance.now() - this.lastTruth >= this.truthIntervalMs) {
              this.lastTruth = performance.now();
              this.onTruth(this.physics.truth());
            }
            return sensors;
          });
        },
      ],
      print: (s: string) => this.onLog(s),
      printErr: (s: string) => this.onLog(s),
    });
    const wrap = this.module.cwrap;
    const malloc = wrap("ardupilot_malloc", "number", ["number"]);
    this.rx = malloc(8192);
    this.tx = malloc(8192);
    this.read = wrap("ardupilot_serial_read", "number", [
      "number",
      "number",
      "number",
    ]);
    this.write = wrap("ardupilot_serial_write", "number", [
      "number",
      "number",
      "number",
    ]);
    const run = async () => {
      try {
        while (!this.stopped) {
          this.poll();
          await delay(4, this.lifetime.signal);
        }
      } catch (e) {
        if (this.disposed) return;
        this.dispose(e);
        this.onError(e instanceof Error ? e.message : String(e));
      }
    };
    void run();
    this.timers.push(
      setInterval(
        () =>
          this.send("HEARTBEAT", {
            type: 6,
            autopilot: 8,
            base_mode: 0,
            system_status: 4,
            mavlink_version: 3,
          }),
        1000,
      ),
    );
    this.timers.push(setInterval(() => this.sendRC(), 50));
    await this.wait(
      (m) =>
        m.name === "HEARTBEAT" &&
        [3, 4].includes(Number(m.values.system_status)),
      45000,
    );
    for (const [id, hz] of [
      [30, 20],
      [33, 20],
      [36, 10],
      [74, 20],
      [42, 2],
      [193, 2],
      [62, 5],
      ...(this.physics.config.lidar ? [[132, 20]] : []),
    ])
      await this.command(511, [id, 1e6 / hz]);
    await this.wait(
      (m) =>
        m.name === "EKF_STATUS_REPORT" && (Number(m.values.flags) & 19) === 19,
      45000,
    );
    if (gainFile && !this.gainFileConfirmed) {
      const error = Error(
        "Firmware did not confirm a compatible LQI gain file",
      );
      this.dispose(error);
      throw error;
    }
    await this.speedup(1);
    this.onLog("EKF ready; simulation speed restored to 1× before launch.");
  }
  /** Set SITL pacing after launch; sensor and controller simulated timesteps stay unchanged. */
  async speedup(factor: number) {
    if (!Number.isFinite(factor) || factor < 1 || factor > 8)
      throw Error("Simulation speed must be between 1 and 8.");
    const accepted = this.wait(
      (m) =>
        m.name === "PARAM_VALUE" &&
        m.values.param_id === "SIM_SPEEDUP" &&
        Math.abs(Number(m.values.param_value) - factor) < 1e-5,
    );
    this.send("PARAM_SET", {
      param_id: "SIM_SPEEDUP",
      param_value: factor,
      param_type: 9,
    });
    await accepted;
  }
  private poll() {
    for (let i = 0; i < 16; i++) {
      const count = this.read(0, this.rx, 8192);
      if (!count) break;
      for (const m of this.mav.feed(
        this.module.HEAPU8.slice(this.rx, this.rx + count),
      )) {
        if (m.system !== 1 || m.component !== 1) continue;
        const v = m.values as Record<string, number>;
        switch (m.name) {
          case "HEARTBEAT":
            this.state.mode = v.custom_mode;
            this.state.armed = !!(v.base_mode & 128);
            this.state.lastHeartbeat = performance.now();
            break;
          case "ATTITUDE":
            Object.assign(this.state, {
              roll: v.roll,
              pitch: v.pitch,
              yaw: v.yaw,
              pitchRate: v.pitchspeed,
            });
            break;
          case "GLOBAL_POSITION_INT":
            Object.assign(this.state, {
              lat: v.lat / 1e7,
              lon: v.lon / 1e7,
              alt: v.relative_alt / 1000,
              climb: -v.vz / 100,
              speed: Math.hypot(v.vx, v.vy) / 100,
              course:
                Math.hypot(v.vx, v.vy) > 50
                  ? Math.atan2(v.vy, v.vx)
                  : this.state.course,
              navigationTimeS: this.physics.time,
            });
            break;
          case "NAV_CONTROLLER_OUTPUT":
            this.state.crossTrack = v.xtrack_error;
            this.state.targetAlt = this.state.alt + v.alt_error;
            this.state.targetRoll = v.nav_roll;
            break;
          case "VFR_HUD":
            // Landing uses GLOBAL_POSITION_INT as its single fused velocity source.
            break;
          case "DISTANCE_SENSOR":
            if (v.id === 0 && v.orientation === 25) {
              this.state.rangefinder = {
                distanceM: v.current_distance / 100,
                minM: v.min_distance / 100,
                maxM: v.max_distance / 100,
                valid:
                  v.current_distance >= v.min_distance &&
                  v.current_distance <= v.max_distance &&
                  v.signal_quality !== 1,
                receivedTimeS: this.physics.time,
                timeBootMs: v.time_boot_ms,
              };
            }
            break;
          case "SERVO_OUTPUT_RAW":
            if (v.port === 0)
              this.state.servos = [v.servo1_raw, v.servo2_raw, v.servo3_raw];
            break;
          case "MISSION_CURRENT":
            this.state.mission = v.seq;
            break;
          case "NAMED_VALUE_FLOAT":
            if (String(v.name) === "PG_LAND")
              this.nativeLandingPhase = Number(v.value);
            if (String(v.name) === "EKF_HAGL")
              this.state.hagl = {
                metres: Number(v.value),
                valid: Number.isFinite(v.value) && v.value >= 0,
                timeBootMs: v.time_boot_ms,
                receivedTimeS: this.physics.time,
              };
            if (String(v.name) === "MIMO")
              this.state.controllerActive = Number(v.value) > 0.5;
            break;
          case "STATUSTEXT":
            this.lastStatus = String(m.values.text);
            if (this.lastStatus === "LQI gain file loaded")
              this.gainFileConfirmed = true;
            this.onLog(this.lastStatus);
            break;
        }
        for (const listener of this.listeners) listener(m);
        this.onUpdate();
      }
    }
    this.updateLanding();
    while (this.queue.length) {
      const bytes = this.queue[0];
      this.module.HEAPU8.set(bytes, this.tx);
      const sent = this.write(0, this.tx, bytes.length);
      if (sent === bytes.length) this.queue.shift();
      else {
        this.queue[0] = bytes.slice(sent);
        break;
      }
    }
  }
  send(name: string, values: Values = {}) {
    if (this.queue.length > 200)
      throw Error("Simulator serial link is stalled. Reset the simulator.");
    this.queue.push(
      this.mav.encode(name, {
        target_system: 1,
        target_component: 1,
        ...values,
      }),
    );
  }
  private wait(predicate: (m: Message) => boolean, timeout = 5000) {
    return new Promise<Message>((resolve, reject) => {
      this.lifetime.signal.throwIfAborted();
      const cleanup = () => {
        clearTimeout(timer);
        this.listeners.delete(listener);
        this.lifetime.signal.removeEventListener("abort", abort);
      };
      const abort = () => {
        cleanup();
        reject(this.lifetime.signal.reason);
      };
      const listener = (m: Message) => {
        if (predicate(m)) {
          cleanup();
          resolve(m);
        }
      };
      const timer = setTimeout(() => {
        cleanup();
        reject(Error("Timed out waiting for ArduPlane. Check the event log."));
      }, timeout);
      this.listeners.add(listener);
      this.lifetime.signal.addEventListener("abort", abort, { once: true });
    });
  }
  async command(command: number, params: number[] = []) {
    const reply = this.wait(
      (m) => m.name === "COMMAND_ACK" && m.values.command === command,
    );
    this.send("COMMAND_LONG", {
      command,
      confirmation: 0,
      ...Object.fromEntries(
        Array.from({ length: 7 }, (_, i) => [`param${i + 1}`, params[i] ?? 0]),
      ),
    });
    const result = Number((await reply).values.result);
    if (result !== 0)
      throw Error(
        `ArduPlane rejected command ${command} (result ${result}). ${this.lastStatus}`,
      );
  }
  async mode(name: string) {
    return this.setMode(name);
  }
  private async setMode(name: string) {
    const mode = MODES[name];
    if (mode === undefined) throw Error("Unknown mode");
    if (this.state.mode === mode) return;
    // Never carry a manual brake pull or climb command across a mode switch.
    this.channels[0] = this.channels[1] = 1500;
    this.sendRC();
    if (name === "AUTOLAND") {
      // AUTOLAND may reject entry before a takeoff direction is captured.
      // Request an ACK so that rejection reaches the UI immediately.
      await this.command(176, [1, mode]);
      if (this.state.mode !== mode)
        await this.wait(
          (m) => m.name === "HEARTBEAT" && m.values.custom_mode === mode,
        );
      return;
    }
    const landingStart = this.missionItems.findIndex((w) => w.command === 189);
    const confirmed = this.wait(
      (m) =>
        (m.name === "HEARTBEAT" && m.values.custom_mode === mode) ||
        // RTL can hand off to AUTO before the next heartbeat. Confirm the
        // native landing mission entry as well, rather than timing out.
        (name === "RTL" &&
          landingStart >= 0 &&
          ["HEARTBEAT", "MISSION_CURRENT"].includes(m.name) &&
          this.state.mode === MODES.AUTO &&
          this.state.mission >= landingStart + 1),
    );
    this.send("SET_MODE", { base_mode: 1, custom_mode: mode });
    await confirmed;
  }
  controls(roll: number, pitch: number, throttle: number) {
    this.applyControls(roll, pitch, throttle);
  }
  private applyControls(roll: number, pitch: number, throttle: number) {
    const channels = pilotChannels(
      this.state.mode,
      roll,
      pitch,
      throttle,
      this.physics.config.command[0],
    );
    this.channels.splice(0, 3, ...channels);
  }
  /** Observe native AUTO/AUTOLAND telemetry. This never generates pilot inputs,
   * switches modes, or uses physics truth to control the landing.
   */
  private updateLanding() {
    const item = landingTarget(this.state, this.missionItems, this.home);
    if (
      !item ||
      (!this.nativeLandingPhase && this.state.mode !== MODES.AUTOLAND)
    ) {
      this.state.landing = undefined;
      this.landingGroundSince = null;
      return;
    }
    if (this.state.simulationEnded) return;
    const terrainHeight = landingHeight(this.state, this.physics.time);
    const navigationFresh =
      this.physics.time - (this.state.navigationTimeS ?? -Infinity) < 0.3;
    const fieldAltitude =
      item.frame === 3 ? item.alt : item.alt - this.home.alt;
    // Presentation fallback only: the mission supplies the field elevation.
    const agl =
      terrainHeight ??
      (navigationFresh ? this.state.alt - fieldAltitude : null);
    const phase =
      this.nativeLandingPhase >= 3
        ? "flare"
        : this.nativeLandingPhase === 2
          ? "final"
          : "approach";
    const mode = this.state.mode === MODES.AUTOLAND ? "AUTOLAND" : "AUTO";
    this.state.landing = {
      phase,
      target: {
        north: (item.lat - this.home.lat) * 111319.49,
        east:
          (item.lon - this.home.lon) *
          111319.49 *
          Math.cos((this.home.lat * Math.PI) / 180),
      },
      agl,
      message:
        phase === "approach"
          ? `${mode} landing approach`
          : phase === "final"
            ? `${mode} power-off final`
            : `${mode} brake flare`,
    };
    // The rigid-canopy model does not represent collapse/rollout. End playback
    // after an estimator-confirmed stop; do not force disarm or alter AP outputs.
    if (
      phase === "flare" &&
      agl !== null &&
      agl < 0.6 &&
      Math.abs(this.state.climb) < 0.3 &&
      this.state.speed < 2
    ) {
      this.landingGroundSince ??= this.physics.time;
      if (this.physics.time - this.landingGroundSince > 0.25) {
        this.state.landing.phase = "landed";
        this.state.landing.message =
          "Landed · simulation paused · reset to fly again";
        this.state.simulationEnded = true;
        this.onTruth(this.physics.truth());
        this.onUpdate();
        setTimeout(() => this.dispose(), 0);
      }
    } else this.landingGroundSince = null;
  }
  async altitude(metres: number) {
    if (this.state.landing)
      throw Error("Select another flight mode before changing altitude");
    if (!Number.isFinite(metres) || metres < 5 || metres > 300)
      throw Error("Altitude must be 5–300 m above home.");
    if (
      ![MODES.FBWB, MODES.CRUISE, MODES.AUTO, MODES.LOITER].includes(
        this.state.mode,
      )
    )
      throw Error("Select FBWB, CRUISE, AUTO or LOITER to command altitude.");
    if (altitudeLever(this.state.mode)) {
      // AP locks current height on stick release. Let that transition finish
      // before installing a numeric target, or it can overwrite the command.
      this.channels[1] = 1500;
      this.sendRC();
      await delay(250, this.lifetime.signal);
    }
    const reply = this.wait(
      (m) => m.name === "COMMAND_ACK" && m.values.command === 186,
    );
    this.send("COMMAND_INT", {
      command: 186,
      frame: 3,
      param1: metres,
      param2: 3,
      current: 0,
      autocontinue: 0,
    });
    const result = Number((await reply).values.result);
    if (result !== 0)
      throw Error(`ArduPlane rejected altitude command (${result}).`);
    // Verify the controller demand, rather than treating an ACK as a climb.
    await this.wait(
      (m) =>
        m.name === "NAV_CONTROLLER_OUTPUT" &&
        Math.abs(this.state.alt + Number(m.values.alt_error) - metres) < 1,
      8000,
    );
  }
  private sendRC() {
    this.send(
      "RC_CHANNELS_OVERRIDE",
      Object.fromEntries(this.channels.map((v, i) => [`chan${i + 1}_raw`, v])),
    );
  }
  async controller(scheme: "baseline" | "lqi") {
    if (this.state.landing)
      throw Error("Select another flight mode before changing controller");
    // Existing ArduPlane auxiliary-function command: low/high for option 109.
    await this.command(218, [109, scheme === "lqi" ? 2 : 0]);
    this.channels[5] = scheme === "lqi" ? 2000 : 1000;
    this.sendRC();
    this.state.controllerSelected = scheme;
    this.onUpdate();
  }
  async launch(mode = "CRUISE") {
    await this.mode("MANUAL");
    await this.command(400, [1]);
    // Arming an airborne model moves home. Restore the ground reference before
    // initializing the height demand, matching the native research harness.
    const home = this.wait(
      (m) =>
        m.name === "HOME_POSITION" &&
        Math.abs(Number(m.values.altitude) / 1000 - this.home.alt) < 0.1,
    );
    await this.command(179, [
      0,
      0,
      0,
      0,
      this.home.lat,
      this.home.lon,
      this.home.alt,
    ]);
    await this.command(512, [242]);
    await home;
    // Keep direct neutral brakes while the operator handles the inverted payload.
    this.controls(0, 0, 0);
    this.sendRC();
    await delay(1000, this.lifetime.signal);
    this.physics.released = true;
    while (!this.physics.powerReady) await delay(50, this.lifetime.signal);
    // Actual ArduPlane takeoff detection, throttle and heading/climb control.
    // No RC throttle injection or FBWA climb to bypass AUTO suppression.
    await this.mode("TAKEOFF");
    this.onLog("TAKEOFF confirmed; ArduPlane controls the climb to 15 m.");
    const deadline = performance.now() + 45000;
    while (this.physics.launchPhase !== 2 || this.state.alt < 15) {
      if (performance.now() > deadline)
        throw Error(
          "Launch did not clear the ground. Reset and check wind conditions.",
        );
      await delay(100, this.lifetime.signal);
    }
    if (mode !== "TAKEOFF") await this.mode(mode);
    this.controls(
      0,
      0,
      altitudeLever(this.state.mode) ? 0.5 : this.physics.config.command[0],
    );
  }
  async upload(items: Waypoint[], progress: (n: number) => void) {
    if (this.busy) throw Error("A mission upload is already in progress.");
    this.busy = true;
    const home: Waypoint = {
      command: 16,
      frame: 0,
      params: [0, 0, 0, 0],
      ...this.home,
    };
    const all = [home, ...items];
    try {
      await new Promise<void>((resolve, reject) => {
        const requested = new Set<number>();
        let last = () =>
            this.send("MISSION_COUNT", { count: all.length, mission_type: 0 }),
          retries = 0;
        const cleanup = () => {
          clearInterval(timer);
          this.listeners.delete(listener);
          this.lifetime.signal.removeEventListener("abort", abort);
        };
        const abort = () => {
          cleanup();
          reject(this.lifetime.signal.reason);
        };
        const fail = (message: string) => {
          cleanup();
          reject(Error(message));
        };
        const listener = (m: Message) => {
          if (
            !["MISSION_ACK", "MISSION_REQUEST", "MISSION_REQUEST_INT"].includes(
              m.name,
            )
          )
            return;
          if (
            m.values.target_system !== 255 ||
            ![0, 190].includes(Number(m.values.target_component)) ||
            Number(m.values.mission_type ?? 0) !== 0
          )
            return;
          if (m.name === "MISSION_ACK") {
            if (Number(m.values.type) === 0 && requested.size !== all.length)
              return;
            cleanup();
            Number(m.values.type) === 0
              ? resolve()
              : reject(Error(`Mission rejected (result ${m.values.type}).`));
            return;
          }
          const seq = Number(m.values.seq),
            w = all[seq];
          if (!w) {
            fail("ArduPlane requested an invalid mission sequence.");
            return;
          }
          requested.add(seq);
          const integer = m.name === "MISSION_REQUEST_INT";
          retries = 0;
          last = () =>
            this.send(integer ? "MISSION_ITEM_INT" : "MISSION_ITEM", {
              seq,
              frame: integer
                ? w.frame === 3
                  ? 6
                  : w.frame === 0
                    ? 5
                    : w.frame
                : w.frame,
              command: w.command,
              current: 0,
              autocontinue: 1,
              param1: w.params[0],
              param2: w.params[1],
              param3: w.params[2],
              param4: w.params[3],
              x: integer ? Math.round(w.lat * 1e7) : w.lat,
              y: integer ? Math.round(w.lon * 1e7) : w.lon,
              z: w.alt,
              mission_type: 0,
            });
          last();
          progress(seq);
        };
        const timer = setInterval(() => {
          if (++retries > 5)
            fail("Mission upload timed out. The mission is not ready to fly.");
          else last();
        }, 1500);
        this.listeners.add(listener);
        this.lifetime.signal.addEventListener("abort", abort, { once: true });
        last();
      });
      this.missionItems = structuredClone(items);
    } finally {
      this.busy = false;
    }
  }
  async execute() {
    const confirmed = this.wait(
      (m) => m.name === "MISSION_CURRENT" && m.values.seq === 1,
    );
    this.send("MISSION_SET_CURRENT", { seq: 1 });
    await confirmed;
    await this.mode("AUTO");
  }
}

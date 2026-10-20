import { RuntimeAssets } from "../assets/runtime";
import { Atmosphere } from "./atmosphere";
import { equivalentAirspeed } from "./airspeed";
import { Lidar, type LidarConfig, type LidarReading } from "./lidar";
import type { Weather } from "@microagu/contracts/weather";
import { Quaternion, Vector3 } from "@microagu/contracts/math";
import type { Frame } from "@microagu/contracts/recordings";
import type { Kernel } from "../../../../generated/physics/sim_wasm";

export interface PhysicsConfig {
  schema: string;
  model: string;
  parameters: number[];
  parameter_keys: string[];
  mass: number;
  imu: number[];
  state: number[];
  rpm: number;
  command: number[];
  voltage: number;
  brake_rate_mm_s: number;
  brake_offsets_m: number[];
  /** Experimental sysid uncertainties; absent means production behavior. */
  brake_lag_s?: number;
  effective_pull_scale?: number;
  defaults: string;
  trim_residual: number;
  launch_settings?: number[];
  /** Opt-in generic downward lidar; rng_1 feeds the existing AP JSON backend. */
  lidar?: LidarConfig;
}
export class Physics {
  state: number[];
  rpm: number;
  time = 0;
  released = false;
  brakes: number[];
  command: number[];
  private laggedPull?: number[];
  private frame: number | null = null;
  private cached = "";
  private windNED = [0, 0, 0];
  private atmosphere = new Atmosphere();
  private lidar?: Lidar;
  lidarReading?: LidarReading;
  private surfaceEnv: Float64Array | null = null;
  launchPhase = 2;
  inflation = 1;
  private inverted = false;
  launchElapsed = 0;
  get powerReady() {
    return !this.inverted || this.launchElapsed >= 5.8;
  }
  get launchStage() {
    if (!this.inverted) return undefined;
    if (!this.released) return "Upright initialization";
    const t = this.launchElapsed;
    return t < 2
      ? "Invert payload"
      : t < 2.8
        ? "Inverted setup"
        : t < 3.8
          ? "Overhead swing"
          : t < 5.4
            ? "Two steps"
            : t < 5.8
              ? "Light toss"
              : "Free flight";
  }
  groundStart(inverted = false) {
    if (!this.config.launch_settings)
      throw Error("Launch configuration missing");
    this.surfaceEnv = new Float64Array([0, 0, 0]);
    const dz = this.state[2] + this.config.launch_settings[0];
    this.state[2] -= dz;
    this.state[15] -= dz;
    this.state.fill(0, 7, 13);
    this.state.fill(0, 20, 26);
    this.rpm = 0;
    this.launchPhase = 0;
    this.inflation = 0;
    this.inverted = inverted;
    this.launchElapsed = 0;
    if (inverted) {
      const pose = this.kernel.inverted_launch(
        new Float64Array(this.config.state),
        0,
        this.config.launch_settings[1],
      );
      this.state = Array.from(pose.subarray(0, 26));
    }
  }
  setWind(speed: number, fromDegrees: number, up: number, weather?: Weather) {
    this.atmosphere.configure(speed, fromDegrees, up, weather);
    this.windNED = this.atmosphere.sample(0, this.state[13], this.state[14]);
  }
  constructor(
    private kernel: Kernel,
    readonly config: PhysicsConfig,
    private readonly homeAltitudeM = 0,
    atmosphere?: Atmosphere,
  ) {
    if (config.lidar) this.lidar = new Lidar(config.lidar);
    if (atmosphere) this.atmosphere = atmosphere;
    if (
      config.schema !== "microagu.physics.v1" ||
      config.state.length !== 26 ||
      config.parameters.length !== 70
    )
      throw Error("Unsupported high-DOF model configuration.");
    if (
      !Number.isFinite(config.brake_lag_s ?? 0) ||
      (config.brake_lag_s ?? 0) < 0 ||
      (config.brake_lag_s ?? 0) > 1 ||
      !Number.isFinite(config.effective_pull_scale ?? 1) ||
      (config.effective_pull_scale ?? 1) <= 0 ||
      (config.effective_pull_scale ?? 1) > 2
    )
      throw Error("Invalid actuator uncertainty configuration");
    this.state = config.state.slice();
    this.rpm = config.rpm;
    this.command = config.command.slice();
    this.brakes = config.command.slice(1);
  }
  static async load(base: string, config?: PhysicsConfig, homeAltitudeM = 0) {
    const assets = base.startsWith("file:")
      ? null
      : await RuntimeAssets.load(base);
    const url = assets?.url("sim_wasm.js") ?? new URL("sim_wasm.js", base).href;
    if (assets) await assets.bytes("sim_wasm.js");
    const [bindings, data] = await Promise.all([
      import(/* @vite-ignore */ url) as Promise<
        typeof import("../../../../generated/physics/sim_wasm")
      >,
      config ??
        (assets
          ? assets
              .bytes("highdof.json")
              .then(
                (bytes) =>
                  JSON.parse(new TextDecoder().decode(bytes)) as PhysicsConfig,
              )
          : fetch(new URL("highdof.json", base)).then((r) => {
              if (!r.ok) throw Error("Could not load high-DOF configuration");
              return r.json() as Promise<PhysicsConfig>;
            })),
    ]);
    const wasmUrl = new URL("sim_wasm_bg.wasm", base);
    if (url.startsWith("file:")) {
      const nodeFs = "node:fs/promises";
      const fs = await import(/* @vite-ignore */ nodeFs);
      await bindings.default({ module_or_path: await fs.readFile(wasmUrl) });
    } else {
      await bindings.default({
        module_or_path: assets
          ? await assets.bytes("sim_wasm_bg.wasm")
          : wasmUrl,
      });
    }
    return new Physics(
      new bindings.Kernel(new Float64Array(data.parameters), data.mass),
      data,
      homeAltitudeM,
      new Atmosphere((seed) => new bindings.Dryden(seed)),
    );
  }
  dispose() {
    this.atmosphere.dispose();
    this.kernel.free();
  }
  private observation() {
    const q = new Quaternion().fromArray(this.state, 3).normalize(),
      inverse = q.clone().invert();
    const omega = new Vector3().fromArray(this.state, 10),
      arm = new Vector3().fromArray(this.config.imu).applyQuaternion(q);
    return {
      q,
      inverse,
      position: new Vector3().fromArray(this.state, 0).add(arm),
      velocity: new Vector3()
        .fromArray(this.state, 7)
        .add(omega.clone().cross(arm)),
      gyro: omega.applyQuaternion(inverse),
    };
  }
  respond(packet: Uint8Array) {
    if (![40, 72].includes(packet.length))
      throw Error("Invalid SITL servo packet length");
    const view = new DataView(
        packet.buffer,
        packet.byteOffset,
        packet.byteLength,
      ),
      magic = view.getUint16(0, true),
      rate = view.getUint16(2, true),
      frame = view.getUint32(4, true);
    if (
      magic !== (packet.length === 40 ? 18458 : 29569) ||
      rate < 1 ||
      rate > 2000
    )
      throw Error("Invalid SITL servo packet");
    if (frame === this.frame) return this.cached;
    if (this.frame !== null && frame !== (this.frame + 1) >>> 0)
      throw Error("Physics frame discontinuity; reset the simulator.");
    const dt = 1 / rate,
      before = this.observation();
    // One wind sample drives forces, airspeed, JSON wind and render truth alike.
    this.windNED = this.atmosphere.sample(
      dt,
      this.state[13],
      this.state[14],
      this.state.slice(7, 10),
    );
    if (this.released) {
      const clamp = (x: number, lo: number, hi: number) =>
        Math.max(lo, Math.min(hi, x));
      const pwm = (channel: number) => view.getUint16(8 + channel * 2, true),
        mmPerUS = ((Math.PI * 55) / 360) * 0.1;
      this.command = [
        (clamp(pwm(0), 1000, 2000) - 1000) / 1000,
        (clamp(pwm(2), 800, 2100) - 800) * mmPerUS,
        (2400 - clamp(pwm(3), 900, 2400)) * mmPerUS,
      ];
      if (this.inverted) {
        this.launchElapsed += dt;
        if (!this.powerReady) this.command[0] = 0;
      }
      const average = this.brakes.map((start, i) => {
        const end =
          start +
          clamp(
            this.command[i + 1] - start,
            -this.config.brake_rate_mm_s * dt,
            this.config.brake_rate_mm_s * dt,
          );
        const travel = Math.abs(end - start) / this.config.brake_rate_mm_s;
        this.brakes[i] = end;
        return (0.5 * (start + end) * travel) / dt + end * (1 - travel / dt);
      });
      const input = [
        this.command[0],
        ...average.map((v, i) => {
          const pull =
            (v / 1000 - this.config.brake_offsets_m[i]) *
            (this.config.effective_pull_scale ?? 1);
          return this.config.parameters[60] > 0 ? pull : Math.max(0, pull);
        }),
      ];
      const lag = this.config.brake_lag_s ?? 0;
      if (lag > 0) {
        const decay = Math.exp(-dt / lag);
        this.laggedPull ??= input.slice(1);
        this.laggedPull = this.laggedPull.map(
          (v, i) => decay * v + (1 - decay) * input[i + 1],
        );
        input.splice(1, 2, ...this.laggedPull);
      }
      const y = new Float64Array(this.state),
        wind = new Float64Array(this.windNED);
      if (this.inverted && this.launchElapsed <= 5.8 + 1e-8) {
        const result = this.kernel.inverted_launch(
          new Float64Array(this.config.state),
          Math.min(5.8, this.launchElapsed),
          this.config.launch_settings![1],
        );
        this.state = Array.from(result.subarray(0, 26));
        this.rpm = 0;
        this.inflation = result[26];
        this.launchPhase = 1;
        this.surfaceEnv = new Float64Array([2, 0, 1]);
      } else if (this.surfaceEnv) {
        if (this.surfaceEnv[0] === 0) this.surfaceEnv[0] = 1;
        const result = this.kernel.surface_step(
          y,
          new Float64Array(input),
          wind,
          this.config.voltage,
          this.rpm,
          dt,
          this.surfaceEnv,
          new Float64Array(this.config.launch_settings!),
        );
        this.state = Array.from(result.subarray(0, 26));
        this.rpm = result[26];
        this.surfaceEnv = result.slice(27);
        this.launchPhase = this.surfaceEnv[0];
        this.inflation = this.surfaceEnv[2];
      } else {
        const result = this.kernel.replay(
          y,
          new Float64Array([...input, ...input]),
          wind,
          this.config.voltage,
          this.rpm,
          dt,
          Math.max(1, Math.ceil(dt / 0.00125)),
          true,
        );
        this.state = Array.from(result.subarray(27, 53));
        this.rpm = result[53];
      }
      if (!this.state.every(Number.isFinite) || !Number.isFinite(this.rpm))
        throw Error("Non-finite physics state; reset the simulator.");
      if (!this.surfaceEnv && this.state[2] >= 0)
        throw Error(
          "Ground reached. This airborne research model has no contact dynamics; reset for another flight.",
        );
    } else {
      for (let i = 0; i < 3; i++) {
        const delta = this.state[7 + i] * dt;
        this.state[i] += delta;
        this.state[13 + i] += delta;
      }
    }
    const obs = this.observation(),
      accel = obs.velocity
        .clone()
        .sub(before.velocity)
        .divideScalar(dt)
        .sub(new Vector3(0, 0, 9.80665))
        .applyQuaternion(obs.inverse);
    this.time += dt;
    this.lidarReading = this.lidar?.sample(this.time, this.state);
    const message = {
      timestamp: this.time,
      battery: { voltage: this.config.voltage },
      imu: { gyro: obs.gyro.toArray(), accel_body: accel.toArray() },
      position: obs.position.toArray(),
      quaternion: [this.state[6], ...this.state.slice(3, 6)],
      velocity: obs.velocity.toArray(),
      // JSON airspeed goes directly to SITL's EAS pressure-sensor input.
      airspeed: equivalentAirspeed(
        Math.hypot(
          ...this.state.slice(7, 10).map((v, i) => v - this.windNED[i]),
        ),
        this.homeAltitudeM - obs.position.z,
      ),
      velocity_wind: this.windNED.slice(),
      // JSON cannot represent NaN. Invalid/no-return is explicitly above the
      // configured maximum, so AP rejects it instead of retaining old terrain.
      ...(this.lidarReading ? { rng_1: this.lidarReading.range_m } : {}),
    };
    this.frame = frame;
    this.cached = "\n" + JSON.stringify(message) + "\n";
    return this.cached;
  }
  truth(): Frame {
    const payload = new Quaternion().fromArray(this.state, 3).normalize(),
      canopy = new Quaternion().fromArray(this.state, 16).normalize();
    const relative = payload.clone().invert().multiply(canopy);
    if (relative.w < 0) {
      relative.x *= -1;
      relative.y *= -1;
      relative.z *= -1;
      relative.w *= -1;
    }
    const sin = Math.hypot(relative.x, relative.y, relative.z),
      scale = sin < 1e-12 ? 2 : (2 * Math.atan2(sin, relative.w)) / sin;
    const rate = new Vector3()
      .fromArray(this.state, 23)
      .sub(new Vector3().fromArray(this.state, 10))
      .applyQuaternion(payload.invert());
    return {
      schema: "microagu.state.v1",
      time_s: this.time,
      state: this.state.slice(),
      rpm: this.rpm,
      voltage_v: this.config.voltage,
      brake_mm: this.brakes.slice(),
      throttle: this.command[0],
      released: this.released,
      launch_phase: this.launchPhase,
      launch_stage: this.launchStage,
      inflation: this.inflation,
      wind_ned_m_s: this.windNED.slice(),
      joint_rotvec_rad: [
        relative.x * scale,
        relative.y * scale,
        relative.z * scale,
      ],
      joint_rate_rad_s: rate.toArray(),
    };
  }
}

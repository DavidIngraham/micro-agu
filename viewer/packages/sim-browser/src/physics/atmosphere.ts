import { calmWeather, type Weather } from "@microagu/contracts/weather";
import type { Dryden } from "../../../../generated/physics/sim_wasm";

/** Synthetic training weather, independent of the aircraft's fitted coefficients. */
export class Atmosphere {
  private mean = [0, 0, 0];
  private weather: Weather = { ...calmWeather };
  private gust = [0, 0, 0];
  private seed = 0x4d414755;
  private dryden?: Dryden;
  private appliedMean = [0, 0, 0];
  private strength = 0;
  constructor(private readonly makeDryden?: (seed: number) => Dryden) {}
  dispose() {
    this.dryden?.free();
  }

  configure(speed: number, from: number, up: number, weather = calmWeather) {
    if (
      weather.seed !== undefined &&
      (!Number.isInteger(weather.seed) ||
        weather.seed < 1 ||
        weather.seed > 0xffffffff)
    )
      throw Error("Weather seed must be a nonzero uint32.");
    const ranges = [
      [speed, 0, 10],
      [from, 0, 360],
      [up, -3, 3],
      [weather.turbulence, 0, 2],
      [weather.thermalStrength, 0, 5],
      [weather.thermalRadius, 5, 200],
      [weather.thermalNorth, -1000, 1000],
      [weather.thermalEast, -1000, 1000],
      [weather.gustLength ?? 20, 1, 200],
      [weather.verticalRatio ?? 0.5, 0, 2],
    ];
    if (
      ranges.some(([v, min, max]) => !Number.isFinite(v) || v < min || v > max)
    )
      throw Error("Weather settings are outside their supported ranges.");
    if (
      weather.turbulenceModel !== undefined &&
      !["legacy", "dryden"].includes(weather.turbulenceModel)
    )
      throw Error("Unknown turbulence model");
    if (weather.turbulenceModel === "dryden" && !this.makeDryden)
      throw Error("Dryden requires the Rust atmosphere binding");
    const angle = (from * Math.PI) / 180;
    this.mean = [-speed * Math.cos(angle), -speed * Math.sin(angle), -up];
    this.weather = { ...weather };
    if (
      weather.turbulenceModel === "dryden" &&
      (!this.dryden || weather.seed !== undefined)
    ) {
      this.dryden?.free();
      this.dryden = this.makeDryden!(weather.seed ?? this.seed);
    }
    if (weather.seed !== undefined) {
      this.seed = weather.seed;
      this.gust.fill(0);
    }
    if (weather.turbulence === 0) this.gust.fill(0);
  }

  private uniform() {
    let x = this.seed;
    x ^= x << 13;
    x ^= x >>> 17;
    x ^= x << 5;
    this.seed = x >>> 0;
    return (this.seed + 0.5) / 4294967296;
  }

  sample(
    dt: number,
    north: number,
    east: number,
    velocity = [0, 0, 0],
  ): number[] {
    const w = this.weather;
    if (w.turbulenceModel === "dryden") {
      // Smooth operator changes; do not reset the random sequence on slider edits.
      const alpha = -Math.expm1(-dt / 2);
      this.appliedMean = this.appliedMean.map(
        (v, i) => v + alpha * (this.mean[i] - v),
      );
      this.strength += alpha * (w.turbulence - this.strength);
      const speed = Math.hypot(
        ...velocity.map((v, i) => v - this.appliedMean[i]),
      );
      const length = w.gustLength ?? 20;
      const gust = this.dryden!.step(
        dt,
        speed,
        new Float64Array([length, length, length]),
        new Float64Array([
          this.strength,
          this.strength,
          this.strength * (w.verticalRatio ?? 0.5),
        ]),
      );
      const heading = Math.atan2(this.appliedMean[1], this.appliedMean[0]);
      const c = Math.cos(heading),
        s = Math.sin(heading);
      const lift =
        w.thermalStrength *
        Math.exp(
          -((north - w.thermalNorth) ** 2 + (east - w.thermalEast) ** 2) /
            w.thermalRadius ** 2,
        );
      return [
        this.appliedMean[0] + c * gust[0] - s * gust[1],
        this.appliedMean[1] + s * gust[0] + c * gust[1],
        this.appliedMean[2] + gust[2] - lift,
      ];
    }
    // Exact Ornstein–Uhlenbeck transition, two-second correlation time.
    const decay = Math.exp(-dt / 2);
    if (w.turbulence > 0 && dt > 0) {
      for (let axis = 0; axis < 3; axis++) {
        const normal =
          Math.sqrt(-2 * Math.log(this.uniform())) *
          Math.cos(2 * Math.PI * this.uniform());
        this.gust[axis] =
          decay * this.gust[axis] + Math.sqrt(1 - decay * decay) * normal;
      }
    }
    const radius2 =
      ((north - w.thermalNorth) ** 2 + (east - w.thermalEast) ** 2) /
      w.thermalRadius ** 2;
    const lift = w.thermalStrength * Math.exp(-radius2);
    return this.mean.map(
      (v, axis) =>
        v +
        w.turbulence * this.gust[axis] * (axis === 2 ? 0.5 : 1) -
        (axis === 2 ? lift : 0),
    );
  }
}

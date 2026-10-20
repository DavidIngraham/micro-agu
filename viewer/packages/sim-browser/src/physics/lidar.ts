import { Quaternion, Vector3 } from "@microagu/contracts/math";

/** Generic downward single-beam lidar, over the simulator's flat ground plane.
 * Slant range in metres. Not a characterization of a specific device.
 */
export interface LidarConfig {
  min_m: number;
  max_m: number;
  rate_hz: number;
  latency_s: number;
  noise_std_m: number;
  bias_m: number;
  dropout_probability: number;
  max_tilt_deg: number;
  seed: number;
  offset_body_m?: [number, number, number];
}
export const defaultLidar: LidarConfig = {
  min_m: 0.1, max_m: 40, rate_hz: 20, latency_s: 0.05,
  noise_std_m: 0.02, bias_m: 0, dropout_probability: 0.01,
  max_tilt_deg: 60, seed: 73,
};
export interface LidarReading {
  captured_s: number;
  delivered_s: number;
  range_m: number;
  valid: boolean;
}
export class Lidar {
  private next = 0;
  private randomState: number;
  private pending: LidarReading[] = [];
  private reading: LidarReading;
  constructor(readonly config: LidarConfig) {
    const c = config;
    if (!Object.values(c).filter(x => typeof x === "number").every(Number.isFinite) ||
        c.min_m < 0 || c.max_m <= c.min_m || c.max_m > 650 ||
        c.rate_hz <= 0 || c.rate_hz > 200 || c.latency_s < 0 || c.latency_s > 2 ||
        c.noise_std_m < 0 || c.dropout_probability < 0 || c.dropout_probability > 1 ||
        c.max_tilt_deg <= 0 || c.max_tilt_deg >= 90 ||
        (c.offset_body_m && (c.offset_body_m.length !== 3 || !c.offset_body_m.every(Number.isFinite))))
      throw Error("Invalid lidar configuration");
    this.randomState = c.seed >>> 0;
    this.reading = { captured_s: -1, delivered_s: 0, range_m: c.max_m + 1, valid: false };
  }
  private random() {
    this.randomState = (1664525 * this.randomState + 1013904223) >>> 0;
    return (this.randomState + 0.5) / 4294967296;
  }
  sample(time: number, state: readonly number[]): LidarReading {
    if (time + 1e-9 >= this.next) {
      const q = new Quaternion().fromArray(state as number[], 3).normalize();
      const down = new Vector3(0, 0, 1).applyQuaternion(q);
      const origin = new Vector3().fromArray(state as number[], 0).add(
        new Vector3(...(this.config.offset_body_m ?? [0, 0, 0])).applyQuaternion(q));
      const c = this.config;
      const geometric = -origin.z / down.z;
      const noise = Math.sqrt(-2 * Math.log(this.random())) * Math.cos(2 * Math.PI * this.random()) * c.noise_std_m;
      const dropped = this.random() < c.dropout_probability;
      const distance = geometric + c.bias_m + noise;
      const valid = !dropped && down.z >= Math.cos(c.max_tilt_deg * Math.PI / 180) &&
        origin.z < 0 && geometric >= c.min_m && geometric <= c.max_m &&
        distance >= c.min_m && distance <= c.max_m;
      this.pending.push({ captured_s: time, delivered_s: time + c.latency_s,
        range_m: valid ? Math.round(distance * 100) / 100 : c.max_m + 1, valid });
      this.next = time + 1 / c.rate_hz;
    }
    while (this.pending.length && this.pending[0].delivered_s <= time + 1e-9)
      this.reading = this.pending.shift()!;
    return { ...this.reading };
  }
}

/** Experimental outer guidance and flare controller. No simulator dependency.
 * Navigation/attitude/sink are EKF telemetry; AGL is an explicit sensor input.
 */
export const clamp = (x: number, a: number, b: number) =>
  Math.max(a, Math.min(b, x));
export const wrap = (x: number) => Math.atan2(Math.sin(x), Math.cos(x));
export interface Measurement {
  north: number;
  east: number;
  agl: number;
  speed: number;
  sink: number;
  yaw: number;
  pitch: number;
  course?: number;
  pitchRate?: number;
}
export interface Target {
  north: number;
  east: number;
  heading: number;
  speed: number;
}
export interface Options {
  feedback: boolean;
  flareHeight: number;
  commonMm: number;
}
export class LandingGuidance {
  private lastPitch: number | null = null;
  private pitchRate = 0;
  private flareStart: number | null = null;
  constructor(
    readonly target: Target,
    readonly options: Options,
  ) {}
  update(m: Measurement, dt: number, manual: boolean, time: number) {
    const dn = Math.cos(this.target.heading),
      de = Math.sin(this.target.heading);
    const n = this.target.north - m.north,
      e = this.target.east - m.east;
    const remaining = n * dn + e * de,
      cross = -n * de + e * dn;
    const desiredHeading =
      this.target.heading + clamp(Math.atan2(cross, 25), -0.4, 0.4);
    const yawError = wrap(desiredHeading - (m.course ?? m.yaw));
    if (this.lastPitch !== null) {
      const q = wrap(m.pitch - this.lastPitch) / dt;
      this.pitchRate += ((q - this.pitchRate) * dt) / (0.15 + dt);
    }
    this.lastPitch = m.pitch;
    const terminalDistance =
      (3 * this.target.speed) / 1.38 + 0.3 * this.target.speed;
    const pathHeight =
      3 + ((remaining - terminalDistance) * 0.8) / this.target.speed;
    const sinkRequest = clamp(
      (0.8 * m.speed) / this.target.speed + 0.2 * (m.agl - pathHeight),
      0,
      0.98,
    );
    if (!manual)
      return {
        roll: clamp((yawError * 1.2) / ((35 * Math.PI) / 180), -0.7, 0.7),
        pitch: 0,
        throttle: 0.5 - sinkRequest / 4,
        remaining,
        cross,
        pathHeight,
        common: 0,
        sinkRequest,
        flare: false,
      };
    if (m.agl <= this.options.flareHeight && this.flareStart === null)
      this.flareStart = time;
    const flare = this.flareStart !== null;
    // Feedforward is a rate-limited servo target, not an instantaneous impulse.
    let common = flare ? this.options.commonMm : 0;
    if (flare && this.options.feedback) {
      const age = time - this.flareStart!;
      const sinkTarget = clamp(1.38 - 1.0 * age, 0.65, 1.38);
      // Bounded corrections; no integrator to wind up near contact.
      // Let the intended pitch-up transient develop. Rate damping is a guard
      // beyond 20 degrees, not opposition to the whole flare maneuver.
      const damping =
        m.pitch > 0.35 ? 8 * Math.max(0, m.pitchRate ?? this.pitchRate) : 0;
      common += clamp(18 * (m.sink - sinkTarget) - damping, -10, 10);
    }
    common = clamp(common, 0, 58);
    const diff = clamp(yawError * 20, -8, 8);
    const left = clamp(common + Math.max(-diff, 0), 0, 62.3955);
    const right = clamp(common + Math.max(diff, 0), 0, 71.9948);
    // Invert the AP common-floor/differential mixer, accounting for unequal
    // servo travel. RC deadzones are explicitly zero in the research config.
    const l = left / 62.3955,
      r = right / 71.9948;
    return {
      roll: r - l,
      pitch: Math.min(l, r),
      throttle: 0,
      remaining,
      cross,
      pathHeight,
      common,
      sinkRequest,
      flare,
    };
  }
}

export const MODES: Record<string, number> = {
  MANUAL: 0,
  FBWA: 5,
  FBWB: 6,
  CRUISE: 7,
  AUTO: 10,
  RTL: 11,
  LOITER: 12,
  TAKEOFF: 13,
};
export interface Telemetry {
  /** The completed landing session is paused; reset creates a fresh session. */
  simulationEnded?: boolean;
  /** EKF terrain-relative current output; negative firmware sentinel is invalid. */
  hagl?: {
    metres: number;
    valid: boolean;
    timeBootMs: number;
    receivedTimeS: number;
  };
  course?: number;
  pitchRate?: number;
  navigationTimeS?: number;
  landing?: {
    phase: "approach" | "final" | "flare" | "touchdown" | "landed" | "aborted";
    target: { north: number; east: number };
    agl: number | null;
    message: string;
  };
  /** Downward rangefinder telemetry; slant range, not tilt-corrected AGL.
   * Consumers must also reject stale receivedTimeS values.
   */
  rangefinder?: {
    distanceM: number;
    minM: number;
    maxM: number;
    valid: boolean;
    receivedTimeS: number;
    timeBootMs: number;
  };
  controllerSelected: "baseline" | "lqi";
  controllerActive: boolean;
  mode: number;
  armed: boolean;
  lat: number;
  lon: number;
  alt: number;
  targetAlt: number | null;
  targetRoll: number | null;
  crossTrack: number | null;
  speed: number;
  climb: number;
  roll: number;
  pitch: number;
  yaw: number;
  servos: number[];
  mission: number;
  lastHeartbeat: number;
}

export function initialTelemetry(home: {
  lat: number;
  lon: number;
}): Telemetry {
  return {
    controllerSelected: "baseline",
    controllerActive: false,
    mode: -1,
    armed: false,
    lat: home.lat,
    lon: home.lon,
    alt: 0,
    targetAlt: null,
    targetRoll: null,
    crossTrack: null,
    speed: 0,
    climb: 0,
    roll: 0,
    pitch: 0,
    yaw: 0,
    servos: [],
    mission: 0,
    lastHeartbeat: 0,
  };
}

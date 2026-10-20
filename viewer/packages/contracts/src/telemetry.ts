export const MODES: Record<string, number> = {
  MANUAL: 0,
  FBWA: 5,
  FBWB: 6,
  CRUISE: 7,
  AUTO: 10,
  LOITER: 12,
  TAKEOFF: 13,
};
export interface Telemetry {
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

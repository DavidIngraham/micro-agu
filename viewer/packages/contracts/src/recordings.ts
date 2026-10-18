export interface Frame {
  schema: "microagu.state.v1";
  time_s: number;
  state: number[];
  rpm: number | null;
  voltage_v: number;
  brake_mm: number[] | null;
  throttle: number | null;
  joint_rotvec_rad: number[];
  joint_rate_rad_s: number[];
  released: boolean;
  // Optional native-log annotations added to a replay, not inferred from motion.
  mode?: "MANUAL" | "TAKEOFF" | "AUTO";
  mission_item?: number;
  wind_ned_m_s?: number[];
  turbulence?: string;
  launch_phase?: number;
  launch_stage?: string;
  inflation?: number;
}
const vector = (x: unknown, n: number): x is number[] =>
  Array.isArray(x) && x.length === n && x.every(Number.isFinite);
export function validate(x: unknown): Frame {
  const f = x as Frame;
  if (
    !f ||
    f.schema !== "microagu.state.v1" ||
    !Number.isFinite(f.time_s) ||
    !vector(f.state, 26) ||
    !(f.brake_mm === null || vector(f.brake_mm, 2)) ||
    !vector(f.joint_rotvec_rad, 3) ||
    !vector(f.joint_rate_rad_s, 3) ||
    !Number.isFinite(f.voltage_v) ||
    ![f.rpm, f.throttle].every((v) => v === null || Number.isFinite(v)) ||
    typeof f.released !== "boolean" ||
    (f.launch_phase !== undefined && ![0, 1, 2, 3].includes(f.launch_phase)) ||
    (f.inflation !== undefined &&
      (!Number.isFinite(f.inflation) || f.inflation < 0 || f.inflation > 1)) ||
    (f.wind_ned_m_s !== undefined && !vector(f.wind_ned_m_s, 3)) ||
    (f.turbulence !== undefined && typeof f.turbulence !== "string") ||
    (f.mode !== undefined && !["MANUAL", "TAKEOFF", "AUTO"].includes(f.mode)) ||
    (f.mission_item !== undefined &&
      (!Number.isInteger(f.mission_item) || f.mission_item < 0))
  )
    throw Error("Unsupported or incomplete microagu.state.v1 frame");
  for (const k of [3, 16])
    if (Math.abs(Math.hypot(...f.state.slice(k, k + 4)) - 1) > 0.01)
      throw Error("Body quaternion is not normalized");
  return f;
}
export function parseRecording(text: string): Frame[] {
  const frames = (
    text.trim().startsWith("[")
      ? JSON.parse(text)
      : text
          .trim()
          .split(/\r?\n/)
          .filter(Boolean)
          .map((x) => JSON.parse(x))
  ).map(validate);
  if (!frames.length) throw Error("Recording is empty");
  for (let i = 1; i < frames.length; i++)
    if (frames[i].time_s <= frames[i - 1].time_s)
      throw Error(
        "Time must increase. Split restarted runs into separate recordings.",
      );
  return frames;
}

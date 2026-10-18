import type { Weather } from "@microagu/contracts/weather";

/** Centre of the northbound 200–400 m leg of the example AUTO circuit. */
export const defaultWeather: Weather = {
  turbulenceModel: "dryden",
  gustLength: 20,
  verticalRatio: 0.5,
  turbulence: 0.2,
  thermalStrength: 1.5,
  thermalRadius: 60,
  thermalNorth: 300,
  thermalEast: 0,
};

export function windTruthLabel(wind?: number[]): string {
  if (!wind || wind.length !== 3 || !wind.every(Number.isFinite))
    return "WIND TRUTH · —";
  const [north, east, down] = wind;
  const speed = Math.hypot(north, east);
  const from =
    (Math.round((Math.atan2(-east, -north) * 180) / Math.PI) + 360) % 360;
  const direction = speed < 0.05 ? "" : ` from ${from}°`;
  const vertical =
    Math.abs(down) < 0.05
      ? "↕ 0.0"
      : `${down < 0 ? "↑" : "↓"} ${Math.abs(down).toFixed(1)}`;
  return `WIND TRUTH · ${speed.toFixed(1)} m/s${direction} · ${vertical} m/s`;
}

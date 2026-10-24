import type { Home, Waypoint } from "@microagu/contracts/mission";
import { MODES, type Telemetry } from "@microagu/contracts/telemetry";

/** Landing destination for presentation only. Native firmware plans the path. */
export function landingTarget(
  state: Pick<Telemetry, "mode" | "mission">,
  items: Waypoint[],
  home: Home,
): Waypoint | undefined {
  if (state.mode === MODES.AUTOLAND)
    return {
      command: 21,
      frame: 3,
      lat: home.lat,
      lon: home.lon,
      alt: 0,
      params: [0, 0, 0, 0],
    };
  const item = items[state.mission - 1];
  return state.mode === MODES.AUTO && item?.command === 21 ? item : undefined;
}

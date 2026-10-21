import type { Waypoint } from "@microagu/contracts/mission";
export const HOME = { lat: 45.997, lon: -121.536, alt: 600 };
export function exampleMission(): Waypoint[] {
  const circuit: Waypoint[] = [
    [200, 0],
    [400, 0],
    [400, 200],
    [0, 200],
    [0, 0],
  ].map(([n, e]) => ({
    command: 16,
    frame: 3,
    params: [0, 22, 0, 0],
    lat: HOME.lat + n / 111319.49,
    lon: HOME.lon + e / (111319.49 * Math.cos((HOME.lat * Math.PI) / 180)),
    alt: 60,
  }));
  return [
    ...circuit,
    // Uploaded sequence zero is home; repeat from the first flight waypoint.
    { command: 177, frame: 2, params: [1, -1, 0, 0], lat: 0, lon: 0, alt: 0 },
    // RTL_AUTOLAND enters here, bypassing the infinite circuit.
    { command: 189, frame: 2, params: [0, 0, 0, 0], lat: 0, lon: 0, alt: 0 },
    ...landingMission(),
  ];
}

/** Clockwise capture/descending orbit, tangent northbound final, NAV_LAND home.
 * West edge of the 40 m circle is on east=0. The final approach fixes lie
 * north of that tangent point; param4=1 tells AP to track from its actual exit.
 */
export function landingMission(): Waypoint[] {
  const point = (
    north: number,
    east: number,
    alt: number,
    command: number,
    params: number[],
  ): Waypoint => ({
    command,
    frame: 3,
    params,
    lat: HOME.lat + north / 111319.49,
    lon: HOME.lon + east / (111319.49 * Math.cos((HOME.lat * Math.PI) / 180)),
    alt,
  });
  return [
    // Capture before descending so inbound travel cannot consume the spiral.
    point(-280, 40, 60, 18, [1, 0, 40, 0]),
    // LOITER_TO_ALT uses param2 for signed radius, unlike LOITER_TURNS.
    point(-280, 40, 30, 31, [1, 40, 0, 1]),
    point(-200, 0, 30, 16, [0, 10, 0, 0]),
    point(0, 0, 0, 21, [0, 0, 0, 0]),
  ];
}

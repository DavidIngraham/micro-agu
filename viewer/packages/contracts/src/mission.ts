export interface Waypoint {
  command: number;
  frame: number;
  params: number[];
  lat: number;
  lon: number;
  alt: number;
}
export interface Home {
  lat: number;
  lon: number;
  alt: number;
}
const supported = new Set([16, 17, 18, 19, 20, 21, 22, 31]);
export function validateMission(items: Waypoint[]) {
  if (!items.length || items.length > 500)
    throw Error("Mission must contain 1–500 items.");
  for (const [i, w] of items.entries()) {
    if (!supported.has(w.command))
      throw Error(
        `Item ${i + 1}: unsupported command ${w.command}. Use simple waypoints, takeoff, land, loiter or RTL.`,
      );
    if (![0, 3].includes(w.frame))
      throw Error(
        `Item ${i + 1}: only absolute or relative-to-home altitude is supported.`,
      );
    if (
      w.params.length !== 4 ||
      ![...w.params, w.lat, w.lon, w.alt].every(Number.isFinite) ||
      Math.abs(w.lat) > 90 ||
      Math.abs(w.lon) > 180
    )
      throw Error(`Item ${i + 1}: invalid coordinates or parameters.`);
    if (w.command !== 20 && (w.lat === 0 || w.lon === 0))
      throw Error(`Item ${i + 1}: supply explicit coordinates.`);
  }
  return items;
}
export function parseMission(text: string): Waypoint[] {
  if (text.trimStart().startsWith("QGC WPL 110")) {
    const rows = text
      .trim()
      .split(/\r?\n/)
      .slice(1)
      .filter((x) => x.trim())
      .map((x) => x.trim().split(/\s+/).map(Number));
    if (
      rows.some(
        (r, i) =>
          r.length !== 12 || r[0] !== i || r.some((x) => !Number.isFinite(x)),
      )
    )
      throw Error("Invalid QGC WPL 110 rows or sequence.");
    // WPL seq 0 is the planned home, not a navigation command.
    return validateMission(
      rows.slice(1).map((r) => ({
        frame: r[2],
        command: r[3],
        params: r.slice(4, 8),
        lat: r[8],
        lon: r[9],
        alt: r[10],
      })),
    );
  }
  const data = JSON.parse(text);
  if (data.fileType !== "Plan" || !Array.isArray(data.mission?.items))
    throw Error(
      "Choose a QGroundControl .plan or QGC WPL 110 .waypoints file.",
    );
  return validateMission(
    data.mission.items.map((item: any) => {
      if (
        item.type !== "SimpleItem" ||
        !Array.isArray(item.params) ||
        item.params.length !== 7
      )
        throw Error(
          "Only QGC SimpleItem missions are supported; expand surveys to waypoints first.",
        );
      const p = item.params.map((x: any, i: number) =>
        x === null && i === 3 ? 0 : x,
      );
      if (p.some((x: any) => typeof x !== "number"))
        throw Error("Mission parameters must be numbers.");
      return {
        command: item.command,
        frame: item.frame,
        params: p.slice(0, 4),
        lat: p[4],
        lon: p[5],
        alt: p[6],
      };
    }),
  );
}
export function offset(lat: number, lon: number, home: Home) {
  return {
    north: (lat - home.lat) * 111319.49,
    east: (lon - home.lon) * 111319.49 * Math.cos((home.lat * Math.PI) / 180),
  };
}

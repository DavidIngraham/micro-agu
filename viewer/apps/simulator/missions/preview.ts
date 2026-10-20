import { offset, type Home, type Waypoint } from "@microagu/contracts/mission";

/** Planned geometry only: AP chooses the actual capture and exit points. */
export function missionPreview(items: Waypoint[], home: Home) {
  const path: { north: number; east: number; alt: number }[] = [];
  items.forEach((item, i) => {
    if (item.command === 20) return;
    const centre = offset(item.lat, item.lon, home);
    const alt = item.frame === 3 ? item.alt : item.alt - home.alt;
    const signedRadius = [17, 18, 19, 31].includes(item.command)
      ? item.params[item.command === 31 ? 1 : 2]
      : 0;
    const radius = Math.abs(signedRadius);
    if (!radius) {
      path.push({ ...centre, alt });
      return;
    }
    const direction = Math.sign(signedRadius);
    const next = items
      .slice(i + 1)
      .find((w) => w.lat !== item.lat || w.lon !== item.lon);
    let exitAngle = 0;
    if (next) {
      const target = offset(next.lat, next.lon, home);
      const dn = target.north - centre.north,
        de = target.east - centre.east;
      const distance = Math.hypot(dn, de);
      if (distance > radius)
        exitAngle =
          Math.atan2(de, dn) - direction * Math.acos(radius / distance);
    }
    const previous = items[i - 1];
    const startAlt =
      item.command === 31 &&
      previous &&
      previous.lat === item.lat &&
      previous.lon === item.lon
        ? previous.frame === 3
          ? previous.alt
          : previous.alt - home.alt
        : alt;
    for (let step = 0; step <= 64; step++) {
      const fraction = step / 64,
        angle = exitAngle + direction * 2 * Math.PI * fraction;
      path.push({
        north: centre.north + radius * Math.cos(angle),
        east: centre.east + radius * Math.sin(angle),
        alt: startAlt + (alt - startAlt) * fraction,
      });
    }
  });
  return path;
}

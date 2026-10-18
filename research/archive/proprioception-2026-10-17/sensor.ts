/** Research-only relative-orientation sensor; does not alter payload navigation. */
import { Quaternion, Vector3 } from "../../../viewer/packages/contracts/src/math";

export function relativeChannels(state: number[]) {
  const payload = new Quaternion().fromArray(state, 3).normalize();
  const canopy = new Quaternion().fromArray(state, 16).normalize();
  const relative = payload.clone().invert().multiply(canopy).normalize();
  if (relative.w < 0) relative.fromArray(relative.toArray().map(v => -v));
  const rate = new Vector3().fromArray(state, 23)
    .sub(new Vector3().fromArray(state, 10)).applyQuaternion(payload.clone().invert());
  return { relative, rate };
}

export function makeSensor(kind: string, seed: number) {
  if (!["none", "angles", "full", "sensor", "stress"].includes(kind)) throw Error(kind);
  let random = (seed + 93719) >>> 0;
  const uniform = () => {
    random ^= random << 13; random ^= random >>> 17; random ^= random << 5;
    return ((random >>> 0) + 1) / 4294967297;
  };
  const normal = () => Math.sqrt(-2 * Math.log(uniform())) * Math.cos(2 * Math.PI * uniform());
  const noisy = kind === "sensor" || kind === "stress";
  const delay = noisy ? (kind === "stress" ? .08 : .04) : 0;
  const sigma = (kind === "stress" ? 2 : .5) * Math.PI / 180;
  const history: { time: number; q: number[]; rate: number[] }[] = [];
  let next = -Infinity;
  return (state: number[], time: number) => {
    if (time >= next) {
      const { relative, rate } = relativeChannels(state);
      if (noisy) {
        const error = new Vector3(normal()*sigma, normal()*sigma, normal()*sigma);
        const magnitude = Math.hypot(error.x, error.y, error.z);
        if (magnitude > 0) {
          const scale = Math.sin(magnitude/2)/magnitude;
          relative.multiply(new Quaternion(error.x*scale, error.y*scale, error.z*scale, Math.cos(magnitude/2)));
        }
      }
      history.push({ time, q: [relative.w, relative.x, relative.y, relative.z], rate: rate.toArray() });
      next = time + .02 - 1e-8;
    }
    while (history.length > 1 && history[1].time <= time - delay) history.shift();
    const sample = history[0];
    return { microagu_relative: sample.q, microagu_rate: sample.rate,
      microagu_mode: kind === "none" ? 0 : noisy ? 3 : kind === "angles" ? 1 : 2 };
  };
}

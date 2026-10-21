import { sample } from "@microagu/playback/recordings";
import { type Frame } from "@microagu/contracts/recordings";

// A continuous playout clock, never reset by packet arrival. Keep enough history
// to absorb worker/message scheduling jitter without extrapolating the physics.
export class TruthBuffer {
  private frames: Frame[] = [];
  private time: number | null = null;
  private rate = 1;
  private anchor: { wall: number; sim: number } | null = null;
  clear() {
    this.frames = [];
    this.time = null;
    this.rate = 1;
    this.anchor = null;
  }
  push(frame: Frame, wall: number) {
    if (this.frames.length && frame.time_s <= this.frames.at(-1)!.time_s)
      return;
    this.frames.push(frame);
    if (!this.anchor) this.anchor = { wall, sim: frame.time_s };
    else if (wall - this.anchor.wall >= 1) {
      this.rate = Math.max(
        0.01,
        Math.min(
          8,
          (frame.time_s - this.anchor.sim) / (wall - this.anchor.wall),
        ),
      );
      this.anchor = { wall, sim: frame.time_s };
    }
    if (this.frames.length > 120) this.frames.shift();
  }
  advance(dt: number): Frame | null {
    if (!this.frames.length) return null;
    const first = this.frames[0].time_s,
      latest = this.frames.at(-1)!.time_s;
    if (this.time === null) this.time = first;
    const delay = 0.12 * this.rate;
    const correction = Math.max(
      -0.1 * this.rate,
      Math.min(0.1 * this.rate, (latest - this.time - delay) * 0.5),
    );
    this.time = Math.max(
      first,
      Math.min(latest, this.time + Math.max(0, dt) * (this.rate + correction)),
    );
    while (this.frames.length > 2 && this.frames[1].time_s < this.time)
      this.frames.shift();
    return sample(this.frames, this.time);
  }
}

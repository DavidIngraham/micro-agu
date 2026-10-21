import type { Frame } from "./recordings";
import type { Telemetry } from "./telemetry";
import type { Waypoint, Home } from "./mission";
import type { Weather } from "./weather";
export interface Calls {
  start: [url: string, home: Home];
  mode: [name: string];
  controller: [scheme: "baseline" | "lqi"];
  altitude: [metres: number];
  speedup: [factor: number];
  wind: [speed: number, from: number, up: number, weather?: Weather];
  launch: [mode: string];
  upload: [items: Waypoint[]];
  execute: [];
}
export type WorkerRequest =
  | {
      [K in keyof Calls]: { id: number; method: K; args: Calls[K] };
    }[keyof Calls]
  | { method: "controls"; args: [number, number, number]; id?: never };
export interface Events {
  state: Telemetry;
  truth: Frame;
  log: string;
  error: string;
  progress: number;
  trimThrottle: number;
}
export type WorkerEvent =
  | { [K in keyof Events]: { kind: K; data: Events[K] } }[keyof Events]
  | { kind: "reply"; id: number; ok: true }
  | { kind: "reply"; id: number; ok: false; error: string };

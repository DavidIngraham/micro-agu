import type { Frame } from "@microagu/contracts/recordings";
import type { Weather } from "@microagu/contracts/weather";
import {
  initialTelemetry,
  type Telemetry,
} from "@microagu/contracts/telemetry";
import { type Home, type Waypoint } from "@microagu/contracts/mission";
import type {
  Calls,
  WorkerRequest,
  WorkerEvent,
} from "@microagu/contracts/worker-protocol";
export class SimulatorClient {
  private disposed = false;
  private started = false;
  state: Telemetry;
  trimThrottle = 0.45;
  onUpdate = () => {};
  onTruth = (frame: Frame) => {};
  onLog = (text: string) => {};
  onError = (text: string) => {};
  private worker = new Worker(new URL("./worker.ts", import.meta.url), {
    type: "module",
  });
  private id = 0;
  private pending = new Map<
    number,
    {
      resolve: () => void;
      reject: (e: Error) => void;
      timer: ReturnType<typeof setTimeout>;
    }
  >();
  private progress = (n: number) => {};
  constructor(
    readonly home: Home,
    private baseURL: string,
  ) {
    this.state = initialTelemetry(home);
    this.worker.onmessage = ({ data }: MessageEvent<WorkerEvent>) => {
      switch (data.kind) {
        case "state": {
          // performance.now() has a different origin in the dedicated worker.
          const heartbeatChanged =
            this.state.lastHeartbeat !== data.data.lastHeartbeat;
          this.state = { ...data.data };
          if (heartbeatChanged) this.lastReceipt = performance.now();
          this.onUpdate();
          return;
        }
        case "truth":
          this.onTruth(data.data);
          return;
        case "log":
          this.onLog(data.data);
          return;
        case "error":
          this.onError(data.data);
          this.failPending(data.data);
          return;
        case "progress":
          this.progress(data.data);
          return;
        case "trimThrottle":
          this.trimThrottle = data.data;
          return;
      }
      const pending = this.pending.get(data.id);
      if (pending) {
        clearTimeout(pending.timer);
        this.pending.delete(data.id);
        data.ok ? pending.resolve() : pending.reject(Error(data.error));
      }
    };
    this.worker.onerror = (e) => {
      this.onError(e.message);
      this.failPending(e.message);
    };
  }
  lastReceipt = 0;
  private failPending(text: string) {
    for (const p of this.pending.values()) {
      clearTimeout(p.timer);
      p.reject(Error(text));
    }
    this.pending.clear();
  }
  private call<K extends keyof Calls>(method: K, ...args: Calls[K]) {
    if (this.disposed)
      return Promise.reject(Error("Simulator session disposed"));
    return new Promise<void>((resolve, reject) => {
      const id = ++this.id;
      const timer = setTimeout(
        () => {
          this.pending.delete(id);
          reject(Error("Simulator request timed out; reset to restart."));
        },
        method === "start" || method === "launch" ? 120000 : 30000,
      );
      this.pending.set(id, { resolve, reject, timer });
      this.worker.postMessage({ id, method, args } as WorkerRequest);
    });
  }
  start() {
    if (this.started)
      return Promise.reject(
        Error("Simulator already started; create a new session to reset"),
      );
    this.started = true;
    return this.call(
      "start",
      new URL("wasm/arduplane.js", this.baseURL).href,
      this.home,
    );
  }
  mode(name: string) {
    return this.call("mode", name);
  }
  controller(scheme: "baseline" | "lqi") {
    return this.call("controller", scheme);
  }
  altitude(metres: number) {
    return this.call("altitude", metres);
  }
  speedup(factor: number) {
    return this.call("speedup", factor);
  }
  launch(mode: string) {
    return this.call("launch", mode);
  }
  execute() {
    return this.call("execute");
  }
  wind(speed: number, from: number, up: number, weather?: Weather) {
    return this.call("wind", speed, from, up, weather);
  }
  controls(roll: number, pitch: number, throttle: number) {
    if (this.disposed) return;
    this.worker.postMessage({
      method: "controls",
      args: [roll, pitch, throttle],
    });
  }
  upload(items: Waypoint[], progress: (n: number) => void) {
    this.progress = progress;
    return this.call("upload", items);
  }
  dispose() {
    if (this.disposed) return;
    this.disposed = true;
    this.worker.onmessage = null;
    this.worker.onerror = null;
    this.worker.terminate();
    this.failPending("Simulator session disposed");
    this.onUpdate = () => {};
    this.onTruth = () => {};
    this.onError = () => {};
    this.onLog = () => {};
    this.progress = () => {};
  }
}

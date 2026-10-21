import { Simulator } from "@microagu/sim-browser/session/runtime";
import type {
  Events,
  WorkerEvent,
  WorkerRequest,
} from "@microagu/contracts/worker-protocol";
let sim: Simulator;
const send = <K extends keyof Events>(kind: K, data: Events[K]) =>
  postMessage({ kind, data } as WorkerEvent);
function connect() {
  let lastUpdate = 0;
  sim.onUpdate = () => {
    if (sim.state.simulationEnded || performance.now() - lastUpdate >= 40) {
      lastUpdate = performance.now();
      send("state", sim.state);
      if (sim.state.simulationEnded) self.close();
    }
  };
  sim.onTruth = (frame) => send("truth", frame);
  sim.onLog = (text) => send("log", text);
  sim.onError = (text) => send("error", text);
}
self.onmessage = async ({ data }: MessageEvent<WorkerRequest>) => {
  const { id, method, args } = data;
  try {
    switch (method) {
      case "start":
        if (sim) throw Error("Session already started");
        sim = new Simulator(args[1]);
        connect();
        await sim.start(args[0]);
        send("trimThrottle", sim.physics.config.command[0]);
        break;
      case "controls":
        sim.controls(...(args as [number, number, number]));
        return;
      case "mode":
        await sim.mode(args[0]);
        break;
      case "controller":
        await sim.controller(args[0]);
        break;
      case "altitude":
        await sim.altitude(args[0]);
        break;
      case "speedup":
        await sim.speedup(args[0]);
        break;
      case "wind":
        sim.physics.setWind(...args);
        break;
      case "launch":
        await sim.launch(args[0]);
        break;
      case "upload":
        await sim.upload(args[0], (n) => send("progress", n));
        break;
      case "execute":
        await sim.execute();
        break;
      default:
        throw Error("Unknown simulator worker request");
    }
    postMessage({ kind: "reply", id, ok: true } satisfies WorkerEvent);
  } catch (e) {
    if (method === "start") sim?.dispose();
    if (id === undefined) {
      send("error", String(e));
      return;
    }
    postMessage({
      kind: "reply",
      id,
      ok: false,
      error: e instanceof Error ? e.message : String(e),
    } satisfies WorkerEvent);
  }
};

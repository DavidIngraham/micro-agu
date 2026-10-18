import { registerApplet } from "@microagu/applet-ui/shell/mount";
import { playback } from "@microagu/applet-ui/controls/playback";
import { parseRecording } from "@microagu/contracts/recordings";
import { sample } from "@microagu/playback/recordings";
import { truthPose } from "@microagu/playback/pose";
import { FlightView } from "@microagu/rendering/scene/flight-view";
import { RotorPhase, phaseAt } from "@microagu/rendering/aircraft/mechanisms";
import { definition } from "./definition";
export const applet = registerApplet(definition, async (scope) => {
  const response = await scope.fetch("takeoff/launch.jsonl");
  if (!response.ok) throw Error("Takeoff recording unavailable");
  const frames = parseRecording(await response.text());
  const status = document.querySelector<HTMLElement>("#status")!;
  const view = scope.own(
    new FlightView(document.querySelector<HTMLElement>("#view")!, (text) => {
      status.textContent = text;
    }),
  );
  const rotor = new RotorPhase();
  const phases = frames.map((f) => rotor.advance(f.time_s, f.rpm));
  let last = performance.now();
  const player = playback(
    scope,
    frames[0].time_s,
    frames.at(-1)!.time_s,
    (time) => {
      const frame = sample(frames, time);
      const now = performance.now();
      view.render(
        truthPose(frame),
        Math.min(0.1, (now - last) / 1000),
        phaseAt(frames, phases, time),
      );
      last = now;
      document.querySelector("#phase")!.textContent =
        frame.mode === "TAKEOFF"
          ? "TAKEOFF · autonomous climb"
          : frame.mode === "AUTO"
            ? "AUTO · mission handoff"
            : (frame.launch_stage ?? "Handling");
      document.querySelector("#telemetry")!.textContent =
        `Height ${(-frame.state[2]).toFixed(1)} m · Speed ${Math.hypot(...frame.state.slice(7, 10)).toFixed(1)} m/s · Throttle ${Math.round((frame.throttle ?? 0) * 100)}% · Brakes ${(frame.brake_mm ?? []).map((x) => x.toFixed(0)).join(" / ")} mm`;
    },
  );
  for (const [id, predicate] of [
    [
      "release",
      (f: (typeof frames)[number]) => f.launch_stage === "Free flight",
    ],
    ["climb", (f: (typeof frames)[number]) => f.mode === "TAKEOFF"],
    ["mission", (f: (typeof frames)[number]) => f.mode === "AUTO"],
  ] as const)
    scope.listen(document.querySelector(`#${id}`)!, "click", () =>
      player.seek(frames.find(predicate)!.time_s),
    );
  scope.listen(document.querySelector("#camera")!, "click", () => {
    document.querySelector("#camera")!.textContent = `Camera: ${view.cycle()}`;
  });
  status.textContent = "Recorded simulation · no SITL startup required";
});

import type { AppletDefinition } from "@microagu/contracts/applet";
export const definition = {
  id: "trim-launch",
  title: "MicroAGU · Recorded Takeoff",
  page: "trim-launch.html",
  html: "apps/applets/trim-launch/index.html",
  entry: "apps/applets/trim-launch/applet.ts",
  kind: "replay",
  assets: [
    "models/microagu-payload-tl.glb",
    "takeoff/launch.jsonl",
    "takeoff/provenance.json",
  ],
  controls: ["playback", "camera"],
  defaultCamera: "follow",
} satisfies AppletDefinition;

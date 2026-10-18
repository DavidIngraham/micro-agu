import type { AppletDefinition } from "@microagu/contracts/applet";
export const definition = {
  id: "joint-mechanics",
  title: "Point-joint springs, dampers and yaw geometry",
  page: "joint-mechanics.html",
  html: "apps/applets/joint-mechanics/index.html",
  entry: "apps/applets/joint-mechanics/applet.ts",
  kind: "explanation",
  assets: [],
  controls: ["axis", "playback", "phase", "yaw", "camera"],
  defaultCamera: "orbit",
} satisfies AppletDefinition;

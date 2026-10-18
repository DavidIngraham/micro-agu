import type { AppletDefinition } from "@microagu/contracts/applet";
export const definition = {
  id: "canopy-panels",
  title: "How the five-panel canopy produces forces",
  page: "canopy-panels.html",
  html: "apps/applets/canopy-panels/index.html",
  entry: "apps/applets/canopy-panels/applet.ts",
  kind: "explanation",
  assets: [],
  controls: ["case", "panel", "camera"],
  defaultCamera: "orbit",
} satisfies AppletDefinition;

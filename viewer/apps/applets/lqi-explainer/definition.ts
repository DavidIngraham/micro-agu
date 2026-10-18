import type { AppletDefinition } from "@microagu/contracts/applet";
export const definition = {
  id: "lqi-explainer",
  title: "Understanding coordinated control",
  page: "lqi-explainer.html",
  html: "apps/applets/lqi-explainer/index.html",
  entry: "apps/applets/lqi-explainer/applet.ts",
  kind: "explanation",
  assets: [],
  controls: ["view", "input", "time", "state", "flow"],
  defaultCamera: "orbit",
} satisfies AppletDefinition;

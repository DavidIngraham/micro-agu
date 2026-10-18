import type { AppletDefinition } from "@microagu/contracts/applet";
export const definition = {
  id: "lqi-development",
  title: "LQI: authority and evidence",
  page: "lqi-development.html",
  html: "apps/applets/lqi-development/index.html",
  entry: "apps/applets/lqi-development/applet.ts",
  kind: "explanation",
  assets: [],
  controls: ["common", "differential", "metric"],
  defaultCamera: "orbit",
} satisfies AppletDefinition;

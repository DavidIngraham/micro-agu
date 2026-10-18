import { definition as app0 } from "./replay/definition";
import { definition as app1 } from "./simulation-replay/definition";
import { definition as app2 } from "./altitude-comparison/definition";
import { definition as app3 } from "./pitch-comparison/definition";
import { definition as app4 } from "./flight-replay/definition";
import { definition as app5 } from "./joint-tour/definition";
import { definition as app6 } from "./pitch-fbd/definition";
import { definition as app7 } from "./trim-launch/definition";
import { definition as app8 } from "./landing/definition";
import { definition as app9 } from "./controller-comparison/definition";
import { definition as app10 } from "./rigging-comparison/definition";
import { definition as app11 } from "../simulator/definition";
import { definition as canopyPanels } from "./canopy-panels/definition";
export const applets = [
  canopyPanels,
  app0,
  app1,
  app2,
  app3,
  app4,
  app5,
  app6,
  app7,
  app8,
  app9,
  app10,
  app11,
];

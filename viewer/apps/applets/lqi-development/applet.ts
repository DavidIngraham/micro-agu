import { registerApplet } from "@microagu/applet-ui/shell/mount";
import { definition } from "./definition";
import evidence from "./evidence.json";
import "./style.css";
const el = (id: string) => document.getElementById(id)!;
export const applet = registerApplet(definition, (scope) => {
  const common = el("common") as HTMLInputElement,
    differential = el("differential") as HTMLInputElement;
  const ctx = (el("authority") as HTMLCanvasElement).getContext("2d")!;
  const clamp = (v: number) => Math.max(0, Math.min(1, v));
  function draw() {
    const c = Number(common.value) / 100,
      d = Number(differential.value) / 100,
      l = clamp(c + d),
      r = clamp(c - d),
      ac = (l + r) / 2,
      ad = (l - r) / 2;
    el("common-value").textContent = `${common.value}%`;
    el("differential-value").textContent = `${differential.value}%`;
    const x = (v: number) => 300 + 460 * v,
      y = (v: number) => 355 - 290 * v;
    ctx.clearRect(0, 0, 600, 420);
    ctx.fillStyle = "#e0efeb";
    ctx.strokeStyle = "#237f78";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x(0), y(0));
    ctx.lineTo(x(0.5), y(0.5));
    ctx.lineTo(x(0), y(1));
    ctx.lineTo(x(-0.5), y(0.5));
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.strokeStyle = "#9cb2ba";
    ctx.beginPath();
    ctx.moveTo(45, y(0));
    ctx.lineTo(555, y(0));
    ctx.moveTo(x(0), 35);
    ctx.lineTo(x(0), 370);
    ctx.stroke();
    ctx.font = "18px system-ui";
    ctx.fillStyle = "#203e49";
    ctx.fillText("Common brake 100%", 195, 26);
    ctx.fillText("Differential: −50%", 38, 397);
    ctx.fillText("0", 295, 397);
    ctx.fillText("+50%", 480, 397);
    ctx.setLineDash([5, 5]);
    ctx.strokeStyle = "#b76234";
    ctx.beginPath();
    ctx.moveTo(x(d), y(c));
    ctx.lineTo(x(ad), y(ac));
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = "#b76234";
    ctx.beginPath();
    ctx.arc(x(d), y(c), 9, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "#fff";
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = "#237f78";
    ctx.beginPath();
    ctx.arc(x(ad), y(ac), 5, 0, Math.PI * 2);
    ctx.fill();
    el("brakes").innerHTML =
      `<strong>After travel clipping</strong><div>Left ${(100 * l).toFixed(0)}%<div class="brake"><span style="width:${100 * l}%"></span></div></div><div>Right ${(100 * r).toFixed(0)}%<div class="brake"><span style="width:${100 * r}%"></span></div></div><p>${Math.abs(c - ac) + Math.abs(d - ad) > 0.001 ? "The requested pair is impossible. Clipping changes both common and differential action." : "Both requested brake positions fit inside the available travel."}</p>`;
  }
  function compare() {
    const metric = (el("metric") as HTMLSelectElement).value as
      "pitch_rms" | "pitch_peak" | "energy_Wh";
    const values = evidence.pairs.map((p) => p.ratios[metric]);
    el("comparison").innerHTML = evidence.pairs
      .map(
        (p, i) =>
          `<div class="row"><span>Case ${p.case}</span><div class="track"><div class="fill" style="width:${values[i] * 100}%"></div></div><strong>${values[i].toFixed(3)}</strong></div>`,
      )
      .join("");
    const sorted = [...values].sort((a, b) => a - b);
    el("summary").textContent =
      `Median ${((sorted[3] + sorted[4]) / 2).toFixed(3)} · worst ${sorted[7].toFixed(3)} · dashed right edge = PG + TECS (1.000).`;
  }
  scope.listen(common, "input", draw);
  scope.listen(differential, "input", draw);
  scope.listen(el("metric"), "change", compare);
  draw();
  compare();
});

/** Exercise the built page with real WASM physics and SITL. */
import { chromium, expect } from "@playwright/test";
import { spawn } from "node:child_process";
import { writeFile } from "node:fs/promises";
const [target = "local", output = "dryden-browser.json"] =
  process.argv.slice(2);
const base = target === "local" ? "http://127.0.0.1:4198/" : target;
const server =
  target === "local"
    ? spawn(
        "python3",
        [
          "-m",
          "http.server",
          "4198",
          "--bind",
          "127.0.0.1",
          "--directory",
          "dist",
        ],
        { stdio: "ignore" },
      )
    : undefined;
const browser = await chromium.launch({
  headless: true,
  args: [
    "--enable-webgl",
    "--use-gl=angle",
    "--use-angle=swiftshader",
    "--enable-unsafe-swiftshader",
  ],
});
const page = await browser.newPage({ viewport: { width: 390, height: 844 } });
const errors: string[] = [];
page.on("pageerror", (e) => errors.push(e.message));
try {
  await page.goto(new URL("simulator.html", base).href, {
    waitUntil: "domcontentloaded",
    timeout: 60000,
  });
  await expect(page.locator("#status")).toContainText("AUTO confirmed", {
    timeout: 130000,
  });
  await page.locator("#panel-toggle").click();
  await expect(page.locator("#controller")).toHaveValue("lqi");
  await expect(page.locator("#controller-status")).toHaveText("LQI active", {
    timeout: 20000,
  });
  await expect(page.locator("#turbulence-model")).toHaveValue("dryden");
  await page.locator("#gust-length").fill("40");
  await page.locator("#gust-length").dispatchEvent("input");
  await page.locator("#vertical-ratio").fill("0.8");
  await page.locator("#vertical-ratio").dispatchEvent("input");
  await expect(page.locator("#gust-length-value")).toHaveText("40 m");
  await expect(page.locator("#vertical-ratio-value")).toHaveText("80%");
  await page.locator("#apply-wind").click();
  await expect(page.locator("#wind-status")).toContainText("Applied:");
  await page.locator("#controller").selectOption("lqi");
  await expect(page.locator("#controller-status")).toHaveText("LQI active", {
    timeout: 20000,
  });
  await page.waitForTimeout(15000);
  await page.locator("#gust-length").scrollIntoViewIfNeeded();
  await page.screenshot({ path: output.replace(/\.json$/, ".png") });
  await page.locator("#turbulence-model").selectOption("legacy");
  await expect(page.locator("#gust-length")).toBeDisabled();
  await page.locator("#apply-wind").click();
  await page.locator("#turbulence-model").selectOption("dryden");
  await expect(page.locator("#gust-length")).toBeEnabled();
  await page.locator("#apply-wind").click();
  await page.waitForTimeout(3000);
  await expect(page.locator("#controller-status")).toHaveText("LQI active");
  expect(errors).toEqual([]);
  await writeFile(
    output,
    JSON.stringify(
      {
        passed: true,
        base,
        checks: [
          "LQI selected and active after automatic launch",
          "mobile controls",
          "Dryden default",
          "parameter application",
          "LQI flight",
          "legacy switch",
          "no page errors",
        ],
      },
      null,
      2,
    ),
  );
} catch (e) {
  await writeFile(
    output,
    JSON.stringify({ passed: false, error: String(e), errors }, null, 2),
  );
  throw e;
} finally {
  await browser.close();
  server?.kill();
}

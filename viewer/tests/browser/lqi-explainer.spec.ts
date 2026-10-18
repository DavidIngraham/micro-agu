import { test, expect } from "@playwright/test";
import { readFileSync } from "node:fs";
import { createHash } from "node:crypto";
test("published matrix identity and contribution scaling", () => {
  const d = JSON.parse(
      readFileSync("apps/applets/lqi-explainer/data.json", "utf8"),
    ),
    b = JSON.parse(readFileSync("assets/runtime/build.json", "utf8"));
  expect(
    createHash("sha256")
      .update(readFileSync("apps/applets/lqi-explainer/deployed-gains.h"))
      .digest("hex"),
  ).toBe(
    b.controller_source_sha256[
      "libraries/AP_CustomControl/AP_CustomControl_LQI_gains.h"
    ],
  );
  expect(d.provenance.gain_reconstruction_max_error).toBeLessThan(1e-7);
  expect(d.physicalGain[0][3]).toBeCloseTo((-10 * d.rawK[0][3]) / 10, 8);
  expect(d.physicalGain[2][17]).toBeCloseTo((-10 * d.rawK[2][17]) / 10, 8);
});
for (const view of [
  "coupling",
  "motion",
  "observer",
  "gains",
  "online",
  "offline",
  "replay",
])
  test(view + " responds on mobile", async ({ page }) => {
    await page.setViewportSize({ width: 390, height: 844 });
    const errors: string[] = [];
    page.on("pageerror", (e) => errors.push(e.message));
    await page.goto("/lqi-explainer.html?view=" + view);
    await expect(page.locator("#detail")).not.toBeEmpty();
    if (["coupling", "observer", "replay"].includes(view)) {
      await page.locator("#time").fill("20");
      await expect(page.locator("#time-value")).not.toHaveText("0.0 s");
      if (view === "observer") {
        await page.locator("#truth").uncheck();
        await expect(page.locator("#readout")).not.toContainText("truth");
      }
    }
    if (view === "gains") {
      await expect(page.locator(".heat-row button")).toHaveCount(57);
      await page
        .getByRole("button", { name: /Relative pitch to Throttle/ })
        .click();
      await expect(page.locator("#detail")).toContainText(
        "Physical command coefficient",
      );
      await page.locator("#factor").fill("-1");
      await expect(page.locator("#detail")).toContainText("-5.00 deg");
    }
    if (view === "motion") {
      await page.locator("#direction").selectOption("-1");
      await expect(page.locator("#detail")).toContainText("-10°/s");
    }
    if (view === "online" || view === "offline") {
      await page.locator(".flow button").last().click();
      await expect(page.locator("#detail")).not.toBeEmpty();
    }
    expect(
      await page.evaluate(
        () => document.documentElement.scrollWidth <= innerWidth,
      ),
    ).toBe(true);
    expect(errors).toEqual([]);
    await page.screenshot({
      path: "test-results/lqi-" + view + "-mobile.png",
      fullPage: true,
    });
  });

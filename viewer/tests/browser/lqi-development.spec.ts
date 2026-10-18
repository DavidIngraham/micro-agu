import { test, expect } from "@playwright/test";
import { readFileSync } from "node:fs";
test("brake geometry, paired evidence and mobile layout", async ({ page }) => {
  const source = JSON.parse(
    readFileSync("assets/runtime/lqi-longitudinal-validation.json", "utf8"),
  );
  const evidence = JSON.parse(
    readFileSync("apps/applets/lqi-development/evidence.json", "utf8"),
  );
  expect(evidence.pairs).toEqual(source.summary.pairs);
  await page.setViewportSize({ width: 390, height: 844 });
  await page.goto("/lqi-development.html");
  await expect(page.locator("#brakes")).toContainText("Left 40%");
  await expect(page.locator("#brakes")).toContainText("Right 0%");
  await page.locator("#common").fill("50");
  await expect(page.locator("#brakes")).toContainText("Both requested");
  await page.locator("#metric").selectOption("energy_Wh");
  await expect(page.locator(".row")).toHaveCount(8);
  await expect(page.locator("#summary")).toContainText("Median 0.956");
  expect(
    await page.evaluate(
      () => document.documentElement.scrollWidth <= innerWidth,
    ),
  ).toBe(true);
});

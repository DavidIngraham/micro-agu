import { test, expect } from "@playwright/test";

test("recorded takeoff scrubs on mobile without loading SITL", async ({
  page,
}) => {
  await page.setViewportSize({ width: 390, height: 844 });
  const errors: string[] = [],
    requests: string[] = [];
  page.on("pageerror", (error) => errors.push(error.message));
  page.on("request", (request) => requests.push(request.url()));
  await page.goto("/trim-launch.html");
  await expect(page.locator("#status")).toContainText("Recorded simulation");
  await page.locator("#play").click();
  await page.locator("#climb").click();
  await expect(page.locator("#phase")).toContainText("TAKEOFF");
  await page.locator("#mission").click();
  await expect(page.locator("#phase")).toContainText("AUTO");
  await page.locator("#time").fill("3");
  await expect(page.locator("#phase")).not.toContainText("AUTO");
  await page.locator("#camera").click();
  await expect(page.locator("#camera")).toHaveText("Camera: orbit");
  await page.locator("#climb").click();
  await page.waitForTimeout(500);
  await page.screenshot({ path: "test-results/takeoff-replay-mobile.png" });
  expect(
    await page.evaluate(
      () => document.documentElement.scrollWidth <= innerWidth,
    ),
  ).toBe(true);
  expect(
    requests.some((url) =>
      /\.wasm(?:\?|$)|arduplane\.js|simulator-sw/.test(url),
    ),
  ).toBe(false);
  expect(errors).toEqual([]);
});

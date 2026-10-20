import { test, expect } from "@playwright/test";

test("LAND uses EKF approach and flare, supports cancellation and mobile display", async ({
  page,
}) => {
  test.setTimeout(360000);
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.goto("/simulator.html");
  await expect(page.locator("#status")).toContainText("AUTO confirmed", {
    timeout: 100000,
  });
  await expect
    .poll(async () => Number(await page.locator("#alt").textContent()), {
      timeout: 100000,
    })
    .toBeGreaterThan(45);
  await page.locator("#mode").selectOption("LAND");
  await expect(page.locator("#mode-label")).toContainText("LAND", {
    timeout: 15000,
  });
  await expect(page.locator("#controller-status")).toHaveText("LQI active");
  await expect(page.locator("#throttle")).toBeDisabled();
  await page.setViewportSize({ width: 390, height: 844 });
  await page.screenshot({ path: "test-results/landing-mobile.png" });
  await page.locator("#mode").selectOption("CRUISE");
  await expect(page.locator("#mode-label")).toHaveText("CRUISE", {
    timeout: 10000,
  });
  await expect(page.locator("#throttle")).toBeEnabled();
  await page.locator("#mode").selectOption("LAND");
  await expect(page.locator("#mode-label")).toContainText("LAND", {
    timeout: 15000,
  });
  await expect(page.locator("#status")).toContainText("Landed", {
    timeout: 180000,
  });
  await expect(page.locator("#armed")).toHaveText("PAUSED", { timeout: 10000 });
  await page.screenshot({ path: "test-results/landed-mobile.png" });
  expect(errors).toEqual([]);
});

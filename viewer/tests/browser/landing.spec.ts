import { test, expect } from "@playwright/test";

test("native AUTO landing mission, cancellation and mobile display", async ({
  page,
}) => {
  test.setTimeout(700000);
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.goto("/simulator.html");
  await expect(page.locator("#status")).toContainText("AUTO confirmed", {
    timeout: 100000,
  });
  await expect(page.locator('#mode option[value="LAND"]')).toHaveCount(0);
  await expect
    .poll(async () => Number(await page.locator("#alt").textContent()), {
      timeout: 100000,
    })
    .toBeGreaterThan(45);
  await page.locator("#mode").selectOption("CRUISE");
  // Mission uploads are normal MAVLink missions; AUTO owns every landing phase.
  await page.locator("#panel-toggle").click();
  await page.locator("#landing-example").click();
  await page.locator("#upload").click();
  await expect(page.locator("#mission-info")).toContainText("accepted");
  await page.locator("#execute").click();
  await expect(page.locator("#mode-label")).toHaveText("AUTO", {
    timeout: 10000,
  });
  await expect(page.locator("#status")).toContainText("AUTO landing approach", {
    timeout: 300000,
  });
  await expect(page.locator("#controller-status")).toHaveText("LQI active");
  await page.locator("#panel-toggle").click();
  await page.setViewportSize({ width: 390, height: 844 });
  await page.screenshot({ path: "test-results/landing-mobile.png" });
  await page.locator("#mode").selectOption("CRUISE");
  await expect(page.locator("#mode-label")).toHaveText("CRUISE", {
    timeout: 10000,
  });
  await expect(page.locator("#throttle")).toBeEnabled();
  await page.locator("#mode").selectOption("AUTO");
  await expect(page.locator("#status")).toContainText("Landed", {
    timeout: 180000,
  });
  await expect(page.locator("#mode-label")).toHaveText("AUTO");
  await expect(page.locator("#armed")).toHaveText("PAUSED");
  await page.screenshot({ path: "test-results/landed-mobile.png" });
  expect(errors).toEqual([]);
});

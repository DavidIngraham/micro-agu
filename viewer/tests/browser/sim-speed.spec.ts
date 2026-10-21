import { test, expect } from "@playwright/test";

test("settings change live simulation speed and reset restores real time", async ({
  page,
}) => {
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.goto("/simulator.html");
  await page.locator("#panel-toggle").click();
  const speed = page.getByLabel("Simulation speed", { exact: true });
  await expect(speed).toHaveValue("1");
  await expect(speed).toBeDisabled();
  await expect(page.locator("#status")).toContainText("AUTO confirmed", {
    timeout: 100000,
  });
  for (const factor of ["2", "4", "8", "1"]) {
    await speed.selectOption(factor);
    await expect(page.locator("#status")).toHaveText(
      `Simulation speed ${factor}× confirmed.`,
      { timeout: 10000 },
    );
    await expect(speed).toBeEnabled();
    await expect(speed).toHaveValue(factor);
  }
  await page.setViewportSize({ width: 390, height: 844 });
  await expect(speed).toBeInViewport();
  await page.screenshot({ path: "test-results/sim-speed-mobile.png" });
  await speed.selectOption("4");
  await expect(page.locator("#status")).toHaveText(
    "Simulation speed 4× confirmed.",
  );
  await page.locator("#panel-toggle").click();
  await page.locator("#launch").click();
  await page.locator("#panel-toggle").click();
  await expect(speed).toHaveValue("1");
  await expect(speed).toBeDisabled();
  expect(errors).toEqual([]);
});

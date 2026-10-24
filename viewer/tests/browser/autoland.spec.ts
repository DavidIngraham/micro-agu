import { test, expect } from "@playwright/test";

test("AUTOLAND generates a home landing and leaves the mission available", async ({
  page,
}) => {
  test.setTimeout(300000);
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.goto("/simulator.html");
  await expect(page.locator("#status")).toContainText("AUTO confirmed", {
    timeout: 100000,
  });
  await page.locator("#panel-toggle").click();
  await page.getByLabel("Simulation speed", { exact: true }).selectOption("8");
  await expect(page.locator("#status")).toHaveText(
    "Simulation speed 8× confirmed.",
  );
  await page.locator("#mode").selectOption("AUTOLAND");
  await expect(page.locator("#mode-label")).toHaveText("AUTOLAND", {
    timeout: 10000,
  });
  await expect(page.locator("#mission-current")).toHaveText("HOME · AUTOLAND");
  await expect(page.locator("#altitude-open")).toBeDisabled();
  await page.locator("#mode").selectOption("AUTO");
  await expect(page.locator("#mode-label")).toHaveText("AUTO");
  await expect(page.locator("#mission-current")).toContainText("ITEM");
  await page.locator("#mode").selectOption("AUTOLAND");
  await expect(page.locator("#mode-label")).toHaveText("AUTOLAND");
  await expect(page.locator("#status")).toContainText("Landed", {
    timeout: 180000,
  });
  await expect(page.locator("#armed")).toHaveText("PAUSED");
  await expect(page.locator("#log")).not.toContainText("Timed out waiting");
  await expect(page.locator("#mode option[value=RTL]")).toHaveText(
    "RTL · mission landing",
  );
  expect(errors).toEqual([]);
});

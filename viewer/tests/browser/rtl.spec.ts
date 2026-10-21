import { test, expect } from "@playwright/test";

test("default mission repeats, then RTL enters the native landing sequence", async ({
  page,
}) => {
  test.setTimeout(360000);
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
  await expect(page.locator("#mission-current")).toHaveText("ITEM 5 / 11", {
    timeout: 150000,
  });
  await expect(page.locator("#mission-current")).toHaveText("ITEM 1 / 11", {
    timeout: 60000,
  });
  await expect(page.locator("#mode-label")).toHaveText("AUTO");
  await page.locator("#panel-toggle").click();
  await page.locator("#mode").selectOption("RTL");
  await expect(page.locator("#mode")).toBeEnabled({ timeout: 10000 });
  await expect(page.locator("#mode-label")).toHaveText("AUTO", {
    timeout: 10000,
  });
  await expect(page.locator("#mission-current")).toHaveText("ITEM 8 / 11", {
    timeout: 10000,
  });
  await expect(page.locator("#status")).toContainText("Landed", {
    timeout: 180000,
  });
  await expect(page.locator("#armed")).toHaveText("PAUSED");
  await expect(page.locator("#log")).not.toContainText("Timed out waiting");
  expect(errors).toEqual([]);
});

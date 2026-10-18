import { test, expect } from "@playwright/test";
test("canopy panels: cases, panel inspection and mobile layout", async ({
  page,
}) => {
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.goto("/canopy-panels.html");
  await expect(page.locator("canvas")).toBeVisible();
  await expect(page.locator("#panel-info")).toContainText("Panel 3");
  await expect(page.locator("#tilt-status")).toContainText(
    "disabled in this preset",
  );
  const inclinations = await page.locator("#front").innerText();
  const symmetric = await page.locator("#totals").innerText();
  await page.getByRole("button", { name: "Right brake", exact: true }).click();
  await expect(page.locator("#condition")).toContainText("right 30 mm");
  await expect(page.locator("#panel-info")).toContainText("brake 15 mm");
  expect(await page.locator("#totals").innerText()).not.toEqual(symmetric);
  expect(await page.locator("#front").innerText()).toEqual(inclinations);
  await page
    .getByRole("button", { name: "Inspect panel 5", exact: true })
    .click();
  await expect(page.locator("#panel-info")).toContainText("brake 30 mm");
  await page.screenshot({
    path: "/tmp/canopy-panels-desktop.png",
    fullPage: true,
  });
  await page.setViewportSize({ width: 390, height: 844 });
  await page.getByRole("button", { name: "Front", exact: true }).click();
  expect(
    await page.evaluate(
      () => document.documentElement.scrollWidth <= innerWidth,
    ),
  ).toBe(true);
  await page.screenshot({
    path: "/tmp/canopy-panels-mobile.png",
    fullPage: true,
  });
  expect(errors).toEqual([]);
});

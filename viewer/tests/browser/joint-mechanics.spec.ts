import { test, expect } from "@playwright/test";
test("joint mechanics: phase, zero pitch spring, geometry and mobile", async ({
  page,
}) => {
  await page.emulateMedia({ reducedMotion: "reduce" });
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.goto("/joint-mechanics.html");
  await expect(
    page.getByRole("button", { name: "Play motion", exact: true }),
  ).toBeVisible();
  const set = async (id: string, value: string) =>
    page.locator(id).evaluate((e, v) => {
      (e as HTMLInputElement).value = v;
      e.dispatchEvent(new Event("input", { bubbles: true }));
    }, value);
  await set("#phase", "0");
  await expect(page.locator("#torques tr").nth(1)).toContainText("0.00000 N·m");
  await expect(page.locator("#phase-note")).toContainText("turning point");
  await set("#phase", "90");
  await expect(page.locator("#torques tr").nth(0)).toContainText("0.00000 N·m");
  await expect(page.locator("#phase-note")).toContainText("Passing alignment");
  await page.getByRole("button", { name: "Pitch", exact: true }).click();
  await set("#phase", "0");
  await expect(page.locator("#axis-note")).toContainText(
    "zero explicit pitch-joint stiffness",
  );
  await expect(page.locator("#torques tr").nth(0)).toContainText("0.00000 N·m");
  await page.getByRole("button", { name: "Yaw", exact: true }).click();
  await set("#phase", "45");
  await set("#yaw", "0");
  await expect(page.locator("#geometry-values")).toContainText("0.00 mm");
  await set("#yaw", "60");
  await expect(page.locator("#geometry-values")).toContainText("12.66 mm");
  await page.screenshot({
    path: "/tmp/joint-mechanics-desktop.png",
    fullPage: true,
  });
  await page.setViewportSize({ width: 390, height: 844 });
  await page.getByRole("button", { name: "Top view", exact: true }).click();
  expect(
    await page.evaluate(
      () => document.documentElement.scrollWidth <= innerWidth,
    ),
  ).toBe(true);
  await page.screenshot({
    path: "/tmp/joint-mechanics-mobile.png",
    fullPage: true,
  });
  await page.getByRole("button", { name: "Play motion", exact: true }).click();
  await expect.poll(() => page.locator("#phase").inputValue()).not.toBe("45");
  await page.getByRole("button", { name: "Pause motion", exact: true }).click();
  expect(errors).toEqual([]);
});

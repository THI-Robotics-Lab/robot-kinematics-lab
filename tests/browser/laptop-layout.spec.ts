import { expect, test } from "@playwright/test";

test("laptop workspace keeps joint sliders and robot visible in every mode", async ({ page }, info) => {
  await page.setViewportSize({ width: 1366, height: 768 });
  await page.goto("/");
  const logo = page.getByRole("img", { name: "Technische Hochschule Ingolstadt" });
  await expect(logo).toBeVisible();
  expect(await logo.evaluate((node: HTMLImageElement) => node.complete && node.naturalWidth > 0)).toBe(true);
  await expect(page.getByRole("link", { name: "THI Robotics Lab", exact: true })).toHaveAttribute(
    "href", "https://www.thi.de/elektro-und-informationstechnik/labore/robotik-labor/",
  );
  const thumbnail = page.getByRole("link", { name: "THI Robotics Lab" }).locator("img");
  await expect(thumbnail).toBeVisible();
  expect(await thumbnail.evaluate((node: HTMLImageElement) => node.complete && node.naturalWidth > 0)).toBe(true);
  for (const mode of ["Forward Kinematics", "Inverse Kinematics", "Kinematic Analysis"]) {
    await page.getByRole("button", { name: mode, exact: true }).click();
    await page.evaluate(() => window.scrollTo(0, 0));
    const canvas = (await page.locator("canvas").boundingBox())!;
    const controls = (await page.getByRole("region", { name: "Workspace joint controls" }).boundingBox())!;
    expect(canvas.y).toBeGreaterThanOrEqual(0);
    expect(canvas.y + canvas.height).toBeLessThanOrEqual(768);
    expect(controls.x).toBeGreaterThanOrEqual(canvas.x);
    expect(controls.y + controls.height).toBeLessThanOrEqual(canvas.y + canvas.height);
    const slider = page.getByRole("slider", { name: "Joint 1 slider", exact: true });
    await slider.fill("45");
    await expect(page.getByTestId("joint-label-1")).toHaveAttribute("data-active", "true");
    await expect(page.getByRole("spinbutton", { name: "Joint 1 (deg)", exact: true })).toHaveValue("45");
    await page.screenshot({ path: info.outputPath(`laptop-${mode.split(" ")[0].toLowerCase()}.png`) });
  }
  await page.getByRole("button", { name: /Franka Panda 7R/ }).click();
  await expect(page.getByRole("slider", { name: /^Joint \d slider$/ })).toHaveCount(7);
  const last = (await page.getByRole("slider", { name: "Joint 7 slider" }).boundingBox())!;
  expect(last.y + last.height).toBeLessThanOrEqual(768);
  await page.getByRole("button", { name: "Joint controls (°) −" }).click();
  await expect(page.getByRole("slider", { name: "Joint 7 slider" })).toBeHidden();
  await page.getByRole("button", { name: "Joint controls (°) +" }).click();
  await page.evaluate(() => window.scrollTo(0, 500));
  const stickyCanvas = (await page.locator("canvas").boundingBox())!;
  const stickySlider = (await page.getByRole("slider", { name: "Joint 7 slider" }).boundingBox())!;
  expect(stickyCanvas.y).toBeGreaterThanOrEqual(0);
  expect(stickySlider.y + stickySlider.height).toBeLessThanOrEqual(768);
  await page.getByRole("button", { name: /^Demo C ·/ }).click();
  await expect(page.getByRole("slider", { name: "Joint 1 slider" })).toBeDisabled();
  await page.getByRole("button", { name: /^Demo D ·/ }).click();
  await expect(page.getByRole("slider", { name: "Joint 1 slider" })).toBeDisabled();
});

test("compact workspace also fits a 1280 by 720 laptop", async ({ page }) => {
  await page.setViewportSize({ width: 1280, height: 720 });
  await page.goto("/");
  await page.getByRole("button", { name: "Inverse Kinematics", exact: true }).click();
  const canvas = (await page.locator("canvas").boundingBox())!;
  const controls = (await page.getByRole("region", { name: "Workspace joint controls" }).boundingBox())!;
  expect(canvas.y + canvas.height).toBeLessThanOrEqual(720);
  expect(controls.y + controls.height).toBeLessThanOrEqual(720);
});

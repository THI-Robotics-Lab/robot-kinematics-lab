import { expect, test, type Page, type TestInfo } from "@playwright/test";

async function capture(page: Page, info: TestInfo, name: string) {
  await expect(page.locator("canvas")).toBeVisible();
  // Reset scroll so the sticky robot column has the same position in each
  // full-page review capture, even after interacting with lower controls.
  await page.evaluate(() => {
    window.scrollTo(0, 0);
    return new Promise<void>((resolve) =>
      requestAnimationFrame(() => resolve()),
    );
  });
  await page.screenshot({
    path: info.outputPath(`${name}.png`),
    fullPage: true,
  });
}
const demo = (page: Page, id: string) =>
  page.getByRole("button", { name: new RegExp(`^Demo ${id} ·`) });
const metric = async (page: Page, name: string) =>
  parseFloat(await page.getByTestId(name).innerText());

test("modes expose relevant controls and retain advanced access", async ({
  page,
}, info) => {
  await page.goto("/");
  await page.getByRole("button", { name: /Planar 2R/ }).click();
  await expect(
    page.getByRole("slider", { name: "Joint 1 slider" }),
  ).toBeVisible();
  await expect(
    page.getByRole("spinbutton", { name: "Target X (m)" }),
  ).toHaveCount(0);
  await capture(page, info, "forward");
  await page
    .getByRole("button", { name: "Inverse Kinematics", exact: true })
    .click();
  await expect(page.getByRole("button", { name: "Solve IK" })).toBeVisible();
  await expect(
    page.getByRole("slider", { name: "Joint 1 slider" }),
  ).toHaveCount(1);
  await capture(page, info, "inverse");
  await page.getByLabel("Advanced information").check();
  await expect(
    page.getByRole("slider", { name: "Joint 1 slider" }),
  ).toBeVisible();
  await expect(page.getByText("Inspect the geometric Jacobian")).toBeVisible();
  await page.getByLabel("Advanced information").uncheck();
  await page
    .getByRole("button", { name: "Kinematic Analysis", exact: true })
    .click();
  await expect(page.getByTestId("jacobian-rank")).toHaveText("2 / 2");
  await expect(page.getByRole("button", { name: "Solve IK" })).toHaveCount(0);
  await page.getByRole("slider", { name: "Joint-rate direction" }).fill("135");
  await expect(
    page.getByRole("img", { name: "Unit circle of joint rates" }),
  ).toBeVisible();
  await capture(page, info, "analysis");
});

test("Demo A compares elbow branches and the planar singularity", async ({
  page,
}, info) => {
  await page.goto("/");
  await demo(page, "A").click();
  await page.getByRole("button", { name: "2 · Positive elbow" }).click();
  await expect(page.getByTestId("actual-x")).toHaveText("0.6000");
  await expect(page.getByTestId("actual-y")).toHaveText("0.2000");
  await page.getByRole("button", { name: "3 · Negative elbow" }).click();
  await expect(page.getByTestId("actual-x")).toHaveText("0.6000");
  await expect(page.getByLabel("Elbow branch", { exact: true })).toHaveValue(
    "negative",
  );
  await capture(page, info, "demo-a");
  await page
    .getByRole("button", { name: "4 · Singular configuration" })
    .click();
  await expect(page.getByTestId("jacobian-rank")).toHaveText("1 / 2");
  await expect(page.getByTestId("rank-description")).toContainText(
    "Rank deficient",
  );
  await capture(page, info, "singularity");
});

test("Demo B distinguishes position success from angular pose matching", async ({
  page,
}, info) => {
  await page.goto("/");
  await demo(page, "B").click();
  expect(await metric(page, "demo-position-error")).toBeLessThan(0.1);
  expect(await metric(page, "demo-rotation-error")).toBeGreaterThan(60);
  await expect(page.getByLabel("Joint frames", { exact: true })).toBeChecked();
  await capture(page, info, "demo-b-position");
  await page.getByRole("button", { name: "Compare complete pose" }).click();
  await expect(page.getByRole("status")).toContainText("Converged");
  expect(await metric(page, "demo-rotation-error")).toBeLessThan(0.058);
  expect(await metric(page, "demo-position-error")).toBeLessThan(0.1);
  await capture(page, info, "demo-b");
});

test("Demo C plays and pauses null-space motion with actual pose errors bounded", async ({
  page,
}, info) => {
  await page.goto("/");
  await demo(page, "C").click();
  await expect(page.getByTestId("jacobian-rank")).toHaveText("6 / 6");
  await page.getByRole("button", { name: "Play", exact: true }).click();
  await expect
    .poll(
      async () =>
        parseFloat(
          (await page.getByTestId("joint-excursion").innerText()).split(":")[1],
        ),
      { timeout: 8000 },
    )
    .toBeGreaterThan(0.15);
  await page.getByRole("button", { name: "Pause", exact: true }).click();
  expect(await metric(page, "demo-position-error")).toBeLessThanOrEqual(0.05);
  expect(await metric(page, "demo-rotation-error")).toBeLessThanOrEqual(0.029);
  await capture(page, info, "demo-c");
  await page.getByRole("button", { name: "Reset demo" }).click();
  await expect(page.getByTestId("joint-excursion")).toContainText("0.000 rad");
  await expect(page.getByRole("status")).toContainText("Paused");
});

test("Demo D samples, plays, pauses, changes speed and resets a feasible Cartesian path", async ({
  page,
}, info) => {
  await page.goto("/");
  await demo(page, "D").click();
  await page
    .getByRole("slider", { name: "Path coordinate s", exact: true })
    .fill("0.2");
  await expect(page.getByTestId("actual-x")).toHaveText("0.5100");
  await expect(page.getByTestId("actual-y")).toHaveText("0.3200");
  expect(await metric(page, "demo-position-error")).toBeLessThan(0.001);
  await page
    .getByRole("slider", { name: "Cartesian playback speed" })
    .fill("0.2");
  await page.getByRole("button", { name: "Play", exact: true }).click();
  await expect(
    page.getByRole("combobox", { name: "Path elbow branch" }),
  ).toBeDisabled();
  await expect
    .poll(() => metric(page, "path-coordinate"))
    .toBeGreaterThan(0.24);
  await page.getByRole("button", { name: "Pause", exact: true }).click();
  const paused = await metric(page, "path-coordinate");
  await page.waitForTimeout(100); // Verify pause across several 20 ms playback ticks.
  expect(await metric(page, "path-coordinate")).toBe(paused);
  await capture(page, info, "demo-d");
  await page
    .getByRole("slider", { name: "Path coordinate s", exact: true })
    .fill("0.399");
  await page.getByRole("button", { name: "Play", exact: true }).click();
  await expect(page.getByRole("status")).toContainText("End of feasible path");
  await expect(page.getByTestId("path-coordinate")).toHaveText("0.400 m");
  await expect(
    page.getByRole("button", { name: "Play", exact: true }),
  ).toBeDisabled();
  await page.getByRole("button", { name: "Reset demo" }).click();
  await expect(page.getByTestId("path-coordinate")).toHaveText("0.000 m");
  await page
    .getByRole("combobox", { name: "Path elbow branch" })
    .selectOption("negative");
  await page.getByLabel("Advanced information").check();
  expect(
    Number(
      await page
        .getByRole("spinbutton", { name: "Joint 2 (deg)", exact: true })
        .inputValue(),
    ),
  ).toBeLessThan(0);
});

test("presentation, keyboard switching, fullscreen and mobile controls work", async ({
  page,
}, info) => {
  await page.setViewportSize({ width: 1920, height: 1080 });
  await page.goto("/");
  await demo(page, "D").click();
  await page
    .getByRole("button", { name: "Presentation mode", exact: true })
    .click();
  const before = (await page.locator("canvas").boundingBox())!.height;
  await page
    .getByRole("button", { name: "Presentation mode", exact: true })
    .click();
  await expect(page.locator(".app")).toHaveClass(/presentation/);
  expect((await page.locator("canvas").boundingBox())!.height).toBeGreaterThan(
    before,
  );
  await capture(page, info, "presentation");
  const full = page.getByRole("button", {
    name: "Enter fullscreen",
    exact: true,
  });
  if (await full.isEnabled()) {
    await full.click();
    await expect(
      page.getByRole("button", { name: "Exit fullscreen", exact: true }),
    ).toBeVisible();
    await page
      .getByRole("button", { name: "Exit fullscreen", exact: true })
      .click();
  }
  await page
    .getByRole("button", { name: "Forward Kinematics", exact: true })
    .click();
  const joint = page.getByRole("slider", {
    name: "Joint 1 slider",
    exact: true,
  });
  await expect(joint).toBeVisible();
  await joint.fill("45");
  await expect(
    page.getByRole("spinbutton", {
      name: "Joint 1 (deg)",
      exact: true,
    }),
  ).toHaveValue("45");
  await expect(page.getByTestId("actual-x")).toBeVisible();
  await capture(page, info, "presentation-forward");
  const mode = page.getByRole("button", {
    name: "Kinematic Analysis",
    exact: true,
  });
  await mode.focus();
  await page.keyboard.press("Enter");
  await expect(page.getByTestId("jacobian-rank")).toBeVisible();
  await page.setViewportSize({ width: 390, height: 844 });
  await demo(page, "D").click();
  await expect(
    page.getByRole("slider", { name: "Path coordinate s", exact: true }),
  ).toBeVisible();
  expect(
    await page.evaluate(
      () => document.documentElement.scrollWidth > innerWidth,
    ),
  ).toBe(false);
  await capture(page, info, "mobile");
});

test("all teaching demos remain local and free of runtime errors", async ({
  page,
}, info) => {
  const external: string[] = [],
    errors: string[] = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.route("**/*", async (route) => {
    if (new URL(route.request().url()).origin !== "http://127.0.0.1:4173") {
      external.push(route.request().url());
      await route.abort();
    } else await route.continue();
  });
  await page.goto("/");
  for (const id of ["A", "B", "C", "D"]) {
    await demo(page, id).click();
    await expect(page.locator("canvas")).toBeVisible();
  }
  await page
    .getByRole("button", { name: "Kinematic Analysis", exact: true })
    .click();
  await page.getByRole("button", { name: /Spatial 6R/ }).click();
  await expect(page.getByTestId("jacobian-rank")).toHaveText("3 / 3");
  await capture(page, info, "analysis-spatial");
  await page
    .getByRole("button", { name: "Full-pose task", exact: true })
    .click();
  await expect(page.getByTestId("jacobian-rank")).toHaveText("6 / 6");
  expect(external).toEqual([]);
  expect(errors).toEqual([]);
});

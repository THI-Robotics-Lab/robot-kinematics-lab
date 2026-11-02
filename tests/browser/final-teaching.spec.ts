import { expect, test, type Page } from "@playwright/test";
import { forward } from "../../src/kinematics/fk";
import { getPosition } from "../../src/kinematics/math";
import { panda } from "../../src/kinematics/models";
import { PerspectiveCamera, Vector3 } from "three";

async function anchor(page: Page, joint: number) {
  const line = page.locator(`.joint-label-layer line[data-joint="${joint}"]`);
  return {
    x: Number(await line.getAttribute("x1")),
    y: Number(await line.getAttribute("y1")),
  };
}

test("Top View frames XY with X right and Y up, restores defaults and survives robot/size changes", async ({
  page,
}, info) => {
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.goto("/");
  await page.getByRole("button", { name: /Planar 2R/ }).click();
  await page
    .getByRole("slider", { name: "Joint 1 slider", exact: true })
    .fill("0");
  await page.getByRole("button", { name: "Top View", exact: true }).click();
  await expect(
    page.getByText("Top view · X → · Y ↑", { exact: true }),
  ).toBeVisible();
  await expect
    .poll(async () =>
      Math.abs((await anchor(page, 1)).y - (await anchor(page, 2)).y),
    )
    .toBeLessThan(0.01);
  const origin = await anchor(page, 1),
    q2 = await anchor(page, 2);
  const canvas = (await page.locator("canvas").boundingBox())!;
  expect(q2.x).toBeGreaterThan(origin.x);
  expect(origin.x).toBeCloseTo(canvas.width / 2, 2);
  expect(origin.y).toBeCloseTo(canvas.height / 2, 2);
  const pixelsPerMetre = (q2.x - origin.x) / 0.6;
  expect(pixelsPerMetre).toBeLessThan(
    Math.min(canvas.width, canvas.height) / 2,
  );
  await page
    .getByRole("slider", { name: "Joint 1 slider", exact: true })
    .fill("90");
  await expect
    .poll(async () =>
      Math.abs((await anchor(page, 1)).x - (await anchor(page, 2)).x),
    )
    .toBeLessThan(0.01);
  expect((await anchor(page, 2)).y).toBeLessThan((await anchor(page, 1)).y);
  await page.getByRole("button", { name: /Reset view/ }).click();
  await expect(page.getByText("Default view", { exact: true })).toBeVisible();
  for (const model of ["Spatial 6R", "Franka Panda 7R"]) {
    await page.getByRole("button", { name: new RegExp(model) }).click();
    await expect(
      page.getByRole("button", { name: "Top View", exact: true }),
    ).toHaveCount(0);
    await expect(page.getByText("Default view", { exact: true })).toBeVisible();
  }
  for (const id of ["A", "D"]) {
    await page
      .getByRole("button", { name: new RegExp(`^Demo ${id} ·`) })
      .click();
    await expect(
      page.getByText("Top view · X → · Y ↑", { exact: true }),
    ).toBeVisible();
  }
  await page.setViewportSize({ width: 390, height: 844 });
  await expect
    .poll(async () => Math.abs((await anchor(page, 1)).x - (await page.locator("canvas").boundingBox())!.width / 2))
    .toBeLessThan(0.05);
  expect(
    await page.evaluate(
      () => document.documentElement.scrollWidth > innerWidth,
    ),
  ).toBe(false);
  await page.screenshot({
    path: info.outputPath("top-view-mobile.png"),
    fullPage: true,
  });
  expect(errors).toEqual([]);
});

test("joint labels use FK pivots and highlight focused slider/numeric controls without overlapping", async ({
  page,
}, info) => {
  await page.goto("/");
  const slider = page.getByRole("slider", {
    name: "Joint 2 slider",
    exact: true,
  });
  await slider.focus();
  await slider.press("ArrowRight");
  await expect(page.getByTestId("joint-label-2")).toHaveAttribute(
    "data-active",
    "true",
  );
  await expect(page.getByTestId("joint-label-1")).toHaveAttribute(
    "data-active",
    "false",
  );
  await page.screenshot({
    path: info.outputPath("joint-highlighting.png"),
    fullPage: true,
  });
  const numeric = page.getByRole("spinbutton", {
    name: "Joint 1 (deg)",
    exact: true,
  });
  await numeric.fill("45");
  await expect(page.getByTestId("joint-label-1")).toHaveAttribute(
    "data-active",
    "true",
  );
  await numeric.press("Enter");
  await expect(page.getByTestId("joint-label-1")).toHaveAttribute(
    "data-active",
    "false",
  );
  await page.getByRole("button", { name: /Franka Panda 7R/ }).click();
  const labels = page.locator(".joint-label");
  await expect(labels).toHaveCount(7);
  // Compare the actual label leader anchors against independently projected FK
  // pivots. Coincident Panda origins must stay coincident at the leader end.
  const box = (await page.locator("canvas").boundingBox())!;
  const camera = new PerspectiveCamera(38, box.width / box.height, 0.01, 60);
  camera.up.set(0, 0, 1);
  camera.position.set(1.25, -1.5, 1.35);
  camera.lookAt(0, 0, 0.5);
  camera.updateMatrixWorld();
  const pivots = forward(panda, panda.home).pivots;
  for (let i = 0; i < 7; i++) {
    const projected = new Vector3()
      .fromArray(getPosition(pivots[i]))
      .project(camera);
    await expect
      .poll(async () =>
        Math.abs(
          (await anchor(page, i + 1)).x - ((projected.x + 1) * box.width) / 2,
        ),
      )
      .toBeLessThan(0.1);
    expect(
      Math.abs(
        (await anchor(page, i + 1)).y - ((1 - projected.y) * box.height) / 2,
      ),
    ).toBeLessThan(0.1);
  }
  const rectangles = await labels.evaluateAll((nodes) =>
    nodes.map((node) => {
      const r = node.getBoundingClientRect();
      return { x: r.x, y: r.y, right: r.right, bottom: r.bottom };
    }),
  );
  rectangles.forEach((a, i) =>
    rectangles
      .slice(i + 1)
      .forEach((b) =>
        expect(
          a.x < b.right && a.right > b.x && a.y < b.bottom && a.bottom > b.y,
        ).toBe(false),
      ),
  );
  await expect(page.getByRole("button", { name: "Presentation mode", exact: true })).toHaveAttribute("aria-pressed", "true");
  await page
    .getByRole("spinbutton", {
      name: "Joint 3 (deg)",
      exact: true,
    })
    .fill("10");
  await expect(page.getByTestId("joint-label-3")).toHaveAttribute(
    "data-active",
    "true",
  );
  expect(
    await page
      .getByTestId("joint-label-3")
      .evaluate((node) => parseFloat(getComputedStyle(node).fontSize)),
  ).toBeGreaterThanOrEqual(18);
  await page.screenshot({
    path: info.outputPath("panda-joint-labels.png"),
    fullPage: true,
  });
  await page
    .getByRole("checkbox", { name: "Joint labels", exact: true })
    .uncheck();
  await expect(
    page.getByRole("group", { name: "Viewport joint labels", exact: true }),
  ).toBeHidden();
});

test("Demo D consistently describes Cartesian IK/FK reconstruction", async ({
  page,
}) => {
  await page.goto("/");
  await page.getByRole("button", { name: "Presentation mode", exact: true }).click();
  await page.getByRole("button", { name: /^Demo D ·/ }).click();
  await expect(page.getByText(/^Cartesian reconstruction error/)).toBeVisible();
  await expect(
    page.getByText("FK-computed position", { exact: true }),
  ).toBeVisible();
  await expect(
    page.getByText(/Green: desired path · orange: FK-reconstructed trajectory/),
  ).toBeVisible();
  await expect(
    page.getByText(/no physical motion or tracking controller is simulated/),
  ).toBeVisible();
  await expect(page.locator(".demo-panel")).toContainText(
    "s[k+1] = s[k] + Ts u[k]",
  );
  await expect(page.locator("main")).not.toContainText("Actual position");
  await expect(page.locator("main")).not.toContainText("achieved");
  await expect(page.locator("main")).not.toContainText("Tracking error");
  await page
    .getByRole("slider", { name: "Path coordinate s", exact: true })
    .fill("0.2");
  await expect(page.getByTestId("actual-x")).toHaveText("0.5100");
  await expect(page.getByTestId("actual-y")).toHaveText("0.3200");
});

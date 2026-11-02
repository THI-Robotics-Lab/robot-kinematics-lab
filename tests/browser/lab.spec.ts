import { expect, test } from "@playwright/test";
import { PerspectiveCamera, Vector3 } from "three";
import { forward } from "../../src/kinematics/fk";
import { getPosition, matrixToRPY } from "../../src/kinematics/math";
import { panda } from "../../src/kinematics/models";

test("initial state selects Panda home and its target in presentation mode without fullscreen", async ({
  page,
}) => {
  await page.goto("/");
  await expect(page.getByRole("button", { name: /Franka Panda 7R/ })).toHaveAttribute("aria-pressed", "true");
  await expect(page.getByRole("button", { name: "Presentation mode", exact: true })).toHaveAttribute("aria-pressed", "true");
  await expect(page.locator(".app")).toHaveClass(/presentation/);
  expect(await page.evaluate(() => document.fullscreenElement)).toBeNull();
  await expect(page.getByRole("button", { name: "Enter fullscreen", exact: true })).toBeVisible();
  await expect(page.getByRole("slider", { name: /^Joint \d slider$/ })).toHaveCount(7);
  for (const [i, angle] of panda.home.entries()) {
    const value = await page.getByRole("spinbutton", { name: `Joint ${i + 1} (deg)`, exact: true }).inputValue();
    expect(Number(value)).toBeCloseTo(angle * 180 / Math.PI, 4);
  }
  await page.getByRole("button", { name: "Inverse Kinematics", exact: true }).click();
  await page.getByRole("button", { name: "Full pose", exact: true }).click();
  const pose = forward(panda, panda.home).pose;
  for (const [i, axis] of ["X", "Y", "Z"].entries()) {
    const value = await page.getByRole("spinbutton", { name: `Target ${axis} (m)`, exact: true }).inputValue();
    expect(Number(value)).toBeCloseTo(getPosition(pose)[i], 4);
  }
  for (const [i, axis] of ["Roll", "Pitch", "Yaw"].entries()) {
    const value = await page.getByRole("spinbutton", { name: `Target ${axis} (deg)`, exact: true }).inputValue();
    expect(Number(value)).toBeCloseTo(matrixToRPY(pose)[i] * 180 / Math.PI, 4);
  }
  await page.getByRole("button", { name: "Solve IK" }).click();
  await expect(page.getByRole("status")).toContainText("Converged");
});

test("all three models render and solve, with no external runtime requests", async ({
  page,
}) => {
  const errors: string[] = [],
    external: string[] = [];
  page.on("pageerror", (error) => errors.push(error.message));
  await page.route("**/*", async (route) => {
    if (!route.request().url().startsWith("http://127.0.0.1:4173/")) {
      external.push(route.request().url());
      await route.abort();
    } else await route.continue();
  });
  await page.goto("/");
  await page
    .getByRole("button", { name: "Inverse Kinematics", exact: true })
    .click();
  await page.getByLabel("Advanced information").check();
  await expect(page).toHaveTitle("THI Robot Kinematics Lab");
  for (const model of ["Planar 2R", "Spatial 6R", "Franka Panda 7R"]) {
    await page.getByRole("button", { name: new RegExp(model) }).click();
    await expect(page.locator("canvas")).toBeVisible();
    await expect(page.getByRole("alert")).toHaveCount(0);
    await page.getByRole("button", { name: "Solve IK" }).click();
    await expect(page.getByRole("status")).toContainText("Converged");
    if (model !== "Planar 2R") {
      await page
        .getByRole("button", { name: "Full pose", exact: true })
        .click();
      const x = page.getByRole("spinbutton", {
        name: "Target X (m)",
        exact: true,
      });
      await x.fill(String(Number(await x.inputValue()) + 0.03));
      await page.getByRole("button", { name: "Solve IK" }).click();
      await expect(page.getByRole("status")).toContainText("Converged");
      expect(
        parseFloat(await page.getByTestId("position-error").innerText()),
      ).toBeLessThanOrEqual(0.1);
    }
  }
  expect(errors).toEqual([]);
  expect(external).toEqual([]);
});

test("2R controls expose branch solutions, live FK, limits, singularities and unreachable targets", async ({
  page,
}) => {
  await page.goto("/");
  await page.getByRole("button", { name: /Planar 2R/ }).click();
  await page
    .getByRole("button", { name: "Inverse Kinematics", exact: true })
    .click();
  await page.getByLabel("Advanced information").check();
  await page
    .getByRole("spinbutton", { name: "Joint 1 (deg)", exact: true })
    .fill("0");
  await page
    .getByRole("spinbutton", { name: "Joint 2 (deg)", exact: true })
    .fill("0");
  await expect(page.getByTestId("actual-x")).toHaveText("1.0000");
  await expect(page.getByTestId("actual-y")).toHaveText("0.0000");
  await expect(page.locator(".singularity-note")).toContainText("Singular:");
  await page
    .getByRole("spinbutton", { name: "Target X (m)", exact: true })
    .fill("0.6");
  await page
    .getByRole("spinbutton", { name: "Target Y (m)", exact: true })
    .fill("0.2");
  await page.getByLabel("Elbow branch").selectOption("negative");
  await page.getByRole("button", { name: "Solve IK" }).click();
  await expect(page.getByRole("status")).toContainText("Converged");
  expect(
    Number(
      await page
        .getByRole("spinbutton", { name: "Joint 2 (deg)", exact: true })
        .inputValue(),
    ),
  ).toBeLessThan(0);
  await page
    .getByRole("spinbutton", { name: "Target X (m)", exact: true })
    .fill("2");
  await expect(page.getByRole("status")).toContainText("Ready to solve");
  await page.getByRole("button", { name: "Solve IK" }).click();
  await expect(page.getByRole("status")).toContainText("Unreachable target");
  await page
    .getByRole("spinbutton", { name: "Joint 1 (deg)", exact: true })
    .fill("999");
  await page
    .getByRole("spinbutton", { name: "Joint 1 (deg)", exact: true })
    .press("Enter");
  expect(
    Number(
      await page
        .getByRole("spinbutton", { name: "Joint 1 (deg)", exact: true })
        .inputValue(),
    ),
  ).toBe(180);
});

test("numeric controls accept signed decimal keyboard input", async ({
  page,
}) => {
  await page.goto("/");
  await page
    .getByRole("button", { name: "Inverse Kinematics", exact: true })
    .click();
  await page.getByLabel("Advanced information").check();
  const joint = page.getByRole("spinbutton", {
    name: "Joint 1 (deg)",
    exact: true,
  });
  await joint.fill("");
  await joint.pressSequentially("-12.5");
  await joint.press("Enter");
  await expect(joint).toHaveValue("-12.5");
  const x = page.getByRole("spinbutton", { name: "Target X (m)", exact: true });
  await x.fill("");
  await x.pressSequentially("0.125");
  await x.press("Enter");
  await expect(x).toHaveValue("0.125");
});

test("target gizmo can be dragged, and release runs IK", async ({ page }) => {
  await page.goto("/");
  await page.getByRole("button", { name: /Planar 2R/ }).click();
  await page
    .getByRole("button", { name: "Inverse Kinematics", exact: true })
    .click();
  await page.getByLabel("Advanced information").check();
  await page.getByRole("button", { name: "Solve IK" }).click();
  const canvas = page.locator("canvas"),
    bounds = (await canvas.boundingBox())!;
  const xInput = page.getByRole("spinbutton", {
    name: "Target X (m)",
    exact: true,
  });
  const beforeX = await xInput.inputValue();
  const yInput = page.getByRole("spinbutton", {
    name: "Target Y (m)",
    exact: true,
  });
  const beforeY = await yInput.inputValue();
  // Project a point on the X handle using the default teaching camera.
  // TransformControls handles are Three objects, not DOM elements.
  const camera = new PerspectiveCamera(
    38,
    bounds.width / bounds.height,
    0.01,
    60,
  );
  camera.up.set(0, 0, 1);
  camera.position.set(0.65, -1.25, 2.7);
  camera.lookAt(0.25, 0.15, 0);
  camera.updateMatrixWorld();
  const point = new Vector3(Number(beforeX) + 0.22, Number(beforeY), 0).project(
    camera,
  );
  const x = bounds.x + ((point.x + 1) * bounds.width) / 2;
  const y = bounds.y + ((1 - point.y) * bounds.height) / 2;
  await page.mouse.move(x, y);
  await page.mouse.down();
  await page.mouse.move(x - 35, y + 5, { steps: 10 });
  await page.mouse.up();
  expect(
    (await xInput.inputValue()) !== beforeX ||
      (await yInput.inputValue()) !== beforeY,
  ).toBe(true);
  await expect(page.getByRole("status")).not.toContainText("Ready to solve");
});

test("small screens keep the numeric controls accessible without horizontal overflow", async ({
  page,
}) => {
  await page.setViewportSize({ width: 390, height: 844 });
  await page.goto("/");
  await page
    .getByRole("button", { name: "Inverse Kinematics", exact: true })
    .click();
  await page.getByLabel("Advanced information").check();
  await expect(page.getByRole("button", { name: "Solve IK" })).toBeVisible();
  const overflows = await page.evaluate(
    () => document.documentElement.scrollWidth > window.innerWidth,
  );
  expect(overflows).toBe(false);
});

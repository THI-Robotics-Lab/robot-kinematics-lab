import { expect, test } from "@playwright/test";

test("schematic flange gripper follows wrist rotation without changing flange position", async ({ page }, info) => {
  await page.setViewportSize({ width: 1366, height: 768 });
  await page.goto("/");
  for (const [model, joint] of [["Spatial 6R", 6], ["Franka Panda 7R", 7]] as const) {
    await page.getByRole("button", { name: new RegExp(model) }).click();
    const canvas = page.locator("canvas");
    const xyz = async () => Promise.all(["x", "y", "z"].map((axis) => page.getByTestId(`actual-${axis}`).innerText()));
    const beforePosition = await xyz();
    const before = await canvas.screenshot();
    const input = page.getByRole("spinbutton", { name: `Joint ${joint} (deg)`, exact: true });
    await input.fill("90");
    await expect(page.getByTestId(`joint-label-${joint}`)).toHaveAttribute("data-active", "true");
    await expect.poll(xyz).toEqual(beforePosition);
    const after = await canvas.screenshot();
    expect(before.equals(after)).toBe(false);
    await page.screenshot({ path: info.outputPath(`${joint}r-flange-gripper.png`) });
  }
});

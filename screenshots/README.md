# Teaching screenshots

Historical captures from the production build in Chromium on 2026-10-30; the current header, joint controls and gripper have since changed. These are review
artifacts, not visual regression baselines. Animated captures show one sampled
state; all displayed errors come from the actual robot configuration.

| Teaching view             | Capture                                                                              |
| ------------------------- | ------------------------------------------------------------------------------------ |
| Forward Kinematics        | [Joint controls and actual flange pose](iteration-2/forward.png)                     |
| Inverse Kinematics        | [Cartesian target and solver controls](iteration-2/inverse.png)                      |
| Kinematic Analysis        | [Planar velocity mapping and singular values](iteration-2/analysis.png)              |
| Spatial position analysis | [6R velocity ellipsoid](iteration-2/analysis-spatial.png)                            |
| Demo A                    | [Top-down elbow configuration at the shared target](iteration-2/demo-a.png)          |
| Demo A singularity        | [Rank loss and collapsed velocity ellipse](iteration-2/singularity.png)              |
| Demo B position task      | [Zero position error with free angular difference](iteration-2/demo-b-position.png)  |
| Demo B full pose          | [Position and orientation matched](iteration-2/demo-b.png)                           |
| Demo C                    | [Panda joint motion and measured full-pose errors](iteration-2/demo-c.png)           |
| Demo D                    | [Top-down Cartesian line and IK/FK reconstruction](iteration-2/demo-d.png)           |
| Presentation mode         | [Cartesian path on a projection layout](iteration-2/presentation.png)                |
| Presentation FK           | [Compact joint controls beside the robot](iteration-2/presentation-forward.png)      |
| Joint highlighting        | [Selected planar joint and label](iteration-2/joint-highlighting.png)                |
| Panda labels              | [Separated FK joint labels in Presentation Mode](iteration-2/panda-joint-labels.png) |
| Mobile top view           | [Complete planar workspace on a narrow viewport](iteration-2/top-view-mobile.png)    |
| Mobile presentation       | [390 px layout](iteration-2/mobile.png)                                              |

Normal desktop viewport: 1440×1100. Projection viewport: 1920×1080. Mobile
viewport: 390×844. Captures include the full page where content extends below
the viewport. Run `npm run build && npm run test:browser` to regenerate current
captures under ignored `test-results/`, then copy selected images here only
when updating the reviewed artifacts. They are not copied into `dist/`.

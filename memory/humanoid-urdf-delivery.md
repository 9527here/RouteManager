# 人形 URDF 对外交付包

发出去的包放在 `assets/deliver/`（和 `RM65-6FB-V_ctag2f120` 并列），
一键重建：`python3 scripts/build_deliver_humanoid.py`。

## 约定（2026-10-15 定）

- **源用 `humanoid_upper_body_v3_ctag2f120_rm65_isaac.urdf`** —— 就是
  `config.json` 里 `robot.presets.v3_ctag2f120_rm65.urdf_path` 指的那份，即仿真
  真在跑的那份。它比 `..._rm65.urdf` 新：腕壳已切（arm_6 直装夹爪，没有
  `*_wrist_link`）、多了 4 路相机 link。
- **包结构**：`package.xml` + `CMakeLists.txt` + `urdf/` + `meshes/` + `launch/` +
  `config/`。**不放 README**（用户明确要求）。
- **两份 URDF**：`<pkg>.urdf` 走 `package://<pkg>/meshes/...`；
  `<pkg>_relative.urdf` 走包内相对路径，给不认 `package://` 的工具（Isaac 的
  URDF 导入器就不认 `package://`，见 CLAUDE.md 踩坑清单）。
- **网格分治**：`meshes/collision/` **原样拷贝**（碰撞体决定物理，不动）；
  `meshes/visual/` 用 open3d 二次误差精简。只拷 URDF 真正引用到的网格
  （visual 目录里 57 个文件，只有 37 个被引用，ZJ/YJ/*_wrist 是历史遗留）。

## 精简预算

`target = min(n, clamp(n // 25, 25_000, 80_000))`。实测 20k 会把机身电机的
散热筋磨平，80k 能保住。整机 202MB → 21.8MB，包总计 25MB，zip 8MB。

**open3d 坑**：`simplify_quadric_decimation` 会丢面法线，之后
`write_triangle_mesh` 直接返回 False（只打一行 WARNING "compute normals first"），
必须先 `compute_triangle_normals()`。

## 校验

- `/opt/ros/humble/bin/check_urdf` 两份都过，61 link / 60 joint，0 warning。
- 另有一步整机装配渲染（把 61 个 visual mesh 按 URDF 链式变换摆好再渲），
  用来同时验"包内路径对不对"和"位姿变换对不对"。

相关 [[rm65-grasp-failure-root-cause]]、[[arm-model-vs-official-rm65]]。

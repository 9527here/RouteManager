# 迁移到 Isaac Sim 6.2（2026-10-21）

RouteManager 默认环境从 `isaacsim5.1` 换到 `isaacsim6.0`（目录名有误导性，
该 env 已于 2026-10-20 就地升级到 **6.2.0.0**，python3.12）。

## 5.1 为什么起不来了

`2026-10-20 12:29` 那次升级把 5.1 的 Kit 扩展缓存覆盖成了 py312 版：

```
isaacsim5.1/.../kit/data/Kit/Isaac-Sim Python/5.1/exts/3/
  omni.isaac.ml_archive-3.0.4+107.3.3.lx64.cp311/pip_prebundle/torch/
    _C.cpython-312-x86_64-linux-gnu.so      ← py312，可 env 是 py311
```

症状链：`import torch` 失败 → `isaacsim.core.*` 全部加载失败 → 服务启动 12 秒即退。
**shell 里单独 import torch 是好的**（走 site-packages），坏的只有 Kit 那份 prebundle。
要用 5.1 得先把 prebundle 换回 py311。

## URDF 导入器：两套 API

`robot_manager._import_urdf_to_usd` 按版本自动分派：

| | 5.1 (`_import_urdf_51`) | 6.2 (`_import_urdf_62`) |
|---|---|---|
| 入口 | `_urdf` | `URDFImporter` + `URDFImporterConfig` |
| 流程 | `parse_urdf` + `import_robot` | `URDFImporter().import_urdf(cfg)` |
| 产物 | 单个 `.usd` | 分层目录 `<dir>/<stem>/<stem>.usda` + `payloads/` |
| defaultPrim | 无，要补 | 有 |
| Physics 变体 | 无 | 有，**必须选 `physx`**，否则关节属性不合成 |
| `usd_path` | 文件路径 | **目录**，且不覆盖（每次建 `<stem>_N`，每个 53 MB） |

## 三个真坑（都是实测出来的）

### 1. mimic 必须剥掉，否则夹爪把仿真带崩

6.2 给 mimic 关节同时挂 `NewtonMimicAPI` **和**自己的 drive（5.1 的
`PhysxMimicJointAPI` 没有 drive）。本项目的软件 mimic 每帧往跟随关节写位置目标，
和原生约束互相顶 —— 实测夹爪跑到 **63 rad**，PhysX 刷 `Invalid PhysX transform`，
`Illegal BroadPhaseUpdateData - non-finite bounds`，整个仿真发散成 NaN。

5.1 用 `parse_mimic=False` 关；6.2 没这个字段，导入后遍历 `RemoveAPI`。
剥完 22 处，夹爪立刻正常（`-1` 的 −0.700、`-0.15` 的 −0.105，其余 +0.700）。

### 2. `GroundPlane.set_world_pose()` 在 6.2 上不生效

设 z=6.222，读回仍是 0 → 机器人直接掉到地面。
改成直接写 `xformOp:translate`（`_set_prim_translate_z`）。

### 3. 关节限位在 USD 里存的是「度」

`physics:lowerLimit = 40.107` 对应 0.7 rad —— 这是 UsdPhysics 的约定，
Isaac 建 articulation 时会转。**运行时读到的 `get_dof_limits()` 仍是弧度**，
别拿 USD 层的数直接当弧度用（我一度误判成"限位丢了"）。

## 同一会话不能二次加载机器人

6.2 抛 `Cannot add the object humanoid ... name is not unique`；
5.1 更糟，URDF 导入器在 `UsdGeomXformable::AddXformOp` 里 abort 把进程带死。
`_reject_if_robot_loaded` 提前拦成一句人话，提示重启。

## 实测结果（6.2）

- 服务 14 s 就绪；`load_scene` 24 s；`load_robot`（URDF 全流程导入）10 s
- 关节跟踪：目标 +0.200 → 实测 +0.202
- 夹爪 12 关节全部按 mimic 倍率到位，`max|q| = 0.7`
- 相机 `head_zed_x` / `tv_cam` 出图正常
- 世界高度 `base_z=6.222` 生效：地面/桌子/机器人同平面，静止 `max|vel|=0`

相关 [[routemanager-project]]、[[vr-teleop-xrobotoolkit-bridge]]。
# 场景摩擦系数实测 ≈0（抓取失败的真正根因）

2026-10-09。**整个场景原本没有任何 PhysicsMaterial 绑定**，PhysX 用默认值，
而在这个场景里实测摩擦系数只有 **≈0.05**（正常应该 0.5）。后果：

- 夹爪指垫压住物体也**夹不住**——一抬就滑脱（探针显示指垫已接触、驱动力矩
  已到上限、leader 差 0.014 rad，但物体纹丝不动）。
- 机器人**轮子在地面打滑**：机械臂的反作用力把底座推走 **0.2m**
  （`base_link` x 从 0 变 −0.198），目标跑出手臂可达范围 → IK 解不出朝下姿态
  （朝下误差 30°，`arm_2` 顶在 −116° 下限）。

## 修法

1. **场景物体**（桌子/方块/圆柱）：在 `scripts/scene/static_scene.usda` 里加
   `PhysicsMaterial "friction_mat"`（static/dynamic friction 1.0, restitution 0），
   并给 `vla_table` / `cube_a` / `cyl_b` / `cube_c` 加
   `rel material:binding = </World/friction_mat>`。
   （`gen_static_usda.py` 重新生成时也要带上，否则会被覆盖掉。）
2. **地面**（运行时由 `robot_manager._create_ground_plane` 建，不在 usda 里）：
   `robot_manager._ensure_friction_material()` 建材质并绑到 `/World/ground_plane*`。
3. **夹爪指垫**在机器人资产里（instance 原型，运行时绑不了）。PhysX 按两侧
   材质取平均，物体/地面有 1.0 就够，所以**不需要**改资产。

修好后 `pick_object.py` 连跑 **4/4** 成功，且**过盈只要 0.5mm**（之前没有摩擦
时要靠 3mm 硬挤，导致指垫明显穿进物体里）。

## 怎么量摩擦

给物体一个水平初速度（`RigidPrim.set_linear_velocities`），看它多久停下：
减速度 = μg。实测 0.30 m/s 在 ~0.5s 内停住 → μ≈0.06。

相关 [[gripper-mimic-chain]]、[[auto-pick-menu-key]]。

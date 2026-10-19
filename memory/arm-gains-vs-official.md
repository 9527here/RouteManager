# 臂的质量是官方的、阻尼/刚度不是（2026-10-14 查证）

用户问："机械臂的质量和阻尼等参数是跟官方保持一致的吗？为什么抓取之后手放平
的时候会有弹动的现象？我如何在 isaacsim 里面确认"

## 答案

**质量 / 惯量：是官方的。** 来自 URDF 的 `<inertial>`（交付包原值）：

```
left_arm_1_link 1.510 kg   left_arm_4_link 0.671 kg
left_arm_2_link 1.653 kg   left_arm_5_link 0.647 kg
left_arm_3_link 0.726 kg   left_arm_6_link 0.144 kg
```

**阻尼 / 刚度：不是官方的。** 官方 RM65-6FB-V 的 URDF **六个臂关节一个
`<dynamics>` 都没有**（`grep` 实测：全是 `damping=None`）。不烘增益则
PhysX 里 damping=0，臂会一直摆。现在 `*.gains.json` 里的值：

```
joint_1  1309.2 / 9.5603      joint_4  502.8 / 5.9247
joint_2  1139.1 / 8.91763     joint_5  244.4 / 4.13066
joint_3  610.5 / 6.52847      joint_6  193.1 / 3.67164
```

（`assets/RM65-6FB-V/...gains.json` 的 `_comment` 写明这 6 个值是**抄自人形
那份**，不是从官方规格来的。）

## 关于"弹动"

按 `ζ = d/(2√k)` 算，六个关节的 ζ **都恰好是 0.132**（看着像欠阻尼 66% 超调）。
但**实测阶跃响应只有 2.3% 超调、0.13s 稳定** —— 说明 acceleration 型驱动的
`get_dof_stiffnesses()` 返回值不能直接套这个公式，**别按这个去改阻尼**。

真正让手"弹"的更可能是：
- 夹爪驱动刚度（2026-10-13 曾提到 100000 → 压在方块上一直弹，已降到 40000）；
- 或者大行程快速运动末尾的整定，用阶跃响应单独测才准。

## 在 Isaac Sim 里怎么确认

1. **Physics Inspector**：`Window → Physics → Physics Inspector`，在 Stage 里
   选中关节 prim（如 `/humanoid/base_link/joints/left_arm_3_joint`），
   Drive 一栏直接显示 Stiffness / Damping / Max Force / Target Position。
   （注意：`_tune_gripper_drives` / `_tune_wheel_drives` / `_tune_neck_drives`
   是运行时写进物理视图的，Inspector 读 USD 属性 —— **USD 上可能还是旧值**，
   要对得上就用下面的物理视图读法。）

2. **Script Editor**（`Window → Script Editor`）或本项目的 `/python` 端点：

```python
import numpy as np
ex = executor                    # Script Editor 里用 standalone_server.EXECUTOR
names = list(ex.articulation.dof_names)
v = ex.articulation._physics_view
k = np.asarray(v.get_dof_stiffnesses()).reshape(-1)   # ≈ USD 值 × 57.29578
d = np.asarray(v.get_dof_dampings()).reshape(-1)
f = np.asarray(v.get_dof_max_forces()).reshape(-1)
for n in ("left_arm_3_joint", "left_gripper_right_2_joint", "front_left_wheel_joint"):
    i = names.index(n)
    print(f"{n:28s} k={k[i]:12.1f}  d={d[i]:10.2f}  maxF={f[i]:7.2f}")
```

3. **质量**：`UsdPhysics.MassAPI` 读 prim 的 `physics:mass`，或对
   `link.GetAttribute("physics:mass")`；也可以直接看 URDF 的 `<inertial>`。

相关 [[rm65-grasp-failure-root-cause]]、[[grasp-verticality-and-body-lowering]]。

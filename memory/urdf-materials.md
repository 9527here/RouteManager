# URDF 颜色一直没生效 —— 被 mesh 级 DefaultMaterial 盖住

**2026-10-12 定位并修复。**

## 症状

不管 URDF 里写了什么 `<material><color rgba=...>`，Isaac 里整机一律渲染成
默认白灰。夹爪的淡蓝、指垫深灰全看不见。

## 根因（不是"导入器没读颜色"）

导入器**读进去了**：每个 link 上都挂了 `Looks/material_<颜色hex>`：

| 材质名 | 颜色 | hex 对照 |
|---|---|---|
| `material_100100100` | (1, 1, 1) | 白 |
| `material_C0C0C0` | (0.75294, ...) | 192,192,192 |
| `material_CAD1EE` | (0.79216, 0.81961, 0.93333) | 202,209,238 淡蓝 |
| `material_626262` | (0.38431, ...) | 98,98,98 深灰 |

但导入器**同时**给每个 mesh 塞了一个更近的 `.../node_STL_BINARY_/Looks/DefaultMaterial`
绑定（那个 DefaultMaterial 是**空壳**，没有 Shader 子节点 ⇒ 渲染成渲染器默认浅灰）。
USD 材质绑定**就近优先** ⇒ mesh 级永远盖过 link 级，URDF 颜色一次都没生效。

## 修法（`robot_manager._fix_visual_materials`）

mesh 藏在 `visuals`（instanceable）的**实例原型**里，而 USD 明确禁止写原型：

```
Cannot create property spec at path </__Prototype_45/.../mesh>;
authoring to an instancing prototype is not allowed.
```

所以：先 `prim.SetInstanceable(False)` 把 `visuals` 展开（在根层写 over，不动资产文件），
再摘掉 mesh 级那个 `DefaultMaterial` 绑定，link 级材质就露出来了。

**坑**：判断"这个 visuals 有没有 link 级材质"**必须从 `prim.GetPrototype()` 读**。
实例开启时 `GetAllChildren()` 拿到的是实例代理，代理上的关系读不出来 —— 读到的
永远是空，会误判成"没有材质"而整段跳过（第一次就踩了这个，日志一条都没有）。
原型只读不写是允许的，写才报错。

调用点：`load_urdf`（建 Articulation **之前**，之后动 stage 会让物理视图失效）
和 `_load_robot_usd`。

## 官方素材里的颜色（来源：厂家 URDF 的 `<material>`）

**RM65-6FB-V**（`assets/RM65-6FB-V/urdf/RM65-6FB-V.urdf`）

| link | rgba |
|---|---|
| base_link / link_1..5 | `1 1 1 1` 白 |
| link_6 / camera_rolink | `0.75294 0.75294 0.75294 1` 灰 |
| camera_link | `0.79216 0.81961 0.93333 1` 淡蓝 |

**CT ctag2f120**（`changingtek/crt_ctag2f120_gripper_visualization/urdf/crt_ctag2f120.urdf`）

| link | rgba |
|---|---|
| base_link / `*_Main_Link` / `R1_Link` / `L1_Link` | `1 1 1 1` 白 |
| `R2/R3/L2/L3_Link` `*_in_Link` `*_Slip_Link` | `0.792 0.820 0.933 1` 淡蓝 |
| `Right_Pad_Link` / `Left_Pad_Link` | `0.384 0.384 0.384 1` 深灰 |

和厂家 RViz 截图（`docs/asserts/003.png`）一致 —— 白本体 + 淡蓝指臂 + 深灰指垫。

## 怎么验证材质真的绑上了

把某个材质的 `inputs:diffuse_color_constant` 临时刷成红色再抓图（`material_626262`
→ 指垫变红即证明绑定链路通）。**别只看"有没有变颜色"**：URDF 那套颜色本身就是
白 + 极淡蓝 + 灰，和 DefaultMaterial 的浅灰肉眼几乎分不出。

## 人形上不受影响

`humanoid_upper_body_v3_ctag2f120_cameras_isaac.usd` 结构里没有 `visuals` 命名的
prim，`_fix_visual_materials` 找不到目标直接跳过；那份资产自带
`material_aluminium_grey` / `material_camera_dark` 两个材质。

相关 [[gripper-mimic-chain]]、[[arm-model-vs-official-rm65]]。

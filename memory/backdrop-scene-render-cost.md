# 背景场景（NuRec 扫描）的渲染开销与降帧对策

2026-10-19 排查"这么卡是不是后台有东西占用"。

## 结论：不是后台，是 NuRec 自己吃满 GPU

后台干净：无残留 Isaac / meshcat / teleop 孤儿进程；Edge GPU 进程瞬时 31.8%
是噪声（稳定后 0.2%）、VS Code / ToDesk / QQ 各百分之几。**GPU 91~100% 全部
来自 Isaac 自己**（唯一 GPU 进程，6.4~10 GB / 24 GB）。

## 实测帧率

| 状态 | FPS |
|---|---|
| 电力院（NuRec）可见，稳态 | **15~17** |
| 电力院隐藏 | **42~45** |
| 泉州3（NuRec）可见 | 6（同会话对照，当时电力院 8） |
| **场景刚加载完 <1 分钟** | **8.6** ← 加速结构还在建 |

⚠️ **最后一行是坑**：加载后头一分钟只有稳态的一半。不知道这点就会得出
"关了某个设置反而更快"之类的假结论（我第一轮就被骗过）。

## 试过、全部无效的杠杆

| 杠杆 | 结果 |
|---|---|
| `render_interval` 2 / 3 / 5 / 8 | 无影响 |
| 渲染分辨率 1280×720 → 960×540 → 640×360 | **无影响，甚至略降** |
| `/rtx/directLighting/sampledLighting/samplesPerPixel` 8→1 | 无影响 |
| 关 `/rtx/reflections/enabled` + `/rtx/translucency/enabled` | 边际（噪声内） |
| NuRec `omni:nurec:crop:minBounds/maxBounds` | **运行时改不了** |

crop 改不了的原因：NuRec 插件在加载后把带 `omni:nurec:*` 的 `Volume` prim
**换成了自己的代理 prim**（运行时读 `/World/dianliyuan_scene/gauss` 只剩
`Xform` + `proxyPrim/purpose/visibility`，nurec 属性全没了），所以
`CreateAttribute` 出来的 override 它根本不看。

瓶颈在**高斯体光栅化本身**（GPU 算力），常规画质旋钮碰不到。

## 唯一有效的对策：`set_scene_visible`

`robot_manager.set_scene_visible(visible, prim_path=None)` —— 开关背景场景的
`UsdGeom.Imageable` 可见性。候选 prim 走 `config.json` 的
`background_scene.prims`（**独立顶层键**，叫 `scene` 的话会被
`_merge_default_params` 并进 `load_scene` 的参数，多出来的键直接报
`unexpected keyword argument` —— 踩过）。

实测 17.14 → **41.72** → 17.15 FPS，可重复。

场景只当背景、不参与物理也不参与操作，所以遥操作 / 数据采集时关掉无损失，
要出图时再开。

## 还想更快的话

- 把 NuRec 扫描换成网格 / 天空盒背景（要重建资产）；
- 离线把 `model.nurec` 降采样（没试过，不知道工具链）。

相关 [[vr-teleop-xrobotoolkit-bridge]]、[[routemanager-project]]。
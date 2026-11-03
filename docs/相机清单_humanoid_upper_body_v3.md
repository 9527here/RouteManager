# 人形机器人相机清单 — humanoid_upper_body_v3

机载相机共 **7 路**（不含第三人称调试相机 `tv_cam`）。

机器人构型：双 RM65 6-DoF 臂 + 升降柱 + 俯仰躯干 + 2-DoF 头 + 4 轮式底盘。

## 一览表

| 相机 | 分辨率 | 挂在 | 静止世界坐标 (m) | 朝向 xyzw | 焦距 (mm) | 裁剪范围 (m) |
|---|---|---|---|---|---|---|
| `head_zed_x` | 960 × 600 | —（静态本地位姿） | 本地 `(-0.115, -0.009, 1.791)` | `0.5332, 0.5650, -0.4595, -0.4306` | 12.49 | 0.10 – 8.0 |
| `head_surround_left` | 1920 × 1200 | `head_pitch_link` | `(+0.1814, +0.0120, +1.6989)` | `0.4451, 0.4454, 0.5492, 0.5494` | 7.34 | 0.30 – 15.0 |
| `head_surround_right` | 1920 × 1200 | `head_pitch_link` | `(+0.1814, +0.0120, +1.6989)` | `0.5492, 0.5494, -0.4451, -0.4454` | 7.34 | 0.30 – 15.0 |
| `head_surround_rear` | 1920 × 1200 | `head_pitch_link` | `(+0.1814, +0.0120, +1.6989)` | `-0.0736, -0.0736, 0.7033, 0.7033` | 7.34 | 0.30 – 15.0 |
| `chest_gemini_335lg` | 1280 × 800 | `torso_pitch_link` | `(+0.1852, -0.0027, +1.3337)` | `0.6296, 0.4552, -0.3687, -0.5104` | 10.48 | 0.15 – 10.0 |
| `left_wrist_gemini_335lg` | 1280 × 800 | `left_camera_link` | `(+0.0400, +0.8200, +1.6200)` | `0.7028, 0.6284, 0.3243, 0.0784` | 10.48 | 0.15 – 10.0 |
| `right_wrist_gemini_335lg` | 1280 × 800 | `right_camera_link` | `(+0.0570, -0.8500, +1.6200)` | `0.6658, 0.7057, 0.0441, -0.2383` | 10.48 | 0.15 – 10.0 |

光圈（像面尺寸）全部为 **20.955 × 13.097 mm**。

- **静止世界坐标** = 机器人处于初始位姿时的相机世界位置（单位米）。
- **朝向** 为四元数，顺序 `(x, y, z, w)`。

## 相对父连杆的位置

**全部为 `[0, 0, 0]`** —— 相机与父连杆**同点**，只差朝向：

- `head_surround_left / right / rear` 三路**共点**于 `head_pitch_link` 原点，靠朝向区分左 / 右 / 后。
- `chest_gemini_335lg` 在 `torso_pitch_link` 原点。
- 两路腕部相机分别在 `left_camera_link` / `right_camera_link` 原点。

所以「相机在哪」等价于「它的父连杆在哪」。跟随关系由 `scripts/scene/cam_follow_offsets.json` 标定，运行时每帧写 `相机世界位姿 = 父连杆位姿 × offset`。

## 腕部相机的连杆链

左右同构，均挂在对应臂第 6 轴末端：

```
arm_6_link
  └─ <side>_camera_rojoint   (revolute)  xyz = [-0.033, -0.0085, -0.05388]  rpy = [1.5708, -1.5708, 3.14]
     └─ <side>_camera_rolink
        └─ <side>_camera_joint (fixed)    xyz = [0.029031, -0.10022, -0.0005]  rpy = [0, 1.5708, 0]
           └─ <side>_camera_link
```

质量：`*_camera_rolink` 0.019695 kg、`*_camera_link` 0.03915 kg。

## 常见画幅

| 相机 | 分辨率 | 宽高比 |
|---|---|---|
| `head_zed_x` | 960 × 600 | 1.60 |
| `head_surround_*` | 1920 × 1200 | 1.60 |
| `chest_gemini_335lg` | 1280 × 800 | 1.60 |
| `*_wrist_gemini_335lg` | 1280 × 800 | 1.60 |

## 可读图的相机

框架里只有 `camera.initialize()` 过的相机能取到图像（建了 render product）。当前初始化的是：

- `head_zed_x`
- `left_wrist_gemini_335lg`

其余各路位姿与内参都有效，但需要额外初始化才能出图。

## 定义位置

| 内容 | 所在 |
|---|---|
| 相机 prim 与内参 | `scripts/scene/static_scene.usda` 的 `/World/cams/*` |
| 挂载关系与跟随偏移 | `scripts/scene/cam_follow_offsets.json` |
| 分辨率 / 初始化名单 | `robot_manager.py` 的 `_STATIC_CAMERAS` / `_STATIC_RENDER_CAMERAS` |
| 腕部相机连杆 | `humanoid_upper_body_v3_ctag2f120_rm65_isaac.urdf` |

> `/World/cams` 属于**静态场景**，只在 `load_scene` 时加载。只调 `load_robot` 时场景里不会有这些相机 prim。
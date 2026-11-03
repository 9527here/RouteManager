# Oxford 电气操作机构（Isaac Sim 6.2）

已迁入三个可动机构：`e_stop_button`、`isolator_switch`、`safety_switch`。
源： https://github.com/ori-arg/manipulation_playground ，固定版本
`3a5b26c785709ffb88048c917d6de84d234bce72`，BSD-3-Clause，见 LICENSE。

## 能力边界

- 4 个活动关节、9 组碰撞，独立 PhysX 场景中三类机构各 3 轮通过。
- 两个开关用临时位置驱动检验往返，撤去测试驱动后恢复上游 normal 增益并检验保持。
- 急停用关节加载夹具验证 5 mm 按压锁定、逆时针旋转解锁、回位。
- 开关的直接力矩加载未通过，尚待排查；**没有进行人形接触验收**，不代表人形能拧动。
- 上游尺寸、质量、惯量和反力未按用户实物标定；无额外铭牌，无机器人相机。
- 急停不是目标文档的“先转外圈再按中心”；开关尚未指定远方/就地、分合闸语义。

## 文件

- `upstream/`：22 个原件，SHA256 与固定源版本对应；不改原件。
- `urdf/`：展开、去 Gazebo 插件和 transmission、重写相对 mesh 引用后的 URDF。
- `usd/<设备>/<设备>/<设备>.usda`：官方 6.2 导入器的分层产物，默认 Physics=physx。
- `gallery.usda`：米制 Z-up 展示场景，相对引用，可整体搬移。
- `catalog.json`、`preparation.json`、`provenance.json`、`validation_summary.json`：入口、改动、来源、验收。
- 状态机在项目 `scripts/oxford_device_runtime.py`，必须由仿真主线程每步调用。

## 在项目使用

保留已有机器人会话；下面 setup 仅允许在**空白 RouteManager 服务**执行：

```bash
python3 scripts/oxford_devices_client.py setup
python3 scripts/oxford_devices_client.py status
```

若手动先将 gallery 引用到 `/World`，待全部资产和机器人加载完成后：

```bash
python3 scripts/oxford_devices_client.py attach
```

`attach_runtime(world, stage, prefix)` 仅包装已有 articulation 并注册状态回调，
不会建立相机或改变机器人关节指令。不要在机器人已经运行时再加这些 prim。
重置/重建物理场景后需重新包装 view，不能继续使用旧 view。当前 18765 服务保留原人形场景。

## 重建与独立验收

系统 Python 负责 Xacro，Isaac Python 负责转换；不把 ROS 路径带进 Isaac 进程。

```bash
python3 scripts/fetch_oxford_devices.py --target assets/oxford_devices
python3 scripts/prepare_oxford_devices.py assets/oxford_devices \
  --ros-python-path /opt/ros/humble/local/lib/python3.10/dist-packages
# 转换器拒绝覆盖已有 usd 输出，重建请指定新的相对目录。
env -u DISPLAY -u VIRTUAL_ENV -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_DISTRO \
  OMNI_KIT_ACCEPT_EULA=YES PYTHONUNBUFFERED=1 \
  "$ISAAC_PYTHON" scripts/convert_oxford_devices.py assets/oxford_devices --usd-dir usd_rebuild
python3 scripts/build_oxford_gallery.py assets/oxford_devices
env -u DISPLAY -u VIRTUAL_ENV -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_DISTRO \
  OMNI_KIT_ACCEPT_EULA=YES PYTHONUNBUFFERED=1 \
  "$ISAAC_PYTHON" scripts/validate_oxford_devices.py assets/oxford_devices \
  --output outputs/oxford_migration
```

`ISAAC_PYTHON` 使用本项目 Isaac Sim 6.2 的解释器。完整结论见
`docs/线上操作资产迁移验收_2026-10-30.md`。

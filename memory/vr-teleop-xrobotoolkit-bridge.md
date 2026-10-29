# VR 遥操作接入 XRoboToolkit 那套（RouteManager 侧）

2026-10-19 起，RouteManager 多了一条**进程外**的 VR 手臂遥操作通路。分支
`feat/vr-md-stack`（RouteManager 与遥操作仓库各一条同名分支）。

## 两条 VR 通路（互斥）

| | 进程内（原有） | 进程外（新增） |
|---|---|---|
| VR 端软件 | CloudXR + `isaacteleop` | XRoboToolkit PC Service + `xrobotoolkit_sdk` |
| IK | 自研 Jacobian 表 + DLS（`isaac_teleop.py`） | placo（`xr-robotics` env） |
| 传输 | 同进程直接写 articulation | UDP `{"names","positions"}` |
| 入口 | `start_vr_teleop` | `start_udp_teleop` |

`start_udp_teleop` 会 `set_suspended(True)` 挂起进程内 VR，`stop_udp_teleop`
恢复。**不要同时开两条**，会互相覆盖关节目标。

## 启动顺序

```bash
# 1) PC Service（不自启）
/opt/apps/roboticsservice/runService.sh

# 2) RouteManager（必须 setsid，见下）
cd /home/jy/RouteManager && DISPLAY=:1 setsid nohup ./scripts/run.sh > /tmp/rm.log 2>&1 &
#    然后 POST /execute {"action":"load_robot"}
#    然后 POST /execute {"action":"start_udp_teleop","parameters":{"port":63900}}

# 3) 遥操作端（指向 RouteManager 的端口）
cd ~/datasets/vla_data/数据与代码/代码/XRoboToolkit-Teleop-Sample-Python
DISPLAY=:1 setsid nohup ./run_teleop.sh \
    scripts/simulation/teleop_humanoid_v3_placo.py --isaac-udp-port 63900 > /tmp/teleop.log 2>&1 &
```

## 坑

- **必须 `setsid`**：RouteManager 自己注册了 SIGHUP→优雅关闭（见 AGENTS.md
  「退出与残留清理」），`nohup` 挡不住 —— 程序会覆盖 nohup 设的 `SIG_IGN`。
  启动它的那个 shell 一退出，服务就被 SIGHUP 带走。实测：用 nohup 起的实例
  活了 41 分钟后随 shell 退出而死，日志是干净的 "Application shutdown complete"。
- **PC Service 只推给一个 SDK 客户端**（md §3.5）。不能同时跑两个遥操作进程，
  第二个拿到的是冻结快照。
- **第二次 `load_robot` 必崩**：URDF 导入器在 `UsdGeomXformable::AddTranslateOp`
  上 abort，2 秒后进程死。实测日志 `load_robot#13` 提交后即崩。要重载必须重启服务。
- `get_joint_names` **默认隐藏夹爪跟随关节**，只列 28 个；物理 DOF 是 50。
  要全量传 `full=true`（返回 `dof_names_all`）。合成 UDP 包时别拿默认那份当关节集。
- 日志里 `Failed to create change watch ... errno=28/No space left on device`
  **不是磁盘满** —— 是 inotify 句柄耗尽（内核也用 ENOSPC 报这个）。本机限额
  128 实例 / 65536 watch，被桌面应用占满。不影响功能。

## 实测（2026-10-19）

- 合成包：30 发 30 收 0 丢，12/12 臂关节跟上 +0.15rad，非臂关节 0 漂移。
- 真遥操作端：50 关节、~75fps、0 丢包、帧龄 0.007s、关节值全有限无发散。

相关 [[isaacteleop-vr-connection]]、[[routemanager-project]]。

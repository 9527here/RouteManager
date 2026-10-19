# 抓取下降段抖动（stop-go 锯齿）的根因与修复

2026-10-13 实测定位并修复。症状：自动抓取时"机器人下夹爪的时候很抖"。

## 根因：每个笛卡尔路点都是一次完整的启停

`send_joint_command(q, duration=0.25)` 走的是 `_advance_motion` 的
**smoothstep 插值**，而 smoothstep 在 t=0 和 t=1 处速度**都是 0**：

```python
# robot_manager._advance_motion
start = self._joint_positions_flat()      # 每次调用都重新取"当前位置"
self._motion = {"start": start, "target": q, "t0": now, "duration": 0.25}
s = smoothstep(elapsed / duration)
interp = [lerp(a, b, s) for a, b in zip(start, target)]
```

下降段为了不斜着蹭到方块，把 250mm 的直线打成了 N 个笛卡尔路点，
每个路点发一次 `send_joint_command` → **N 次"加速→减速→停死"**。

实测每步位移（物理步，不是墙钟速度）：

| | 每步位移序列 | 峰/中位 |
|---|---|---|
| 旧（离散路点） | `0.6 2.7 1.4 5.7 1.6 5.9 1.2 3.3 0.5 0.8 0.3 3.1 1.2 5.8 …` | **152.8** |
| 新（连续轨迹） | `5.7 5.7 4.3 4.1 3.5 3.2 2.8 2.4 2.3 2.1 1.8 … 4.9 4.7 5.1 4.8 5.1 … 1.8 1.5 1.1` | **1.6** |

旧的是锯齿，新的是一条 S 曲线。抓取成功率也从"没夹住"变成连续两次"完成"。

## 修复：整段连续流送

`robot_manager._auto_pick_advance` 的 `descent`/`ascent` 分支：

1. 路点之间用 **Catmull-Rom 插值**（`RobotManager._traj_sample`，C1 连续，
   路点处速度不跳变）；
2. 整段用**一条** smoothstep 时间律（`u = smoothstep(el / T)`，只在起点/终点
   速度为 0）；
3. **每帧**把参考点直接写进伺服目标：`send_joint_command(q, duration=0.0)`。

`T` 在规划阶段按 `行程 / 0.12 m/s` 定，夹在 [1.0, 4.0]s（`ap["traj_T"]`）。
下降 2.1s / 上升 1.7s，比旧的 6.5s 还快。

两个配套点：

- 进入下降/上升时先 `self._motion = None` —— 否则上一段遗留的 smoothstep
  插值会继续按旧目标写伺服目标，和流送打架。
- `preclose` 相位：走完轨迹 ≠ 手臂停了，合爪前等**实测关节速度**归零
  （`_arm_at_rest`），否则指垫还在横移时合爪会把方块扫出虎口。

## 测量教训（重要）

**不要用墙钟时间算速度。** `step()` 的 `world.step(render=should_render)` 在
`_render_interval > 1` 时隔帧渲染，采样时间戳会成对出现（间隔 ~8ms），
真实位移除以 8ms 会算出 400~700mm/s 的假速度峰 —— 看起来像 16Hz 抖动，
其实是采样假象。**判断平顺性要看"每步位移"**（物理步 dt 恒定），
或者至少把成对的采样合并。

## 还没验的

以上在 `v3_ctag2f120_cameras`（旧 USD 机器人）上验证通过。RM65 双臂版
（`v3_ctag2f120_rm65`，走 `load_urdf`）用的是同一份 `robot_manager.py`，
但 RM65 那台还有别的毛病（下降末段会把方块推开 ~22cm），需要单独再查。

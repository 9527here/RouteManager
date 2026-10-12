# 关节正方向实测法 + head_yaw 实测结论

## 结论（2026-09-29 实测，以此为准，不要按 URDF 推）
- **`head_yaw_joint` 正值 = 机器人头向"它自己的右边"转**。
  URDF 里 axis=(0,0,1)、父链 rpy≈0，按 REP-103 会推成"正值=左转"——**推错了**。
  仿真里就是这么表现的，所以头显映射必须用实测值：客户端 yaw 提取是"右为正"，
  头目标取**同号** `dy = +(yaw − ref)`（用户左转 → yaw 变小 → dy 为负 → 头左转）。

## 判定关节正方向的通用方法（可复用于任何关节）
1. 直接驱动该关节（`send_joint_command`，把其它关节填当前实测值），抓 ±0.4~0.5 rad 两帧
   `/World/cams/head_zed_x` 图（`get_camera_image` → data.image_base64）。
2. 用 PIL+numpy 做颜色阈值分割找目标质心（橙方块 R>180,G 90-190,B<110；蓝圆柱 B>170,R<150,G>130），
   比较 ±两帧质心的位移方向：
   - 物体**左移** = 相机向右转；物体**右移** = 相机向左转（= 你转头时世界反向滑）。
3. 关键前提：VR 遥操作在主线程每帧会写 `_q_all`，**会覆盖 HTTP 的关节指令**。
   测之前用 `/python` 把 `executor._vr_teleop._head_joints = {}`（或对应通道）临时摘掉，
   测完恢复。实测：不摘的话蓝柱质心三帧纹丝不动（关节根本没动）。

## 踩坑：`/python` 端点不回显 `result`，也不回 stdout
`POST /python {"code": ...}` 只返回 `{"status":"success","message":"任务执行成功"}`，
代码里赋值的 `result` 拿不到（为此白跑过几轮诊断）。
需要读回数据时：**让 /python 里的代码写文件**（如 `/tmp/head_dbg.json`），外面再读文件。

相关 [[isaac-camera-aim-math]]、[[realman-pico-teleop]]。
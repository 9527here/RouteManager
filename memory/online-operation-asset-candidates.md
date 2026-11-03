# 线上操作资产候选（2026-10-30）

用户要求查网上是否有相关操作资产。本轮实际联网搜索及原始网页/仓库文本核对。

- 最有价值来源：`https://github.com/ori-arg/manipulation_playground`，BSD-3-Clause（LICENSE 已查）。
  ROS1/Gazebo，不是原生 USD。急停 Xacro 有 revolute+prismatic 两个可动关节和碰撞，
  隔离开关有 revolute 和碰撞；安全开关的 DAE 分 base/lever。动态反力未经实物标定。
  急停是按下、旋转解锁，不完全等同用户要求的先旋转外圈、再按中心。
- Schneider 官方 TeSys GV6P CAD 页列 `MCADPD0004073_3D-simplified.stp` 约1.3MB；
  仅确认文件入口，未下载，不能认定分件/内部机构完整。
- EMKA CAD 服务说明提供 STEP/IGES、需登录；锁具/锁舌/铰链/钥匙可按型号组合，未下载具体装配。
- PartNet-Mobility 可动柜门数据是候选，但官方条款限非商业研究教育，需账户权限。
- 本轮未找到可核实公开下载的电力螺丝压板/提拉压板完整操作模型。机械夹具压板检索是误匹配。

详细链接、限制及推荐顺序在 `docs/线上操作资产候选_2026-10-30.md`。
未下载模型包、未安装其它仿真环境、未做新资产 Isaac 接触验收。

# 动力柜资产操作验证（2026-10-29）

## 现象

普通 PowerDistributingCabinet.usdc 与 cabinet_v2/cabinet.usd 没有关节；真正有
11 个关节的是 `/home/jy/VLA/simManager/data/environment/dongligui/dongligui.usd`。
原配置锁按钮不动，锁柄运动不稳定；加载还报告 articulation/kinematic 冲突及锚点不重合。

## 根因与修法

独立机构带有多余 articulation 根，固定端又是 kinematic；锁按钮、锁柄嵌套在锁体刚体下；
原零锚点不符合部件初始变换。`scripts/cabinet_inspect.py setup --repair` 在独立引用层中
移除 articulation 根、排除独立关节、修正嵌套刚体变换和关节锚点，并显式选择 convex hull。
原资产和源物理参数不写回。

## 实测

11 个关节中 10 个通过两次往返验证（直线容差 0.5 mm、转角容差 2°）：
按钮约 5.89/6 mm，锁按钮约 4.91/5 mm，两个旋钮约 ±45°。
锁柄仍未通过：目标 45°，等待 10 秒后实际约 36.4°；其剩余根因未确定，不能标记已解决。
顶部 Btn01～03 是指示灯，不能因资产带滑动关节就作为操作按钮；真正操作按钮是 Btn04～07。
柜体没有门铰链或锁舌联动。按钮与灯之间的逻辑未作猜测绑定，机器人接触未测。

完整命令和反馈位置见 `docs/动力柜资产操作验证.md`。

## 无额外铭牌与人形整机（同日后续）

用户要求其它柜体不贴额外铭牌，并明确机器人用本项目人形。改为直接引用 `dongligui.usd`，
不经过包含 nameplates_overlay 的场景入口。当前独立输出在 `outputs/cabinet_humanoid/`。
加入 config 的 `v3_ctag2f120_rm65_l2` 整机，50 DOF；FK/mimic 同步到同款 RM65 人形 URDF。
地面 0、根高度 0.329 沿用 L2 归档最小场景；覆盖本会话 `_base_z=0`，不写 config。
link `(1,61,7)` 和 Jacobian `(1,61,6,56)` 有限，额外 Nameplates prim 不存在。

datasets/vla_data 中另外找到中置柜、直流电源柜；实查均是 3 prim 的 Gaussian Splat，0 个关节，
可作视觉背景。domain-randomization-collection 下的其它动力柜文件是同款外观/单旋钮版本。
详见 `docs/电柜资产清单与人形场景.md`。机器人接触控件仍未验证。

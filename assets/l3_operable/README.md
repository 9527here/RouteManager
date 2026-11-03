# L3 可操作电气机构（参考参数版）

来源：`/home/jy/datasets/vla_data/L3_前端展示/model/`。原始 GLB 不改动，副本在 `source/`。这是外观模型的物理重建，不是厂家已标定数字孪生。所有资产 `calibrated_to_user_hardware=false`。

| 类别 | 资产 ID | 可操作行为 |
|---|---|---|
| 按钮 4 个 | object_0、object_0_variant、object_1、object_2 | 真实滑动、弹簧回位、行程触发亮灯/松开灭灯 |
| 旋钮 6 个 | object_3、object_4、object_5、object_6、object_6_variant、object_19 | 两档90°或三档±45°、限位、弱卡位保持 |
| 螺丝压板 5 个 | object_13、object_14、object_15、object_16、object_111 | 上下螺母旋松与轴移、压板投出/投入、锁止与状态反馈 |

`variant` 对应源文件名中的 `(1)`。7 是静态仪表，8–12/17–18 是铭牌，未列入操作机构。

## 在当前人形场景操作

已安装 `/World/L3Station`，15件全部显示并接入物理控制。左侧保留绿色按钮0(1)、旋钮3、压板14；右侧展示板上排为按钮0/1/2，中排为旋钮4/5/6/6(1)/19，下排为压板13/15/16/111。没有新增铭牌。

Isaac Sim 右侧窗口 **电力柜动作控制** 先按编号选择资产，再点击对应动作。不适用或冲突的动作按钮会禁用。

- 按钮：**按下并保持（亮灯）**、**松开复位（灭灯）**、**按压并复位**。
- 旋钮：直接选择 **-45° / 0° / 45° / 90°**，只开放该件支持的档位。
- 压板：**投出（旋松后拨开）**、**投入（拨回后旋紧）**。
- 人形：**人形右手：按压绿色按钮并复位**，通过实际指垫接触完成。
- 辅助：**停止当前动作**、**查看全部资产**、**查看选中资产**、**查看人形**。

机构动作施加力/扭矩，只有单列的“人形右手”按钮会驱动机器人。当前人形接触只验证绿色0(1)，旋钮/压板的人形接触尚未接入。详细说明见 [中文动作插件](../../docs/电力柜中文动作插件.md)。

在项目根运行（HTTP服务18765和人形已加载）：

```bash
python3 scripts/l3_station.py install       # 幂等安装全部15件，需要已有50DOF RM65人形
python3 scripts/l3_station.py all           # 将旧三件工位扩展为全部15件
python3 scripts/l3_station.py press
python3 scripts/l3_station.py selector --asset object_5
python3 scripts/l3_station.py plate_out --asset object_13
python3 scripts/l3_station.py plate_in --asset object_13
python3 scripts/l3_station.py focus --asset object_111
python3 scripts/l3_station.py status
python3 scripts/l3_station.py robot_button --cycles 3  # 当前柜体接触验证环境专用
```

队列执行期间等待完成或先 `stop`。`status` 读 `outputs/l3_operable/live/live_status.json`。加载快照 USD 后还须安装 runtime；单纯打开 USD 只有几何、碰撞和基础弹簧，灯/档位/等效螺纹联锁需 `scripts/l3_controls_runtime.py` 每物理步执行。

## 物理与参数边界

单位米、Z向上、资产局部 +X 朝正面。共35个活动自由度、50刚体、49碰撞体，基座固定。塑料外观、惯量/质量、摩擦、接触间隙、驱动与限位均已补充。

`reference_profile.json` 明确哪些来自参考数据表、哪些是工程假设。按钮22mm安装规格、4.3mm总行程、2.6mm触发、3.8N名义触发力；旋钮90°/±45°；压板80mm高、M6×1等效耦合、松2圈解锁、90°摆动。**这些不代表L3模型的实际型号或尺寸。**

当前整机使用240Hz物理步进，沿用 TGS。TGS 的名义弹簧参数与实际力—行程读回有数值偏差；PGS 独立恒力检查符合理论曲线，但在现有人形整机场景发生底盘漂移，因此未采用。当前适合动作/接触/状态流程仿真，不能用来认定真实按压力或扭矩。

压板采用等效角度—轴移耦合和软件联锁，无真实螺纹牙接触或接触电阻。14按四个原始连通件拆分，其余压板为近似分件；导电桥为参考重建件。红/绿/白圆形件按按钮候选使用，其真实按钮/灯语义待确认。

## 重建与校准

先备份当前包，不在已加载的USD上原位重建。系统 Python 有 pxr/trimesh 时：

```bash
python3 scripts/build_l3_operable.py /home/jy/datasets/vla_data/L3_前端展示/model \
  --output /tmp/l3_rebuilt --profile /path/to/measured_profile.json
```

以现有 `reference_profile.json` 为模板填实测参数；重新验收后再替换。实际型号、尺寸、行程、档位、按压力/扭矩、螺纹与松紧圈数仍待实测。

完整记录：`docs/L3_电气机构物理改造与验收.md`；数据：`outputs/l3_operable/`。

全量布局 `station_all.usda` 引用原 `station.usda`，原三件的路径/位置保持不变；新增12件同样位于 `Controls/` 下。全量加载记录 `outputs/l3_operable/live/all_assets_install.json`，逐件GUI入口检查 `all_15_live_checks.json`，场景快照 `scene_all_assets.usda`。

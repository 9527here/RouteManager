# SimScene资产工具与当前场景接入

2026-11-03已安装并注册。SimScene负责资产发现、下载缓存、测量和场景方案；SimPeople负责人物活动；RouteManager继续控制原有机器人、阀门和物理主循环。

## 已安装

- 源码`/home/jy/simscene`，提交`7a51df945d7010233023b8bfe3ca41aa1c1427ce`，版本`0.1.0.dev4`。
- 独立环境`/home/jy/simscene/.controller-venv`，含MCP、USD、NumPy/SciPy；未改Isaac环境依赖。
- Codex MCP名称`simscene`，启动`RouteManager/scripts/simscene_mcp.py`；stdio协议实测34个工具。
- 配置`~/.config/simscene/config.json`，场景方案`~/.local/share/simscene`，资产缓存`~/.cache/simscene/assets`，容量2GiB。
- pip版Isaac目录兼容入口`~/.local/share/simscene/isaac-runtime/python.sh`，`kit`/`VERSION`指向现有6.2安装，用于识别内建MDL/版本。本次没有执行它启动仿真。
- MCP注册使用`codex mcp add`，其余原配置已逐字段核对保留。私有备份在`~/.codex/config.toml.before-simscene-20261103`。

这属于本地MCP注册，不是插件市场安装。本轮已通过真实stdio客户端连接/调用；当前聊天的工具列表不会自动增加，新的Codex会话加载注册项后才显示。未另开模型聊天做自然语言验收。

## 当前会话模式

原版SimScene的`run`/`simulation_start`会启动独立Isaac worker，不具备直接附着现有RouteManager的入口。
本适配复用官方资产/场景工具，但从实际MCP工具表移除了启动/控制独立worker及单独生成机器人控制器的入口。
没有新开Isaac，没有暂停后自动恢复旧机器人动作，没有更换RM65控制器。

新增入口：

| 工具 | 作用 |
|---|---|
| `attached_scene_info` | 从RouteManager主线程读取当前场景身份、原生机器人反馈和对象 |
| `attached_scene_preview` | 对保存的SimScene方案测量、导出，保留版本和资产根文件hash；不修改现场 |
| `attached_scene_import` | 核对场景身份/版本，在暂停状态以独立Session覆盖层附着静态视觉方案 |
| `attached_scene_remove` | 只移除适配器拥有的层，并验证机器人状态/目标/增益保留 |

导入仅接受静态`box`、`sphere`、`asset`。资产须显式设置`asset_physics: visual_only`和`asset_lights: disabled`。
相机、活跃灯、物理API、动态投料和控制器都会拒绝。带物理的机构需要针对实际资产另做导入/接触验证，不能把当前视觉导入当作完成物理适配。
导入前使用现有真实形状包络（包括隐藏碰撞体）及整个人物巡检NavMesh做保守布局检查；5cm为本入口参考缓冲，非硬件标定。
这只是当前暂停姿态的包络检查，不证明将来机械臂完整轨迹安全；视觉资产也不会产生物理碰撞。

先`asset_search`，再检查元数据/缩略图、`asset_inspect`和选定资源的`asset_acquire`。搜索结果只是候选，不是资产可用或授权证明。
下载后检查依赖、单位/轴向，再通过`scene_open`、带revision/request_id的`scene_apply`建立方案。
依次调用`attached_scene_preview`、读取现场身份、暂停任务后`attached_scene_import`。同名已附着方案不能原位覆盖，须明确移除或使用新ID。
本地外部资产及依赖不复制到覆盖层；根文件hash与依赖可用性检查不等于完整文件快照或资产内容签名。

## 命令与验证

```bash
/home/jy/simscene/.controller-venv/bin/python scripts/simscene_mcp.py --inspect
/home/jy/simscene/.controller-venv/bin/python scripts/simscene_mcp.py --preview 场景ID
codex mcp get simscene --json
/home/jy/simscene/.controller-venv/bin/python scripts/validate_simscene_attached.py
```

烟测通过真实stdio MCP创建/编辑一个2cm静态方块，导出后附着到[-4,-4,1]m，再清除；原生q/v、root位姿/速度、位置/速度目标、kp/kd、力/速度上限、限位、质量与惯量逐项相同。
将测试物改放机器人作业区后被拒绝，未导入。现场仍PID377760、同一stage、96机器人形状、反馈有效且暂停；临时方块已清理。
公共`industrial cabinet`搜索返回216个候选总数，本轮仅取3条；已有电柜本地USD测量/依赖检查通过。没有为烟测下载任意新大模型。
记录在`outputs/simscene_integration_20261103/`，主报告为`mcp_acceptance.json`，注册核对为`config_preservation.json`。

Kit fast importer可能看不到新增Python文件或继续加载旧模块。适配器从磁盘编译自己的小型主线程模块，避免重启/热替换整个RobotManager。
移除层后若旧Articulation丢了view，只对现有有效物理后端重新绑定，不reset、不回灌关节状态；保留原场景暂停状态。

参考：[SimScene源码](https://github.com/ros-claw/simscene)、[官方MCP注册说明](https://developers.openai.com/codex/mcp)。

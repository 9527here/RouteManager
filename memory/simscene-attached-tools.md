# SimScene现有会话资产工具（2026-11-03）

用户明确继续添加ros-claw/simscene，并沿用不启动独立仿真的约束。
原版run/simulation_start创建worker；不把它当现成Kit附着插件。
源码`/home/jy/simscene` commit7a51df945d7010233023b8bfe3ca41aa1c1427ce，独立`.controller-venv`安装，Isaac env未装控制端依赖。
Codex MCP simscene启动`scripts/simscene_mcp.py`，实际工具表保留资产/保存方案工具和4个attached入口，共34个；移除独立worker/生成机器人profile的启动入口。

真实stdio烟测：小方块创建/编辑/测量/导出/附着/移除通过，故意放进机器人区被拒绝。
PID377760未变、原stage、96形状、暂停；原生q/v/root/root-v/目标/kp/kd/maxForce/maxVelocity/限位/质量/惯量逐项保持。
现有机器人和人物均保留；临时测试方块已清除。当前仅静态视觉层，不认证机构物理、机器人轨迹或传感器识别。
Kit fast importer会对新文件报找不到，须用ModuleType+compile执行小型受控模块；不用重启或重载整个执行器。

配置`~/.config/simscene/config.json`；兼容pip版布局的运行时路径`~/.local/share/simscene/isaac-runtime/python.sh`，kit/VERSION软链至现有6.2。
该shim仅为版本/系统MDL识别，本轮未执行启动Isaac。缓存默认2GiB，与运行记录分离。
公共工业柜搜索216个候选/本轮3条；已有柜体本地依赖与尺寸检查通过，没有任意大资产下载。
codex mcp add会省略原node_repl空args字段，本轮已恢复并逐字段核对其它配置不变；注册通过但不等于市场插件或新聊天自然语言验收。
证据`outputs/simscene_integration_20261103/mcp_acceptance.json`、`config_preservation.json`，流程见`docs/SimScene_资产工具与当前场景接入.md`。

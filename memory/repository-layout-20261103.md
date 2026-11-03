# 平台定位与源码整理（2026-11-03）

用户明确项目目标为一站式仿真配置，整合多方资源，配合自研机器人完成工厂多任务数据采集（Isaac Lab批量）与可达性验证，最终Real→Sim→Real。
README改为定位/工作流/现有能力与验证边界/快速开始/采集/集成/路线图；新docs导航、HTTP接口、资产与文件指南、配置模板。

GitLab main比GitHub main多出6.2/ROS2及归档阀门RL研究历史；在整理分支合并，保留双方历史与可选UDP模块，不强推重写。
旧机器人包当前未被活动USD引用，完整移到`~/RouteManager_assets/legacy/humanoid_upper_body_v3`，根路径保留软链接。
历史123个已跟踪文件有sha256清单与恢复脚本；独立空目录恢复全部通过，不包含原本未入Git的base_link.STL/生成USD。
5个无功能入口引用的相机试验与无关comments样本移出代码树；原始源码、索引与二进制patch在`/home/jy/data/repo-backups/RouteManager/20261103-restructure/`。
生成背景图片取消跟踪但本机保留；outputs约2.3GiB保持原位并忽略，避免破坏活动场景。新CAD/GLB/USD缓存/纹理与验收几何不发布，保留来源/参数/构建入口。
当前Git树113.242→约5.42MiB，Git历史未改写，不能称既有完整克隆的.git缩小。

17个纯unittest模块迁至tests/test_*.py，统一bootstrap，修复两处测试间导入。
本机严格模式192项通过；不带私有模型与outputs的新克隆烟测168项、2个fixture组skip。缺失私有输入可skip，`ROUTE_MANAGER_REQUIRE_MODEL_TESTS=1`时禁止跳过。
153个Python、10个shell、24个JSON语法检查与主入口Markdown链接检查通过。
image-analyzer Claude入口改为指向.agents单份技能源；AGENTS同步ROS2进程内6.2绑定与回归说明，未改主要运行控制算法。

整理期间并行新增的workcell_plugin.py/workcell_valve_switch.py保留本机，未自动纳入本轮提交；后续按其任务验收后再提交。
同步须普通快进并逐远端核对SHA，不能用当前整理覆盖独立研究历史。当前场景未重启，动作/采集未为整理而重跑。

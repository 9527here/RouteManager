# 资产与本机数据

外部资产库的统一入口为`third_party/vla_assets`，当前本机目标是`/home/jy/datasets/vla_data`。
接入与换机操作已合并到[根README](../README.md#接入资产并启动)；本页只维护源码内资产、来源参数与旧包恢复。

仓库提交控制代码、资产来源/目录清单、参数、参考规格与构建脚本。第三方原始CAD、GLB、转换后的大USD、纹理、数据集及验收快照留在本机或外部资产存储中。
原始资产的许可证由提供方决定，不随项目代码的MIT声明改变；没有确认来源/授权的模型不作为可再分发资产包上传。

| 目录/清单 | 用途 | 准备方式 |
|---|---|---|
| `RM65-6FB-V/` | 已有官方RM65机械臂参考描述、规格与少量网格 | 保留现有来源文件；整机预设的L1/L2资产另行准备 |
| `l3_operable/` | L3参考机构参数、目录、来源说明、运输姿态 | 用自己的源GLB和`build_l3_operable.py`生成USD；参考参数未标定到实物 |
| `oxford_devices/` | Oxford机构来源与校验记录 | `fetch_oxford_devices.py`取固定版本，再按转换流程生成 |
| `USD001/` | 用户提供资产的目录、拓扑与动作候选 | 原包与纹理在本机；不是已完成物理适配的机构 |
| `manufacturer_cad/` | 厂商CAD出处与处理说明 | 按来源/许可证获取，生成物不入Git |
| `robot_safety/policy.json` | 当前模型的碰撞例外策略 | 使用前核对实际URDF与几何；不能把旧几何快照当当前场景 |
| `legacy_humanoid_manifest.json` | 旧人形资源包已跟踪部分的hash与历史提交 | 可按下方命令恢复；不是当前六轴整机的完整资产分发包 |

当前本机完整旧包位于`~/RouteManager_assets/legacy/humanoid_upper_body_v3`，仓库根同名路径为软链接。
迁移保留了整包相对依赖和未跟踪文件；Git仅移除当前树的资源包，历史没有改写，所以既有完整克隆的Git历史大小不会因此缩小。

```bash
python3 scripts/restore_legacy_assets.py --dry-run
python3 scripts/restore_legacy_assets.py --destination /path/to/legacy/humanoid_upper_body_v3 --link
```

恢复脚本只取旧提交里的123个已跟踪文件（约105MiB），不会覆盖不同内容。原本未跟踪的超大`base_link.STL`与导入USD需要单独准备。
`outputs/`中的现场USD、采集数据与失败证据没有移动，避免破坏正在使用的引用；这些文件不会上传。

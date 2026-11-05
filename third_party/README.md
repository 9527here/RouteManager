# 外部资产库与工具

本目录保存外部库登记`libraries.json`及本机访问入口。模型原件仍归资产库管理，RouteManager保存任务代码、适配参数、来源和验收结果。

## 本机资产库

用户指定的根目录：`/home/jy/datasets/vla_data`。它是本机归档目录，目前没有根级Git仓库，也没有提供独立远端。

```bash
# 从RouteManager根目录执行；重复链接同一目标不会修改内容
python3 scripts/link_asset_library.py link --root /home/jy/datasets/vla_data
python3 scripts/link_asset_library.py status

# 其它机器可用自己的路径
ROUTE_MANAGER_ASSET_ROOT=/data/vla_data python3 scripts/link_asset_library.py link
```

创建的入口为`third_party/vla_assets`。该链接本身及其内容都由Git忽略；新克隆运行命令重建入口。脚本不复制、不递归扫描资产，不修改`config.json`和当前USD引用；已有不同链接或实体目录会拒绝覆盖。

符号链接供本地读写访问，并不是权限隔离。修改模型时先在独立构建目录生成和验收，不直接覆盖L0/L1源文件。

| 库内层级 | 项目用途 | 接入边界 |
|---|---|---|
| `L0_原始档案/` | CAD、扫描和厂商原始描述 | 保留原件，不直接视为可运行资产 |
| `L1_制作资产/` | 载体设备、操作资产、语义标注 | 核对URDF/网格依赖、单位、版本及实际参数 |
| `L2_仿真运行/` | USD机器人与工厂场景 | 检查相对引用、材质、物理和6.2兼容性 |
| `L3_前端展示/` | GLB外观与展示 | 外观模型需另做关节/碰撞/操作语义适配 |
| `数据与代码/` | 关联框架、数据集和分析 | 按各项目环境使用，不混装到Isaac环境 |

库中已有`数字资产分类-按仿真资产管理标准.md`、`数字资产盘点.md`与`数字资产汇总.md`；详细盘点继续在库内维护，不复制进本仓库制造第二份目录。此处只核实层级和接入点，不将旧盘点数量当作本次完整审计。

例如，已存在的整机描述可经统一入口访问：

```text
third_party/vla_assets/L1_制作资产/载体设备/humanbody_urdf/humanoid_upper_body_v3/urdf/humanoid_upper_body_v3_ctag2f120_rm65_isaac.urdf
```

该路径仅说明访问方式，不自动选为当前机器人。模型适配、软件mimic、真实碰撞/驱动配置和已有运行期产物仍按实际预设核对。

## 已使用的公开资源

| 资源 | 用途 | 固定版本/入口 |
|---|---|---|
| [Oxford manipulation_playground](https://github.com/ori-arg/manipulation_playground) | 急停、隔离和安全开关的源机构 | `3a5b26c`；现有fetch脚本按完整revision取子集，清单见`assets/oxford_devices/provenance.json` |
| [SimPeople](https://github.com/ros-claw/simpeople) | 原生人物巡检 | `8b31548`；外部环境安装，附着现有会话 |
| [SimScene](https://github.com/ros-claw/simscene) | 资产发现与静态方案工具 | `7a51df9`；外部环境安装，本项目禁止独立worker |

这些版本是既有集成验收版本，不代表上游最新版本，也不是自动解析依赖的安装锁文件。完整commit与入口统一登记在`libraries.json`；原文件hash继续保留在各资产provenance清单。

## 后续版本管理

当前采用“本机外部目录+可重建链接”。有独立资产Git远端后，再评估submodule管理小型索引/描述，模型大件由适合的外部存储获取；逐资产版本完善列入ASSET-02；是否设置独立远端在有实际存储需求时决定。

每个正式任务逐步补齐稳定资产ID、源文件hash、相对入口、单位/轴向、依赖、实测/参考参数、许可和验收报告。路径存在仅证明能访问文件，不等于已完成物理适配或允许再分发。

常用操作继续阅读[使用手册](../docs/README.md)。源码内资产及旧包恢复见[assets说明](../assets/README.md)。

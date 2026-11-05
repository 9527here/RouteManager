# 外部库登记

资产接入命令、L0～L3分层和使用边界已合并到[根README的资产接入](../README.md#接入资产并启动)，本页只说明登记文件。

- `libraries.json`：本机资产库位置约定，以及Oxford、SimPeople、SimScene的既有验收revision和入口；不是自动安装锁文件。
- `vla_assets`：本机指向`/home/jy/datasets/vla_data`的忽略链接，新克隆需运行接入命令重建；不是Git子模块。
- 资产库详细盘点继续在库内维护；项目仅保留实际使用资产的适配参数和来源/验收清单。
- 模型留在外部库，遵循各自许可；稳定资产ID、依赖与逐资产版本清单由TODO的ASSET-02继续完善。

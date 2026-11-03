# Schneider GV6P320H 厂家 CAD 底模

来源为 Schneider 官方文档 MCADPD0004073_3D-CAD，具体下载与 SHA256 见 provenance.json。
原 STEP 1,329,459 字节；HOOPS 6.2 转换为 `usd/asset.usda`，米制 Z-up、中心落地。
模型包络 0.255 × 0.179600 × 0.139715 m，19 个 Mesh，包含独立 MANETTE 手柄零件。
这是 CAD 作者定义的模型尺寸，尚未核对用户目标设备。

**仅完成分件几何迁移**：没有添加推定关节、限位、扭矩、碰撞或电气状态。
缺少目标手柄转轴、档位/行程和阻力数据，不能把模型外观迁入算作断路器合/分闸通过。
厂家 CAD 不能按 Oxford 的 BSD 许可分发；本轮未确认其再分发授权。

- `usd/asset.usda`：项目引用入口，相对依赖，已转换毫米到米。
- `usd/geometry.usda`：保留原零件名称的几何层。
- `usd/conversion.json`：尺寸、19 个 Mesh 路径和依赖验收。
- `usd/preview.usda`：独立灯光预览场景。

重建：使用项目 `scripts/convert_operation_step.py`，在清理 ROS 环境的 Isaac Sim 6.2 进程运行。

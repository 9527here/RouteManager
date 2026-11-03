# 地面普通绑定被physics用途绑定覆盖（2026-11-02）

## 现象

阀门验证前排查整机活动态振荡。代码声称地面摩擦1、恢复系数0，但实际
`UsdShade.MaterialBindingAPI(...).ComputeBoundMaterial(materialPurpose='physics')`
对两个同高碰撞面解析出不同材质：

- `/World/ground_plane/geom` → `/World/Physics_Materials/physics_material`：.5/.5/.8。
- `/World/ground_plane/collisionPlane` → `/World/PhysicsMaterials/friction_mat`：1/1/0。

这里均为staticFriction/dynamicFriction/restitution。两个碰撞面都启用，网格顶点Z=0、
Plane世界Z=0。不能只看想绑定的材质属性就认定已生效。

## 根因

本机6.2 legacy GroundPlane创建时将`geom`作为collision_prim包装，并对其绑定默认
PhysicsMaterial(.5/.5/.8)。项目`_ensure_friction_material()`只做普通`Bind(mat)`，
没有覆盖优先级更高的`material:binding:physics`，所以网格仍使用默认可弹跳材质。

## 修法与现场边界

`robot_manager.py::_ensure_friction_material`保留普通绑定兼容，并增加physics用途的
`strongerThanDescendants`绑定，兑现已有`_FRICTION=1`、restitution=0的设定。
没有移除地面、原Leg或任何机器人碰撞，没有改变几何/质量/刚度/阻尼。

当前运行实例未整类reload；仅SessionLayer更新三个现有地面prim的physics关系。
保存原Session关系存在性/spec，首真实物理步root平移约0.951µm、朝向0.0000893°，
50位置/速度目标、kp/kd/maxforce全部精确保留，视图有效。再次ComputeBoundMaterial
对根/geom/Plane均解析到既有1/1/0材质。

证据：`outputs/valve_operation_20261102/jitter/ground_material_audit.json`、
`ground_binding_fix/apply_result.json`。纯mock回归`validate_floor_physics_material.py`
4/4，包含旧代码失败复现、全部有效地面prim覆盖、幂等和跳过缺失prim。

**不能把绑定错误修正夸大为全部抖动已解决**：随后strict小动作仍因非头SDK瞬时速度
.03参考门失败。实际位姿/动作稳定性与原始速度诊断应分别记录，完整动态对照仍在验证。

11-03收尾：原GUI进程后来退出，未自动重启；上文Session修改描述的是退出前的实际验证，
不是当前服务存在的证明。源码修复保留；最后纯待机约8s的末2s全50关节速度和摆幅0，
仍不能代替小动作或阀门操作动态验收。

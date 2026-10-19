# STEP→USD：Isaac 5.1 HOOPS 无头导入与验收流程

2026-10 导入 `人形上装机器人-简化模型.STEP`（161MB，SolidWorks 2025 AP214，mm）
走通的完整管线。产物在 `~/下载/`：
`...usd`（转换器原始，mm）、`..._meters.usd`（仅改 metersPerUnit=1）、
`..._isaac.usda`（米制 wrapper，可直接拖 Isaac）。

## 转换（不用装 FreeCAD）

headless SimulationApp 里运行时启用 `omni.kit.converter.hoops_core`（509.1.0）：
`omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(...)`，
然后 `omni.kit.converter.lib`？不——用
`from omni.kit.converter.hoops_core import get_instance`（模块路径以实际 ext 为准），
`get_instance().config_path_to_args("/tmp/hoops_cfg.json")`（`{}` 即可），
`await get_instance().create_converter_task(in_step, out_usd, args)`。
脚本要点：`sys.argv=[sys.argv[0]]`（headless 会把 argv[1] 当 stage 路径）、
路径用环境变量传、`print(flush=True)`、`app.close()` 后的代码不执行。

## 转换器产物结构（Mesh=0 不是丢模型！）

- 零件原型在 `Prototypes` Scope 下且 `instanceable`，装配子树用**同层 internal
  reference**（无 assetPath，只 primPath）引用原型。
- 普通 `stage.Traverse()` 被实例裁剪，进不了原型内部（Mesh 数=0）。
  要数几何：Sdf 层直接数（693 Mesh / 105 万顶点 / 120 万面），
  或遍历 `GetAllChildren()`（UsdPrim **没有 GetDescendants()**）。
- 找实例站点：递归 GetAllChildren + `HasAuthoredReferences()`，
  BBoxCache.ComputeWorldBound 在站点 prim 上可直接算世界 bbox。
  注意实例站点内部不会继续 GetAllChildren（被裁剪），只能逐站点算。
- 零件名：ASCII 的可读（`tn__ZEDXNarrow_...`、`tn__gemini_335lg_with_fov...`、
  `tn__PHU32H142...`）；中文名是私有 hash，无法解码。
- `bConvertCurves=false`：STEP 里的相机 FOV 辅助线（671 条）不会转曲线，正常。

## mm→m 双层修正法（坐标数值不动，最稳）

1. 拷贝原始 usd，只把层 metadata `metersPerUnit` 0.001→1（坐标仍是 mm 数值）。
2. 外层 wrapper（米）reference 内层根 prim，加 `scale=0.001`。
3. 该 STEP 的 **CAD -Z 才是机器人上方**（轮子在 +z 端）→ wrapper 加
   `rotateXYZ=(180,0,0)`，再 translate z=+1.07747（原 mm bbox max z），
   轮底精确落在 z=0。最终 1.108×0.760×1.696m，机器人正面朝世界 -X。
   Prototypes Scope 自身 bbox 为空，不会在原点多渲染一份。

## 两个渲染大坑

- **pxr 先于 omni 开 stage → `World()` 创建物理场景崩溃**。
  必须先 `omni.usd.get_context().open_stage()`，在 omni stage 上算 bbox，再建 World。
- **本机 Gf python 绑定 `Matrix3d.SetRow()` 实际按列存**，ExtractRotation 读到
  入参的转置；`Matrix4d.SetRotate+ExtractRotationQuat` 会静默返回错四元数。
  正确做法：look-at 旋转矩阵 R 的列 = [s, u, −f]（USD 相机视线 -Z、上 +Y），
  把三个列向量依次 `SetRow(0..2)`，`ExtractRotation()` 得 GfRotation(angle 是**度**)，
  再手动转四元数 (cos θ/2, axis·sin θ/2)，`set_world_pose(..., camera_axes="usd")`。
  构造后务必数值验证 q(0,0,−1)=f、q(0,1,0)=u。参见全局记忆 isaac-camera-aim-math。

## STEP 文本级点名开口曲面（SHELL_BASED_SURFACE_MODEL）

6 个开口件归属：末端视觉相机、PHU-11H-52 电机、eBom-FreeParts、
PHU-32H-142 电机、gemini_335lg、ZED X Narrow-头部——全是相机/电机外购件，
镜头玻璃与供应商模型常见开口，非丢件。
追链要点：shell→MANIFOLD_SURFACE_SHAPE_REPRESENTATION→
**SHAPE_REPRESENTATION_RELATIONSHIP 要横向跳到对端 rep**（光走反向引用会断）
→SHAPE_DEFINITION_REPRESENTATION（顺边跳到 PRODUCT_DEFINITION_SHAPE）
→PRODUCT_DEFINITION→FORMATION→PRODUCT。中文名 `\X2\hex\X0\`，
先试 UTF-16BE 再 UTF-8/GBK。1.66M 实体的 STEP 全量正则 dict 约几百 MB，能跑。

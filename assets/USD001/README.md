# USD001 操作场景资产

2026-10-30 从 `/home/jy/下载/USD001/` 导入。11 个 USDZ、18 个源文件逐一 SHA-256 比对，
USDZ CRC 和贴图依赖检查通过。原始文件与源目录内容一致；未给原资产加铭牌、物理参数或缩放。

这批是 **001～010 共十个柜体/箱体外观，以及 011 一个独立方形指针表**，
不是与《电力柜动作梳理.md》11 类动作一一对应的十一套机构。

## 项目入口

- `gallery.usda`：所有资产按原始比例排列的静态展示场景。
- `normalized/001.usda`～`normalized/011.usda`：单资产引用层，去掉源自带的黑色环境光，
  只做水平居中和落地平移；不改源几何、不缩放、不补物理。
- `manifest.json`：源文件 SHA-256、单位、外包络、材质、关节/碰撞及依赖检查。
- `catalog.json`：外观候选名称、可见部件与候选动作，不能视作真实型号/功能认定。
- `action_coverage.json`：逐项对应 22 个功能动作，区分候选与仍缺的机构。
- `topology_audit.json`：合并 Mesh 的几何连通分量检查；连通分量不等于运动学部件。

每个 USDZ 都只有一个合并 Mesh；11 个资产合计关节、刚体、碰撞 API 均为 0。
源文件虽然保留多个材质分组、部分不相连几何，但没有独立可动 prim、转轴或行程。

## 加载

在空白 RouteManager 会话中，可通过现有 action 加载：

```json
{
  "action": "load_scene",
  "parameters": {
    "usd_path": "assets/USD001/normalized/010.usda",
    "prim_path": "/World/ControlCabinet010"
  }
}
```

随后加载项目人形预设。已有机器人在运行时，不直接追加/删除这些结构，以免 articulation
物理视图失效。单资产引用层不自带灯、地面或物理，使用 RouteManager 场景中的环境。

## 独立检查与预览

```bash
python3 scripts/import_operation_assets.py /home/jy/下载/USD001 --target assets/USD001

env -u DISPLAY -u VIRTUAL_ENV -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_DISTRO \
  OMNI_KIT_ACCEPT_EULA=YES PYTHONUNBUFFERED=1 \
  /home/jy/micromamba/envs/isaacsim6.0/bin/python \
  scripts/preview_operation_assets.py assets/USD001/manifest.json \
  --output outputs/usd001_inspection
```

预览已在 Isaac Sim 6.2 独立进程完成，未给运行中的人形注册相机。
截图和索引图位于 `outputs/usd001_inspection/`。

详细动作分析见项目 `docs/USD001_新资产与电力柜动作对应.md`。

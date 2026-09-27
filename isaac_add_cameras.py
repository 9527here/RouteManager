#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Isaac Sim: 按 URDF 外参为 7 路外置相机批量创建 USD Camera prim (+可选 ROS2 发布)。

用法
----
1) 先把 urdf/humanoid_upper_body_v3_ctag2f120_cameras.urdf 用 URDF Importer 导入
   (Merge Fixed Joints 勾不勾都行——本脚本两种情况都能定位, 见下);
2) Isaac Sim 菜单 Window > Script Editor, 打开本文件, Run;
   或命令行:  ./python.sh /path/to/isaac_add_cameras.py
3) ROS2 发布需要 Isaac 启动前 source ROS2 环境(4.x 默认自动加载 ros2 bridge);
   ENABLE_ROS2=False 可只建相机不出图。

定位逻辑(与导入选项无关)
------------------------
- 若 {robot}/{cam}_link prim 存在(未合并固定关节) -> 直接在其下建 camera prim;
- 若被合并进父 link -> 在父 link prim 下按 URDF joint origin 新建 {cam}_link
  Xform(位姿=URDF xyz/rpy), 再挂 camera。位姿始终来自 URDF, 零手工误差。

相机朝向约定
------------
URDF 本体系 x=视轴前/y=左/z=上; USD Camera 沿自身 -Z 看向 +Y 上。
camera prim 相对本体帧 rpy=(90,0,-90)°: 视向=本体+x, 图像右=本体-y, 上=+z,
与 ROS optical (z前x右y下) 等价, 故 frameId 用 {cam}_optical_link。

注意
----
- 分辨率/FOV/深度量程按规格书核实后改 CAMERA_PARAMS 即可(现值标注待核实);
- 相机出图需先点 Play; 若 initialize 报错, Play 后重跑本脚本即可;
- ROS2CameraHelper 的参数名随版本可能微调(已做两种签名兼容), 失败时相机
  prim 已建好, 可在 GUI/OmniGraph 里手动接 ROS2 Camera Helper。

--check: 不进 Isaac, 只解析 URDF 打印将要创建的 prim/位姿(用于离线核对)。
"""

import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

# 包目录: 优先按脚本位置推算; Script Editor 会把脚本拷到 /tmp/carb.* 运行,
# __file__ 失真, 此时回退本机安装路径(也可用 --urdf= 指定别的 URDF)
_PKG_CANDIDATES = [
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
    '/home/jy/下载/humanoid_upper_body_v3_ctag2f120_min/humanoid_upper_body_v3',
]
_PKG_URDF = 'urdf/humanoid_upper_body_v3_ctag2f120_cameras.urdf'
PKG = next((p for p in _PKG_CANDIDATES if os.path.isfile(os.path.join(p, _PKG_URDF))),
           _PKG_CANDIDATES[0])
URDF = os.path.join(PKG, _PKG_URDF)

# ------------------------------------------------------------ 可改配置
ROBOT_PRIM_PATH = None       # 例 '/World/humanoid_upper_body_v3'; None=自动探测
ENABLE_ROS2 = True           # False: 只建相机 prim, 不接 ROS2 发布
PUBLISH_DEPTH = True         # ROS2 侧尽量同时挂 depth helper(版本差异见文末注释)

# 分辨率/FOV 2026-09-23 官方规格书确认(视恩特 Viewsitec 代理手册, ~/图片/zedx*.png):
#  - ZED X: 1920x1200✓; 2.2mm 广角 SKU: Max 110°H×80°V×120°D, f/2.2, 畸变<5.7%,
#    深度 0.3~20m; 4mm 窄角 SKU: 80°H×52°V, 0.1~8m。默认按广角 SKU(ZED-311120)
#  - ZED X One S: 1920x1200✓; 宽镜头 Max 110°H×81°V×120°D f/2.2(0.3~15m);
#    窄 80°H×52°V / 鱼眼 204°H×137°V。默认按宽镜头(ZED-221120)
#  - Gemini 335L: 深度 H90°V65° / RGB H94°V68° @1280x800(orbbec 官网✓), 取深度 FOV
#  - Gemini 305: 已从 URDF 移除(09-23 腕部改 335L), 参数仅存档
CAMERA_PARAMS = {
    'ZED X':       dict(resolution=(1920, 1200), hfov_deg=110.0, near=0.30, far=20.0),
    'ZED X One S': dict(resolution=(1920, 1200), hfov_deg=110.0, near=0.30, far=15.0),
    'Gemini 335Lg': dict(resolution=(1280, 800),  hfov_deg=90.0,  near=0.15, far=10.0),
    'Gemini 305':  dict(resolution=(1280, 800),  hfov_deg=90.0,  near=0.10, far=8.0),
}

# 真实标定内参（像素单位）——拿到标定数据后按相机名/型号填，优先级高于 hfov 标称值：
#   INTRINSICS['head_zed_x'] = dict(fx=..., fy=..., cx=..., cy=...)
# 方形像素假设(fx≈fy)；cx/cy 缺省=画面中心；fx/fy 差异>5% 会告警；
# 主点经 apertureOffset 落地，符号建议投影验证一次。空表 = 全部回退 hfov 标称。
INTRINSICS = {}

# 运行参数(--intrinsics / --urdf= 由 main() 填充; Script Editor 直跑时用默认值)
_ARGS = {'intrinsics': False, 'urdf': None}

# URDF 本体帧名 -> 型号
MODEL_OF = {
    'head_zed_x':             'ZED X',
    'head_surround_left':     'ZED X One S',
    'head_surround_right':    'ZED X One S',
    'head_surround_rear':     'ZED X One S',
    'chest_gemini_335lg':     'Gemini 335Lg',
    'left_wrist_gemini_335lg': 'Gemini 335Lg',
    'right_wrist_gemini_335lg': 'Gemini 335Lg',
}

# ROS2 话题: 最终 topic = /{namespace}/{topic}(camera_info 由 helper 自动派生)
ROS2_TOPICS = {n: dict(namespace=n, topic='image_color') for n in MODEL_OF}

D2R = math.pi / 180.0


# ------------------------------------------------------------ URDF 解析
def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def parse_cameras(urdf=URDF):
    """返回 7 台相机的安装信息: parent link / xyz / rpy / 型号。"""
    root = ET.parse(urdf).getroot()
    by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
    cams = []
    for name, model in MODEL_OF.items():
        j = by_child[name + '_link']
        o = j.find('origin')
        xyz = np.array(list(map(float, (o.get('xyz') or '0 0 0').split())))
        rpy = list(map(float, (o.get('rpy') or '0 0 0').split()))
        cams.append(dict(name=name, model=model,
                         parent=j.find('parent').get('link'),
                         xyz=xyz, rpy=rpy))
    return cams


def plan_paths(cams, robot_path):
    """离线预览: 每台相机计划创建的 prim 路径(实际以 stage 里有无 link prim 为准)。"""
    rows = []
    for c in cams:
        rows.append((c['name'], c['model'],
                     f"{robot_path}/{c['parent']}.../{c['name']}_link/camera"))
    return rows


def pinhole_from_params(name, model, params, use_intrinsics=True):
    """针孔参数: 返回 (focal_mm, off_h_m, off_v_m, fx_px, 来源描述)。
    优先 INTRINSICS 标定值(fx→焦距, cx/cy→apertureOffset 米)；
    无标定时回退 hfov 标称值(主点居中, 零偏移)。"""
    res_w, res_h = params['resolution']
    AP_MM = 20.955                      # USD 默认水平像元, 保持不变
    intr = (INTRINSICS.get(name) or INTRINSICS.get(model)) if use_intrinsics else None
    if intr:
        fx = float(intr['fx'])
        fy = intr.get('fy')
        if fy is not None and abs(fy - fx) / fx > 0.05:
            print(f'  ⚠ {name}: fx({fx:.1f})/fy({fy:.1f}) 相差>5%, 按方形像素仅取 fx')
        cx = float(intr.get('cx', res_w / 2.0))
        cy = float(intr.get('cy', res_h / 2.0))
        focal_mm = fx * AP_MM / res_w
        off_h_m = (res_w / 2.0 - cx) * (AP_MM / 1000.0) / res_w
        off_v_m = (res_h / 2.0 - cy) * (AP_MM / 1000.0) / res_h
        return (focal_mm, off_h_m, off_v_m, fx,
                f'标定内参 fx={fx:.1f}px cx={cx:.1f} cy={cy:.1f}')
    h = math.radians(params['hfov_deg'])
    focal_mm = (AP_MM / 2.0) / math.tan(h / 2.0)
    fx = res_w / (2.0 * math.tan(h / 2.0))
    return (focal_mm, 0.0, 0.0, fx,
            f'hfov {params["hfov_deg"]:.0f}°标称·待核实 fx≈{fx:.0f}px')


# ------------------------------------------------------------ Isaac 部分
def find_robot_root(stage):
    """自动探测机器人根 prim(仅用于打印): 任一已知 link 名上溯到顶层。"""
    targets = {'head_pitch_link', 'left_wrist_link', 'torso_pitch_link',
               'base_link'}
    for prim in stage.Traverse():
        if prim.GetName() in targets:
            chain = []
            p = prim
            while p.IsValid() and p.GetPath() != '/':
                chain.append(p)
                p = p.GetParent()
            return str(chain[-2].GetPath()) if len(chain) >= 2 else str(chain[0].GetPath())
    return None


def build_name_map(stage):
    """整个 stage 里找需要的 link 名 -> prim path (取第一个, 重名告警)。"""
    wanted = set(MODEL_OF) | {'head_pitch_link', 'torso_pitch_link',
                              'left_wrist_link', 'right_wrist_link'}
    m = {}
    for prim in stage.Traverse():
        n = prim.GetName()
        if n in wanted:
            if n in m:
                print(f'  ⚠ 重名 prim: {n} @ {m[n]} 与 {prim.GetPath()}, 取先者')
            else:
                m[n] = str(prim.GetPath())
    return m


def set_xform_from_urdf(stage, prim_path, xyz, rpy):
    """把 prim 的局部变换设为 URDF joint origin (xyz + rpy)。"""
    from pxr import Gf, UsdGeom
    xf = UsdGeom.Xform.Define(stage, prim_path)
    R = rpy_to_R(*[a * D2R for a in rpy])
    m = Gf.Matrix4d(R[0, 0], R[0, 1], R[0, 2], xyz[0],
                    R[1, 0], R[1, 1], R[1, 2], xyz[1],
                    R[2, 0], R[2, 1], R[2, 2], xyz[2],
                    0, 0, 0, 1)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(m)


def aim_camera_local(prim):
    """camera prim 局部姿态: 本体系 -> USD 相机 (rpy=90,0,-90)。"""
    from pxr import UsdGeom
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddRotateXYZOp().Set((90.0, 0.0, -90.0))   # 见文件头"相机朝向约定"


def get_camera_cls():
    try:
        from isaacsim.sensors.camera import Camera      # Isaac Sim 4.x
        return Camera
    except Exception:
        from omni.isaac.sensor import Camera            # Isaac Sim 2023.x
        return Camera


def get_ros2_helper_cls():
    if not ENABLE_ROS2:
        return None
    try:
        from isaacsim.ros2.bridge import ROS2CameraHelper      # Isaac 4.x
        return ROS2CameraHelper
    except Exception:
        try:
            from omni.isaac.ros2_bridge import ROS2CameraHelper  # 2023.x
            return ROS2CameraHelper
        except Exception:
            return None


def make_camera(stage, cam, name_map):
    """在一台相机的帧上建 USD Camera prim, 返回 (cam_prim_path, 位置来源)。"""
    from pxr import Sdf

    name, model = cam['name'], cam['model']
    link_name = name + '_link'
    params = CAMERA_PARAMS[model]
    if link_name in name_map:                     # 未合并固定关节: link prim 已在
        cam_link_path = name_map[link_name]
        src = '已存在 link'
    else:                                         # 被合并: 按外参在父 link 下新建
        parent_path = name_map.get(cam['parent'])
        if not parent_path:
            have = [k for k in ('head_pitch_link', 'torso_pitch_link',
                                'left_wrist_link', 'right_wrist_link') if k in name_map]
            raise RuntimeError(
                f'找不到 {cam["parent"]} 的 prim。找到的相关 link: {have}。'
                f'确认导入的是 cameras(_isaac).urdf 且机器人已在当前 stage')
        cam_link_path = str(Sdf.Path(parent_path).AppendChild(link_name))
        set_xform_from_urdf(stage, cam_link_path, cam['xyz'], cam['rpy'])
        src = f'按 URDF 外参新建(父 {cam["parent"]})'
    cam_path = str(Sdf.Path(cam_link_path).AppendChild('camera'))

    Camera = get_camera_cls()
    cam = Camera(prim_path=cam_path, resolution=params['resolution'])
    prim = stage.GetPrimAtPath(cam_path)
    aim_camera_local(prim)
    # 针孔参数: 优先标定内参(--intrinsics), 缺省 hfov 标称。
    # ⚠ Isaac 5.x 的 set_focal_length 收米(attr=米×10) —— 旧版脚本传 mm 数当 cm
    #   用, 实际被当成米, FOV 缩成 ~0.16°, 已修。
    focal_mm, off_h_m, off_v_m, _fx, intr_src = pinhole_from_params(
        name, model, params, use_intrinsics=_ARGS.get('intrinsics', False))
    try:
        cam.set_focal_length(focal_mm / 1000.0)           # mm → 米
    except Exception as e:
        print(f'  ⚠ {name}: set_focal_length 失败({e}), FOV 请在 GUI 里设置 '
              f'{params["hfov_deg"]}°(焦距 {focal_mm:.2f}mm)')
    if abs(off_h_m) > 1e-12 or abs(off_v_m) > 1e-12:
        prim.GetAttribute('horizontalApertureOffset').Set(off_h_m * 10.0)  # 米→attr
        prim.GetAttribute('verticalApertureOffset').Set(off_v_m * 10.0)
    print(f'  针孔: 焦距 {focal_mm:.2f}mm | {intr_src}')
    try:
        cam.set_clipping_range(params['near'], params['far'])   # m
    except Exception as e:
        print(f'  ⚠ {name}: set_clipping_range 失败({e})')
    try:
        cam.add_distance_to_image_plane()                 # 深度流
    except Exception as e:
        print(f'  ⚠ {name}: 深度流开启失败({e})')
    return cam_path, src


def main_isaac():
    import omni.usd

    print(f'URDF: {URDF}')
    stage = omni.usd.get_context().get_stage()
    cams = parse_cameras()
    robot_root = ROBOT_PRIM_PATH or find_robot_root(stage)
    print(f'机器人根 prim: {robot_root or "(未自动识别, 不影响挂载)"}')
    name_map = build_name_map(stage)
    missing = [c['name'] + '_link' for c in cams
               if c['name'] + '_link' not in name_map
               and c['parent'] not in name_map]
    if missing:
        raise RuntimeError(
            f'这些相机的 link 和父 link 都不在 stage: {missing}。'
            f'确认当前 stage 已导入 cameras(_isaac).urdf')

    Helper = get_ros2_helper_cls()
    if ENABLE_ROS2 and Helper is None:
        print('⚠ ROS2 桥不可用(未装/未启用 isaacsim.ros2.bridge), 只创建相机 prim')

    print(f'{"相机":24s} {"prim 路径":58s} {"来源":28s} 话题')
    ok = 0
    for cam in cams:
        try:
            cam_path, src = make_camera(stage, cam, name_map)
        except Exception as e:
            print(f'  ✗ {cam["name"]}: {e}')
            continue
        ros = ROS2_TOPICS[cam['name']]
        topic = f'/{ros["namespace"]}/{ros["topic"]}'
        if Helper is not None:
            for attempt in ('full', 'minimal'):
                try:
                    if attempt == 'full':
                        h = Helper(prim_path=cam_path, topicName=ros['topic'],
                                   frameId=cam['name'] + '_optical_link',
                                   nodeNamespace=ros['namespace'])
                    else:
                        h = Helper(prim_path=cam_path, topicName=ros['topic'],
                                   frameId=cam['name'] + '_optical_link')
                    h.initialize()
                    break
                except TypeError:
                    continue
                except Exception as e:
                    print(f'  ⚠ {cam["name"]}: ROS2 helper 失败({e}); '
                          f'相机已建好, 可在 GUI 手动接发布器')
                    break
            if PUBLISH_DEPTH and attempt == 'full':
                try:
                    h2 = Helper(prim_path=cam_path, topicName='depth_image',
                                frameId=cam['name'] + '_optical_link',
                                nodeNamespace=ros['namespace'])
                    h2.initialize()
                    print(f'{"":90s} 深度: /{ros["namespace"]}/depth_image')
                except Exception:
                    pass
        print(f'{cam["name"]:24s} {cam_path:58s} {src:28s} {topic}')
        ok += 1
    print(f'\n完成 {ok}/7 台相机。点 Script Editor Run 后出图; ROS2 端 ros2 topic list 检查。')


def main():
    # 不用 argparse: Script Editor 里 sys.argv 是 Isaac 自己的启动参数,
    # argparse 会报 unrecognized arguments 直接退出。手动解析标志。
    argv = sys.argv[1:] if len(sys.argv) > 1 else []
    urdf = URDF
    for a in argv:
        if a.startswith('--urdf='):
            urdf = a.split('=', 1)[1]
    _ARGS['intrinsics'] = '--intrinsics' in argv
    _ARGS['urdf'] = urdf
    if '--check' in argv:
        cams = parse_cameras(urdf)
        print(f'{"相机":24s} {"型号":12s} {"父link":18s} xyz(m) / rpy(deg)')
        for c in cams:
            params = CAMERA_PARAMS[c['model']]
            f_mm, _oh, _ov, _fx, src = pinhole_from_params(
                c['name'], c['model'], params, use_intrinsics=_ARGS['intrinsics'])
            print(f'{c["name"]:24s} {c["model"]:12s} {c["parent"]:18s} '
                  f'({c["xyz"][0]:+.4f},{c["xyz"][1]:+.4f},{c["xyz"][2]:+.4f}) '
                  f'({c["rpy"][0]/D2R:+.2f},{c["rpy"][1]/D2R:+.2f},{c["rpy"][2]/D2R:+.2f})')
            print(f'{"":24s} → {params["resolution"][0]}×{params["resolution"][1]} '
                  f'焦距 {f_mm:.2f}mm | {src}')
        if _ARGS['intrinsics'] and not INTRINSICS:
            print('\n⚠ --intrinsics 已启用但 INTRINSICS 表为空: 全部回退 hfov 标称值。'
                  '\n  拿到标定数据后填 INTRINSICS[相机名] = dict(fx=, fy=, cx=, cy=)')
        return
    main_isaac()


if __name__ == '__main__':
    main()

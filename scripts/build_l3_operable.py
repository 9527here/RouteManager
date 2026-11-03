#!/usr/bin/env python3
"""将 L3 外观重建成参数化电气机构；全部未实测参数随包标注。"""
import argparse
import hashlib
import json
import math
import shutil
from pathlib import Path

import numpy as np
import trimesh
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

PROFILE = {
    'physx_solver': 'TGS',
    'force_calibration_status': '参考弹簧参数；TGS力-行程存在数值偏差，不能当作真实按压力标定',
    'calibrated_to_user_hardware': False,
    'authorization': '用户授权先按参考规格做可操作版，参数注明待实测',
    'button': {'mount_diameter_m': .022, 'bezel_diameter_m': .030,
               'stroke_m': .0043, 'switch_travel_m': .0026,
               'switch_force_N': 3.8, 'preload_N': 1., 'damping_Ns_m': 3.,
               'mass_kg': .080, 'cap_mass_kg': .015, 'cap_diameter_m': .022,
               'data_sheet_fields': ['mount_diameter_m', 'bezel_diameter_m', 'stroke_m',
                                     'switch_travel_m', 'switch_force_N', 'mass_kg'],
               'engineering_assumptions': ['cap_diameter_m', 'cap_mass_kg', 'preload_N', 'damping_Ns_m'],
               'reference': 'Schneider XB4BA31 数据表；作为参考而非 L3 型号认定',
               'source': 'https://www.coelpuebla.com.mx/files/producto/f_109_descargas_9h0n2ctyc_734.pdf'},
    'selector': {'panel_width_m': .048, 'bare_handle_height_m': .032,
                 'base_mass_kg': .065, 'handle_mass_kg': .018,
                 'two_positions_deg': [0., 90.], 'three_positions_deg': [-45., 0., 45.],
                 'detent_stiffness_Nm_rad': .08, 'damping_Nms_rad': .0015,
                 'max_resisting_torque_Nm': .08,
                 'reference': '90°/±45°保持形式参考 Harmony；尺寸/阻力为待实测工程参考值',
                 'source': 'https://www.se.com/ca/en/product/XB4BK123M5/'},
    'plate': {'height_m': .080, 'screw_pitch_m': .001, 'screw_diameter_m': .006,
              'base_mass_kg': .065, 'carrier_mass_kg': .004, 'nut_mass_kg': .012, 'bridge_mass_kg': .015,
              'release_turns': 2., 'max_turns': 2.5, 'swing_deg': 90.,
              'axial_stiffness_N_m': 20000., 'axial_damping_Ns_m': 5.,
              'screw_damping_Nms_rad': .001, 'swing_damping_Nms_rad': .001,
              'reference': 'M6x1等效螺旋副+80mm外形为工程参考值，非已确认厂家型号',
              'thread_model': '等效角度-轴移耦合；没有真实螺纹牙接触仿真'},
    'contact': {'static_friction': .6, 'dynamic_friction': .5, 'restitution': 0.,
                'contact_offset_m': .0003, 'rest_offset_m': 0.,
                'reference': '干燥塑料/手指垫的工程参考值，非实测'},
}


def components(mesh):
    _, inverse = np.unique(np.round(mesh.vertices, 7), axis=0, return_inverse=True)
    faces = inverse[mesh.faces]
    edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    n = inverse.max() + 1
    count, labels = connected_components(coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])),
                                                    shape=(n, n)), directed=False)
    return [np.flatnonzero(labels[faces[:, 0]] == k) for k in range(count)]


def load_mesh(path):
    scene = trimesh.load(path, force='scene', process=False)
    if len(scene.geometry) != 1:
        raise RuntimeError('本工具仅处理已审计的单 Mesh GLB')
    return next(iter(scene.geometry.values()))


class Builder:
    def __init__(self, output, name):
        self.name = name
        self.path = output / (name + '.usda')
        self.stage = Usd.Stage.CreateNew(str(self.path))
        self.root = '/Asset'
        self.stage.SetDefaultPrim(UsdGeom.Xform.Define(self.stage, self.root).GetPrim())
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.)
        UsdGeom.SetStageUpAxis(self.stage, 'Z')
        material = UsdShade.Material.Define(self.stage, self.root + '/Looks/Contact')
        physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        for key in ['static_friction', 'dynamic_friction', 'restitution']:
            getattr(physics, 'Create' + ''.join(x.capitalize() for x in key.split('_')) + 'Attr')().Set(PROFILE['contact'][key])
        self.contact_material = material
        self.bodies = {}
        self.joints = []
        self.shader_paths = {}
        self.body('Base', mass=.065)
        base = self.stage.GetPrimAtPath(self.root + '/Base')
        UsdPhysics.ArticulationRootAPI.Apply(base)
        base.AddAppliedSchema('PhysxArticulationAPI')
        base.CreateAttribute('physxArticulation:enabledSelfCollisions', Sdf.ValueTypeNames.Bool).Set(False)
        joint = UsdPhysics.FixedJoint.Define(self.stage, self.root + '/Joints/Mount')
        joint.CreateBody0Rel().SetTargets([Sdf.Path(self.root)])
        joint.CreateBody1Rel().SetTargets([base.GetPath()])

    def body(self, name, position=(0, 0, 0), mass=.01, com=(0, 0, 0)):
        prim = UsdGeom.Xform.Define(self.stage, self.root + '/' + name).GetPrim()
        UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*position))
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr(mass)
        mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*com))
        # 小零件惯量正定，数量级按参考包络；后续参数文件可用实测替换。
        mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(max(mass * .01**2, 1e-8)))
        prim.AddAppliedSchema('PhysxRigidBodyAPI')
        prim.CreateAttribute('physxRigidBody:disableGravity', Sdf.ValueTypeNames.Bool).Set(False)
        prim.CreateAttribute('physxRigidBody:solverPositionIterationCount', Sdf.ValueTypeNames.Int).Set(64)
        prim.CreateAttribute('physxRigidBody:solverVelocityIterationCount', Sdf.ValueTypeNames.Int).Set(16)
        self.bodies[name] = np.asarray(position, dtype=float)
        return prim

    def material(self, name, texture=None, color=(.2, .2, .2), metal=0., emissive=False):
        path = self.root + '/Looks/' + name
        material = UsdShade.Material.Define(self.stage, path)
        shader = UsdShade.Shader.Define(self.stage, path + '/Surface')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.45)
        shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(metal)
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput('emissiveColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0))
        if texture:
            reader = UsdShade.Shader.Define(self.stage, path + '/UV')
            reader.CreateIdAttr('UsdPrimvarReader_float2')
            reader.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
            reader.CreateOutput('result', Sdf.ValueTypeNames.Float2)
            tex = UsdShade.Shader.Define(self.stage, path + '/Texture')
            tex.CreateIdAttr('UsdUVTexture')
            tex.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(texture))
            tex.CreateInput('sourceColorSpace', Sdf.ValueTypeNames.Token).Set('sRGB')
            tex.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), 'result')
            tex.CreateOutput('rgb', Sdf.ValueTypeNames.Float3)
            shader.GetInput('diffuseColor').ConnectToSource(tex.ConnectableAPI(), 'rgb')
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        self.shader_paths[name] = str(shader.GetPath())
        return material

    def collision(self, prim, mesh=False):
        UsdPhysics.CollisionAPI.Apply(prim)
        if mesh:
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull')
        prim.AddAppliedSchema('PhysxCollisionAPI')
        prim.CreateAttribute('physxCollision:contactOffset', Sdf.ValueTypeNames.Float).Set(.0003)
        prim.CreateAttribute('physxCollision:restOffset', Sdf.ValueTypeNames.Float).Set(0.)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.contact_material, materialPurpose='physics')

    def mesh(self, body, name, points, faces, uv, material, collision=True):
        used, inverse = np.unique(faces.reshape(-1), return_inverse=True)
        points = points[used] - self.bodies[body]
        faces = inverse.reshape(-1, 3)
        mesh = UsdGeom.Mesh.Define(self.stage, f'{self.root}/{body}/{name}')
        mesh.CreatePointsAttr(points.astype(np.float32))
        mesh.CreateFaceVertexCountsAttr([3] * len(faces))
        mesh.CreateFaceVertexIndicesAttr(faces.reshape(-1).astype(np.int32))
        mesh.CreateSubdivisionSchemeAttr('none')
        mesh.CreateDoubleSidedAttr(True)
        if uv is not None:
            UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray,
                                                   UsdGeom.Tokens.vertex).Set(uv[used].astype(np.float32))
        mesh.CreateExtentAttr([Gf.Vec3f(*points.min(0)), Gf.Vec3f(*points.max(0))])
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
        if collision:
            self.collision(mesh.GetPrim(), True)
        return mesh

    def cylinder(self, body, name, position, radius, length, material, collision=True):
        geom = UsdGeom.Cylinder.Define(self.stage, f'{self.root}/{body}/{name}')
        geom.CreateAxisAttr('X')
        geom.CreateRadiusAttr(radius)
        geom.CreateHeightAttr(length)
        UsdGeom.Xformable(geom).AddTranslateOp().Set(Gf.Vec3d(*(np.asarray(position) - self.bodies[body])))
        UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(material)
        if collision:
            self.collision(geom.GetPrim())
        return geom

    def box(self, body, name, position, size, material, collision=True):
        geom = UsdGeom.Cube.Define(self.stage, f'{self.root}/{body}/{name}')
        geom.CreateSizeAttr(1.)
        xf = UsdGeom.Xformable(geom)
        xf.AddTranslateOp().Set(Gf.Vec3d(*(np.asarray(position) - self.bodies[body])))
        xf.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(material)
        if collision:
            self.collision(geom.GetPrim())
        return geom

    def joint(self, name, parent, child, pivot, axis, limits, stiffness=0., damping=0., max_force=100.):
        path = self.root + '/Joints/' + name
        linear = axis == 'linear'
        joint = (UsdPhysics.PrismaticJoint if linear else UsdPhysics.RevoluteJoint).Define(self.stage, path)
        joint.CreateAxisAttr('X')
        joint.CreateBody0Rel().SetTargets([Sdf.Path(self.root + '/' + parent)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(self.root + '/' + child)])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*(np.asarray(pivot) - self.bodies[parent])))
        joint.CreateLocalPos1Attr(Gf.Vec3f(*(np.asarray(pivot) - self.bodies[child])))
        joint.CreateLowerLimitAttr(limits[0] if linear else math.degrees(limits[0]))
        joint.CreateUpperLimitAttr(limits[1] if linear else math.degrees(limits[1]))
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), 'linear' if linear else 'angular')
        drive.CreateTypeAttr('force')
        factor = 1. if linear else math.pi / 180.
        drive.CreateStiffnessAttr(stiffness * factor)
        drive.CreateDampingAttr(damping * factor)
        drive.CreateMaxForceAttr(max_force)
        drive.CreateTargetPositionAttr(0.)
        self.joints.append({'name': name, 'type': 'prismatic' if linear else 'revolute',
                            'limits': list(limits), 'stiffness': stiffness, 'damping': damping})
        return drive


def canonical(mesh, scale=1.):
    # L3 顶点实际 Z 竖直、+Y 正面；转到项目的 +X 正面，保持右手坐标。
    return np.asarray(mesh.vertices)[:, [1, 0, 2]] * [scale, -scale, scale]


def make_button(builder, mesh, texture, exploded=False):
    b = builder
    p = PROFILE['button']
    cap_d, bezel_d = p['cap_diameter_m'], p['bezel_diameter_m']
    UsdPhysics.MassAPI(b.stage.GetPrimAtPath(b.root + '/Base')).GetMassAttr().Set(p['mass_kg'] - p['cap_mass_kg'])
    dark = b.material('Bezel', color=(.08, .08, .08), metal=.65)
    fixed = b.material('FixedTexture', texture=texture)
    moving = b.material('MovingTexture', texture=texture)
    b.cylinder('Base', 'MountBody', [-.008, 0, 0], p['mount_diameter_m'] / 2, .024, dark)
    b.cylinder('Base', 'Bezel', [.006, 0, 0], bezel_d / 2, .005, dark)
    b.body('Cap', mass=p['cap_mass_kg'], com=(.013, 0, 0))
    UsdPhysics.MassAPI(b.stage.GetPrimAtPath(b.root + '/Cap')).GetDiagonalInertiaAttr().Set(
        Gf.Vec3f(.5 * p['cap_mass_kg'] * (cap_d/2)**2, p['cap_mass_kg'] * (3 * (cap_d/2)**2 + .006**2) / 12,
                 p['cap_mass_kg'] * (3 * (cap_d/2)**2 + .006**2) / 12))
    vertices = canonical(mesh)
    faces = mesh.faces
    if exploded:
        parts = components(mesh)
        # 该源的三个分散部件沿 Z 排列；最下方绿圆头，其余归为固定装配。
        parts.sort(key=lambda ix: mesh.vertices[np.unique(faces[ix])][:, 2].mean())
        cap_faces = faces[parts[0]]
        cap_used = np.unique(cap_faces)
        lo, hi = vertices[cap_used].min(0), vertices[cap_used].max(0)
        cap = (vertices - (lo + hi) / 2) * ([.006, cap_d, cap_d] / (hi - lo)) + [.013, 0, 0]
        b.mesh('Cap', 'Visual', cap, cap_faces, mesh.visual.uv, moving, False)
        for k, indices in enumerate(parts[1:]):
            ff = faces[indices]
            used = np.unique(ff)
            lo, hi = vertices[used].min(0), vertices[used].max(0)
            target = [-.012 - k * .012, 0, 0]
            dims = [.012, .026, .026] if k else [.003, .03, .03]
            vv = (vertices - (lo + hi) / 2) * (np.asarray(dims) / np.maximum(hi - lo, 1e-6)) + target
            b.mesh('Base', f'SourcePart{k}', vv, ff, mesh.visual.uv, fixed, False)
    else:
        lo, hi = vertices.min(0), vertices.max(0)
        split = lo[0] + .45 * (hi[0] - lo[0])
        selected = vertices[faces].mean(1)[:, 0] >= split
        if not selected.any() or selected.all():
            raise RuntimeError('按钮未能拆分固定/活动面')
        cap_faces = faces[selected]
        used = np.unique(cap_faces)
        clo, chi = vertices[used].min(0), vertices[used].max(0)
        cap = (vertices - (clo + chi) / 2) * ([.006, cap_d, cap_d] / (chi - clo)) + [.013, 0, 0]
        b.mesh('Cap', 'Visual', cap, cap_faces, mesh.visual.uv, moving, False)
        back = (vertices - (lo + hi) / 2) * ([.012, bezel_d, bezel_d] / (hi - lo))
        b.mesh('Base', 'SourceBezel', back, faces[~selected], mesh.visual.uv, fixed, False)
    proxy = b.cylinder('Cap', 'ContactShape', [.013, 0, 0], cap_d / 2, .006, moving)
    proxy.CreateVisibilityAttr('invisible')
    spring = (p['switch_force_N'] - p['preload_N']) / p['switch_travel_m']
    drive = b.joint('button_slide', 'Base', 'Cap', [0, 0, 0], 'linear', [-p['stroke_m'], 0.], spring, p['damping_Ns_m'], 20.)
    drive.GetTargetPositionAttr().Set(p['preload_N'] / spring)
    light = UsdLux.SphereLight.Define(b.stage, b.root + '/Cap/FeedbackLight')
    light.CreateRadiusAttr(.005)
    light.CreateIntensityAttr(0.)
    UsdGeom.Xformable(light).AddTranslateOp().Set(Gf.Vec3d(.018, 0, 0))
    return {'kind': 'button', 'indicator_shader': b.shader_paths['MovingTexture'],
            'indicator_light': str(light.GetPath()), 'surface_local': [.016, 0, 0],
            'reference_dimensions': [.030, .030, .04]}


def make_selector(builder, mesh, texture, full_panel=False, three=False):
    b = builder
    p = PROFILE['selector']
    UsdPhysics.MassAPI(b.stage.GetPrimAtPath(b.root + '/Base')).GetMassAttr().Set(p['base_mass_kg'])
    fixed = b.material('FixedTexture', texture=texture)
    moving = b.material('MovingTexture', texture=texture)
    dark = b.material('MountMaterial', color=(.1, .1, .1), metal=.4)
    vertices = canonical(mesh)
    faces = mesh.faces
    lo, hi = vertices.min(0), vertices.max(0)
    if full_panel:
        scale = p['panel_width_m'] / max(hi[1] - lo[1], hi[2] - lo[2])
        vertices *= scale
        # 从贴图白色面板采样确定面板表面，不把面板跟着旋钮转。
        image = np.asarray(mesh.visual.material.baseColorTexture.convert('RGB'))
        uv = mesh.visual.uv
        pix = image[np.clip(((1 - uv[:, 1]) * (image.shape[0] - 1)).astype(int), 0, image.shape[0] - 1),
                    np.clip((uv[:, 0] * (image.shape[1] - 1)).astype(int), 0, image.shape[1] - 1)]
        white = (pix.min(1) > 110) & (pix.max(1) - pix.min(1) < 75)
        plane = np.quantile(vertices[white, 0], .85) if white.sum() > 20 else np.quantile(vertices[:, 0], .55)
        selected = vertices[faces].mean(1)[:, 0] > plane + .0005
        if selected.sum() < 80 or (~selected).sum() < 80:
            raise RuntimeError('面板/旋钮拆分失败，不能整件转动')
        axis = np.array([plane, (vertices[:, 1].min() + vertices[:, 1].max()) / 2,
                         (vertices[:, 2].min() + vertices[:, 2].max()) / 2])
        vertices -= axis
        b.mesh('Base', 'PanelVisual', vertices, faces[~selected], uv, fixed)
        active_faces = faces[selected]
    else:
        scale = p['bare_handle_height_m'] / (hi[2] - lo[2])
        vertices *= scale
        lo, hi = vertices.min(0), vertices.max(0)
        # 圆盘上沿减半宽确定轴心；延伸的柄不应拉低旋转中心。
        axis = [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2,
                hi[2] - (hi[1] - lo[1]) / 2]
        vertices -= axis
        active_faces = faces
        b.cylinder('Base', 'Collar', [-.006, 0, 0], .012, .006, dark)
    b.body('Handle', mass=p['handle_mass_kg'])
    b.mesh('Handle', 'VisualContact', vertices, active_faces, mesh.visual.uv, moving)
    solid = trimesh.Trimesh(vertices=vertices, faces=active_faces, process=False).convex_hull
    inertial = UsdPhysics.MassAPI(b.stage.GetPrimAtPath(b.root + '/Handle'))
    inertial.GetCenterOfMassAttr().Set(Gf.Vec3f(*solid.center_mass))
    # 惯量采用包络凸体推算，质量仍为显式参考值；不从未封闭的源网格取体积。
    inertia = solid.moment_inertia * (p['handle_mass_kg'] / max(solid.mass, 1e-10))
    inertial.GetDiagonalInertiaAttr().Set(Gf.Vec3f(*np.maximum(np.diag(inertia), 1e-8)))
    detents = p['three_positions_deg'] if three else p['two_positions_deg']
    limits = np.radians([min(detents), max(detents)])
    b.joint('selector_turn', 'Base', 'Handle', [0, 0, 0], 'angular', limits,
            p['detent_stiffness_Nm_rad'], p['damping_Nms_rad'], p['max_resisting_torque_Nm'])
    return {'kind': 'selector', 'detents_deg': detents,
            'segmentation': 'texture-guided front plane' if full_panel else 'whole independent handle',
            'reference_dimensions': (vertices.max(0) - vertices.min(0)).tolist()}


def make_plate(builder, mesh, texture, template):
    from scipy.spatial import cKDTree
    b = builder
    p = PROFILE['plate']
    UsdPhysics.MassAPI(b.stage.GetPrimAtPath(b.root + '/Base')).GetMassAttr().Set(p['base_mass_kg'])
    plastic = b.material('InsulatorTexture', texture=texture)
    copper = b.material('Conductor', color=(.55, .24, .09), metal=.8)
    vertices = canonical(mesh)
    # 黄色源是横排；转成竖排参考机构，纹理/顶点同步旋转。
    if b.name.endswith('111'):
        vertices[:, [1, 2]] = vertices[:, [2, 1]] * [-1, 1]
    vertices *= p['height_m'] / np.ptp(vertices[:, 2])
    ref_mesh, ref_parts = template
    ref = canonical(ref_mesh) * (p['height_m'] / np.ptp(ref_mesh.vertices[:, 2]))
    ref_centers = ref[ref_mesh.faces].mean(1)
    labels = np.zeros(len(ref_mesh.faces), dtype=int)
    # 最大/靠背的部分是底座；余下按 Z 位置得到上下螺帽与中间手持块。
    ref_parts.sort(key=lambda ix: len(ix), reverse=True)
    base = ref_parts[0]
    rest = sorted(ref_parts[1:], key=lambda ix: ref[ref_mesh.faces[ix]].mean((0, 1))[2])
    assignments = [base, rest[-1], rest[0], rest[1]]
    for k, indices in enumerate(assignments):
        labels[indices] = k
    if b.name.endswith('14'):
        face_labels = labels
        segmentation = '4 position-connected components; UV preserved'
    else:
        # 无分件版本只作参考外观迁移，利用14的空间分区重建角色，记录近似。
        normalized = vertices.copy()
        for axis in (0, 1):
            normalized[:, axis] *= np.ptp(ref[:, axis]) / max(np.ptp(vertices[:, axis]), 1e-6)
        _, nearest = cKDTree(ref_centers).query(normalized[mesh.faces].mean(1))
        face_labels = labels[nearest]
        segmentation = 'geometry transfer from object_14; approximate reconstructed parts'
    roles = {}
    for k, name in enumerate(['Base', 'Upper', 'Lower', 'Bridge']):
        ff = mesh.faces[face_labels == k]
        if len(ff) < 20:
            raise RuntimeError(f'{b.name} {name} 未形成有效分件')
        roles[name] = ff
    pivots = {}
    for name in ['Upper', 'Lower']:
        pts = vertices[np.unique(roles[name])]
        center = (pts.min(0) + pts.max(0)) / 2
        pivots[name] = np.array([0., center[1], center[2]])
    # 视觉/碰撞均使用实际拆出的部件，轴移载体是等效螺旋副，不假造螺纹网格。
    b.mesh('Base', 'Insulator', vertices, roles['Base'], mesh.visual.uv, plastic)
    for name in ['Upper', 'Lower']:
        pivot = pivots[name]
        b.body(name + 'Slide', pivot, p['carrier_mass_kg'])
        b.body(name + 'Nut', pivot, p['nut_mass_kg'])
        b.mesh(name + 'Nut', 'KnurledNut', vertices, roles[name], mesh.visual.uv, plastic)
        low = name.lower()
        b.joint(low + '_slide', 'Base', name + 'Slide', pivot, 'linear', [0., p['max_turns'] * p['screw_pitch_m']],
                p['axial_stiffness_N_m'], p['axial_damping_Ns_m'], 200.)
        b.joint(low + '_turn', name + 'Slide', name + 'Nut', pivot, 'angular', [0., p['max_turns'] * 2 * math.pi],
                0., p['screw_damping_Nms_rad'], 1.)
    pivot = pivots['Lower']
    b.body('Bridge', pivot, p['bridge_mass_kg'])
    b.mesh('Bridge', 'InsulatedGrip', vertices, roles['Bridge'], mesh.visual.uv, plastic)
    pitch = float(pivots['Upper'][2] - pivots['Lower'][2])
    inertia = UsdPhysics.MassAPI(b.stage.GetPrimAtPath(b.root + '/Bridge'))
    inertia.GetCenterOfMassAttr().Set(Gf.Vec3f(.006, 0, pitch / 2))
    inertia.GetDiagonalInertiaAttr().Set(Gf.Vec3f(p['bridge_mass_kg'] * (pitch**2 + .009**2) / 12,
                                                p['bridge_mass_kg'] * (pitch**2 + .003**2) / 12,
                                                p['bridge_mass_kg'] * (.009**2 + .003**2) / 12))
    b.box('Bridge', 'RebuiltConductor', [.003, float(pivot[1]), float(pivot[2] + pitch / 2)], [.002, .009, pitch], copper)
    b.joint('bridge_swing', 'Base', 'Bridge', pivot, 'angular', [0., math.radians(p['swing_deg'])], 0., p['swing_damping_Nms_rad'], 2.)
    return {'kind': 'plate', 'segmentation': segmentation,
            'conductor': 'rebuilt reference bridge; source central red part retained as insulated grip',
            'pivot': pivot.tolist(), 'stud_pitch_m': pitch,
            'reference_dimensions': (vertices.max(0) - vertices.min(0)).tolist()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', type=Path, help='实测到位后用校准JSON重建；不要在活动场景加载时覆盖USD')
    args = parser.parse_args()
    source, output = args.source.resolve(), args.output.resolve()
    if args.profile:
        updated = json.loads(args.profile.read_text())
        for section, values in updated.items():
            if isinstance(values, dict) and isinstance(PROFILE.get(section), dict):
                PROFILE[section].update(values)
            else:
                PROFILE[section] = values
    output.mkdir(parents=True, exist_ok=True)
    (output / 'source').mkdir(exist_ok=True)
    (output / 'textures').mkdir(exist_ok=True)
    (output / 'reference_profile.json').write_text(json.dumps(PROFILE, ensure_ascii=False, indent=2) + '\n')
    template_mesh = load_mesh(source / 'object_14.glb')
    template = (template_mesh, components(template_mesh))
    buttons = ['object_0', 'object_0 (1)', 'object_1', 'object_2']
    selectors = ['object_3', 'object_4', 'object_5', 'object_6', 'object_6 (1)', 'object_19']
    plates = ['object_13', 'object_14', 'object_15', 'object_16', 'object_111']
    items = []
    for stem in buttons + selectors + plates:
        name = stem.replace(' (1)', '_variant')
        path = source / (stem + '.glb')
        mesh = load_mesh(path)
        shutil.copy2(path, output / 'source' / path.name)
        texture_name = name + '.png'
        mesh.visual.material.baseColorTexture.save(output / 'textures' / texture_name)
        texture = './textures/' + texture_name
        builder = Builder(output, name)
        if stem in buttons:
            result = make_button(builder, mesh, texture, exploded=stem == 'object_0')
        elif stem in selectors:
            result = make_selector(builder, mesh, texture, full_panel=stem.startswith('object_6') or stem == 'object_19',
                                   three=stem in ['object_4', 'object_5', 'object_19'])
        else:
            result = make_plate(builder, mesh, texture, template)
        builder.stage.GetRootLayer().Save()
        result.update(id=name, usd=builder.path.name, root_prim=builder.root, joints=builder.joints,
                      source_file=path.name, source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                      calibrated_to_user_hardware=False, robot_contact_verified=False)
        items.append(result)
        print('BUILT', name, result['kind'], len(result['joints']), flush=True)
    (output / 'catalog.json').write_text(json.dumps(items, ensure_ascii=False, indent=2) + '\n')
    # 固定场景参考姿态 +X朝前，控制台安装时整体绕Z旋转即可。
    gallery = Usd.Stage.CreateNew(str(output / 'gallery.usda'))
    world = UsdGeom.Xform.Define(gallery, '/World')
    gallery.SetDefaultPrim(world.GetPrim())
    UsdGeom.SetStageMetersPerUnit(gallery, 1.)
    UsdGeom.SetStageUpAxis(gallery, 'Z')
    for index, item in enumerate(items):
        prim = UsdGeom.Xform.Define(gallery, '/World/Controls/' + item['id']).GetPrim()
        prim.GetReferences().AddReference('./' + item['usd'])
        position = [0., (index % 5 - 2) * .18, 1.25 - (index // 5) * .24]
        UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*position))
        item['gallery_prim'] = str(prim.GetPath())
        item['gallery_position'] = position
    UsdLux.DomeLight.Define(gallery, '/World/Dome').CreateIntensityAttr(300.)
    sun = UsdLux.DistantLight.Define(gallery, '/World/Sun')
    sun.CreateIntensityAttr(1200.)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-25, 30, 0))
    gallery.GetRootLayer().Save()
    (output / 'catalog.json').write_text(json.dumps(items, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    main()

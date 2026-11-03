#!/usr/bin/env python3
"""AST提取地面材质方法的纯mock回归；不导入/实例化RobotManager或Isaac。

绑定替身只实现本缺陷涉及的purpose优先与strongerThanDescendants规则。
真实ComputeBoundMaterial已由现场只读证据验证；本测试防止source退回generic-only。
"""

from _bootstrap import ROOT as _TEST_ROOT  # 统一离线测试的项目/工具导入路径
import ast
import copy
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock


REPO = Path(__file__).resolve().parents[1]
DESIRED = '/World/PhysicsMaterials/friction_mat'
DEFAULT = '/World/Physics_Materials/physics_material'
FLOORS = ('/World/ground_plane', '/World/ground_plane/geom',
          '/World/ground_plane/collisionPlane', '/World/ground_plane/col')
STRONG = 'strongerThanDescendants'


class Prim:
    def __init__(self, path, valid=True):
        self.path, self.valid, self.bindings, self.attributes = path, valid, {}, {}

    def IsValid(self):
        return self.valid


class Stage:
    def __init__(self, valid_floors=FLOORS):
        self.prims = {path: Prim(path) for path in valid_floors}
        self.prims[DEFAULT] = Prim(DEFAULT)
        self.prims[DEFAULT].attributes = {'staticFriction': .5, 'dynamicFriction': .5, 'restitution': .8}
        self.calls = []

    def GetPrimAtPath(self, path):
        return self.prims.get(path, Prim(path, False))

    def resolved_physics_material(self, path):
        # 本bug中purpose-specific绑定优先于generic；祖先强绑定覆盖后代。
        ancestors = ['/'.join(path.split('/')[:i]) for i in range(2, len(path.split('/')) + 1)]
        for purpose in ('physics', ''):
            winner = None
            for ancestor in ancestors:
                prim = self.GetPrimAtPath(ancestor)
                binding = prim.bindings.get(purpose)
                if binding is not None:
                    winner = binding[0]
                    if binding[1] == STRONG:
                        return winner
            if winner is not None:
                return winner
        return None


def source_method(legacy_generic_only=False):
    tree = ast.parse((REPO / 'robot_manager.py').read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'RobotManager')
    method = copy.deepcopy(next(node for node in cls.body if isinstance(node, ast.FunctionDef)
                                and node.name == '_ensure_friction_material'))
    if legacy_generic_only:
        class RemovePurposeBind(ast.NodeTransformer):
            def visit_Expr(self, node):
                call = node.value
                if (isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
                        and call.func.attr == 'Bind' and any(k.arg == 'materialPurpose' for k in call.keywords)):
                    return None
                return self.generic_visit(node)
        method = RemovePurposeBind().visit(method)
    module = ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[]))
    namespace = {'logger': SimpleNamespace(info=lambda *args: None)}
    exec(compile(module, str(REPO / 'robot_manager.py'), 'exec'), namespace)
    return namespace['_ensure_friction_material']


def invoke(stage, method):
    class Material:
        def __init__(self, prim):
            self.prim = prim

        @staticmethod
        def Define(given_stage, path):
            return Material(given_stage.prims.setdefault(path, Prim(path)))

        def GetPrim(self):
            return self.prim

    class MaterialAPI:
        @staticmethod
        def Apply(prim):
            return MaterialAPI(prim)

        def __init__(self, prim):
            self.prim = prim

        def CreateStaticFrictionAttr(self, value):
            self.prim.attributes['staticFriction'] = value

        def CreateDynamicFrictionAttr(self, value):
            self.prim.attributes['dynamicFriction'] = value

        def CreateRestitutionAttr(self, value):
            self.prim.attributes['restitution'] = value

    class Binding:
        @staticmethod
        def Apply(prim):
            return Binding(prim)

        def __init__(self, prim):
            self.prim = prim

        def Bind(self, material, bindingStrength='weakerThanDescendants', materialPurpose=''):
            path = material.GetPrim().path
            self.prim.bindings[materialPurpose] = path, bindingStrength
            stage.calls.append({'prim': self.prim.path, 'material': path,
                                'purpose': materialPurpose, 'strength': bindingStrength})

    omni, usd, pxr = ModuleType('omni'), ModuleType('omni.usd'), ModuleType('pxr')
    usd.get_context = lambda: SimpleNamespace(get_stage=lambda: stage)
    omni.usd = usd
    pxr.Usd = SimpleNamespace()
    pxr.UsdShade = SimpleNamespace(Material=Material, MaterialBindingAPI=Binding,
                                  Tokens=SimpleNamespace(strongerThanDescendants=STRONG))
    pxr.UsdPhysics = SimpleNamespace(MaterialAPI=MaterialAPI)
    with mock.patch.dict(sys.modules, {'omni': omni, 'omni.usd': usd, 'pxr': pxr}), \
            mock.patch('urllib.request.urlopen', side_effect=AssertionError('禁止 HTTP')):
        method(SimpleNamespace(_FRICTION=1.0))


class FloorPhysicsMaterialRegression(unittest.TestCase):
    def fixture(self):
        stage = Stage()
        geom = stage.prims['/World/ground_plane/geom']
        geom.bindings['physics'] = DEFAULT, STRONG
        deep = Prim('/World/ground_plane/geom/collisionPlane')
        deep.bindings[''] = DESIRED, 'weakerThanDescendants'
        stage.prims[deep.path] = deep
        return stage

    def test_legacy_generic_only_reproduces_physics_purpose_priority_bug(self):
        stage = self.fixture()
        invoke(stage, source_method(legacy_generic_only=True))
        self.assertEqual(stage.prims['/World/ground_plane/geom'].bindings[''][0], DESIRED)
        self.assertEqual(stage.resolved_physics_material('/World/ground_plane/geom'), DEFAULT)
        self.assertEqual(stage.resolved_physics_material('/World/ground_plane/geom/collisionPlane'), DEFAULT)
        self.assertEqual(stage.prims[DEFAULT].attributes['restitution'], .8)

    def test_current_source_strong_physics_binding_covers_each_valid_floor(self):
        stage = self.fixture()
        default_before = copy.deepcopy(stage.prims[DEFAULT].attributes)
        invoke(stage, source_method())
        for path in FLOORS:
            with self.subTest(path=path):
                self.assertEqual(stage.prims[path].bindings['physics'], (DESIRED, STRONG))
                self.assertEqual(stage.prims[path].bindings[''][0], DESIRED)
                self.assertEqual(stage.resolved_physics_material(path), DESIRED)
        self.assertEqual(stage.resolved_physics_material('/World/ground_plane/geom/collisionPlane'), DESIRED)
        self.assertEqual(stage.prims[DESIRED].attributes,
                         {'staticFriction': 1., 'dynamicFriction': 1., 'restitution': 0.})
        self.assertEqual(stage.prims[DEFAULT].attributes, default_before)
        self.assertEqual(len([call for call in stage.calls if call['purpose'] == 'physics']), 4)

    def test_missing_floor_is_skipped_without_touching_unrelated_robot(self):
        stage = Stage(valid_floors=('/World/ground_plane/geom',))
        robot = Prim('/humanoid/Geometry/base_link')
        robot.bindings['physics'] = DEFAULT, STRONG
        stage.prims[robot.path] = robot
        invoke(stage, source_method())
        self.assertEqual({call['prim'] for call in stage.calls}, {'/World/ground_plane/geom'})
        self.assertEqual(robot.bindings['physics'], (DEFAULT, STRONG))

    def test_repeated_source_call_is_idempotent_and_keeps_generic_compatibility(self):
        stage = self.fixture()
        method = source_method()
        invoke(stage, method)
        paths = set(stage.prims)
        state = {p: copy.deepcopy(stage.prims[p].bindings) for p in FLOORS}
        invoke(stage, method)
        self.assertEqual(set(stage.prims), paths)
        self.assertEqual({p: stage.prims[p].bindings for p in FLOORS}, state)


if __name__ == '__main__':
    unittest.main(verbosity=2)

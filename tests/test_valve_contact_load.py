"""纯mock核验6.2 ContactData承载语义；不导入Kit或连接运行中仿真。"""

from _bootstrap import ROOT as _TEST_ROOT  # 统一离线测试的项目/工具导入路径
import json
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
import valve_feedback as feedback


WHEEL = '/World/SyntheticValve/wheel_link'
ROBOT = '/humanoid'
LEFT = ROBOT + '/right_gripper_left_pad_link'
RIGHT = ROBOT + '/right_gripper_right_pad_link'
BACKING = ROBOT + '/right_gripper_left_in_link'
PATHS = {1: WHEEL, 2: LEFT, 3: RIGHT, 4: BACKING}


class Prim:
    def __init__(self, path, reports=False):
        self.path = path
        self.reports = reports

    def GetPath(self):
        return self.path

    def GetName(self):
        return self.path.rsplit('/', 1)[-1]

    def HasAPI(self, schema):
        return self.reports


class Stage:
    def __init__(self, wheel_reports=True):
        self.prims = {path: Prim(path, path == WHEEL and wheel_reports) for path in PATHS.values()}

    def GetPrimAtPath(self, path):
        return self.prims[path]

    def Traverse(self):
        return iter(self.prims.values())


def datum(*, normal=(1., 0., 0.), impulse=(0., 0., 0.), separation=.00002):
    # 与安装版 ContactData 属性名一致，不模拟不存在的force scalar。
    return SimpleNamespace(position=(0., 0., 0.), normal=normal,
                           impulse=impulse, separation=separation)


def contact(actor, samples, offset):
    return SimpleNamespace(actor0=1, actor1=actor, collider0=1, collider1=actor,
                           num_contact_data=len(samples), contact_data_offset=offset)


def read_contacts(entries=(), *, failure=None, wheel_reports=True, as_dict=False):
    headers, data = [], []
    for actor, samples in entries:
        headers.append(contact(actor, samples, len(data)))
        data.extend(samples)
    value = {'contact_headers': headers, 'contact_data': data} if as_dict else (headers, data)
    getter = mock.Mock(side_effect=failure, return_value=value)
    omni = ModuleType('omni')
    omni.__path__ = []
    physx = ModuleType('omni.physx')
    physx.get_physx_simulation_interface = lambda: SimpleNamespace(get_contact_report=getter)
    omni.physx = physx
    pxr = ModuleType('pxr')
    pxr.PhysicsSchemaTools = SimpleNamespace(intToSdfPath=PATHS.__getitem__)
    pxr.PhysxSchema = SimpleNamespace(PhysxContactReportAPI=object())
    with mock.patch.dict(sys.modules, {'omni': omni, 'omni.physx': physx, 'pxr': pxr}):
        result = feedback._contacts(Stage(wheel_reports), WHEEL, ROBOT)
    # 报告不得把不可读NaN保存成非标准JSON number。
    json.dumps(result, allow_nan=False)
    return result, getter


class ContactLoadRegression(unittest.TestCase):
    def test_empty_success_is_available_but_load_is_unverified(self):
        result, getter = read_contacts()
        getter.assert_called_once_with()
        self.assertTrue(result['available'])
        self.assertTrue(result['read_succeeded'])
        self.assertFalse(result['load_verifiable'])
        self.assertNotIn('read_error', result)
        self.assertIn('absence_reason', result)
        self.assertIn('load_verification_note', result)
        self.assertFalse(result['left_pad_load_bearing'])
        self.assertFalse(result['right_pad_load_bearing'])

    def test_positive_separation_zero_impulse_patch_is_not_load(self):
        result, _ = read_contacts([(2, [datum(separation=.000013)]),
                                   (3, [datum(separation=.000041)])])
        self.assertTrue(result['available'])
        self.assertTrue(result['load_verifiable'])
        self.assertTrue(result['left_pad_touching'])
        self.assertTrue(result['right_pad_touching'])
        self.assertEqual(result['contact_count'], 2)
        self.assertEqual(result['impulse_sum_Ns'], 0.)
        for side in ('left', 'right'):
            self.assertEqual(result[side + '_pad_normal_impulse_Ns'], 0.)
            self.assertFalse(result[side + '_pad_load_bearing'])
        self.assertEqual(result['events'][1]['samples'][0]['separation_m'], .000041)

    def test_signed_normal_projection_uses_unit_normal_and_absolute_value(self):
        result, _ = read_contacts([(2, [datum(normal=(0., 2., 0.), impulse=(1., -.004, 0.))]),
                                   (3, [datum(normal=(0., 0., -3.), impulse=(0., 0., .002))])],
                                  as_dict=True)
        self.assertAlmostEqual(result['left_pad_normal_impulse_Ns'], .004)
        self.assertAlmostEqual(result['right_pad_normal_impulse_Ns'], .002)
        self.assertTrue(result['left_pad_load_bearing'])
        self.assertTrue(result['right_pad_load_bearing'])
        raw = result['events'][0]['samples'][0]
        self.assertEqual(raw['normal_world'], [0., 2., 0.])
        self.assertEqual(raw['impulse_Ns'], [1., -.004, 0.])
        self.assertIn('ContactData.normal', result['load_measurement_source']['normal'])

    def test_tangential_impulse_does_not_prove_normal_loading(self):
        result, _ = read_contacts([(2, [datum(impulse=(0., .005, 0.))])])
        self.assertEqual(result['impulse_sum_Ns'], .005)
        self.assertEqual(result['left_pad_normal_impulse_Ns'], 0.)
        self.assertFalse(result['left_pad_load_bearing'])

    def test_only_one_pad_loaded_cannot_prove_bilateral_loading(self):
        result, _ = read_contacts([(2, [datum(impulse=(.003, 0., 0.))]),
                                   (3, [datum()])])
        self.assertTrue(result['left_pad_load_bearing'])
        self.assertFalse(result['right_pad_load_bearing'])

    def test_missing_required_field_and_nonfinite_data_are_read_errors(self):
        samples = []
        for field in ('normal', 'impulse', 'separation'):
            item = datum()
            delattr(item, field)
            samples.append(item)
        samples.extend([datum(normal=(math.nan, 0., 0.)), datum(normal=(0., 0., 0.)),
                        datum(impulse=(math.inf, 0., 0.)), datum(separation=math.nan)])
        for item in samples:
            with self.subTest(item=repr(item)):
                # 即使同批有正常正载荷，不完整反馈也不得报loaded。
                result, _ = read_contacts([(2, [datum(impulse=(.001, 0., 0.)), item])])
                self.assertFalse(result['available'])
                self.assertFalse(result['read_succeeded'])
                self.assertFalse(result['load_verifiable'])
                self.assertIn('read_error', result)
                self.assertFalse(result['left_pad_load_bearing'])
                self.assertFalse(result['right_pad_load_bearing'])

    def test_nonpad_positive_normal_load_is_identified(self):
        result, _ = read_contacts([(4, [datum(impulse=(-.006, 0., 0.))])])
        self.assertTrue(result['unexpected_robot_load_bearing'])
        self.assertEqual(result['unexpected_robot_normal_impulse_Ns'], .006)
        self.assertFalse(result['left_pad_touching'])
        zero, _ = read_contacts([(4, [datum()])])
        self.assertFalse(zero['unexpected_robot_load_bearing'])

    def test_getter_exception_is_unavailable(self):
        result, _ = read_contacts(failure=RuntimeError('synthetic getter failure'))
        self.assertFalse(result['available'])
        self.assertFalse(result['read_succeeded'])
        self.assertIn('synthetic getter failure', result['read_error'])

    def test_unconfigured_report_does_not_call_native_getter(self):
        result, getter = read_contacts(wheel_reports=False)
        getter.assert_not_called()
        self.assertFalse(result['available'])
        self.assertFalse(result['load_verifiable'])
        self.assertEqual(result['absence_reason'], 'contact_report_api_not_configured')


if __name__ == '__main__':
    unittest.main(verbosity=2)

"""PhysX 原生按需唤醒；不会复制关节速度来“顺便唤醒”。

只能在 SimulationApp 已启动后的仿真主线程调用。此模块顶层不导入 Isaac/omni/pxr。
它只避免额外状态写入，不声称消除已存在的待机抖动。

直接运行 ``python scripts/physics_wake.py`` 只执行纯 mock 回归，不连接或启动仿真。
"""
import math


def wake_if_sleeping(executor):
    """验证当前 PhysX 刚体后，只有真正 sleep 才调用 native wake_up；异常只报告。"""
    report = {
        'supported': False, 'active_engine': None, 'stage_id': None,
        'body_path': None, 'body_id': None, 'body_verified': False,
        'sleep_before': None, 'sleep_after': None,
        'wake_attempted': False, 'wake_called': False,
        'state_setters_used': [], 'targets_or_gains_written': False,
        'method': 'IPhysxSimulation.is_sleeping/wake_up with current context stage and actual tensor base_link',
        'jitter_fix_claimed': False,
    }
    phase = 'active_engine'
    try:
        from isaacsim.core.simulation_manager import SimulationManager
        engine = SimulationManager.get_active_physics_engine()
        report['active_engine'] = str(engine)
        if engine != 'physx':
            report['reason'] = '当前活动物理引擎不是PhysX，未作唤醒'
            return report

        from pxr import PhysicsSchemaTools
        import omni.usd
        import omni.physx

        phase = 'current_context_stage'
        # 不读取旧attached-stage接口；它返回0不能证明当前core上下文无效。
        stage_id = int(omni.usd.get_context().get_stage_id())
        report['stage_id'] = stage_id
        if stage_id <= 0:
            report['reason'] = '当前USD上下文没有有效stage ID'
            return report

        phase = 'actual_tensor_body'
        articulation = executor.articulation
        view = articulation._physics_view
        names = list(articulation.body_names)
        paths = list(view.link_paths)
        if paths and isinstance(paths[0], str):
            row = paths
        elif len(paths) == 1:
            row = list(paths[0])
        else:
            report['reason'] = '不支持歧义的多机器人tensor view'
            return report
        matches = [i for i, name in enumerate(names) if name == 'base_link']
        if len(row) != len(names) or len(matches) != 1:
            report['reason'] = 'tensor link_paths/body_names中没有唯一匹配的base_link'
            return report
        body_path = str(row[matches[0]])
        prefix = str(getattr(executor, 'robot_prim_path', '')).rstrip('/')
        if (body_path.rsplit('/', 1)[-1] != 'base_link' or not body_path.startswith('/') or
                (prefix and body_path != prefix and not body_path.startswith(prefix + '/'))):
            report['reason'] = 'base_link路径不属于当前机器人实际刚体'
            return report
        report['body_path'] = body_path

        phase = 'native_body_verification'
        observed = omni.physx.get_physx_interface().get_rigidbody_transformation(body_path)
        if not isinstance(observed, dict) or observed.get('ret_val') is not True:
            report['reason'] = 'SDK没有确认该路径对应实际刚体，未查询sleep/唤醒'
            return report
        position, quaternion = observed['position'], observed['rotation']
        if (len(position) != 3 or len(quaternion) != 4 or
                not all(math.isfinite(float(v)) for v in (*position, *quaternion)) or
                sum(float(v) ** 2 for v in quaternion) < 1e-12):
            report['reason'] = 'SDK刚体位姿非有限/无效，未作唤醒'
            return report
        report['body_verified'] = True
        body_id = int(PhysicsSchemaTools.sdfPathToInt(body_path))
        if body_id <= 0:
            report['reason'] = '实际刚体路径编码无效'
            return report
        report['body_id'] = body_id
        simulation = omni.physx.get_physx_simulation_interface()
        phase = 'native_sleep_query'
        sleeping = simulation.is_sleeping(stage_id, body_id)
        if type(sleeping) is not bool:
            report['reason'] = 'native sleep查询未返回明确布尔值'
            return report
        report['sleep_before'] = sleeping
        if not sleeping:
            report.update(supported=True, sleep_after=False, reason='刚体已经awake，跳过唤醒和全部状态写入')
            return report

        phase = 'native_wake'
        report['wake_attempted'] = True
        simulation.wake_up(stage_id, body_id)
        report['wake_called'] = True
        phase = 'native_post_wake_query'
        after = simulation.is_sleeping(stage_id, body_id)
        if type(after) is not bool:
            raise RuntimeError('native唤醒后的sleep查询未返回明确布尔值')
        report.update(supported=True, sleep_after=after,
                      reason='仅调用native wake_up；未写入关节/根状态或驱动参数')
        return report
    except Exception as error:
        # wake自身若抛异常，只保留attempted标记；绝不再尝试速度/位置setter作为fallback。
        report.update(supported=False, error_type=type(error).__name__, error=str(error),
                      error_phase=phase, reason='原生查询/唤醒不可用，未执行任何状态setter fallback')
        return report


def _mock_tests():
    import sys
    import types
    import unittest
    from unittest import mock

    class WakeContract(unittest.TestCase):
        def fixture(self, sleeping=False, engine='physx', found=True, stage_id=9223001):
            self.sleeping, self.calls = sleeping, []
            simulation = types.SimpleNamespace()

            def is_sleeping(sid, body):
                self.calls.append(('is_sleeping', sid, body))
                return self.sleeping

            def wake_up(sid, body):
                self.calls.append(('wake_up', sid, body))
                self.sleeping = False

            simulation.is_sleeping, simulation.wake_up = is_sleeping, wake_up
            interface = types.SimpleNamespace(get_rigidbody_transformation=lambda path: {
                'ret_val': found, 'position': [0., 0., 0.], 'rotation': [0., 0., 0., 1.]})
            modules = {name: types.ModuleType(name) for name in (
                'isaacsim', 'isaacsim.core', 'isaacsim.core.simulation_manager',
                'pxr', 'omni', 'omni.usd', 'omni.physx')}
            modules['isaacsim.core.simulation_manager'].SimulationManager = types.SimpleNamespace(
                get_active_physics_engine=lambda: engine)
            modules['pxr'].PhysicsSchemaTools = types.SimpleNamespace(sdfPathToInt=lambda path: 12345)
            modules['omni.usd'].get_context = lambda: types.SimpleNamespace(get_stage_id=lambda: stage_id)
            modules['omni.physx'].get_physx_interface = lambda: interface
            modules['omni.physx'].get_physx_simulation_interface = lambda: simulation
            modules['omni'].usd, modules['omni'].physx = modules['omni.usd'], modules['omni.physx']
            modules['isaacsim'].core = modules['isaacsim.core']
            modules['isaacsim.core'].simulation_manager = modules['isaacsim.core.simulation_manager']
            self.addCleanup(mock.patch.stopall)
            mock.patch.dict(sys.modules, modules).start()
            forbidden = mock.Mock(side_effect=AssertionError('禁止q/v/root/targets/gains状态setter'))
            view = types.SimpleNamespace(link_paths=[['/humanoid/Geometry/base_link', '/humanoid/Geometry/arm']],
                                         set_dof_velocities=forbidden, set_dof_positions=forbidden,
                                         set_dof_velocity_targets=forbidden, set_dof_position_targets=forbidden,
                                         set_root_transforms=forbidden, set_dof_stiffnesses=forbidden)
            self.executor = types.SimpleNamespace(robot_prim_path='/humanoid', articulation=types.SimpleNamespace(
                body_names=['base_link', 'arm'], _physics_view=view, set_joint_velocities=forbidden,
                set_joint_positions=forbidden, set_gains=forbidden))
            self.forbidden, self.simulation = forbidden, simulation

        def test_already_awake_never_writes(self):
            self.fixture()
            result = wake_if_sleeping(self.executor)
            self.assertTrue(result['supported'])
            self.assertFalse(result['wake_attempted'])
            self.assertEqual(self.calls, [('is_sleeping', 9223001, 12345)])
            self.forbidden.assert_not_called()

        def test_only_sleeping_body_gets_native_wake(self):
            self.fixture(sleeping=True)
            result = wake_if_sleeping(self.executor)
            self.assertTrue(result['supported'])
            self.assertTrue(result['wake_called'])
            self.assertFalse(result['sleep_after'])
            self.assertEqual([row[0] for row in self.calls], ['is_sleeping', 'wake_up', 'is_sleeping'])
            self.forbidden.assert_not_called()

        def test_engine_and_missing_native_body_refuse(self):
            self.fixture(engine='newton')
            self.assertFalse(wake_if_sleeping(self.executor)['supported'])
            self.assertEqual(self.calls, [])
            self.fixture(found=False)
            self.assertFalse(wake_if_sleeping(self.executor)['supported'])
            self.assertEqual(self.calls, [])

        def test_bad_metadata_and_stage_never_wake(self):
            self.fixture()
            self.executor.articulation.body_names = ['arm', 'other']
            self.assertFalse(wake_if_sleeping(self.executor)['supported'])
            self.assertEqual(self.calls, [])
            self.fixture(stage_id=0)
            self.assertFalse(wake_if_sleeping(self.executor)['supported'])
            self.assertEqual(self.calls, [])

        def test_current_context_used_not_legacy_attached_stage(self):
            self.fixture(sleeping=True, stage_id=9223001)
            self.simulation.get_attached_stage = mock.Mock(return_value=0)
            self.assertTrue(wake_if_sleeping(self.executor)['supported'])
            self.simulation.get_attached_stage.assert_not_called()

        def test_native_query_error_has_no_state_fallback(self):
            self.fixture(sleeping=True)
            self.simulation.is_sleeping = mock.Mock(side_effect=RuntimeError('query unavailable'))
            result = wake_if_sleeping(self.executor)
            self.assertFalse(result['supported'])
            self.assertFalse(result['wake_attempted'])
            self.assertEqual(result['error_phase'], 'native_sleep_query')
            self.forbidden.assert_not_called()

        def test_native_wake_error_is_reported_without_other_action(self):
            self.fixture(sleeping=True)
            self.simulation.wake_up = mock.Mock(side_effect=RuntimeError('native unavailable'))
            result = wake_if_sleeping(self.executor)
            self.assertFalse(result['supported'])
            self.assertTrue(result['wake_attempted'])
            self.assertFalse(result['wake_called'])
            self.forbidden.assert_not_called()

        def test_nonboolean_query_is_not_treated_as_awake(self):
            self.fixture()
            self.simulation.is_sleeping = lambda *args: None
            self.assertFalse(wake_if_sleeping(self.executor)['supported'])
            self.forbidden.assert_not_called()

    suite = unittest.defaultTestLoader.loadTestsFromTestCase(WakeContract)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__':
    raise SystemExit(_mock_tests())

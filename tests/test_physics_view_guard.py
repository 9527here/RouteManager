"""只模拟Tensor生命周期；不启动Kit、不破坏真实view。"""

from _bootstrap import ROOT as _TEST_ROOT  # 统一离线测试的项目/工具导入路径
from types import SimpleNamespace
from unittest import TestCase, main
from unittest.mock import Mock, patch
from physics_view_guard import guard, acknowledge_rebuilt_view


class ViewGuardRegression(TestCase):
    def executor(self):
        return SimpleNamespace(articulation=SimpleNamespace(_physics_view=SimpleNamespace(check=Mock(return_value=True))),
            world=SimpleNamespace(pause=Mock(), _scene=object()), _motion=object(), _auto_pick=object(),
            _base_cmd=object(), _base_parking=object())

    def test_invalid_simulation_never_touches_articulation_and_logs_once(self):
        e=self.executor();log=Mock()
        for _ in range(100):self.assertFalse(guard(e,log,SimpleNamespace(is_valid=False)))
        e.articulation._physics_view.check.assert_not_called();log.error.assert_called_once()
        self.assertIsNone(e._motion);self.assertIsNone(e._base_cmd)

    def test_healthy_view_preserves_pending_control_and_does_not_pause(self):
        e=self.executor();motion=e._motion
        self.assertTrue(guard(e,Mock(),SimpleNamespace(is_valid=True)))
        self.assertIs(e._motion,motion);e.world.pause.assert_not_called()

    def test_latch_needs_explicit_acknowledgement_and_keeps_pause(self):
        e=self.executor();log=Mock();guard(e,log,SimpleNamespace(is_valid=False))
        self.assertFalse(guard(e,log,SimpleNamespace(is_valid=True)))
        result=acknowledge_rebuilt_view(e,log,SimpleNamespace(is_valid=True))
        self.assertTrue(result['valid']);self.assertIsNone(e._physics_view_fault)
        self.assertIsNone(e._motion);e.world.pause.assert_called()

    def test_missing_or_bad_articulation_is_rejected(self):
        for value in (None,SimpleNamespace(check=Mock(return_value=False)),
                      SimpleNamespace(check=Mock(side_effect=RuntimeError('invalid')))):
            e=self.executor();e.articulation._physics_view=value
            self.assertFalse(guard(e,Mock(),SimpleNamespace(is_valid=True)))

    def test_empty_service_needs_no_tensor_backend(self):
        self.assertTrue(guard(SimpleNamespace(articulation=None,world=SimpleNamespace(_scene=object())),Mock(),None))

    def test_empty_robot_does_not_hide_destroyed_world(self):
        e=self.executor();e.articulation=None;del e.world._scene
        self.assertFalse(guard(e,Mock(),None));self.assertIn('World',e._physics_view_fault)
        e.world.pause.assert_called_once()

    def test_destroyed_world_uses_current_timeline_without_native_reads(self):
        import sys, types
        e=self.executor();e.world.pause.side_effect=AttributeError('World cleared')
        timeline=types.ModuleType('omni.timeline');api=SimpleNamespace(pause=Mock())
        timeline.get_timeline_interface=lambda:api
        omni=types.ModuleType('omni');omni.timeline=timeline
        with patch.dict(sys.modules,{'omni':omni,'omni.timeline':timeline}):
            self.assertFalse(guard(e,Mock(),SimpleNamespace(is_valid=False)))
        api.pause.assert_called_once();e.articulation._physics_view.check.assert_not_called()


if __name__=='__main__':main(verbosity=2)

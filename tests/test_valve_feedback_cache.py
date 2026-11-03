"""缓存失效规则的故障注入；不导入Kit、不操作实际场景。"""

from _bootstrap import ROOT as _TEST_ROOT  # 统一离线测试的项目/工具导入路径
from types import SimpleNamespace
from unittest import TestCase, main
from unittest.mock import Mock
from valve_feedback_cache import SceneGuard


class Path:
    def __init__(self, prim, name):self.prim,self.name=prim,name
    def GetPrimPath(self):return self.prim
    def __str__(self):return self.prim+'.'+self.name


class CacheGuardRegression(TestCase):
    def guard(self):
        g=SceneGuard.__new__(SceneGuard);g.reason=None;g.rigid_paths={'/Robot/Base'};g.camera_paths={'/Camera'}
        g.token=Mock();return g
    def notice(self, infos=(), resync=()):
        return SimpleNamespace(GetChangedInfoOnlyPaths=lambda:infos,GetResyncedPaths=lambda:resync)
    def test_current_rigid_pose_and_joint_state_are_not_static_geometry(self):
        g=self.guard();g.changed(self.notice([Path('/Robot/Base','xformOp:translate'),Path('/Robot/Joint','state:angular:physics:position')]),None)
        g.check()
    def test_scale_collision_mesh_joint_and_static_obstacle_changes_invalidate(self):
        for path in [Path('/Robot/Base','xformOp:scale'),Path('/HiddenCube','physics:collisionEnabled'),
                     Path('/Robot/Mesh','points'),Path('/Robot/Joint','physics:localPos0'),
                     Path('/Support','xformOp:translate'),Path('/Material','physics:staticFriction')]:
            g=self.guard();g.changed(self.notice([path]),None)
            with self.assertRaises(RuntimeError):g.check()
    def test_topology_change_including_new_hidden_collider_invalidates(self):
        g=self.guard();g.changed(self.notice(resync=['/NewHiddenCollider']),None)
        with self.assertRaises(RuntimeError):g.check()
    def test_viewport_camera_motion_keeps_geometry_cache(self):
        g=self.guard();g.changed(self.notice([Path('/Camera','xformOp:transform')]),None);g.check()
    def test_close_revokes_subscription_once(self):
        g=self.guard();token=g.token;g.close();g.close();token.Revoke.assert_called_once()


if __name__=='__main__':main(verbosity=2)

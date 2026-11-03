#!/usr/bin/env python3
"""阀门 IK 的纯离线回归；不启动 Isaac，也不允许任何 HTTP 请求。

用法：python3 tests/test_valve_ik.py
目标由已知关节角的 URDF FK 生成；测试快照明确为 synthetic，不能作为
仿真量测证据。故障注入检查错误锚点和 dry-run 的禁止下发约束。
"""

from _bootstrap import ROOT as _TEST_ROOT  # 统一离线测试的项目/工具导入路径
import contextlib
import copy
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np
from scipy.spatial.transform import Rotation

_REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO))

import arm_kin  # noqa: E402
import valve_ik  # noqa: E402
import valve_trajectory  # noqa: E402
import valve_turn  # noqa: E402


def _pose(p, R):
    xyzw = Rotation.from_matrix(R).as_quat()
    return {"position": np.asarray(p).tolist(),
            "orientation": xyzw[[3, 0, 1, 2]].tolist()}


class ValveIKRegression(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(_REPO / "scripts/arm_calib.json") as f:
            calib = json.load(f)
        cls.chain = arm_kin.load_chain(calib["urdf"], "right_arm_base_link",
                                      "right_gripper_base_link")
        cls.A, cls.b = np.array(calib["A"]), np.array(calib["b"])
        cls.q0 = np.array(calib["q_ref"]) + [0, .5, -.8, 0, .6, 0]
        cls.limits = arm_kin.joint_limits(cls.chain)
        cls.T = np.eye(4)
        cls.T[:3, :3], cls.T[:3, 3] = cls.A, cls.b
        # 测试专用构造状态：用已知 FK 生成，绝不伪称独立仿真实测。
        p, R = arm_kin.ee_world(cls.chain, cls.q0, cls.A, cls.b)
        cls.synthetic = {
            "source": "synthetic: offline URDF FK regression only",
            "state": {"joint_names": arm_kin.joint_names(cls.chain),
                      "positions": cls.q0.tolist(),
                      "sides": {"right": {
                          "flange": _pose(p, R),
                          "tcp": _pose(p + R @ [0, 0, .2572], R)}}},
            "arm_base": _pose(cls.b, cls.A),
        }

    def setUp(self):
        self.addCleanup(mock.patch.stopall)
        mock.patch("urllib.request.urlopen",
                   side_effect=AssertionError("离线回归禁止网络请求")).start()
        mock.patch.object(valve_trajectory.time, "sleep",
                          side_effect=AssertionError("dry-run 不应 sleep")).start()
        mock.patch.object(valve_turn, "READ_ONLY", False).start()
        mock.patch.object(valve_trajectory, "READ_ONLY", False).start()

    def assert_solution(self, chain, result, p_target, R_target, A=None, b=None):
        q, reported_pe, reported_ae = result
        p, R = arm_kin.ee_world(chain, q, self.A if A is None else A,
                               self.b if b is None else b)
        pe = np.linalg.norm(p_target - p)
        # 独立以 trace/acos 检查最终姿态，避免测试重复求解器的旋转误差实现。
        ae = math.degrees(math.acos(np.clip((np.trace(R.T @ R_target) - 1) / 2,
                                          -1, 1)))
        self.assertLess(pe, 5e-4)
        self.assertLess(ae, .2)
        self.assertAlmostEqual(reported_pe, pe, places=10)
        self.assertAlmostEqual(reported_ae, ae, places=6)
        return q

    def test_known_fk_target_both_entrypoints(self):
        q_target = self.q0 + [.04, -.05, .06, -.07, .04, -.03]
        p, R = arm_kin.ee_world(self.chain, q_target, self.A, self.b)
        for entry in ("valve_turn", "valve_trajectory"):
            with self.subTest(entry=entry):
                if entry == "valve_turn":
                    result = valve_turn.solve_ik(self.chain, self.q0, p, R,
                                                self.A, self.b, self.limits)
                else:
                    result = valve_trajectory.solve_ik(self.chain, self.q0, p, R,
                                                       self.T, self.limits, self.q0)
                self.assert_solution(self.chain, result, p, R)

    def test_world_rotation_and_translation_equivariance(self):
        q_target = self.q0 + [.15, .2, -.25, .3, .2, -.4]
        p, R = arm_kin.ee_world(self.chain, q_target, self.A, self.b)
        result = valve_ik.solve_ik(self.chain, self.q0, p, R,
                                  self.A, self.b, self.limits)
        H = Rotation.from_euler("xyz", [.8, -.4, 1.2]).as_matrix()
        offset = np.array([1., -2., .5])
        A2, b2, p2, R2 = H @ self.A, H @ self.b + offset, H @ p + offset, H @ R
        result2 = valve_ik.solve_ik(self.chain, self.q0, p2, R2,
                                   A2, b2, self.limits)
        q1 = self.assert_solution(self.chain, result, p, R)
        q2 = self.assert_solution(self.chain, result2, p2, R2, A2, b2)
        np.testing.assert_allclose(q1, q2, atol=1e-6, rtol=0)

    def test_exact_pi_single_joint_has_known_solution(self):
        chain = [{"name": "axis", "type": "revolute", "parent": "root",
                  "xyz": [0, 0, 0], "rpy": [0, 0, 0], "axis": [0, 0, 1],
                  "limit": [-3.3, 3.3]}]
        target = np.diag([-1., -1., 1.])
        result = valve_ik.solve_ik(chain, np.zeros(1), np.zeros(3), target,
                                  np.eye(3), np.zeros(3), np.array([[-3.3, 3.3]]))
        q = self.assert_solution(chain, result, np.zeros(3), target,
                                 np.eye(3), np.zeros(3))
        self.assertAlmostEqual(abs(q[0]), math.pi, places=3)

    def test_seven_joint_redundancy_preserves_task(self):
        chain = self.chain + [{"name": "extra", "type": "revolute",
                              "parent": "tool", "xyz": [0, 0, 0],
                              "rpy": [0, 0, 0], "axis": [0, 0, 1],
                              "limit": [-3.3, 3.3]}]
        q0 = np.r_[self.q0, .2]
        target_q = q0 + [.04, -.05, .06, -.07, .04, -.03, .08]
        p, R = arm_kin.ee_world(chain, target_q, self.A, self.b)
        result = valve_ik.solve_ik(chain, q0, p, R, self.A, self.b,
                                  arm_kin.joint_limits(chain), q_ref=np.zeros(7),
                                  w_posture=.15)
        q = self.assert_solution(chain, result, p, R)
        self.assertEqual(q.shape, (7,))

    def test_independent_anchor_rejects_corrupt_flange_and_base(self):
        with contextlib.redirect_stdout(io.StringIO()):
            valve_turn.anchor(self.chain, self.synthetic)
        bad_rotation = copy.deepcopy(self.synthetic)
        pose = bad_rotation["state"]["sides"]["right"]["flange"]
        pose["orientation"] = [1, 0, 0, 0]
        bad_base = copy.deepcopy(self.synthetic)
        bad_base["arm_base"]["position"][0] += .02
        for snapshot in (bad_rotation, bad_base):
            with self.subTest(snapshot=snapshot["arm_base"]), \
                    contextlib.redirect_stdout(io.StringIO()), \
                    self.assertRaisesRegex(SystemExit, "FK 与独立实测法兰不符"):
                valve_turn.anchor(self.chain, snapshot)

    def test_probe_dry_run_never_sends_sleeps_or_rereads(self):
        with tempfile.TemporaryDirectory(prefix="valve_ik_synthetic_") as tmp:
            path = Path(tmp) / "synthetic.json"
            path.write_text(json.dumps(self.synthetic))
            argv = ["valve_trajectory.py", "--probe", "--dry-run", "--snapshot", str(path)]
            output = io.StringIO()
            with mock.patch.object(sys, "argv", argv), \
                    mock.patch.object(valve_trajectory, "load_robot_chain", return_value=self.chain), \
                    mock.patch.object(valve_trajectory, "anchor", wraps=valve_trajectory.anchor) as read, \
                    mock.patch.object(valve_trajectory, "_post",
                                      side_effect=AssertionError("不能发送 action")), \
                    contextlib.redirect_stdout(output):
                valve_trajectory.main()
            self.assertEqual(read.call_count, 1)
            self.assertIn("未下发；实际位移未验证", output.getvalue())

    def test_snapshot_without_dry_run_is_rejected_before_io(self):
        for module, args in ((valve_turn, []), (valve_trajectory, ["--probe"])):
            with self.subTest(module=module.__name__), \
                    mock.patch.object(sys, "argv", [module.__name__, *args,
                                                    "--snapshot", "must-not-open.json"]), \
                    mock.patch("builtins.open", side_effect=AssertionError("不能读取执行快照")), \
                    contextlib.redirect_stderr(io.StringIO()), \
                    self.assertRaises(SystemExit) as error:
                module.main()
            self.assertEqual(error.exception.code, 2)

    def test_legacy_turn_execution_rejected_before_network_or_model_read(self):
        with mock.patch.object(sys, "argv", ["valve_turn.py", "--angle", "90"]), \
                mock.patch.object(valve_turn, "load_chain", side_effect=AssertionError("不能加载模型")), \
                mock.patch.object(valve_turn, "_post", side_effect=AssertionError("不能下发动作")), \
                mock.patch.object(valve_turn, "_python", side_effect=AssertionError("不能访问现场")), \
                contextlib.redirect_stderr(io.StringIO()) as error_output, \
                self.assertRaises(SystemExit) as error:
            valve_turn.main()
        self.assertEqual(error.exception.code, 2)
        self.assertIn("仅保留 --dry-run", error_output.getvalue())

    def test_failed_plan_and_probe_orientation_stop_before_commands(self):
        with tempfile.TemporaryDirectory(prefix="valve_ik_synthetic_") as tmp:
            path = Path(tmp) / "synthetic.json"
            path.write_text(json.dumps(self.synthetic))
            cases = [([".75", "0", "7.0691"], .006, 0.), (["--probe"], 0., 3.1)]
            for args, pe, ae in cases:
                with self.subTest(args=args), \
                        mock.patch.object(sys, "argv", ["valve_trajectory.py", *args,
                                                        "--dry-run", "--snapshot", str(path)]), \
                        mock.patch.object(valve_trajectory, "load_robot_chain", return_value=self.chain), \
                        mock.patch.object(valve_trajectory, "solve_ik", return_value=(self.q0, pe, ae)), \
                        mock.patch.object(valve_trajectory, "send", side_effect=AssertionError("不能发送运动")), \
                        mock.patch.object(valve_trajectory, "gripper", side_effect=AssertionError("不能开爪")), \
                        contextlib.redirect_stdout(io.StringIO()), self.assertRaises(SystemExit):
                    valve_trajectory.main()


if __name__ == "__main__":
    unittest.main(verbosity=2)

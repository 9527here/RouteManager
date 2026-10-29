"""IK 轨迹 v2：照 piper_lerobot/test/test_valve.py 的思路重写。

关键差异（相比 v1 手写 DLS）：
  * IK 交给 isaaclab 的 DifferentialIKController，不自己写求解循环
  * **姿态也要作为目标**（v1 只约束位置，所以不收敛）
  * 拧动 = 原地改 yaw，不在圆上绕
  * 先用过渡姿态(俯仰 45°)接近，再摆正到 90° 垂直
  * yaw 初始给 2° 避开奇异点

欧拉约定与源脚本一致：scipy Rotation.from_euler('xyz', ...) 外旋，四元数 (x,y,z,w)。
"""
import argparse
import math

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(conflict_handler="resolve")
parser.add_argument("--physics", default="isaacsim_physx", choices=["isaacsim_physx"])
parser.add_argument("--gui", action="store_true")
parser.add_argument("--angle", type=float, default=90.0, help="拧动角度(度)")
parser.add_argument("--approach", type=float, default=0.12)
add_launcher_args(parser)
args_cli, _ = parser.parse_known_args()
parser.set_defaults(headless=not args_cli.gui, visualizer=["kit"] if args_cli.gui else [])
args_cli = parser.parse_args()

ARM = [f"right_arm_{i}_joint" for i in range(1, 7)]


def euler_to_quat_wxyz(roll, pitch, yaw):
    from scipy.spatial.transform import Rotation
    x, y, z, w = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_quat()
    return (w, x, y, z)


def main():
    from isaaclab.physics import PhysicsCfg

    with launch_simulation(cfg=PhysicsCfg(), launcher_args=args_cli):
        import torch

        from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
        from isaaclab.envs import ManagerBasedRLEnv

        from valve_rl.tasks.valve_env_cfg import GRIP_CLOSE, GRIP_OPEN, ValveTurnEnvCfg

        cfg = ValveTurnEnvCfg()
        cfg.scene.num_envs = 1
        env = ManagerBasedRLEnv(cfg=cfg)
        env.reset()

        robot, valve = env.scene["robot"], env.scene["valve"]
        dev, dt = env.device, env.physics_dt
        arm_ids = [robot.joint_names.index(n) for n in ARM]
        ee_i = robot.body_names.index("right_gripper_base_link")
        wh_i = valve.body_names.index("wheel_link")
        grip_ids = [robot.joint_names.index(n) for n in GRIP_OPEN]
        g_open = torch.tensor([GRIP_OPEN[n] for n in GRIP_OPEN], device=dev)
        g_close = torch.tensor([GRIP_CLOSE[n] for n in GRIP_CLOSE], device=dev)

        ik = DifferentialIKController(
            DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls"),
            num_envs=1, device=dev,
        )

        def wheel():
            return valve.data.body_pos_w[0, wh_i].clone()

        def ee_pose():
            return robot.data.body_pos_w[0, ee_i].clone(), robot.data.body_quat_w[0, ee_i].clone()

        def solve_and_hold(pos, quat_wxyz, grip, steps=45):
            """把末端位姿目标解成关节目标并平滑走过去。"""
            for _ in range(steps):
                ee_p, ee_q = ee_pose()
                Jall = robot.root_physx_view.get_jacobians()
                Jnp = Jall.numpy() if hasattr(Jall, "numpy") else Jall
                J = torch.tensor(Jnp[0, ee_i][:, arm_ids], device=dev, dtype=torch.float32)
                cmd = torch.tensor([[pos[0], pos[1], pos[2], quat_wxyz[0], quat_wxyz[1],
                                     quat_wxyz[2], quat_wxyz[3]]], device=dev)
                ik.set_command(cmd)
                q_des = ik.compute(ee_p.unsqueeze(0), ee_q.unsqueeze(0), J.unsqueeze(0),
                                   robot.data.joint_pos[:, arm_ids])
                full = robot.data.joint_pos[0].clone()
                full[arm_ids] = q_des[0]
                full[grip_ids] = grip
                robot.set_joint_position_target(full.unsqueeze(0))
                env.sim.step(render=not args_cli.headless)
                env.scene.update(dt)

        w = wheel()
        print(f"RESULT wheel {[round(float(x), 3) for x in w]}")
        x, y, z = float(w[0]), float(w[1]), float(w[2])
        ah = args_cli.approach

        q_pre = euler_to_quat_wxyz(0.0, math.radians(45), math.radians(2))
        q_vert = euler_to_quat_wxyz(0.0, math.radians(90), math.radians(2))
        q_turn = euler_to_quat_wxyz(0.0, math.radians(90),
                                    math.radians(2) + math.radians(args_cli.angle))

        steps = [
            ("张开夹爪", (x, y, z + ah + 0.10), q_pre, g_open),
            ("摆正到上方", (x, y, z + ah), q_vert, g_open),
            ("下降到阀门", (x, y, z + 0.02), q_vert, g_open),
            ("合爪", (x, y, z + 0.02), q_vert, g_close),
            (f"拧 {args_cli.angle:.0f}°", (x, y, z + 0.02), q_turn, g_close),
            ("张开", (x, y, z + 0.02), q_turn, g_open),
            ("抬升", (x, y, z + ah), q_turn, g_open),
        ]
        for name, pos, quat, grip in steps:
            solve_and_hold(pos, quat, grip)
            ee_p, _ = ee_pose()
            d = float(torch.norm(ee_p - w))
            ang = float(valve.data.joint_pos[0, 0])
            print(f"RESULT {name:12s} dist={d:.3f} valve={ang:+.3f}")


if __name__ == "__main__":
    main()

"""IK 脚本轨迹演示：张开 → 从上接近 → 下降 → 合爪 → 拧 → 松开 → 收回。

不走策略，纯数值 IK（阻尼最小二乘）+ 关节目标回放，用来验证动作序列对不对。
"""
import argparse
import math

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(conflict_handler="resolve")
parser.add_argument("--physics", default="isaacsim_physx", choices=["isaacsim_physx"])
parser.add_argument("--gui", action="store_true")
parser.add_argument("--turn", type=float, default=1.8, help="拧动角度(rad)")
add_launcher_args(parser)
args_cli, _ = parser.parse_known_args()
parser.set_defaults(headless=not args_cli.gui, visualizer=["kit"] if args_cli.gui else [])
args_cli = parser.parse_args()


def main():
    from isaaclab.physics import PhysicsCfg

    with launch_simulation(cfg=PhysicsCfg(), launcher_args=args_cli):
        import torch

        from isaaclab.envs import ManagerBasedRLEnv

        from valve_rl.tasks.valve_env_cfg import GRIP_CLOSE, GRIP_OPEN, ValveTurnEnvCfg

        cfg = ValveTurnEnvCfg()
        cfg.scene.num_envs = 1
        env = ManagerBasedRLEnv(cfg=cfg)
        env.reset()

        robot, valve = env.scene["robot"], env.scene["valve"]
        dev, dt = env.device, env.physics_dt
        arm = ["right_arm_%d_joint" % i for i in range(1, 7)]
        arm_ids = [robot.joint_names.index(n) for n in arm]
        ee_i = robot.body_names.index("right_gripper_base_link")
        wh_i = valve.body_names.index("wheel_link")
        grip_ids = [robot.joint_names.index(n) for n in GRIP_OPEN]
        grip_open = torch.tensor([GRIP_OPEN[n] for n in GRIP_OPEN], device=dev)
        grip_close = torch.tensor([GRIP_CLOSE[n] for n in GRIP_CLOSE], device=dev)

        def ee_pos():
            return robot.data.body_pos_w[0, ee_i].clone()

        def wheel_pos():
            return valve.data.body_pos_w[0, wh_i].clone()

        def ik(target, iters=120, lam=0.05, lr=0.6):
            """位置式阻尼最小二乘 IK（只约束末端位置）。"""
            q = robot.data.joint_pos[0, arm_ids].clone()
            for _ in range(iters):
                qq = robot.data.joint_pos[0].clone()
                qq[arm_ids] = q
                robot.write_joint_state_to_sim(qq.unsqueeze(0), torch.zeros_like(qq).unsqueeze(0))
                env.sim.step(render=False)
                env.scene.update(dt)
                e = target - ee_pos()
                if torch.norm(e) < 1e-3:
                    break
                Jall = robot.root_physx_view.get_jacobians()
                Jnp = Jall.numpy() if hasattr(Jall, "numpy") else Jall
                J = torch.tensor(Jnp[0, ee_i][:3, :][:, arm_ids], device=dev, dtype=torch.float32)
                dq = J.T @ torch.linalg.solve(J @ J.T + lam**2 * torch.eye(3, device=dev), e)
                q = q + lr * dq
            return q

        def play(q_target, grip, steps=40):
            q_cur = robot.data.joint_pos[0, arm_ids].clone()
            g_cur = robot.data.joint_pos[0, grip_ids].clone()
            for i in range(steps):
                a = (i + 1) / steps
                full = robot.data.joint_pos[0].clone()
                full[arm_ids] = q_cur + (q_target - q_cur) * a
                full[grip_ids] = g_cur + (grip - g_cur) * a
                robot.set_joint_position_target(full.unsqueeze(0))
                env.sim.step(render=not args_cli.headless)
                env.scene.update(dt)

        w = wheel_pos()
        print(f"RESULT wheel {[round(float(x),3) for x in w]}")

        # 1) 张开，抬到阀门正上方
        q_above = ik(w + torch.tensor([0.0, 0.0, 0.18], device=dev))
        play(q_above, grip_open, 50)
        print(f"RESULT stage1_above dist={float(torch.norm(ee_pos()-w)):.3f}")

        # 2) 下降到手轮
        q_grasp = ik(w + torch.tensor([0.0, 0.0, 0.02], device=dev))
        play(q_grasp, grip_open, 40)
        print(f"RESULT stage2_down dist={float(torch.norm(ee_pos()-w)):.3f}")

        # 3) 合爪
        play(q_grasp, grip_close, 20)
        print("RESULT stage3_closed")

        # 4) 绕手轮 Z 轴走圆弧（拧）
        r = 0.06
        for k in range(1, 9):
            ang = args_cli.turn * k / 8
            tgt = torch.tensor([w[0] + r * math.sin(float(ang)),
                                w[1] - r * (1 - math.cos(float(ang))),
                                w[2] + 0.02], device=dev)
            play(ik(tgt, iters=60), grip_close, 12)
        print(f"RESULT stage4_turned valve={float(valve.data.joint_pos[0,0]):+.3f}")

        # 5) 松开 + 收回
        play(q_grasp, grip_open, 20)
        q_up = ik(w + torch.tensor([-0.10, 0.0, 0.25], device=dev))
        play(q_up, grip_open, 50)
        print(f"RESULT stage5_retract dist={float(torch.norm(ee_pos()-w)):.3f}")
        print(f"RESULT final_valve {float(valve.data.joint_pos[0,0]):+.3f}")


if __name__ == "__main__":
    main()

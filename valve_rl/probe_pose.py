"""搜索右臂朝阀门的前伸初始姿态（随机采样 + 正运动学）。

不做解析 IK，直接采样关节角、算末端世界位置、取离手轮最近的那组。
"""
import argparse

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(conflict_handler="resolve")
parser.add_argument("--physics", default="isaacsim_physx", choices=["isaacsim_physx"])
parser.add_argument("--samples", type=int, default=400)
add_launcher_args(parser)
parser.set_defaults(headless=True, visualizer=[])
args_cli = parser.parse_args()


def main():
    from isaaclab.physics import PhysicsCfg

    with launch_simulation(cfg=PhysicsCfg(), launcher_args=args_cli):
        import torch

        from isaaclab.envs import ManagerBasedRLEnv

        from valve_rl.tasks.valve_env_cfg import ARM_JOINTS, ValveTurnEnvCfg

        cfg = ValveTurnEnvCfg()
        cfg.scene.num_envs = 1
        env = ManagerBasedRLEnv(cfg=cfg)
        env.reset()

        robot = env.scene["robot"]
        valve = env.scene["valve"]
        dev = env.device

        arm_ids = [robot.joint_names.index(n) for n in ARM_JOINTS]
        ee_idx = robot.body_names.index("right_gripper_base_link")
        wheel_idx = valve.body_names.index("wheel_link")
        target = valve.data.body_pos_w[0, wheel_idx]
        print(f"RESULT target {[round(float(x),3) for x in target]}")

        zero_act = torch.zeros(env.action_space.shape, device=dev)
        limits = robot.data.joint_pos_limits[0, arm_ids]      # (6,2)
        lo, hi = limits[:, 0], limits[:, 1]
        # 别贴限位，留 20% 余量
        span = (hi - lo) * 0.4
        lo2, hi2 = lo + span * 0.5, hi - span * 0.5

        best_d, best_q = 1e9, None
        g = torch.Generator(device="cpu").manual_seed(0)
        for i in range(args_cli.samples):
            q_arm = lo2 + (hi2 - lo2) * torch.rand(6, generator=g).to(dev)
            full = robot.data.joint_pos[0].clone()
            full[arm_ids] = q_arm
            robot.write_joint_state_to_sim(
                full.unsqueeze(0), torch.zeros_like(full).unsqueeze(0)
            )
            env.step(zero_act)
            ee = robot.data.body_pos_w[0, ee_idx]
            d = float(torch.norm(ee - target))
            if d < best_d:
                best_d, best_q, best_ee = d, q_arm.tolist(), ee.tolist()

        print(f"RESULT best_distance {best_d:.4f}")
        print(f"RESULT best_ee {[round(x,3) for x in best_ee]}")
        for n, v in zip(ARM_JOINTS, best_q):
            print(f"RESULT pose {n} {v:+.4f}")
        env.close()


if __name__ == "__main__":
    main()

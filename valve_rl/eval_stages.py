"""按阶段评估训练好的策略：接近 → 抓握 → 拧动 → 松开 → 收回。"""
import argparse

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(conflict_handler="resolve")
parser.add_argument("--physics", default="isaacsim_physx", choices=["isaacsim_physx"])
parser.add_argument("--ckpt", required=True)
parser.add_argument("--steps", type=int, default=400)
add_launcher_args(parser)
parser.set_defaults(headless=True, visualizer=[])
args_cli = parser.parse_args()


def main():
    from isaaclab.physics import PhysicsCfg

    with launch_simulation(cfg=PhysicsCfg(), launcher_args=args_cli):
        import torch

        from isaaclab.envs import ManagerBasedRLEnv
        from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
        from isaaclab_rl.rsl_rl.utils import (
            check_rsl_rl_version, create_rsl_rl_runner, handle_deprecated_rsl_rl_cfg,
        )

        from valve_rl.tasks.agents.rsl_rl_ppo_cfg import ValveTurnPPORunnerCfg
        from valve_rl.tasks.valve_env_cfg import ValveTurnEnvCfg

        env_cfg = ValveTurnEnvCfg()
        env_cfg.scene.num_envs = 1
        agent_cfg = handle_deprecated_rsl_rl_cfg(ValveTurnPPORunnerCfg(), check_rsl_rl_version())
        agent_cfg.device = env_cfg.sim.device
        env_cfg.seed = agent_cfg.seed

        env = ManagerBasedRLEnv(cfg=env_cfg)
        env = RslRlVecEnvWrapper(env)
        runner = create_rsl_rl_runner(env, agent_cfg, None)
        runner.load(args_cli.ckpt)
        policy = runner.get_inference_policy(device=env_cfg.sim.device)

        u = env.unwrapped
        robot, valve = u.scene["robot"], u.scene["valve"]
        ee_i = robot.body_names.index("right_gripper_base_link")
        wh_i = valve.body_names.index("wheel_link")
        grip_i = robot.joint_names.index("right_gripper_right_2_joint")

        obs = env.get_observations()
        max_turn, min_d, max_grip = 0.0, 9.9, 0.0
        for i in range(args_cli.steps):
            with torch.inference_mode():
                obs, _, _, _ = env.step(policy(obs))
            ee = robot.data.body_pos_w[0, ee_i]
            wh = valve.data.body_pos_w[0, wh_i]
            d = float(torch.norm(ee - wh))
            dz = float(ee[2] - wh[2])
            a = float(valve.data.joint_pos[0, 0])
            g = float(robot.data.joint_pos[0, grip_i]) / 0.7
            max_turn = max(max_turn, a)
            min_d = min(min_d, d)
            max_grip = max(max_grip, g)
            if i % 40 == 0:
                print(f"RESULT t={i:3d} dist={d:.3f} dz={dz:+.3f} grip={g:.2f} valve={a:+.3f}")

        print(f"RESULT min_dist      {min_d:.3f}   (抓握需要 <0.12)")
        print(f"RESULT max_grip_open {max_grip:.2f}   (张开最大 1.0)")
        print(f"RESULT max_turn      {max_turn:+.3f} (目标 >1.5)")
        env.close()


if __name__ == "__main__":
    main()

"""加载 checkpoint 跑确定性策略，实测阀门到底被拧动了多少。"""
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
            check_rsl_rl_version,
            create_rsl_rl_runner,
            handle_deprecated_rsl_rl_cfg,
        )

        from valve_rl.tasks.agents.rsl_rl_ppo_cfg import ValveTurnPPORunnerCfg
        from valve_rl.tasks.valve_env_cfg import ValveTurnEnvCfg

        env_cfg = ValveTurnEnvCfg()
        env_cfg.scene.num_envs = 1
        agent_cfg = ValveTurnPPORunnerCfg()
        agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, check_rsl_rl_version())
        agent_cfg.device = env_cfg.sim.device
        env_cfg.seed = agent_cfg.seed

        env = ManagerBasedRLEnv(cfg=env_cfg)
        env = RslRlVecEnvWrapper(env)
        runner = create_rsl_rl_runner(env, agent_cfg, None)
        runner.load(args_cli.ckpt)
        policy = runner.get_inference_policy(device=env_cfg.sim.device)

        obs = env.get_observations()
        angle0 = float(env.unwrapped.scene["valve"].data.joint_pos[0, 0])
        best = angle0
        for i in range(args_cli.steps):
            with torch.inference_mode():
                act = policy(obs)
                obs, _, _, _ = env.step(act)
            a = float(env.unwrapped.scene["valve"].data.joint_pos[0, 0])
            best = max(best, a) if a > angle0 else best
            if i % 50 == 0:
                print(f"RESULT step {i:4d} valve_angle {a:+.3f}")

        angle_end = float(env.unwrapped.scene["valve"].data.joint_pos[0, 0])
        print(f"RESULT angle_start {angle0:+.3f}")
        print(f"RESULT angle_end   {angle_end:+.3f}")
        print(f"RESULT max_angle   {best:+.3f}")
        print(f"RESULT turned_rad  {best - angle0:+.3f}")
        env.close()


if __name__ == "__main__":
    main()

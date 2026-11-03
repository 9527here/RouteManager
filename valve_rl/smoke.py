"""环境冒烟测试：建 2 个环境、跑若干步随机动作，检查形状/奖励/重置。

注意：所有 isaaclab / omni / pxr 相关导入都必须发生在 launch_simulation 之后，
否则 USD 插件会在启动阶段解析失败（undefined symbol: pxrInternal_...）。
"""
import argparse

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(conflict_handler="resolve")
parser.add_argument("--physics", default="isaacsim_physx", choices=["isaacsim_physx"])
parser.add_argument("--num_envs", type=int, default=2)
parser.add_argument("--steps", type=int, default=20)
add_launcher_args(parser)
parser.set_defaults(headless=True, visualizer=[])
args_cli = parser.parse_args()


def main():
    from isaaclab.physics import PhysicsCfg

    with launch_simulation(cfg=PhysicsCfg(), launcher_args=args_cli):
        import torch

        from isaaclab.envs import ManagerBasedRLEnv

        from valve_rl.tasks.valve_env_cfg import ValveTurnEnvCfg

        cfg = ValveTurnEnvCfg()
        cfg.scene.num_envs = args_cli.num_envs
        env = ManagerBasedRLEnv(cfg=cfg)

        obs, _ = env.reset()
        print("RESULT action_space", env.action_space)
        print("RESULT obs_shape", tuple(obs["policy"].shape))
        print("RESULT num_actions", env.action_space.shape)

        total = torch.zeros(env.num_envs, device=env.device)
        for i in range(args_cli.steps):
            act = 2.0 * torch.rand(env.action_space.shape, device=env.device) - 1.0
            obs, rew, term, trunc, _ = env.step(act)
            total += rew
            if i == 0:
                print("RESULT rew_shape", tuple(rew.shape), "finite", bool(torch.isfinite(rew).all()))
        print("RESULT reward_sum_mean", float(total.mean()))
        print("RESULT valve_angle", env.scene["valve"].data.joint_pos[:, 0].tolist())
        print("RESULT OK")
        env.close()


if __name__ == "__main__":
    main()

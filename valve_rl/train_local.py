"""本地训练入口：不走 Isaac Lab 的 Hydra CLI，直接在自己验证过的
launch_simulation 路径里跑 rsl_rl。

为什么要有这个文件：Isaac Lab 的 train.py CLI 在 `isaacsim_physx.KitLauncher` 那条
启动路径下 import pxr 时会撞上 USD 库版本冲突（libusd_usdPhysics.so 加载失败），
而同样的 cfg 在直接 launch_simulation 下完全正常。

注意：**必须照抄 CLI 后端的准备顺序**（entrypoints/backends/train_rsl_rl.py:129-160），
其中 `handle_deprecated_rsl_rl_cfg` 不能省 —— 它负责把旧式 `stochastic` 参数迁移成
新版 rsl_rl 的 `distribution_cfg`。漏了就会报：
    TypeError: MLPModel.__init__() got an unexpected keyword argument 'stochastic'
"""
import argparse

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(conflict_handler="resolve")
parser.add_argument("--physics", default="isaacsim_physx", choices=["isaacsim_physx"])
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--iterations", type=int, default=20)
parser.add_argument("--gui", action="store_true", help="开窗口看画面（需要 DISPLAY）")
add_launcher_args(parser)
args_cli, _ = parser.parse_known_args()
if args_cli.gui:
    parser.set_defaults(headless=False, visualizer=["kit"])
else:
    parser.set_defaults(headless=True, visualizer=[])
args_cli = parser.parse_args()


def main():
    from isaaclab.physics import PhysicsCfg

    with launch_simulation(cfg=PhysicsCfg(), launcher_args=args_cli):
        import os
        from datetime import datetime

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
        env_cfg.scene.num_envs = args_cli.num_envs
        agent_cfg = ValveTurnPPORunnerCfg()

        # --- 照抄 CLI 后端的准备顺序 ---
        installed = check_rsl_rl_version()
        print(f"RESULT rsl_rl version {installed}")
        agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, installed)
        if args_cli.iterations is not None:
            agent_cfg.max_iterations = args_cli.iterations
        agent_cfg.device = env_cfg.sim.device
        env_cfg.seed = agent_cfg.seed

        run_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        log_dir = os.path.join(
            "/home/jy/valve_rl/logs/rsl_rl", agent_cfg.experiment_name, run_name
        )
        os.makedirs(log_dir, exist_ok=True)
        env_cfg.log_dir = log_dir
        print(f"RESULT log_dir {log_dir}")

        env = ManagerBasedRLEnv(cfg=env_cfg)
        env = RslRlVecEnvWrapper(env)
        print("RESULT env ok")

        runner = create_rsl_rl_runner(env, agent_cfg, log_dir)
        print("RESULT runner ok")
        runner.learn(
            num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True
        )
        print("RESULT TRAIN OK")
        env.close()


if __name__ == "__main__":
    main()

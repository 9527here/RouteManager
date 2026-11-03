"""阀门任务包：import 本模块即完成 gym 环境注册。"""
import gymnasium as gym

from . import agents

gym.register(
    id="Isaac-Valve-Turn-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.valve_env_cfg:ValveTurnEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agents.rsl_rl_ppo_cfg:ValveTurnPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

"""漂移探针：给零动作，看哪些关节在自己动（判断是不是零刚度导致的塌陷）。"""
import argparse

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(conflict_handler="resolve")
parser.add_argument("--physics", default="isaacsim_physx", choices=["isaacsim_physx"])
parser.add_argument("--steps", type=int, default=120)
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
        cfg.scene.num_envs = 1
        env = ManagerBasedRLEnv(cfg=cfg)
        env.reset()

        robot = env.scene["robot"]
        names = list(robot.joint_names)
        j0 = robot.data.joint_pos[0].clone()
        base0 = robot.data.root_pos_w[0].clone()

        zero = torch.zeros(env.action_space.shape, device=env.device)
        for _ in range(args_cli.steps):
            env.step(zero)

        j1 = robot.data.joint_pos[0]
        base1 = robot.data.root_pos_w[0]
        delta = (j1 - j0).abs()

        order = torch.argsort(delta, descending=True)
        print("RESULT base_delta", [round(float(x), 4) for x in (base1 - base0)])
        for k in order[:14].tolist():
            print(f"RESULT drift {names[k]:32s} {float(j0[k]):+.3f} -> {float(j1[k]):+.3f}  |d|={float(delta[k]):.3f}")
        env.close()


if __name__ == "__main__":
    main()

"""阀门手轮可动性探针：直接写关节状态，看它能不能保持/转动。

如果写进去的位置马上回到 0，说明手轮关节被物理引擎锁死了
（阀门资产的惯量是 {1,1,1}、质量为负，PhysX 会当退化刚体处理）。
"""
import argparse

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(conflict_handler="resolve")
parser.add_argument("--physics", default="isaacsim_physx", choices=["isaacsim_physx"])
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

        valve = env.scene["valve"]
        dev = env.device
        n = valve.num_joints
        print(f"RESULT valve_joints {list(valve.joint_names)}")
        try:
            m = valve.root_physx_view.get_masses().numpy()
            print(f"RESULT masses {m}")
        except Exception as e:
            print(f"RESULT masses_unavailable {e!r}")

        # 直接把手轮拨到 0.8 rad，看它保持还是弹回
        target = torch.tensor([[0.8]], device=dev)
        zeros = torch.zeros((1, n), device=dev)
        valve.write_joint_state_to_sim(target, zeros)

        zero_act = torch.zeros(env.action_space.shape, device=dev)
        for i in range(30):
            env.step(zero_act)
        a = float(valve.data.joint_pos[0, 0])
        print(f"RESULT after_30_steps {a:+.4f}")

        # 再给一个角速度，看能否继续转
        vel = torch.tensor([[2.0]], device=dev)
        valve.write_joint_state_to_sim(torch.tensor([[a]], device=dev), vel)
        for i in range(30):
            env.step(zero_act)
        b = float(valve.data.joint_pos[0, 0])
        print(f"RESULT after_velocity {b:+.4f}")
        print(f"RESULT movable {abs(b - a) > 0.05}")
        env.close()


if __name__ == "__main__":
    main()

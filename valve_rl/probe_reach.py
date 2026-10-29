"""可达性探针：机器人各 link 到阀门手轮的距离，判断任务是否物理可行。"""
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

        robot = env.scene["robot"]
        valve = env.scene["valve"]

        names = robot.body_names
        pos = robot.data.body_pos_w[0]          # (num_bodies, 3)
        vpos = valve.data.body_pos_w[0]         # 阀门各 body
        vnames = valve.body_names

        print("RESULT valve_bodies", list(vnames))
        for i, n in enumerate(vnames):
            print(f"RESULT valve_body {n} {[round(float(x),3) for x in vpos[i]]}")

        # 手轮位置（wheel_link）
        wi = None
        for i, n in enumerate(vnames):
            if "wheel" in n:
                wi = i
                break
        target = vpos[wi] if wi is not None else vpos[0]
        print(f"RESULT wheel_world {[round(float(x),3) for x in target]}")

        d = torch.norm(pos - target, dim=1)
        order = torch.argsort(d)
        for k in order[:8].tolist():
            print(f"RESULT nearest {names[k]:32s} d={float(d[k]):.3f} pos={[round(float(x),3) for x in pos[k]]}")

        # 右臂可达范围粗估：肩到最远臂 link 的最大距离
        arm_idx = [i for i, n in enumerate(names) if n.startswith("right_arm") or "gripper" in n and n.startswith("right")]
        if arm_idx:
            sh = None
            for i, n in enumerate(names):
                if n == "torso_pitch_link" or n == "base_link":
                    sh = i
            maxr = 0.0
            if sh is not None:
                for i in arm_idx:
                    maxr = max(maxr, float(torch.norm(pos[i] - pos[sh])))
            print(f"RESULT arm_extent_from_base {maxr:.3f}")
        env.close()


if __name__ == "__main__":
    main()

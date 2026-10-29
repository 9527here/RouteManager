"""探针：把 RouteManager 那个机器人 USD 当 Isaac Lab articulation 加载，打印 DOF。"""
import argparse

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(conflict_handler="resolve")
parser.add_argument("--physics", default="isaacsim_physx", choices=["isaacsim_physx"],
                    help="Physics backend.")
add_launcher_args(parser)
parser.set_defaults(headless=True, visualizer=[])
args_cli = parser.parse_args()

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.physics import PhysicsCfg
from isaaclab.sim import SimulationCfg, SimulationContext

from valve_rl.tasks.valve_env_cfg import ROBOT_USD as ROBOT


def main():
    with launch_simulation(cfg=PhysicsCfg(), launcher_args=args_cli) as physics_cfg:
        sim = SimulationContext(SimulationCfg(physics=physics_cfg))
        cfg = ArticulationCfg(
            prim_path="/World/Robot",
            spawn=sim_utils.UsdFileCfg(usd_path=ROBOT),
            init_state=ArticulationCfg.InitialStateCfg(),
            actuators={
                "all": ImplicitActuatorCfg(
                    joint_names_expr=[".*"], stiffness=20.0, damping=1.0
                )
            },
        )
        art = Articulation(cfg)
        sim.reset()

        names = list(art.joint_names)
        print(f"RESULT dof={len(names)}")
        print("DOF_NAMES " + ",".join(names))


if __name__ == "__main__":
    main()

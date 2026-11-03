"""拧阀门（valve handwheel turning）manager-based 环境配置。

机器人 / 阀门资产来自 RouteManager 那边整理好的 L2 仿真运行资产：
  /home/jy/datasets/vla_data/L2_仿真运行/机器人/humanoid_upper_body_v3_ctag2f120_rm65/
  /home/jy/datasets/vla_data/L2_仿真运行/场景/操作台_piper_table_v2/asserts/valve/valve_1/

设计取舍：
  * 场景里**不放桌子**。桌子资产内部自带一个 valve_asset 子 prim，若同时放进来会
    变成第二个 articulation，物理上互相干扰。阀门直接固定在桌面高度。
  * 只驱动**右臂 6 个关节**，其余关节给极小增益当被动件。
  * 阀门的 wheel_joint 无驱动（自由旋转），奖励按"每步转过的角度"给稠密信号。
"""
import os
from dataclasses import MISSING

from isaaclab_physx.physics import PhysxCfg

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab.visualizers import VisualizerCfg

from isaaclab.envs import mdp as base_mdp

from . import mdp

# 资产根目录：默认指向 RouteManager 资产整理后的位置，可用环境变量覆盖。
# 这两个 USD 不在本仓库内，获取方式见 valve_rl/README.md。
ASSETS = os.environ.get("VALVE_ASSETS_ROOT", "/home/jy/datasets/vla_data/L2_仿真运行")
ROBOT_USD = f"{ASSETS}/机器人/humanoid_upper_body_v3_ctag2f120_rm65/humanoid_upper_body_v3_ctag2f120_rm65.usda"
VALVE_USD = f"{ASSETS}/场景/操作台_piper_table_v2/asserts/valve/valve_1/valve_asset.usd"

ARM_JOINTS = [f"right_arm_{i}_joint" for i in range(1, 7)]
VALVE_JOINT = ["wheel_joint"]

#: 这个资产真实的 50 个 DOF（probe_robot.py 实测得到），用于过滤 gains.json
ROBOT_JOINTS = frozenset(
    """lift_joint caster_front_left_joint caster_rear_left_joint caster_front_right_joint
    caster_rear_right_joint torso_pitch_joint front_left_wheel_joint rear_left_wheel_joint
    front_right_wheel_joint rear_right_wheel_joint head_yaw_joint head_pitch_joint
    left_arm_1_joint right_arm_1_joint left_arm_2_joint right_arm_2_joint left_arm_3_joint
    right_arm_3_joint left_arm_4_joint right_arm_4_joint left_arm_5_joint right_arm_5_joint
    left_arm_6_joint right_arm_6_joint left_camera_rojoint right_camera_rojoint
    left_gripper_right_2_joint left_gripper_right_3_joint left_gripper_right_main_joint
    left_gripper_left_2_joint left_gripper_left_3_joint left_gripper_left_main_joint
    right_gripper_right_2_joint right_gripper_right_3_joint right_gripper_right_main_joint
    right_gripper_left_2_joint right_gripper_left_3_joint right_gripper_left_main_joint
    left_gripper_right_in_joint left_gripper_right_1_joint left_gripper_right_slip_joint
    left_gripper_left_in_joint left_gripper_left_1_joint left_gripper_left_slip_joint
    right_gripper_right_in_joint right_gripper_right_1_joint right_gripper_right_slip_joint
    right_gripper_left_in_joint right_gripper_left_1_joint right_gripper_left_slip_joint""".split()
)

#: 项目给这个机器人烘好的按关节增益（stiffness/damping，USD 级数值）
GAINS_ROOT = os.environ.get(
    "VALVE_GAINS_ROOT",
    "/home/jy/datasets/vla_data/L1_制作资产/载体设备/humanbody_urdf",
)
GAINS_JSON = f"{GAINS_ROOT}/humanoid_upper_body_v3/urdf/humanoid_upper_body_v3_ctag2f120_rm65_isaac.gains.json"


def _build_actuators() -> dict:
    """按 gains.json 把关节分组，每组一个 ImplicitActuatorCfg。

    相同 (stiffness, damping) 的关节并成一组，避免 50 个组。没在 gains.json 里的
    关节不覆盖，保留 USD 自带的 drive。
    """
    import json

    try:
        with open(GAINS_JSON) as f:
            gains = json.load(f)
    except Exception as e:  # pragma: no cover
        print(f"[valve_env_cfg] 读不到 gains.json（{e}），退回资产自带 drive")
        return {}

    buckets: dict[tuple[float, float], list[str]] = {}
    for name, g in gains.items():
        if not isinstance(g, dict):
            continue
        # gains.json 有 56 条，但实际 DOF 只有 50 —— 多出来的是别的变体的关节名。
        # Isaac Lab 要求每条 joint_names_expr 都必须匹配到真实关节，所以先取交集。
        if name not in ROBOT_JOINTS:
            continue
        key = (round(float(g.get("stiffness", 0.0)), 3), round(float(g.get("damping", 0.0)), 3))
        buckets.setdefault(key, []).append(name)

    out = {}
    for i, ((k, d), names) in enumerate(sorted(buckets.items())):
        out[f"g{i:02d}_k{int(k)}_d{int(d)}"] = ImplicitActuatorCfg(
            joint_names_expr=names, stiffness=k, damping=d
        )
    print(f"[valve_env_cfg] 按 gains.json 建了 {len(out)} 个 actuator 组，覆盖 {len(gains)} 个关节")
    return out

##
# 资产
##

ROBOT_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(usd_path=ROBOT_USD, activate_contact_sensors=False),
    init_state=ArticulationCfg.InitialStateCfg(
        # 脚轮挂在底盘下方 0.329m，抬起来让轮子落在 z=0
        pos=(0.0, 0.0, 0.329),
        # 右臂朝阀门的前伸姿态：probe_pose.py 随机采样 + 正运动学搜出来的，
        # 末端距手轮约 0.22 m（零姿态时臂是垂下的，离得很远，随机探索撞不到）。
        joint_pos={
            "right_arm_1_joint": 0.6165,
            "right_arm_2_joint": 1.3609,
            "right_arm_3_joint": 0.7332,
            "right_arm_4_joint": 1.1585,
            "right_arm_5_joint": -0.4691,
            "right_arm_6_joint": 1.8080,
        },
    ),
    # **增益直接读项目烘好的 gains.json**，不要自己拍脑袋写。
    #
    # 踩过的坑：一开始按"只有右臂要被驱动"的思路，把其余关节设成 stiffness=0，
    # 结果升降轴扛不住上半身，零动作时就自己往下塌（lift_joint 漂 -0.120 rad），
    # 左臂也垂下来 —— GUI 里看就是"一直升降、手臂莫名其妙地动"。
    #
    # 换成拍的中等值（45 / 1000）仍然不够：这个资产是 cm 级换算过的，官方标定值
    # 比直觉大 2~4 个数量级（lift_joint = 182356）。这些值 RouteManager 早已烘进
    # USD 的 drive 里，这里只要按同样数值分组，就不会把它们覆盖成错的。
    actuators=_build_actuators(),
)

VALVE_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Valve",
    # 资产是厘米制，整体缩到 0.01
    spawn=sim_utils.UsdFileCfg(usd_path=VALVE_USD, scale=(0.01, 0.01, 0.01)),
    init_state=ArticulationCfg.InitialStateCfg(
        # 原来在 x=0.75（臂展 74%），随机探索几乎撞不到；挪到 0.52 降到约 50%。
        pos=(0.52, 0.0, 0.80),
        joint_pos={"wheel_joint": 0.0},
    ),
    actuators={
        # 手轮不加驱动：靠机器人去拧它
        "wheel": ImplicitActuatorCfg(joint_names_expr=VALVE_JOINT, stiffness=0.0, damping=0.5),
    },
)


##
# 场景 / MDP
##


@configclass
class ValveSceneCfg(InteractiveSceneCfg):
    """机器人 + 阀门 + 地面 + 灯光。"""

    robot: ArticulationCfg = ROBOT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    valve: ArticulationCfg = VALVE_CFG.replace(prim_path="{ENV_REGEX_NS}/Valve")

    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)),
    )
    dome = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(intensity=800.0, color=(1.0, 1.0, 1.0)),
    )



#: 夹爪开合：主动关节 right_2 = 0~0.7，其余按 URDF mimic 链 ±1 / -0.15 跟随
_GRIP_RATIO = {
    "right_gripper_right_2_joint": 1.0,
    "right_gripper_right_in_joint": 1.0,
    "right_gripper_right_3_joint": 1.0,
    "right_gripper_right_main_joint": 1.0,
    "right_gripper_right_slip_joint": 1.0,
    "right_gripper_right_1_joint": -0.15,
    "right_gripper_left_2_joint": 1.0,
    "right_gripper_left_in_joint": 1.0,
    "right_gripper_left_slip_joint": 1.0,
    "right_gripper_left_3_joint": -1.0,
    "right_gripper_left_main_joint": -1.0,
    "right_gripper_left_1_joint": -0.15,
}
GRIP_OPEN = {k: round(0.7 * r, 4) for k, r in _GRIP_RATIO.items()}
GRIP_CLOSE = {k: 0.0 for k in _GRIP_RATIO}

@configclass
class ActionsCfg:
    """右臂 6 关节位置 + 夹爪开合。"""

    arm_action: base_mdp.JointPositionActionCfg = base_mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=ARM_JOINTS,
        scale=0.5,
        use_default_offset=True,
    )
    gripper_action: base_mdp.BinaryJointPositionActionCfg = base_mdp.BinaryJointPositionActionCfg(
        asset_name="robot",
        joint_names=list(GRIP_OPEN.keys()),
        open_command_expr=GRIP_OPEN,
        close_command_expr=GRIP_CLOSE,
    )


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        joint_pos = ObsTerm(func=base_mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=base_mdp.joint_vel_rel)
        valve_pos = ObsTerm(
            func=base_mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("valve", joint_names=VALVE_JOINT)},
        )
        valve_vel = ObsTerm(
            func=base_mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("valve", joint_names=VALVE_JOINT)},
        )
        actions = ObsTerm(func=base_mdp.last_action, clip=(-5.0, 5.0))

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    reset_scene = EventTerm(func=base_mdp.reset_scene_to_default, mode="reset")
    reset_valve = EventTerm(
        func=mdp.reset_valve_progress, mode="reset", params={}
    )


@configclass
class RewardsCfg:
    """分阶段（v2，全部平滑核）：张开 → 从上接近 → 下降 → 抓握 → 拧动 → 松开 → 收回。"""

    # 阶段0：远离时张开夹爪
    open_first = RewTerm(func=mdp.open_when_far, weight=0.5,
                         params={"robot_cfg": SceneEntityCfg("robot", body_names=["right_gripper_base_link"]), "valve_cfg": SceneEntityCfg("valve", body_names=["wheel_link"]), "gripper_cfg": SceneEntityCfg("robot", joint_names=["right_gripper_right_2_joint"])})
    # 阶段1：水平靠近且不掉到下方
    approach = RewTerm(func=mdp.approach_soft, weight=2.0,
                       params={"robot_cfg": SceneEntityCfg("robot", body_names=["right_gripper_base_link"]), "valve_cfg": SceneEntityCfg("valve", body_names=["wheel_link"])})
    # 阶段2a：对齐后下降（原来完全没有垂直梯度，所以只会悬停）
    descend = RewTerm(func=mdp.descend_to_valve, weight=3.0,
                      params={"robot_cfg": SceneEntityCfg("robot", body_names=["right_gripper_base_link"]), "valve_cfg": SceneEntityCfg("valve", body_names=["wheel_link"])})
    # 阶段2b：靠近 × 合爪
    grasp = RewTerm(func=mdp.grasp_soft, weight=4.0,
                    params={"robot_cfg": SceneEntityCfg("robot", body_names=["right_gripper_base_link"]), "valve_cfg": SceneEntityCfg("valve", body_names=["wheel_link"]), "gripper_cfg": SceneEntityCfg("robot", joint_names=["right_gripper_right_2_joint"])})
    # 阶段3：抓着时才计入拧动角度
    turn = RewTerm(func=mdp.turn_grasped_soft, weight=20.0,
                   params={"valve_cfg": SceneEntityCfg("valve", joint_names=VALVE_JOINT),
                            "robot_cfg": SceneEntityCfg("robot", body_names=["right_gripper_base_link"]), "gripper_cfg": SceneEntityCfg("robot", joint_names=["right_gripper_right_2_joint"])})
    # 阶段4/5：转够后松爪、收回
    release = RewTerm(func=mdp.release_after_turn, weight=3.0,
                      params={"robot_cfg": SceneEntityCfg("robot", body_names=["right_gripper_base_link"]), "valve_cfg": SceneEntityCfg("valve", body_names=["wheel_link"]), "gripper_cfg": SceneEntityCfg("robot", joint_names=["right_gripper_right_2_joint"])})
    retract = RewTerm(func=mdp.retract_after_release, weight=3.0,
                      params={"robot_cfg": SceneEntityCfg("robot", body_names=["right_gripper_base_link"]), "valve_cfg": SceneEntityCfg("valve", body_names=["wheel_link"]), "gripper_cfg": SceneEntityCfg("robot", joint_names=["right_gripper_right_2_joint"])})
    action_rate = RewTerm(func=base_mdp.action_rate_l2, weight=-1e-2)
    joint_vel = RewTerm(func=base_mdp.joint_vel_l2, weight=-1e-4)


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=base_mdp.time_out, time_out=True)
    # 整套动作做完：转够 → 松开 → 收回
    sequence_done = DoneTerm(
        func=mdp.sequence_complete,
        params={
            "robot_cfg": SceneEntityCfg("robot", body_names=["right_gripper_base_link"]),
            "valve_cfg": SceneEntityCfg("valve", body_names=["wheel_link"]),
            "gripper_cfg": SceneEntityCfg("robot", joint_names=["right_gripper_right_2_joint"]),
        },
    )


@configclass
class ValveTurnEnvCfg(ManagerBasedRLEnvCfg):
    """拧阀门环境。"""

    sim: SimulationCfg = SimulationCfg(
        dt=1.0 / 60.0,
        render_interval=4,
        physics=PhysxCfg(bounce_threshold_velocity=0.01),
        default_visualizer_cfg=VisualizerCfg(eye=(1.6, -1.6, 1.4), lookat=(0.6, 0.0, 0.8)),
    )
    scene: ValveSceneCfg = ValveSceneCfg(num_envs=64, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()

    decimation: int = 4
    episode_length_s: float = 6.0

    def __post_init__(self):
        self.sim.device = "cuda:0"
        self.scene.num_envs = self.scene.num_envs

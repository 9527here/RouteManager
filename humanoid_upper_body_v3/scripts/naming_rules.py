#!/usr/bin/env python3
"""规则化命名映射 —— 唯一权威表,三个消费者共用:

- build_isaac_urdf.py:  写 URDF 前把内部构建名重命名成规则名(最后一步)
- finalize_usd.py:      用反向表把新 prim 名映射回内部构建名查增益表
- gen_joint_config.py:  用反向表查 drive_gains.json(源名带连字符)

规则依据: /home/jy/文档/urdf命名规则
  小写 a-z + 数字 + 下划线、语义化、层级从大到小、
  轮子 front|rear_<side>_wheel_link/joint、万向轮 caster_<位置>_link/joint、
  升降 lift_top_link + lift_joint、手臂 left/right_ 前缀按基座→末端编号。

映射来源(源 CAD 字母 → 语义):
  Z=左 Y=右 Q=前 H=后  SX=升降(SX_Link/枝)  XQ=躯干俯仰级  TZ/TF=头 yaw/pitch
  Z1/Y1=臂座(固定)  Z2..Z7/Y2..Y7=臂关节 1..6  ZJ/YJ=腕回转
  ZQY/ZHY/YQY/YHY=万向轮支架(位置锁定)  ZQL/ZHL/YQL/YHL=从动轮
  夹爪 ctag2f120(源 _ctag2f120.urdf 变体): ZJ_/YJ_ 后接 R/L 指 + Main/in/Slip,
  见下方夹爪段; 指内 right/left 与整臂左右无关。

注意: 网格文件名(TF_0.STL 等)保持源 CAD 命名不动 —— 只管溯源,
命名规则只约束 link/joint/prim 模型名。
"""

# ---- 内部构建名(= 源名,连字符已换下划线) → 规则名 ----
RENAME_LINKS = {
    "base_link": "base_link",          # 根,本身合规
    "SX_Link": "lift_top_link",
    "XQ": "torso_pitch_link",
    "TZ": "head_yaw_link",
    "TF": "head_pitch_link",
    "Z1": "left_arm_base_link",
    "Z2": "left_arm_1_link",
    "Z3": "left_arm_2_link",
    "Z4": "left_arm_3_link",
    "Z5": "left_arm_4_link",
    "Z6": "left_arm_5_link",
    "Z7": "left_arm_6_link",
    "ZJ": "left_wrist_link",
    "Y1": "right_arm_base_link",
    "Y2": "right_arm_1_link",
    "Y3": "right_arm_2_link",
    "Y4": "right_arm_3_link",
    "Y5": "right_arm_4_link",
    "Y6": "right_arm_5_link",
    "Y7": "right_arm_6_link",
    "YJ": "right_wrist_link",
    "ZQY": "caster_front_left_link",
    "ZQL": "front_left_wheel_link",
    "ZHY": "caster_rear_left_link",
    "ZHL": "rear_left_wheel_link",
    "YQY": "caster_front_right_link",
    "YQL": "front_right_wheel_link",
    "YHY": "caster_rear_right_link",
    "YHL": "rear_right_wheel_link",
}

RENAME_JOINTS = {
    "SX_JOINT": "lift_joint",                 # prismatic z
    "XQ_JOINT": "torso_pitch_joint",          # revolute y
    "TZ_JOINT": "head_yaw_joint",
    "TF_JOINT": "head_pitch_joint",
    # fixed 臂座安装 —— 规则文档 3.3.4 明文规定 lift_to_<侧>_arm_joint
    # (物理父级是 XQ 躯干俯仰级,但按文档命名)
    "Z1_JOINT": "lift_to_left_arm_joint",
    "Z2_JOINT": "left_arm_1_joint",
    "Z3_JOINT": "left_arm_2_joint",
    "Z4_JOINT": "left_arm_3_joint",
    "Z5_JOINT": "left_arm_4_joint",
    "Z6_JOINT": "left_arm_5_joint",
    "Z7_JOINT": "left_arm_6_joint",
    "ZJ_JOINT": "left_wrist_joint",
    "Y1_JOINT": "lift_to_right_arm_joint",    # fixed 臂座安装(同上)
    "Y2_JOINT": "right_arm_1_joint",
    "Y3_JOINT": "right_arm_2_joint",
    "Y4_JOINT": "right_arm_3_joint",
    "Y5_JOINT": "right_arm_4_joint",
    "Y6_JOINT": "right_arm_5_joint",
    "Y7_JOINT": "right_arm_6_joint",
    "YJ_JOINT": "right_wrist_joint",
    "ZQY_JOINT": "caster_front_left_joint",
    "ZQL_JOINT": "front_left_wheel_joint",
    "ZHY_JOINT": "caster_rear_left_joint",
    "ZHL_JOINT": "rear_left_wheel_joint",
    "YQY_JOINT": "caster_front_right_joint",
    "YQL_JOINT": "front_right_wheel_joint",
    "YHY_JOINT": "caster_rear_right_joint",
    "YHL_JOINT": "rear_right_wheel_joint",
}

# ---- 夹爪 ctag2f120(2026-09-15 加入, 用户拍板"数字式"扩展) ----
# 规则文档 3.3.3 只定义到单个 <侧>_gripper_joint, 多指节夹爪按同一精神扩展:
#   <侧>_gripper_<指>_<序号|部件>_link/joint
# ZJ=左腕侧 YJ=右腕侧(整臂的左右); 指内 right/left 是 ctag2f120 自身两指,
# 与整臂左右无关 —— 左手的 "ZJ_Right_Main" 也映射到 left_gripper_right_*。
# 每手 15 关节: 挂载(fixed) + 两指×(3 指节 + main/in/slip 耦合) + 两 pad(fixed)。
_GRIP_SIDES = {"ZJ": "left", "YJ": "right"}
_GRIP_FINGERS = (("R", "Right", "right"), ("L", "Left", "left"))
for _side, _s in _GRIP_SIDES.items():
    RENAME_LINKS[f"{_side}_gripper_base_link"] = f"{_s}_gripper_base_link"
    RENAME_JOINTS[f"{_side}_gripper_mount"] = f"{_s}_wrist_to_gripper_joint"
    for _a, _full, _f in _GRIP_FINGERS:
        for _n in "123":
            RENAME_LINKS[f"{_side}_{_a}{_n}_Link"] = f"{_s}_gripper_{_f}_{_n}_link"
            RENAME_JOINTS[f"{_side}_{_a}{_n}"] = f"{_s}_gripper_{_f}_{_n}_joint"
        for _p in ("Main", "in", "Slip"):
            RENAME_LINKS[f"{_side}_{_full}_{_p}_Link"] = f"{_s}_gripper_{_f}_{_p.lower()}_link"
            RENAME_JOINTS[f"{_side}_{_full}_{_p}"] = f"{_s}_gripper_{_f}_{_p.lower()}_joint"
        RENAME_LINKS[f"{_side}_{_full}_Pad_Link"] = f"{_s}_gripper_{_f}_pad_link"
        RENAME_JOINTS[f"{_side}_{_full}_Pad"] = f"{_s}_gripper_{_f}_pad_joint"

# 反向表: 规则名 → 内部构建名
REVERSE_LINKS = {v: k for k, v in RENAME_LINKS.items() if k != v}
REVERSE_JOINTS = {v: k for k, v in RENAME_JOINTS.items() if k != v}

# 快捷集合(规则名)
WHEEL_LINKS = {"front_left_wheel_link", "rear_left_wheel_link",
               "front_right_wheel_link", "rear_right_wheel_link"}
WHEEL_JOINTS = {"front_left_wheel_joint", "rear_left_wheel_joint",
                "front_right_wheel_joint", "rear_right_wheel_joint"}


def joint_source_key(rule_name: str) -> str:
    """规则名 → drive_gains/joint_dynamics.json 的源键('SX-JOINT' 带连字符)。"""
    built = REVERSE_JOINTS.get(rule_name, rule_name)
    return built.replace("_JOINT", "-JOINT")

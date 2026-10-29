"""阀门任务的 MDP 自定义项。"""
from __future__ import annotations

import torch

from isaaclab.envs import ManagerBasedRLEnv


def _ee_to_valve_distance(env: ManagerBasedRLEnv, robot_cfg, valve_cfg) -> torch.Tensor:
    """末端的某个 body 到阀门手轮 body 的距离（米）。"""
    robot = env.scene[robot_cfg.name]
    valve = env.scene[valve_cfg.name]
    ee = robot.data.body_pos_w[:, _bid(robot_cfg)]
    tgt = valve.data.body_pos_w[:, _bid(valve_cfg)]
    return torch.norm(tgt - ee, dim=-1)


def approach_valve(env: ManagerBasedRLEnv, robot_cfg, valve_cfg, std: float = 0.25) -> torch.Tensor:
    """稠密引导：末端越靠近手轮奖励越高（0~1）。

    没有这一项时，只有"拧动"的稀疏奖励，策略几乎不可能随机探索到阀门位置。
    """
    d = _ee_to_valve_distance(env, robot_cfg, valve_cfg)
    return torch.exp(-d / std)


def ee_valve_distance(env: ManagerBasedRLEnv, robot_cfg, valve_cfg) -> torch.Tensor:
    """观测用：末端到阀门手轮的距离。"""
    return _ee_to_valve_distance(env, robot_cfg, valve_cfg)


def valve_angle(env: ManagerBasedRLEnv, asset_cfg) -> torch.Tensor:
    """手轮关节当前角度（rad）。"""
    asset = env.scene[asset_cfg.name]
    return asset.data.joint_pos[:, asset_cfg.joint_ids]


def valve_angle_vel(env: ManagerBasedRLEnv, asset_cfg) -> torch.Tensor:
    """手轮关节角速度（rad/s）。"""
    asset = env.scene[asset_cfg.name]
    return asset.data.joint_vel[:, asset_cfg.joint_ids]


def valve_turn_progress(
    env: ManagerBasedRLEnv,
    asset_cfg,
    direction: float = 1.0,
) -> torch.Tensor:
    """稠密奖励：本步手轮沿目标方向转过的角度。

    每步增量式（不是绝对值），所以策略可以连续多圈旋转，不会因为转多了反而掉分。
    """
    asset = env.scene[asset_cfg.name]
    prev = env.extras.setdefault("_valve_prev_angle", torch.zeros(env.num_envs, 1, device=env.device))
    cur = asset.data.joint_pos[:, asset_cfg.joint_ids]
    delta = (cur - prev) * direction
    env.extras["_valve_prev_angle"] = cur.clone()
    return delta.squeeze(-1)


def valve_turned(env: ManagerBasedRLEnv, asset_cfg, threshold: float = 1.5, direction: float = 1.0) -> torch.Tensor:
    """终止条件：手轮累计转过 threshold 弧度。"""
    asset = env.scene[asset_cfg.name]
    cur = asset.data.joint_pos[:, asset_cfg.joint_ids]
    return (cur * direction).squeeze(-1) > threshold


def reset_valve_progress(env: ManagerBasedRLEnv, env_ids) -> None:
    """重置时清掉增量奖励的上一帧缓存。"""
    if "_valve_prev_angle" in env.extras:
        env.extras["_valve_prev_angle"][env_ids] = 0.0
    if "_vprev" in env.extras:
        env.extras["_vprev"][env_ids] = 0.0



def _bid(cfg) -> int:
    """SceneEntityCfg.body_ids 同样可能是 slice。"""
    ids = cfg.body_ids
    if isinstance(ids, slice):
        return ids.start or 0
    return ids[0]


def _jid(cfg) -> int:
    """SceneEntityCfg.joint_ids 可能是 slice（选中全部关节），统一取第一个下标。"""
    ids = cfg.joint_ids
    if isinstance(ids, slice):
        return ids.start or 0
    return ids[0]


# ---------------------------------------------------------------------------
# 分阶段任务：张开 → 从上接近 → 抓握 → 拧动 → 松开 → 收回
# ---------------------------------------------------------------------------

ACTIVE_GRIPPER_JOINT = "right_gripper_right_2_joint"


def gripper_openness(env: ManagerBasedRLEnv, asset_cfg) -> torch.Tensor:
    """夹爪张开程度 0(全闭)~1(全开)，用主动关节角 / 上限得到。"""
    asset = env.scene[asset_cfg.name]
    q = asset.data.joint_pos[:, _jid(asset_cfg)]
    return torch.clamp(q / 0.7, 0.0, 1.0)


def approach_from_above(env, robot_cfg, valve_cfg, std: float = 0.20,
                        above_h: float = 0.25) -> torch.Tensor:
    """阶段1：从上方向手轮接近。要求末端在阀门上方 above_h 以内。"""
    robot = env.scene[robot_cfg.name]
    valve = env.scene[valve_cfg.name]
    ee = robot.data.body_pos_w[:, _bid(robot_cfg)]
    tgt = valve.data.body_pos_w[:, _bid(valve_cfg)]
    d_xy = torch.norm(ee[:, :2] - tgt[:, :2], dim=-1)
    dz = ee[:, 2] - tgt[:, 2]
    above = (dz > 0.0) & (dz < above_h)
    return torch.exp(-d_xy / std) * above.float()


def grasp_valve(env, robot_cfg, valve_cfg, gripper_cfg, close_thresh: float = 0.25,
                max_dist: float = 0.12) -> torch.Tensor:
    """阶段2：手轮附近把夹爪合上。越近且合得越紧奖励越高。"""
    robot = env.scene[robot_cfg.name]
    valve = env.scene[valve_cfg.name]
    ee = robot.data.body_pos_w[:, _bid(robot_cfg)]
    tgt = valve.data.body_pos_w[:, _bid(valve_cfg)]
    near = (torch.norm(ee - tgt, dim=-1) < max_dist).float()
    closed = 1.0 - gripper_openness(env, gripper_cfg)
    return near * torch.clamp((closed - (1.0 - close_thresh)) / close_thresh, 0.0, 1.0)


def turn_while_grasping(env, valve_cfg, robot_cfg, gripper_cfg,
                        max_dist: float = 0.15, close_thresh: float = 0.4) -> torch.Tensor:
    """阶段3：**抓着**的时候手轮转过的角度才算数（否则推着转也得分）。"""
    asset = env.scene[valve_cfg.name]
    robot = env.scene[robot_cfg.name]
    prev = env.extras.setdefault("_vprev", torch.zeros(env.num_envs, 1, device=env.device))
    cur = asset.data.joint_pos[:, valve_cfg.joint_ids]
    delta = (cur - prev).squeeze(-1)
    env.extras["_vprev"] = cur.clone()

    ee = robot.data.body_pos_w[:, _bid(robot_cfg)]
    tgt = asset.data.body_pos_w[:, _bid(valve_cfg)]
    near = (torch.norm(ee - tgt, dim=-1) < max_dist).float()
    closed = (1.0 - gripper_openness(env, gripper_cfg)) > close_thresh
    return delta * near * closed.float()


def release_after_turn(env, robot_cfg, valve_cfg, gripper_cfg,
                       turned: float = 1.5, max_dist: float = 0.15) -> torch.Tensor:
    """阶段4：转够角度之后再张开夹爪。"""
    asset = env.scene[valve_cfg.name]
    robot = env.scene[robot_cfg.name]
    angle = asset.data.joint_pos[:, _jid(valve_cfg)]
    ee = robot.data.body_pos_w[:, _bid(robot_cfg)]
    tgt = asset.data.body_pos_w[:, _bid(valve_cfg)]
    near = (torch.norm(ee - tgt, dim=-1) < max_dist).float()
    done = (angle > turned).float()
    return done * near * gripper_openness(env, gripper_cfg)


def retract_after_release(env, robot_cfg, valve_cfg, gripper_cfg,
                          turned: float = 1.5, far: float = 0.30) -> torch.Tensor:
    """阶段5：松开后把手臂收回去。"""
    asset = env.scene[valve_cfg.name]
    robot = env.scene[robot_cfg.name]
    angle = asset.data.joint_pos[:, _jid(valve_cfg)]
    ee = robot.data.body_pos_w[:, _bid(robot_cfg)]
    tgt = asset.data.body_pos_w[:, _bid(valve_cfg)]
    dist = torch.norm(ee - tgt, dim=-1)
    opened = gripper_openness(env, gripper_cfg)
    return (angle > turned).float() * opened * torch.clamp((dist - far) / 0.3, 0.0, 1.0)


def sequence_complete(env, robot_cfg, valve_cfg, gripper_cfg,
                      turned: float = 1.5, far: float = 0.35) -> torch.Tensor:
    """终止：转够 → 松开 → 收回，整套动作做完。"""
    asset = env.scene[valve_cfg.name]
    robot = env.scene[robot_cfg.name]
    angle = asset.data.joint_pos[:, _jid(valve_cfg)]
    ee = robot.data.body_pos_w[:, _bid(robot_cfg)]
    tgt = asset.data.body_pos_w[:, _bid(valve_cfg)]
    dist = torch.norm(ee - tgt, dim=-1)
    opened = gripper_openness(env, gripper_cfg) > 0.7
    return (angle > turned) & opened & (dist > far)


# ---------------------------------------------------------------------------
# v2：把硬阈值换成平滑核，让每个阶段都有梯度
# ---------------------------------------------------------------------------

def _dist3(env, robot_cfg, valve_cfg):
    robot = env.scene[robot_cfg.name]
    valve = env.scene[valve_cfg.name]
    ee = robot.data.body_pos_w[:, _bid(robot_cfg)]
    tgt = valve.data.body_pos_w[:, _bid(valve_cfg)]
    return ee, tgt, torch.norm(ee - tgt, dim=-1)


def approach_soft(env, robot_cfg, valve_cfg, xy_std: float = 0.18, below_std: float = 0.12):
    """阶段1（平滑版）：水平靠近，且不要掉到阀门下方。"""
    ee, tgt, _ = _dist3(env, robot_cfg, valve_cfg)
    d_xy = torch.norm(ee[:, :2] - tgt[:, :2], dim=-1)
    dz = ee[:, 2] - tgt[:, 2]
    penal = torch.exp(-torch.clamp(-dz, min=0.0) / below_std)
    return torch.exp(-d_xy / xy_std) * penal


def descend_to_valve(env, robot_cfg, valve_cfg, xy_std: float = 0.10, z_std: float = 0.08):
    """阶段2a：水平对齐后往下走 —— 补上原来完全没有的垂直梯度。"""
    ee, tgt, _ = _dist3(env, robot_cfg, valve_cfg)
    d_xy = torch.norm(ee[:, :2] - tgt[:, :2], dim=-1)
    dz = ee[:, 2] - tgt[:, 2]
    return torch.exp(-d_xy / xy_std) * torch.exp(-torch.abs(dz) / z_std)


def open_when_far(env, robot_cfg, valve_cfg, gripper_cfg, far: float = 0.22, far_std: float = 0.12):
    """阶段0：远离阀门时把夹爪张开（否则策略永远学不会"先张开"）。"""
    _, _, d = _dist3(env, robot_cfg, valve_cfg)
    farness = torch.clamp((d - far) / far_std, 0.0, 1.0)
    return farness * gripper_openness(env, gripper_cfg)


def grasp_soft(env, robot_cfg, valve_cfg, gripper_cfg, dist_std: float = 0.07,
               close_thresh: float = 0.4):
    """阶段2b（平滑版）：靠近 × 合爪，处处有梯度。"""
    _, _, d = _dist3(env, robot_cfg, valve_cfg)
    near = torch.exp(-d / dist_std)
    closed = 1.0 - gripper_openness(env, gripper_cfg)
    return near * torch.clamp((closed - (1.0 - close_thresh)) / close_thresh, 0.0, 1.0)


def turn_grasped_soft(env, valve_cfg, robot_cfg, gripper_cfg, dist_std: float = 0.07,
                      close_thresh: float = 0.4):
    """阶段3（平滑版）：抓着时转过的角度；远处/松开的转动按平滑权重衰减。"""
    asset = env.scene[valve_cfg.name]
    prev = env.extras.setdefault("_vprev", torch.zeros(env.num_envs, 1, device=env.device))
    cur = asset.data.joint_pos[:, valve_cfg.joint_ids]
    delta = (cur - prev).squeeze(-1)
    env.extras["_vprev"] = cur.clone()

    _, _, d = _dist3(env, robot_cfg, valve_cfg)
    near = torch.exp(-d / dist_std)
    closed = 1.0 - gripper_openness(env, gripper_cfg)
    gate = torch.clamp((closed - (1.0 - close_thresh)) / close_thresh, 0.0, 1.0)
    return delta * near * gate

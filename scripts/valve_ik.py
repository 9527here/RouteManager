"""阀门脚本共用的离线 IK：只依赖 URDF FK、numpy 和 scipy。

角误差与数值雅可比都在世界系；SO(3) 对数在 180° 处仍保留旋转量，
避免反对称部分 sin(theta) 为零而把翻转姿态误判成收敛。
"""
import math

import numpy as np
from scipy.spatial.transform import Rotation

import arm_kin


def rotation_error(R_current, R_target):
    """从当前朝向到目标朝向的世界系旋转向量，单位 rad。"""
    return Rotation.from_matrix(
        np.asarray(R_target, float) @ np.asarray(R_current, float).T
    ).as_rotvec()


def rotation_angle(R_current, R_target):
    """真实最短旋转角，单位度；与求解停止判据使用同一误差。"""
    return math.degrees(float(np.linalg.norm(rotation_error(R_current, R_target))))


def ee_world(chain, q, R_ab, t_ab, B=None):
    """预测法兰位姿；B 是标定的 URDF 末端到仿真法兰的常量旋转。"""
    p, R = arm_kin.ee_world(chain, q, R_ab, t_ab)
    return p, R if B is None else R @ B


def solve_ik(chain, q0, tgt_pos, tgt_R, R_ab, t_ab, lim, q_ref=None,
             iters=200, lam=0.03, step_max=0.08, w_rot=0.5, w_posture=0.02,
             position_tol=5e-4, rotation_tol=3e-3):
    """世界系 URDF 末端目标 → (关节角, 位置残差 m, 朝向残差度)。

    若目标描述的是仿真法兰，调用方须先用 ``tgt_R @ B.T`` 转回 URDF
    末端朝向。常量右乘 B 在世界角速度差分中消去，不需修改数值雅可比。
    """
    q0 = np.asarray(q0, float)
    n = len(arm_kin.joint_names(chain))
    lim = np.asarray(lim, float)
    tgt_pos = np.asarray(tgt_pos, float)
    tgt_R = np.asarray(tgt_R, float)
    q_ref = q0.copy() if q_ref is None else np.asarray(q_ref, float)
    if q0.shape != (n,) or q_ref.shape != (n,) or lim.shape != (n, 2):
        raise ValueError("关节角、参考姿态和限位的维度必须与 URDF 可动关节一致")
    if tgt_pos.shape != (3,) or tgt_R.shape != (3, 3):
        raise ValueError("目标位置必须是三维向量，朝向必须是 3×3 矩阵")
    if not all(np.isfinite(v).all() for v in (q0, q_ref, lim, tgt_pos, tgt_R)):
        raise ValueError("IK 输入含 NaN 或无穷值")
    if np.any(lim[:, 1] <= lim[:, 0]):
        raise ValueError("关节上限必须大于下限")
    if lam <= 0 or step_max <= 0 or w_rot <= 0 or iters < 1:
        raise ValueError("阻尼、步长、朝向权重和迭代数必须为正")
    if not all(math.isfinite(v) and v>0 for v in (position_tol,rotation_tol)):
        raise ValueError('IK收敛精度必须有限且为正')

    # 小范围关节也保留可用区间，不能把固定的 0.02rad 余量扣成反向限位。
    margin = np.minimum(0.02, 0.1 * (lim[:, 1] - lim[:, 0]))
    lo, hi = lim[:, 0] + margin, lim[:, 1] - margin
    q = np.clip(q0, lo, hi)
    best = None
    for _ in range(iters):
        p_cur, R_cur = arm_kin.ee_world(chain, q, R_ab, t_ab)
        e_p = tgt_pos - p_cur
        e_r = rotation_error(R_cur, tgt_R)
        err = np.concatenate([e_p, w_rot * e_r])
        score = float(err @ err)
        if best is None or score < best[0]:
            best = (score, q.copy())
        if np.linalg.norm(e_p) < position_tol and np.linalg.norm(e_r) < rotation_tol:
            break

        # arm_kin 的每一行是一个关节，先转置成 (6,n)，才能按空间量分块。
        J = arm_kin.jacobian(chain, q, R_ab, t_ab).T
        J[3:, :] *= w_rot
        dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), err)
        if w_posture:
            # 直接回拉会持续抵消任务修正；阻尼伪逆的“零空间”也会漏入任务。
            # 用 Moore-Penrose 伪逆作严格投影，六轴满秩时自然没有姿态自由度。
            null = np.eye(n) - np.linalg.pinv(J) @ J
            dq += w_posture * (null @ (q_ref - q))
        length = np.linalg.norm(dq)
        if length > step_max:
            dq *= step_max / length
        q = np.clip(q + dq, lo, hi)

    # 不可达或贴限位时可能后期振荡，返回实测残差最小的一次而不是最后一次。
    p_end, R_end = arm_kin.ee_world(chain, q, R_ab, t_ab)
    e_p, e_r = tgt_pos - p_end, rotation_error(R_end, tgt_R)
    if float(e_p @ e_p + w_rot ** 2 * (e_r @ e_r)) > best[0]:
        q = best[1]
        p_end, R_end = arm_kin.ee_world(chain, q, R_ab, t_ab)
    return (q, float(np.linalg.norm(tgt_pos - p_end)),
            rotation_angle(R_end, tgt_R))

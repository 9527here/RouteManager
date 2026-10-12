"""grasp_planner.py — 抓取规划（纯 numpy/scipy，不依赖 Isaac）。

被两处共用：
  · `scripts/pick_object.py`（离线/HTTP 脚本，人在终端敲）
  · `robot_manager.py` 的自动抓取状态机（VR 菜单键触发）

**TCP 必须取"闭合态"**：两指垫的中点随开口沿夹爪接近轴漂移 ~11mm，
用张开态的 TCP 规划会让物体落在指垫外侧（实测闭合后指垫离物体面还有 3~4mm）。
调用方负责在**目标闭合角**下量出 `tcp_local` 再传进来。
"""
import numpy as np

import arm_kin as K


def load_chain(urdf_path, root_link, tip_link):
    """root_link → tip_link 的关节链（按父子顺序）。"""
    return K.load_chain(urdf_path, root_link, tip_link)


def joint_names(chain):
    return K.joint_names(chain)


def joint_limits(chain):
    return K.joint_limits(chain)


def fk(chain, q):
    """正运动学：返回 (R, t)，tip 在 root 系里的位姿（列向量约定）。"""
    return K.fk(chain, q)


def _limit_penalty(q, limits, margin=0.12):
    """贴限位惩罚：离限位越近越大（0 = 远离限位）。"""
    lo = np.maximum(limits[:, 0] + margin - q, 0.0)
    hi = np.maximum(q - (limits[:, 1] - margin), 0.0)
    return float(np.sum(lo + hi))


def plan(chain, q0, limits, T_ab_col, tcp_local, jaw_local, target,
         down=np.array([0, 0, -1.0]), jaw_dir=np.array([0, 1.0, 0]),
         w_pos=20.0, w_dir=1.0, w_jaw=1.0, w_posture=0.05, seeds=None,
         fast=False):
    """求关节角使 TCP 落在 target、TCP 轴指向 down、**虎口张开方向**指向 jaw_dir。

    第三个约束是抓取成败的关键：只锁"朝下"的话腕滚转是自由的，解出来的虎口
    方向是斜的，于是手指**顶在立方体的棱上**——接触面极小、一抬就滑脱。
    锁成对着 ±y 面（正对面夹持）才夹得住。

    **必须多起点 + 限位回避**：7 自由度锁 6 个约束只剩 1 个零空间，单起点很
    容易收敛到"某个关节顶在限位"的假解——实测 `left_arm_2` 卡在 +78.0°(限位)
    时朝下误差 6.5°，夹爪下压时会歪着蹭到物体。多起点取"姿态误差 + 限位惩罚"
    最小的解。

    :param T_ab_col: 手臂基座世界位姿（列向量约定 4x4）
    :param tcp_local: TCP 在**末端(夹爪基座)**系里的位置（闭合态）
    :param jaw_local: 虎口张开方向在末端系里的单位向量
    :param seeds: 额外初始猜测；默认 [q0, 限位中位] 再按中位加扰动
    """
    R_ab, t_ab = T_ab_col[:3, :3], T_ab_col[:3, 3]
    a_local = tcp_local / np.linalg.norm(tcp_local)
    mid = 0.5 * (limits[:, 0] + limits[:, 1])
    span = np.maximum(limits[:, 1] - limits[:, 0], 1e-6)

    def pose_world(q):
        R, t = fk(chain, q)
        R_w = R_ab @ R
        return R_w @ tcp_local + (R_ab @ t + t_ab), R_w @ a_local, R_w @ jaw_local

    def resid(q):
        p, a, j = pose_world(q)
        r = np.concatenate([(p - target) * w_pos,
                            (a - down) * w_dir,
                            (j - jaw_dir) * w_jaw])
        return np.concatenate([r, w_posture * (q - mid) / span])

    from scipy.optimize import least_squares
    if fast:
        # 只跑一个起点: 笛卡尔路点这类"已知邻域内的小步"用它, 每次求解便宜
        # 一个数量级。整个规划要跑上百次 least_squares, 不省就冻住主线程。
        cand = [np.asarray((seeds or [q0])[0], float)]
    else:
        cand = list(seeds or [])
        cand.append(np.asarray(q0, float))
        cand.append(mid.copy())
        for k in (0.3, -0.3):
            cand.append(np.clip(mid + k * span, limits[:, 0] + 1e-3,
                                limits[:, 1] - 1e-3))

    best = None
    for s in cand:
        s = np.clip(np.asarray(s, float), limits[:, 0] + 1e-4,
                    limits[:, 1] - 1e-4)
        try:
            sol = least_squares(resid, s, bounds=(limits[:, 0], limits[:, 1]),
                                xtol=1e-10, ftol=1e-10, max_nfev=800)
        except Exception:
            continue
        q = sol.x
        p, a, j = pose_world(q)
        info = {"pos_err_mm": float(np.linalg.norm(p - target) * 1000),
                "dir_err_deg": float(np.degrees(
                    np.arccos(np.clip(np.dot(a, down), -1.0, 1.0)))),
                "jaw_err_deg": float(np.degrees(
                    np.arccos(np.clip(abs(np.dot(j, jaw_dir)), 0, 1)))),
                "tcp": [round(float(v), 4) for v in p],
                "jaw_world": [round(float(v), 3) for v in j],
                "cost": float(sol.cost),
                "limit_pen": round(_limit_penalty(q, limits), 4),
                "tcp_axis": [round(float(v), 4) for v in a]}
        # 打分：位置(mm) 为主，姿态按角度加权，再罚贴限位
        score = (info["pos_err_mm"] + 8.0 * info["dir_err_deg"]
                 + 8.0 * info["jaw_err_deg"] + 200.0 * info["limit_pen"])
        if best is None or score < best[0]:
            best = (score, q, info)
    if best is None:
        raise RuntimeError("IK 全部起点都失败")
    best[2]["score"] = round(float(best[0]), 3)
    return best[1], best[2]


def plan_grasp(chain, q0, limits, T_ab_col, tcp_local, jaw_local, obj_c,
               approach=0.12, lift=0.20, max_err_mm=15.0, max_ang_deg=12.0,
               descent_steps=8, lift_steps=8,
               body_offsets=(0.0, -0.08, -0.16, -0.24, -0.32, -0.40, -0.48),
               seeds=None, jaw_axes=None):
    """三段轨迹规划，**同时搜索机身升降高度**。

    `lift_joint` 是棱柱关节，整个上装能上下移动。臂够不到/姿态解不开时，
    把机身降一点比硬掰手臂好得多（用户 2026-10-09 指出）。这里对若干机身
    高度各规划一次，取"姿态误差 + 限位惩罚"最小的那个。

    :return: (qs, info)；`info["body_dz"]` 是选中的机身升降量(m)，调用方
        必须先把 `lift_joint` 移动该量再走手臂轨迹。
    """
    best = None
    for score, qs, info, dz in plan_grasp_steps(
            chain, q0, limits, T_ab_col, tcp_local, jaw_local, obj_c,
            approach=approach, lift=lift, max_err_mm=max_err_mm,
            max_ang_deg=max_ang_deg, descent_steps=descent_steps,
            lift_steps=lift_steps,
            body_offsets=body_offsets,
            seeds=seeds, jaw_axes=jaw_axes):
        if best is None or score < best[0]:
            best = (score, qs, info, dz)
    if best is None:
        return None, {"error": "所有机身高度都规划失败"}
    best[2]["body_dz"] = best[3]
    best[2]["plan_score"] = round(float(best[0]), 2)
    return best[1], best[2]


def plan_grasp_steps(chain, q0, limits, T_ab_col, tcp_local, jaw_local, obj_c,
                     approach=0.12, lift=0.20, max_err_mm=15.0,
                     max_ang_deg=12.0, descent_steps=6, lift_steps=6,
                     body_offsets=(0.0, -0.12, -0.24, -0.36, -0.48),
                     seeds=None, jaw_axes=None, accept_score=45.0):
    """**生成器**：每次 next() 试一档机身高度，yield (score, qs, info, dz)。

    为什么要生成器：一次完整规划要跑上百次 least_squares（几秒到几十秒），
    全在仿真主线程里同步跑会把画面彻底冻住（用户实测"按了 X 卡顿很久"）。
    调用方**每帧只 next() 一次**，画面就一直在动。
    某档得分够好（姿态误差小且不贴限位）就直接 return，不再试更低的机身。
    """
    if not body_offsets:
        body_offsets = (0.0,)
    for dz in body_offsets:
        T = np.array(T_ab_col, float).copy()
        T[2, 3] += float(dz)
        qs, info = _plan_grasp_at(chain, q0, limits, T, tcp_local, jaw_local,
                                  obj_c, approach, lift, max_err_mm,
                                  max_ang_deg, descent_steps, lift_steps,
                                  seeds=seeds,
                                  jaw_axes=jaw_axes)
        if qs is None:
            continue
        score = sum(info["infos"][t]["score"] for t in ("pre", "grasp", "lift"))
        yield float(score), qs, info, float(dz)
        if score <= accept_score:
            return


def _plan_grasp_at(chain, q0, limits, T_ab_col, tcp_local, jaw_local, obj_c,
                   approach, lift, max_err_mm, max_ang_deg,
                   descent_steps, lift_steps,
                   seeds=None, jaw_axes=None):
    """给定**固定**手臂基座位姿的三段轨迹规划。

    **先解抓取点，预备/抬起沿用它的实际姿态** —— 三个点各自独立解时姿态会差
    几度，机械臂"下去的时候不是垂直的、会歪一下"，指垫就蹭到物体把它推走
    （用户实测）。锁成同一姿态后上下段是纯平移。
    """
    qs, infos = {}, {}
    best = None
    for jd in (jaw_axes or (np.array([0.0, 1.0, 0.0]),
                            np.array([0.0, -1.0, 0.0]))):
        jd = np.asarray(jd, float)
        q, info = plan(chain, q0, limits, T_ab_col, tcp_local, jaw_local,
                       np.asarray(obj_c, float), jaw_dir=jd, seeds=seeds)
        dq = float(np.linalg.norm(q - q0))
        if best is None or dq < best[2]:
            best = (q, info, dq)
    qs["grasp"], infos["grasp"] = best[0], best[1]
    if (infos["grasp"]["pos_err_mm"] > max_err_mm
            or infos["grasp"]["dir_err_deg"] > max_ang_deg
            or infos["grasp"]["jaw_err_deg"] > max_ang_deg):
        return None, {"error": f"IK grasp 误差过大: {infos['grasp']}",
                      "infos": infos}

    down_g = np.asarray(infos["grasp"]["tcp_axis"], float)
    down_g = down_g / (np.linalg.norm(down_g) + 1e-9)
    jw = np.asarray(infos["grasp"]["jaw_world"], float)
    jaw_g = jw / (np.linalg.norm(jw) + 1e-9)
    for tag, dz in (("pre", approach), ("lift", lift)):
        q, info = plan(chain, q0, limits, T_ab_col, tcp_local, jaw_local,
                       np.asarray(obj_c, float) + np.array([0.0, 0.0, dz]),
                       down=down_g, jaw_dir=jaw_g, seeds=[qs["grasp"]],
                       fast=True)
        qs[tag], infos[tag] = q, info
        if (info["pos_err_mm"] > max_err_mm
                or info["dir_err_deg"] > max_ang_deg
                or info["jaw_err_deg"] > max_ang_deg):
            return None, {"error": f"IK {tag} 误差过大: {info}", "infos": infos}

    # --- 笛卡尔直线路径 ---------------------------------------------------
    # `send_joint_command` 是**关节空间**插值: 两个位形之间 TCP 走的是曲线,
    # 不是直线。pre→grasp 差 12cm 时 TCP 会横向甩出几厘米, 指垫斜着蹭到物体
    # 把它推走(用户实测"下去的时候不是垂直下去的, 会歪一下")。
    # 所以把直线打成密集路点, 每段只走一小步。
    def _seg(p_from, p_to, n, q_seed):
        out, seed = [], np.asarray(q_seed, float)
        for k in range(1, n + 1):
            tgt = p_from + (p_to - p_from) * (k / n)
            q, info = plan(chain, seed, limits, T_ab_col, tcp_local, jaw_local,
                           tgt, down=down_g, jaw_dir=jaw_g, seeds=[seed],
                           fast=True)
            out.append(q)
            seed = q
        return out

    obj = np.asarray(obj_c, float)
    qs["descent"] = _seg(obj + np.array([0.0, 0.0, approach]), obj,
                         max(2, int(descent_steps)), qs["pre"])
    qs["ascent"] = _seg(obj, obj + np.array([0.0, 0.0, lift]),
                        max(2, int(lift_steps)), qs["grasp"])

    return qs, {"infos": infos}

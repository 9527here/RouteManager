"""固定全身其余关节的单右臂避碰接近规划；纯几何计算，没有 Isaac/HTTP/驱动。

搜索时边以 <=0.04rad 取样作精确三维碰撞检查，任何找到的路线还必须通过
ValveSafety 的 2mm 表面 / 0.01rad 自适应全扫掠，粗搜索结果不能直接用于执行。
"""
import math
import time

import numpy as np


def plan_transit(safety, planner, start_full, goal_full, collision_kwargs, *,
                 max_nodes=2000, timeout_s=60., seed=17):
    """返回 {safe,configurations,metrics,final_sweep}；失败时 configurations 为空。"""
    started = time.monotonic()
    start, goal = np.asarray(start_full, float), np.asarray(goal_full, float)
    indices = np.array([safety.model.names.index(n) for n in planner.names], dtype=int)
    if planner.side != 'right':
        raise ValueError('本接近规划器仅支持明确的单右臂')
    if (start.shape != safety.q.shape or goal.shape != start.shape or
            not np.isfinite(start).all() or not np.isfinite(goal).all()):
        raise ValueError('接近路径全身关节向量无效')
    fixed = np.ones(len(start), bool)
    fixed[indices] = False
    if not np.allclose(start[fixed], goal[fixed], atol=1e-10, rtol=0):
        raise ValueError('接近规划禁止改变右臂之外的关节，包括夹爪/底盘/升降')
    if max_nodes < 2 or not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError('搜索节点和时间限制必须有效')
    q_start, q_goal = start[indices].copy(), goal[indices].copy()
    lo, hi = np.asarray(planner.lo), np.asarray(planner.hi)
    cache, stats = {}, {'configuration_queries': 0, 'cache_hits': 0, 'edge_queries': 0,
                        'rrt_nodes': 0, 'final_sweep_failures': 0}

    def full(q):
        value = start.copy()
        value[indices] = q
        return value

    def expired():
        return time.monotonic() - started >= timeout_s

    def valid(q):
        if np.any(q < lo) or np.any(q > hi):
            return False
        key = q.tobytes()
        if key in cache:
            stats['cache_hits'] += 1
            return cache[key]
        # refresh=False，不读取/推进/修改现场；查询保持捕获的全身与环境真值。
        report = safety.check_configuration(q=full(q), **collision_kwargs)
        result = bool(report['safe'])
        cache[key] = result
        stats['configuration_queries'] += 1
        return result

    def edge(a, b):
        stats['edge_queries'] += 1
        count = max(1, math.ceil(float(np.max(np.abs(b - a))) / .04))
        for u in np.linspace(0., 1., count + 1):
            if expired() or not valid(a + u * (b - a)):
                return False
        return True

    def finish(path, method):
        configurations = [full(q) for q in path]
        report = safety.check_path(configurations, surface_step_m=.002,
                                    joint_step_rad=.01, **collision_kwargs)
        if not report['safe']:
            stats['final_sweep_failures'] += 1
            return None
        return {'safe': True, 'configurations': [q.tolist() for q in configurations],
                'metrics': {**stats, 'elapsed_s': time.monotonic() - started,
                            'method': method, 'controlled_joint_names': list(planner.names),
                            'search_joint_step_rad': .04, 'final_joint_step_rad': .01,
                            'final_surface_step_m': .002, 'waypoint_count': len(configurations),
                            'other_joints_fixed': True, 'environment_fixed': True},
                'final_sweep': report}

    if not valid(q_start) or not valid(q_goal):
        return {'safe': False, 'configurations': [], 'metrics': stats,
                'reason': '接近起点/终点不满足完整碰撞阈值'}
    if edge(q_start, q_goal):
        result = finish([q_start, q_goal], 'direct_exactly_verified')
        if result:
            return result

    # 优先试小幅关节中点绕行；不假设哪个关节正号就是“抬手”。
    middle = (q_start + q_goal) * .5
    offsets = []
    for amplitude in (.10, .20, .35, .55, .80):
        for joint in (1, 2, 0, 3, 4):
            for sign in (1., -1.):
                offset = np.zeros(len(q_start))
                offset[joint] = amplitude * sign
                offsets.append(offset)
    for offset in offsets:
        if expired():
            break
        mid = np.clip(middle + offset, lo, hi)
        if valid(mid) and edge(q_start, mid) and edge(mid, q_goal):
            result = finish([q_start, mid, q_goal], 'deterministic_joint_bend')
            if result:
                return result

    rng = np.random.default_rng(seed)
    trees = [[{'q': q_start, 'parent': None}], [{'q': q_goal, 'parent': None}]]

    def extend(tree, target):
        nearest = int(np.argmin([np.linalg.norm(n['q'] - target) for n in tree]))
        source = tree[nearest]['q']
        delta = target - source
        length = float(np.max(np.abs(delta)))
        destination = source + delta * min(1., .20 / max(length, 1e-12))
        if length < 1e-10:
            return nearest, True
        if not edge(source, destination):
            return None, False
        tree.append({'q': destination, 'parent': nearest})
        stats['rrt_nodes'] += 1
        return len(tree) - 1, length <= .20

    def backtrack(tree, index):
        values = []
        while index is not None:
            values.append(tree[index]['q'])
            index = tree[index]['parent']
        return list(reversed(values))

    iteration = 0
    while not expired() and sum(len(t) for t in trees) < max_nodes:
        side = iteration % 2
        a, b = trees[side], trees[1 - side]
        draw = rng.random()
        if draw < .20:
            target = b[0]['q']
        elif draw < .80:
            # 确定种子的近路径采样优先，避免大行程腕关节主导无效全空间搜索。
            target = np.clip(q_start * (1 - (u := rng.random())) + q_goal * u +
                             rng.normal(0., .70, len(q_start)), lo, hi)
        else:
            target = rng.uniform(lo, hi)
        ia, _ = extend(a, target)
        iteration += 1
        if ia is None:
            continue
        while not expired() and sum(len(t) for t in trees) < max_nodes:
            ib, reached = extend(b, a[ia]['q'])
            if ib is None:
                break
            if reached:
                pa, pb = backtrack(a, ia), backtrack(b, ib)
                path = pa + list(reversed(pb))[1:] if side == 0 else pb + list(reversed(pa))[1:]
                # 先局部直连缩短，再按最终细扫掠验收，不能仅凭RRT边采样宣告成功。
                i = 0
                while i < len(path) - 2 and not expired():
                    shortened = False
                    for j in range(len(path) - 1, i + 1, -1):
                        if edge(path[i], path[j]):
                            path = path[:i+1] + path[j:]
                            shortened = True
                            break
                    i += 1
                result = finish(path, 'rrt_connect_final_exact_sweep')
                if result:
                    return result
                break
    return {'safe': False, 'configurations': [],
            'metrics': {**stats, 'elapsed_s': time.monotonic() - started,
                        'controlled_joint_names': list(planner.names),
                        'other_joints_fixed': True, 'environment_fixed': True},
            'reason': '限定节点/时间内未找到通过最终细扫掠的接近路线'}

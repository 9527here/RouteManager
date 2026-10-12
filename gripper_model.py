"""gripper_model.py — 夹爪(ctag2f120) 主动关节角 ↔ 开口 的实测标定表。

**为什么要标定**：主动关节角与指垫开口不是线性关系，也不是任何简单的解析式
（四连杆 + 软件 mimic）。而且"指垫中心距"随开口变化时，两指垫的**中点**会沿
夹爪接近轴漂移 ~11mm（见 `mid_drift_local`）——所以抓取规划时必须用**闭合态**
的 TCP，不能用张开态的。

表来源：仿真实测（60 N·m 驱动、往返扫描完全重合、无迟滞），2026-10-09。
     leader 0.00 → 中心距  2.68mm      leader 0.40 → 53.80mm
     leader 0.10 → 15.59mm             leader 0.50 → 65.95mm
     leader 0.20 → 28.49mm             leader 0.60 → 77.58mm
     leader 0.30 → 41.27mm             leader 0.70 → 88.49mm

**开口(opening)** = 两指垫内表面间距 = 中心距 − 2mm（每片指垫厚 2mm）。
睿尔曼 rm_driver 语义：`set_gripper_position_cmd` 的 position 1~1000 ↔ 开口 0~70mm。
本模块让仿真与真机在同一刻度上。
"""
import numpy as np

#: 主动关节角(rad) → 两指垫**中心**距(mm)，按 leader 升序。
#: 2026-10-09 烘焙左手关节限位后重测（此前左手无限位，机构会卡在坏平衡态）。
LEADER_TO_SEP_MM = (
    (0.000, 2.53), (0.100, 15.00), (0.200, 27.96), (0.300, 40.83),
    (0.400, 53.41), (0.500, 65.64), (0.600, 77.31), (0.700, 88.23),
)

#: 每片指垫厚度(mm) —— 开口 = 中心距 − 2×PAD_THICKNESS_MM/2
PAD_THICKNESS_MM = 2.0

#: 开口行程(mm)：position 1000 ↔ 该值。
#:
#: **这是"这一只夹爪"的行程，不是跨平台常量**：
#:   · 仿真里的 ctag2f120 实测满行程 **86.2mm**（leader 0.7 → 指垫中心距 88.23）
#:   · 真机 rm 夹爪是 **70mm**
#: 曾经硬写成 70 让仿真只开 70mm，60mm 的物体单边只剩 5mm 余量，
#: 指垫一偏就蹭到物体、把它推走（用户实测"感觉是张开的宽度不够"）。
#: `robot_manager` 会用 config.json 的 `gripper.stroke_mm` 覆盖。
#: **可移植的是 position/1000 这个比例，不是毫米数**。
STROKE_MM = 86.2

#: 主动关节全开角(rad)，URDF 上限
LEADER_OPEN_RAD = 0.700

#: 指垫中点沿夹爪接近轴(local +z)从全开→全闭的漂移(m)。
#: 张开态 TCP = 闭合态 TCP + 该值×(+z)。规划用闭合态 TCP 时不需要它，
#: 只有"用张开态 TCP 近似"时才需要补偿。
MID_DRIFT_LOCAL_Z = 0.0111


def leader_for_sep_mm(sep_mm: float) -> float:
    """指垫中心距(mm) → 主动关节角(rad)。"""
    xs = [s for _, s in LEADER_TO_SEP_MM]
    ys = [q for q, _ in LEADER_TO_SEP_MM]
    return float(np.interp(sep_mm, xs, ys))


def sep_mm_for_leader(leader: float) -> float:
    """主动关节角(rad) → 指垫中心距(mm)。"""
    qs = [q for q, _ in LEADER_TO_SEP_MM]
    ss = [s for _, s in LEADER_TO_SEP_MM]
    return float(np.interp(leader, qs, ss))


def leader_for_opening_mm(opening_mm: float) -> float:
    """开口(mm, 指垫内间距) → 主动关节角(rad)。"""
    return leader_for_sep_mm(opening_mm + PAD_THICKNESS_MM)


def opening_mm_for_leader(leader: float) -> float:
    """主动关节角(rad) → 开口(mm, 指垫内间距)。"""
    return sep_mm_for_leader(leader) - PAD_THICKNESS_MM


def leader_for_fraction(frac: float) -> float:
    """开口比例 0~1（睿尔曼刻度 0~70mm）→ 主动关节角(rad)。"""
    frac = min(max(float(frac), 0.0), 1.0)
    return leader_for_opening_mm(frac * STROKE_MM)


def fraction_for_leader(leader: float) -> float:
    """主动关节角(rad) → 开口比例 0~1（睿尔曼刻度）。"""
    return min(max(opening_mm_for_leader(leader) / STROKE_MM, 0.0), 1.0)


def fraction_for_sep_mm(sep_mm: float) -> float:
    """指垫中心距(mm) → 开口比例 0~1（睿尔曼刻度 0~70mm 开口）。

    注意：这是**开口比例**，不是主动关节角比例 —— `set_gripper_position`
    的 `fraction` 参数按开口解释。两者在中间段能差 30% 以上。
    """
    return min(max((sep_mm - PAD_THICKNESS_MM) / STROKE_MM, 0.0), 1.0)

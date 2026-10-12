"""离线把左手夹爪关节的**限位**烘焙进机器人 physics.usd。

**为什么必须离线**: 运行时给关节 prim 写属性会触发物理后端失效(关节读写全废),
所以只能在资产里改、重启生效。

**问题**: URDF 里两侧 6 个夹爪关节都有 limit，但导入后的资产里
**左手 6 个全是 ±inf**（右手正常）。限位缺失 → 四连杆可以被驱到物理行程之外，
实测夹爪会卡在"主动关节到位但指垫只分开 34mm"的坏平衡态，且很难恢复。

**取值**: 左指是右指的镜像，直接照搬右指的烘焙值：
  · left_2 / left_in / left_slip 与右指同号同值
  · left_1 同值
  · left_3 / left_main 取右指值的**取反并交换上下限**（URDF 里倍率是 −1）
"""
import json
import os
import shutil

from pxr import Usd

P = ("/home/jy/datasets/vla_data/L1_制作资产/载体设备/humanbody_urdf/"
     "humanoid_upper_body_v3/usd/humanoid_upper_body_v3_ctag2f120_cameras_isaac/"
     "configuration/humanoid_upper_body_v3_ctag2f120_cameras_isaac_physics.usd")
BAK = P + ".pre_griplimit_bak"

#: 左指关节 ← 右指关节（None 表示取反并交换上下限）
MIRROR = {
    "left_gripper_left_2_joint": ("left_gripper_right_2_joint", False),
    "left_gripper_left_in_joint": ("left_gripper_right_in_joint", False),
    "left_gripper_left_slip_joint": ("left_gripper_right_slip_joint", False),
    "left_gripper_left_1_joint": ("left_gripper_right_1_joint", False),
    "left_gripper_left_3_joint": ("left_gripper_right_3_joint", True),
    "left_gripper_left_main_joint": ("left_gripper_right_main_joint", True),
}
#: 只烘焙左手；右手当模板
TARGET_SIDES = ("left",)

if not os.path.exists(BAK):
    shutil.copy2(P, BAK)
    print("backup ->", BAK)

stage = Usd.Stage.Open(P)
by_name = {}
for p in stage.Traverse():
    if p.GetName() in MIRROR or p.GetName().endswith("_joint"):
        by_name.setdefault(p.GetName(), p)

changed = []
for lname, (rname, negate) in MIRROR.items():
    lp, rp = by_name.get(lname), by_name.get(rname)
    if lp is None or rp is None:
        print(f"  ! 找不到 {lname} 或 {rname}")
        continue
    lo = rp.GetAttribute("physics:lowerLimit").Get()
    hi = rp.GetAttribute("physics:upperLimit").Get()
    if lo is None or hi is None:
        print(f"  ! {rname} 没有限位，跳过")
        continue
    if negate:
        lo, hi = -hi, -lo
    la = lp.GetAttribute("physics:lowerLimit")
    if not la or not la.IsValid():
        la = lp.CreateAttribute("physics:lowerLimit", la.GetTypeName()
                                if la else None)
    lb = lp.GetAttribute("physics:upperLimit")
    if not lb or not lb.IsValid():
        lb = lp.CreateAttribute("physics:upperLimit", lb.GetTypeName()
                                if lb else None)
    la.Set(lo)
    lb.Set(hi)
    changed.append((lname, lo, hi))

stage.Save()
print(f"烘焙了 {len(changed)} 个左手夹爪关节限位:")
for n, lo, hi in changed:
    print(f"  {n:36s} [{lo:.4f}, {hi:.4f}]")

# run_in_sim 靠这个文件回传结果
with open("/tmp/bake_gripper_limits.json", "w") as f:
    json.dump({"backup": BAK, "n": len(changed),
               "changed": [{"joint": n, "lower": lo, "upper": hi}
                           for n, lo, hi in changed]}, f, indent=1)

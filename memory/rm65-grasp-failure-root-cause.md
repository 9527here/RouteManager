# RM65 抓不住 / "抓住但抓不起来" 的根因链（2026-10-13/14 结案）

用户问题："抓住但是抓不起来"。**三个独立缺陷叠在一起**，逐个修掉后抓取
从 **0/N → 8/10**。按修复顺序记，后面每一条都以前一条修好为前提。

---

## 缺陷 1：夹爪连杆碰撞被剥光（已修）

`install_standalone_arm.py` 第 3 步原本把 15 个 `{side}_gripper_*` link 里
13 个的 `<collision>` 删掉，只留两片指垫。结果 30 个夹爪连杆里只有 4 个有碰撞。

**改成 `restore_linkage_collision()` 照抄各自的 `<visual>` 补回来**（26 个）。
夹爪自由行程不受影响（全闭仍到 2.05→2.11mm），抓取从 0/N 变 2/3。
`strip_linkage_collision()` 保留作回滚。

## 缺陷 2：下降段在手臂还没到位时就开跑（已修）

`robot_manager.py` 的 `pre` 段原来只等 `self._motion is None`（**插值发完了**）
就进 `descent`。但那时伺服还在追 —— 实测 `pre` 插值刚结束那一帧，
**实测关节角与目标最多差 0.277 rad (16°)**。

后果：竖直下降的同时手臂横向甩过去，指垫扫到方块把它推走。实测
**方块被推 160mm**，最后夹爪在离方块 **104mm 的空气**里合拢（`close_sep` 看着
还挺正常，其实夹的是空气）。

修法（与 `preclose` 同一套判据）：
```python
if el > 0.3 and self._motion is None:
    if el > 3.5 or self._arm_err(ap, np.asarray(ap["qs"]["pre"], float)) < 0.02:
        nxt("descent")
```
修完方块在整个下降段只动 **0.5mm**，TCP 全程在方块中心 1.4~4mm 内。

**测量教训**：判"手臂到位"必须用**实测关节角/速度**，不能用
`_motion is None`（那只说明指令发完了）或插值参数 u。

## 缺陷 3：夹爪驱动太软，挤不出法向力（已修）★ 真正的"抓不起来"

`*.gains.json` 里夹爪关节 `stiffness: 5000 / damping: 50`，导入器写的是
`drive:angular:physics:type = acceleration` —— **力矩 = 指节惯量 × k × 角度差**。
指节等效惯量只有 ~5e-5 kg·m²，所以 k=5000 折出的**等效刚度只有 ~14 N·m/rad**。

实测证据（合爪后抬升段，主动关节 `left_gripper_right_2_joint`）：

| 量 | 值 |
|---|---|
| 实测角 vs 指令角之差 | 0.014 rad |
| **实测关节力矩** | **±0.02 N·m**（`maxForce` 是 60） |
| 指垫法向力（反推） | ~4 N |
| 结果 | 抬到 54mm 后方块从虎口滑回桌面 |

注意：**力矩小 ≠ maxForce 不够**，是位置差太小乘上软刚度就这么多。
`_tune_gripper_drives` 把 maxForce 提到 60 是对的但不够用。

修法：`GRIPPER_STIFFNESS 5000 → 100000`、`GRIPPER_DAMPING 50 → 1000`
（`install_standalone_arm.py` 的常量，重跑即幂等生效）。
等效刚度 → ~280 N·m/rad，同样位置差给出 ~4 N·m。**抬升 188mm 不滑**。

同一件事的另一面：k 也要大到能压住重力引起的从动关节残差（k=100 时残差
0.0755 rad → 左右指连杆互插），所以这个值越大越好，100000 下残差 <0.0002 rad。

---

## 修好之后合爪瞬间的实测几何（理想）

```
left_gripper_right_pad_link   -0.15mm   ← 负 = 压进方块
left_gripper_left_pad_link    -0.06mm
left_gripper_*_3_link          1.3mm    ← 其余连杆都不碰
_off_normal_deg                0.05°    ← 虎口正对方块面法向
_eff_width_mm                 60.06     ← 方块真实 60mm
```

---

## 被证伪的假设（别再走）

1. **"旧臂虎口偏 3.4° 咬棱、RM65 对正所以夹不住"** —— 旧结论，**错**。
   修好缺陷 1+2 之后 RM65 虎口对正 0.05°、指垫压进方块 0.1mm，照样能抓。
   当时观察到的"对正就零力接触"其实是缺陷 3 的表象。
2. **"夹爪刚度 286,478 是旧臂的 50 倍所以压不进去"** —— 单向实验不成立。
3. **"方块朝向 = spawn.yaw"** 的判断一度出错：`Gf.Matrix4d.ExtractRotationQuat()`
   对这种矩阵返回**非单位四元数**（0.535²+0.0397²≠1），算出的 yaw 完全不对。
   要取朝向请用行向量约定直接读 `atan2(R[0][1], R[0][0])`。

## 排除清单（全部实测）

FK/IK 链（自检 0.00mm）、规划质量（下降 TCP 竖直落在方块中心 0.0mm）、
底盘漂移（下降段 7.7mm）、关节空间直线性、伺服到位精度、夹爪自由行程
（2.05~88.4mm）、指垫 mesh/URDF 定义、接触摩擦材质（`/World/friction_mat`
static/dynamic 都 1.0，确实绑在 cube_c 上）、方块朝向对齐（0.05°）。

相关 [[auto-pick-descent-jitter]]、[[gripper-mimic-chain]]、[[scene-friction-zero]]。

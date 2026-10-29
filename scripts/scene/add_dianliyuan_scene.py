"""把福州电力院的 NuRec 场景(usdz)作为环境挂进静态场景, 替换掉泉州3。幂等重跑。

- 引用方式: 静态场景里加一个 Xform 引用 usdz 的 /World（静态场景在机器人
  之前加载, 避免"运行时加 prim → articulation 物理后端失效"）
- 泉州3 的块**保留成注释**, 想切回去把注释符号去掉、再把本块注掉即可。
- 平移: 使地面落到 z=0、内容水平居中于机器人(x=0,y=0)
- 桌子本来就在静态场景里, 机器人装载在原点 → 二者自动落在厂房内

**包围盒怎么来的**（电力院没有实测值, 是从 nurec 体积反推的）:
  usdz 里的 `gauss.usda` 有 `float3[] extent`。泉州3 的 extent 尺寸
  884.76 × 1442.74 对应实测米制 10.3 × 16.8 m → **比例 85.90**（x/y 两个轴
  独立算出来一致, 可信）。电力院 extent 除以同一比例即得米制范围。
  z 跨度 [-6.513, 7.076]（13.59 m 高）。

**`--ground-scan-z` 默认 0（场景保持在原地）**:
  2026-10-19 用户用 Isaac 里的 `/Xform` 标出扫描的**可站地面高度 = 6.222 m**。
  但**不要**用本脚本把场景降下去 —— 实测降下去之后相机在 z≈0.9 就整个埋在
  扫描数据里（那一层是地面/地下的噪声），机器人也看不见；可用的房间在世界
  z≈5~6。所以改成**把机器人+桌子抬上去**，走 config.json 的 `world.base_z`
  （= 6.222），本脚本只管水平居中 + 场景原位。

  踩过的坑：先按"场景下移"做过（T_z=0.291），结果机器人埋在数据里；
  方向反过来才对。
"""
import argparse
import os

HERE = os.path.dirname(os.path.abspath(__file__))
USDA = os.path.join(HERE, "static_scene.usda")
USDZ = ("/home/jy/digital_asset/福州电力院usdz/lcc-usdz-result/model.usdz")

NAME = "dianliyuan_scene"
BEGIN = "    # ---- DLYUAN SCENE BEGIN (add_dianliyuan_scene.py) ----"
END = "    # ---- DLYUAN SCENE END ----"

# 泉州3 的块（切换用；本脚本会把它整段注释掉）
QZ_BEGIN = "    # ---- QZ3 SCENE BEGIN (add_qz3_scene.py) ----"
QZ_END = "    # ---- QZ3 SCENE END ----"

#: extent / 85.90 折算出来的内容范围(米)。z 未实测, 见模块 docstring。
DEFAULT_MN = (-9.653, -12.639, -6.513)
DEFAULT_MX = (13.521, 13.496, 7.076)


def build_block(mn, mx, ground_scan_z=0.0):
    """生成环境块。`ground_scan_z` = 扫描系里"可视地面"的 z（米）。"""
    t = (round(-(mn[0] + mx[0]) / 2.0, 3),          # x 居中
         round(-(mn[1] + mx[1]) / 2.0, 3),          # y 居中
         round(-mn[2] - ground_scan_z, 3))          # 地面 → z=0
    return t, f"""{BEGIN}
    def Xform "{NAME}" (
        prepend references = @{USDZ}@</World>
    )
    {{
        double3 xformOp:translate = {t}
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }}
{END}
"""


def comment_out(text: str) -> str:
    """把一段 usda 注释掉（保留原文，便于切回）。"""
    return "\n".join(("# " + ln) if ln.strip() else ln for ln in text.splitlines())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bbox", default=None,
                    help="minx,miny,minz,maxx,maxy,maxz（米）；不给用默认折算值")
    ap.add_argument("--ground-scan-z", type=float, default=0.0,
                    help="扫描系里可视地面的 z（米），用于把地面落到 z=0。"
                         "默认 0 = 场景保持原位，机器人那边用 config.json 的 "
                         "world.base_z=6.222 抬上去（别用本参数降场景，见 docstring）")
    ap.add_argument("--restore-qz3", action="store_true",
                    help="把泉州3 换回来（取消注释它的块、注掉电力院块）")
    args = ap.parse_args()

    src = open(USDA, encoding="utf-8").read()
    if args.bbox:
        v = [float(x) for x in args.bbox.split(",")]
        assert len(v) == 6, "bbox 要 6 个数"
        mn, mx = tuple(v[:3]), tuple(v[3:])
    else:
        mn, mx = DEFAULT_MN, DEFAULT_MX

    t, block = build_block(mn, mx, args.ground_scan_z)

    if args.restore_qz3:
        if BEGIN in src and END in src:
            src = src[:src.index(BEGIN)] + src[src.index(END) + len(END):]
        if QZ_BEGIN in src and QZ_END in src:
            qz = src[src.index(QZ_BEGIN):src.index(QZ_END) + len(QZ_END)]
            un = "\n".join(ln[2:] if ln.startswith("# ") else ln
                           for ln in qz.splitlines())
            src = src.replace(qz, un)
        open(USDA, "w", encoding="utf-8").write(src)
        print("已切回泉州3（电力院块已移除）")
        print("括号平衡:", src.count("{") - src.count("}"))
        return

    # 1) 移除旧的电力院块
    if BEGIN in src and END in src:
        src = src[:src.index(BEGIN)] + src[src.index(END) + len(END):]

    # 2) 注释掉泉州3 块（没注释过的话）
    #    注意: marker 本身就是 `#` 开头的注释行, 不能用 lstrip().startswith("#")
    #    判断 —— 那永远为真。要看**块体**里有没有没被注释的 `def Xform`。
    if QZ_BEGIN in src and QZ_END in src:
        qz = src[src.index(QZ_BEGIN):src.index(QZ_END) + len(QZ_END)]
        body_active = any(ln.strip().startswith("def Xform")
                          for ln in qz.splitlines())
        if body_active:
            src = src.replace(qz, comment_out(qz))
            print("泉州3 块已注释（--restore-qz3 可切回）")

    # 3) 插入电力院块
    idx = src.rstrip().rfind("}")
    src = src[:idx] + block + src[idx:]
    open(USDA, "w", encoding="utf-8").write(src)

    print(f"引用: {USDZ}")
    print(f"内容范围(米): x[{mn[0]:.2f},{mx[0]:.2f}] y[{mn[1]:.2f},{mx[1]:.2f}] "
          f"z[{mn[2]:.2f},{mx[2]:.2f}]")
    print(f"平移: {t}   (水平居中 + 地面→z=0, ground_scan_z={args.ground_scan_z})")
    print("括号平衡:", src.count("{") - src.count("}"))


if __name__ == "__main__":
    main()

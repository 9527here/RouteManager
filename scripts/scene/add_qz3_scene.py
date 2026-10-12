"""把泉州3 的 NuRec 场景(usdz)作为环境挂进静态场景, 幂等重跑。

- 引用方式: 静态场景里加一个 Xform 引用 usdz 的 /World（静态场景在机器人
  之前加载, 避免"运行时加 prim → articulation 物理后端失效"）
- 平移: 扫描的实测内容范围(米): x∈[-6.2,4.1] y∈[-3.0,13.8] z∈[-2.5,4.2]
  → 平移使地面落到 z=0、内容水平居中于机器人(x=0,y=0)
- 桌子本来就在静态场景里, 机器人装载在原点 → 二者自动落在厂房内
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
USDA = os.path.join(HERE, "static_scene.usda")
USDZ = ("/home/jy/datasets/vla_data/L2_仿真运行/场景/泉州3/"
        "lcc-usdz-result/model.usdz")

BEGIN = "    # ---- QZ3 SCENE BEGIN (add_qz3_scene.py) ----"
END = "    # ---- QZ3 SCENE END ----"

# 实测内容包围盒(米)
MN = (-6.2, -3.0, -2.5)
MX = (4.1, 13.8, 4.2)
T = (round(-(MN[0] + MX[0]) / 2.0, 3),      # x 居中
     round(-(MN[1] + MX[1]) / 2.0, 3),      # y 居中
     round(-MN[2] - 2.4424, 3))             # 地面 → z=0 (按用户实测: 可视地面在扫描系 z≈-0.0576)

BLOCK = f"""{BEGIN}
    def Xform "qz3_scene" (
        prepend references = @{USDZ}@</World>
    )
    {{
        double3 xformOp:translate = {T}
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }}
{END}
"""


def main():
    src = open(USDA, encoding="utf-8").read()
    if BEGIN in src and END in src:
        head = src[:src.index(BEGIN)]
        tail = src[src.index(END) + len(END):].lstrip("\n")
        src = head + BLOCK + tail
        print("已更新 qz3 场景块")
    else:
        idx = src.rstrip().rfind("}")
        src = src[:idx] + BLOCK + src[idx:]
        print("已插入 qz3 场景块")
    open(USDA, "w", encoding="utf-8").write(src)
    print(f"引用: {USDZ}")
    print(f"平移: {T}  (扫描地面 → z=0, 内容水平居中)")
    print("括号平衡:", src.count("{") - src.count("}"))


if __name__ == "__main__":
    main()

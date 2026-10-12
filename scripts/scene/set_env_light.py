"""把静态场景的"背景"改成 DomeLight 环境贴图(不挡光), 幂等。

- 删除旧的 background 立方体块(它会挡住 DomeLight/太阳 → 场景发暗)
- 插入 DomeLight(带等距柱状全景贴图) + DistantLight(太阳)
  (运行时 robot_manager._ensure_default_environment 见已有灯就跳过, 不会重复建)
用法: python3 scripts/scene/set_env_light.py [全景图路径]
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
USDA = os.path.join(HERE, "static_scene.usda")
PANO = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    HERE, "background_faces", "env_equirect.png")

CUBE_BEGIN = "    # ---- BACKGROUND CUBE BEGIN (gen_background_cube.py) ----"
CUBE_END = "    # ---- BACKGROUND CUBE END ----"
L_BEGIN = "    # ---- ENV LIGHT BEGIN (set_env_light.py) ----"
L_END = "    # ---- ENV LIGHT END ----"

LIGHTS = f"""{L_BEGIN}
    def DomeLight "env_dome"
    {{
        float inputs:intensity = 1000
        token inputs:texture:format = "latlong"
        asset inputs:texture:file = @{PANO}@
        double3 xformOp:rotateXYZ = (0, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:rotateXYZ"]
    }}
    def DistantLight "env_sun"
    {{
        float inputs:intensity = 500
        float inputs:angle = 0.53
        double3 xformOp:rotateXYZ = (0, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:rotateXYZ"]
    }}
{L_END}
"""


def main():
    src = open(USDA, encoding="utf-8").read()
    # 1) 去掉立方体块
    if CUBE_BEGIN in src and CUBE_END in src:
        head = src[:src.index(CUBE_BEGIN)]
        tail = src[src.index(CUBE_END) + len(CUBE_END):].lstrip("\n")
        src = head + tail
        print("已移除 background 立方体块")
    # 2) 替换/插入灯光块
    if L_BEGIN in src and L_END in src:
        head = src[:src.index(L_BEGIN)]
        tail = src[src.index(L_END) + len(L_END):].lstrip("\n")
        src = head + LIGHTS + tail
        print("已更新环境灯光块")
    else:
        idx = src.rstrip().rfind("}")
        src = src[:idx] + LIGHTS + src[idx:]
        print("已插入环境灯光块")
    open(USDA, "w", encoding="utf-8").write(src)
    print(f"DomeLight 贴图: {PANO}")
    print("括号平衡:", src.count("{") - src.count("}"))


if __name__ == "__main__":
    main()

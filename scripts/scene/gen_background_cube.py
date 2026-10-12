"""把 3x2 六格全景图切成六个面, 生成"内壁贴图背景立方体"写进 static_scene.usda。

必须烘焙进静态场景(机器人加载前): 运行时新建 prim 会让 articulation 物理后端
失效(本项目踩过)。反复运行是幂等的: 只替换 BEGIN/END 标记之间的内容。
"""
import os

SRC = ("/tmp/claude-1000/-home-jy-RouteManager/8138b6c7-db40-4548-8971-"
       "66d93425fd0f/images/11.png")
HERE = os.path.dirname(os.path.abspath(__file__))
FACES_DIR = os.path.join(HERE, "background_faces")
USDA = os.path.join(HERE, "static_scene.usda")

HALF = 8.0          # 立方体半边长(m)
CENTER = (0.0, 0.0, 2.5)
# 3x2 标准 cubemap 顺序: 读序 [px, nx, py, ny, pz, nz]
ORDER = ["px", "nx", "py", "ny", "pz", "nz"]

BEGIN = "    # ---- BACKGROUND CUBE BEGIN (gen_background_cube.py) ----"
END = "    # ---- BACKGROUND CUBE END ----"


def split_faces():
    from PIL import Image
    os.makedirs(FACES_DIR, exist_ok=True)
    im = Image.open(SRC).convert("RGB")
    W, H = im.size
    fw, fh = W / 3.0, H / 2.0
    paths = []
    for i, tag in enumerate(ORDER):
        c, r = i % 3, i // 3
        box = (int(round(c * fw)), int(round(r * fh)),
               int(round((c + 1) * fw)), int(round((r + 1) * fh)))
        p = os.path.join(FACES_DIR, f"{tag}.png")
        im.crop(box).save(p)
        paths.append(p)
    return paths


def face_quad(tag, s, c):
    """返回 (四点, 法线)。UV 固定 (0,0)(1,0)(1,1)(0,1), 顺序按各面朝向排。"""
    x, y, z = c[0], c[1], c[2]
    if tag == "px":      # 右壁, 面朝 -X
        pts = [(x + s, y + s, z - s), (x + s, y - s, z - s),
               (x + s, y - s, z + s), (x + s, y + s, z + s)]
        n = (-1, 0, 0)
    elif tag == "nx":    # 左壁
        pts = [(x - s, y - s, z - s), (x - s, y + s, z - s),
               (x - s, y + s, z + s), (x - s, y - s, z + s)]
        n = (1, 0, 0)
    elif tag == "py":    # 天花板, 面朝下
        pts = [(x + s, y + s, z + s), (x - s, y + s, z + s),
               (x - s, y - s, z + s), (x + s, y - s, z + s)]
        n = (0, 0, -1)
    elif tag == "ny":    # 地面
        pts = [(x + s, y - s, z - s), (x - s, y - s, z - s),
               (x - s, y + s, z - s), (x + s, y + s, z - s)]
        n = (0, 0, 1)
    elif tag == "pz":    # 后壁(+Y 侧)
        pts = [(x - s, y + s, z - s), (x + s, y + s, z - s),
               (x + s, y + s, z + s), (x - s, y + s, z + s)]
        n = (0, -1, 0)
    else:                # nz: 前壁(-Y 侧)
        pts = [(x + s, y - s, z - s), (x - s, y - s, z - s),
               (x - s, y - s, z + s), (x + s, y - s, z + s)]
        n = (0, 1, 0)
    return pts, n


def build_block():
    paths = split_faces()
    L = [BEGIN]
    L.append('    def Xform "background"')
    L.append("    {")
    for tag, p in zip(ORDER, paths):
        L.append(f'        def Mesh "{tag}"')
        L.append("        {")
        L.append("            uniform bool doubleSided = true")
        L.append("            int[] faceVertexCounts = [4]")
        L.append("            int[] faceVertexIndices = [0, 1, 2, 3]")
        pts, n = face_quad(tag, HALF, CENTER)
        L.append("            point3f[] points = ["
                 + ", ".join(f"({a:.3f}, {b:.3f}, {c:.3f})" for a, b, c in pts)
                 + "]")
        L.append("            normal3f[] normals = ["
                 + ", ".join(f"({n[0]}, {n[1]}, {n[2]})" for _ in range(4))
                 + "] (interpolation = \"vertex\")")
        L.append("            texCoord2f[] primvars:st = [(0, 0), (1, 0), "
                 "(1, 1), (0, 1)] (interpolation = \"vertex\")")
        L.append(f"            rel material:binding = </World/background/mat_{tag}>")
        L.append("        }")
    for tag, p in zip(ORDER, paths):
        L.append(f'        def Material "mat_{tag}"')
        L.append("        {")
        L.append(f"            token outputs:surface.connect = "
                 f"</World/background/mat_{tag}/Shader.outputs:surface>")
        L.append('            def Shader "Shader"')
        L.append("            {")
        L.append('                uniform token info:id = "UsdPreviewSurface"')
        L.append("                color3f inputs:diffuseColor = (1, 1, 1)")
        L.append("                float inputs:roughness = 1")
        L.append(f"                color3f inputs:emissiveColor.connect = "
                 f"</World/background/mat_{tag}/Tex.outputs:rgb>")
        L.append("                token outputs:surface")
        L.append("            }")
        L.append('            def Shader "Tex"')
        L.append("            {")
        L.append('                uniform token info:id = "UsdUVTexture"')
        L.append(f"                asset inputs:file = @{p}@")
        L.append('                token inputs:wrapS = "clamp"')
        L.append('                token inputs:wrapT = "clamp"')
        L.append("                float2 inputs:st.connect = "
                 f"</World/background/mat_{tag}/uv.outputs:result>")
        L.append("                float3 outputs:rgb")
        L.append("            }")
        L.append('            def Shader "uv"')
        L.append("            {")
        L.append('                uniform token info:id = "UsdPrimvarReader_float2"')
        L.append('                token inputs:varname = "st"')
        L.append("                float2 outputs:result")
        L.append("            }")
        L.append("        }")
    L.append("    }")
    L.append(END)
    return "\n".join(L) + "\n"


def main():
    block = build_block()
    src = open(USDA, encoding="utf-8").read()
    if BEGIN in src and END in src:
        head = src[:src.index(BEGIN)]
        tail = src[src.index(END) + len(END):]
        # 去掉 END 后紧跟的换行, 保持格式
        tail = tail.lstrip("\n")
        out = head + block + tail
    else:
        # 插到 World 的最后一个右花括号之前
        idx = src.rstrip().rfind("}")
        out = src[:idx] + block + src[idx:]
    open(USDA, "w", encoding="utf-8").write(out)
    print(f"OK: 背景立方体已写入 {USDA} (半边长 {HALF}m, 面贴图 {FACES_DIR})")


if __name__ == "__main__":
    main()

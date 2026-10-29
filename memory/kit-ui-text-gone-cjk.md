# Isaac GUI 中文字形：正确的配置姿势（2026-10-27）

## 症状（曾误判为「图集太大」）

Kit 窗口面板、边距、图标、3D 视口全对，**唯独一个字都没有**，日志零报错。
图标不受影响是因为它们走 `libcarb.svg-nanosvg.plugin.so` 三角化成矢量，不碰字体图集。

## 真正根因

`isaacsim.exp.base.kit` 里通过 `omni.ui.glyphmanager` 挂的自定义字体
`DroidSansFallbackFull.ttf` **不含任何拉丁字形**（`cmap` 里查 `A` = False）。
`/app/font/customFontPath` 是**整体替换**而非「只补缺字」：设了它之后所有字符都去
这个字体里查，英文字母查不到 → 整个界面文字全空白。中文其实一直是好的，只是当时
屏幕上本来就没有中文，所以看起来像「全没了」。

**图集尺寸不是原因**：挂了自定义字体后图集宽度恒为 16384（164 字时 16384×222、
7106 字时 16384×2085），这是正常布局，不会因此失败。

## 可用的配置（已验证）

```ini
# isaacsim/apps/isaacsim.exp.base.kit  →  [settings.app]
font.customFontPath = "/home/jy/.local/share/kit-cjk/NotoSansSC-Medium.ttf"
font.customRegionFiles = ["/home/jy/.local/share/kit-cjk/chinese.txt"]
font.overresolutionSize = 66
```

三条硬要求：

1. **必须是 TrueType 轮廓**——`stb_truetype` 不吃 CFF/OTF。系统的
   `NotoSansCJK-*.ttc` 全是 CFF，**用不了**；arphic 的 `uming/ukai` 是 TrueType
   但只有衬线体，会把整个界面变成明朝体且宽度偏大、文本被截断。
2. **必须同时含拉丁字形**（见上面的根因）。
3. **region 文件每行 192 字**——厂商自带的 `data/regions/japanese.txt` 就是这个格式；
   一整个 2 万字的单行文件踩 imgui CHANGELOG 里点名的「long lines」问题。
   `overresolutionSize=66` 是厂商日文测试里用来压小 ULTRA 字号、让图集塞进单张
   纹理的同款参数。

字体来源：Google Fonts 的 `notosanssc/NotoSansSC[wght].ttf` 是**可变字体、默认
wght=100（极细）**，必须先用 fontTools 实例化到 500：

```python
from fontTools.ttLib import TTFont
from fontTools.varLib import instancer
instancer.instantiateVariableFont(
    TTFont(src), {"wght": 500}, inplace=True, updateFontNames=True).save(dst)
```

## 验证方法

- `--/app/font/saveAtlas=1` 让 Kit 把字体图集导出成 `font_atlas.png`（写进 CWD），
  用它比尺寸/看字形是否烘焙进去。注意 16384 宽是正常的，别当故障信号。
- 端到端验收：`load_scene(usd_path=..., prim_path="/中文测试场景")` 加载一个带
  中文 prim 名的 USDA，Stage 面板会直接显示中文，截图即可确认中英混排正常。
  （prim 名不能带空格，USD 会报 not a valid prim name。）

## 排查手法（可复用）

- `DISPLAY=:1 xwininfo -root -tree | grep "Isaac Sim"` 拿窗口 id，
  `DISPLAY=:1 import -window <id> out.png` 截 GUI；被别的窗口盖住不影响抓取。
- 怀疑「某处配置导致」时先找 `*.bak*`：这次就是
  `isaacsim.exp.base.kit.bak-before-cjk` 这个文件名直接指出的。
- 做字体/系统级 A/B 时优先用「厂商自带的参考配置」当对照组：
  换上 `NotoSansJP-SemiBold.ttf + data/regions/japanese.txt` 能正常渲染，
  一步就把「机制坏了」和「字体文件坏了」分开了。

相关 [[ros2-inprocess-contract]]、[[isaac62-migration]]。

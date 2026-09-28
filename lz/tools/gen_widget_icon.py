#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
生成 Pilot 2 控件图标（96x96 RGBA PNG）。

## 为什么是"纯标准库手写 PNG"

本机**没有 PIL / ImageMagick**（`python3 -c "import PIL"` → ModuleNotFoundError，
首次生成控件图标时实测）。为一个 96x96 的图标给工程引入图像库依赖，
与 lz_core "桌面上 clone 下来就能编"的性质相冲突。

而 PNG 的最小可用子集其实很小：IHDR + IDAT(zlib) + IEND，每行前面加一个
filter type 字节（这里恒用 0 = None）。`zlib` 与 `struct` 都是标准库。
见 tools/gen_vision_testdata.js 的注释 —— 那里也是同一个取舍
（纯 JS + jpeg-js，**故意不依赖 PIL/OpenCV**）。

## 抗锯齿

按 4 倍分辨率绘制、再盒式降采样。不这么做的话 45° 的斜线与圆弧全是锯齿，
而**户外强光下图标本来就小、对比度又低**，锯齿会直接吃掉可辨识度。

## 已生成的图标

| 文件名 | 控件 | 形状 |
|---|---|---|
| icon_visual_align.png | 6「识别目标」 | 取景框四角 + 中心十字（= 对焦/瞄准） |

颜色沿用「记录」两个按钮的橙色 `(255,152,0)` —— 同一族"按下即执行一个
瞬时动作"的按钮用同一个颜色，操作员在强光下靠颜色分区找按钮。

用法：
    python3 tools/gen_widget_icon.py <输出目录>          # 生成全部
    python3 tools/gen_widget_icon.py --list              # 看有哪些
"""

import os
import struct
import sys
import zlib

SIZE = 96          # 成品边长（与其它控件图标一致）
SS = 4             # 超采样倍数
N = SIZE * SS

# 与 icon_record_*.png 实测取到的同一个橙色（见 CLAUDE.md 的图标一节）
COLOR = (255, 152, 0)


def rgba_to_png(path, rgba):
    """rgba: bytes, 长度 = w*h*4，行优先。写单 IDAT 的 PNG。"""
    w = h = SIZE
    raw = bytearray()
    for y in range(h):
        raw.append(0)                       # filter type 0 = None
        raw += rgba[y * w * 4:(y + 1) * w * 4]
    comp = zlib.compress(bytes(raw), 9)

    def chunk(typ, data):
        return (struct.pack('>I', len(data)) + typ + data
                + struct.pack('>I', zlib.crc32(typ + data) & 0xFFFFFFFF))

    png = b'\x89PNG\r\n\x1a\n'
    png += chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 6, 0, 0, 0))
    png += chunk(b'IDAT', comp)
    png += chunk(b'IEND', b'')
    with open(path, 'wb') as f:
        f.write(png)


def new_canvas():
    return bytearray(N * N * 4)             # 全透明


def put(canvas, x, y, a=255):
    """把一个超采样像素置为 COLOR，alpha 取 a（0-255）"""
    if 0 <= x < N and 0 <= y < N:
        i = (y * N + x) * 4
        canvas[i] = COLOR[0]
        canvas[i + 1] = COLOR[1]
        canvas[i + 2] = COLOR[2]
        canvas[i + 3] = a


def rect(canvas, x0, y0, x1, y1):
    for y in range(int(y0), int(y1)):
        for x in range(int(x0), int(x1)):
            put(canvas, x, y)


def downsample(canvas):
    """盒式降采样 N×N → SIZE×SIZE。

    ⚠️ **按"覆盖到的超采样像素数"算 alpha，不是按平均值** ——
    边缘那些只覆盖了一部分的像素应当是**半透明**的，而不是"颜色变淡"。
    平均 RGB 会让边缘混进黑色（因为背景是 (0,0,0,0) 的透明黑），
    表现为图标边缘一圈发暗的脏边。只对覆盖到的像素求颜色平均即可。
    """
    out = bytearray(SIZE * SIZE * 4)
    for y in range(SIZE):
        for x in range(SIZE):
            r = g = b = 0
            cov = 0
            for dy in range(SS):
                for dx in range(SS):
                    i = ((y * SS + dy) * N + (x * SS + dx)) * 4
                    if canvas[i + 3] > 0:
                        r += canvas[i]
                        g += canvas[i + 1]
                        b += canvas[i + 2]
                        cov += 1
            o = (y * SIZE + x) * 4
            if cov > 0:
                out[o] = r // cov
                out[o + 1] = g // cov
                out[o + 2] = b // cov
                out[o + 3] = 255 * cov // (SS * SS)
            # cov == 0 时保持全 0（透明）
    return out


def icon_visual_align():
    """取景框四角 + 中心十字 —— 「识别目标 / 对准」的通用图形。

    为什么不用"一只眼睛"或"准星+圆"：96 px 下笔画一多就糊成一团。
    四角括号 + 一小段十字，笔画少、对称、任意缩放下都能认出来。"""
    c = new_canvas()
    u = N / 96.0                # 1 个成品像素对应的超采样长度

    # 取景框：占 8..88，角落各留一段（经典 focus box）
    x0, y0, x1, y1 = 8 * u, 10 * u, 88 * u, 90 * u
    t = 6 * u                   # 笔画粗
    arm = 20 * u                # 每个角伸出的长度

    # 左上
    rect(c, x0, y0, x0 + arm, y0 + t)
    rect(c, x0, y0, x0 + t, y0 + arm)
    # 右上
    rect(c, x1 - arm, y0, x1, y0 + t)
    rect(c, x1 - t, y0, x1, y0 + arm)
    # 左下
    rect(c, x0, y1 - t, x0 + arm, y1)
    rect(c, x0, y1 - arm, x0 + t, y1)
    # 右下
    rect(c, x1 - arm, y1 - t, x1, y1)
    rect(c, x1 - t, y1 - arm, x1, y1)

    # 中心十字
    cx = (x0 + x1) / 2.0
    cy = (y0 + y1) / 2.0
    half = 11 * u
    rect(c, cx - half, cy - t / 2, cx + half, cy + t / 2)
    rect(c, cx - t / 2, cy - half, cx + t / 2, cy + half)

    return downsample(c)


ICONS = {
    'icon_visual_align.png': icon_visual_align,
}


def main():
    args = sys.argv[1:]
    if '--list' in args:
        for k in sorted(ICONS):
            print(k)
        return 0
    outdir = args[0] if args else '.'
    for name, fn in sorted(ICONS.items()):
        path = os.path.join(outdir, name)
        rgba_to_png(path, fn())
        print('生成 %s (%d 字节)' % (path, os.path.getsize(path)))
    return 0


if __name__ == '__main__':
    sys.exit(main())

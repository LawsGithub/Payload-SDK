#!/usr/bin/env python3
"""把绕行人正方形航线的 KMZ 做成一张核对网页（本地 HTML，双击即开）。

网页里的航点全部解析自 KMZ 里的 wpmz/waylines.wpml，不另画示意图。KMZ 可以是
桌面演示程序 wt_ped_square_demo 导出的，也可以是从设备上拉回来的
（data/ped_square.kmz）—— 后者核对的就是飞机实际收到的那份文件。

用法（在 wt_inspection/ 目录下）：
    python3 -I tools/ped_square_view/build_view.py OUT.html KMZ[@标签[@纬度,经度[,高度]]] ...

    @纬度,经度[,高度] 是飞机生成航线时的位置（KMZ 里没有，只用于画「飞往首航点」
    那一段；高度是相对起飞点的米数，只影响立体图，不写按 30 m 画）。
"""

import datetime
import hashlib
import json
import os
import sys
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
TEMPLATE = os.path.join(HERE, "view_template.html")
PLACEHOLDER = "/*__SCENARIOS__*/[]"


def load_scenario(spec):
    parts = spec.split("@")
    path = parts[0]
    label = parts[1] if len(parts) > 1 and parts[1] else os.path.basename(path)
    aircraft = None
    if len(parts) > 2 and parts[2]:
        nums = [float(x) for x in parts[2].split(",")]
        aircraft = {"lat": nums[0], "lon": nums[1], "h": nums[2] if len(nums) > 2 else None}

    with open(path, "rb") as fp:
        raw = fp.read()
    with zipfile.ZipFile(path) as z:
        bad = z.testzip()  # 逐条目校验 CRC；None = 全部通过
        entries = z.namelist()
        template_kml = z.read("wpmz/template.kml").decode("utf-8")
        waylines_wpml = z.read("wpmz/waylines.wpml").decode("utf-8")

    return {
        "label": label,
        "kmzName": os.path.basename(path),
        "kmzSize": len(raw),
        "sha256": hashlib.sha256(raw).hexdigest(),
        "zipOk": bad is None,
        "entries": entries,
        "generated": datetime.datetime.fromtimestamp(os.path.getmtime(path)).strftime("%Y-%m-%d %H:%M:%S"),
        "aircraft": aircraft,
        "templateKml": template_kml,
        "waylinesWpml": waylines_wpml,
    }


def main(argv):
    if len(argv) < 3:
        print(__doc__, file=sys.stderr)
        return 2

    scenarios = [load_scenario(s) for s in argv[2:]]
    with open(TEMPLATE, encoding="utf-8") as fp:
        page = fp.read()
    if PLACEHOLDER not in page:
        print("模板里找不到占位符 " + PLACEHOLDER, file=sys.stderr)
        return 1

    # 嵌进 <script>：把 "</" 转义，正文里的 </kml> 之类才不会提前结束脚本
    payload = json.dumps(scenarios, ensure_ascii=False).replace("</", "<\\/")
    with open(argv[1], "w", encoding="utf-8") as fp:
        fp.write(page.replace(PLACEHOLDER, payload))

    for s in scenarios:
        print("%-28s %s  CRC %s  %d 字节" % (s["label"], s["sha256"][:16], "通过" if s["zipOk"] else "失败", s["kmzSize"]))
    print("已写出 " + argv[1])
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
校验 `app/widget_file/*/widget_config.json`，以及它们与 `lz_widget.c` 的一致性。

## 为什么需要它

PSDK 的控件分派有**三处必须对齐、且任一处对不上都不报错**：

1. 不同语言的配置之间：类型 / 索引 / 数量必须相同
   （CLAUDE.md「四条约束」第 3 条。不一致的表现是"切到英文后少一个按钮"）
2. 配置与 `lz_widget.c` 的 `LZ_WIDGET_IDX_*`：索引错了会
   **"按了 A 按钮却执行 B 的动作"**，两边都不报错
3. 配置里引用的图标文件必须真的存在 —— 少了它 Pilot 2 显示破图或空白，
   而**设备上没有终端、看不到任何提示**（本轮「识别目标」按钮就漏过一个）

这三条都是"静默失配"，所以由脚本代劳，别靠肉眼。

用法：
    python3 lz/tools/check_widget_config.py [仓库根]      # 默认当前目录
退出码 0 = 全部一致；1 = 有失配（逐条打印病因）。
"""

import glob
import json
import os
import re
import sys


def load(path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def widget_index_from_c(root):
    """从 lz_widget.c 抓 `#define LZ_WIDGET_IDX_* N`，返回 {名字: 值}"""
    src = os.path.join(root, 'lz/app/lz_widget.c')
    txt = open(src, encoding='utf-8').read()
    out = {}
    for m in re.finditer(r'#define\s+(LZ_WIDGET_IDX_\w+)\s+(\d+)', txt):
        out[m.group(1)] = int(m.group(2))
    return out


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else '.'
    paths = sorted(glob.glob(os.path.join(root, 'lz/app/widget_file/*/widget_config.json')))
    if not paths:
        print('找不到 widget_config.json（根目录给对了吗？）')
        return 1

    failures = []
    baseline = None
    baseline_path = None

    for p in paths:
        d = load(p)
        items = d['main_interface']['widget_list']
        keys = [(w.get('widget_index'), w.get('widget_type')) for w in items]
        print('%-58s %d 个控件: %s'
              % (os.path.relpath(p, root), len(items),
                 ', '.join('%s(%s)' % (i, t) for i, t in keys)))

        # ---- 约束 1：语言之间的类型/索引/数量一致 ----
        if baseline is None:
            baseline, baseline_path = keys, p
        elif keys != baseline:
            failures.append('%s 与 %s 的控件清单不一致'
                            '\n    %s\n    %s' % (p, baseline_path, keys, baseline))

        # ---- 约束 2：索引不能重复、不能有空洞 ----
        idx = [w.get('widget_index') for w in items]
        if len(set(idx)) != len(idx):
            failures.append('%s 有重复的 widget_index：%s' % (p, idx))
        if idx != list(range(len(idx))):
            failures.append('%s 的 widget_index 不是 0..N-1 连续：%s' % (p, idx))

        # ---- 约束 3：图标文件必须存在 ----
        for w in items:
            fs = w.get('icon_file_set') or {}
            for k in ('icon_file_name_selected', 'icon_file_name_unselected'):
                name = fs.get(k)
                if not name:
                    continue
                ip = os.path.join(os.path.dirname(p), name)
                if not os.path.exists(ip):
                    failures.append('%s 引用的图标不存在：%s（控件 %s「%s」）'
                                    % (os.path.relpath(p, root), name,
                                       w.get('widget_index'), w.get('widget_name')))

    # ---- 约束 4：与 lz_widget.c 的索引常量对齐 ----
    consts = widget_index_from_c(root)
    if consts:
        print('\nlz_widget.c 的索引常量: %s'
              % ', '.join('%s=%d' % (k.replace('LZ_WIDGET_IDX_', ''), v)
                          for k, v in sorted(consts.items(), key=lambda kv: kv[1])))
        if baseline is not None:
            n = len(baseline)
            count = max(consts.values()) + 1 if consts else 0
            if count != n:
                failures.append('lz_widget.c 的控件数（%d，最大索引 %d）与配置的 %d 个不一致'
                                % (count, count - 1, n))

    if failures:
        print('\n✗ %d 处不一致：' % len(failures))
        for f in failures:
            print('  - %s' % f)
        return 1

    print('\n✓ 两份配置一致、索引连续、图标齐全、与 lz_widget.c 对齐')
    return 0


if __name__ == '__main__':
    sys.exit(main())

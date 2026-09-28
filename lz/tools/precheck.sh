#!/usr/bin/env bash
# 上机前的本机自检（秒级，不需要设备、不需要飞机）。
#
# ## 为什么要有它
#
# 本项目迄今踩过的坑里，有一整类形状是**"能编译、能跑、但不生效"**：
#
#   - 控件索引与 json 对不上   → 按了 A 按钮执行 B 的动作，两边都不报错
#   - 配置引用了不存在的图标   → Pilot 2 显示破图，设备上没有任何提示
#   - 控件回调里做了阻塞调用   → PSDK 线程被拖死，进程闪退
#   - 两个语言目录配置不一致   → 切到英文少一个控件
#
# 这些都不该等到上机才发现。本脚本把它们压成一条命令。
#
# 用法：  bash lz/tools/precheck.sh          （在仓库根跑）
# 退出码：0 = 全过；1 = 有失败项（逐条打印）
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
FAIL=0
step() { printf '\n\033[1m== %s ==\033[0m\n' "$1"; }
ok()   { printf '  ✓ %s\n' "$1"; }
bad()  { printf '  ✗ %s\n' "$1"; FAIL=1; }

# ------------------------------------------------------------------
step "1. 控件配置（索引 / 图标 / 两语言一致 / 与 lz_widget.c 对齐）"
# ------------------------------------------------------------------
if python3 lz/tools/check_widget_config.py . ; then
    ok "控件配置一致"
else
    bad "控件配置有失配（见上）"
fi

# ------------------------------------------------------------------
step "2. 控件回调里不得有阻塞调用"
# ------------------------------------------------------------------
# 纪律原文见 lz_widget.c 的 `LzWidget_SetWidgetValue` 上方：
# 回调跑在 PSDK 工作线程上，任何阻塞调用都会把 SDK 链路拖死
# （2026-09-22 实测：`semaphore wait timeout` 刷屏后进程闪退）。
#
# 做法：把 SetWidgetValue 的函数体抠出来（从它的左花括号到下一个顶层
# `}`），在**剥掉注释**之后搜禁用符号。剥注释是必须的 —— 说明文字里
# 恰好会提到这些函数名（比如"要调 DjiCameraManager_GetLaserRangingInfo"），
# 不剥的话每一行注释都会变成假阳性。
CB_BODY=$(python3 - <<'PY'
import re, sys
src = open('lz/app/lz_widget.c', encoding='utf-8').read()
m = re.search(r'static T_DjiReturnCode LzWidget_SetWidgetValue\([^)]*\)\s*\{', src)
if not m:
    sys.exit('找不到 LzWidget_SetWidgetValue')
i = m.end(); depth = 1
while i < len(src) and depth:
    if src[i] == '{': depth += 1
    elif src[i] == '}': depth -= 1
    i += 1
body = src[m.end():i-1]
body = re.sub(r'/\*.*?\*/', '', body, flags=re.S)   # 块注释
body = re.sub(r'//[^\n]*', '', body)                # 行注释
print(body)
PY
)
if [ -z "$CB_BODY" ]; then
    bad "抠不出 SetWidgetValue 的函数体（脚本要跟着改）"
else
    HIT=0
    for sym in DjiCameraManager_ LzPole_Record LzBridge_GetCurrentPosition \
               LzVisionSource_ LzVisualAlign_Tick DjiGimbalManager_ fopen sleep_ms \
               LzVision_ Detect; do
        if printf '%s' "$CB_BODY" | grep -q "$sym"; then
            bad "控件回调里出现了阻塞/跨层调用：$sym"
            HIT=1
        fi
    done
    [ "$HIT" = 0 ] && ok "回调只置标志（未发现阻塞/跨层调用）"
fi

# ------------------------------------------------------------------
step "2b. 判据真的被接线调用了（「判据对 ≠ 接线对」）"
# ------------------------------------------------------------------
# 本项目踩过这个形状：给激光零解闸门写的测试只断言了
# `LzGeo_IsNullSolution()` **判据本身**有效，没断言取数函数真的调用它 ——
# 反向验证时删掉闸门，测试**照样全绿**，因为那条路径没参与编译。
# 见 CLAUDE.md「判据对 ≠ 接线对」。
#
# 这里做的是同一件事的**静态版**：判据全在 `lz_core/src/lz_align.c`，
# 而调用点在 app 层（桌面测不到）。所以至少要确认调用点存在 ——
# 判据再好，没人调它等于没有。
check_call() {  # 文件 符号 说明
    if grep -q "$2" "$1"; then
        ok "$3"
    else
        bad "$1 里没有调用 $2 —— $3"
    fi
}
check_call lz/app/lz_visual_align.c 'LzAlign_DecideStep' \
    "照准的每一步判定走 lz_core 的判据（不是 app 里另写一份）"
check_call lz/app/lz_visual_align.c 'LzAlign_Retry_NoteFrameOk' \
    "取到帧时清计数走 lz_core（旧写法在这里手写 s_waitTicks = 0）"
check_call lz/app/lz_visual_align.c 'LzAlign_Retry_NoteDetectMiss' \
    "检测失败计数走 lz_core"
check_call lz/app/lz_visual_align.c 'LzAlign_DefaultPolicy' \
    "参数取自 lz_core 的缺省值（两处各写一份会漂移）"
check_call lz/app/lz_visual_align.c 'LzAlign_DecideToggle' \
    "按钮的开始/停止/吞掉走 lz_core 的判据（旧写法手判 s_step != ST_IDLE，会在启动期自杀）"

# 互斥必须**两个方向都拦** —— 而且是**逐方向**查的。
#
# ⚠️ 只数"调了几次"不够：实测把绕飞那一侧的实参换成常量、只留调用形式，
# `grep -c` 照样数到 2 处，检查会**绿着放过**。所以这里查的是两个方向
# 各自的**实参形态**：
#   · 照准方向问的是"绕飞在跑吗" ⇒ 实参里有 LzMission_IsRunning()
#   · 绕飞方向问的是"照准在跑吗" ⇒ 实参里有 LZ_ALIGN_RUNNING
# 两个方向都各有且仅有它自己那个判据来源。
N_ORBIT=$(grep -A1 'LzAlign_CheckConflict(' lz/app/lz_mission.c | grep -c 'LzMission_IsRunning()' || true)
N_ALIGN=$(grep -c 'LzVisualAlign_State() == LZ_ALIGN_RUNNING' lz/app/lz_mission.c || true)
if [ "$N_ORBIT" -ge 1 ]; then
    ok "互斥方向一：按「识别目标」时查了绕飞是否在跑"
else
    bad "按「识别目标」时**没有**查绕飞是否在跑（照准会插进正在执行的航线）"
fi
if [ "$N_ALIGN" -ge 2 ]; then
    ok "互斥方向二：拨绕飞开关时查了照准是否在跑（两处：请求转发处 + 启动处）"
else
    bad "拨绕飞开关时**没有**查照准是否在跑（云台会被两处同时抢）"
fi

check_call lz/app/lz_mission.c 'LzWidget_TakeAlignRequest' \
    "「识别目标」按钮的请求在主循环里被消费"
check_call lz/app/lz_widget.c   's_alignPending' \
    "「识别目标」按钮的回调置了待办标志"

# ---- 控件的**界面归属**：int_input_box 绝不能放 main_interface ----
# 2026-09-28 实测：放 main 时 SDK 收下（解析计数正常）但 Pilot 不渲染成
# 可编辑控件 —— 日志里 type=5 的回调**零命中**。官方样例里
# main_interface 根本没有这个类型；int_input_box 只出现在 config_interface。
if python3 - <<'PY'
import json, glob, sys
bad = []
for p in glob.glob('lz/app/widget_file/*/widget_config.json'):
    d = json.load(open(p, encoding='utf-8'))
    for w in d.get('main_interface', {}).get('widget_list', []):
        if w.get('widget_type') == 'int_input_box':
            bad.append('%s: index %s' % (p, w.get('widget_index')))
sys.exit(1 if bad else 0)
PY
then
    ok "int_input_box 都在 config_interface（Payload Settings）"
else
    bad "int_input_box 出现在 main_interface —— 放那里 Pilot 不会渲染成可编辑控件"
fi

# 「按下当场回执」是打断"没反应→再按一次"循环的那一环，掉了就退回旧现场形态。
# ⚠️ 查的是**回调函数体里**有没有 PostMessage，不是全文件 ——
# 文件里别处也有 PostMessage（浮窗状态推送），全文件查等于没查。
if printf '%s' "$CB_BODY" | grep -q 'LzWidget_PostMessage'; then
    ok "控件回调里按下当场回执（打断「没反应→再按一次」的循环）"
else
    bad "控件回调里没有按下回执 —— 操作员会因「没反应」而重复按，触发启动期自杀"
fi

# 取图收尾**不能**反初始化 liveview 模块 —— 那会吃掉 Pilot 的飞行画面。
# ⚠️ 必须先剥注释：那段说明本身就会引用 `DjiLiveview_Deinit()` 这个名字
# （解释"为什么刻意不调它"），不剥的话检查永远红 —— 与回调阻塞检查同一个坑。
SRC_NOCOMMENT=$(python3 - <<'PY'
import re
src = open('lz/app/lz_vision_source.c', encoding='utf-8').read()
src = re.sub(r'/\*.*?\*/', '', src, flags=re.S)
src = re.sub(r'//[^\n]*', '', src)
print(src)
PY
)
if printf '%s' "$SRC_NOCOMMENT" | grep -q 'DjiLiveview_Deinit'; then
    bad "取图里仍在调 DjiLiveview_Deinit() —— 会让 Pilot 画面变黑（2026-09-28 实测）"
else
    ok "取图只停流、不反初始化 liveview（保 Pilot 画面）"
fi

# ------------------------------------------------------------------
step "3. 桌面回归（stub 与 hsv 两个后端都跑）"
# ------------------------------------------------------------------
for backend in stub hsv; do
    B="build-precheck-$backend"
    if [ "$backend" = "stub" ]; then
        cmake -S lz -B "$B" >/dev/null 2>&1
    else
        cmake -S lz -B "$B" -DLZ_VISION_BACKEND=hsv >/dev/null 2>&1
    fi
    if ! cmake --build "$B" -j4 >/tmp/precheck-$backend.log 2>&1; then
        bad "$backend 后端构建失败（/tmp/precheck-$backend.log）"
        continue
    fi
    if grep -qE 'warning:' /tmp/precheck-$backend.log; then
        # 只关心**我们自己的**文件；PSDK 的 hal/osal 有既存 warning
        OURS=$(grep -E 'warning:' /tmp/precheck-$backend.log \
               | grep -E 'lz/|/lz/(app|src|include|tests)/' || true)
        if [ -n "$OURS" ]; then
            bad "$backend 后端：我们的代码有 warning"
            printf '%s\n' "$OURS" | sed 's/^/      /'
        else
            ok "$backend 后端：0 warning（PSDK 官方 hal/osal 的既存 warning 不计）"
        fi
    else
        ok "$backend 后端：0 warning"
    fi
    if ctest --test-dir "$B" >/tmp/precheck-$backend-ctest.log 2>&1; then
        ok "$backend 后端：$(grep -oE '[0-9]+% tests passed[^,]*' /tmp/precheck-$backend-ctest.log | head -1)"
    else
        bad "$backend 后端：ctest 有失败（/tmp/precheck-$backend-ctest.log）"
    fi
done

# ------------------------------------------------------------------
step "4. 两条配置的必需字段（app.json 的 build_dpk.sh 前置）"
# ------------------------------------------------------------------
APPJSON=lz/app_json/app.json
if [ -f "$APPJSON" ]; then
    # ⚠️ 字段是**嵌套**的：`description` 是个对象，四语言是它的键
    # （CLAUDE.md 里简称 `description_jp`）。早先按扁平字段查过，
    # 结果误报"四个全缺" —— 校验脚本自己也要能自证，所以这里连
    # `name` 的四语言一起查，缺哪条就报哪条。
    MISS=$(python3 - "$APPJSON" <<'PY'
import json, sys
d = json.load(open(sys.argv[1], encoding='utf-8'))
miss = []
for parent in ('name', 'description'):
    obj = d.get(parent) or {}
    for lang in ('cn', 'en', 'jp', 'fr'):
        if not obj.get('%s_%s' % (parent, lang)):
            miss.append('%s_%s' % (parent, lang))
print(' '.join(miss))
PY
)
    if [ -n "$MISS" ]; then
        bad "app.json 缺字段：$MISS（build_dpk.sh 会 KeyError 退出）"
    else
        ok "app.json 四语言描述齐全"
    fi
    BIN=$(python3 -c "import json;print(json.load(open('$APPJSON'))['bin'])")
    case "$BIN" in
      *build-native*) ok "app.json 的 bin 指向 build-native（$BIN）" ;;
      *) bad "app.json 的 bin 不是 build-native 路径：$BIN" ;;
    esac
else
    bad "找不到 $APPJSON"
fi

# ------------------------------------------------------------------
printf '\n'
if [ "$FAIL" = 0 ]; then
    printf '\033[32m全部通过 —— 可以上机了\033[0m\n'
else
    printf '\033[31m有失败项，先修再上机\033[0m\n'
fi
exit $FAIL

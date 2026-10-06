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

# ---- 状态机的**转移判据必须在 lz_core**，且 app 侧不得再抄一份 ----
#
# 2026-10-07 抽的。此前整张转移表在 `app/lz_mission.c` 里 ⇒ 桌面测不到 ⇒
# **一条断言都没有**，而走错的后果是安全相关的谎话
# （"飞机还在绕而界面说停了"）。
#
# 这里守的是**抽出去之后没有被抄回来**：
#   · 两相式必须真的被调用（`LzMission_Decide` / `LzMission_Apply`）
#   · app 侧不许再出现那两句关键判断的字面形态
#     （`s_state == LZ_MISSION_STATE_RUNNING` 之外的结束判定、
#      以及"停止失败也置 IDLE"）
#
# ⚠️ 与云台那两条同一个教训：`check_call` 只问符号在不在文件里，
# 真正拦得住"抄回来"的是**查关键形态**。
check_call lz/app/lz_mission.c 'LzMission_Decide' \
    "状态机第一相走 lz_core 的判据（不是 app 里另写一份 if/else）"
check_call lz/app/lz_mission.c 'LzMission_Apply' \
    "状态机第二相走 lz_core 的判据（新状态与要报哪条消息都由它给）"
check_call lz/app/lz_mission.c 'LzMission_ShouldReportWaypoint' \
    "航点号去重走 lz_core（旧写法在回调里手比 s_lastReportedWaypoint）"
check_call lz/app/lz_mission.c 'LzMission_ShouldNoteEnded' \
    "「收到 IDLE 该不该置结束标志」走 lz_core（旧写法手判 s_state == RUNNING）"

# ⚠️ **"停止失败也置 IDLE"这个缺陷形态不许回来**。
#
# 原缺陷的写法是：`(void)LzBridge_StopMissionV3();` 然后**无条件**
# `s_state = LZ_MISSION_STATE_IDLE;` —— 飞机还在绕而界面说停了。
#
# ⚠️ 第一版这条检查写错了：它数"全文有几处 `s_state = IDLE`"，
# 而 `Init` / `DeInit` 里本来就有两处合法的 ⇒ **恒红**。
# 改成查**Tick 那个函数体里**有没有裸的 IDLE 赋值 ——
# 与「`teardown()` 要抠函数体再查」是同一个教训：
# **数全文出现次数不够，要问"在哪个函数里"。**
STRAY_IDLE=$(python3 - <<'PYEOF'
import re
src = open('lz/app/lz_mission.c', encoding='utf-8').read()
src = re.sub(r'/\*.*?\*/', '', src, flags=re.S)
src = re.sub(r'//[^\n]*', '', src)
m = re.search(r'void LzMission_Tick\(void\)\s*\{', src)
if not m:
    print(-1); raise SystemExit
i = m.end(); depth = 1
while i < len(src) and depth:
    if src[i] == '{': depth += 1
    elif src[i] == '}': depth -= 1
    i += 1
body = src[m.end():i-1]
# Tick 里合法的那一处：飞机侧结束的收尾（紧跟 s_missionEnded = false）
bad = 0
for mm in re.finditer(r's_state\s*=\s*LZ_MISSION_STATE_IDLE\s*;', body):
    if 's_missionEnded = false;' not in body[mm.end():mm.end()+120]:
        bad += 1
print(bad)
PYEOF
)
if [ "$STRAY_IDLE" = "0" ]; then
    ok "Tick 里没有裸的 s_state = IDLE（新状态一律由 Apply 给）"
elif [ "$STRAY_IDLE" = "-1" ]; then
    bad "抠不出 LzMission_Tick() 的函数体（脚本要跟着改）"
else
    bad "LzMission_Tick 里有 $STRAY_IDLE 处裸的 s_state = IDLE —— 状态转移判据被抄回 app 层了"
fi

# ---- 云台状态的**判据必须在 lz_core**，且 PSDK 侧不得再抄一份 ----
#
# 2026-10-01 加的 `LzBridge_GimbalStatusStr()` 里有两处**方向性**判据
# （限位 1=顶限位 / ESC 1=正常，极性相反；mountStatus=0 时其余位无意义），
# 抄反了不报错、只让告警恰好反过来。⇒ 判据抽到零依赖的
# `src/lz_gimbal_status.c`，桌面用 `lz_test_gimbal` 钉死。
#
# 这里守的是**抽出去之后没有被抄回来**：
#   · 格式化必须走 lz_core（`LzGimbalStatus_Format`）
#   · PSDK 侧不许再出现那七个报警词的**字面量**（抄回去就会各写一份）
# 反向验证：把 `LzGimbalStatus_Format(&st, buf, size);` 换成手写 snprintf
# 拼字面量 ⇒ 两条都变红。
check_call lz/src/lz_bridge_psdk.c 'LzGimbalStatus_Format' \
    "云台状态的格式化走 lz_core 的判据（不是 PSDK 侧另写一份）"

# ⚠️ 上面那条 `check_call` 是**弱**的：它只问符号在不在文件里，而
# `LzGimbalStatus_Format` 在**注释**里也出现。实测把真正的调用换成
# `snprintf(buf, size, "%s", st.yawLimited ? "偏航限位" : "正常")` 之后
# 它**照样绿**。真正拦得住的是下面这条"报警项文案不许出现在 PSDK 侧"
# —— 与互斥那两条「逐方向查实参形态」同一个教训：
# **数符号出现次数不够，要问"在哪个函数里、什么形态"。**
# ⚠️ **必须先剥注释再数**：那七个报警词的说明文字在注释里本来就会出现
# （"用来看见「云台电机异常」"这种），不剥的话每行注释都是假阳性 ——
# 与第 2 项（回调里不得有阻塞调用）踩过的是同一个坑。
#
# ⚠️ 查的是**带引号的完整项名**（`"偏航限位"` 这种），不是"限位"两个字 ——
# 原始位图那行日志里有 `限位[pitch=...]` 这种**标签**，它不是报警项文案，
# 按子串查会把它误判成"抄回来了"。
GIMBAL_DUP=$(python3 - <<'PYEOF'
import re
src = open('lz/src/lz_bridge_psdk.c', encoding='utf-8').read()
src = re.sub(r'/\*.*?\*/', '', src, flags=re.S)   # 块注释
src = re.sub(r'//[^\n]*', '', src)                # 行注释
names = ('俯仰限位', '横滚限位', '偏航限位', '俯仰电机异常', '横滚电机异常',
         '偏航电机异常', '陀螺故障')
print(sum(src.count('"%s"' % n) for n in names))
PYEOF
)
if [ "$GIMBAL_DUP" -eq 0 ]; then
    ok "PSDK 侧没有抄回报警项的文案（文案真值只在 src/lz_gimbal_status.c）"
else
    bad "lz_bridge_psdk.c 的**代码**里出现了 $GIMBAL_DUP 处报警项文案 —— 判据被抄回来了，两处会漂移"
fi

# ---- 浮窗上按文案分派的那几处，必须与 lz_core 的文案对得上 ----
#
# `lz_mission.c` 用 `strstr(now, "偏航限位")` 分派出两条针对性浮窗。
# 文案真值在 `src/lz_gimbal_status.c`，改那边不同步改这边**不会报错**，
# 只会让那两条浮窗静默退化成通用的那条 —— 与控件索引、图标文件名
# 那类"两边各写一份、对不上也不报错"同一个形状。
for W in 偏航限位 偏航电机异常; do
    if grep -q "$W" lz/src/lz_gimbal_status.c && grep -q "$W" lz/app/lz_mission.c; then
        ok "浮窗分派文案「$W」两边一致"
    else
        bad "文案「$W」在 lz_core 与 lz_mission.c 之间**对不上** —— 那条浮窗会静默失效"
    fi
done

# ---- 照准收尾必须**把云台模式还回去** ----
#
# ⚠️ 2026-10-01 上机实测的缺陷：`do_init()` 把云台设成 `FREE`（横向照准需要
# 它），而**收尾时不恢复** —— `FREE` 是飞机上的全局状态，于是它一直留着。
# 绕飞时机头绕杆连续转 360°，而 `FREE` 要求云台保持地面姿态 ⇒ pan 关节必须
# 反向补偿 360°，而 M4T 的 pan 只有 ±60° ⇒ 顶到限位、电机异常。
# 航线一结束遥控器拿回控制权就“自己好了”。
#
# ⚠️ **必须抠 `teardown()` 的函数体来查，不能全文件搜这个符号** ——
# `do_init()` 里也有一处 `SetMode(YAW_FOLLOW)`（FREE 被拒时的降级路径），
# 全文件搜会让检查恒绿。这与互斥那两条「逐方向查实参形态」是同一个教训：
# **数符号出现次数不够，要问“在哪个函数里、什么形态”。**
# 第一版就是这么写错的，反向验证（把恢复那段整块删掉）时它照样绿。
TEARDOWN_BODY=$(python3 - <<'PYEOF'
import re, sys
src = open('lz/app/lz_visual_align.c', encoding='utf-8').read()
m = re.search(r'static void teardown\(void\)\s*\{', src)
if not m:
    sys.exit('找不到 teardown()')
i = m.end(); depth = 1
while i < len(src) and depth:
    if src[i] == '{': depth += 1
    elif src[i] == '}': depth -= 1
    i += 1
body = src[m.end():i-1]
body = re.sub(r'/\*.*?\*/', '', body, flags=re.S)   # 块注释（说明里会提到这两个符号）
body = re.sub(r'//[^\n]*', '', body)                # 行注释
print(body)
PYEOF
)
if [ -z "$TEARDOWN_BODY" ]; then
    bad "抠不出 teardown() 的函数体（脚本要跟着改）"
elif printf '%s' "$TEARDOWN_BODY" | grep -q 'DJI_GIMBAL_MODE_YAW_FOLLOW'; then
    ok "照准收尾把云台模式恢复成 YAW_FOLLOW（FREE 用完必须还，否则绕飞撞限位）"
else
    bad "照准收尾**没有**恢复云台模式 —— FREE 会留在飞机上，绕飞时云台偏航撞 ±60° 限位"
fi

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
step "3b. 消毒器（ASan + UBSan）—— 内存类缺陷只有它能抓"
# ------------------------------------------------------------------
# 2026-10-06 加的。第一次跑就抓到 `lz_detect_pole_column()` 里一处
# **use-after-free**：`free(votesHist)` 写在中位数那段**之前**，而那段
# 紧接着就读它。被释放的内存通常还没被复用 ⇒ 读到原值 ⇒ 测试全绿、
# 黄金值全对，换个分配器/负载就会变成错值或崩溃。
#
# ⚠️ 它慢（约 0.3 s vs 0.06 s），所以**不放进第 3 项**、单独一步；
# 失败时的日志指向 ASan 报告，而那份报告会直接给出读/写与 free 的
# 两个栈 —— 比任何人工排查都快。
ASAN_DIR="build-precheck-asan"
cmake -S lz -B "$ASAN_DIR" -DLZ_VISION_BACKEND=hsv -DLZ_SANITIZE=ON >/dev/null 2>&1
if cmake --build "$ASAN_DIR" -j4 >/tmp/precheck-asan.log 2>&1; then
    # ⚠️ 黄金值文件走环境变量传**绝对路径** —— ctest 的工作目录是构建目录，
    # 相对路径 `tests/data/...` 在那里找不到（本项目踩过）。
    if LZ_VISION_TESTDATA="$PWD/lz/tests/data" \
       ctest --test-dir "$ASAN_DIR" >/tmp/precheck-asan-ctest.log 2>&1; then
        ok "ASan+UBSan：$(grep -oE '[0-9]+% tests passed[^,]*' /tmp/precheck-asan-ctest.log | head -1)"
    else
        bad "ASan+UBSan 有失败（/tmp/precheck-asan-ctest.log）—— 内存类缺陷，先看 ASan 报告"
    fi
else
    bad "消毒器构建失败（/tmp/precheck-asan.log）"
fi

# ------------------------------------------------------------------
step "3c. aarch64 交叉编译（不是产物验证，是「设备上才会撞到的编译错误」）"
# ------------------------------------------------------------------
# ⚠️ 先说清楚它**不能**证明什么：产物要求 GLIBC_2.34，而设备是 2.31 ⇒
# **交叉编译的产物在设备上跑不起来**，本项目的硬约束「绝不在本机交叉编译
# 交付产物」**不变**（见 CLAUDE.md「硬约束 3」）。
#
# 那它有什么用：**把"设备上才会撞到的编译错误"提前到桌面**。设备不在线时，
# 一个 aarch64 特有的语法/类型错误要等到能上机才发现；而本机有
# `aarch64-linux-gnu-gcc`，整个 PSDK 应用编一遍只要 **2 秒**。
#
# 反向验证（2026-10-06）：往 lz_mission.c 塞一个
# `struct { unsigned int a : 40; }` ⇒ 这里报
# `error: width of 'a' exceeds its type` 而**桌面构建不报** —— 说明它确实
# 在查桌面查不到的东西，不是重复第 3 步。
if ! command -v aarch64-linux-gnu-gcc >/dev/null 2>&1; then
    echo "  - 本机没有 aarch64-linux-gnu-gcc，跳过（不影响上机）"
else
    ARM_DIR="build-precheck-arm"
    if cmake -S lz -B "$ARM_DIR" -DLZ_BUILD_PSDK_APP=ON -DLZ_TARGET_ARCH=aarch64 \
             -DLZ_VISION_BACKEND=hsv -DLZ_BUILD_TESTS=OFF -DLZ_BUILD_DESKTOP_TOOLS=OFF \
             -DLZ_POLE_SOURCE_LASER=ON \
             -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc \
             -DPSDK_ROOT="$HOME/projects/Payload-SDK" >/tmp/precheck-arm-cfg.log 2>&1 \
       && cmake --build "$ARM_DIR" -j4 >/tmp/precheck-arm.log 2>&1; then
        OURS=$(grep -E 'warning:|error:' /tmp/precheck-arm.log | grep 'lz/' || true)
        if [ -n "$OURS" ]; then
            bad "aarch64 交叉编译：我们的代码有 warning"
            printf '%s\n' "$OURS" | sed 's/^/      /'
        else
            ok "aarch64 交叉编译通过（0 warning；产物不用于交付，见 CLAUDE.md 硬约束 3）"
        fi
    else
        bad "aarch64 交叉编译失败（/tmp/precheck-arm.log）—— 这类错误在桌面构建里看不到"
    fi
fi

# ------------------------------------------------------------------
step "3d. 覆盖率基线（gcov）—— 只在明显掉下来时提醒，不设硬门槛"
# ------------------------------------------------------------------
# 2026-10-07 加。为什么**不设硬门槛**：本项目的覆盖率本来就是"哪些判据
# 抽到了桌面可测的位置"的副产品，而不是目标本身 —— 拿它当 KPI 会诱导
# 写"走过场"的用例。它的用处是**发现"判据被搬回了 app 层"**：
# 抽走判据时覆盖率会跳，搬回去时会掉。
#
# ⚠️ **统计口径必须先对齐**：`.gcda` 是按"哪个测试走到了它"累加的 ——
# 单独跑某一个测试会得到偏低的值（实测 `bridge.c` 单跑 43% / 全跑 93%；
# `vision.c` 单跑 0%）。所以这里把**全部测试目标**都跑一遍再合并。
if ! command -v gcov >/dev/null 2>&1; then
    echo "  - 本机没有 gcov，跳过（不影响上机）"
else
    COV_DIR="build-precheck-cov"
    rm -rf "$COV_DIR" && mkdir -p "$COV_DIR"
    CORE_SRC="types target geo plan bridge wpml kmz align gimbal_status mission_logic"
    for f in $CORE_SRC; do
        gcc -std=gnu99 --coverage -O0 -g -Ilz/include -Ilz/tests \
            -c "lz/src/lz_$f.c" -o "$COV_DIR/lz_$f.o" 2>/dev/null &
    done
    wait
    for t in geo plan kmz wpml validate align gimbal mission bridge; do
        gcc -std=gnu99 --coverage -O0 -g -Ilz/include -Ilz/tests \
            -o "$COV_DIR/t_$t" "lz/tests/lz_test_$t.c" "$COV_DIR"/lz_*.o -lm 2>/dev/null \
          && (cd "$COV_DIR" && "./t_$t" >/dev/null 2>&1)
    done
    # ⚠️ **`lz_test_vision` 必须也算进来**：`target.c` 的 `LzTargetList_*`
    # 主要由它覆盖 —— 漏了它 `target.c` 只有 19%（实测），
    # 而那条会被误读成"判据被搬回 app 层了"。
    # 这正是"统计口径要先对齐"那条注释的实例。
    gcc -std=gnu99 --coverage -O0 -g -Ilz/include -Ilz/tests \
        -o "$COV_DIR/t_vision" "lz/tests/lz_test_vision.c" "$COV_DIR"/lz_*.o \
        lz/src/lz_vision.c lz/src/lz_vision_shared.c -lm 2>/dev/null \
      && (cd "$COV_DIR" && LZ_VISION_TESTDATA="$ROOT/lz/tests/data" ./t_vision >/dev/null 2>&1)
    gcc -std=gnu99 --coverage -O0 -g -Ilz/include -Ilz/tests \
        -o "$COV_DIR/t_vm" "lz/tests/lz_test_vision_math.c" "$COV_DIR"/lz_*.o \
        lz/src/lz_vision_shared.c -lm 2>/dev/null \
      && (cd "$COV_DIR" && ./t_vm >/dev/null 2>&1)
    # ⚠️ 门槛取 80%：它是"明显掉下来"的判据，不是目标。
    # 实测基线（2026-10-07）：types 100 / mission_logic 100 / align 99 /
    # wpml 96 / gimbal_status 95 / bridge 93 / geo 92 / kmz 90 /
    # plan 89 / target 87 —— 最低 87%，留 7 个点余量。
    COV_MIN=80   # 门槛的**唯一定义处**（报错文案从它取）
    # ⚠️ gcov 的汇总行打在 **stdout**，不在 `.gcov` 文件里（实测）——
    # 第一版去 grep `.gcov` 文件，于是永远取到空值、检查恒绿。
    # 这是"检查看起来在跑、实际什么都没查"的又一个实例。
    LOW=""
    MIN=""
    for f in $CORE_SRC; do
        PCT=$( ( cd "$COV_DIR" && gcov -o . "lz_$f.gcda" 2>/dev/null ) \
               | grep -m1 'Lines executed' | grep -oE '[0-9]+\.[0-9]+' )
        if [ -z "$PCT" ]; then
            bad "取不到 $f.c 的覆盖率（gcov 输出格式变了？脚本要跟着改）"
            LOW="yes"
            continue
        fi
        # ⚠️ 门槛只在**一个**变量里，文案从它取 —— 写死两处会漂移
        # （第一版就是这么写的，把门槛临时改成 88% 时文案仍报"< 80%"）。
        if awk -v p="$PCT" -v t="$COV_MIN" 'BEGIN{ exit !(p+0 < t+0) }'; then
            bad "$f.c 覆盖率 ${PCT}%（< ${COV_MIN}%）—— 判据可能被搬回 app 层了"
            LOW="yes"
        fi
        # ⚠️ 第一版写成 `printf '%s\n%s\n' "$MIN" "$PCT" | sort -n | head -1`
        # —— 那是**给 MIN 赋一个含两行的字符串**，下一轮再拼进去越滚越长。
        # 正确做法：用 `sort -n` 比大小后**只取一个数**。
        if [ -z "$MIN" ] || awk -v a="$PCT" -v b="$MIN" 'BEGIN{ exit !(a+0 < b+0) }'; then
            MIN="$PCT"
        fi
    done
    [ -z "$LOW" ] && ok "lz_core 各文件覆盖率均 ≥ ${COV_MIN}%（最低 ${MIN}%）"
fi

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

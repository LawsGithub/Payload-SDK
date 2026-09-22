# HANDOFF — 读全文再开始干活

生成时间: 2026-09-22T11:59:58+0800 · Git HEAD: c1463ba
信任规则: [V] = 交接时已用命令验证；[?] = 仅记忆未复核，当线索对待；[X] = 已证伪，别用。

## 0. 复核（下一会话先做）

- 锚点: `feature/liangzhourenwu` @ `c1463ba`（2026-09-22 11:59）
- 漂移检查: `git rev-parse HEAD~1` 应 = `c1463ba`——HEAD 是本次 handoff 提交，
  其 parent 才是本快照记录的 SHA；变了说明快照可能过期
- **远端已同步** `[V]`（push 成功，见 §2 末尾）。若 `git status` 显示领先，
  重试 `git push fork feature/liangzhourenwu` —— **必须走 HTTP 代理**，
  直连会挂起（见 §5）
- **设备当前不可达** `[V]`（11:52 实测：WSL 侧已无 `192.168.42.x` 网卡，
  `ping` 100% 丢包、`ssh` 超时）。需在 Windows 侧重做 USB 网络共享附加
- 先读: `~/projects/liangzhourenwu/CLAUDE.md`（项目约定，本会话大改过）

## 1. 当前目标

**让「操作员在 Pilot 2 拨开关 → 飞机绕国旗杆飞一圈」完整可用。**

完成定义 = 拨 ON 后飞机按设定半径/高度绕记录的点一周并返航。

**当前卡点：只剩最后一环未跑 —— 绕飞完整链路。**

## 2. 已验证状态 — 工作实际停在哪

**本会话把「圆心来源」从写死的坐标改成操作员现场记录，并修掉两个闪退/误报。**

### 本会话新验证的（都是设备实测）

- **航点任务能真正启动** `[V]` —— 2026-09-22 室外首次成功。
  错误码随环境变化：模拟器 770 / 室内 769+771 / 室外 770  /
  **修完 wpml 后成功**。详见 CLAUDE.md「绕飞启动曾经失败的原因」
- **激光测距可用** `[V]` —— 记录 `28.1788176, 112.9210889`（距离 4.4 m）；
  另一组 `28.1789403, 112.9210401`（距离 14.8 m）。
  **与「飞机位置 + 距离」反算一致**（4.1 m / 14.7 m）→ 冗余数据自洽
- **记录飞机位可用** `[V]` —— `28.1788120, 112.9210473`
- **落盘** `[V]` —— `data/pole.txt`：`lon=... lat=... alt=... src=aircraft|laser`
- **两个按钮回调都触发** `[V]` —— 日志 `[widget] ✓ 已记录...`
- **相机模块初始化** `[V]` —— 无报错（只在失败时打印）

### 本会话修掉的（各自都有反向验证）

| 问题 | 根因 | 证据 |
|---|---|---|
| 按「记录激光点」**进程闪退** | 回调里调了阻塞 1.2s 的相机接口，卡死 SDK 链路线程 | 日志戛然而止于 `semaphore wait timeout` + `send msg to queue error` |
| 激光**误报「无回波」** | `exception` 白名单只认 `3`，而实测 `0` 也是有效的 | 距离在变（4.4→14.8 m），无回波时恒为 0 |
| 记录下**零解 (0,0)** | 零解判据写成 `== 0.0`，而残差是 `3e-7` | `pole.txt` 里出现过 `lon=0.0000000` |

### 测试/build 输出（本次交接 run 的**真实输出**）

```
PC 侧:   cmake -S lz -B build && cmake --build build -j4   → 0 error / 0 warning
         ctest --test-dir build                             → 100% passed, 0 failed out of 7
x64 侧:  cmake --build lz/build-x64 -j4                     → 我们自己代码 0 warning
lz_test_pole 明细: 53 项检查，0 项失败（新增 3 个激光闸门用例）
反向验证: 拆闸门① → 2 条变红；拆闸门③ → 3 条变红
```

### git 状态（事实快照，非待办）

```
feature/liangzhourenwu @ c1463ba，工作区干净，本地与远端一致
c1463ba docs(claude): 上机确认一节改写成事实 + 新增「绕飞启动曾经失败的原因」
3035380 docs(claude): 更正 exception 判据表 + 圆心改为操作员输入 + 四条新坑
786be96 fix(lz): 激光判据改用 distance，补零解闸门
9308714 fix(lz): 激光记录搬出 PSDK 回调线程 —— 修按下按钮闪退
76d2d5f feat(lz): 绕飞圆心改为操作员在 Pilot 2 上记录的两个点
```

## 3. 决策与理由

- **圆心改为操作员现场记录，不再用写死的固定坐标** `[V]` ——
  记录落 `data/pole.txt`，未记录时**拒绝启动绕飞**（`LZ_ERR_NOT_READY`）。
  否决方案：未记录时回落到固定坐标 `28.1788480, 112.9210020` ——
  **回落到几十公里外的点会让飞机飞过去，而操作员以为在原地绕圈**。
  ⚠️ 只拦「启动绕飞」，**不拦飞机起飞**（程序从不控制起飞）。
- **激光判据用 `distance` 而非 `exception` 白名单** `[V]` ——
  `exception` 取值官方从未公开，实测已见 0/1/2/3，白名单必然漏。
  `distance` 是唯一可自证的量（无回波恒为 0）。**可推广：取值域无权威来源时
  别用枚举白名单。**
- **控件回调禁止阻塞调用** `[V]` —— 回调跑在 PSDK 工作线程，阻塞会卡死链路。
  「取数」搬到主循环，回调只置标志。**违反它不会编译报错**，只能靠审查。
- **判定逻辑抽成零依赖纯函数**（`LzPole_JudgeLaserReading`）`[V]` ——
  否则未定义 `LZ_POLE_SOURCE_LASER` 时整个激光分支不参与编译，测试看不见它。
- **两个记录按钮语义分开** `[V]`（用户 2026-09-22 决定）：飞机位=「我在圆心
  正上方」；激光点=「我瞄的就是杆」。写同一槽位，后记录的覆盖先前的。

## 4. 失败的尝试 — 不要再试

- **`DjiFcSubscription_GetLatestValueOfTopic`** `[V]` —— SIGSEGV，栈在
  `DjiDataSubscriptionDds_v3_GetLastValueOfTopic` 内部（SDK 的 bug）。
  已排除读太快/传 NULL 回调/无数据/初始化时机。**一律走回调缓存。**
- **在 `ApplicationStart()` 之前调 `DjiFcSubscription_Init()`** `[V]` ——
  返回 `SUCCESS` 但随后 SIGSEGV。**返回成功不代表调用合法。**
- **在控件回调里调任何阻塞接口** `[V]`（2026-09-22）—— 进程闪退，
  日志表现为 `dji_msgq.c:227 semaphore wait timeout` +
  `dji_linker.c:309 send msg to queue error` 刷屏后死掉。
- **`exception` 白名单** `[X]` —— 见 §3。实测 `0` 也是有效读数。
- **零解判据写 `== 0.0`** `[X]` —— 退化值带浮点残差（`3e-7`），必须给邻域。
- **`(unsigned)rc` 打 PSDK 返回码** `[X]` —— `T_DjiReturnCode` 是 `uint64_t`，
  模块号在高 32 位，截断后含义完全不同。用 `%llX`。
- **`-DPSDK_ROOT=~/path`** `[X]` —— `~` 不被展开，用 `$HOME`。
  且改 `-D` 参数**必须重跑 cmake 配置**（`CMakeCache.txt` 记住旧值）。
- **"模拟器不实现航点启动"** `[X]` —— 已撤回。它建立在被截断的 `0x000000FF`
  上，且当时 KMZ 确实缺必需元素、从未通过内容校验。
- **交叉编译** `[V]` —— 产物要求 `GLIBC_2.34`，设备 2.31。
- **`tar --mtime='@0'`** `[V]` —— 导致 make 静默跳过编译。用设备时间 `-60s`。
- **不排除 `lz/build*` 就打 tar** `[V]` —— 本机 x86 产物混进 aarch64 构建。
- **`app.json` 缺 `description_jp/fr`** `[V]` —— `build_dpk.sh` 直接报 `KeyError`。
- **用 `/dev/tcp` 扫网段找设备** `[X]` —— 254 个假阳性。用 `ping` + `ip neigh`。

（以下为上一份 HANDOFF.md 前向搬运，标 `[?]`，本会话未重新验证：）

- **`DjiWidget_DeInit()` 不存在** `[X]` —— 全模块只有 `Init`。
- **Waypoint V2 用于 M4T** `[X]` —— 官方只支持 M300/M350。
- **兴趣点环绕（`DjiInterestPoint_*`）可控半径** `[X]` —— settings 里无 radius。
- **"KMZ 动作对第三方负载没意义"** `[X]` —— `gimbalRotate` 独立于相机。
- **在妙算3 应用管理面板里找控件** `[X]` —— 在 Pilot 2 相机视图左侧「PSDK」菜单。
- **只判激光 `exception` 不判 `distance`** `[X]` —— 见 §3。
- **方位角不能用绝对差比较** `[?]` —— 正北处返回 `359.999999998°`。用 `LZ_CHECK_ANGLE_NEAR`。
- **`LzGeo_NormalizeDeg` 区间 [0,360) 左闭右开** `[?]` —— 极小负数加 360 会因舍入得 `360.0`。

## 5. 已知坑

- **pi 直连 `git push fork` 会挂起** `[V]`（2026-09-22）—— 报
  `send disconnect: Broken pipe` 或 `Timeout`。必须走代理：
  ```bash
  GIT_SSH_COMMAND="ssh -o ProxyCommand='nc -X connect -x 127.0.0.1:7890 %h %p'" \
    git push fork feature/liangzhourenwu
  ```
- **注释里不能写 `*/`** `[V]` —— 在块注释里写 `widget_file/*/` 会提前闭合注释块。
- **`-1` 会满足 `<= 1`** `[V]` —— 负值哨兵与正数阈值共用变量时，判断顺序决定成败。
- **管道下 stdio 全缓冲** `[V]` —— 实时监视类程序每行 `fflush(stdout)`。
- **gdb 抓 PSDK 进程要先 `handle SIG32 nostop noprint pass`** `[V]`。
- **诊断程序必须在读之前就打印并 flush** `[V]` —— 崩溃会吞掉缓冲区。
- **`/blackbox/system/app_temp_files/` 是 root:root 0755** `[?]` —— 删不掉。
- **本 worktree 是稀疏检出** `[?]` —— `lz` 目录需先 `git sparse-checkout add lz`。
- **`cam` 模块刷屏 `data size = 66 > 49 is too large`** `[V]`（2026-09-22）——
  SDK 内部警告（相机状态包比它预期的大），**与闪退无关**，但说明
  M4T 与该 SDK 版本有真实不一致。未深究。

## 6. 下一步（有序）

1. **恢复设备连接**（Windows 侧重做 USB 网络共享附加）。
   判据：`ping 192.168.42.120` 通 + `ip neigh` 有 ARP 条目。
   **不要用 `/dev/tcp` 扫描。**
2. **跑通绕飞完整链路 —— 唯一还没验的一环**：
   - 到室外，等 GPS 3D Fix
   - 手动起飞、飞到杆附近
   - 按「记录飞机位」或「记录激光点」记下圆心
     （激光要打中目标；`cat data/pole.txt` 可核对）
   - 拨开关 ON → 观察：飞机是否先爬升到 80.9 m、再平飞到首航点、顺时针绕圈
   - 拨 OFF 可急停（但**停完悬停不返航**，会耗电）
3. 若成功 → 验绕行方向（顺时针，8 点只覆盖 7/8 圈 = 315°）、云台逐点指向、
   返航高度（`globalRTHHeight` = max(航线高度, 30)）
4. 若失败 → `error_code` 现在会被 `lz_sdk_log_watch` 捞进日志，
   连同「启动被拒：<飞行状态> RC=… 」一起看

## 7. 留给用户的开放问题

- **失败提示的措辞** —— 用户 2026-09-22 指出「启动被拒」那条诊断会被
  「绕飞结束：…请手动拨回」覆盖掉（浮窗单缓冲，两条消息相隔 1ms）。
  建议合并成一条、诊断在前、失败时不清理本地状态。**用户未确认是否要改。**
- **急停之后没有返航按钮** —— 现在拨 OFF 只是 STOP，飞机悬停耗电。
  用户提过「意外时避险」，我建议加一个独立的 `button` 做一键返航
  （`DjiFlightController_StartGoHome()`，已核实存在）。**未实现。**
- **`LzWidget_PostMessage` 内部直接调 `DjiWidgetFloatingWindow_ShowMessage`** ——
  理论上也是回调线程里的阻塞调用。绕飞开关这条路径一直正常，留作已知风险。
- **启动前 GPS 预检查** —— 现在要等 4 秒上传完才报「没有定位」，
  应在拨开关第一步就检查 GPS 并立即拒绝。
- **视觉层仍未接入** `[V]` —— `lz_vision_source.c` 只有声明、无实现、无调用者。
  激光可用后，视觉的职责可能降级为「确认瞄准点是不是杆」。**未定。**
- **杆的 WGS84 坐标最终用哪条路** —— 激光（现成、精度高）vs 视觉定位（未做）。
  注意红旗横向展开，激光打中旗面中心会偏离杆轴最多半个旗宽（1–1.5 m）。

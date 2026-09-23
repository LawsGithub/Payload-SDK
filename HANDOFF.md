# HANDOFF — 读全文再开始干活

生成时间: 2026-09-23T11:08:51+0800 · Git HEAD: 5ced88c
信任规则: [V] = 交接时已用命令验证；[?] = 仅记忆未复核，当线索对待；[X] = 已证伪，别用。

## 0. 复核（下一会话先做）

- 锚点: `feature/liangzhourenwu` @ `5ced88c`（2026-09-23 11:08）
- 漂移检查: `git rev-parse HEAD~1` 应 = `5ced88c`——HEAD 是本次 handoff 提交，
  其 parent 才是本快照记录的 SHA；变了说明快照可能过期
- **远端已同步** `[V]`（`fac1474..5ced88c` push 成功，走 HTTP 代理）。
  若 `git status` 显示领先，重试 `git push fork feature/liangzhourenwu` ——
  **必须走代理**，直连会挂起（见 §5）
- **设备当前不可达** `[V]`（11:0x 实测：WSL 侧无 `192.168.42.x` 网卡，
  `eth2`/`eth3` 均 DOWN，`ping` 100% 丢包，`ip neigh` 无 ARP 条目）。
  需在 **Windows 侧重做 USB 网络共享附加**
- 先读: `~/projects/liangzhourenwu/CLAUDE.md`（项目约定）
  + **[`lz/doc/ONDEVICE-CHECKLIST.md`](lz/doc/ONDEVICE-CHECKLIST.md)（本次新增的上机执行单）**

## 1. 当前目标

**让「操作员在 Pilot 2 拨开关 → 飞机绕国旗杆飞一圈」完整可用。**

完成定义 = 拨 ON 后飞机按设定半径/高度绕记录的点一周。
⚠️ **完成定义里没有"返航"** —— 收尾动作已是 `gotoFirstWaypoint`，
飞完停在**航线起始点**悬停，要返航得手动接管（见 §3）。

**当前卡点：只剩最后一环未跑 —— 绕飞完整链路。**

## 2. 已验证状态 — 工作实际停在哪

**上一会话（2026-09-22 晚 ~ 09-23 凌晨）在 HANDOFF 写完后继续干了 7 个提交，
本次会话把这 7 个提交的结论收敛进持久文档并补齐上机清单。**

### 本会话新验证的（都是命令实测）

- **PC 侧全绿** `[V]` —— 见下方测试输出
- **x64 侧 PSDK 编译通过** `[V]` —— `lz_app` / `lz_rangefinder_probe` /
  `lz_mission_probe` 三个产物都生成，我们自己代码 0 warning
- **反向验证：拆闭合变红** `[V]` —— 172 条断言失败、`lz_test_plan` 退出码 1、
  ctest 2 个目标红
- **反向验证：`LZ_CHECK_ANGLE_NEAR` 漏计确实能掩盖真实失败** `[V]` ——
  宏坏掉 + 9 条纯角度断言失败时，汇总打印"**0 项失败**"、ctest **100% passed**；
  把宏修好同一份用例立刻报 9 项失败、ctest 变红。**这是该 bug 唯一能演示的构造**
  （拆闭合那条路径用不上角度宏，掩盖不了）—— 详见 CLAUDE.md「反向验证」一节
- **`finishAction` 反向验证** `[V]` —— 改回 `goHome` → 6 条断言失败
- **CLAUDE.md 里三处过期陈述已更正** `[V]` —— 见 §3 第三条

### 上一会话验证过的（本次**未**重跑，标 `[?]`）

- **航点任务能真正启动** `[?]`（2026-09-22 室外首次成功，修完 wpml 必需元素后）
- **激光测距可用** `[?]` —— `28.1788176, 112.9210889`（4.4 m），
  与「飞机位置 + 距离」反算一致（4.1 m）；**记录飞机位** `[?]` `28.1788120, 112.9210473`
- **落盘** `[?]` —— `data/pole.txt`：`lon=... lat=... alt=... src=aircraft|laser`
- **控件回调触发** `[?]` —— 3 个控件的时代实测过；**新增的控件 5 从未上机**

### 本会话改了什么

1. **`lz/doc/ONDEVICE-CHECKLIST.md`（新增，171 行）** —— 现场操作单：
   控件与航线参数对照表、8 个待验证项的**判据与回退方案**、已知噪声、排查顺序。
2. **CLAUDE.md 三处过期陈述更正**（都是实测发现的）：总路径长（`107.151 m`
   是闭合前旧值，闭合后 `8 × 15.307 = 122.459 m`）；航点数上限 64 的**理由已失效**
   （转弯模式改成过点不停了）；控件表补上**控件 5**并标注"尚未上机"。
3. 删掉「仍待确认」里重复的两条（POI 与高级权限各出现两次）。

### 测试/build 输出（本次交接 run 的**真实输出与退出码**）

```
PC 侧（rm -rf build 后全新建）:
  cmake -S lz -B build                                   → CFG_RC=0
  cmake --build build -j4                                → BUILD_RC=0 / 0 error 0 warning
  ctest --test-dir build --output-on-failure             → CTEST_RC=0
                                                          100% tests passed, 0 failed out of 7
  ./build/lz_test_plan                                   → exit 0，1075 项检查，0 项失败
x64 侧:
  cmake -S lz -B lz/build-x64 -DLZ_BUILD_PSDK_APP=ON -DLZ_TARGET_ARCH=x86_64 \
        -DPSDK_ROOT=$HOME/projects/Payload-SDK            → X64_CFG_RC=0
  cmake --build lz/build-x64 -j4                          → X64_BUILD_RC=0 / 0 warning
  → bin/ 下 lz_app / lz_mission_probe / lz_rangefinder_probe 三产物齐全

反向验证（四格矩阵，逐条回退后立刻还原，`git status` 已确认干净）:
  基线                     → lz_test_plan exit=0，ctest 100% passed
  只拆闭合（i <= → i <）    → exit=1，1040 检查/172 失败；ctest 71% passed, 2 failed
  只让角度宏不计数          → exit=0，1075 检查/0 失败；ctest 100% passed（假象）
  宏坏 + 9 条纯角度断言失败 → exit=0，**0 项失败**；ctest **100% passed** ← 假绿
  上一行把宏修好            → exit=1，9 项失败；ctest 86% passed, 1 failed
```

### git 状态（事实快照，非待办）

```
feature/liangzhourenwu @ 5ced88c，工作区干净（仅 pt1.md 未跟踪，周报草稿，故意不入库），
本地与远端 fork/feature/liangzhourenwu 一致
5ced88c docs(claude): 更正三处过期陈述 + 补控件 5 与反向验证实测表
6095d2d docs(lz): 新增上机执行清单 —— 待验证项逐条带判据与回退方案
fac1474 feat(lz): 收尾动作改为 gotoFirstWaypoint
9de795d fix(test): LZ_CHECK_ANGLE_NEAR 漏计 lz_test_failures
7b21487 feat(lz): 严格闭合 + 走弧线（turnMode）
444f85f feat(lz): 航点数改为 Pilot 2 控件可填（index 5）
（更早见 `git log`）
```

## 3. 决策与理由

- **收尾动作 = `gotoFirstWaypoint`，不再返航** `[?]`（2026-09-23，用户要求）——
  「闭合」与「收尾行为」是**正交的两件事**：闭合管轨迹与收尾处机头朝向，
  收尾行为管飞完去哪儿。规范里的"航线起始点"是**第一个航点**（杆旁边），
  **不是起飞点**。后果：任务正常结束后飞机停在杆旁边悬停，不返航也不降落 ——
  与拨 OFF 急停一致，两条路径都要操作员手动接管。
  `globalRTHHeight` 仍是必需元素、取值不改（操作员手动返航/失控返航时生效）。
- **闭合 = 末尾补一个与首点坐标完全相同的收尾点** `[?]`（2026-09-22）——
  航线逐点执行，飞完最后一个"真实"方位点就按 `finishAction` 走了，
  回起点那段弧不在航线里。补上才真闭合，且顺手修掉一个朝向缺陷：
  `towardPOI` 下一个点的朝向作用于"飞向**下一个**航段"，
  最后那个点没有后继航段，收尾处会朝错方向。
- **走弧线而非正多边形：`turnMode` + `useStraightLine=1`** `[?]` ——
  航点只给离散采样点，两点间走直线还是曲线由转弯模式决定，
  不是靠挪航点位置。直线模式 8 点 20 m 半径边心距只有 18.48 m（近 7.6%）。
  截距由**真实段长反算**（最短段 45%），不接调用方给的值。
  **代价（本次发现的过期陈述）**：换模式后"上限 64"的原始理由失效了。
- **反向验证要按宏逐条做** `[V]`（2026-09-23 实测）—— 见 §2 的四格矩阵。
  用一条路径一次测完所有修复，会漏掉"某条断言路径根本没用上被测宏"的情形。

## 4. 失败的尝试 — 不要再试

- **`DjiFcSubscription_GetLatestValueOfTopic`** `[V]` —— SIGSEGV，栈在
  `DjiDataSubscriptionDds_v3_GetLastValueOfTopic` 内部（SDK 的 bug）。
  已排除读太快/传 NULL 回调/无数据/初始化时机。**一律走回调缓存。**
- **在 `ApplicationStart()` 之前调 `DjiFcSubscription_Init()`** `[V]` ——
  返回 `SUCCESS` 但随后 SIGSEGV。**返回成功不代表调用合法。**
- **在控件回调里调任何阻塞接口** `[V]`（2026-09-22）—— 进程闪退，
  日志表现为 `dji_msgq.c:227 semaphore wait timeout` +
  `dji_linker.c:309 send msg to queue error` 刷屏后死掉。
- **`exception` 白名单** `[X]` —— 实测 `0` 也是有效读数，白名单必然漏。
- **零解判据写 `== 0.0`** `[X]` —— 退化值带浮点残差（`3e-7`），必须给邻域。
- **`(unsigned)rc` 打 PSDK 返回码** `[X]` —— `T_DjiReturnCode` 是 `uint64_t`，
  模块号在高 32 位，截断后含义完全不同。用 `%llX`。
- **`-DPSDK_ROOT=~/path`** `[X]` —— `~` 不被展开，用 `$HOME`。
  且改 `-D` 参数**必须重跑 cmake 配置**（`CMakeCache.txt` 记住旧值）。
- **"模拟器不实现航点启动"** `[X]` —— 已撤回。它建立在被截断的 `0x000000FF`
  上，且当时 KMZ 确实缺必需元素、从未通过内容校验。
- **把日志注释掉的 sed 写法** `[V]`（2026-09-23 本次踩到）—— 用 `//` 注释
  宏体里的一行会让宏展开成语法垃圾（`error: expected 'while' before 'do'`）。
  反向验证要停掉某个计数，改 `(void)0;` 而不是注释掉。
- **交叉编译** `[V]` —— 产物要求 `GLIBC_2.34`，设备 2.31。
- **`tar --mtime='@0'`** `[V]` —— 导致 make 静默跳过编译。用 `-60s`。
- **不排除 `lz/build*` 就打 tar** `[V]` —— 本机 x86 产物混进 aarch64 构建。
- **`app.json` 缺 `description_jp/fr`** `[V]` —— `build_dpk.sh` 报 `KeyError`。
- **用 `/dev/tcp` 扫网段找设备** `[X]` —— 254 个假阳性。用 `ping` + `ip neigh`。

（以下为前向搬运，标 `[?]`，近两轮会话未重新验证：）

- **`DjiWidget_DeInit()` 不存在** `[X]` —— 全模块只有 `Init`。
- **Waypoint V2 用于 M4T** `[X]` —— 官方只支持 M300/M350。
- **兴趣点环绕（`DjiInterestPoint_*`）可控半径** `[X]` —— settings 里无 radius。
- **"KMZ 动作对第三方负载没意义"** `[X]` —— `gimbalRotate` 独立于相机。
- **在妙算3 应用管理面板里找控件** `[X]` —— 在 Pilot 2 相机视图左侧「PSDK」菜单。
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
  SDK 内部警告，**与闪退无关**，别去修。详见上机清单 §4。

## 6. 下一步（有序）

**现场执行单在 [`lz/doc/ONDEVICE-CHECKLIST.md`](lz/doc/ONDEVICE-CHECKLIST.md)，
每一步的判据与回退方案都在那里，别凭这份摘要操作。**

1. **恢复设备连接**（Windows 侧重做 USB 网络共享附加）。
   判据：`ping 192.168.42.120` 通 + `ip neigh` 有 ARP 条目。
   **不要用 `/dev/tcp` 扫描。**
2. **设备上编译 + 装包**（命令见清单 §0）。装完在日志里找
   `Update dji sdk policy file successfully` 确认起来了。
3. **先花 5 分钟验控件 5**（航点数输入框，**唯一从未上机的新控件**）：
   填 100 必须回 `已按 64 使用`。这是最便宜的先验项，不通就先修它。
4. **跑通绕飞完整链路 —— 唯一还没验的一环**：室外等 GPS 3D Fix → 手动起飞
   → 记录圆心（`cat data/pole.txt` 核对）→ 拨开关 ON → 观察是否按
   **先爬升 → 平飞到首航点 → 顺时针绕圈**。
5. 若成功 → 按清单 §3.3–§3.7 逐项验（`towardPOI` 侧飞、是否真走弧线、
   `gotoFirstWaypoint` 收尾、云台 yaw、机型枚举）。
   **§3.4 的"实际离杆距离是否回到设定半径附近"是区分"改了没生效"与
   "生效了"的唯一硬判据。**
6. 若失败 → 日志里看 `启动被拒：<飞行状态> RC=<档位> GPS状态=<x> 卫星=<n>`
   + `error_code` 原始值（`%llX`）。

## 7. 留给用户的开放问题

- **失败提示的措辞** —— 用户 2026-09-22 指出「启动被拒」那条诊断会被
  「绕飞结束：…请手动拨回」覆盖掉（浮窗单缓冲，两条消息相隔 1ms）。
  建议合并成一条、诊断在前、失败时不清理本地状态。**用户未确认是否要改。**
- **急停之后没有返航按钮** —— 现在拨 OFF 只是 STOP，飞机悬停耗电；
  加上 `gotoFirstWaypoint` 后**正常飞完也停在杆旁悬停不返航**。
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

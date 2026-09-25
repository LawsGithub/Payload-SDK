# HANDOFF — 读全文再开始干活

生成时间: 2026-09-25T12:10:00+0800 · Git HEAD: __HEAD__
信任规则: [V] = 交接时已用命令验证；[?] = 仅记忆未复核，当线索对待；[X] = 已证伪，别用。

## 0. 复核（下一会话先做）

- 锚点: `feature/liangzhourenwu` @ `__HEAD__`（2026-09-25 12:10）
- 漂移检查: `git rev-parse HEAD~1` 应 = `__HEAD__`——HEAD 是本次 handoff 提交，
  其 parent 才是本快照记录的 SHA；变了说明快照可能过期
- **本段新增的切片尚未推送**：`c72f0ad`。推送要**走代理**，
  且**别直连**（直连 `git push` 会挂起，见 §5）
- **远端已同步** `[V]`（`6da77dc..cd28fde` push 成功，走 HTTP 代理）。
  若 `git status` 显示领先：重试 `git push fork feature/liangzhourenwu`
  —— **必须走代理**，直连会挂起（见 §5）
- **设备可达** `[V]`（本次实测：`ping 192.168.42.120` 通、
  `eth4` UP `192.168.42.1/24`、`ip neigh` 有 ARP 条目）。上轮交接记的
  "设备不可达"已不成立
- 先读: `~/projects/liangzhourenwu/CLAUDE.md`（项目约定，本次大改）
  + [`lz/doc/ONDEVICE-CHECKLIST.md`](lz/doc/ONDEVICE-CHECKLIST.md)（上机执行单）

## 1. 当前目标

**让「操作员在 Pilot 2 拨开关 → 飞机绕国旗杆飞一圈」完整可用。**

完成定义 = 拨 ON 后飞机按设定半径/高度绕记录的点一周。
⚠️ **完成定义里没有"返航"** —— 收尾动作已是 `gotoFirstWaypoint`，
飞完停在**航线起始点**悬停，要返航得手动接管。

**当前卡点：绕飞完整链路仍未跑过。** 本轮（2026-09-25 第二段）做的是
**视觉调俯仰的方案 + 取图接线骨架**，不是上机。

### 本轮（第二段）新增 `[V]`

用户定下**方向**：**用视觉取图识别来调云台俯仰**，替代"按几何公式算"。
方案见 **[`lz/doc/VISION-GIMBAL-PITCH.md`](lz/doc/VISION-GIMBAL-PITCH.md)**
（535 行，含硬约束、分步实施、待定项）。

核心认识：**几何公式里的 `-h/2` 就是「瞄杆中点」，与视觉瞄准是同一个物理点**；
绕飞时**高度定 + 半径定 ⇒ 俯仰恒定**（用户指出）⇒ **「测一次」与「持续测」
几何等价** ⇒ 飞行前一次性测定是**充分的**，且顺带避开「云台手动控制与
航线里 `gimbalRotate` 抢同一个云台」这个未实测的冲突。

已交付：**方案文档** + `lz/app/lz_vision_source.{h,c}`（取图骨架）+
`lz/app/lz_liveview_probe.c`（上机探针，`scan` 模式逐个试 source 并存 PPM）。
已提交 `c72f0ad`。

配套的既有结论更正见下方 §4「PSDK 不能转自带云台」。

## 2. 已验证状态 — 工作实际停在哪

### 本轮完成的三件算法层工作 `[V]`（全部桌面可测）

1. **云台俯仰改为几何反算** —— `LzPlan_ComputeGimbalPitchDeg()` +
   `LzOrbitProfile.autoGimbalPitch`（`lz_mission.c` 已打开）。
   实测：写死的 −15° 在默认滑杆值（半径 12.5 m、高度 80.9 m、杆高 15 m）下
   正解是 **−80.34°**，差 **65°**。椭球高差只在 `LzPlan_BuildOrbit` 一处补。
2. **视觉层实现**（`hsv` 后端）—— 红色连通域取旗面 + 竖线对比度滤波定杆列。
   实测 6 张真实照片：杆列位置 0–2 px 稳定，旗面积第一名/第二名比 7.9x~103x。
3. **视觉回归测试**（`lz_test_vision`，8 个用例）+ 6 张照片数据入库。

### 本轮最重要的发现 `[V]` —— 原设计「取旗面 bbox 中点当杆位」是错的

实测 6 张俯拍照片，旗 bbox 中心与真实杆列相差 **−3.7% ~ +4.1% 画面宽**，
且**符号随风向翻转**（风把旗吹向一侧时杆在另一侧）。
⇒ **不是常数偏差，标定不掉**，半径越大放得越大。
详见 CLAUDE.md「检测的是**杆**，不是旗面中心」一节（含逐张偏差表）。

### 测试/build 输出（本次交接 run 的真实输出与退出码）

```
hsv 后端（cmake -S lz -B build -DLZ_VISION_BACKEND=hsv）:
  cmake --build build -j4        → 0 error 0 warning
  ctest --test-dir build         → CTEST_RC=0
                                   100% tests passed, 0 failed out of 8
  逐目标:
    lz_test_geo       exit=0   34 项检查，0 项失败
    lz_test_plan      exit=0 1133 项检查，0 项失败
    lz_test_bridge    exit=0   12 项检查，0 项失败
    lz_test_kmz       exit=0   22 项检查，0 项失败
    lz_test_wpml      exit=0  139 项检查，0 项失败
    lz_test_validate  exit=0  173 项检查，0 项失败
    lz_test_pole      exit=0   53 项检查，0 项失败
    lz_test_vision    exit=0  121 项检查，0 项失败

stub 后端（默认，无 -DLZ_VISION_BACKEND）:
  → 0 warning；ctest 100% passed, 0 failed out of 7（视觉测试不注册，见下）

本轮（第二段）重跑，含取图骨架与探针:
  stub 后端: ctest 100% passed, 0 failed out of 7   （7/7）
  hsv  后端: ctest 100% passed, 0 failed out of 8   （8/8，含 lz_test_vision）
  x64 PSDK 侧: RC=0，**我们代码 0 warning**（PSDK 官方 hal/osal 的 warning 未动）
    产物 4 个：lz_app / lz_liveview_probe(新) / lz_mission_probe / lz_rangefinder_probe

x64 PSDK 侧:
  cmake -S lz -B build-x64 -DLZ_BUILD_PSDK_APP=ON -DLZ_TARGET_ARCH=x86_64 \
        -DPSDK_ROOT=$HOME/projects/Payload-SDK  → RC=0，我们代码 0 warning
  → bin/ 下 lz_app / lz_mission_probe / lz_rangefinder_probe 三产物齐全

反向验证（10 条，逐条回退后立刻还原，git status 已确认干净）:
  杆位退回「旗 bbox 中点」    → vision exit=1，6 项失败
  HSV minSaturation 100→40    → exit=1，11 项失败
  投票窗口改回整幅图          → exit=1，3 项失败
  置信度只看旗面（不看杆）     → exit=1，1 项失败
  去掉「红块过大」惩罚         → exit=1，1 项失败
  连通域不做 union            → exit=1，1 项失败
  俯仰去掉椭球高差补偿         → plan exit=1，5 项失败
  俯仰 atan2 参数写反          → exit=1，12 项失败
  autoGimbalPitch 关掉（回写死常数）→ exit=1，14 项失败
```

### git 状态（事实快照，非待办）

```
feature/liangzhourenwu @ 71789df，工作区干净（仅 pt1.md 未跟踪，周报草稿，故意不入库）
本轮（第二段）新增切片（**尚未推送**）：
  c72f0ad feat(lz): 视觉调俯仰方案 + 取图接线骨架与探针
（以下为上一轮，已推送：）
  cd28fde feat(lz): 新增视觉黄金值核对图工具
  7ac1f58 feat(lz): 视觉层实现 —— 杆列检测 + 6 张真实照片的回归测试
  86599b0 feat(lz): 云台俯仰改为几何反算 —— 写死的 -15° 在默认剖面下差 65°
  71789df docs: 收敛视觉层与云台俯仰的结论（含三处过期陈述更正）
（更早见 git log）
```

## 3. 决策与理由

- **云台俯仰必须几何反算** `[V]`（2026-09-25，用户要求）——
  `俯仰 = -atan2(飞机相对目标高度 - 目标高/2, 半径)`。用户的直觉对：
  高度定+半径定 ⇒ 俯角恒定，飞行中不用变；但**值**随半径高度变化，
  写死必然错。瞄**目标中点**（半高），与"机头瞄杆心"在水平方向的居中对称。
  否决方案：钳到相机限位 `[-90, 30]` —— 会把"飞机低于目标时该往上看、
  而相机上仰极限只有 +30°，物理上做不到"伪装成做得到。改为照实返回，
  由 `LzPlan_Validate` 用 `LZ_ERR_RANGE` 拒绝。
- **检测杆，不检测旗面中心** `[V]`（2026-09-25 实测）—— 见 §2 的发现。
  否决方案：用旗 bbox 中点 —— 随风向变号，标定不掉。
- **不加形状判据（矩形度之类）** `[V]` —— 那需要再定一批阈值，而阈值在
  阴天/逆光下该不该跟着变没有依据。**少一个自由度就少一处现场失配的地方。**
- **测试数据入库（1.6 MB）** `[V]` —— 价值在于"不需要现场照片也能复现
  检测行为"。期望值来自全图检测+人工目视核对再**纯平移**，
  **不是算法自己的输出**（那样是循环论证）。
- **生成工具纯 JS（只需 jpeg-js）** `[V]` —— **故意不依赖 PIL/OpenCV**：
  生成测试数据不该给工程引入新的构建依赖。本机无 PIL/pip，实测靠
  `npm i jpeg-js` 走通。

## 4. 失败的尝试 — 不要再试

- **`DjiFcSubscription_GetLatestValueOfTopic`** `[V]` —— SIGSEGV，栈在
  `DjiDataSubscriptionDds_v3_GetLastValueOfTopic` 内部（SDK 的 bug）。
  一律走回调缓存。
- **在 `ApplicationStart()` 之前调 `DjiFcSubscription_Init()`** `[V]` ——
  返回 SUCCESS 但随后 SIGSEGV。**返回成功不代表调用合法。**
- **在控件回调里调任何阻塞接口** `[V]` —— 进程闪退
  （`semaphore wait timeout` + `send msg to queue error` 刷屏后死掉）。
- **「PSDK 不能转自带云台」** `[X]`（2026-09-25 更正）—— **这条不成立**，
  它把 `dji_gimbal.h`（服务**第三方**云台，确实只有 `Init`/`DeInit`/`RegCommonHandler`）
  与 `dji_gimbal_manager.h`（服务**挂载在飞机上的云台**，含自带）混为一谈了。
  后者有 **9 个 setter**，其中 `DjiGimbalManager_Rotate()` 就是转到指定
  pitch/roll/yaw 角，单位 deg。四条支持证据（妙算3 高级功能清单含「云台管理」、
  能力矩阵 Gimbal Management=advanced、issue #563 有人在 M4T+妙算上用它控
  pitch、issue #555 能设模式）见 [`lz/doc/VISION-GIMBAL-PITCH.md`](lz/doc/VISION-GIMBAL-PITCH.md) §12。
  ⚠️ #555 说的限制是 **yaw**（与既有结论一致，不冲突）；#563 说 `Rotate`
  的**实速只有下发的 10–20%** ⇒ 用它必须**读回确认到位**。
- **期望 PSDK 提供「追踪拍摄」** `[X]`（2026-09-25 全库实查）——
  grep 39 个头文件 / 374 个导出函数，`track`/`follow`/`detect`/`recogni`/
  `aim`/`lock` 的函数名**零命中**；wpml 三份规范里也零命中。
  固件有 Smart Track（`dji_hms_info_table.h` 里 **75 条**相关文案），
  但它是**固件 + Pilot 2** 的功能，PSDK 无接口。别再找。
  ⚠️ `dji_liveview.h` 的 `DJI_LIVEVIEW_OBJ_STATE_TRACKED`（注释
  `steady state following`）**是上报给 Pilot 的字段**，方向相反 ——
  唯一的 `SendAiMetaToPilot` 是「**我**识别，推框给 Pilot 画」。
- **`exception` 白名单** `[X]` —— 实测 `0` 也是有效读数，白名单必然漏。
- **零解判据写 `== 0.0`** `[X]` —— 退化值带浮点残差，必须给邻域。
  新踩一次：`LzGeo_IsValid` 取反**也拦不住**零解（`(0,0)` 在范围内合法），
  要用 `LzGeo_IsNullSolution`。
- **`(unsigned)rc` 打 PSDK 返回码** `[X]` —— `uint64_t`，模块号在高 32 位。
  用 `%llX`。
- **`-DPSDK_ROOT=~/path`** `[X]` —— `~` 不被展开，用 `$HOME`。
  且改 `-D` 参数**必须重跑 cmake 配置**。
- **"模拟器不实现航点启动"** `[X]` —— 已撤回（证据不足）。
- **给宏体里的一行加 `//` 注释** `[V]` —— 宏展开成语法垃圾。反向验证要停掉
  某个计数，改 `(void)0;`。
- **交叉编译** `[V]` —— 产物要求 `GLIBC_2.34`，设备 2.31。
- **`tar --mtime='@0'`** `[V]` —— 导致 make 静默跳过编译。用 `-60s`。
- **不排除 `lz/build*` 就打 tar** `[V]` —— x86 产物混进 aarch64 构建。
- **`app.json` 缺 `description_jp/fr`** `[V]` —— `build_dpk.sh` 报 `KeyError`。
- **用 `/dev/tcp` 扫网段找设备** `[X]` —— 254 个假阳性。用 `ping` + `ip neigh`。
- **在 ctest 里给测试传相对路径的数据目录** `[V]` —— ctest 在构建目录跑，
  数据在源树。要用 CMake 把绝对路径塞进环境变量（`LZ_VISION_TESTDATA`），
  且**先试 argv、再试环境变量、最后回落相对路径**。

（以下为前向搬运，标 `[?]`，本轮未重新验证：）

- **`DjiWidget_DeInit()` 不存在** `[X]` —— 全模块只有 `Init`。
- **Waypoint V2 用于 M4T** `[X]` —— 官方只支持 M300/M350。
- **兴趣点环绕（`DjiInterestPoint_*`）可控半径** `[X]` —— settings 里无 radius。
- **"KMZ 动作对第三方负载没意义"** `[X]` —— `gimbalRotate` 独立于相机。
- **在妙算3 应用管理面板里找控件** `[X]` —— 在 Pilot 2 相机视图左侧「PSDK」菜单。
- **方位角不能用绝对差比较** `[?]` —— 正北处返回 `359.999999998°`。
  用 `LZ_CHECK_ANGLE_NEAR`。
- **`LzGeo_NormalizeDeg` 区间 [0,360) 左闭右开** `[?]` —— 极小负数加 360
  会因舍入得 `360.0`。

## 5. 已知坑

- **pi 直连 `git push fork` 会挂起** `[V]` —— 必须走代理：
  ```bash
  GIT_SSH_COMMAND="ssh -o ProxyCommand='nc -X connect -x 127.0.0.1:7890 %h %p'" \
    git push fork feature/liangzhourenwu
  ```
- **`hsv` 后端的视觉测试只在 hsv 下注册** `[V]` —— CMake 里按
  `LZ_VISION_BACKEND` 条件注册。默认 stub 时 ctest 只有 7 个目标，
  **不是**测试丢了。
- **先跑 cmake 再跑 ctest，别在仓库根跑** `[V]` —— 本次踩到：
  在仓库根跑 `cmake -S . -B build` 会建出一个没有测试的空项目，
  `ctest` 报 "No tests were found!!!"。构建目录是 `lz/build`。
- **加"人造图形"测试用例时，必须先验证"拆掉被测逻辑它会红"** `[V]` ——
  本轮三条用例连续踩同一个坑（画布太小被置信度拦掉 / union 靠左邻传播
  侥幸正确）。**没变红的反向验证等于没测。**
- **注释里不能写 `*/`** `[V]` —— 块注释里写 `widget_file/*/` 会提前闭合。
- **`-1` 会满足 `<= 1`** `[V]` —— 负值哨兵与正数阈值共用变量时判断顺序决定成败。
- **管道下 stdio 全缓冲** `[V]` —— 实时监视类程序每行 `fflush(stdout)`。
- **gdb 抓 PSDK 进程要先 `handle SIG32 nostop noprint pass`** `[V]`。
- **诊断程序必须在读之前就打印并 flush** `[V]` —— 崩溃会吞掉缓冲区。
- **`/blackbox/system/app_temp_files/` 是 root:root 0755** `[?]` —— 删不掉。
- **本 worktree 是稀疏检出** `[?]` —— `lz` 目录需先 `git sparse-checkout add lz`。
- **`cam` 模块刷屏 `data size = 66 > 49 is too large`** `[V]` ——
  SDK 内部警告，**与闪退无关**，别去修。

## 6. 下一步（有序）

**⚠️ 顺序在本轮（第二段）变了。** 用户 2026-09-25 明确：

- 「先问用户要红旗实测杆高」**不再是要做的事** —— 用户要的是**用视觉调俯仰**，
  杆高只需作为公式初值；测定会修正它。
- 「设备编译不着急」；控件 5、绕飞链路「不着急」。

⇒ **本轮之后的主线是视觉调俯仰**，方案在
[`lz/doc/VISION-GIMBAL-PITCH.md`](lz/doc/VISION-GIMBAL-PITCH.md) §7 的步骤 2–6，
按"能桌面测的先做"排：

1. **步骤 2（纯函数，桌面可测）**：`LzVision_VerticalFovDeg(w, h, diagFovDeg)`。
   ⚠️ 必须防"把 82° 当水平 FOV"这个错（差 11°），用**反向验证**守它。
2. **步骤 3（桌面可测）**：把 `lz_vision.c:610-611` 丢弃的
   `poleTop/poleBottom` 接上（那是 **`LzTarget` 契约的改动**，两侧同改），
   6 张照片的黄金值补上杆的上下端（**人工核对后再平移**，不能取算法输出）。
3. **步骤 4（纯函数，桌面可测）**：像素偏差 → 角度，用 `atan` 不用线性近似。
4. **步骤 5（只能上机）**：云台设 + 读回。**先只做"设一个角、读回来核对"** ——
   issue #555/#563 说明"下发了 ≠ 执行了"，这一步本身就是要验的。
5. **步骤 6（只能上机）**：接进流程（方案 A：操作员手动飞到观测位按按钮）。
6. **上机时顺手做**：清单 §3.9 的取图探针 `./lz_liveview_probe scan`
   —— 它答的是 §7 步骤 2 的**分母**（哪个镜头 / 通道顺序 / 分辨率），
   没有它，步骤 2 的 FOV 值就还是猜的。

**以下为绕飞主线，本轮未推进，仍然有效：**
2. **设备上编译 + 装包**（清单 §0）。装完在日志里找
   `Update dji sdk policy file successfully` 确认起来了。装包需飞机通电并连接。
3. **先花 5 分钟验控件 5**（航点数输入框，**唯一从未上机的新控件**）：
   填 100 必须回 `已按 64 使用`。最便宜的先验项。
4. **跑通绕飞完整链路 —— 唯一还没验的一环**：室外等 GPS 3D Fix → 手动起飞
   → 记录圆心（`cat data/pole.txt` 核对）→ 拨开关 ON → 观察是否按
   **先爬升 → 平飞到首航点 → 顺时针绕圈**，且**杆大致停在画面中央**
   （后者是本轮俯仰改动的新判据，见清单 §2 的对照表）。
5. 若成功 → 按清单 §3.3–§3.7 逐项验（`towardPOI` 侧飞、是否真走弧线、
   `gotoFirstWaypoint` 收尾、云台 yaw、机型枚举）。
   **§3.4 的"实际离杆距离是否回到设定半径附近"是区分"改了没生效"与
   "生效了"的唯一硬判据。**
6. 若失败 → 日志里看 `启动被拒：<飞行状态> RC=<档位> GPS状态=<x> 卫星=<n>`
   + `error_code` 原始值（`%llX`）。

**本轮（第二段）刻意没做的**：

- **主链路仍未接入取图** —— 但**理由变了**，不再是"取图那一环空着"：
  `lz_vision_source.{h,c}` 本轮已实现并**编进了 `lz_app` 的源列表**
  （`lz/CMakeLists.txt`），只是 `main.c` / `lz_mission.c` 里**没有调用点**。
  不接是刻意的：测定（步骤 4–6）还没做，接了也没有东西用它，
  只会平白增加启动路径的风险。
- **步骤 2–6 全部**（见方案文档 §7）：垂直 FOV 纯函数、杆端点、
  像素→角度、云台设+读回、接进流程。
- **控件 6**（「视觉标定俯仰」按钮）。

⇒ 这三条是**下一段的全部内容**，方案文档 §7 有逐步的桌面/上机分工。

## 7. 留给用户的开放问题

- ~~红旗顶的实际高度~~ —— **用户 2026-09-25 已定：不用要了**，
  改为用视觉测俯仰（`VISION-GIMBAL-PITCH.md`）。写死的 15 m 只作为公式初值。
- ~~视觉接进主链路的目标形态~~ —— **用户 2026-09-25 已定**：用视觉**取图识别**
  来调**俯仰角**，方案见 `VISION-GIMBAL-PITCH.md`。
  ⚠️ 旧记录里"PSDK 不能转自带云台"**已证伪**（把 `dji_gimbal.h` 与
  `dji_gimbal_manager.h` 混为一谈了）—— 见 §4 与方案文档 §12。
- **失败提示的措辞** —— 用户 2026-09-22 指出「启动被拒」那条诊断会被
  「绕飞结束：…请手动拨回」覆盖（浮窗单缓冲，两条相隔 1ms）。**未确认是否要改。**
- **急停之后没有返航按钮** —— 拨 OFF 只是 STOP；加上 `gotoFirstWaypoint` 后
  正常飞完也停在杆旁悬停不返航。建议加独立 `button` 做一键返航
  （`DjiFlightController_StartGoHome()`，已核实存在）。**未实现。**
- **启动前 GPS 预检查** —— 现在要等 4 秒上传完才报「没有定位」，
  应在拨开关第一步就检查。
- **半径上限 20 m** —— 用户提过"后期飞行半径会变大"。改它要先确认新的
  场地约束；用户 2026-09-24 说"在红旗的高度及以上，20 米以内没有建筑物"
  （**竖直方向**的约束，不能直接推出水平方向）。
- **杆的 WGS84 坐标最终用哪条路** —— 激光（现成、精度高）vs 视觉定位（未做）。

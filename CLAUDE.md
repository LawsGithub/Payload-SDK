# CLAUDE.md — liangzhourenwu（视觉航点规划）

本文件是**项目专属**约束。仓库级规则（设备、硬规则、worktree 布局）见上一级
`CLAUDE.md`（提交在 `master` 上，每个分支都继承）——**先读那一份**。
本机专属约定（设备连接、ssh 凭据）在 `~/projects/CLAUDE.md`。

工作区：`~/projects/liangzhourenwu/`（分支 `feature/liangzhourenwu`），
代码在子目录 `lz/`。

## Cross-session handoff

- On session start: read [HANDOFF.md](HANDOFF.md) fully, then summarize the previous session's goal, current state, and next step before proceeding.
- 信任标记：`[V]` = 交接时已用命令验证；`[?]` = 仅记忆未复核，当线索对待；`[X]` = 已证伪，别用。
- 漂移检查：`git rev-parse HEAD~1` 应等于 HANDOFF.md 记录的 SHA——HEAD 是本次 handoff 提交，其 parent 才是快照的锚点。

## 项目目标

**识别国旗杆，对杆做绕飞观测。** 全流程机载完成，操作员用 Pilot 2 控件控制：

```text
取杆位（激光测距 | 固定坐标）→ 算绕飞圆周 → 生成 KMZ → 上传航点（V3）→ 执行
                    ↑
        操作员在 Pilot 2 上拨开关 / 调半径高度
```

场地约束：杆周围 20 m 内无建筑物、水平高度一致 → 可贴飞，碰撞风险低。

## 分层（依赖严格单向）

```text
lz_core      纯算法，零外部依赖（不依赖 PSDK / 任何第三方）  ← 永远可编译可测试
lz_vision    视觉，两个互斥后端（stub 占位 | hsv 真算法）     ← 也是零依赖
lz_app       机载应用，依赖 PSDK                            ← -DLZ_BUILD_PSDK_APP=ON
  ├─ platform/      平台层注册（移植自官方样例）
  ├─ lz_pole_source 绕飞圆心从哪来
  ├─ lz_widget      Pilot 2 控件（操作员入口）
  ─ lz_mission     作业状态机（**所有决策集中在此**）
```

**这条分界线是刻意的**：规划与安全校验出错代价最大（撞杆、漏拍），必须能在
桌面上不接飞机就反复验证。PSDK 很重，不该成为跑测试的前提。
视觉单独成层是因为它**改动频繁**，不该拖着规划层的测试一起翻车。
依赖方向单向：`lz_vision → lz_core`（用它的 LzTarget/几何），反向没有。

**视觉刻意不用 OpenCV** `[V]`（2026-09-18 决策，2026-09-19 核实设备后维持）：
国旗是高饱和红色块，HSV 双区间阈值足够。设备上**确实装了** OpenCV 4.2
（含 dnn），但结论不变 —— 纯 C 版本能在桌面直接跑测试、零构建依赖、
**失败可解释**（阴天→调 `minSaturation`），而模型失效是不可预测的自信错误。
**有工具不等于该用工具。**

## PC 侧自检（秒级，零依赖）

```bash
cd lz
cmake -S . -B build && cmake --build build -j4
ctest --test-dir build --output-on-failure
./build/lz_plan_demo
```

**还能在 WSL 上编译并运行 PSDK 侧代码** `[V]`（Payload-SDK 自带 x86_64 静态库）：

```bash
cmake -S lz -B build-x64 -DLZ_BUILD_PSDK_APP=ON -DLZ_TARGET_ARCH=x86_64 \
      -DPSDK_ROOT=$HOME/projects/Payload-SDK
cmake --build build-x64 -j4
```

⚠️ **必须用 `$HOME` 不能用 `~`** `[V]`（2026-09-21 实测）——
`-DVAR=~/path` 里的 `~` **不会被 shell 展开**（波浪号展开只认词首与 `=`/`:` 之后，
而 `-DPSDK_ROOT=~/...` 整个是一个词）。CMake 会把它当字面目录名，
报的是 `找不到 PSDK 静态库: ~/projects/...` —— 错误信息指向"库不存在"，
而真实原因是输入格式。
**注意 `-DPSDK_ROOT=...` 改了必须重跑 cmake 配置**：只 `--build` 不会重新生成
`dji_sdk_app_info.h`，而且新的文件列表也不会被采纳。

产物三个：`lz_app` / `lz_rangefinder_probe` / `lz_mission_probe`。
x86 侧只能验证**编译与链接**（设备没有 `/dev/usb-ffs/bulk*`，跑起来会在
`hal_usb_bulk.c` 报 Bad file descriptor，这是预期的）。
**不替代设备上的 aarch64 编译。**

`lz_test_plan` 会打印 SKIP 块 —— 那些是绕飞几何的**规格说明**，等你实现
`src/lz_plan.c` 的 `TODO(human)` 后把该文件的 `LZ_TODO_PENDING` 改成 `0` 启用。
**它们比注释可靠**：尤其是"方位角递增对应顺时针还是逆时针"这类
"符号反了但看起来对"的错误，肉眼极难看出，只有测试里的方位角断言能抓住。

## 关键接缝：LzTarget

`include/lz_target.h` 的 `LzTarget` 是视觉与规划之间的**唯一契约**，
**不含任何 OpenCV 或 PSDK 类型**。这样两件事同时成立：

1. 换视觉算法（模板匹配 → 深度学习）不必动规划器一行
2. 规划器能用**手写死的**目标数组跑回归测试，不必有图

改动这个结构体等于改动两层的接口，**要同时改两侧**。

## 绕飞：已定路线 —— Waypoint V3（自建 KMZ）+ towardPOI 机头对准杆心

**机型是 M4T + 妙算3，这直接推翻了最初的选择。** 官方文档原文（`[V]` 2026-09-18）：

| 功能 | 支持机型 |
|---|---|
| Waypoint 2.0 | "currently only supports **Matrice 300 RTK and Matrice 350 RTK**" |
| Waypoint 3.0 | "supports Matrice 30 Series, Mavic 3 Enterprise Series, Matrice 3D/3TD, **and subsequent models**" |
| POI 兴趣点环绕 | "supports the **Mavic 3 Enterprise series and subsequent models**" |

M4T 属 M4 系列（`DJI_AIRCRAFT_TYPE_M4T = 99`），即"后续机型"一档。
**所以 V2 排除（它只给 M300/M350），改用 V3。** 佐证：官方 V2 样例源码写着
`"Waypoint V2 sample only support M300 RTK"`，而 **V3 样例没有任何机型门禁** `[V]`。

### 反向验证为什么变红：三个独立原因 `[V]`（2026-09-23）

拿掉闭合（`i <=` 改回 `i <`）会红，**不是一条断言在喊，是三件事同时塌**：

| 失败断言数 | 用例 | 直接原因 |
|---|---|---|
| 112 | 严格闭合：末点与首点坐标完全相同 | `route.count` 少 1；末点变成了 270° 那个方位点，不再是 360°≡0° |
| 2 | 相邻航点间距是弦长，不是弧长 | 收尾段不存在（`closing` 算的是 p_{n-2}→p_{n-1} 的真弦，≈15 m）；总长退回 (n−1) 段弦 |
| 1 | 航点数与半径正确 | `count == waypointCount + 1` 不成立 |
| 1 | 相邻方位角均匀 | 少了那个 +360° 的收尾方位，步进序列不完整 |

112 条来自最后那个用例把 **3–16 点 × 顺逆时针**全跑了一遍，每个组合 ~7 条。

⚠️ **`LZ_CHECK_ANGLE_NEAR` 曾漏计失败数** `[V]`（2026-09-23 发现并修复）：
该宏只递增了 `lz_test_checks`，没有递增 `lz_test_failures`，而
`LZ_TEST_SUMMARY()` 是靠 failures 判成败的 —— 于是**该宏的失败不计入退出码，
ctest 报绿**。同一份反向验证因此被报成"87 条失败"（实为 116 条），
差的 29 条正是这个宏的。

教训与 `LZ_TEST_SUMMARY` 里那段"检查项数为 0 也算失败"同源：
**测试框架自身的计数也必须对称**，否则它会把真实回归伪装成通过。
加断言宏时先看三个宏的 checks/failures 是否都成对。

### 严格闭合：末尾补一个与首点重合的收尾点 `[V]`（2026-09-22）

用户明确要求"补一个与首点重合的收尾点"。改法是在生成循环里多跑一次：

```c
for (int i = 0; i <= profile->waypointCount; ++i)   // 注意 <=
    stationBearing = startBearing + dir * stepDeg * i
```

第 `waypointCount` 次的方位角比首点多走整 360°，`LzGeo_NormalizeDeg`
缩回后与首点**同坐标**（实测收尾距离 `0.00e+00 m`）。

**为什么收尾点必须是独立的一个航点**：航线逐点执行，飞完最后一个"真实的"
方位点就按 `finishAction=goHome` 走了 —— 回起点那段弧**不在航线里**。
补上它才真的闭合整圈。

**它顺手修掉一个朝向缺陷**：`towardPOI` 下，一个航点的朝向作用于"飞向
**下一个**航段"。不补收尾点时，最后一个点 `p_{n-1}` 的朝向作用在任何航段上
都无意义（后面没有航段），而 `p_{n-1}→p0` 那段又不存在 —— 于是机头在最后
半段会停在更早给的方向上，**收尾处朝错方向**。补上重合点后，
`p_{n-1}` 的朝向有了归宿。

### 收尾行为：`finishAction = gotoFirstWaypoint` `[V]`（2026-09-23）

**这与"闭合"是正交的两件事，别混：**

| | 目的 | 手段 |
|---|---|---|
| 闭合 | 轨迹绕满整圈、收尾处机头朝向受控 | `lz_plan.c` 末尾补**坐标重合**的收尾点 |
| 收尾行为 | 飞完后**去哪儿** | `lz_wpml.c` 的 `finishAction` |

规范取值域（`30.waylines-wpml.md:130` / `20.template-kml.md:159`，两份都标必需元素）：

```text
goHome             完成航线后退出航线模式并返航
noAction           完成航线后退出航线模式（就地悬停）
autoLand           完成航线后退出航线模式并原地降落
gotoFirstWaypoint  完成航线后立即飞向航线起始点，到达后退出航线模式
```

本项目用 `gotoFirstWaypoint`（常量 `LZ_WPML_FINISH_ACTION`，定义在 `lz_wpml.h`）。

⚠️ 规范里的"航线起始点"是**航线第一个航点**（圆周上那一点、杆旁边
17.5 m 处），**不是起飞点**。

⚠️ **后果：任务正常结束后飞机停在杆旁边悬停，不返航也不降落。**
与拨 OFF 急停的行为一致（`lz_widget.h` 的安全语义）—— 两条路径的操作员
都得手动接管。`globalRTHHeight` 因此在本流程里不再作用于收尾，
**但它仍是必需元素、取值不改**：操作员手动返航或触发失控返航时，
飞机用的还是这个高度。

配合重合收尾点使用时效果最好：飞机飞完最后一个航点时**已经在起始点上**，
本动作几乎立即结束 —— 既停在起点，收尾弧线与朝向又都受航线控制。

### 走弧线而不是正多边形：`turnMode` `[V]`（2026-09-22）

用户要求"用物理圆，无人机走弧线"。**关键认识：航点只给出离散采样点，
两点之间走直线还是曲线由转弯模式决定** —— 不是靠把航点挪位置。

| `turnMode` | wpml 取值 | 轨迹 |
|---|---|---|
| `LZ_TURN_TO_POINT_AND_STOP` | `toPointAndStopWithDiscontinuityCurvature` | 内接正多边形 |
| `LZ_TURN_PASS_WITH_CURVE` | `toPointAndPassWithContinuityCurvature` + `useStraightLine=1` | 近似圆弧 |

后者就是 DJI Pilot 2 里「平滑过点，提前转弯」的设置方法
（规范 `40.common-element.md` 的 `waypointTurnMode` 一行有明确注解）。

**差别有多大**（实测，半径 20 m）：

| 航点数 | 直线模式边心距 | 比圆小 | 弧线模式 |
|---|---|---|---|
| 4 | 14.14 m | −29.3% | 近似圆 |
| 8 | 18.48 m | **−7.6%** | 近似圆 |
| 16 | 19.62 m | −1.9% | 近似圆 |

直线模式下 8 点飞 20 m 半径，飞机实际离杆只有 **18.48 m** —— 近 1.5 m。
改弧线后 8 个点就够，不必靠堆到 16 点去逼近。

**提前转弯截距由几何反算**（`LzPlan_SuggestDampingM`）。规范对
`waypointTurnDampingDist` 有两条硬约束，**都是相对于航段长度的**：

1. 取值域 `(0, 航段最大长度]`
2. "两航点间航段长度必需大于两航点航点转弯截距之和"（段长 > 2×截距）

让调用方自己填就等于要求它先算出段长再倒推，而它多半不会。从 `route` 的
真实几何反算则**不可能与实际不一致**。取最短段 45%（留 10% 裕度避开第 2 条）。

⚠️ 该元素写盘精度用 `%.2f` 不是 `%.1f` —— 规范要求落在**开区间下端**
`(0, …]`；半径 5 m / 64 点时建议截距只有 0.22 m，`%.1f` 会舍成 "0.2"，
再小一点就违反开区间。

⚠️ 截距**只对弧线模式有意义**（规范注明该元素仅在此模式必需），
直线模式下写 0。

⚠️ **弧线模式的取值本项目尚未上机验证** —— 见待确认清单。

### 绕飞怎么让相机盯着杆：靠**机头**，不是靠云台 `[V]`（2026-09-22 定案）

原方案是逐点写 `gimbalRotate` + `gimbalYawRotateEnable=1` + 绝对方位角，
同时机头 `followWayline`（沿航线）。**这个组合在 M4T 上非法**，
试飞时飞机报「一些航点角度过大无法转向」。

两条独立证据：

1. 规范里 **三处**（`gimbalRotate` / `orientedShoot` / `rotateYaw`）都标：

   > `wpml:gimbalYawRotateAngle` 与 `wpml:aircraftHeading` 需保持一致
   > 机型：M3E/M3T，M3D/M3TD，**M4D/M4TD，M4E/M4T**

   出处：Cloud-API-Doc `00.dji-wpml/40.common-element.md`。

2. M4T 规格页（enterprise.dji.com/zh-tw/matrice-4-series/specs）：

   > Controllable Rotation Range — **Pan: Not controllable**
   > Yaw Axis — Manual operation is uncontrollable;
   >            **The MSDK interface program is controllable.**
   > 机械软限位 Pan: **-60° ~ +60°**

   即：M4T 的云台 yaw **不能独立于机头偏转**，只有 MSDK 程序能在
   机械限位内驱动那一小段。而我们要求光轴转 45°、机头不跟 —— 做不到。

**所以改走 `towardPOI`**：`waypointHeadingMode=towardPOI` +
`waypointPoiPoint` 填杆的经纬度。飞机**侧飞**绕圈，机头（因而光轴）
始终指向圆心。这正是 DJI Pilot 2 里「兴趣点环绕」的做法，用户 2026-09-22
确认要的就是这个效果。

配套改动：

- `gimbalYawRotateAngle` 仍写，但数值 = 该点看向杆心的方位角，
  与飞机自己算的 `aircraftHeading` **天然一致** —— 满足上面那条规范。
  它不再代表"云台独立偏转"。
- `waypointHeadingPathMode` 从 `followBadArc` 改为**跟随绕行方向**
  （clockwise / counterClockwise）：机头要主动去追杆，必须明确告诉它朝哪边转。
- `waypointPoiPoint` 从占位符 `0.000000,0.000000,0.000000` 改为**真实杆位**。
  老写法让机头朝几内亚湾转 —— 等于乱转。
- `pole` 从"不参与生成"变成兴趣点坐标来源，因此 `LzWpml_Build` 现在
  **拒绝 NULL / 非法 / 零解**的杆位。

> **教训（方法论）**：判"某个 wpml 能力可用"时，不能只看字段存在 ——
> 还要看「支持机型」列里的**附加约束**。`gimbalYawRotateAngle` 字段确实
> 存在、官方样例也写了，但 M4T 那一档注明了它必须与 `aircraftHeading` 一致。
> **字段存在 ≠ 能按你的意思用。**

### 实现（已完成，2026-09-19）

| 文件 | 职责 |
|---|---|
| `lz/src/lz_plan.c` | `LzPlan_BuildOrbit()` 绕飞圆周生成 —— **已实现** |
| `lz/src/lz_wpml.c` | 生成 `template.kml` + `waylines.wpml` |
| `lz/src/lz_kmz.c` | 打成 zip（**store 模式，零依赖，不引 zlib**） |
| `lz/src/lz_bridge.c` | `LzBridge_ExportKmz()` 串联 |

**绕行方向的约定**（写死在 `lz_plan.c` 的注释里，别改）：
方位角约定是正北 0°、**顺时针为正**。站点由
`LzGeo_Destination(杆, stationBearing, r)` 得到，所以 stationBearing=0
站点在杆正北、=90 在正东 —— 地图上 N→E→S→W 即**俯视顺时针**。
**故 stationBearing 递增 = 俯视顺时针**，`clockwise=true` 取递增。

这条靠推理定，靠两处独立验证守：测试里逐点断言方位角递增/递减；
另有独立程序用**多边形有向面积（鞋带公式）**复核 ——
`[V]` 实测 clockwise=true 得 −1131.37 m²（ENU 中有向面积为负 = 顺时针）。

### 航点数由操作员在 Pilot 2 里填 `[V]`（2026-09-22）

`waypointCount` 曾写死在 `lz_mission.c` 的剖面里。现在它是**控件**：

| index | 类型 | 名称 |
|---|---|---|
| 5 | `int_input_box` | 航点数 / Waypoints |

**为什么用输入框而不是 scale 滑杆**：滑杆只有 0–100 的整数档，而航点数
是很有界的小整数（3–64），输入框能让操作员直接填 16、24，不必心算档位。
代价是**输入框能敲任意整数**，所以取值必须过 `LzPlan_ClampWaypointCount()`。

**两道闸门，同源**：

```text
操作员输入 → LzPlan_ClampWaypointCount()  → 夹到 [3,64]，并回一条浮窗告知
                （控件回调 + getter 各夹一次）
           → LzPlan_Validate()            → 起飞前复核，越界判 LZ_ERR_UNSAFE
```

`LZ_PLAN_WAYPOINT_MIN` / `MAX` 定义在 `lz_plan.h`，**控件层与校验层引用同一组
常量** —— 与半径/高度那组包线同一个理由：两处各写一份会出现"界面允许 100
但校验拒绝 64"这种打架。`lz_test_validate.c` 里有一条用例专门守它
（"夹取之后的值必须能通过校验"）。

**夹取失败要说话**：操作员敲了 100，浮窗回
`航点数 100 超出 3–64，已按 64 使用`。不吭声会让界面显示的 100 与实际飞的
64 长期矛盾 —— 那个差异要等航线画出来才发现。

**上限 64 是现场判断，不是规范限制** —— DJI 协议能收 200 个航点。
⚠️ **这个上限的原始理由已经变过一次**（2026-09-23）：它原先是"转弯模式是
到点停，n 个点就是 n 次起停"，而 2026-09-22 起改用
`LZ_TURN_PASS_WITH_CURVE`（**过点不停**），那个理由**已不成立**。
现在 64 只是"再多也没精度收益、还会撑大 KMZ 与逐点朝向计算"的现场判断。
**改这一节时，`lz_plan.h` 的注释也要一起改**（两处理由已经同步过一次）。

**`waypointCount` 是"圆周上均分几个方位"，不是"航线里几个点"。**
`LzPlan_BuildOrbit` 会补一个与首点**坐标完全相同**的收尾点，所以
`route.count == waypointCount + 1`。别把这两个数混着用。

**航点间距是弦长不是弧长** `[V]`：8 点 20 m 半径，单段弦长 15.307 m。
⚠️ **总路径长度要按段数算清**（2026-09-23 更正）：闭合后段数 = n（含收尾段），
所以总长 = 8 × 15.307 = **122.459 m**；**107.151 m 是闭合前的旧值**
（7 段，对应改 `i <` 那次反向验证的输出）。整圈周长 125.664 m ——
弦长总长与周长差 2.6%（不再是 17%），因为收尾段把最后那段弧补上了。
wpml 的 `distance`/`duration` 必须按实际路径算，不能按 2πr。

**零依赖的理由**：航点文件才几 KB，压缩毫无收益；引 zlib 会破坏 lz_core
"桌面上 clone 下来就能编"的性质。

### 已验证 `[V]`

```bash
./build/lz_kmz_demo /tmp/lz_test.kmz
python3 -c "import zipfile;z=zipfile.ZipFile('/tmp/lz_test.kmz');print(z.testzip())"
# → None（CRC 全部通过）
```

**用独立的解包器验证** —— 自己写的包自己解，什么都证明不了。

## wpml 必需元素：以规范为准，**不以官方样例为准** `[V]`（2026-09-21）

规范原文在 **Cloud-API-Doc 仓库**（不在 `.psdk-apiref` 里，那个只收了 PSDK 文档）：

```bash
B=https://raw.githubusercontent.com/dji-sdk/Cloud-API-Doc/master/docs/cn/60.api-reference/00.dji-wpml
curl -sL $B/20.template-kml.md $B/30.waylines-wpml.md $B/40.common-element.md
```

**为什么样例不能当权威**：实测官方样例 KMZ 自身也缺
`wpml:globalRTHHeight`（规范标为必需元素）却照样能飞。所以：

> "样例里有" 推不出 "必需"；"样例里没有" 推不出 "不必需"。**方向是反的。**

而且**逐元素加"必需"项是错的**，必须同时看两列：

| 元素 | 「是否必需」 | 「支持机型」 | 结论 |
|---|---|---|---|
| `gimbalHeadingYawBase` | 必需 | 含 M4T | **要加**（我们使能 yaw，依赖这个基准） |
| `missionAutoRerouteMode` | 必需 | 仅 **M3D/M3TD** | 不加 |
| `imageFormat` | 必需 | — | 不加（属相机动作，我们不做 takePhoto） |
| 各类 `shootType`/`margin`/… | 必需 | — | 不加（属 `mapping2d/3d` 模板，我们用 `waypoint`） |

**只 grep "必需元素" 会把 M3D 专属元素也加进来。** 这条方法论写进了
`tests/lz_test_wpml.c`（"不应写入 M3D 专属的绕行元素"用例）。

`gimbalHeadingYawBase` 已补上（`north`），且 yaw 的**语义**已在
2026-09-22 改掉 —— 见上文「绕飞怎么让相机盯着杆」。现在它不再是
"云台独立偏转"，而是与机头目标角一致的从属值。

## 上机已确认与仍待确认

**已确认（上机实测）**：

- 控件在 Pilot 2 **相机视图左侧"PSDK"菜单**里显示并能触发回调 `[V]`
  （2026-09-20 实测 3 个控件；2026-09-22 加到 5 个后按钮、滑杆同样触发。
⚠️ 2026-09-22 新增的**控件 5（航点数输入框）尚未上机**）
- **KML/KMZ 上传全链路通**：41 个分片 + `Check kmz file md5sum success` `[V]`
- **航点任务能真正启动** `[V]`（2026-09-22 室外首次成功）—— 见下方
  「绕飞启动曾经失败的原因」
- **激光测距可用** `[V]`（2026-09-22）：记下 `28.1788176, 112.9210889`
  （距离 4.4 m），与「飞机位置 + 距离」反算一致
- **记录飞机位可用** `[V]`（2026-09-22）：`28.1788120, 112.9210473`

**仍待确认**：**现场执行清单见 [`lz/doc/ONDEVICE-CHECKLIST.md`](lz/doc/ONDEVICE-CHECKLIST.md)**
（每项带判据、观察点与回退方案）。下面只是索引：

- [ ] **绕飞完整链路（拨开关 → 飞机绕记录的点飞一圈）** —— 唯一还没跑完的环节。
      所有中间环节都验过，但"记录圆心 → 拨开关 → 飞机真的绕圈"这一整条链从未走通
      ⚠️ 收尾动作已是 `gotoFirstWaypoint`，**完成定义里没有"返航"** ——
      飞完停在航线起始点悬停，要返航得手动接管
- [ ] **航点数输入框（控件 5）从未上机** —— 2026-09-22 新增后没试过。
      现场先验它：填 100 必须回 `已按 64 使用`（不吭声才是 bug）
- [ ] `droneEnumValue=99` / `payloadEnumValue=89`（M4T）是否被飞机接受
- [ ] **towardPOI 在 M4T 上实测能否生效** —— 首次上机。观察点：飞机是否侧飞、
      机头是否始终对着杆
- [ ] 云台 yaw 是否真如规范所说"与 aircraftHeading 一致"（不再独立偏转）
- [ ] **`LZ_TURN_PASS_WITH_CURVE` 实测是否真的走弧线** ——
      `toPointAndPassWithContinuityCurvature` + `useStraightLine=1` +
      `waypointTurnDampingDist` 这个组合**从未上机验证过**。
      观察点：轨迹是否仍是一段段直线（用航迹回放看）、
      实际离杆距离是否回到设定半径附近（直线模式会近 7.6%）
- [ ] **`gotoFirstWaypoint` 在 M4T 上是否被接受** —— 规范标必需元素且
      机型列含 M4 系列，但**从未上机验证**。观察点：飞完一圈后飞机是否
      停在航线起始点（杆北侧、距离 = 设定半径），而不是返航回起飞点
- [ ] **收尾点是否被接受** —— 末点与首点坐标完全相同（距离 0 m）。
      理论上合法（航线本就是一系列坐标），但**未实测**。
      若飞机报错，回退方案是把收尾点的方位角偏一极小量（如 0.01°）
- [ ] **POI（`DjiInterestPoint_*`）在 M4T 上是否可用** —— 文档说"及后续机型"
      也覆盖 M4T；若可用且能接受半径不可控，它比自建 KMZ 省事得多
- [ ] 运动规划是否需要额外申请 PSDK 高级权限？矩阵里个别功能有此标注，未确认

### 绕飞启动曾经失败的原因 `[V]`（2026-09-22 定案）

`DjiWaypointV3_Action(START)` 曾在三次不同环境下全部被拒，错误码随环境变化：

| 环境 | GPS | 飞机给的 error_code |
|---|---|---|
| 模拟器 | fixState=3，15 星 | **770** `CANNOT_START_AT_CURRENT_RC_MODE` |
| 真机室内 | fixState=0，0 星 | **769** `GPS_INVALID` + **771** `HOME_POINT_NOT_RECORDED` |
| 真机室外 | fixState=3，17–19 星 | **770** |
| **真机室外（修完 wpml 后）** | fixState=3 | **成功** `[V]` |

**"模拟器不实现"那条结论已撤回** `[X]`：它建立在 `0x000000FF` 上，
而那是 `(unsigned)` 截断后的假象；且当时那份 KMZ 确实缺 wpml 必需元素
（`globalRTHHeight` / `gimbalHeadingYawBase` / `waypointHeadingPathMode`），
**从未通过过任何一次内容校验** —— 上传只做字节与 MD5 传输校验，
内容校验发生在 START 时才报出来。

⇒ **修完 wpml 合规项后，室外真机启动成功。** 770 那句官方文案
（"Cannot start in the current RC mode"）在这台 M4T 上大概率是误导 ——
它是 SDK 拿裸数字查自己那张表得出的英文，**不等于飞机给的原始原因**。

**⚠️ PSDK 3.16.0-beta 的实测边界（2026-09-20，别再试）**：

`DjiFcSubscription_GetLatestValueOfTopic` **必崩**（SIGSEGV，栈在
`DjiDataSubscriptionDds_v3_GetLastValueOfTopic` 内部）。已排除读太快、
传 NULL 回调、无数据、初始化时机四个嫌疑。**取飞机状态一律走回调缓存**，
实现见 `src/lz_bridge_psdk.c` 的启动诊断段。

## 失败可观测性：三条"看起来有、实际没有"的坑 `[V]`（2026-09-21 审计）

这三条都是"代码写了、注释说得像真的、但实际没生效"，比明显的 bug 危险：

### 1. PSDK **没有**反向推控件状态的接口 —— 别写"把开关拨回去"

`lz_widget.c` 曾这样写，注释还写着"程序化拨回 OFF"：

```c
(void)LzWidget_SetWidgetValue(DJI_WIDGET_TYPE_SWITCH, ..., OFF, NULL);  // 空头承诺
```

那是**我们自己**注册给 SDK 的回调，被 Pilot 调用时才生效；直接调它只改本地变量。
`dji_widget.h` 全部 **9 个**导出函数里没有任何 setter；`dji_widget_manager.h` 的
`DjiWidgetManager_SetWidgetState` 目标是**机上挂载的负载**，不是本应用在 Pilot 的 UI。

**真实后果**：开关**保持 ON**（持续可见）而浮窗说"绕飞结束"（会被下一条覆盖）——
矛盾的两条信息里，持续可见的那条是错的。现在的做法是如实提示
"⚠ 开关仍在 ON 位，请手动拨回"。

### 2. 浮窗去重表：发送失败也标记"已发" = 该消息永不重发

`LzWidget_MsgTask` 里发完就 `memcpy(lastMsg, ...)`，一次失败（通道未就绪）
就让这条消息**永远消失**。浮窗是室外**唯一**的反馈通道，
"启动被拒：…" 丢了等于没有反馈。**只在返回成功时才更新去重表。**

### 3. `(void)f()` 丢掉的是"停止失败"

拨 OFF 的 `(void)LzBridge_StopMissionV3()` 曾把 STOP 失败完全吞掉，
随后照样报"绕飞结束"。**静默的失败等于假装成功**，而在飞控语境里
这直接关系到安全（飞机还在杆旁边绕，操作员以为停了）。
现在：加日志、检查返回值、失败时**保持 RUNNING**（状态与事实一致）
并只报一次浮窗（Tick 是 100 ms 一拍，不加闸会每秒刷 10 条）。

## 测试框架：0 项检查也算失败 `[V]`（2026-09-21）

`LZ_TEST_SUMMARY()` 现在把"检查项数为 0"判为失败。

**为什么**：本工程的用例大量写成 `if (obj != NULL) { LZ_CHECK(...) }`。
一旦构建静默失败（缓冲区算小了、入参校验误拒），那些断言**整体不执行**，
汇总打印"0 项检查，0 项失败"并返回 0 —— **测试绿着，但什么都没验**。
这种"空跑成功"比失败更危险：它让一次真实的回归伪装成通过。

## 反向验证：修复必须能变红 `[V]`（2026-09-21 起执行）

每修一处关键缺陷，都**逐条回退该修复、确认对应断言变红**，再恢复。
2026-09-21 那轮实测：

| 回退的修复 | 变红的断言 |
|---|---|
| wpml 三元素（`globalRTHHeight`/`yaw` 归一/…） | 12 条 FAIL |
| 安全包线检查 | 6 条 FAIL |
| 零检查守卫 | 单跑空测试退出码 1 |

**不这么做的代价**：断言可能因为某个前置条件恒假而从未真正执行过 ——
那正是本项目已经踩过的形状（`LZ_TODO_PENDING`、`if (f.xxx != NULL)` 守卫）。

**2026-09-23 那轮实测**（本次交接用新鲜构建重跑，从全新 shell 取真实退出码）：

| 状态 | `lz_test_plan` 退出码 | ctest |
|---|---|---|
| 基线与远端一致 | 0（1075 项检查，0 失败） | 100% passed, 0 failed out of 7 |
| 只拆闭合（`i <=` 改回 `i <`） | **1**（1040 项检查，172 项失败） | 71% passed, **2 failed** |
| 只让 `LZ_CHECK_ANGLE_NEAR` 不计数 | 0（1075 项检查，0 失败） | 100% passed —— **看起来全绿** |
| 宏带 bug + 9 条纯角度断言失败 | **0、且汇总打印"0 项失败"** | **100% passed** ← 真实失败被伪装成通过 |
| 上一条把宏修好 | **1**（1075 项检查，9 项失败） | 86% passed, 1 failed |

第 4 行是这次实测**唯一能演示该 bug 的构造**：拆闭合那条路径的断言全走
`LZ_CHECK`/`LZ_CHECK_NEAR`，用不上角度宏，所以宏坏掉时它的 172 条失败里
仍有 143 条被正常计数、ctest 照样变红 —— **掩盖不了**。
必须**单独**让一条只用角度宏的用例失败（这里把 `toPole` 人为偏 45°），
才能看到"`0 项失败` + ctest 全绿"这个假象。
⇒ **反向验证要按宏逐条做，不能用一条路径一次测完所有修复。**

## 激光测距：已实测可用 `[V]`（2026-09-19）

**位置 1**（`DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1`，M4T 自带云台相机那一路）
就是激光测距所在，`enable_lidar=1`。实测断联后重连仍成立。

### ⚠️ 判据是 `distance`，**不是** `exception` 白名单 `[V]`（2026-09-22 更正）

**这条是按错误结论改过两次的，最后一次的教训值得单独记。**

`exception` 的取值**官方从未公开**（头文件与中英文档都只有"异常标志"四字），
只能实测反推。实测见过的**四个**值：

| exc | 伴随的 distance | 结论 |
|---|---|---|
| **0** | **4.4 ~ 14.8 m（有实数、随目标变化）** | **正常读数** `[V]` 2026-09-22 室外实测 |
| 1 | 0（恒为 0） | 无回波 |
| 2 | 0 | 过渡态，含义不明 |
| 3 | 有实数 | 也是正常读数（早先以为只有它是） |

**踩的坑**：2026-09-19 那次对照实验只观察到 `3 = 正常`，于是写成白名单
`if (exception != 3) 拒绝`。2026-09-22 室外实测全是 `exception=0` 且
**距离在变**（4.4 → 14.8 m），全部被误拒，浮窗还报"激光无回波" ——
**操作员被引去调瞄准，而实际该做的是改代码。**

⇒ **判据改为只看 `distance`**：

```c
if (info.distance <= 0) { 拒绝 }        /* distance 单位 0.1 m */
```

`distance` 是三个字段里唯一**可自证**的量（由激光独立测量、不参与坐标解算，
无回波恒为 0）；`exception` 是飞机给的语义标签，取值域没有权威来源。

> **可推广的经验**：当一个字段的取值域**没有权威来源**时，不要用枚举白名单 ——
> 改用有物理意义、能自证的量。白名单会随新观测不断打补丁，
> 而漏掉的那个恰恰会在最需要它的时候出现。

### 三道闸门各自独立，不能互相担保

激光的经纬度 = **机身位置 + 云台朝向 + 距离** 解算出来的，所以它
**依赖飞机自身有定位**。实测（室内无 GPS）：

    distance = 2.0 m（有效！）  而  lat/lon = 0.0000000, 0.0000000

**"有距离"推不出"坐标有效"** —— 解算退化成零解，而 (0,0) 在经纬度范围内
完全合法，`LzGeo_IsValid` 拦不住。三道闸缺一不可：

| 闸门 | 判据 | 拦什么 |
|---|---|---|
| ① | `distance > 0` | 无回波；也拦"退化成机身位置"（模拟器实测：`exception=2` 且 `distance=0` 时 `lat/lon` 是飞机初始位置 113.1700000, 28.2666000） |
| ② | `LzGeo_IsValid` | 坐标越界 |
| ③ | **`LzGeo_IsNullSolution`** | **零解邻域**（飞机没定位 → 瞄准点解算退化） |

判据与阈值见 `include/lz_types.h` 的 `LZ_GEO_NULL_SOLUTION_DEG`（0.5°）。
**必须给邻域、不能写 `== 0.0`** —— 退化值带浮点残差，这是本项目踩过两次的形状。

### 排查时先做冗余校验

`distance` 与 `lat/lon` 来自**两个独立来源**。用「飞机位置 + distance」
反算两点距离，与日志里的 distance 对照 —— 对得上说明两者都真。
实测 2026-09-22：

    飞机 28.1788120,112.9210473 → 激光点 28.1788176,112.9210889，报距离 4.4 m，反算 4.1 m ✓
    飞机 28.1788x,112.9210x     → 激光点 28.1789403,112.9210401，报距离 14.8 m，反算 14.7 m ✓

**能自洽的冗余数据本身就是校验** —— 比单看"回执是 ✓"强得多。

**`distance` 单位是 0.1 m**（分辨率 0.1 m），所以读数恒定不代表没工作 ——
真实距离落在同一 0.1 m 区间内读数就该恒定。**判"测距是否工作"要看
读数是否随目标移动而变化，不是看它有没有抖动。**

查挂载位置的探针：`lz/app/lz_rangefinder_probe.c`
（`./lz_rangefinder_probe` 扫全部位置；`./lz_rangefinder_probe 1 -1` 连续监视）

查任务链路崩溃点的探针：`lz/app/lz_mission_probe.c`（4 个模式，
`0`=只上传 / `1`=上传+订阅+启动 / `2`=加读话题 / `3`/`4`=只验证订阅与回调）。
用法与前提见文件头注释。

## Pilot 2 控件：操作员的入口 `[V]`

`app/lz_widget.c`（控件）+ `app/lz_mission.c`（作业状态机）+ `app/widget_file/`（配置）。

```text
控件 0  switch        绕飞       OFF=停止(STOP 非 PAUSE)   ON=上传航线并开始
控件 1  scale         半径m      0-100% → 5~20 m（默认 50% → 12.5 m）
控件 2  scale         高度m      0-100% → 5~120 m（默认 66% → 约 80.9 m）
控件 3  button        记录飞机位  按下即记录飞机当前位置为绕飞圆心
控件 4  button        记录激光点  按下即记录激光瞄准点为绕飞圆心
控件 5  int_input_box 航点数      直接填个数，夹到 3~64（默认 8）
```

⚠️ `widget_file/` 下**每 6 个控件**（`cn_big_screen` / `en_big_screen` 两份），
且两份必须"类型、索引、数量都相同"（见下方四条约束第 3 条）。

**控件在 Pilot 2 的相机视图左侧「PSDK」菜单**里，**不在妙算3 应用管理面板** `[V]`
（2026-09-20 实测；应用管理面板里那个按钮只是 `dji_app_ctl start`）。

**配置目录路径不能写死** `[V]`：装成 dpk 后 CWD = 包根（`widget_file/` 在包根），
从源码树跑时 CWD = `lz/`（`app/widget_file/`）。`lz_widget.c` 用运行时解析两个候选。

**控件是基础功能**（`basic-function/widget.html`），比航点那条高级功能的路好走
—— 没有"是否需要申请 PSDK 高级权限"的不确定性。

**架构要点**：控件回调**只置标志、不做动作**，真正的作业决策集中在
`LzMission_Tick()`。

⚠️ **这条是硬约束，不是风格偏好** `[V]`（2026-09-22 踩过，进程闪退）：
回调跑在 **PSDK 的工作线程**上，任何**阻塞调用**都会把 SDK 的链路拖死 ——
日志表现为 `dji_msgq.c:227 semaphore wait timeout` +
`dji_linker.c:309 send msg to queue error` 刷屏后进程死掉。

踩的过程：给「记录激光点」按钮写的回调里直接调了
`DjiCameraManager_GetLaserRangingInfo()`（同步阻塞，最多 1.2 s）。
日志里连该函数的第一条 log 都没打出来就崩了。**「记录飞机位」一直没事，
因为它只读订阅回调写好的静态缓存，不碰阻塞接口。**

> **判据**：`LzWidget_SetWidgetValue` 的函数体剥掉注释后，不应出现
> `DjiCameraManager` / `LzPole_Record*` / `LzBridge_GetCurrentPosition` /
> `fopen` 等任何阻塞或跨层调用 —— 只剩置标志与 `PostMessage`。
> 有脚本做过这个检查，见 `lz_widget.c` 里的说明。

**危险处在于：违反这条约定不会编译报错。** 「上传 KMZ 要搬出去」这课
2026-09-20 已经学过一遍，但当时只学成了"KMZ 要搬"，没推广到
"任何阻塞调用都要搬" —— 两天后在激光上又踩一次。

**安全包线是硬约束，唯一真值在 `lz_plan.h`** `[V]`（2026-09-21）：

```c
#define LZ_PLAN_RADIUS_MIN_M     5.0
#define LZ_PLAN_RADIUS_MAX_M     20.0   /* 场地约束：杆周围 20 m 内无建筑物 */
#define LZ_PLAN_ALTITUDE_MIN_M   5.0
#define LZ_PLAN_ALTITUDE_MAX_M   120.0  /* 相对起飞点，不是 ASL */
```

`LzPlan_Validate` 按它拒（超限返回 `LZ_ERR_UNSAFE`，不是 `LZ_ERR_PARAM` ——
前者"合法但危险、操作员可修正"，后者"数值非法、是编程错误"，排查方向不同）；
**控件层引用同一组常量**，不得自备一份。

> 教训：原先上限只活在控件层的滑杆映射宏里，`LzPlan_Validate` 看不到 ——
> 换个调用点（探针、demo、将来的自动规划）就能构造超限剖面并通过校验。
> **校验层的判据不该取决于谁在调用。**

**四条约束**（`[V]` 已核实）：
1. **没有 `DjiWidget_DeInit()`** —— 全模块 9 个接口里只有 `Init`
2. 配置是**目录**不是文件（json + 图标 PNG 放一起，按语言/屏幕分目录）
3. 不同语言的配置**必须有相同的控件类型、索引和数量**
4. 浮窗消息带宽上限 **2 KB/s**

## 绕飞圆心从哪来：`lz/app/lz_pole_source.c`

整条链路上**唯一的外部输入**，也是最后一处需要现场确定的东西。

```c
LzStatus LzPole_Acquire(LzTarget *out);   // 取不到就不起飞
```

**2026-09-22 起改为操作员在 Pilot 2 控件上现场记录** `[V]` ——
原先写死在源码里的固定坐标不再使用（只在显式
`-DLZ_POLE_SOURCE_FIXED` 时生效，调试用）。

| 方式 | 取数 | 前提 |
|---|---|---|
| **记录飞机位**（按钮 idx 3） | 融合位置（订阅缓存，非阻塞） | 飞机自身有定位 |
| **记录激光点**（按钮 idx 4） | 激光瞄准点（相机接口，阻塞 1.2 s） | 打中目标 **且** 飞机自身有定位 |
| 固定坐标 | 源码宏 | 仅 `-DLZ_POLE_SOURCE_FIXED` |

**未记录时拒绝启动绕飞**（`LZ_ERR_NOT_READY`），**不回落固定坐标** ——
回落到几十公里外的点会让飞机飞过去，而操作员以为在原地绕圈。
⚠️ 这只拦「启动绕飞」，**不拦飞机起飞**（程序从不控制起飞）。

记录落到 `data/pole.txt`（人类可读一行，现场可 `cat` 核对）：
`lon=... lat=... alt=... src=aircraft|laser`

**为什么要单列一个模块**：至少三条实现路径（激光 / 视觉定位 / 固定坐标），
选哪条取决于现场条件。收在一个接口后面，换方式时只动这一个文件。

## 视觉层有两个互斥后端

```text
-DLZ_VISION_BACKEND=stub   (默认) 固定杆位占位实现，让链路先跑通
-DLZ_VISION_BACKEND=hsv           src/lz_vision.c，**真算法，已实现**（2026-09-25）
```

两者都定义 `LzVision_Detect`，**同时编会符号冲突**，CMake 里二选一。
`hsv` 后端额外注册 `lz_test_vision`（拿 6 张真实照片跑回归），stub 后端不注册
—— 它刻意不看画面，比对了也证明不了什么。

⚠️ **视觉层仍未接进主链路** `[V]`（2026-09-25 grep 确认）：
`app/lz_vision_source.c` 里 `LzVisionSource_Start/Stop()` 只有声明、没有实现，
也没有任何地方调用。所以**程序不看图，也感知不到"有没有红旗"**。
本轮做的是**算法层**（`lz_core` 侧的 `lz_vision` 库），取图那一环还空着。

### ⚠️ 检测的是**杆**，不是旗面中心 `[V]`（2026-09-25 实测，重要）

早先的注释说"取旗面外接框下边中点作为杆的像素位置"。**这条规则在真实照片上
是错的**，实测 6 张俯拍照片：

| 照片 | 旗 bbox 中心 x | 真实杆列 x | 偏差 |
|---|---|---|---|
| 1 | 668 | 647 | −21 px |
| 2 | 604 | 657 | **+52 px** |
| 3 | 676 | 629 | −47 px |
| 4 | 661 | 632 | −29 px |
| 5 | 667 | 626 | −41 px |
| 6 | 665 | 631 | −34 px |

偏差 **−3.7% ~ +4.1% 画面宽**（FOV 82° ⇒ 约 ±3.3°），且**符号随风向翻转**
—— 风把旗吹向一侧时杆在另一侧。这是"半个旗宽取决于风向"的必然结果，
**不是常数偏差，标定不掉**，半径越大放得越大。

⇒ 现在两级检测：① 红色连通域取最大块 = 旗面（可信度高：第一名/第二名面积比
**7.9x ~ 103x**）② 旗面附近 ROI 内做**竖线对比度滤波** = 杆的精确像素列。
杆比周围**亮**（压绿篱/水面）或**暗**（压浅色铺装）都算，故不依赖背景明暗。
实测跨 54 组参数（3 邻域 × 3 阈值 × 6 图）杆列只动 **0–2 px**。

`LzTarget.pixel.u` 现在是**杆列**的归一化横坐标。**上层做对齐判定要对杆，
不对旗** —— 激光要打的是杆，旗会飘；而杆是竖直的，打在杆上任何一点反算出的
经纬度都一样。且"对齐"只需比 **x**（水平），因为杆竖直、纵向差多少都还在杆上
（这正好对上"高度定、半径定 ⇒ 俯仰角不变"）。

### 现场形态 ✓ 与未覆盖的情况

`[V]` 已核实：现场是"**旗在杆顶、迎风向水平展开**"（不是"垂下来贴着杆"）——
两者红色块的形状与相对杆的位置完全不同，本实现只在前者上验证过。

⚠️ **未覆盖**：阴天、逆光。HSV 阈值在阴天的表现是这套算法的公认弱点，
好在失败可解释（调 `minSaturation`），这正是选它而不用模型的原因。
HSV 分布实测（16966 样本）：H 双峰（低段 8.2% / 高段 91.8%），
S p5=129 / V p5=170 —— 现用的 `100` / `60` 很保守，阴天余量充足。

### 杆列检测的两个细节 `[V]`（都是实测逼出来的）

1. **投票窗口必须只取旗附近，不能取整幅图。** 一列是不是杆靠"有多少行满足
   对比度判据"投票；整幅图投票时画面下缘的绿篱边缘、铺装接缝都会投票，
   而且**结果依赖"图有多大"而不是"画面里有什么"** —— 实测裁剪后偏 6 px。
   收窄到旗附近后，全图与裁剪图给出同一答案（0 px）。
2. **置信度的分母用"画面高度的 25%"，不是配置里的窗口行数。** 窗口总是比杆长
   （要为旗下方那段杆留余量），拿窗口当分母量到的是"杆长/窗口长"，
   与取景方式有关。更要命的是旗靠近画面下缘时窗口被截短 ——
   好目标被无谓压低判成不可用。

### 视觉测试数据：6 张真实照片入库

`tests/data/`（1.6 MB，6 个裁剪后 PPM + `vision_golden.txt`）。**入库是刻意的**
—— 它的价值就在于"不需要现场照片也能复现检测行为"。

- 期望值来自**全图检测 + 人工目视核对**再**纯平移**到裁剪坐标，
  **不是算法自己的输出**（那样是循环论证，测不出任何东西）。
- 生成工具 `tools/gen_vision_testdata.js`（纯 JS，只需 `jpeg-js`，
  **故意不依赖 PIL/OpenCV**：生成测试数据不该给工程引入新构建依赖）。
- 核对图工具 `tools/render_vision_check.js` —— 把黄金框画在图上供人工过目
  （绿框=旗面框，黄线=杆列）。输出不参与测试。
- ⚠️ 已知偏差**故意不掩盖**：裁剪前后杆列差 0–2 px（ROI 被 clamp 到图边的
  边界效应），故容差取 ±2 px。收紧到 0 会让将来真的偏移 2 px 看不出来。

### 写视觉测试时的三条教训（都是"构造没逼出要测的东西"）

这三条同源，值得单独记：

1. **连通域用例画布取 34×24** → 红块占画面 28%，撞上置信度里"红块过大"的惩罚，
   被测对象从连通域变成了置信度。
2. **改成 200×200 的大 U 形后，去掉 union 整段代码测试照样全绿** ——
   只要两块上下接触，第二遍路径压缩时的**左邻/上邻传播**就足以连起来。
   可用的形状是「两块**横向**分离 + 一条 1 行的横桥」（桥最左像素的左邻与
   左上都是背景 → 建新根 → 向右走到左块下方时左邻≠左上，必须真合并）。
   **并且要先用对照程序确认差异够大**（正确 326 px / 不做 union 176 px）。
3. **"无杆证据"用例画布取 120×120** → 红块占 16.7%，又被旗面那条拦掉。
   放大到 400×400（红块只占 1.5%）才真的隔离出杆这一项。

⇒ **加"人造图形"用例时，必须先验证"把被测逻辑拆掉它会红"。** 否则那条用例
测的是别的东西，而且它照样绿 —— 与 `LZ_CHECK_ANGLE_NEAR` 漏计是同一形状。

## 云台俯仰必须几何反算，不能写死 `[V]`（2026-09-25）

**你的直觉是对的**：绕飞时高度定、半径定 ⇒ 视线俯角恒定 ⇒ 俯仰角飞行中不需
变化。但它的**值**随半径与高度变化，写死必然在某些组合下错得离谱：

    俯仰 = -atan2(飞机相对目标高度 - 目标高/2, 半径)

| 半径 | 高度 | 看向杆底 | 看向杆顶 |
|---|---|---|---|
| 5 m | 80.9 m | −86.5° | −85.7° |
| 12.5 m | 80.9 m | −81.2° | −79.3° |
| 20 m | 20 m | −45.0° | −14.0° |
| 500 m | 120 m | −13.5° | −11.9° |

实测：写死的 **−15°** 只在"低高度 + 大半径"下碰巧接近，在**默认滑杆值**
（半径 50% → 12.5 m，高度 66% → 80.9 m，杆高 15 m）下正解是 **−80.34°**，
差 **65°** —— 相机根本没对着目标，而画面上看不出异常。

实现：`LzPlan_ComputeGimbalPitchDeg()` + `LzOrbitProfile.autoGimbalPitch`
（`lz_mission.c` 已打开）。瞄**目标中点**（= 半高），与"机头瞄杆心"在水平方向
的居中对称。

⚠️ **椭球高差只在 `LzPlan_BuildOrbit` 一处补齐**：

    相对目标底 = (起飞点椭球高 + 相对起飞点高度) - 目标椭球高

`profile->altitudeM` 是**相对起飞点**的，不是相对目标的。忘了补的表现是俯仰整体
偏掉，而"偏一点"在画面上看不出来。

⚠️ **`LzPlan_ComputeGimbalPitchDeg` 照实返回、不钳位**，越界由 `LzPlan_Validate`
用 `LZ_ERR_RANGE` 拒绝。中途试过钳到相机限位 `[-90, 30]`，但那样会把
「**飞机低于目标时该往上看，而相机上仰极限只有 +30°，物理上做不到**」
伪装成"做得到" —— 与"静默的失败等于假装成功"是同一个形状。

## 平台层已抽出共用模块 `[V]`

`lz/app/platform/`（`lz_platform.c` + `lz_user_info.c`，共 ~330 行）——
移植自官方样例 `DjiUser_PrepareSystemEnvironment()` 与 `DjiUser_FillInUserInfo()`，
逻辑未改。

**为什么要抽**：这段有近 150 行 handler 样板，每个 PSDK 应用都得原样做一遍。
抽出后 `lz_app` / `lz_rangefinder_probe` / `lz_mission_probe` 共用一份，
避免"探针能跑、主应用不行"这类由样板差异引起的怪问题。

⚠️ **include 路径的坑** `[V]`：CMake 加进搜索路径的是 `${LZ_M3_DIR}/hal`
（hal 目录**里面**），所以应写 `#include "hal_usb_bulk.h"`，
写成 `"hal/hal_usb_bulk.h"` 会找不到文件。

## 凭据（构建期注入）

⚠️ **凭据是 configure 期注入的** `[V]`：拷进来之后**必须重跑 `cmake` 配置**，
只 `cmake --build` 不会重新生成 `dji_sdk_app_info.h`。会看到提示
`已从 ... 载入机载应用凭据（App ID 184842）`；没有这行就是没读进去。

`lz/lz_credentials.ini`（**未跟踪**）由 `cmake/gen_app_info.cmake` 在构建期生成
`dji_sdk_app_info.h` 到构建目录，并遮蔽官方样例的同名头文件 —— 明文密钥因此
不进源码树。机制由 `wt_inspection` 移植而来，理由见该脚本头部注释。

**换 App 时必须同时改两处**：`lz_credentials.ini` 与 `app_json/app.json` 的
`user_app_id`（后者写真实值是"凭据只有一处真值"原则的唯一例外）。

新 worktree 里这个文件**不会自动出现**（`git worktree add` 不复制未跟踪文件）。

## 打包

```bash
# 设备上编译完成后（在仓库根，即 ~/lzbuild）
tools/build_dpk/build_dpk.sh -i lz/app_json/app.json -o ~/dpk
dji_app_ctl install -i ~/dpk/liangzhourenwu_v01.00.00.00.dpk
```

- 构建目录名必须是 `build-native`（`app.json` 的 `bin` 指向 `../build-native/bin/lz_app`）。
- `app.json` 的 `userconfig` 必须是 `["../app/widget_file"]` —— `build_dpk.sh` 会
  `cp -r` 到**包根**，产出 `/open_app/<app>/widget_file/`，与运行时 CWD（包根）一致。
- 装包**需要飞机通电并连接**，且启动路径不能提前退出
  （见仓库级 CLAUDE.md 的「安装器会试运行应用」）。
- **`app.json` 必须四语言齐全**（`description_{cn,en,jp,fr}`）—— `build_dpk.sh`
  逐个字段校验，缺一个直接退出，报 `KeyError: 'description_jp'`。

### 设备清理记录（2026-09-19）

设备是**多项目共用**的，wt_inspection 在上一轮开发中留下了不少产物。已清理：

| 位置 | 清了什么 | 结果 |
|---|---|---|
| 已安装应用 | `wt-widget-demo`（2026-09-17 装，data 285 MB） | ✅ `APP UNINSTALL SUCCESS` |
| `/open_app/` | `wind-turbine-inspector_*.dpk`、`wti-build-src/`(11M)、`wti_debug/`(1.7M)、`wti-source-m3.tar.gz`(960K) | ✅ 已删 |
| `~/dpk/` | `wt-inspection_v01.00.00.00.dpk`(276K) | ✅ 已删 |
| `~/data/logs/WT/` | wt 的运行日志 | ✅ 已删 |
| `/blackbox/system/app_temp_files/wt-*.log` | 27 个 | ❌ **删不掉** |

`/blackbox/system/app_temp_files/` 是 **root:root 0755**，`dji` 用户无写权限，
**且没有 sudo** —— 这 27 个日志只能留着。它们只是文本日志，不影响功能与通道。

> 该目录里还有**别的项目的日志**（如 `matrice4t-square-mission_*.log`，7 月），
> 所以它是 DJI 统一管理的应用日志区，别把它当自己的地盘。

**留给下次的提醒**：清理时先 `dji_app_ctl uninstall` **应用**（它会连带清
`/open_app/wt-widget-demo/` 与 data），再手工删源码目录与安装包。

## 设备侧打包清单（约 1MB）

```bash
tar czf /tmp/lzbuild.tar.gz --mtime="$(date -d '+30 seconds' '+%Y-%m-%d %H:%M:%S')" \
     --exclude='lz/build*' --exclude='*.dpk' \
     lz tools/build_dpk psdk_lib/include psdk_lib/lib/aarch64-linux-gnu-gcc \
     samples/sample_c/platform/linux/{manifold3,common}
```

⚠️ **必须带 `--exclude='lz/build*'`** —— 否则会把本机 PC 侧的 `build/`
（x86 产物）一起传上去，设备上解包后与 aarch64 构建混在一个目录里。
`tools/build_dpk` 也要带上（打包 dpk 在设备上做）。

⚠️ **不要用 `--mtime='@0'`** `[V]`（2026-09-19 实测踩到）—— 它把源码时间戳压到
1970，`make` 会发现**源码比 `.o` 旧**，于是**静默跳过编译**。现象是"传了新代码
上去，跑的还是旧二进制"，极难察觉。

正确做法：时间戳取**设备当前时间附近**。实测 `+30 seconds` 会报
"is 29.2 s in the future"（tar 取的是打包那一刻，而设备时间在走），
**用 `-60 seconds`（过去）最稳** —— 既不在未来，也不会倒挂到 1970。
仓库级 CLAUDE.md 说的"用 `--mtime='@0'` 压到过去"防的是"时间戳在未来"，
但压到 1970 会造成反方向的倒挂 —— 两边都要防，取中间值。

「CMake 硬编码找 `psdk_lib/lib/aarch64-linux-gnu-gcc/libpayloadsdk.a`、
别压平目录层级」见仓库级 CLAUDE.md。

## 已证伪的路

沿仓库级 CLAUDE.md 与 `wt_inspection` 的结论，不重复试：

- **交叉编译** → 产物要求 `GLIBC_2.34`，设备 `ldd` 报 not found `[V]`
- **在 `/tmp` 下打包/编译** → 设备重启后 `/tmp` 被清空 `[V]`
- **`.dpk` 不解决 `/data` 写权限** → 应用仍以 `uid=1000` 运行 `[V]`
- **Waypoint V2 用于 M4T** → 官方文档明文"仅支持 M300 RTK 和 M350 RTK" `[V]`
- **兴趣点环绕（`DjiInterestPoint_*`）可控半径** → settings 里没有 radius 字段 `[V]`
- **"KMZ 对第三方负载没意义"** → **已证伪**：`gimbalRotate` 是独立于相机的动作，
  且支持 `gimbalYawRotateEnable` + `absoluteAngle` `[V]`
- **`tar --mtime='@0'`** → **已证伪**：导致 make 跳过编译（见上）`[V]`
- **"模拟器不实现航点启动"** → **已证伪（证据不足）** `[X]`（2026-09-21）：
  该结论建立在 `0x000000FF` 上，而那是被 `(unsigned)` 截断后的假象；
  且当时那份 KMZ 确实缺 wpml 必需元素，**从未通过过内容校验**。
  详见「上机已确认与仍待确认」第一条。
- **`-DPSDK_ROOT=~/path`** → **已证伪** `[V]`：`~` 不被展开，用 `$HOME`（见上）

## 已知坑（本项目特有）

- **方位角不能用绝对差比较** `[V]`（2026-09-18）——正北方向处球面计算返回
  `359.999999998°`，与 `0°` 只差 2e-9°，绝对差却是 360。测试须用
  `LZ_CHECK_ANGLE_NEAR`（圆周差），见 `tests/lz_test.h`。
- **`LzGeo_NormalizeDeg` 的区间是 [0,360) 左闭右开** `[V]` ——极小负数加 360
  会因舍入得到恰好 `360.0`，函数里已收回；改动时别删那个判断。
- **`exception` 白名单式判据害人** `[V]`（2026-09-22）—— 详见「激光测距」一节。
  取值域没有权威来源的字段，别用枚举白名单，改用能自证的量。
- **判据对 ≠ 接线对** `[V]`（2026-09-22）—— 给激光零解闸门写的测试只断言
  `LzGeo_IsNullSolution()` 判据本身有效，**没断言取数函数真的调用它**。
  反向验证时删掉闸门，测试**照样全绿** —— 因为未定义 `LZ_POLE_SOURCE_LASER`
  时整个激光实现分支不参与编译，测试看不见它。
  **修法**：把判定抽成零依赖的纯函数（`LzPole_JudgeLaserReading`）放在
  `#ifdef` **之前**，测试直接打它。
- **`(unsigned)rc` 会丢掉 PSDK 返回码的模块号** `[V]`（2026-09-21）——
  `T_DjiReturnCode` 是 `uint64_t`，模块号在高 32 位
  （`DJI_ERROR_MODULE_INDEX_OFFSET = 32`）。用 `(unsigned)` 截成 32 位后，
  日志里那个"小错误码"可能根本不是你以为的那个。用 `%llX` 配
  `(unsigned long long)`。
- **`-DPSDK_ROOT=~/path` 里的 `~` 不会被展开** `[V]`（2026-09-21）——
  波浪号展开只认词首与 `=`/`:` 之后，而 `-DVAR=~/...` 整是一个词。
  CMake 把它当字面目录名，报的是"找不到静态库"（病因与提示不符）。用 `$HOME`。
  ⚠️ 改了 `-D` 参数必须重跑 `cmake` 配置：`CMakeCache.txt` 会记住旧值。
- **SDK 的日志流在我们手上路过** `[V]`（2026-09-21）—— PSDK 不把真正的错误码
  通过 API 给我们（`DjiWaypointV3_Action` 只回 `0x000000FF`），但飞机给的原因在
  SDK 自己的 `USER_LOG_ERROR` 里，而那个 console 是我们注册的。
  实现见 `app/platform/lz_sdk_log_watch.c`：两个 console **先喂再输出**，
  自己拼行（`ConsoleFunc` 的 `dataLen` 是任意长度，可能给半行 ——
  直接对 chunk 做 strstr 会在跨 chunk 时漏掉目标串）。
  **抓不到要返回 NULL 并让调用方容忍"没有这条信息"，不能把"抓不到"当成"没失败"。**
- **注释里不能写 `*/`** `[V]`（2026-09-20）——在 `/* ... */` 块注释里写
  `widget_file/*/` 会**提前闭合注释块**，后面整段代码变成语法垃圾。报错位置
  和病因完全对不上。
- **`-1` 会满足 `<= 1`** `[V]`（2026-09-19）——探针里 `watchMode(-1)` 的判断
  写在 `samples <= 1` 之后，导致连续监视模式掉进单次分支。**负值哨兵与
  正数阈值共用变量时，判断顺序决定成败。**
- **输出重定向到管道/文件时 stdio 是全缓冲** `[V]`（2026-09-19）——实时监视
  类程序必须每行 `fflush(stdout)`，否则攒够 4 KB 才可见，"实时"变成"批次"。靠
  `sshd` 转发时同样会踩到。
- **gdb 抓 PSDK 进程要先 `handle SIG32 nostop noprint pass`** `[V]`
  （2026-09-20）——PSDK 的 linker 线程用 SIG32 做实时事件通知，gdb 默认会
  停在它上面，真正的 SIGSEGV 反而看不到。用法：
  `gdb -batch -ex "handle SIG32 nostop noprint pass" -ex run -ex bt --args <bin> <args>`
- **诊断程序必须在读之前就打印并 flush** `[V]`（2026-09-20）——崩溃会吞掉
  stdio 缓冲区里没刷出去的内容，而那几行恰恰是定位崩溃点的关键。

## API 怎么查

不要凭记忆写 PSDK API。权威原文在 `~/projects/.psdk-apiref/`，
协议见 `LEARNING-PROTOCOL.md`，其中 §7 有**豁免规则**（用户 2026-09-18 制定）：

1. 空指针检查、返回值是否 `== DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS` —— 免检
2. **函数签名（参数个数/类型/顺序）一律 grep，不豁免**
3. 其他 PSDK 项目已 `[V]` 验证的代码照抄免检，**前提是两边版本号相同**

本项目用到的 PSDK 模块（头文件已核实存在 `[V]`）：`dji_liveview.h`（取图）、
`dji_waypoint_v3.h`（KMZ 航点）、`dji_interest_point.h`（绕飞，路线 A）、
`dji_fc_subscription.h`（飞机数据）、`dji_core.h`（初始化）。

**已发现的文档与头文件不一致** `[V]`：`dji_interest_point.h` 的枚举是
`DJI_INTEREST_POINT_**MISSION_**ACTION_STATE_*`，而官方文档漏了 `MISSION`。
照文档写会编译失败。**头文件 > 文档。**

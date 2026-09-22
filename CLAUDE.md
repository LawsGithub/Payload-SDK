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

## 绕飞：已定路线 —— Waypoint V3（自建 KMZ）+ 逐点 gimbalRotate 绝对 yaw

**机型是 M4T + 妙算3，这直接推翻了最初的选择。** 官方文档原文（`[V]` 2026-09-18）：

| 功能 | 支持机型 |
|---|---|
| Waypoint 2.0 | "currently only supports **Matrice 300 RTK and Matrice 350 RTK**" |
| Waypoint 3.0 | "supports Matrice 30 Series, Mavic 3 Enterprise Series, Matrice 3D/3TD, **and subsequent models**" |
| POI 兴趣点环绕 | "supports the **Mavic 3 Enterprise series and subsequent models**" |

M4T 属 M4 系列（`DJI_AIRCRAFT_TYPE_M4T = 99`），即"后续机型"一档。
**所以 V2 排除（它只给 M300/M350），改用 V3。** 佐证：官方 V2 样例源码写着
`"Waypoint V2 sample only support M300 RTK"`，而 **V3 样例没有任何机型门禁** `[V]`。

### 关键发现：wpml 的云台动作支持 yaw 绝对角度 `[V]`

解包官方样例 KMZ，`gimbalRotate` 的参数里有：

```xml
<wpml:gimbalRotateMode>absoluteAngle</wpml:gimbalRotateMode>
<wpml:gimbalYawRotateEnable>0</wpml:gimbalYawRotateEnable>   ← 官方样例是 0
<wpml:gimbalYawRotateAngle>0</wpml:gimbalYawRotateAngle>
```

官方样例把 yaw 关掉了（它只要俯仰），**但字段存在**。绕飞把 yaw 置 1、
角度填绝对方位角，就是"光轴指向杆心"。

> **教训**：我上一轮判定"KMZ 对第三方负载没意义"是**错的** —— 当时只看了
> `takePhoto`/`fileSuffix` 那些相机参数，没注意 `gimbalRotate` 是**独立动作**。
> 判断一个能力缺失前，要把容器里所有动作类型列一遍
> （`grep -o 'wpml:[a-zA-Z]*' | sort -u`）。

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

**航点间距是弦长不是弧长** `[V]`：8 点 20 m 半径，单段弦长 15.307 m，
总路径 107.151 m，而整圈周长是 125.664 m（差 17%）。
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

**当前 KMZ 缺的元素里，只有 `gimbalHeadingYawBase` 是真正可疑的** ——
官方样例也缺它，但样例的 `gimbalYawRotateEnable` 是 **0**（只用俯仰），
而我们使能 yaw，`absoluteAngle` 正依赖"相对正北"这个声明。
**样例能飞不能用来给我们开脱：两者处境不同。**

## 上机已确认与仍待确认

**已确认（上机实测）**：

- 控件在 Pilot 2 **相机视图左侧"PSDK"菜单**里显示并能触发回调 `[V]`
  （2026-09-20，当时 3 个控件；2026-09-22 加到 5 个后按钮同样触发）
- **KML/KMZ 上传全链路通**：41 个分片 + `Check kmz file md5sum success` `[V]`
- **航点任务能真正启动** `[V]`（2026-09-22 室外首次成功）—— 见下方
  「绕飞启动曾经失败的原因」
- **激光测距可用** `[V]`（2026-09-22）：记下 `28.1788176, 112.9210889`
  （距离 4.4 m），与「飞机位置 + 距离」反算一致
- **记录飞机位可用** `[V]`（2026-09-22）：`28.1788120, 112.9210473`

**仍待确认**：

- [ ] **绕飞完整链路（拨开关 → 飞机绕记录的点飞一圈 → 返航）**
      —— 这是**唯一还没跑完的环节**。所有中间环节都验过了，
      但"记录圆心 → 拨开关 → 飞机真的绕圈"这一整条链从未走通
- [ ] `droneEnumValue=99` / `payloadEnumValue=89`（M4T）是否被飞机接受
- [ ] 逐点 `gimbalYawRotateAngle` 实测能否驱动云台指向杆心
- [ ] **POI（`DjiInterestPoint_*`）在 M4T 上是否可用** —— 文档说"及后续机型"
      也覆盖 M4T；若可用且能接受半径不可控，它比自建 KMZ 省事得多
- [ ] 运动规划是否需要额外申请 PSDK 高级权限？矩阵里个别功能有此标注，未确认
- [ ] **POI（`DjiInterestPoint_*`）在 M4T 上是否可用** —— 文档说"及后续机型"
      也覆盖 M4T；若可用且能接受半径不可控，它比自建 KMZ 省事得多，**值得先试**
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
本会话实测：

| 回退的修复 | 变红的断言 |
|---|---|
| wpml 三元素（`globalRTHHeight`/`yaw` 归一/…） | 12 条 FAIL |
| 安全包线检查 | 6 条 FAIL |
| 零检查守卫 | 单跑空测试退出码 1 |

**不这么做的代价**：断言可能因为某个前置条件恒假而从未真正执行过 ——
那正是本项目已经踩过的形状（`LZ_TODO_PENDING`、`if (f.xxx != NULL)` 守卫）。

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
控件 0  switch  绕飞       OFF=停止(STOP 非 PAUSE)   ON=上传航线并开始
控件 1  scale   半径m      0-100% → 5~20 m（默认 50% → 12.5 m）
控件 2  scale   高度m      0-100% → 5~120 m（默认 66% → 约 80.9 m）
控件 3  button  记录飞机位  按下即记录飞机当前位置为绕飞圆心
控件 4  button  记录激光点  按下即记录激光瞄准点为绕飞圆心
```

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
-DLZ_VISION_BACKEND=hsv           src/lz_vision.c，真算法，连通域未实现
```

两者都定义 `LzVision_Detect`，**同时编会符号冲突**，CMake 里二选一。

**为什么现在默认 stub**：激光测距已实测可用，视觉层的职责可能从"解算绝对
坐标"降级为"确认激光瞄准点是不是杆"—— 那用不着连通域。**在职责定下来之前
把视觉算法做深，是白做。**

⚠️ **视觉层目前根本没接进主链路** `[V]`（2026-09-20 grep 确认）：
`app/lz_vision_source.c` 里 `LzVisionSource_Start/Stop()` 只有声明、没有实现，
也没有任何地方调用。所以**程序不看图，也感知不到"有没有红旗"**。

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

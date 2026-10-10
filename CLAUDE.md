# CLAUDE.md — liangzhourenwu（视觉航点规划）

本文件是**项目专属**约束。仓库级规则（设备、硬规则、worktree 布局）见上一级
`CLAUDE.md`（提交在 `master` 上，每个分支都继承）——**先读那一份**。
本机专属约定（设备连接、ssh 凭据）在 `~/projects/CLAUDE.md`。

工作区：`~/projects/liangzhourenwu/`（分支 `feature/liangzhourenwu`），
代码在子目录 `lz/`。

> **本文件只留「现在该怎么做」的规则、决策与索引。** 推导过程、规范原文、
> 实测数字、反向验证记录都外置在 [`lz/doc/`](lz/doc/) 下 —— 见文末
> 「文档导航」。**要动某块代码前，先读对应文档。**

## Cross-session handoff

- On session start: read [HANDOFF.md](HANDOFF.md) fully, then summarize the previous
  session's goal, current state, and next step before proceeding.
- 信任标记：`[V]` = 交接时已用命令验证；`[?]` = 仅记忆未复核，当线索对待；`[X]` = 已证伪，别用。
- **漂移检查用 `git log --oneline -5` 的实际输出比对**，别写死 `HEAD~N` ——
  handoff 之后可能又落了若干提交，写死的判据会静默失效（踩过）。
- ⚠️ **初始化时不要主动探测设备是否在线**（用户 2026-10-10 要求）—— 设备状态是
  反复变化的外部条件，只在**真要上机时**（编译打包、装 dpk、现场验证）才探。
  探不通时判据是 `ping` + `ip neigh`，**不要用 `/dev/tcp` 扫网段**。

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
              └─ lz_align.c   视觉照准的**判据**（接线在 lz_app/lz_visual_align.c）
lz_vision    视觉，两个互斥后端（stub 占位 | hsv 真算法）     ← 也是零依赖
lz_app       机载应用，依赖 PSDK                            ← -DLZ_BUILD_PSDK_APP=ON
  ├─ platform/         平台层注册（移植自官方样例）
  ├─ lz_pole_source    绕飞圆心从哪来
  ├─ lz_widget         Pilot 2 控件（操作员入口）
  ├─ lz_visual_align   视觉照准的**接线**（取帧/转云台），判据在 lz_core
  ─ lz_mission         作业状态机（**所有决策集中在此**）
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

**三条跨模块纪律**（都踩过，细节见对应文档）：

1. **判据抽进 `lz_core`** —— 写在 PSDK 侧 = 桌面测不到 = 没有测试。
2. **"设置型 SDK 调用"改的是飞机上的全局状态** —— 借了必须还，且还得能被检查到。
3. **画面比例与视场角必须随倍率现算** —— 绝对常量在变焦下失真（四处实例）。

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

完整的规范引用、闭合/收尾/弧线的推导、反向验证记录、wpml 必需元素、
以及「绕飞链路首次飞通」的实测日志，全部在
[`lz/doc/WPML-ORBIT.md`](lz/doc/WPML-ORBIT.md)。

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
2026-09-22 改掉 —— 见 [`lz/doc/WPML-ORBIT.md`](lz/doc/WPML-ORBIT.md)
的「绕飞怎么让相机盯着杆」。现在它不再是"云台独立偏转"，
而是与机头目标角一致的从属值。

## 上机已确认与仍待确认

**已确认（上机实测）**：控件在 Pilot 2 相机视图左侧「PSDK」菜单里可触发；KML/KMZ
上传全链路通（41 分片 + `Check kmz file md5sum success`）；航点任务能真正启动；
激光测距与记录飞机位可用；**绕飞完整链路已跑通**（2026-09-28 真机，记录圆心 →
拨 ON → 航点 1…9 → 正常结束）；`towardPOI` 生效（操作员目视机头始终对圆心）；
收尾点被接受（8 航点跑出 1…9）。实测日志见
[`lz/doc/WPML-ORBIT.md`](lz/doc/WPML-ORBIT.md)。

**仍待确认** —— **唯一真值是现场执行清单
[`lz/doc/ONDEVICE-CHECKLIST.md`](lz/doc/ONDEVICE-CHECKLIST.md)**（每项带判据、
观察点与回退方案），别在这里另抄一份。下面只是"9.29 之后改的代码"里未复验的索引：

⚠️ **9-28 那次飞的是"改之前"的代码**（`useStraightLine` 还是 1、云台模式残留修复
与双轴照准都还没有）⇒ **「链路通了」推不出后面几处改动生效了。**

- [ ] `useStraightLine=0` 是否真走弧线（9-29 改，**未上机**；9-28 飞的是直线模式）
- [ ] **云台模式用完没还的修复**（10-01 报的偏航顶限位/电机异常，10-05 改完，**未复验**）
- [ ] 双轴照准（云台偏航 + 俯仰）—— 9-29 改完，**未上机**
- [ ] `gotoFirstWaypoint` 是否真的飞回起始点（取值被接受，但无独立观测）
- [ ] 航点数输入框从未上机（在 **Payload Settings**，不在 PSDK 菜单）
- [ ] `droneEnumValue=99` / `payloadEnumValue=89`（M4T）是否被接受
- [ ] 云台 yaw 是否真与 `aircraftHeading` 一致（逐点 `gimbalRotate` 已关，**未复验**）
- [ ] POI（`DjiInterestPoint_*`）在 M4T 上是否可用
- [ ] `SetMode(FREE)` 是否被接受且生效（判据：飞机 yaw 转时画面跟不跟转）
- [ ] 云台角 `.z` 是不是 yaw；`GetOpticalZoomParam` 报的是光学还是总倍率
- [ ] 目标高度那行「高程自洽校验」的三个海拔数（决定参考面是否一致）
- [ ] 运动规划是否需要额外申请 PSDK 高级权限

## Pilot 2 控件：操作员的入口 `[V]`

`app/lz_widget.c`（控件）+ `app/lz_mission.c`（作业状态机）+ `app/widget_file/`（配置）。

**控件分两个界面** —— 归属由**语义**决定，不是随意放的 `[V]`（2026-09-28）：

```text
main_interface（Pilot 相机视图左侧「PSDK」菜单 —— 飞行中操作）
  控件 0  switch        绕飞       OFF=停止(STOP 非 PAUSE)   ON=上传航线并开始
  控件 1  button        记录飞机位  按下即记录飞机当前位置为绕飞圆心
  控件 2  button        记录激光点  按下即记录激光瞄准点为绕飞圆心
  控件 3  button        识别目标    视觉照准：看画面 → 转云台，把目标对到画面中心

config_interface（Pilot 的 Payload Settings「负载设置」—— 起飞前配置）
  控件 4  scale         半径m      0-100% → 5~20 m（默认 50% → 12.5 m）
  控件 5  scale         高度m      0-100% → 5~120 m（默认 66% → 约 80.9 m）
  控件 6  int_input_box 航点数      直接填个数，夹到 3~64（默认 8）
```

⚠️ **`int_input_box` 只能放在 `config_interface`** `[V]`。
放进 `main_interface` 时 SDK **收下它（解析计数正常、不报错）但 Pilot
不渲染成可编辑控件** —— 判据是它的回调**全历史零命中**（`type=5`）。
官方样例的 `main_interface` 里根本没有这个类型。
**"看得见但改不了"就是这个形状。**

⚠️ **两个界面的索引共用一个序列、不重叠**（0-3 与 4-6）——
**SDK 的 handler 表是扁平的、不区分界面**，重叠会让分派指向错的回调。

⚠️ `widget_file/` 下**每 7 个控件**（`cn_big_screen` / `en_big_screen` 两份），
且两份必须"类型、索引、数量都相同"（见下方四条约束第 3 条）。
**图标文件必须真实存在** —— `icon_visual_align.png` 曾漏生成而配置已引用它，
表现是 Pilot 2 显示破图、**设备上没有任何提示**。用
`bash lz/tools/precheck.sh` 查（见「本机自检」一节）。

**控件在 Pilot 2 的相机视图左侧「PSDK」菜单**里，**不在妙算3 应用管理面板** `[V]`
（2026-09-20 实测；应用管理面板里那个按钮只是 `dji_app_ctl start`）。

**配置目录路径不能写死** `[V]`：装成 dpk 后 CWD = 包根（`widget_file/` 在包根），
从源码树跑时 CWD = `lz/`（`app/widget_file/`）。`lz_widget.c` 用运行时解析两个候选。

**控件是基础功能**（`basic-function/widget.html`），比航点那条高级功能的路好走
—— 没有"是否需要申请 PSDK 高级权限"的不确定性。

**架构要点**：控件回调**只置标志、不做动作**，真正的作业决策集中在
`LzMission_Tick()` —— 判据在 `lz_core` 的 `lz_mission_logic.c`，
`lz_mission.c` 只剩「读输入 → Decide → 调 SDK → Apply → 呈现」。

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
3. 不同语言的配置**必须有相同的控件类型、索引和数量** ——
   且**跨 main/config 两个界面的索引必须连续不重复**（handler 表是扁平的）
4. 浮窗消息带宽上限 **2 KB/s**

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

## 本机自检：`lz/tools/precheck.sh`（秒级，不需要设备）`[V]`

四类**"能编译、能跑、但不生效"**的坑压成一条命令：

| 检查 | 拦什么 | 不查会怎样 |
|---|---|---|
| 控件配置一致性 | 两份 json 的类型/索引/数量不一致、索引不连续、**引用的图标文件不存在**、与 `lz_widget.c` 的 `LZ_WIDGET_IDX_*` 对不上 | **"按了 A 按钮执行 B 的动作"**，两边都不报错；图标缺失表现为 Pilot 2 显示破图，**设备上没有终端、没有任何提示** |
| 回调里不得有阻塞调用 | 把 `LzWidget_SetWidgetValue` 的函数体抠出来、**剥掉注释**后搜禁用符号 | PSDK 工作线程被拖死 → 进程闪退（2026-09-22 踩过） |
| **判据真的被接线调用了** | `lz_mission.c` / `lz_visual_align.c` 里是否存在对 `lz_align.c` 各判据的调用；互斥**逐方向**查实参形态 | 「**判据对 ≠ 接线对**」—— 本项目已踩过一次（激光零解闸门：判据有测试守着，但取数函数根本没调它，反向验证时删掉闸门测试照样全绿） |
| 两个后端都编都跑 | stub 与 hsv 各自的构建目录 + ctest | "切了后端却没编到"这件事本身看不出来 |
| `app.json` 四语言字段 | `name_*` / `description_*` 八项 | `build_dpk.sh` 直接 `KeyError` 退出 |

⚠️ **第 3 项是"逐方向"的，不是数次数** `[V]`（实测逼出来的）：最初写成
`grep -c 'LzAlign_CheckConflict'`，把绕飞那一侧的实参换成常量、只留调用形式，
它照样数到 2 处、**绿着放过**。改成查两个方向各自的实参形态
（`LzMission_IsRunning()` / `LZ_ALIGN_RUNNING`）之后，两个方向**分别**
拆掉都会变红。**与「连通域去掉 union 测试照样全绿」是同一个形状 ——
检查要能区分"这个方向做了"与"看起来做了"。**

**"引用的图标文件"这一项是补上的** `[V]`（2026-09-27）——「识别目标」按钮的
`icon_visual_align.png` 两处都没生成，而 `widget_config.json` 已经引用了它。
**这是本轮唯一一个"编译、测试、上机都不会报错"的缺陷**，只有脚本能抓住。

⚠️ 剥注释是必须的：说明文字里恰好会提到这些函数名（如"要调
`DjiCameraManager_GetLaserRangingInfo`"），不剥的话每行注释都是假阳性。

反向验证（`[V]` 2026-09-27 实测）：往回调里塞一行
`LzBridge_GetCurrentPosition(NULL)` → 第 2 项变红；移走
`icon_visual_align.png` → 第 1 项变红；把 en 的 index 6 改成 7 → 第 1 项报两条
（两语言不一致 + 索引不连续）。**三条都变红，脚本本身也是测过的。**

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

## 文档导航（细节都在这里）

| 文档 | 什么时候读 |
|---|---|
| [`lz/doc/WPML-ORBIT.md`](lz/doc/WPML-ORBIT.md) | 动 KMZ 生成：闭合 / 弧线 / `finishAction` / `towardPOI` / wpml 必需元素 / 启动失败史 |
| [`lz/doc/VISION.md`](lz/doc/VISION.md) | 动视觉：后端切换 / 杆列与端点检测 / 变焦折算 / 双轴照准 / 照准闭环 |
| [`lz/doc/GIMBAL.md`](lz/doc/GIMBAL.md) | 动云台：俯仰限位 −90°~70° / 模式借用与归还 / `TOPIC_GIMBAL_STATUS` / 几何反算俯仰 |
| [`lz/doc/LASER-AND-POLE.md`](lz/doc/LASER-AND-POLE.md) | 动激光或绕飞圆心来源（三种取圆心方式、三道闸门、目标高度） |
| [`lz/doc/CORE-TESTING.md`](lz/doc/CORE-TESTING.md) | 写测试 / 把判据抽进 `lz_core` / 跑 ASan+aarch64+覆盖率三样自检 |
| [`lz/doc/BUILD-DEPLOY.md`](lz/doc/BUILD-DEPLOY.md) | 打包、传源码上设备、查「已知坑」与「已证伪的路」 |
| [`lz/doc/VISION-GIMBAL-PITCH.md`](lz/doc/VISION-GIMBAL-PITCH.md) | 视觉调俯仰的完整方案；§12 更正「PSDK 不能转自带云台」 |
| [`lz/doc/ONDEVICE-CHECKLIST.md`](lz/doc/ONDEVICE-CHECKLIST.md) | **上机前**必读：现场操作单，待确认项的唯一真值 |
| [`lz/doc/README.md`](lz/doc/README.md) + [`lz/doc/wpmz/`](lz/doc/wpmz/) | 航点文件（KMZ/KML/wpml）格式参考 |
| [`HANDOFF.md`](HANDOFF.md) | 会话开始（规则见本文件顶部） |

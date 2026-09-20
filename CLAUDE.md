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
      -DPSDK_ROOT=~/projects/Payload-SDK
cmake --build build-x64 -j4 && ./build-x64/bin/lz_app
```

飞机不在手边时也能验证「程序能不能起来」—— 链接是否通过、启动路径是否正常退出
（后者正是 `dji_app_ctl install` 会试运行的那段）。**不替代设备上的 aarch64 编译。**

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

## 待确认（开工后第一件事）

- [ ] **运动规划是否需要额外申请 PSDK 高级权限**？矩阵里个别功能有此标注，未确认
- [ ] `droneEnumValue=99` / `payloadEnumValue=89`（M4T）是否被飞机接受
- [ ] 逐点 `gimbalYawRotateAngle` 实测能否驱动云台指向杆心
- [ ] 控件上机验证：Pilot 2 里能否看到三个控件、拨动是否触发回调
- [ ] 激光模式下取到的 `lat/lon` 是否真为飞机所在位置（需室外有 GPS）
- [ ] **POI（`DjiInterestPoint_*`）在 M4T 上是否可用** —— 文档说"及后续机型"
      也覆盖 M4T；若可用且能接受半径不可控，它比自建 KMZ 省事得多，**值得先试**
- [x] ~~`lz_credentials.ini` 尚未创建~~ → **已从 wt_inspection 拷入**（2026-09-19）
- [x] ~~激光测距在哪个挂载位置~~ → **位置 1，已实测可用**（2026-09-19）
- [x] ~~视觉层用什么方案~~ → **暂用 stub 占位，等圆心来源定下来再做**

## 激光测距：已实测可用 `[V]`（2026-09-19）

**位置 1**（`DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1`，M4T 自带云台相机那一路）
就是激光测距所在，`enable_lidar=1`。实测断联后重连仍成立。

**`exception` 取值官方未公开** `[V]`（头文件与中英文档都只有"异常标志"四字），
靠对照实验反推：

| exc | 含义 | 判据 |
|---|---|---|
| **1** | **无回波/测不到** | 静止时恒为 1，`distance=0`，`lat/lon=0` |
| **3** | **正常读数** | 一有目标立刻变 3，`distance` 随实际距离变化 |
| 2 | 过渡态，含义不明 | 夹在 1 和 3 之间 |

**取坐标前必须判 `exception`** —— 否则会把"无回波时的 `0,0`"当成真坐标，
生成的航线会指向几内亚湾。实现见 `lz/app/lz_pole_source.c`。

**`distance` 单位是 0.1 m**（分辨率 0.1 m），所以读数恒定不代表没工作 ——
真实距离落在同一 0.1 m 区间内读数就该恒定。**判"测距是否工作"要看
读数是否随目标移动而变化，不是看它有没有抖动。**

查挂载位置的探针：`lz/app/lz_rangefinder_probe.c`
（`./lz_rangefinder_probe` 扫全部位置；`./lz_rangefinder_probe 1 -1` 连续监视）

## Pilot 2 控件：操作员的入口 `[V]`

`app/lz_widget.c`（控件）+ `app/lz_mission.c`（作业状态机）+ `app/widget_file/`（配置）。

```text
控件 0  switch  绕飞     OFF=停止(STOP 非 PAUSE)   ON=上传航线并开始
控件 1  scale   半径m    0-100% → 5~30 m
控件 2  scale   高度m    0-100% → 5~40 m
```

**控件是基础功能**（`basic-function/widget.html`），比航点那条高级功能的路好走
—— 没有"是否需要申请 PSDK 高级权限"的不确定性。

**架构要点**：控件回调**只记状态不做动作**，真正的作业决策集中在
`LzMission_Tick()`。理由是回调跑在 PSDK 工作线程上，而上传 KMZ 是耗时操作。

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

| 方式 | 开关 | 现状 |
|---|---|---|
| **固定坐标** | 默认 | 写死的占位值，室内联调用 |
| **激光测距** | `-DLZ_POLE_SOURCE_LASER=ON` | 真实路径，需 GPS 锁定 + 激光打中杆 |

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

## 平台层已抽出共用模块 `[V]`

`lz/app/platform/`（`lz_platform.c` + `lz_user_info.c`，共 ~330 行）——
移植自官方样例 `DjiUser_PrepareSystemEnvironment()` 与 `DjiUser_FillInUserInfo()`，
逻辑未改。

**为什么要抽**：这段有近 150 行 handler 样板，每个 PSDK 应用都得原样做一遍。
抽出后 `lz_app` 与 `lz_rangefinder_probe` 共用一份，避免"探针能跑、主应用不行"
这类由样板差异引起的怪问题。

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
# 设备上编译完成后
tools/build_dpk/build_dpk.sh lz/app_json
dji_app_ctl install <包名>.dpk
```

- 构建目录名必须是 `build-native`（`app.json` 的 `bin` 指向 `../build-native/bin/lz_app`）。
- 装包**需要飞机通电并连接**，且启动路径不能提前退出
  （见仓库级 CLAUDE.md 的「安装器会试运行应用」）。

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
     lz psdk_lib/include psdk_lib/lib/aarch64-linux-gnu-gcc \
     samples/sample_c/platform/linux/{manifold3,common}
```

⚠️ **不要用 `--mtime='@0'`** `[V]`（2026-09-19 实测踩到）—— 它把源码时间戳压到
1970，`make` 会发现**源码比 `.o` 旧**，于是**静默跳过编译**。现象是"传了新代码
上去，跑的还是旧二进制"，极难察觉。

正确做法：时间戳取**设备当前时间附近**（`date -d '+30 seconds'`）。
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

## 已知坑（本项目特有）

- **方位角不能用绝对差比较** `[V]`（2026-09-18）——正北方向处球面计算返回
  `359.999999998°`，与 `0°` 只差 2e-9°，绝对差却是 360。测试须用
  `LZ_CHECK_ANGLE_NEAR`（圆周差），见 `tests/lz_test.h`。
- **`LzGeo_NormalizeDeg` 的区间是 [0,360) 左闭右开** `[V]` ——极小负数加 360
  会因舍入得到恰好 `360.0`，函数里已收回；改动时别删那个判断。
- **注释里不能写 `*/`** `[V]`（2026-09-20）——在 `/* ... */` 块注释里写
  `widget_file/*/` 会**提前闭合注释块**，后面整段代码变成语法垃圾。报错位置
  和病因完全对不上。
- **`-1` 会满足 `<= 1`** `[V]`（2026-09-19）——探针里 `watchMode(-1)` 的判断
  写在 `samples <= 1` 之后，导致连续监视模式掉进单次分支。**负值哨兵与
  正数阈值共用变量时，判断顺序决定成败。**
- **输出重定向到管道/文件时 stdio 是全缓冲** `[V]`（2026-09-19）——实时监视
  类程序必须每行 `fflush(stdout)`，否则攒够 4 KB 才可见，"实时"变成"批次"。靠
  `sshd` 转发时同样会踩到。

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

# 绕飞航线的 wpml 细节（KMZ / 闭合 / 弧线 / 相机朝向）

> 从 `CLAUDE.md` 外置（2026-10-10）。这里放**为什么这么定**的推导、规范原文引用、
> 实测数字与反向验证记录。CLAUDE.md 只保留结论与索引。
> 信任标记：`[V]` 已用命令验证 · `[?]` 仅记忆未复核 · `[X]` 已证伪。

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
| `LZ_TURN_PASS_WITH_CURVE` | `toPointAndPassWithContinuityCurvature` + `useStraightLine=**0**` | 近似圆弧 |

后者就是 DJI Pilot 2 里「平滑过点，提前转弯」的设置方法
（规范 `40.common-element.md` 的 `waypointTurnMode` 一行有明确注解）。

⚠️ **`useStraightLine` 的语义曾读反** `[V]`（2026-09-29 修正）。
规范原文：`0：航段轨迹全程为曲线` / `1：航段轨迹尽量贴合两点连线`。
原实现写 1（"贴直线"= 内接正多边形），却在注释里称它"就是 Pilot 2 的
平滑过点" —— **把两件事混成了一件**：

```text
waypointTurnMode  → 过点**停不停**
useStraightLine   → 两点之间**走直线还是曲线**（0 = 曲线）
```

⇒ 常量 `LZ_WPML_USE_STRAIGHT_LINE = 0`（`lz_wpml.h`），两处落盘点都引它。
现场症状正是用户反馈的「到点瞬间机械地折一个角度」。
**同一处还有第二个成因**：逐点 `gimbalRotate` 的触发器是 `reachPoint`，
到点才执行 ⇒ 云台 yaw 在航段中被"停在"上一个航点、每次到点才跳一次。
机头已由 `towardPOI` 连续跟随杆心，所以改成
`LZ_WPML_GIMBAL_YAW_IN_ACTION = 0`（元素仍照写，规范标为必需）。
⚠️ **两者都未上机验过**。

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

1. 规范里 **两处**（`orientedShoot` / `rotateYaw` 的 `aircraftHeading` 行）
   标着：

   > `wpml:gimbalYawRotateAngle` 与 `wpml:aircraftHeading` 需保持一致
   > 机型：M30/M30T，M3E/M3T，M3D/M3TD

   出处：Cloud-API-Doc `00.dji-wpml/40.common-element.md`。

   ⚠️ **本条曾被记错** `[V]`（2026-10-01 逐份核对规范原文更正）：
   旧记录写的是「三处都标」且机型列里带 **M4D/M4TD，M4E/M4T** ——
   而三份规范文件（`20.template-kml.md` / `30.waylines-wpml.md` /
   `40.common-element.md`）的机型列里 **`M4` 出现 0 次**（逐份 grep 核实）。
   那句注只列 M30/M30T、M3E/M3T、M3D/M3TD。
   ⇒ **M4T 不在那条约束的机型列里。**
   **这条更正不改变结论** —— 改走 `towardPOI` 是对的（理由 2 独立成立，
   且实测有效），但**不能再拿这条注当"M4T 的硬要求"来引**。

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
> 存在、官方样例也写了，而它的机型列只到 M3D/M3TD 那一档。
> **字段存在 ≠ 能按你的意思用。**
>
> ⚠️ **而这行引文本身也曾被记错**（见上方更正）：把不属于该档的机型
> 读进了机型列，于是得出"这是 M4T 的硬要求"这个**不成立**的推论。
> ⇒ 教训再深一层：**引规范时要把"引文"与"推论"分开**，
> 引文必须逐字可复核（给文件路径 + 可 grep 的原句），
> 否则一个抄错的机型列会伪装成权威，而后面所有推理都建在它上面。

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
2026-09-22 改掉 —— 见上文「绕飞怎么让相机盯着杆：靠**机头**，不是靠云台」。现在它不再是
"云台独立偏转"，而是与机头目标角一致的从属值。

### 绕飞链路首次飞通 `[V]`（2026-09-28 15:59–16:02 真机）

**「记录圆心 → 拨开关 → 飞机绕一圈」这条链在此之前的记录是"从未走通"**
（那行写在 2026-09-23），**2026-09-28 已飞通**。设备日志：

```text
16:00:26  记录激光点 28.1743615, 112.9220320
16:00:33  收到绕飞请求：半径 12.5 m，高度 80.9 m，8 个航点
16:00:33  KMZ 已读入内存：46121 字节
16:00:38  KMZ 上传完成
16:00:38  航点任务已启动（杆位取自已记录（激光点））
16:00:57  绕飞中：航点 1
   ...
16:02:49  绕飞中：航点 9        ← 8 个方位点 + 与首点重合的收尾点
16:02:50  绕飞结束：已结束（回到 IDLE）
```

**这次顺带独立证实了三件事**：

| 结论 | 证据 |
|---|---|
| `towardPOI` 在 M4T 上生效 | 操作员目视：「无人机确实是一直朝向我打点的位置（圆心点）」 |
| 收尾点被接受 | 8 航点配置跑出 航点 1…9 —— 第 9 个就是坐标与首点完全相同的收尾点 |
| `gotoFirstWaypoint` 被接受 | 任务正常结束、未报错（**但"是否真的飞回起始点"没有独立观测**，见待确认清单） |

⚠️ **这次飞的是"改之前"的代码** —— `useStraightLine` 当时还是 1（直线模式）、
云台模式残留修复与双轴照准都还没有。**"链路通了"推不出"后面四处改动生效了"。**
另外这一轮暴露的两个问题（浮窗被 716 条刷屏堵死控件、逐点 `gimbalRotate`
把云台调歪）当场定位并修掉，改动同样**未复验**。

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

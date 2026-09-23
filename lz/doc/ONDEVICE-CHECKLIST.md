# 上机执行清单 —— 绕飞完整链路

本文件是**现场照着做的操作单**，不是设计文档。设计依据见
`~/projects/liangzhourenwu/CLAUDE.md`；每条待验证项都在那里有对应章节。

数值来自当前代码（`lz/include/lz_plan.h` / `lz/app/lz_widget.c` /
`lz/src/lz_wpml.c`），**改动那些文件后要回来核对本文件的表**。

---

## 0. 出发前（在设备上，不在本机）

```bash
# 让出 PSDK 通道 —— 有进程就停掉（退出码 1 = 无进程 = 通道可用）
pgrep -x Smart3DExplore >/dev/null || /system/bin/dji_app_ctl stop Smart3DExplore

# 设备本机编译（绝不交叉编译）
cd ~/lzbuild && cmake -S lz -B build-native && cmake --build build-native -j4

# 打包安装（装包需要飞机通电并连接）
tools/build_dpk/build_dpk.sh -i lz/app_json/app.json -o ~/dpk
dji_app_ctl install -i ~/dpk/liangzhourenwu_v01.00.00.00.dpk
```

传源码用 `--mtime="-60 seconds"`，**必须带 `--exclude='lz/build*'`**，
且不能用 `--mtime='@0'`（会让 make 静默跳过编译）。完整命令见 CLAUDE.md
「设备侧打包清单」。

**装完先确认它真的起来了**：应用日志里应出现
`Update dji sdk policy file successfully`（`dji_identity_verify.c:654`）——
没有这行说明身份校验没走完，安装器会把它误报成
`Error, verify app user_app_id or version info error`。

日志位置：`/blackbox/system/app_temp_files/liangzhourenwu_*.log` `[?]`
（文件名前缀按 `app.json` 的 `name_en`，**未实测**；找不到就 `ls` 该目录看实际名）。

---

## 1. 控件对照表（Pilot 2 相机视图左侧「PSDK」菜单）

| index | 类型 | 名称 | 出厂默认 | 映射 |
|---|---|---|---|---|
| 0 | switch | 绕飞 | OFF | OFF=STOP 急停；ON=上传航线并开始 |
| 1 | scale | 半径m | 50% | 0–100% → **5–20 m**（50% → 12.5 m） |
| 2 | scale | 高度m | 66% | 0–100% → **5–120 m**（66% → 约 80.9 m） |
| 3 | button | 记录飞机位 | — | 把飞机当前位置记为圆心 |
| 4 | button | 记录激光点 | — | 把激光瞄准点记为圆心 |
| 5 | int_input_box | 航点数 | 8 | **直接填个数**，夹到 3–64 |

**控件不在妙算3 应用管理面板里**（那里那个按钮只是 `dji_app_ctl start`）。

---

## 2. 航线参数（用于对照观察，不要照抄去改代码）

| 项 | 当前值 | 出处 |
|---|---|---|
| 航点转弯模式 | `toPointAndPassWithContinuityCurvature` + `useStraightLine=1` | `lz_mission.c` 的 `LZ_TURN_PASS_WITH_CURVE` |
| 提前转弯截距 | 最短段的 45%（8 点 20 m 半径 → **6.888 m**） | `LzPlan_SuggestDampingM()` |
| 机头朝向 | `towardPOI`，兴趣点 = 记录的杆位 | `lz_wpml.c` |
| 云台俯仰 | −15° | `lz_mission.c` 的 `gimbalPitchDeg` |
| 巡航速度 | 3.0 m/s | 同上 |
| 收尾动作 | `gotoFirstWaypoint` | `lz_wpml.h` 的 `LZ_WPML_FINISH_ACTION` |
| 返航高度 | `max(航线高度, 30)` | `lz_wpml.c`（手动/失控返航时生效） |
| 机型/负载枚举 | `droneEnumValue=99` / `payloadEnumValue=89` | `LzWpml_DefaultIdentity()` |
| 航点总数 | `waypointCount + 1`（末尾补了与首点**坐标完全相同**的收尾点） | `lz_plan.c` |

**8 点 20 m 半径的几何**（对照 Pilot 显示的里程）：
单段弦长 **15.307 m**、总路径 **122.459 m**、整圈周长 125.664 m。
Pilot 显示里程若接近 125.7 说明它按周长算（错），接近 122.5 才对。

---

## 3. 待验证项（按顺序做，每项写判据）

### 3.1 航点数输入框能否用 —— 5 分钟内可验

- [ ] 在 Pilot 2 上把「航点数」填 **16**，浮窗应回
      `航点数 16`（未夹取时不回调整消息）。
- [ ] 填 **100**，浮窗必须回
      `航点数 100 超出 3–64，已按 64 使用`。
      回落消息说明夹取闸门活着；**不吭声才是 bug**。
- [ ] 填 **2**，应回 `已按 3 使用`。

判据：浮窗出现对应提示，且拨开关后浮窗里报的「N 个航点」与夹取后的值一致。

### 3.2 绕飞完整链路 —— 核心目标，唯一没跑通的一环

前提：室外、等 **GPS 3D Fix**（`fixState=3`）、手动起飞、飞到杆附近。

- [ ] 按「记录飞机位」或「记录激光点」记圆心
      （激光要打中目标）。核对：
      ```bash
      cat data/pole.txt      # lon=... lat=... alt=... src=aircraft|laser
      ```
- [ ] 未记录就拨 ON → 浮窗应拒绝：
      `尚未记录绕飞圆心 —— 请先在 PSDK 控件里按…`（`LZ_ERR_NOT_READY`）。
- [ ] 拨 ON → 观察顺序：**先爬升到「高度m」设定值 → 平飞到首航点 →
      顺时针绕圈**。首航点在**杆的正北**，距离 = 「半径m」设定值。
- [ ] 拨 OFF → 急停（STOP）。**停完悬停不返航，会耗电** —— 及时接管。

失败时看日志里这两行（它们现在会被 `lz_sdk_log_watch` 捞出来）：
`启动被拒：<飞行状态> RC=<档位> GPS状态=<x> 卫星=<n>`，以及 SDK 自己打的
`error_code` 数值。**错误码用 `%llX` 看**，不要看截断的 32 位。

### 3.3 `towardPOI` 在 M4T 上是否生效 —— 首次上机

- [ ] 飞机绕圈时**机头始终朝着杆**（光轴对着圆心），机身是**侧飞**状态。
- [ ] 若机头不追杆：说明 `towardPOI` 在这台 M4T 上没生效。回退方案见
      CLAUDE.md「绕飞怎么让相机盯着杆」——那里已排除「云台独立偏航」这条路
      （M4T 云台 yaw 不可独立控制）。

### 3.4 是否真的走弧线（`LZ_TURN_PASS_WITH_CURVE` 首次上机）

- [ ] 航迹回放看轨迹：应是**一段段圆弧**，不是内接正多边形。
- [ ] 实测离杆距离应回到**设定半径附近**。直线模式下 8 点 20 m 半径
      实际只有 **18.48 m**（近 7.6%）；改弧线后应接近 20 m。
      这一条是区分"改了没生效"和"生效了"的**唯一硬判据**。

### 3.5 `gotoFirstWaypoint` 与收尾点 —— 都是首次上机

- [ ] 飞完一圈后，飞机**停在航线起始点**（杆的正北、距离 = 半径处）悬停，
      **不返航、不降落**。
- [ ] 若飞机返航回起飞点：说明 `gotoFirstWaypoint` 未被接受（或被忽略）。
- [ ] 若启动就被拒、且换了别的改动也不通：**先怀疑收尾点**——
      末点与首点坐标完全相同（距离 0 m）从未实测。
      回退方案：把收尾点方位角偏 **0.01°**（改 `lz_plan.c` 的生成循环）。

### 3.6 云台 yaw 的实际行为

- [ ] 观察相机画面：机头对杆时画面是否**始终正对杆**。
      规范要求 `gimbalYawRotateAngle` 与 `aircraftHeading` 一致；
      我们写的值就是「该点看向杆心的方位角」，理论上天然一致 —— 需实测确认。

### 3.7 机型/负载枚举是否被接受

- [ ] 若启动报错且前置条件（飞行状态/RC/GPS/起飞点）全部正常，
      把 `droneEnumValue` 从 99 试成官方样例的 77（M3E）再看一次。
      这是**排除法**，不是首选方案。

### 3.8 备选路线：PSDK 兴趣点环绕（`DjiInterestPoint_*`）

- [ ] 若自建 KMZ 这条路反复不通，试 `DjiInterestPoint_*`：
      文档说覆盖"及后续机型"，若能接受**半径不可控**则比自建 KMZ 省事得多。
      注意头文件枚举名是 `DJI_INTEREST_POINT_**MISSION**_ACTION_STATE_*`，
      官方文档漏了 `MISSION`。

---

## 4. 已知噪声（不是故障，别去修）

- `cam` 模块刷屏 `data size = 66 > 49 is too large` —— SDK 内部警告
  （相机状态包比它预期的大），**与闪退无关**。它说明 M4T 与该 SDK 版本
  有真实不一致，但尚未深究，也还没影响到功能。
- `/blackbox/system/app_temp_files/` 是 `root:root 0755`，`dji` 用户**删不掉**
  里面的日志（也没有 sudo）。别把它当自己的地盘清。

---

## 5. 遇到问题时的排查顺序

1. **通道被占** → `pgrep -x Smart3DExplore` 有进程就
   `/system/bin/dji_app_ctl stop Smart3DExplore`。
   症状是误导性的"机型不匹配"报错，不是通道报错。
2. **启动被拒** → 看日志里的前置条件四行（飞行状态 / RC 档位 / GPS / 起飞点）
   + `error_code` 原始值（`%llX`）。
3. **进程闪退** → 优先怀疑「控件回调里做了阻塞调用」。
   症状：日志戛然而止于 `dji_msgq.c:227 semaphore wait timeout` +
   `dji_linker.c:309 send msg to queue error`。
4. **改完代码行为没变** → 回看是不是 `tar --mtime` 把时间戳弄反了，
   导致 `make` 静默跳过编译（传了新代码、跑的还是旧二进制）。

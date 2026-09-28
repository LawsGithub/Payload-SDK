# HANDOFF — 读全文再开始干活

生成时间: 2026-09-29T03:50:14+0800 · Git HEAD: 111d4b2
信任规则: [V] = 交接时已用命令验证；[?] = 仅记忆未复核，当线索对待；[X] = 已证伪，别用。

## 0. 复核（下一会话先做）

- 锚点: `feature/liangzhourenwu` @ `111d4b2`（2026-09-29 03:5x）
- 漂移检查: `git rev-parse HEAD~1` 应 = `111d4b2`（HEAD 是本次 handoff 提交，
  其 parent 是快照锚点）。权威以 `git log --oneline -5` 实际输出为准。
- **远端已同步到 `111d4b2`** `[V]`。推送**必须走代理**，直连挂起：
  ```bash
  GIT_SSH_COMMAND="ssh -o ProxyCommand='nc -X connect -x 127.0.0.1:7890 %h %p'" \
    git push fork feature/liangzhourenwu
  ```
- **设备在线** `[V]`：USB 扩展坞 `192.168.1.180`（本机 `eth3`）。
  时钟**与本机同步**（实测两者都报 03:5x，不再是 43.7 h / 2.4 h 的偏差）——
  但 `tar --mtime` 仍必须**从设备取时间**再减 60 秒。
- **飞机当前关机** `[V]` —— 这是下面"安装失败"的直接原因。**通电后重装即可。**
- `lz_app` **未在运行**、`Smart3DExplore` **已被停** `[V]`（通道已让出）。
- 先读: `~/projects/liangzhourenwu/CLAUDE.md`（本轮新增三节，**必须读**）
  + [`lz/doc/ONDEVICE-CHECKLIST.md`](lz/doc/ONDEVICE-CHECKLIST.md)（上机执行单 §3.12b）

## 1. 当前目标

**让「操作员按识别目标 → 红旗被锁到画面中心」在变焦下也稳定可用**，
并**让绕飞真的走弧线**。

本轮用户报的四个问题已全部改完代码（见 §2），**但全部只在桌面上验过**。
完成定义：上机按 §6 的顺序验过一遍，四个问题都不再复现。

## 2. 已验证状态 — 工作实际停在哪

### 代码改动（6 个提交，全部已推送）

| 提交 | 内容 |
|---|---|
| `4a516b3` | 视觉三项：窗口按旗高缩放 + 判别比置信度 + 旗面惩罚 5%→15% + 区分两种"没目标" |
| `d6d495b` | `useStraightLine` 语义修正（1→**0**）+ 云台 yaw 不再逐点跳 |
| `fd04f4a` | 照准按变焦倍数算视场角（俯仰增益原先差 **6.98 倍**） |
| `c7bbef8` | 目标高度 = 激光海拔 − 起飞点海拔（用户方案）+ 新订阅起飞点海拔 |
| `858d032` | 死区改成画面比例（固定 1.5° 在 7× 下等于画面高的 17.6%） |
| `e1b9b00` | 照准改双轴（横向偏航 + 纵向俯仰）+ 订阅四元数取机头朝向 + `SetMode(FREE)` |

### 一、根因：**「绝对常量在变焦下失真」一个形状四处实例** `[V]`

现场把画面放大到 **7.0X**，四处独立缺陷同源 —— 都写在"广角端"量出来的
绝对常量上。完整表格与反推证据见 CLAUDE.md 同名节。反推倍率 7.0× 与
Pilot 界面显示的「变焦 7.0X」吻合；**也解释了 15:31 那次为什么能跑通**
（只微变焦 1.34×，误差小到闭环能吃下）。

### 二、桌面验证 `[V]`（本次交接从全新 shell 跑，真实退出码）

```
cmake -S lz -B /tmp/hv -DLZ_VISION_BACKEND=hsv   rc=0
cmake --build /tmp/hv -j4                        rc=0，warning 0
ctest --test-dir /tmp/hv                         rc=0，10/10 passed
逐目标（rc 全 0）：
  align 164 ｜ plan 1144 ｜ validate 173 ｜ wpml 145 ｜ vision 135
  pole 78 ｜ vision_math 104 ｜ geo 34 ｜ kmz 24 ｜ bridge 12
bash lz/tools/precheck.sh                        rc=0（全部通过）
x64 PSDK 侧（-DLZ_TARGET_ARCH=x86_64）            0 warning
```

### 三、设备侧编译打包 `[V]`（**本轮已完成，不要重做**）

```
源码传到 /tmp/lzsrc（tar --mtime 取设备时间 −60 s）
cmake … -DLZ_VISION_BACKEND=hsv -DLZ_POLE_SOURCE_LASER=ON    ✅ 载入凭据 App ID 184842
cmake --build lz/build-native -j4                            ✅ 0 warning（SDK 侧 3 个既有）
产物 lz_app 只要求 GLIBC_2.17（设备 2.31）                    ✅
ctest（设备上 aarch64）                                       ✅ 10/10 passed
tools/build_dpk/build_dpk.sh → ~/dpk/liangzhourenwu_v01.00.00.00.dpk  ✅ 294708 B
```

### 四、⚠️ **安装失败** `[V]` —— 死因是**飞机没通电**，不是代码

```
dji_app_ctl install → APP INSTALL FAILURE
Error, verify app user_app_id or version info error
```

**这句错误是误导性的**（CLAUDE.md 有专门一节）。诊断过程：

1. 把新编的 `lz_app` **在设备上手动跑起来** `[V]` —— 它**正常启动并持续运行**
   至 15 秒超时被杀；日志里有 `Identify device : manifold3` +
   `Start dji sdk application`，无任何错误。
2. 日志里**没有** `dji_identity_verify.c:654 Update dji sdk policy file
   successfully` —— 那正是"走完 SDK 身份校验"的判据（CLAUDE.md 记过）。
   **飞机不在 ⇒ 身份校验走不完 ⇒ 安装器判失败。**
3. 用户确认飞机当前关机。

**现场记录的圆心没丢** `[V]`：`/open_app/liangzhourenwu/data/pole.txt` 仍是
`lon=112.9263555 lat=28.1712038 alt=50.4 src=laser`；旧应用也仍在
`bin/lz_app`。安装前已备份 data 到 `/tmp/lz_data_backup_1790623015`。

## 3. 决策与理由

- **横向可达性比「相对机头」的角，不比绝对方位角** `[V]` —— M4T 规格页
  原文 `Pan: Not controllable / The MSDK interface program is controllable /
  机械软限位 −60°~+60°`。pitch 那对常量（−90~70°）是**绝对角**，两者本质不同。
  比绝对角会**误拒**（机头朝东时转到 100° 被"距正北 100°>60"拦下）。
- **超限取 0 而不钳位**（目标高度、俯仰角）`[V]` —— 钳位会把"物理上做不到"
  或"明显错的量"伪装成"做得到"。与 `LzPlan_Validate` 拒绝而非钳位同一条纪律。
- **`deadzoneFrac` 是权威值、`deadzoneDeg` 降级为广角端等价初值** `[V]` ——
  同一条纪律：画面上的比例不该由角度常量承载。
- **两个后端都要实现 `LzVision_LastMiss`** `[V]` —— 头文件按"全集"声明的
  东西两个后端都得给，否则切后端就链接失败（`LzVision_DefaultConfig` 踩过）。
- **不做**：给"旗下方可见杆长"再加一条判据 —— 实测旗占画面 41% 时新规则
  判拒（conf 0.37），那是**合理的拒绝**（杆只剩一小截），不该放行。

## 4. 失败的尝试 — 不要再试

- **`DjiFcSubscription_GetLatestValueOfTopic`** `[?]` —— SIGSEGV，栈在
  `DjiDataSubscriptionDds_v3_GetLastValueOfTopic` 内部。一律走回调缓存。
- **在 `ApplicationStart()` 之前调 `DjiFcSubscription_Init()`** `[?]` ——
  返回 SUCCESS 但随后 SIGSEGV。**返回成功不代表调用合法。**
- **在控件回调里调任何阻塞接口** `[?]` —— 进程闪退。
- **`DjiLiveview_Deinit()` 在常驻应用里调** `[V]`（2026-09-28 修）——
  它反初始化整个 liveview 模块，而妙算3 转发给 Pilot 的图传走这个模块
  ⇒ 黑屏且不自恢复。**官方样例那样写没问题是因为它是一次性工具。**
- **用「整图等比放大」构造变焦用例** `[V]`（2026-09-29 踩到）—— 票数与
  分母同比例增长、比值不变，**退回旧规则照样全绿**。真实变焦是"画幅不变、
  目标变大"⇒ 必须**画幅定死 + 只缩放目标**才逼得出缺陷。
- **用 `fscanf` 读"不定字段数"的行** `[V]`（2026-09-29 踩到）—— 旧格式下
  `%lf` 会去解析 `"src=laser"` 里的 's'，在 4 字段处停下返回 3，于是
  "旧格式"与"文件被截断"返回值撞车。⇒ 用 `fgets` + 两次 `sscanf`。
- **拿 18:32:49 那一轮反推俯仰增益** `[V]` —— 它的 `u` 从 0.484 跳到 0.838
  （横移 35% 画面宽），**俯仰指令不可能造成横向移动** ⇒ 那是检测框换到
  别的红色物体上了。只有 18:17:16 是干净数据点。
- **拿 66.2° 当 HFOV** `[V]` —— 那是"把 82° 当水平 FOV 反算纵向"的错值。
  三个视场角是**同一焦距的三个投影**。正确值 **69.63°**（4:3 + 82°）。
- **「PSDK 不能转自带云台」** `[X]` —— 把 `dji_gimbal.h`（第三方云台）与
  `dji_gimbal_manager.h`（机上云台）混为一谈。实测 `Rotate` 可用。
- **「让 Pilot 2 显示红旗」** `[X]` —— 三处独立失败，官方那句"仅 Matrice 400
  + H30 + 妙算3"是真的。`SendAiMetaToPilot` 返回 SUCCESS 是**假阳性**。
- **`exception` 白名单** `[X]` —— 实测 `0` 也是有效读数。
- **零解判据写 `== 0.0`** `[X]` —— 必须给邻域，用 `LzGeo_IsNullSolution`。
- **`(unsigned)rc` 打 PSDK 返回码** `[X]` —— `uint64_t`，用 `%llX`。
- **`-DPSDK_ROOT=~/path`** `[X]` —— `~` 不被展开（用 `$HOME`）；改了必须重跑 cmake。
- **"模拟器不实现航点启动"** `[X]` —— 已撤回（证据不足）。
- **给宏体里的一行加 `//` 注释** `[V]` —— 宏展开成语法垃圾。
- **交叉编译** `[V]` —— 产物要求 GLIBC 2.34，设备 2.31。
- **`tar --mtime='@0'`** `[V]` —— 让 make 静默跳过编译。用设备时间 −60 s。
- **不排除 `lz/build*` 就打 tar** `[V]` —— x86 产物混进 aarch64 构建。
- **在仓库根建 `build-native`** `[V]` —— 必须是 **`lz/build-native`**
  （`app.json` 在 `lz/app_json/`，`build_dpk.sh` 按 json 所在目录解析）。
- **`app.json` 缺 `description_jp/fr`** `[V]` —— `build_dpk.sh` 报 `KeyError`。
- **用 `/dev/tcp` 扫网段找设备** `[X]` —— 254 个假阳性，用 `ping` + `ip neigh`。
- **在 ctest 里给测试传相对路径数据目录** `[V]` —— 用 `LZ_VISION_TESTDATA`
  环境变量传绝对路径（或 argv[1]）。
- **`DjiWidget_DeInit()` 不存在** `[X]` —— 全模块只有 `Init`。
- **Waypoint V2 用于 M4T** `[X]` —— 官方只支持 M300/M350。
- **兴趣点环绕（`DjiInterestPoint_*`）可控半径** `[X]` —— settings 无 radius。
- **在妙算3 应用管理面板里找控件** `[X]` —— 在 Pilot 相机视图左侧「PSDK」菜单。
- **方位角不能用绝对差比较** `[?]` —— 用 `LZ_CHECK_ANGLE_NEAR`。
- **`LzGeo_NormalizeDeg` 区间 [0,360) 左闭右开** `[?]`。

## 5. 已知坑

- **`.dpk` 安装器会试运行应用并要求走完 SDK 身份校验** `[V]`（本轮再次验证）——
  **飞机不通电就装不上**，而报的是误导性的 `verify app user_app_id or
  version info error`。判据：应用日志里有
  `dji_identity_verify.c:654 Update dji sdk policy file successfully`。
  ⇒ **启动路径上不能有"配置缺失就退出"**（fail-closed 划在"作业开始"）。
- **`SetMode(FREE)` 在 M4T 上是否生效未知** `[?]` —— issue #555 说没有 FREE
  模式，但 `lz_gimbal_probe` 实测**两个模式都返回 SUCCESS**，而
  "返回 SUCCESS ≠ 生效"。横向闭环依赖它。判据：飞机 yaw 转动时画面跟不跟。
- **云台 `.z` 是不是 yaw 未单独验过** `[?]` —— `.x` = pitch 已实测标定，
  yaw 那一轴是推的。日志里有 `yaw=%.2f° 机头=%.2f°`，可一眼判。
- **`GetOpticalZoomParam().currentOpticalZoomFactor` 可能只报光学倍率** `[?]` ——
  字段名带 "Optical"，而 M4T 7× 那段可能含数码变焦。判据：日志 `zoom=` 与
  Pilot 界面显示对照。
- **激光海拔与起飞点海拔的参考面可能不一致** `[?]` ——
  `LaserRangingInfo.altitude` 头文件没说参考面，`ALTITUDE_OF_HOMEPOINT` 是
  **气压高**。已用"三个高程同框"日志暴露，等现场读数。
- **`Smart3DExplore` 会自启占 PSDK 通道** `[V]` —— 跑程序前先
  `pgrep -x Smart3DExplore >/dev/null || dji_app_ctl stop Smart3DExplore`。
- **`systemctl restart dji_sdk_agent` 需要 root** `[V]` —— dji 用户做不了。
- **`CMakeCache.txt` 会记住旧开关** `[V]` —— 传新源码后要显式重配。
- **`hsv` 后端的视觉测试只在 hsv 下注册** `[V]` —— stub 时 ctest 只有 9 个目标。
- **加"人造图形"用例必须先验证"拆掉被测逻辑它会红"** `[V]`。
- **注释里不能写 `*/`** `[V]`。
- **`-1` 会满足 `<= 1`** `[V]` —— 负值哨兵与正数阈值共用变量时判断顺序决定成败。
- **管道下 stdio 全缓冲** `[V]` —— 实时监视类程序每行 `fflush(stdout)`。
- **gdb 抓 PSDK 进程要先 `handle SIG32 nostop noprint pass`** `[V]`。
- **`/blackbox/system/app_temp_files/` 是 root:root 0755** `[?]` —— 删不掉。
- **Probe 退出时 SDK 在 deinit 段报一堆错并 dump core** `[V]` ——
  三个探针都有，**不影响前面的功能**。
- **Pilot 会攥着缓存的控件会话不刷新** `[V]` —— 控件全无响应时**先查
  `0x3C1A`**（有 = 应用侧正常；心跳 `0x2105` 在而它为 0 = Pilot 没发按键），
  **别一上来就改代码**。

## 6. 下一步（有序）

1. **飞机通电并连接，重跑安装**（dpk 已在设备上现成，**不要重编译**）：
   ```bash
   sshpass -p dji ssh dji@192.168.1.180
   pgrep -x Smart3DExplore >/dev/null || /system/bin/dji_app_ctl stop Smart3DExplore
   /system/bin/dji_app_ctl install -i ~/dpk/liangzhourenwu_v01.00.00.00.dpk
   ```
   装成功后判据：应用日志出现 `Update dji sdk policy file successfully`。
2. **验变焦（修得最大的那个，先验它）**：Pilot 里变焦到 **7.0X**，
   按「识别目标」。日志应出现 `zoom=7.0×` 且 `vfov≈8.5`。
   ⚠️ **若 `zoom` 显示 3.0 而非 7.0** ⇒ `GetOpticalZoomParam` 只报光学倍率，
   **告诉用户**（需换有效倍率来源）。
3. **验 `SetMode(FREE)` + 云台 yaw 轴（不需飞行）**：**不按按钮**，徒手把机头
   转 30° 左右，看日志 `yaw=` 是否**不变**（变 = 仍是 YAW_FOLLOW，横向会被拖动）。
4. **验横向闭环**：按「识别目标」，看红旗水平位置是否被拉到中央。
   若机头离红旗超过 ±60° 会报"请把机头朝红旗方向拨一下" —— **预期行为**。
5. **验目标高度**：激光打**旗面**按「记录激光点」，核对"高程自洽校验"
   那行三个海拔数是否自洽，浮窗是否报出目标高。
6. **验绕飞弧线**：拨开关，看航迹回放是不是弧、到点瞬间云台还跳不跳。
7. **回填 `[?]`**：把 §5 里四条 `[?]` 按实测结果升级为 `[V]` 或 `[X]`，
   更新 CLAUDE.md 与 `lz/doc/ONDEVICE-CHECKLIST.md` §3.12b。

## 7. 留给用户的开放问题

- **激光飘走打到后方地面**没有自动判据（旗面飘动 ±0.5 m 与"打到地面"之间
  没有干净分界，硬判会误拒）。目前只如实报出量到的值由操作员核对 ——
  用户当初指出过这个风险，**是否需要更主动的检测**？
- **检测框换目标时不察觉**：若另一个红色物体恰好也在 v≈0.5 附近，照准会报
  "已对准"。要根治得加"身份连续性"判据（单轮 Δθ 残差方向不该反号、u 不该
  突变）—— **那是新功能，不是修 bug**，且阈值要拿现场画面序列标定。
- **急停后没有返航按钮** —— 拨 OFF 只是 STOP；`finishAction=gotoFirstWaypoint`
  正常飞完也停在杆旁不返航。建议加独立 button 调
  `DjiFlightController_StartGoHome()`。**未实现。**
- **半径上限 20 m** —— 用户提过后期会变大，需先确认新场地约束。
- **`Smart3DExplore` 要不要卸载** —— 它占通道且会自启，但是 DJI 预装的。
- **`gimbalEvenlyRotate` 要不要用** —— 规范里它是"航段间均匀转动云台"的
  正解，与逐点 `gimbalRotate` 是**两套互斥机制**。本次只关掉了逐点 yaw，
  若试飞发现云台在航段间仍不平滑，才考虑换过去。

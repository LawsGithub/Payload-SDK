# HANDOFF — 读全文再开始干活

生成时间: 2026-10-08T01:02+0800 · Git HEAD: `21cf273`
信任规则: [V] = 交接时已用命令验证；[?] = 仅记忆未复核，当线索对待；[X] = 已证伪，别用。

## 0. 复核（下一会话先做）

- 锚点: `feature/liangzhourenwu` @ `21cf273`（2026-10-08 01:02）
- 漂移检查: `git rev-parse HEAD~1` 应 = `21cf273`（本次 handoff 是**一个**提交）。
  ⚠️ **上一份 HANDOFF 的判据在这里失效了** —— 它写 `HEAD~2`，因为它自己是
  两个提交；而它落盘之后**又来了 6 个提交**（10-06/07），锚点从未更新。
  **教训：判据要写"用 `git log --oneline -5` 的实际输出比对"，别写死 `HEAD~N`。**
- **远端已同步到 `21cf273`** `[V]`。推送**必须走代理**，直连挂起：
  ```bash
  GIT_SSH_COMMAND="ssh -o ProxyCommand='nc -X connect -x 127.0.0.1:7890 %h %p'" \
    git push fork feature/liangzhourenwu
  ```
- **⚠️ 设备不在线** `[V]`（2026-10-08 01:02 实测）：`192.168.1.180` ping 100%
  丢包、ARP `INCOMPLETE`。本机 `eth1` = `192.168.1.46/24`（网段对），
  **网关 `192.168.1.1` 通（2.3 ms）⇒ 网线没问题，是设备侧没响应**。
  扫过整个 `192.168.1.0/24`：活着 5 台（`.58/.173/.79/.194/.214`），
  **22 端口全部拒绝，没有一台是妙算3**。⇒ 先确认扩展坞/设备通电。
  ```bash
  ip -4 -br addr && ping -c2 192.168.1.180 && ip neigh show dev eth1
  ```
- 先读: [`lz/doc/ONDEVICE-CHECKLIST.md`](lz/doc/ONDEVICE-CHECKLIST.md)
  （现场操作单，§1 控件表按界面分两张）+ `CLAUDE.md`（已瘦身，含文档导航）
  + [`lz/doc/GIMBAL.md`](lz/doc/GIMBAL.md) 的「云台模式是飞机上的全局状态」
  与 [`lz/doc/CORE-TESTING.md`](lz/doc/CORE-TESTING.md) 的「本机自检三条新武器」。

## 1. 当前目标

**把 2026-10-01 现场报的「绕飞时云台偏航顶限位 + 云台电机异常」修掉。**

代码已改完（`1fb4cd4`）、桌面已验、`precheck` 全绿，
**完成定义 = 上机按清单 §6 走一遍，绕飞全程不再出现那两条告警。**

## 2. 已验证状态 — 工作实际停在哪

### 本会话（2026-10-08）只做了文档更正，**没碰代码**

`21cf273` 修正两处**与代码矛盾**的控件陈述 —— 现场照着做会找错地方：

| 文件 | 原文 | 实际 |
|---|---|---|
| `CLAUDE.md` | "实测 3 个控件 / 控件 5 = 航点数" | **7 个**；航点数是 **6**，半径/高度是 **4/5** |
| `ONDEVICE-CHECKLIST.md` §1 | 7 个控件平铺成一张表，标题写「PSDK 菜单」 | 分两个界面：`main` 0–3、`config` 4–6 |

⇒ 同一个事实抄了两份、只有一份跟着改。改法是**删掉重复的那份**、指向唯一真值表。

### 桌面验证 `[V]`（本会话实跑，真实退出码）

```
hsv  后端: ctest --test-dir lz/build            → 12/12 passed, rc=0
stub 后端: cmake -DLZ_VISION_BACKEND=stub + ctest → 11/11 passed, rc=0
bash lz/tools/precheck.sh                       → rc=0，全部通过
  3b ASan+UBSan ✓ / 3c aarch64 交叉编译 ✓ / 3d 覆盖率最低 87.10% ✓
```

⚠️ **目标数是 hsv 12 / stub 11** —— 上一份 HANDOFF 写的 "11/10" 已过期
（多了 `lz_test_mission`）。

### ⚠️ 设备侧编译打包：**从未做过**

`1fb4cd4` 的新文件（`lz_gimbal_status.{h,c}`、`lz_test_gimbal.c`、
`lz_mission_logic.{h,c}`）**从未在 aarch64 上编译过**。
（precheck 3c 在本机交叉编译过，风险比之前小，但**不等于设备上编得过**。）
设备上那个旧 dpk 落后得比想象中多 —— `lz_mission.c` 改了 185 行、
`lz_pole_source.c` 改了 173 行、还多两个新文件。**别直接装旧的。**

### git 状态（事实快照，非待办）

```
feature/liangzhourenwu @ 21cf273，工作区干净，本地与远端一致
```

## 3. 决策与理由

- **控件的界面归属由语义定** `[V]`（2026-09-28）—— 飞行中操作的（绕飞/记录/识别）
  放 `main_interface`；起飞前配置的（半径/高度/航点数）放 `config_interface`。
  索引 0–6 是**跨界面连续序列**，SDK 的 handler 表是扁平的、不区分界面。
- **`int_input_box` 只能放 `config_interface`** `[V]` —— 放 `main_interface` 时
  SDK 收下但不渲染成可编辑控件（判据：回调全历史零命中）。**"看得见但改不了"。**
- **判据一律抽进 `lz_core`** `[V]` —— 云台状态格式化、绕飞状态机转移表、
  激光判据、`LzAlign_DecideStep` 都搬走了。**判据写在 PSDK 侧 = 桌面测不到 = 没有测试。**
- **同一事实只留一处真值** `[V]` —— 本会话又踩一次（控件索引抄了两份）。
  与「安全包线唯一真值在 `lz_plan.h`」是同一条。
- **覆盖率基线刻意不当 KPI** `[V]` —— 它是"判据抽到桌面"的副产品，
  用处是**发现判据被搬回 app 层**（抽走会跳、搬回会掉），不是考核指标。

## 4. 失败的尝试 — 不要再试

（沿上一份，**全部前向搬运**。上一份说"沿上一轮未变"，本轮同样。）

- **`DjiFcSubscription_GetLatestValueOfTopic`** `[?]` —— SIGSEGV，栈在
  `DjiDataSubscriptionDds_v3_GetLastValueOfTopic` 内部。一律走回调缓存。
- **在 `ApplicationStart()` 之前调 `DjiFcSubscription_Init()`** `[?]` ——
  返回 SUCCESS 但随后 SIGSEGV。**返回成功不代表调用合法。**
- **在控件回调里调任何阻塞接口** `[?]` —— 进程闪退。
- **`DjiLiveview_Deinit()` 在常驻应用里调** `[V]` —— 黑屏且不自恢复。
- **同一订阅项重复订阅** `[V]` —— 头文件明写不可。`GIMBAL_ANGLES` 被
  `lz_visual_align.c` 以 50Hz 订走，新订阅走 `GIMBAL_STATUS`（10Hz）。
- **用「整图等比放大」构造变焦用例** `[V]` —— 票数与分母同比例增长，
  退回旧规则照样全绿。必须**画幅定死 + 只缩放目标**。
- **用 `fscanf` 读"不定字段数"的行** `[V]` —— 返回值会撞车。用 `fgets` + 两次 `sscanf`。
- **拿 18:32:49 那一轮反推俯仰增益** `[V]` —— 检测框换到别的红色物体上了。
- **拿 66.2° 当 HFOV** `[V]` —— 正确值 69.63°（4:3 + 82°）。
- **给宏体里的一行加 `//` 注释** `[V]` —— 宏展开成语法垃圾。
- **交叉编译** `[V]` —— 产物要求 GLIBC 2.34，设备 2.31。
- **`tar --mtime='@0'`** `[V]` —— 让 make 静默跳过编译。用**设备时间 −60 s**。
- **不排除 `lz/build*` 就打 tar** `[V]` —— x86 产物混进 aarch64 构建。
- **在仓库根建 `build-native`** `[V]` —— 必须是 **`lz/build-native`**。
- **`app.json` 缺 `description_jp/fr`** `[V]` —— `build_dpk.sh` 报 `KeyError`。
- **在 ctest 里给测试传相对路径数据目录** `[V]` —— 用 `LZ_VISION_TESTDATA`。
- **`precheck` 里用 `check_call` 查"判据有没有被抄回来"** `[V]` ——
  它只问符号在不在文件里，而注释里也算。实测替换掉真正的调用它**照样绿**。
  拦得住的是"文案不许出现在 PSDK 侧"那条。两条都留，分工写在注释里。
- **`[X]` 类（已证伪，别用）**：`exception` 白名单 / 零解判据写 `== 0.0` /
  `(unsigned)rc` 打返回码 / `-DPSDK_ROOT=~/path` / "模拟器不实现航点启动" /
  "PSDK 不能转自带云台" / "让 Pilot 2 显示红旗"（`SendAiMetaToPilot` 返回
  SUCCESS 是假阳性）/ `DjiWidget_DeInit()` 不存在 / Waypoint V2 用于 M4T /
  兴趣点环绕可控半径 / 在妙算3 应用管理面板里找控件 / 用 `/dev/tcp` 扫网段。

## 5. 已知坑

- **`.dpk` 安装器会试运行应用并要求走完 SDK 身份校验** `[V]` ——
  **飞机不通电就装不上**，报误导性的 `verify app user_app_id or version
  info error`。判据：日志里有 `dji_identity_verify.c:654 Update dji sdk
  policy file successfully`。
- **`Smart3DExplore` 会自启占 PSDK 通道** `[V]` —— 跑程序前先
  `pgrep -x Smart3DExplore >/dev/null || /system/bin/dji_app_ctl stop Smart3DExplore`。
- **改了源码却是旧行为** `[V]` —— 传完源码必须 `rm -rf lz/build-native` 全量重编。
- **`CMakeCache.txt` 会记住旧开关** `[V]` —— 传新源码后要显式重配。
- **`hsv` 12 个 ctest 目标、stub 11 个** `[V]`（2026-10-08 实测）——
  stub 不注册 `lz_test_vision`。**别按旧数字（11/10）记。**
- **`SetMode(FREE)` 在 M4T 上是否生效未知** `[?]` —— issue #555 说没有 FREE
  模式，而探针实测两个模式都返回 SUCCESS，"返回 SUCCESS ≠ 生效"。
- **云台 `.z` 是不是 yaw 未单独验过** `[?]`；**`GetOpticalZoomParam` 可能只报
  光学倍率** `[?]`；**激光海拔与起飞点海拔参考面可能不一致** `[?]`。
- **Pilot 会攥着缓存的控件会话不刷新** `[V]` —— 控件全无响应时**先查 `0x3C1A`**
  （有 = 应用侧正常；心跳 `0x2105` 在而它为 0 = Pilot 没发按键），**别一上来改代码**。
- **`-1` 会满足 `<= 1`** / **注释里不能写 `*/`** / **管道下 stdio 全缓冲** /
  **gdb 抓 PSDK 进程要先 `handle SIG32 nostop noprint pass`** /
  **`/blackbox/system/app_temp_files/` 是 root:root 0755**（删不掉）/
  **Probe 退出时 SDK 在 deinit 段 dump core**（不影响前面的功能）——
  均 `[V]`，细节见 [`lz/doc/BUILD-DEPLOY.md`](lz/doc/BUILD-DEPLOY.md)「已知坑（本项目特有）」。

## 6. 下一步（有序）

1. **让设备上线**（本会话两次实测都不通）。判据：`ping 192.168.1.180` 通
   且 `ip neigh` 有 ARP 条目。**不要用 `/dev/tcp` 扫网段。**
2. **设备侧编译打包**（新文件从未在 aarch64 上编过）：
   ```bash
   DEVT=$(sshpass -p 'dji' ssh dji@192.168.1.180 'date +%s')
   tar czf /tmp/lzbuild.tar.gz --mtime="@$((DEVT - 60))" \
        --exclude='lz/build*' --exclude='*.dpk' \
        lz tools/build_dpk psdk_lib/include psdk_lib/lib/aarch64-linux-gnu-gcc \
        samples/sample_c/platform/linux/{manifold3,common}
   scp /tmp/lzbuild.tar.gz dji@192.168.1.180:/tmp/lzsrc.tar.gz
   # 设备上：解包 → rm -rf lz/build-native → cmake（hsv + LASER=ON）→ build → ctest
   ```
   判据：`ctest` 上 **12/12 passed**（aarch64 侧与桌面 hsv 同数）。
3. **飞机通电并连接，装新 dpk**：判据是日志出现
   `Update dji sdk policy file successfully`。
4. **验修复（主目标）**：跑一次「识别目标」再跑一次绕飞。
   日志应有 `绕飞前：云台模式已确认为 YAW_FOLLOW` 与
   `照准收尾：云台模式已恢复 YAW_FOLLOW`，且**绕飞全程不再出现
   `云台状态：偏航限位` / `偏航电机异常`**。
5. **核对首帧原始位图**（清单 §3.13）—— **这是唯一能验"极性没抄反"的办法**。
   徒手把云台顶到某轴限位，看哪一位翻转，与 `lz_bridge_psdk.c` 的 bit 表对照。
6. **按清单 §3.1–§3.8 逐条走**（航点数输入框 → 绕飞链路 → `towardPOI` →
   弧线 → `gotoFirstWaypoint` → 云台 yaw → 机型枚举 → POI 备选）。
   ⚠️ **航点数在 Payload Settings 里，不在 PSDK 菜单** —— 见清单 §1b。
7. **回填 `[?]`**：把 §5 里四条按实测结果升级，同步 `CLAUDE.md` 与清单。

## 7. 留给用户的开放问题

- **设备为什么不在线？** 2026-10-05 与本次两次实测都不通，网关却通。
  扩展坞没插？设备没通电？还是又换网段了？
- **急停后没有返航按钮** —— 拨 OFF 只是 STOP；`finishAction=gotoFirstWaypoint`
  正常飞完也停在杆旁不返航。建议加独立 button 调
  `DjiFlightController_StartGoHome()`。**未实现。**
- **激光飘走打到后方地面**没有自动判据（旗面飘动 ±0.5 m 与"打到地面"之间
  没有干净分界，硬判会误拒）。是否需要更主动的检测？
- **检测框换目标时不察觉** —— 另一个红色物体恰在 v≈0.5 附近时，照准会报"已对准"。
  根治要加"身份连续性"判据，**那是新功能不是修 bug**。
- **半径上限 20 m** —— 用户提过后期会变大，需先确认新场地约束。
- **`gimbalEvenlyRotate` 要不要用** —— 规范里它是"航段间均匀转动云台"的正解，
  与逐点 `gimbalRotate` 是两套互斥机制。本次只关掉了逐点 yaw。

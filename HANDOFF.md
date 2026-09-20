# HANDOFF — 读全文再开始干活

生成时间: 2026-09-20T21:39:45+0800 · Git HEAD: 7383809
信任规则: [V] = 交接时已用命令验证；[?] = 仅记忆未复核，当线索对待；[X] = 已证伪，别用。

## 0. 复核（下一会话先做）

- 锚点: `feature/liangzhourenwu` @ `7383809`（2026-09-20 21:39）
- 漂移检查: 快照写完后我又补了一个文档修正提交（`2ec41b7` 改 push 状态），
  所以现在 `HEAD~1` = `9031463`、`HEAD~2` = `7383809`。
  **判据改成：`git log --oneline` 里应能看到 `7383809` 这条持久文档提交**；
  看不到说明历史被改写，快照作废。
- **远端已同步** `[V]`（push 成功，`2b00fdf..9031463`）—— 本地与 `fork/feature/liangzhourenwu`
  一致、工作区干净。若 `git status` 显示领先，按仓库约定重试 `git push fork feature/liangzhourenwu`，
  不要用 `--no-verify`。
- 设备**当前不可达** `[V]`（21:39 实测 `kex_exchange_identification: Connection closed`，
  且 `eth5` 网卡消失、ping 不通）。需先恢复妙算3（重插 USB-C 或等它起来）。
- 先读: `~/projects/liangzhourenwu/CLAUDE.md`（项目约定）+ `~/projects/.psdk-apiref/liangzhourenwu/API-MAP.md`

## 1. 当前目标

**让「操作员在 Pilot 2 拨开关 → 飞机绕国旗杆飞一圈」完整可用。**

完成定义 = 拨 ON 后飞机按设定半径/高度绕杆一周并返航。

**当前卡点：整条链路只差"启动航点任务"这一步，且它卡在 DJI Assistant 2 模拟器上。**

## 2. 已验证状态 — 工作实际停在哪

**代码侧全部就绪并已提交** `[V]`（两个切片，见 §2 末尾 SHA）。本会话把三个上机暴露的
问题逐个定位并修掉。

### 本会话新验证的（都是设备实测）

- **控件在 Pilot 2 显示且回调真的触发** `[V]` —— 日志 `DJI_0002`：19:53:53.100
  `[widget] 收到绕飞请求：半径 17.5 m，高度 100.7 m`。
  控件面板在 **Pilot 2 相机视图左侧「PSDK」菜单**里，**不在妙算3 应用管理面板**。
- **KMZ 上传全链路通** `[V]` —— 41 个分片 + `Check kmz file md5sum success`，
  `LzBridge_UploadKmzV3` 的 `上传返回 rc=0x00000000`。
- **GPS/RC/飞行状态全部正常** `[V]`（探针模式 4 实测）：`fixState=3`（3D Fix）、
  卫星 15、`TOPIC_RC.mode=0`（对应 N 档）、融合位置 `28.1788565, 112.9210166`
  —— 与场地坐标只差 1.2 m，飞机就在场地上方。
- **`DjiFcSubscription_GetLatestValueOfTopic` 必崩** `[V]` —— gdb 抓到栈：
  `DjiDataSubscriptionDds_v3_GetLastValueOfTopic` 内 SIGSEGV，**SDK 内部**。
  已排除读太快 / 传 NULL 回调 / 无数据 / 初始化时机四个嫌疑。
- **DPK 全流程通** `[V]` —— `build_dpk.sh` 打包 → `uninstall` → `install`
  （APP INSTALL SUCCESS）→ `start` → 进程持续运行，`widget_file/` 在包根。

### 启动航点失败 —— 真实死因已定位到 SDK，不是我们代码

```
DjiWaypointV3_Action(START) → rc = 0x000000FF
```

**注意是 `0xFF` 不是 `770/0x302`**（RC 模式错）。GPS ✓、N 档 ✓、上传 ✓、MD5 ✓
全部正常，仍返回 `0xFF` —— **这不是"哪项前置条件不满足"**。
判断：DJI Assistant 2 模拟器不实现航点任务的启动路径 `[?]`（无确证，但证据链完整）。

### 测试/build 输出（本会话真实运行，含退出码）

```
PC 侧:    cmake --build build   exit 0
          ctest --test-dir build  exit 0   → 100% tests passed, 0 failed out of 4
x64 侧:   cmake --build lz/build-x64  exit 0，我们自己代码 0 warning
设备侧:   本机 aarch64 make  exit 0，0 warning，产物 lz_app / lz_mission_probe
探针模式 4（回调缓存版）: exit 0，回调 240 次，5 个话题全部有数据
```

### git 状态（事实快照，非待办）

```
feature/liangzhourenwu @ 7383809，工作区干净
af3e284 feat(lz_app): 上机链路修复 —— 启动诊断走回调、返回值拆分、控件图标重做
7383809 docs(claude): 更新项目约束 —— 上机实测结论、PSDK 3.16.0-beta 的坑、打包细节
9031463 docs: handoff —— 上机链路修复完成，启动航点卡在模拟器，待真机首飞
2ec41b7 docs(handoff): 修正 push 状态与下一步
本地与远端一致（push 成功，见 §0）
```

## 3. 决策与理由

- **启动诊断改走回调缓存，不用 `GetLatestValueOfTopic`** `[V]` —— 后者必崩且崩在
  SDK 内部。回调 10 秒收 518 次，数据一直在到，**说明取数据不必经过那个 getter**。
  否决方案：继续调 getter（试过 sleep 3s、传回调、改顺序，全崩）。
- **`LZ_ERR_UPLOAD` 与 `LZ_ERR_START` 拆成两个返回值** `[V]` —— 原来两者都返回
  `LZ_ERR_IO`，上层只能报"文件读写失败"，而真实死因是启动被拒。
  **与 `.dpk` 安装器把"提前退出"误报成"凭据错"是同一形状：一个返回值承载多种失败，
  调用方必然误报。**
- **高度方案取"相对起飞点 100 m"而非绝对 ASL** `[V]` —— 用户 2026-09-20 选甲。
  绝对模式要改 `executeHeightMode` 并处理大地水准面差距，错了会飞到错误高度。
- **固定坐标 = 用户提供的场地坐标** `[V]`（`28.1788480, 112.9210020`）——
  场地无实物旗杆，该点作虚拟圆心。飞机在圆心正上方，即"绕自己画圆"。
- **控件路径运行时解析两个候选** `[V]` —— dpk 装后 CWD = 包根（`widget_file/`），
  源码树里跑 CWD = `lz/`（`app/widget_file/`）。否决方案：只写一个路径
  （两种调试方式必有一种坏）。
- **switch 图标重做成圆环+三角/方块** `[V]` —— 原用 DJI 官方样例图标
  （半圆+三横线），**形状与"开关"无任何视觉关联，开关状态只体现在颜色上**。
  新图标形状随状态改变 —— 户外强光下颜色不可靠。
  用纯标准库（zlib+struct）手写 PNG，本机无 PIL/ImageMagick。

## 4. 失败的尝试 — 不要再试

- **`DjiFcSubscription_GetLatestValueOfTopic`** `[V]` —— SIGSEGV，栈在
  `DjiDataSubscriptionDds_v3_GetLastValueOfTopic` 内部。已排除：读太快
  （sleep 3s 仍崩）、传 NULL 回调（传了也崩）、无数据（回调 518 次）、
  初始化时机（已在 ApplicationStart 之后）。**换回调缓存，别再回头试。**
- **在 `ApplicationStart()` 之前调 `DjiFcSubscription_Init()`** `[V]` —— 它返回
  `SUCCESS` 但随后 SIGSEGV。官方文档："请勿在 main() 函数中调用本接口……
  启动调度器后，该接口将正常运行。"**返回成功不代表调用合法**，且崩溃点与病因
  在栈上完全对不上。已改为 `LzMission_StartPostApp()`，在 ApplicationStart 之后调。
- **用 `/dev/tcp` 扫网段找设备** `[X]` —— 会给出 **254 个假阳性**（本地拦截器
  让所有 IP 的 22 端口都显示 OPEN）。正确判据是 `ping` + `ip neigh`（看 ARP）。
- **`tar --mtime='@0'` 传源码** `[V]` —— 时间戳压到 1970 后 `make` 发现源码比
  `.o` 旧，**静默跳过编译**。改用**设备当前时间 `-60 seconds`**（`+30s` 仍会报
  "is 29.2 s in the future"，因为设备时间在走）。
- **不排除 `lz/build*` 就打 tar** `[V]` —— 会把本机 x86 侧 build/ 传上设备，
  与 aarch64 构建混在一个目录。
- **`app.json` 只写 cn/en 两语言描述** `[V]` —— `build_dpk.sh` 逐个字段校验，
  缺 `description_jp` 直接退出报 `KeyError`。**四语言必须齐全。**
- **在妙算3 应用管理面板里找控件** `[X]` —— 那只是 `dji_app_ctl start`。
  控件在 **Pilot 2 相机视图左侧「PSDK」菜单**。
- **只判激光 `exception` 不判 `distance`** `[V]` —— `distance=0` 时激光给出的
  `lat/lon` 是模拟器飞机初始位置（113.1700000, 28.2666000），是**精确的错误值**
  不是垃圾值。已加 `distance <= 0` 拒绝。

（以下为上一份 HANDOFF.md 前向搬运，标 `[?]`，未在本会话重新验证：）

- **`tar --mtime='@0'`** `[?]` —— 见上，已用 `-60 秒` 规避。
- **`lz_widget.c` 注释里写 `widget_file/*/`** `[?]` —— `*/` 提前闭合块注释。
- **探针用 `samples <= 1` 判断单次模式** `[?]` —— `-1` 哨兵满足 `<= 1`。
- **`DjiWidget_DeInit()`** `[X]` —— 该函数不存在，全模块只有 `Init`。
- **"KMZ 动作对第三方负载没意义"** `[X]` —— `gimbalRotate` 独立于相机。
- **Waypoint V2 用于 M4T** `[X]` —— 官方只支持 M300/M350。
- **兴趣点环绕（`DjiInterestPoint_*`）可控半径** `[X]` —— settings 里无 radius 字段。
- **交叉编译** `[V]` —— 产物要求 `GLIBC_2.34`，设备 2.31。

## 5. 已知坑

- **`proj1_app` 是系统预装的，不是残留** `[V]`（三条证据：`dji_app_ctl list`
  只有 1 个应用；`/open_app/install` 时间戳 Apr–Jul 2025；其构建路径
  `/home/dji/Payload-SDK/proj1_app/` 在设备上不存在）。**别再清它。**
- **gdb 抓 PSDK 进程要先 `handle SIG32 nostop noprint pass`** `[V]` —— PSDK 的
  linker 线程用 SIG32 做实时事件，gdb 默认停在它上面，真正的 SIGSEGV 看不到。
- **诊断程序必须在读之前就打印并 flush** `[V]` —— 崩溃吞掉 stdio 缓冲，
  而那几行正是定位关键。
- **方位角不能用绝对差比较** `[?]` —— 正北处返回 `359.999999998°`。用 `LZ_CHECK_ANGLE_NEAR`。
- **`LzGeo_NormalizeDeg` 区间 [0,360) 左闭右开** `[?]` —— 极小负数加 360 会因舍入得 `360.0`。
- **`-1` 会满足 `<= 1`** `[?]` —— 负值哨兵与正数阈值共用变量时，判断顺序决定成败。
- **管道下 stdio 全缓冲** `[?]` —— 实时监视类程序每行 `fflush(stdout)`。
- **`/blackbox/system/app_temp_files/` 是 root:root 0755** `[?]` —— 删不掉，且混有别的项目日志。
- **本 worktree 是稀疏检出** `[?]` —— `lz` 目录需先 `git sparse-checkout add lz` 才能 `git add`。

## 6. 下一步（有序）

1. **恢复设备连接**（重插 USB-C / 等妙算3 起来）。判据：`ping 192.168.42.120` 通 +
   `ip neigh` 有 ARP 条目。**不要用 `/dev/tcp` 扫描。**
2. （已完成，见 §0）push 本地提交 —— 远端已同步
3. **真机首飞**（不再耗在模拟器上）：
   - 停在场地坐标处起飞，`pgrep -x Smart3DExplore >/dev/null || /system/bin/dji_app_ctl stop Smart3DExplore`
   - 拨 Pilot 2 的 PSDK 开关 → 观察浮窗（新版会直接显示 `启动被拒：空中 RC=0 GPS状态=3 卫星=15`）
   - 若成功 → 验绕行方向（顺时针）、云台逐点指向、返航
4. **`-DLZ_POLE_SOURCE_LASER=ON` 编译**，室外用激光打旗杆，验 `lat/lon` 是否为瞄准点
5. **定视觉层职责**：激光若可用，视觉降级为"确认瞄准点是不是杆"，`lz_vision` 的
   连通域可不实现。注意 `lz_vision_source.c` 目前是**空壳、未接进主链路**。

## 7. 留给用户的开放问题

- **DJI Assistant 2 模拟器到底支不支持航点启动？** `[?]` 证据指向不支持，但无确证。
  如果不想上真机，可在模拟器里用**官方 waypoint_v3 样例**做对照实验 —— 它同样
  失败即坐实模拟器问题；它成功则说明是我们 KMZ 的问题，要回去查 wpml。
- **杆的 WGS84 坐标最终用哪条路？** 激光（现成、精度高，需打中杆）vs 视觉定位（未做）。
  注意红旗横向展开，激光打中旗面中心会偏离杆轴最多半个旗宽（1–1.5 m），
  对 5–30 m 半径是 2–20% 的圆心偏差 —— **要"打杆不打旗"，或做几何下推。**
- **室外调试通道怎么解决？** 妙算3 无内置无线，但内核带 103 个无线驱动模块、
  `dnsmasq`/`nmcli`/`wpa_supplicant` 齐全。插一个驱动已覆盖的 USB WiFi dongle
  接手机热点是最现实的路径 `[?]` 未实测。
- **150 m ASL 要不要改成真正的绝对高度？** 当前是"相对起飞点 100.7 m"，
  而场地地面实测约 17 m ASL，所以实际约 118 m ASL，不是 150。

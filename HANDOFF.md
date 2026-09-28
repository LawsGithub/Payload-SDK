# HANDOFF — 读全文再开始干活

生成时间: 2026-09-28T17:14:26+0800 · Git HEAD: 5049eeb
信任规则: [V] = 交接时已用命令验证；[?] = 仅记忆未复核，当线索对待；[X] = 已证伪，别用。

## 0. 复核（下一会话先做）

- 锚点: `feature/liangzhourenwu` @ `5049eeb`（2026-09-28 17:1x）
- 漂移检查: `git rev-parse HEAD~1` 应 = `5049eeb`（HEAD 是本次 handoff 提交，
  其 parent 是快照锚点）。权威以 `git log --oneline -5` 实际输出为准。
- **远端已同步到 `5049eeb`** `[V]`。推送**必须走代理**，直连挂起：
  ```bash
  GIT_SSH_COMMAND="ssh -o ProxyCommand='nc -X connect -x 127.0.0.1:7890 %h %p'" \
    git push fork feature/liangzhourenwu
  ```
- **设备在线** `[V]`：USB 扩展坞 `192.168.1.180`（本机 `eth3`）。
  时钟落后本机 **2.4 小时**（已不是上轮记的 43.7 h）——
  `tar --mtime` 仍必须**从设备取时间**再减 60 秒。
- **`lz_app` 正在设备上运行**（pid 32011，装于 17:00:56）——
  **改代码前先 `dji_app_ctl stop liangzhourenwu`**。
- 先读: `~/projects/liangzhourenwu/CLAUDE.md`（项目约定，本轮大改）
  + [`lz/doc/ONDEVICE-CHECKLIST.md`](lz/doc/ONDEVICE-CHECKLIST.md)（上机执行单）

## 1. 当前目标

**让「操作员在 Pilot 2 拨开关 → 飞机绕记录的点飞一圈」在现场稳定可用。**

**本轮的重大进展：视觉闭环首次完整跑通**（见 §2）。绕飞链路本身
**在 2026-09-28 也第一次真的飞完了 8 个航点**（15:59 那次，见 §2）。

**当前待用户验证**：控件归位到两个界面后，「航点数」能否在
Pilot 的 `Payload Settings` 里编辑（这是本轮改动的直接目的，**尚未上机确认**）。

## 2. 已验证状态 — 工作实际停在哪

### ✅ 视觉闭环**首次完整跑通** `[V]`（2026-09-28 15:31 真机）

```text
15:31:04.792  按键 index=6              ← Pilot 命令到达（0x3C1A=6）
15:31:06.911  第1轮 u=0.5431 v=0.1815 conf=1.000 Δθ=+18.38° → 转到 18.18°
15:31:08.655  第2轮 u=0.5424 v=0.5394 conf=0.626 Δθ=-2.35°  → 转到 15.75°
15:31:10.419  第3轮 u=0.5410 v=0.4968 conf=0.711 Δθ=+0.19°  ← 进死区
15:31:11.045  照准完成：共 2 轮，最终 Δθ=+0.19°
```

`v` 最终 **0.4968**（目标 0.5，偏差 0.003）；`u` 稳定 0.541。

### ✅ 绕飞链路**首次真的飞完** `[V]`（15:59 那次，8 航点 + 收尾点）

```
16:00:26  记录激光点 28.1743615, 112.9220320
16:00:33  拨 ON → KMZ 46121 字节 → 上传完成 → 航点任务已启动
16:00:57  航点 1 … 16:02:49 航点 9 → 16:02:50 正常结束（回到 IDLE）
```

### 本轮修的三处（都已验生效）

| # | 缺陷 | 判据 |
|---|---|---|
| 1 | **控件"点不动"** | 浮窗被「绕飞中：航点 N」刷屏 **716 条**堵死（上限 2 KB/s）。改只在航点号变化时发 → **716 → 9 条** |
| 2 | **`heightM` 写死 15** | 打地面点时俯仰偏 **5.04°**（118 px = 画面高度 11%）。改由来源定（点目标 = 0） |
| 3 | **`int_input_box` 放错界面** | 「航点数」看得见但改不了。归位到 `config_interface` |

### 工作区/仓库状态（事实快照）

```
feature/liangzhourenwu @ 5049eeb，工作区干净，远端已同步
本轮三个切片（均已推送）：
  b868477 fix(lz): 控件按界面语义归位 + 目标高度按来源定 + 浮窗刷屏堵死控件
  6798154 chore(lz): 工具跟上两界面控件
  5049eeb docs: 控件两界面划分 + heightM 按来源定 + 浮窗刷屏 入库
```

### 测试/build 输出（本次交接从全新目录跑，真实退出码）

```
cmake -S lz -B /tmp/hv -DLZ_VISION_BACKEND=hsv   rc=0
cmake --build /tmp/hv -j4                        rc=0，warning 0
ctest --test-dir /tmp/hv                         rc=0，10/10 passed
逐目标（rc 全 0）：
  lz_test_align 108 ｜ plan 1144 ｜ validate 173 ｜ wpml 140 ｜ vision 121
  lz_test_pole 56 ｜ vision_math 56 ｜ geo 34 ｜ kmz 22 ｜ bridge 12
bash lz/tools/precheck.sh                        rc=0（全部通过）
x64 PSDK 侧（-DLZ_TARGET_ARCH=x86_64）            0 warning
设备侧 aarch64：cmake+build 0 warning，ctest 10/10，装包 SUCCESS
  启动日志 main interface widget count = 4 / config interface widget count = 3
```

## 3. 决策与理由

- **控件归属按语义分两个界面** `[V]` —— `main_interface`（PSDK 菜单，飞行中操作）
  vs `config_interface`（Payload Settings，起飞前配置）。**`int_input_box` 只能
  放 config** —— 放 main 时 SDK 收下（解析计数正常）**但 Pilot 不渲染成可编辑
  控件**。判据：它的回调**全历史零命中**（type=5），官方样例的 main 里也没这个类型。
- **两界面索引共用一个序列、不重叠** `[V]` —— **SDK 的 handler 表是扁平的、
  不区分界面**。0-3（main）+ 4-6（config）。
- **`heightM` 由来源决定** `[V]` —— `-h/2` 的语义是「瞄目标的中点」，**对杆对、
  对地面点错**。激光点/飞机位记录的**都是一个点** ⇒ 0。
- **俯仰一圈只算一次**（不改逐航点）`[V]` —— 试过逐航点算，实测 9 个航点真实
  距离**差 0.000000 m**、俯仰**差 0.000000°**（`LzGeo_Destination` 在等半径
  圆周上逐点距离相同）⇒ 无用复杂度，还改红 4 条断言。**已撤回。**
- **云台 pitch 必须自己算——WPML 没有"朝向 POI"** `[V]` —— 规范原文
  （`40.common-element.md` 的 `waypointPoiPoint`）：「**目前不支持 Z 方向
  朝向兴趣点**」。`towardPOI` **只管机头 yaw**。
- **不做**：给「航点数」再加一个 main 滑杆 —— 航点数是起飞前定的量，
  本来不该在飞行中改。

## 4. 失败的尝试 — 不要再试

- **`DjiFcSubscription_GetLatestValueOfTopic`** `[?]` —— SIGSEGV，栈在
  `DjiDataSubscriptionDds_v3_GetLastValueOfTopic` 内部。一律走回调缓存。
- **在 `ApplicationStart()` 之前调 `DjiFcSubscription_Init()`** `[?]` ——
  返回 SUCCESS 但随后 SIGSEGV。**返回成功不代表调用合法。**
- **在控件回调里调任何阻塞接口** `[?]` —— 进程闪退。
- **`DjiLiveview_Deinit()` 在常驻应用里调** `[V]`（2026-09-28 修）——
  它反初始化**整个 liveview 模块**，而妙算3 转发给 Pilot 的图传走这个模块
  ⇒ 黑屏且不自恢复。已改为只 `StopImageStream`。**官方样例那样写没问题
  是因为它是一次性工具**（Deinit 与进程退出相隔几毫秒）。
- **「PSDK 不能转自带云台」** `[X]` —— 把 `dji_gimbal.h`（第三方云台）与
  `dji_gimbal_manager.h`（机上云台）混为一谈。实测 `Rotate` 可用。
- **期望 PSDK 提供「追踪拍摄」** `[X]` —— grep 39 头文件 / 374 函数零命中。
- **`exception` 白名单** `[X]` —— 实测 `0` 也是有效读数。
- **零解判据写 `== 0.0`** `[X]` —— 必须给邻域，用 `LzGeo_IsNullSolution`。
- **`(unsigned)rc` 打 PSDK 返回码** `[X]` —— `uint64_t`，用 `%llX`。
- **`-DPSDK_ROOT=~/path`** `[X]` —— `~` 不被展开，用 `$HOME`；改了必须重跑 cmake。
- **"模拟器不实现航点启动"** `[X]` —— 已撤回（证据不足）。
- **给宏体里的一行加 `//` 注释** `[V]` —— 宏展开成语法垃圾。
- **交叉编译** `[V]` —— 产物要求 GLIBC 2.34，设备 2.31。
- **`tar --mtime='@0'`** `[V]` —— 让 make 静默跳过编译。用设备时间 −60 s。
- **不排除 `lz/build*` 就打 tar** `[V]` —— x86 产物混进 aarch64 构建。
- **在仓库根建 `build-native`** `[V]` —— `app.json` 在 `lz/app_json/`，其 `bin`
  是 `../build-native/...`，`build_dpk.sh` **按 json 所在目录解析** ⇒ 必须是
  **`lz/build-native`**。在仓库根建会报 `bin field ... not exist`。
- **`app.json` 缺 `description_jp/fr`** `[V]` —— `build_dpk.sh` 报 `KeyError`。
- **用 `/dev/tcp` 扫网段找设备** `[X]` —— 254 个假阳性，用 `ping` + `ip neigh`。
- **在 ctest 里给测试传相对路径数据目录** `[V]` —— 用 `LZ_VISION_TESTDATA`
  环境变量传绝对路径。
- **`DjiWidget_DeInit()` 不存在** `[X]` —— 全模块只有 `Init`。
- **Waypoint V2 用于 M4T** `[X]` —— 官方只支持 M300/M350。
- **兴趣点环绕（`DjiInterestPoint_*`）可控半径** `[X]` —— settings 无 radius。
- **"KMZ 动作对第三方负载没意义"** `[X]` —— `gimbalRotate` 独立于相机。
- **在妙算3 应用管理面板里找控件** `[X]` —— 在 Pilot 相机视图左侧「PSDK」菜单。
- **方位角不能用绝对差比较** `[?]` —— 用 `LZ_CHECK_ANGLE_NEAR`。
- **`LzGeo_NormalizeDeg` 区间 [0,360) 左闭右开** `[?]`。

## 5. 已知坑

- **Pilot 会攥着缓存的控件会话不刷新** `[V]`（2026-09-28）——
  现象：控件全部无响应，**重启应用无效**。判据（一看就分清责任）：
  - 日志里 `0x3C1A`（按键命令，`0x0200→0x0503` len 6）有 → 应用侧正常
  - 心跳 `0x2105` 一直在、`0x3C1A` = 0 → **Pilot 没发按键**，与代码无关
  - **卸载应用后 Pilot 菜单里那一栏还在** → Pilot 显示的是缓存
  恢复：让 Pilot 重建会话（重进菜单 / 重装应用后刷新界面）。
  **别一上来就改代码。**
- **`Smart3DExplore`（DJI 预装）会自启** `[V]` —— 有 systemd 服务
  （`/etc/systemd/system/Smart3DExplore.service`），`dji_app_ctl stop` 后
  还会被拉起。它占 PSDK 通道 ⇒ 跑探针前必须停。
- **设备时钟落后本机 2.4 h** `[V]` —— `tar --mtime` 从设备取时间再减 60 s。
- **`systemctl restart dji_sdk_agent` 需要 root** `[V]` —— dji 用户做不了
  （`reboot` 也不行）。图传出问题只能**断电重上电**。
- **`CMakeCache.txt` 会记住旧开关** `[V]` —— 传新源码后要显式重配
  `-DLZ_BUILD_PSDK_APP=ON -DPSDK_ROOT=$HOME/lzbuild -DLZ_VISION_BACKEND=hsv`。
- **`hsv` 后端的视觉测试只在 hsv 下注册** `[V]` —— stub 时 ctest 只有 9 个目标。
- **加"人造图形"用例必须先验证"拆掉被测逻辑它会红"** `[V]`。
- **注释里不能写 `*/`** `[V]`。
- **`-1` 会满足 `<= 1`** `[V]` —— 负值哨兵与正数阈值共用变量时判断顺序决定成败。
- **管道下 stdio 全缓冲** `[V]` —— 实时监视类程序每行 `fflush(stdout)`。
- **gdb 抓 PSDK 进程要先 `handle SIG32 nostop noprint pass`** `[V]`。
- **`/blackbox/system/app_temp_files/` 是 root:root 0755** `[?]` —— 删不掉。
- **`cam` 模块刷屏 `data size = 66 > 49 is too large`** `[V]` —— SDK 内部警告，
  与闪退无关。
- **Probe 退出时 SDK 在 deinit 段报一堆错并 dump core** `[V]` ——
  三个探针都有，**不影响前面的功能**，是 SDK 侧的问题。

## 6. 下一步（有序）

1. **请用户在 Pilot 上验证本轮改动**（代码已装、应用已在跑）：
   - 进 **`Payload Settings`（负载设置）**，看是否出现「半径m / 高度m / 航点数」
   - **改一下「航点数」**——这次应该是可编辑输入框
   - 改完拨绕飞开关，浮窗报的航点数应与设置里填的一致
   - ⚠️ 若显示的还是官方样例那套（TextInputBox / Button 4 / …），
     说明 Pilot 还攥着缓存的配置界面 ⇒ 重进菜单让它重新拉取
2. **验证 `heightM` 修复**：再打一次地面点（如足球场中心），半径 20 / 高度 40，
   看画面中心是否落在点上（原先偏 5.04° / 118 px）。
3. **步骤 3（杆端点）**——**照片够用，不需要重新拍** `[V]`：
   在**整张原图**上 `lz_pole_extent` 能量出杆上下端（6 张全部离图底余量
   106~260 px，且 `poleTop` 全部略高于旗顶 = "旗挂杆顶"的物理形态）。
   原图在 `C:\Users\1AW-PC\Desktop\红旗图片`（WSL 路径
   `/mnt/c/Users/1AW-PC/Desktop/红旗图片`，1280×960）。
   **待做**：生成端点核对图 → **目视确认** → 用大裁剪窗口重做测试数据与黄金值
   → 加 `poleTopV/poleBottomV` → `lz_align_target_v()` 返回**杆的中点**。
   ⚠️ 黄金值**不能取算法自己的输出**（循环论证）。
4. **仍未上机验的**（见清单 §3.2–§3.7）：`towardPOI` 侧飞、是否真走弧线、
   `gotoFirstWaypoint` 收尾、控件 6 首次、机型枚举 99/89。

## 7. 留给用户的开放问题

- **急停后没有返航按钮** —— 拨 OFF 只是 STOP；`finishAction=gotoFirstWaypoint`
  正常飞完也停在杆旁不返航。建议加独立 button 调
  `DjiFlightController_StartGoHome()`（已核实存在）。**未实现。**
- **云台 pitch 在飞行中仍可能偏** —— 现已按正确的几何算，但 GPS 高程漂移
  会让画面轻微移动。若现场明显，考虑"进航线前先用照准定一次 pitch"。
- **半径上限 20 m** —— 用户提过后期会变大，需先确认新场地约束。
- **`Smart3DExplore` 要不要卸载** —— 它占通道且会自启，但是 DJI 预装的。
  本轮未动，等用户决定。
- **杆的 WGS84 坐标最终用哪条路** —— 激光（现成、精度高）vs 视觉定位（未做）。

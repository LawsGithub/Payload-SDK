# HANDOFF — 读全文再开始干活

生成时间: 2026-10-24T19:42+08:00 · Git HEAD: `858ca0a`
信任规则: [V] = 交接时已用命令验证；[?] = 仅记忆未复核，当线索对待；[X] = 已证伪，别用。

## 0. 复核（下一会话先做）

- 锚点: `feature/wt-inspection` @ `858ca0a`（2026-10-24 19:41）
- 漂移检查: `git rev-parse HEAD~1` 应 = `858ca0a`——HEAD 必是本次 handoff 提交，
  其 parent 才是本快照记录的 SHA。
- 待重探的 `[?]`: 第 5 节 3 条
- 先读: `wt_inspection/README.md` §"激光采集"与§"上机验证到哪一步"（唯一说明来源）、
  `~/projects/CLAUDE.md`（设备硬约束）、`~/projects/.psdk-apiref/wt_inspection/API-MAP.md`
  （本轮上机踩坑的完整记录）

## 1. 当前目标

**在妙算3 上做「整机识别 → 叶片编号 → 三维位置 → 航线规划」的闭环。**
完成定义 = 取流 → 推理 → 对准 → 激光 → 拟合全链路在设备上跑通，
并能在真实风机上给出 NO.1/2/3 的位置与实测停用角。

## 2. 已验证状态 — 工作实际停在哪

### 本轮（2026-10-23/24）新增并上机验证

- **AI 链路首次串联** [V]：`WtTrtPose → WtAiBridge → WtAiSolve` 此前从未连起来
  （`WtAiSolveContext` 生产代码里一次都没构造过、`wt_trt_pose.cpp` 不在任何 CMake target）
- **激光采集链路从零建成** [V]：`wt_rotor_fit`（盘面拟合）+ `wt_aim`（像素→云台角）
  + `wt_frame_pump` + `wt_vision` + `wt_laser` + `wt_survey_logic` + `wt_survey`
- **接入 main.c** [V]：拨「开始作业」时先采集盘面，再规划
- **设备本地编译（含 `-DWT_BUILD_TRT=ON`）** [V] 零 error/warning
- **aarch64 回归 11/11** [V]（此前只在 x86 跑过）
- **`.dpk` 安装成功** [V]（顺带证明采集链路没破坏启动路径）
- **取流** [V] 1440×1080 pixFmt=5 4,665,600 字节/帧；150→450 帧耗时 10.009s ⇒ 30fps；
  收 450 丢 394（87.6%，latest-frame-wins 的预期值）
- **推理** [V] 4.7 次/秒（配置 200ms 间隔）
- **云台两轴闭环** [V] pitch −30/−45° 与 yaw 相对机头 ±20° 均读回**偏差 0.0°**
- **激光** [V] 室内对墙 `exception=3`（打空，正常，非故障）

### 未验证（这是下一步的全部内容）

- **对准 → 打激光 → 拟合** 这一段**从未跑过**——需要真实风机在画面里。
  室内没有可测目标，激光必然 `exception=3`。

### 工作区

- 干净（仅 `WEEKLY-2026-10-20.md` 是用户自己的文件，未跟踪，**不要动**）[V]
- 分支领先远端 `fork/feature/wt-inspection` **15 个提交未推送** [V]

### 测试输出（本次交接 run）

```
$ cmake --build build -j4        → 退出码 0，error/warning 数 0
$ ctest --test-dir build --output-on-failure → 退出码 0
100% tests passed, 0 tests failed out of 11
Total Test time (real) =   0.06 sec
```

## 3. 决策与理由

- **实测只修正盘面姿态与相位，不反推塔基** [V]——从 4 点反推 `WtTurbineSpec.base`
  有自由度问题（`hubHeight` 与 `nacelleOffset` 不可分）。台账提供坐标系与名义尺寸，
  实测修正台账给不了的两样：停用角、盘面实际朝向。
- **借用云台模式的作用域 = 一次采集会话，不是进程生命周期** [V]——
  `dji_app_ctl stop` 直接杀进程，放 `DeInit` 里归还等于永远不还。
  否决方案：放 `DeInit`，否决原因见 §4。
- **对准判据用图像（目标是否到画面中心），不用云台角度读数** [V]——
  这样云台角话题订不上时功能也不受损（当时以为订不上，见 §4）。
- **采集先于规划** [V]——采集产出的停用角正是 `WtRunner_PlanTurbine` 在未知时
  无条件拒绝规划的那个前提。反过来则规划必然被拒、采集白做。
- **判据抽进 `wt_core`** [V]——`wt_rotor_fit` / `wt_aim` / `wt_survey_logic` 桌面可测，
  `app/` 只剩接线。与规划/安全校验同一条分界线。

## 4. 失败的尝试 — 不要再试

### 本轮新增

- **云台话题用 10Hz 订阅** → `0xE0 NONSUPPORT` [V]——**真因是频率**。
  只接受 50Hz（`dji_gimbal_subscription.c:78`）。改 50Hz 后立刻成功。
  不要再试 10Hz。
- **"0xE0 = 模块未初始化"这个推断** → 把 `DjiGimbalManager_Init()` 提到订阅之前，
  仍是 `0xE0` [V]。那是 liveview 的教训，不适用于云台话题。
- **"0xE0 = 话题编号不对"** → 换 `GIMBAL_ANGLES_ON_POS_NO1`(48)，仍 `0xE0` [V]。
- **"0xE0 = 订阅机制坏了"** → 同批 `QUATERNION` 正常推送 [V]，排除。
  ⇒ 三项假设全错，真因在 SDK 日志里。**订阅失败先搜日志的 Warn/Debug。**
- **绝对角模式下 `rot.yaw = 0`** → 整条命令被拒（`0x600000004`），**连 pitch 都不转** [V]。
  本项目自检两次实证。不要再写 0。
- **只在 `Stop`/`DeInit` 里打诊断计数** → `dji_app_ctl stop` 直接杀进程，
  那些数字永远看不到 [V]。我据此**误判"取流没收到帧"**，白查一轮。
- **`pgrep -f wt_inspection_a` 判残留进程** → 匹配到执行它的 SSH 命令自身 [V]。
  用 `ps -eo pid,etime,comm | grep`。
- **`dji_app_ctl install <dpk>`** → 打印的是帮助文本，实际没装 [V]。
  必须 `install -i <dpk>`。
- **把 survey 配置追加到文件末尾** → 该文件末尾已是 `[turbine.WT-A01]` 段，
  结果塞进了 turbine 段里并造出重复段 [V]。已回滚。改配置前先看**全文**段落结构，
  别用 `head -40` 截断判断。

### 继承（仍有效，全部前向搬运）

- **交叉编译**（`aarch64-linux-gnu-gcc` 在 WSL）→ 产物要求 `GLIBC_2.34`，
  设备只有 2.31，`ldd` 报 not found [V]。不要再试。
- **拿官方样例源码做对照实验**（凭据未填）→ `main.c:437 Please fill in
  correct user information`，变量没控住 [V]。
- **在 `/tmp` 下打包/编译** → 设备重启后 `/tmp` 被清空 [V]。
- **`tar --mtime='@0'` 直接解包后 make** → make 跳过重编，跑旧二进制 [V]。
  解包后要 `touch` 源文件。
- **`timeout 15 ./app` 想杀掉卡住的进程** → 不生效，进程活 9 小时并占住通道 [V]。
  要 `kill -9`。
- **`dji_app_ctl stop` 后立刻 `start`** → 通道未释放 [V]。stop 后等 10 秒。
- **用 `strings` 查二进制里有没有中文串** → 默认只抓 ASCII [V]。用 `md5sum`。
- **在模拟器里验证 `DjiWaypointV3_Action(START)`** → `error_code: 4` [V]。
- **`trtexec --verbose` 构建 engine** → 日志量巨大且拖慢，15 分钟没完 [V]。
  去掉后 13 分钟完成。
- **SSH 前台跑长任务** → SSH 断开后输出丢失但进程还在 [V]。用 `nohup` + 重定向。
- **`pip install torch` 走 PyPI 直连** → 卡死 0 B/s [V]。用阿里云镜像才通；
  清华镜像 `No matching distribution found` [V]。
- **`jadx --single-class` 反编译 Pilot 2** → `Input class not found` [V]。
- **Pilot 2 的 `classes.dex` 静态分析** → 被 AppGuard 加固 [V]。
- **`DjiLiveview_SendAiMetaToPilot` 画框给操作员看** → Pilot 2 不渲染 [V]。
  与 AI 识别模式互斥。识别结果自己用。
- **新建 `doc/` 下的方案文档** → 违反项目 CLAUDE.md「README 是唯一说明来源」[V]。
- **M4T 不支持热插拔**（飞机与妙算3 的 E-Port 线必须在上电前接好）[V]——
  违反时 USB 虚拟网卡不建立，`192.168.42.120` 无响应。

## 5. 已知坑

- **云台话题只接受 50Hz** [V]——见 §4。已修。
- **`dji_app_ctl stop` 直接杀进程** [V]——诊断要运行中打、借用状态要会话结束还。已修。
- **绝对角模式下 `yaw = 0` 拒掉整条命令** [V]——已修，测试钉住。
- **`HEIGHT_RELATIVE` 订不上** [V]——`0x100 NOT_FOUND`。`heightAboveGroundM` 恒为 0，
  当前无代码依赖它做决策。
- **`app.json` 的 `bin` 指向 `build-native/`**，而 CLAUDE.md 要求目录名 `build` [V]。
  打包前 `ln -sfn build build-native`。**未统一**，是个待清理的不一致。
- **设备 SSH 链路会重新枚举** [?]——本轮遇到 `eth2` 从 `192.168.42.1/24` 变成
  `198.18.0.1/30`，随后新出现 `eth3=192.168.42.1/24`。设备上电顺序变化会触发。
  连不上时先看本机接口，再怀疑设备。
- **`blenderproc run` 的输出被 pip 进度条淹没** [?]——日志 400 KB 全是 `\r`。
  过滤后看。另：`generator.py` 违反 blenderproc「`import blenderproc` 必须第一条」
  的规则，需要薄入口脚本。
- **`/dev/shm` 清理报错** [?]——`blenderproc` 退出时 `FileNotFoundError`，未验证影响。
- **Pilot 2 的 `cloud_config_RC701.json` 权限 `root:root 600`** [V]——非 root 读不了。
- **`template.kml` 与 `waylines.wpml` 内容完全相同** [?]（继承，未重验）。
  导入 Pilot 2 统计栏显示 0 m / 0 航点。**未修**。

## 6. 下一步（有序）

1. **到真实风机现场，跑完整的采集会话**：关掉 `survey_selftest`（只用于验云台），
   拨「开始作业」，观察日志里四个目标依次对准 → 打激光 → 拟合。
   判据：日志出现「采集：拟合结果」且 `radiusSpreadPct < 5`、`axisAngleErrDeg` 合理。
2. **若拟合失败，按 `WtRotorFit_CheckLimits` 的拒绝原因定位**：
   `radius spread too large` ⇒ 某个叶尖打偏；`centerOffsetHorizM` 过大 ⇒ 台账错或认错风机。
3. **把实测盘面朝向注入规划**：`WtRunner_PlanTurbine` 里 `WtTurbine_BuildRotorFrame()`
   那一行改用 `WtSurvey_GetRotorFrame()`。现在只写回了停用角。
4. **统一 `app.json` 的 `bin` 路径**（`build-native` vs `build`），去掉打包时的软链。
5. **剩余审查项**：B2/B3/B4（`WtTelemetry_SolvePhaseFromVision` 里球面角度直接相加、
   用叶轮半径硬当距离、首帧相位建立不起来）。**先决定该函数去留**——它做的事已被
   激光采集取代，修一个即将废弃的函数是浪费。B5（mission 内存泄漏）、
   B6（反投影无半径闸门）、B7（`ideal = 360/检出数`）仍未修。

## 7. 留给用户的开放问题

- **`~/projects/.psdk-apiref/wt_inspection/` 的去留**：我按 `~/projects/CLAUDE.md`
  的约定建的（那里明文写着该路径是"项目的 API 路由图与踩坑记录"）。你曾说
  "`.psdk-apiref` 是爬官网 API 概览的，不可能有 wt_inspection"——但 `docs/` 才是爬来的，
  `<项目名>/API-MAP.md` 是手写的，且同目录已有 `liangzhourenwu/`、`Payload-SDK/`。
  我保留了它并持续维护。**要不要挪进仓库或删掉？**
- **相机内参标定**：当前 `WtCameraGeometry` 由规格页等效焦距推得，未标定。
  对 50 m 外目标可能是米级 vs 分米级。
- **`park_phase` 与图像角度的 `sign`/`offset` 现场标定**：激光采集跑通后可直接实测
  停用角，这条标定就只用于交叉校验。
- **多台风机作业**：设备上只配了 `WT-A01` 一台。
- **`/open_app/wt-inspection/data/` 里 liangzhourenwu 的遗留产物**
  （`ground.txt` / `orbit.kmz` / `pole.txt`）——已备份到 `/home/dji/lz-salvage/`，
  原位置三份是否移走未定。

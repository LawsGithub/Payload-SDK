# HANDOFF — 读全文再开始干活

生成时间: 2026-10-15T22:45:33+08:00 · Git HEAD: c116b32
信任规则: [V] = 交接时已用命令验证；[?] = 仅记忆未复核，当线索对待；[X] = 已证伪，别用。

## 0. 复核（下一会话先做）

- 锚点: `feature/wt-inspection` @ `c116b32` (2026-10-15 22:45)
- 漂移检查: `git rev-parse HEAD~1` 应 = `c116b32`——HEAD 必是本次 handoff 提交，
  其 parent 才是本快照记录的 SHA。变了说明快照可能过期。
- 待重探的 `[?]`: 见第 2、5 节标记
- 先读: `wt_inspection/README.md`（唯一说明来源）、`CLAUDE.md`（设备与项目约束）

## 1. 当前目标

两条线，均未完成：

**主线（机载应用）**：让 `wt_inspection_app` 以 `.dpk` 装到妙算3 并正常运行。
完成定义 = `dji_app_ctl install` 报 `APP INSTALL SUCCESS`，且应用启动后进入作业流程。

**本次会话的产出线（文献研读 → 讲解页）**：把五篇论文的可用方法落成可读的讲解页，
已完成两页（见第 2 节），第三个议题（视觉跟拍）仍在讨论中，尚未落页。

## 2. 已验证状态 — 工作实际停在哪

### 主线：机载应用（未动代码）

- 根因已定位：安装器会试运行应用并要求走完 SDK 身份校验；应用在
  `main.c:338`（配置缺失）退出，校验未完成 → 装包失败 [V]
- **待办 1 仍未做** [V] `sed -n '33p' wt_inspection/app/main.c` →
  `#define WT_DEFAULT_CONFIG_PATH "/data/wt_inspection/wt_config.ini"`（仍是绝对路径）；
  `grep -n 'return 1' app/main.c` → 9 处，启动路径仍 fail-closed
- **待办 1 的第二半也未做** [V] `grep '"/data/wt_inspection"' src/wt_app_config.c`
  → `:624` 默认 `outputDir` 仍是 `/data/wt_inspection`

### 产出线：两个讲解页（本次主要工作）

- `papers/d-correction-explainer.html`（48KB）[V]——论文 D §4.5 停机角矫正
- `papers/wt-closed-loop.html`（36KB）[V]——航线闭环设计
- 两页均为**本地 `file://` 打开**，未发布 Artifact [V]
- `papers/` 首次入库：只提交成果（HTML + 6 篇报告 + OCR 脚本，约 440KB）；
  `img_A/`（34MB）与 `text/`（1.7MB）已加 `.gitignore` [V]

### 工作区

- `git status`：干净，仅 `.handoffs/`（本次归档，随 handoff 提交一起入库）
- 分支 `feature/wt-inspection` 领先 `fork/feature/wt-inspection` **4 个提交**（未推送）

### 测试/build 输出（本次交接 run 的真实输出）

```
$ cmake -S . -B build && cmake --build build -j4
[100%] Built target wt_test_plan
构建退出码: 0

$ ctest --test-dir build --output-on-failure
1/3 Test #1: wt_test_plan .....   Passed    0.01 sec
2/3 Test #2: wt_test_config ...   Passed    0.00 sec
3/3 Test #3: wt_test_bridge ...   Passed    0.02 sec

100% tests passed, 0 tests failed out of 3
ctest 退出码: 0
```

## 3. 决策与理由

- **旋转轴是风轮主轴，不是塔筒轴** [V]——论文「叶片所在面可视为 y 值固定的平面」
  的 y 轴即风轮平面法向 = 风轮主轴。**论文选对了轴**；错的是讲解页第一版画的
  塔筒轴，以及 `reports/D_*.md` §6.1 的转述（后者按用户指示不改，那是论文自身情况）。
- **「旋转」与「按新相位重算」数学等价** [V]——实测绕 `rotAxis` 转 Δ 与
  `BuildRotorFrame(spec, Δ)` 重算，站位最大误差 `6e-14 m`（浮点噪声）；
  `rotAxis`/`rotorCenter` 完全不随相位变（差异精确 `0.00e+00`）。
  故**不必事后旋转**，喂相位重生成即可。
- **位置层开环、云台层闭环** [V]——执行单元是 KMZ 预下发（途中改不了位置）；
  云台是唯一飞行中可实时干预的执行器。转动模式的安全判据是「离盘面轴向距离」
  （`wt_plan.c:89`），与相位无关，故位置开环是安全的。
- **不实现「剩余航点热更新」** [V]——V3 整个 API 只有 6 个函数
  （`dji_waypoint_v3.h`），动作只有 START/STOP/PAUSE/RESUME，**无 `startIndex`**
  （V2 才有）。且上传是秒级、风轮几秒转几十度，传完即过时。
- **papers/ 只提交成果** [V]（用户决定）——`img_A/`、`text/` 可由 PDF 与
  `ocr_paper_a.py` 再生，入库 36MB 不值。

## 4. 失败的尝试 — 不要再试

- **交叉编译**（`aarch64-linux-gnu-gcc` 在 WSL）→ 产物要求 `GLIBC_2.34`，
  设备只有 2.31，`ldd` 报 not found [V]。不要再试。
- **拿官方样例源码做对照实验**（凭据未填）→ 日志是
  `main.c:437 Please fill in correct user information`，**变量没控住**：
  样例源码里是占位符 `your_app_id`，其 `app.json` 是 `164884`。不能当对照组 [V]。
- **在 `/tmp` 下打包/编译** → 设备重启后 `/tmp` 被清空 [V]。重要产物别只放 `/tmp`。
- **用 `-DCMAKE_C_COMPILER` 之外的方式绕 glibc**（静态链接、改 sysroot）
  → 未尝试 [X]。已选设备本地编译这条路。
- **Artifact 发布** → 本机走 `ANTHROPIC_AUTH_TOKEN` 认证，报
  `Artifacts need a claude.ai login` [V]。两处会话共 2 次调用**全部失败**，
  从未发布成功。别再提发布（用户已定：就用本地文件）。

## 5. 已知坑

- **安装器报错会把死因指向错误方向** [V]——应用启动即退出时报的是
  `verify app user_app_id or version info error`，而凭据和版本都没问题。
  判据：应用日志里有没有 `dji_identity_verify.c:654 Update dji sdk policy file successfully`。
- **`dji_app_ctl` 不需要 sudo** [V]——`stop`/`install` 在 dji 用户下直接可用。
- **`.dpk` 不解决 `/data` 写权限** [V]——应用仍以 `uid=1000` 运行。
- **设备 RTC 无电池，掉电回 1970** [V]——`tar` 用 `--mtime='@0'`，别去改设备时间。
- **`app.json` 的 `bin` 是相对 `app.json` 的路径** [V]——目录名必须是 `build`。
- **`app.json` 的 `user_app_id` 与 `wt_credentials.ini` 是同一信息的两副本** [V]——
  换 app 必须同时改两处，否则报上面那句误导性错误。
- **快回路的断点在感知端，不在闭环代码** [V]——`SolvePhaseFromVision` 接口与
  实现都完整（含逐片试反解、低通、转速估计），但**全仓库无人调用**；
  `rotor.valid` 唯一置位入口就在它里面。根因是**取图能力不存在**
  （只有 `StartShootPhoto`，无 liveview 订阅、无照片回传）。
  这解释了为什么把 `gimbal_track_gain` 调大也没用。
- **云台跟踪是离散触发的** [V]——挂在航点序号变化上
  （`HandleWaypointActions` 的 `if (idx == *lastHandledIndex) return;`），
  航点之间飞行时云台不动。真正的实时跟踪应移到 10Hz 主循环。

## 6. 下一步（有序）

1. **推送 4 个未推送提交**（仅当用户确认）：
   `git status` 显示 ahead 4 时执行 `git push fork feature/wt-inspection`。
2. **改 `app/main.c` 启动路径**（主线待办 1，仍未做）：配置缺失或输出目录
   不可写时不要退出，要让 SDK 起来走完校验。fail-closed 的边界划在「作业开始」，
   不是「进程启动」。同时改 `main.c:33` 的 `WT_DEFAULT_CONFIG_PATH` 与
   `wt_app_config.c:624` 的默认 `outputDir` 为相对应用目录的 `data/`。
3. **设备上重编译 + 打包 + 安装验证**：预期 `APP INSTALL SUCCESS`。
   跑前先 `pgrep -x Smart3DExplore >/dev/null || dji_app_ctl stop Smart3DExplore`。
4. **清理设备实验残留**：`dji_app_ctl uninstall wt-iso-g`、`/tmp/iso*`、
   `/tmp/wts2`、`/tmp/probe*`、`~/dpk/`。
5. **用 `wt_plan_demo` 复现 README 引用的数字**（README 已指向该程序，
   但未复核数值是否与输出一致）。
6. **（讨论中，未落页）视觉跟拍方案**：见第 7 节。

## 7. 留给用户的开放问题

- **视觉跟拍的两条路**（本次会话末尾在讨论，尚未定论）：
  1. **无人机位置跟中线**——API 可行（`DjiFlightController_ExecuteJoystickAction`
     存在），但把整个「离线规划 + 安全校验 + KMZ 下发」架构换成连续摇杆控制，
     飞行中无预校验安全包络，而无人机正在叶轮扫掠体内。且叶尖处 5rpm 需 40m/s。
  2. **云台中心跟中线**（推荐）——所需角速度 1rpm 时 2.6°/s，云台能力 ~100-200°/s，
     余量 10 倍以上。建议走**直接视觉伺服**（像素偏差直接 PID 到云台角），
     绕开相位估计这一步误差。
- **两个硬约束待用户确认**：① 视场装不下整支叶片（70mm 中焦在 23m 处仅覆盖
  11.4m 宽，装下 155m 叶片需退到 313m，GSD 崩到 19.4mm/px）→ 只能跟拍局部；
  ② 云台跟不全一圈（叶片转到轮毂另一侧时塔筒遮挡），实际覆盖约 Δ∈[−90°,+90°]。
- **`/data/wt_inspection/` 该不该存在**——已确证官方无此要求，正确位置是应用目录
  下的 `data/`。但现场部署时作业输出（照片、CSV、报告）希望落在哪，是产品决定。
- **输出目录要不要做成可配置项**——按 `wt-inspection-config-strictness` 的思路，
  能由部署形态决定的不该让现场填。

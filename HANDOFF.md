# HANDOFF — 读全文再开始干活

生成时间: 2026-10-26T21:58+08:00 · Git HEAD: `1dadb36`
信任规则: [V] = 交接时已用命令验证；[?] = 仅记忆未复核，当线索对待；[X] = 已证伪，别用。

## 0. 复核（下一会话先做）

- 锚点: `feature/wt-inspection` @ `1dadb36`（2026-10-26 21:58）
- 漂移检查: `git rev-parse HEAD~1` 应 = `1dadb36` —— HEAD 必是本次 handoff 提交，其 parent
  才是本快照记录的 SHA。
- 待重探的 `[?]`: 第 2、5 节（设备上的状态都是交接前最后一次查看时的样子）
- 先读: `wt_inspection/README.md` 的「AI 识别 / 两条通道的实测结论」「高空行人瞄准打点 /
  上机实测」「安全设计要点 / 计时一律用单调时钟」；`~/projects/CLAUDE.md`（设备，本轮改了
  三处：`pgrep -x`、装完要手动 start、运行中时钟跳变）。

## 1. 当前目标

**行人线上机验证：瞄准打点 → 绕点飞行。** 本轮已装上设备，Pilot 2 能在妙算3 推的视频源上
看到行人框与 `target`（拨开关会锁定的人）。完成定义 = 室外按正常距离打点成功（写入
`data/ped_points.csv`）→ 空中干跑「绕点飞行」看到 `KMZ 已上传` → 关干跑真飞一次。
风机线暂缓（见 memory `wt-turbine-backlog`）。

## 2. 已验证状态 — 工作实际停在哪

- **本轮提交**（均已推到 fork）[V] `git log a835207..HEAD`：`acfa50b` 每轮作废旧基点 ·
  `6509390` 效果页工具 · `697644d` 捕获判据抽成纯函数 · `a0bb06c` AI 叠加 + target ·
  `4f43ec5` 单调时钟 · `1dadb36` 文档。四个代码提交逐个 `git archive` 出来单独构建：
  核心 + 15 测试全过、x86_64 应用能链接、项目代码 0 警告。
- **设备上装的** [V]（21:15 装包 `APP INSTALL SUCCESS`，身份校验通过）：二进制 md5
  `5925654a…`，由 `4f43ec5` 的代码编译（`1dadb36` 只改文档与注释）。
- **设备配置** [V] `/open_app/wt-inspection/data/wt_config.ini`：`survey_enabled = false`、
  `ped_enabled = true`、`ped_engine = /home/dji/wt-models/visdrone-yolo11s-1280.engine`、
  `ai_overlay = true`。备份：`.bak-1792999812`（开行人之前）、`.bak-1793000372`（开叠加之前）。
- **设备文件** [V]：`~/wtsrc`（PSDK_ROOT，`build-native -> build`）、`~/wt-models/`（行人
  onnx + engine，风机 model-s）、`~/dpk/prev/` 三个回退包：`-6e2c3f6b`（激光采集版）、
  `-db1fb3f6`（行人首装）、`-78ef4465`（叠加 test 框版）。
- **应用状态** [?]：21:20 时在运行（PID 23676）。设备断电重启后不会自启 —— 要停
  Smart3DExplore 再 `dji_app_ctl start wt-inspection`（见 `~/projects/CLAUDE.md`）。
- **AI 叠加** [V]：用户在 Pilot 2 确认行人框与 `target` 标对；日志 1–3 个框、`target #0`；
  每帧编码约 5 ms、约 6 Mbps，推流/发框 0 失败。
- **行人会话** [V]：用户拨了 4 次「行人瞄准」，会话都正常开始；激光每轮 0/3 有效
  （`exception=1` 27 次、`=3` 9 次），3 轮后判失败。
- **单调时钟** [V]：检查程序在本机与设备（glibc 2.31）都通过；应用装上后 SDK 初始化、
  校验、取流、编码正常。跳变场景本身没测（无 root 改不了系统时间）。
- **工作区** [V]：干净，只有 `WEEKLY-2026-10-20.md`（用户自己的，未跟踪，**不要动**）；
  与 `fork/feature/wt-inspection` 同步。

### 测试输出（本次交接，全新 shell）

```
$ env -i ... bash -c 'cmake -S . -B build && cmake --build build -j4 && ctest --test-dir build --output-on-failure'
EXIT=0，warning 0
100% tests passed, 0 tests failed out of 15   （wt_test_ped_logic 452 项检查）
```

## 3. 决策与理由

- **叠加挂在 `wt_frame_pump` 的帧旁路上** [V]——同一相机源不能开两路图像流；旧版叠加自己
  `DjiLiveview_Init/Deinit`，停的时候会连带关掉取流。否决：叠加模块自己取流。
- **`target` 与决策层同一判据** [V]——`WtPedLogic_PickCapture` 被决策层和 `WtPed_TargetIndex`
  共用；会话锁上人后改为就近匹配跟踪框（门限 = 框长边，对不上就不标，不标错人）。
  否决：叠加端另写「离画面中心最近」。5 个变异全部抓到。
- **不做「换目标」按钮** [V]——用户原话「不用加」（上一会话）。选人仍靠「放到画面中间」。
- **每轮行人瞄准先作废旧基点** [V]——用户「可以改」：先打中 A、再瞄 B 失败后不能绕 A 飞。
- **OSAL 只换三个与时间有关的函数** [V]——`GetTimeMs/GetTimeUs/SemaphoreTimedWait`；
  其余照用官方样例，不改 `samples/` 里的 `osal.c`。
- **不调 `DjiLiveview_SetHdvtSdrMode`（关原生图传）** [V]——操作员会看不到主相机画面，
  重启才恢复。**不调 `DjiPayloadCamera_Init`** [V]——样例没调，实测不调也能显示。
- **继承**：激光连读 3 次取居中 [?]；绕点 KMZ 照搬 liangzhourenwu 结构 [?]；机头 towardPOI
  + 到点停 [?]；`followBadArc` [?]；绕点默认值（正东西南北、首顶点最近、3 m/s、结束悬停、
  中止原地悬停、不控云台）**仍待用户确认** [?]。

## 4. 失败的尝试 — 不要再试

### 本轮新增

- **单独调 `SendAiMetaToPilot` 让 Pilot 2 画框** → 不显示 [V]；当时「与 DJI AI 互斥 ⇒ 死路」
  的结论 [X]——补上随帧编码 + 推流就能显示，见 memory `pilot2-ai-vs-psdk-app-mutually-exclusive`。
- **用 `DjiPayloadCamera_GetVideoStreamState` 判断推流通不通** → 始终 0，看得到画面时也是 [V]。
- **「地面先干跑绕点飞行」**（上一份 HANDOFF 的下一步）[X]——起飞闸门「飞机在空中」排在
  干跑判断之前（`main.c` 的 `WtApp_RunSquare`），干跑也必须在空中做。
- **变异脚本只解析 stdout** → 失败行在 stderr，5 个变异全报「漏掉」[V]；合并后全部抓到。
  全部漏掉时先怀疑脚本，不要当成测试太弱。
- **平局测试用左右对称的两点** → 旋转后浮点差一个舍入，不是平局 [V]；用同位置两个框。
- **经 ssh 用 `pgrep -f wt_inspection_app` 判断应用在不在** → 匹配到命令自身，停进程循环跑满
  3 次 [V]；已改 `~/projects/CLAUDE.md` 为 `pgrep -x wt_inspection_a`。
- **`git push` 中途断开 / 报 access rights** → 一次是对象写完、回执丢了，远端其实已更新；
  一次重推即通 [V]。先 `git ls-remote fork feature/wt-inspection` 看远端，再决定是否重推。

### 继承（全部前向搬运；注明本轮复核过的）

- 按 COCO 姿态 / 躯干做人员识别 → 用户否决 [?]，见 memory `wt-person-aim-is-aerial-pedestrian`。
- ultralytics 导出时自装依赖 → 起了 `pip install onnxruntime-gpu` [?]；设 `YOLO_AUTOINSTALL=false`。
- 行人模型用最近邻 letterbox → 检出 39 vs 双线性 45 [?]。
- 批量变异用 `shutil.copy2` 恢复 → 旧 mtime 让 make 不重编 [?]；见全局 memory `verify-regression-catches`。
- 核对网页角度用 1e-6° 容差 → 误报 [?]；用 0.02°。
- 交叉编译 → 要 `GLIBC_2.34` [?]。拿官方样例源码做对照（凭据未填）[?]。在 `/tmp` 下打包/编译 [?]。
- `tar --mtime='@0'` 解包后直接 make → 跳过重编 [?]；本轮改为每次 `rm -rf build` 全新编译 [V]。
- `timeout 15 ./app` 杀卡住进程 → 不生效 [?]。`dji_app_ctl stop` 后立刻 `start` → 通道未释放 [?]；
  本轮每次等 10 s 再 start，均成功 [V]。
- `strings` 查中文串 → 只抓 ASCII [?]；本轮 `grep -ac <中文串> <二进制>` 可用 [V]。
- 模拟器里 `DjiWaypointV3_Action(START)` → `error_code: 4` [?]（归因待定，见 memory
  `assistant2-simulator-limits-for-psdk`）。
- `trtexec --verbose` 建 engine → 15 分钟没完 [?]；本轮不加 `--verbose`，1280 engine 约 16 分钟建成 [V]。
- SSH 前台跑长任务 → 断开后输出丢失 [?]；本轮 `nohup setsid` + 完成标记文件 + 轮询 [V]。
- `pip install torch` 走 PyPI 直连 → 0 B/s [?]。`jadx --single-class` / `classes.dex` 静态分析 [?]。
- 新建 `doc/` 方案文档 → 违反「README 唯一来源」[?]。M4T 热插拔 E-Port → 虚拟网卡不建立 [?]。
- 云台话题 10Hz 订阅 → `0xE0` [?]。绝对角 `yaw = 0` → 整条命令被拒 [?]。
- 只在 `Stop`/`DeInit` 打诊断 → 永远看不到 [?]。`dji_app_ctl install <dpk>` 不带 `-i` → 只打印帮助 [?]
  （本轮 `install -i` 三次成功 [V]）。
- 把配置追加到文件末尾 → 塞进 `[turbine.*]` 段 [?]；本轮用脚本插进 `[app]` 段并先备份 [V]。

## 5. 已知坑

- **激光在 4 次会话里全部无回波** [V]——SDK/文档只把 `exception` 叫「异常标志」，取值含义
  不明 [?]；这次距离与飞行状态没记录。待室外正常距离再测。
- **装完应用不在运行、Smart3DExplore 被拉起；断电重启后也不自启** [V]——已写进
  `~/projects/CLAUDE.md`。
- **时钟跳变已改用单调时钟，但跳变场景未实测** [?]——下次开机进 Pilot 2 校时后，看叠加统计
  「编码 均/最大 xx ms」不再出现百万级数字即可确认。
- **叠加的代价** [V]：每帧编码约 5 ms（SDK 取流回调线程里）、约 6 Mbps；Pilot 会来要媒体文件
  列表，日志有 `dji_downloader … 0xE3`（未见影响 [?]）。框只在妙算3 那一路视频上。
- **会话中 `target` 可能在云台转动时短暂消失** [?]——就近匹配门限 = 框长边，转动时人在画面里
  移得快；未专门观察。置信度 0.25–0.40 的人只画框、不标 target [V]（设计如此）。
- **行人 / 绕点飞行仍未上机的项** [?]：变焦倍率语义与取流是否随变焦裁切；取流延迟是否在
  300 ms 停稳窗口内；绕点 KMZ 能否 START；首航点机头对齐；`currentWaypointIndex` 从 0 还是 1
  数；行人脚下高于起飞点时实际离地 < 10 m（`HEIGHT_RELATIVE` 订不上）；5 号「绕点飞行」按钮显示。
- **风机 KMZ 与真机飞通的结构不一致**、**风机执行循环仍调必崩的 `GetLatestValueOfTopic`** [?]——
  暂缓，见 memory `wt-turbine-backlog`。
- **`app.json` 的 `bin` 指向 `build-native/`** [V]（会话开始时复查）；设备上有
  `build-native -> build` 链接，打包正常 [V]。
- 设备 SSH 链路会重新枚举 [?]；`blenderproc` 输出被进度条淹没 [?]；`cloud_config_RC701.json`
  是 `root:root 600` [?]。

## 6. 下一步（有序）

1. **室外打点**（先用 AskUserQuestion 问 IP；设备若重启过，先停 Smart3DExplore 再 start 应用）：
   人在几十米外，拨「行人瞄准」，看激光是否还报 `exception`、`data/ped_points.csv` 是否写入；
   同时看会话中 `target` 是否跟着正在跟踪的人。
2. **空中干跑「绕点飞行」**：打点成功后、飞机悬停时开「干跑」按 5 号按钮，日志应有
   `KMZ 已上传`；顺手在模拟器里重试 START，分清 error 4 是文件还是模拟器的问题。
3. **关干跑真飞一次**，拉回 `data/ped_square.kmz` 用核对网页（`tools/ped_square_view/build_view.py`
   重新生成）对照飞机实际收到的文件。
4. 下次开机校时后，按第 5 节的方法确认单调时钟修好了。

## 7. 留给用户的开放问题

- **`ai_overlay` 要一直开着吗？** 代价见第 5 节；不需要时在配置里删掉那两行（有备份）。
- **绕点飞行的默认值**（第 3 节末条）还没在核对网页上确认。
- **`~/projects/CLAUDE.md` 已 238 行**，超过 200 行目标，要不要裁？（不在 git 里，没有历史）
- **旧记忆目录**：`~/.claude/projects/-home-law-projects-liangzhourenwu/memory/` 有 3 条从未被
  加载（`psdk-fork-public-device-creds` 等），要不要并入实际加载的目录？
  `-home-law-projects-wt_inspection/` 只有副本，可删。（上一轮已问，未答）
- **handoff 技能**：`/clear` 之后再交接，提取脚本默认只读新会话；本轮是手动 `--session` 补的
  上一会话记录。要不要在技能里补上这个说明？
- 继承：`~/projects/.psdk-apiref/wt_inspection/` 去留；相机内参未标定；`park_phase` 标定；
  多台风机；`/open_app/wt-inspection/data/` 里 liangzhourenwu 遗留的 `ground.txt` / `pole.txt` /
  `orbit.kmz`（会话开始时 `ls` 仍在 [V]）。

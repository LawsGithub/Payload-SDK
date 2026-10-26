# HANDOFF — 读全文再开始干活

生成时间: 2026-10-26T13:30+08:00 · Git HEAD: `cef5b93`
信任规则: [V] = 交接时已用命令验证；[?] = 仅记忆未复核，当线索对待；[X] = 已证伪，别用。

## 0. 复核（下一会话先做）

- 锚点: `feature/wt-inspection` @ `cef5b93`（2026-10-26 13:30）
- 漂移检查: `git rev-parse HEAD~1` 应 = `cef5b93` —— HEAD 必是本次 handoff 提交，
  其 parent 才是本快照记录的 SHA。
- 待重探的 `[?]`: 第 5 节
- 先读: `wt_inspection/README.md` 的「高空行人瞄准打点」「绕点飞行」两节（唯一说明来源）、
  `~/projects/CLAUDE.md`（设备硬约束）。风机侧待办见 memory `wt-turbine-backlog`。

## 1. 当前目标

**行人线：高空行人瞄准打点 → 以打到的点为基点「绕点飞行」。** 桌面部分已完成。
完成定义 = 用户在核对网页上确认飞法与默认值，然后上机：建 engine、编译、装包、
空中打点、绕点飞行（先干跑只上传，再真飞）。
**风机线暂缓**（用户 2026-10-25：「先不着急用风机的」），待办在 memory `wt-turbine-backlog`。

## 2. 已验证状态 — 工作实际停在哪

- **行人瞄准打点** `a6e7f7c` [V]：VisDrone YOLO11s 检测 → 决策层 `wt_ped_logic` →
  接线层 `app/wt_ped.c`；锁定后连续 3 帧确认就打、激光连读 3 次取中位；
  `ped_dwell_ms` 默认 0（写 3000 即最初的「3 s 后打」）。该提交单独导出构建：
  0 警告、13/13，带 PSDK 头语法检查（开/不开 TRT）与 x86_64 链接均通过。
- **绕点飞行** `16dcb72` [V]：`wt_ped_square`（规划）+ `WtBridge_ExportPedSquareKmz`
  + `wt_mission_watch`（进度判定）+ `app/wt_square_mission.c`（执行）+ Pilot 2 5 号按钮。
- **README** `cef5b93` [V]：两节说明 + 控件表 + 更正一条过时告警（待命循环 Poll）。
- **核对网页** [V]：`wt_inspection/build/ped_square_view/ped_square_view.html`
  （build 目录不进 git，可用 `tools/ped_square_view/build_view.py` 重新生成）。
  两个场景 Edge 无头 `--dump-dom` 读出「全部 10 条符合」。
- **变异测试 30/30 抓到** [V]（覆盖规划、KMZ 文本、进度判定、确认帧、激光取中位、配置）。
- **全程没碰设备** [V]：用户此前答「暂不上机」。设备上的装包/配置状态沿用上一份快照 [?]。
- **工作区** [V]：干净（只有 `WEEKLY-2026-10-20.md`，用户自己的文件，未跟踪，**不要动**）；
  已推送，与 `fork/feature/wt-inspection` 同步。

### 测试输出（本次交接，全新 shell 跑）

```
$ env -i ... bash -c 'cmake -S . -B build && cmake --build build -j4 && ctest --test-dir build --output-on-failure'
exit=0，warning 数 0
100% tests passed, 0 tests failed out of 15
（含新增 wt_test_ped_square 285 项、wt_test_mission_watch 20 项、wt_test_ped_logic 430 项）
```

## 3. 决策与理由

- **锁定判据改为「停稳后连续 3 帧在容差内」** [V]——只要求打一次、拿位置时固定等 3 s
  不提供证据。按用户「现在先弄行人的」理解为同意上次方案一（**未明确确认，见 §7**）。
- **激光连读 3 次取距离居中那次，不平均经纬度** [V]——平均出的点可能哪次都没打到；
  离散度 > max(0.5 m, 1%) 判光斑压在人边缘、重打。
- **绕点飞行的 KMZ 照搬 liangzhourenwu 真机飞通的结构，不沿用风机 KMZ 的写法** [V]——
  两份文件分开写、补 `globalRTHHeight`、机型 99/1 相机 89（= `dji_typedef.h` 的 M4T 枚举）。
  风机 KMZ 从没 START 成功过（模拟器 error 4），且两份文件同一正文、缺必需元素。
  否决：照抄风机写法（无任何 START 证据）。风机导出未改，ZIP 抽公共函数后 sha256 不变。
- **机头 `towardPOI`（每点）+ 航点到点停** [V]——规范：towardPOI「飞向下一航点途中机头
  都将朝向兴趣点」；`smoothTransition` 线性插值角度，沿直线边会偏离基点约 4°。
  到点停才能让升降真的垂直。
- **`waypointHeadingPathMode = followBadArc`** [?]——每条边机头都顺时针转 90°，最短路径
  即顺时针；强制 clockwise 可能让首航点原地转近一圈。姊妹项目用 clockwise 飞通过。
- **默认值（待用户确认）**：边沿正东西南北；首顶点 = 离飞机最近的；3 m/s（转场 5）；
  结束 `noAction` 悬停；中止 = STOP 原地悬停（不返航：贴着人，返航要先爬高）；不控云台。
- **起飞闸门** [V]：只认本进程打到的点（不读旧 CSV）、必须在空中、云台不在 FREE、
  基点 ≤ 150 m、按键 5 s 内有效、状态回调注册失败不 START、「干跑」只上传。
- **航点模块用时 Init、用完 DeInit** [V]——与风机执行器同一做法，两边不互相冲掉回调。

## 4. 失败的尝试 — 不要再试

### 本轮新增

- **按 COCO 姿态 / 躯干方案做人员识别** → 用户当场否决 [V]——要的是高空俯视行人检测。
  见 memory `wt-person-aim-is-aerial-pedestrian`。
- **ultralytics 导出时让它自装依赖** → 自动起了 `pip install onnxruntime-gpu`，被及时杀掉、
  环境未变 [V]。脚本里设 `YOLO_AUTOINSTALL=false`。
- **行人模型用最近邻 letterbox** → 1440×1080 人群图检出 39 vs 双线性 45 [V]。用双线性。
- **批量变异用 `shutil.copy2` 恢复文件** → 旧 mtime 让 make 不重编，上一个变异的 .o
  残留进后续几轮 [V]；在源码目录跑测试 → 写出来路不明的 `data/` [V]。
  见全局 memory `verify-regression-catches`。
- **核对网页角度比较用 1e-6° 容差** → %.8f 经纬度（约 1 mm）就让「顺时针」误报不符 [V]。
  用 0.02°。
- **`git push`（本轮第一次）** → `Could not read from remote repository`，第二次卡住 >2 分钟 [V]；
  20 s 后 `ssh -T git@github.com` 认证成功、重推即通。瞬时网络问题，先测 SSH 再推。

### 继承（全部前向搬运，本轮未重验）

- 交叉编译（WSL `aarch64-linux-gnu-gcc`）→ 产物要 `GLIBC_2.34`，设备只有 2.31 [?]。
- 拿官方样例源码做对照（凭据未填）→ `Please fill in correct user information` [?]。
- 在 `/tmp` 下打包/编译 → 设备重启被清空 [?]。
- `tar --mtime='@0'` 解包后直接 make → 跳过重编、跑旧二进制；要先 `touch` [?]。
- `timeout 15 ./app` 杀卡住的进程 → 不生效，活 9 小时占通道；要 `kill -9` [?]。
- `dji_app_ctl stop` 后立刻 `start` → 通道未释放；等 10 秒 [?]。
- `strings` 查二进制里的中文串 → 只抓 ASCII；用 `md5sum` [?]。
- 模拟器里 `DjiWaypointV3_Action(START)` → `error_code: 4` [?]。
  ⚠️ 归因有新疑点：可能怪风机 KMZ 而非模拟器，见 memory `assistant2-simulator-limits-for-psdk`。
- `trtexec --verbose` 建 engine → 15 分钟没完；去掉后 13 分钟 [?]。
- SSH 前台跑长任务 → 断开后输出丢失；用 `nohup` + 重定向 [?]。
- `pip install torch` 走 PyPI 直连 → 0 B/s；阿里云镜像可用，清华镜像无此包 [?]。
- `jadx --single-class` 反编译 Pilot 2 → `Input class not found` [?]；
  `classes.dex` 静态分析 → AppGuard 加固 [?]。
- `DjiLiveview_SendAiMetaToPilot` 画框给操作员 → Pilot 2 不渲染、与 AI 识别互斥 [?]。
- 新建 `doc/` 下方案文档 → 违反「README 是唯一说明来源」[?]。
- M4T 热插拔 E-Port → USB 虚拟网卡不建立 [?]。
- 云台话题用 10Hz 订阅 → `0xE0`；只接受 50Hz（其余三种猜测都验过、都不是）[?]。
- 绝对角模式 `yaw = 0` → 整条命令被拒、连 pitch 都不转 [?]。
- 只在 `Stop`/`DeInit` 里打诊断 → `dji_app_ctl stop` 直接杀进程，永远看不到 [?]。
- `pgrep -f wt_inspection_a` → 匹配到 SSH 命令自身；用 `ps -eo pid,etime,comm` [?]。
- `dji_app_ctl install <dpk>` → 只打印帮助；要 `install -i <dpk>` [?]。
- 把配置追加到文件末尾 → 塞进了 `[turbine.*]` 段并造出重复段；改配置先看全文 [?]。

## 5. 已知坑

- **行人 / 绕点飞行尚未上机** [?]：engine 构建与 1280 推理耗时；变焦倍率语义与取流是否
  随变焦裁切；取流延迟是否在 300 ms 停稳窗口内；激光打在人身上的回波；Pilot 2 上
  4、5 号控件的显示；绕点 KMZ 能否 START；首航点机头对齐；`currentWaypointIndex`
  从 0 还是 1 数；行人脚下高于起飞点时实际离地 < 10 m（`HEIGHT_RELATIVE` 订不上，
  没法自动查地形高差）。
- **风机 KMZ 与真机飞通的结构不一致** [V]（读 `src/wt_bridge.c` 风机导出）——同一正文
  写进两个文件、机型 1100、缺必需元素；导入 Pilot 2 显示 0 m / 0 航点 [?]。暂缓，
  见 memory `wt-turbine-backlog` 第 3 条。
- **风机执行循环** [V]（读 `src/wt_runner_psdk.c`）——仍调必崩的
  `GetLatestValueOfTopic`，任务状态从不读回调。见 `wt-turbine-backlog` 第 4 条。
- **`app.json` 的 `bin` 指向 `build-native/`** [?]——打包前 `ln -sfn build build-native`，未统一。
- **设备 SSH 链路会重新枚举**（`eth2`/`eth3` 地址互换）[?]——连不上先看本机接口。
- **`blenderproc` 输出被 pip 进度条淹没、`/dev/shm` 清理报错** [?]。
- **Pilot 2 的 `cloud_config_RC701.json` 是 `root:root 600`** [?]。

## 6. 下一步（有序）

1. **等用户看核对网页**，按反馈改默认值（朝向、首顶点、速度、结束/中止行为、云台）。
   改完重跑 `wt_ped_square_demo` + `build_view.py` 重新生成网页。
2. **上机**（先用 AskUserQuestion 问 IP；`dji_app_ctl list` 查 184842 装的是谁；
   停 Smart3DExplore 并 pgrep 复查；`ps -eo pid,etime,comm` 查残留进程）：
   打包（`--mtime='@0'`，解包后 `touch`）→ 设备上 `trtexec` 建行人 engine（不加
   `--verbose`、不放 /tmp）→ `-DWT_BUILD_TRT=ON` 编译 → 改配置（先读全文；
   `ped_enabled = true`、`ped_engine`、`survey_enabled = false`）→ `install -i` 装包。
3. **地面先干跑**：「干跑」开着按「绕点飞行」，看日志有 `KMZ 已上传`；顺手在模拟器里
   重试一次 START，分清 error 4 是文件还是模拟器的问题。
4. **室外**：60 m 内的行人打点（看 `data/ped_points.csv`）→ 关干跑飞一次绕点；
   拉回 `data/ped_square.kmz` 用核对网页对照飞机实际收到的文件。

## 7. 留给用户的开放问题

- **行人提速是否按你的意思**：「现在先弄行人的」我理解为同意上次的方案一（连续 3 帧确认、
  激光 3 次取中位、默认不停顿）。要恢复 3 s 停顿只需配置 `ped_dwell_ms = 3000`。
- **绕点飞行的默认值**：见 §3，等你在网页上确认。
- **交接技能本轮被我改了两处**（`~/.claude/skills/handoff/`）：信号提取脚本漏掉粘贴消息、
  误收截图附注；第 4b 步的记忆目录公式对 worktree 和下划线都算错。三条被写错目录的
  记忆已复制到实际加载的目录。可自行复核或回退。
- 继承：`~/projects/.psdk-apiref/wt_inspection/` 去留；相机内参未标定；`park_phase`
  的 sign/offset 标定；多台风机；`/open_app/wt-inspection/data/` 里 liangzhourenwu 遗留产物。

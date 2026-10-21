# HANDOFF — 读全文再开始干活

生成时间: 2026-10-21T14:37:00+08:00 · Git HEAD: `c7a30d0`
信任规则: [V] = 交接时已用命令验证；[?] = 仅记忆未复核，当线索对待；[X] = 已证伪，别用。

## 0. 复核（下一会话先做）

- 锚点: `feature/wt-inspection` @ `c7a30d0` (2026-10-21 14:37)
- 漂移检查: `git rev-parse HEAD~1` 应 = `c7a30d0`——HEAD 必是本次 handoff 提交，
  其 parent 才是本快照记录的 SHA。
- 待重探的 `[?]`: 第 5 节 3 条
- 先读: `wt_inspection/README.md`（唯一说明来源）、`~/projects/CLAUDE.md`

## 1. 当前目标

**让航线能在真机（或 Pilot 2 手动导入）上验证通过。** 完成定义 =
KMZ 导入 Pilot 2 后机型/负载/航点数/航点类型/偏航模式全部正确显示，
且航线形态符合用户预期（trace 模式沿叶片飞）。

## 2. 已验证状态 — 工作实际停在哪

### 本次完成：控件可用性 + 三处安全/规范修复（代码层全部提交）

- **停用角改滑块** [V]：`int_input_box` 在 config_interface 里 Pilot 2 不渲染
  （日志 `config interface widget count = 1` 正常，界面看不到），改用 `scale`
  滑块放 main_interface，0–100% → 0–359°。设备实测可拖动、角度实时换算。
- **未知停用角一律拒绝** [V]：判据从 `!phaseKnown && autoStart` 改为
  `!phaseKnown`。原写法在控件启动路径下保护失效，日志实录
  `停用角取自配置文件（控件未设）：-1.0°` 紧跟着 `safety : PASS`。
- **拒绝后回待命不退出** [V]：实测「拒绝 → 重启 → 控件消失 → 无法修正」死循环。
  现在清空开始请求、回到待命，改完可再拨。待命循环会响应开始请求。
- **待命循环不调 `WtTelemetry_Poll`** [V]：该函数在待命阶段稳定 SEGV/ABRT
  （`code=dumped, status=11`），进入循环 3 秒内崩溃。
- **航点类型改为平滑过点** [V]：`toPointAndPassWithContinuityCurvature` +
  `useStraightLine=1`（官方规范要求两条都满足）。123/123 全部生效。
- **巡检段机头锁定正对盘面** [V]：`smoothTransition` + 每点同角，
  102 个巡检点全部 `30.00`（该值由 `rotAxis` 反算，非写死）。21 个叶尖
  掉头点保持 `towardPOI`。
- **机型/负载声明** [V]：`droneEnumValue 99 → 1100` + 新增 `payloadInfo`(10052)。
- 全新目录构建 + 测试：**cmake 0 / build 0（零警告）/ ctest 0，3/3 通过** [V]

### 设备状态

- 妙算3 当前 IP 走 **USB 网络共享**：`192.168.42.120`（USB 扩展坞路线是
  `192.168.2.175`，两条都通）[V]
- 已装版本含滑块与安全修复 [V]；**最新的 KMZ 航向/转弯改动尚未装到设备**
- 设备配置文件 `data/wt_config.ini` 当前：`no_actions = false`、
  `park_phase = 61.0` [V]

### 工作区

- 干净；`WEEKLY-2026-10-20.md` 是用户自己的文件，未跟踪，不要动 [V]

### 测试/build 输出（本次交接 run 的真实输出）

```
cmake -S . -B /tmp/hof        → 退出码 0
cmake --build /tmp/hof -j4    → 退出码 0，error 0 / warning 0
ctest --test-dir /tmp/hof     → 退出码 0
  100% tests passed, 0 tests failed out of 3
```

## 3. 决策与理由

- **用滑块不用输入框** [V]——`int_input_box` 放 main 会被 SDK 收下但不渲染
  （liangzhourenwu 2026-09-28），放 config 同样不渲染（本项目 2026-10-20 实测）。
  滑块是唯一验证过能显示的数值控件。代价约 3.6°/档，对现场目视得来的停用角够用。
- **偏航用 `smoothTransition` 而非 `towardPOI`** [V]——需求是"朝向整个风机平面"，
  `towardPOI` 朝一个点，飞机移动时机头方位会持续变化；`smoothTransition` +
  每点同角则方位恒定。
- **机头角度按每台风机算，不写死** [V]——用户明确要求「每一个角度都不一定一样，
  不要写定值」。取值用 `WtRotorFrame.rotAxis` 反算，而非 `spec.headingDeg`
  （后者没算进主轴仰角与锥角）。
- **机型改 1100 + 补 payloadInfo** [V]——Pilot 2 切 M4E 显示 0 航点、切 M4T 才正常，
  根因是缺负载声明，Pilot 只能按机型猜。

## 4. 失败的尝试 — 不要再试

- **交叉编译**（`aarch64-linux-gnu-gcc` 在 WSL）→ 产物要求 `GLIBC_2.34`，
  设备只有 2.31，`ldd` 报 not found [V]。不要再试。
- **拿官方样例源码做对照实验**（凭据未填）→ 日志是
  `main.c:437 Please fill in correct user information`，变量没控住 [V]。
- **在 `/tmp` 下打包/编译** → 设备重启后 `/tmp` 被清空 [V]。
- **Artifact 发布** → 本机 token 认证报 `Artifacts need a claude.ai login` [V]。
- **可视化按「段类型」分组画线** → 把不同叶片连成假航段（141.7 m）[V]。
- **`tar --mtime='@0'` 直接解包后 make** → make 跳过重编，跑旧二进制 [V]。
  解包后要 `touch` 源文件。
- **用 glob 清理共享设备的 `/tmp`** → 被权限分类器拦下 [V]。用精确文件名。
- **`timeout 15 ./app ...` 想杀掉卡住的进程** → **不生效**，进程活 9 小时
  并占住 PSDK 通道 [V]。要 `kill -9`。
- **用 `nohup` 手动启动应用** → 绕开 dpk 注册的 systemd 服务，服务报
  `Result: timeout` 后 SIGKILL 整个 cgroup，**连 `bulk_recv_task` 一起杀掉**，
  USB 端点随后损坏、后续所有启动卡在 `Identify device : manifold3` [V]。
  正确做法是 `/system/bin/dji_app_ctl start wt-inspection`。
- **`pgrep -x wt_inspection_app` 判断残留进程** → **永远匹配不到**：Linux 进程名
  上限 15 字符，实际是 `wt_inspection_a` [V]。判据改为查谁打开了
  `/dev/usb-ffs/bulk*`：
  `for p in /proc/[0-9]*; do ls -l $p/fd 2>/dev/null | grep -q usb-ffs && echo $p; done`
- **`pkill -f wt_inspection_app`** → 会匹配到执行它的那条 SSH 命令自身，
  连接被自己掐断 [V]。用 `kill -9 <pid>`。
- **`dji_app_ctl stop` 后立刻 `start`** → 通道未释放，报
  `Address already in use`，且会被误判成"内核残留需要重启飞机" [V]。
  正确做法：stop 后等 10 秒、确认占用进程数为 0，再 start。
- **用 `strings` 查二进制里有没有中文串** → 默认只抓 ASCII，中文 UTF-8 被跳过，
  误判成"没编进新代码" [V]。用 `md5sum` 对比或 `grep -aob`。
- **在模拟器里验证 `DjiWaypointV3_Action(START)`** → 飞控返回
  `Start waypoint v3 mission failed, error_code: 4`（= `WPMZ_FILE_LOAD_ERROR`），
  且 SDK 返回 SUCCESS 不报错，监控循环永远等不到任务开始 [V]。
  即使 Pilot 2 生成的标准航线也只飞几秒就暂停（10 个航点、悬停）。
  见 memory `assistant2-simulator-limits-for-psdk`。
- **引用 OSDK/MSDK 文档证明"模拟器支持航线"** → 三套 SDK 的模拟器支持是分开
  承诺的，PSDK 从未明确说明 [V]。用户当场指出这个错误。

## 5. 已知坑

- **安装器两条失败路径** [V]：① 应用提前退出 → 报误导性的
  `verify app user_app_id or version info error`；② 应用**长时间阻塞** →
  安装超时（exit 124）。
- **安装超时且日志 0 字节 = `DjiCore_Init` 之前就失败** [V]——最常见原因是
  PSDK 通道被占（`Smart3DExplore` **或自己留下的进程**）。
- **`dji_app_ctl stop` 报 SUCCESS 但进程仍在** [V]——要执行两次 + 复查。
- **`.dpk` 不解决 `/data` 写权限** [V]——落点是包内 `data/`，工作目录是包根。
- **设备 RTC 无电池** [V]——本次实测落后本机数天。
- **设备 cmake 3.16.3 不支持 `ctest --test-dir`** [V]——用 `cd build && ctest`。
- **trace 模式有固有高度约束** [V]——叶片朝下时叶尖接近地面，安全校验拒绝。
  实测 61° FAIL、45° PASS（最低航点 15.2 m，`ground_clearance` 15.0）。
- **叶片几何是零厚度轴线模型** [V]——无弦长/厚度，「中线」只能用轴线近似。
- **`template.kml` 与 `waylines.wpml` 内容完全相同** [V]——`wt_bridge.c` 把同一个
  buffer 写了两次。PSDK 上传不受影响（飞控只读 wpml），但导入 Pilot 2 时
  统计栏显示 0 m / 0 航点。**未修**。
- **模拟器只模拟飞控，不模拟云台和相机** [V]——日志
  `Get mounted position 1 camera's type failed, 0x000000E1`。
- **云台最大角速度无官方数值** [?]——原文库只给加速度公式。
- **`DjiCore_DeInit` 阶段报 `dji_sdk_state_push.c:175 0x100`** [?]——出现在
  收到 SIGTERM 之后，是否影响装包未单独验证。
- **`error_code: 4` 的含义** [?]——查到的云 API 错误码表说 4 =
  `WPMZ_FILE_LOAD_ERROR`，但那张表属于 Cloud API 体系，是否适用于
  PSDK Waypoint V3 未确证。

## 6. 下一步（有序）

1. **把最新的 KMZ 改动装上设备验证**：设备上的应用还不含航向/转弯/机型改动。
   按 `wt_inspection/CLAUDE.md` 的打包清单送上设备、**设备上编译**、
   `dji_app_ctl install`。
2. **导入 Pilot 2 验证显示**：用 `~/wt_export/WT-A01_trace_v2.kmz`（trace 版，
   123 航点，机头 30°）确认机型不再切 M4E 显示 0、航点类型显示「平滑过点，
   提前转弯」、偏航显示「自定义 30°」。
3. **修 `template.kml` / `waylines.wpml` 分离**（若第 2 步统计栏仍为 0）。
   需要让 template 只含航点定义、waylines 含执行参数，而不是复制同一份。
4. **真机验证**：只能去室外开阔场地。室内无法验证起飞。

## 7. 留给用户的开放问题

- **多台风机作业**：机头角度已按每台风机算，但设备上只配了 `WT-A01` 一台。
  多机作业需要配置里加多个 `[turbine.xxx]` 段。
- **trace vs 默认模式选哪个**：用户看过 `blade-trace.html` 后倾向 trace，
  但 trace 对停用角更敏感（61° FAIL、45° PASS）。现场停用角不确定时可能
  只能退回默认模式。
- **`/open_app/wt-inspection/data/` 里 liangzhourenwu 的遗留产物**——
  已备份到 `/home/dji/lz-salvage/`，原位置三份是否移走未定。
- **`park_phase` 的现场取值**——目前用的 45°/61° 都是测试值，实飞前必须
  现场实测。

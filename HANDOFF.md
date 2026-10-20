# HANDOFF — 读全文再开始干活

生成时间: 2026-10-20T10:40:00+08:00 · Git HEAD: `857bfca`
信任规则: [V] = 交接时已用命令验证；[?] = 仅记忆未复核，当线索对待；[X] = 已证伪，别用。

## 0. 复核（下一会话先做）

- 锚点: `feature/wt-inspection` @ `857bfca` (2026-10-20 10:40)
- 漂移检查: `git rev-parse HEAD~1` 应 = `857bfca`——HEAD 必是本次 handoff 提交，
  其 parent 才是本快照记录的 SHA。
- 待重探的 `[?]`: 第 5 节 2 条（云台角速度、DeInit 报错）
- 先读: `wt_inspection/README.md`（唯一说明来源）、`~/projects/CLAUDE.md`
  （**设备层三条硬约束本次仍全部有效**）

## 1. 当前目标

**让新版应用（含 `auto_start` 闸门 + `dry_run` + 本次的 `park_phase` 修复）
装上设备并验证。** 完成定义 = `dji_app_ctl install` 报 `APP INSTALL SUCCESS`。

## 2. 已验证状态 — 工作实际停在哪

### 本次完成：`park_phase` 断点已修（代码层）

- **停用角现在只从配置读取** [V]：`WtRunner_PlanTurbine` / `WtRunner_RunTurbine`
  去掉了 `parkPhaseDeg` 参数，运行器直接读 `job->turbine->parkPhaseDeg`。
  提交 `947c846`。原 `main.c:440` 的 `double parkPhase = -1.0;` 已删除。
- **新增回归用例** `TestParkPhaseReachesRunner` [V]：从**规划入口**断言
  `runner.job.parkPhaseDeg == 45.0`（配置里写 45.0）。已用变异测试确认它会红
  （把读取改回 `-1.0` → 3 条断言 FAIL，`parkPhaseDeg = 0.000000`）。
- 全新目录构建 + 测试：**cmake 0 / build 0（零警告）/ ctest 0，3/3 通过** [V]
- 文档同步 [V]：`README.md` 的「已知限制」删掉「尚未接通」，改写成取值来源与
  拒绝执行机制；配置模板注释补上「这是停用角的唯一来源」。提交 `857bfca`。

### 设备：**新版仍未装上**，旧版仍在（本次未动设备）

- 应用 `wt-inspection` `install_time = 2026-10-13 14:25:04`（旧版）[?]——本次未连设备复核
- 旧二进制 1351600 字节；新产物 1355792 字节 [?]——**但本地产物已丢失**，见下
- 三次安装尝试全部超时失败（exit 124）、日志 0 字节 [?]——继承自上份交接，本次未复核

### ⚠️ 交接前的准备工作已失效

- `/tmp/wtsrc/wt_inspection/build-native/bin/wt_inspection_app` **不存在** [V]
- `~/dpk/wt-inspection_v01.00.00.00.dpk` **不存在** [V]
- ⇒ 上一份 HANDOFF §6 第 1 项「手动跑这个二进制」**已经做不了**，需重新编译。

### 工作区

- 干净，分支领先 `fork/feature/wt-inspection` **4 个提交**（`ahead 4`）[V]

### 测试/build 输出（本次交接 run 的真实输出）

```
cmake -S . -B /tmp/pp_build2      → 退出码 0
cmake --build /tmp/pp_build2 -j4  → 退出码 0，error+warning 0 行
ctest --test-dir /tmp/pp_build2   → 退出码 0
  100% tests passed, 0 tests failed out of 3
```

## 3. 决策与理由

- **删掉 `parkPhaseDeg` 参数，而不是「在 main.c 里改成读配置」** [V]——只改
  调用点会留下两个来源（参数 + 配置），下一个调用者仍可传值把配置架空。
  用户要求「解决该问题」，根因是双来源本身。否决方案：保留参数、main.c 传
  `s_runner.config.turbines[i].parkPhaseDeg`——治标，且签名继续暗示「可以另传」。
- **回归用例从规划入口验证，而非只测 `WtAppConfig_Load`** [V]——配置解析一直
  是好的，断点在下游调用链。只测解析的用例永远绿，是假保险。见全局记忆
  `verify-regression-catches`。
- **`auto_start` 是作业总闸门** [V]——见上份交接，本次未改。

## 4. 失败的尝试 — 不要再试

- **交叉编译**（`aarch64-linux-gnu-gcc` 在 WSL）→ 产物要求 `GLIBC_2.34`，
  设备只有 2.31，`ldd` 报 not found [V]。不要再试。
- **拿官方样例源码做对照实验**（凭据未填）→ 日志是
  `main.c:437 Please fill in correct user information`，变量没控住 [V]。
- **在 `/tmp` 下打包/编译** → 设备重启后 `/tmp` 被清空 [V]；且本次证实本机
  `/tmp` 的构建产物与 `~/dpk` 的 dpk 都已丢失，上一份交接的前提因此失效 [V]。
- **Artifact 发布** → 本机 token 认证报 `Artifacts need a claude.ai login` [V]。
- **可视化按「段类型」分组画线** → 把不同叶片连成假航段（141.7 m）[V]。
  必须按「叶片 + 段类型」切分，已加回归用例。
- **`tar --mtime='@0'` 直接解包后 make** → make 跳过重编，跑旧二进制 [V]。
  解包后要 `touch` 源文件。见 `~/projects/CLAUDE.md`。
- **用 glob 清理共享设备的 `/tmp`** → 被权限分类器拦下 [V]。用精确文件名。
- **`timeout 15 ./app ...` 想杀掉卡住的进程** → **不生效**，进程活了 9 小时
  并占住 PSDK 通道，导致后续安装全部超时 [V]。要 `kill -9`。
- **在共享设备上手动跑应用做测试** → 留下的进程会污染后续装包 [V]。
  测试完必须 `pgrep` 复查并 `kill -9`。

## 5. 已知坑

- **安装器两条失败路径** [V]：① 应用提前退出 → 报误导性的
  `verify app user_app_id or version info error`；② 应用**长时间阻塞** →
  安装超时（exit 124）。两者都要避免。
- **安装超时且日志 0 字节 = `DjiCore_Init` 之前就失败** [V]——最常见原因是
  PSDK 通道被占（`Smart3DExplore` **或自己留下的进程**）。
- **`dji_app_ctl stop` 报 SUCCESS 但进程仍在** [V]——要执行两次 + `pgrep` 复查。
- **`.dpk` 不解决 `/data` 写权限** [V]——正确落点是包内 `data/`，
  应用工作目录就是包根。
- **设备 RTC 无电池** [V]——上次实测落后本机 6.38 天。
- **设备 cmake 3.16.3 不支持 `ctest --test-dir`** [V]——用 `cd build && ctest`。
- **trace 模式有固有高度约束** [V]——叶片朝下时叶尖接近地面，安全校验拒绝。
- **叶片几何是零厚度轴线模型** [V]——无弦长/厚度，「中线」只能用轴线近似。
- **~~`park_phase` 配置项未接通~~ → 已修复** [V]（提交 `947c846`）。
- **云台最大角速度无官方数值** [?]——原文库只给加速度公式。
- **`DjiCore_DeInit` 阶段报 `dji_sdk_state_push.c:175 0x100`** [?]——出现在
  收到 SIGTERM 之后，不在启动路径上，是否影响装包未单独验证。

## 6. 下一步（有序）

1. **先重建产物**（上份交接的二进制已丢）：按 `wt_inspection/CLAUDE.md` 的
   「设备侧打包清单」打包送上设备，在**设备上**编译（绝不交叉编译）。
2. **单变量验证**：确认无任何 `wt_inspection_app` 进程后，**手动**跑新二进制，
   观察是否按新语义停在待命状态（`auto_start` 默认 false）。用 `kill -9` 收尾，
   不要用 `timeout`。
3. **再试 `dji_app_ctl install`**。装成功后核对二进制大小、并确认
   `park_phase` 在设备日志里能按配置值打印（`[runner] 停用角 N°（配置 park_phase）`）。
4. **干跑验证**：配置 `dry_run = true` + `auto_start = true`，看 KMZ 能否被真飞控
   接受。**先测默认模式，再测 trace**。

## 7. 留给用户的开放问题

- **Gazebo/ROS 要不要做**——PSDK 无飞控仿真 API，PX4 结果不能迁移到 M4T；
  Gazebo 只对**视觉**那一步有价值（合成图像）。以周计的工程，待定。
- **`/open_app/wt-inspection/data/` 里 liangzhourenwu 的遗留产物**——
  已备份到 `/home/dji/lz-salvage/`，原位置三份是否移走未定。
- **liangzhourenwu 项目本身**——其应用被 10-19 的安装覆盖，`lz/doc/` 里有实飞
  记录，建议确认有无未入库状态。
- **`park_phase` 的现场取值**——本次只是打通了「配置→规划」这条链路，
  45° 是测试值。实飞前必须现场实测每台风机填入真实停用角。

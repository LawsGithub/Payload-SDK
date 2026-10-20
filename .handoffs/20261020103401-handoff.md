# HANDOFF — 读全文再开始干活

生成时间: 2026-10-20T10:23:00+08:00 · Git HEAD: 829f714
信任规则: [V] = 交接时已用命令验证；[?] = 仅记忆未复核，当线索对待；[X] = 已证伪，别用。

## 0. 复核（下一会话先做）

- 锚点: `feature/wt-inspection` @ `829f714` (2026-10-20 10:23)
- 漂移检查: `git rev-parse HEAD~1` 应 = `829f714`——HEAD 必是本次 handoff 提交，
  其 parent 才是本快照记录的 SHA。
- 待重探的 `[?]`: 见第 2、5 节标记
- 先读: `wt_inspection/README.md`、`~/projects/CLAUDE.md`
  （**设备层三条硬约束本次都有更新，务必读**）

## 1. 当前目标

**让新版应用（含 `auto_start` 作业闸门 + `dry_run`）装上设备并验证。**

完成定义 = `dji_app_ctl install` 报 `APP INSTALL SUCCESS`，且设备上二进制
大小为 **1355792** 字节（新）而非 1351600（旧）。

## 2. 已验证状态 — 工作实际停在哪

### 代码：已完成并提交，测试全绿

- `auto_start` 改为**作业总闸门** [V]：`false`（默认）→ 进程待命不作业；
  `true` → 执行作业。用户 2026-10-20 明确同意。
- `dry_run` 新增 [V]：只做「规划 → 生成 KMZ → 上传」，不调 `Action(START)`。
- 全新目录构建 + 测试：**cmake 0 / build 0（零警告）/ ctest 0，3/3 通过** [V]

### 设备：**新版未装上**，旧版仍在

- 应用列表：`wt-inspection` `install_time = 2026-10-13 14:25:04`（**旧版**）[V]
- 二进制：`/open_app/wt-inspection/bin/wt_inspection_app` = **1351600 字节**（旧）[V]
- 新版编译产物 = **1355792 字节**，已在 `/tmp/wtsrc/wt_inspection/build-native/bin/`
  与 `~/dpk/wt-inspection_v01.00.00.00.dpk` [V]
- **三次安装尝试全部超时失败**（exit 124），且 `/blackbox/system/app_temp_files/`
  里的日志是 **0 字节** [V]

### 已排除的直接原因

- **僵尸进程占通道** [V]：14:51 手动测试的 `wt_inspection_app`（PID 82923）
  跑了 9 小时占着 PSDK 通道。**已 `kill -9` 清理**。清理后重装**仍超时**，
  所以它不是唯一原因。
- 设备当前无任何 `wt_inspection_app` 进程 [V]；`Smart3DExplore` 未跑 [V]。

### 工作区

- 干净。分支与 `fork/feature/wt-inspection` 同步（工作切片已推送前先确认）。[V]

### 测试/build 输出（本次交接 run 的真实输出，全新目录）

```
cmake -S . -B /tmp/ho2_build     → 退出码 0
cmake --build /tmp/ho2_build -j4 → 退出码 0，error+warning 0 行
ctest --test-dir /tmp/ho2_build  → 退出码 0
  100% tests passed, 0 tests failed out of 3
```

## 3. 决策与理由

- **`auto_start` 成为作业总闸门** [V]——原名只控制「出错后是否继续」，名实
  不符，导致配置已存在时进程直接进作业流程、阻塞、安装器超时。用户确认修改。
  否决方案：保留旧语义另加开关 —— 会让两个名字都误导。
- **`dry_run` 而非「单独做一个上传测试程序」** [V]——干跑走的是**同一条**
  生产代码路径（规划→KMZ→上传），只是不启动任务；独立测试程序会引入
  第二套实现，验证的东西与真实链路可能不一致。
- **ROS/Gazebo 不是当前最优解** [V]——PSDK **不暴露飞控仿真器**（`simulat`
  只出现在错误码与 HMS 告警里，40+ 个 `DjiFlightController_*` 函数无启动
  仿真的 API）。PX4 SITL 的飞控实现与 DJI 不同，跑通**不能推出** M4T 能飞。
  Gazebo 唯一不可替代的价值是**合成图像**（服务视觉那一步，不是航线验证）。

## 4. 失败的尝试 — 不要再试

- **交叉编译**（`aarch64-linux-gnu-gcc` 在 WSL）→ 产物要求 `GLIBC_2.34`，
  设备只有 2.31，`ldd` 报 not found [V]。不要再试。
- **拿官方样例源码做对照实验**（凭据未填）→ 日志是
  `main.c:437 Please fill in correct user information`，变量没控住 [V]。
- **在 `/tmp` 下打包/编译** → 设备重启后 `/tmp` 被清空 [V]。
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
- **设备 RTC 无电池** [V]——本次实测落后本机 **6.38 天**。
- **设备 cmake 3.16.3 不支持 `ctest --test-dir`** [V]——用 `cd build && ctest`。
- **trace 模式有固有高度约束** [V]——叶片朝下时叶尖接近地面，安全校验拒绝。
- **叶片几何是零厚度轴线模型** [V]——无弦长/厚度，「中线」只能用轴线近似。
- **`park_phase` 配置项未接通** [V]——`main.c:408` 硬编码 `parkPhase = -1.0`，
  即使配置里填了也读不到。**这是 trace 实飞前必须修的断点**。
- **云台最大角速度无官方数值** [?]——原文库只给加速度公式。
- **`DjiCore_DeInit` 阶段报 `dji_sdk_state_push.c:175 0x100`** [?]——出现在
  收到 SIGTERM 之后，不在启动路径上，是否影响装包未单独验证。

## 6. 下一步（有序）

1. **先做单变量验证，别再盲目重装**：确认无任何 `wt_inspection_app` 进程后，
   **手动**跑 `/tmp/wtsrc/.../build-native/bin/wt_inspection_app`，观察它是否
   按新语义停在待命状态。用 `kill -9` 收尾，不要用 `timeout`。
2. **若手动跑正常**，再试 `dji_app_ctl install`。装成功后核对二进制大小
   是否为 **1355792**。
3. **若手动跑仍卡住**，用 `gdb` 或加日志定位卡在哪个 PSDK 调用；重点看
   `WtTelemetry_Init`（话题订阅）与 `WtRunner_Init`。
4. **修 `park_phase` 断点**：`main.c` 改成读 `config.turbines[i].parkPhaseDeg`，
   负值才当未知。这是 trace 实飞的前提。
5. **干跑验证**（新版本装上后）：配置 `dry_run = true` + `auto_start = true`，
   看 KMZ 能否被真飞控接受。**先测默认模式，再测 trace**。

## 7. 留给用户的开放问题

- **Gazebo/ROS 要不要做**——用户说三个层次都需要，但真正约束是「没有现场」。
  已说明：PSDK 无飞控仿真 API，PX4 结果不能迁移到 M4T；Gazebo 只对**视觉**
  那一步有价值（合成图像）。是否投入以周计的工程，待定。
- **`/open_app/wt-inspection/data/` 里 liangzhourenwu 的遗留产物**——
  已备份到 `/home/dji/lz-salvage/`，原位置三份是否移走未定。
- **liangzhourenwu 项目本身**——其应用被 10-19 的安装覆盖，`lz/doc/` 里
  有实飞记录，建议确认有无未入库状态。

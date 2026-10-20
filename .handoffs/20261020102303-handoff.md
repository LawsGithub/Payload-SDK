# HANDOFF — 读全文再开始干活

生成时间: 2026-10-19T23:49:00+08:00 · Git HEAD: 98a4810
信任规则: [V] = 交接时已用命令验证；[?] = 仅记忆未复核，当线索对待；[X] = 已证伪，别用。

## 0. 复核（下一会话先做）

- 锚点: `feature/wt-inspection` @ `98a4810` (2026-10-19 23:49)
- 漂移检查: `git rev-parse HEAD~1` 应 = `98a4810`——HEAD 必是本次 handoff 提交，
  其 parent 才是本快照记录的 SHA。变了说明快照可能过期。
- 待重探的 `[?]`: 见第 2、5 节标记
- 先读: `wt_inspection/README.md`（唯一说明来源）、
  `~/projects/CLAUDE.md`（设备硬约束，**本次新增 3 条，务必读**）

## 1. 当前目标

**主线已完成**：`wt_inspection_app` 以 `.dpk` 装到妙算3，`APP INSTALL SUCCESS` [V]。

**新目标（本会话确立）**：把「沿叶片中线飞行」模式（trace）做成可实飞的功能。
C 实现与设备编译已完成，**尚未实飞**。

## 2. 已验证状态 — 工作实际停在哪

### 装包成功（主线完成定义达成）

- `dji_app_ctl install` → **`APP INSTALL SUCCESS`**，退出码 0 [V]
- 应用列表：`Smart3DExplore` + `wt-inspection`（total 2）[V]
- 装机目录 `/open_app/wt-inspection/`，`data/` 属主 `dji:dji` **可写** [V]
  （`touch data/.wtest` 成功）
- 安装期日志含 `dji_identity_verify.c:654 Update dji sdk policy file successfully` [V]
  —— CLAUDE.md 记的身份校验判据
- **但配置仍是模板**（`takeoff_lat = 41.5236000` 等样例值，`trace_enable = false`）[V]
  → **应用装上了，还不能真作业**。带样例坐标起飞比不飞更糟（代码注释明写）。

### trace 模式（沿叶片中线飞行）

- `WtPlan_BladeTrace()` 已实现，设备上编译通过、零警告（项目代码）[V]
- 设备实测与 PC 完全一致：123 航点 / 600.3 m / 96 张 / 5.9 min，
  `safety: PASS`，min dist to blade axis **9.20 m**（限值 3.00）[V]
- KMZ 产物：`towardPOI` 21 处、`gimbalYawRotateEnable=0` 21 处、
  `takePhoto` 96、`Placemark` 123 [V]
- **POI 精确性已验证**：KMZ 里 3 个 `waypointPoiPoint` 与规划器算出的
  三片叶尖坐标**逐位一致** [V]
- 回归测试 5 条新用例，共 163 项检查全过 [V]

### 启动路径修复

- `DjiCore_ApplicationStart` 之后**再无 `return 1`** [V]（`grep -n 'return 1' app/main.c`）
- 真机实测（配置缺失 + 飞机连接）：进程存活 8 秒以上、返回码 **0**、
  模板落盘到 `data/wt_config.ini` [V]

### 工作区

- `git status`：干净。分支领先 `fork/feature/wt-inspection` **10 个提交**（未推送）[V]

### 测试/build 输出（本次交接 run 的真实输出，全新目录）

```
cmake -S . -B /tmp/handoff_build     → 退出码 0
cmake --build /tmp/handoff_build -j4 → 退出码 0，error+warning 0 行
ctest --test-dir /tmp/handoff_build  → 退出码 0
  1/3 wt_test_plan   Passed
  2/3 wt_test_config Passed
  3/3 wt_test_bridge Passed
  100% tests passed, 0 tests failed out of 3
```

## 3. 决策与理由

- **trace 与站位模式互斥、不合并** [V]——两者成像几何完全不同（正对叶面 vs
  斜视），`WtTraceProfile` 独立于 `WtInspectionProfile`，避免两套采样推导纠缠。
- **掉头段用 `towardPOI`，云台只控俯仰** [V]——M4T 云台 Pan 轴
  `Not controllable`，云台 yaw 无法独立于机头偏转。让机头承担偏转是唯一可行做法。
  否决方案：`gimbalRotate` 的绝对 yaw，M4T 上做不到。
- **trace 模式 GSD 校验显式跳过** [V]——该模式 GSD 随叶弦变化而非斜距，
  用斜距公式算出来是错的。置 0 表示「未计算」，若照常走判据会**静默通过**
  （`0 > 目标GSD` 恒为假）。
- **启动路径 fail-closed 边界从「进程启动」移到「作业开始」** [V]——安装器要
  试运行并要求走完身份校验，进程提前退出 = 装不上包。
- **设备上 app_id 冲突按「同一时间只装一个」处理** [V]（用户 2026-10-19 决定）——
  见 `~/projects/CLAUDE.md` 的「app_id 冲突」一节。

## 4. 失败的尝试 — 不要再试

- **交叉编译**（`aarch64-linux-gnu-gcc` 在 WSL）→ 产物要求 `GLIBC_2.34`，
  设备只有 2.31，`ldd` 报 not found [V]。不要再试。
- **拿官方样例源码做对照实验**（凭据未填）→ 日志是
  `main.c:437 Please fill in correct user information`，**变量没控住**：
  样例源码里是占位符 `your_app_id`，其 `app.json` 是 `164884`。不能当对照组 [V]。
- **在 `/tmp` 下打包/编译** → 设备重启后 `/tmp` 被清空 [V]。重要产物别只放 `/tmp`。
- **Artifact 发布** → 本机走 `ANTHROPIC_AUTH_TOKEN` 认证，报
  `Artifacts need a claude.ai login` [V]。讲解页就用本地 `file://`，别再提发布。
- **可视化按「段类型」分组画线** → 把不同叶片的点连成假航段
  （1 号叶尖 → 2 号叶尖，**141.7 m 直线**）[V]。必须按「叶片 + 段类型」切分。
  已加回归用例钉死（相邻间距 ≤ 21 m）。
- **`tar --mtime='@0'` 直接解包后 make** → **make 跳过重编**，跑的还是旧二进制 [V]。
  源文件时间戳变 1970，比设备上的 `.o` 旧。解法：解包后 `touch` 源文件。
  详见 `~/projects/CLAUDE.md`。
- **用 `rm -f /tmp/chk*.sh` 这类 glob 清理共享设备** → 被权限分类器拦下 [V]
  （理由正当：CLAUDE.md 自己警告过 `/tmp` 会撞名）。清理要写**精确文件名**。

## 5. 已知坑

- **安装器报错会把死因指向错误方向** [V]——应用启动即退出时报的是
  `verify app user_app_id or version info error`，而凭据和版本都没问题。
- **`dji_app_ctl stop` 报 SUCCESS 但进程仍在** [V]——**要执行两次**，
  且必须 `pgrep` 复查。见 `~/projects/CLAUDE.md`。
- **`.dpk` 不解决 `/data` 写权限** [V]——应用以 `uid=1000` 运行，正确落点是
  包内 `data/`（**应用的工作目录就是包根**）。
- **设备 RTC 无电池，掉电回 1970** [V]——本次实测落后本机 **6.38 天**。
- **设备 cmake 3.16.3 不支持 `ctest --test-dir`** [V]——报
  `No tests were found!!!`（该选项 CMake 3.20 才加入）。要用 `cd build && ctest`。
- **`app.json` 的 `bin` 是相对 `app.json` 的路径** [V]——目录名必须是 `build`。
- **`app.json` 的 `user_app_id` 与 `wt_credentials.ini` 是同一信息的两副本** [V]——
  换 app 必须同时改两处。
- **trace 模式有固有高度约束** [V]——叶片朝下时（相位 180°）叶尖接近地面，
  最低航点 U=12.3 m < 默认余量 15 m，安全校验**拒绝**。这是物理约束不是 bug，
  需调整停用角或等风向。
- **叶片几何是零厚度轴线模型** [V]——无弦长/厚度剖面（`grep chord|thickness`
  零命中），故「叶片中线」目前只能用轴线近似（真实中线叶根处离轴线约 2 m）。
- **快回路的断点在「无人调用」而非「没有能力」** [X] 已更正——
  `DjiLiveview_StartImageStream`（仅 Manifold 3）、
  `DjiCameraManager_DownloadFileByIndex` 都存在 [V]，项目没用而已。
- **云台最大角速度无官方数值** [?]——原文库只给加速度公式
  `10000 × 0.8^(1+X) deg/s²`。「~100-200 °/s」是消费级典型值，别当官方规格引。
- **`DjiCore_DeInit` 阶段报 `dji_sdk_state_push.c:175 init error 0x100`** [?]——
  出现在进程收到 SIGTERM 之后，不在启动路径上，身份校验此前已成功。
  是否影响装包未单独验证（本次装包成功，故推测无影响）。

## 6. 下一步（有序）

1. **填现场台账**：编辑设备 `/open_app/wt-inspection/data/wt_config.ini`，
   把样例坐标换成真实风机参数。**这是能否实飞的前提**。
2. **决定 trace 是否启用**：同一文件里 `trace_enable = true` 走沿中线飞行，
   `false` 走原盘外站位。启用时 `trace_offset`（默认 10 m）必须 > `min_safe_dist`。
3. **选停用角**：trace 模式在叶片朝下时会被地面余量拒绝，需现场确认叶片姿态。
4. **清理设备残留**（共享设备，需用户确认可删项）：
   `wt-iso-g`、`wt-widget-demo` 等更早会话的痕迹、`/tmp/wtsrc`、`/tmp/wtbuild.tar.gz`。
   **`/home/dji/lz-salvage/` 不要删**（抢救出的 liangzhourenwu 产物）。
5. **（未做）根治 app_id 冲突**：申请第二个 app，同步改
   `wt_credentials.ini` 与 `app.json`。

## 7. 留给用户的开放问题

- **`/open_app/wt-inspection/data/` 里 liangzhourenwu 的产物怎么处理**——
  已备份到 `/home/dji/lz-salvage/`，但原位置那三份（`ground.txt` / `pole.txt` /
  `orbit.kmz`）是否要移走或删除，未定。
- **liangzhourenwu 项目本身的状态**——它的应用被本次安装覆盖了。
  其 `lz/doc/` 里有实飞验证记录，建议去看看是否有未入库的状态。
- **视觉→控制那一段的取舍**——本会话讨论了「沿中线跟拍」的两条路
  （无人机位置跟中线 vs 云台跟中线），结论是走云台 + 直接视觉伺服。
  但取图通路（`DjiLiveview_StartImageStream`）**尚未实测**，M4T 上是否可用
  未验证。
- **输出目录是否做成可配置项**——按 `wt-inspection-config-strictness` 的思路，
  能由部署形态决定的不该让现场填。

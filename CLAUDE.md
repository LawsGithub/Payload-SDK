# CLAUDE.md — 风机叶片巡检（wt_inspection）

本文件是**项目专属**约束。仓库级规则（设备、硬规则、worktree 布局）见上一级
`CLAUDE.md`（提交在 `master` 上，每个分支都继承）——**先读那一份**。

工作区：`~/projects/wt_inspection/`（分支 `feature/wt-inspection`），
代码在子目录 `wt_inspection/`。

## 跨会话交接

- 会话开始时先读 [HANDOFF.md](HANDOFF.md) 全文，复述上一会话的目标、当前状态、
  下一步，然后才动手。
- 信任标记：`[V]` = 交接时已用命令验证；`[?]` = 仅记忆未复核，当线索对待；
  `[X]` = 已证伪，别用。
- 漂移检查：`git rev-parse HEAD~1` 应等于 HANDOFF.md 记录的 SHA——HEAD 应是
  本次 handoff 提交，其 parent 才是快照记录的锚点。

## 项目文档的唯一来源

`wt_inspection/README.md` 是**唯一**的说明来源（架构分层、参数、快速开始、
打包流程）。不要再新建 `docs/` 下的方案文档——历史上 README 与方案文档对
同一套参数有两处描述，已出现实质矛盾，收敛掉了。

## PC 侧自检（秒级，不依赖 PSDK）

```bash
cd wt_inspection
cmake -S . -B build && cmake --build build -j4
ctest --test-dir build --output-on-failure
```

`CMakeLists.txt` 刻意划了一条界：`wt_core` 纯算法库完全不依赖 PSDK，规划/几何/
安全校验这些"出错代价最大"的部分必须能在桌面上反复验证；只有真正碰硬件的
代码才进 PSDK 目标（`src/wt_*_psdk.c`）。

## 机载应用凭据（构建期注入）

`wt_inspection/wt_credentials.ini`（**未跟踪**，权限 600）由
`cmake/gen_app_info.cmake` 在构建期生成 `dji_sdk_app_info.h` 到构建目录，
并遮蔽官方样例里的同名头文件。

**换 app 时必须同时改两处**：`wt_credentials.ini` 与 `app_json/app.json` 的
`user_app_id`（后者写真实值是"凭据只有一处真值"原则的唯一例外）。

新 worktree 里这个文件**不会自动出现**（`git worktree add` 不复制未跟踪文件），
要手工从别的 worktree 拷。

## 打包与安装

```bash
# 设备上编译完成后
tools/build_dpk/build_dpk.sh <路径>
dji_app_ctl install <包名>.dpk
```

- `app.json` 的 `bin` 是**相对 `app.json` 的路径**，且目录名必须是 `build`
  （写成 `build-native` 时打包直接报 `bin field ... not exist`）。
- 装包**需要飞机通电并连接**，且启动路径不能提前退出
  （见仓库级 CLAUDE.md 的「安装器会试运行应用」）。

## 设备侧打包清单

本机打包送上设备的必需部分（约 1MB）：

```bash
tar czf /tmp/wtbuild.tar.gz --mtime='@0' \
     wt_inspection tools/build_dpk psdk_lib/include \
     psdk_lib/lib/aarch64-linux-gnu-gcc \
     samples/sample_c/platform/linux/{manifold3,common}
```

`--mtime='@0'` 与「CMake 硬编码找 `psdk_lib/lib/aarch64-linux-gnu-gcc/libpayloadsdk.a`、
别压平目录层级」两条，见仓库级 CLAUDE.md。

## 已证伪的路 —— 不要再试

- **交叉编译**（`aarch64-linux-gnu-gcc` 在 WSL）→ 产物要求 `GLIBC_2.34`，
  设备 `ldd` 报 not found [V]。
- **拿官方样例源码做对照实验**（凭据未填）→ 日志是
  `main.c:437 Please fill in correct user information`，**变量没控住**：
  官方样例源码里是占位符 `your_app_id`，而它的 `app.json` 是 `164884`。
  这个包不能当对照组 [V]。
- **在 `/tmp` 下打包/编译** → 设备重启后 `/tmp` 被清空 [V]。重要产物别只放 `/tmp`。
- **静态链接 / 改 sysroot 绕 glibc** → 未尝试 [X]。已选设备本地编译这条路。

## 已知坑

- **安装器报错会把死因指向错误方向** [V]——应用启动即退出时，报的是
  `verify app user_app_id or version info error`，而凭据和版本都没问题。
- **`dji_app_ctl` 不需要 sudo** [V]——`stop` / `install` 在 dji 用户下直接可用。
- **`.dpk` 不解决 `/data` 写权限** [V]——应用仍以 `uid=1000` 运行。
- **设备 RTC 无电池，掉电回 1970** [V]——进一次 Pilot 2 飞行界面会经飞机校时。

## 待办（接 HANDOFF.md 第 6 节）

1. 改 `wt_inspection/app/main.c` 的启动路径：配置缺失或输出目录不可写时
   **不要退出**，要让 SDK 起来走完校验。fail-closed 的边界划在「作业开始」，
   不是「进程启动」。同时把 `output_dir` 默认值从 `/data/wt_inspection`
   改为相对应用目录的 `data/`（`WT_DEFAULT_CONFIG_PATH` 同理，`main.c:33`）。
2. 设备上重新编译 + 打包 + 安装验证：预期 `APP INSTALL SUCCESS`。
3. 清理设备上的实验残留：`/open_app/wt-iso-g`（`dji_app_ctl uninstall wt-iso-g`）、
   `/tmp/iso*`、`/tmp/wts2`、`/tmp/probe*`、`~/dpk/`。
4. 用 `wt_plan_demo` 复现 README 里引用的数字（README 已改为指向该程序，
   但**没有复核过具体数值是否与程序输出一致**）。
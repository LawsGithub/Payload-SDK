# HANDOFF — 读全文再开始干活

生成时间: 2026-09-17T19:48:57+08:00 · Git HEAD: c598897
信任规则: [V] = 交接时已用命令验证；[?] = 仅记忆未复核，当线索对待；[X] = 已证伪，别用。

## 0. 复核（下一会话先做）

- 锚点: `feature/wt-inspection` @ `c598897` (2026-09-17 19:48)
- 漂移检查: `git rev-parse HEAD~1` 应 = `c598897`——HEAD 必是本次 handoff 提交，
  其 parent 才是本快照记录的 SHA。变了说明快照可能过期。
- 待重探的 `[?]`: 见第 4、5 节标记
- 先读: `wt_inspection/README.md`（唯一说明来源）、`CLAUDE.md`（设备约束）
- 设备: `sshpass -p 'dji' ssh dji@192.168.42.120`（本次可达，USB 共享经本机 eth4）

## 1. 当前目标

让 `wt_inspection_app` 能以 `.dpk` 装到妙算3 上并正常运行。

**根因已定位**，完成定义 = 设备上 `dji_app_ctl install` 报
`APP INSTALL SUCCESS`，且应用启动后能进入作业流程（而非启动即退出）。

## 2. 已验证状态 — 工作实际停在哪

- `wt-widget-demo`（官方 widget 样例，用我们的凭据）**安装成功并已在妙算上显示** [V]
  `dji_app_ctl list` → `name_en: wt-widget-demo, install_time: 2026-09-17 18:35:42`
- **根因**：安装器会试运行应用并要求走完 SDK 身份校验；我们的应用在
  `main.c:334`（配置缺失 + `/data` 不可写）退出，校验未完成 → 装包失败 [V]
  证据：隔离实验 C（我们的二进制）失败 / G（同一二进制，仅改 `output_dir`
  到可写路径）**成功**；G 的应用日志走到相机表、风机参数、KMZ 下发
- 应用跑在 **`uid=1000`（dji），不是 root** [V] 探针脚本 `id -u` → `1000`；
  `mkdir /data/wt_inspection` → `Permission denied`
- `/data` 是 `root:root 755`；应用目录 `/open_app/<name>/data/logs` 是
  `dji:dji`，**可写** [V] `touch` 成功
- 官方约定是**相对应用目录**：`build_dpk.sh` 在包内建 `data/logs/` `data/media/`，
  官方样例写 `"data/logs/DJI"`（相对路径）[V]
- `184842` 这对凭据**完全正常**（此前所有失败与凭据无关）[V]
  同一凭据被 `wt-widget-demo`、`wt-iso-a/b/g` 装成功过
- 交叉编译产物要求 `GLIBC_2.34`，设备只有 2.31 [V] `readelf -V` →
  `GLIBC_2.17 / 2.33 / 2.34`
- 设备本机编译产物最高只要求 `GLIBC_2.17` [V] 同一条命令
- 设备时间已由飞机校时同步到与本机差 1 秒 [V]（此前曾慢 23 小时）
- 工作区: 干净，除 `wt_inspection/app_json/`（已提交）与 `.vscode/`（已忽略）

### 测试/build 输出（本次交接 run 的真实输出）

```
$ cd wt_inspection && ctest --test-dir build --output-on-failure
1/3 Test #1: wt_test_plan .....................   Passed    0.01 sec
2/3 Test #2: wt_test_config ...................   Passed    0.01 sec
3/3 Test #3: wt_test_bridge ...................   Passed    0.01 sec

100% tests passed, 0 tests failed out of 3
Total Test time (real) =   0.03 sec
退出码 0
```

## 3. 决策与理由

- **输出目录改为应用目录下的 `data/`** [V]——官方约定、包内已建好、可写、
  卸载随包走。否决：`/data/wt_inspection/`（需 root，dpk 也不给），
  `/tmp`（重启即失）。**尚未落到代码**，见第 6 节。
- **打包放哪都行，编译必须贴设备** [V]——`build_dpk.sh` 只依赖 python3 +
  dpkg-deb，不碰编译器，与目标架构无关；glibc 约束只作用于编译。
- **文档收敛为 README 一份** [V]——原来 `docs/风机叶片巡检方案.md` 与 README
  对同一套参数有两处描述，已出现实质矛盾（README 教交叉编译、CLAUDE.md 禁）。
- **`app.json` 的 `user_app_id` 写真实值**（用户明确决定）——这是「凭据只有
  一处真值」原则在仓库里的唯一例外，换 app 时必须同时改 `wt_credentials.ini`。

## 4. 失败的尝试 — 不要再试

- 交叉编译（`aarch64-linux-gnu-gcc` 在 WSL）→ 产物要求 `GLIBC_2.34`，
  设备 `ldd` 报 not found [V]——本机 glibc 2.35 > 设备 2.31。不要再试。
- 用官方样例源码（凭据未填）做对照 → 装包失败，但日志是
  `main.c:437 Please fill in correct user information` [V]——**变量没控住**，
  官方样例源码里是占位符 `your_app_id`，而它的 `app.json` 是 `164884`。
  这个包不能当对照组，别再拿它试。
- 用 `-DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc` 之外的方式绕 glibc
  （静态链接、改 sysroot）→ 未尝试 [X]——已选设备本地编译这条路，无需再走。
- 在 `/tmp` 下打包/编译 → 设备重启后 `/tmp` 被清空 [V]——本次实测踩过，
  重要产物别只放 `/tmp`。

## 5. 已知坑

- **安装器报错会把死因指向错误方向** [V]——应用启动即退出时，报的是
  `verify app user_app_id or version info error`，而凭据和版本都没问题。
  判据看应用日志 `/blackbox/system/app_temp_files/<name>_*.log` 里有没有
  `dji_identity_verify.c:654 Update dji sdk policy file successfully`。
- **`dji_app_ctl` 不需要 sudo** [V]——`stop` / `install` 在 dji 用户下直接
  可用。CLAUDE.md 里"需要 root"那条已修正。
- **`.dpk` 不解决 `/data` 写权限** [V]——应用仍以 uid=1000 运行。CLAUDE.md
  里"由系统授权因而能写 /data"已修正。
- **设备 RTC 无电池，掉电回 1970** [V]——进一次 Pilot 2 飞行界面会经飞机
  校时。`tar` 用 `--mtime='@0'` 压时间戳，别去改设备时间。
- **`app.json` 的 `bin` 是相对 `app.json` 的路径** [V]——官方样例写成
  `../../../../../../build/bin/...` 这种深相对路径，且目录名必须是 `build`
  （用 `build-native` 时打包直接报 `bin field ... not exist`）。

## 6. 下一步（有序）

1. **改 `wt_inspection/app/main.c` 的启动路径**：配置缺失或输出目录不可写时
   不要退出，要让 SDK 起来走完校验。fail-closed 的边界应划在「作业开始」，
   不是「进程启动」。同时把 `output_dir` 默认值从 `/data/wt_inspection`
   改为相对应用目录的 `data/`（`WT_DEFAULT_CONFIG_PATH` 同理，
   `app/main.c:33`）。
2. **设备上重新编译 + 打包 + 安装验证**：预期 `APP INSTALL SUCCESS`。
   `Smart3DExplore` 运行中时先 `pgrep -x Smart3DExplore >/dev/null || dji_app_ctl stop Smart3DExplore`。
3. 清理设备上的实验残留：`/open_app/wt-iso-g`（卸载：`dji_app_ctl uninstall wt-iso-g`）、
   `/tmp/iso*`、`/tmp/wts2`、`/tmp/probe*`、`~/dpk/`。
4. 用 `wt_plan_demo` 复现 README 里引用的数字（文档已改为指向该程序，
   但**没有复核过 README 里的具体数值是否与程序输出一致**）。

## 7. 留给用户的开放问题

- `/data/wt_inspection/` 到底该不该存在？已确证官方无此要求，正确位置是
  应用目录下的 `data/`。但现场部署时作业输出（照片、CSV、报告）希望落在哪，
  是产品决定——`/data` 掉电不丢、`data/` 随包走但卸载即失。
- 输出目录要不要做成可配置项？按 `wt-inspection-config-strictness` 的思路，
  能由部署形态决定的不该让现场填。
- `wind-turbine-inspector`（`/open_app/wti_debug/`）是设备上的前代项目，
  它的路径处理方式是否值得参考？
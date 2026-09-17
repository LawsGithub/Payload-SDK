# CLAUDE.md — Payload-SDK / 风机叶片巡检

## Cross-session handoff

- On session start: read [HANDOFF.md](HANDOFF.md) fully, then summarize the previous session's goal, current state, and next step before proceeding.

## 妙算3 测试设备（目标机）

```bash
sshpass -p 'dji' ssh -o StrictHostKeyChecking=accept-new dji@192.168.42.120
scp -o StrictHostKeyChecking=accept-new <文件> dji@192.168.42.120:/tmp/
```

用户 `dji` / 密码 `dji` / Ubuntu 20.04 aarch64 / glibc **2.31**，
USB 网络共享经本机 `eth4`（`192.168.42.0/24`）。系统可能被重置，
重置后 `authorized_keys` 清空，密码是唯一入口。若不通先扫网段找 IP。

## 两条硬约束（每次都要遵守）

### 1. 绝不用 sudo —— dji 用户没有 root 权限

`sudo` 一律失败（`Sorry, user dji is not allowed to execute ...`），
`date -s` 也不行。但 **`dji_app_ctl` 不需要 sudo** —— `stop` / `install`
等命令在 dji 用户下直接可用，别把它归到"需要 root"那一类。

- **改不了系统时间** → 设备时钟可能落后本机几十小时（RTC 无电池，掉电回
  1970；进一次 Pilot 2 飞行界面会经飞机校时）。若 `tar` 报"时间戳在未来"、
  `make` 报 `Clock skew detected`（**可能导致构建不完整**），用
  `tar --mtime='@0'` 把时间戳压到过去再传，不要去改设备时间。
- **`/data/` 不可写，且 `.dpk` 也解决不了** → 装成 dpk 后应用仍以
  `uid=1000`（dji）运行，**不是 root**。输出的正确位置是应用目录下的
  `data/`：官方打包脚本 `build_dpk.sh` 会在包内建好，属主 `dji:dji`、可写。
  绝对路径 `/data/wt_inspection/` 是当初"手工跑二进制"场景带进来的错误假设。

### 2. 运行前必须让出 PSDK 通道

`Smart3DExplore`（中文"自动探索"，DJI 预装）**开机会自启**并占用 PSDK 通道。
不同时会导致我们的程序报出**误导性的错误**（看起来像机型不匹配）：

```
dji_channel_local.c:  local channel bind failed, error: Address already in use
dji_core.c:           Identify device error, Please confirm that only one PSDK program...
或  main.c:288          获取机型信息失败，无法确认硬件是否匹配
```

**每次跑程序前先执行**（判据，退出码 1 = 无进程 = 通道可用）：

```bash
pgrep -x Smart3DExplore >/dev/null || /system/bin/dji_app_ctl stop Smart3DExplore
```

即：**只要有进程就停掉**。`pgrep -x` 是精确匹配进程名，别用 `ps | grep`。

### 3. `.dpk` 安装器会试运行应用（2026-09-17 实测）

`dji_app_ctl install` 会在安装过程中**试运行应用，并要求它走完 SDK 身份
校验**。应用若在 `DjiCore_Init` 之后、校验完成之前退出，安装即失败，而报
的却是一句误导性的错误：

```
Error, verify app user_app_id or version info error
```

**真正的死因在应用内部，安装器看不到**（它只能观测"进程没了"，就归因到
它唯一能校验的两样东西）。所以：**启动路径上不能有"配置缺失就退出"这类
防御** —— fail-closed 的边界应划在「作业开始」，不是「进程启动」。

判据：应用日志（`/blackbox/system/app_temp_files/<name>_*.log`）里能看到
`dji_identity_verify.c:654 Update dji sdk policy file successfully`
才算走完校验。装包**需要飞机通电并连接**。

## 工作方式

**在设备上本地编译，编好再部署。**不用交叉编译：本机 WSL glibc 2.35 >
设备 2.31，交叉编译产物要求 `GLIBC_2.34`，设备上 `ldd` 直接报 not found。

```bash
# 本机：只打包构建必需的部分（约 1MB，含 dpk 打包脚本 28KB）
tar czf /tmp/wtbuild.tar.gz --mtime='@0' \
     wt_inspection tools/build_dpk psdk_lib/include \
     psdk_lib/lib/aarch64-linux-gnu-gcc \
     samples/sample_c/platform/linux/{manifold3,common}
# 注意：CMake 硬编码找 psdk_lib/lib/aarch64-linux-gnu-gcc/libpayloadsdk.a
#      （即使原生编译也走这条路径），打包时别压平目录层级

# 设备：解包后
cmake -S . -B build-native -DWT_BUILD_PSDK_APP=ON -DPSDK_ROOT=<包根目录>
make -C build-native -j4
ctest --test-dir build-native --output-on-failure
```

E-Port 端点 `/dev/usb-ffs/bulk{2,3,4,5}/`（`ep0/ep1/ep2`，可读写）。

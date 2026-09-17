# CLAUDE.md — Payload-SDK / 风机叶片巡检

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
连 `date -s`、`dji_app_ctl` 都不行。需要 root 的事只能换思路：

- **改不了系统时间** → 设备时钟可能落后本机几十小时。若 `tar` 报
  "时间戳在未来"、`make` 报 `Clock skew detected`（**可能导致构建不完整**），
  用 `tar --mtime='@0'` 之类把时间戳压到过去再传，不要去改设备时间。
- **写不了 `/data/`** → 默认输出目录 `/data/wt_inspection/` 建不出来。
  实验时改配置 `output_dir = /tmp/wtrun`；正式部署需走 `.dpk` 安装包
  由系统授权（`dji_app_ctl install -i <file.dpk>`）。

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

## 工作方式

**在设备上本地编译，编好再部署。**不用交叉编译：本机 WSL glibc 2.35 >
设备 2.31，交叉编译产物要求 `GLIBC_2.34`，设备上 `ldd` 直接报 not found。

```bash
# 本机：只打包构建必需的部分（约 1MB）
tar czf /tmp/wtbuild.tar.gz wt_inspection psdk_lib/include \
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

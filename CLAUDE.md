# CLAUDE.md — Payload-SDK (DJI PSDK fork)

本仓库是 `dji-sdk/Payload-SDK` 的 fork（远端 `fork` = LawsGithub/Payload-SDK）。
**本文件提交在 `master` 上，因此每个新分支都会继承它 —— 新分支开工前先读这一份。**

设备连接方式与妙算3 的环境约束在上一级：`../CLAUDE.md`（本机专属，不入版本库）。

---

## 仓库布局：一个 .git + 每项目一个 worktree

本仓库同时承载多个项目，**每个项目一条独立分支、一个平级工作区**，
共用 `Payload-SDK/.git`（对象库只此一份）：

```text
~/projects/
├── Payload-SDK/       master（上游参照系，samples 全量）
├── wt_inspection/     feature/wt-inspection
└── liangzhourenwu/    feature/liangzhourenwu（稀疏检出）
```

**新建一个项目**：

```bash
cd ~/projects/Payload-SDK
git worktree add -b feature/<项目名> ~/projects/<目录名> master
```

**新 worktree 建议稀疏掉 `samples/`**（152 M / 865 文件，占跟踪文件 91%）。
编译实际只需要 `samples/sample_c/platform/linux/{manifold3,common}`（21 文件 / 200 KB）：

```bash
cd ~/projects/<目录名>
git sparse-checkout init --cone
git sparse-checkout set psdk_lib doc tools
git sparse-checkout add samples/sample_c/platform/linux/manifold3 \
                        samples/sample_c/platform/linux/common
# 需要看样例时按需取回：
git sparse-checkout add samples/sample_c++/platform/linux/manifold3
```

稀疏配置是 **per-worktree** 的（存在 `.git/worktrees/<名>/info/sparse-checkout`），
不会影响其它工作区。

### 合并上游

`master` 与 `origin/master` 保持零差异，**不要往 master 提交项目代码**——
否则将来 `git merge origin/master` 会一直带着这笔分叉。项目代码只进 `feature/*`。

---

## 硬规则（每个分支都遵守）

### 1. 绝不用 sudo —— dji 用户没有 root 权限

设备上 `sudo` 一律失败。但 **`dji_app_ctl` 不需要 sudo**——`stop` / `install`
在 dji 用户下直接可用，别把它归到"需要 root"那一类。

### 2. 绝不交叉编译 —— 必须在设备上本地编译

本机 WSL glibc **2.35** > 设备 **2.31**，交叉编译产物要求 `GLIBC_2.34`，
设备上 `ldd` 直接报 not found。**设备本机编译的产物最高只要求 `GLIBC_2.17`。**

```bash
# 本机：只打包构建必需的部分（约 1MB，含 dpk 打包脚本）
tar czf /tmp/wtbuild.tar.gz --mtime='@0' \
     wt_inspection tools/build_dpk psdk_lib/include \
     psdk_lib/lib/aarch64-linux-gnu-gcc \
     samples/sample_c/platform/linux/{manifold3,common}
# 注意：CMake 硬编码找 psdk_lib/lib/aarch64-linux-gnu-gcc/libpayloadsdk.a
#      （即使原生编译也走这条路径），打包时别压平目录层级

# 设备：解包后
cmake -S . -B build-native -DWT_BUILD_PSDK_APP=ON -DPSDK_ROOT=<包根目录>
make -C build-native -j4
```

`--mtime='@0'` 是必须的：设备 RTC 无电池、掉电回 1970，不压时间戳会报
"时间戳在未来"，`make` 的 `Clock skew detected` **可能导致构建不完整**。

### 3. 运行前必须让出 PSDK 通道

`Smart3DExplore`（DJI 预装的"自动探索"）**开机会自启**并占用 PSDK 通道，
导致我们程序报出**误导性的错误**（看起来像机型不匹配）：

```text
local channel bind failed, error: Address already in use
Identify device error, Please confirm that only one PSDK program...
获取机型信息失败，无法确认硬件是否匹配
```

**每次跑程序前先执行**：

```bash
pgrep -x Smart3DExplore >/dev/null || /system/bin/dji_app_ctl stop Smart3DExplore
```

`pgrep -x` 是精确匹配进程名，别用 `ps | grep`。

### 4. `.dpk` 安装器会试运行应用 —— 启动路径上不能有"配置缺失就退出"

`dji_app_ctl install` 会**试运行应用并要求走完 SDK 身份校验**。应用若在
`DjiCore_Init` 之后、校验完成之前退出，安装即失败，而报的却是：

```text
Error, verify app user_app_id or version info error
```

**真正的死因在应用内部，安装器看不到。** fail-closed 的边界必须划在
「作业开始」，不是「进程启动」。

判据：应用日志 `/blackbox/system/app_temp_files/<name>_*.log` 里能看到
`dji_identity_verify.c:654 Update dji sdk policy file successfully`
才算走完校验。装包**需要飞机通电并连接**。

### 5. 设备是多个项目共用的同一台机器

装了哪个 dpk、占没占 PSDK 通道、`/tmp` 里留了什么，都跨项目互相影响。
用完清干净自己造的东西，别留下会撞名的安装包和进程。

---

## 跨会话交接

- 仓库根若有 `HANDOFF.md`，**会话开始时先读全文**，再复述上一会话的目标、
  当前状态、下一步，然后才动手。
- `HANDOFF.md` 的信任标记：`[V]` = 交接时已用命令验证；`[?]` = 仅记忆未复核，
  当线索对待；`[X]` = 已证伪，别用。
- 漂移检查：`git rev-parse HEAD~1` 应等于 `HANDOFF.md` 里记录的 SHA——
  HEAD 应是本次 handoff 提交，其 parent 才是快照记录的锚点。

---

## 本地文件（不入版本库）

| 文件 | 说明 |
|---|---|
| `wt_credentials.ini` / `app_info.txt` | 明文 App Key / License，**绝不入库** |
| `.vscode/`、`.claude/` | 机器相关配置，已在 `.git/info/exclude` 本地忽略 |
| `wt_inspection/build*`、`data/`、`*.dpk` | 构建/运行产物 |

注意：`wt_credentials.ini` 是**未跟踪**文件，`git worktree add` **不会**复制它——
新建 worktree 后要手工从别的 worktree 拷过去。

---

## E-Port 端点

`/dev/usb-ffs/bulk{2,3,4,5}/`（`ep0/ep1/ep2`，可读写）。
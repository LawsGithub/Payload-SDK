# 打包 · 设备侧打包清单 · 已证伪的路 · 已知坑

> 从 `CLAUDE.md` 外置（2026-10-10）。含 dpk 打包要点、tar 时间戳的两个方向、
> 设备清理记录、不要再试的路、以及本项目特有的踩坑清单。
> 信任标记：`[V]` 已用命令验证 · `[?]` 仅记忆未复核 · `[X]` 已证伪。

## 打包

```bash
# 设备上编译完成后（在仓库根，即 ~/lzbuild）
tools/build_dpk/build_dpk.sh -i lz/app_json/app.json -o ~/dpk
dji_app_ctl install -i ~/dpk/liangzhourenwu_v01.00.00.00.dpk
```

- 构建目录名必须是 `build-native`（`app.json` 的 `bin` 指向 `../build-native/bin/lz_app`）。
- `app.json` 的 `userconfig` 必须是 `["../app/widget_file"]` —— `build_dpk.sh` 会
  `cp -r` 到**包根**，产出 `/open_app/<app>/widget_file/`，与运行时 CWD（包根）一致。
- 装包**需要飞机通电并连接**，且启动路径不能提前退出
  （见仓库级 CLAUDE.md 的「安装器会试运行应用」）。
- **`app.json` 必须四语言齐全**（`description_{cn,en,jp,fr}`）—— `build_dpk.sh`
  逐个字段校验，缺一个直接退出，报 `KeyError: 'description_jp'`。

### 设备清理记录（2026-09-19）

设备是**多项目共用**的，wt_inspection 在上一轮开发中留下了不少产物。已清理：

| 位置 | 清了什么 | 结果 |
|---|---|---|
| 已安装应用 | `wt-widget-demo`（2026-09-17 装，data 285 MB） | ✅ `APP UNINSTALL SUCCESS` |
| `/open_app/` | `wind-turbine-inspector_*.dpk`、`wti-build-src/`(11M)、`wti_debug/`(1.7M)、`wti-source-m3.tar.gz`(960K) | ✅ 已删 |
| `~/dpk/` | `wt-inspection_v01.00.00.00.dpk`(276K) | ✅ 已删 |
| `~/data/logs/WT/` | wt 的运行日志 | ✅ 已删 |
| `/blackbox/system/app_temp_files/wt-*.log` | 27 个 | ❌ **删不掉** |

`/blackbox/system/app_temp_files/` 是 **root:root 0755**，`dji` 用户无写权限，
**且没有 sudo** —— 这 27 个日志只能留着。它们只是文本日志，不影响功能与通道。

> 该目录里还有**别的项目的日志**（如 `matrice4t-square-mission_*.log`，7 月），
> 所以它是 DJI 统一管理的应用日志区，别把它当自己的地盘。

**留给下次的提醒**：清理时先 `dji_app_ctl uninstall` **应用**（它会连带清
`/open_app/wt-widget-demo/` 与 data），再手工删源码目录与安装包。

## 设备侧打包清单（约 1MB）

```bash
# ⚠️ 时间戳必须取「**设备**当前时间附近」，不是本机时间附近。
# 设备时钟可能落后几十小时（2026-09-26 实测落后 43.7 h），
# 按本机时间算出来的 --mtime 对设备是**未来** → tar 报 "time stamp ... in the future"。
DEVT=$(sshpass -p 'dji' ssh -o StrictHostKeyChecking=accept-new dji@192.168.1.180 'date +%s')
tar czf /tmp/lzbuild.tar.gz --mtime="@$((DEVT - 60))" \
     --exclude='lz/build*' --exclude='*.dpk' \
     lz tools/build_dpk psdk_lib/include psdk_lib/lib/aarch64-linux-gnu-gcc \
     samples/sample_c/platform/linux/{manifold3,common}
```

⚠️ **必须带 `--exclude='lz/build*'`** —— 否则会把本机 PC 侧的 `build/`
（x86 产物）一起传上去，设备上解包后与 aarch64 构建混在一个目录里。
`tools/build_dpk` 也要带上（打包 dpk 在设备上做）。

⚠️ **不要用 `--mtime='@0'`** `[V]`（2026-09-19 实测踩到）—— 它把源码时间戳压到
1970，`make` 会发现**源码比 `.o` 旧**，于是**静默跳过编译**。现象是"传了新代码
上去，跑的还是旧二进制"，极难察觉。

⇒ **两个方向的坑都要防**：往未来压会被 tar 拒（且 `make` 可能报
`Clock skew detected` 并**产出不完整的构建**）；往 1970 压会让 `make` 静默跳过。
安全区是「**设备当前时间往前一点**」—— 用 `@$((DEVT-60))` 从设备取时间再减 60 秒。

正确做法：时间戳取**设备当前时间往前 60 秒**（上面的命令块已经这么写了）。
两个方向的坑都实测过：`+30 seconds` 报 "is 29.2 s in the future"
（tar 取的是打包那一刻，而设备时间在走）；压到 `@0` 让 make 静默跳过编译。
**取中间值** —— 既不在未来，也不倒挂。

⚠️ 术语澄清：上级 `~/projects/CLAUDE.md` 说的"用 `--mtime='@0'` 压到过去"
防的是"时间戳在未来"，但压到 1970 是反方向的倒挂。**这里以本文件的
`@$((DEVT-60))` 为准。**

「CMake 硬编码找 `psdk_lib/lib/aarch64-linux-gnu-gcc/libpayloadsdk.a`、
别压平目录层级」见仓库级 CLAUDE.md。

## 已证伪的路

沿仓库级 CLAUDE.md 与 `wt_inspection` 的结论，不重复试：

- **交叉编译** → 产物要求 `GLIBC_2.34`，设备 `ldd` 报 not found `[V]`
- **在 `/tmp` 下打包/编译** → 设备重启后 `/tmp` 被清空 `[V]`
- **`.dpk` 不解决 `/data` 写权限** → 应用仍以 `uid=1000` 运行 `[V]`
- **Waypoint V2 用于 M4T** → 官方文档明文"仅支持 M300 RTK 和 M350 RTK" `[V]`
- **兴趣点环绕（`DjiInterestPoint_*`）可控半径** → settings 里没有 radius 字段 `[V]`
- **"KMZ 对第三方负载没意义"** → **已证伪**：`gimbalRotate` 是独立于相机的动作，
  且支持 `gimbalYawRotateEnable` + `absoluteAngle` `[V]`
- **`tar --mtime='@0'`** → **已证伪**：导致 make 跳过编译（见上）`[V]`
- **"模拟器不实现航点启动"** → **已证伪（证据不足）** `[X]`（2026-09-21）：
  该结论建立在 `0x000000FF` 上，而那是被 `(unsigned)` 截断后的假象；
  且当时那份 KMZ 确实缺 wpml 必需元素，**从未通过过内容校验**。
  详见 [`lz/doc/WPML-ORBIT.md`](WPML-ORBIT.md)「绕飞启动曾经失败的原因」。
- **`-DPSDK_ROOT=~/path`** → **已证伪** `[V]`：`~` 不被展开，用 `$HOME`（见上）

## 已知坑（本项目特有）

- **方位角不能用绝对差比较** `[V]`（2026-09-18）——正北方向处球面计算返回
  `359.999999998°`，与 `0°` 只差 2e-9°，绝对差却是 360。测试须用
  `LZ_CHECK_ANGLE_NEAR`（圆周差），见 `tests/lz_test.h`。
- **`LzGeo_NormalizeDeg` 的区间是 [0,360) 左闭右开** `[V]` ——极小负数加 360
  会因舍入得到恰好 `360.0`，函数里已收回；改动时别删那个判断。
- **`exception` 白名单式判据害人** `[V]`（2026-09-22）—— 详见
  [`lz/doc/LASER-AND-POLE.md`](LASER-AND-POLE.md)。
  取值域没有权威来源的字段，别用枚举白名单，改用能自证的量。
- **判据对 ≠ 接线对** `[V]`（2026-09-22）—— 给激光零解闸门写的测试只断言
  `LzGeo_IsNullSolution()` 判据本身有效，**没断言取数函数真的调用它**。
  反向验证时删掉闸门，测试**照样全绿** —— 因为未定义 `LZ_POLE_SOURCE_LASER`
  时整个激光实现分支不参与编译，测试看不见它。
  **修法**：把判定抽成零依赖的纯函数（`LzPole_JudgeLaserReading`）放在
  `#ifdef` **之前**，测试直接打它。
- **`(unsigned)rc` 会丢掉 PSDK 返回码的模块号** `[V]`（2026-09-21）——
  `T_DjiReturnCode` 是 `uint64_t`，模块号在高 32 位
  （`DJI_ERROR_MODULE_INDEX_OFFSET = 32`）。用 `(unsigned)` 截成 32 位后，
  日志里那个"小错误码"可能根本不是你以为的那个。用 `%llX` 配
  `(unsigned long long)`。
- **`-DPSDK_ROOT=~/path` 里的 `~` 不会被展开** `[V]`（2026-09-21）——
  波浪号展开只认词首与 `=`/`:` 之后，而 `-DVAR=~/...` 整是一个词。
  CMake 把它当字面目录名，报的是"找不到静态库"（病因与提示不符）。用 `$HOME`。
  ⚠️ 改了 `-D` 参数必须重跑 `cmake` 配置：`CMakeCache.txt` 会记住旧值。
- **SDK 的日志流在我们手上路过** `[V]`（2026-09-21）—— PSDK 不把真正的错误码
  通过 API 给我们（`DjiWaypointV3_Action` 只回 `0x000000FF`），但飞机给的原因在
  SDK 自己的 `USER_LOG_ERROR` 里，而那个 console 是我们注册的。
  实现见 `app/platform/lz_sdk_log_watch.c`：两个 console **先喂再输出**，
  自己拼行（`ConsoleFunc` 的 `dataLen` 是任意长度，可能给半行 ——
  直接对 chunk 做 strstr 会在跨 chunk 时漏掉目标串）。
  **抓不到要返回 NULL 并让调用方容忍"没有这条信息"，不能把"抓不到"当成"没失败"。**
- **注释里不能写 `*/`** `[V]`（2026-09-20）——在 `/* ... */` 块注释里写
  `widget_file/*/` 会**提前闭合注释块**，后面整段代码变成语法垃圾。报错位置
  和病因完全对不上。
- **`-1` 会满足 `<= 1`** `[V]`（2026-09-19）——探针里 `watchMode(-1)` 的判断
  写在 `samples <= 1` 之后，导致连续监视模式掉进单次分支。**负值哨兵与
  正数阈值共用变量时，判断顺序决定成败。**
- **输出重定向到管道/文件时 stdio 是全缓冲** `[V]`（2026-09-19）——实时监视
  类程序必须每行 `fflush(stdout)`，否则攒够 4 KB 才可见，"实时"变成"批次"。靠
  `sshd` 转发时同样会踩到。
- **gdb 抓 PSDK 进程要先 `handle SIG32 nostop noprint pass`** `[V]`
  （2026-09-20）——PSDK 的 linker 线程用 SIG32 做实时事件通知，gdb 默认会
  停在它上面，真正的 SIGSEGV 反而看不到。用法：
  `gdb -batch -ex "handle SIG32 nostop noprint pass" -ex run -ex bt --args <bin> <args>`
- **诊断程序必须在读之前就打印并 flush** `[V]`（2026-09-20）——崩溃会吞掉
  stdio 缓冲区里没刷出去的内容，而那几行恰恰是定位崩溃点的关键。

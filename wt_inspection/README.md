# 风机叶片无人机巡检（M4T + 妙算3）

基于 DJI Payload SDK 的风机叶片自主巡检方案实现。复刻 DJI 行业官网
[M4 系列风机叶片巡检方案](https://enterprise.dji.com/cn/news/detail/matrice-4-series-wind-turbine-inspection)
的**两阶段**技术路线（粗模环绕建模 → 精细巡检），执行主体为妙算3 机载计算。

本文档是该方案的**唯一说明来源** —— 含系统架构、成像参数推导、安全设计
原理与部署流程；所有参数数字由规划器产出，可用 `wt_plan_demo` 复现。

---

## 快速开始

### PC 侧：算法自检（无需 PSDK，秒级编译）

```bash
cmake -S . -B build && cmake --build build
ctest --test-dir build --output-on-failure     # 回归测试，秒级
./build/wt_plan_demo                          # 规划一条样例航线并打印全过程
./build/wt_plan_demo mission.csv mission.kmz  # 同时导出航点表与 PSDK 航线文件
./build/wt_plan_demo --trace                  # 改用「沿叶片中线飞行」模式
```

`wt_plan_demo` 会输出：相机模型核验表、风轮参考系、采样密度推导、
航线概览、安全校验报告 —— 这些是**参数是否合理的直接依据**，
调参时应当反复看这张输出，而不是凭感觉改数字。

采样密度那一块直接调用 `WtPlan_ResolveBladeSampling()`，与规划器共用同一份
公式；`wt_plan_demo` 打印的数字也是从同一处取的。这套「数字只有一个来源」的约束
由 `tests/` 下的回归测试守住：`wt_test_plan` 核对规划结果，
`wt_test_config` 核对「配置文件里写下的参数必须真的生效」，
`wt_test_bridge` 核对「报告里的预计耗时与 KMZ 下发的速度同源」。

### 妙算3 目标：在设备上本地编译

**不要交叉编译。** 本机 WSL 的 glibc 2.35 新于设备的 2.31，交叉编译产物会
要求 `GLIBC_2.34`（实测 `readelf -V`），在设备上 `ldd` 直接报 not found，
**编得出、跑不了**。设备自带 gcc 9.4 + cmake 3.16，本地编译最高只要求
`GLIBC_2.17`。

```bash
# 本机：只打包构建必需的部分（约 1MB）
tar czf /tmp/wtbuild.tar.gz --mtime='@0' \
     wt_inspection tools/build_dpk psdk_lib/include \
     psdk_lib/lib/aarch64-linux-gnu-gcc \
     samples/sample_c/platform/linux/{manifold3,common}

# 设备：解包后
cmake -S . -B build-native -DWT_BUILD_PSDK_APP=ON -DPSDK_ROOT=<包根目录>
make -C build-native -j4        # 产出 bin/wt_inspection_app
```

`tools/build_dpk/` 只为打包 `.dpk` 而传，只有 28KB（打包脚本 + 说明）。
`--mtime='@0'` 把时间戳压到过去：设备 RTC 无电池、掉电回 1970，不压会触发
`Clock skew detected`（**可能导致构建不完整**）。

### 机载应用凭据（构建期注入）

APP 信息不写进源码树，而是放在仓库外的 `wt_inspection/wt_credentials.ini`：

```ini
# 从 https://developer.dji.com/user/apps/#all 申请后填入
app_name=风机叶片识别
app_id=<APP ID>
app_key=<App Key>
app_license=<App License>
developer_account=<开发者账号>
baud_rate=460800
```

配置阶段由 `cmake/gen_app_info.cmake` 读取该文件，生成
`build-m3/generated/dji_sdk_app_info.h` 并置于头文件搜索路径最前，
遮蔽源码树中官方样例那份同名占位文件。

之所以绕这一圈：凭据是明文，写进源码树后一旦提交就**永久留在 git 历史里**，
事后删除也清不掉。生成物落在构建目录，与源码树和版本库双向隔离。

- 该文件不存在时配置阶段给出 WARNING，程序仍能编出来，但启动时会明确
  拒绝并提示 —— 比含糊的"授权失败"好定位。
- 文件里出现未知键名、缺失必需项、或值中含引号/反斜杠时，配置阶段直接
  FATAL_ERROR，不静默降级。
- 生成的文件名与官方样例**故意保持一致**，这样 PSDK 分发包里任何
  `#include "dji_sdk_app_info.h"` 都无需改动。

跨平台一致性由 CMake 保证，无需 `sed` 之类的平台相关手法。

### 安装包（.dpk）

`app_json/app.json` 是生成 `.dpk` 安装包所需的配置，由官方打包脚本
`../tools/build_dpk/build_dpk.sh` 读取（依赖仅 Python 3 + dpkg，**与目标架构
无关**，在 WSL 或设备上打都一样）：

```bash
bash ../tools/build_dpk/build_dpk.sh -i app_json/app.json -o ~/dpk
/system/bin/dji_app_ctl install -i ~/dpk/wt-inspection_v01.00.00.00.dpk
```

`app.json` 的 `bin` 字段是**相对 `app.json` 的路径**，当前指向
`../build-native/bin/wt_inspection_app`（即设备原生编译的产物）。
`build_dpk.sh` 会在包内创建 `data/logs/` 与 `data/media/` —— 这是官方的
**相对应用目录**约定，无需任何绝对路径。

> **`user_app_id` 必须与 `wt_credentials.ini` 的 `app_id` 一致**，且
> `firmware_version` 必须与 `app/main.c` 里 `T_DjiFirmwareVersion` 的初值
> 一致（当前均为 `01.00.00.00`）。不一致时应用**装不上**，而报错是含糊的
> `verify app user_app_id or version info error`。这是同一份信息的两个副本、
> 两个来源，改一处必须同时改另一处。

#### 安装器会试运行应用 —— 启动阶段不能退出

`dji_app_ctl install` 会在安装过程中**试运行应用并要求它走完 SDK 身份校验**。
若应用在 `DjiCore_Init` 之后、校验完成之前退出，安装就判失败，且报的是上述
那句误导性的凭据/版本错误 —— **真正的死因在应用内部，安装器看不到**。

所以应用启动路径上**不能有"配置缺失就退出"这类防御**：fail-closed 的边界
应划在「作业开始」，不是「进程启动」。应用跑在 `uid=1000`（**不是 root**），
`/data/` 不可写；输出的正确位置是应用目录下的 `data/`（包内已建好、属主
`dji:dji`、可写）。

已落实为：配置缺失或非法时只置 `s_configReady = false`，进程保持存活走完
身份校验，等到要执行作业时才拒绝，并以**返回码 0** 退出。安装实测通过
（`APP INSTALL SUCCESS`），安装器试运行期间配置模板被正常落盘到
`data/wt_config.ini`。

配置路径与默认输出目录都是**相对应用目录**的（`data/wt_config.ini`、
`data/`）—— 应用的工作目录就是包根，这也是 `WT_LOG_PATH = "data/logs/WT"`
这个相对路径成立的前提。

---

## 分层结构

```
include/  src/
├── wt_geometry      WGS84 ↔ ENU 切平面、矢量与角度运算
├── wt_turbine       塔筒/机舱/叶片解析几何、风轮参考系、相位反解
├── wt_camera        等效画幅 GSD、视场角、重叠率 ↔ 站位间距
├── wt_plan          三阶段航线规划 + 安全校验 + 危险航段修补
├── wt_bridge        航线 → KMZ(wpml)、动作表、飞行参数表
├── wt_app_config    作业配置解析与模板生成
├── wt_telemetry     相位反解/滤波/外推     (+ _psdk.c: 话题订阅)
└── wt_runner        作业流水线编排          (+ _psdk.c: 下发与监控)

app/main.c           机载应用入口（平台注册 + 作业调度）
tools/wt_plan_demo.c PC 侧自检程序
tests/                PC 侧回归测试（wt_test_plan / _config / _bridge）
```

**分层准则**：`wt_core` 里任何一行都不包含 PSDK 头文件。
规划、几何、安全校验是出错代价最大的部分，必须能在桌面上反复验证；
只有真正碰硬件的代码才进 `*_psdk.c`。

验证方式：

```bash
grep -rn "dji_" include/ src/wt_geometry.c src/wt_turbine.c \
     src/wt_camera.c src/wt_plan.c src/wt_bridge.c \
     src/wt_app_config.c src/wt_telemetry.c src/wt_runner.c    # 应无输出
```

---

## 安全设计要点

风轮可能转动时，「叶片现在在哪」毫无意义 —— 唯一安全的判据是
**保持在整个叶轮圆盘之外**。规划器据此区分两种作业模式：

| 模式 | 判据 | 适用 |
|---|---|---|
| 叶片静止 | 到任一叶片轴线的距离 > 安全值 | 停机受控精细巡检 |
| 风轮可转动 | 到旋转轴的轴向分量 > 安全值 | 叶尖追踪 / 未受控 |

### 两种航线形态

配置里的 `trace_enable` 在两种互斥的航线形态之间切换：

| | 盘外站位斜拍（默认） | 沿叶片中线飞行（`trace_enable = true`） |
|---|---|---|
| 站位 | 叶片旁边 `blade_standoff` 米，斜视 | 沿叶片轴线平行飞，离轴线 `trace_offset` 米 |
| 航线 | 盘外一列平行站位，132 点 / 1996 m | 去程→叶尖掉头→回程，123 点 / 600 m |
| 面覆盖 | 一次飞行拍一个面 | **一趟同时覆盖迎风面与背风面** |
| 换面 | 换面段单独规划 | 叶尖外绕半圈掉头，`towardPOI` 让机头转向 |

沿中线飞行模式的拓扑是：`捕获 → 轮毂 → 1号叶往返 → 轮毂 → 2号叶往返 → 轮毂 → 3号叶往返 → 结束`。
相邻叶片经轮毂正面/背面转场，**不跨叶片直连**。

叶尖掉头段用 `waypointHeadingMode = towardPOI`（兴趣点 = 该叶片叶尖）让**机头**绕叶尖转
180°，云台只保留俯仰（`gimbalYawRotateEnable = 0`）—— 这与需求「云台不变、只有无人机
偏航变化」一致，也是 M4T 上唯一可行的做法（M4T 的云台 Pan 轴 `Not controllable`，
云台 yaw 无法独立于机头偏转）。

⚠️ **该模式有固有的高度约束**：沿叶片全长飞，当某片叶片指向下方时叶尖接近地面，
航点会低于 `ground_clearance` 而被安全校验拒绝。规划器会打印是哪片叶片、低到多少，
此时应调整停用角或等待风向使叶片转开。

此外，航线组装的最后会**统一扫描全部航段**并修补穿越叶片扫掠区的段落
（出壳 → 沿弧 → 换高，全程位于安全圆柱面之外）。实测中暴露的六类典型
危险航段 —— 进场段横穿盘面、粗模环绕圆落在盘内、双面换面段擦过叶尖、
塔筒段与叶片冲突、站位在盘内却「当前无叶片」、圆弧中间落回盘内 ——
都由这套修补覆盖，其中塔筒段在风轮可转动时整段跳过。

---

## 已知限制

- 停用角取自每台风机的 `[turbine.xxx] park_phase`。未填（或填负数）时按
  「叶片停在 12 点方向」假设规划并告警，且在 `auto_start = true` 下**直接
  拒绝执行**（`WtRunner_PlanTurbine` 返回 `WT_RUN_ERR_SAFETY`）。现场必须
  实测填值后才能自动作业
- 停用角的地面站下发通道仍未实现：目前只有配置文件一个来源，改值后需重启
  应用；视觉反解（`wt_telemetry`）已有接口但无人调用
- 转动模式下塔筒巡检段被整体跳过（塔筒必然在叶轮圆盘内，无法规避）
- 相位视觉反解只提供接口，**目前无人调用**，故闭环叶尖跟踪尚未真正生效；
  像素级换算需由视觉模块接入相机内参
- 未考虑风速下叶尖的动态摆幅，转动模式下 `min_safe_dist` 需按现场风速调大
- 配置解析是严格的：已知段落内出现模板之外的键名会拒绝启动（这是有意的，
  防止 `min_safe_distt` 这类拼写错误被静默丢弃）
- 沿中线飞行模式的 **GSD 不做斜距判定** —— 该模式相机正对叶面，GSD 随叶弦变化
  而非随斜距变化，用斜距公式算出来是错的值。规划器把该模式的 `gsdMmPerPx` 置 0
  表示「未计算」，安全校验显式跳过而非静默通过。该模式的成像质量需按弦长另行评估
- 叶片几何是**零厚度轴线模型**，没有弦长/厚度剖面。因此「叶片中线」目前只能用
  轴线近似 —— 真实中线在叶根处离轴线约 2 m、叶尖处约 0.02 m。要精确实现需给
  风机模型加弦长剖面
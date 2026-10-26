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

### Pilot 2 控件 —— 操作员界面

停机位相位是**现场实测值**，而飞机在天上时 SSH 改配置这条路根本不存在。
`app/wt_widget.c` 把作业控制做成 Pilot 2 上的控件，操作员在遥控器上直接操作：

| 界面 | 控件 | 作用 |
|---|---|---|
| index | 控件 | 作用 |
|---|---|---|
| 0 | 开始作业（开关） | ON = 开始，OFF = 清除请求 |
| 1 | 中止作业（按钮） | 风机作业：暂停任务并返航；行人瞄准：结束会话；绕点飞行：STOP、原地悬停 |
| 2 | 干跑（开关） | ON = 只上传不启动（风机作业与绕点飞行都认） |
| 3 | 停用角（滑块） | 0–100% 映射到 0–359° |
| 4 | 行人瞄准（开关） | ON = 开始一次瞄准打点会话，OFF = 中止 |
| 5 | 绕点飞行（按钮） | 以最近一次行人打点为基点飞正方形，见「绕点飞行」一节 |

全部在 PSDK 菜单（`main_interface`，飞行中操作）。

**停用角有两个来源，控件优先**：控件设过就用控件的，没设过回落到配置文件
`[turbine.xxx] park_phase`。取值在 `WtRunner_RunTurbine` 内部、**规划前一刻**
才做，不是作业开始时快照 —— 操作员在作业进行中填的值也能生效。

**停用角未知（负值）时一律拒绝执行**，不看是从哪条路径启动的。这条判据
曾经挂在 `autoStart` 上，而控件启动时该开关仍为 false，导致保护完全失效
（停用角 -1.0 却照样上传航线）。

**作业被拒绝后回到待命，不退出**：清空开始请求，操作员改完参数可再拨一次，
不必重启应用、不用重新拉控件列表。

⚠️ **`int_input_box` 两个界面都不渲染**。放 `main_interface` 会被 SDK 收下
但 Pilot 不显示（liangzhourenwu 2026-09-28）；放 `config_interface` 时日志
解析计数正常、Pilot 2 界面上同样看不到那个框（本项目 2026-10-20 实测，
反复退出重进也没有）。⇒ 要让操作员改数值，**用 `scale` 滑块放 main 界面**，
滑块在 main 验证过能正常显示。代价是精度约 3.6°/档，对来自现场目视/台账的
停用角够用。

⚠️ **控件回调跑在 PSDK 工作线程上，只置标志不做动作**。规划 + 导出 KMZ + 上传
要数秒，中止要调阻塞的 `DjiWaypointV3_Action` / `StartGoHome` —— 在回调里做会
把 SDK 链路线程卡死（liangzhourenwu 项目 2026-09-22 因此闪退过）。动作一律由
主循环或执行循环取走执行。

⚠️ **空转的待命循环不调 `WtTelemetry_Poll()`**（2026-10-20 实测每秒调一次会在
3 秒内 SEGV/ABRT）。根因后来查清是当时 Poll 里用的 `GetLatestValueOfTopic`
在本 SDK 版本上必崩，2026-10-24 已改成读回调缓存（323787e）；从待命循环发起的
会话（采集、行人瞄准、绕点飞行）都会调 Poll，采集会话改完后上机跑通过。
空转的待命循环本身没有要用遥测的地方，所以仍然不调。

控件注册必须在 `DjiCore_ApplicationStart()` **之前**，否则 Pilot 拉不到列表。
注册失败**不阻断启动** —— 控件是操作便利性，不是作业前提；在这里 fail-closed
会让 `.dpk` 装不上。

---

## 分层结构

```
include/  src/
├── wt_geometry      WGS84 ↔ ENU 切平面、矢量与角度运算
├── wt_turbine       塔筒/机舱/叶片解析几何、风轮参考系、相位反解
├── wt_camera        等效画幅 GSD、视场角、重叠率 ↔ 站位间距
├── wt_blade_id      叶片编号：角度 → NO.1/2/3（按 park_phase 匹配）
├── wt_projection    像素 ↔ 三维：相机反投影、射线与叶轮盘面求交
├── wt_ai_solve      端到端：AI 关键点 → 叶片编号 + 三维位置
├── wt_plan          三阶段航线规划 + 安全校验 + 危险航段修补
├── wt_bridge        航线 → KMZ(wpml)、动作表、飞行参数表
├── wt_app_config    作业配置解析与模板生成
├── wt_telemetry     相位反解/滤波/外推     (+ _psdk.c: 话题订阅)
├── wt_ped_logic     行人瞄准打点的决策层（选人、关联、停稳、逐帧确认、激光取中位）
├── wt_ped_square    绕点飞行：以行人坐标为中心的正方形航线（KMZ 导出在 wt_bridge）
├── wt_mission_watch 航点任务进度判定（开始了没有、结束了没有）
└── wt_runner        作业流水线编排          (+ _psdk.c: 下发与监控)

app/main.c           机载应用入口（平台注册 + 作业调度）
app/wt_widget.c      Pilot 2 自定义控件（操作员界面，+ widget_file/ 配置）
app/wt_ai_stream_probe.c 诊断：相机取流探针（统计帧数与分辨率）
app/wt_ai_overlay.c      诊断：向 Pilot 2 发识别框（见下方"已知限制"）
app/wt_trt_pose.{h,cpp}  TensorRT YOLO 推理封装（风机关键点 / 行人检测共用）
app/wt_yolo_decode.c     YOLO 预处理与解码（纯 C，桌面可测）
app/wt_vision.c          推理线程（风机 / 行人两种模型）
app/wt_ped.c             行人瞄准打点的接线层（云台、激光、结果文件）
app/wt_square_mission.c  绕点飞行的上传、启动、监控与中止（Waypoint V3）
tools/wt_plan_demo.c PC 侧自检程序
tools/wt_ped_square_demo.c 绕点飞行航线的 PC 侧演示（规划 + 导出 KMZ + 航点表）
tools/ped_square_view/  把绕点飞行 KMZ 做成核对网页（Python + 本地 HTML）
tools/ped_model/     行人模型导出 ONNX + 解码交叉核对（Python）
tests/                PC 侧回归测试（wt_test_plan / _config / _bridge
                      / _blade_id / _projection / _ai_solve / _ped_logic
                      / _ped_square / _mission_watch / _yolo_decode 等，`ctest` 全跑）
```

**分层准则**：`wt_core` 里任何一行都不包含 PSDK 头文件。
规划、几何、安全校验是出错代价最大的部分，必须能在桌面上反复验证；
只有真正碰硬件的代码才进 `*_psdk.c`。

验证方式：

```bash
grep -rn "dji_" include/ src/wt_geometry.c src/wt_turbine.c \
     src/wt_camera.c src/wt_plan.c src/wt_bridge.c \
     src/wt_app_config.c src/wt_telemetry.c src/wt_runner.c \
     src/wt_ped_logic.c src/wt_ped_square.c src/wt_mission_watch.c   # 应无输出
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

**巡检段（轮毂↔叶尖）的机头朝向锁定为正对风轮盘面**，用
`waypointHeadingMode = smoothTransition` + 每点相同的 `waypointHeadingAngle`。
所有巡检点填同一个角，航段间过渡过程角度恒定 —— 相机因此始终正对叶面。

为什么不用 `towardPOI`：它是**朝一个点**，飞机从轮毂飞向叶尖时机头会持续转向
该点，方位一路在变；拍叶面要的是方位不变。

**这个角度不写死**：由 `wt_runner.c` 按每台风机的 `rotAxis` 反算后传入
（`WtEnu_Azimuth(WtEnu_Scale(frame.rotAxis, -1.0))`）。多台风机机舱朝向不同，
写死会让其余风机的拍摄角全错。用 `rotAxis` 而不是 `spec.headingDeg`，因为后者
没算进主轴仰角与锥角的影响。

航点类型统一为**平滑过点，提前转弯**（`toPointAndPassWithContinuityCurvature`
+ `useStraightLine = 1`，官方规范要求两条都满足）。原先用「直线飞行、到点停」，
每个航点刹车再加速，巡检拍图时姿态抖动大、耗电也高。

KMZ 里同时声明机型与负载（`droneEnumValue = 1100`、`payloadInfo` 的
`payloadEnumValue = 10052`）。**缺 `payloadInfo` 时 Pilot 2 只能按机型猜相机**，
表现为同一份 KMZ 切 M4E 显示 0 个航点、切 M4T 才正常。

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
  「叶片停在 12 点方向」假设规划并告警，且**一律拒绝执行**
  （`WtRunner_PlanTurbine` 返回 `WT_RUN_ERR_SAFETY`，不看是从配置还是控件启动）。
  现场必须实测填值后才能作业
- 停用角的地面站下发通道仍未实现：目前只有配置文件与 Pilot 控件两个来源；
  视觉反解（`wt_telemetry`）已有接口但无人调用
- **`template.kml` 与 `waylines.wpml` 内容完全相同**（`wt_bridge.c` 把同一个
  buffer 写了两次）。PSDK 上传不受影响（飞控只读 `waylines.wpml`），但导入
  Pilot 2 时统计栏会显示 0 m / 0 航点 —— Pilot 需要两份文件各司其职。
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

## AI 识别（进行中）

目标是识别**整个风机**：轮毂位置 + 三片叶片并编号 NO.1/NO.2/NO.3，用于把
视觉结果换算成航线规划需要的三维坐标。

### 硬件底座（2026-10-22 实测）

妙算3 是 **NVIDIA Orin NX**（`/proc/device-tree/model`），L4T R35.5.0。
DJI 出厂镜像已预装 **CUDA 11.4 / TensorRT 8 / cuDNN 8**，`dji` 用户无 sudo
也能用。`nvcc` 与 `trtexec` 不在 PATH 里，全路径分别是
`/usr/local/cuda-11.4/bin/nvcc`、`/usr/src/tensorrt/bin/trtexec`。

### 两条通道的实测结论

| API | 结果 |
|---|---|
| `DjiLiveview_StartImageStream` | ✅ **30fps 1440×1080 RGB_PACKED**（446 ms 出首帧） |
| `DjiLiveview_SendAiMetaToPilot` | 发送返回 SUCCESS，但 **Pilot 2 不渲染** |

后者不是 API 不可用：**Pilot 2 的「AI 识别」与妙算3 上的 PSDK 应用互斥**
（实测弹窗"请先关闭已运行 AI 应用再开启"）。开着 DJI AI 时不收外部框，
关掉又不在 AI 模式。所以自研识别的结果**自己用**，不要指望画给操作员看。

⚠️ **调用 liveview 前必须先 `DjiLiveview_Init()`。** 漏掉时
`RegUserAiTargetLableList` 返回 `0xE0` 且日志打
`Ai target Lable only support on manifold3` —— 看起来像硬件不支持，实际只是
SDK 内部缓存的 `s_mountPositionType` 还是 0（该变量正是在 `DjiLiveview_Init`
里由 `DjiAccessAdapter_GetMountPositionType()` 填充的）。

### 模型与关键点

模型来自 RWTH Aachen 开源项目
[`rwth-irt/Wind-Turbine-Synthetic-Vision`](https://github.com/rwth-irt/Wind-Turbine-Synthetic-Vision)
（ICMV 2025，`arXiv:2507.21611`），YOLOv11-pose，用 BlenderProc 合成数据训练。
公开数据集几乎全是叶片缺陷，没有整机正面视角的；而风机是刚性几何体，
合成图训练后在真实图上 mPose mAP50 = 0.99。

7 个关键点（顺序从 `generator.py:472-486` 确认）：

| 索引 | 名称 | 用途 |
|---|---|---|
| 0 | 机舱后部 | — |
| 1 | 塔顶 | 辅助估距 |
| 2 | 塔底 | 辅助估距 |
| **3** | **机舱前部 = 轮毂中心** | ★ 定位 |
| **4/5/6** | **三个叶尖** | ★ 编号与径向 |

**三片叶尖是无序的**：模型用排列不变损失训练（对 3 个点枚举 6 种排列取
误差最小者），因为三片叶片旋转对称、视觉上本就无法区分。
**编号由几何决定**，见下节。

### 编号为什么要靠几何

1 号叶片的物理定义是「相位角 = `park_phase` 的那片」。流程是：

```
叶尖像素 → 反投影到叶轮盘面 → 盘面内相位角 → 与 park_phase 匹配 → NO.1
```

**不能直接用图像极角编号** —— 实际拍摄总有透视。实测（正面特写但非完美
正对）三片叶尖的图像径向距离是 874 / 1069 / 848 px，差 22%，极角间隔因此
不均（偏差 24.7°）。反投影后径向散布归零。

`wt_test_ai_solve` 的斜视用例是这个结论的直接证据：

```
图像径向散布 29.5%  →  反投影后 0.00%
```

### 部署链路

```
PC:   .pt → .onnx          （ultralytics export，opset 12）
设备: .onnx → .engine      （trtexec --fp16）
```

**engine 必须在设备上构建** —— 它与 GPU 架构和 TensorRT 版本强绑定，
x86 上转的在 Orin 上加载会直接失败。

构建命令（实测约 13 分钟）：

```bash
/usr/src/tensorrt/bin/trtexec \
  --onnx=model-s.onnx --saveEngine=model-s.engine \
  --fp16 --workspace=2048
```

⚠️ **不要加 `--verbose`**：日志量巨大且拖慢构建；首次实测跑了 15 分钟还没完，
去掉后 13 分钟完成。

⚠️ **engine 别放 `/tmp`**：设备重启会清空。实测产物 22 MB，
重建要 13 分钟。放到 `~/wt-models/` 之类的持久位置。

### 推理封装（`app/wt_trt_pose.{h,cpp}`）

对外 C 接口，内部 C++（TensorRT API 是 C++ 的）。

实测性能（Orin NX，TensorRT 8.5.2，FP16，engine 22 MB）：

| 指标 | 值 |
|---|---|
| GPU 计算 | **8.6 ms/帧**（`trtexec` 中位） |
| 端到端 | **16.1 ms/帧**（含 letterbox + H2D/D2H + NMS） |
| 吞吐 | 110~114 QPS |

30fps 每帧预算 33.3 ms，占用约 48%。预处理（CPU letterbox）占 7.5 ms，
是端到端与纯推理之差的主要来源；上 GPU 能再省，但要引入 CUDA kernel，
当前不值得。

张量形状（`trtexec` 实测确认）：

```
输入  images   1 × 3 × 640 × 640   float32 NCHW
输出  output0  1 × 19 × 8400       float32
```

`19 = 4 (bbox) + 1 (conf) + 14 (7 关键点 × 2)`。

### 集成时三个必须注意的点

1. **帧回调线程不能做重活**。`DjiLiveview_ImageCallback` 是 SDK 回调线程，
   在里面推理会阻塞取流。只把帧指针塞进无锁队列，推理在独立线程消费；
   队列满则丢帧。
2. **letterbox 必须还原**。模型输入 640×640 是等比缩放 + 灰边填充得到的，
   关键点要从 640×640 坐标按 `orig = (pred - pad) / scale` 还原回原图。
   漏了这步关键点会系统性偏移，且偏移量随长宽比变化，很难察觉。
3. **关键点没有置信度**。`kpt_shape: [7,2]` 只有 x,y。下游要 conf 门槛时
   用框的置信度代替。

### 字段转换（`app/wt_ai_bridge.{h,c}`）

`WtAiBridge_FromTrtResult` 把 `WtTrtResult` 转成 `WtAiSolve_FromDetection` 的
输入：取 `items[0]`（conf 最高，NMS 已按 conf 降序排列），按关键点索引
3→hub、4/5/6→tip 搬运字段。只依赖 `wt_trt_pose.h` 的类型声明，不链接
TensorRT，所以能在桌面独立测试（`wt_test_ai_bridge`，含一个故意错位索引
验证测试会变红的变异测试）。

## 激光采集：实测盘面几何

> `-DWT_BUILD_TRT=ON` 开启（默认关）。**只能在设备上编译** ——
> 仓库级 CLAUDE.md 硬约束 3：绝不交叉编译。

### 这条链路解决什么

台账能给出轮毂中心与盘面朝向的**名义值**，但给不了两样：**停用角**
（叶片停在哪，每次停机都不同）和**盘面实际朝向**（机舱随风偏航）。
而这两样正是贴叶航线的安全前提 —— `WtRunner_PlanTurbine` 在停用角未知时
**无条件拒绝规划**。本链路用 4 次激光测距把它们测出来。

```
取流(wt_frame_pump) → 推理(wt_vision) → 稳定排序的 4 个像素
     ↓
逐目标：算指向(wt_aim) → 转云台 pitch(wt_laser) → 等稳 → 打激光
     ↓
4 个 WGS84 点 → 转塔基 ENU → 拟合(wt_rotor_fit) → WtRotorFrame
     ↓
喂给既有 WtPlan_BladeTrace，生成贴叶 KMZ
```

| 模块 | 位置 | 桌面可测 |
|---|---|---|
| `wt_rotor_fit` | `src/` + `include/`（wt_core） | ✅ `wt_test_rotor_fit` |
| `wt_aim` | `src/` + `include/`（wt_core） | ✅ `wt_test_aim` |
| `wt_frame_pump` | `app/` | 仅 PSDK，x86_64 链接自检可编 |
| `wt_vision` | `app/`（需 TRT） | — |
| `wt_laser` | `app/` | 仅 PSDK，x86_64 链接自检可编 |
| `wt_survey` | `app/`（编排） | — |

"容易算错且错了不报错"的两块（盘面拟合、像素→云台角）刻意放进 `wt_core`，
这条分界线与规划/安全校验用的是同一条。

### 云台两轴都控，但 pan 只有「相对机头 ±60°」

DJI 规格页在「Controllable Rotation Range」里把 M4T 的 Pan 标成
`Not controllable`，容易读成"pan 动不了"。实际不是：

- 同一页的「Yaw Axis」又写着 *Manual operation is uncontrollable /
  The MSDK interface program is controllable*
- Pan 机械范围 ±65°、软限位 ±60°，**不是锁死的**
- 姊妹项目 `liangzhourenwu` 已实测 **PSDK 能转 pan**（照准闭环在用）

所以真实约束是**范围**而不是**能力**：

| 轴 | 可达范围 |
|---|---|
| pitch | 绝对角 **−90° ~ +70°**（M4E 才是 +35°） |
| pan/yaw | **机头朝向 ±60°** |

⇒ 分工：**两轴都由程序闭环**，只有目标超出 pan 的 ±60° 时才请操作员转机头
（决策层报 `WT_SURVEY_ACT_NEED_BODY_YAW`）。程序**不操纵飞机**——那需要
飞控控制权，风险与收益不成比例。

### 三个会让云台"莫名不动"的坑

**① 绝对角模式下 `yaw` 绝不能填 0。**
0 的意思是"转到正北"，而 pan 只有相对机头 ±60° —— 机头不朝北时正北够不到，
**整条命令被拒**（`0x600000004 YAW_REACH_POSITIVE_LIMIT`），**连 pitch 都不转**
（还可能部分执行：先动了 pitch 才在 yaw 撞限位）。不想动 yaw 要表达成
**"yaw 保持当前值"**。官方样例 `test_gimbal_manager.c` 正是先读回当前 yaw 再原样传回。

**② 必须先判可达性再下发。** 否则"参数超限"会表现成"云台莫名不动"，
和"对不准"长得一样。`WtAim_IsReachable()` 做这件事，注意 pan 的相对角要
用 `Wt_AngleDiff` 折到 [−180,180]——直接相减在跨正北时会得到 350° 而误判超限。

**③ `FREE` 模式借了必须还成 `YAW_FOLLOW`。**
`FREE` 是"锁定地面姿态、忽略飞机运动"，静止观测需要它。但它残留在飞机上之后，
一旦机头连续转动（比如绕飞航线），pan 关节为保持地面姿态要反向补偿而它只有
±60° ⇒ 顶机械限位、电机持续给力 ⇒ 现场报「偏航顶限位 + 云台电机异常」。
姊妹项目为此踩过一次实打实的事故。PSDK 没有 `GetGimbalMode`，读不回原值，
所以只能还成 `YAW_FOLLOW`（M4T 的自然状态）。

另：`Rotate` 的 `time` 填 **0**（让云台自己定速）。Payload-SDK issue #563
实测「实际速度只有下发的 10–20%」，自己指定时长只会给出一个假的到位预期——
到位与否只能靠**读回角度**判断。

### 编号由采集顺序定义

模型输出的三个叶尖是**无序的**（排列不变损失训练）。所以：

1. `wt_vision` 存快照前按图像极角做**稳定排序**（`WtAim_SortTipsByImageAngle`）
2. `wt_survey` 按那个顺序依次采集
3. `wt_rotor_fit` 把**第一个采集点**的相位当作停用角

三者是同一个约定的三段，改任何一段都要同时改另外两段。不排序的后果是
操作员采的"NO.1"与程序记的"NO.1"对不上，**全程无报错**，最终停用角偏 120°。

### 三个反直觉的量

- **叶尖到轮毂的直线距离不是盘面内半径**。它是斜距，含锥角的轴向分量；
  盘面内半径 = `R·cos θ`。
- **三个叶尖所在平面不过轮毂中心**。锥角与预弯把叶尖拉向上游，偏移
  `R·sin θ + prebend`（80 m 叶轮、3° 锥角、2 m 预弯约 4.1 m）。
  这个量被**报出来**（`axialOffsetM`），不计入拟合残差。
- **只采到两个叶尖时法向误差约 11.6°，不是锥角的 3°**。闭式解
  `atan( c·|u₀−u₁| / (r·|sin Δφ|) )`，已由 `wt_test_rotor_fit` 按公式钉住。
  两叶结果只适合粗定位。

### 对激光海拔基准的免疫

激光返回的 `altitude` 基准没有文档承诺（姊妹项目实测与气压高差约 38 m）。
拟合只用**点与点之间的相对量**，整体平移被抵消 —— `wt_test_rotor_fit` 有一条
用例把 4 个点同时抬高 38 m，验证法向/半径/相位/间隔一字不变。

⚠️ 但**不要**拿激光海拔去和飞机 `position.alt` 相减。

### 上机验证到哪一步（2026-10-22）

已在本机 + 妙算3 上跑通并实测：

| 环节 | 结果 |
|---|---|
| 设备本地编译（`-DWT_BUILD_TRT=ON`） | ✓ 零 error/warning |
| aarch64 回归测试 | ✓ 11/11 |
| `.dpk` 安装 | ✓（顺带证明采集链路没破坏启动路径） |
| 取流 | ✓ 1440×1080 @30fps，4,665,600 字节/帧 |
| 推理 | ✓ 4.7 次/秒（配置 200ms 间隔） |
| 云台 pitch | ✓ 下发 −30/−45° → 读回偏差 **0.0°** |
| 云台 pan | ✓ 相对机头 ±20° → 读回偏差 **0.0°** |
| 激光 | 室内对墙 `exception=3`（打空，正常） |

⚠️ **云台话题只接受 50Hz**。用 10Hz 订阅返回 `0xE0`，而 SDK 内部 Debug
日志写着 `only support 'DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ'`。排查时先后
怀疑过"模块未初始化""话题编号不对""订阅机制坏了"，全不是——**订阅失败
先搜 SDK 日志的 Warn/Debug，别只信返回码**。

### 尚未完成

- **对准 → 打激光 → 拟合 这一段需要真实风机**。上述验证止步于
  "云台能对准、激光能读"，因为室内没有可测目标。
- **相机内参标定**：当前 `WtCameraGeometry` 由规格页等效焦距推得，未标定。
  这是反投影精度的瓶颈 —— 对 50 m 外目标，标定前后可能是米级 vs 分米级
- **现场标定**：`park_phase` 与图像角度的对应关系（`sign` / `offset`）。
  （注：激光采集链路跑通后，停用角可直接实测，这条标定只用于交叉校验）

### 编号的质量指标（`dev`）

`WtAiSolveResult.angleGapDevDeg` 是**相邻叶片角度间隔与理想值 360/n 的最大偏差**。
真实风机三片叶片等间隔 120°，所以：

| dev | 含义 | 处置 |
|---|---|---|
| 0–5° | 基本正对盘面 | 编号可信 |
| 5–15° | 轻微透视 | 可用 |
| 15–30° | 明显透视 | 需反投影修正 |
| >30° | 严重斜视 | 拒绝采信 |

实测（5 张公开图、29 个目标）：7 个 dev<5°，13 个 5–15°，6 个 15–30°，
3 个 >30°。**目前尚未在代码里设阈值**——调用方需自行判断。

### 许可

模型代码含 YOLOv11（**AGPL-3.0**）与 BlenderProc（GPLv3）。内部使用不受影响；
对外分发需评估 AGPL 传染性，或向 Ultralytics 购买商业许可。

---

## 高空行人瞄准打点

> 编译时 `-DWT_BUILD_TRT=ON`，配置 `ped_enabled = true`，在 Pilot 2 拨「行人瞄准」开关触发。
> 与激光采集（`survey_enabled`）、取流探针（`stream_probe`）**互斥** —— 都要独占
> 同一个相机源的图像流，推理线程也只能装一个模型。

识别画面里的行人，云台对准画面中心附近的那个人，**锁定后连续 3 帧确认就自动激光
打点**（连读 3 次取中位），得到这个人所在位置的 WGS84 坐标。这个坐标同时是
「绕点飞行」的基点。

```
取流 1440×1080 → 行人检测（VisDrone YOLO11s，1280 输入）→ 行人框
     ↓
决策（wt_ped_logic）：捕获离光轴最近的人 → 转云台 → 读回停稳 → 看停稳后的新一帧
                      → 还差就再转
     ↓
锁定：停稳后连续 3 帧在容差内（约 0.3 s）；可再加停顿 ped_dwell_ms（默认 0）
     ↓
激光连读 3 次（间隔 300 ms）取距离居中那次 → WGS84 落点
     → 日志 + data/ped_points.csv + Pilot 浮窗 + 绕点飞行的基点
```

### 模型

要的是**高空俯视下的行人检测**，不是 COCO 那类平视近景的人体模型 —— 后者在
俯视、十几像素的小人上基本检不出。

| 项 | 值 |
|---|---|
| 来源 | HF `dronefreak/visdrone-yolo11s`（DetectionBench），**AGPL-3.0** |
| 训练 | VisDrone2019-DET，YOLO11s，imgsz 1280，100 轮 |
| 指标 | 测试集 mAP50 43.6%（全部类别，模型卡数字，**未自测**） |
| 类别 | **11 类**（VisDrone 10 类 + `others`）；只收 0 `pedestrian`、1 `people`（坐着、骑车的人） |
| 张量 | 输入 1×3×1280×1280，输出 1×15×33600 |

⚠️ 类别数是导出时实测的（标准 VisDrone.yaml 是 10 类），所以推理封装从 engine
的输出形状反推类别数，不写死。换模型时**类别 0/1 必须仍是行人**（导出脚本会核对类别名）。

**为什么 1280 输入**：M4T 广角水平视场 71.6°，60 m 外的行人在 1440×1080 画面里
约 23×10 px，缩到 640 只剩约 10×4 px；这份权重本身也是 1280 下训练的。
推理开销约为 640 的 4 倍（87 GFLOPs），设备上的耗时**尚未实测**。

部署（engine 必须在设备上建，见「AI 识别 / 部署链路」）：

```bash
# 本机：导出并核对类别名与形状
python3 -I tools/ped_model/export_onnx.py <best.pt> --imgsz 1280

# 设备：别加 --verbose，engine 别放 /tmp
/usr/src/tensorrt/bin/trtexec --onnx=best.onnx \
  --saveEngine=visdrone-yolo11s-1280.engine --fp16 --workspace=2048
```

### 预处理与解码：桌面上已对齐 ultralytics（2026-10-24）

解码从 `wt_trt_pose.cpp` 拆进了纯 C 的 `app/wt_yolo_decode.c`，桌面可测。
`tools/ped_model/crosscheck_decode.py` 把它编成动态库，拿 ultralytics 自己的结果对照
（两张 VisDrone 原图）：

| 对照 | 结果 |
|---|---|
| 同一份输出张量：C 解码 vs ultralytics NMS | 20/20、46/46 人逐一对上，坐标差 0.0000 px，conf 差 0 |
| C 双线性 letterbox vs `cv2.resize(INTER_LINEAR)` | 最大差 0.625/255（OpenCV 定点误差） |
| C 全流程 vs ultralytics 完整预测 | 重合 20/22、44/46 |

**缩放方式对小目标有影响**：同一个 ONNX、1440×1080 的人群图上，最近邻检出 39 人、
双线性 45 人。所以行人模型用双线性；风机模型保持最近邻（已上机验证，没必要动）。

拆分时顺手补了一个缺口：原先把输出形状写死成 `19×8400`、从不和 engine 核对，
装错 engine 照样读出一片错位的数字。现在 Init 时读 engine 的输入输出形状并与
声明的布局核对，对不上就拒绝加载。

### 决策层（`wt_ped_logic`，桌面可测）

和激光采集的决策层有三处不同，都因为目标是人、会动：

1. **按绝对方向关联目标**，不按像素。云台一转，同一个人的像素位置全变了，
   他在世界里的方向不变。按像素关联会在转动后认错人。
2. **停稳靠云台读回判断，且只用停稳之后拍到的帧**。取流有延迟，转动中拍到的帧
   配上"现在"的云台角，算出的目标方向会偏向转动方向 —— 被当成"还差一点"
   继续转，形成过冲。
3. **锁定容差随目标大小**：框半尺寸（角度）× 0.5，夹在 [0.2°, 1.0°]。
   从高处斜看，打到行人脚边的地面与打在人身上，落点只差一两米 —— "行人位置"
   这个用途对它不敏感，下限不必压得极小。

**锁定后不再固定等 3 s**（2026-10-25 用户同意）。3 s 来自最初的需求描述，但只要求
打一次、拿到位置时，固定等待本身不提供证据；真正说明「对准了」的是停稳之后拍到的
几帧都落在容差内。老师若确实要停顿，配 `ped_dwell_ms = 3000`，确认帧与停顿两个
条件都满足才打。

| 参数 | 默认 | 说明 |
|---|---|---|
| 锁定确认 | 连续 3 帧 | 停稳后拍到、在容差内的新帧；中途一帧偏出容差（未脱锁）就从头数 |
| 额外停顿 | 0 ms | 配置 `ped_dwell_ms`；要「瞄准后 3 s 再打」就写 3000 |
| 激光读数 | 3 次取中位 | 间隔 300 ms（避开 5 Hz 限频）；离散度 > max(0.5 m, 1% × 距离) 判不可用 |
| 捕获置信度 | 0.40 | 配置 `ped_conf`；跟上之后不再要求（推理阈值 0.25） |
| 停稳 | 下发后 ≥ 400 ms 且读回静止 300 ms | 静止 = 角速度 < 1.5°/s |
| 关联门限 | 5° | 取方向最近的那个，门限外的人不顶替 |
| 跟丢 | 停稳后 2.5 s 没看到 | 回到搜索 |
| 脱锁 | 偏差 > 1.6 × 锁定容差 | 回到对准，确认帧与停顿都重新算 |
| 激光重试 | 没打成后 1 s、且重新攒满 3 帧再打，共 3 轮 | 超次数判失败 |
| 修正上限 | 12 次 | 追不上（人在快速移动）判失败 |
| 会话上限 | 120 s | 不会因为人走了永久挂住 |

超出云台 pan 范围（相对机头 ±60°）时报「请把机头转 N°」，不下发、不耗修正次数；
拿不到机头朝向时 yaw 回填当前云台值（**绝不填 0**，原因见「激光采集」一节）。
`wt_test_ped_logic` 用一个带取流延迟与云台响应延迟的小仿真把以上行为逐条钉住，
并逐条做过变异测试（改错实现，确认用例会变红）。

### 配置（`[app]` 段）

| 键 | 默认 | 说明 |
|---|---|---|
| `ped_enabled` | false | 启动取流与行人推理线程 |
| `ped_engine` | — | 设备上构建的 engine 路径 |
| `ped_dwell_ms` | 0 | 锁定确认后再停多久才打激光，范围 0~30000，越界拒绝启动 |
| `ped_conf` | 0.40 | 新捕获所需置信度，范围 (0, 1)，越界拒绝启动 |

### 结果

每次打点追加一行到 `<output_dir>/ped_points.csv`：时间、经纬度、激光海拔读数、
距离、激光落在画面的位置、云台与机头角度、目标置信度与框大小、偏差、变焦倍率、
激光有效读数次数与离散度（`laser_n`、`laser_spread_m`）。
⚠️ `wall_time` 取设备时钟，可能落后几十小时（RTC 无电池），按行序认先后；
激光海拔的基准没有文档承诺（见「激光采集」），只看相对量。

### 尚未上机验证 `[?]`

- engine 构建、1280 输入在 Orin NX 上的推理耗时
- `GetOpticalZoomParam` 的倍率语义（1× 是否就是广角 24 mm），以及 `M4T_VIS` 取流
  画面是否随变焦裁切。程序按读到的倍率换算视场 —— 若取流其实不随变焦，只会少转、
  多迭代几次，仍然收敛；反过来才会发散，所以拿不准时这样算是安全的一侧
- 取流延迟是否在 300 ms 停稳窗口之内
- 激光打在行人身上时的回波情况
- Pilot 2 上新开关的显示

### 已知限制

- **走动中的人仍然难锁**：确认只要约 0.3 s，但每次修正要等停稳与新帧（一两秒），
  人持续走动会在 12 次修正后判失败。要跟踪移动目标，需要先标定取流延迟、再做带
  预测的连续跟踪。
- 只做了可见光。夜间要换红外模型（如 HIT-UAV 高空红外数据集）与红外取流源。
- 推理线程只装一个模型，所以行人瞄准与激光采集不能同时开。

---

## 绕点飞行（以行人坐标为基点的正方形航线）

> 在 Pilot 2 按「绕点飞行」按钮触发。基点 = 本次运行里最近一次「行人瞄准」打到的点。

### 飞法（2026-10-25 用户需求）

以基点为几何中心建边长 20 m 的正方形。从第一个顶点开始：

```
V1 10 m → 垂直升到 20 m → 顺时针沿边平飞到 V2 → 垂直降到 10 m → 升回 20 m
       → 平飞到 V3 → 降 → 升 → 平飞到 V4 → 降 → 升 → 平飞回 V1 → 降到 10 m（结束，悬停）
```

共 13 个航点，高度都**相对起飞点**。平飞时机头始终朝向基点。

| 项 | 取值 | 依据 |
|---|---|---|
| 机头 | 每个航点 `towardPOI`，兴趣点 = 基点 | 规范：「从该航点飞向下一航点途中机头都将朝向兴趣点」。`smoothTransition` 在两点间线性插值角度，沿直线边飞时会偏离基点 |
| 航点类型 | `toPointAndStopWithDiscontinuityCurvature`（直线、到点停） | 升降才真的垂直；过点不停的模式会把拐角抹圆 |
| 高度模式 | `relativeToStartPoint` | 用户要「距起飞点」10 / 20 m |
| 结束动作 | `noAction` | 停在 V1 的 10 m 处悬停，由操作员接管 |
| 速度 | 航线 3 m/s，飞往首航点 5 m/s | 离人近，慢一点留反应时间；整圈 160 m、至少 53 s |
| 失控 | 返航，`globalRTHHeight` = max(30 m, 高点) | 返航高度比航线低的话，第一步就是在人旁边下降 |
| 云台 | 不下发动作（`gimbalPitchMode = manual`） | 俯仰由操作员用遥控器拨 |

**用户没指定、这里定下的两条**（核对页上明示，可改）：正方形的边沿正东西、
正南北；「第一个顶点」取**离飞机当前位置最近**的那个 —— 从飞机过去不必横穿
正方形，不会从行人头顶飞过。

### 起飞前的闸门（`app/main.c` 的 `WtApp_RunSquare`）

- 本次运行打到过点。**不读 `ped_points.csv`**：那里可能是昨天、别处的点。
- 飞机位置有效、**飞机在空中**：在地面按下会直接起飞到离人十来米的地方，拒绝。
- 云台没有残留在 FREE 模式：机头要连续转一整圈，FREE 会让 pan 顶到 ±60° 限位。
- 基点离飞机不超过 150 m：远到这个程度，多半是激光打到了别处。
- 按钮按下超过 5 s 才轮到处理（当时程序正忙）的请求作废：旧的按键不能事后把飞机飞起来。
- 任务状态回调注册失败就不 START：没有它就判断不了「进入航线了没有」。
- 「干跑」开着时只上传不启动 —— 可用来验证飞控是否接受这份 KMZ。

中止用 1 号「中止作业」按钮：下发 STOP，飞机退出航线、**原地悬停**。不像风机作业
那样返航：这条航线贴着人、高度只有 10–20 m，返航要先爬高再飞走，路线不受控。
任务状态判定在 `wt_mission_watch`（wt_core）：刚 START 时推来的 IDLE 不算结束，
要先见到执行状态再回到 IDLE 才算；30 s 见不到执行状态判为没启动起来；整趟 10 分钟上限。

### KMZ 结构：与风机航线不同（有意为之）

`WtBridge_ExportPedSquareKmz` 复用了 `wt_bridge` 的打包代码，但正文**分两份写**：
waylines.wpml 带 `executeHeightMode` / `waylineId` / `executeHeight`，template.kml 带
`templateType` / `waylineCoordinateSysParam` / `height`；补了必需元素 `globalRTHHeight`；
机型声明用 M4T 的 99 / 1、相机 89（与 `dji_typedef.h` 的 `DJI_AIRCRAFT_TYPE_M4T`、
`DJI_CAMERA_TYPE_M4T` 一致）。依据是真机证据：姊妹项目 liangzhourenwu 用这套结构
在同一台 M4T 上 START 成功并飞完 towardPOI 绕飞（2026-09-28）。风机航线仍是把同一份
混合正文写进两个文件、机型写 1100 —— 那份从没 START 成功过（模拟器报 `error_code: 4`
= 航线文件加载错误），风机侧暂缓，留待统一。

### 核对：先看网页再上机

```bash
cd wt_inspection
cmake --build build
# 导出一份 KMZ（参数：基点纬度 经度 飞机纬度 经度 输出）
./build/wt_ped_square_demo 41.5236 111.7461 41.52338 111.7458 build/sq.kmz
# 做成核对网页（@标签@飞机纬度,经度[,高度] 可省）
python3 -I tools/ped_square_view/build_view.py build/ped_square_view.html \
    "build/sq.kmz@飞机在西南@41.52338,111.7458,30"
```

网页解析的是 KMZ 里的 `waylines.wpml` 本身（飞机执行的那份），把用户的每一句要求
逐条对文件做测量（中心、边长、顺时针、垂直升降、首尾、朝向、到点停），并画出
俯视图、立体图和按 `towardPOI` 语义模拟的飞行动画。设备上生成的
`data/ped_square.kmz` 拉回来同样能看 —— 核对的就是飞机实际收到的文件。

`wt_test_ped_square`（几何 + KMZ 文本）与 `wt_test_mission_watch`（任务状态判定）
钉住以上行为，并逐条做过变异测试。

### 尚未上机验证 `[?]`

- 这份 KMZ 在 M4T 上能否 START（结构照搬飞通的写法，本文件本身没飞过）。可先开
  「干跑」只上传：拿到 `KMZ 已上传` 说明飞控收下了；START 才做内容校验
- 到首个航点时机头怎么对齐基点（`followBadArc` 应走最短方向）
- `currentWaypointIndex` 是从 0 还是从 1 数（浮窗里的「航点 k / 13」以此为准）
- 高度相对起飞点：行人脚下若比起飞点高，实际离地会低于 10 m，现场要目视确认

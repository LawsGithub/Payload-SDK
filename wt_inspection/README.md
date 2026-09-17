# 风机叶片无人机巡检（M4T + 妙算3）

基于 DJI Payload SDK 的风机叶片自主巡检方案实现。复刻 DJI 行业官网
[M4 系列风机叶片巡检方案](https://enterprise.dji.com/cn/news/detail/matrice-4-series-wind-turbine-inspection)
的**两阶段**技术路线（粗模环绕建模 → 精细巡检），执行主体为妙算3 机载计算。

**完整方案文档见 [`../docs/风机叶片巡检方案.md`](../docs/风机叶片巡检方案.md)** —— 含系统架构、
成像参数推导、安全设计原理、实测数据与部署检查单。

---

## 快速开始

### PC 侧：算法自检（无需 PSDK，秒级编译）

```bash
cmake -S . -B build && cmake --build build
ctest --test-dir build --output-on-failure     # 回归测试，秒级
./build/wt_plan_demo                          # 规划一条样例航线并打印全过程
./build/wt_plan_demo mission.csv mission.kmz  # 同时导出航点表与 PSDK 航线文件
```

`wt_plan_demo` 会输出：相机模型核验表、风轮参考系、采样密度推导、
航线概览、安全校验报告 —— 这些是**参数是否合理的直接依据**，
调参时应当反复看这张输出，而不是凭感觉改数字。

采样密度那一块直接调用 `WtPlan_ResolveBladeSampling()`，与规划器共用同一份
公式；文档 §4/§6 的数字也是从同一处取的。这套「数字只有一个来源」的约束
由 `tests/` 下的回归测试守住：`wt_test_plan` 核对规划结果，
`wt_test_config` 核对「配置文件里写下的参数必须真的生效」，
`wt_test_bridge` 核对「报告里的预计耗时与 KMZ 下发的速度同源」。

### 妙算3 目标：交叉编译机载应用

```bash
cmake -S . -B build-m3 -DWT_BUILD_PSDK_APP=ON \
      -DPSDK_ROOT=<Payload-SDK 根目录> \
      -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc
cmake --build build-m3        # 产出 bin/wt_inspection_app
```

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

此外，航线组装的最后会**统一扫描全部航段**并修补穿越叶片扫掠区的段落
（出壳 → 沿弧 → 换高，全程位于安全圆柱面之外）。实测中暴露的六类典型
危险航段见方案文档 §5.3。

---

## 已知限制

- 停用角未知时按「叶片停在 12 点方向」假设，程序会告警；但**只有
  `auto_start = true` 才会被硬性拒绝**，默认的 `false` 会照常起飞 ——
  报告里那一行必须人工确认（见方案文档 §8.1）
- 配置里的 `park_phase` 已能解析，但尚未接通到规划调用；地面站下发通道也未实现
- 转动模式下塔筒巡检段被整体跳过（塔筒必然在叶轮圆盘内，无法规避）
- 相位视觉反解只提供接口，**目前无人调用**，故闭环叶尖跟踪尚未真正生效；
  像素级换算需由视觉模块接入相机内参
- 未考虑风速下叶尖的动态摆幅，转动模式下 `min_safe_dist` 需按现场风速调大
- 配置解析是严格的：已知段落内出现模板之外的键名会拒绝启动（这是有意的，
  防止 `min_safe_distt` 这类拼写错误被静默丢弃）
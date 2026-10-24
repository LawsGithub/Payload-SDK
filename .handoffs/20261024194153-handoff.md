# HANDOFF — 读全文再开始干活

生成时间: 2026-10-24T00:52:00+08:00 · Git HEAD: `f9ce6cc`
信任规则: [V] = 交接时已用命令验证；[?] = 仅记忆未复核，当线索对待；[X] = 已证伪，别用。

## 0. 复核（下一会话先做）

- 锚点: `feature/wt-inspection` @ `f9ce6cc` (2026-10-24 00:52)
- 漂移检查: `git rev-parse HEAD~1` 应 = `f9ce6cc`——HEAD 必是本次 handoff 提交，
  其 parent 才是本快照记录的 SHA。
- 待重探的 `[?]`: 第 5 节 2 条
- 先读: `wt_inspection/README.md` §"AI 识别"（唯一说明来源）、`~/projects/CLAUDE.md`

## 1. 当前目标

**在妙算3 上做「整机识别 → 叶片编号 → 三维位置 → 航线规划」的闭环。**
完成定义 = 取流 → 推理 → 反投影 → 编号全链路在设备上跑通，
且能在真实风机上给出 NO.1/2/3 的位置。

## 2. 已验证状态 — 工作实际停在哪

### 几何链（PC，纯算法，27 个测试）

- `wt_blade_id` 角度 → NO.1/2/3，8 个用例 [V] `ctest` 6/6 通过
- `wt_projection` 像素 ↔ 三维（内参/姿态/射线求交），11 个用例 [V]
- `wt_ai_solve` 端到端，4 个用例 [V]
- **透视修正的直接证据** [V]：`wt_test_ai_solve` 斜视用例
  ```
  图像径向散布 29.5%  →  反投影后 0.00%
  ```

### PSDK 通道（设备实测）

- `DjiLiveview_StartImageStream` **30fps 1440×1080 RGB_PACKED** [V]
  446 ms 出首帧，30 秒 300 帧
- `DjiLiveview_SendAiMetaToPilot` 发送返回 SUCCESS，**但 Pilot 2 不渲染** [V]
  原因：Pilot 2 的 AI 识别与妙算3 应用互斥（用户实测弹窗）

### 模型与推理

- 开源模型 `model-s.pt` 19.8 MB（RWTH Aachen，YOLOv11-pose）[V]
- 7 个关键点语义从 `generator.py:472-486` 源码确认 [V]
  索引 3 = 轮毂，4/5/6 = 三叶尖（**无序**，排列不变损失训练）
- 真实图实测 **29/29 目标全部检出**（5 张公开图）[V]
- ONNX 导出 37.4 MB（opset 12）[V]
- **TensorRT engine 22 MB，设备上构建成功** [V]
  ```
  GPU 计算 8.6 ms/帧 · 端到端 16.1 ms · 110~114 QPS（Orin NX，FP16）
  ```
- `wt_trt_pose.{h,cpp}` 推理封装 [V] 设备上 g++ 编译通过、链接运行成功

### 设备环境

- 妙算3 = **NVIDIA Orin NX**，L4T R35.5.0 [V]
- 预装 CUDA 11.4 / TensorRT 8.5.2 / cuDNN 8 [V]
- `nvcc` 与 `trtexec` 不在 PATH：`/usr/local/cuda-11.4/bin/nvcc`、
  `/usr/src/tensorrt/bin/trtexec` [V]
- engine 已存到 `~/wt-models/`（不在 `/tmp`）[V]

### Blender / BlenderProc（本会话最后在做）

- Blender **4.2.1 LTS** 已装到 `~/blender/blender-4.2.1-linux-x64/` [V]
  `--version` 正常；补装了 `libsm6`/`libice6`
- BlenderProc 2.8.0 pip 装好 [V]
- **Blender 内置 Python 的依赖已装齐**（用阿里云镜像）[V]
  ```
  Successfully installed imageio-2.38.1 lazy-loader-0.6 networkx-3.6.1
  numpy-2.4.6 packaging-26.3 pillow-13.0.0 scikit-image-0.23.2
  scipy-1.17.1 tifffile-2026.3.3
  ```
- **`blenderproc run` 仍未确认成功** [?]：最后一次跑 EXIT=124（超时），
  日志被 pip 进度条淹没，尾部有 `/dev/shm/...` 清理报错。
  尚未确认是启动成功还是卡在别处

### 工作区

- 干净（除 `WEEKLY-2026-10-20.md` 是用户自己的文件，未跟踪，不要动）[V]

### 测试输出（本次交接 run 的真实输出）

```
$ cd /tmp/wt-bid && ctest
100% tests passed, 0 tests failed out of 6
Total Test time (real) =   0.05 sec
```

## 3. 决策与理由

- **编号由几何决定，不由视觉** [V]——模型用排列不变损失训练，三叶尖输出无序；
  三片叶片旋转对称，视觉上本就无法区分。按 `park_phase` 匹配才有物理意义。
- **演示脚本改用纯几何编号**（最小角度 = NO.1）[V]——用户明确要求
  「东为 0°，逆时针增大，以最小角度为基准」。这版不依赖 `park_phase`，
  同一张图永远得到同一组编号。否决方案：按 park 匹配，否决原因是
  演示里没有真实停用角，假设值会让结果看起来对但语义错。
- **`dev` 当质量指标而非错误** [V]——它是相邻角度间隔与理想值 360/n 的偏差，
  纯几何量，不受 `park`/`offset` 影响。实测 29 个目标：7 个 <5°、13 个 5–15°、
  6 个 15–30°、3 个 >30°。
- **预处理留在 CPU** [V]——letterbox 约 7.5 ms，上 GPU 能再省但要引 CUDA
  kernel，当前 16.1 ms 端到端已够 30fps，不值得。
- **NMS 自己实现** [V]——不依赖 `opencv_dnn`，设备上 OpenCV 4.2/4.5 并存，
  版本不统一。

## 4. 失败的尝试 — 不要再试

- **交叉编译**（`aarch64-linux-gnu-gcc` 在 WSL）→ 产物要求 `GLIBC_2.34`，
  设备只有 2.31，`ldd` 报 not found [V]。不要再试。
- **拿官方样例源码做对照实验**（凭据未填）→ `main.c:437 Please fill in
  correct user information`，变量没控住 [V]。
- **在 `/tmp` 下打包/编译** → 设备重启后 `/tmp` 被清空 [V]。
- **`tar --mtime='@0'` 直接解包后 make** → make 跳过重编，跑旧二进制 [V]。
  解包后要 `touch` 源文件。
- **`timeout 15 ./app` 想杀掉卡住的进程** → 不生效，进程活 9 小时并占住
  通道 [V]。要 `kill -9`。
- **`pgrep -x wt_inspection_app` 判断残留进程** → 永远匹配不到：Linux 进程名
  上限 15 字符，实际是 `wt_inspection_a` [V]。查谁打开了 `/dev/uv-ffs/bulk*`。
- **`pkill -f wt_inspection_app`** → 会匹配到执行它的 SSH 命令自身 [V]。
- **`dji_app_ctl stop` 后立刻 `start`** → 通道未释放 [V]。stop 后等 10 秒。
- **用 `strings` 查二进制里有没有中文串** → 默认只抓 ASCII [V]。用 `md5sum`。
- **在模拟器里验证 `DjiWaypointV3_Action(START)`** → `error_code: 4` [V]。
- **`trtexec --verbose` 构建 engine** → 日志量巨大且拖慢，首次跑 15 分钟没完 [V]。
  去掉后 13 分钟完成。
- **SSH 前台跑长任务**（`trtexec` 15 分钟）→ SSH 断开后输出丢失，
  但进程还在跑 [V]。用 `nohup` + 输出重定向到设备文件。
- **`pip install torch` 走 PyPI 直连** → 下载卡死（速度 0 B/s）[V]。
  最终用 `pypi.org` 装 torch 成功（耗时很长），但 ultralytics 直连失败，
  改用**阿里云镜像** `https://mirrors.aliyun.com/pypi/simple` 才通 [V]。
- **清华 PyPI 镜像装 torch** → `No matching distribution found` [V]。
  阿里云/腾讯镜像查 ultralytics 正常，清华不行。
- **`jadx --single-class` 反编译 Pilot 2** → `Input class not found` [V]。
  120 MB DEX 上该选项不工作，只能用全量反编译。
- **Pilot 2 的 `classes.dex` 静态分析** → 被 **AppGuard 加固** [V]。
  DEX 头 `class_defs=1684`、`data_size=1.5 MB` 但文件 120 MB，
  真实业务代码在加密段。`com.dji.drone.*` 读不到。
- **`DjiLiveview_SendAiMetaToPilot` 画框给操作员看** → Pilot 2 不渲染 [V]。
  与 AI 识别模式互斥。识别结果自己用，不要指望显示。
- **新建 `doc/` 下的方案文档** → 违反项目 CLAUDE.md「README 是唯一说明来源」，
  已删除 [V]。

## 5. 已知坑

- **PSDK liveview 调用前必须 `DjiLiveview_Init()`** [V]——漏掉时
  `RegUserAiTargetLableList` 返回 `0xE0` + 日志
  `Ai target Lable only support on manifold3`，看起来像硬件不支持。
  见 memory `psdk-liveview-must-init-first`。
- **Pilot 2 的 AI 识别与妙算3 应用互斥** [V]——见 memory
  `pilot2-ai-vs-psdk-app-mutually-exclusive`。
- **安装器两条失败路径** [V]：① 应用提前退出 → 误导性的
  `verify app user_app_id or version info error`；② 长时间阻塞 → 安装超时。
- **安装超时且日志 0 字节 = `DjiCore_Init` 之前就失败** [V]——最常见是
  通道被占（`Smart3DExplore` 或自己的残留进程）。
- **`dji_app_ctl stop` 报 SUCCESS 但进程仍在** [V]——要执行两次 + 复查。
- **`.dpk` 不解决 `/data` 写权限** [V]——落点是包内 `data/`。
- **设备 RTC 无电池** [V]——掉电回 1970，`tar` 报时间戳在未来。
- **设备 cmake 3.16.3 不支持 `ctest --test-dir`** [V]——用 `cd build && ctest`。
- **设备 SSH 不稳** [V]——USB 网络共享链路丢包 33%、RTT 最高 1.2 s，
  经常 `Connection timed out`，重试即可。
- **`blenderproc run` 的输出被 pip 进度条淹没** [?]——日志 400 KB 全是
  `\r` 刷新的进度条，`tail` 拿不到有效信息。要看结果得先过滤 `\r`。
- **`/dev/shm` 清理报错** [?]——`blenderproc` 退出时
  `FileNotFoundError: /dev/shm/blender_proc_xxx`，是否影响功能未验证。
- **Pilot 2 的 `cloud_config_RC701.json` 权限 `root:root 600`** [V]——非 root 读不了。
- **`template.kml` 与 `waylines.wpml` 内容完全相同** [?]（继承，未重验）。
  PSDK 上传不受影响，但导入 Pilot 2 统计栏显示 0 m / 0 航点。**未修**。

## 6. 下一步（有序）

1. **确认 `blenderproc run` 是否真的能跑**——上次 EXIT=124（超时），
   用 `tr '\r' '\n'` 过滤日志后再看。若卡住，先查是不是首次启动初始化。
2. **用合成生成器造正对盘面数据**（用户要求的下一步）：改
   `gen_config.yaml` 的 `ANGLE_BOUND: 2`，固定 `ROTOR_ROTATION`，
   生成带真值的数据集。生成器在 `~/projects/refs/wt-synth/`。
3. **把 `WtTrtPose` 输出接给 `WtAiSolve_FromDetection`**——两者语义一致，
   只差字段转换（`WtTrtDetection` → `WtAiDetection`）。
4. **与取流回调对接**：帧指针塞无锁队列，独立推理线程消费，队列满丢帧。
5. **给 `angleGapDevDeg` 加采信阈值**（README 已写建议值 15°），
   目前只计算不判断。

## 7. 留给用户的开放问题

- **Blender 装 WSL 已成功，但 `blenderproc run` 是否正常未确认**。
  如果一直卡住，可能需要改用 Windows 侧 Blender 或调 `/dev/shm` 大小。
- **相机内参标定**：当前 `WtCameraGeometry` 来自规格页等效焦距，未标定。
  这是反投影精度的瓶颈——对 50 m 外目标可能是米级 vs 分米级。
- **现场标定 `park_phase` 与图像角度的 `sign`/`offset`**：方法已写进 README，
  但需要真实风机配合。
- **多台风机作业**：设备上只配了 `WT-A01` 一台。
- **`/open_app/wt-inspection/data/` 里 liangzhourenwu 的遗留产物**——
  已备份到 `/home/dji/lz-salvage/`，原位置三份是否移走未定。

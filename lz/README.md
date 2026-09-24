# liangzhourenwu —— 国旗杆绕飞

妙算3 机载应用：**识别国旗杆 → 生成绕飞航线 → 上传航点执行**，
操作员通过 Pilot 2 控件控制。

## 分层

```text
lz_core      零依赖（不依赖 PSDK / 任何第三方）        ← 永远可编译可测试
  ├─ lz_types   基础类型（LzGeo / LzStatus）
  ├─ lz_target  视觉与规划之间的唯一契约（LzTarget = 一根杆）
  ├─ lz_geo     大地几何（距离/方位角/推算坐标）
  ├─ lz_plan    绕飞航线生成 + 安全校验          ← 全部实现
  ├─ lz_wpml    wpml XML 生成（template.kml + waylines.wpml）
  ├─ lz_kmz     zip 打包（store 模式，手写 251 行，不引 zlib）
  └─ lz_bridge  规划结果 → KMZ / CSV

lz_vision    视觉层，两个**互斥**后端（-DLZ_VISION_BACKEND=stub|hsv，默认 stub）
  ├─ src/lz_vision.c    **真算法，已实现**（HSV 阈值 + 连通域 + 竖线对比度滤波）
  └─ src/lz_vision_stub.c  固定杆位占位
lz_app       机载应用，依赖 PSDK（-DLZ_BUILD_PSDK_APP=ON）← 默认关闭
  ├─ platform/      平台层注册（移植自官方样例，逻辑未改）
  ├─ lz_pole_source 绕飞圆心从哪来（固定坐标 | 激光测距）
  ├─ lz_widget      Pilot 2 控件（操作员入口）
  └─ lz_mission     作业状态机（**所有决策集中在此**）
```

**视觉刻意不用 OpenCV**：国旗是高饱和红色块，HSV 双区间阈值足够。
好处是 WSL 与妙算3 都不必装 OpenCV，且视觉层能在桌面上直接跑测试。

**视觉后端为什么默认 `stub`**：`hsv` 后端已实现，但**还没接进主链路**
（取图那一环空着，见 CLAUDE.md）。默认 stub 是为了让「拨开关 → 飞机绕一圈」
这条链路先能整体跑通。

**`hsv` 后端怎么验证**（6 张真实俯拍照片的回归测试，约 1.6 MB 数据入库）：

```bash
cmake -S . -B build -DLZ_VISION_BACKEND=hsv && cmake --build build -j4
./build/lz_test_vision        # 6 张照片 + 8 个人造用例
node tools/render_vision_check.js tests/data /tmp/check.png   # 人工核对图
```

⚠️ 检测的是**杆**（`LzTarget.pixel.u` = 杆列），**不是旗面中心** ——
实测两者相差 −3.7% ~ +4.1% 画面宽且**随风向变号**，标定不掉。详见 CLAUDE.md。

## 桌面自检（秒级，零依赖）

```bash
cd lz
cmake -S . -B build && cmake --build build -j4
ctest --test-dir build --output-on-failure     # 7 个测试（hsv 后端下 8 个）
./build/lz_plan_demo                            # 打印一条绕飞航线
./build/lz_kmz_demo /tmp/out.kmz                # 生成 KMZ 供解包器校验
```

**独立验证**（不要只信自己写的检查）：

```bash
python3 -c "import zipfile;z=zipfile.ZipFile('/tmp/out.kmz');print(z.testzip())"
# → None（CRC 全通过），包内 wpmz/template.kml + wpmz/waylines.wpml
```

## 设备上编译与打包

```bash
# 本机：打包构建必需部分
tar czf /tmp/lzbuild.tar.gz --mtime="$(date -d '+30 seconds' '+%Y-%m-%d %H:%M:%S')" \
     lz psdk_lib/include psdk_lib/lib/aarch64-linux-gnu-gcc \
     samples/sample_c/platform/linux/{manifold3,common}

# 设备：解包后（时间戳必须是设备当前时间附近，见 CLAUDE.md 的坑）
cmake -S lz -B build-native -DLZ_BUILD_PSDK_APP=ON -DPSDK_ROOT=<包根>
make -C build-native -j4

# 打包安装
tools/build_dpk/build_dpk.sh lz/app_json
dji_app_ctl install <包名>.dpk
```

编译选项：

| 选项 | 默认 | 作用 |
|---|---|---|
| `LZ_BUILD_PSDK_APP` | OFF | 编译机载应用 |
| `LZ_TARGET_ARCH` | aarch64 | `aarch64`（妙算3）/ `x86_64`（PC 链接自检） |
| `LZ_VISION_BACKEND` | stub | `stub`（固定杆位）/ `hsv`（真算法） |
| `LZ_POLE_SOURCE_LASER` | OFF | 绕飞圆心取自激光测距而非固定坐标 |
| `LZ_BUILD_DESKTOP_TOOLS` | ON | 桌面演示工具（设备上 tools/ 没传时关掉） |

## 凭据

`lz/lz_credentials.ini`（**未跟踪**）由 `cmake/gen_app_info.cmake` 在构建期生成
`dji_sdk_app_info.h` 到构建目录，遮蔽官方样例的同名文件 —— 明文密钥因此不进源码树。

**换 App 时必须同时改两处**：`lz_credentials.ini` 与 `app_json/app.json` 的
`user_app_id`。机制与理由详见 `cmake/gen_app_info.cmake` 头部注释与 `CLAUDE.md`。

## API 怎么查

不要凭记忆写 PSDK API。查 `~/projects/.psdk-apiref/`，协议见其
`LEARNING-PROTOCOL.md`（含豁免规则：**函数签名一律 grep**）。
本项目专用的路由与踩坑见 `.psdk-apiref/liangzhourenwu/API-MAP.md`。
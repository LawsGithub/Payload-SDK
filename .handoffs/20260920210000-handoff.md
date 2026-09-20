# HANDOFF — 读全文再开始干活

生成时间: 2026-09-20T11:50:53+0800 · Git HEAD: de05a46
信任规则: [V] = 交接时已用命令验证；[?] = 仅记忆未复核，当线索对待；[X] = 已证伪，别用。

## 0. 复核（下一会话先做）

- 锚点: `feature/liangzhourenwu` @ `de05a46`（2026-09-20 11:50）
- 漂移检查: `git rev-parse HEAD~1` 应 = `de05a46` —— HEAD 是本次 handoff 提交，其 parent 才是快照锚点；变了说明快照可能过期
- 先读: `~/projects/liangzhourenwu/CLAUDE.md`（项目约定）+ `~/projects/.psdk-apiref/liangzhourenwu/API-MAP.md`（API 路由与实测记录）
- 设备: `sshpass -p 'dji' ssh dji@192.168.42.120`（`[V]` 通；链路刚建立时 ping 会假性丢包，重试一次再下结论）

## 1. 当前目标

**让「操作员在 Pilot 2 拨开关 → 飞机绕国旗杆飞一圈」这条链路完整可用。**
代码已全部就位，只差现场参数确认。完成定义：拨 ON 后飞机按设定半径高度绕杆一周并返航。

## 2. 已验证状态 — 工作实际停在哪

**全部代码已提交并推送** `[V]`（见下方三个 SHA）。工作区干净，无未提交改动。

- **绕飞算法完整**：8 点圆、半径误差 0.000000 m、云台逐点指向杆心 8/8 一致 `[V]`
- **绕行方向经双重独立验证**：测试逐点断言方位角 + 独立程序用鞋带公式算有向面积，`clockwise=true` 得 −1131.37 m²（= 顺时针）`[V]`
- **KMZ 生成经独立解包器验证**：Python `zipfile.testzip()` 返回 None（CRC 全通过）`[V]`
- **激光测距已实测可用**：位置 1，`enable_lidar=1`；移动飞机时 `distance` 在 0.8–2.1 m 间实时变化 `[V]`
- **控件模块已实现未上机**：三个控件（绕飞开关/半径/高度）配置与回调都在，[?] 能否在 Pilot 2 显示未验证

### 测试/build 输出（本次交接从全新 shell 跑的真实输出）

```
cmake 配置退出码: 0
cmake 构建退出码: 0    告警/错误行数: 0
ctest 退出码: 0        100% tests passed, 0 tests failed out of 4
  lz_test_geo     34 项检查，0 项失败
  lz_test_plan    74 项检查，0 项失败
  lz_test_bridge  12 项检查，0 项失败
  lz_test_kmz     22 项检查，0 项失败

四个构建组合全部 0 问题：
  PC 侧 stub 后端 / PC 侧 hsv 后端 / 机载 x86_64 固定杆位 / 机载 x86_64 激光杆位
产物：build/bin/{lz_app, lz_rangefinder_probe}
```

## 3. 决策与理由

- **绕飞走 Waypoint V3（自建 KMZ）而非 V2** `[V]` —— 官方文档明文 V2「仅支持 M300 RTK 和 M350 RTK」，M4T 不在内。否决方案：V2 结构体直传（机型不支持）。
- **KMZ 里用 `gimbalRotate` + `gimbalYawRotateEnable` + `absoluteAngle`** `[V]` —— 那是绕飞的核心。**推翻了上一会话"KMZ 对第三方负载没意义"的判断**：当时只看了 `takePhoto` 一个动作实例就下了全局结论，而 `gimbalRotate` 是独立于相机的动作。`takePhoto`/`fileSuffix` 那些相机语义参数我们不填。
- **视觉用纯 C 不用 OpenCV** `[V]` —— 设备上**确实有** OpenCV 4.2（含 dnn），但结论不变：纯 C 能桌面测、零依赖、**失败可解释**（阴天→调阈值），模型失效是不可预测的自信错误。否决方案：训练模型（需要先拍旗杆才能标注，而拍旗杆需要先有这套系统 —— 前置依赖死循环）。
- **控件回调只记状态、不做动作** `[V]` —— 回调在 PSDK 工作线程上，上传 KMZ 会阻塞它。所有作业决策集中在 `LzMission_Tick()`。否决方案：回调里直接开飞（阻塞 + 决策散落各处难审查）。
- **拨 OFF 走 STOP 而非 PAUSE** `[V]` —— 操作员想停通常是发现了异常，要的是"停下来"不是"悬停等决定"。
- **视觉后端默认 `stub` 占位而非 `hsv`** `[V]` —— 激光测距可用后，视觉职责可能从"解算绝对坐标"降级为"确认瞄准点是不是杆"，那样用不着连通域。**职责定下来之前做深视觉是白做。**

## 4. 失败的尝试 — 不要再试

- **`tar --mtime='@0'` 传源码** `[V]` —— 时间戳压到 1970 后 `make` 发现**源码比 `.o` 旧**，**静默跳过编译**。现象是"传了新代码上去，跑的还是旧二进制"。改用 `--mtime="$(date -d '+30 seconds' ...)"`。**仓库级 CLAUDE.md 那条"必须用 `--mtime='@0'`"在这里反而成了陷阱。**
- **在 `lz_widget.c` 注释里写 `widget_file/*/`** `[V]` —— `*/` 提前闭合了块注释，后面整段代码变语法垃圾，报错位置与病因完全对不上。
- **探针用 `samples <= 1` 判断单次模式** `[V]` —— `-1`（连续监视哨兵）满足 `<= 1`，掉进单次分支。**负值哨兵与正数阈值共用变量时判断顺序决定成败。**
- **`DjiWidget_DeInit()`** `[X]` —— 该函数**不存在**。核实过 `dji_widget.h` 全部 9 个导出函数，只有 `Init`。别再找它。
- **"KMZ 的动作对第三方负载没意义"** `[X]` —— 已证伪，见 §3。
- **Waypoint V2 用于 M4T** `[X]` —— 官方明文只支持 M300/M350。
- **兴趣点环绕（`DjiInterestPoint_*`）可控半径** `[X]` —— `T_DjiInterestPointSettings` 里没有 radius/height 字段，半径由飞机按"当前位置到兴趣点的距离"自动定。
- **交叉编译** `[V]` —— 产物要求 `GLIBC_2.34`，设备 glibc 是 2.31，`ldd` 报 not found。设备本机编译只要求 `GLIBC_2.17` `[V]`。

## 5. 已知坑

- **官方文档没写 `exception` 取值** `[V]` —— 头文件与中英文档都只有"异常标志"四字。实测反推：**1 = 无回波，3 = 正常读数，2 = 过渡态**。取坐标前必须判 `exception`，否则会把无回波时的 `0,0` 当成真坐标。
- **`distance` 单位是 0.1 m** `[V]` —— 分辨率 0.1 m，所以读数恒定**不代表**没工作。判"测距是否工作"要看读数是否随目标移动而变化。
- **方位角不能用绝对差比较** `[V]` —— 正北处球面计算返回 `359.999999998°`，与 `0°` 绝对差是 360。角量是圆周量，容差调多大都没用。测试用 `LZ_CHECK_ANGLE_NEAR`。
- **`LzGeo_NormalizeDeg` 区间是 [0,360) 左闭右开** `[V]` —— 极小负数加 360 会因舍入恰好得 `360.0`，函数里已收回，别删那个判断。
- **管道下 stdio 是全缓冲** `[V]` —— 实时监视类程序必须每行 `fflush(stdout)`，否则攒够 4 KB 才可见。
- **`/blackbox/system/app_temp_files/` 是 root:root 0755** `[V]` —— `dji` 用户无写权限且没有 sudo，里面的日志删不掉。且该目录**混有别的项目的日志**（如 `matrice4t-square-mission_*.log`），不是我们的地盘。
- **本 worktree 是稀疏检出** `[V]` —— `lz` 目录是在 `git sparse-checkout add lz` 之后才能 `git add` 的。**新建 worktree 后第一件事是 `git sparse-checkout add lz`**，否则 `git add lz/...` 会静默失败（报 "outside of your sparse-checkout definition"）。

## 6. 下一步（有序）

1. **上机跑控件验证**：设备上编译后装 dpk，在 Pilot 2 里确认三个控件显示、拨动触发回调。
   - 前置：让出 PSDK 通道 `pgrep -x Smart3DExplore >/dev/null || /system/bin/dji_app_ctl stop Smart3DExplore`
   - 前置：飞机通电并连接（`.dpk` 安装器会试运行应用并要求走完身份校验）
2. **室外验证杆位来源**：用 `-DLZ_POLE_SOURCE_LASER=ON` 编译，确认激光取到的 `lat/lon` 是飞机所在位置（室内无 GPS 恒为 0，这条只能在室外验）。
3. **定视觉层的职责**：若第 2 步成立（激光能打中杆），视觉层只需"确认瞄准点是不是杆"—— 那 `lz_vision` 的连通域可以不实现，改用 `screenX/screenY` 与红色像素占比的简单判据。
4. **补 `lz_app_config.h` 的实际使用**：该文件目前只有类型定义，`lz_mission.c` 直接从控件取值，没有读配置文件。若要支持"不改代码调参数"，把半径/高度/速度落到 yaml/ini。

## 7. 留给用户的开放问题

- **运动规划是否需要额外申请 PSDK 高级权限？** 能力矩阵里 M4 系标注"高级功能需要用妙算3"（本配置满足），但个别功能另有"需要申请"的标注，运动规划是否在内**未查到确证**。这是上机前唯一无法从文档确定的事。
- **`droneEnumValue=99` / `payloadEnumValue=89` 是否被 M4T 接受？** KMZ 里写的是这两个值（对照 `dji_typedef.h` 的 `DJI_AIRCRAFT_TYPE_M4T` / `DJI_CAMERA_TYPE_M4T`），但没有实测。
- **室外调试通道怎么解决？** 妙算3 本体无内置无线（只有 `rndis0` USB 网卡），但**内核带了 103 个无线驱动模块**、`dnsmasq`/`nmcli`/`wpa_supplicant` 齐全 —— 插一个驱动已覆盖的 USB WiFi dongle 接手机热点是最现实的路径 `[?]` 未实测。
- **杆的 WGS84 坐标最终用哪条路？** 激光（现成、精度高，但需打中杆）vs 视觉定位（算法未做）。这决定了 `lz_vision` 要做多深。
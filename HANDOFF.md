# HANDOFF — 读全文再开始干活

生成时间: 2026-10-12T19:40+0800 · Git HEAD: `efeddd0`
信任规则: [V] = 交接时已用命令验证；[?] = 仅记忆未复核，当线索对待；[X] = 已证伪，别用。

## 0. 复核（下一会话先做）

- 锚点: `feature/liangzhourenwu` @ `efeddd0`（2026-10-12 19:40）
- 漂移检查: **用 `git log --oneline -5` 的实际输出比对**，别写死 `HEAD~N`
  —— handoff 之后可能又落了若干提交，写死的判据会静默失效（已踩过两次）。
- ⚠️ **`efeddd0` 这个提交可能还没推到远端** `[?]` —— 交接时推送连续失败
  （分类器超时，非网络问题）。先 `git status --short --branch` 看 `ahead N`。
  推送**必须走代理**，直连挂起：
  ```bash
  GIT_SSH_COMMAND="ssh -o ProxyCommand='nc -X connect -x 127.0.0.1:7890 %h %p'" \
    git push fork feature/liangzhourenwu
  ```
- **设备不在线** `[?]`（10-08 实测，本次未探）—— 按用户要求**初始化时不主动探设备**，
  只在真要上机时才探。判据 `ping` + `ip neigh`，**不用 `/dev/tcp` 扫网段**。
- 先读: [`lz/doc/ONDEVICE-CHECKLIST.md`](lz/doc/ONDEVICE-CHECKLIST.md)
  （现场操作单，**待确认项的唯一真值**）+ `CLAUDE.md`（已瘦身，含文档导航）。

## 1. 当前目标

**主目标未变**：把 2026-10-01 现场报的「绕飞时云台偏航顶限位 + 云台电机异常」
修掉（代码 `1fb4cd4` 已改完、桌面已验），**完成定义 = 上机按清单 §3.13 走一遍，
绕飞全程不再出现那两条告警**。

⚠️ **但设备侧编译打包从未做过，设备也不在线** ⇒ 主目标**卡在设备上**。
本会话（2026-10-12）全程在设备离线状态下工作，做的是**代码与文档**。

## 2. 已验证状态 — 工作实际停在哪

### 本会话做了什么

**① 清单可信化**（`4323385`）—— 3 个章节标题写「已验通」而勾选框全空，
两种读法都错。按"该动作本身已上机做过且有证据"重打勾：
`49 未 / 1 已` → **`30 未 / 20 已`**。顺带修 4 处与实测矛盾的陈述
（见 §3 决策）。

**② 云台俯仰改为瞄杆顶**（`9301986`）—— 用户指定。见 §3。

**③ 修三处过期文案**（`41a13fe`）—— ② 漏改的面向操作员的文案。

**④ 持久文档**（`efeddd0`）—— 删易变待办索引，新增俯仰瞄杆顶约定。

### 桌面验证 `[V]`（本会话实跑，真实退出码）

```
hsv  后端: ctest --test-dir lz/build       → 12/12 passed, rc=0
stub 后端: ctest --test-dir lz/build-stub  → 11/11 passed, rc=0
lz_test_plan                              → 1146 项检查，0 项失败
x64 PSDK 侧编译 (build-x64)                → 0 error 0 warning
bash lz/tools/precheck.sh                  → rc=0，全部通过
  3b ASan+UBSan ✓ / 3c aarch64 交叉编译 ✓ / 3d 覆盖率最低 87.10% ✓
```

⚠️ **目标数是 hsv 12 / stub 11**（stub 不注册 `lz_test_vision`）。**别按 11/10 记。**

### ⚠️ 设备侧编译打包：**从未做过**

`1fb4cd4` + 本会话的新改动**从未在 aarch64 上编译过**（precheck 3c 在本机
交叉编译过，风险比之前小，**但不等于设备上编得过**）。
设备上那个旧 dpk 落后得比想象中多 —— **别直接装旧的**。

### git 状态（事实快照，非待办）

```
feature/liangzhourenwu @ efeddd0
本会话四个提交：4323385 → 9301986 → 41a13fe → efeddd0
⚠️ efeddd0 可能未推送（见 §0）
```

## 3. 决策与理由

- **俯仰瞄杆顶而非中点** `[V]`（用户 2026-10-12 指定）——
  `pitch = -atan2(alt − h, r)`（原为 `alt − h×0.5`）。
  理由：现场要的是**红旗落在画面中心**，而旗挂在杆顶；旧实现瞄中点，
  预设了"观测对象是整根杆"。差别在低高度 + 大半径时最大
  （20 m/20 m/杆高 15 m：−32.0° vs −14.0°，**差 18°**）。
  **代价已确认接受**：低高度 + 大半径时杆身掉出画面下缘。
- **`targetHeightM` 仍是"杆高"**，来源是"激光海拔 − 起飞点海拔"。
  ⇒ **打杆顶得杆高 ✅ / 打旗面低 1.5 m ⚠️ / 打地面得 0 ❌**。
  误差**随飞行高度放大**：40 m 时 1.6°（32 px，可忽略），
  **20 m 时 5.7°（111 px，肉眼可见）**。用户工况是 20–40 m
  ⇒ **低空务必打杆顶**。浮窗会报「目标高 X m」，那是操作员唯一判据。
- **几何公式瞄杆顶、视觉照准仍瞄旗面框中心** —— 两者**自 2026-10-12 起
  不再指向同一个物理点**，`VISION-GIMBAL-PITCH.md` 里"互相校验"那条
  **暂时不成立**，已在文档写明。
- **清单的勾选语义** `[V]`（2026-10-11）——`[x]` = 该动作**已上机做过且有证据**；
  `[ ]` = 真没做过。两个常见误读写进了清单 §3 开头：
  `[x]` ≠「现在还是对的」（代码改过要重跑）；`[ ]` ≠「没实现」。
- **同一事实只留一处真值** `[V]` —— 本会话又踩两次（`useStraightLine=1`
  过期、`vfov` 判据写反）。清单是待确认项的唯一真值，CLAUDE.md 只留指针。

## 4. 失败的尝试 — 不要再试

（沿上一份，**全部前向搬运**。）

- **`DjiFcSubscription_GetLatestValueOfTopic`** `[?]` —— SIGSEGV。一律走回调缓存。
- **在 `ApplicationStart()` 之前调 `DjiFcSubscription_Init()`** `[?]` ——
  返回 SUCCESS 但随后 SIGSEGV。**返回成功不代表调用合法。**
- **在控件回调里调任何阻塞接口** `[?]` —— 进程闪退。
- **`DjiLiveview_Deinit()` 在常驻应用里调** `[V]` —— 黑屏且不自恢复。
- **同一订阅项重复订阅** `[V]` —— 头文件明写不可。
- **用「整图等比放大」构造变焦用例** `[V]` —— 票数与分母同比例增长。
  必须**画幅定死 + 只缩放目标**。
- **用 `fscanf` 读"不定字段数"的行** `[V]` —— 返回值会撞车。用 `fgets` + 两次 `sscanf`。
- **拿 18:32:49 那一轮反推俯仰增益** `[V]` —— 检测框换到别的红色物体上了。
- **拿 66.2° 当 HFOV** `[V]` —— 正确值 69.63°（4:3 + 82°）。
- **给宏体里的一行加 `//` 注释** `[V]` —— 宏展开成语法垃圾。
- **交叉编译** `[V]` —— 产物要求 GLIBC 2.34，设备 2.31。
- **`tar --mtime='@0'`** `[V]` —— 让 make 静默跳过编译。用**设备时间 −60 s**。
- **不排除 `lz/build*` 就打 tar** `[V]` —— x86 产物混进 aarch64 构建。
- **在仓库根建 `build-native`** `[V]` —— 必须是 **`lz/build-native`**。
- **`app.json` 缺 `description_jp/fr`** `[V]` —— `build_dpk.sh` 报 `KeyError`。
- **在 ctest 里给测试传相对路径数据目录** `[V]` —— 用 `LZ_VISION_TESTDATA`。
- **`precheck` 里用 `check_call` 查"判据有没有被抄回来"** `[V]` ——
  它只问符号在不在文件里，注释里也算。实测替换掉真正的调用**照样绿**。
- **设备侧构建不传 `-DLZ_VISION_BACKEND=hsv`** `[?]`（本会话新发现）——
  CMake 默认是 `stub`，而清单 §0b 的构建命令**没传这个开关** ⇒
  按它构建出来的是 stub 后端，**按「识别目标」不会真的检测红旗**
  （表现为永远"已对准"）。**上机前必须核对 dpk 里装的是哪个后端。**
- **`[X]` 类（已证伪，别用）**：`exception` 白名单 / 零解判据写 `== 0.0` /
  `(unsigned)rc` 打返回码 / `-DPSDK_ROOT=~/path` / "模拟器不实现航点启动" /
  "PSDK 不能转自带云台" / "让 Pilot 2 显示红旗"（`SendAiMetaToPilot` 返回
  SUCCESS 是假阳性）/ `DjiWidget_DeInit()` 不存在 / Waypoint V2 用于 M4T /
  兴趣点环绕可控半径 / 在妙算3 应用管理面板里找控件 / 用 `/dev/tcp` 扫网段。

## 5. 已知坑

- **`.dpk` 安装器会试运行应用并要求走完 SDK 身份校验** `[V]` ——
  **飞机不通电就装不上**。判据：日志里有 `dji_identity_verify.c:654`。
- **`Smart3DExplore` 会自启占 PSDK 通道** `[V]` —— 跑程序前先
  `pgrep -x Smart3DExplore >/dev/null || /system/bin/dji_app_ctl stop Smart3DExplore`。
- **改了源码却是旧行为** `[V]` —— 传完源码必须 `rm -rf lz/build-native` 全量重编。
- **`SetMode(FREE)` 在 M4T 上是否生效未知** `[?]` —— "返回 SUCCESS ≠ 生效"。
- **云台 `.z` 是不是 yaw 未单独验过** `[?]`；**`GetOpticalZoomParam` 可能只报
  光学倍率** `[?]`；**激光海拔与起飞点海拔参考面可能不一致** `[?]`。
- **Pilot 会攥着缓存的控件会话不刷新** `[V]` —— 控件全无响应时**先查 `0x3C1A`**，
  别一上来改代码。
- **取图分辨率是 1440×1080（4:3），不是照片的 8064×6048** `[V]`（本会话新确认）——
  `StartImageStream` 走的是**图传预览流**（视频管线，2×2 binning），
  不是拍照管线。`source=3` 的 4K 档是 4032×3024（= 照片的 1/4），
  且被 `LZ_VISION_SOURCE_MAX_PIXELS`(1920×1080) 正确拒掉。
  ⇒ 对风机叶片这类**低对比、靠形状**的目标，**分辨率可能成为瓶颈**。
- **`-1` 会满足 `<= 1`** / **注释里不能写 `*/`** / **管道下 stdio 全缓冲** /
  **gdb 抓 PSDK 进程要先 `handle SIG32 nostop noprint pass`** /
  **`/blackbox/system/app_temp_files/` 是 root:root 0755**（删不掉）/
  **Probe 退出时 SDK 在 deinit 段 dump core** —— 均 `[V]`，
  细节见 [`lz/doc/BUILD-DEPLOY.md`](lz/doc/BUILD-DEPLOY.md)「已知坑」。

## 6. 下一步（有序）

1. **核对 dpk 的视觉后端**（本会话新发现，零成本）：确认设备侧构建命令是否传了
   `-DLZ_VISION_BACKEND=hsv`。**没传则「识别目标」是假的**。把这条写进清单 §0b。
2. **把「航点任务中 PSDK 能否控云台」加进清单**（生死判据，十分钟）。
   判据**必须两条同时满足**：`DjiGimbalManager_Rotate` 返回 SUCCESS
   **且** `TOPIC_GIMBAL_ANGLES` 读回的角度真的跟到。**只看返回码会拿到假阳性**
   （`SendAiMetaToPilot` 的先例）。`lz_gimbal_probe` 本来就在做第二条。
3. **设备上线** → 判据 `ping` + `ip neigh`。
4. **设备侧编译打包**（新文件从未在 aarch64 上编过）：
   ```bash
   DEVT=$(sshpass -p 'dji' ssh dji@192.168.1.180 'date +%s')
   tar czf /tmp/lzbuild.tar.gz --mtime="@$((DEVT - 60))" \
        --exclude='lz/build*' --exclude='*.dpk' \
        lz tools/build_dpk psdk_lib/include psdk_lib/lib/aarch64-linux-gnu-gcc \
        samples/sample_c/platform/linux/{manifold3,common}
   # 设备上：解包 → rm -rf lz/build-native → cmake（**显式 -DLZ_VISION_BACKEND=hsv
   # + -DLZ_POLE_SOURCE_LASER=ON**）→ build → ctest
   ```
   判据：`ctest` 上 **12/12 passed**（aarch64 侧与桌面 hsv 同数）。
5. **装新 dpk**（飞机通电并连接）→ 判据日志出现 `Update dji sdk policy file successfully`。
6. **验主目标**（清单 §3.13 四条）—— 绕飞全程不再出现
   `云台状态：偏航限位` / `偏航电机异常`。
7. **核对首帧原始位图** —— **唯一能验"极性没抄反"的办法**。
8. **按清单 §3.1–§3.8 逐条走**（清单 §3 已按 [x]/[ ] 重新梳理）。
9. **回填 `[?]`**：把 §5 四条按实测结果升级，同步 `CLAUDE.md` 与清单。

## 7. 留给用户的开放问题

- **设备为什么不在线？** 10-05 与 10-08 两次实测都不通，网关却通。
  扩展坞没插？设备没通电？还是又换网段了？
- **风机叶片项目**（本会话讨论，未动手）—— 用户最终目标是**识别风机叶片 →
  反解停机相位角 → 生成三维航点航线**；国旗杆是**替身**（练"识别 → 几何 →
  规划"链路）。已查明的关键事实：
  - 妙算3 = **Jetson Orin NX，100 TOPS**（算力不是瓶颈）
  - 叶片**无颜色特征**，HSV 无效；**公开数据集多是"近拍缺陷"**，
    与"远距看整机"视角不符
  - 停机 + 风致微动 ⇒ 需要的是"**拍照时对准**"而非"连续跟踪"
  - **航点任务是开环的**，要闭环必须 PSDK 接管云台 —— 而控制权**未验证**
    （见 §6 第 2 条）
  - 单目**给不出深度**；纯 SfM 重建叶片**原理上不成立**（无纹理、反光）
- **急停后没有返航按钮** —— 拨 OFF 只是 STOP。建议加独立 button 调
  `DjiFlightController_StartGoHome()`。**未实现。**
- **半径上限 20 m** —— 用户提过后期会变大，需先确认新场地约束。
- **`gimbalEvenlyRotate` 要不要用** —— 规范里它是"航段间均匀转动云台"的正解，
  与逐点 `gimbalRotate` 是两套互斥机制。本次只关掉了逐点 yaw。

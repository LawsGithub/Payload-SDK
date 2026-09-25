/**
 * @file lz_vision_source.h
 * @brief 取图：把 PSDK 的图像流变成 `LzFrame`。
 *
 * 设计依据与全部待定项见 [`doc/VISION-GIMBAL-PITCH.md`](../doc/VISION-GIMBAL-PITCH.md) §3①、§6.1。
 *
 * ## 本模块只做一件事：把「最新一帧」放到能被读的地方
 *
 * 它**不做检测、不做任何决策** —— 检测是 `lz_vision` 的事。这条分工与
 * `lz_vision` 自称"唯一职责是回答杆在画面里的哪里"是同一条纪律。
 *
 * ## 三条硬约束（写在这里因为它们是这个模块的全部难点）
 *
 * 1. **回调里的 `buf` 在回调返回后失效** —— 必须当场拷贝。
 *    这条**不是文档写的**，是从官方样例反推的：
 *    `dji_liveview_object_detection.cpp` 一进回调就 `clone()`，
 *    `test_liveview.c` 里则是立刻 `fwrite`。两种做法都指向"不能存指针"。
 *
 * 2. **回调跑在 PSDK 的工作线程上**，不能阻塞。
 *    与「控件回调只置标志」是**同一条纪律**（本项目为此闪退过一次，
 *    见 `lz_widget.h`）。所以本模块的设计是：
 *
 *        回调（SDK 线程）              调用方（我们的线程）
 *        ────────────────              ──────────────────
 *        加锁 → memcpy → 解锁   →     加锁 → 拷出 → 解锁 → 在外面慢慢算
 *
 *    **检测在锁外做** —— 否则回调会被检测耗时拖住，等于阻塞 SDK 线程。
 *
 * 3. **`pixFmt` 选错不是报错，是花屏。** 所以回调对**收到的**
 *    `imageInfo.pixFmt` 做校验，不符就丢帧并计数（`droppedFmt`），
 *    而不是把垃圾数据当图用 —— 后者会让上层报"检测不到红色"，
 *    而病因在格式上，排查方向完全被带偏。
 */

#ifndef LZ_VISION_SOURCE_H
#define LZ_VISION_SOURCE_H

#include <stdbool.h>
#include <stdint.h>

/* 本模块只编进 lz_app（它总是有 PSDK），所以头文件直接用 PSDK 的类型 ——
 * 不必像 lz_vision 那样把 PSDK 挡在外面。挡的是"检测算法"那一层，
 * 不是"接线"这一层。 */
#include <dji_liveview.h>

#include "lz_types.h"
#include "lz_vision.h"

/**
 * 单帧像素数上限。超出的帧被丢弃并计入 `droppedTooBig`。
 *
 * 1920×1080 是 FHD 流的上限（M4T 的码流规格是 4K/FHD 两档，
 * 而 `StartImageStream` 给的是解码后图像，未见文档说明其分辨率）。
 * 定这个上限是为了**避免按帧尺寸做动态分配** —— 在 SDK 回调里
 * malloc/free 会把分配器卷进回调线程，而那种故障极难定位。
 */
#define LZ_VISION_SOURCE_MAX_PIXELS (1920u * 1080u)

/** 取图状态统计。用于探针与日志 —— **取图不工作时，这是唯一能说话的东西** */
typedef struct {
    uint32_t frames;        /*!< 成功收到的帧数 */
    uint32_t droppedFmt;    /*!< 因像素格式不是 RGB_PACKED 而丢的帧 */
    uint32_t droppedTooBig; /*!< 因超出 `LZ_VISION_SOURCE_MAX_PIXELS` 而丢的帧 */
    uint32_t droppedStride; /*!< 因行字节数 = w*3 不成立而丢的帧（打包格式应成立） */
    int lastWidth;
    int lastHeight;
    bool lastIsBgr;         /*!< 对 3 通道的自述；真值待上机确认，见 `.h` 顶部说明 */
    uint32_t lastFrameId;
} LzVisionSourceStats;

/**
 * 取图配置：**要哪个相机、哪一路子源**
 *
 * 单列成一个结构体而不是写死在 `.c` 里，理由是这两个值**都还没有权威依据**：
 *
 * - `position`：M4T 原生相机（可见光/红外/4K）实测挂在**位置 1**
 *   （`[V]` 2026-09-19，与激光测距同一路）。
 * - `source`：`E_DjiLiveViewCameraSource` 里 M4T 有 `M4T_VIS=1` /
 *   `M4T_IR=2` / `M4T_4K=3`，而 M4T 的镜头是**广角 82° / 中长焦 35° /
 *   长焦 15°** —— **对不上**。`VIS` 到底是哪一个，无从判断。
 *   ⚠️ 这个值错不会报错，只会让视场角换算的分母错最多 **5 倍**
 *   （见 `doc/VISION-GIMBAL-PITCH.md` §6.2 #10 与 §9 #1）。
 * - `pixFmt`：官方样例用的是 `PIXFMT_RGB_PACKED`。选错**不是报错，是花屏**。
 *
 * 所以做成参数、默认值只是"目前的最优猜测"，**由探针去试**。
 */
typedef struct {
    E_DjiLiveViewCameraPosition position;
    E_DjiLiveViewCameraSource source;
    E_DjiLiveViewPixFormate pixFmt;
} LzVisionSourceConfig;

/**
 * @brief 目前的默认配置（= 最优猜测，**未上机验证**）
 *
 * ```text
 *   position = NO_1        （实测原生相机在位置 1）
 *   source   = M4T_VIS (1) （猜的 —— 见上）
 *   pixFmt   = RGB_PACKED  （照官方样例）
 * ```
 */
LzVisionSourceConfig LzVisionSource_DefaultConfig(void);

/**
 * @brief 初始化并开流（需在 `DjiCore_Init()` 之后）
 *
 * ⚠️ **失败不阻断启动。** 仓库级硬规则：fail-closed 的边界划在
 * 「作业开始」，不是「进程启动」—— `dji_app_ctl install` 会试运行应用
 * 并要求走完 SDK 身份校验，此时退出会被误报成
 * `Error, verify user_app_id or version info error`。
 * 所以本函数失败只记日志，由上层决定"没有画面时怎么办"。
 */
LzStatus LzVisionSource_Init(void);

/** @brief 同上，但用指定的配置。探针用它逐个试组合。 */
LzStatus LzVisionSource_InitWith(const LzVisionSourceConfig *cfg);

/** @brief 停流并释放缓冲。可重复调用。 */
void LzVisionSource_Stop(void);

/** @brief 是否已开流（不代表已经收到过帧 —— 那个看 `frames > 0`） */
bool LzVisionSource_IsReady(void);

/**
 * @brief 把**最新一帧**拷贝到调用方给的缓冲
 *
 * @param frame  [out] 描述结构；`data` 指向 `dst`，其余字段由本模块填
 * @param dst    [out] 像素数据落点，长度须 >= `LZ_VISION_SOURCE_MAX_PIXELS * 3`
 * @param dstCapacity [in] `dst` 的字节数
 * @return `LZ_OK`；`LZ_ERR_NOT_READY` = 还没收到过帧；
 *         `LZ_ERR_PARAM` = 入参为空或 `dstCapacity` 不够
 *
 * ⚠️ **拷贝是在锁内做的，检测不是** —— 见文件头第 2 条。
 */
LzStatus LzVisionSource_LatestFrame(LzFrame *frame, uint8_t *dst, size_t dstCapacity);

/** @brief 取一份统计快照 */
void LzVisionSource_GetStats(LzVisionSourceStats *out);

/**
 * @brief 本模块当前假设的像素格式与画面来源的**文字说明**，供日志打印
 *
 * 存在的理由：`pixFmt` 与 `source` 都是编译期常量，而"实际取到的是什么"
 * 是本方案的头号待定项（`VISION-GIMBAL-PITCH.md` §9 #1）。
 * 把它们打成一行字，现场才能一眼看出"这次跑的是哪个假设"。
 */
const char *LzVisionSource_DescribeAssumptions(void);

#endif /* LZ_VISION_SOURCE_H */

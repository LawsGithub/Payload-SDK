/**
 * @file wt_frame_pump.h
 * @brief 相机取流 + 帧缓冲，供推理线程按需拉取最新一帧。
 *
 * ## 为什么不能把回调里的指针塞进队列
 *
 * `DjiLiveview_ImageCallback` 的 `const uint8_t *buf` **没有任何生命周期
 * 承诺** —— 头文件里对它只有类型声明，没有 @note 说明回调返回后是否仍然
 * 有效。对这种情况只能保守假定「返回即失效」（SDK 会复用解码缓冲）。
 *
 * 所以 README 现有描述里的「只把帧指针塞进无锁队列」是不成立的：
 * 那样队列里放的是**悬垂指针**，消费者读到的是下一帧甚至半帧的内容，
 * 而现象是推理结果间歇性地荒谬 —— 极难定位。本模块在回调里**拷贝**。
 *
 * ## latest-frame-wins，而不是队列
 *
 * 30fps × 1440×1080×3 = 4.6 MB/帧、140 MB/s。排队会有两个坏处：
 * 内存占用随积压增长，且消费者拿到的是**过时的**帧 —— 而对"把叶尖转到
 * 画面中心"这种闭环，过时的帧比没有帧更糟（会据此下发错误的云台角）。
 *
 * 所以只保留一帧：新帧直接覆盖未被取走的旧帧。采集激光点时推理只需要
 * 每秒几帧，绝大多数帧本来就该丢掉。
 *
 * ## 回调里只做 memcpy + trylock
 *
 * 回调跑在 SDK 线程上，阻塞它会拖死整条取流链路（姊妹项目踩过：在控件回调
 * 里直接调阻塞接口，日志刷 `semaphore wait timeout` 后进程死掉）。
 * 因此这里用 `pthread_mutex_trylock` —— 抢不到锁就**丢掉这一帧**，
 * 绝不等待。丢帧计数会报出来，便于判断是否丢得过多。
 */

#ifndef WT_FRAME_PUMP_H
#define WT_FRAME_PUMP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 帧缓冲上限：实测 M4T 可见光流是 1440×1080 RGB_PACKED = 4,665,600 字节。
 * 这里按 1920×1080×3 留余量，超过上限的帧会被丢弃并计数（而不是越界写）。
 */
#define WT_FRAME_MAX_BYTES (1920u * 1080u * 3u)

/** 一帧的元信息 */
typedef struct {
    int widthPx;
    int heightPx;
    uint32_t frameId;     /*!< 取自 T_DjiLiveviewImageInfo.frameId，可用于判丢帧 */
    uint32_t capturedAtMs;
    size_t byteCount;
} WtFrameInfo;

/**
 * @brief 启动取流。
 *
 * @note 内部会调 `DjiLiveview_Init()`。若 `wt_ai_stream_probe` 也在跑，
 *       两者都会 Init/Deinit 同一个模块 —— **不要同时启用**，
 *       取流探针是诊断用的，正式链路用本模块。
 */
bool WtFramePump_Start(void);

/** @brief 停止取流并释放缓冲 */
void WtFramePump_Stop(void);

/**
 * @brief 取走最新一帧的拷贝。
 *
 * @param dst     目标缓冲，需至少 WT_FRAME_MAX_BYTES 字节
 * @param dstCap  dst 的容量
 * @param outInfo 帧元信息
 * @return true 表示取到了一帧新的；false 表示暂无新帧（上一帧已被取走）
 *
 * @note 取走后该帧即被标记为已消费，重复调用不会拿到同一帧。
 */
bool WtFramePump_TakeLatest(uint8_t *dst, size_t dstCap, WtFrameInfo *outInfo);

/** @brief 累计收到的帧数 */
uint32_t WtFramePump_ReceivedCount(void);

/** @brief 累计丢弃的帧数（消费者来不及取 / 抢不到锁 / 超出缓冲上限） */
uint32_t WtFramePump_DroppedCount(void);

/**
 * @brief 帧旁路：每收到一帧，在 SDK 回调线程里**同步**调用一次。
 *
 * 只给 AI 叠加用 —— 把 SDK 的原始帧交给它的编码器（见 wt_ai_overlay.c）。
 * 调用先于拷进推理缓冲，且每帧都调，与这一帧能否进推理缓冲无关：
 * 推出去的视频要连续。实现必须快、不能等锁（理由同上方「回调里只做
 * memcpy + trylock」）。`buf` 只在调用期间有效。
 *
 * @param pixFmt  `E_DjiLiveViewPixFormate` 的取值（本模块固定 RGB_PACKED）
 */
typedef void (*WtFrameTap)(const uint8_t *buf, uint32_t len, int widthPx, int heightPx,
                           int pixFmt, uint32_t frameId);

/**
 * @brief 设置帧旁路，传 NULL 取消。`WtFramePump_Stop` 会自动取消。
 */
void WtFramePump_SetTap(WtFrameTap tap);

#ifdef __cplusplus
}
#endif

#endif /* WT_FRAME_PUMP_H */

/**
 * @file lz_visual_align.c
 * @brief 视觉照准的实现。设计依据与"为什么闭环"见 lz_visual_align.h。
 *
 * ## 分工：本文件只做"接 PSDK 的活"，判据全在 `lz_align.c`
 *
 * 三组判据（这一步该做什么 / 两个动作能否并存 / 重试计数）都在
 * `lz_core` 的 `lz_align.c` 里，是**零依赖纯函数，桌面上可完整回归**
 * （`tests/lz_test_align.c`，102 项断言）。本文件只负责：开图、取帧、读云台角、
 * 下发转角、把失败翻译成操作员看得懂的一句话。
 *
 * 这么分的理由与 `LzPlan_ClampWaypointCount()`、`LzPole_JudgeLaserReading()`
 * 被抽出去完全相同：`app/` 依赖 PSDK ⇒ 桌面上编不了也测不了 ⇒ 写在这里的
 * 判据等于没有测试。本轮据此修掉的三处缺陷**全都是判据写在这里时看不出来的**
 * （见下方"本轮修复"）。
 *
 * ## 本轮修复（三处都是"判据在、注释在、不生效"）
 *
 * 1. **`NaN` 能穿过包线检查。** C 里 `NaN < x` 与 `NaN > x` 都是 false，
 *    所以 `fabs(delta) <= 死区` 与 `target < MIN || target > MAX` 两处
 *    都放过它 ⇒ NaN 被原样发给云台。现在由 `LzAlign_DecideStep` 最前面的
 *    `isfinite` 守卫拦下 —— **"两个检查都没反对"不等于"检查通过了"**。
 * 2. **"检测失败 5 次就放弃"永远到不了上限。** 旧 `do_grab()` 成功时一句
 *    `s_waitTicks = 0` 把"等帧"与"检测失败"两个计数一起清了，于是每轮都是
 *    "取到帧（清零）→ 再失败一次"，计数在 1~5 之间打转 ⇒ **照准永不结束**。
 *    现在两个计数分开（`LzAlignRetry`），且各有各的用例守着。
 *
 * 4. **轮数上限的语义被败者撑破。** 旧代码在**检测那一步**（`++s_round`）
 *    就计一轮，于是"等帧失败 / 检测失败"也吃掉轮数 —— 上限是"最多转多少次"
 *    的语义，被"最多出错多少次"挤掉会提前报"未收敛"。
 *    现在只在**真的下发了转角**之后才计轮（见 `do_detect()` 里那句注释）。
 *
 * 5. **启动竞态被误报成"坏数据"。** `read_pitch()` 读不到角返回 `NAN`
 *    （**不返回 0**），而判据把 `NAN` 判为 `BAD_MEASURE` 并终止。
 *    但订阅刚建立时的"还不知道"与"读到坏值"是两回事 ——
 *    旧代码不分，一次竞态就以"测量数据不可信"收场，把操作员引去查瞄准与光照。
 *    现在开局有 `ST_WAIT_ANGLES`（最多 2 秒）把"暂时不知道"挡在循环之外。
 * 3. **照准在跑时拨绕飞开关不会被拦。** 互斥原先只写在一个方向
 *    （按「识别目标」时查绕飞），反方向没查 —— 于是照准转云台的同时
 *    可以启动航线，航线里的 `gimbalRotate` 与手动控制抢同一个云台。
 *    现在两个方向都过 `LzAlign_CheckConflict()`（见 `lz_mission.c`）。
 *
 * ## 为什么是一个显式的小状态机，而不是一个循环
 *
 * 一次照准 = 十几秒（多轮 × 每轮约 1.5 s 等云台）。而主循环是 100 ms 一拍。
 * 两种做法：
 *
 *   A. 在 Tick 里写个 while 循环跑完 → **主循环卡死十几秒**，
 *      这期间拨绕飞开关、按记录按钮全都没响应。
 *   B. 每拍只推进一步 → 主循环照常转，操作员随时能停。
 *
 * 选 B。代价是状态要显式列出来（下面那个 enum），收益是**不阻塞**——
 * 这与本项目一贯的纪律一致（控件回调不做耗时动作、检测在锁外做）。
 */

#include "lz_visual_align.h"

#include <dji_camera_manager.h>
#include <dji_fc_subscription.h>
#include <dji_gimbal_manager.h>
#include <dji_logger.h>
#include <dji_platform.h>

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "lz_align.h"         /* 判据全在这里，零依赖 */
#include "lz_plan.h"          /* LZ_GIMBAL_PITCH_MIN/MAX_DEG —— 唯一真值处 */
#include "lz_vision.h"
#include "lz_vision_source.h"
#include "lz_widget.h"        /* 浮窗消息 */

/* M4T 原生相机（含激光、含云台）实测在位置 1 —— 见 CLAUDE.md */
#define LZ_ALIGN_MOUNT DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1

/**
 * 等待云台转完的时间（ms）。
 *
 * 实测（2026-09-27）：绝对角模式转到 −30/−60/−90/0 的稳定耗时是
 * **1.0–1.2 s**（`lz_gimbal_probe` 用轮询读回实测）。取 1500 ms 留余量。
 * ⚠️ 这是个**实测值**，不是猜的；换了机型/大角度要重新测。
 */
#define LZ_ALIGN_SETTLE_MS      1500

/** 一步里最多等几拍（100 ms/拍）—— 用于把"等"也做成非阻塞 */
#define LZ_ALIGN_WAIT_TICKS     (LZ_ALIGN_SETTLE_MS / 100)

/** 复核前多等这一会儿，让云台彻底停稳（死区判定依赖画面的稳定） */
#define LZ_ALIGN_CONFIRM_SETTLE_MS 300

/**
 * 启动保护期的拍数（100 ms/拍 ⇒ **2 秒**）。
 *
 * ## 它防的是什么
 *
 * 现场实测（2026-09-28）：按下按钮 → 主循环跑 `do_init()`，里面
 * `LzVisionSource_Init()` 与 `DjiFcSubscription_Init()` 都是**阻塞**的
 * （前者要等图流建立），实测从按下到开流完成 **1.65 秒**
 * （01:51:47.359 → 01:51:49.004）。这期间主循环完全没响应。
 *
 * ⇒ 操作员几乎必然再按一次，而那一拍会被判成"再按一次 = 停止"
 *   ⇒ **照准启动后 100 ms 自杀**。
 *
 * 所以启动之后设一段保护期，这期间的按压**全部吞掉**并明确告诉操作员
 * 为什么（`LzAlign_DecideToggle` 的 `inGrace`）。
 *
 * ## 为什么是 2 秒而不是"等 init 真的结束"
 *
 * `do_init()` 是**同步跑完**才返回的，所以保护期从它之后才开始计 ——
 * 保护期防的不是"init 期间"（那时候主循环压根没在看标志），
 * 而是 **init 结束后、操作员那第二下按压才被处理到的那一刻**。
 * 2 秒覆盖"人再按一次的间隔"（现场实测那两次相隔 1.2 秒）。
 */
#define LZ_ALIGN_GRACE_TICKS 20

/**
 * 等第一条云台角推送的拍数上限（100 ms/拍 ⇒ 2 秒）。
 *
 * ## 为什么要有这一步
 *
 * `read_pitch()` 读不到角时返回 `NAN`（**不返回 0** —— 理由见该函数），
 * 而 `LzAlign_DecideStep()` 把 `NAN` 判为 `BAD_MEASURE` 并**终止整个照准**。
 * 那是对的：拿一个假起点去算"该转到哪"毫无依据。
 *
 * 但订阅刚建立时 `s_gotAngles` 还是假 —— **那是暂时不知道，不是坏数据**。
 * 不分清这两者，一次启动竞态就会以"测量数据不可信"收场，而操作员照着那句话
 * 去查的东西（瞄准、光照）与真实病因（订阅还没推第一条）完全无关。
 *
 * ⇒ 开局显式等一小会儿，把「暂时不知道」挡在循环之外，
 * 让 `BAD_MEASURE` 在循环里**真的**只代表坏数据。
 *
 * 实测（2026-09-27，`lz_gimbal_probe`）：50 Hz 订阅下 1 秒内收到 50 条。
 * 给 2 秒是留余量，不是猜的。 */
#define LZ_ALIGN_ANGLES_WAIT_TICKS 20

/* ------------------------------------------------------------------ */
/* 状态                                                                */
/* ------------------------------------------------------------------ */

typedef enum {
    ST_IDLE = 0,
    ST_START,        /* 置了请求，准备开始 */
    ST_WAIT_ANGLES,  /* 等第一条云台角推送（见下方说明） */
    ST_GRAB,         /* 等一帧可用 */
    ST_DETECT,       /* 检测 + 算 Δθ + 决定转不转 */
    ST_WAIT,         /* 等云台转完（非阻塞式等待） */
    ST_FINISH_DONE,
    ST_FINISH_FAIL,
    ST_STOP,         /* 操作员停止 / 失败后收尾 */
} LzAlignStep;

static volatile bool s_toggleRequested = false;
static LzAlignStep s_step = ST_IDLE;
static LzAlignState s_state = LZ_ALIGN_IDLE;

static LzVision *s_vision = NULL;
static LzVisionConfig s_cfg;
static uint8_t *s_frameBuf = NULL;   /* 用 osal 的 Malloc 统一管理 */

static bool s_sourceReady = false;
static bool s_gimbalReady = false;
static bool s_anglesSubscribed = false;
/**
 * `DjiFcSubscription_Init()` 是否已经调过。
 *
 * ⚠️ **刻意只调一次、且 `teardown()` 里不调 DeInit。**
 * 理由有两层：
 *
 * 1. `lz_bridge_psdk.c` 的启动诊断**已经调过它了**（同样有一次性守卫），
 *    同一个模块重复 Init 的语义没有文档保证 —— 头文件只写"必须在订阅之前
 *    调用"，没说可以重复调用几次。**没有文档保证的事不做假设。**
 * 2. `DjiFcSubscription_DeInit()` 会释放整个订阅模块占用的系统资源，
 *    而启动诊断那 5 个话题还在用它 —— 照准结束时把它关掉，
 *    等于顺手把"启动被拒"时的诊断信息也一起废了（那正是最需要它的时候）。
 */
static bool s_fcInitialized = false;

/* 云台角缓存（走回调 —— 不用在本 SDK 版本上必崩的 getter） */
static volatile T_DjiVector3f s_angles = {0};
static volatile bool s_gotAngles = false;

static LzAlignPolicy s_policy;
static LzAlignRetryPolicy s_retryPolicy;
static LzAlignRetry s_retry;

static int s_round = 0;
static int s_anglesWaitTicks = 0;
static int s_rotateWaitTicks = 0;
static int s_graceTicks = 0;
static bool s_awaitingConfirm = false;   /* 真：这一轮只是复核，不转云台 */
static double s_lastDelta = 0.0;
static double s_lastConf = 0.0;
static double s_lastU = 0.0, s_lastV = 0.0;
static const char *s_failReason = "";

/* ------------------------------------------------------------------ */

static void sleep_ms(uint32_t ms)
{
    const T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    if (osal != NULL && osal->TaskSleepMs != NULL) {
        (void)osal->TaskSleepMs(ms);
    }
}

static T_DjiReturnCode on_gimbal_angles(const uint8_t *data, uint16_t size,
                                        const T_DjiDataTimestamp *ts)
{
    (void)ts;
    if (data == NULL || size < sizeof(T_DjiVector3f)) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    memcpy((void *)&s_angles, data, sizeof(T_DjiVector3f));
    s_gotAngles = true;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/** 云台当前俯仰角；读不到时返回 NAN —— **不返回 0**。
 *
 * ⚠️ 返回 0 会让"读不到角"看起来像"云台在水平位"，于是下一步的
 * `cur + step` 从一个假起点出发，转到一个毫无依据的角度。
 * NAN 会被 `LzAlign_DecideStep` 显式拦下（BAD_MEASURE），这是有意的设计：
 * **不知道就是不知道，别替它猜一个。** */
static double read_pitch(void)
{
    return s_gotAngles ? (double)s_angles.x : NAN;
}

/**
 * @brief 当前光学变焦倍数；读不到时返回 1.0（= 广角端）
 *
 * ## 为什么照准必须知道变焦倍数（2026-09-28 现场实测）
 *
 * `vfov` 原先由写死的 82° DFOV 算出 = 55.09°，那是**广角端**的值。
 * 操作员用 Pilot 2 的变焦放大到 7.0X 之后，真实垂直视场角只有 **8.55°**
 * —— 相差 **6.98 倍**。后果不是"偏一点"：
 *
 * ```text
 * 18:17:16  下发 +3.29°（按 55.09° 模型算出"该转这么多"）
 *           实测 v: 0.5551 → 0.1708（Δv=−0.384）
 *           按 55.09° 模型只该动 +0.055
 * ```
 *
 * ⇒ 检测框一步就被甩出画面，下一轮报"看不到红色目标"。
 * 日志表现为**云台突然跳一个角度然后照准失败**。
 *
 * ## 为什么返回 1.0 而不是 NAN
 *
 * 读不到变焦倍数时按广角端处理是**保守**的：真实视场只会更小，
 * 于是算出的 Δθ 只会更**小**、转得更**慢** —— 多转两轮能收敛，
 * 而按相反方向猜（当成大倍率）会让每一轮都过冲，**永远收敛不了**。
 *
 * ⚠️ 与 `read_pitch()` 的取舍**不一样**，理由也不一样：那里"不知道"
 * 会导致一个**毫无依据**的起点（假 0°），必须判 BAD_MEASURE；这里
 * "不知道"有一个安全侧的默认值可用。**两个函数的取舍不同，是因为
 * 错的代价不同，不是因为标准不一致。**
 */
static double read_zoom(void)
{
    T_DjiCameraManagerOpticalZoomParam p;
    memset(&p, 0, sizeof(p));
    if (DjiCameraManager_GetOpticalZoomParam(LZ_ALIGN_MOUNT, &p)
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return 1.0;
    }
    const double z = (double)p.currentOpticalZoomFactor;
    if (!isfinite(z) || !(z > 0.0)) {
        /* 0 / NaN / 负数都当"广角端"处理 —— 同上，保守侧 */
        return 1.0;
    }
    return z;
}

/**
 * @brief 照准的目标点在画面里的归一化纵坐标
 *
 * ## ⚠️ 目前返回的是**旗面框的中心**
 *
 * "杆的中点"需要杆的上下端（`lz_pole_extent`），而它**还没被验证过**：
 * 现有 6 张测试图的裁剪窗口恰好等于整幅图（旗都在 minY=20，裁剪取
 * `旗top-20 .. 旗top+300`），于是杆的上下端在图上**全贴着图边**
 * —— 量到的是"裁到哪儿"，不是"杆在哪儿"。见 CLAUDE.md「步骤 3 卡在测试数据上」。
 *
 * ⇒ **这是降级方案**：现场"旗在杆顶"，旗中点比杆中点高，
 *   15 m 杆、旗占顶上 1.5 m ⇒ 差 6.75 m，12.5 m 外约 28°。
 *   **必然瞄偏**，所以调用方必须在浮窗与日志里显式说明这件事。
 *
 * 等拿到现场杆的近景照片、验证了 `lz_pole_extent`，
 * **只需要改这一个函数**（返回杆中点的 v）。
 */
static double lz_align_target_v(const LzTarget *t, LzPixelBox *flagBox)
{
    if (flagBox != NULL) {
        *flagBox = t->pixel;
    }
    return (t->pixel.topV + t->pixel.bottomV) * 0.5;
}

/* ------------------------------------------------------------------ */
/* 状态切换                                                            */
/* ------------------------------------------------------------------ */

static void enter(LzAlignStep s)
{
    s_step = s;
}

/**
 * @brief 记下失败原因并进入收尾
 *
 * ⚠️ **变参形式是为了让原因里能带上实测数值**。原先只接受一个字符串字面量，
 * 于是"置信度 0.34 不足"这类关键数字没法进浮窗，操作员只能看到一个
 * 笼统的结论 —— 而这正是本项目反复强调要避免的"只报结论、不报原因"
 * （见 `LzVisionMiss` 的说明）。
 *
 * 缓冲是静态的：调用方传进来的字符串字面量生命周期不可控，
 * 而 `s_failReason` 要在几拍之后（`do_finish_fail()`）才被读走。 */
static char s_failBuf[160];

static void fail(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_failBuf, sizeof(s_failBuf), fmt, ap);
    va_end(ap);
    s_failReason = s_failBuf;
    enter(ST_FINISH_FAIL);
}

/** 收尾：停流、关云台模块。幂等。 */
static void teardown(void)
{
    if (s_anglesSubscribed) {
        (void)DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES);
        s_anglesSubscribed = false;
    }
    if (s_gimbalReady) {
        (void)DjiGimbalManager_Deinit();
        s_gimbalReady = false;
    }
    if (s_sourceReady) {
        LzVisionSource_Stop();
        s_sourceReady = false;
    }
    if (s_vision != NULL) {
        LzVision_Deinit(s_vision);
        s_vision = NULL;
    }
    if (s_frameBuf != NULL) {
        const T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
        if (osal != NULL && osal->Free != NULL) {
            osal->Free(s_frameBuf);
        }
        s_frameBuf = NULL;
    }
    /* ⚠️ `s_gotAngles` **不清** —— 回调缓存里那个值仍然有效（话题退订不代表
     * 值变成未知）。清了它会让下一次照准的头几拍读到 NAN 而白报一次
     * BAD_MEASURE。 */
}

/* ------------------------------------------------------------------ */

void LzVisualAlign_RequestToggle(void)
{
    /* ⚠️ 与两个记录按钮同一个模式：这里**只置标志**。
     * 本函数由控件回调（PSDK 工作线程）调用，绝不能做耗时动作。 */
    s_toggleRequested = true;
}

LzAlignState LzVisualAlign_State(void)
{
    return s_state;
}

void LzVisualAlign_DeInit(void)
{
    teardown();
    s_step = ST_IDLE;
    s_state = LZ_ALIGN_IDLE;
}

/* ------------------------------------------------------------------ */
/* 一拍                                                                */
/* ------------------------------------------------------------------ */

static bool do_init(void)
{
    s_policy = LzAlign_DefaultPolicy();
    s_retryPolicy = LzAlign_DefaultRetryPolicy();
    LzAlign_Retry_Reset(&s_retry);

    /* ① 取图 */
    const LzStatus st = LzVisionSource_Init();
    if (st != LZ_OK) {
        USER_LOG_ERROR("照准：取图初始化失败 %s", LzStatus_Str(st));
        fail("取图失败");
        return false;
    }
    s_sourceReady = true;

    /* ② 检测器 */
    s_cfg = LzVision_DefaultConfig();
    if (LzVision_Init(&s_cfg, &s_vision) != LZ_OK) {
        fail("检测器初始化失败");
        return false;
    }

    /* ③ 帧缓冲（用 osal 的 Malloc，与 SDK 同一套分配器） */
    const T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    if (osal == NULL || osal->Malloc == NULL) {
        fail("平台未就绪");
        return false;
    }
    s_frameBuf = (uint8_t *)osal->Malloc(LZ_VISION_SOURCE_MAX_PIXELS * 3);
    if (s_frameBuf == NULL) {
        fail("帧缓冲分配失败");
        return false;
    }

    /* ④ 云台：Init + 订阅角度 */
    if (!s_fcInitialized) {
        if (DjiFcSubscription_Init() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            fail("订阅模块初始化失败");
            return false;
        }
        s_fcInitialized = true;
    }
    if (DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
                                         DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                         on_gimbal_angles)
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        fail("云台角订阅失败");
        return false;
    }
    s_anglesSubscribed = true;

    if (DjiGimbalManager_Init() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        fail("云台模块初始化失败");
        return false;
    }
    s_gimbalReady = true;

    /* 云台模式：用 YAW_FOLLOW —— issue #555 说 M4T 无 FREE 模式，
     * 而我们要动的只是 pitch，YAW_FOLLOW 对它没有影响。 */
    (void)DjiGimbalManager_SetMode(LZ_ALIGN_MOUNT, DJI_GIMBAL_MODE_YAW_FOLLOW);

    s_round = 0;
    s_awaitingConfirm = false;
    s_lastDelta = 0.0;
    s_lastConf = 0.0;
    s_failReason = "";
    s_state = LZ_ALIGN_RUNNING;
    LzWidget_PostMessage("照准开始：请在飞行界面上看着画面");
    LzWidget_PostMessage("⚠ 当前瞄的是「旗面中心」，不是杆中点（杆端点未标定）");
    s_anglesWaitTicks = 0;
    s_graceTicks = LZ_ALIGN_GRACE_TICKS;   /* 见 LZ_ALIGN_GRACE_TICKS 的说明 */
    enter(ST_WAIT_ANGLES);
    /* 浮窗里说清楚"它已经在跑了" —— 操作员看不到这个进程，
     * 唯一的反馈就是浮窗。措辞要能回答"那我现在该干什么"。 */
    LzWidget_PostMessage("照准已启动 —— 正在转云台，请**不要重复按**；"
                         "要停请等本提示消失后再按一次");
    return true;
}

/**
 * @brief 等第一条云台角推送（非阻塞，每拍查一次）
 *
 * 超过上限就判失败 —— **不无限等**：等不到说明话题没推，
 * 那正是「云台角读不回来」这个已记录过的结论，该如实报出来。
 */
static void do_wait_angles(void)
{
    if (s_gotAngles) {
        enter(ST_GRAB);
        return;
    }
    if (++s_anglesWaitTicks > LZ_ALIGN_ANGLES_WAIT_TICKS) {
        LzWidget_PostMessage("照准停止：云台角读不回来（订阅已建立但话题不推）");
        fail("读不到云台角度");
        return;
    }
}

static void do_grab(void)
{
    LzFrame f;
    memset(&f, 0, sizeof(f));
    if (LzVisionSource_LatestFrame(&f, s_frameBuf, LZ_VISION_SOURCE_MAX_PIXELS * 3) != LZ_OK) {
        /* 还没帧 —— 留在本步，下一拍再试。
         * ⚠️ 判据与计数在 `LzAlign_Retry` 里，**不在这里另写一份**。 */
        if (LzAlign_Retry_NoteNoFrame(&s_retry, &s_retryPolicy)) {
            fail("取不到帧");
        }
        return;
    }
    /* ⚠️ 这里**只清"等帧"计数**。旧写法是 `s_waitTicks = 0` 一处清两个，
     * 后果是"检测失败 5 次就放弃"永远到不了上限 ⇒ 照准永不结束。 */
    LzAlign_Retry_NoteFrameOk(&s_retry);
    enter(ST_DETECT);
}

/* ⚠️ `do_detect()` 里**又取了一次帧**，这不是冗余：
 * `do_grab()` 取的那一帧只用来确认"流是通的"，而检测要用的是**那一刻**的画面。
 * 两拍之间隔了 100 ms，而云台正在转 —— 拿 100 ms 前的帧去算"现在该转到哪"，
 * 等于把误差算进闭环里。**代价是每轮多一次 memcpy（1440×1080×3 ≈ 4.4 MB），
 * 而收益是检测用的帧与判定用的云台角在同一时刻附近。**
 *
 * 更彻底的做法是让 `do_grab()` 把帧留在全局缓冲里、`do_detect()` 直接用，
 * 但那要求"帧的生命周期跨状态转移"—— 而 `LzVisionSource_LatestFrame()`
 * 的契约是"拷到调用方的缓冲"，跨状态共享缓冲会把"谁在用这块内存"
 * 变得含糊。多一次拷贝换取清晰的所有权，值得。 */

static void do_detect(void)
{
    LzFrame f;
    memset(&f, 0, sizeof(f));
    if (LzVisionSource_LatestFrame(&f, s_frameBuf, LZ_VISION_SOURCE_MAX_PIXELS * 3) != LZ_OK) {
        enter(ST_GRAB);
        return;
    }

    LzTargetList list;
    LzTargetList_Init(&list);
    const LzStatus st = LzVision_Detect(s_vision, &f, &list);

    if (st != LZ_OK || list.count == 0) {
        LzTargetList_Free(&list);
        /* ⚠️ 检测失败**不立刻判死** —— 可能是这一帧恰好糊了/被挡了。
         * 容忍次数由 `LzAlign_RetryPolicy::maxDetectMisses` 给（5 次）。 */
        if (LzAlign_Retry_NoteDetectMiss(&s_retry, &s_retryPolicy)) {
            /* ⚠️ **两种未命中的处置完全不同，必须分开报**：
             *   · 画面里没有够大的红块 → 真的看不到旗，去调取景
             *   · 有红块但杆的证据不足 → **别动取景**，是检测判据的问题
             * 只报"看不到红色目标（检查取景）"会把第二种引去反复调整瞄准，
             * 而病因在代码里。见 `LzVisionMiss`。 */
            double mc = 0.0;
            const LzVisionMiss miss = LzVision_LastMiss(s_vision, &mc);
            if (miss == LZ_VISION_MISS_LOW_CONF) {
                fail("看到红色但认不出杆（置信度 %.2f）—— 不是取景问题",
                     mc);
            } else {
                fail("看不到红色目标（检查取景）");
            }
            return;
        }
        enter(ST_GRAB);
        return;
    }
    LzAlign_Retry_NoteDetectHit(&s_retry);

    const LzTarget *t = &list.items[0];
    LzPixelBox box;
    const double vTarget = lz_align_target_v(t, &box);
    s_lastConf = t->confidence;
    s_lastU = t->pixel.u;
    s_lastV = vTarget;

    /* 垂直 FOV 由**实测分辨率** + 广角 DFOV 82° + **当前变焦倍数**现算。
     *
     * ⚠️ 变焦那一项是 2026-09-28 现场踩出来的：写死 82° 时，
     * 操作员一放大（7.0X）真实视场就只有 8.55°，俯仰增益差 **6.98 倍**，
     * 每一步都把目标甩出画面。详见 read_zoom() 的注释。 */
    const double zoom = read_zoom();
    const double diagFov = LzVision_ZoomedDiagFovDeg(82.0, zoom);
    const double vfov = LzVision_VerticalFovDeg(f.width, f.height, diagFov);
    const double delta = LzVision_PixelOffsetToDeg(vTarget, vfov);
    const double cur = read_pitch();
    s_lastDelta = delta;   /* 可能是 NaN —— 下面 DecideStep 会拦；这里只用于日志 */
    LzTargetList_Free(&list);

    /* ---- 判据全部在 lz_align.c（零依赖、桌面可测） ---- */
    const LzAlignDecision d =
        LzAlign_DecideStep(delta, cur, s_round, s_awaitingConfirm, &s_policy);

    USER_LOG_INFO("照准第 %d 轮：u=%.4f v=%.4f conf=%.3f zoom=%.1f× vfov=%.2f "
                  "Δθ=%+.2f° pitch=%.2f° → 判定 %d",
                  s_round + 1, s_lastU, s_lastV, s_lastConf, zoom, vfov,
                  delta, cur, (int)d.action);

    switch (d.action) {
    case LZ_ALIGN_ACT_BAD_MEASURE:
        /* ⚠️ 这一条是"NaN 会穿过阈值判据"的落点。措辞要指向**数据不可信**，
         * 而不是"没对准" —— 后者的排查方向（去调瞄准）是错的。 */
        LzWidget_PostMessage("照准停止：测量数据不可信（Δθ=%.2f°, 云台角=%.2f°）",
                             delta, cur);
        USER_LOG_ERROR("照准：非法测量值 Δθ=%f pitch=%f", delta, cur);
        fail("测量数据不可信");
        return;

    case LZ_ALIGN_ACT_CONFIRM:
        s_awaitingConfirm = true;
        LzWidget_PostMessage("接近对准（偏差 %+.2f°），复核一次…", delta);
        /* ⚠️ 这是**唯一**一处留在主循环里的阻塞（300 ms），刻意的：
         * 要让云台彻底停稳才能复核，而"非阻塞地等 300 ms"要多一个状态、
         * 多一份计数，换来的只是主循环早 300 ms 响应。
         * 300 ms 远小于"人察觉到界面卡住"的量级，而取图/云台那几处
         * 才是真正需要拆成多拍的（它们以秒计）。 */
        sleep_ms(LZ_ALIGN_CONFIRM_SETTLE_MS);
        enter(ST_GRAB);
        return;

    case LZ_ALIGN_ACT_DONE:
        enter(ST_FINISH_DONE);
        return;

    case LZ_ALIGN_ACT_ROUNDS_EXHAUSTED:
        fail("未收敛（轮数用尽）");
        return;

    case LZ_ALIGN_ACT_OUT_OF_RANGE:
        /* **如实拒绝**（不钳位）—— 钳位会把"物理上做不到"伪装成"做得到" */
        LzWidget_PostMessage("照准停止：目标角 %.1f° 超出云台范围 [%.0f, %.0f]",
                             d.targetDeg, LZ_GIMBAL_PITCH_MIN_DEG, LZ_GIMBAL_PITCH_MAX_DEG);
        fail("目标角越界");
        return;

    case LZ_ALIGN_ACT_ROTATE:
    default:
        break;
    }

    /* ---- 下发转角 ---- */
    USER_LOG_INFO("照准第 %d 轮：当前 %.2f° → 目标 %.2f°（步进 %+.2f°）",
                  s_round + 1, cur, d.targetDeg, d.stepDeg);

    T_DjiGimbalManagerRotation rot;
    memset(&rot, 0, sizeof(rot));
    rot.rotationMode = DJI_GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE;
    rot.pitch = (dji_f32_t)d.targetDeg;
    rot.roll = 0.0f;
    /* ⚠️ yaw 必须填**当前值**，不能填 0 —— 填 0 意为"转到正北"，
     * 而 M4T pan 只有 ±60°，够不到 ⇒ 整条命令被拒
     * （2026-09-27 实测踩过，报 YAW_REACH_POSITIVE_LIMIT 0x600000004） */
    rot.yaw = s_gotAngles ? s_angles.z : 0.0f;
    rot.time = 0.0;

    const T_DjiReturnCode rc = DjiGimbalManager_Rotate(LZ_ALIGN_MOUNT, rot);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        /* ⚠️ 失败要**分病因**：NON_CONTROL_AUTHORITY 是有人抢控制权，
         * 而 YAW_*_LIMIT 是参数错。两者的处置完全不同。 */
        if (rc == DJI_ERROR_GIMBAL_MODULE_CODE_NON_CONTROL_AUTHORITY) {
            LzWidget_PostMessage("照准停止：云台控制权被占 —— 请勿同时操作云台轮");
            fail("云台控制权被占");
        } else {
            LzWidget_PostMessage("照准停止：转云台被拒 rc=0x%08llX",
                                 (unsigned long long)rc);
            fail("转云台被拒");
        }
        return;
    }

    /* 只有真的下发了才计一轮 —— 轮数上限的语义是"最多转多少次"，
     * 把"等帧/检测失败"也算进去会让上限提前耗尽。 */
    s_round++;
    s_awaitingConfirm = false;
    /* ⚠️ 这里**不动** `s_retry`：等帧计数在 `do_grab()` 里已自管自
     * （成功即清零、失败即累加）。在这里额外清一次会让"取图坏了但每次都
     * 还能挤出一帧"的情形绕过判据 —— 判据只该由一个地方维护。 */
    s_rotateWaitTicks = 0;
    enter(ST_WAIT);
}

static void do_wait(void)
{
    /* 非阻塞式等待：每拍减一格，减到 0 就去看结果。
     * ⚠️ 不 sleep 一整段 —— 那样主循环这 1.5 秒没响应。 */
    if (++s_rotateWaitTicks < LZ_ALIGN_WAIT_TICKS) {
        return;
    }
    s_rotateWaitTicks = 0;
    enter(ST_GRAB);
}

static void do_finish_done(void)
{
    s_graceTicks = 0;

    LzWidget_PostMessage("✓ 已对准：目标在画面中心附近（偏差 %+.2f°，置信度 %.2f）",
                         s_lastDelta, s_lastConf);
    LzWidget_PostMessage("（瞄的是旗面中心；杆中点需现场照片标定后才能用）");
    USER_LOG_INFO("照准完成：共 %d 轮，最终 Δθ=%+.2f°", s_round, s_lastDelta);
    s_state = LZ_ALIGN_DONE;
    enter(ST_STOP);
}

static void do_finish_fail(void)
{
    s_graceTicks = 0;

    LzWidget_PostMessage("✗ 照准失败：%s", s_failReason);
    USER_LOG_ERROR("照准失败：%s（第 %d 轮，最后 Δθ=%+.2f°）",
                   s_failReason, s_round, s_lastDelta);
    s_state = LZ_ALIGN_FAILED;
    enter(ST_STOP);
}

void LzVisualAlign_Tick(void)
{
    /* ---- 启动保护期倒计时（见 LZ_ALIGN_GRACE_TICKS）---- */
    if (s_graceTicks > 0) {
        s_graceTicks--;
    }

    /* 操作员按了按钮：开始 / 停止 / **吞掉**（三选一，判据在 lz_align.c） */
    if (s_toggleRequested) {
        s_toggleRequested = false;

        /* 「能不能停」的判据：**不是** `s_step != ST_IDLE` ——
         * 那会把"刚按下、还在启动"当成"已经在跑"（缺陷的成因）。 */
        const bool canStop = (s_step != ST_IDLE
                              && s_step != ST_START
                              && s_state == LZ_ALIGN_RUNNING
                              && s_step != ST_FINISH_DONE
                              && s_step != ST_FINISH_FAIL);
        const LzToggleAction act = LzAlign_DecideToggle(canStop, s_graceTicks > 0);

        if (act == LZ_TOGGLE_IGNORED) {
            /* ⚠️ 必须**说出来**，不能默默吞掉。操作员刚按下去，
             * 浮窗若什么都不说，他会以为按键丢了而再按一次 ——
             * 那正是要防的那个循环。
             * 措辞要给出"等多久"和"怎么停"，否则只是换了句不响的话。 */
            LzWidget_PostMessage("照准正在启动，已忽略这次按压"
                                 "（约 %.0f 秒后可停）—— 请勿重复按",
                                 (double)s_graceTicks * 0.1);
            USER_LOG_INFO("照准：启动保护期内忽略一次按压（剩余 %d 拍）",
                          s_graceTicks);
        } else if (act == LZ_TOGGLE_START) {
            /* 上一轮的 `s_round` / `s_lastDelta` 等**不清** —— `do_init()`
             * 里统一重置。这样"上一次跑到第几轮"在 `do_init` 之前仍可读，
             * 万一 init 中途失败，日志里那条失败信息还带着上一轮的上下文。 */
            s_step = ST_START;
            s_state = LZ_ALIGN_IDLE;
        } else {
            LzWidget_PostMessage("照准已按操作员要求停止");
            USER_LOG_INFO("照准被操作员停止（已跑 %d 轮，最后 Δθ=%+.2f°）",
                          s_round, s_lastDelta);
            /* ⚠️ 状态记 `FAILED` 而不是新加一个 `STOPPED`：从调用方的角度看，
             * "没对准"这个结论是一样的，都得重按一次才能再来。多一个枚举
             * 就要多一处 switch 分支、多一种组合，而它带来的信息
             * （"是人停的还是自己失败的"）浮窗与日志里已经有了。 */
            s_state = LZ_ALIGN_FAILED;
            s_failReason = "操作员停止";
            s_step = ST_STOP;
        }
    }

    switch (s_step) {
    case ST_IDLE:        break;
    case ST_START:       teardown(); (void)do_init(); break;
    case ST_WAIT_ANGLES: do_wait_angles(); break;
    case ST_GRAB:        do_grab(); break;
    case ST_DETECT:      do_detect(); break;
    case ST_WAIT:        do_wait(); break;
    case ST_FINISH_DONE: do_finish_done(); break;
    case ST_FINISH_FAIL: do_finish_fail(); break;
    case ST_STOP:
        teardown();
        s_step = ST_IDLE;
        break;
    }
}

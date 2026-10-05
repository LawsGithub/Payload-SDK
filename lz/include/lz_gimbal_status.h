/**
 * @file lz_gimbal_status.h
 * @brief 云台状态 → 一行人话（**零依赖，桌面上可完整验证**）。
 *
 * ## 为什么抽到 lz_core，而不是留在 `src/lz_bridge_psdk.c` 里
 *
 * 与 `LzPlan_ClampWaypointCount()`、`LzPole_JudgeLaserReading()`、
 * `LzAlign_DecideStep()` 被抽出来是**同一条理由**：`lz_bridge_psdk.c`
 * 依赖 PSDK，**桌面上编不了也测不了** ⇒ 写在它里面的判据等于没有测试。
 *
 * 而这里恰好有两处**方向性**的判据，写反了都不报错、只让结论反掉：
 *
 * 1. **两组位域的极性相反**（`dji_fc_subscription.h:1129-1140`）：
 *    `pitchLimited/rollLimited/yawLimited` —— **1 = 顶到限位**；
 *    `escPitch/Roll/YawStatus` —— **1 = 正常**（要取反才是"异常"）。
 *    抄反了，告警会**恰好反过来**：正常时喊异常、异常时报正常。
 *    与「`useStraightLine` 语义读反」是同一个形状 —— 本项目记过：
 *    **判据的方向错了，比判据缺失更难发现。**
 * 2. **`mountStatus = 0`（没挂载）时其余位无意义**，而 ESC 位默认 0
 *    经取反会翻成"三轴电机全异常" —— 一条**完全错误**的告警，
 *    会把操作员引去拆云台。所以"挂没挂"必须**先判**。
 *
 * 这两条都能用一张真值表在桌面上钉死（见 `tests/lz_test_gimbal.c`），
 * 留在 PSDK 侧就只能靠上机看日志猜。
 *
 * ## 输入结构体为什么把字段名改成 `escPitchOk` 而不是 `escPitchStatus`
 *
 * 原字段名 `escPitchStatus` 的注释是 "1 - Pitch data is normal, 0 - fault"
 * —— **名字里看不出极性**，这正是抄反的温床。本结构体按**语义**命名
 * （`true = 正常`），极性的转换被挤到边界上**唯一一处**
 * （`lz_bridge_psdk.c` 里那个 `!= 0`），核心逻辑从此不必再想这件事。
 */

#ifndef LZ_GIMBAL_STATUS_H
#define LZ_GIMBAL_STATUS_H

#include <stdbool.h>
#include <stddef.h>

/**
 * 输出缓冲区的最小尺寸。
 *
 * ⚠️ **不是"够用就行"**：七项全报警时最长的一行是 **108 字节 + NUL**
 * （`俯仰限位 横滚限位 偏航限位 俯仰电机异常 横滚电机异常 偏航电机异常
 * 陀螺故障`）。给 96 会在**最坏情况下丢掉最后一项** —— 而"全报警"
 * 恰恰是最需要完整一行的时候。**少一项的诊断信息同样会把人引偏。**
 */
#define LZ_GIMBAL_STATUS_BUF 128

/** 云台状态的**语义化**视图（极性已在边界上归一，见文件头）。 */
typedef struct {
    bool mounted;        /**< 云台是否挂载。**为 false 时其余字段无意义** */
    bool pitchLimited;   /**< 俯仰轴顶到限位（原字段 1 = 顶限位） */
    bool rollLimited;    /**< 横滚轴顶到限位 */
    bool yawLimited;     /**< 偏航轴顶到限位 */
    bool escPitchOk;     /**< 俯仰电机**正常**（原字段 1 = 正常，**不是**"故障"） */
    bool escRollOk;      /**< 横滚电机正常 */
    bool escYawOk;       /**< 偏航电机正常 */
    bool gyroFault;      /**< 陀螺故障（原字段 1 = 故障） */
} LzGimbalStatus;

/**
 * @brief 格式化成一行 —— **只列报警项**，一项一个短词；全好则输出 `正常`
 *
 * ```text
 * 正常                              ← 一切正常（调用方靠它判"变了没"）
 * 偏航限位 偏航电机异常              ← 出问题：把现场看到的那句话原样对上
 * 云台未挂载（其余位无意义）          ← mounted = false
 * ```
 *
 * ## 为什么不做成 `限位[俯仰0 横滚0 偏航1]` 那种定长表格
 *
 * 1. **调用方要能一眼判出"哪一项报了"**。表格里 `偏航` 在限位段与电机段
 *    各出现一次，`strstr(now, "偏航1")` 两处都会命中 —— 而这两件事的处置
 *    完全不同（限位 = 姿态问题，电机异常 = 硬件报警）。这正是本项目反复
 *    踩过的形状：**光看"失败了"会混，要分病因。**
 * 2. **浮窗带宽只有 2 KB/s**，表格里一堆 `0` 是纯负担。
 * 3. 一切正常时输出一个 `正常`，让"变了"这件事仍然可判。
 *
 * ## 刻意**不含** `isBusy` / `calibrating`
 *
 * `isBusy` 在云台顶着限位时会反复翻转，放进字符串等于让这条告警刷屏
 * （浮窗 2 KB/s，本项目已因此踩过两次）。
 *
 * ## 不变式：**任何 `size` 下都不吐半个汉字**
 *
 * 先量后写（`strlen + 1` 放不下就**整项不写**并停手）—— 不能写完再看
 * `snprintf` 的返回值，那时截断**已经发生**，会把一个多字节字从中间切断。
 * 三个出口（正常项、未挂载、全好兜底）**各要单独防**，漏一个就漏一个。
 *
 * @param st   状态；NULL 视为"无数据"（输出空串）
 * @param buf  [out] 建议 `LZ_GIMBAL_STATUS_BUF` 字节
 * @param size 缓冲大小；0 时什么都不写
 */
void LzGimbalStatus_Format(const LzGimbalStatus *st, char *buf, size_t size);

#endif /* LZ_GIMBAL_STATUS_H */

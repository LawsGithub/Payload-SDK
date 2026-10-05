/**
 * @file lz_gimbal_status.c
 * @brief `LzGimbalStatus_Format()` 的实现 —— 零依赖，见头文件。
 *
 * 实现本身很短，但它守住的三条不变式各对应一个真实缺陷（头文件里逐条写了）：
 * 极性、`mountStatus` 先判、"任何 size 下都不吐半个汉字"。
 * 三者的反向验证见 `tests/lz_test_gimbal.c` 末尾。
 */

#include "lz_gimbal_status.h"

#include <stdio.h>
#include <string.h>

/** 报警项表：一项一个短词。
 *
 * ⚠️ 名字里的"限位"与"电机异常"是**刻意分开**的 —— 现场操作员报的是
 * 「偏航角达到限位」与「云台电机异常」两句话，这里原样用上，他才能确认
 * 我们看到的是同一件事。合并成一个"云台故障"就分不出病因了。 */
static const char *const kPitchLimitName = "俯仰限位";
static const char *const kRollLimitName  = "横滚限位";
static const char *const kYawLimitName   = "偏航限位";
static const char *const kPitchEscName   = "俯仰电机异常";
static const char *const kRollEscName    = "横滚电机异常";
static const char *const kYawEscName     = "偏航电机异常";
static const char *const kGyroName       = "陀螺故障";

/** 未挂载时那一句 —— 单独提出来，因为它的 `sizeof` 在调用点被量错过一次
 *  （写成前缀 `sizeof("云台未挂载")`，于是 size=17..36 一路截出半个汉字）。
 *  提到这里、只写一遍，就没有"两个 sizeof 各量各的"的机会。 */
static const char *const kNotMounted = "云台未挂载（其余位无意义）";

/**
 * 往 `buf + n` 追加一项；**放不下就整项不写**并返回 false（调用方停手）。
 *
 * 先量后写：`snprintf` 返回的是"本该写多少"，那时截断已经发生 ——
 * `size - n` 是**字节数**，会把多字节字从中间切断。
 */
static bool append_item(char *buf, size_t size, size_t *n, const char *name)
{
    const size_t need = strlen(name) + ((*n > 0) ? 1 : 0);   /* 分隔空格 */
    if (*n + need + 1 > size) {                                /* +1 给 NUL */
        return false;
    }
    const int w = snprintf(buf + *n, size - *n, "%s%s",
                           (*n > 0) ? " " : "", name);
    if (w < 0) {
        return false;
    }
    *n += (size_t)w;
    return true;
}

/** 整句写入；装不下就留空串 —— 同样不吐半个汉字。 */
static void put_whole(char *buf, size_t size, const char *s)
{
    if (buf == NULL || size == 0) {
        return;
    }
    if (size >= strlen(s) + 1) {
        snprintf(buf, size, "%s", s);
    } else {
        buf[0] = '\0';
    }
}

void LzGimbalStatus_Format(const LzGimbalStatus *st, char *buf, size_t size)
{
    if (buf == NULL || size == 0) {
        return;
    }
    buf[0] = '\0';
    if (st == NULL) {
        return;   /* "无数据" —— 由调用方决定报不报，这里只交空串 */
    }

    /* ★ 未挂载必须**先判**：ESC 位默认 0 会被取反逻辑翻成"三轴电机全异常"，
     * 那是一条**完全错误**的告警，会把操作员引去拆云台。
     * 头文件原文：`mountStatus: 1 - gimbal mounted, 0 - gimbal not mounted`。
     *
     * ⚠️ 这与激光 `exception` 白名单是同一个教训的两面：那边是**漏掉了合法
     * 取值**，这边是**把无意义的取值当成了报警**。共同点是判据必须挂在
     * **能自证的量**上 —— 这里是"挂没挂"，而不是"几个位看起来像故障"。 */
    if (!st->mounted) {
        put_whole(buf, size, kNotMounted);
        return;
    }

    struct { bool set; const char *name; } items[] = {
        {st->pitchLimited, kPitchLimitName},   /* 语义字段：true = 顶限位 */
        {st->rollLimited,  kRollLimitName},
        {st->yawLimited,   kYawLimitName},
        {!st->escPitchOk,  kPitchEscName},     /* true = 正常，故取反 = 异常 */
        {!st->escRollOk,   kRollEscName},
        {!st->escYawOk,    kYawEscName},
        {st->gyroFault,    kGyroName},
    };

    size_t n = 0;
    for (size_t i = 0; i < sizeof(items) / sizeof(items[0]); ++i) {
        if (!items[i].set) {
            continue;
        }
        if (!append_item(buf, size, &n, items[i].name)) {
            break;   /* 整项放不下就停 —— 已写的项仍然完整且有效 */
        }
    }
    if (n == 0) {
        put_whole(buf, size, "正常");
    }
}

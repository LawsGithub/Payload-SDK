/**
 * @file lz_test_gimbal.c
 * @brief 云台状态 → 人话的回归测试（零依赖，桌面可跑）。
 *
 * ## 本文件守什么，以及每条用例对应的真实缺陷
 *
 * | 用例组 | 守的东西 | 写错的表现 |
 * |---|---|---|
 * | A 极性 | 限位 `1`=顶限位、ESC `1`=正常 | 告警**恰好反过来**，且不报错 |
 * | B 未挂载 | `mounted=false` 时先判、单独报 | 报"三轴电机全异常"，把人引去拆云台 |
 * | C 全好 | 输出 `正常`（调用方靠它判"变了没"） | 去重逻辑失效，每拍都报 |
 * | D 截断 | **任何 size 下都不吐半个汉字** | 浮窗上挂半个汉字 |
 * | E 最坏行长 | 七项全报警在 128 字节里装得下 | 最坏时丢掉最后一项 |
 *
 * ## 为什么 D 组要**穷举**而不是挑几个 size 试
 *
 * 这条缺陷的形状是"某个出口漏了一处保护"，而**出口有三个**
 * （报警项循环、未挂载、全好兜底）。挑几个 size 试的时候恰好没撞上那个出口，
 * 就会以为写对了 —— 实测正是如此：第一版"先量后写"只加在循环里，
 * 未挂载那句仍被 `sizeof` 量成了前缀，**穷举当场抓到 1088 组非法 UTF-8**。
 * ⇒ 与「加人造图形用例必须先验证拆掉被测逻辑它会红」同源：
 * **覆盖要靠构造逼出来，不能靠"我想到了"。**
 */

#include "lz_gimbal_status.h"
#include "lz_test.h"

#include <string.h>

/** 是不是合法 UTF-8（不接受被截断的多字节序列）。 */
static bool utf8_ok(const char *s)
{
    const size_t len = strlen(s);
    size_t i = 0;
    while (i < len) {
        const unsigned char c = (unsigned char)s[i];
        int k;
        if (c < 0x80)              { k = 1; }
        else if ((c >> 5) == 0x6)  { k = 2; }
        else if ((c >> 4) == 0xE)  { k = 3; }
        else if ((c >> 3) == 0x1E) { k = 4; }
        else                       { return false; }   /* 非法首字节 */
        if (i + (size_t)k > len)   { return false; }   /* 尾部被切断 */
        i += (size_t)k;
    }
    return true;
}

/** 一个"全好"的状态：挂载、三轴 ESC 正常、无陀螺故障、不顶限位。 */
static LzGimbalStatus all_ok(void)
{
    LzGimbalStatus s;
    memset(&s, 0, sizeof(s));
    s.mounted = true;
    s.escPitchOk = s.escRollOk = s.escYawOk = true;
    return s;
}

int main(void)
{
    char buf[LZ_GIMBAL_STATUS_BUF];

    /* ---------------- A 极性 ---------------- */
    LZ_CASE("A1 偏航顶限位 —— 报的是「限位」，不是「正常」");
    {
        LzGimbalStatus s = all_ok();
        s.yawLimited = true;
        LzGimbalStatus_Format(&s, buf, sizeof(buf));
        LZ_CHECK(strstr(buf, "偏航限位") != NULL);
        LZ_CHECK(strcmp(buf, "正常") != 0);   /* 顶限位时绝不能报正常 */
    }

    LZ_CASE("A2 ESC 位是「1 = 正常」—— 取反取错会让告警恰好反过来");
    {
        LzGimbalStatus s = all_ok();
        /* 全好：ESC 三个都是 true（= 正常）⇒ 输出里**不该**出现"电机异常" */
        LzGimbalStatus_Format(&s, buf, sizeof(buf));
        LZ_CHECK(strstr(buf, "电机异常") == NULL);
        LZ_CHECK(strcmp(buf, "正常") == 0);

        /* 只把偏航电机置为"不正常"⇒ 只该报偏航那一条 */
        s.escYawOk = false;
        LzGimbalStatus_Format(&s, buf, sizeof(buf));
        LZ_CHECK(strstr(buf, "偏航电机异常") != NULL);
        LZ_CHECK(strstr(buf, "俯仰电机异常") == NULL);
        LZ_CHECK(strstr(buf, "横滚电机异常") == NULL);
    }

    LZ_CASE("A3 gyroFault 是「1 = 故障」（与 ESC 位相反）");
    {
        LzGimbalStatus s = all_ok();
        s.gyroFault = true;
        LzGimbalStatus_Format(&s, buf, sizeof(buf));
        LZ_CHECK(strstr(buf, "陀螺故障") != NULL);
    }

    LZ_CASE("A4 现场那两个现象能同时出现，且各自可 grep 到");
    {
        /* 用户 2026-10-01 原话：「云台显示偏航角达到限位，然后显示云台电机
         * 异常」—— 两件事是**两个不同的位**，输出必须能分别匹配到。 */
        LzGimbalStatus s = all_ok();
        s.yawLimited = true;
        s.escYawOk = false;
        LzGimbalStatus_Format(&s, buf, sizeof(buf));
        LZ_CHECK(strstr(buf, "偏航限位") != NULL);
        LZ_CHECK(strstr(buf, "偏航电机异常") != NULL);
        LZ_CHECK(strcmp(buf, "偏航限位 偏航电机异常") == 0);
    }

    /* ---------------- B 未挂载 ---------------- */
    LZ_CASE("B1 未挂载时单独报，绝不报「三轴电机全异常」");
    {
        /* 关键构造：`mounted=false` 而 ESC 位**全是默认的 0**
         * —— 若不做先判，取反逻辑会翻出三条假告警。 */
        LzGimbalStatus s;
        memset(&s, 0, sizeof(s));
        s.mounted = false;
        LzGimbalStatus_Format(&s, buf, sizeof(buf));
        LZ_CHECK(strstr(buf, "未挂载") != NULL);
        LZ_CHECK(strstr(buf, "电机异常") == NULL);
        LZ_CHECK(strstr(buf, "限位") == NULL);
    }

    LZ_CASE("B2 未挂载优先于一切 —— 哪怕别的位看着像报警");
    {
        LzGimbalStatus s;
        memset(&s, 0, sizeof(s));
        s.mounted = false;
        s.yawLimited = true;
        s.gyroFault = true;
        LzGimbalStatus_Format(&s, buf, sizeof(buf));
        LZ_CHECK(strstr(buf, "未挂载") != NULL);
        LZ_CHECK(strstr(buf, "偏航限位") == NULL);   /* 无意义，不该报 */
    }

    /* ---------------- C 全好 ---------------- */
    LZ_CASE("C1 全好时输出「正常」—— 调用方靠它判状态有没有变");
    {
        LzGimbalStatus s = all_ok();
        LzGimbalStatus_Format(&s, buf, sizeof(buf));
        LZ_CHECK(strcmp(buf, "正常") == 0);
    }

    LZ_CASE("C2 NULL 状态 → 空串（不是「正常」）");
    {
        /* "没有数据"与"云台正常"是两件事 —— 混淆会让调用方以为
         * 收到过状态。与 `LzBridge_GimbalStatusStr` 返回 false 同一条纪律。 */
        LzGimbalStatus_Format(NULL, buf, sizeof(buf));
        LZ_CHECK(buf[0] == '\0');
    }

    /* ---------------- D 截断：穷举 ---------------- */
    LZ_CASE("D1 任何 size × 任何报警组合：都 NUL 终止、合法 UTF-8、不越界写");
    {
        /* 3 轴限位 + 3 轴 ESC + 陀螺 = 7 个独立布尔 ⇒ 128 种组合。
         * `mounted` 另算（0/1）⇒ 256 种状态 × size 1..200。
         *
         * 三个不变式一起查，因为它们**在同一个 `snprintf` 出口上失效**：
         * 截断会同时造成"没终止"与"半个汉字"，写超了则是越界。
         * 只查其中一个，另外两个的缺陷会从旁边溜过去。 */
        int bad = 0;
        char small[256];
        for (int mounted = 0; mounted <= 1; ++mounted) {
            for (unsigned mask = 0; mask < 128u; ++mask) {
                LzGimbalStatus s;
                s.mounted      = (mounted != 0);
                s.pitchLimited = (mask & 1u) != 0;
                s.rollLimited  = (mask & 2u) != 0;
                s.yawLimited   = (mask & 4u) != 0;
                s.escPitchOk   = (mask & 8u) == 0;
                s.escRollOk    = (mask & 16u) == 0;
                s.escYawOk     = (mask & 32u) == 0;
                s.gyroFault    = (mask & 64u) != 0;
                for (size_t size = 1; size <= 200; ++size) {
                    memset(small, 0xAA, sizeof(small));   /* 也顺便查越界写 */
                    LzGimbalStatus_Format(&s, small, size);
                    /* ⚠️ 不能只把 small[size] 置 0 再查 —— 那会**替函数补上**
                     * 它自己该写的 NUL，把"没终止"这个缺陷盖掉。
                     * 先显式查 [0, size) 里有没有 NUL（不终止 = 越界读），
                     * 再查整串是不是合法 UTF-8。 */
                    if (memchr(small, '\0', size) == NULL) {
                        ++bad;
                        continue;
                    }
                    if (!utf8_ok(small)) {
                        ++bad;
                    }
                    /* 也不许写到 size 之外（紧邻那一字节的哨兵必须原样还在） */
                    if ((unsigned char)small[size] != 0xAA) {
                        ++bad;
                    }
                }
            }
        }
        LZ_CHECK(bad == 0);
        if (bad != 0) {
            printf("    size×组合里出问题的次数 = %d（不终止 / 非法 UTF-8 / 越界写）\n",
                   bad);
        }
    }

    LZ_CASE("D2 size 很小时宁可空串，也不给半个汉字");
    {
        LzGimbalStatus s = all_ok();
        s.yawLimited = true;
        char tiny[4];
        /* ⚠️ **必须预填**：函数在这种 size 下只写 `buf[0] = '\0'`，
         * 后面几字节不碰 —— 不预填就是读未初始化内存，断言会随机飘。
         * （第一版这里没预填、且断言 `tiny[3] == '\0'`，那是在要求函数
         * 写它根本不写的字节 —— 用例本身写错了，不是实现错了。） */
        memset(tiny, 0xAA, sizeof(tiny));
        LzGimbalStatus_Format(&s, tiny, sizeof(tiny));
        LZ_CHECK(tiny[0] == '\0');        /* 整句装不下 ⇒ 空串 */
        LZ_CHECK(utf8_ok(tiny));          /* 空串也是合法 UTF-8 */
    }

    /* ---------------- E 最坏行长 ---------------- */
    LZ_CASE("E1 七项全报警在 LZ_GIMBAL_STATUS_BUF 里装得下（不丢项）");
    {
        LzGimbalStatus s = all_ok();
        s.pitchLimited = s.rollLimited = s.yawLimited = true;
        s.escPitchOk = s.escRollOk = s.escYawOk = false;
        s.gyroFault = true;
        LzGimbalStatus_Format(&s, buf, sizeof(buf));
        LZ_CHECK(strstr(buf, "俯仰限位") != NULL);
        LZ_CHECK(strstr(buf, "横滚限位") != NULL);
        LZ_CHECK(strstr(buf, "偏航限位") != NULL);
        LZ_CHECK(strstr(buf, "俯仰电机异常") != NULL);
        LZ_CHECK(strstr(buf, "横滚电机异常") != NULL);
        LZ_CHECK(strstr(buf, "偏航电机异常") != NULL);
        LZ_CHECK(strstr(buf, "陀螺故障") != NULL);
        LZ_CHECK(strlen(buf) + 1 <= LZ_GIMBAL_STATUS_BUF);
        /* 最坏行长是硬数字 —— 缓冲区常量改小了，这条会红 */
        LZ_CHECK(strlen(buf) == 108);
    }

    LZ_CASE("E2 少给 1 字节就丢最后一项（证明 E1 的余量不是白留的）");
    {
        LzGimbalStatus s = all_ok();
        s.pitchLimited = s.rollLimited = s.yawLimited = true;
        s.escPitchOk = s.escRollOk = s.escYawOk = false;
        s.gyroFault = true;
        /* 最坏内容 108 字节 + NUL = 109 ⇒ 给 108 差 1 字节 */
        char tight[108];
        LzGimbalStatus_Format(&s, tight, sizeof(tight));
        LZ_CHECK(strstr(tight, "陀螺故障") == NULL);      /* 最后一项放不下 */
        LZ_CHECK(strstr(tight, "偏航电机异常") != NULL);  /* 但前面的都在 */
        LZ_CHECK(utf8_ok(tight));
    }

    return LZ_TEST_SUMMARY();
}

/* ------------------------------------------------------------------
 * 反向验证（逐条回退本层实现，确认对应用例变红 —— 已实测）
 *
 * | 回退的改动 | 结果 |
 * |---|---|
 * | 三个 `{!st->escXxxOk, ...}` 改成 `{st->escXxxOk, ...}`（极性抄反） | **8 项失败**（A2 ×5、A4 ×2、C1 ×1） |
 * | 去掉 `if (!st->mounted)` 那一段 | **4 项失败**（B1 ×2、B2 ×2） |
 * | `append_item` 退回"写完再看 snprintf 返回值" | **2 项失败**；D1 报「出问题的次数 = 3280」 |
 * | `put_whole` 退回直接 `snprintf`（不先量整句长度） | **2 项失败**；D1 报「= 3840」 |
 * | `LZ_GIMBAL_STATUS_BUF` 从 128 改回 96 | **2 项失败**（E1 丢 `陀螺故障`、`strlen == 108`） |
 *
 * ⚠️ 第 3、4 行都是"只在**某个出口**漏了保护"，而 D1 的报数能区分它们
 * （3280 vs 3840）—— 这正是"穷举 + 把三类失效一起查"的价值：
 * 挑几个 size 试的话，两者都会看起来没事。
 * ------------------------------------------------------------------ */

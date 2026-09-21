/**
 * @file lz_sdk_log_watch.c
 * @brief 见头文件 —— 这里只讲实现上的两个取舍。
 *
 * ## 1. 为什么要自己拼行
 *
 * `ConsoleFunc` 的 `dataLen` 是**任意长度**，SDK 内部一次 write 可能给半行，
 * 也可能一次给好几行。所以必须自己维护一个行缓冲：攒到 '\n' 才判一次。
 * 直接对每个 chunk 做 strstr 会在跨 chunk 时漏掉目标串 —— 而这种漏
 * 恰好发生在"日志量大、被切开"的时候，也就是最需要它的时刻。
 *
 * ## 2. 为什么两道闸都是"子串匹配 + 上限"
 *
 * 目标行由 SDK 的 `dji_waypoint_v3.c` 打印，格式我们无法控制。用子串而不是
 * 整行相等，是为了容忍行首的时间戳/级别/文件名前缀（它们由 logger 加上去）。
 * 同时用**行内长度上限**保护：不设上限的话，一个畸形长行会把缓冲写爆。
 *
 * 这个模块**故意做成"尽力而为"**：SDK 换了日志措辞它就抓不到，返回 NULL。
 * 调用方（`lz_mission.c`）在 NULL 时回退到状态摘要 —— 少一条信息，
 * 但绝不误报成"成功"。
 */

#include "lz_sdk_log_watch.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* 行缓冲：SDK 单行日志远短于此。超长行会被截断 —— 截断只影响可读性，
 * 不影响"是否命中目标子串"（目标子串都很短且靠前）。 */
#define LZ_LOGWATCH_LINE_MAX 512

/* 抓取结果缓冲：够装下目标行原样。 */
#define LZ_LOGWATCH_KEEP_MAX 320

static char s_lineBuf[LZ_LOGWATCH_LINE_MAX];
static size_t s_lineLen = 0;

static char s_startReject[LZ_LOGWATCH_KEEP_MAX];
static char s_actionError[LZ_LOGWATCH_KEEP_MAX];

/** 目标行 1：飞机给出的具体原因（携带 error_code 数字） */
static const char *kStartRejectMark = "Start waypoint v3 mission failed";

/** 目标行 2：SDK 把同一个失败转成 PSDK 返回值后的打印 */
static const char *kActionErrorMark = "waypoint v3 action";

/** 去掉行首/行尾的空白与 CR（日志行以 \r\n 结尾的情况很常见） */
static void lz_trim(char *s)
{
    const size_t len = strlen(s);
    size_t end = len;
    while (end > 0 && (s[end - 1] == '\r' || s[end - 1] == '\n' ||
                       s[end - 1] == ' ' || s[end - 1] == '\t')) {
        s[--end] = '\0';
    }
    size_t beg = 0;
    while (beg < end && (s[beg] == ' ' || s[beg] == '\t')) {
        beg++;
    }
    if (beg > 0) {
        memmove(s, s + beg, end - beg + 1);
    }
}

/** 判一行是否命中某个标记；命中则整行存进 dst（按 dst 容量显式截断） */
static void lz_try_keep(const char *line, const char *mark, char *dst, size_t cap)
{
    if (strstr(line, mark) == NULL) {
        return;
    }
    /* 原地 trim：line 指向的就是 s_lineBuf，唯一调用方保证它可写 */
    lz_trim((char *)line);

    /* 显式截断而不是靠 snprintf 静默丢 —— 目标行很短，"被截断"本身
     * 是个信号：说明这行不是我们以为的那一行。截图后仍以 NUL 收尾。 */
    const size_t len = strlen(line);
    const size_t n = (len < cap - 1) ? len : (cap - 1);
    memcpy(dst, line, n);
    dst[n] = '\0';
}

/** 一行攒满了，判一次 */
static void lz_flush_line(void)
{
    if (s_lineLen == 0) {
        return;
    }
    s_lineBuf[s_lineLen] = '\0';

    lz_try_keep(s_lineBuf, kStartRejectMark, s_startReject, sizeof(s_startReject));
    /* 过滤掉我们自己打的那些含 "waypoint v3" 的行：SDK 那行的特征是
     * "action <数字> failed" + "error: 0x..."，用后半段当标记更稳。 */
    if (strstr(s_lineBuf, kActionErrorMark) != NULL &&
        strstr(s_lineBuf, "error:") != NULL &&
        strstr(s_lineBuf, "failed") != NULL) {
        lz_try_keep(s_lineBuf, kActionErrorMark, s_actionError, sizeof(s_actionError));
    }

    s_lineLen = 0;
}

void LzSdkLogWatch_Feed(const uint8_t *data, uint16_t len)
{
    if (data == NULL) {
        return;
    }

    for (uint16_t i = 0; i < len; ++i) {
        const char c = (char)data[i];
        if (c == '\n') {
            lz_flush_line();
            continue;
        }
        /* 超长行：丢弃多余部分而不是覆盖缓冲 —— 覆盖会把已经攒下的
         * 前半行（可能含标记）冲掉。 */
        if (s_lineLen < LZ_LOGWATCH_LINE_MAX - 1) {
            s_lineBuf[s_lineLen++] = c;
        }
    }
}

const char *LzSdkLogWatch_LastStartReject(void)
{
    return (s_startReject[0] != '\0') ? s_startReject : NULL;
}

const char *LzSdkLogWatch_LastActionError(void)
{
    return (s_actionError[0] != '\0') ? s_actionError : NULL;
}

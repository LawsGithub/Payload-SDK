/**
 * @file wt_test.h
 * @brief PC 侧回归测试的极简断言框架（仅宏，不引入任何外部依赖）
 *
 * 为什么不用 <assert.h>：本工程默认 CMAKE_BUILD_TYPE=Release，assert 会被
 * NDEBUG 整体编译掉 —— 那正是最需要断言跑起来的时候。这里的宏无条件生效，
 * 并且会把「文件:行号 + 用例名 + 失败表达式」打到 stderr。
 *
 * 为什么不引第三方框架：wt_core 是不依赖任何东西的静态库，测试也应当保持
 * 同样的性质，让「clone 下来就能在桌面上跑一遍」这句话成立。
 *
 * 用法：
 *     int main(void) {
 *         WT_CASE("我的用例");
 *         WT_CHECK_EQ_INT(1 + 1, 2);
 *         return WT_TEST_SUMMARY();
 *     }
 */

#ifndef WT_TEST_H
#define WT_TEST_H

#include <math.h>
#include <stdio.h>
#include <string.h>

static int wt_test_checks = 0;
static int wt_test_failures = 0;
static const char *wt_test_case = "(未命名)";

#define WT_CASE(name)                                                          \
    do {                                                                       \
        wt_test_case = (name);                                                 \
        printf("[ RUN  ] %s\n", wt_test_case);                                 \
    } while (0)

#define WT_CHECK(cond)                                                         \
    do {                                                                       \
        wt_test_checks++;                                                      \
        if (!(cond)) {                                                         \
            wt_test_failures++;                                                \
            fprintf(stderr, "[ FAIL ] %s:%d [%s] 断言失败: %s\n",              \
                    __FILE__, __LINE__, wt_test_case, #cond);                  \
        }                                                                      \
    } while (0)

#define WT_CHECK_EQ_INT(actual, expect)                                        \
    do {                                                                       \
        long wt_act_ = (long)(actual);                                         \
        long wt_exp_ = (long)(expect);                                         \
        wt_test_checks++;                                                      \
        if (wt_act_ != wt_exp_) {                                              \
            wt_test_failures++;                                                \
            fprintf(stderr, "[ FAIL ] %s:%d [%s] %s = %ld, 期望 %ld\n",        \
                    __FILE__, __LINE__, wt_test_case, #actual, wt_act_, wt_exp_); \
        }                                                                      \
    } while (0)

#define WT_CHECK_NEAR(actual, expect, tol)                                     \
    do {                                                                       \
        double wt_a_ = (double)(actual);                                       \
        double wt_e_ = (double)(expect);                                       \
        wt_test_checks++;                                                      \
        if (!(fabs(wt_a_ - wt_e_) <= (double)(tol))) {                         \
            wt_test_failures++;                                                \
            fprintf(stderr, "[ FAIL ] %s:%d [%s] %s = %.6f, 期望 %.6f (±%g)\n",\
                    __FILE__, __LINE__, wt_test_case, #actual, wt_a_, wt_e_,   \
                    (double)(tol));                                            \
        }                                                                      \
    } while (0)

#define WT_CHECK_BETWEEN(actual, lo, hi)                                       \
    do {                                                                       \
        double wt_a_ = (double)(actual);                                       \
        wt_test_checks++;                                                      \
        if (wt_a_ < (double)(lo) || wt_a_ > (double)(hi)) {                    \
            wt_test_failures++;                                                \
            fprintf(stderr, "[ FAIL ] %s:%d [%s] %s = %.4f, 期望 [%g, %g]\n",  \
                    __FILE__, __LINE__, wt_test_case, #actual, wt_a_,          \
                    (double)(lo), (double)(hi));                               \
        }                                                                      \
    } while (0)

#define WT_CHECK_STR_EQ(actual, expect)                                        \
    do {                                                                       \
        const char *wt_a_ = (const char *)(actual);                            \
        const char *wt_e_ = (const char *)(expect);                            \
        wt_test_checks++;                                                      \
        if (wt_a_ == NULL || strcmp(wt_a_, wt_e_) != 0) {                      \
            wt_test_failures++;                                                \
            fprintf(stderr, "[ FAIL ] %s:%d [%s] %s = \"%s\", 期望 \"%s\"\n",  \
                    __FILE__, __LINE__, wt_test_case, #actual,                 \
                    wt_a_ != NULL ? wt_a_ : "(null)", wt_e_);                  \
        }                                                                      \
    } while (0)

/** main 的返回值：0 = 全部通过，非 0 = 有失败（供 ctest 判定） */
#define WT_TEST_SUMMARY()                                                      \
    (printf("[ ===  ] %s: %d 项检查, %d 项失败\n", __FILE__, wt_test_checks,   \
            wt_test_failures),                                                 \
     wt_test_failures == 0 ? 0 : 1)

#endif /* WT_TEST_H */
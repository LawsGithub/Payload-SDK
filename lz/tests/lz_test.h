/**
 * @file lz_test.h
 * @brief 极简断言框架（仅宏，无外部依赖）。
 *
 * 不用 <assert.h>：Release 构建下 NDEBUG 会把 assert 整体编译掉 ——
 * 而那正是最需要断言跑起来的时候。这里的宏无条件生效。
 */

#ifndef LZ_TEST_H
#define LZ_TEST_H

#include <math.h>
#include <stdio.h>
#include <string.h>

static int lz_test_checks = 0;
static int lz_test_failures = 0;
static const char *lz_test_case = "(未命名)";

#define LZ_CASE(name)                                                          \
    do {                                                                       \
        lz_test_case = (name);                                                 \
        printf("[ RUN  ] %s\n", lz_test_case);                                 \
    } while (0)

#define LZ_CHECK(cond)                                                         \
    do {                                                                       \
        lz_test_checks++;                                                      \
        if (!(cond)) {                                                         \
            lz_test_failures++;                                                \
            fprintf(stderr, "[FAIL ] %s @ %s:%d: %s\n", lz_test_case,          \
                    __FILE__, __LINE__, #cond);                                \
        }                                                                      \
    } while (0)

#define LZ_CHECK_NEAR(a, b, tol)                                               \
    do {                                                                       \
        double lz_va = (a), lz_vb = (b);                                       \
        lz_test_checks++;                                                      \
        if (!(fabs(lz_va - lz_vb) <= (tol))) {                                 \
            lz_test_failures++;                                                \
            fprintf(stderr, "[FAIL ] %s @ %s:%d: %s(%.6f) != %s(%.6f)\n",      \
                    lz_test_case, __FILE__, __LINE__, #a, lz_va, #b, lz_vb);   \
        }                                                                      \
    } while (0)

/**
 * 角度专用的近似比较。
 *
 * 不能用 LZ_CHECK_NEAR 比较方位角：角量是**圆周量**，0° 与 359.999999°
 * 只差 1e-6 度，但绝对差是 360 —— 在 0/360 接缝附近，绝对差判据必然误报。
 * 先取圆周差（折算到 (-180,180]）再比绝对值，才是角量的正确定义。
 */
#define LZ_CHECK_ANGLE_NEAR(a, b, tol)                                         \
    do {                                                                       \
        double lz_d = fmod((a) - (b), 360.0);                                  \
        if (lz_d > 180.0)  { lz_d -= 360.0; }                                  \
        if (lz_d <= -180.0) { lz_d += 360.0; }                                 \
        lz_test_checks++;                                                      \
        if (!(fabs(lz_d) <= (tol))) {                                          \
            fprintf(stderr, "[FAIL ] %s @ %s:%d: %s(%.9f) != %s(%.9f)，圆周差 %.6f°\n", \
                    lz_test_case, __FILE__, __LINE__, #a, (double)(a), #b,     \
                    (double)(b), lz_d);                                        \
        }                                                                      \
    } while (0)

/**
 * @brief 打印汇总并返回进程退出码
 *
 * ⚠️ **检查项数为 0 也算失败。**
 *
 * 为什么：本工程的测试用例大量依赖"先构建一个对象、再断言它的内容"，
 * 写成 `if (obj != NULL) { LZ_CHECK(...) }` 的形式。一旦构建静默失败
 * （缓冲区算小了、依赖的入参校验误拒…），那些断言**整体不执行**，
 * 汇总会打印"0 项检查，0 项失败"并返回 0 —— **测试绿着，但什么都没验**。
 * 这种"空跑成功"比失败更危险：它会让一次真实的回归伪装成通过。
 *
 * 判据取自 lz_test_plan.c 的历史：早期 `LZ_TODO_PENDING` 把整块规格用例
 * 用 `#if 0` 关掉时，正是靠"检查项数为 0"才发现那些断言没跑。
 */
#define LZ_TEST_SUMMARY()                                                      \
    (printf("[=====] %d 项检查，%d 项失败\n", lz_test_checks, lz_test_failures),\
     (lz_test_checks == 0)                                                     \
         ? (fprintf(stderr, "[FAIL ] 一项检查都没跑 —— 断言被整体跳过了，"     \
                            "不能算通过\n"), 1)                                \
         : (lz_test_failures == 0 ? 0 : 1))

#endif /* LZ_TEST_H */

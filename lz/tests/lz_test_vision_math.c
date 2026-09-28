/**
 * @file lz_test_vision_math.c
 * @brief 像素 ↔ 角度的纯数学测试（步骤 2 与 步骤 4）。
 *
 * ## 为什么单独一条测试，且**两个视觉后端下都跑**
 *
 * 这两个函数是"视觉调俯仰"整条链路里**唯一完全可桌面验证**的部分 ——
 * 不依赖 PSDK、不依赖图像、不依赖机型。把它们挂在
 * `lz_test_vision`（只在 hsv 后端注册）上，会让"今天选 stub 后端"
 * 变成"这两条也测不到"。见 `lz_vision_shared.c` 的文件头。
 *
 * ## 本文件守的三件事
 *
 * 1. **DFOV ≠ HFOV ≠ VFOV** —— 把 82° 当水平 FOV 用会错 11°，
 *    而那个错在画面上只表现为"杆偏了一点"。用例里两种算法都算一遍，
 *    断言它们**确实不同**且正确值对上规格推导。
 * 2. **宽高比必须现算** —— 同一台相机换个分辨率，VFOV 就不一样。
 * 3. **符号** —— 画面偏下 ⇒ 下压（负）；偏上 ⇒ 上仰（正）。
 *    符号反了不会崩，只会让相机越调越偏，而这是最难看出来的那类错。
 */

#include "lz_vision.h"
#include "lz_test.h"

#include <math.h>

/* ⚠️ 这些期望值来自**规格推导**（DJI 官网 M4T：广角 DFOV 82°、
 * 传感器 8064×6048 = 4:3），不是"函数自己跑出来的结果" ——
 * 拿输出当期望是循环论证，测不出东西。推导过程见 lz_vision.h 的注释。 */
#define M4T_WIDE_DFOV_DEG   82.0
#define M4T_MID_DFOV_DEG    35.0
#define M4T_TELE_DFOV_DEG   15.0

int main(void)
{
    LZ_CASE("垂直 FOV：4:3 + 82° 应为 55.1°（照着 lz_vision.h 的推导手算）");
    {
        /* tan(VFOV/2) = tan(41°) · 3/5 = 0.869287 · 0.6 = 0.521572
         * VFOV = 2·atan(0.521572) = 55.098° */
        const double v = LzVision_VerticalFovDeg(8064, 6048, M4T_WIDE_DFOV_DEG);
        LZ_CHECK(isfinite(v));
        LZ_CHECK_NEAR(v, 55.098, 0.05);

        /* 换个分辨率但保持 4:3，结果必须一样 —— VFOV 只与宽高比有关，
         * 与绝对像素数无关。这条守的是"别把像素数掺进取值"。 */
        LZ_CHECK_NEAR(LzVision_VerticalFovDeg(4000, 3000, M4T_WIDE_DFOV_DEG), v, 1e-9);
        LZ_CHECK_NEAR(LzVision_VerticalFovDeg(400, 300, M4T_WIDE_DFOV_DEG), v, 1e-9);
        LZ_CHECK_NEAR(LzVision_VerticalFovDeg(4, 3, M4T_WIDE_DFOV_DEG), v, 1e-9);
    }

    LZ_CASE("垂直 FOV：三种宽高比下 VFOV < HFOV < DFOV 且单调");
    {
        /* 宽画面（16:9）：垂直方向占的比例更小 ⇒ VFOV 更小 */
        const double v169 = LzVision_VerticalFovDeg(1920, 1080, M4T_WIDE_DFOV_DEG);
        const double v43  = LzVision_VerticalFovDeg(1920, 1440, M4T_WIDE_DFOV_DEG);
        const double v11  = LzVision_VerticalFovDeg(1000, 1000, M4T_WIDE_DFOV_DEG);

        LZ_CHECK(isfinite(v169) && isfinite(v43) && isfinite(v11));

        /* 16:9 的垂直视场比 4:3 窄 */
        LZ_CHECK(v169 < v43);
        /* 正方形时 VFOV == HFOV，且都小于 DFOV
         * 2·atan(tan(41°)·(1/√2)) = 2·atan(0.61468) = 63.156° */
        LZ_CHECK_NEAR(v11, 63.156, 0.05);
        LZ_CHECK(v11 < M4T_WIDE_DFOV_DEG);
        /* 三者都小于 DFOV —— 这是"DFOV 是其中最大的那个"的固化 */
        LZ_CHECK(v169 < M4T_WIDE_DFOV_DEG);
        LZ_CHECK(v43  < M4T_WIDE_DFOV_DEG);
    }

    LZ_CASE("垂直 FOV：把 DFOV 当水平 FOV 用会错 11°（反向验证的靶子）");
    {
        /* ⚠️ 这条用例的写法刻意是"两种算法都算，断言它们**不同**，
         * 且只有一个对上规格"。这样将来若有人把实现改成错误那种，
         * 不是"少一条断言"而是**这条直接变红**（见文件末尾的反向验证记录）。 */
        const double correct = LzVision_VerticalFovDeg(8064, 6048, M4T_WIDE_DFOV_DEG);

        /* 错误算法：把 82° 当水平 FOV，tan(VFOV/2) = tan(41°)·3/4 */
        const double wrongHalfV = tan(41.0 * M_PI / 180.0) * 3.0 / 4.0;
        const double wrong = 2.0 * atan(wrongHalfV) * 180.0 / M_PI;

        LZ_CHECK_NEAR(wrong, 66.17, 0.05);          /* 错的那个确实是 66.2° */
        LZ_CHECK(fabs(correct - wrong) > 10.0);     /* 两者差 11°，不是"差不多" */
        LZ_CHECK(correct < wrong);                  /* 正确值更小 */
    }

    LZ_CASE("垂直 FOV：三个镜头的 VFOV 相差 5 倍（所以必须知道取到哪个）");
    {
        const double wide = LzVision_VerticalFovDeg(1920, 1080, M4T_WIDE_DFOV_DEG);
        const double mid  = LzVision_VerticalFovDeg(1920, 1080, M4T_MID_DFOV_DEG);
        const double tele = LzVision_VerticalFovDeg(1920, 1080, M4T_TELE_DFOV_DEG);

        LZ_CHECK(wide > mid && mid > tele);
        /* 用同一个画面上的偏差去折算角度，分母差 5 倍 ⇒ 角度差 5 倍。
         * 这一条把"必须先用探针确认是哪个镜头"变成可量化的后果。 */
        LZ_CHECK(wide / tele > 4.0);
    }

    LZ_CASE("像素→角度：居中时不转，且与像素偏差单调");
    {
        const double vfov = LzVision_VerticalFovDeg(1920, 1080, M4T_WIDE_DFOV_DEG);

        /* 已经在正中 ⇒ 不需要转。这条最容易写成"返回 0.5 对应的角"。 */
        LZ_CHECK_NEAR(LzVision_PixelOffsetToDeg(0.5, vfov), 0.0, 1e-12);

        /* 单调：目标越靠下（v 越大），需要下压越多（越负） */
        const double a = LzVision_PixelOffsetToDeg(0.55, vfov);
        const double b = LzVision_PixelOffsetToDeg(0.70, vfov);
        const double c = LzVision_PixelOffsetToDeg(0.95, vfov);
        LZ_CHECK(a < 0.0 && b < a && c < b);
    }

    LZ_CASE("像素→角度：符号 —— 偏上=上仰(正)，偏下=下压(负)");
    {
        const double vfov = 55.0;
        LZ_CHECK(LzVision_PixelOffsetToDeg(0.25, vfov) > 0.0);   /* 目标在上方 */
        LZ_CHECK(LzVision_PixelOffsetToDeg(0.75, vfov) < 0.0);   /* 目标在下方 */

        /* 上下对称：v=0.5±d 得到等值反号的角 */
        for (double d = 0.05; d < 0.5; d += 0.05) {
            const double up = LzVision_PixelOffsetToDeg(0.5 - d, vfov);
            const double dn = LzVision_PixelOffsetToDeg(0.5 + d, vfov);
            LZ_CHECK_NEAR(up, -dn, 1e-12);
        }
    }

    LZ_CASE("像素→角度：线性近似的误差在 v≈0.78 最大、在边缘为 0");
    {
        const double vfov = 55.1;   /* 4:3 + 82° DFOV，M4T 广角 */

        /* ⚠️ 本用例守的是一个**反直觉**的事实，它同时更正了文档里一处错说法：
         * 早期写"线性近似在画面边缘差约 3°"，实测**边缘差恰好为 0**
         * （v=1.0 时两种算法都给半个 VFOV），最大误差出现在 **v≈0.78**。 */
        const double exactEdge = LzVision_PixelOffsetToDeg(1.0, vfov);
        LZ_CHECK_NEAR(fabs(exactEdge), vfov * 0.5, 1e-9);   /* 边缘 = 半个 VFOV */
        LZ_CHECK_NEAR((1.0 - 0.5) * vfov, fabs(exactEdge), 1e-9);   /* 线性在此也对上 */

        /* 最大误差：实测 55.1° 下 0.857°，出现在 |v − 0.5| ≈ 0.279 处。
         *
         * ⚠️ **它是对称的两点**（v ≈ 0.221 与 v ≈ 0.779）—— 因为误差只取决于
         * |2v−1|。我第一版只断言 v≈0.78，而循环取到的是**前一个**（0.221，
         * 因为用了严格大于），于是红了。改成断言"偏离中心的距离"就没有这个
         * 取哪边的歧义 —— 这也更贴近事实：误差是 |2v−1| 的函数，不是 v 的。 */
        double worst = 0.0, worstOffset = 0.0;
        for (int i = 0; i <= 1000; ++i) {
            const double v = (double)i / 1000.0;
            const double d = fabs(LzVision_PixelOffsetToDeg(v, vfov) - (-(v - 0.5) * vfov));
            if (d > worst) { worst = d; worstOffset = fabs(v - 0.5); }
        }
        LZ_CHECK_NEAR(worst, 0.86, 0.02);            /* 不是 3° —— 差一个数量级 */
        LZ_CHECK_NEAR(worstOffset, 0.279, 0.005);    /* |v−0.5| 最大处 */

        /* 对称性：同样偏离中心、一上一下，误差相同 */
        LZ_CHECK_NEAR(fabs(LzVision_PixelOffsetToDeg(0.221, vfov) - (-(0.221 - 0.5) * vfov)),
                      fabs(LzVision_PixelOffsetToDeg(0.779, vfov) - (-(0.779 - 0.5) * vfov)),
                      1e-12);

        /* 长焦档下这个误差小到无关紧要 —— 所以"统一用 atan"的理由
         * 主要是广角档，这一点也固化下来 */
        const double vfovTele = 12.5;   /* 16:9 + 15° DFOV */
        double worstTele = 0.0;
        for (int i = 0; i <= 1000; ++i) {
            const double v = (double)i / 1000.0;
            const double d = fabs(LzVision_PixelOffsetToDeg(v, vfovTele) - (-(v - 0.5) * vfovTele));
            if (d > worstTele) { worstTele = d; }
        }
        LZ_CHECK(worstTele < 0.02);
    }

    LZ_CASE("反向验证：用错公式时上面那些断言会红");
    {
        /* 把"错误算法"当成被测对象跑一遍，确认它**确实**与本实现的输出
         * 不符 —— 这是"反向验证"的可执行版本：不需要改源码再跑，
         * 断言本身就把两种算法区分开了。
         *
         * （真正的反向验证另做：把 lz_vision_shared.c 里的 `/ diag` 改成
         * `/ w`，看 ctest 是否变红 —— 记录在 commit message 里。） */
        const double w = 1920.0, h = 1080.0;
        const double diag = sqrt(w * w + h * h);

        const double halfDiag = 41.0 * M_PI / 180.0;
        const double right = 2.0 * atan(tan(halfDiag) * (h / diag)) * 180.0 / M_PI;
        const double asHfov = 2.0 * atan(tan(halfDiag) * (h / w)) * 180.0 / M_PI;

        LZ_CHECK(fabs(right - asHfov) > 3.0);   /* 16:9 下差 3.4°（4:3 下差 11°） */
        LZ_CHECK_NEAR(LzVision_VerticalFovDeg(1920, 1080, M4T_WIDE_DFOV_DEG), right, 1e-9);
    }

    LZ_CASE("退化输入：越界与非法值返回 NAN，不是「随便一个数」");
    {
        /* ⚠️ 返回 NAN 而不是 0 或 ULONG_MAX 之类的哨兵：NAN 会污染
         * 后续运算（下游若忘了检查，NAN 会一路传下去；而 0 会被当成
         * "不需要转"，静默给出错误结论）。这是本项目"负值哨兵"那课的
         * 同一个道理。 */
        LZ_CHECK(isnan(LzVision_VerticalFovDeg(0, 100, 82.0)));
        LZ_CHECK(isnan(LzVision_VerticalFovDeg(100, 0, 82.0)));
        LZ_CHECK(isnan(LzVision_VerticalFovDeg(-10, 100, 82.0)));
        LZ_CHECK(isnan(LzVision_VerticalFovDeg(100, 100, 0.0)));
        LZ_CHECK(isnan(LzVision_VerticalFovDeg(100, 100, -5.0)));
        LZ_CHECK(isnan(LzVision_VerticalFovDeg(100, 100, 180.0)));
        LZ_CHECK(isnan(LzVision_VerticalFovDeg(100, 100, 200.0)));
        LZ_CHECK(isnan(LzVision_VerticalFovDeg(100, 100, NAN)));

        LZ_CHECK(isnan(LzVision_PixelOffsetToDeg(NAN, 55.0)));
        LZ_CHECK(isnan(LzVision_PixelOffsetToDeg(0.5, NAN)));
        LZ_CHECK(isnan(LzVision_PixelOffsetToDeg(0.5, 0.0)));
        LZ_CHECK(isnan(LzVision_PixelOffsetToDeg(0.5, -10.0)));
        LZ_CHECK(isnan(LzVision_PixelOffsetToDeg(0.5, 180.0)));

        /* v 超出 [0,1] 不算非法（画面外的目标也应有确定的角），
         * 但要 finite —— 这是"不自作主张拒绝"的边界的固化。 */
        LZ_CHECK(isfinite(LzVision_PixelOffsetToDeg(-0.2, 55.0)));
        LZ_CHECK(isfinite(LzVision_PixelOffsetToDeg(1.3, 55.0)));
    }

    LZ_CASE("整链：从 DFOV 与像素偏差一路算到 thetaStar");
    {
        /* 模拟一次真实测定：
         *   画面 1920×1080、镜头 DFOV 82°（**假设**是广角，由探针确认）、
         *   当前云台俯仰 −80.0°、目标中点出现在 v = 0.68（偏下） */
        const double vfov = LzVision_VerticalFovDeg(1920, 1080, M4T_WIDE_DFOV_DEG);
        const double thetaNow = -80.0;
        const double vMid = 0.68;

        const double delta = LzVision_PixelOffsetToDeg(vMid, vfov);
        const double thetaStar = thetaNow + delta;

        /* 目标偏下 ⇒ 要下压 ⇒ 更负 */
        LZ_CHECK(delta < 0.0);
        LZ_CHECK(thetaStar < thetaNow);

        /* 量级要合理：v 偏 0.18，16:9 下 82° DFOV 给出的 VFOV 是 46.17°，
         * 折算 2·atan 之差 ≈ 8.7°。断言区间而不是精确值 —— 精确值取决于
         * VFOV，而 VFOV 取决于探针还没答出的镜头。
         * 区间断言在这里更有信息量：它同时守住"没算成 0"和"没算成 180"。 */
        LZ_CHECK(delta < -8.0 && delta > -10.0);
    }


    LZ_CASE("变焦折算：半角正切按倍数缩小，且 1.0X 是恒等");
    {
        /* ★ 这条守的是 2026-09-28 现场实测的那个 **7 倍**误差。
         * 现场用 Pilot 2 变焦到 7.0X，而代码里写死"DFOV=82°"（广角端）。
         * 反推的真实垂直视场角是 8.55°，与按下式折算出的 8.52° 吻合：
         *
         *     tan(DFOV_zoomed/2) = tan(DFOV_wide/2) / zoomFactor
         *
         * ⚠️ **不能直接对角度做除法**（82/7 = 11.7°，差 4°）。
         * 下面第 3 条断言就是守这个：线性折算会给出 11.71，
         * 而正确值 15.87 —— 差得足够远，反向验证时必然变红。 */
        const double wide = M4T_WIDE_DFOV_DEG;   /* 82.0 */

        /* 1. 1.0X 是恒等 */
        LZ_CHECK_NEAR(LzVision_ZoomedDiagFovDeg(wide, 1.0), wide, 1e-9);

        /* 2. 变焦只会**缩小**视场（单调） */
        double prev = wide;
        for (double z = 2.0; z <= 64.0; z *= 2.0) {
            const double cur = LzVision_ZoomedDiagFovDeg(wide, z);
            LZ_CHECK(isfinite(cur));
            LZ_CHECK(cur > 0.0 && cur < prev);
            prev = cur;
        }

        /* 3. 7.0X 下必须落在实测反推值附近，且**不是**线性折算值 */
        const double at7 = LzVision_ZoomedDiagFovDeg(wide, 7.0);
        LZ_CHECK_NEAR(at7, 14.158, 0.05);            /* tan(41°)/7 → 14.158° */
        /* 线性折算（82/7 = 11.71°）差 2.45° —— 足够把两者区分开 */
        LZ_CHECK(fabs(at7 - wide / 7.0) > 2.0);

        /* 4. 与 VerticalFovDeg 串起来，得出实测的 8.5° 量级 */
        const double vfov7 = LzVision_VerticalFovDeg(1440, 1080, at7);
        /* 8.5225° —— 与现场日志反推的 8.55° 吻合（差 0.03°） */
        LZ_CHECK_NEAR(vfov7, 8.523, 0.1);

        /* 5. 俯仰增益也跟着变 —— 这是误差真正伤人的地方。
         *
         * ⚠️ **方向（"谁大"）值得写下来，第一版写反过**：
         * 视场窄了，同一个像素偏差对应的角度**更小**，所以
         *   `degPerV(变焦) < degPerV(广角)`。
         * 写死广角模型 = **每轮多转 7 倍**，一步把目标甩出画面
         * （现场 18:17:16 那轮：代码算出 3.29°，正解 0.47°）。
         *
         * 换个说法：`dv/dθ`（每度云台转动画面上移动多少）在变焦下**更大**，
         * 而 `dθ/dv`（要让目标走这么多像素需要转多少度）**更小**。
         * 这两个互为倒数，很容易在断言里写反 —— 所以下面两条都断言。 */
        const double gainWide = fabs(LzVision_PixelOffsetToDeg(0.7, 55.09));
        const double gain7    = fabs(LzVision_PixelOffsetToDeg(0.7, vfov7));
        LZ_CHECK(gain7 < gainWide);
        LZ_CHECK_NEAR(gainWide / gain7, 6.90, 0.2);   /* 现场实测比值 */

        /* 6. 复现现场那一轮：v=0.5551 时
         *     广角模型算出 3.29°（就是日志里那个数）
         *     变焦 7× 正解只有 0.47° */
        LZ_CHECK_NEAR(fabs(LzVision_PixelOffsetToDeg(0.5551, 55.0906)), 3.29, 0.05);
        LZ_CHECK_NEAR(fabs(LzVision_PixelOffsetToDeg(0.5551, vfov7)),     0.47, 0.05);

        /* 7. 退化输入 */
        LZ_CHECK(isnan(LzVision_ZoomedDiagFovDeg(0.0, 7.0)));
        LZ_CHECK(isnan(LzVision_ZoomedDiagFovDeg(180.0, 7.0)));
        LZ_CHECK(isnan(LzVision_ZoomedDiagFovDeg(82.0, 0.0)));
        LZ_CHECK(isnan(LzVision_ZoomedDiagFovDeg(82.0, -3.0)));
        LZ_CHECK(isnan(LzVision_ZoomedDiagFovDeg(82.0, NAN)));
        LZ_CHECK(isnan(LzVision_ZoomedDiagFovDeg(NAN, 7.0)));
    }

    return LZ_TEST_SUMMARY();
}

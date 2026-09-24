#!/usr/bin/env node
/**
 * @file gen_vision_testdata.js
 * @brief 从真实俯拍照片生成视觉检测的回归测试数据（PPM + 黄金值）。
 *
 * ## 为什么要有这个工具
 *
 * `lz_vision` 的检测算法（红色阈值 + 连通域 + 杆的对比度滤波）需要调参，
 * 而**在设备上盲调是没有反馈的**：飞机飞起来才知道认没认对，代价太高。
 * 这个脚本把真实照片固化成测试数据，让 C 实现能在桌面上反复跑 ——
 * 与 `lz_core` 能在桌面上测是同一个理由。
 *
 * ## 期望值从哪来（这条最要紧）
 *
 * **不是**"算法自己跑出来的结果" —— 那是循环论证，测不出任何东西。
 * 期望值来自**全图**检测 + 人工目视核对：6 张图上检测到的杆列都逐张
 * 确认过黄线确实压在杆上，本脚本把那些**全图坐标纯平移**到裁剪坐标。
 *
 * ⚠️ 已知偏差：裁剪前后杆列检测值相差 1–2 px（ROI 被 clamp 到图边的
 * 边界效应）。这是真实的，所以黄金值按全图给、测试容差取 ±2 px ——
 * 不掩盖它，否则将来真偏移 2 px 就看不出来了。
 *
 * ## 依赖
 *
 * `jpeg-js`（纯 JS，`npm i jpeg-js`）。**故意不依赖 PIL/OpenCV**：
 * 生成测试数据不该给工程引入新的构建依赖。输出 PPM 也是零依赖的选择 ——
 * C 侧读 PPM 只要 20 行，而 PNG 要引 zlib。
 *
 * 用法：node lz/tools/gen_vision_testdata.js <照片目录> <输出目录>
 */

const fs = require('fs');
const path = require('path');
const jpeg = require('jpeg-js');

/**
 * 检测参数 —— **必须与 C 侧 `LzVisionConfig` 默认值逐项一致**。
 * 不一致的话，本脚本生成的期望值和 C 实现跑出来的就对不上，
 * 而那种失配会被误读成"算法写错了"。
 */
const CFG = {
    redHueLowMax: 10,           /*!< 低段红 H ∈ [0,10] */
    redHueHighMin: 170,         /*!< 高段红 H ∈ [170,180] */
    minSaturation: 100,
    minValue: 60,
    minBlobArea: 200,           /*!< 连通域最小面积（像素） */
    poleRoiMarginX: 70,         /*!< 杆列扫描 ROI：旗 bbox 左右各扩这么多像素 */
    poleContrastOffset: 9,      /*!< 对比度滤波的邻域偏移 */
    poleContrastThreshold: 34,  /*!< 与左右邻域的最小亮度差 */
    poleHalfWidth: 1,           /*!< 判定"该行属于杆"时允许的 x 偏移 */
    poleGap: 40,                /*!< 竖向延伸允许的最大间断（行） */

    /* ---- 打分行窗（**裁剪不变性的关键**）----
     *
     * 杆列是谁，靠"这一列有多少行满足对比度判据"来投票。**投票窗口必须
     * 只取旗附近这一段**，不能取整幅图：
     *
     * 实测（2026-09-24）整幅图投票时，把同一张照片裁成小图（行数 960→321）
     * 会让某些列的票数变化，杆列判定因此**偏移 6 px**（flag3）——
     * 也就是说判定结果依赖于"图有多大"，而不是"画面里有什么"。
     * 窗口收窄到旗附近后，全图与裁剪图给出同一个答案（0 px）。
     *
     * 业务上也更对：杆的证据应当来自目标附近，画面底部的绿篱边缘、
     * 铺装接缝不该参与投票。
     */
    poleWinAbove: 20,           /*!< 投票窗口上界 = 旗 bbox 顶 - 这个值 */
    poleWinBelow: 300,          /*!< 投票窗口下界 = 旗 bbox 顶 + 这个值 */
};

/* ============================ 图像算法 ============================
 * 以下与 C 实现（src/lz_vision.c）是**同一套算法**。
 * 改动必须两侧同步，否则黄金值失去意义。
 */

/** RGB → HSV。H 折半到 [0,180)（为塞进 1 字节，与 OpenCV 同约定） */
function rgb2hsv(r, g, b) {
    const mx = Math.max(r, g, b), mn = Math.min(r, g, b), d = mx - mn;
    let h = 0;
    if (d !== 0) {
        if (mx === r) h = 60 * ((g - b) / d);
        else if (mx === g) h = 60 * (2 + (b - r) / d);
        else h = 60 * (4 + (r - g) / d);
        if (h < 0) h += 360;
    }
    return [h / 2, mx === 0 ? 0 : 255 * d / mx, mx];
}

/** 步骤 1：红色二值掩码（双区间：红在色环两端各有一段） */
function buildMask(D, W, H) {
    const m = new Uint8Array(W * H);
    for (let y = 0; y < H; y++) {
        for (let x = 0; x < W; x++) {
            const i = (y * W + x) * 4;
            const [h, s, v] = rgb2hsv(D[i], D[i + 1], D[i + 2]);
            if ((h <= CFG.redHueLowMax || h >= CFG.redHueHighMin) &&
                s >= CFG.minSaturation && v >= CFG.minValue) {
                m[y * W + x] = 1;
            }
        }
    }
    return m;
}

/** 步骤 2：连通域标记（两遍扫描 + 并查集，8 邻域），按面积降序 */
function labelBlobs(m, W, H) {
    const lb = new Int32Array(W * H).fill(-1);
    const par = [0];
    const find = a => { while (par[a] !== a) a = par[a] = par[par[a]]; return a; };
    const uni = (a, b) => { a = find(a); b = find(b); if (a !== b) par[b] = a; };
    let next = 1;
    for (let y = 0; y < H; y++) {
        for (let x = 0; x < W; x++) {
            const i = y * W + x;
            if (!m[i]) continue;
            const nb = [];
            if (x > 0 && m[i - 1]) nb.push(lb[i - 1]);
            if (y > 0) {
                if (m[i - W]) nb.push(lb[i - W]);
                if (x > 0 && m[i - W - 1]) nb.push(lb[i - W - 1]);
                if (x < W - 1 && m[i - W + 1]) nb.push(lb[i - W + 1]);
            }
            if (!nb.length) { par[next] = next; lb[i] = next++; }
            else { const mn = Math.min(...nb); lb[i] = mn; for (const n of nb) uni(mn, n); }
        }
    }
    const st = new Map();
    for (let i = 0; i < W * H; i++) {
        if (!m[i]) continue;
        const r = find(lb[i]);
        let s = st.get(r);
        if (!s) { s = { n: 0, minX: 1e9, maxX: -1, minY: 1e9, maxY: -1 }; st.set(r, s); }
        const y = (i / W) | 0, x = i - y * W;
        s.n++;
        if (x < s.minX) s.minX = x;
        if (x > s.maxX) s.maxX = x;
        if (y < s.minY) s.minY = y;
        if (y > s.maxY) s.maxY = y;
    }
    return [...st.values()].sort((a, b) => b.n - a.n);
}

/**
 * 步骤 3：杆列检测（局部对比度竖线滤波）
 *
 * ## 为什么不直接用旗 bbox 的几何中心
 *
 * 实测 6 张真实照片：旗 bbox 中心与真实杆位相差 **−3.7% ~ +4.1% 画面宽**，
 * 且**符号随风向翻转**（风把旗吹向一侧时杆在另一侧）。这是"半个旗宽取决于
 * 风向"的必然结果 —— **不是常数偏差，标定不掉**。
 *
 * ## 判据为什么是"与左右邻域的亮度差"
 *
 * 杆是一根细长竖线：压在绿篱/水面上时比周围**亮**，压在浅色铺装上时比
 * 周围**暗**。两个方向都算，因此不依赖背景明暗。
 *
 * 实测（6 图 × 3 种邻域 × 3 种阈值 = 54 组参数):杆列位置的跨度只有 0–2 px。
 */
function detectPoleColumn(D, W, H, flag) {
    const off = CFG.poleContrastOffset, th = CFG.poleContrastThreshold;
    /* ROI：旗 bbox 左右各扩 margin，且保证 ±off 不越界 */
    const x0 = Math.max(off, flag.minX - CFG.poleRoiMarginX);
    const x1 = Math.min(W - 1 - off, flag.maxX + CFG.poleRoiMarginX);
    /* 打分行窗：只取旗附近这一段，理由见 CFG.poleWinAbove 的注释 */
    const wy0 = Math.max(0, flag.minY - CFG.poleWinAbove);
    const wy1 = Math.min(H - 1, flag.minY + CFG.poleWinBelow);

    const V = new Int32Array(W * H);
    for (let i = 0; i < W * H; i++) V[i] = Math.max(D[i * 4], D[i * 4 + 1], D[i * 4 + 2]);

    const on = new Uint8Array(W * H);
    const score = new Int32Array(W);
    for (let x = x0; x <= x1; x++) {
        for (let y = wy0; y <= wy1; y++) {
            const v = V[y * W + x], l = V[y * W + x - off], r = V[y * W + x + off];
            if ((v - l > th && v - r > th) || (l - v > th && r - v > th)) {
                on[y * W + x] = 1;
                score[x]++;
            }
        }
    }
    let bx = x0, bs = -1;
    for (let x = x0; x <= x1; x++) if (score[x] > bs) { bs = score[x]; bx = x; }

    /* 竖向延伸：命中行数最多的一段（允许 poleGap 行间断）。
     * 同样只在打分行窗内找 —— 窗外的长竖线（绿篱边缘、铺装接缝）不是杆。 */
    let best = null, start = -1, last = -1;
    for (let y = wy0; y <= wy1; y++) {
        let hit = false;
        for (let k = -CFG.poleHalfWidth; k <= CFG.poleHalfWidth; k++) {
            const xx = bx + k;
            if (xx >= 0 && xx < W && on[y * W + xx]) { hit = true; break; }
        }
        if (hit) { if (start < 0) start = y; last = y; }
        else if (start >= 0 && y - last > CFG.poleGap) {
            const len = last - start;
            if (!best || len > best.len) best = { top: start, bot: last, len };
            start = -1;
        }
    }
    if (start >= 0) {
        const len = last - start;
        if (!best || len > best.len) best = { top: start, bot: last, len };
    }
    return { x: bx, score: bs, run: best };
}

/** 完整检测：旗（面积最大的合格红色连通域）+ 杆列 */
function detect(D, W, H) {
    const blobs = labelBlobs(buildMask(D, W, H), W, H)
        .filter(b => b.n >= CFG.minBlobArea);
    if (!blobs.length) return null;
    const flag = blobs[0];
    return { flag, second: blobs[1] ? blobs[1].n : 0, pole: detectPoleColumn(D, W, H, flag) };
}

/* ============================ 输出 ============================ */

/** 写 PPM P6 —— C 侧读它只要 20 行，且不需要任何压缩库 */
function writePPM(file, W, H, D) {
    const buf = Buffer.alloc(W * H * 3);
    for (let i = 0; i < W * H; i++) {
        buf[i * 3] = D[i * 4];
        buf[i * 3 + 1] = D[i * 4 + 1];
        buf[i * 3 + 2] = D[i * 4 + 2];
    }
    fs.writeFileSync(file, Buffer.concat([Buffer.from(`P6\n${W} ${H}\n255\n`), buf]));
}

const clamp = (v, a, b) => (v < a ? a : v > b ? b : v);

/* ============================ 主流程 ============================ */

const srcDir = process.argv[2];
const outDir = process.argv[3];
if (!srcDir || !outDir) {
    console.error(`用法: node ${path.relative(process.cwd(), process.argv[1])} <照片目录> <输出目录>`);
    process.exit(2);
}
fs.mkdirSync(outDir, { recursive: true });

const files = fs.readdirSync(srcDir).filter(f => /\.jpe?g$/i.test(f)).sort();
if (!files.length) {
    console.error(`目录里没有 JPEG: ${srcDir}`);
    process.exit(2);
}

/* 裁剪边距：保证 ROI（flag ± margin）与对比度邻域（± offset）都完整落在图内，
   再多留 25 px 余量 —— 少了会让 ROI 被 clamp 到图边、改变检测结果。 */
const PAD = CFG.poleRoiMarginX + CFG.poleContrastOffset + 25;

const golden = [];
let total = 0;

for (const [idx, f] of files.entries()) {
    const raw = jpeg.decode(fs.readFileSync(path.join(srcDir, f)), { useTArray: true });
    const { width: W, height: H, data: D } = raw;

    /* ---- 全图上检测：**真值来源**（已逐张目视核对） ---- */
    const full = detect(D, W, H);
    if (!full) {
        console.error(`${f}: 全图未检出红色目标，跳过`);
        continue;
    }

    /* ---- 裁剪成 320 行左右的小图，压缩仓库体积 ---- */
    const x0 = clamp(full.flag.minX - PAD, 0, W - 1);
    const x1 = clamp(full.flag.maxX + PAD, 0, W - 1);
    const y0 = clamp(full.flag.minY - 20, 0, H - 1);
    const y1 = clamp(full.flag.minY + 300, 0, H - 1);
    const cw = x1 - x0 + 1, ch = y1 - y0 + 1;

    const cd = Buffer.alloc(cw * ch * 4);
    for (let y = 0; y < ch; y++) {
        for (let x = 0; x < cw; x++) {
            const si = ((y0 + y) * W + (x0 + x)) * 4, di = (y * cw + x) * 4;
            cd[di] = D[si]; cd[di + 1] = D[si + 1]; cd[di + 2] = D[si + 2]; cd[di + 3] = 255;
        }
    }
    const name = `flag${idx + 1}.ppm`;
    writePPM(path.join(outDir, name), cw, ch, cd);
    total += fs.statSync(path.join(outDir, name)).size;

    /* ---- 期望值 = 全图真值**平移**到裁剪坐标（纯平移，无重采样） ---- */
    const cropPoleX = detect(cd, cw, ch).pole.x;
    const poleX = full.pole.x - x0;

    golden.push({
        name, cw, ch,
        flag: {
            minX: full.flag.minX - x0, minY: full.flag.minY - y0,
            maxX: full.flag.maxX - x0, maxY: full.flag.maxY - y0,
            n: full.flag.n,
        },
        poleX, second: full.second,
        cropDeviation: Math.abs(cropPoleX - poleX),
    });

    console.log(`${name.padEnd(12)} ${String(cw + 'x' + ch).padStart(9)}` +
                `  旗面积 ${String(full.flag.n).padStart(6)}` +
                `  第2名 ${String(full.second).padStart(5)}` +
                `  杆x ${String(poleX).padStart(4)}` +
                `  (裁剪前后差 ${golden[golden.length - 1].cropDeviation}px)`);
}

const header =
    '# 视觉检测黄金值 —— 自动生成，勿手改\n' +
    '# 生成工具: lz/tools/gen_vision_testdata.js\n' +
    '# 期望值来源: 全图检测后逐张目视核对，再**纯平移**到裁剪坐标。\n' +
    '#   不是"裁剪后算法自己的输出"—— 那样是循环论证，测不出东西。\n' +
    '# 已知偏差: 裁剪前后杆列检测值差 1-2 px（ROI 被 clamp 到图边的边界效应），\n' +
    '#   故测试容差取 ±2 px。不掩盖它 —— 掩盖掉的话，将来真的偏移 2px 就看不出来了。\n' +
    '# 每行: 文件名 宽 高 旗minX 旗minY 旗maxX 旗maxY 旗面积 杆x 第2名面积\n';

fs.writeFileSync(path.join(outDir, 'vision_golden.txt'), header +
    golden.map(g => `${g.name} ${g.cw} ${g.ch} ${g.flag.minX} ${g.flag.minY} ` +
                    `${g.flag.maxX} ${g.flag.maxY} ${g.flag.n} ${g.poleX} ${g.second}`).join('\n') + '\n');

console.log(`\n已写出 ${golden.length} 个 PPM（${(total / 1024).toFixed(0)} KB）` +
            ` + vision_golden.txt → ${outDir}`);

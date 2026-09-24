#!/usr/bin/env node
/**
 * @file render_vision_check.js
 * @brief 把 `tests/data/vision_golden.txt` 的黄金值画成一张核对图（PNG）。
 *
 * ## 用途
 *
 * 黄金值是**人**核对出来的，所以必须能被人再过目一遍。本工具把每张小图
 * 与它的黄金框并排画出来：绿框 = 旗面外接框，黄线 = 杆列。
 * 看一眼就知道黄金值对不对 —— 比读一行数字可靠得多。
 *
 * ⚠️ 输出**不参与测试**，是给人看的。放在 `doc/` 下。
 *
 * 依赖：jpeg-js 不必要（输入已是 PPM），但 PNG 写出用 zlib（Node 内置）。
 *
 * 用法：node lz/tools/render_vision_check.js <数据目录> <输出.png>
 */

const fs = require('fs');
const path = require('path');
const zlib = require('zlib');

/** 读 PPM P6 */
function readPPM(file) {
    const b = fs.readFileSync(file);
    /* 头部是 "P6\n<w> <h>\n255\n"，用空白分词找数据起点 */
    let i = 0, tok = 0, vals = [];
    while (tok < 4 && i < b.length) {
        const c = b[i];
        if (c === 0x23) {            /* '#' 注释行 */
            while (i < b.length && b[i] !== 0x0a) i++;
            continue;
        }
        if (c === 0x20 || c === 0x0a || c === 0x09 || c === 0x0d) { i++; continue; }
        let s = '';
        while (i < b.length && ![0x20, 0x0a, 0x09, 0x0d].includes(b[i])) {
            s += String.fromCharCode(b[i++]);
        }
        vals.push(s);
        tok = vals.length;
    }
    i++;    /* 跳过分隔数据的那一个空白 */
    return { w: +vals[1], h: +vals[2], data: b.slice(i) };
}

/** 写 PNG（真彩无 alpha，无滤波） */
function writePNG(file, w, h, rgb) {
    const raw = Buffer.alloc((w * 3 + 1) * h);
    for (let y = 0; y < h; y++) {
        raw[y * (w * 3 + 1)] = 0;
        for (let x = 0; x < w * 3; x++) {
            raw[y * (w * 3 + 1) + 1 + x] = rgb[y * w * 3 + x];
        }
    }
    const idat = zlib.deflateSync(raw);
    const chunk = (type, data) => {
        const len = Buffer.alloc(4);
        len.writeUInt32BE(data.length);
        const t = Buffer.from(type);
        let c = ~0;
        for (const byte of Buffer.concat([t, data])) {
            c ^= byte;
            for (let k = 0; k < 8; k++) c = (c >>> 1) ^ (0xEDB88320 & -(c & 1));
        }
        const crc = Buffer.alloc(4);
        crc.writeUInt32BE((~c) >>> 0);
        return Buffer.concat([len, t, data, crc]);
    };
    const ihdr = Buffer.alloc(13);
    ihdr.writeUInt32BE(w, 0);
    ihdr.writeUInt32BE(h, 4);
    ihdr[8] = 8; ihdr[9] = 2;   /* 8 bit, truecolor RGB */
    fs.writeFileSync(file, Buffer.concat([
        Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]),
        chunk('IHDR', ihdr), chunk('IDAT', idat), chunk('IEND', Buffer.alloc(0)),
    ]));
}

const dataDir = process.argv[2];
const outFile = process.argv[3];
if (!dataDir || !outFile) {
    console.error(`用法: node ${process.argv[1]} <数据目录> <输出.png>`);
    process.exit(2);
}

const golden = fs.readFileSync(path.join(dataDir, 'vision_golden.txt'), 'utf8')
    .split('\n').filter(l => l && !l.startsWith('#'))
    .map(l => l.trim().split(/\s+/));

/* 黄金值每行: 名称 宽 高 旗minX 旗minY 旗maxX 旗maxY 面积 杆x 第2名面积 */
const items = golden.map(g => {
    const [name, , , fx0, fy0, fx1, fy1, , poleX] = g;
    return { name, im: readPPM(path.join(dataDir, name)),
             fx0: +fx0, fy0: +fy0, fx1: +fx1, fy1: +fy1, poleX: +poleX };
});

/* 横排，图间留 6 px 黑边 */
const GAP = 6;
const totalW = items.reduce((a, it) => a + it.im.w, 0) + GAP * (items.length + 1);
const totalH = Math.max(...items.map(it => it.im.h)) + GAP * 2;
const out = Buffer.alloc(totalW * totalH * 3, 25);

const px = (x, y, r, g, b) => {
    if (x < 0 || y < 0 || x >= totalW || y >= totalH) return;
    const i = (y * totalW + x) * 3;
    out[i] = r; out[i + 1] = g; out[i + 2] = b;
};

let ox = GAP;
for (const it of items) {
    const { w, h, data } = it.im;
    for (let y = 0; y < h; y++) {
        for (let x = 0; x < w; x++) {
            const s = (y * w + x) * 3;
            px(ox + x, GAP + y, data[s], data[s + 1], data[s + 2]);
        }
    }
    /* 绿框 = 旗面外接框 */
    for (let x = it.fx0; x <= it.fx1; x++) {
        px(ox + x, GAP + it.fy0, 0, 255, 0);
        px(ox + x, GAP + it.fy1, 0, 255, 0);
    }
    for (let y = it.fy0; y <= it.fy1; y++) {
        px(ox + it.fx0, GAP + y, 0, 255, 0);
        px(ox + it.fx1, GAP + y, 0, 255, 0);
    }
    /* 黄线 = 杆列（贯穿整幅高度） */
    for (let y = 0; y < h; y++) {
        px(ox + it.poleX, GAP + y, 255, 220, 0);
    }
    ox += w + GAP;
}

fs.mkdirSync(path.dirname(outFile), { recursive: true });
writePNG(outFile, totalW, totalH, out);
console.log(`已写出 ${outFile}（${totalW}x${totalH}）`);
console.log('绿框 = 旗面外接框；黄线 = 杆列（黄金值）。请目视核对黄线是否压在杆上。');

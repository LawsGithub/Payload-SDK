#!/usr/bin/env python3
"""对论文 A 的 300dpi 页面图逐页做 tesseract 中文 OCR，拼成全文。

为什么需要这个：这篇 PDF 的文字层是坏的（汉字被空格切断、全角化、
公式丢失），pdftotext / pymupdf 都只能拿到"看起来通顺、实际错漏"的文本。
实测 tesseract chi_sim 在这份扫描件上质量很好，公式也能还原出主体。

用法：
    python3 ocr_paper_a.py <图片目录> <输出txt>

支持断点续跑：已存在的分页 OCR 结果会跳过。
"""
import os
import subprocess
import sys

IMGDIR = sys.argv[1] if len(sys.argv) > 1 else "/tmp/ocrA"
OUT = sys.argv[2] if len(sys.argv) > 2 else "/home/law/papers/wt-inspection/text/A_liu_mengqi_OCR.txt"
CACHE = os.path.join(os.path.dirname(OUT), ".ocr_cache_A")
os.makedirs(CACHE, exist_ok=True)

pages = sorted(f for f in os.listdir(IMGDIR) if f.endswith(".png"))
print(f"共 {len(pages)} 页", flush=True)

chunks = []
for i, name in enumerate(pages, 1):
    n = name[1:4]
    cache = os.path.join(CACHE, f"{n}.txt")
    if not os.path.exists(cache):
        # psm 6 = 假定统一文本块；--oem 1 = LSTM
        r = subprocess.run(
            ["tesseract", os.path.join(IMGDIR, name), "-", "-l", "chi_sim",
             "--psm", "6", "--oem", "1"],
            capture_output=True, text=True,
        )
        if r.returncode != 0:
            print(f"  第 {n} 页 OCR 失败: {r.stderr.strip()[:200]}", flush=True)
            text = ""
        else:
            text = r.stdout
        with open(cache, "w", encoding="utf-8") as f:
            f.write(text)
    else:
        text = open(cache, encoding="utf-8").read()

    # tesseract 会在汉字间插空格，去掉（保留 ASCII 词间空格）
    lines = []
    for ln in text.splitlines():
        s = ln.strip()
        if s:
            lines.append(s.replace(" ", ""))
    chunks.append(f"\n\n===== PDF页 {int(n)} =====\n" + "\n".join(lines))
    if i % 10 == 0 or i == len(pages):
        print(f"  已完成 {i}/{len(pages)}", flush=True)

with open(OUT, "w", encoding="utf-8") as f:
    f.write("".join(chunks))
print(f"写入 {OUT}")

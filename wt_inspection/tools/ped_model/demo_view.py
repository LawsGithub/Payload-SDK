#!/usr/bin/env python3
"""行人检测效果网页：按设备端同一套预处理与解码跑模型，把框画出来；有标注时算漏检 / 误检。

推理用 ONNX Runtime 代替设备上的 TensorRT（同一份权重导出的 ONNX）；letterbox 与解码
直接调 app/wt_yolo_decode.c（编成动态库），所以框的位置与置信度和设备端一致
（engine 的 FP16 误差除外）。阈值与设备一致：推理 0.25、NMS 0.45、只收 pedestrian/people。

网页里还标出「此刻按下行人瞄准会打谁」：置信度 ≥ 0.40（ped_conf 默认值）的框里
离画面中心最近的那个 —— 与 wt_ped_logic 的捕获判据相同。

用法（在 wt_inspection/ 下）：

    python3 -I tools/ped_model/demo_view.py <best.onnx> <图片...> --out view.html \\
        [--gt samples.json]   # FiftyOne 格式标注（如 HF Voxel51/VisDrone2019-DET 的 samples.json）

没有标注的图（例如自己飞的照片）也能用：只画检测框和瞄准目标，不算召回率。
"""

import argparse
import base64
import ctypes
import datetime
import json
import os
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True   # 别在工具目录里留 __pycache__
import crosscheck_decode as cc  # noqa: E402  （它会补 user site 并关掉 ultralytics 自动安装）

import numpy as np  # noqa: E402

ACQUIRE_CONF = 0.40       # ped_conf 默认值：新捕获一个行人所需的置信度
MATCH_IOU = 0.5           # 与标注配对的 IoU 门限（VisDrone / COCO 的 AP50 口径）
PERSON_LABELS = ("pedestrians", "pedestrian", "people")
IGNORE_LABELS = ("ignore_regions", "ignored_regions")
# 身高分档（网络输入 1280 上的像素）。M4T 广角 1440×1080 取流缩到 1280（×0.889），
# 60 m 外的行人约 23 px 高 → 网络输入约 20 px。距离按反比粗估，取决于俯视角。
SIZE_BINS = [(0, 8), (8, 12), (12, 16), (16, 24), (24, 32), (32, 48), (48, 1e9)]
PX_AT_60M = 20.0


def load_gt(path):
    if not path:
        return {}
    data = json.load(open(path, encoding="utf-8"))
    samples = data["samples"] if isinstance(data, dict) else data
    out = {}
    for s in samples:
        out[os.path.basename(s["filepath"])] = s
    return out


def gt_boxes(sample, w, h):
    persons, ignores = [], []
    for d in sample["ground_truth"]["detections"]:
        x, y, bw, bh = d["bounding_box"]
        box = [x * w, y * h, (x + bw) * w, (y + bh) * h]
        if d["label"] in PERSON_LABELS:
            persons.append(box + [d["label"], d.get("occlusion", 0)])
        elif d["label"] in IGNORE_LABELS:
            ignores.append(box)
    return persons, ignores


def evaluate(dets, persons, ignores):
    """按置信度从高到低贪心配对。返回每个检测的判定与每个标注是否被检出。"""
    order = sorted(range(len(dets)), key=lambda i: -dets[i][4])
    det_tag = ["fp"] * len(dets)
    det_match = [-1] * len(dets)
    gt_hit = [-1] * len(persons)
    for i in order:
        best, bj = MATCH_IOU, -1
        for j, g in enumerate(persons):
            if gt_hit[j] >= 0:
                continue
            v = cc.iou(dets[i][:4], g[:4])
            if v >= best:
                best, bj = v, j
        if bj >= 0:
            gt_hit[bj] = i
            det_tag[i] = "tp"
            det_match[i] = bj
            continue
        cx, cy = (dets[i][0] + dets[i][2]) / 2, (dets[i][1] + dets[i][3]) / 2
        if any(r[0] <= cx <= r[2] and r[1] <= cy <= r[3] for r in ignores):
            det_tag[i] = "ign"   # 落在标注的「忽略区」里：按 VisDrone 规则不计误检
    return det_tag, det_match, gt_hit


def py_decode(out, lb, cap=None):
    """numpy 复刻 WtYolo_Decode（同样的 argmax→筛类别→降序→不分类贪心 NMS），但可以不截断。

    设备端每帧最多留 WT_TRT_MAX_DETECTIONS（64）个框，人群图的召回率会被截掉；
    这个复刻版用来看「模型本身」能认出多少。main 里会核对它截到 64 时与 C 逐框一致。
    """
    o = out[0]
    scores = o[4:]
    cls = scores.argmax(axis=0)            # 并列取下标小的，与 C 的严格大于一致
    best = scores.max(axis=0)
    keep = np.nonzero((best >= np.float32(cc.CONF)) & (cls <= 1))[0]
    order = sorted(keep.tolist(), key=lambda i: (-float(best[i]), i))
    f32 = np.float32
    half, pad_x, pad_y, sc = f32(0.5), f32(lb.padX), f32(lb.padY), f32(lb.scale)
    kept = []
    for i in order:
        cx, cy, bw, bh = o[0][i], o[1][i], o[2][i], o[3][i]
        box = [(cx - bw * half - pad_x) / sc, (cy - bh * half - pad_y) / sc,
               (cx + bw * half - pad_x) / sc, (cy + bh * half - pad_y) / sc]
        if any(iou32(box, k) > f32(cc.IOU) for k in kept):
            continue
        kept.append(box + [best[i], int(cls[i])])
        if cap is not None and len(kept) >= cap:
            break
    return [[round(float(v), 1) for v in k[:4]] + [round(float(k[4]), 4), k[5]] for k in kept]


def iou32(a, b):
    f32 = np.float32
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    w, h = f32(x2 - x1), f32(y2 - y1)
    if w <= 0 or h <= 0:
        return f32(0)
    inter = f32(w * h)
    uni = f32((a[2] - a[0]) * (a[3] - a[1])) + f32((b[2] - b[0]) * (b[3] - b[1])) - inter
    return f32(inter / uni) if uni > 0 else f32(0)


def aim_target(dets, w, h):
    best, bi = 1e18, -1
    for i, d in enumerate(dets):
        if d[4] < ACQUIRE_CONF:
            continue
        cx, cy = (d[0] + d[2]) / 2, (d[1] + d[3]) / 2
        r = (cx - w / 2) ** 2 + (cy - h / 2) ** 2
        if r < best:
            best, bi = r, i
    return bi


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("onnx")
    ap.add_argument("images", nargs="+")
    ap.add_argument("--gt", help="FiftyOne 格式的 samples.json（可以只含部分样本）")
    ap.add_argument("--out", required=True, help="输出的 HTML 路径")
    args = ap.parse_args()

    import cv2
    import onnxruntime as ort

    repo = os.path.dirname(os.path.dirname(HERE))
    lib = cc.build_lib(repo, tempfile.mkdtemp(prefix="wtyolo-"))
    sess = ort.InferenceSession(args.onnx, providers=["CPUExecutionProvider"])
    inp = sess.get_inputs()[0]
    side_h, side_w = inp.shape[2], inp.shape[3]
    gt_all = load_gt(args.gt)

    items = []
    replica_ok = True
    for path in args.images:
        bgr = cv2.imread(path)
        if bgr is None:
            print(f"[demo] 读不了 {path}", file=sys.stderr)
            continue
        rgb = np.ascontiguousarray(bgr[:, :, ::-1])
        h, w = rgb.shape[:2]

        tensor = np.empty((1, 3, side_h, side_w), dtype=np.float32)
        lb = cc.Letterbox()
        if not lib.WtYolo_Letterbox(rgb.ctypes.data_as(ctypes.c_void_p), w, h, side_w, side_h,
                                    cc.RESIZE_BILINEAR,
                                    tensor.ctypes.data_as(ctypes.c_void_p), ctypes.byref(lb)):
            print(f"[demo] letterbox 失败 {path}", file=sys.stderr)
            continue
        t0 = time.perf_counter()
        out = np.ascontiguousarray(sess.run(None, {inp.name: tensor})[0], dtype=np.float32)
        infer_ms = (time.perf_counter() - t0) * 1000.0
        channels, ncand = out.shape[1], out.shape[2]
        layout = cc.Layout(channels - 4, 0, 0)
        work = (cc.Cand * ncand)()
        res = cc.Result()
        flt = cc.Filter(cc.CONF, cc.IOU, cc.PERSON_MASK)
        if not lib.WtYolo_Decode(out.ctypes.data_as(ctypes.c_void_p), channels, ncand,
                                 ctypes.byref(layout), ctypes.byref(lb), ctypes.byref(flt),
                                 work, ncand, ctypes.byref(res)):
            print(f"[demo] 解码失败 {path}", file=sys.stderr)
            continue
        dets = [[round(d.x1, 1), round(d.y1, 1), round(d.x2, 1), round(d.y2, 1),
                 round(d.conf, 4), d.cls] for d in (res.items[i] for i in range(res.count))]
        full = py_decode(out, lb)
        # 自检：复刻版截到 64 个必须与 C 逐框一致，否则「模型本身」那组数不可信
        same = py_decode(out, lb, cap=cc.MAX_DET) == dets
        replica_ok &= same

        item = {
            "name": os.path.basename(path), "w": w, "h": h, "scale": lb.scale,
            "inferMs": round(infer_ms), "saturated": res.count >= cc.MAX_DET,
            "modes": {"device": {"dets": dets, "aim": aim_target(dets, w, h)},
                      "model": {"dets": full, "aim": aim_target(full, w, h)}},
        }
        sample = gt_all.get(os.path.basename(path))
        if sample is not None:
            persons, ignores = gt_boxes(sample, w, h)
            item["gt"] = [[round(v, 1) for v in g[:4]] + g[4:] for g in persons]
            item["ignores"] = [[round(v, 1) for v in r] for r in ignores]
            for mode in item["modes"].values():
                tags, match, hit = evaluate(mode["dets"], persons, ignores)
                mode.update({"tags": tags, "match": match, "hit": hit})
        ok, jpg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])
        item["jpg"] = base64.b64encode(jpg.tobytes()).decode("ascii")
        items.append(item)

        msg = (f"[demo] {item['name']} {w}x{h} 推理 {infer_ms:.0f} ms（本机 CPU）"
               f"检出 设备端 {len(dets)} / 模型 {len(full)}；复刻{'一致' if same else '不一致!'}")
        if "gt" in item:
            for key, mode in item["modes"].items():
                tp = sum(t == "tp" for t in mode["tags"])
                fp = sum(t == "fp" for t in mode["tags"])
                msg += (f"\n        {key:6s} 标注 {len(item['gt'])} 人，检中 {tp}，"
                        f"误检 {fp}，漏检 {len(item['gt']) - tp}")
        print(msg)

    meta = {
        "model": os.path.basename(args.onnx), "input": f"{side_w}x{side_h}",
        "conf": cc.CONF, "iou": cc.IOU, "acquire": ACQUIRE_CONF, "matchIou": MATCH_IOU,
        "maxDet": cc.MAX_DET, "bins": SIZE_BINS[:-1] + [[48, None]], "pxAt60": PX_AT_60M,
        "generated": datetime.datetime.now().strftime("%Y-%m-%d %H:%M"),
    }
    page = open(os.path.join(HERE, "demo_view_template.html"), encoding="utf-8").read()
    payload = json.dumps({"meta": meta, "items": items}, ensure_ascii=False).replace("</", "<\\/")
    with open(args.out, "w", encoding="utf-8") as fp:
        fp.write(page.replace("/*__DATA__*/null", payload))
    print(f"[demo] 已写出 {args.out}")
    print(f"[demo] numpy 复刻解码与 C 逐框{'一致' if replica_ok else '不一致 —— 「模型本身」那组数不可信'}")
    return 0 if replica_ok else 1


if __name__ == "__main__":
    sys.exit(main())

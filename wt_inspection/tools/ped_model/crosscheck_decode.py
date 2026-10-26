#!/usr/bin/env python3
"""拿 ultralytics 自己的解码结果，核对 app/wt_yolo_decode.c。

推理在设备上走 TensorRT，桌面跑不了；但「预处理 + 解码」是纯算术，和
ultralytics 对得上，设备上就只剩 engine 本身这一个变量。

三层对照（同一个 ONNX、同一张图）：

  0. 预处理对照：C 的双线性 letterbox vs cv2.resize(INTER_LINEAR) 拼出的
     同一几何的画布。OpenCV 用定点运算，允许差 1/255。
  1. 严格对照：**同一份输出张量**，C 解码 vs ultralytics 的 NMS。
     框坐标、置信度、类别必须一一对上 —— 这层不过，说明通道下标、
     letterbox 还原或 NMS 写错了。
  2. 整体参照：C 预处理 + 解码 vs ultralytics 完整预测。只报重合度 ——
     两边 padding 方式不同（ultralytics 对 .pt 用最小填充），不要求一致。

用法（本机）：

    python3 -I tools/ped_model/crosscheck_decode.py <best.onnx> <图片...> \
        [--pt <best.pt>]     # 整体参照用的同一份权重，默认取 onnx 同目录的 .pt
        [--out-dir <目录>]   # 把 C 解码的框画出来

为什么用 -I：图片与模型都来自外部下载，见 export_onnx.py 的说明。
"""

import argparse
import ctypes
import os
import site
import subprocess
import sys
import tempfile

sys.path.insert(0, site.getusersitepackages())
# ultralytics 发现缺依赖时会自己 pip install（2026-10-24 实测：加载 ONNX 时
# 因为有 CUDA 就去装 onnxruntime-gpu）。核对脚本不许改动 Python 环境。
os.environ["YOLO_AUTOINSTALL"] = "false"

import numpy as np  # noqa: E402

KPT_CAP = 7          # WT_TRT_KPT_COUNT
MAX_DET = 64         # WT_TRT_MAX_DETECTIONS
PERSON_MASK = 0b11   # 类别 0 = pedestrian, 1 = people
RESIZE_BILINEAR = 1  # E_WtYoloResize：行人模型用双线性
CONF, IOU = 0.25, 0.45


class Detection(ctypes.Structure):
    _fields_ = [("x1", ctypes.c_float), ("y1", ctypes.c_float),
                ("x2", ctypes.c_float), ("y2", ctypes.c_float),
                ("conf", ctypes.c_float), ("cls", ctypes.c_int),
                ("kx", ctypes.c_float * KPT_CAP), ("ky", ctypes.c_float * KPT_CAP)]


class Result(ctypes.Structure):
    _fields_ = [("count", ctypes.c_int), ("items", Detection * MAX_DET)]


class Layout(ctypes.Structure):
    _fields_ = [("nc", ctypes.c_int), ("kpt", ctypes.c_int), ("dims", ctypes.c_int)]


class Letterbox(ctypes.Structure):
    _fields_ = [("scale", ctypes.c_float), ("padX", ctypes.c_float), ("padY", ctypes.c_float)]


class Filter(ctypes.Structure):
    _fields_ = [("conf", ctypes.c_float), ("iou", ctypes.c_float), ("mask", ctypes.c_uint32)]


class Cand(ctypes.Structure):
    _fields_ = [("score", ctypes.c_float), ("index", ctypes.c_int), ("cls", ctypes.c_int)]


def build_lib(repo: str, out_dir: str) -> ctypes.CDLL:
    src = os.path.join(repo, "app", "wt_yolo_decode.c")
    so = os.path.join(out_dir, "libwtyolo.so")
    subprocess.run(["cc", "-std=c99", "-O2", "-shared", "-fPIC", "-I", os.path.join(repo, "app"),
                    "-o", so, src, "-lm"], check=True)
    lib = ctypes.CDLL(so)
    lib.WtYolo_Letterbox.restype = ctypes.c_bool
    lib.WtYolo_Decode.restype = ctypes.c_bool
    return lib


def iou(a, b) -> float:
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def match(xs, ys, thresh):
    """贪心一一配对（按 IoU），返回配对列表。"""
    pairs, used = [], set()
    for i, a in enumerate(xs):
        best, bj = thresh, -1
        for j, b in enumerate(ys):
            if j not in used and iou(a, b) >= best:
                best, bj = iou(a, b), j
        if bj >= 0:
            used.add(bj)
            pairs.append((i, bj))
    return pairs


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("onnx")
    ap.add_argument("images", nargs="+")
    ap.add_argument("--pt", help="整体参照用的 .pt（默认与 onnx 同名同目录）")
    ap.add_argument("--out-dir")
    args = ap.parse_args()

    import cv2
    import onnxruntime as ort
    import torch
    from ultralytics import YOLO
    try:
        from ultralytics.utils.nms import non_max_suppression
    except ImportError:  # 旧版 ultralytics
        from ultralytics.utils.ops import non_max_suppression

    repo = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    tmp = tempfile.mkdtemp(prefix="wtyolo-")
    lib = build_lib(repo, tmp)

    sess = ort.InferenceSession(args.onnx, providers=["CPUExecutionProvider"])
    inp = sess.get_inputs()[0]
    side_h, side_w = inp.shape[2], inp.shape[3]
    # 整体参照用 .pt 而不是 ONNX：ultralytics 加载 ONNX 会去检查/安装
    # onnxruntime-gpu。权重相同，差别只在预处理 —— 正是这层要量化的。
    ref_model = YOLO(args.pt or os.path.splitext(args.onnx)[0] + ".pt", task="detect")

    ok = True
    for path in args.images:
        bgr = cv2.imread(path)
        if bgr is None:
            print(f"[crosscheck] 读不了 {path}", file=sys.stderr)
            ok = False
            continue
        rgb = np.ascontiguousarray(bgr[:, :, ::-1])
        h, w = rgb.shape[:2]

        # ---- C letterbox ----
        tensor = np.empty((1, 3, side_h, side_w), dtype=np.float32)
        lb = Letterbox()
        if not lib.WtYolo_Letterbox(rgb.ctypes.data_as(ctypes.c_void_p), w, h, side_w, side_h,
                                    RESIZE_BILINEAR,
                                    tensor.ctypes.data_as(ctypes.c_void_p), ctypes.byref(lb)):
            print("[crosscheck] letterbox 失败", file=sys.stderr)
            return 1

        # ---- 0. 预处理对照：同一几何下用 OpenCV 拼一张 ----
        nw, nh = int(np.round(w * lb.scale)), int(np.round(h * lb.scale))
        canvas = np.full((side_h, side_w, 3), 114, np.uint8)
        px, py = int(lb.padX), int(lb.padY)
        canvas[py:py + nh, px:px + nw] = cv2.resize(rgb, (nw, nh), interpolation=cv2.INTER_LINEAR)
        cv_tensor = canvas.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        pre_diff = float(np.abs(tensor - cv_tensor).max()) * 255.0
        pre_ok = pre_diff <= 1.0 + 1e-3
        ok &= pre_ok

        out = np.ascontiguousarray(sess.run(None, {inp.name: tensor})[0], dtype=np.float32)
        channels, ncand = out.shape[1], out.shape[2]
        layout = Layout(channels - 4, 0, 0)

        # ---- C 解码 ----
        work = (Cand * ncand)()
        res = Result()
        flt = Filter(CONF, IOU, PERSON_MASK)
        if not lib.WtYolo_Decode(out.ctypes.data_as(ctypes.c_void_p), channels, ncand,
                                 ctypes.byref(layout), ctypes.byref(lb), ctypes.byref(flt),
                                 work, ncand, ctypes.byref(res)):
            print("[crosscheck] 解码失败", file=sys.stderr)
            return 1
        c_dets = [(d.x1, d.y1, d.x2, d.y2, d.conf, d.cls)
                  for d in (res.items[i] for i in range(res.count))]

        # ---- 1. 严格对照：同一份张量交给 ultralytics 的 NMS ----
        ref = non_max_suppression(torch.from_numpy(out), conf_thres=CONF, iou_thres=IOU,
                                  classes=[0, 1], agnostic=True, max_det=MAX_DET)[0].numpy()
        u_dets = [((r[0] - lb.padX) / lb.scale, (r[1] - lb.padY) / lb.scale,
                   (r[2] - lb.padX) / lb.scale, (r[3] - lb.padY) / lb.scale, r[4], int(r[5]))
                  for r in ref]
        pairs = match(c_dets, u_dets, 0.999)
        max_px = max((max(abs(c_dets[i][k] - u_dets[j][k]) for k in range(4))
                      for i, j in pairs), default=0.0)
        max_conf = max((abs(c_dets[i][4] - u_dets[j][4]) for i, j in pairs), default=0.0)
        cls_ok = all(c_dets[i][5] == u_dets[j][5] for i, j in pairs)
        strict = (len(pairs) == len(c_dets) == len(u_dets) and max_px < 0.05
                  and max_conf < 1e-5 and cls_ok)
        ok &= strict

        # ---- 2. 整体参照：ultralytics 完整预测（双线性 letterbox） ----
        pred = ref_model.predict(path, imgsz=(side_h, side_w), conf=CONF, iou=IOU,
                                 classes=[0, 1], agnostic_nms=True, max_det=MAX_DET,
                                 verbose=False)[0]
        p_dets = [tuple(b) for b in pred.boxes.xyxy.cpu().numpy().tolist()]
        overlap = match(c_dets, p_dets, 0.5)
        denom = max(len(c_dets), len(p_dets), 1)

        sizes = [((d[2] - d[0]), (d[3] - d[1])) for d in c_dets]
        med = np.median(np.array(sizes), axis=0) if sizes else (0, 0)
        print(f"[crosscheck] {os.path.basename(path)} {w}x{h}  "
              f"scale={lb.scale:.4f} pad=({lb.padX:.0f},{lb.padY:.0f})")
        print(f"    预处理：C 双线性 vs OpenCV 最大差 {pre_diff:.3f}/255 → {'✓' if pre_ok else '✗'}")
        print(f"    C 解码 {len(c_dets)} 人 / ultralytics NMS {len(u_dets)} 人  "
              f"配对 {len(pairs)}  坐标最大差 {max_px:.4f} px  conf 最大差 {max_conf:.2e}  "
              f"类别一致 {cls_ok}  → {'✓' if strict else '✗'}")
        print(f"    整体参照：ultralytics 完整预测 {len(p_dets)} 人，与 C 结果 IoU≥0.5 重合 "
              f"{len(overlap)}/{denom}；C 框中位尺寸 {med[0]:.0f}×{med[1]:.0f} px")

        if args.out_dir:
            os.makedirs(args.out_dir, exist_ok=True)
            vis = bgr.copy()
            for d in c_dets:
                color = (0, 255, 0) if d[5] == 0 else (0, 200, 255)
                cv2.rectangle(vis, (int(d[0]), int(d[1])), (int(d[2]), int(d[3])), color, 2)
            dst = os.path.join(args.out_dir, "c_" + os.path.basename(path))
            cv2.imwrite(dst, vis)
            print(f"    已画框：{dst}")

    print(f"[crosscheck] {'✓ C 解码与 ultralytics 一致' if ok else '✗ 有不一致'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""把 VisDrone 行人检测权重导出成 ONNX，并核对类别名与张量形状。

用法（本机）：

    python3 -I tools/ped_model/export_onnx.py <best.pt> [--imgsz 1280]

产物与 .pt 同目录（<best>.onnx）。之后拷到设备上用 trtexec 建 engine ——
engine 必须在设备上建（见 README「高空行人瞄准打点」）。

为什么用 -I：权重是第三方 .pt，本质是 pickle，加载即执行其中的代码。-I 让
解释器不从脚本目录 / 当前目录导入模块，避免同目录下被放进来的 .py 被顺手
执行。代价是用户 site 不再自动加载，所以下面手动把它加回 sys.path。

为什么 imgsz 默认 1280：这份权重（dronefreak/visdrone-yolo11s）就是在 1280
下训练的；而 M4T 广角 60 m 外的行人在 1440×1080 画面里只有约 23×10 px，
缩到 640 只剩一半，基本检不出。
"""

import argparse
import os
import site
import sys

sys.path.insert(0, site.getusersitepackages())
# ultralytics 发现缺依赖时会自己 pip install，导出脚本不许改动 Python 环境
os.environ["YOLO_AUTOINSTALL"] = "false"

# VisDrone 的类别顺序（ultralytics VisDrone.yaml 的 10 类）。决策层只要前两类。
# 注意 dronefreak/visdrone-yolo11s 实际是 11 类（末尾多一个 others），
# 所以这里只核对前两类，类别总数由 engine 的输出通道反推。
VISDRONE_NAMES = [
    "pedestrian", "people", "bicycle", "car", "van",
    "truck", "tricycle", "awning-tricycle", "bus", "motor",
]


def expected_candidates(imgsz: int) -> int:
    """三个检测尺度（stride 8/16/32）的格点数之和：640 → 8400，1280 → 33600。"""
    return sum((imgsz // s) ** 2 for s in (8, 16, 32))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("weights")
    ap.add_argument("--imgsz", type=int, default=1280)
    args = ap.parse_args()

    from ultralytics import YOLO  # 延后导入：先让 sys.path 生效
    import onnx

    model = YOLO(args.weights)
    names = [model.names[i] for i in sorted(model.names)]
    print(f"[export] 类别（{len(names)}）: {names}")

    # 决策层把类别 0/1 当成"人"。顺序不对就会去瞄自行车，而且不报错。
    if names[:2] != VISDRONE_NAMES[:2]:
        print("[export] ✗ 前两类不是 pedestrian / people，与决策层的约定不符", file=sys.stderr)
        return 1

    out_path = model.export(
        format="onnx",
        imgsz=args.imgsz,
        opset=12,        # 与风机模型同一个 opset（README「部署链路」）
        dynamic=False,   # 推理封装只支持固定形状
        simplify=False,  # 免得 ultralytics 去在线装 onnxslim（本机外网很慢）
        half=False,      # 输入输出保持 float32；FP16 交给 trtexec --fp16
        batch=1,
    )

    m = onnx.load(out_path)
    dims = lambda v: [d.dim_value for d in v.type.tensor_type.shape.dim]  # noqa: E731
    inp, outp = dims(m.graph.input[0]), dims(m.graph.output[0])
    want_in = [1, 3, args.imgsz, args.imgsz]
    want_out = [1, 4 + len(names), expected_candidates(args.imgsz)]

    print(f"[export] 输入 {m.graph.input[0].name} {inp}（期望 {want_in}）")
    print(f"[export] 输出 {m.graph.output[0].name} {outp}（期望 {want_out}）")

    if inp != want_in or outp != want_out:
        print("[export] ✗ 形状与推理封装的约定不符", file=sys.stderr)
        return 1

    size_mb = os.path.getsize(out_path) / 1e6
    print(f"[export] ✓ {out_path}（{size_mb:.1f} MB）")
    print("[export] 下一步（设备上，约十几分钟；别加 --verbose，别放 /tmp）：")
    print(f"  /usr/src/tensorrt/bin/trtexec --onnx={os.path.basename(out_path)} "
          f"--saveEngine={os.path.splitext(os.path.basename(out_path))[0]}.engine "
          "--fp16 --workspace=2048")

    return 0


if __name__ == "__main__":
    sys.exit(main())

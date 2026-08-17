# -*- coding: utf-8 -*-
"""YOLO11n-OBB 320x320 模型调用模板（方案B 修正版）

相对旧版 yolo11_obb_320.py 的改动：
1. 修正 angle 解码公式：ONNX 内为 (sigmoid - 0.25) * pi，
   旧脚本写的 (sigmoid - 0.5) * pi 存在约 45° 系统性偏差。
2. 解码后做四点几何规范化：w/h 交换时角度补 90°，
   折叠近正方形目标的 ±90° 跳变（与训练时 xyxyxyxy2xywhr 同语义）。
3. 可选 Soft-NMS，小目标（area<225px）IoU 阈值降至 0.15 并分数衰减。
4. 输出增加 corners（四点）字段，供 CLED CV 修正直接使用。

依赖：yolo11_obb_post_320.py（与本文件同目录）
"""

import numpy as np
import math

# ============================================================
# 常量（如需定制，通过构造函数参数传入）
# ============================================================
CLASS_NAMES = ["ccap", "cled", "cres"]
NC = 3
REG_MAX = 16
IMG_SIZE = 320
STRIDES = [8, 16, 32]
FEAT_SIZES = [40, 20, 10]

# ============================================================
# 预计算 grid（模块加载时一次性计算）
# ============================================================
DFL_WEIGHT = np.arange(REG_MAX, dtype=np.float32).reshape(1, REG_MAX, 1)


def _make_grid():
    grids_xy = []
    strides_list = []
    for fs, s in zip(FEAT_SIZES, STRIDES):
        gx = np.arange(fs, dtype=np.float32) + 0.5
        gy = np.arange(fs, dtype=np.float32) + 0.5
        gxv, gyv = np.meshgrid(gx, gy)
        grids_xy.append(np.stack([gxv.ravel(), gyv.ravel()], axis=0))
        strides_list.append(np.full(fs * fs, s, dtype=np.float32))
    return (
        np.concatenate(grids_xy, axis=1),
        np.concatenate(strides_list, axis=0),
    )


_grid_xy, _grid_stride = _make_grid()


# ============================================================
# 数学工具
# ============================================================
def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-np.clip(x, -20, 20)))


def _dfl_decode(bbox_dfl):
    """DFL 解码：[64, 2100] -> [4, 2100]"""
    if bbox_dfl.ndim == 3:
        bbox_dfl = bbox_dfl[0]
    bbox_dfl = bbox_dfl.reshape(4, REG_MAX, -1)
    bbox_dfl = np.exp(bbox_dfl - bbox_dfl.max(axis=1, keepdims=True))
    bbox_dfl = bbox_dfl / bbox_dfl.sum(axis=1, keepdims=True)
    return (bbox_dfl * DFL_WEIGHT).sum(axis=1)


def _decode_boxes(lt_rb, rotated_offsets):
    """解码中心+宽高：[4,2100] + [2,2100] -> [2100,4]"""
    cx = (_grid_xy[0] + rotated_offsets[0]) * _grid_stride
    cy = (_grid_xy[1] + rotated_offsets[1]) * _grid_stride
    w = (lt_rb[0] + lt_rb[2]) * _grid_stride
    h = (lt_rb[1] + lt_rb[3]) * _grid_stride
    return np.stack([cx, cy, w, h], axis=1)


def _xywhr_to_corners(cx, cy, w, h, angle_rad):
    """旋转框 -> 四个角点 [N,8]（x1,y1,x2,y2,x3,y3,x4,y4）"""
    cos_a, sin_a = np.cos(angle_rad), np.sin(angle_rad)
    hw, hh = w / 2.0, h / 2.0
    v1x, v1y = hw * cos_a, hw * sin_a
    v2x, v2y = -hh * sin_a, hh * cos_a
    pts = np.stack(
        [
            cx + v1x + v2x, cy + v1y + v2y,
            cx + v1x - v2x, cy + v1y - v2y,
            cx - v1x - v2x, cy - v1y - v2y,
            cx - v1x + v2x, cy - v1y + v2y,
        ],
        axis=1,
    )
    return pts


def _normalize_box_angle(boxes):
    """四点规范化等价操作：保证 w>=h，w<h 时交换并角度补 90°。

    近正方形目标在角度输出上存在 ±90° 模糊（同框等价），
    统一到 w>=h 语义后，跳变被折叠为同一角度。
    boxes: [N,5] (cx,cy,w,h,angle_rad)
    """
    boxes = np.array(boxes, dtype=np.float32, copy=True)
    swap = boxes[:, 3] > boxes[:, 2]
    if swap.any():
        w = boxes[swap, 2].copy()
        boxes[swap, 2] = boxes[swap, 3]
        boxes[swap, 3] = w
        boxes[swap, 4] += np.pi / 2
    # 归一化到 [-pi/2, pi/2)
    boxes[:, 4] = np.mod(boxes[:, 4] + np.pi / 2, np.pi) - np.pi / 2
    return boxes


# ============================================================
# ProbIoU（高斯分布 IoU，与 ultralytics 一致）
# ============================================================
def _get_covariance(boxes):
    w, h = boxes[..., 2], boxes[..., 3]
    angle = boxes[..., 4]
    cos_a, sin_a = np.cos(angle), np.sin(angle)
    w2, h2 = w * w, h * h
    a = (w2 * cos_a * cos_a + h2 * sin_a * sin_a) / 12.0
    b = (w2 * sin_a * sin_a + h2 * cos_a * cos_a) / 12.0
    c = (w2 - h2) * cos_a * sin_a / 12.0
    return a, b, c


def _probiou(query, candidates, eps=1e-7):
    """query[5] vs candidates[M,5] -> [M]"""
    x1, y1 = query[0], query[1]
    a1, b1, c1 = _get_covariance(query.reshape(1, 5))
    a2, b2, c2 = _get_covariance(candidates)

    a12, b12, c12 = a1 + a2, b1 + b2, c1 + c2
    dx, dy = x1 - candidates[..., 0], y1 - candidates[..., 1]
    denom = a12 * b12 - c12 * c12 + eps

    t1 = ((a12 * dy * dy + b12 * dx * dx) / denom) * 0.25
    t2 = ((c12 * (-dx) * dy) / denom) * 0.5

    det1 = np.maximum(a1 * b1 - c1 * c1, 0.0)
    det2 = np.maximum(a2 * b2 - c2 * c2, 0.0)
    t3 = np.log(np.maximum(
        denom / (4.0 * np.sqrt(det1 * det2) + eps), eps
    )) * 0.5

    bd = np.clip(t1 + t2 + t3, eps, 100.0)
    return 1.0 - np.sqrt(1.0 - np.exp(-bd) + eps)


def _class_specific_nms(boxes, scores, class_ids, iou_thresh, soft=False):
    """类特定 ProbIoU NMS / Soft-NMS。

    soft=True 时：重叠框分数衰减 score *= (1 - IoU)，小目标用 0.15 阈值。
    boxes:    [N, 5] (cx, cy, w, h, angle_rad)
    scores:   [N]
    class_ids:[N]
    返回:     (keep_mask, scores)
    """
    keep = np.ones(len(boxes), dtype=bool)
    scores = np.array(scores, dtype=np.float32, copy=True)
    for cls in np.unique(class_ids):
        mask = class_ids == cls
        idx = np.where(mask)[0]
        n = len(idx)
        if n <= 1:
            continue
        order = np.argsort(scores[mask])[::-1]
        for i in range(n):
            gi = idx[order[i]]
            if not keep[gi]:
                continue
            rest = order[i + 1:]
            if len(rest) == 0:
                break
            ious = _probiou(boxes[gi], boxes[idx[rest]])
            for k, j in enumerate(rest):
                gj = idx[j]
                if not keep[gj]:
                    continue
                area_i = boxes[gi, 2] * boxes[gi, 3]
                area_j = boxes[gj, 2] * boxes[gj, 3]
                min_area = min(area_i, area_j)
                thresh = 0.15 if min_area < 225 else iou_thresh
                if ious[k] > thresh:
                    if soft:
                        scores[gj] *= (1.0 - ious[k])
                    else:
                        keep[gj] = False
    return keep, scores


# ============================================================
# 主类
# ============================================================
class YOLO11OBBDetector:
    """YOLO11n-OBB 320×320 检测器（方案B）

    参数:
        model_path : str  MUD 文件路径（如 "./compiled_320.mud"）
        conf_thresh: float 置信度阈值（默认 0.25）
        iou_thresh : float NMS IoU 阈值（默认 0.25）
        max_det    : int   最大检出数（默认 300）
        soft_nms   : bool  是否启用 Soft-NMS（默认 True）
    """

    def __init__(self, model_path, conf_thresh=0.25, iou_thresh=0.25,
                 max_det=300, soft_nms=True):
        from maix import nn
        self.model = nn.NN(model_path)
        self.conf_thresh = conf_thresh
        self.iou_thresh = iou_thresh
        self.max_det = max_det
        self.soft_nms = soft_nms

    # ---- 底层 API ----

    def forward(self, img):
        """推理：maix.image.Image -> outputs dict（原始 tensor 字典）"""
        return self.model.forward_image(
            img, mean=[0, 0, 0], scale=[1, 1, 1], chw=False)

    def postprocess(self, outputs):
        """后处理：outputs dict -> list[dict]

        返回格式:
            [{
                "bbox":   [cx, cy, w, h, angle_deg],
                "corners":[x1,y1,x2,y2,x3,y3,x4,y4],  # 320 坐标系像素
                "score":  float,
                "class_id": int,
                "class_name": str,
            }, ...]

        angle_deg 已按 w>=h 语义归一化到 [-90, 90)，
        近正方形 ±90° 跳变被折叠。
        """
        # ── tensor → numpy ──
        def _t2n(t):
            s = t.shape()
            return np.array(t.to_float_list(), dtype=np.float32).reshape(s)

        bbox_dfl   = _t2n(outputs["/model.23/Concat_output_0"])
        cls_logits = _t2n(outputs["/model.23/Concat_1_output_0"])
        angle_raw  = _t2n(outputs["/model.23/Concat_2_output_0"])
        raw_offset = _t2n(outputs["/model.23/Concat_3_output_0"])

        # 去 batch 维（MaixPy Tensor 带 batch 维，ONNX 直出不带）
        if bbox_dfl.ndim == 3 and bbox_dfl.shape[0] == 1:
            bbox_dfl = bbox_dfl[0]
        if cls_logits.ndim == 3 and cls_logits.shape[0] == 1:
            cls_logits = cls_logits[0]
        if angle_raw.ndim == 3 and angle_raw.shape[0] == 1:
            angle_raw = angle_raw[0]
        if raw_offset.ndim == 3 and raw_offset.shape[0] == 1:
            raw_offset = raw_offset[0]

        # ── 解码 ──
        lt_rb = _dfl_decode(bbox_dfl)
        cls_scores = _sigmoid(cls_logits)
        max_scores = cls_scores.max(axis=0)
        class_ids  = cls_scores.argmax(axis=0)
        # 与 ONNX 内部一致：angle = (sigmoid(raw) - 0.25) * pi
        angle = (_sigmoid(angle_raw) - 0.25) * math.pi
        if angle.ndim == 2:
            angle = angle[0]

        cos_a, sin_a = np.cos(angle), np.sin(angle)
        rd = np.stack([
            raw_offset[0] * cos_a - raw_offset[1] * sin_a,
            raw_offset[0] * sin_a + raw_offset[1] * cos_a,
        ])

        boxes = _decode_boxes(lt_rb, rd)
        boxes = np.column_stack([boxes, angle])  # [N, 5]
        boxes = _normalize_box_angle(boxes)

        # ── 过滤 ──
        mask = max_scores > self.conf_thresh
        if not mask.any():
            return []
        boxes, scores, class_ids = boxes[mask], max_scores[mask], class_ids[mask]

        # ── NMS ──
        keep, scores = _class_specific_nms(
            boxes, scores, class_ids, self.iou_thresh, soft=self.soft_nms)
        boxes, scores, class_ids = boxes[keep], scores[keep], class_ids[keep]

        # Soft-NMS 衰减后再过滤
        if self.soft_nms:
            mask = scores > self.conf_thresh
            if not mask.any():
                return []
            boxes, scores, class_ids = boxes[mask], scores[mask], class_ids[mask]
            # Soft-NMS only lowers scores; w/h-swapped duplicates of the same
            # target can still survive. A final hard-NMS pass keeps only the
            # highest-scoring box per overlapping group.
            keep, scores = _class_specific_nms(
                boxes, scores, class_ids, self.iou_thresh, soft=False)
            boxes, scores, class_ids = boxes[keep], scores[keep], class_ids[keep]

        # ── 截断 ──
        if len(scores) > self.max_det:
            idx = np.argsort(scores)[::-1][:self.max_det]
            boxes, scores, class_ids = boxes[idx], scores[idx], class_ids[idx]

        # ── 格式化 ──
        corners = _xywhr_to_corners(
            boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3], boxes[:, 4])
        results = []
        for i in range(len(scores)):
            results.append({
                "bbox": [
                    float(boxes[i, 0]), float(boxes[i, 1]),
                    float(boxes[i, 2]), float(boxes[i, 3]),
                    (float(boxes[i, 4]) * 180.0 / math.pi),
                ],
                "corners": [float(v) for v in corners[i]],
                "score":      float(scores[i]),
                "class_id":   int(class_ids[i]),
                "class_name": CLASS_NAMES[int(class_ids[i])],
            })
        return results

    # ---- 便捷 API ----

    def detect(self, img):
        """完整推理：maix.image.Image -> list[dict]"""
        return self.postprocess(self.forward(img))


# ============================================================
# 自测（不依赖 MaixPy，用 numpy 模拟 tensor 输入）
# ============================================================
if __name__ == "__main__":
    print(f"YOLO11n-OBB 320x320 detector (方案B) ready.")
    print(f"  Classes: {CLASS_NAMES}")
    print(f"  Input size: {IMG_SIZE}x{IMG_SIZE}")

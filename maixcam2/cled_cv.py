# -*- coding: utf-8 -*-
"""CLED 角度偏移计算模块（方案B 修复版 + 白色切角辅助）

角度定义（以元件中心为原点建立 XY 系，X=右、Y=上）：
  0°  = 白色小切角位于元件左上角（设备 0° 实拍图
        eb66db74-d653-4bfd-8d84-169fd0c89499.png 所示状态，
        切角方向实测为 REF_NOTCH_DEG）
  负值 = 需要顺时针旋转才能回到 0°
  正值 = 需要逆时针旋转才能回到 0°
  输出 = 两种等价角度中绝对值最小的那个，范围 [-180, 180)

检测流程：
  1. 按 YOLO OBB 四点裁剪 CLED 区域（不旋转内容），去掉框外背景
  2. 用 BODY_THRESH 找 CLED 壳体最大连通域，取其质心作为元件中心；
     有 YOLO OBB 时直接用 OBB 中心（center_hint），兼容浅色背景
  3. 用 WHITE_THRESH 找白色切角候选块，只保留壳体角部最外侧的白斑
     （排除中心白环高亮反光）；无上一帧角度时优先取最亮（最白）的
     候选块再取半径最大，避免把框外浅色背景误判为切角；有上一帧
     角度时，取与上一帧预测角度最接近的白色块（防止多个白斑间跳变）
  4. offset = normalize(REF_NOTCH_DEG - 切角方向)
"""

import math
import sys
import cv2
import numpy as np


# ============================================================
# 可调参数
# ============================================================
BODY_THRESH = 90
WHITE_THRESH = 180
MIN_NOTCH_AREA = 4
# 主阈值找不到切角时的回退阈值（应对稍暗的光照）
FALLBACK_WHITE_THRESH = 150
# 设备 0° 实拍图（eb66db74-d653-4bfd-8d84-169fd0c89499.png）中
# 切角相对壳体中心的实测方向，用于把“左上角=0°”落到数值上。
# 换相机/光照后若 0° 图不再输出 0°，将这里改为调试打印的 theta_deg 即可。
REF_NOTCH_DEG = 139.45
ROI_EXPAND = 0.4
MORPH_KERNEL_SIZE = 3
OBB_PAD = 8
# 切角位于方形壳体角落，半径约为半对角线；白环反光半径只有约 0.6 倍半对角线。
# 只保留半径 >= OUTER_RATIO * 半对角线 的白色块，排除中心白环反光。
OUTER_RATIO = 0.72


# ============================================================
# 内部工具
# ============================================================
def _normalize_deg(value):
    """把任意角度折叠到 [-180, 180)，即取绝对值最小的等价表示。"""
    return (value + 180.0) % 360.0 - 180.0


def _contour_centroid(contour):
    M = cv2.moments(contour)
    if M["m00"] < 1e-6:
        return None
    return (float(M["m10"] / M["m00"]), float(M["m01"] / M["m00"]))


def _find_component_center(gray, region_mask=None):
    """用壳体灰度阈值找 CLED 最大连通域，返回 (中心, 外接矩形, 掩膜)。"""
    body_mask = (gray >= BODY_THRESH).astype(np.uint8)
    if region_mask is not None:
        body_mask = cv2.bitwise_and(body_mask, region_mask)
    n, labels, stats, cents = cv2.connectedComponentsWithStats(body_mask, 8)
    if n < 2:
        return None, None, None
    idx = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[idx, cv2.CC_STAT_AREA] < 50:
        return None, None, None
    center = (float(cents[idx][0]), float(cents[idx][1]))
    bbox = (
        int(stats[idx, cv2.CC_STAT_LEFT]),
        int(stats[idx, cv2.CC_STAT_TOP]),
        int(stats[idx, cv2.CC_STAT_WIDTH]),
        int(stats[idx, cv2.CC_STAT_HEIGHT]),
    )
    comp_mask = np.where(labels == idx, 255, 0).astype(np.uint8)
    return center, bbox, comp_mask


def _find_notch_center(contours, center, body_bbox, prev_angle_deg, ref_deg=None,
                       body_mask=None, gray=None, keep_polygon=None):
    """从白色候选块中选角落白色小切角，返回 (中心, 方向角, 候选列表) 或 None。

    prev_angle_deg 为上一帧角度偏差（0° 状态约等于 0），有值时优先选
    与预测切角方向最接近的白色块；无上一帧时优先选最亮（最白）的
    候选块，再选离壳体中心最远的白色块（浅色背景下切角比背景更白，
    且位于方形壳体角落，天然比镜头白环更靠外）。
    只接受位于壳体掩膜内、且半径 >= OUTER_RATIO*半对角线的白色块，
    用于排除元件外浅色背景和中心白环高亮反光。
    """
    cx, cy = center
    bw, bh = body_bbox[2], body_bbox[3]
    half_diag = 0.5 * math.hypot(bw, bh)
    outer_min = OUTER_RATIO * half_diag
    ref = REF_NOTCH_DEG if ref_deg is None else ref_deg
    target_theta = None
    if prev_angle_deg is not None:
        target_theta = (ref - prev_angle_deg) % 360.0

    best = None
    candidates = []
    for cnt in contours:
        centroid = _contour_centroid(cnt)
        if centroid is None:
            continue
        area = cv2.contourArea(cnt)
        if area < MIN_NOTCH_AREA:
            continue
        nx, ny = centroid
        theta_deg = math.degrees(math.atan2(-(ny - cy), nx - cx)) % 360.0
        radius = math.hypot(nx - cx, ny - cy)
        peak = 0.0
        if gray is not None:
            cc_mask = np.zeros(gray.shape, dtype=np.uint8)
            cv2.drawContours(cc_mask, [cnt], -1, 255, -1)
            vals = gray[cc_mask == 255]
            if vals.size:
                peak = float(vals.max())
        valid = radius >= outer_min
        if valid and body_mask is not None:
            ix, iy = int(round(nx)), int(round(ny))
            if 0 <= iy < body_mask.shape[0] and 0 <= ix < body_mask.shape[1]:
                valid = body_mask[iy, ix] != 0
            else:
                valid = False
        # 白点中心必须在 CLED 检测框内（含边界），不允许落在框外
        if valid and keep_polygon is not None:
            valid = cv2.pointPolygonTest(
                np.asarray(keep_polygon, dtype=np.float32),
                (float(nx), float(ny)), False) >= 0
        score = None
        if target_theta is not None:
            score = abs(_normalize_deg(theta_deg - target_theta))
        else:
            # 无上一帧时优先选最亮的白色块：浅色背景上白切角比背景更白，
            # 再按离中心最远排序，避免把框外浅色背景误判为切角。
            score = (-peak, -radius)
        candidates.append({
            "theta_deg": theta_deg,
            "area": area,
            "center": (nx, ny),
            "radius": radius,
            "peak": peak,
            "score": score if valid else None,
            "valid": valid,
            "selected": False,
        })
        if valid:
            if best is None or score < best[0]:
                best = (score, (nx, ny), theta_deg)

    if best is None:
        return None
    for cand in candidates:
        if abs(cand["theta_deg"] - best[2]) < 1e-9:
            cand["selected"] = True
    return best[1], best[2], candidates


# ============================================================
# 核心流程
# ============================================================
def crop_cled_roi(img_bgr, cx, cy, w, h, expand_ratio=None):
    if expand_ratio is None:
        expand_ratio = ROI_EXPAND
    size = int(max(w, h) * (1.0 + expand_ratio))
    x1 = max(0, int(cx - size / 2))
    y1 = max(0, int(cy - size / 2))
    x2 = min(img_bgr.shape[1], int(cx + size / 2))
    y2 = min(img_bgr.shape[0], int(cy + size / 2))
    roi = img_bgr[y1:y2, x1:x2]
    return roi, (x1, y1)


def _crop_body_roi(roi_bgr, region_mask=None):
    """按壳体灰度最大连通域的外接矩形裁剪，返回 (裁剪图, 相对原 ROI 的偏移)。"""
    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    _, bbox, _ = _find_component_center(gray, region_mask=region_mask)
    if bbox is None:
        return roi_bgr, (0, 0)
    x, y, w, h = bbox
    pad = max(5, int(0.1 * max(w, h)))
    x1 = max(0, x - pad)
    y1 = max(0, y - pad)
    x2 = min(roi_bgr.shape[1], x + w + pad)
    y2 = min(roi_bgr.shape[0], y + h + pad)
    return roi_bgr[y1:y2, x1:x2], (x1, y1)


def _find_white_regions(gray, threshold=None, mask=None):
    if threshold is None:
        threshold = WHITE_THRESH
    _, white_mask = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)
    if mask is not None:
        white_mask = cv2.bitwise_and(white_mask, mask)
    ks = MORPH_KERNEL_SIZE
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ks, ks))
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours, white_mask


def crop_cled_obb(img_bgr, det):
    """按 YOLO OBB 四点裁剪 CLED 区域（不旋转内容），返回 (裁剪图, 原点, 区域掩膜, 旋转框四点)。

    det 需包含 corners 四点（x1,y1,x2,y2,x3,y3,x4,y4），坐标为整帧图像坐标。
    区域掩膜用于后续把白斑检测限制在旋转框附近，排除框外浅色背景；
    旋转框四点为严格框坐标，用于要求最终白点落在检测框内。
    """
    corners = np.asarray(det["corners"], dtype=np.float32).reshape(4, 2)
    x1 = max(0, int(np.floor(corners[:, 0].min())) - OBB_PAD)
    y1 = max(0, int(np.floor(corners[:, 1].min())) - OBB_PAD)
    x2 = min(img_bgr.shape[1], int(np.ceil(corners[:, 0].max())) + OBB_PAD)
    y2 = min(img_bgr.shape[0], int(np.ceil(corners[:, 1].max())) + OBB_PAD)
    if x2 <= x1 or y2 <= y1:
        full = np.full((img_bgr.shape[0], img_bgr.shape[1]), 255, dtype=np.uint8)
        return img_bgr, (0, 0), full, corners.reshape(4, 2)
    crop = img_bgr[y1:y2, x1:x2]
    mask = np.zeros((crop.shape[0], crop.shape[1]), dtype=np.uint8)
    pts = (corners - np.array([x1, y1], dtype=np.float32)).astype(np.int32)
    cv2.fillPoly(mask, [pts], 255)
    # 切角可能略微伸出旋转框，掩膜向外扩 OBB_PAD 像素，避免切角被裁掉；
    # 最终白点位置再由严格四点做框内校验，不允许画在检测框外。
    kernel = np.ones((2 * OBB_PAD + 1, 2 * OBB_PAD + 1), dtype=np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=1)
    return crop, (x1, y1), mask, pts.reshape(4, 2)


# ============================================================
# 公开 API
# ============================================================
def compute_cled_angle_offset(roi_bgr, prev_angle_deg=None, ref_deg=None,
                              region_mask=None, center_hint=None,
                              box_size=None, region_polygon=None):
    """计算 CLED 角度偏移。

    center_hint: (x, y) ROI 内坐标，提供时直接作为元件中心使用（来自 YOLO
                 OBB 中心），不再依赖灰度最大连通域的质心，兼容浅色背景。
    box_size:    (w, h) OBB 宽高，提供时切角半径阈值按 OBB 尺寸计算；
                 未提供时退化为按壳体外接矩形尺寸计算。
    region_polygon: 严格 OBB 四点（ROI 内坐标），提供时要求白点中心
                    必须位于检测框内（含边界），不允许落在框外。
    """
    ref = REF_NOTCH_DEG if ref_deg is None else ref_deg
    cropped, crop_off = _crop_body_roi(roi_bgr, region_mask=region_mask)
    gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    rm_cropped = None
    if region_mask is not None:
        dx, dy = crop_off
        rm_cropped = region_mask[dy:dy + cropped.shape[0], dx:dx + cropped.shape[1]]
    keep_polygon = None
    if region_polygon is not None:
        dx, dy = crop_off
        keep_polygon = (np.asarray(region_polygon, dtype=np.float32).reshape(-1, 2)
                        - np.array([dx, dy], dtype=np.float32))
    center_source = "component"
    if center_hint is not None:
        dx, dy = crop_off
        center = (center_hint[0] - dx, center_hint[1] - dy)
        center_source = "hint"
        if box_size is not None:
            body_bbox = (0, 0, int(box_size[0]), int(box_size[1]))
        else:
            _, body_bbox, _ = _find_component_center(gray, region_mask=rm_cropped)
            if body_bbox is None:
                return None, None
    else:
        center, body_bbox, _ = _find_component_center(gray, region_mask=rm_cropped)
        if center is None:
            return None, None
    # 用“壳体亮度掩膜 + OBB 区域掩膜”限制白斑检测范围。
    # 不能用最大连通域掩膜：角落切角可能因暗缝与壳体断开，会被误删。
    raw_body_mask = (gray >= BODY_THRESH).astype(np.uint8) * 255
    combined_mask = raw_body_mask
    if rm_cropped is not None and rm_cropped.shape == raw_body_mask.shape:
        combined_mask = cv2.bitwise_and(raw_body_mask, rm_cropped)
    contours, _ = _find_white_regions(gray, mask=combined_mask)
    notch = _find_notch_center(contours, center, body_bbox, prev_angle_deg, ref,
                               body_mask=combined_mask, gray=gray,
                               keep_polygon=keep_polygon)
    candidates = None
    if notch is None:
        # 光照偏暗时主白阈值可能只剩零星像素，降低阈值再试一次
        contours, _ = _find_white_regions(gray, threshold=FALLBACK_WHITE_THRESH,
                                          mask=combined_mask)
        notch = _find_notch_center(contours, center, body_bbox, prev_angle_deg, ref,
                                   body_mask=combined_mask, gray=gray,
                                   keep_polygon=keep_polygon)
    if notch is None:
        return None, None
    notch_center, theta_deg, candidates = notch
    offset_deg = _normalize_deg(ref - theta_deg)
    dx, dy = crop_off
    debug_info = {
        "center": center,
        "body_bbox": body_bbox,
        "notch_center": notch_center,
        "center_roi": (center[0] + dx, center[1] + dy),
        "body_bbox_roi": (
            body_bbox[0] + dx, body_bbox[1] + dy,
            body_bbox[2], body_bbox[3],
        ),
        "notch_roi": (notch_center[0] + dx, notch_center[1] + dy),
        "theta_deg": theta_deg,
        "ref_deg": ref,
        "roi_size": (w, h),
        "center_source": center_source,
        "candidates": candidates,
    }
    return offset_deg, debug_info


# ============================================================
# 命令行入口：python cled_cv.py <image_path> [--bbox cx cy w h] [--debug]
# ============================================================
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="CLED 角度偏移计算")
    parser.add_argument("image", help="输入图像路径")
    parser.add_argument("--bbox", nargs=4, type=float, metavar=("CX", "CY", "W", "H"),
                        help="手动指定 CLED 的边界框中心坐标和宽高（代替 YOLO 检测）")
    parser.add_argument("--corners", nargs=8, type=float,
                        metavar=("X1", "Y1", "X2", "Y2", "X3", "Y3", "X4", "Y4"),
                        help="手动指定 OBB 四角点 x1,y1,...,x4,y4（模拟设备 YOLO 输出）")
    parser.add_argument("--debug", action="store_true", help="输出调试信息并保存中间图像")
    parser.add_argument("--ref", type=float, default=None,
                        help="覆盖 0° 参考方向 REF_NOTCH_DEG（用 0° 图调试打印的 theta_deg 标定）")
    args = parser.parse_args()

    img = cv2.imread(args.image)
    if img is None:
        print(f"错误: 无法读取图像 {args.image}")
        sys.exit(1)

    if args.corners:
        det = {"corners": list(args.corners)}
        vis_base, (ox, oy), obb_mask, obb_poly = crop_cled_obb(img, det)
        pts = np.asarray(args.corners, dtype=np.float32).reshape(4, 2)
        (cx, cy), (bw, bh), _ = cv2.minAreaRect(pts)
        offset_deg, debug = compute_cled_angle_offset(
            vis_base, ref_deg=args.ref, region_mask=obb_mask,
            region_polygon=obb_poly,
            center_hint=(cx - ox, cy - oy), box_size=(bw, bh))
    elif args.bbox:
        cx, cy, w, h = args.bbox
        vis_base, (ox, oy) = crop_cled_roi(img, cx, cy, w, h)
        offset_deg, debug = compute_cled_angle_offset(vis_base, ref_deg=args.ref)
    else:
        h_img, w_img = img.shape[:2]
        cx, cy = w_img / 2, h_img / 2
        w, h = w_img, h_img
        vis_base, (ox, oy) = crop_cled_roi(img, cx, cy, w, h)
        offset_deg, debug = compute_cled_angle_offset(vis_base, ref_deg=args.ref)

    if offset_deg is not None:
        sign = "+" if offset_deg >= 0 else ""
        direction = "逆时针" if offset_deg >= 0 else "顺时针"
        print(f"CLED 角度偏移: {sign}{offset_deg:.2f}° (需{direction}旋转 {abs(offset_deg):.2f}°)")
        if args.debug and debug:
            print(f"  壳体中心: ({debug['center_roi'][0]:.1f}, {debug['center_roi'][1]:.1f})")
            print(f"  壳体 bbox: {debug['body_bbox_roi']}")
            print(f"  切角中心: ({debug['notch_roi'][0]:.1f}, {debug['notch_roi'][1]:.1f})")
            print(f"  方向角 theta: {debug['theta_deg']:.2f}°")
            print(f"  参考方向 ref: {debug['ref_deg']:.2f}°")
            print(f"  ROI 尺寸: {debug['roi_size']}")
            for cand in debug["candidates"]:
                mark = "*" if cand["selected"] else " "
                ok = "Y" if cand.get("valid") else "N"
                print(f"  {mark} 候选块: theta={cand['theta_deg']:.2f}° area={cand['area']:.1f} "
                      f"radius={cand['radius']:.1f} valid={ok}")
            if args.ref is None:
                print(f"  标定提示: 若此图是 0° 状态，把 REF_NOTCH_DEG 设为 {debug['theta_deg']:.2f}，"
                      f"或运行时加 --ref {debug['theta_deg']:.2f}")

            vis = vis_base.copy()
            cc = debug["center_roi"]
            ct = debug["notch_roi"]
            cv2.circle(vis, (int(cc[0]), int(cc[1])), 5, (0, 255, 0), -1)
            cv2.circle(vis, (int(ct[0]), int(ct[1])), 5, (0, 0, 255), -1)
            cv2.line(vis, (int(cc[0]), int(cc[1])), (int(ct[0]), int(ct[1])), (255, 0, 0), 2)
            cv2.putText(vis, f"offset={sign}{offset_deg:.1f}deg", (10, 25),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            out_path = args.image.rsplit(".", 1)[0] + "_cled_debug.png"
            cv2.imwrite(out_path, vis)
            print(f"  调试图像已保存: {out_path}")
    else:
        print("错误: 无法检测到 CLED 的特征（壳体或白色切角）")
        sys.exit(1)

# common/config.py
# MaixCAM2 视觉定位系统 — 共享配置

# ============================================================
# 串口配置
# ============================================================
SERIAL_CONFIG = {
    "device": "/dev/ttyS1",
    "baudrate": 115200,
    "receive_timeout_ms": 2000,
}

# ============================================================
# UART 引脚配置（MaixCAM2 引脚映射）
# ============================================================
UART_PIN_CONFIG = {
    "tx_pin": "A30",
    "rx_pin": "A31",
    "tx_func": "UART1_TX",
    "rx_func": "UART1_RX",
}

# ============================================================
# 相机配置（上相机 / 共用）
# ============================================================
CAM_CONFIG = {
    "width": 320,
    "height": 320,
    "fps": 30,
    "warmup_frames": 3,
}

# ============================================================
# LED GPIO 配置
# ============================================================
LED_CONFIG = {
    "pin": "B0",
    "gpio": "GPIOB0",
}

# ============================================================
# 显示配置
# ============================================================
DISPLAY_CONFIG = {
    "crosshair_len": 20,
    "font_scale": 1.0,
}

# ============================================================
# 模型配置（YOLO11-OBB, MaixCAM2 nn.NN 后端）
# ============================================================
MODEL_CONFIG = {
    "model_path": "compiled_320.mud",
    "conf_thresh": 0.45,
    "iou_thresh": 0.25,
    "max_det": 32,
    "input_size": 320,
    "labels": ["ccap", "cled", "cres"],
}

# ============================================================
# P0 — 握手配置
# ============================================================
P0_CONFIG = {
    "watchdog_timeout_s": 30,
}

# ============================================================
# P1 — 上相机单次检测对位 (YOLO11-OBB)
# ============================================================
P1_CONFIG = {
    # 相机（detect模式）
    "cam_width": 320,
    "cam_height": 320,
    "cam_fps": 50,
    "warmup_frames": 3,

    # YOLO 参数
    "yolo_conf_th": 0.45,
    "yolo_iou_th": 0.45,

    # Phase0 快速搜索
    "hunt_lock_frames": 1,          # 一帧识别到目标即锁定（方案B）
    "fail_streak_reinit": 5,        # 连续None触发重初始化
    "fail_total_max": 50,           # 累计None上限 -> err1_1
    "scan_limit_frames": 50,        # Phase0 无目标扫描帧数 -> 发送 mv 请求切换视野
    "center_margin_px": 10,         # 目标中心距图像边缘最小像素，小于则舍弃

    # Phase1 静止检测
    "init_frames": 3,               # 采集帧数（3帧取平均）
    "wait_go_timeout_ms": 30000,    # 等待 go 信号超时

    # 重复修正偏差（与 P2 aligning 类似）
    "align_th": 4.0,                # 对准阈值(像素)
    "align_frames": 5,              # 每轮检测帧数（全部在阈值内才 ok）
    "align_max_iter": 7,            # 最大修正迭代次数 -> err1_6

}

# ============================================================
# P2 — Mark 多点寻标对位 (Canny边缘+轮廓)
# ============================================================
P2_CONFIG = {
    # 相机
    "cam_width": 320,
    "cam_height": 320,
    "cam_fps": 50,
    "warmup_frames": 3,

    # 视场
    "fov_mm": 11.2,
    "pixel_to_mm": 11.2 / 320.0,

    # ROI裁剪
    "crop_ratio": 0.95,

    # 团块筛选
    "blob_pixel_th": 50,
    "blob_area_min": 165,
    "blob_area_max": 260,
    "blob_circ_min": 0.65,
    "cv_circ_min": 0.4,
    "aspect_max": 1.5,
    "extent_max": 0.88,
    "edge_margin": 8,
    "conf_min": 0.4,
    "subpixel_margin": 10,

    # Canny 参数
    "canny_low": 15,
    "canny_high": 80,
    "canny_min_pixels": 50,
    "contour_close_k": 7,

    # 搜索稳定性
    "stability_frames": 1,
    "stability_th": 3.0,
    "stability_min_hits": 3,

    # 对准参数
    "align_th": 4.0,                # 对准阈值(像素)
    "align_confirm": 10,            # 连续对准确认帧数
    "align_max_iter": 7,            # 最大迭代次数
    "mark_distinct": 40,            # Mark间距阈值

    # 搜索超时
    "search_timeout_s": 60,

    # 目标数量
    "target_count": 3,

    # pos_detect 采样
    "pos_frames": 5,             # 有效帧数
    "pos_max_try": 500,          # 最多尝试次数
    "pos_dev_th": 4.0,           # 5帧中心与中位数偏差上限(像素)，超限整批重采
}

# ============================================================
# P3 — 下相机单次检测对位 (边缘检测 + 角度)
# ============================================================
P3_CONFIG = {
    # 下相机 (USB)：P3/P4 统一使用 index 0（已验证），分辨率/fps 两进程共用
    "cam_width": 640,
    "cam_height": 480,
    "cam_fps": 60,
    "warmup_frames": 5,

    # 吸嘴检测 (Phase0)
    "nozzle_enabled": True,
    "nozzle_check_frames": 10,
    "nozzle_detect_threshold": 4,   # 10帧中>=4帧检测到圆 -> 吸嘴为空，否则视为吸到物品
    "nozzle_area_min": 270,
    "nozzle_area_max": 500,
    "nozzle_circ_min": 0.54,
    "nozzle_center_bgr_max": [50, 70, 50],   # 目标中心颜色上限 (BGR)，与p4_circle一致
    "nozzle_blur_ksize": 5,
    "nozzle_dark_thresholds": [50, 65, 80, 95, 110, 125, 135],
    "nozzle_discard_frames": 35,    # Phase0 稳定相机时先丢弃的帧数

    # 边缘检测 (Phase1)
    "blur_ksize": 5,
    "canny_low": 20,
    "canny_high": 80,
    "dilate_ksize": 3,
    "dilate_iters": 2,

    # 灰度图预处理（反光抑制）：CLAHE 增强轮廓 + 高亮截断去掉反光点
    "gray_pre_enabled": False,
    "gray_clahe_enabled": True,
    "gray_clahe_clip_limit": 2.0,
    "gray_clahe_tile_grid": 8,
    "gray_bright_cap": 160,

    # 矩形筛选
    "rect_area_min": 2500,
    "rect_area_max": 50000,
    "rect_ratio_min": 0.3,
    "rect_ratio_max": 3.0,
    "rect_solidity_min": 0.75,
    "rect_centroid_dist_max": 15,

    # 检测帧数
    "avg_frames": 5,
    "avg_max_try": 60,              # Phase1 最多尝试采集帧数（不要求连续）
    "avg_outlier_th": 4.0,          # Phase1 异常帧判定：与中位数偏差超过该值(像素)则剔除
    "avg_min_keep": 3,              # 剔除后至少保留帧数，不足则回退使用全部帧

    # 重复修正偏差（与 P2 aligning 类似）
    "align_th": 6.0,
    "align_frames": 7,              # 每轮检测帧数（剔除异常帧后取平均判定）
    "align_max_try": 40,            # 每轮最多尝试采集帧数（不要求连续）
    "align_max_iter": 7,
    "align_outlier_th": 4.0,        # 异常帧判定：与中位数偏差超过该值(像素)则剔除
    "align_min_keep": 3,            # 剔除后至少保留帧数，不足则回退使用全部帧
    "consistency_area_ratio": 0.5,   # 面积相对差异上限
    "consistency_side_ratio": 0.5,   # 长边/短边相对差异上限
    "consistency_angle_deg": 6.0,    # 角度差异上限(度)

    # 等待 go 信号超时
    "wait_go_timeout_ms": 30000,

    # 输出增益
    "dx_gain": 6.7777,
    "dy_gain": 6.7777,
}

# ============================================================
# P4 — 下相机圆形标定对位
# ============================================================
P4_CONFIG = {
    # 下相机统一使用 P3_CONFIG 的宽高/fps/预热（P3/P4 共用 _init_lower_camera）

    # P4 进程开始后先丢弃的帧数（稳定相机画面后再识别）
    "discard_frames": 8,

    # 圆面积范围 (px²)
    "circle_area_min": 200,
    "circle_area_max": 550,
    # 目标中心颜色上限 (BGR)，与p4_circle一致；2026-08-10 先放松10% ([55,77,55])，再放松5% (×1.05)
    "center_bgr_max": [60, 83, 60],
    # 2026-08-12 重新放宽 3% (×1.03)，改回 [60,83,60]

    # 对准阈值
    "align_threshold": 5,
    "max_iter": 5,

    # 检测帧数
    "detect_frames": 8,

    # EMA平滑
    "ema_alpha": 0.6,

    # 位置稳定性
    "stability_dist": 25,
    "stability_frames": 5,
    "reset_frames": 10,

    # 多帧投票（2026-08-10 vote_frames 由 8 调整为 5）
    "vote_ratio": 0.85,
    "vote_frames": 5,

    # 圆形验证
    "edge_coverage_min": 0.65,
    "arc_min": 10,
    "interior_darkness_max": 38,
    "center_dist_th": 15,
    "min_concentric": 1,

    # 图像处理
    "blur_ksize": 5,
    "canny_low": 15,
    "canny_high": 100,

    # 暗色阈值列表
    "dark_thresholds": [50, 65, 80, 95, 110, 125, 135],

    # 等待go信号超时
    "wait_go_timeout_ms": 30000,
}

# ============================================================
# 看门狗超时 (ms)
# ============================================================
WATCHDOG_TIMEOUT_MS = 30000

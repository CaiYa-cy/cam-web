# ============================================================
# MaixCAM2 视觉定位系统 - 主程序
# 按照程序流程.md 编写，适配 MaixCAM2 (MaixPy 4.x) 硬件
#
# 模型: YOLO11-OBB (compiled_320.mud, 320x320, 3类)
# 通信: 串口帧协议 (0x7E/0x7F)
# 流程: P0握手 -> 主循环 -> P1/P2/P3/P4 进程
# ============================================================

import sys as _sys
import os as _os
import math
import gc
import cv2

# 将项目根目录加入路径
_PROJ_ROOT = _os.path.dirname(_os.path.abspath(__file__))
if _PROJ_ROOT not in _sys.path:
    _sys.path.insert(0, _PROJ_ROOT)

from maix import camera, display, image, app, gpio, pinmap, err, time

# ============================================================
# 配置导入
# ============================================================
from common.config import (
    SERIAL_CONFIG, UART_PIN_CONFIG, CAM_CONFIG, MODEL_CONFIG,
    LED_CONFIG, DISPLAY_CONFIG,
    P0_CONFIG, P1_CONFIG, P2_CONFIG, P3_CONFIG, P4_CONFIG,
    WATCHDOG_TIMEOUT_MS,
)

# 模型模块（P1 检测逻辑：方案B修正版后处理 + CLED CV 角度修正）
from yolo11_obb_320 import YOLO11OBBDetector
from cled_cv import crop_cled_obb, compute_cled_angle_offset

# 串口模块
from usart import CamComm

try:
    import numpy as np
except ImportError:
    np = None

# ============================================================
# 网页端传图（参考 camera_upload.py / maixcam2_guide.md）
# 只管把最后一帧交给后台发送线程，不做结果检测、不等待、不超时
# ============================================================
CLOUD_URL = 'http://124.222.148.162:8080/camera/upload'
JPEG_QUALITY = 65

try:
    import urequests as requests
except ImportError:
    try:
        import requests
    except ImportError:
        requests = None

try:
    import _thread
    _HAS_THREAD = True
except ImportError:
    _HAS_THREAD = False

_last_upload_data = None
_upload_latest = None
_upload_id = 0
_upload_sent_id = 0
_upload_running = True
_upload_thread_started = False


def _maix_img_to_jpeg_bytes(img):
    """将 maix image 编码为 JPEG bytes。"""
    try:
        jpeg_img = img.to_jpeg(quality=JPEG_QUALITY)
    except Exception:
        return None
    try:
        return jpeg_img.to_bytes()
    except AttributeError:
        pass
    try:
        return jpeg_img.tobytes()
    except AttributeError:
        pass
    return bytes(jpeg_img)


def _cv_frame_to_jpeg_bytes(frame):
    """将 OpenCV BGR 帧编码为 JPEG bytes。"""
    try:
        ok, buf = cv2.imencode('.jpg', frame,
                               [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if ok:
            return buf.tobytes()
    except Exception:
        pass
    return None


def _data_to_jpeg_bytes(data):
    """bytes 直接返回，maix image 用 to_jpeg，numpy 帧用 cv2.imencode。"""
    if data is None:
        return None
    if isinstance(data, bytes):
        return data
    if hasattr(data, 'to_jpeg'):
        return _maix_img_to_jpeg_bytes(data)
    return _cv_frame_to_jpeg_bytes(data)


def _reset_upload_frame():
    global _last_upload_data
    _last_upload_data = None


def _remember_maix_frame(img):
    global _last_upload_data
    try:
        _last_upload_data = img.copy()
    except Exception:
        jpeg = _maix_img_to_jpeg_bytes(img)
        if jpeg is not None:
            _last_upload_data = jpeg


def _remember_cv_frame(frame):
    global _last_upload_data
    try:
        _last_upload_data = frame.copy()
    except Exception:
        _last_upload_data = frame


def _upload_worker():
    """后台线程：只发送，不检测结果、不设置超时。"""
    global _upload_latest, _upload_id, _upload_sent_id, _upload_running
    while _upload_running:
        if _upload_latest is not None and _upload_id != _upload_sent_id:
            data = _upload_latest
            sid = _upload_id
            try:
                jpeg = _data_to_jpeg_bytes(data)
                if jpeg is not None and requests is not None:
                    requests.post(CLOUD_URL, data=jpeg)
            except Exception:
                pass
            _upload_sent_id = sid
        else:
            time.sleep_ms(10)


def _start_upload_thread():
    global _upload_thread_started
    if _upload_thread_started or not _HAS_THREAD:
        return
    try:
        _thread.start_new_thread(_upload_worker, ())
        _upload_thread_started = True
    except Exception:
        pass


def _upload_last_frame(tag):
    """进程退出前把最后一帧交给后台发送，立即返回，不阻塞主流程。"""
    global _upload_latest, _upload_id
    data = _last_upload_data
    if data is None:
        return False
    if _HAS_THREAD and _upload_thread_started:
        _upload_latest = data
        _upload_id += 1
        return True
    # 无线程时的兜底：同步发送一次，仍不做结果检测和超时
    jpeg = _data_to_jpeg_bytes(data)
    if jpeg is None or requests is None:
        return False
    try:
        requests.post(CLOUD_URL, data=jpeg)
        return True
    except Exception:
        return False

# ============================================================
# ==== 第1步：配置 UART 引脚 ====
# ============================================================
print('\n[INIT] 1. 配置 UART 引脚 ({}={}, {}={})'.format(
    UART_PIN_CONFIG['tx_pin'], UART_PIN_CONFIG['tx_func'],
    UART_PIN_CONFIG['rx_pin'], UART_PIN_CONFIG['rx_func']))
err.check_raise(
    pinmap.set_pin_function(UART_PIN_CONFIG['tx_pin'], UART_PIN_CONFIG['tx_func']),
    'set UART TX failed')
err.check_raise(
    pinmap.set_pin_function(UART_PIN_CONFIG['rx_pin'], UART_PIN_CONFIG['rx_func']),
    'set UART RX failed')
print('[INIT]    UART 引脚配置完成')

# ============================================================
# ==== 第2步：加载 YOLO11OBB 模型 ====
# ============================================================
print('[INIT] 2. 加载 YOLO11OBB 模型: {}'.format(MODEL_CONFIG['model_path']))
model_path = _os.path.join(_PROJ_ROOT, MODEL_CONFIG['model_path'])
detector = YOLO11OBBDetector(
    model_path,
    conf_thresh=MODEL_CONFIG['conf_thresh'],
    iou_thresh=MODEL_CONFIG['iou_thresh'],
    max_det=MODEL_CONFIG['max_det'],
    soft_nms=True,
)
print('[INIT]    模型加载完成, 标签: {}, 输入: {}x{}'.format(
    MODEL_CONFIG['labels'], MODEL_CONFIG['input_size'], MODEL_CONFIG['input_size']))

# ============================================================
# ==== 第3步：初始化串口 ====
# ============================================================
print('[INIT] 3. 初始化串口: {} @ {} baud'.format(
    SERIAL_CONFIG['device'], SERIAL_CONFIG['baudrate']))
cam_uart = CamComm(
    device=SERIAL_CONFIG['device'],
    baudrate=SERIAL_CONFIG['baudrate'],
    timeout_ms=SERIAL_CONFIG['receive_timeout_ms'],
)
print('[INIT]    串口初始化完成')

# ============================================================
# ==== 第4步：初始化显示屏 ====
# ============================================================
print('[INIT] 4. 初始化显示屏')
disp = display.Display()
img_cx = CAM_CONFIG['width'] // 2
img_cy = CAM_CONFIG['height'] // 2
print('[INIT]    显示屏初始化完成, 中心: ({}, {})'.format(img_cx, img_cy))

# ============================================================
# ==== 第5步：初始化上相机 (320x320) ====
# ============================================================
print('[INIT] 5. 初始化上相机 ({}x{}, {}fps)'.format(
    CAM_CONFIG['width'], CAM_CONFIG['height'], CAM_CONFIG['fps']))
upper_cam = camera.Camera(
    CAM_CONFIG['width'], CAM_CONFIG['height'], fps=CAM_CONFIG['fps'])
upper_cam_cfg = (CAM_CONFIG['width'], CAM_CONFIG['height'], CAM_CONFIG['fps'])

# 预热
warm_ok = 0
for i in range(CAM_CONFIG['warmup_frames']):
    try:
        f = upper_cam.read()
        ok = 'OK' if f else 'None'
        if f:
            warm_ok += 1
        print('[INIT]    预热 {}/{}: {}'.format(i + 1, CAM_CONFIG['warmup_frames'], ok))
    except Exception as e:
        print('[INIT]    预热 {}/{}: 异常 {}'.format(i + 1, CAM_CONFIG['warmup_frames'], e))
print('[INIT]    上相机初始化完成 ({}/{} 帧正常)'.format(warm_ok, CAM_CONFIG['warmup_frames']))

# ============================================================
# ==== 第6步：初始化 GPIO (LED) ====
# ============================================================
print('[INIT] 6. 初始化 GPIO ({})'.format(LED_CONFIG['pin']))
try:
    err.check_raise(
        pinmap.set_pin_function(LED_CONFIG['pin'], LED_CONFIG['gpio']),
        'set LED GPIO failed')
    led = gpio.GPIO(LED_CONFIG['gpio'], gpio.Mode.OUT)
    led.value(0)
    print('[INIT]    LED GPIO 初始化完成')
except Exception as e:
    print('[INIT]    LED GPIO 初始化失败: {}'.format(e))
    led = None

# ============================================================
# 工具函数
# ============================================================

def _safe_cam_read(cam_obj, retries=2, delay_ms=50):
    for attempt in range(retries + 1):
        try:
            img = cam_obj.read()
            return img
        except RuntimeError as e:
            if attempt < retries:
                print('[cam] RuntimeError ({}/{}): {}, retrying...'.format(
                    attempt + 1, retries + 1, e))
                time.sleep_ms(delay_ms)
            else:
                print('[cam] RuntimeError ({}/{}): {}, 放弃'.format(
                    attempt + 1, retries + 1, e))
        except Exception as e:
            print('[cam] 读取异常: {}'.format(e))
            break
    return None


def _switch_upper_cam(width, height, fps=30):
    global upper_cam, upper_cam_cfg
    need = (width, height, fps)
    if upper_cam_cfg == need and upper_cam is not None:
        return
    print('[cam] 切换上相机: {}x{}@{}fps -> {}x{}@{}fps'.format(
        upper_cam_cfg[0], upper_cam_cfg[1], upper_cam_cfg[2], width, height, fps))
    if upper_cam is not None:
        del upper_cam
        gc.collect()
    upper_cam = camera.Camera(width, height, fps=fps)
    upper_cam_cfg = need
    time.sleep_ms(500)
    ok = 0
    for i in range(CAM_CONFIG['warmup_frames']):
        f = _safe_cam_read(upper_cam)
        if f is not None:
            ok += 1
        print('[cam] 预热 {}: {}'.format(i, 'OK' if f else 'None'))
    print('[cam] 切换完成 ({}/{} 帧正常)'.format(ok, CAM_CONFIG['warmup_frames']))


def _reinit_upper_cam(width, height, fps=30):
    global upper_cam, upper_cam_cfg
    print('[cam] 重建: 销毁旧相机...')
    if upper_cam is not None:
        del upper_cam
        upper_cam = None
        gc.collect()
    time.sleep_ms(300)
    print('[cam] 重建: 创建新相机 ({}x{}, fps={})...'.format(width, height, fps))
    try:
        upper_cam = camera.Camera(width, height, fps=fps)
    except Exception as e:
        print('[cam] 重建失败: {}'.format(e))
        upper_cam = None
        return False
    upper_cam_cfg = (width, height, fps)
    time.sleep_ms(500)
    ok = 0
    for i in range(CAM_CONFIG['warmup_frames'] * 2):
        f = _safe_cam_read(upper_cam)
        if f is not None:
            ok += 1
        print('[cam] 重建预热 {}: {}'.format(i, 'OK' if f else 'None'))
    return ok >= CAM_CONFIG['warmup_frames']


# ============================================================
# P3/P4 下相机 (USB) — 全局变量，P3/P4 共用
# ============================================================
p3_cap = None


def _init_lower_camera():
    """P3/P4 共用下相机初始化：统一 index=0、统一 P3_CONFIG 分辨率配置。"""
    global p3_cap
    if p3_cap is not None:
        try:
            if p3_cap.isOpened():
                return True
            p3_cap.release()
        except Exception:
            pass
        p3_cap = None
    try:
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, P3_CONFIG['cam_width'])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, P3_CONFIG['cam_height'])
            cap.set(cv2.CAP_PROP_FPS, P3_CONFIG['cam_fps'])
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            ok = 0
            for _ in range(P3_CONFIG['warmup_frames']):
                ret, _ = cap.read()
                if ret:
                    ok += 1
            if ok > 0:
                p3_cap = cap
                print('[CAM] 下相机初始化成功 (index=0, {}/{}预热)'.format(
                    ok, P3_CONFIG['warmup_frames']))
                return True
            cap.release()
    except Exception as e:
        print('[CAM] 下相机初始化失败: {}'.format(e))
    return False


def _get_center_bgr(bgr, cx, cy, radius=2):
    """p4_circle 中心颜色采样：取目标中心附近区域的平均 BGR。"""
    h, w = bgr.shape[:2]
    x0 = max(0, int(cx) - radius)
    x1 = min(w - 1, int(cx) + radius)
    y0 = max(0, int(cy) - radius)
    y1 = min(h - 1, int(cy) + radius)
    b, g, r = cv2.mean(bgr[y0:y1 + 1, x0:x1 + 1])[:3]
    return b, g, r


def _circular_mean_deg(angles_deg):
    if not angles_deg:
        return 0.0, 0
    sx = sum(math.cos(math.radians(a)) for a in angles_deg)
    sy = sum(math.sin(math.radians(a)) for a in angles_deg)
    n = len(angles_deg)
    raw = math.degrees(math.atan2(sy / n, sx / n))
    if raw < -90:
        raw += 180
    elif raw > 90:
        raw -= 180
    if abs(raw) <= 45:
        return raw, n
    elif raw > 0:
        return raw - 90, n
    else:
        return raw + 90, n


def _circular_mean_deg_180(angles_deg):
    """OBB 角度按 180° 周期做环形平均，保留 [-90, 90) 的方向性。"""
    if not angles_deg:
        return 0.0, 0
    sx = sum(math.cos(math.radians(2 * a)) for a in angles_deg)
    sy = sum(math.sin(math.radians(2 * a)) for a in angles_deg)
    n = len(angles_deg)
    raw = math.degrees(math.atan2(sy / n, sx / n)) / 2.0
    if raw >= 90.0:
        raw -= 180.0
    elif raw < -90.0:
        raw += 180.0
    return raw, n

def _circular_mean_deg_360(angles_deg):
    """CLED 的 [-180, 180) 角度按 360° 周期做环形平均。"""
    if not angles_deg:
        return 0.0, 0
    sx = sum(math.cos(math.radians(a)) for a in angles_deg)
    sy = sum(math.sin(math.radians(a)) for a in angles_deg)
    n = len(angles_deg)
    return math.degrees(math.atan2(sy / n, sx / n)), n


def _draw_crosshair(img_obj, cx, cy, length=20, color=None):
    if color is None:
        color = image.COLOR_WHITE
    img_obj.draw_line(cx - length, cy, cx + length, cy, color, 1)
    img_obj.draw_line(cx, cy - length, cx, cy + length, color, 1)


def _draw_rotated_box(img_obj, cx, cy, w, h, angle_deg, color, thickness=2):
    rad = math.radians(angle_deg)
    cos_a = math.cos(rad)
    sin_a = math.sin(rad)
    hw = w / 2.0
    hh = h / 2.0
    corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
    pts = []
    for dx, dy in corners:
        rx = cx + dx * cos_a - dy * sin_a
        ry = cy + dx * sin_a + dy * cos_a
        pts.append((int(rx), int(ry)))
    for i in range(4):
        img_obj.draw_line(pts[i][0], pts[i][1],
                          pts[(i + 1) % 4][0], pts[(i + 1) % 4][1],
                          color, thickness)


def _draw_obb_corners(img_obj, corners, color, thickness=2):
    for i in range(4):
        x1 = int(corners[i * 2])
        y1 = int(corners[i * 2 + 1])
        x2 = int(corners[((i + 1) % 4) * 2])
        y2 = int(corners[((i + 1) % 4) * 2 + 1])
        img_obj.draw_line(x1, y1, x2, y2, color, thickness)


# P1 帧间角度平滑系数（与 p1/main.py 一致）
ANGLE_SMOOTH = 0.6
# 大角度跳变门控：|差值| >= ANGLE_JUMP_DEG 时先不跟随，连续
# ANGLE_JUMP_CONFIRM 帧仍在新方向才接受，抑制近方形目标 ±90°
# w/h 互换导致的短暂跳变（例如竖直 -88° 偶发跳到 +18°）。
ANGLE_JUMP_DEG = 70.0
ANGLE_JUMP_CONFIRM = 3


# ============================================================
# 颜色映射
# ============================================================
CLASS_COLORS = {
    'ccap': image.Color(0, 255, 0),
    'cled': image.Color(0, 0, 255),
    'cres': image.Color(255, 0, 0),
}
CLR_WHITE = image.Color(255, 255, 255)
CLR_YELLOW = image.Color(255, 255, 0)
CLR_GREEN = image.Color(0, 255, 0)
CLR_RED = image.Color(255, 0, 0)
CLR_BLUE = image.Color(0, 0, 255)
# CLED 中心点/切角点/连线颜色（与 p1/main.py 一致）
CLED_CENTER_COLOR = image.Color(0, 255, 0)
CLED_NOTCH_COLOR = image.Color(255, 0, 0)
CLED_LINE_COLOR = image.Color(0, 0, 255)
CLR_CYAN = image.Color(0, 255, 255)

print('')
print('=' * 50)
print('  MaixCAM2 视觉定位系统')
print('  模型: YOLO11-OBB 320x320')
print('  标签: {}'.format(str(MODEL_CONFIG['labels'])))
print('  串口: {} @ {}'.format(SERIAL_CONFIG['device'], SERIAL_CONFIG['baudrate']))
print('=' * 50)
print('')

# ============================================================
# ==== 第7步：初始化下相机 (USB 640x480@60) ====
# ============================================================
print('[INIT] 7. 初始化下相机 ({}x{}@{}fps, index=0)'.format(
    P3_CONFIG['cam_width'], P3_CONFIG['cam_height'], P3_CONFIG['cam_fps']))
if _init_lower_camera():
    print('[INIT]    下相机初始化完成 (P3/P4 共用)')
else:
    print('[INIT]    下相机初始化失败，P3/P4 执行时会重试')

# ============================================================
# ==== 第8步：发送 ok 给主控，进入 P0 握手 ====
# ============================================================
print('[INIT] 8. 发送 ok，进入 P0 握手')
cam_uart.send_string('ok')
print('[INIT] ==== 初始化完成 ====')
_start_upload_thread()
print('')

# ============================================================
# System management commands: rst/quit/srst
# Every state machine reads these via _recv/_check_mgmt and
# executes them immediately.
# ============================================================
_MGMT_CMDS = ('rst', 'quit', 'srst')


class _P0Rehandshake(Exception):
    """P0 握手信号打断当前进程，返回主循环 P0 阶段。"""
    pass


def _handle_p0_handshake():
    """按 P0 握手格式应答 rdy，并通知主循环返回 P0 阶段。"""
    cam_uart.send_string('rdy')
    print('[P0] 收到 p0 -> 发送 rdy, 返回 P0 握手阶段')
    raise _P0Rehandshake()


def _cleanup_exit():
    global _upload_running
    print('[EXIT] program cleanup')
    _upload_running = False
    if led is not None:
        try:
            led.value(0)
        except Exception:
            pass
    if p3_cap is not None:
        try:
            p3_cap.release()
        except Exception:
            pass
    print('[EXIT] cleanup done')


def _exec_mgmt_cmd(cmd):
    """rst: restart current app, quit: exit app, srst: reboot system.

    After restart/reboot the new process sends 'ok' again once init is done.
    """
    print('[MGMT] exec management cmd: ' + cmd)
    try:
        if cmd == 'rst':
            app.restart()
        elif cmd == 'quit':
            app.exit()
        elif cmd == 'srst':
            try:
                _os.sync()
            except Exception:
                pass
            _os.system('reboot')
    finally:
        _cleanup_exit()
    raise SystemExit(0)


def _recv(timeout=200, p0_handshake=False):
    cmd = cam_uart.receive(timeout=timeout)
    if cmd in _MGMT_CMDS:
        _exec_mgmt_cmd(cmd)
    if cmd == 'p0' and not p0_handshake:
        _handle_p0_handshake()
    return cmd


def _check_mgmt():
    """Non-blocking management check for compute-only states."""
    cmd = cam_uart.receive(timeout=0)
    if cmd is None:
        return
    if cmd in _MGMT_CMDS:
        _exec_mgmt_cmd(cmd)
    if cmd == 'p0':
        _handle_p0_handshake()
    cam_uart.put_back(cmd)

# ============================================================
# P0 — 握手流程
# ============================================================
print('[P0] 等待主机发送 p0 握手信号...')
p0_received = False
p0_start = time.ticks_ms()
while not p0_received and not app.need_exit():
    if time.ticks_diff(time.ticks_ms(), p0_start) > WATCHDOG_TIMEOUT_MS:
        cam_uart.send_string('err0')
        print('[P0] 看门狗超时 ({}s), 发送 err0'.format(WATCHDOG_TIMEOUT_MS // 1000))
        p0_start = time.ticks_ms()
    cmd = _recv(timeout=200, p0_handshake=True)
    if cmd == 'p0':
        cam_uart.send_string('rdy')
        print('[P0] 收到 p0 -> 发送 rdy, 握手成功')
        p0_received = True
        break

print('[P0] 握手完成，进入主循环')
print('')

_last_p0_time = time.ticks_ms()


# ============================================================
# ====================   主循环   =============================
# ============================================================

print('[MAIN] 主循环启动, 等待指令...')

while not app.need_exit():
    try:
        # 看门狗: 30s无指令 -> err0
        if time.ticks_diff(time.ticks_ms(), _last_p0_time) > WATCHDOG_TIMEOUT_MS:
            cam_uart.send_string('err0')
            print('[WDT] 看门狗超时, 发送 err0')
            _last_p0_time = time.ticks_ms()
    
        resp = _recv(timeout=200)
        if resp is None:
            continue
    
        _last_p0_time = time.ticks_ms()
        print('[MAIN] 收到指令: ' + resp)
    
        # P0 重握手已由 _recv/_check_mgmt 全局处理：
        # 收到 p0 即按原格式回复 rdy，并返回主循环 P0 阶段
    
        # ============================================================
        # P1: 上相机 YOLO-OBB 单次检测对位
        # ============================================================
        if resp == 'p1':
            cam_uart.reset()
            if led: led.value(1)
            print('[P1] ======== P1 START ========')
            _reset_upload_frame()
            cam_uart.send_string('rdy')
    
            # ---- 等待主机发送目标类别: cls -> N:{class_id} -> end ----
            print('[P1] 等待主机发送目标类别...')
            target_cls_id = 0
            got_cls = False
            cls_to = time.ticks_ms()
            while not got_cls and time.ticks_diff(time.ticks_ms(), cls_to) < 30000:
                r = _recv(timeout=200)
                if r == 'cls':
                    n_cmd = _recv(timeout=1000)
                    if n_cmd and n_cmd.startswith('N:'):
                        try:
                            target_cls_id = int(n_cmd[2:])
                            target_cls_id = max(0, min(target_cls_id, len(MODEL_CONFIG['labels']) - 1))
                            end_cmd = _recv(timeout=1000)
                            if end_cmd == 'end':
                                got_cls = True
                                print('[P1] 目标类别: id={} name={}'.format(
                                    target_cls_id, MODEL_CONFIG['labels'][target_cls_id]))
                        except ValueError:
                            pass
                elif r == 'end':
                    break
                if r is not None and r not in ('cls',):
                    print('[P1] 收到非预期指令: {}, 使用默认类别0'.format(r))
                    got_cls = True
                    break
    
            if not got_cls:
                print('[P1] 未收到有效类别(或收到 end), 退出 P1')
                if led: led.value(0)
                continue
    
            target_cls_name = MODEL_CONFIG['labels'][target_cls_id % len(MODEL_CONFIG['labels'])]
            print('[P1] 最终目标类别: {} (id={})'.format(target_cls_name, target_cls_id))
    
            # ---- 主控电机停稳后发送 go，P1 再开始 Phase0 ----
            print('[P1] 等待主控发送 go 后开始 Phase0...')
            go0_ok = False
            go0_abort = False
            go0_to = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), go0_to) < P1_CONFIG['wait_go_timeout_ms']:
                r = _recv(timeout=200)
                if r == 'go':
                    go0_ok = True
                    break
                if r == 'end':
                    go0_abort = True
                    break
            if go0_abort:
                print('[P1] 收到 end, 退出 P1')
                if led: led.value(0)
                continue
            if not go0_ok:
                cam_uart.send_string('err1_3')
                print('[P1] 等待 go 超时 -> err1_3')
                if led: led.value(0)
                continue
    
            _switch_upper_cam(P1_CONFIG['cam_width'], P1_CONFIG['cam_height'], fps=P1_CONFIG['cam_fps'])
            p1_hw = P1_CONFIG['cam_width'] / 2.0
            p1_hh = P1_CONFIG['cam_height'] / 2.0
    
            def _p1_center_ok(d):
                """目标中心距图像边缘至少 center_margin_px 像素才保留。"""
                cm = P1_CONFIG['center_margin_px']
                cx_d, cy_d = d['bbox'][0], d['bbox'][1]
                return (cm <= cx_d <= P1_CONFIG['cam_width'] - cm and
                        cm <= cy_d <= P1_CONFIG['cam_height'] - cm)
    
            # ---- Phase 0: 快速搜索 ----
            print('[P1] Phase0: 快速搜索开始（一帧识别到目标即锁定）')
            target_locked = False
            fail_streak = 0
            fail_total = 0
            p0_frame = 0
            scan_frames = 0
            scan_limit = P1_CONFIG['scan_limit_frames']
            p1_aborted = False
            while not target_locked and not app.need_exit():
                cmd = _recv(timeout=0)
                if cmd == 'end':
                    print('[P1] Phase0: 收到 end, 退出 P1')
                    p1_aborted = True
                    break
                if cmd is not None:
                    print('[P1] Phase0: 忽略指令: ' + cmd)
                p0_frame += 1
                img = _safe_cam_read(upper_cam)
                if img is None:
                    fail_streak += 1
                    fail_total += 1
                    if fail_streak >= P1_CONFIG['fail_streak_reinit']:
                        print('[P1] 连续{}帧None, 重初始化相机...'.format(fail_streak))
                        if not _reinit_upper_cam(P1_CONFIG['cam_width'], P1_CONFIG['cam_height'], fps=P1_CONFIG['cam_fps']):
                            break
                        fail_streak = 0
                    if fail_total >= P1_CONFIG['fail_total_max']:
                        cam_uart.send_string('err1_1')
                        print('[P1] Phase0: 累计{}帧None -> err1_1'.format(fail_total))
                        break
                    continue
                fail_streak = 0
    
                # 搜索过程也实时显示画面（含十字线）
                _draw_crosshair(img, int(p1_hw), int(p1_hh), DISPLAY_CONFIG['crosshair_len'])
    
                detections = detector.detect(img)
                best = None
                best_dist = float('inf')
                for d in detections:
                    if d['class_id'] != target_cls_id:
                        continue
                    if not _p1_center_ok(d):
                        continue
                    cx_d, cy_d = d['bbox'][0], d['bbox'][1]
                    dist = (cx_d - p1_hw) ** 2 + (cy_d - p1_hh) ** 2
                    if dist < best_dist:
                        best_dist = dist
                        best = d
    
                if best is not None:
                    _draw_obb_corners(img, best['corners'],
                                      CLASS_COLORS.get(best['class_name'], CLR_WHITE))
                    img.draw_string(4, 4, 'P1 Phase0: FOUND', CLR_GREEN)
                    _remember_maix_frame(img)
                    target_locked = True
                    print('[P1] Phase0: 目标锁定 (帧={})'.format(p0_frame))
                else:
                    scan_frames += 1
                    if scan_frames >= scan_limit:
                        cam_uart.send_string('mv')
                        print('[P1] Phase0: {} 帧未检测到目标, 发送 mv 请求切换视野'.format(scan_limit))
                        scan_frames = 0
                        mv_ok = False
                        mv_abort = False
                        mv_to = time.ticks_ms()
                        while time.ticks_diff(time.ticks_ms(), mv_to) < P1_CONFIG['wait_go_timeout_ms']:
                            r = _recv(timeout=200)
                            if r == 'go':
                                mv_ok = True
                                break
                            if r == 'end':
                                mv_abort = True
                                break
                        if mv_abort:
                            print('[P1] Phase0: 收到 end, 退出 P1')
                            p1_aborted = True
                            break
                        if not mv_ok:
                            cam_uart.send_string('err1_3')
                            print('[P1] Phase0: 等待 mv 后 go 超时 -> err1_3')
                            p1_aborted = True
                            break
                disp.show(img)
    
            if not target_locked:
                if led: led.value(0)
                if p1_aborted:
                    print('[P1] Phase0: 退出 P1 回到主循环')
                    _last_p0_time = time.ticks_ms()
                else:
                    print('[P1] Phase0: 未锁定目标, exit')
                _upload_last_frame('[P1]')
                continue
    
            cam_uart.send_string('stp')
            print('[P1] Phase0: 发送 stp, 等待主机发 go...')
            go_timeout = time.ticks_ms()
            got_go = False
            while time.ticks_diff(time.ticks_ms(), go_timeout) < P1_CONFIG['wait_go_timeout_ms']:
                r = _recv(timeout=200)
                if r == 'go':
                    got_go = True
                    break
                if r == 'end':
                    print('[P1] 收到 end, 中止')
                    break
            if not got_go:
                cam_uart.send_string('err1_3')
                print('[P1] 等待 go 超时 -> err1_3')
                if led: led.value(0)
                _upload_last_frame('[P1]')
                continue
    
            # ---- Phase 1: 第一次检测记录全部目标，每个目标采集 init_frames 帧平均 ----
            print('[P1] Phase1: 记录全部目标并对每个目标采集{}帧平均'.format(P1_CONFIG['init_frames']))
            p1_targets = [None]  # [targets]; targets = [{cx, cy, w, h, class_name, det, dx[], dy[], ang[]}]
            fail_total_1 = 0
            sent_err_1 = False
            last_done = 0
    
            def _p1_calc_angle(maix_img, d):
                """返回单个目标的检测角度，CLED 类别用 CV 修正。"""
                ang = d['bbox'][4]
                if d['class_name'] == 'cled' and np is not None:
                    raw_bytes = maix_img.to_bytes()
                    img_rgb = np.frombuffer(raw_bytes, dtype=np.uint8).reshape(
                        P1_CONFIG['cam_height'], P1_CONFIG['cam_width'], 3)
                    img_bgr = img_rgb[:, :, ::-1].copy()
                    crop, (obb_ox, obb_oy), obb_mask, obb_poly = crop_cled_obb(
                        img_bgr, d)
                    offset_deg, cv_debug = compute_cled_angle_offset(
                        crop, None, region_mask=obb_mask,
                        region_polygon=obb_poly,
                        center_hint=(d['bbox'][0] - obb_ox, d['bbox'][1] - obb_oy),
                        box_size=(d['bbox'][2], d['bbox'][3]))
                    if offset_deg is not None:
                        ang = offset_deg
                    if cv_debug is not None:
                        d['cled_center'] = (
                            cv_debug['center_roi'][0] + obb_ox,
                            cv_debug['center_roi'][1] + obb_oy,
                        )
                        d['cled_notch'] = (
                            cv_debug['notch_roi'][0] + obb_ox,
                            cv_debug['notch_roi'][1] + obb_oy,
                        )
                return ang
    
            def _p1_sample_frame(maix_img, detections):
                """第一次检测建立全部目标列表，之后按位置匹配累计每个目标的样本。"""
                found = [d for d in detections
                         if d['class_id'] == target_cls_id and _p1_center_ok(d)]
                if p1_targets[0] is None:
                    if not found:
                        return
                    targets = []
                    for d in found:
                        targets.append({
                            'cx': d['bbox'][0],
                            'cy': d['bbox'][1],
                            'w': d['bbox'][2],
                            'h': d['bbox'][3],
                            'class_name': d['class_name'],
                            'det': d,
                            'dx': [],
                            'dy': [],
                            'ang': [],
                        })
                    # N1 为离视野中心最近的目标，其余按距离由近到远排列
                    targets.sort(key=lambda t: (t['cx'] - p1_hw) ** 2 + (t['cy'] - p1_hh) ** 2)
                    p1_targets[0] = targets
                found = [d for d in detections
                         if d['class_id'] == target_cls_id and _p1_center_ok(d)]
                used = set()
                for t in p1_targets[0]:
                    if len(t['dx']) >= P1_CONFIG['init_frames']:
                        continue
                    best_i = None
                    best_dd = float('inf')
                    for i, d in enumerate(found):
                        if i in used:
                            continue
                        cx_d, cy_d = d['bbox'][0], d['bbox'][1]
                        dd = (cx_d - t['cx']) ** 2 + (cy_d - t['cy']) ** 2
                        if dd < best_dd:
                            best_dd = dd
                            best_i = i
                    if best_i is None:
                        continue
                    d = found[best_i]
                    thr = (max(t['w'], t['h']) * 1.5) ** 2
                    if best_dd > thr:
                        continue
                    used.add(best_i)
                    t['dx'].append(p1_hw - d['bbox'][0])
                    t['dy'].append(p1_hh - d['bbox'][1])
                    t['ang'].append(_p1_calc_angle(maix_img, d))
                    t['det'] = d
    
            while not app.need_exit():
                _check_mgmt()
                if p1_targets[0] is not None and all(
                        len(t['dx']) >= P1_CONFIG['init_frames'] for t in p1_targets[0]):
                    break
                img = _safe_cam_read(upper_cam)
                if img is None:
                    fail_total_1 += 1
                    if fail_total_1 >= P1_CONFIG['fail_total_max']:
                        cam_uart.send_string('err1_4')
                        print('[P1] Phase1: 相机失败 -> err1_4')
                        sent_err_1 = True
                        break
                    continue
                fail_total_1 = 0
    
                _draw_crosshair(img, int(p1_hw), int(p1_hh), DISPLAY_CONFIG['crosshair_len'])
                detections = detector.detect(img)
                _p1_sample_frame(img, detections)
                if p1_targets[0] is not None:
                    for t in p1_targets[0]:
                        _draw_obb_corners(img, t['det']['corners'],
                                          CLASS_COLORS.get(t['det']['class_name'], CLR_WHITE))
                        if "cled_center" in t['det']:
                            ccx, ccy = t['det']['cled_center']
                            ncx, ncy = t['det']['cled_notch']
                            img.draw_circle(int(ccx), int(ccy), 3, CLED_CENTER_COLOR, -1)
                            img.draw_circle(int(ncx), int(ncy), 3, CLED_NOTCH_COLOR, -1)
                            img.draw_line(int(ccx), int(ccy), int(ncx), int(ncy),
                                          CLED_LINE_COLOR, 2)
                    done = sum(1 for t in p1_targets[0]
                               if len(t['dx']) >= P1_CONFIG['init_frames'])
                    img.draw_string(4, 4, 'P1 Phase1: {}/{}'.format(
                        done, len(p1_targets[0])), CLR_GREEN)
                    if done != last_done:
                        print('[P1] Phase1: 采样进度 {}/{}'.format(done, len(p1_targets[0])))
                        last_done = done
                else:
                    img.draw_string(4, 4, 'P1 Phase1: SEARCHING', CLR_GREEN)
                _remember_maix_frame(img)
                disp.show(img)
    
            if p1_targets[0] is None:
                if not sent_err_1:
                    cam_uart.send_string('err1_5')
                    print('[P1] Phase1: 未检测到目标 -> err1_5')
                if led: led.value(0)
                _upload_last_frame('[P1]')
                continue
    
            targets = p1_targets[0]
            # ---- 上报全部目标偏差（每个目标取 init_frames 帧平均，一次发完） ----
            print('[P1] 上报 {} 个目标偏差...'.format(len(targets)))
            cam_uart.send_string('pos')
            cam_uart.send_number(len(targets))
            for idx, t in enumerate(targets, 1):
                avg_dx = sum(t['dx']) / len(t['dx'])
                avg_dy = sum(t['dy']) / len(t['dy'])
                if t['class_name'] == 'cled':
                    final_ang, _ = _circular_mean_deg_360(t['ang'])
                else:
                    final_ang, _ = _circular_mean_deg_180(t['ang'])
                send_dx = int(avg_dx * 38.1)
                send_dy = int(avg_dy * 38.1)
                final_ao = int(final_ang * 100)
                print('[P1] T{}: dx={:.1f}({}) dy={:.1f}({}) ang={:.1f} ao={}'.format(
                    idx, avg_dx, send_dx, avg_dy, send_dy, final_ang, final_ao))
                cam_uart.send_string('N{}:{}'.format(idx, send_dx))
                cam_uart.send_string('N{}:{}'.format(idx, send_dy))
                cam_uart.send_string('N{}:{}'.format(idx, final_ao))
            cam_uart.send_number(target_cls_id)
            cam_uart.send_string('end')
            print('[P1] ======== P1 完成（已退出，不进入循环修正） ========')
            _upload_last_frame('[P1]')
            if led: led.value(0)
            _switch_upper_cam(CAM_CONFIG['width'], CAM_CONFIG['height'], fps=CAM_CONFIG['fps'])
            continue
    
        # ============================================================
        # P2: Mark 多点寻标对位
        # ============================================================
        elif resp == 'p2':
            cam_uart.reset()
            print('[P2] ======== P2 START ========')
            _reset_upload_frame()
    
            _switch_upper_cam(P2_CONFIG['cam_width'], P2_CONFIG['cam_height'], fps=P2_CONFIG['cam_fps'])
            p2_hw = P2_CONFIG['cam_width'] / 2.0
            p2_hh = P2_CONFIG['cam_height'] / 2.0
    
            def _p2_cv_circularity(gray, cx, cy, margin=15):
                try:
                    h, w = gray.shape[:2]
                    x0 = max(0, int(cx - margin))
                    y0 = max(0, int(cy - margin))
                    x1 = min(w, int(cx + margin))
                    y1 = min(h, int(cy + margin))
                    if x1 <= x0 + 5 or y1 <= y0 + 5:
                        return 0.0
                    roi = gray[y0:y1, x0:x1]
                    _, binary = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    best = 0.0
                    for cnt in contours:
                        area = cv2.contourArea(cnt)
                        if area < 20:
                            continue
                        peri = cv2.arcLength(cnt, True)
                        if peri < 1e-6:
                            continue
                        circ = 4 * math.pi * area / (peri * peri)
                        if circ > best:
                            best = circ
                    return best
                except:
                    return 0.0
    
            def _p2_confidence(m):
                s = []
                cmin = P2_CONFIG['blob_circ_min']
                cv_min = P2_CONFIG['cv_circ_min']
                asp_max = P2_CONFIG['aspect_max']
                ami = P2_CONFIG['blob_area_min']
                ama = P2_CONFIG['blob_area_max']
                s.append(max(0.0, min(1.0, (m['roundness'] - cmin) / max(1.0 - cmin, 0.01))))
                s.append(max(0.0, min(1.0, (m['cv_circ'] - cv_min) / max(1.0 - cv_min, 0.01))))
                s.append(max(0.0, min(1.0, (asp_max - m['aspect']) / max(asp_max - 1.0, 0.01))))
                ac = (ami + ama) / 2.0
                ar = (ama - ami) / 2.0
                s.append(max(0.0, 1.0 - abs(m['area'] - ac) / max(ar, 1.0)))
                return sum(s) / len(s)
    
            def _p2_subpixel_refine(gray, cx, cy, margin=None):
                if margin is None:
                    margin = P2_CONFIG['subpixel_margin']
                if np is None:
                    return None
                h, w = gray.shape[:2]
                x0 = max(0, int(cx - margin))
                y0 = max(0, int(cy - margin))
                x1 = min(w, int(cx + margin))
                y1 = min(h, int(cy + margin))
                if x1 <= x0 + 3 or y1 <= y0 + 3:
                    return None
                roi = gray[y0:y1, x0:x1]
                th, _ = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                if th <= 0:
                    return None
                mask = roi > th
                if np.sum(mask) < 4:
                    return None
                wgt = roi.astype(np.int16) - th
                wgt[~mask] = 0
                sm = float(np.sum(wgt))
                if sm < 1e-6:
                    return None
                cols = np.arange(roi.shape[1], dtype=np.float32) + 0.5
                rows = np.arange(roi.shape[0], dtype=np.float32) + 0.5
                sx = float(np.sum(np.sum(wgt, axis=0) * cols))
                sy = float(np.sum(np.sum(wgt, axis=1) * rows))
                return (x0 + sx / sm, y0 + sy / sm)
    
            # ---- P2 Mark 检测函数（对齐 p2_mark 检测逻辑）----
            def _p2_detect_mark(maix_img):
                try:
                    crop_r = P2_CONFIG['crop_ratio']
                    cam_w = P2_CONFIG['cam_width']
                    cam_h = P2_CONFIG['cam_height']
    
                    crop_w = int(cam_w * crop_r)
                    crop_h = int(cam_h * crop_r)
                    crop_x = (cam_w - crop_w) // 2
                    crop_y = (cam_h - crop_h) // 2
    
                    cv_full = image.image2cv(maix_img, ensure_bgr=True, copy=True)
                    cv_crop = cv_full[crop_y:crop_y + crop_h, crop_x:crop_x + crop_w]
                    gy = cv2.cvtColor(cv_crop, cv2.COLOR_BGR2GRAY)
    
                    gy_blur = cv2.GaussianBlur(gy, (5, 5), 0)
                    edges = cv2.Canny(gy_blur, P2_CONFIG['canny_low'], P2_CONFIG['canny_high'])
                    edges[0:3, :] = 0
                    edges[-3:, :] = 0
                    edges[:, 0:3] = 0
                    edges[:, -3:] = 0
                    edge_pixels = cv2.countNonZero(edges)
                    if edge_pixels < P2_CONFIG['canny_min_pixels']:
                        _, binary = cv2.threshold(gy_blur, 0, 255,
                                                  cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                    else:
                        ck = int(P2_CONFIG.get('contour_close_k', 1))
                        if ck % 2 == 0:
                            ck += 1
                        if ck > 1 and np is not None:
                            binary = cv2.morphologyEx(edges, cv2.MORPH_CLOSE,
                                                      np.ones((ck, ck), np.uint8))
                        else:
                            binary = edges
    
                    cnts_raw, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL,
                                                   cv2.CHAIN_APPROX_SIMPLE)
                    all_blobs = []
                    for cnt in cnts_raw:
                        area = float(cv2.contourArea(cnt))
                        if area < P2_CONFIG['blob_pixel_th']:
                            continue
                        x, y, w, h = cv2.boundingRect(cnt)
                        cx = x + w / 2.0
                        cy = y + h / 2.0
                        perimeter = cv2.arcLength(cnt, True)
                        roundness_val = (4.0 * math.pi * area / (perimeter * perimeter)
                                         if perimeter > 1e-6 else 0.0)
                        rect = cv2.minAreaRect(cnt)
                        corners = cv2.boxPoints(rect)
                        all_blobs.append({
                            'cx': cx, 'cy': cy, 'w': w, 'h': h,
                            'area': area, 'roundness': roundness_val,
                            'corners': corners,
                        })
    
                    ami = P2_CONFIG['blob_area_min']
                    ama = P2_CONFIG['blob_area_max']
                    cmin = P2_CONFIG['blob_circ_min']
                    cv_min = P2_CONFIG['cv_circ_min']
                    asp_max = P2_CONFIG['aspect_max']
                    ext_max = P2_CONFIG['extent_max']
                    em = P2_CONFIG['edge_margin']
                    conf_min = P2_CONFIG['conf_min']
    
                    marks = []
                    for b in all_blobs:
                        a = b['area']
                        if not (ami <= a <= ama):
                            continue
                        br = b['roundness']
                        if br < cmin:
                            continue
                        corners = b['corners']
                        if corners is None or len(corners) != 4:
                            continue
                        rect = cv2.minAreaRect(np.array(corners, dtype=np.float32))
                        rw, rh = rect[1]
                        if rw < 1 or rh < 1:
                            continue
                        aspect = max(rw, rh) / min(rw, rh)
                        if aspect > asp_max:
                            continue
                        extent = a / (rw * rh) if (rw * rh) > 0 else 1.0
                        if extent > ext_max:
                            continue
                        cv_circ = _p2_cv_circularity(gy, b['cx'], b['cy'])
                        if cv_circ < cv_min:
                            continue
                        if (b['cx'] < em or b['cx'] > crop_w - em or
                                b['cy'] < em or b['cy'] > crop_h - em):
                            continue
                        marks.append({'cx': float(b['cx']), 'cy': float(b['cy']),
                                      'area': a, 'roundness': br,
                                      'cv_circ': cv_circ, 'aspect': aspect,
                                      'extent': extent})
    
                    for m in marks:
                        m['conf'] = _p2_confidence(m)
                    marks = [m for m in marks if m['conf'] >= conf_min]
    
                    for m in marks:
                        m['cx'] = m['cx'] + crop_x
                        m['cy'] = m['cy'] + crop_y
    
                    if marks:
                        marks.sort(key=lambda c: (c['cx'] - p2_hw) ** 2 + (c['cy'] - p2_hh) ** 2)
                        best = marks[0]
                        bcx = best['cx'] - crop_x
                        bcy = best['cy'] - crop_y
                        r = _p2_subpixel_refine(gy, bcx, bcy)
                        if r and abs(r[0] - bcx) < 5 and abs(r[1] - bcy) < 5:
                            best['cx'] = r[0] + crop_x
                            best['cy'] = r[1] + crop_y
                        return True, best
                    return False, None
                except Exception as e:
                    print('[P2] detect error: ' + str(e))
                    return False, None
    
            def _p2_draw_mark(maix_img, mark, label):
                mx = int(mark['cx']); my = int(mark['cy'])
                maix_img.draw_circle(mx, my, 6, CLR_GREEN, 2)
                maix_img.draw_circle(mx, my, 2, CLR_GREEN, -1)
                maix_img.draw_line(mx - 12, my, mx + 12, my, CLR_GREEN, 1)
                maix_img.draw_line(mx, my - 12, mx, my + 12, CLR_GREEN, 1)
                maix_img.draw_string(4, 4, label, CLR_GREEN)
                _remember_maix_frame(maix_img)
                disp.show(maix_img)
    
            cam_uart.send_string('rdy')
            print('[P2] 发送 rdy')
    
            state = 'searching'
            mark_idx = 0
            align_iter = 0
    
            while mark_idx < P2_CONFIG['target_count'] and not app.need_exit():
                _check_mgmt()
                print('[P2] Mark {}/{} state={}'.format(mark_idx + 1, P2_CONFIG['target_count'], state))
    
                if state == 'searching':
                    print('[P2] searching: 等待 go...')
                    go_ok = False
                    go_wt = time.ticks_ms()
                    while time.ticks_diff(time.ticks_ms(), go_wt) < 30000:
                        r = _recv(timeout=200)
                        if r == 'go': go_ok = True; break
                        if r == 'end': break
                    if not go_ok:
                        cam_uart.send_string('err2_1')
                        print('[P2] searching: 未收到 go -> err2_1')
                        break
    
                    found = False
                    for _ in range(P2_CONFIG['search_timeout_s'] * 5):
                        _check_mgmt()
                        img = _safe_cam_read(upper_cam)
                        if img is None: continue
                        disp.show(img)
                        has_m, mi = _p2_detect_mark(img)
                        if has_m:
                            print('[P2] Mark {} FOUND ({:.1f},{:.1f}) conf={:.2f}'.format(
                                mark_idx + 1, mi['cx'], mi['cy'], mi['conf']))
                            _p2_draw_mark(img, mi, 'P2 Mark {} FOUND'.format(mark_idx + 1))
                            found = True; break
                        time.sleep_ms(100)
                    if not found:
                        cam_uart.send_string('err2_3')
                        print('[P2] searching: 未检测到 Mark -> err2_3')
                        break
                    cam_uart.send_string('stp')
                    state = 'pos_detect'
    
                elif state == 'pos_detect':
                    print('[P2] pos_detect: 等待 go...')
                    go_ok = False
                    go_wt = time.ticks_ms()
                    while time.ticks_diff(time.ticks_ms(), go_wt) < 30000:
                        r = _recv(timeout=200)
                        if r == 'go': go_ok = True; break
                        if r == 'end': break
                    if not go_ok: break
    
                    pf = P2_CONFIG['pos_frames']
                    dev_th = P2_CONFIG['pos_dev_th']
                    cx_buf = []; cy_buf = []; col = 0; mt = P2_CONFIG['pos_max_try']
                    while col < pf and mt > 0:
                        _check_mgmt()
                        mt -= 1
                        img = _safe_cam_read(upper_cam)
                        if img is None: continue
                        has_m, mi = _p2_detect_mark(img)
                        if has_m:
                            _p2_draw_mark(img, mi, 'P2 Mark {} pos'.format(mark_idx + 1))
                            cx_buf.append(mi['cx']); cy_buf.append(mi['cy']); col += 1
                        if col == pf:
                            scx = sorted(cx_buf); mcx = scx[len(scx) // 2]
                            scy = sorted(cy_buf); mcy = scy[len(scy) // 2]
                            ok = True
                            for i in range(pf):
                                if abs(cx_buf[i] - mcx) > dev_th or abs(cy_buf[i] - mcy) > dev_th:
                                    ok = False
                                    break
                            if ok:
                                break
                            print('[P2] pos_detect: 5帧偏差超限, 重新采集 (剩余尝试 {})'.format(mt))
                            cx_buf = []; cy_buf = []; col = 0
                    if len(cx_buf) < pf:
                        cam_uart.send_string('err2_3')
                        print('[P2] pos_detect: 有效帧不足 ({}/{}) -> err2_3'.format(len(cx_buf), pf))
                        break
                    avg_cx = sum(cx_buf) / float(pf)
                    avg_cy = sum(cy_buf) / float(pf)
                    dx = p2_hw - avg_cx; dy = p2_hh - avg_cy
                    sdx = int(dx * 37.7); sdy = int(dy * 37.7)
                    print('[P2] pos: dx={:.1f}({}) dy={:.1f}({})'.format(dx, sdx, dy, sdy))
                    cam_uart.send_string('pos'); cam_uart.send_number(sdx); cam_uart.send_number(sdy); cam_uart.send_string('end')
                    state = 'aligning'; align_iter = 0
    
                elif state == 'aligning':
                    print('[P2] aligning: 等待 go...')
                    go_ok = False
                    go_wt = time.ticks_ms()
                    while time.ticks_diff(time.ticks_ms(), go_wt) < 30000:
                        r = _recv(timeout=200)
                        if r == 'go': go_ok = True; break
                        if r == 'end': break
                    if not go_ok: break
    
                    align_iter += 1
                    print('[P2] aligning iter {}/{}'.format(align_iter, P2_CONFIG['align_max_iter']))
                    af = 0; col = 0; dxb = []; dyb = []; mt = 5 * 3
                    while col < 5 and mt > 0:
                        _check_mgmt()
                        mt -= 1
                        img = _safe_cam_read(upper_cam)
                        if img is None: continue
                        has_m, mi = _p2_detect_mark(img)
                        if has_m:
                            _p2_draw_mark(img, mi, 'P2 Mark {} align'.format(mark_idx + 1))
                            dx = p2_hw - mi['cx']; dy = p2_hh - mi['cy']
                            col += 1
                            if abs(dx) < P2_CONFIG['align_th'] and abs(dy) < P2_CONFIG['align_th']: af += 1
                            dxb.append(dx); dyb.append(dy)
    
                    adx = sum(dxb) / max(1, len(dxb)); ady = sum(dyb) / max(1, len(dyb))
                    print('[P2] aligning iter {}/{}: avg dx={:.1f} dy={:.1f}'.format(
                        align_iter, P2_CONFIG['align_max_iter'], adx, ady))
                    if af >= 5:
                        cam_uart.send_string('ok')
                        print('[P2] aligning: 对准成功 -> ok')
                        state = 'searching'; mark_idx += 1
                        continue
                    else:
                        if align_iter >= P2_CONFIG['align_max_iter']:
                            cam_uart.send_string('err2_4')
                            print('[P2] aligning: 达到最大迭代次数 {} -> err2_4'.format(P2_CONFIG['align_max_iter']))
                            break
                        sdx = int(adx * 37.7); sdy = int(ady * 37.7)
                        cam_uart.send_string('pos'); cam_uart.send_number(sdx); cam_uart.send_number(sdy); cam_uart.send_string('end')
    
            print('[P2] ======== P2 完成 ========')
            _upload_last_frame('[P2]')
            _switch_upper_cam(CAM_CONFIG['width'], CAM_CONFIG['height'], fps=CAM_CONFIG['fps'])
            continue
    
        # ============================================================
        # P3: 下相机单次检测对位
        # ============================================================
        elif resp == 'p3':
            cam_uart.reset()
            if led: led.value(1)
            print('[P3] ======== P3 START ========')
            _reset_upload_frame()
    
            if not _init_lower_camera():
                cam_uart.send_string('err3_1')
                print('[P3] 下相机初始化失败 -> err3_1')
                if led: led.value(0)
                _upload_last_frame('[P3]')
                continue
    
            p3_hw = P3_CONFIG['cam_width'] / 2.0
            p3_hh = P3_CONFIG['cam_height'] / 2.0
    
            # ---- 圆形检测辅助函数（用于吸嘴检查，基于p4_circle算法） ----
            def _p3_nozzle_has_circle(bgr):
                """p4_circle 暗色团块圆形检测（含中心颜色过滤）: 返回圆心半径或 None"""
                n_amn = P3_CONFIG['nozzle_area_min']
                n_amx = P3_CONFIG['nozzle_area_max']
                n_circ = P3_CONFIG['nozzle_circ_min']
                n_cb_max = P3_CONFIG['nozzle_center_bgr_max']
                n_blur_k = P3_CONFIG['nozzle_blur_ksize']
                if n_blur_k % 2 == 0: n_blur_k += 1
                gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
                blur = cv2.GaussianBlur(gray, (n_blur_k, n_blur_k), 0)
                for th in P3_CONFIG['nozzle_dark_thresholds']:
                    _, bi = cv2.threshold(blur, th, 255, cv2.THRESH_BINARY_INV)
                    cnts, _ = cv2.findContours(bi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    for cnt in cnts:
                        ca = cv2.contourArea(cnt)
                        if ca < n_amn * 0.43 or ca > n_amx * 1.74: continue
                        perim = cv2.arcLength(cnt, True)
                        if perim < 8: continue
                        circ = 4 * math.pi * ca / (perim * perim)
                        if circ < n_circ: continue
                        (cx, cy), cr = cv2.minEnclosingCircle(cnt)
                        if math.pi * cr * cr < n_amn * 0.33 or math.pi * cr * cr > n_amx * 1.74: continue
                        b, g, r = _get_center_bgr(bgr, cx, cy)
                        if b > n_cb_max[0] or g > n_cb_max[1] or r > n_cb_max[2]: continue
                        if not (n_amn <= math.pi * cr * cr <= n_amx): continue
                        return (cx, cy, cr)
                return None
    
            def _p3_show_frame(frame):
                """OpenCV BGR 帧转 Maix 图像后上屏，失败不阻塞主流程。"""
                try:
                    disp.show(image.cv2image(frame, bgr=True, copy=True))
                except Exception:
                    pass
    
            # Phase0: 吸嘴检查（反逻辑：检测到圆=空吸嘴，有料时圆被遮住）
            if P3_CONFIG['nozzle_enabled']:
                print('[P3] Phase0: 吸嘴检查（圆形检测，检测到圆=空吸嘴）')
                for _ in range(P3_CONFIG['nozzle_discard_frames']):
                    _check_mgmt()
                    p3_cap.read()
                    time.sleep_ms(20)
                nc = 0; nt = 0
                for _ in range(P3_CONFIG['nozzle_check_frames']):
                    _check_mgmt()
                    ret, frame = p3_cap.read()
                    if not ret: continue
                    nt += 1
                    circle = _p3_nozzle_has_circle(frame)
                    if circle is not None:
                        ccx, ccy, crr = circle
                        cv2.circle(frame, (int(ccx), int(ccy)), int(crr), (0, 0, 255), 2)
                        cv2.circle(frame, (int(ccx), int(ccy)), 2, (0, 0, 255), -1)
                        cv2.putText(frame, 'P3 Nozzle', (10, 25),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        _remember_cv_frame(frame)
                        _p3_show_frame(frame)
                        nc += 1
                print('[P3] Phase0: 检测到圆 {}/{} 帧'.format(nc, nt))
                # 反逻辑：检测到圆→空吸嘴→报错退出
                if nt > 0 and nc >= P3_CONFIG['nozzle_detect_threshold']:
                    cam_uart.send_string('err3_8')
                    print('[P3] Phase0: 吸嘴为空（检测到圆） -> err3_8')
                    if led: led.value(0)
                    _upload_last_frame('[P3]')
                    continue
    
            # Phase1: 矩形检测（使用p3的Canny+多边形逼近算法）
            print('[P3] Phase1: 矩形检测, 采集{}帧'.format(P3_CONFIG['avg_frames']))
    
            # ---- p3 矩形检测参数（参照p3/main.py） ----
            _APPROX_EPS_STEPS = (0.02, 0.03, 0.04, 0.06, 0.08)
            _ANGLE_LO = 50.0
            _ANGLE_HI = 130.0
            _CANNY_LO = int(P3_CONFIG.get('canny_low', 15))
            _CANNY_HI = int(P3_CONFIG.get('canny_high', 60))
            _DILATE_IT = 2
            _AMI = 2500
            _AMA = 30000
            _RMIN = 1.0
            _RMAX = 2.5
            _SMIN = float(P3_CONFIG.get('rect_solidity_min', 0.85))
            _TRACK_MATCH_DIST = 80.0
            _RECTNESS_MIN = 0.60
            _RECTNESS_MIN_EDGE = 0.58
            _RECTNESS_SORT_WEIGHT = 200.0
    
            def _p3_rect_feature_score(r, template):
                """与 p3/main.py 相同的特征相似度评分（越小越相似）。"""
                if template is None:
                    return 0.0
                da = abs(math.log(max(r['area'], 1.0) / max(template['area'], 1.0)))
                dw = abs(r['w'] - template['w']) / max(template['w'], 1.0)
                dh = abs(r['h'] - template['h']) / max(template['h'], 1.0)
                dr = abs(r['ratio'] - template['ratio'])
                ds = abs(r['solidity'] - template['solidity'])
                dq = abs(r['rectness'] - template['rectness'])
                return 3.0 * da + dw + dh + 1.5 * dr + 2.0 * ds + 2.0 * dq

            def _p3_select_best_index(rects, center, template, match_dist, relock=False):
                """按 p3/main.py 的评分选择最佳候选：特征分>6丢弃，距离超限丢弃。"""
                if center is None:
                    return -1
                best_i = -1
                best_score = None
                for i, r in enumerate(rects):
                    fs = _p3_rect_feature_score(r, template)
                    if template is not None and fs > 6.0:
                        continue
                    if relock and template is not None and fs > 5.0:
                        continue
                    d = math.hypot(r['cx'] - center[0], r['cy'] - center[1])
                    if not relock and template is not None and d > match_dist * 2.0:
                        continue
                    score = d / float(match_dist) + fs
                    if best_score is None or score < best_score:
                        best_score = score
                        best_i = i
                return best_i
    
            def _p3_preprocess_gray(gray):
                """Grayscale preprocess; disabled by config so bright targets are not clipped."""
                if not P3_CONFIG.get('gray_pre_enabled', True):
                    return gray
                if P3_CONFIG.get('gray_clahe_enabled', True):
                    try:
                        clip = P3_CONFIG.get('gray_clahe_clip_limit', 2.0)
                        tile = max(2, int(P3_CONFIG.get('gray_clahe_tile_grid', 8)))
                        clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(tile, tile))
                        gray = clahe.apply(gray)
                    except Exception:
                        pass
                cap = P3_CONFIG.get('gray_bright_cap', 0)
                if cap and cap > 0:
                    gray = cv2.min(gray, cap)
                return gray

            def _p3_detect_rectangles(cv_img, ref_rect=None):
                """p3 Canny+多边形逼近矩形检测"""
                try:
                    gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
                    gray = _p3_preprocess_gray(gray)
                    blur = cv2.GaussianBlur(gray, (P3_CONFIG['blur_ksize'], P3_CONFIG['blur_ksize']), 0)
                    edges = cv2.Canny(blur, _CANNY_LO, _CANNY_HI)
                    k = np.ones((P3_CONFIG['dilate_ksize'], P3_CONFIG['dilate_ksize']), np.uint8)
                    binary = cv2.dilate(edges, k, iterations=_DILATE_IT)
                    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, k, iterations=1)
                    cnts, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                except Exception as e:
                    print('[P3] detect error: %s' % e)
                    return []

                rects = []
                print('[P3DBG] contours=%d' % len(cnts))
                for cnt in cnts:
                    area = float(cv2.contourArea(cnt))
                    if area < _AMI or area > _AMA:
                        print('[P3DBG] area fail %.1f' % area)
                        continue
                    peri = cv2.arcLength(cnt, True)
                    approx = None
                    for _eps in _APPROX_EPS_STEPS:
                        approx = cv2.approxPolyDP(cnt, _eps * peri, True)
                        if len(approx) == 4:
                            break
                    if len(approx) != 4:
                        print('[P3DBG] quad fail n=%d' % len(approx))
                        continue
                    (cx_r, cy_r), (rw, rh), ang = cv2.minAreaRect(cnt)
                    if rw < 1 or rh < 1:
                        print('[P3DBG] rect_size fail %.1f x %.1f' % (rw, rh))
                        continue
                    ratio = max(rw, rh) / min(rw, rh)
                    if ratio < _RMIN or ratio > _RMAX:
                        print('[P3DBG] ratio fail %.3f' % ratio)
                        continue
                    rect_area = rw * rh
                    rectness = area / rect_area if rect_area > 0.0 else 0.0
                    dist_norm = math.hypot(cx_r - p3_hw, cy_r - p3_hh) / math.hypot(p3_hw, p3_hh)
                    rqmin = _RECTNESS_MIN + (_RECTNESS_MIN_EDGE - _RECTNESS_MIN) * min(1.0, dist_norm)
                    if rectness < rqmin:
                        print('[P3DBG] rectness fail %.3f th %.3f dist %.2f' % (rectness, rqmin, dist_norm))
                        continue
                    hull = cv2.convexHull(cnt)
                    ha = cv2.contourArea(hull)
                    solid = area / ha if ha > 1e-6 else 0
                    if solid < _SMIN:
                        print('[P3DBG] solidity fail %.3f' % solid)
                        continue
                    pts = approx.reshape(4, 2).astype(np.float64)
                    ok = True
                    for i in range(4):
                        p0 = pts[i]; p1 = pts[(i+1)%4]; p2 = pts[(i+2)%4]
                        v1, v2 = p0 - p1, p2 - p1
                        d1 = math.sqrt(v1[0]**2 + v1[1]**2)
                        d2 = math.sqrt(v2[0]**2 + v2[1]**2)
                        if d1 < 1e-6 or d2 < 1e-6: ok = False; break
                        ca = max(-1., min(1., (v1[0]*v2[0]+v1[1]*v2[1])/(d1*d2)))
                        ad = math.degrees(math.acos(abs(ca)))
                        if ad < _ANGLE_LO or ad > _ANGLE_HI: ok = False; break
                    if not ok:
                        print('[P3DBG] angle fail ad=%.1f' % ad)
                        continue
                    # 角度归一化: 需要旋转多少度到达水平/垂直
                    ang_norm = ang % 90
                    if ang_norm <= 45: dev = ang_norm
                    else: dev = ang_norm - 90
                    rects.append({
                        'cx': float(cx_r), 'cy': float(cy_r),
                        'w': float(rw), 'h': float(rh),
                        'rot': -dev,
                        'area': area,
                        'ratio': ratio,
                        'solidity': solid,
                        'rectness': rectness,
                    })
                if ref_rect is not None and rects:
                    idx = _p3_select_best_index(
                        rects, (ref_rect.get('cx'), ref_rect.get('cy')),
                        ref_rect, _TRACK_MATCH_DIST)
                    rects = [rects[idx]] if idx >= 0 else []
                if rects:
                    rects.sort(key=lambda r: (r['cx']-p3_hw)**2 + (r['cy']-p3_hh)**2
                               - _RECTNESS_SORT_WEIGHT * r['rectness'])
                print('[P3DBG] rects=%d' % len(rects))
                return rects
    
            def _p3_median(vals):
                s = sorted(vals)
                n = len(s)
                if n % 2 == 1:
                    return s[n // 2]
                return (s[n // 2 - 1] + s[n // 2]) / 2.0
    
            def _p3_filter_outliers(dxb, dyb, angb, wb, hb, outlier_th, min_keep):
                """按中位数剔除中心偏差异常帧，返回过滤后的数据与保留索引。"""
                n = len(dxb)
                if n <= 1:
                    return dxb, dyb, angb, wb, hb, list(range(n))
                mdx = _p3_median(dxb)
                mdy = _p3_median(dyb)
                keep = [i for i in range(n)
                        if abs(dxb[i] - mdx) <= outlier_th and abs(dyb[i] - mdy) <= outlier_th]
                if len(keep) < min_keep:
                    keep = list(range(n))
                return ([dxb[i] for i in keep],
                        [dyb[i] for i in keep],
                        [angb[i] for i in keep],
                        [wb[i] for i in keep],
                        [hb[i] for i in keep],
                        keep)
    
            cx_buf = []; cy_buf = []; ang_buf = []; w_buf = []; h_buf = []
            sol_buf = []; q_buf = []
            col = 0; mt = P3_CONFIG['avg_max_try']; rf = 0
    
            while col < P3_CONFIG['avg_frames'] and mt > 0 and not app.need_exit():
                _check_mgmt()
                mt -= 1
                ret, frame = p3_cap.read()
                if not ret:
                    rf += 1
                    if rf >= 10:
                        print('[P3] 连续读取失败, 重初始化...')
                        if not _init_lower_camera(): break
                        rf = 0
                    continue
                rf = 0
    
                rects = _p3_detect_rectangles(frame)
                if rects:
                    best = rects[0]
                    fdx = best['cx'] - p3_hw
                    fdy = best['cy'] - p3_hh
                    cx_buf.append(best['cx']); cy_buf.append(best['cy'])
                    ang_buf.append(best['rot'])
                    w_buf.append(best['w']); h_buf.append(best['h'])
                    sol_buf.append(best['solidity']); q_buf.append(best['rectness'])
                    col += 1
                    bx = int(best['cx']); by = int(best['cy'])
                    cv2.circle(frame, (bx, by), 6, (0, 255, 0), 2)
                    cv2.circle(frame, (bx, by), 2, (0, 255, 0), -1)
                    cv2.line(frame, (bx - 15, by), (bx + 15, by), (0, 255, 0), 1)
                    cv2.line(frame, (bx, by - 15), (bx, by + 15), (0, 255, 0), 1)
                    cv2.putText(frame, 'P3 Rect {}'.format(col), (10, 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    _remember_cv_frame(frame)
                    _p3_show_frame(frame)
    
            if col < 1:
                cam_uart.send_string('err3_5')
                print('[P3] Phase1: 无检测结果 -> err3_5')
                if led: led.value(0)
                _upload_last_frame('[P3]')
                continue
    
            raw_n = len(cx_buf)
            dxb = [x - p3_hw for x in cx_buf]
            dyb = [y - p3_hh for y in cy_buf]
            fdxb, fdyb, fangb, fwb, fhb, keep = _p3_filter_outliers(
                dxb, dyb, ang_buf, w_buf, h_buf,
                P3_CONFIG['avg_outlier_th'], P3_CONFIG['avg_min_keep'])
            # 下相机仰视成像，dx/dy 方向与上相机相反
            dx = sum(fdxb) / len(fdxb); dy = sum(fdyb) / len(fdyb)
            fa, _ = _circular_mean_deg(fangb) if fangb else (0.0, 0)
            fao = int(fa * 100)
            avg_w = sum(fwb) / len(fwb) if fwb else 0.0
            avg_h = sum(fhb) / len(fhb) if fhb else 0.0
            solb = [sol_buf[i] for i in keep] if sol_buf else []
            qb = [q_buf[i] for i in keep] if q_buf else []
            ref_rect = {
                'area': avg_w * avg_h,
                'w': avg_w, 'h': avg_h,
                'long': max(avg_w, avg_h), 'short': min(avg_w, avg_h),
                'ratio': max(avg_w, avg_h) / min(avg_w, avg_h) if min(avg_w, avg_h) > 1e-6 else 0.0,
                'solidity': sum(solb) / len(solb) if solb else 0.0,
                'rectness': sum(qb) / len(qb) if qb else 0.0,
                'ang': fa,
                'cx': p3_hw + dx, 'cy': p3_hh + dy,
            }
            sdx = -int(dx * 13.5); sdy = -int(dy * 13.5)
            print('[P3] Phase1: keep={}/{} dx={:.1f}({}) dy={:.1f}({}) ang={:.1f} ao={}'.format(
                len(fdxb), raw_n, dx, sdx, dy, sdy, fa, fao))
    
            cam_uart.send_string('pos'); cam_uart.send_number(sdx); cam_uart.send_number(sdy)
            cam_uart.send_number(fao); cam_uart.send_string('end')
    
            # ---- 重复修正偏差（与 P2 aligning 类似） ----
            align_iter = 0
            aligned = False
            while not aligned and not app.need_exit():
                print('[P3] aligning: 等待 go...')
                go_ok = False
                go_wt = time.ticks_ms()
                while time.ticks_diff(time.ticks_ms(), go_wt) < P3_CONFIG['wait_go_timeout_ms']:
                    r = _recv(timeout=200)
                    if r == 'go':
                        go_ok = True
                        break
                    if r == 'end':
                        break
                if not go_ok:
                    cam_uart.send_string('err3_6')
                    print('[P3] aligning: 等待 go 超时 -> err3_6')
                    break
    
                align_iter += 1
                print('[P3] aligning iter {}/{}'.format(align_iter, P3_CONFIG['align_max_iter']))
                col = 0
                dxb = []
                dyb = []
                angb = []
                wb = []
                hb = []
                sol_buf = []
                q_buf = []
                mt = P3_CONFIG['align_max_try']
                while col < P3_CONFIG['align_frames'] and mt > 0:
                    _check_mgmt()
                    mt -= 1
                    ret, frame = p3_cap.read()
                    if not ret:
                        continue
                    rects = _p3_detect_rectangles(frame, ref_rect)
                    if not rects:
                        continue
                    best = rects[0]
                    fdx = best['cx'] - p3_hw
                    fdy = best['cy'] - p3_hh
                    col += 1
                    dxb.append(fdx)
                    dyb.append(fdy)
                    angb.append(best['rot'])
                    wb.append(best['w']); hb.append(best['h'])
                    sol_buf.append(best['solidity']); q_buf.append(best['rectness'])
                    bx = int(best['cx']); by = int(best['cy'])
                    cv2.circle(frame, (bx, by), 6, (0, 255, 0), 2)
                    cv2.circle(frame, (bx, by), 2, (0, 255, 0), -1)
                    cv2.line(frame, (bx - 15, by), (bx + 15, by), (0, 255, 0), 1)
                    cv2.line(frame, (bx, by - 15), (bx, by + 15), (0, 255, 0), 1)
                    cv2.putText(frame, 'P3 Align {}'.format(col), (10, 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    _remember_cv_frame(frame)
                    _p3_show_frame(frame)
    
                raw_n = len(dxb)
                fdxb, fdyb, fangb, fwb, fhb, keep = _p3_filter_outliers(
                    dxb, dyb, angb, wb, hb,
                    P3_CONFIG['align_outlier_th'], P3_CONFIG['align_min_keep'])
                adx = sum(fdxb) / max(1, len(fdxb))
                ady = sum(fdyb) / max(1, len(fdyb))
                aang, _ = _circular_mean_deg(fangb) if fangb else (fa, 0)
                aao = int(aang * 100)
                if col < P3_CONFIG['align_frames']:
                    print('[P3] aligning iter {}/{}: valid={}/{} (特征评分过滤后不足)'.format(
                        align_iter, P3_CONFIG['align_max_iter'], col, P3_CONFIG['align_frames']))
                print('[P3] aligning iter {}/{}: keep={}/{} avg dx={:.1f} dy={:.1f} ang={:.1f} ao={}'.format(
                    align_iter, P3_CONFIG['align_max_iter'], len(fdxb), raw_n, adx, ady, aang, aao))
                if fwb:
                    aw = sum(fwb) / len(fwb); ah = sum(fhb) / len(fhb)
                    solb = [sol_buf[i] for i in keep] if sol_buf else []
                    qb = [q_buf[i] for i in keep] if q_buf else []
                    ref_rect = {
                        'area': aw * ah,
                        'w': aw, 'h': ah,
                        'long': max(aw, ah), 'short': min(aw, ah),
                        'ratio': max(aw, ah) / min(aw, ah) if min(aw, ah) > 1e-6 else 0.0,
                        'solidity': sum(solb) / len(solb) if solb else 0.0,
                        'rectness': sum(qb) / len(qb) if qb else 0.0,
                        'ang': aang,
                        'cx': p3_hw + adx, 'cy': p3_hh + ady,
                    }
    
                if abs(adx) < P3_CONFIG['align_th'] and abs(ady) < P3_CONFIG['align_th']:
                    cam_uart.send_string('ok')
                    print('[P3] aligning: 对准成功 -> ok')
                    aligned = True
                    break
                if align_iter >= P3_CONFIG['align_max_iter']:
                    cam_uart.send_string('err3_7')
                    print('[P3] aligning: 达到最大迭代次数 -> err3_7')
                    break
                sdx = -int(adx * 13.5)
                sdy = -int(ady * 13.5)
                cam_uart.send_string('pos')
                cam_uart.send_number(sdx)
                cam_uart.send_number(sdy)
                cam_uart.send_number(aao)
                cam_uart.send_string('end')
            print('[P3] ======== P3 完成 ========')
            _upload_last_frame('[P3]')
            if led: led.value(0)
            continue
    
        # ============================================================
        # P4: 下相机圆形标定对位（基于p4_circle检测逻辑 + 稳定性投票）
        # ============================================================
        elif resp == 'p4':
            cam_uart.reset()
            if led: led.value(1)
            print('[P4] ======== P4 START ========')
            _reset_upload_frame()
    
            if not _init_lower_camera():
                cam_uart.send_string('err4_1'); print('[P4] 相机失败 -> err4_1')
                if led: led.value(0)
                _upload_last_frame('[P4]')
                continue
    
            p4_hw = P3_CONFIG['cam_width'] / 2.0; p4_hh = P3_CONFIG['cam_height'] / 2.0

            # P4 进程开始后先丢弃若干帧，画面稳定后再识别
            for _ in range(P4_CONFIG['discard_frames']):
                _check_mgmt()
                p3_cap.read()
                time.sleep_ms(20)
            print('[P4] 已丢弃 {} 帧，开始识别'.format(P4_CONFIG['discard_frames']))
    
            def _p4_detect_circles(bgr_frame):
                """p4_circle 圆形检测（暗色团块 + 面积过滤 + 中心颜色过滤）"""
                gray = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2GRAY)
                k = P4_CONFIG['blur_ksize']
                if k % 2 == 0: k += 1
                blur = cv2.GaussianBlur(gray, (k, k), 0)
                amn = P4_CONFIG['circle_area_min']; amx = P4_CONFIG['circle_area_max']
                cb_max = P4_CONFIG['center_bgr_max']
                all_c = []
                for th in P4_CONFIG['dark_thresholds']:
                    _, bi = cv2.threshold(blur, th, 255, cv2.THRESH_BINARY_INV)
                    cnts, _ = cv2.findContours(bi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    for cnt in cnts:
                        ca = cv2.contourArea(cnt)
                        if ca < amn * 0.43 or ca > amx * 1.74: continue
                        perim = cv2.arcLength(cnt, True)
                        if perim < 8: continue
                        circ = 4 * math.pi * ca / (perim * perim)
                        if circ < 0.54: continue
                        (cx, cy), cr = cv2.minEnclosingCircle(cnt)
                        if math.pi * cr * cr < amn * 0.33 or math.pi * cr * cr > amx * 1.74: continue
                        b, g, r = _get_center_bgr(bgr_frame, cx, cy)
                        if b > cb_max[0] or g > cb_max[1] or r > cb_max[2]: continue
                        all_c.append([cx, cy, cr])
                af = []
                for c in all_c:
                    area = math.pi * c[2] * c[2]
                    if amn <= area <= amx: af.append(c)
                if np is not None and af:
                    return True, np.array(af, dtype=np.float32)
                elif af: return True, af
                return False, None
    
            for iteration in range(P4_CONFIG['max_iter']):
                _check_mgmt()
                print('[P4] Iter {}/{}'.format(iteration + 1, P4_CONFIG['max_iter']))
                stable_center = None
                stability_count = 0
                unstability_count = 0
                vote_history = []
                tf = 0
                # 单轮检测最多 180 帧（2026-08-10 由 120 调整）
                rl = 180
    
                while tf < rl and not app.need_exit():
                    _check_mgmt()
                    ret, frame = p3_cap.read(); tf += 1
                    if not ret: continue
                    hc, circles = _p4_detect_circles(frame)
                    if hc and circles is not None and len(circles) > 0:
                        if np is not None and hasattr(circles, 'ndim'):
                            bi = np.argmin((circles[:, 0] - p4_hw) ** 2 + (circles[:, 1] - p4_hh) ** 2)
                            cx, cy = float(circles[bi][0]), float(circles[bi][1])
                        else:
                            bd = float('inf'); bi = 0
                            for ii, c in enumerate(circles):
                                d = (c[0] - p4_hw) ** 2 + (c[1] - p4_hh) ** 2
                                if d < bd: bd = d; bi = ii
                            cx, cy = float(circles[bi][0]), float(circles[bi][1])
                        cv2.circle(frame, (int(cx), int(cy)), 8, (0, 255, 0), 2)
                        cv2.circle(frame, (int(cx), int(cy)), 2, (0, 255, 0), -1)
                        cv2.line(frame, (int(cx) - 16, int(cy)), (int(cx) + 16, int(cy)),
                                 (0, 255, 0), 1)
                        cv2.line(frame, (int(cx), int(cy) - 16), (int(cx), int(cy) + 16),
                                 (0, 255, 0), 1)
                        cv2.putText(frame, 'P4 Circle', (10, 25),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        _remember_cv_frame(frame)
                        # 使用原始中心（与p4_circle一致，无EMA）
                        stable_center = (cx, cy)
                        stability_count += 1
                        unstability_count = 0
                    else:
                        unstability_count += 1
    
                    if tf % 5 == 0:
                        try:
                            disp_img = image.cv2image(frame, bgr=True, copy=True)
                            disp_img.draw_line(int(p4_hw) - 20, int(p4_hh),
                                              int(p4_hw) + 20, int(p4_hh), CLR_GREEN, 1)
                            disp_img.draw_line(int(p4_hw), int(p4_hh) - 20,
                                              int(p4_hw), int(p4_hh) + 20, CLR_GREEN, 1)
                            disp_img.draw_string(10, 10, 'P4 iter {}/{}'.format(
                                iteration + 1, P4_CONFIG['max_iter']), CLR_GREEN, 2)
                            disp.show(disp_img)
                        except Exception:
                            pass
    
                    if unstability_count >= P4_CONFIG['reset_frames']:
                        stable_center = None; stability_count = 0
                        unstability_count = 0; vote_history = []
    
                    circle_stable = stability_count >= P4_CONFIG['stability_frames']
                    vote_history.append(circle_stable)
                    vf = P4_CONFIG['vote_frames']
                    if len(vote_history) > vf: vote_history.pop(0)
    
                    # 每5帧打印一次数据（与p4_circle一致）
                    if circle_stable and stability_count % 5 == 0:
                        sdx = stable_center[0] - p4_hw
                        sdy = stable_center[1] - p4_hh
                        print('[P4] frame=%d dx=%.1f dy=%.1f st=%d votes=%d/%d' % (
                            tf, sdx, sdy, stability_count,
                            sum(1 for h in vote_history if h), len(vote_history)))
    
                    if len(vote_history) >= vf:
                        pos_v = sum(1 for h in vote_history if h)
                        if pos_v / len(vote_history) >= P4_CONFIG['vote_ratio'] and stable_center is not None:
                            dx = stable_center[0] - p4_hw
                            dy = stable_center[1] - p4_hh
                            print('[P4] Iter {}: center=({:.1f},{:.1f}) dx={:.1f} dy={:.1f} vote={}/{}'.format(
                                iteration + 1, stable_center[0], stable_center[1], dx, dy, pos_v, len(vote_history)))
                            break
                else:
                    if iteration == 0: cam_uart.send_string('err4_2')
                    else: cam_uart.send_string('err4_3')
                    break
    
                if abs(dx) < P4_CONFIG['align_threshold'] and abs(dy) < P4_CONFIG['align_threshold']:
                    cam_uart.send_string('ok')
                    print('[P4] ALIGNED: |dx|={:.1f} |dy|={:.1f} < {}'.format(
                        abs(dx), abs(dy), P4_CONFIG['align_threshold']))
                    break
    
                # 下相机仰视成像，直接发送像素偏差（方向与上相机相反）
                sdx = -int(dx * 13.5); sdy = -int(dy * 13.5)
                cam_uart.send_string('pos'); cam_uart.send_number(sdx); cam_uart.send_number(sdy); cam_uart.send_string('end')
    
                if iteration < P4_CONFIG['max_iter'] - 1:
                    gw = time.ticks_ms(); go_ok = False
                    while time.ticks_diff(time.ticks_ms(), gw) < P4_CONFIG['wait_go_timeout_ms']:
                        r = _recv(timeout=200)
                        if r == 'go': go_ok = True; break
                        if r == 'end': break
                    if not go_ok:
                        cam_uart.send_string('err4_5'); break
            else:
                cam_uart.send_string('err4_4')
            print('[P4] ======== P4 完成 ========')
            _upload_last_frame('[P4]')
            if led: led.value(0)
            continue
    
        # ============================================================
        # 未知指令
        # ============================================================
        else:
            print('[MAIN] 未知指令: ' + resp)
            continue
    except _P0Rehandshake:
        print('[P0] 已返回 P0 握手阶段（主循环）')
        if led:
            led.value(0)
        _last_p0_time = time.ticks_ms()
        continue

# ============================================================
# 程序退出清理
# ============================================================
_cleanup_exit()

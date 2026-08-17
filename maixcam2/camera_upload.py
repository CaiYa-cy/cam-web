# ============================================================
# MaixCAM2 camera upload - non-blocking (background thread)
# 320x320 JPEG frames uploaded via HTTP POST to cloud server
# ============================================================

import gc
from maix import camera, display, app, time
try:
    import _thread
    _HAS_THREAD = True
except ImportError:
    _HAS_THREAD = False
try:
    import urequests as requests
except ImportError:
    import requests

# ============================================================
# Config
# ============================================================
CLOUD_URL = 'http://124.222.148.162:8080/camera/upload'
CAM_WIDTH = 320
CAM_HEIGHT = 320
CAM_FPS = 60
JPEG_QUALITY = 65
UPLOAD_TIMEOUT = 2

# ============================================================
# Shared state between main thread and upload thread
# ============================================================
_latest_jpeg = None   # latest encoded JPEG bytes
_frame_id = 0         # incremented by main thread on each new frame
_sent_id = 0          # last frame id sent by upload thread
_running = True


def _img_to_jpeg_bytes(img):
    """Convert maix image to JPEG bytes, trying multiple methods."""
    jpeg_img = img.to_jpeg(quality=JPEG_QUALITY)
    try:
        return jpeg_img.to_bytes()
    except AttributeError:
        pass
    try:
        return jpeg_img.tobytes()
    except AttributeError:
        pass
    return bytes(jpeg_img)


def _upload_thread():
    """Background thread: continuously upload latest JPEG frame."""
    global _latest_jpeg, _frame_id, _sent_id, _running
    ok = 0
    fail = 0
    last_stat = time.ticks_ms()

    while _running:
        if _latest_jpeg is not None and _frame_id != _sent_id:
            data = _latest_jpeg
            sid = _frame_id
            try:
                r = requests.post(CLOUD_URL, data=data, timeout=UPLOAD_TIMEOUT)
                if r.status_code == 200:
                    ok += 1
                else:
                    fail += 1
                r.close()
            except Exception as e:
                fail += 1
                if fail <= 3 or fail % 100 == 0:
                    print('[CamUp] err: ' + str(e))
            _sent_id = sid
        else:
            time.sleep_ms(10)

        # stats every 3 seconds
        now = time.ticks_ms()
        if time.ticks_diff(now, last_stat) >= 3000:
            print('[CamUp] OK:' + str(ok) + ' FAIL:' + str(fail) + ' fps:~' + str(ok // 3))
            ok = 0
            fail = 0
            last_stat = now


# ============================================================
# Init
# ============================================================
print('[Camera] init camera 320x320 @ 60fps')
cam = camera.Camera(width=CAM_WIDTH, height=CAM_HEIGHT, fps=CAM_FPS)

disp = None
try:
    disp = display.Display()
except Exception:
    print('[Camera] no display')

print('[Camera] target: ' + CLOUD_URL)

# ============================================================
# Start upload thread
# ============================================================
if _HAS_THREAD:
    _thread.start_new_thread(_upload_thread, ())
    print('[Camera] upload thread started (non-blocking)')
else:
    print('[Camera] WARNING: no _thread, upload will block main loop')

# ============================================================
# Main loop: capture + encode only
# ============================================================
frame_total = 0
last_fps_print = time.ticks_ms()

print('[Camera] started')

while not app.need_exit():
    img = cam.read()
    if img is None:
        time.sleep_ms(5)
        continue

    frame_total += 1
    if frame_total == 1:
        print('[Camera] first frame captured')

    # encode to JPEG and share with upload thread
    try:
        _latest_jpeg = _img_to_jpeg_bytes(img)
        _frame_id = frame_total
    except Exception as e:
        print('[Camera] encode err: ' + str(e))

    # display preview
    if disp is not None:
        try:
            disp.show(img)
        except Exception:
            pass

    # capture FPS (separate from upload FPS)
    now = time.ticks_ms()
    if time.ticks_diff(now, last_fps_print) >= 3000:
        fps = frame_total / (time.ticks_diff(now, last_fps_print) / 1000.0)
        print('[Camera] capture: {:.1f} fps'.format(fps))
        frame_total = 0
        last_fps_print = now

    del img
    gc.collect()

_running = False
print('[Camera] exit')

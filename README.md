# cam&web

贴片机视觉定位与网页远程控制程序。

## 目录

- `ESP_web/`：ESP32 Web 固件（`camweb.ino`）、浏览器控制页（`index.html`）、云端中转服务（`relay_server.py`）
- `maixcam2/`：MaixCAM2 视觉定位程序，包含 YOLO11-OBB 检测、Canny/圆标定对位、串口通信与云帧上传

## 说明

仓库只包含程序源码与模型文件，不包含任何密钥或账号凭据。

#!/usr/bin/env python3
# relay_server.py - 贴片机云端中转服务（含摄像头帧转发）
# 监听 8080 端口，负责 HTTP <-> WebSocket 双向转发

import json
import asyncio
import time
import zlib
from collections import deque
from aiohttp import web

# ==================== 配置 ====================
WS_PORT = 8080              # 监听端口
COMMAND_QUEUE_SIZE = 10     # 指令队列上限
COMMAND_DEDUP_SECONDS = 3   # 相同指令去重窗口
FILE_LIMIT = 102400         # CSV 文件上限（原始字节）
FILE_CHUNK_SIZE = 121       # 单块字节数
FILE_EXPIRE_SECONDS = 30 * 60   # 30 分钟无拉取活动即过期
FILE_EXPIRE_SCAN_SECONDS = 60   # 后台清理扫描间隔

# ==================== 全局状态 ====================
ws_clients = set()                       # 已连接的浏览器 WebSocket
last_command_seen = {}                   # 指令去重时间戳
commands_queue = deque(maxlen=COMMAND_QUEUE_SIZE)  # 待下发指令
latest_frame = None                      # 最新摄像头帧（bytes）
camera_clients = set()                   # 摄像头 WebSocket 客户端

# 内存暂存的 CSV 文件（同一时刻只保留最新版本）
csv_file = {
    "id": "csv-current",
    "version": 0,
    "data": None,          # None 表示文件不存在/已过期
    "size": 0,
    "crc32": "",
    "uploaded_at": None,
    "last_pull_at": None,
}

# ==================== CORS ====================
CORS_HEADERS = {
    "Access-Control-Allow-Origin": "*",
    "Access-Control-Allow-Methods": "GET, POST, OPTIONS",
    "Access-Control-Allow-Headers": "Content-Type, X-Upload-Token",
}

@web.middleware
async def cors_middleware(request, handler):
    if request.method == "OPTIONS":
        return web.Response(status=200, headers=CORS_HEADERS)
    response = await handler(request)
    for key, value in CORS_HEADERS.items():
        response.headers[key] = value
    return response

# ==================== CSV 文件过期清理 ====================
def csv_file_exists():
    return csv_file["data"] is not None

def lazy_expire_csv_file():
    """惰性检查：30 分钟无拉取活动则清除暂存文件。"""
    if not csv_file_exists():
        return
    if csv_file["last_pull_at"] is None:
        return
    if time.time() - csv_file["last_pull_at"] > FILE_EXPIRE_SECONDS:
        print("[CSV] 文件已过期，清除内存暂存")
        csv_file["data"] = None
        csv_file["size"] = 0
        csv_file["crc32"] = ""
        csv_file["last_pull_at"] = None

async def expire_loop():
    """后台任务：每 60 秒扫描一次过期文件。"""
    while True:
        await asyncio.sleep(FILE_EXPIRE_SCAN_SECONDS)
        lazy_expire_csv_file()

async def start_background_tasks(app):
    app["expire_task"] = asyncio.create_task(expire_loop())

# ==================== CSV 文件接口 ====================
# 网页: POST /upload，body 为原始文件字节
def _normalize_csv_utf8(data: bytes) -> bytes:
    # Strip UTF-8/UTF-16 BOM and convert UTF-16 content to UTF-8.
    if data.startswith(b"\xef\xbb\xbf"):
        return data[3:]
    if data.startswith(b"\xff\xfe"):
        return data[2:].decode("utf-16-le").encode("utf-8")
    if data.startswith(b"\xfe\xff"):
        return data[2:].decode("utf-16-be").encode("utf-8")
    return data

async def upload_handler(request):
    lazy_expire_csv_file()
    body = await request.read()
    try:
        body = _normalize_csv_utf8(body)
    except UnicodeDecodeError:
        return web.json_response(
            {"ok": False, "error": "invalid utf-16 content"},
            status=400,
        )
    if len(body) > FILE_LIMIT:
        return web.json_response(
            {"ok": False, "error": "file too large"},
            status=413,
        )

    csv_file["version"] += 1
    csv_file["data"] = body
    csv_file["size"] = len(body)
    csv_file["crc32"] = format(zlib.crc32(body) & 0xFFFFFFFF, "08x")
    csv_file["uploaded_at"] = time.time()
    csv_file["last_pull_at"] = time.time()

    print(f"[CSV] 上传成功: version={csv_file['version']} size={csv_file['size']} crc32={csv_file['crc32']}")
    return web.json_response({
        "ok": True,
        "id": csv_file["id"],
        "version": csv_file["version"],
        "size": csv_file["size"],
        "crc32": csv_file["crc32"],
    })

# ESP32: GET /file?version=V&offset=N，返回原始二进制块
async def file_handler(request):
    lazy_expire_csv_file()

    if not csv_file_exists():
        return web.json_response({"ok": False, "error": "file not found"}, status=404)

    try:
        version = int(request.query.get("version", "0"))
    except ValueError:
        version = -1
    try:
        offset = int(request.query.get("offset", "0"))
    except ValueError:
        offset = 0

    if version != csv_file["version"]:
        return web.json_response(
            {
                "ok": False,
                "error": "version mismatch",
                "version": csv_file["version"],
            },
            status=409,
        )

    if offset < 0:
        offset = 0
    data = csv_file["data"]
    size = csv_file["size"]

    if offset >= size:
        csv_file["last_pull_at"] = time.time()
        return web.Response(
            body=b"",
            headers={
                "Content-Type": "application/octet-stream",
                "X-Version": str(csv_file["version"]),
                "X-Size": str(size),
                "X-Offset": str(offset),
                "X-Next-Offset": str(offset),
                "X-Crc32": csv_file["crc32"],
                "X-Done": "1",
            },
        )

    chunk = data[offset:offset + FILE_CHUNK_SIZE]
    csv_file["last_pull_at"] = time.time()
    return web.Response(
        body=chunk,
        headers={
            "Content-Type": "application/octet-stream",
            "X-Version": str(csv_file["version"]),
            "X-Size": str(size),
            "X-Offset": str(offset),
            "X-Next-Offset": str(offset + len(chunk)),
            "X-Crc32": csv_file["crc32"],
            "X-Done": "0",
        },
    )

# ==================== WebSocket: ESP32 数据端点 ====================
# 浏览器连接: ws://服务器IP:8080/ws

async def ws_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    ws_clients.add(ws)
    print(f"[WS] 客户端连接, 当前在线: {len(ws_clients)}")
    try:
        async for msg in ws:
            if msg.type == web.WSMsgType.TEXT:
                print(f"[WS] 收到原始消息: {msg.data}")
                try:
                    cmd = json.loads(msg.data)
                    if isinstance(cmd, dict):
                        in_queue = any(
                            isinstance(c, dict)
                            and c.get("cmd") == cmd.get("cmd")
                            and c.get("subcmd") == cmd.get("subcmd")
                            and c.get("payload", "") == cmd.get("payload", "")
                            for c in commands_queue
                        )
                        key = (cmd.get("cmd"), cmd.get("subcmd"), cmd.get("payload", ""))
                        now = time.time()
                        recent = last_command_seen.get(key, 0)
                        if in_queue or (recent and now - recent < COMMAND_DEDUP_SECONDS):
                            last_command_seen[key] = now
                            print(f"[WS] 忽略重复指令: {cmd}")
                        else:
                            last_command_seen[key] = now
                            commands_queue.append(cmd)
                            print(f"[WS] 收到指令: {cmd}")
                    else:
                        print(f"[WS] 非对象 JSON, 忽略: {msg.data}")
                except json.JSONDecodeError:
                    print(f"[WS] 非法 JSON: {msg.data}")
            elif msg.type == web.WSMsgType.ERROR:
                print(f"[WS] 错误: {ws.exception()}")
    finally:
        ws_clients.discard(ws)
        print(f"[WS] 客户端断开, 当前在线: {len(ws_clients)}")
    return ws

# ==================== WebSocket: 摄像头端点 ====================
# 浏览器连接: ws://服务器IP:8080/camera

async def camera_ws_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    camera_clients.add(ws)
    print(f"[CAM] 客户端连接, 当前在线: {len(camera_clients)}")

    # 如果有最新帧，立即发送
    if latest_frame is not None:
        try:
            await ws.send_bytes(latest_frame)
        except Exception:
            pass

    try:
        async for msg in ws:
            pass  # 网页端不发消息，只接收
    finally:
        camera_clients.discard(ws)
        print(f"[CAM] 客户端断开, 当前在线: {len(camera_clients)}")
    return ws

# ==================== HTTP POST: MaixCAM2 上传帧 ====================
# MaixCAM2 调用: POST http://服务器IP:8080/camera/upload

async def camera_upload_handler(request):
    global latest_frame
    try:
        data = await request.read()
        if len(data) > 0:
            latest_frame = data
            # 广播给所有摄像头客户端
            dead = set()
            for ws in camera_clients.copy():
                try:
                    await ws.send_bytes(data)
                except Exception:
                    dead.add(ws)
            camera_clients.difference_update(dead)
        return web.Response(text="OK")
    except Exception as e:
        return web.Response(text=f"ERR:{e}", status=500)

# ==================== HTTP POST 端点 ====================
# ESP32 调用此端点: POST http://服务器IP:8080/update

async def _broadcast_update(data):
    dead = set()
    for ws in ws_clients.copy():
        try:
            await asyncio.wait_for(ws.send_json(data), timeout=1.0)
        except Exception:
            dead.add(ws)
    if dead:
        ws_clients.difference_update(dead)
    if ws_clients:
        print(f"[UPDATE] 已广播给 {len(ws_clients)} 个浏览器")

async def update_handler(request):
    try:
        data = await request.json()
    except Exception:
        return web.json_response({"error": "invalid json"}, status=400)

    print(f"[UPDATE] 收到 ESP32 数据: {json.dumps(data, ensure_ascii=False, sort_keys=True)}")

    # Broadcast in a background task so a slow browser cannot stall /update.
    asyncio.create_task(_broadcast_update(data))

    # Return queued commands to ESP32.
    if commands_queue:
        response = {"commands": list(commands_queue)}
        commands_queue.clear()
        print(f"[UPDATE] 返回 {len(response['commands'])} 条待执行指令")
    else:
        response = {"commands": []}
    return web.json_response(response)

# ==================== 启动 ====================
def main():
    app = web.Application()
    app.middlewares.append(cors_middleware)
    app.on_startup.append(start_background_tasks)
    app.router.add_get("/ws", ws_handler)
    app.router.add_post("/update", update_handler)
    app.router.add_post("/upload", upload_handler)
    app.router.add_get("/file", file_handler)
    app.router.add_get("/camera", camera_ws_handler)
    app.router.add_post("/camera/upload", camera_upload_handler)

    print(f"[Server] 中转服务启动, 监听 0.0.0.0:{WS_PORT}")
    web.run_app(app, host="0.0.0.0", port=WS_PORT)

if __name__ == "__main__":
    main()

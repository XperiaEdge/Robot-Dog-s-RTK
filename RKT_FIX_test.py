#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import base64
import time
import serial
import datetime
import threading
import json
import asyncio
import websockets
from pyproj import Transformer

# ===================== 串口配置 =====================
SERIAL_PORT = "/dev/ttyCH341USB0"
BAUDRATE = 115200

serport = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
serport.flushInput()
time.sleep(0.5)

# ===================== 全局变量 =====================
heading_yaw = None
GGA_STRING = None
latest_rtk_data = None
rtk_data_lock = threading.Lock()

# ===================== WebSocket =====================
WEBSOCKET_HOST = "0.0.0.0"
WEBSOCKET_PORT = 8765
connected_clients = set()
websocket_loop = None

# ===================== NTRIP =====================
NTRIP_IP = "120.253.239.161"
NTRIP_PORT = 8002
NTRIP_USERNAME = "csar11061"
NTRIP_PASSWORD = "a8888628"
NTRIP_MOUNT_POINT = "RTCM33_GRCEJ"

# ===================== 坐标系 =====================
def get_utm_epsg(lon, lat):
    zone = int((lon + 180) / 6) + 1
    return (32600 + zone) if lat >= 0 else (32700 + zone), zone

# ===================== NTRIP 连接 =====================
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((NTRIP_IP, NTRIP_PORT))

auth = base64.b64encode(
    f"{NTRIP_USERNAME}:{NTRIP_PASSWORD}".encode()
).decode()

request = (
    f"GET /{NTRIP_MOUNT_POINT} HTTP/1.0\r\n"
    f"User-Agent: RTK-PY\r\n"
    f"Authorization: Basic {auth}\r\n\r\n"
)

sock.send(request.encode())
resp = sock.recv(1024).decode(errors="ignore")
if "200 OK" not in resp:
    raise RuntimeError("NTRIP login failed")

print("[NTRIP] Connected")

# ===================== WebSocket 处理 =====================
async def websocket_handler(ws):
    connected_clients.add(ws)
    try:
        if latest_rtk_data:
            await ws.send(json.dumps(latest_rtk_data, ensure_ascii=False))
        async for _ in ws:
            pass
    finally:
        connected_clients.discard(ws)

def start_websocket():
    global websocket_loop
    websocket_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(websocket_loop)

    async def run_server():
        async with websockets.serve(
            websocket_handler,
            WEBSOCKET_HOST,
            WEBSOCKET_PORT
        ):
            print(f"[WebSocket] ws://{WEBSOCKET_HOST}:{WEBSOCKET_PORT}")
            await asyncio.Future()  # 永久运行

    websocket_loop.run_until_complete(run_server())
    websocket_loop.run_forever()

def broadcast(data):
    global latest_rtk_data
    with rtk_data_lock:
        latest_rtk_data = data

    if websocket_loop is None:
        return

    for ws in list(connected_clients):
        asyncio.run_coroutine_threadsafe(
            ws.send(json.dumps(data, ensure_ascii=False)),
            websocket_loop
        )

# ===================== 任务1：NTRIP（发 GGA + 收 RTCM） =====================
def task_ntrip():
    global GGA_STRING
    while True:
        if GGA_STRING:
            try:
                sock.send(GGA_STRING.encode())
                rtcm = sock.recv(4096)
                if rtcm:
                    serport.write(rtcm)
            except Exception as e:
                print("[NTRIP ERROR]", e)
        time.sleep(1)

# ===================== 任务2：读取 RTK =====================
def task_rtk():
    global GGA_STRING, heading_yaw

    while True:
        rx = serport.readline()
        if not rx:
            continue

        # 航向角（双天线）
        if rx.startswith((b"$GPHDT", b"$GNHDT")):
            try:
                heading_yaw = float(rx.decode().split(",")[1])
            except:
                pass

        # 只处理 GGA
        if not rx.startswith(b"$GNGGA"):
            continue

        line = rx.decode(errors="ignore").strip().split(",")

        if len(line) < 7 or not line[2] or not line[4]:
            continue

        # 实时 GGA 给 NTRIP
        GGA_STRING = rx.decode()

        lat = float(line[2][:2]) + float(line[2][2:]) / 60.0
        lon = float(line[4][:3]) + float(line[4][3:]) / 60.0
        if line[3] == "S":
            lat = -lat
        if line[5] == "W":
            lon = -lon

        fix = int(line[6])

        epsg, zone = get_utm_epsg(lon, lat)
        transformer = Transformer.from_crs(
            "EPSG:4326",
            f"EPSG:{epsg}",
            always_xy=True
        )
        utm_x, utm_y = transformer.transform(lon, lat)

        now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        rtk_data = {
            "time": now,
            "longitude": round(lon, 6),
            "latitude": round(lat, 6),
            "utm_x": round(utm_x, 3),
            "utm_y": round(utm_y, 3),
            "yaw": None if heading_yaw is None else round(heading_yaw, 2),
            "fix": fix
        }

        broadcast(rtk_data)

        with open("/tmp/rtk_position.json", "w") as f:
            json.dump(
                {
                    "utm_x": rtk_data["utm_x"],
                    "utm_y": rtk_data["utm_y"],
                    "fix": fix
                },
                f
            )

        print(f"[RTK] FIX={fix}  Lon={lon:.6f}  Lat={lat:.6f}")

# ===================== 启动 =====================
threading.Thread(target=start_websocket, daemon=True).start()
threading.Thread(target=task_ntrip, daemon=True).start()
threading.Thread(target=task_rtk, daemon=True).start()

while True:
    time.sleep(1)

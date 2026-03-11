import socket
import base64
import time
import serial
import datetime
import threading
import math
from pyproj import Transformer
import matplotlib.pyplot as plt

serport = serial.Serial("COM4", 115200)  # 端口根据你实际使用的进行更换
serport.flushInput()
time.sleep(0.5)  # 单位是s

last_utm_x = None
last_utm_y = None
heading_deg = None

# ======== 新增：地图原点 & 轨迹 ========
origin_utm_x = None
origin_utm_y = None
map_x = []
map_y = []

# 计算EPSG
def get_utm_epsg(lon, lat):
    zone = int((lon + 180) / 6) + 1
    if lat >= 0:
        epsg = 32600 + zone  # 北半球
    else:
        epsg = 32700 + zone  # 南半球
    return epsg, zone


# ================= NTRIP 原代码（完全未动） =================
NTRIP_IP = "120.253.239.161"
NTRIP_PORT = 8002
NTRIP_USERNAME = "csar11061"
NTRIP_PASSWORD = "a8888628"
NTRIP_MOUNT_POINT = "RTCM33_GRCEJ"
GGA_STRING = "$GNGGA,115713.000,3149.301528,N,11706.920684,E,1,17,0.88,98.7,M,-3.6,M,,*58\r\n"

try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((NTRIP_IP, NTRIP_PORT))
    print("Connected.")
except socket.error:
    print("Connect fail,exit.")
    exit(1)

Authorization = (NTRIP_USERNAME + ":" + NTRIP_PASSWORD)
Authorization = base64.b64encode(Authorization.encode()).decode()
requestHead = f"""GET /{NTRIP_MOUNT_POINT} HTTP/1.0\r
User-Agent: MOZIHAO-GNSS\r
Accept: */*\r
Connection: close\r
Authorization: Basic {Authorization}\r
\r
"""
sock.send(requestHead.encode())
time.sleep(0.2)
response = sock.recv(1024)
if "ICY 200 OK" not in response.decode():
    print("login to NTRIP caster fail,exit.")
    exit(1)
print('login to NTRIP caster successful.')

# ================= 线程1：NTRIP → 串口（未动） =================
def task1():
    while True:
        global GGA_STRING
        sock.send(GGA_STRING.encode())
        response = sock.recv(4096)
        serport.write(response)
        time.sleep(1)

# ================= 线程2：RTK 解算（仅加几行） =================
def task2():
    global GGA_STRING, last_utm_x, last_utm_y, heading_deg
    global origin_utm_x, origin_utm_y

    while True:
        rxData = serport.readline()
        if rxData.find(b'$GNGGA') == -1:
            continue

        GGA_STRING = rxData.decode('utf-8')
        line = GGA_STRING.split(',')

        if len(line) > 6 and line[2] and line[4]:
            weidu = float(line[2][:2]) + float(line[2][2:]) / 60
            jingdu = float(line[4][:3]) + float(line[4][3:]) / 60

            if line[3] == 'S':
                weidu = -weidu
            if line[5] == 'W':
                jingdu = -jingdu

            epsg, zone = get_utm_epsg(jingdu, weidu)
            transformer = Transformer.from_crs(
                "EPSG:4326",
                f"EPSG:{epsg}",
                always_xy=True
            )
            utm_x, utm_y = transformer.transform(jingdu, weidu)

            # ===== 航向角（你原逻辑，未改） =====
            if last_utm_x is not None:
                dx = utm_x - last_utm_x
                dy = utm_y - last_utm_y
                distance = math.hypot(dx, dy)

                if (line[6] == '4' or line[6] == '3') and distance > 0.3:
                    heading_rad = math.atan2(dx, dy)
                    heading_deg = math.degrees(heading_rad)
                    if heading_deg < 0:
                        heading_deg += 360

            last_utm_x = utm_x
            last_utm_y = utm_y

            # ===== 新增：设定地图原点 =====
            if origin_utm_x is None and line[6] == '4':
                origin_utm_x = utm_x
                origin_utm_y = utm_y
                print("地图原点已设定")

            # ===== 新增：记录相对地图坐标 =====
            if origin_utm_x is not None and line[6] in ['4', '5']:
                map_x.append(utm_x - origin_utm_x)
                map_y.append(utm_y - origin_utm_y)

            # ===== 输出（原样） =====
            print(f"UTM Zone: {zone}")
            print(f"Easting: {utm_x:.3f}")
            print(f"Northing: {utm_y:.3f}")
            print("longitude:", '{:.6f}'.format(jingdu))
            print("latitude:", '{:.6f}'.format(weidu))
            print("当前时间：", datetime.datetime.now())
            if heading_deg is not None:
                print(f"航向角 Heading: {heading_deg:.2f}°")

# ================= 新增线程3：地图绘制 =================
def plot_rtk_map():
    plt.ion()
    fig, ax = plt.subplots()

    margin = 2.0  # 边界留白（米）

    while True:
        if len(map_x) > 1:
            ax.clear()
            ax.set_title("RTK Local Map (Auto Scale)")
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.set_aspect('equal', adjustable='box')

            # 绘制路径
            ax.plot(map_x, map_y, linewidth=2)
            ax.scatter(map_x[-1], map_y[-1], s=40)  # 当前点

            # ===== 核心：自动缩放到全部路径 =====
            min_x = min(map_x)
            max_x = max(map_x)
            min_y = min(map_y)
            max_y = max(map_y)

            ax.set_xlim(min_x - margin, max_x + margin)
            ax.set_ylim(min_y - margin, max_y + margin)

            ax.grid(True)
            plt.pause(0.3)

        time.sleep(0.3)


# ================= 启动（仅多一个线程） =================
thread1 = threading.Thread(target=task1)
thread2 = threading.Thread(target=task2)
thread3 = threading.Thread(target=plot_rtk_map, daemon=True)

thread1.start()
thread2.start()
thread3.start()

plt.show()


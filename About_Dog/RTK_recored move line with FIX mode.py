import socket
import base64
import time
import serial
import datetime
import threading
import math
from pyproj import Transformer
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrow



serport = serial.Serial("/dev/ttyCH341USB0", 115200)  # 端口根据你实际使用的进行更换
serport.flushInput()
time.sleep(0.5)  # 单位是s

last_utm_x = None
last_utm_y = None
heading_deg = None   # ===== 最终使用的航向角（来自 KSXT） =====

# ================= 新增：自动发送 KSXT 1 =================
def send_ksxt_cmd():
    """
    自动开启双天线姿态输出 KSXT 1
    上电后只发一次
    """
    time.sleep(2)  # 等串口和接收机完全稳定
    try:
        serport.write(b"Ksxt 1\r\n")
        print("[INIT] KSXT 1 command sent")
    except Exception as e:
        print("[INIT] KSXT send failed:", e)

# ======== 新增：KSXT 姿态 ========
ksxt_heading = None
ksxt_pitch = None
ksxt_roll = None
ksxt_status = 0
gga_quality = 0

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

# ================= 新增：KSXT 解析 =================
def parse_ksxt(line):
    """
    解析 KSXT 双天线姿态
    返回：heading, pitch, roll, status
    """
    try:
        if not line.startswith("$KSXT"):
            return None

        data = line.split(',')

        # 基本长度保护
        if len(data) < 11:
            return None

        heading = float(data[5])   # 航向角（真北，顺时针）
        pitch   = float(data[6])
        roll    = float(data[7])
        status  = int(data[10])    # 姿态解算状态

        return heading, pitch, roll, status
    except Exception as e:
        return None


# ================= 线程2：RTK 解算（加 KSXT，不改原逻辑） =================
def task2():
    global GGA_STRING, last_utm_x, last_utm_y, heading_deg
    global origin_utm_x, origin_utm_y
    global ksxt_heading, ksxt_pitch, ksxt_roll, ksxt_status, gga_quality

    while True:
        rxData = serport.readline()
        line_raw = rxData.decode('utf-8', errors='ignore').strip()

        # ===== KSXT 航向 =====
        ksxt = parse_ksxt(line_raw)
        if ksxt:
            ksxt_heading, ksxt_pitch, ksxt_roll, ksxt_status = ksxt

            # 只有 FIX + KSXT 有效才更新最终航向
            if gga_quality == 4 and ksxt_status >= 2:
                if ksxt_heading != 0:
                       heading_deg = ksxt_heading

        # ===== 原 GGA 逻辑（完全保留） =====
        if line_raw.find('$GNGGA') == -1:
            continue

        GGA_STRING = line_raw + "\r\n"
        line = GGA_STRING.split(',')

        if len(line) > 6:
            gga_quality = int(line[6]) if line[6].isdigit() else 0

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

            # ===== 输出（原样 + KSXT） =====
            print(f"UTM Zone: {zone}")
            print(f"Easting: {utm_x:.3f}")
            print(f"Northing: {utm_y:.3f}")
            print("longitude:", '{:.6f}'.format(jingdu))
            print("latitude:", '{:.6f}'.format(weidu))
            print("GGA Quality:", gga_quality)
            print("当前时间：", datetime.datetime.now())

            if heading_deg is not None:
                print(f"航向角 Heading(KSXT): {heading_deg:.2f}°")
            else:
                print("航向角 Heading(KSXT): 无")

# ================= 地图绘制线程（未动） =================
def plot_rtk_map():
    plt.ion()
    fig, ax = plt.subplots()
    margin = 2.0

    while True:
        if len(map_x) > 1:
            ax.clear()
            ax.set_title("RTK Local Map (Auto Scale)")
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.set_aspect('equal', adjustable='box')

            # ===== 绘制路径 =====
            ax.plot(map_x, map_y, linewidth=2)

            # ===== 当前点 =====
            cur_x = map_x[-1]
            cur_y = map_y[-1]
            ax.scatter(cur_x, cur_y, s=30)

            # ===== 自动缩放 =====
            min_x = min(map_x)
            max_x = max(map_x)
            min_y = min(map_y)
            max_y = max(map_y)

            ax.set_xlim(min_x - margin, max_x + margin)
            ax.set_ylim(min_y - margin, max_y + margin)
            ax.grid(True)

            # =================================================
            # 右上角固定「圆形指南针 + 航向箭头（KSXT）」
            # =================================================
            if heading_deg is not None:
                # —— 指南针参数（Axes 坐标系）——
                compass_x = 0.88
                compass_y = 0.85
                compass_r = 0.08

                # 画圆
                compass_circle = Circle(
                    (compass_x, compass_y),
                    compass_r,
                    transform=ax.transAxes,
                    fill=False,
                    linewidth=2
                )
                ax.add_patch(compass_circle)

                # KSXT：0°=北，顺时针 → matplotlib 角度
                angle_rad = math.radians(90 - heading_deg)

                # 箭头向量
                arrow_len = compass_r * 0.85
                dx = arrow_len * math.cos(angle_rad)
                dy = arrow_len * math.sin(angle_rad)

                # 画箭头
                arrow = FancyArrow(
                    compass_x,
                    compass_y,
                    dx,
                    dy,
                    width=0.005,
                    length_includes_head=True,
                    head_width=0.02,
                    head_length=0.03,
                    transform=ax.transAxes,
                    color='red'
                )
                ax.add_patch(arrow)

                # 标注 N
                ax.text(
                    compass_x,
                    compass_y + compass_r + 0.03,
                    "N",
                    transform=ax.transAxes,
                    ha='center',
                    va='center',
                    fontsize=10,
                    fontweight='bold'
                )

                # 显示航向角数值
                ax.text(
                    compass_x,
                    compass_y - compass_r - 0.05,
                    f"{heading_deg:.1f}°",
                    transform=ax.transAxes,
                    ha='center',
                    va='center',
                    fontsize=9
                )

            plt.pause(0.3)

        time.sleep(0.3)


# ================= 启动 =================
thread0 = threading.Thread(target=send_ksxt_cmd, daemon=True)
thread1 = threading.Thread(target=task1)
thread2 = threading.Thread(target=task2)
thread3 = threading.Thread(target=plot_rtk_map, daemon=True)

thread0.start()
thread1.start()
thread2.start()
thread3.start()

plt.show()

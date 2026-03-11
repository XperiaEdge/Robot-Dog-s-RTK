import time
import serial
import datetime
import threading
import math
from pyproj import Transformer
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrow


# ================= 串口 =================
serport = serial.Serial("COM5", 115200, timeout=1)
serport.flushInput()
time.sleep(0.5)

last_utm_x = None
last_utm_y = None
heading_deg = None

gga_quality = 0

origin_utm_x = None
origin_utm_y = None
map_x = []
map_y = []


# ================= UTM EPSG =================
def get_utm_epsg(lon, lat):
    zone = int((lon + 180) / 6) + 1
    if lat >= 0:
        epsg = 32600 + zone
    else:
        epsg = 32700 + zone
    return epsg, zone


# ================= AGRICA 解析 =================
def parse_agrica(line):
    """
    解析 Unicore AGRICA 报文
    返回：
        heading, lat, lon, rtk_status, heading_status
    """
    try:
        if "#AGRICA" not in line:
            return None

        # 取 GNSS 部分
        data_part = line.split(";")[1]
        data_part = data_part.split("*")[0]

        fields = data_part.split(",")

        if len(fields) < 35:
            return None

        rtk_status = int(fields[8])
        heading_status = int(fields[9])

        heading = float(fields[19])
        lat = float(fields[29])
        lon = float(fields[30])

        return heading, lat, lon, rtk_status, heading_status

    except:
        return None


# ================= 主线程：读取 AGRICA =================
def task2():
    global last_utm_x, last_utm_y, heading_deg
    global origin_utm_x, origin_utm_y
    global map_x, map_y, gga_quality

    while True:
        line_raw = serport.readline().decode(errors='ignore').strip()

        result = parse_agrica(line_raw)
        if not result:
            continue

        heading, lat, lon, rtk_status, heading_status = result

        gga_quality = rtk_status

        # 只有 RTK FIX 才更新
        if rtk_status == 4 and heading_status in [4, 5]:
            heading_deg = heading

        # 转 UTM
        epsg, zone = get_utm_epsg(lon, lat)
        transformer = Transformer.from_crs(
            "EPSG:4326",
            f"EPSG:{epsg}",
            always_xy=True
        )
        utm_x, utm_y = transformer.transform(lon, lat)

        last_utm_x = utm_x
        last_utm_y = utm_y

        # 设置地图原点
        if origin_utm_x is None and rtk_status == 4:
            origin_utm_x = utm_x
            origin_utm_y = utm_y
            print("地图原点已设定")

        # 记录轨迹
        if origin_utm_x is not None and rtk_status in [4, 5]:
            map_x.append(utm_x - origin_utm_x)
            map_y.append(utm_y - origin_utm_y)

        # 输出
        print(f"UTM Zone: {zone}")
        print(f"Easting: {utm_x:.3f}")
        print(f"Northing: {utm_y:.3f}")
        print("longitude:", '{:.8f}'.format(lon))
        print("latitude:", '{:.8f}'.format(lat))
        print("RTK Status:", rtk_status)
        print("当前时间：", datetime.datetime.now())

        if heading_deg is not None:
            print(f"航向角 Heading(AGRICA): {heading_deg:.2f}°")
        else:
            print("航向角: 无效")
        print("-------------------------------------------------")


# ================= 地图绘制线程 =================
def plot_rtk_map():
    plt.ion()
    fig, ax = plt.subplots()
    margin = 2.0

    while True:
        if len(map_x) > 1:
            ax.clear()
            ax.set_title("RTK Local Map (AGRICA)")
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.set_aspect('equal', adjustable='box')

            ax.plot(map_x, map_y, linewidth=2)

            cur_x = map_x[-1]
            cur_y = map_y[-1]
            ax.scatter(cur_x, cur_y, s=30)

            min_x = min(map_x)
            max_x = max(map_x)
            min_y = min(map_y)
            max_y = max(map_y)

            ax.set_xlim(min_x - margin, max_x + margin)
            ax.set_ylim(min_y - margin, max_y + margin)
            ax.grid(True)

            # 指南针
            if heading_deg is not None:
                compass_x = 0.88
                compass_y = 0.85
                compass_r = 0.08

                compass_circle = Circle(
                    (compass_x, compass_y),
                    compass_r,
                    transform=ax.transAxes,
                    fill=False,
                    linewidth=2
                )
                ax.add_patch(compass_circle)

                angle_rad = math.radians(90 - heading_deg)
                arrow_len = compass_r * 0.85
                dx = arrow_len * math.cos(angle_rad)
                dy = arrow_len * math.sin(angle_rad)

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
thread2 = threading.Thread(target=task2)
thread3 = threading.Thread(target=plot_rtk_map, daemon=True)

thread2.start()
thread3.start()

plt.show()
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import sys
import serial
import datetime
import threading
import math
from pyproj import Transformer

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.b2.sport.sport_client import SportClient

# ================= 串口 =================
serport = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
serport.flushInput()
time.sleep(0.5)

# ================= 全局变量 =================
last_utm_x = None
last_utm_y = None
origin_utm_x = None
origin_utm_y = None
filtered_heading = None
gga_quality = 0
latest_fields = None

# ================= 控制参数 =================
alpha = 0.2
kp_yaw = 0.02
kp_dist = 0.3
max_yaw_rate = 1.0
max_vx = 1.7
arrival_radius = 0.2


# ================= 工具函数 =================
def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def get_utm_epsg(lon, lat):
    zone = int((lon + 180) / 6) + 1
    if lat >= 0:
        epsg = 32600 + zone
    else:
        epsg = 32700 + zone
    return epsg


# ================= AGRICA 解析 =================
def parse_agrica(line):
    try:
        if "#AGRICA" not in line:
            return None

        data_part = line.split(";")[1]
        data_part = data_part.split("*")[0]
        fields = data_part.split(",")

        return fields
    except:
        return None


# ================= RTK线程 =================
def task2():
    global last_utm_x, last_utm_y
    global origin_utm_x, origin_utm_y
    global filtered_heading, gga_quality
    global latest_fields

    while True:
        line = serport.readline().decode(errors='ignore').strip()
        fields = parse_agrica(line)
        if not fields:
            continue

        latest_fields = fields

        try:
            rtk_status = int(fields[8])
            heading_status = int(fields[9])
            heading = float(fields[19])
            lat = float(fields[29])
            lon = float(fields[30])
        except:
            continue

        gga_quality = rtk_status

        if rtk_status == 4 and heading_status in [4, 5]:
            if filtered_heading is None:
                filtered_heading = heading
            else:
                diff = normalize_angle(heading - filtered_heading)
                filtered_heading = filtered_heading + alpha * diff
                filtered_heading %= 360


        epsg = get_utm_epsg(lon, lat)
        transformer = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
        utm_x, utm_y = transformer.transform(lon, lat)

        last_utm_x = utm_x
        last_utm_y = utm_y

        if origin_utm_x is None and rtk_status == 4:
            origin_utm_x = utm_x
            origin_utm_y = utm_y
            print("地图原点已设定")


# ================= 自动导航（改进版） =================
def navigate_to(target_x, target_y, sport_client):
    """
    高速精准锁头导航模式
    连续角速度函数 + 预测补偿 + 无分段跳变
    """

    print("\n========== START NAVIGATION ==========\n")

    last_vyaw = 0.0

    # 连续函数控制参数
    yaw_gain = 0.025           # 基础角速度增益
    brake_gain = 0.35          # 预测刹车强度
    smooth_gain = 1.2          # 衰减曲线形状
    print_timer = time.time()

    while True:

        if last_utm_x is None or filtered_heading is None:
            continue

        # 当前坐标
        cur_x = last_utm_x - origin_utm_x
        cur_y = last_utm_y - origin_utm_y

        dx = target_x - cur_x
        dy = target_y - cur_y
        dist = math.hypot(dx, dy)

        # 到达判断
        if dist < arrival_radius:
            sport_client.StopMove()
            print("\nArrived at target\n")
            break

        # 目标航向计算（保持原逻辑）
        angle = math.degrees(math.atan2(dy, dx))
        target_heading = 90 - angle
        if target_heading < 0:
            target_heading += 360

        target_heading = (target_heading + 180) % 360

        # 计算误差（自动最短路径）
        error = normalize_angle(filtered_heading - target_heading)

        # 基础比例
        vyaw_raw = yaw_gain * error

        #双曲正切压缩（避免突变）
        vyaw_smooth = max_yaw_rate * math.tanh(vyaw_raw / max_yaw_rate)

        #小角度指数衰减
        decay_factor = math.exp(-smooth_gain * abs(error) / 30.0)
        vyaw_smooth *= (1 - decay_factor)

        # 预测刹车补偿
        vyaw = vyaw_smooth - brake_gain * (vyaw_smooth - last_vyaw)

        last_vyaw = vyaw

        # 前进速度连续衰减（随角度变化）
        heading_factor = math.exp(-abs(error) / 40.0)
        vx = min(kp_dist * dist, max_vx) * heading_factor

        # 发送控制
        sport_client.Move(vx, 0, vyaw)

        if time.time() - print_timer > 0.1:
            print(
                f"\rDistance: {dist:6.2f} m | "
                f"Heading Error: {error:7.2f} deg | "
                f"Yaw Rate: {vyaw:7.3f} | "
                f"Forward Speed: {vx:7.3f}",
                end=""
            )
            print_timer = time.time()

        time.sleep(0.05)



# ================= 主程序 =================
if __name__ == "__main__":

    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} networkInterface")
        sys.exit(-1)

    ChannelFactoryInitialize(0, sys.argv[1])

    sport_client = SportClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    threading.Thread(target=task2, daemon=True).start()


    input("等待FIX，移动到目标点后按 Enter 采集航点...")

    target_x = last_utm_x - origin_utm_x
    target_y = last_utm_y - origin_utm_y

    input("按 Enter 开始导航...")
    navigate_to(target_x, target_y, sport_client)

    input("按 Enter 返航...")
    navigate_to(0, 0, sport_client)

    print("任务完成")
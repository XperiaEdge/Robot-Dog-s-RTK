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
max_yaw_rate = 0.4
max_vx = 0.5
arrival_radius = 0.5


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
    导航到指定目标点
    使用：航向PID + 小角度进入前进模式
    """

    print("开始导航...")

    # 创建航向 PID 控制器
    yaw_pid = PID(
        kp=0.02,
        ki=0.0005,
        kd=0.002,
        output_limit=max_yaw_rate
    )

    heading_threshold = 10.0  # 小于3度才允许前进

    while True:

        # ---- 等待 RTK 数据 ----
        if last_utm_x is None or filtered_heading is None:
            continue

        # ---- 当前坐标 ----
        cur_x = last_utm_x - origin_utm_x
        cur_y = last_utm_y - origin_utm_y

        # ---- 计算目标向量 ----
        dx = target_x - cur_x
        dy = target_y - cur_y
        dist = math.hypot(dx, dy)

        # ---- 判断是否到达 ----
        if dist < arrival_radius:
            sport_client.StopMove()
            print("到达目标")
            break

        # =====================================================
        # 关键：计算目标航向
        # 注意这里使用 atan2(dx, dy) 匹配 AGRICA 北为0°
        # =====================================================
        angle = math.degrees(math.atan2(dy, dx))
        target_heading = 90 - angle
        if target_heading < 0:
            target_heading += 360

        target_heading = (target_heading + 180) % 360
        # ---- 计算航向误差 ----
        error = normalize_angle(filtered_heading - target_heading)

        # 衰减角速度
        gain = 0.01
        vyaw = gain * error
        vyaw = max(min(vyaw, max_yaw_rate), -max_yaw_rate)

        # =====================================================
        # 阶段1：先对准
        # =====================================================
        if abs(error) > heading_threshold:

            vx = 0  # 不前进

            print(f"对准中 | 误差: {error:.2f}° | 距离: {dist:.2f}m")

        # =====================================================
        # 阶段2：进入前进模式
        # =====================================================
        else:

            # 前进速度随距离递减
            vx = min(kp_dist * dist, max_vx)

            print(f"前进中 | 误差: {error:.2f}° | 距离: {dist:.2f}m")

        # ---- 发送运动指令 ----
        sport_client.Move(vx, 0, vyaw)

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
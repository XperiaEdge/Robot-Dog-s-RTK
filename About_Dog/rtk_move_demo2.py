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
max_vx = 1
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

    print("\n========== 开始导航 ==========\n")

    # ===== 可调参数 =====
    yaw_gain = 0.055              # 航向比例增益（略提高）
    brake_strength = 3          # 提前刹车能力（增强）
    yaw_acc_limit = 0.04          # 角加速度限制（防首次过冲）
    direction_boost = 0.2        # 小误差指向增强
    heading_scale = 30.0          # 前进降速尺度（越大越不容易降速）

    last_vyaw = 0.0
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

        if dist < arrival_radius:
            sport_client.StopMove()
            print("\n已到达目标点\n")
            break

        # ===== 目标航向 =====
        angle = math.degrees(math.atan2(dy, dx))
        target_heading = 90 - angle
        if target_heading < 0:
            target_heading += 360
        target_heading = (target_heading + 180) % 360

        error = normalize_angle(filtered_heading - target_heading)

        # =====================================================
        # 角速度控制（增强指向 + 提前刹车 + 加速度限制）
        # =====================================================

        # 1. 基础比例输出
        vyaw_target = yaw_gain * error

        # 2. 小误差方向增强（让指向更干脆）
        vyaw_target += direction_boost * math.tanh(error / 20.0)

        # 3. 限幅
        vyaw_target = max_yaw_rate * math.tanh(vyaw_target / max_yaw_rate)

        # 4. 动态刹车距离（增强版）
        braking_angle = (last_vyaw ** 2) / (2 * brake_strength + 1e-6)

        brake_factor = math.exp(-abs(error) / (braking_angle + 0.5))
        vyaw_target *= (1 - brake_factor)

        # 5. 角加速度限制（解决第一次过冲核心）
        delta = vyaw_target - last_vyaw
        delta = max(min(delta, yaw_acc_limit), -yaw_acc_limit)

        vyaw = last_vyaw + delta

        last_vyaw = vyaw

        # =====================================================
        # 前进速度控制（平方衰减，更平滑）
        # =====================================================

        # 小角度几乎不降速，大角度明显降速
        heading_factor = 1.0 / (1.0 + (error / heading_scale) ** 2)

        vx_raw = min(kp_dist * dist, max_vx)
        vx = vx_raw * heading_factor

        # 满足宇树最低速度限制
        if dist > arrival_radius:
            vx = max(vx, 0.3)

        # =====================================================
        # 发送控制
        # =====================================================
        sport_client.Move(vx, 0, vyaw)

        # 输出
        if time.time() - print_timer > 0.1:
            print(
                f"\r距离: {dist:6.2f} 米 | "
                f"航向误差: {error:7.2f} 度 | "
                f"角速度: {vyaw:7.3f} | "
                f"前进速度: {vx:7.3f}",
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
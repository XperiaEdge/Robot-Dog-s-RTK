#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import sys
import serial
import datetime
import threading
import math
from pyproj import Transformer
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import ttk

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
map_x = []
map_y = []

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

        if origin_utm_x is not None:
            map_x.append(utm_x - origin_utm_x)
            map_y.append(utm_y - origin_utm_y)


# ================= 绘图线程 =================
def plot_map():
    plt.ion()
    fig, ax = plt.subplots()

    while True:
        if len(map_x) > 1:
            ax.clear()
            ax.set_title("RTK Navigation Map")
            ax.set_aspect('equal')
            ax.plot(map_x, map_y)

            cur_x = map_x[-1]
            cur_y = map_y[-1]
            ax.scatter(cur_x, cur_y)

            if filtered_heading:
                angle = math.radians(90 - filtered_heading)
                ax.arrow(cur_x, cur_y,
                         0.5 * math.cos(angle),
                         0.5 * math.sin(angle))

            ax.grid(True)
            plt.pause(0.3)

        time.sleep(0.3)


# ================= 美观数据窗口 =================
def serial_monitor_window():
    root = tk.Tk()
    root.title("RTK AGRICA 数据监视系统")
    root.geometry("1100x700")

    style = ttk.Style()
    style.configure("Treeview", rowheight=25, font=("Consolas", 10))
    style.configure("Treeview.Heading", font=("Arial", 11, "bold"))

    tree = ttk.Treeview(root, columns=("value", "unit"), show="headings")
    tree.heading("value", text="数值 Value")
    tree.heading("unit", text="单位 Unit")
    tree.column("value", width=200)
    tree.column("unit", width=150)
    tree.pack(fill=tk.BOTH, expand=True)

    labels = [
        ("RTK Status 定位状态", 8, ""),
        ("Heading Status 航向状态", 9, ""),
        ("Heading 航向角", 19, "deg"),
        ("Pitch 俯仰角", 20, "deg"),
        ("Roll 横滚角", 21, "deg"),
        ("Speed 速度", 22, "m/s"),
        ("Latitude 纬度", 29, "deg"),
        ("Longitude 经度", 30, "deg"),
        ("Altitude 高程", 31, "m"),
        ("Baseline_N 北向基线", 13, "m"),
        ("Baseline_E 东向基线", 14, "m"),
        ("Baseline_U 天顶基线", 15, "m"),
        ("GPS Num GPS卫星", 10, ""),
        ("BDS Num 北斗卫星", 11, ""),
        ("GAL Num Galileo卫星", 60, ""),
        ("DiffAge 差分龄期", 56, "s"),
    ]

    items = {}
    for name, idx, unit in labels:
        iid = tree.insert("", tk.END, values=(name, "", unit))
        items[iid] = (idx, unit)

    def update():
        if latest_fields:
            for iid, (idx, unit) in items.items():
                try:
                    value = latest_fields[idx]
                except:
                    value = ""

                tree.item(iid, values=(tree.item(iid)["values"][0], value, unit))

        root.after(300, update)

    update()
    root.mainloop()


# ================= PID 控制器类 =================
class PID:
    """
    简单 PID 控制器
    用于航向角控制
    """
    def __init__(self, kp, ki, kd, output_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit

        self.integral = 0
        self.last_error = 0
        self.last_time = time.time()

    def compute(self, error):
        """
        计算 PID 输出
        """
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        if dt == 0:
            return 0

        # ---- 积分项 ----
        self.integral += error * dt

        # 防止积分爆炸
        self.integral = max(min(self.integral, 50), -50)

        # ---- 微分项 ----
        derivative = (error - self.last_error) / dt
        self.last_error = error

        # ---- PID 输出 ----
        output = (self.kp * error +
                  self.ki * self.integral +
                  self.kd * derivative)

        # 限幅
        output = max(min(output, self.output_limit), -self.output_limit)

        return output


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
    threading.Thread(target=plot_map, daemon=True).start()
    #threading.Thread(target=serial_monitor_window, daemon=True).start()

    input("等待FIX，移动到目标点后按 Enter 采集航点...")

    target_x = last_utm_x - origin_utm_x
    target_y = last_utm_y - origin_utm_y

    input("按 Enter 开始导航...")
    navigate_to(target_x, target_y, sport_client)

    input("按 Enter 返航...")
    navigate_to(0, 0, sport_client)

    print("任务完成")
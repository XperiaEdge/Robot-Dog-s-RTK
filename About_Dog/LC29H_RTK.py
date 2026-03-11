import socket
import base64
import time
import serial
import datetime
import threading
import math
from pyproj import Transformer

serport = serial.Serial("/dev/ttyUSB1", 115200)  # 端口根据你实际使用的进行更换
serport.flushInput()
time.sleep(0.5)  # 单位是s

last_utm_x = None
last_utm_y = None
heading_deg = None

# 计算EPSG
def get_utm_epsg(lon, lat):
    zone = int((lon + 180) / 6) + 1
    if lat >= 0:
        epsg = 32600 + zone  # 北半球
    else:
        epsg = 32700 + zone  # 南半球
    return epsg, zone


# NTRIP Caster information
NTRIP_IP = "120.253.239.161"  # 差分平台的IP地址端口一般都是8002固定，用户名和密码进行修改
NTRIP_PORT = 8002
NTRIP_USERNAME = "csar11061"
NTRIP_PASSWORD = "a8888628"
NTRIP_MOUNT_POINT = "RTCM33_GRCEpro"  # 这个根据不同平台的接入点不一样进行改动，移动的一般就是用这个
GGA_STRING = "$GNGGA,115713.000,3149.301528,N,11706.920684,E,1,17,0.88,98.7,M,-3.6,M,,*58\r\n"
# Create a TCP socket and connect to the NTRIP Caster15.
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((NTRIP_IP, NTRIP_PORT))
    print("Connected.")
except socket.error as err:
    print("Connect fail,exit.")
    exit(1)
# Send login request

Authorization = (NTRIP_USERNAME + ":" + NTRIP_PASSWORD)
Authorization = base64.b64encode(Authorization.encode()).decode()
requestHead = f"""GET /{NTRIP_MOUNT_POINT} HTTP/1.0\r\nUser-Agent: MOZIHAO-GNSS\r\nAccept: */*\r\nConnection: close\r\nAuthorization: Basic {Authorization}\r\n\r\n"""
sock.send(requestHead.encode())
print("send auth msg to NTRIP caster:%s" % requestHead)
time.sleep(0.2)
# Receive and parse the NTRIP login response
response = sock.recv(1024)
print("receive from server:%s" % response.decode())
if "ICY 200 OK" not in response.decode():
    print("login to NTRIP caster fail,exit.")
    exit(1)
print('login to NTRIP caster successful.')


# Send GGA data to NtripCaster
def task1():
    while True:
        global GGA_STRING
        sock.send(GGA_STRING.encode())
        #print("\r\n\r\nSend GGA to NTRIP caster:%s" % GGA_STRING)
        response = sock.recv(4096)
        #print("Receive RTCM data from server:%d bytes.(should send the RTCM data to GNSS RTK module)" % len(response))
        hex_string = ''.join(format(b, '02x') for b in response).upper()
        #print(hex_string)
        serport.write(response)
        time.sleep(1)


def task2():
    while True:
        global GGA_STRING,last_utm_x, last_utm_y, heading_deg
        rxData = serport.readline()  # 读取里面的内容
        if rxData.find(b'$GNGGA') != -1:  #
            GGA_STRING = rxData.decode('utf-8')
            line = str(rxData.decode('utf-8')).split(',')  # 将line以“，”为分隔符
        if len(line) > 6 and line[2] and line[4]:  # jingwei is ok
            weidu = float(line[2][:2]) + float(line[2][2:]) / 60
            # 读取第5个字符串信息，从0-3为经度，即经度为117，再加上后面的一串除60将分转化为度
            jingdu = float(line[4][:3]) + float(line[4][3:]) / 60
            # 纬度同理

            # 纬度如果是南纬，将纬度取负，经度如果是西经，将经度取负
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

            utm_x, utm_y = transformer.transform(jingdu, weidu)  # 转换为UMT坐标系的x,y
            
            if last_utm_x is not None and last_utm_y is not None:
               dx = utm_x - last_utm_x
               dy = utm_y - last_utm_y

               distance = math.hypot(dx, dy)  # 欧式距离

               # 防止静止抖动（RTK FIX + 位移 > 10cm）
               if line[6] == '4' or line[6] == '3' and distance > 0.10:
                   heading_rad = math.atan2(dx, dy)
                   heading_deg = math.degrees(heading_rad)

                   if heading_deg < 0:
                      heading_deg += 360


            last_utm_x = utm_x
            last_utm_y = utm_y

            print(f"UTM Zone: {zone}")
            print(f"Easting: {utm_x:.3f}")
            print(f"Northing: {utm_y:.3f}")
            print("longitude:",'{:.6f}'.format(jingdu))
            print("latitude:",'{:.6f}'.format(weidu))
            current_time = datetime.datetime.now()
            print("当前时间：", current_time)
            if heading_deg is not None:
               print(f"航向角 Heading: {heading_deg:.2f}°")
        if line[6] == '1':
            print('GPS DATA VAULE')
        elif line[6] == '2':
            print('DIFF MODE')
        elif line[6] == '4':
            print('RTK FIXD')
        elif line[6] == '5':
            print('RTK FLOAT')
        # str(GGA_STRING,'utf-8')
        # print(GGA_STRING)


thread1 = threading.Thread(target=task1)  # 开启线程用来连接到Ntrip服务器
thread2 = threading.Thread(target=task2)  # 开启线程用来获取经纬度数据，可以输出RTK FIXD数据
thread1.start()
thread2.start()

# time.sleep(1)

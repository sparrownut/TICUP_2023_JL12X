import math
import os
import time
import traceback

import cv2
import numpy as np
import serial
from screeninfo import get_monitors

from tools.getOS import detect_os

# from matplotlib import pyplot as plt


if detect_os() == 'Windows':
    from raspberry_getinfo import getinfo

followMode = False
pixelShapeX = 210
pixelShapeY = 210


def editConfig(x_start_edit=0, y_start_edit=0, width_edit=0, height_edit=0, rotate_edit=0):
    print(f"{x_start_edit} {y_start_edit} {width_edit} {height_edit}")
    with open('config.conf', 'r', encoding='utf8') as confRead:
        x_start, y_start, width, height = 51, 58, pixelShapeX, pixelShapeY
        for it in confRead.readlines():
            if ',' in it:
                x_start = int(it.split(',')[0])
                y_start = int(it.split(',')[1])
                width = int(it.split(',')[2])
                height = int(it.split(',')[3])
                rotate = int(it.split(',')[4])
                print(f"{x_start} {y_start} {width} {height} {rotate}")

    # y_start += 1
    # print(f'y_start:{y_start} y_start_edit:{y_start_edit} res:{y_start + y_start_edit}')
    with open('config.conf', 'w', encoding='utf8') as confWrite:
        confWrite.write(
            f'{x_start + x_start_edit},{y_start + y_start_edit},{width + width_edit},{height + height_edit},{rotate + rotate_edit}')
    return
    # print()


def distance(p1, p2):
    """计算两点之间的距离"""
    return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


def interpolate_points(points, target_distance=1):
    """插值以使每对连续点之间的距离为指定值"""

    new_points = []
    for i in range(len(points) - 1):
        p1 = points[i]
        p2 = points[i + 1]

        # 添加第一个点
        new_points.append(p1)

        d = distance(p1, p2)

        # 计算需要插入多少个点
        num_points = int(d / target_distance)

        for j in range(1, num_points):
            ratio = j / num_points
            x = (1 - ratio) * p1[0] + ratio * p2[0]
            y = (1 - ratio) * p1[1] + ratio * p2[1]
            new_points.append((x, y))

    new_points.append(points[-1])  # 添加最后一个点
    return new_points


def getBorderEvent(contours, isSlowMode=False):
    global followMode
    for i in range(2):
        if len(contours) > 20:
            print(f"+ 获取到循迹边框 {len(contours)}个采样点")
            points = []
            for it in contours:
                points.append((it[0][0], it[0][1]))
            points = interpolate_points(points)  # 插值优化
            for it in points:
                # print(it[0][0],it[0][1])
                x, y = pixel2mm(it[0], it[1])  # 转mm
                x = int(x)
                y = int(y)
                send_cmd_GO(x, y)
                # print(f'{x} {y}')
                if isSlowMode:
                    time.sleep(8 / len(contours))
                time.sleep(1 / len(contours))
    followMode = False
    for i in range(0, 10):
        send_cmd_GO(999, 999)
        time.sleep(0.1)


# 串口
def send_cmd(port='/dev/ttyAMA0', baudrate=115200, cmd="", wait_time=0.1):
    # Initialize serial port
    with serial.Serial(port, baudrate, timeout=1) as ser:
        ser.write(cmd.encode())  # Send the message


def send_cmd_GO(x, y):
    print(f'{x} {y}')
    try:
        send_cmd(cmd=f"GO:{x},{y};")
    except Exception as e:
        print(f'- 发送串口失败')
        traceback.print_exception(e)


def pixel2mm(x, y):
    return 500 * (x / pixelShapeX), 500 * (y / pixelShapeY)


def find_midpoints(outer_contour, inner_contour):
    midpoints = []

    for point in outer_contour:
        point = point[0]
        distances = np.sqrt(np.sum((inner_contour - point) ** 2, axis=2))
        closest_point_index = np.argmin(distances)
        closest_point = inner_contour[closest_point_index][0]

        midpoint = ((point[0] + closest_point[0]) // 2, (point[1] + closest_point[1]) // 2)
        midpoints.append(midpoint)

    return np.array(midpoints).reshape(-1, 1, 2)


def detect_tape_edge(frame, slowMode):
    # 转换为HSV空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 定义黑色的范围
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 100])

    # 创建黑色的掩码
    mask = cv2.inRange(hsv, lower_black, upper_black)

    # 对掩码进行一些形态学操作以消除噪声
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # 寻找掩码中的轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # for it in contours:
    if len(contours) >= 2:  # 找到了边框
        print('检测到边框')
        midpoints = find_midpoints(contours[0], contours[1])
        cv2.drawContours(frame, [midpoints], 0, (255, 255, 255), 2)
        getBorderEvent(midpoints, slowMode)
    else:
        # print(f"* Debug : len(contours)=={len(contours)}")
        pass

    # 绘制轮廓
    cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
    # print(contours) # [[x,y],[x2,y2]...]
    # with open('output.csv','a',errors=None) as write:
    #     for it in contours:
    #         write.write(str(it))

    return frame


cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("无法打开摄像头")
    exit()

# 关闭自动白平衡（注意，不是所有摄像头/驱动都支持此设置）
if cap.set(cv2.CAP_PROP_AUTO_WB, 0):
    print(f"+ 成功关闭自动白平衡!")
else:
    print(f"- 无法关闭自动白平衡.")

# 设置摄像头的分辨率
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 300)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 300)

# 关闭自动曝光（注意，不是所有摄像头/驱动都支持此设置）
# 'cv2.CAP_PROP_AUTO_EXPOSURE' 的设置方法因摄像头而异，一般来说：
# 0 = 手动模式 (关闭自动曝光)
# 1 = 自动模式
# 3 = 应用相机的默认设置
closeautoExposure = os.system("v4l2-ctl -d /dev/video0 -c exposure_auto=1")
openDisplay = os.system("export DISPLAY=:0")
print(f'* 打开窗口反馈:{openDisplay}')
print(f'* 关闭自动曝光结果 {closeautoExposure}')
if cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0):  # 0.25表示关闭自动曝光
    print(f"+ 成功关闭自动曝光!")
else:
    print(f"- 无法关闭自动曝光.")

imagesN = 0


def fixCancvs(frame):
    with open('config.conf', 'r', encoding='utf8') as confRead:
        x_start, y_start, width, height = 51, 58, pixelShapeX, pixelShapeY
        for it in confRead.readlines():
            if ',' in it:
                x_start = int(it.split(',')[0])
                y_start = int(it.split(',')[1])
                width = int(it.split(',')[2])
                height = int(it.split(',')[3])
                rotate = int(it.split(',')[4])
        center = (frame.shape[1] // 2, frame.shape[0] // 2)
        angle = rotate  # 旋转角度
        scale = 1.0  # 缩放因子
        M = cv2.getRotationMatrix2D(center, angle, scale)
        # 旋转感兴趣区
        frame = cv2.warpAffine(frame, M, (frame.shape[1], frame.shape[0]))
        # 剪裁感兴趣区
        frame = frame[y_start:y_start + height, x_start:x_start + width]
        return frame


time.sleep(10)  # 防止开机启动时启动太快导致窗口大小加载不对
slowMode = False

while True:
    imagesN += 1
    ret, frame = cap.read()
    frame = fixCancvs(frame)  # 剪切

    if not ret:
        print("无法接收帧")
        break

    key = cv2.waitKey(1)

    if key == ord('q'):
        break
    elif key == ord('w'):
        editConfig(y_start_edit=-1)
    elif key == ord('s'):
        editConfig(y_start_edit=1)
    elif key == ord('a'):
        editConfig(x_start_edit=-1)
    elif key == ord('d'):
        editConfig(x_start_edit=1)
    elif key == ord('t'):
        editConfig(height_edit=-1)
    elif key == ord('g'):
        editConfig(height_edit=1)
    elif key == ord('f'):
        editConfig(width_edit=-1)
    elif key == ord('h'):
        editConfig(width_edit=1)
    elif key == ord('e'):
        editConfig(rotate_edit=-1)
    elif key == ord('r'):
        editConfig(rotate_edit=1)
    elif key == ord('n'):
        followMode = not followMode
    elif key == ord('c'):
        slowMode = not slowMode
    if followMode:
        frame = detect_tape_edge(frame, slowMode)

    # 获取屏幕分辨率
    try:
        monitor = get_monitors()[0]
        screen_res = monitor.width, monitor.height
        scale_width = screen_res[0] / frame.shape[1]
        scale_height = screen_res[1] / frame.shape[0]
        scale = min(scale_width, scale_height)

        window_width = int(frame.shape[1] * scale)
        window_height = int(frame.shape[0] * scale)

        cv2.namedWindow('BLACK TAPE', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('BLACK TAPE', window_width, window_height)

        cv2.imshow('BLACK TAPE', frame)  # 首先显示图像
        cv2.setWindowProperty('BLACK TAPE', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)  # 然后设置窗口属性
    except Exception as e:
        traceback.print_exception(e)
cap.release()
cv2.destroyAllWindows()

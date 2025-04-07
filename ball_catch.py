#------------------README--------------------------#
'''
抓取小球demo
识别圆的过程中，如果识别不准确，需要调整circles = cv2.HoughCircles(gray_img, cv2.HOUGH_GRADIENT, 1, 30, param1=56, param2=28, minRadius=10, maxRadius=50)中的参数
当小球距离过近时，摄像头拍到的只有半个小球，无法识别，如果追求准确度，建议可以减小xgo每一步中前进调整幅度，避免走多。同样如果想加快调整，也需要更改。主要通过修改xgo的运动速度和运动时间。
注释掉的代码主要是将识别结果显示在xgo上，由于屏幕初始化以及显示过慢，故没有可视化。
颜色识别部分，不同颜色的HSV范围受光等条件影响
由于每台xgo未经过标定，所以满足抓取时的条件也不太一致，需要自行调整，调整部分在代码中已经标注
右上按键切换夹取颜色，可以抓取红，绿，蓝三种颜色小球
可视化部分已经注释掉
'''
#---------------------------------------------------#
# import math
# import os
# import socket
import sys
import time

import numpy as np
import spidev as SPI
# import xgoscreen.LCD_2inch as LCD_2inch
# from PIL import Image, ImageDraw, ImageFont
from key import Button
from xgolib import XGO

#------------------------------------------------------
#机械臂抓取
def catch_arm(dog):
    dog.translation('z', 10)
    dog.attitude('p', 15)
    dog.claw(5)
    time.sleep(1)
    dog.arm_polar(200,130)
    time.sleep(2)
    dog.claw(245)
    time.sleep(1)
    dog.arm_polar(90,100)
# ---------------------初始化------------------------ #
#屏幕初始化
# display = LCD_2inch.LCD_2inch()
# display.clear()
# splash = Image.new("RGB", (display.height, display.width), "black")
# display.ShowImage(splash)
button = Button()

import cv2
from picamera2 import Picamera2
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (320, 240)}))
picam2.start()
print("摄像头初始化完毕")

dog = XGO(port='/dev/ttyAMA0', version="xgomini")

fm = dog.read_firmware()
if fm[0] == 'M':
    print('XGO-MINI')
    dog = XGO(port='/dev/ttyAMA0', version="xgomini")
    dog_type = 'M'
else:
    print('XGO-LITE')
    dog = XGO(port='/dev/ttyAMA0', version="xgolite")
    dog_type = 'L'
CircleCount = 0
UnCircleCount = 0
mx = 0
my = 0
mr = 0
distance = 0
yaw_err = 0
if dog_type == 'L':
    mintime_yaw = 0.8
    mintime_x = 0.1
    x_speed_far = 16
    x_speed_slow = 8
    turn_speed = 8
else:
    mintime_yaw = 0.7
    mintime_x = 0.3
    x_speed_far = 16
    x_speed_slow = 8
    turn_speed = 8

# ---------------------MAIN------------------------ #
mode=0
def change_color():
    global mode
    if mode==4:
        mode=1
    else:
        mode+=1
    if mode == 1:  # red
        a = 'red'
        return a
    elif mode == 2:  # green
        a = 'green'
        return a
    elif mode == 3:  # blue
        a = 'blue'
        return a

#-------------------------------------------------------
def Image_Processing(dog,a,mintime_yaw,mintime_x,picam2):
    global CircleCount, mx, my, mr, distance, yaw_err, UnCircleCount
    turn_time = mintime_yaw
    run_time = mintime_x
    image = picam2.capture_array()
    if image is None:
        exit()
    x, y, r = 0, 0, 0
    image = cv2.GaussianBlur(image, (3, 3), 0)

#-------------------------------MASK------------------------
    # 将图像转换到 HSV 颜色空间
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    def color_recognize(a: str):
    # 识别的颜色范围
        if a == "blue":
            lower_blue = np.array([100, 100, 100])
            upper_blue = np.array([140, 255, 255])
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            return mask
        elif a == "green":
            lower_green = np.array([35, 100, 100])
            upper_green = np.array([85, 255, 255])
            mask = cv2.inRange(hsv, lower_green, upper_green)
            return mask
        elif a == "red":
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)
            return mask
    mask = color_recognize(a)
    masked_image = cv2.bitwise_and(image, image, mask=mask)
    image = masked_image
    
#---------------------------------------------------------------
    gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(gray_img, cv2.HOUGH_GRADIENT, 1, 30, param1=36, param2=18, minRadius=8, maxRadius=45)
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        # 找出半径最大的圆
        max_radius_index = np.argmax(circles[:, 2])
        max_radius_circle = circles[max_radius_index]
        x, y, r = max_radius_circle
        CircleCount += 1
        mx = (CircleCount - 1) * mx / CircleCount + x / CircleCount
        my = (CircleCount - 1) * my / CircleCount + y / CircleCount
        mr = (CircleCount - 1) * mr / CircleCount + r / CircleCount

        #绘制半径最大的圆的轮廓
        cv2.circle(image, (x, y), r, (0, 255, 0), 2)
        # 绘制圆心
        cv2.circle(image, (x, y), 2, (0, 0, 255), 3)


        # 显示绘制了圆的图像
        # b, g, r1 = cv2.split(image)
        # image = cv2.merge((r1, g, b))
        # imgok = Image.fromarray(image)
        # display.ShowImage(imgok)


        if len(circles[0]) == 1:
            ball = circles[0][0]
            x, y, r = int(ball[0]), int(ball[1]), int(ball[2])
            cv2.circle(image, (x, y), r, (0, 0, 255), 2)
            cv2.circle(image, (x, y), 2, (0, 0, 255), 2)
            print("x:{}, y:{}, r:{}".format(x, y, r))
            CircleCount += 1
            mx = (CircleCount - 1) * mx / CircleCount + x / CircleCount
            my = (CircleCount - 1) * my / CircleCount + y / CircleCount
            mr = (CircleCount - 1) * mr / CircleCount + r / CircleCount
    else:
        UnCircleCount += 1
    
    if UnCircleCount >=15:
        UnCircleCount = 0 
        print('没看见球，可能走过了，后退一点重新找球')
        dog.move_x(-10)
        time.sleep(0.8)
        dog.stop()
        time.sleep(0.2)
        return 1

    if CircleCount >= 15:
        CircleCount = 0
        distance = 54.82 - mr
        yaw_err = -(mx - 160) / distance
        if dog_type == 'L':
            if  abs(mx-160) > 25:
                turn_time = min(abs(0.4 * yaw_err)/8 + mintime_yaw, 2)
            if distance >=30:
                run_time = distance / 40 + mintime_x
                turn_time = abs(9 * yaw_err) / 8 + mintime_yaw
            elif 25 <= distance <30 :
                run_time = distance / 75 + mintime_x 
                turn_time = abs(5 * yaw_err) / 8 + mintime_yaw
            else:
                run_time = mintime_x + 0.3
            x_distance = 20  # 如果夹球时距离远，需要减小x_distance

        else:
            if  abs(mx-160) > 25:
                turn_time = min(abs(0.4 * yaw_err)/8 + mintime_yaw, 2)
            if distance >=30:
                run_time = distance / 50 + mintime_x
                turn_time = abs(9 * yaw_err) / 8 + mintime_yaw
            elif 25 <= distance <30 :
                run_time = distance / 80 + mintime_x 
                turn_time = abs(5 * yaw_err) / 8 + mintime_yaw
            else:
                run_time = mintime_x + 0.2
            x_distance = 22 # 如果夹球时距离远，需要减小x_distance

        if distance < x_distance and -20 / distance <= yaw_err <= 20 / distance :  #distance控制距离球的前后，yaw_err控制球的左右，需要根据每台xgo，具体微调一下抓球的位置
            print("满足条件一抓取")
            dog.attitude('y', 5 * yaw_err)
            dog.translation('x',(distance-20))
            time.sleep(0.5)
            catch_arm(dog)
            time.sleep(2)
            return 0

        if distance < x_distance and -25 <= mx-160 <= 25  : #distance控制距离球的前后，mx-160控制球的左右，需要根据每台xgo，具体微调一下抓球的位置
            print("满足条件二抓取")
            dog.attitude('y',0.25 * (160-mx))
            dog.translation('x',(distance-20))
            time.sleep(0.5)
            catch_arm(dog)
            time.sleep(2)
            return 0
        if distance >= 25:
            print("距离还远，快速前进")
            dog.gait_type('trot')
            dog.move_x(x_speed_far)
            time.sleep(run_time)
            dog.move_x(0)
            time.sleep(0.2)
        elif 19 <= distance <= 25:
            print("距离不远了，可以微调")
            dog.gait_type("slow_trot")
            dog.move_x(x_speed_slow)
            time.sleep(run_time)
            dog.move_x(0)
            time.sleep(0.1)
        if yaw_err > 20 / distance:
            print("左转")
            dog.gait_type("slow_trot")
            dog.turn(turn_speed)
            time.sleep(turn_time)
            dog.turn(0)
            time.sleep(0.1)
        elif yaw_err < -20/ distance:
            print("右转")
            dog.gait_type("slow_trot")
            dog.turn(-turn_speed)
            time.sleep(turn_time)
            dog.turn(0)
            time.sleep(0.1)
    return 1


dog.attitude('p', 15)
dog.translation('z', 75)
time.sleep(3)
color = "red"
while True:
    if button.press_b():
        #cv2.destroyAllWindows()
        sys.exit()
        break
    if button.press_d():
        color = change_color()
    # 可传入三个颜色:red,blue,green
    # print(f'抓球的颜色为{color}')
    stop = Image_Processing(dog, color, mintime_yaw, mintime_x, picam2)
    if stop == 0:
        dog.translation('x', 0)
        dog.translation('y', 0)
        dog.attitude('y',0)
        dog.attitude('p',0)
        break
    time.sleep(0.1)




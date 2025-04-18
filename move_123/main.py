'''
语音控制运动，需要在联网情况下运行
每次命令需要用“你好，lulu”唤醒，然后开始给命令，支持前进，后退，左转，右转，
以及趴下,站起,转圈,匍匐前进,原地踏步,蹲起,沿x转动,沿y转动,沿z转动,三轴转动,撒尿,坐下,招手,伸懒腰,波浪运动,摇摆运动,求食,找食物,握手,展示机械臂,俯卧撑，张望，跳舞，调皮
如需添加其他功能，可以在大模型调用的prompt给出，并同步修改后续运动
可视化相关的已经注释掉
'''
#--------------------大模型调用--------------------------------
import os
import ast
from volcenginesdkarkruntime import Ark
def model_output(content):

    api_key = '67e1719b-fc92-43b7-979d-3692135428a4'

    model = "doubao-1.5-pro-32k-250115"
    # 初始化Ark客户端
    client = Ark(api_key = api_key)
    prompt = '''
        我接下来会给你一段话，如果有退出，停止等意思，请返回字符'退出'，如果不符合下面要求也请返回'退出',请根据以下规则对其进行处理，并以列表形式返回结果。列表格式为：  
        `[['x', step], ['y', -step], ['turn', yaw], ...]`，列表中元素都是成对出现的，各元素的含义如下：  
        1. name:'x'：表示前后移动。  
           - `step` 为正时，表示向前移动的距离。  
           - `step` 为负时，表示向后移动的距离。  
        2. name:'y'：表示左右移动。  
           - `step` 为正时，表示向左平移的距离。  
           - `step` 为负时，表示向右平移的距离。  
        3. name:'turn'：表示转向。  
           - 'yaw'：为正时，表示向左转动的角度
           - 'yaw'：为负时，表示向右转动的角度
        4. name:action
           - id:取值范围为1-24,分别对应[趴下,站起,转圈,匍匐前进,原地踏步,蹲起,沿x转动,沿y转动,沿z转动,三轴转动,撒尿,坐下,招手,伸懒腰,波浪运动,摇摆运动,求食,找食物,握手,展示机械臂,俯卧撑，张望，跳舞，调皮]，其中机械臂的上抓，中抓，下抓对应的id分别为128，129，130。举例说明，即趴下的id为1,匍匐前进为4,求食为18
        5. name:pace
           - mode, 取值为slow, normal, high,分别对应慢速，正常，高速
        6. name:translation
           - [direction,data], direction取值为'x','y','z' ,data的单位是毫米，沿X轴正方向平移为正数，0表示回到初始位置，沿着X负方向平移为负数，取值范围是[-35,35]mm，y轴和z轴同理。
        7. name:attitude
           - [direction,data], direction取值为'r','p','y' ,data的单位是度，沿X轴正时针旋转为正数，0表示回到初始位置，沿着X逆时针旋转为负数，取值范围是[-20,20]mm，y轴和z轴旋转运动同理。
        8. name:arm
           - [arm_x, arm_z],arm_x取值范围是[-80,155]和arm_z的取值范围是[-95，155] 
        9. name:claw
           - pos,取值是0-255，其中0表示夹爪完全张开，255表示夹爪完全闭合。          
        10. name:imu
           - mode,取值为0或者1，0代表关闭自稳定模式，1表示打开自稳定模式。        
        11. name:perform
           - mode, 取值为0或者1，0代表关闭表演模式，1表示打开表演模式。         
        12. name:reset
           - mode,取值为0,表示让机器狗复位      
        13. name:lcd_picture
           - filename,此函数用于机器狗显示表情，有攻击，愤怒，厌恶，喜欢，调皮，祈祷，伤心，敏感，困，道歉，惊讶, filename对应的参数是attack, anger, disgust, like, naughty, pray, sad, sensitive, sleepy, apologize, surprise.
        14. name:Speaker     
           - filename,#此函数用于机器狗发出声音，有攻击，愤怒，厌恶，喜欢，调皮，祈祷，伤心，敏感，困，道歉，惊讶, filename对应的参数是attack, anger, disgust, like, naughty, pray, sad, sensitive, sleepy, apologize, surprise.
        15. name:motor
           - [motor_id, data],motor_id的取值范围是 [11,12,13,21,22,23,31,32,33,41,42,43,51,52,53]，data表示舵机角度。
        16. name:battery
           - mode,取值为0,读取当前电池电量
       **默认值规则**：  
        - 如果未指定移动距离，默认移动距离为 `30`。  
        - 如果未指定转动角度，默认转动角度为 `90`。 
       name是指方法名字，后面是需要传入的参数以及参数规则,多个参数时需要以列表形式给出
       返回结果以'['作为开始，以']'作为结束
        请严格按照上述规则处理输入内容，并返回结果列表。
    '''
    prompt = prompt + ':' + content
    # 创建一个对话请求
    completion = client.chat.completions.create(
        model = model,
        messages = [
            {"role": "user", "content": prompt},
        ],
    )
    result = completion.choices[0].message.content
    result = ast.literal_eval(result)
    return result
#-----------------------唤醒词检测----------------
from libnyumaya import AudioRecognition, FeatureExtractor
from auto_platform import AudiostreamSource, play_command
import time
import os
from datetime import datetime

automark = 1
quitmark = 0


def wake_up():
    break_luyin = False
    audio_stream = AudiostreamSource()
    libpath = "./libnyumaya_premium.so.3.1.0"
    extractor = FeatureExtractor(libpath)
    detector = AudioRecognition(libpath)
    extactor_gain = 1.0
    # Add one or more keyword models
    keywordIdlulu = detector.addModel("./lulu_v3.1.907.premium", 0.6)
    bufsize = detector.getInputDataSize()
    audio_stream.start()
    while True:
        frame = audio_stream.read(bufsize * 2, bufsize * 2)
        if not frame:
            time.sleep(0.01)
            continue
        features = extractor.signalToMel(frame, extactor_gain)
        prediction = detector.runDetection(features)
        if prediction != 0:
            now = datetime.now().strftime("%d.%b %Y %H:%M:%S")
            if prediction == keywordIdlulu:
                print("唤醒成功" + "lulu detected:" + now)
                # os.system(play_command + " /home/pi/RaspberryPi-CM4/move_auto/ding.wav")
                audio_stream.stop()
                return 1

#-------------------------------------------------
import os, re
from xgolib import XGO
import cv2
import os, socket, sys, time
import spidev as SPI
from PIL import Image, ImageDraw, ImageFont
from key import Button
import threading
import json, base64
import subprocess
import pyaudio
import wave
import numpy as np
from scipy import fftpack
from datetime import datetime
#from xgoedu import XGOEDU
#---------------------------豆包------------------------------
from audio import start_recording  #录音
from language_recognize import test_one #调用大模型将语音转化为文字识别
#--------------------------INIT-------------------------------
#XGO_edu = XGOEDU()
dog = XGO(port='/dev/ttyAMA0', version="xgomini")
fm = dog.read_firmware()
if fm[0] == 'M':
    print('XGO-MINI')
    dog = XGO(port='/dev/ttyAMA0', version="xgomini")
    dog_type = 'M'
else:
    dog = XGO(port='/dev/ttyAMA0', version="xgolite")
    print('XGO-LITE')
    dog_type = 'L'
if dog_type == 'L':
    mintime_yaw = 0.8
    mintime_x = 0.1
else:
    mintime_yaw = 0.7
    mintime_x = 0.3
button = Button()
#------------------屏幕初始化-------------------
# import xgoscreen.LCD_2inch as LCD_2inch
# display = LCD_2inch.LCD_2inch()
# display.clear()
# splash_theme_color = (15, 21, 46)
# splash = Image.new("RGB", (display.height, display.width), splash_theme_color)
# font2 = ImageFont.truetype("/home/pi/model/msyh.ttc", 16)
# draw = ImageDraw.Draw(splash)
# display.ShowImage(splash)
# def lcd_draw_string(
#     splash,
#     x,
#     y,
#     text,
#     color=(255, 255, 255),
#     font_size=1,
#     scale=1,
#     mono_space=False,
#     auto_wrap=True,
#     background_color=(0, 0, 0),
# ):
#     splash.text((x, y), text, fill=color, font=scale)

#----------------------速度调整---------------------------------
def adaptive_move(distance):
    if distance < 15:
        speed = 10
    elif distance < 30:
        speed = 15
    else:
        speed = 20
    return speed
def adaptive_turn(yaw):
    if yaw < 20:
        turn_speed = 10
    elif yaw < 50 :
        turn_speed = 20
    else:
        turn_speed = 30
    return turn_speed
#-------------------------------------
p = pyaudio.PyAudio()
# 录音参数
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK)
while True:
    print("等待唤醒词")
    wake = wake_up()
    if wake:
        print("唤醒成功")
        stream.stop_stream()
        time.sleep(0.1)
        stream.start_stream()
        start_recording(p, stream)
    content = test_one()
    if content == 0:
        print("录音出错")
        exit()
    else:
        print(content)
        #可视化显示
        # lcd_draw_string(
        #     draw,
        #     10,
        #     111,
        #     content,
        #     color=(255, 255, 255),
        #     scale=font2,
        #     mono_space=False,
        # )
        result = model_output(content=content)
        print(result)
        if result == '退出':
            sys.exit()
            stream.stop_stream()
            stream.close()
            p.terminate()
            dog.reset()
            break
        for i in result:
            if i[0] == 'x' or i[0] == 'y':
                speed = adaptive_move(int(i[1]))
                dog.move(i[0], speed * int(i[1]) / abs(int(i[1])))
                time.sleep(abs(int(i[1] / speed)))
                dog.stop()
                time.sleep(0.5)
            if i[0] == 'turn':
                turn_speed = adaptive_turn(int(i[1]))
                dog.turn(turn_speed * int(i[1]) / abs(int(i[1])))
                time.sleep(abs(int(i[1] / turn_speed)))
                dog.stop()
                time.sleep(0.5)
            if i[0] == 'action':
                dog.action(int(i[1]))
                time.sleep(8)
                dog.stop()
                time.sleep(0.5)
            if i[0] == 'pace':
                dog.pace(i[1])
                time.sleep(0.2)
            if i[0] == 'translation':
                dog.translation(i[1][0], i[1][1])
                time.sleep(1)
            if i[0] == 'attitude':
                dog.attitude(i[1][0], i[1][1])
                time.sleep(1)
            if i[0] == 'arm':
                dog.arm(i[1][0], i[1][1])
                time.sleep(1)
            if i[0] == 'claw':
                dog.claw(i[1])
                time.sleep(1)
            if i[0] == 'imu':
                dog.imu(i[1])
                time.sleep(1)
            if i[0] == 'perform':
                dog.perform(i[1])
                time.sleep(10)
            if i[0] == 'reset':
                dog.reset()
                time.sleep(1)
            if i[0] == 'lcd_picture':
                continue
                #XGO_edu.lcd_picture(i[1])
            if i[0] == 'Speaker':
                continue
                #XGO_edu.xgoSpeaker(i[1])
            if i[0] == 'motor':
                dog.motor(i[1][0], i[1][1])
                time.sleep(1)
            if i[0] == 'battery':
                print(dog.read_battery())
                time.sleep(1)
            else:
                continue
    if button.press_b():
        sys.exit()
        stream.stop_stream()
        stream.close()
        p.terminate()
        dog.reset()
        break

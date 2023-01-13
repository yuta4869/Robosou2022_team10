# -*- coding: utf-8 -*-
import picamera
import picamera.array
import cv2 as cv
import numpy as np
import time

from smbus import SMBus

from adafruit_rgb_display.rgb import color565
from adafruit_rgb_display.ili9341 import ILI9341

from busio import SPI
from digitalio import DigitalInOut
import board

from PIL import Image, ImageDraw

# Pin Configuration
cs_pin = DigitalInOut(board.D8)
dc_pin = DigitalInOut(board.D25)
rst_pin = DigitalInOut(board.D24)

# Set up SPI bus
spi = SPI(clock=board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# Create the ILI9341 display:
disp = ILI9341(
    spi,
    cs=cs_pin, dc=dc_pin, rst=rst_pin,
    width=240, height=320,
    rotation=90,
    baudrate=24000000
)

stm_main = 0x29
i2cbus = SMBus(1)
stm_sleep_time = 0.001
#arm_status = 0x00
#robot_status = 0x00
Tar = (1088/2)/2
pre_err = 0
Kp = 0.0035
Kd = 0.002
Ki = 0.001
duty = 0


# Define image size (320x240, rotated)
IMAGE_SIZE = (disp.height, disp.width)

# Open image
image = Image.open("名称未設定のアートワーク.jpg")

# Resize to screen size
image = image.resize(IMAGE_SIZE, resample=Image.LANCZOS)

# Display image
disp.image(image)

# カメラ初期化
with  picamera.PiCamera() as camera:
    # カメラの画像をリアルタイムで取得するための処理
    with picamera.array.PiRGBArray(camera) as stream:
        # 解像度の設定
        camera.resolution = (1088, 400)

        while True:
            # カメラから映像を取得する
            camera.capture(stream, 'bgr', use_video_port=True)
            # 顔検出の処理効率化のために、写真の情報量を落とす
            grayimg = cv.cvtColor(stream.array, cv.COLOR_BGR2GRAY)

            # 顔検出のための学習元データを読み込む
            face_cascade = cv.CascadeClassifier('haarcascades/haarcascade_frontalface_default.xml')
            # 顔検出を行う
            facerect = face_cascade.detectMultiScale(grayimg, scaleFactor=1.2, minNeighbors=2, minSize=(100, 100))
            
            # 顔が検出された場合
            if len(facerect) > 0:
                # 検出した場所すべてに赤色で枠を描画
                for rect in facerect:
                    cv.rectangle(stream.array, tuple(rect[0:2]), tuple(rect[0:2]+rect[2:4]), (0, 0, 255), thickness=3)        
                    #print(rect)# 中身をプリント
                    [x, y, w, t] = rect
                    #x_g = (x + w/2)/10

                    #中心の取得
                    x_g = (x + w/2)/2

                    #整数化
                    x_new = int(x_g)
                    
                    #偏差の取得
                    err = x_new - int(Tar)

                    #微分項
                    differ  = err - pre_err
                    pre_err = err
                    
                    #積分項
                    integ += err

                    #コントローラ
                    duty = Kp * err + Kd * differ + Ki * integ
                    
                    if duty > 0.4:
                       duty = 0.4
                       
                    if duty < -0.4:
                       duty = -0.4
                        
                    #I2C用の変換
                    #mmm = int(duty * 100)
                    new_duty = int(127*duty) + int(127)
                    #check_duty = (new_duty - 127)/127

                    print(new_duty)
                    #print(y)
                    #print(x)
                    #pre_y = 0
                    #lpf_y = pre_y*0.7 + y*0.3
                    #pre_y = y
                    #print(lpf_y)
                    
                    if -0.01 < duty < 0.01:
                        image = Image.open("ok-03.png")
                        image = image.resize(IMAGE_SIZE, resample=Image.LANCZOS)
                        disp.image(image)
                        
                    if duty == 0.0:
                        print("finish")
                    
                i2cbus.write_byte(stm_main, (new_duty))
                time.sleep(stm_sleep_time)
                #i2cbus.write_byte(stm_main,arm_status)

#                    except KeyboardInterrupt:
#                            cv2.destroyAllWindows()

#int __name__ == '__main__':
#    try:
#        print('This file should be imported')

#    except KeyboardInterrupt:
#        cv2.destroyAllWindows()
           
            # 結果の画像を表示する
            #cv.imshow('camera', stream.array)

            # カメラから読み込んだ映像を破棄する
            stream.seek(0)
            stream.truncate()
            
             #何かキーが押されたかどうかを検出する（検出のため、1ミリ秒待つ）
            if cv.waitKey(1) > 0:
                break

        # 表示したウィンドウを閉じる
        cv.destroyAllWindows()
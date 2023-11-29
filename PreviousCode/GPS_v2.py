
#!/usr/bin/python
# -*- coding:utf-8 -*-
import RPi.GPIO as GPIO
import serial
import time

ser = serial.Serial('/dev/ttyS0',115200)
ser.flushInput()

power_key = 6
rec_buff = ''

# 开启SIM7600X模块--设定启动时间20s
def power_on(power_key):
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(power_key,GPIO.OUT)
    time.sleep(0.1)
    GPIO.output(power_key,GPIO.HIGH)
    time.sleep(2)
    GPIO.output(power_key,GPIO.LOW)
    time.sleep(5)
    ser.flushInput()
# 关闭SIM7600X模块--设定关闭时间21s
def power_down(power_key):
    GPIO.output(power_key,GPIO.HIGH)
    time.sleep(3)
    GPIO.output(power_key,GPIO.LOW)
    time.sleep(18)

# command:AT+CGPSINFO | back:OK | timeout:1
def send_at(command,back,timeout):
    rec_buff = ''
    ser.write((command+'\r\n').encode())
    time.sleep(timeout)
    if ser.inWaiting():
        time.sleep(0.01 )
        rec_buff = ser.read(ser.inWaiting())
    if rec_buff != '':
        if back not in rec_buff.decode():
            print(command + ' ERROR')
            print(command + ' back:\t' + rec_buff.decode())
            return 0
        else:
            str_gps=rec_buff.decode()
            print(rec_buff.decode())
            if len(str_gps)>50:
                fp = open("/home/pi/Desktop/Code/Data/GPSData.txt","a+")
                print (time.ctime(),file=fp,end='  ')
                print(float(str_gps[25:27])+float(str_gps[27:36])/60,end=' ',file=fp)
                print(str_gps[37:38],end='  ',file=fp)
                print(float(str_gps[39:42])+float(str_gps[42:51])/60,end=' ',file=fp)
                print(str_gps[52:53],file=fp)
                fp.close()
            return 1
    else:
        return 0

# 获取GPS位置信息
def get_gps_position():
    rec_null = True
    answer = 0
    rec_buff = ''
    send_at('AT+CGPS=1,1','OK',1)
    time.sleep(2)
    while rec_null:
        answer = send_at('AT+CGPSINFO','+CGPSINFO: ',1)
        if 1 == answer:
            answer = 0
            if ',,,,,,' in rec_buff:
                rec_null = False
                time.sleep(1)
        else:
            rec_buff = ''
            send_at('AT+CGPS=0','OK',1)
            return False
        time.sleep(1.5)

# 执行程序
try:
    # 开启SIM7600X
    # power_on(power_key)
    # 获取GPS位置数据
    get_gps_position()
    # 关闭SIM7600X
    # power_down(power_key)
except:
    if ser != None:
        ser.close()
    power_down(power_key)
    GPIO.cleanup()
if ser != None:
        ser.close()
        GPIO.cleanup()

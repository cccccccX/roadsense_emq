from picamera2 import Picamera2, Preview
from time import sleep
import time
import smbus  
import threading
import RPi.GPIO as GPIO
import serial

gap = 2

class AutoCam(threading.Thread):
    
    picam2 = Picamera2()
    #print("1...")
    def dostart(self):
        self.picam2.start_preview(False)
        while True:
            #print("1...")
            #self.picam2.start_and_capture_file("/home/pi/Desktop/Code/Data/photos/"+time.ctime()+".jpg")
            #camera_config = self.picam2.create_preview_configuration()
            #self.picam2.configure(camera_config)
            self.picam2.start_and_capture_file("/home/pi/Desktop/Code/Data/photos/"+time.ctime()+".jpg")
            #self.picam2.start()
            time.sleep(gap)
            #self.picam2.capture_file("/home/pi/Desktop/Code/Data/photos/"+time.ctime()+".jpg")
            #print("1...")
            
            
    def run(self):
        self.dostart()
        
        
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

Device_Address = 0x68   # MPU6050 device address
bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards

class AutoObtainAcc(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        #write to sample rate register
        bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
        
        #Write to power management register
        bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
        
        #Write to Configuration register
        bus.write_byte_data(Device_Address, CONFIG, 0)
        
        #Write to Gyro configuration register
        bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
        
        #Write to interrupt enable register
        bus.write_byte_data(Device_Address, INT_ENABLE, 1)

    def read_raw_data(self,addr):
        #Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value

    def dostart(self):
        while True:
            #Read Accelerometer raw value
            acc_x = self.read_raw_data(ACCEL_XOUT_H)
            acc_y = self.read_raw_data(ACCEL_YOUT_H)
            acc_z = self.read_raw_data(ACCEL_ZOUT_H)
            
            #Read Gyroscope raw value
            gyro_x = self.read_raw_data(GYRO_XOUT_H)
            gyro_y = self.read_raw_data(GYRO_YOUT_H)
            gyro_z = self.read_raw_data(GYRO_ZOUT_H)
            
            #Full scale range +/- 250 degree/C as per sensitivity scale factor
            Ax = acc_x/16384.0
            Ay = acc_y/16384.0
            Az = acc_z/16384.0
            
            Gx = gyro_x/131.0
            Gy = gyro_y/131.0
            Gz = gyro_z/131.0
            

            #print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)
            fp = open("/home/pi/Desktop/Code/Data/MpuData.txt","a+")
            print (time.ctime(),file=fp,end=' ')
            print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az,file=fp)
            fp.close()
            sleep(gap)

    def run(self):
        self.dostart()
        
ser = serial.Serial('/dev/ttyS0',115200)
ser.flushInput()

power_key = 6
rec_buff = ''

class AutoGps(threading.Thread):

    # 开启SIM7600X模块--设定启动时间20s
    def power_on(self,power_key):
        print(time.ctime()+' SIM7600X is starting:')
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(power_key,GPIO.OUT)
        time.sleep(0.1)
        GPIO.output(power_key,GPIO.HIGH)
        time.sleep(1)
        GPIO.output(power_key,GPIO.LOW)
        time.sleep(20)
        ser.flushInput()
        print(time.ctime()+' SIM7600X is ready')
    # 关闭SIM7600X模块--设定关闭时间21s
    def power_down(self,power_key):
        print('SIM7600X is loging off:')
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(power_key,GPIO.OUT)
        time.sleep(0.1)
        GPIO.output(power_key,GPIO.HIGH)
        time.sleep(3)
        GPIO.output(power_key,GPIO.LOW)
        print("shutdown...")
        time.sleep(18)
        print('Good bye')

    # command:AT+CGPSINFO | back:OK | timeout:1
    def send_at(self,command,back,timeout):
        rec_buff = ''
        ser.write((command+'\r\n').encode())
        time.sleep(timeout)
        if ser.inWaiting():
            time.sleep(0.01)
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
                    #fp = open("/home/pi/Desktop/Code/Data/GPSData.txt","a+")
                    #print (time.ctime(),file=fp,end='  ')
                    #print(float(str_gps[25:27])+float(str_gps[27:36])/60,end=' ',file=fp)
                    #print(float(str_gps[25:27])+float(str_gps[27:36])/60,end=' ',file=fp)
                    #print(str_gps[37:38],end='  ',file=fp)
                    #print(float(str_gps[39:42])+float(str_gps[42:51])/60,end=' ',file=fp)
                    #print(str_gps[52:53],file=fp)
                    #fp.close()
                    
                    print(float(str_gps[25:27])+float(str_gps[27:36])/60)
                    print(float(str_gps[39:42])+float(str_gps[42:51])/60)
                    fp = open("/home/pi/Desktop/Code/Data/GPSData.txt","a+")
                    print (time.ctime(),file=fp,end='  ')
                    print(float(str_gps[25:27])+float(str_gps[27:36])/60,end=' ',file=fp)
                    print(str_gps[37:38],end='  ',file=fp)
                    print(float(str_gps[39:42])+float(str_gps[42:51])/60,end=' ',file=fp)
                    print(str_gps[52:53],file=fp)
                    fp.close()
                return 1
        else:
            print('GPS is not ready')
            return 0

    # 获取GPS位置信息
    def get_gps_position(self):
        print('doing get_gps_postion')
        rec_null = True
        answer = 0
        print('Start GPS session...')
        rec_buff = ''
        self.send_at('AT+CGPS=1,1','OK',1)
        time.sleep(2)
        while rec_null:
            answer = self.send_at('AT+CGPSINFO','+CGPSINFO: ',1)
            if 1 == answer:
                answer = 0
                if ',,,,,,' in rec_buff:
                    print('GPS is not ready')
                    rec_null = False
                    time.sleep(1)
            else:
                print('error %d'%answer)
                rec_buff = ''
                self.send_at('AT+CGPS=0','OK',1)
                return False
                time.sleep(gap)
    def dostart(self):
            # 执行程序
        try:
            # 开启SIM7600X
            #self.power_on(power_key)
            # 获取GPS位置数据
            ser.flushInput()
            self.get_gps_position()
            # 关闭SIM7600X
            #self.power_down(power_key)
        except:
            if ser != None:
                ser.close()
            self.power_down(power_key)
            GPIO.cleanup()
        if ser != None:
                ser.close()
                GPIO.cleanup()
    def run(self):
        self.dostart()
        
def main():
    
    
    #start taking photos automatically
    autoCam = AutoCam()
    #print("startCam...")
    autoCam.setDaemon(True)
    
    
    #get acceleration and angular velocity automatically
    autoObtainAcc = AutoObtainAcc()
    #print("startObtainAcc...")
    autoObtainAcc.setDaemon(True)
    
    
    #get gps location automatically
    autoGps = AutoGps()
    #print("startGps...")
    autoGps.setDaemon(True)
    #print("startGps...")
    
    
    print("main thread start Cam...")
    #start all threads
    autoCam.start()
    #print("startall...")
    
    print("main thread start MPU...")
    autoObtainAcc.start()
    #print("startall...")
    
    
    print("main thread start GPS...")
    autoGps.start()
    #print("startall...")
    
    while(True):
        tmp = 1
    
    
if __name__ == '__main__':
    main()


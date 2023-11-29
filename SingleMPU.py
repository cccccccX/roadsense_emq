from picamera2 import Picamera2, Preview
from time import sleep
import time
import smbus  
import threading
import RPi.GPIO as GPIO
import serial
import matplotlib.pyplot as plt
import numpy as np

gap = 30
alpha = 0.9

       
        
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

cnt = 0
x = []
xMPU = []
yMPU = []
zMPU = []

xMPU2 = []
yMPU2 = []
zMPU2 = []

Ax_ = 0
Ay_ = 0
Az_ = 0

#plt.ion()

class AutoObtainAcc():

    def init(self):
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
        startTime = time.ctime().split()[-2]
        startTime = startTime.split(':')
        for i in range(len(startTime)):
            startTime[i] = int(startTime[i])
        startTime = startTime[0] * 60 * 60 + startTime[1] * 60 + startTime[2]
        
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
            Az -= 1
            
            Gx = gyro_x/131.0
            Gy = gyro_y/131.0
            Gz = gyro_z/131.0
            global cnt

            #print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)
            #fp = open("/home/pi/Desktop/Code/Data/SingleMpuData.txt","a+")
            #print (time.ctime(),file=fp,end=' ')
            #print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az,file=fp)
            #print ("id=%d" %cnt, "Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)
            
            
            ##plt paint
            
            
            cnt+=1
            x.append(cnt)
            #Momentum
            global Ax_
            global Ay_
            global Az_
            
            Ax_ = alpha * Ax_ + (1 - alpha) * Ax
            Ay_ = alpha * Ay_ + (1 - alpha) * Ay
            Az_ = alpha * Az_ + (1 - alpha) * Az
            #Az_ = Az_ / (1 - pow(alpha,cnt))
            #
            xMPU.append(Ax_ / (1 - pow(alpha,cnt)))
            yMPU.append(Ay_ / (1 - pow(alpha,cnt)))
            zMPU.append(Az_ / (1 - pow(alpha,cnt)))
            
            xMPU2.append(Ax)
            yMPU2.append(Ay)
            zMPU2.append(Az)
            
            print ("Time = ",time.ctime(),"id=%d" %cnt, "Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx_=%.2f g" %(Ax_ / (1 - pow(alpha,cnt))), "\tAy=%.2f g" %(Ay_ / (1 - pow(alpha,cnt))), "\tAz=%.2f g" %(Az_ / (1 - pow(alpha,cnt))))
            
            #plt.clf()#clear old paint
            '''
            fig = plt.figure(1)
            #X zhou de tu xiang
            ax1 = plt.subplot(3,1,1)
            plt.plot(x,xMPU,linewidth=0.5,color='b')
            plt.xlabel("time")
            plt.ylabel("X - MPU")
            plt.title("test")
            #plt.axis([0,100,-1,1])
            plt.ylim((-0.75,0.75))
            
            #Y zhou de tu xiang
            ax2 = plt.subplot(3,1,2)
            plt.plot(x,yMPU,linewidth=0.5,color='b')
            plt.xlabel("time")
            plt.ylabel("Y - MPU")
            #plt.axis([0,100,-1,1])
            plt.ylim((-0.75,0.75))
            
            #Z zhou de tu xiang
            ax2 = plt.subplot(3,1,3)
            plt.plot(x,zMPU,linewidth=0.5,color='b')
            plt.xlabel("time")
            plt.ylabel("Z - MPU")
            #plt.axis([0,100,0.5,1.5])
#             plt.ylim((0.5,1.5))
            plt.ylim((-0.75,0.75))
            '''
            #plt.pause(gap)#sleep draw
            #plt.ioff()#close now window of plaint
            ##
            
            
            #fp.close()
#             if (cnt == 200):
#                 break

            
            endTime = time.ctime().split()[-2].split(':')
            global gap
            if (self.check(startTime,endTime,gap)):
#                 print(startTime)
#                 print(endTime)
                break
            
            #sleep(gap)

    def check(self,startTime,end,gap):
        for i in range(len(end)):
            end[i] = int(end[i])
        
        endTime = end[0] * 60 * 60 + end[1] * 60 + end[2]
        
        if ((endTime - startTime) >= gap):
#             print("gap = ",gap)
#             print("startTimeSecond = ",startTime)
#             print("endTimeSecond = ",endTime)
            return True
        else:
            return False
        

    def run(self):
        self.init()
        self.dostart()
        
ser = serial.Serial('/dev/ttyS0',115200)
ser.flushInput()

power_key = 6
rec_buff = ''

def main():
    
    
    autoObtainAcc = AutoObtainAcc()
    
    autoObtainAcc.run()
    
#     plt.show()
#     print(xMPU[len(xMPU) - 1])
    t = time.ctime()
    print(t)
    print(t.split())
    t = t.split()[-2]
    print(t)
    t = t.split(':')
    print(t)
    #sleep(5)
    
    zitaZ = 100000000
    for i in zMPU:
        zitaZ = min(zitaZ,i)
    print("zitaZ = ",zitaZ)
    
#     minZ = zitaZ
#     minX = 100000000
#     minY = 100000000
#     for i in xMPU:
#         minX = min(minX,i)
#     for i in yMPU:
#         minY = min(minY,i)
#         
#     zitaG1 = max(minZ,max(minX,minY))
#     
#     maxX = -100000000
#     maxY = -100000000
#     maxZ = -100000000
#     for i in xMPU:
#         maxX = max(maxX,i)
#     for i in yMPU:
#         maxY = max(maxY,i)
#     for i in zMPU:
#         maxZ = max(maxZ,i)
#     
#     zitaG2 = min(maxX,min(maxY,maxZ))
#     
#     print("zitaG1 = ",zitaG1)
#     print("zitaG2 = ",zitaG2)
    
    
    """
    fig = plt.figure(1)
    #X zhou de tu xiang
    ax1 = plt.subplot(3,1,1)
    plt.plot(x,xMPU)
    plt.xlabel("time")
    plt.ylabel("X - MPU")
    plt.title("test")
    plt.axis([0,100,-1,1])
    #Y zhou de tu xiang
    ax2 = plt.subplot(3,1,2)
    plt.plot(x,yMPU)
    plt.xlabel("time")
    plt.ylabel("Y - MPU")
    plt.axis([0,100,-1,1])
    #Z zhou de tu xiang
    ax2 = plt.subplot(3,1,3)
    plt.plot(x,zMPU)
    plt.xlabel("time")
    plt.ylabel("Z - MPU")
    plt.axis([0,100,0.5,1.5])
    """
    ###plt.show()
    
    ##exit(0)
    
    
    
    #plt.clf()#clear old paint
    
    fig = plt.figure(1)
    #X zhou de tu xiang
    ax1 = plt.subplot(4,1,1)
    plt.plot(x,xMPU,linewidth=1,color='g')
    plt.plot(x,xMPU2,linewidth=0.5,color='b')
    plt.xlabel("time")
    plt.ylabel("X - MPU")
    plt.title("test")
    #plt.axis([0,100,-1,1])
    plt.ylim((-0.75,0.75))
    
    #Y zhou de tu xiang
    ax2 = plt.subplot(4,1,2)
    plt.plot(x,yMPU,linewidth=1,color='g')
    plt.plot(x,yMPU2,linewidth=0.5,color='b')
    plt.xlabel("time")
    plt.ylabel("Y - MPU")
    #plt.axis([0,100,-1,1])
    plt.ylim((-0.75,0.75))
    
    #Z zhou de tu xiang
    ax2 = plt.subplot(4,1,3)
    plt.plot(x,zMPU,linewidth=1,color='g')
    plt.plot(x,zMPU2,linewidth=0.5,color='b')
    plt.xlabel("time")
    plt.ylabel("Z - MPU")
    #plt.axis([0,100,0.5,1.5])
#             plt.ylim((0.5,1.5))
    plt.ylim((-0.75,0.75))
    
    #plt.pause(gap)#sleep draw
    #plt.ioff()#close now window of plaint
    ##
    
    mo = []
    mo2 = [] # fix jiang zao mount
    len2 = 0
    k = len(xMPU)
    for i in range(k):
        len1 = xMPU[i] * xMPU[i] + yMPU[i] * yMPU[i] + zMPU[i] * zMPU[i]
        len1 = pow(len1,0.5)
        len2 = alpha * len2 + (1 - alpha) * len1
        
        mo.append(len1)
        mo2.append(len2 / (1 - pow(alpha,i + 1)))
        print(len1,len2 / (1 - pow(alpha,i + 1)))
    
    ax2 = plt.subplot(4,1,4)
    plt.plot(x,mo,linewidth=0.5,color='b',label='origin')
    plt.plot(x,mo2,linewidth=1,color='g',label='mount')
    plt.xlabel("time")
    plt.ylabel("mo")
    #plt.ylim((-0.75,0.75))
    
    plt.show()
    
    
if __name__ == '__main__':
    main()



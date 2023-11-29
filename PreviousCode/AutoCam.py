from picamera2 import Picamera2, Preview
import time
picam2 = Picamera2()
while True:
    picam2.start_and_capture_file("/home/pi/Desktop/Code/Data/photos/"+time.ctime()+".jpg")
    time.sleep(2)

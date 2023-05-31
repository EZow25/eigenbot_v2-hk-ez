import serial
import time
#s = serial.Serial('COM4')
print("Working")
s = serial.Serial('/dev/ttyACM0')
t2 = time.time()

freq = 0
while 1:
    t1 = time.time()
    res = s.readline()
    
    print(res)
    # freq = 1/(t1-t2)
    # print(res.decode() + ", Frequency: " + str(freq))
    # t2 = t1

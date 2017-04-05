import myroomba
import time
import serial

ZhuLi=myroomba.robot()
ZhuLi._debug=True
ZhuLi.connect("COM4")
speed=-50

for i in range (20):
	ZhuLi._drive(speed, 0, turn_inplace=False)
	speed-=10
	time.sleep(1)

ZhuLi.disconnect()

"""
puerto=serial.Serial("COM4", baudrate=115200, timeout=0.5)
time.sleep(.3)
puerto.write(bytes([128]))
time.sleep(.3)
puerto.write(bytes([131]))

puerto.write(bytes([0x89]))
puerto.write(bytes([0xFF]))
puerto.write(bytes([0x00]))
puerto.write(bytes([0xFF]))
puerto.write(bytes([0xFF]))
#puerto.write(bytes([131]))

time.sleep(4)


puerto.close()
"""
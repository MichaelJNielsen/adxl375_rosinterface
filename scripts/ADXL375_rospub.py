#!/usr/bin/env python3
from smbus2 import SMBus
from ctypes import c_int8
from sensor_msgs.msg import Imu

import signal, time, rospy

def keyboardInterruptHandler(signal,frame):
    print('Interrupted')
    exit(0)

signal.signal(signal.SIGINT,keyboardInterruptHandler)

def talker(x,y,z):
    pub = rospy.Publisher('ADXL375/SafeEye', Imu, queue_size=10 )
    rospy.init_node('ADXL375/SafeEye', anonymous=True)
    rate = rospy.Rate(50) #50hz
    data.linear_acceleration.x = x
    data.linear_acceleration.y = y
    data.linear_acceleration.z = z
    pub.publish(data)
    rate.sleep()

#I2C channel
i2c_ch = 0

#Device address
ADXL375_DEVICE = 0x53

#Register addresses
ADXL375_POWER_CTL = 0x2D
ADXL375_DATAX0 = 0x32
ADXL375_OFSX = 0x1E
ADXL375_OFSY = 0x1F
ADXL375_OFSZ = 0x20

#Initialize I2C (SMBus)
bus = SMBus(i2c_ch)

#Power on ADXL375
bus.write_byte_data(ADXL375_DEVICE, ADXL375_POWER_CTL,0)
bus.write_byte_data(ADXL375_DEVICE, ADXL375_POWER_CTL,16)
bus.write_byte_data(ADXL375_DEVICE, ADXL375_POWER_CTL,8)

#Set offset variables found through calibration script
bus.write_byte_data(ADXL375_DEVICE, ADXL375_OFSX, -1)
bus.write_byte_data(ADXL375_DEVICE, ADXL375_OFSY, -1)
bus.write_byte_data(ADXL375_DEVICE, ADXL375_OFSZ, 3)

data = Imu()
while True:
    block = bus.read_i2c_block_data(ADXL375_DEVICE,ADXL375_DATAX0,6)
    x_raw = c_int8(block[0]).value | c_int8(block[1]).value << 8
    x = x_raw/20.5
    y_raw = c_int8(block[2]).value | c_int8(block[3]).value << 8
    y = y_raw/20.5
    z_raw = c_int8(block[4]).value | c_int8(block[5]).value << 8
    z = z_raw/20.5
    print(x,y,z)
    talker(x,y,z)




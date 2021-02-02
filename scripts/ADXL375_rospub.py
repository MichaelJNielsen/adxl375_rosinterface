#!/usr/bin/env python3
from smbus2 import SMBus
from ctypes import c_int8
from sensor_msgs.msg import Imu

import signal, time, rospy

def keyboardInterruptHandler(signal,frame):
    print('Interrupted!')
    exit(0)

signal.signal(signal.SIGINT,keyboardInterruptHandler)

def Setup(ADXL375_DEVICE,OFSX,OFSY,OFSZ):
    #Power on ADXL375
    bus.write_byte_data(ADXL375_DEVICE, ADXL375_POWER_CTL,0) #Standby during setup
    bus.write_byte_data(ADXL375_DEVICE, ADXL375_BW_RATE, 9) #50Hz
    bus.write_byte_data(ADXL375_DEVICE, ADXL375_FIFO_CTL, 0) #Bypass
    bus.write_byte_data(ADXL375_DEVICE, ADXL375_POWER_CTL,8) #Setup finished - enter measuring mode
    

    #Set offset variables found through calibration script
    bus.write_byte_data(ADXL375_DEVICE, ADXL375_OFSX, OFSX)
    bus.write_byte_data(ADXL375_DEVICE, ADXL375_OFSY, OFSY)
    bus.write_byte_data(ADXL375_DEVICE, ADXL375_OFSZ, OFSZ)

def ReadAxes(ADXL375_DEVICE):
    block = bus.read_i2c_block_data(ADXL375_DEVICE,ADXL375_DATAX0,6)
    x_raw = c_int8(block[0]).value | c_int8(block[1]).value << 8
    x = x_raw/20.5
    y_raw = c_int8(block[2]).value | c_int8(block[3]).value << 8
    y = y_raw/20.5
    z_raw = c_int8(block[4]).value | c_int8(block[5]).value << 8
    z = z_raw/20.5
    #print(x_raw)
    return(x,y,z)

#I2C channel
i2c_ch = 0

#Device address
ADXL375_DEVICE1 = 0x53
ADXL375_DEVICE2 = 0x1D

#Register addresses
ADXL375_POWER_CTL = 0x2D
ADXL375_BW_RATE = 0x2C
ADXL375_FIFO_CTL = 0x38
ADXL375_DATAX0 = 0x32
ADXL375_OFSX = 0x1E
ADXL375_OFSY = 0x1F
ADXL375_OFSZ = 0x20

#Initialize I2C (SMBus)
bus = SMBus(i2c_ch)

#Startup
Setup(ADXL375_DEVICE1,-1,2,1)
#Setup(ADXL375_DEVICE2,0,-2,-1)

if __name__ == '__main__':
    pub1 = rospy.Publisher('ADXL375/Accel1', Imu, queue_size=10)
    pub2= rospy.Publisher('ADXL375/Accel2', Imu, queue_size=10)
    rospy.init_node('ADXL375', anonymous=True)
    rate = rospy.Rate(2000) #50hz
    data1 = Imu()
    data2 = Imu()

    while True:
        [x1,y1,z1] = ReadAxes(ADXL375_DEVICE1)
        #[x2,y2,z2] = ReadAxes(ADXL375_DEVICE2)
        [x2,y2,z2] = [1,2,3]

        data1.linear_acceleration.x = x1
        data1.linear_acceleration.y = y1
        data1.linear_acceleration.z = z1

        data2.linear_acceleration.x = x2
        data2.linear_acceleration.y = y2
        data2.linear_acceleration.z = z2

        pub1.publish(data1)
        pub2.publish(data2)
        rate.sleep()

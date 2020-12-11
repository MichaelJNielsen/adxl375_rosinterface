from smbus2 import SMBus
from ctypes import c_int8
import signal, time

def keyboardInterruptHandler(signal,frame):
    print('Interrupted')
    print('x offset:', x_offset, 'y offset:', y_offset, 'z offset:', z_offset)
    exit(0)

signal.signal(signal.SIGINT,keyboardInterruptHandler)

#I2C channel
i2c_ch = 0

#Device address
device = input('Which device do you wish to calibrate? (1 or 2?)')
if device == '1':
    ADXL375_DEVICE = 0x53
elif device == '2':
    ADXL375_DEVICE = 0x1D
else:
    print('device not available')
    exit(0)

#Register addresses
ADXL375_POWER_CTL = 0x2D
ADXL375_DATAX0 = 0x32
ADXL375_OFSX = 0x1E
ADXL375_OFSY = 0x1F
ADXL375_OFSZ = 0x20

#Initialize I2C (SMBus)
bus = SMBus(i2c_ch)

#Power on ADXL375
bus.write_byte_data(ADXL375_DEVICE,ADXL375_POWER_CTL,0)
bus.write_byte_data(ADXL375_DEVICE,ADXL375_POWER_CTL,16)
bus.write_byte_data(ADXL375_DEVICE,ADXL375_POWER_CTL,8)
i = 0
x_list = []
y_list = []
z_list = []

bus.write_byte_data(ADXL375_DEVICE, ADXL375_OFSX, 0x00)
bus.write_byte_data(ADXL375_DEVICE, ADXL375_OFSY, 0x00)
bus.write_byte_data(ADXL375_DEVICE, ADXL375_OFSZ, 0x00)

start_flag = input('Is the accelerometer on a level surface? [Y/n]')
if start_flag != 'Y':
    print('Place accelerometer on level surface and try again')
    exit(0)

while i<= 20:
    block = bus.read_i2c_block_data(ADXL375_DEVICE,ADXL375_DATAX0,6)
    x_raw = c_int8(block[0]).value | c_int8(block[1]).value << 8
    x = x_raw/20.5
    x_list.append(x_raw)
    y_raw = c_int8(block[2]).value | c_int8(block[3]).value << 8
    y = y_raw/20.5
    y_list.append(y_raw)
    z_raw = c_int8(block[4]).value | c_int8(block[5]).value << 8
    z = z_raw/20.5
    z_list.append(z_raw)
    print('x:',x,'y:',y,'z:',z)
    i = i+1
    time.sleep(0.01) #100Hz
x_avr = sum(x_list)/len(x_list)
y_avr = sum(y_list)/len(y_list)
z_avr = sum(z_list)/len(z_list)
print('x_avr:', x_avr, 'y_avr:', y_avr, 'z_avr', z_avr)
x_offset = -round(x_avr/4)
y_offset = -round(y_avr/4)
z_error = z_avr - 20.5
print('z_error:',z_error)
z_offset = -round(z_error/4)
print('x_offset:',x_offset,'y_offset:',y_offset,'z_offset:',z_offset)
bus.write_byte_data(ADXL375_DEVICE, ADXL375_OFSX, x_offset)
bus.write_byte_data(ADXL375_DEVICE, ADXL375_OFSY, y_offset)
bus.write_byte_data(ADXL375_DEVICE, ADXL375_OFSZ, z_offset)

while True:
    block = bus.read_i2c_block_data(ADXL375_DEVICE,ADXL375_DATAX0,6)
    x = (c_int8(block[0]).value | c_int8(block[1]).value << 8)/20.5
    y = (c_int8(block[2]).value | c_int8(block[3]).value << 8)/20.5
    z = (c_int8(block[4]).value | c_int8(block[5]).value << 8)/20.5
    print('x:',x,'y:',y,'z:',z)
    time.sleep(0.1) #10Hz



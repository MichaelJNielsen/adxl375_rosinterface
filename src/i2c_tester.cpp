#include <unistd.h>			//Needed for I2C port
#include <fcntl.h>			//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

//Device address
const int ADXL375_DEVICE1 = 0x53;
const int ADXL375_DEVICE2 = 0x1D;

//Register addresses
unsigned char ADXL375_POWER_CTL = 0x2D;
const int ADXL375_BW_RATE = 0x2C;
const int ADXL375_FIFO_CTL = 0x38;
const int ADXL375_DATAX0 = 0x32;
const int ADXL375_OFSX = 0x1E;
const int ADXL375_OFSY = 0x1F;
const int ADXL375_OFSZ = 0x20;

int file_i2c;
unsigned char a[6] = {0};

void open_bus() {
    char *filename = (char*)"/dev/i2c-0"; //Define which i2c port we use. To see which one the device is connected to use "sudo i2cdetect -y 0" or 1
    file_i2c = open(filename, O_RDWR); //Open the i2c bus as both read and write.
    if (file_i2c < 0)
    {
        printf("Failed to open the specified i2c bus");
        exit(0);
    }
    usleep(20000);
return;
}

void connect_device(int addr) {
    if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
    {
	    printf("Failed to acquire bus access and/or talk to slave.\n");
	    //ERROR HANDLING; you can check errno to see what went wrong
	    exit(0);
    }
    else
        printf("Connected to device");
    usleep(20000);
return;
}

void i2c_write(unsigned char bytes) {
    unsigned char outbuffer[1] = {0};
    outbuffer[0] = bytes;
    ssize_t w { write(file_i2c, outbuffer, sizeof(outbuffer))};
    if (w!=sizeof(outbuffer)) {
        std::cout << "Could not write full array" << std::endl;
        exit(0);
    }
    if (w<0) {
        std::cout << "Write error" << std::endl;
        exit(0);
    }
return;
}

void setup(int OFSX, int OFSY, int OFSZ)
{
    printf("enter setup \n");
    unsigned char outbuffer[1] = {0};

    //Contact power control register:

    //i2c_write(ADXL375_POWER_CTL);

    //outbuffer[0] = ADXL375_POWER_CTL;
    //ssize_t w { write(file_i2c, outbuffer, sizeof(outbuffer))};
    //if (w!=sizeof(outbuffer)) {
    //    std::cout << "Could not write full array" << std::endl;
    //    exit(0);
    //}
    //if (w<0) {
    //    std::cout << "Write error" << std::endl;
    //    exit(0);
    //}
    usleep(20000);

    //Set into standby mode:
    outbuffer[0] = 0b00000000;
    ssize_t w { write(file_i2c, outbuffer, sizeof(outbuffer))};
    if (w!=sizeof(outbuffer)) {
        std::cout << "Could not write full array" << std::endl;
        exit(0);
    }
    if (w<0) {
        std::cout << "Write error" << std::endl;
        exit(0);
    }
    usleep(20000);

    //Set into measure mode:
    outbuffer[0] = 0b00001000;
    w = write(file_i2c, outbuffer, sizeof(outbuffer));
    if (w!=sizeof(outbuffer)) {
        std::cout << "Could not write full array" << std::endl;
        exit(0);
    }
    if (w<0) {
        std::cout << "Write error" << std::endl;
        exit(0);
    }
    usleep(20000);

    //write(file_i2c, &ADXL375_POWER_CTL, 1);
    //buffer[0] = 0;
    //std::cout << "POWER_CTL - writing: " << buffer;
    //write(file_i2c, buffer, 1);

    //std::cout << "BW_RATE - writing: " << &ADXL375_BW_RATE;
    //write(file_i2c, &ADXL375_BW_RATE, 1);
    //buffer[0] = 9;
    //std::cout << "BW_RATE - writing: " << buffer;
    //write(file_i2c, buffer, 1);

    //std::cout << "FIFO_CTL - writing: " << &ADXL375_FIFO_CTL;
    //write(file_i2c, &ADXL375_FIFO_CTL, 1);
    //buffer[0] = 0;
    //std::cout << "FIFO_CTL - writing: " << buffer;
    //write(file_i2c, buffer, 1);

    //std::cout << "POWER_CTL - writing: " << &ADXL375_POWER_CTL;
    //write(file_i2c, &ADXL375_POWER_CTL, 1);
    //buffer[0] = 8;
    //std::cout << "POWER_CTL - writing: " << buffer;
    //write(file_i2c, buffer, 1);

    //Offset x,y,z
return;
}

void read_axes()
{
    unsigned char outbuffer[1] = {0};
    //Contact output registers:
    outbuffer[0] = ADXL375_DATAX0;
    ssize_t w { write(file_i2c, outbuffer, sizeof(outbuffer))};
    if (w!=sizeof(outbuffer)) {
        std::cout << "Could not write full array" << std::endl;
        exit(0);
    }
    if (w<0) {
        std::cout << "Write error" << std::endl;
        exit(0);
    }

    //Read what it sends
    ssize_t const r { read(file_i2c, a, sizeof(a))};
    if (r!=sizeof(a)) {
        std::cout << "Could not read full array" << std::endl;
        printf("r = %ld", r);
        printf("\n");
        exit(0);
    }
    if (r<0) {
        std::cout << "Read error" << std::endl;
        exit(0);
    }
return;
}

int main()
{
    open_bus();                      //Open I²C bus.
    connect_device(ADXL375_DEVICE1); //Establish connection to device.
    setup(0,0,0);    //Start the accelerometer and set offsets.

    while (true)
    {
        read_axes();
        printf("read: ");
        for(unsigned int i(0); i<sizeof(a); ++i) {
            //printf("%02x", unsigned int(a[i]));
	    printf("%d",a[i]);
	    printf(" ");
        }
	usleep(200000);
	printf("\n");
	int x_raw = int8_t(a[0]) | int8_t(a[1]) << 8;
	float x = x_raw/20.5;
	int y_raw = int8_t(a[2]) | int8_t(a[3]) << 8;
	float y = y_raw/20.5;
	int z_raw = int8_t(a[4]) | int8_t(a[5]) << 8;
	float z = z_raw/20.5;
	printf("x = %f ", x);
	printf("y = %f ", y);
	printf("z = %f ", z);
	printf("\n");
    }
return 0;
}

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
const int ADXL375_POWER_CTL = 0x2D;
const int ADXL375_BW_RATE = 0x2C;
const int ADXL375_FIFO_CTL = 0x38;
const int ADXL375_DATAX0 = 0x32;
const int ADXL375_OFSX = 0x1E;
const int ADXL375_OFSY = 0x1F;
const int ADXL375_OFSZ = 0x20;

int file_i2c;


void open_bus()
{
    char *filename = (char*)"/dev/i2c-0"; //Define which i2c port we use. To see which one the device is connected to use "sudo i2cdetect -y 0" or 1
    file_i2c = open(filename, O_RDWR); //Open the i2c bus as both read and write.
    if (file_i2c < 0)
    {
        printf("Failed to open the specified i2c bus");
        exit(0);
    }
return; 
}

void connect_device(int addr)
{
    if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
    {
	    printf("Failed to acquire bus access and/or talk to slave.\n");
	    //ERROR HANDLING; you can check errno to see what went wrong
	    exit(0);
    }
    else
        printf("Connected to device");
return;
}

void setup(int OFSX, int OFSY, int OFSZ)
{   
    printf("enter setup");
    unsigned char buffer[60] = {0};
    std::cout << "POWER_CTL - writing: " << &ADXL375_POWER_CTL;
    write(file_i2c, &ADXL375_POWER_CTL, 1);
    buffer[0] = 0;
    std::cout << "POWER_CTL - writing: " << buffer;
    write(file_i2c, buffer, 1);
    
    std::cout << "BW_RATE - writing: " << &ADXL375_BW_RATE;
    write(file_i2c, &ADXL375_BW_RATE, 1);
    buffer[0] = 9;
    std::cout << "BW_RATE - writing: " << buffer;
    write(file_i2c, buffer, 1);
    
    std::cout << "FIFO_CTL - writing: " << &ADXL375_FIFO_CTL;
    write(file_i2c, &ADXL375_FIFO_CTL, 1);
    buffer[0] = 0;
    std::cout << "FIFO_CTL - writing: " << buffer;
    write(file_i2c, buffer, 1);
    
    std::cout << "POWER_CTL - writing: " << &ADXL375_POWER_CTL;
    write(file_i2c, &ADXL375_POWER_CTL, 1);
    buffer[0] = 8;
    std::cout << "POWER_CTL - writing: " << buffer;
    write(file_i2c, buffer, 1);
    
    //Offset x,y,z
return;
}

void read_axes()
{
    unsigned char buffer[60] = {0};
    int length = 6;
    write(file_i2c, &ADXL375_DATAX0, 1);
    read(file_i2c, buffer, length);
    printf("Data read: %s\n", buffer);
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
        usleep(1000000);
    }
return 0;
}









//int main() {
//    int file_i2c;
//    int length;
//    unsigned char buffer[60] = {0};

//    while (true)
  //  {
    //    read(file_i2c, buffer, 4);
    //    printf("Data read: %s\n", buffer);
//	usleep(1000000);
 //   }
    //----- READ BYTES -----
    //length = 16;			//<<< Number of bytes to read
    //if (read(file_i2c, buffer, length) != length)		//read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
    //{
	    //ERROR HANDLING: i2c transaction failed
	//    printf("Failed to read from the i2c bus.\n");
    //}
    //else
    //{
	//    printf("Data read: %s\n", buffer);
    //}


    //----- WRITE BYTES -----
    //buffer[0] = 0x01;
    //buffer[1] = 0x02;
    //length = 2;			//<<< Number of bytes to write
    //if (write(file_i2c, buffer, length) != length)		//write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
    //{
	    /* ERROR HANDLING: i2c transaction failed */
//	    printf("Failed to write to the i2c bus.\n");
  //  }
//return 0;
//}

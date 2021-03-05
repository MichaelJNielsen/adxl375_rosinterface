#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "adxl375_rosinterface/Accel.h"

#include <unistd.h>			//Needed for I2C port
#include <fcntl.h>			//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <string>
#include <bitset>

using namespace std;

//Device address
const int ADXL375_DEVICE1 = 0x53; //1010011
const int ADXL375_DEVICE2 = 0x1D; //11101

//Register addresses
const int ADXL375_POWER_CTL = 0x2D; //101101
const int ADXL375_BW_RATE = 0x2C;       //101100
const int ADXL375_FIFO_CTL = 0x38;      //111000
const int ADXL375_DATAX0 = 0x32;        //110010
const int ADXL375_OFSX = 0x1E;          //11110
const int ADXL375_OFSY = 0x1F;          //11111
const int ADXL375_OFSZ = 0x20;          //100000

int file_i2c;
unsigned char a[6] = {0};
int16_t x_raw; int16_t y_raw; int16_t z_raw;

string dotfile;
ofstream outputFile;

//--------------------------- Termination sequence -------------------------------------
void signal_callback_handler(int signum) {
    printf("Interrupted! \n Closing csv file... \n");
    outputFile.close();
    exit(signum);
}



//-------------------------- I2C functions ---------------------------------------------
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
return; 
}

void i2c_write(unsigned char bytes0, unsigned char bytes1) {
    unsigned char outbuffer[2] = {0};
    outbuffer[0] = bytes0;
    outbuffer[1] = bytes1;
    ssize_t w { write(file_i2c, outbuffer, sizeof(outbuffer))};
    w = write(file_i2c, outbuffer, sizeof(outbuffer));
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

void read_axes(double *x, double *y, double *z)
{
    //Contact data register
    i2c_write(ADXL375_DATAX0,0b00000000);

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
    
	x_raw = ((int)(a[0] | a[1] << 8));
	*x = x_raw/20.5;
	y_raw = ((int)(a[2] | a[3] << 8));
	*y = y_raw/20.5;
	z_raw = ((int)(a[4] | a[5] << 8));
	*z = z_raw/20.5;
return;
}

void setup(int OFSX, int OFSY, int OFSZ)
{
    //Set into standby mode
    i2c_write(ADXL375_POWER_CTL, 0b00000000);
    usleep(20000);

    //Set bandwidth and output data rate 800Hz
    i2c_write(ADXL375_BW_RATE,0b00001101);
    usleep(2000);

    //Set FIFO to Bypass
    i2c_write(ADXL375_FIFO_CTL, 0b00000000);
    usleep(2000);

    //Set into measure mode
    i2c_write(ADXL375_POWER_CTL, 0b00001000);
    usleep(20000);

    //Set Offset calibrations
    i2c_write(ADXL375_OFSX, OFSX);
    usleep(20000);
    i2c_write(ADXL375_OFSY, OFSY);
    usleep(20000);
    i2c_write(ADXL375_OFSZ, OFSZ);
    usleep(20000);

return;
}





//---------------------------------- CSV functions ---------------------------------------------------
void csv_setup()
{
	string file;
    	char input;

    	printf("Input desired file name \n");
    	getline(cin, file);
    	dotfile = file + ".csv";

	ifstream ifile(dotfile);
	if (ifile) {
		printf("File already exists. Overwrite? [y/n] \n");
		cin >> input;
		if (input == 'y' || input == 'Y') {
            		printf("Overwriting %s \n", dotfile.c_str());
		}
		else {
	    	printf("Exiting, please try a different file name. \n");
	    	exit(0);
		}
	}
	outputFile.open(dotfile);
	printf("csv file opened \n");
	
	string header = "byte0 , byte1 , byte2 , byte3 , byte4 , byte5 , x_raw , y_raw , z_raw , x , y , z \n";
	
	printf("Writing header \n");
	outputFile << header;
return;
}

void csv_updater(float x, float y, float z)
{
	bitset<8> byte0(a[0]);
	bitset<8> byte1(a[1]);
	bitset<8> byte2(a[2]);
	bitset<8> byte3(a[3]);
	bitset<8> byte4(a[4]);
	bitset<8> byte5(a[5]);
	
	outputFile << byte0 << "," << byte1 << "," << byte2 << "," << byte3 << "," << byte4 << "," << byte5 << "," << x_raw << "," << y_raw << "," << z_raw << "," << x << "," << y << "," << z << "\n" ;
}






//--------------------------------------- Main -----------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "adxl_cppinterface");
  ros::NodeHandle n;
  //ros::Publisher chatter_pub = n.advertise<adxl375_rosinterface::Accel>("ADXL375/Accel1", 3);
  ros::Rate loop_rate(800);
  
  open_bus();                       //Open I²C bus
  connect_device(ADXL375_DEVICE2);  //Establish connection to device
  //setup(-1,2,1);                   //Start the accelerometer and set offsets
  setup(0,-2,-1);
  csv_setup();
  
  adxl375_rosinterface::Accel data1;

  while (ros::ok())
  {
    read_axes(&data1.x, &data1.y, &data1.z);
    data1.stamp = ros::Time::now();

    csv_updater(data1.x, data1.y, data1.z);
    
    //ROS_INFO("x: %f, y: %f, z: %f", data1.x, data1.y, data1.z);
    //chatter_pub.publish(data1);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}

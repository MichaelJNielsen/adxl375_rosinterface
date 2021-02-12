#include "ros/ros.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "adxl375_rosinterface/Accel.h"
#include <string>
#include <ctime>
#include <chrono>
#include "std_msgs/String.h"
#include "geometry_msgs/TransformStamped.h"
#include "serial_interface/Razorimu.h"
#include "xsens_mti_driver/nine_dof_imu.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Joy.h"

using namespace std;

//--------------------------------- Global Variables ---------------------------------------------------------------
geometry_msgs::TransformStamped vicon_data;
serial_interface::Razorimu seye_data;
xsens_mti_driver::nine_dof_imu xsens_data;
sensor_msgs::Imu djiimu_data;
sensor_msgs::Joy djirc_data;


int csv_file;
std::string dotfile;

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





//--------------------------------- Various functions ---------------------------------------------------------------
//Get time stamp in milli seconds
uint64_t millis()
{
	uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
	return ms;
}

//Get time stamp in micro seconds
uint64_t micros()
{
	uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
	return us;
}


//--------------------------------- I2C functions and variables ------------------------------------------------------

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

void connect_device(int device_addr) {
    if (ioctl(file_i2c, I2C_SLAVE, device_addr) < 0)
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

void read_axes(int device_addr, double *x, double *y, double *z)
{
    //Establish connection to device
    connect_device(device_addr);
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
    
	int x_raw = int8_t(a[0]) | int8_t(a[1]) << 8;
	*x = x_raw/20.5;
	int y_raw = int8_t(a[2]) | int8_t(a[3]) << 8;
	*y = y_raw/20.5;
	int z_raw = int8_t(a[4]) | int8_t(a[5]) << 8;
	*z = z_raw/20.5;
return;
}

void setup(int device_addr, int OFSX, int OFSY, int OFSZ)
{
    //Establish connection to device
    connect_device(device_addr);
    
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


//------------------------------ Write to CSV functions and variables ---------------------------------------
void csv_write(string text) {
	ssize_t const w { write(csv_file, text.c_str(), text.length())};
    	if (w != text.length()) {
		printf("Could not write full string \n");
		exit(0);
    	}
    	if (w<0) {
		printf("Write error \n");
		exit(0);
    	}
return;
}

void setup_csv()
{
    string file;
    char input;

    printf("Input desired file name \n");
    getline(cin, file);
    dotfile = file + ".csv";

    csv_file = open(dotfile.c_str(), O_WRONLY);
    if (csv_file > 0) {
	printf("File already exists. Overwrite? [y/n] \n");
	cin >> input;
	if (input == 'y' || input == 'Y') {
            printf("Overwriting %s \n", dotfile.c_str());
	}
	else {
	    close(csv_file);
	    printf("Exiting, please try a different file name. \n");
	    exit(0);
	}
    }
    else {
        printf("Creating file \n");
	csv_file = open(dotfile.c_str(), O_CREAT | O_WRONLY, 00444);
    }
    if (csv_file < 0) {
        printf("Failed to open specified file \n");
	exit(0);
    }
    printf("csv file opened \n");

    string header_string, metadata, vicon, SafeEye, Accel1, Accel2, DJI;

    metadata = "name, date, time_since_start";
    vicon =  "vicon_sequence, vicon_time_stamp, vicon_translation_x, vicon_translation_y, vicon_translation_z, vicon_rotation_x, vicon_rotation_y, vicon_rotation_z, vicon_rotation_w";
    SafeEye = "seye_razor_time_stamp, seye_razor_acc_x, seye_razor_acc_y, seye_razor_acc_z, seye_razor_gyro_x, seye_razor_gyro_y, seye_razor_gyro_z, seye_razor_mag_x, seye_razor_mag_y, seye_razor_mag_z";
    Accel1 = "accel1_xsens_time_stamp, accel1_xsens_acc_x,  accel1_xsens_acc_y, accel1_xsens_acc_z, accel1_xsens_gyro_x, accel1_xsens_gyro_y, accel1_xsens_gyro_z, accel1_xsens_mag_x, accel1_xsens_mag_y, accel1_xsens_mag_z, accel1_adxl_sequence, accel1_adxl_acc_x, accel1_adxl_acc_y, accel1_adxl_acc_z";
    Accel2 = "accel2_adxl_sequence, accel2_adxl_acc_x, accel2_adxl_acc_y, accel2_adxl_acc_z";
    DJI = "dji_imu_sequence, dji_imu_time_stamp, dji_imu_orientation_x, dji_imu_orientation_y, dji_imu_orientation_z, dji_imu_orientation_w, dji_imu_ang_vel_x, dji_imu_ang_vel_y, dji_imu_ang_z, dji_imu_acc_x, dji_imu_acc_y, dji_imu_acc_z, rc_sequence, rc_time_stamp, rc_roll, rc_pitch, rc_yaw, rc_throttle";

    header_string = metadata + "," + vicon + "," + SafeEye + "," + Accel1 + "," + Accel2 + ","  + DJI + "\n";

    csv_write(header_string);
}



void csv_updater(adxl375_rosinterface::Accel accel1_data, adxl375_rosinterface::Accel accel2_data)  {
	static int start_flag = 0;
	static time_t start = time(0);
	static uint64_t start_us = micros();
	string metadata_string, vicon_string, seye_string, accel1_string, accel2_string, dji_string;

	//vicon
	vicon_string = to_string(vicon_data.header.seq) + "," + to_string(vicon_data.header.stamp.sec) + "." + to_string(vicon_data.header.stamp.nsec*10^-8) + "," + to_string(vicon_data.transform.translation.x) + "," + to_string(vicon_data.transform.translation.y) + "," + to_string(vicon_data.transform.translation.z) + "," + to_string(vicon_data.transform.rotation.x) + "," + to_string(vicon_data.transform.rotation.y) + "," + to_string(vicon_data.transform.rotation.z) + "," + to_string(vicon_data.transform.rotation.w);

	//seye
	seye_string = to_string(seye_data.time_stamp) + "," + to_string(seye_data.acc_x) + "," + to_string(seye_data.acc_y) + "," + to_string(seye_data.acc_z) + "," + to_string(seye_data.gyro_x) + "," + to_string(seye_data.gyro_y) + "," + to_string(seye_data.gyro_z) + "," + to_string(seye_data.mag_x) + "," + to_string(seye_data.mag_y) + "," + to_string(seye_data.mag_z);

	//accel1
	accel1_string = to_string(xsens_data.stamp.sec) + "." + to_string(xsens_data.stamp.nsec*10^-8) + "," + to_string(xsens_data.acc_x) + "," + to_string(xsens_data.acc_y) + "," + to_string(xsens_data.acc_z) + "," + to_string(xsens_data.gyro_x) + "," +  to_string(xsens_data.gyro_y) + "," +  to_string(xsens_data.gyro_z) + "," + to_string(xsens_data.mag_x) + "," +  to_string(xsens_data.mag_y) + "," +  to_string(xsens_data.mag_z) + "," + to_string(accel1_data.stamp.sec) + "." + to_string(accel1_data.stamp.nsec*10^-8) + "," + to_string(accel1_data.x) + "," + to_string(accel1_data.y) + "," + to_string(accel1_data.z);
	
	//accel2
	accel2_string = to_string(accel2_data.stamp.sec) + "." + to_string(accel2_data.stamp.nsec*10^-8) + "," + to_string(accel2_data.x) + "," + to_string(accel2_data.y) + "," + to_string(accel2_data.z);

	//dji
	dji_string = to_string(djiimu_data.header.seq) + "," + to_string(djiimu_data.header.stamp.sec) + "." + to_string(djiimu_data.header.stamp.nsec*10^-8) + "," + to_string(djiimu_data.orientation.x) + "," + to_string(djiimu_data.orientation.y) + "," + to_string(djiimu_data.orientation.z) + "," + to_string(djiimu_data.orientation.w) + "," + to_string(djiimu_data.angular_velocity.x) + "," + to_string(djiimu_data.angular_velocity.y) + "," + to_string(djiimu_data.angular_velocity.z) + "," + to_string(djiimu_data.linear_acceleration.x) + "," +  to_string(djiimu_data.linear_acceleration.y) + "," +  to_string(djiimu_data.linear_acceleration.z) + "," + to_string(djirc_data.header.seq) + "," + to_string(djirc_data.header.stamp.sec) + "." + to_string(djirc_data.header.stamp.nsec * 10^-8) + "," + to_string(djirc_data.axes[0]) + "," + to_string(djirc_data.axes[1]) + "," + to_string(djirc_data.axes[2]) + "," + to_string(djirc_data.axes[3]);

	//Metadata
	if (start_flag == 0) {
		printf("start = %d \n", start);
		tm *ltm = localtime(&start);
		int year = 1900 + ltm->tm_year;
		int month = 1+ltm->tm_mon;
		int day = ltm->tm_mday;
		metadata_string = dotfile + "," + std::to_string(year) + std::to_string(month) + std::to_string(day) + ",";
		start_flag = 1;
	}
	else {
		metadata_string = " , ,";
	}
	time_t now = micros();
	time_t time_since_start = now-start_us;
	metadata_string = metadata_string + std::to_string(time_since_start);

	//Collect and print
	string data_string = metadata_string + "," + vicon_string + "," + seye_string + "," + accel1_string + "," + accel2_string + "," + dji_string + "\n";
    	csv_write(data_string);
}


//------------------------------------Callback functions--------------------------------

void viconCallback(geometry_msgs::TransformStamped msg)
{
	vicon_data = msg;
}

void seyeCallback(serial_interface::Razorimu msg)
{
	seye_data = msg;
}

void xsensCallback(xsens_mti_driver::nine_dof_imu msg)
{
	xsens_data = msg;
}

void djiimuCallback(sensor_msgs::Imu msg)
{
	djiimu_data = msg;
}

void djircCallback(sensor_msgs::Joy msg)
{
	djirc_data = msg;
}




//----------------------------- Main ----------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "sensorlogger");
  	ros::NodeHandle n;
	ros::Subscriber sub1 = n.subscribe("/vicon/test_obj/test_obj", 1000, viconCallback);
	ros::Subscriber sub2 = n.subscribe("/Razor_IMU/SafeEye", 1000, seyeCallback);
	ros::Subscriber sub3 = n.subscribe("/imu/raw_data", 1000, xsensCallback);
	ros::Subscriber sub4 = n.subscribe("/dji_sdk/imu", 1000, djiimuCallback);
	ros::Subscriber sub5 = n.subscribe("/dji_sdk/rc", 1000, djircCallback);
	ros::Rate loop_rate(800);	

  	//Open I²C bus
  	open_bus();

  	//Start the accelerometer and set offsets
  	setup(ADXL375_DEVICE1,-1,2,1);
  	setup(ADXL375_DEVICE2,0,-2,-1); 

 	setup_csv();

	adxl375_rosinterface::Accel accel1_data;
	adxl375_rosinterface::Accel accel2_data;

	djirc_data.axes = {0,0,0,0,0,0,0,0,0,0,0,0};

  	while (ros::ok())
  	{
		read_axes(ADXL375_DEVICE1, &accel1_data.x, &accel1_data.y, &accel1_data.z);
		accel1_data.stamp = ros::Time::now();

    		read_axes(ADXL375_DEVICE2, &accel2_data.x, &accel2_data.y, &accel2_data.z);
    		accel2_data.stamp = ros::Time::now();

    		csv_updater(accel1_data,accel2_data);
		//printf("axes: %ld \n", djirc_data.axes);
		//float tmp = djirc_data.axes[1];
		//printf("tmp: %f \n", tmp);

    		ros::spinOnce();

    		loop_rate.sleep();
  	}
  	return 0;
}

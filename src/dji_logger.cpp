#include "ros/ros.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <ctime>
#include <chrono>
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Joy.h"

using namespace std;

//--------------------------------- Global Variables ---------------------------------------------------------------
sensor_msgs::Imu djiimu_data;
sensor_msgs::Joy djirc_data;
float time_since_start;

int csv_file;
std::string dotfile;

//--------------------------------- Time functions ---------------------------------------------------------------
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

//------------------------------ Write to CSV functions ---------------------------------------------------------
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

    string header_string, metadata, DJI;

    metadata = "name, date, time_since_start";
    DJI = "dji_imu_sequence, dji_imu_time_stamp, dji_imu_orientation_x, dji_imu_orientation_y, dji_imu_orientation_z, dji_imu_orientation_w, dji_imu_ang_vel_x, dji_imu_ang_vel_y, dji_imu_ang_z, dji_imu_acc_x, dji_imu_acc_y, dji_imu_acc_z, rc_sequence, rc_time_stamp, rc_roll, rc_pitch, rc_yaw, rc_throttle";

    header_string = metadata + "," + DJI + "\n";

    csv_write(header_string);
return;
}



void csv_updater()  {
	static int start_flag = 0;
	static time_t start = time(0);
	string metadata_string, dji_string;

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
	metadata_string = metadata_string + to_string(time_since_start);

	//Collect and print
	string data_string = metadata_string + "," + dji_string + "\n";
    	csv_write(data_string);
return;
}


//------------------------------------Callback functions--------------------------------

void djiimuCallback(sensor_msgs::Imu msg)
{
	djiimu_data = msg;
return;
}

void djircCallback(sensor_msgs::Joy msg)
{
	djirc_data = msg;
return;
}

void clockCallback(std_msgs::Float32 msg)
{
	time_since_start = msg.data;
return;
}


//----------------------------- Main ----------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "djilogger");
  	ros::NodeHandle n;
	ros::Subscriber sub1 = n.subscribe("/dji_sdk/imu", 1000, djiimuCallback);
	ros::Subscriber sub2 = n.subscribe("/dji_sdk/rc", 1000, djircCallback);
	ros::Subscriber sub3 = n.subscribe("/sensor_clock", 1000, clockCallback);
	ros::Rate loop_rate(50);

	djirc_data.axes = {0,0,0,0,0,0,0,0,0,0,0,0};	

	//Setup csv header
 	setup_csv();

  	while (ros::ok())
  	{
    		loop_rate.sleep();

    		csv_updater();
		
		ros::spinOnce();
  	}
  	return 0;
}

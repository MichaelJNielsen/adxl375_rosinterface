#include "ros/ros.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <ctime>
#include <chrono>

#include <fcntl.h>
#include <sys/ioctl.h>

#include "std_msgs/Float32.h"
#include "geometry_msgs/TransformStamped.h"
#include "serial_interface/Razorimu.h"

using namespace std;

//--------------------------------- Global Variables ---------------------------------------------------------------
geometry_msgs::TransformStamped vicon_data;
serial_interface::Razorimu seye_data;
float time_since_start;

int csv_file;
string dotfile;


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

    string header_string, metadata, vicon, SafeEye;

    metadata = "name, date, time_since_start";
    vicon =  "vicon_sequence, vicon_time_stamp, vicon_translation_x, vicon_translation_y, vicon_translation_z, vicon_rotation_x, vicon_rotation_y, vicon_rotation_z, vicon_rotation_w";
    SafeEye = "seye_razor_time_stamp, seye_razor_acc_x, seye_razor_acc_y, seye_razor_acc_z, seye_razor_gyro_x, seye_razor_gyro_y, seye_razor_gyro_z, seye_razor_mag_x, seye_razor_mag_y, seye_razor_mag_z";

    header_string = metadata + "," + vicon + "," + SafeEye + "\n";

    csv_write(header_string);
}



void csv_updater()  {
	static int start_flag = 0;
	time_t start = time(0);
	string metadata_string, vicon_string, seye_string;

	//vicon
	vicon_string = to_string(vicon_data.header.seq) + "," + to_string(vicon_data.header.stamp.sec) + "." + to_string(vicon_data.header.stamp.nsec*10^-8) + "," + to_string(vicon_data.transform.translation.x) + "," + to_string(vicon_data.transform.translation.y) + "," + to_string(vicon_data.transform.translation.z) + "," + to_string(vicon_data.transform.rotation.x) + "," + to_string(vicon_data.transform.rotation.y) + "," + to_string(vicon_data.transform.rotation.z) + "," + to_string(vicon_data.transform.rotation.w);

	//seye
	seye_string = to_string(seye_data.time_stamp) + "," + to_string(seye_data.acc_x) + "," + to_string(seye_data.acc_y) + "," + to_string(seye_data.acc_z) + "," + to_string(seye_data.gyro_x) + "," + to_string(seye_data.gyro_y) + "," + to_string(seye_data.gyro_z) + "," + to_string(seye_data.mag_x) + "," + to_string(seye_data.mag_y) + "," + to_string(seye_data.mag_z);


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
	metadata_string = metadata_string + std::to_string(time_since_start);

	//Collect and print
	string data_string = metadata_string + "," + vicon_string + "," + seye_string + "\n";
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

void clockCallback(std_msgs::Float32 msg)
{
	time_since_start = msg.data;
}



//----------------------------- Main ----------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "razorviconlogger");
  	ros::NodeHandle n;
	//ros::Subscriber sub1 = n.subscribe("/vicon/test_obj/test_obj", 1000, viconCallback);
	ros::Subscriber sub2 = n.subscribe("/Razor_IMU/SafeEye", 1000, seyeCallback);
	ros::Subscriber sub3 = n.subscribe("/sensor_clock", 1000, clockCallback);
	ros::Rate loop_rate(100);	

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

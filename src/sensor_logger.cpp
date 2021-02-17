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
#include <thread>

//Libraries for xsens
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xstime.h>
#include <xscommon/xsens_mutex.h>
#include <iomanip>
#include <list>

using namespace std;

//--------------------------------- Global Variables ---------------------------------------------------------------
geometry_msgs::TransformStamped vicon_data;
serial_interface::Razorimu seye_data;
sensor_msgs::Imu djiimu_data;
sensor_msgs::Joy djirc_data;
adxl375_rosinterface::Accel accel1_data;
adxl375_rosinterface::Accel accel2_data;
xsens_mti_driver::nine_dof_imu xsens_data;
string data_string;

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

Journaller* gJournal = 0;	//Used by xsens


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


//---------------------------------Xsens reader functions------------------------------------------------------------

class CallbackHandler : public XsCallback
{
public:
	CallbackHandler(size_t maxBufferSize = 5)
		: m_maxNumberOfPacketsInBuffer(maxBufferSize)
		, m_numberOfPacketsInBuffer(0)
	{
	}

	virtual ~CallbackHandler() throw()
	{
	}

	bool packetAvailable() const
	{
		xsens::Lock locky(&m_mutex);
		return m_numberOfPacketsInBuffer > 0;
	}

	XsDataPacket getNextPacket()
	{
		assert(packetAvailable());
		xsens::Lock locky(&m_mutex);
		XsDataPacket oldestPacket(m_packetBuffer.front());
		m_packetBuffer.pop_front();
		--m_numberOfPacketsInBuffer;
		return oldestPacket;
	}

protected:
	void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override
	{
		xsens::Lock locky(&m_mutex);
		assert(packet != 0);
		while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
			(void)getNextPacket();

		m_packetBuffer.push_back(*packet);
		++m_numberOfPacketsInBuffer;
		assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer);
	}
private:
	mutable xsens::Mutex m_mutex;

	size_t m_maxNumberOfPacketsInBuffer;
	size_t m_numberOfPacketsInBuffer;
	list<XsDataPacket> m_packetBuffer;
};

CallbackHandler callback;

void xsens_setup()
{
	cout << "Creating XsControl object..." << endl;
	XsControl* control = XsControl::construct();
	assert(control != 0);

	// Lambda function for error handling
	auto handleError = [=](string errorString)
	{
		control->destruct();
		cout << errorString << endl;
		cout << "Press [ENTER] to continue." << endl;
		cin.get();
		exit(0);
	};

	cout << "Scanning for devices..." << endl;
	XsPortInfoArray portInfoArray = XsScanner::scanPorts();
	// Find an MTi device
	XsPortInfo mtPort;
	for (auto const &portInfo : portInfoArray)
	{
		if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
		{
			mtPort = portInfo;
			break;
		}
	}
	if (mtPort.empty())
	{
		printf("No MTi device found. Aborting.");
		exit(0);
	}
	cout << "Found a device with ID: " << mtPort.deviceId().toString().toStdString() << " @ port: " << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << endl;
	cout << "Opening port..." << endl;
	if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate()))
	{
		printf("Could not open port. Aborting.");
		exit(0);
	}
	// Get the device object
	XsDevice* device = control->device(mtPort.deviceId());
	assert(device != 0);

	cout << "Device: " << device->productCode().toStdString() << ", with ID: " << device->deviceId().toString() << " opened." << endl;
		
	// Create and attach callback handler to device
	//CallbackHandler callback;	//Made this global...
	device->addCallbackHandler(&callback);

	cout << "Putting device into measurement mode..." << endl;
	if (!device->gotoMeasurement())
	{
		printf("Could not put device into measurement mode. Aborting.");
		exit(0);
	}

	cout << "Starting recording..." << endl;
	if (!device->startRecording())
	{
		printf("Failed to start recording. Aborting.");
		exit(0);
	}
	usleep(100000);
	printf("Xsens ready to record \n");
return;
}

void xsens_read()
{
	while (true)
	{
		int i = 0;
		time_t time1 = micros();
		while (i == 0) {
			if (callback.packetAvailable())
			{
				XsDataPacket packet = callback.getNextPacket();
				if (packet.containsRawData())
				{

					xsens_data.stamp = ros::Time::now();

					XsScrData data = packet.rawData();

            				xsens_data.acc_x = -1*(0.00238555*data.m_acc[0] - 0.000012349*data.m_acc[1] - 0.0000155809*data.m_acc[2] - 77.2247);
            				xsens_data.acc_y = 0.0000108991*data.m_acc[0] + 0.0023431*data.m_acc[1] - 0.0000108443*data.m_acc[2] - 76.7395;
            				xsens_data.acc_z = 0.00000732219*data.m_acc[1] - 0.000013511*data.m_acc[0] + 0.0023663*data.m_acc[2] - 77.3443;
            
           				xsens_data.gyro_x = 0.000323147*data.m_gyr[0] - 0.00000435472*data.m_gyr[1] - 6.53455e-7*data.m_gyr[2] - 10.4746;
            				xsens_data.gyro_y = -1*(0.000000997*data.m_gyr[0] + 0.000326813*data.m_gyr[1] - 0.00000520815*data.m_gyr[2] - 10.4636);
            				xsens_data.gyro_z = -1*(0.00000344737*data.m_gyr[0] + 0.00000462106*data.m_gyr[1] + 0.000326496*data.m_gyr[2] - 11.0188);
            
            				xsens_data.mag_x = 0.00235686*data.m_mag[0] + 0.000106221*data.m_mag[1] - 0.0000979062*data.m_mag[2] - 77.3397;
            				xsens_data.mag_y = 0.0022755*data.m_mag[1] - 0.0000719264*data.m_mag[0] + 0.0000885163*data.m_mag[2] - 75.1698;
            				xsens_data.mag_z = 0.000199498*data.m_mag[0] + 0.0000234849*data.m_mag[1] + 0.0018866*data.m_mag[2] - 68.9712;      
				}
				else {
					printf("Packet does not contain raw data \n");
					exit(0);
				}
			i++;
			}
		}
		time_t time2 = micros();
		time_t duration = time2-time1;
		while (duration < 1250)
		{
			time2 = micros();
			duration = time2-time1;
			nanosleep((const struct timespec[]){{0,1}}, NULL);
		}
	}
return;
}



//--------------------------------- I2C functions --------------------------------------------------------------------

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

void read_axis(int device_addr, int dev)
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

    	if (dev == 1) {
		int x_raw = int8_t(a[0]) | int8_t(a[1]) << 8;
		accel1_data.x = x_raw/20.5;
		int y_raw = int8_t(a[2]) | int8_t(a[3]) << 8;
		accel1_data.y = y_raw/20.5;
		int z_raw = int8_t(a[4]) | int8_t(a[5]) << 8;
		accel1_data.z = z_raw/20.5;

		accel1_data.stamp = ros::Time::now();
    	}
    	if (dev == 2) {
		int x_raw = int8_t(a[0]) | int8_t(a[1]) << 8;
		accel2_data.x = x_raw/20.5;
		int y_raw = int8_t(a[2]) | int8_t(a[3]) << 8;
		accel2_data.y = y_raw/20.5;
		int z_raw = int8_t(a[4]) | int8_t(a[5]) << 8;
		accel2_data.z = z_raw/20.5;

		accel2_data.stamp = ros::Time::now();
    	}
return;
}

void read_two_axes()
{

	while (true) 
	{
		time_t time1 = micros();

		read_axis(ADXL375_DEVICE1, 1);
		read_axis(ADXL375_DEVICE2, 2);

		time_t time2 = micros();
		time_t duration = time2-time1;

		while (duration < 1250)
		{
			time2 = micros();
			duration = time2-time1;
			nanosleep((const struct timespec[]){{0,1}}, NULL);
		}
	}
return;
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

    string header_string, metadata, vicon, SafeEye, Accel1, Accel2, DJI;

    metadata = "name, date, time_since_start";
    vicon =  "vicon_sequence, vicon_time_stamp, vicon_translation_x, vicon_translation_y, vicon_translation_z, vicon_rotation_x, vicon_rotation_y, vicon_rotation_z, vicon_rotation_w";
    SafeEye = "seye_razor_time_stamp, seye_razor_acc_x, seye_razor_acc_y, seye_razor_acc_z, seye_razor_gyro_x, seye_razor_gyro_y, seye_razor_gyro_z, seye_razor_mag_x, seye_razor_mag_y, seye_razor_mag_z";
    Accel1 = "accel1_xsens_time_stamp, accel1_xsens_acc_x,  accel1_xsens_acc_y, accel1_xsens_acc_z, accel1_xsens_gyro_x, accel1_xsens_gyro_y, accel1_xsens_gyro_z, accel1_xsens_mag_x, accel1_xsens_mag_y, accel1_xsens_mag_z, accel1_adxl_sequence, accel1_adxl_acc_x, accel1_adxl_acc_y, accel1_adxl_acc_z";
    Accel2 = "accel2_adxl_sequence, accel2_adxl_acc_x, accel2_adxl_acc_y, accel2_adxl_acc_z";
    DJI = "dji_imu_sequence, dji_imu_time_stamp, dji_imu_orientation_x, dji_imu_orientation_y, dji_imu_orientation_z, dji_imu_orientation_w, dji_imu_ang_vel_x, dji_imu_ang_vel_y, dji_imu_ang_z, dji_imu_acc_x, dji_imu_acc_y, dji_imu_acc_z, rc_sequence, rc_time_stamp, rc_roll, rc_pitch, rc_yaw, rc_throttle";

    header_string = metadata + "," + vicon + "," + SafeEye + "," + Accel1 + "," + Accel2 + ","  + DJI + "\n";

    csv_write(header_string);
return;
}



void csv_updater()  {
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
	data_string = metadata_string + "," + vicon_string + "," + seye_string + "," + accel1_string + "," + accel2_string + "," + dji_string + "\n";
    	csv_write(data_string);
return;
}

void csv_thread()
{
	while (true)
	{
		time_t time1 = micros();
		csv_updater();
		time_t time2 = micros();
		time_t duration = time2-time1;

		while (duration < 1250)
		{
			time2 = micros();
			duration = time2-time1;
			nanosleep((const struct timespec[]){{0,1}}, NULL);
		}
	}
	
}


//------------------------------------Callback functions--------------------------------

void viconCallback(geometry_msgs::TransformStamped msg)
{
	vicon_data = msg;
return;
}

void seyeCallback(serial_interface::Razorimu msg)
{
	seye_data = msg;
return;
}

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

void ros_spinner()
{
	while (true)
	{
		time_t time1 = micros();
		ros::spinOnce();
		time_t time2 = micros();
		time_t duration = time2-time1;

		while (duration < 5000)
		{
			time2 = micros();
			duration = time2-time1;
			nanosleep((const struct timespec[]){{0,1}}, NULL);
		}
	}
}



//----------------------------- Main ----------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "sensorlogger");
  	ros::NodeHandle n;
	ros::Subscriber sub1 = n.subscribe("/vicon/test_obj/test_obj", 1000, viconCallback);
	ros::Subscriber sub2 = n.subscribe("/Razor_IMU/SafeEye", 1000, seyeCallback);
	ros::Subscriber sub3 = n.subscribe("/dji_sdk/imu", 1000, djiimuCallback);
	ros::Subscriber sub4 = n.subscribe("/dji_sdk/rc", 1000, djircCallback);

  	//Open I²C bus
  	open_bus();

  	//Start the accelerometer and set offsets
  	setup(ADXL375_DEVICE1,-1,2,1);
  	setup(ADXL375_DEVICE2,0,-2,-1); 

	djirc_data.axes = {0,0,0,0,0,0,0,0,0,0,0,0};

	//Start xsens
	xsens_setup();

	//Setup csv header
 	setup_csv();

	thread t1(read_two_axes);
	thread t2(ros_spinner);
	thread t3(xsens_read);

  	while (ros::ok())
  	{
		time_t time1 = micros();

    		csv_updater();

		time_t time2 = micros();
		time_t duration = time2-time1;

		while (duration < 1250)
		{
			time2 = micros();
			duration = time2-time1;
		}
  	}
  	return 0;
}

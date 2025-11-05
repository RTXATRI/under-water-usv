/*
Author Mingxi Zhou, mzhou@uri.edu
This driver is created to convert between common ros messages into nmea0183
for serial communication.
The code was made for Digi RF Xbee pro 900Mhz module.
*/
#pragma once
extern "C" {
#include "roboticscape.h"
}
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int32.h"
#include <string.h>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <cstdio>
#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdlib>
#include <pthread.h>
#include "sensor_msgs/Imu.h"
#include "gps_common/GPSFix.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/TimeReference.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/BatteryState.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point32.h>

using namespace std;
/*#define DEFAULT_PORT "/dev/ttyUSB0"    // Default port name
#define DEFAULT_BAUD 115200              // Default baudate
#define DEFAULT_PARITY 'N'               // Default parity
#define DEFAULT_DATABITS 8               // Default databits
#define DEFAULT_STOPBITS 1               // Default stopbits
#define DEFAULT_TIMEOUT 1000             // Default timeout*/

#define FOO_MSG_ID 0
#define IMU_MSG_ID 1
#define GPS_MSG_ID 2
#define ODO_MSG_ID 3
#define MOT_MSG_ID 4
#define SYS_MSG_ID 5
#define CTR_MSG_ID 6
#define JOY_MSG_ID 7


//struct variables for diffferent msg id.
// please refer to readme.md for the msg id explaination
struct IMU_value
{
  double ang_vel[3];
  double lin_accel[3];
  double ori[4];
  double time;
  char frame[20];
  int num=12;
};

struct GPS_value
{
	char frame[20];
	double time;
	double latitude;
	double longitude;
	double sog;
	double cog;
	double roll;
	double pitch;
	double utc_time;
	int num = 9;
};

struct JOY_value
{
	double time;
	double ls1;
	double rs1;
	double rs2;
	int st; //start;
	int bk; //back
	int A;
	int B;
	int X;
	int Y;
	int num = 9;
};

struct CTR_value
{
	int mode;
	double stbd_dc;
	double port_dc;
	double c_heading;
	double c_wptx;
	double c_wpty;
	int num = 6;
};

struct SYS_value
{
	double time;
	float vm;
	float im;
	float vp;
	float ip;
	int num=5;
};

struct PLAN_value
{
	int ponitNO;
	geometry_msgs::Point32 newPoint;

};

class RF
{
	public:
		IMU_value imu_val;
		GPS_value gps_val;
		JOY_value joy_val;
		SYS_value sys_val;
		PLAN_value plan_val;
		char MSG_UPDATE_FLAG[10]={0}; 	//bit indicate the message update status
		char imu_data[256];	// /imu/data msg in char array
		char gps_data[256];
		char joy_data[256];
		char sys_data[256];
		char path_data[256];
		geometry_msgs::PolygonStamped pathplan;
		RF(ros::NodeHandle nh);
		bool calcChecksum(int &crc, char cdata[256]);
		//imu to nmea
		void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
		//GPS to nmea
		void fix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
		void vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
		void time_callback(const sensor_msgs::TimeReference::ConstPtr& msg);
		void rpy_callback(const geometry_msgs::Vector3::ConstPtr& msg);
		//joy to nmea
		void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);
		//VI messages to nmea0183
		void vim_callback(const sensor_msgs::BatteryState::ConstPtr& msg);
		void vip_callback(const sensor_msgs::BatteryState::ConstPtr& msg);

		int parse_joy(char data[256]);
		int parse_path(char data[256]);
		int parse_pathPub(char data[256]);


	private:

		//imu receiver
		ros::Subscriber imu_sub;

		//GPS receiver
		ros::Subscriber fix_sub;
		ros::Subscriber vel_sub;
		ros::Subscriber rpy_sub;
		ros::Subscriber time_sub;
		//joy receiver
		ros::Subscriber joy_sub;
		//sys receiver
		ros::Subscriber vim_sub;
		ros::Subscriber vip_sub;

		ros::Publisher path_pub;
		ros::Publisher joy_pub;


};

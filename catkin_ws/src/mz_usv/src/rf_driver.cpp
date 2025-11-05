#include<rf/rf_driver.h>

RF::RF(ros::NodeHandle nh)
{
	ROS_INFO("R2N IMU");
	imu_sub = nh.subscribe("/sensor/m_imu",2, &RF::imu_callback,this);

	ROS_INFO("R2N GPS");
	fix_sub = nh.subscribe("/sensor/m_fix",2, &RF::fix_callback,this);
	vel_sub = nh.subscribe("/sensor/m_vel",2, &RF::vel_callback,this);
	time_sub = nh.subscribe("/sensor/m_time_reference",2, &RF::time_callback,this);
	rpy_sub = nh.subscribe("/sensor/imu/rpy",2, &RF::rpy_callback,this);

	ROS_INFO("R2N JOY");
	joy_sub = nh.subscribe("/joy",2,&RF::joy_callback,this);

	ROS_INFO("R2N SYS");
	vim_sub = nh.subscribe("/sensor/m_VI",2,&RF::vim_callback,this);
	vip_sub = nh.subscribe("/sensor/m_VI_payload",2,&RF::vip_callback,this);

	sprintf(imu_data,"$IMU,...\r\n");
	sprintf(gps_data,"$GPS,...\r\n");
	sprintf(joy_data,"$JOY,...\r\n");
	sprintf(sys_data,"$SYS,...\r\n");

	joy_pub = nh.advertise<sensor_msgs::Joy>("/joy",2);
	path_pub=nh.advertise<geometry_msgs::PolygonStamped>("/mapviz/rawplan",2);
}

//imu callback
void RF::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	char o_data[256];
	int cs;
	int num;
	strcpy(imu_val.frame,msg->header.frame_id.c_str());
	imu_val.time = msg->header.stamp.toSec();
	imu_val.ori[0]=msg->orientation.x;
	imu_val.ori[1]=msg->orientation.y;
	imu_val.ori[2]=msg->orientation.z;
	imu_val.ori[3]=msg->orientation.w;
	imu_val.ang_vel[0]=msg->angular_velocity.x;
	imu_val.ang_vel[1]=msg->angular_velocity.y;
	imu_val.ang_vel[2]=msg->angular_velocity.z;
	imu_val.lin_accel[0]=msg->linear_acceleration.x;
	imu_val.lin_accel[1]=msg->linear_acceleration.y;
	imu_val.lin_accel[2]=msg->linear_acceleration.z;
	//01| $IMU,frame,time,ori.x,ori.y,ori.z,orientation.w,ang_v.x,ang_v.y,ang_v.z,l_v.x,l_v.y,l_v.z*cs\r\n
	num = sprintf(o_data,"$IMU,%s,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf*00\r\n",
																			imu_val.frame, imu_val.time,
																			imu_val.ori[0],imu_val.ori[1],imu_val.ori[2],imu_val.ori[3],
																			imu_val.ang_vel[0],imu_val.ang_vel[1],imu_val.ang_vel[2],
																			imu_val.lin_accel[0],imu_val.lin_accel[1],imu_val.lin_accel[2]);
	std::string o_s=o_data;
	//	ROS_INFO("%s",o_data);
	calcChecksum(cs,o_data);
	//re-do the sprintf to imu_data
	num = sprintf(imu_data,"$IMU,%s,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf*%02X\r\n",
																	imu_val.frame, imu_val.time,
																	imu_val.ori[0],imu_val.ori[1],imu_val.ori[2],imu_val.ori[3],
																	imu_val.ang_vel[0],imu_val.ang_vel[1],imu_val.ang_vel[2],
																	imu_val.lin_accel[0],imu_val.lin_accel[1],imu_val.lin_accel[2],cs);
	//ser->write(o_data);
}

//GPS callback
void RF::fix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	strcpy(gps_val.frame,msg->header.frame_id.c_str());
	gps_val.latitude=msg->latitude;
	gps_val.longitude=msg->longitude;
	gps_val.time = msg->header.stamp.toSec();
}
void RF::vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	gps_val.sog = sqrt(pow(msg->twist.linear.x,2) + pow(msg->twist.linear.y,2) ); //m/s
	gps_val.cog = atan2(msg->twist.linear.x, msg->twist.linear.y)*180/3.1415926; //deg
}

void RF::rpy_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	gps_val.roll=msg->x;
	gps_val.pitch=msg->y;
}

void RF::time_callback(const sensor_msgs::TimeReference::ConstPtr& msg)
{
	char o_data[256];
	int num;
	int cs;
	gps_val.utc_time=msg->time_ref.toSec();
	num=sprintf(o_data,"$GPS,%s,%.3lf,%.5lf,%.5lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf*00\r\n",
														 gps_val.frame,gps_val.time,gps_val.latitude,gps_val.longitude,
													   gps_val.cog,gps_val.sog,
														 gps_val.roll,gps_val.pitch,gps_val.utc_time);
	calcChecksum(cs,o_data);
	num=sprintf(gps_data,"$GPS,%s,%.3lf,%.5lf,%.5lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf*%02X\r\n",
														 gps_val.frame,gps_val.time,gps_val.latitude,gps_val.longitude,
													   gps_val.cog,gps_val.sog,
														 gps_val.roll,gps_val.pitch,gps_val.utc_time,cs);
  //MSG_UPDATE_FLAG[GPS_MSG_ID] = 1; //SET THE FLAG
}

//joy callback
void RF::joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
	char o_data[256];
		int num;
		int cs;
		joy_val.time = msg->header.stamp.toSec();
		joy_val.ls1   = msg->axes[1];
	//	joy_val.ls2   = msg->axes[2];

		joy_val.rs1   = msg->axes[3];
		joy_val.rs2   = msg->axes[2];
		joy_val.A    = msg->buttons[1];
		joy_val.B		 = msg->buttons[2];
		joy_val.X    = msg->buttons[0];
		joy_val.Y		 = msg->buttons[4];
		joy_val.st   = msg->buttons[9];
		joy_val.bk	 = msg->buttons[8];
		num=sprintf(o_data,"$JOY,%.3lf,%.3lf,%.3lf,%.3lf,%d,%d,%d,%d,%d,%d*00\r\n",
														joy_val.time, joy_val.ls1, joy_val.rs1,joy_val.rs2,
														joy_val.st, joy_val.bk, joy_val.A, joy_val.B, joy_val.X, joy_val.Y);
		calcChecksum(cs,o_data);
		num=sprintf(joy_data,"$JOY,%.3lf,%.3lf,%.3lf,%.3lf,%d,%d,%d,%d,%d,%d*%02X\r\n",
														joy_val.time, joy_val.ls1, joy_val.rs1,joy_val.rs2,
														joy_val.st, joy_val.bk, joy_val.A, joy_val.B, joy_val.X, joy_val.Y,cs);
		//MSG_UPDATE_FLAG[JOY_MSG_ID]= 1;
}

//VI sensor callbacks
void RF::vim_callback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
	cout<<"vim"<<endl;
	sys_val.time = msg->header.stamp.toSec();
	sys_val.vm   = msg->voltage;
	sys_val.im = msg->current;
	sprintf(sys_data,"$SYS,%.3lf,%.1lf,%.1lf\r\n",sys_val.time, sys_val.vm, sys_val.im);
}
void RF::vip_callback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
	char o_data[256];
	int num;
	int cs;

	sys_val.time = msg->header.stamp.toSec();
	sys_val.vp   = msg->voltage;
	sys_val.ip = msg->current;
	num=sprintf(o_data,"$SYS,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf*00\r\n",
													sys_val.time, sys_val.vm, sys_val.im, sys_val.vp, sys_val.ip);
	calcChecksum(cs,o_data);
	num=sprintf(sys_data,"$SYS,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf*%02X\r\n",
													sys_val.time, sys_val.vm, sys_val.im, sys_val.vp, sys_val.ip,cs);
	//MSG_UPDATE_FLAG[SYS_MSG_ID]= 1;
}

int RF::parse_joy(char data[256])
{
	strcpy(joy_data,data);
	JOY_value rf_val;
	sensor_msgs::Joy pub_data;
	pub_data.axes.resize(5);
	pub_data.buttons.resize(11);
	int cs=0;
	int num = sscanf(data,"$JOY,%lf,%lf,%lf,%lf,%d,%d,%d,%d,%d,%d*",&rf_val.time, &rf_val.ls1, &rf_val.rs1, &rf_val.rs2,
																	&rf_val.st, &rf_val.bk,
																	&rf_val.A, &rf_val.B, &rf_val.X, &rf_val.Y);

	  //calcChecksum(cs,data);
	//	if(num == rf_val.num & calcChecksum(cs,data))
	//	{
			pub_data.header.stamp=ros::Time(rf_val.time);
			//pub_data.axes[0]=0;
			pub_data.axes[1]=rf_val.ls1;
			pub_data.axes[2]=rf_val.rs2;
			pub_data.axes[3]=rf_val.rs1;
			//pub_data.axes[4]=0;
			//pub_data.axes[5]=0;
			pub_data.buttons[0]=rf_val.X;
			pub_data.buttons[1]=rf_val.A;
			pub_data.buttons[2]=rf_val.B;
			//pub_data.buttons[3]=0;
			pub_data.buttons[3]=rf_val.Y;
			//pub_data.buttons[5]=0;
			//pub_data.buttons[6]=0;
			//pub_data.buttons[7]=0;
			pub_data.buttons[8]=rf_val.bk;
			pub_data.buttons[9]=rf_val.st;
			//pub_data.buttons[10]=0;
			//pub_data.buttons[11]=0;
			joy_pub.publish(pub_data);
			return 1;
//	}
//	return 0;
}
int RF::parse_path(char data[256])
{
	cout<<data<<endl;
	sscanf(data,"$PLAN,%d,%f,%f,%f*", &plan_val.ponitNO, &plan_val.newPoint.x, &plan_val.newPoint.y, &plan_val.newPoint.z);
	cout<<plan_val.newPoint.x<<endl;
	cout<<plan_val.newPoint.y<<endl;
	cout<<plan_val.newPoint.z<<endl;
	pathplan.polygon.points.push_back(plan_val.newPoint);
	if(plan_val.ponitNO==99)
	{
		path_pub.publish(pathplan);
		pathplan.polygon.points.clear();
	}


}
int RF::parse_pathPub(char data[256])
{
	cout<<data<<endl;
	sscanf(data,"$EndPLAN,%d,%f,%f,%f*", &plan_val.ponitNO, &plan_val.newPoint.x, &plan_val.newPoint.y, &plan_val.newPoint.z);
	pathplan.polygon.points.push_back(plan_val.newPoint);
	cout<<pathplan.polygon.points.size()<<endl;
	cout<<plan_val.newPoint.x<<endl;
	cout<<plan_val.newPoint.y<<endl;
	cout<<plan_val.newPoint.z<<endl;
	path_pub.publish(pathplan);
	cout<<pathplan.polygon.points[2].x<<endl;
	pathplan.polygon.points.clear();
	cout<<pathplan.polygon.points.size()<<endl;
}
////////////////checksum calculation//////////////////////////
bool RF::calcChecksum(int &crc, char cdata[256])	//return check flag and return actual crc in the bracket
{
    	//NMNA0183
		std::string ss = cdata;
    	int sz = ss.size();
		//	ROS_INFO("%s,%d,%c",cdata,sz,cdata[sz-4]);
    	int i;
			//for (i = 0; i < sz; i ++)
			//{
				//printf("%c=%d,",cdata[i],cdata[i]);
			//}
		crc=0;
    	//$*00<CR><LF>
		//	if(!checkIntegrity(ss))
		//	{
			// return 0;
	 		//}
    	for (i = 1; i < sz - 5; i ++)
			{
				crc ^= cdata[i];
    	}

		//	ROS_INFO("end");
		//	int checksum = std::stoi(cs,0,16);
		int checksum =stoi(ss.substr(sz-4, 2), nullptr, 16);
  		return crc == checksum;

}

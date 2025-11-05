/*
################################################################################
Sep-09-2020, Jianguang, add the new function of docking. subscribe cv_data, get the computer vision and control the usv.
*/
//include C language libraries
//extern "C" {
//#include "rc_usefulincludes.h"
//}
extern "C" {
#include "roboticscape.h"
}

#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h>  // for atoi() and exit()
//#include <rc/mpu.h>
//#include <rc/time.h>
#include <sstream>
#include "sensor_msgs/Imu.h"
#include "ros/ros.h"
//include ros message types
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64.h"
#include <tf/tf.h>
#define CENTER 512  

class Docking
{
	std_msgs::Int32MultiArray cv_data;  //heading data
	std_msgs::Float64 stdb_pwm, port_pwm;
	float K_p, K_i,K_d,K_g,total_pwm, delta_pwm,error,d_error,i_error;
	int rarea=500000,barea=0,rcenterX,bcenterX,centerX;
	int circle=0;
	int turning_wait=0;
	double ct, dt;
	float hz=2;
	public:
	Docking()
	{
		stdb_thrust_pub = nh.advertise<std_msgs::Float64>("/gnc/stdb_cmd", 2);
		port_thrust_pub = nh.advertise<std_msgs::Float64>("/gnc/port_cmd", 2);
	  	cv_sub = nh.subscribe("/cv_data",2,&Docking::cv_callback,this);
		imu_sub = nh.subscribe("/sensor/imu/data",2,&Docking::imu_callback,this);;
		//parameters
		//default
		K_p=1;
		K_i=0;
		K_d=0;
		K_g=0;
		i_error = 0;
		total_pwm=1;	//max 2
		///
		ros::NodeHandle nh_priv("~");
		nh_priv.getParam("K_g",K_g);
		nh_priv.getParam("K_p",K_p);
		nh_priv.getParam("K_i",K_i);
		nh_priv.getParam("K_d",K_d);
		nh_priv.getParam("Total_thrust", total_pwm);

		ROS_INFO("Node started");
		ct = ros::Time::now().toSec();
	}

	void cv_callback(const std_msgs::Int32MultiArray::ConstPtr& msg)
	{
		rarea=msg->data[0];
		barea=msg->data[1];
		rcenterX=msg->data[2];
		bcenterX=msg->data[3];
		circle=0;
	}

	void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
	{
		d_error = -msg->angular_velocity.z*RAD_TO_DEG;// added minus sign becaues error = c_heading -m_heading
	}
	
	void running()
	{
		//ros::Rate loop_rate(hz);
		while(ros::ok())
		{
			circle++;
			dt = ros::Time::now().toSec()-ct; //compute delta t
			ct = ros::Time::now().toSec();//update time
			if((rarea>=100&&rarea<=30000&&  barea<=100)||( barea>100 &&rarea>=5000 &&rarea<=30000))//追踪
			{
				//printf("Docking: %d\n",rcenterX);
				pid_centerxrack();
			}
			else if(rarea<100)//慢速前进
			{
					stdb_pwm.data = 0.3;
					port_pwm.data = 0.3;	
					stdb_thrust_pub.publish(stdb_pwm);
					port_thrust_pub.publish(port_pwm);
			}
			else if(rarea>100&&  barea>100 &&rarea<5000){//调整前进
				turning_wait++;
				if (turning_wait>4){
					turning_wait=0;
					if(bcenterX-rcenterX>20)
					{
						stdb_pwm.data = 0.6;
						port_pwm.data = -0.6;
						stdb_thrust_pub.publish(stdb_pwm);
						port_thrust_pub.publish(port_pwm);
						sleep(2);
						stdb_pwm.data = 0.6;
						port_pwm.data = 0.6;
						stdb_thrust_pub.publish(stdb_pwm);
						port_thrust_pub.publish(port_pwm);
						usleep(4500000);
						stdb_pwm.data = -0.6;
						port_pwm.data = 0.6;
						stdb_thrust_pub.publish(stdb_pwm);
						port_thrust_pub.publish(port_pwm);
						usleep(2600000);
					//ioctl(fd_pwm,IOCTL_CONFIG_PWM2, 925);  //左推进器
					//ioctl(fd_pwm,IOCTL_CONFIG_PWM3, 925);  //右推进器
					}
					else if(bcenterX-rcenterX<-20)
					{
						stdb_pwm.data = -0.6;
						port_pwm.data = 0.6;
						stdb_thrust_pub.publish(stdb_pwm);
						port_thrust_pub.publish(port_pwm);
						sleep(2);
						stdb_pwm.data = 0.6;
						port_pwm.data = 0.6;
						stdb_thrust_pub.publish(stdb_pwm);
						port_thrust_pub.publish(port_pwm);
						usleep(4500000);
						stdb_pwm.data = 0.6;
						port_pwm.data = -0.6;
						stdb_thrust_pub.publish(stdb_pwm);
						port_thrust_pub.publish(port_pwm);
						usleep(2600000);
					//ioctl(fd_pwm,IOCTL_CONFIG_PWM2, 925);  //左推进器
					//ioctl(fd_pwm,IOCTL_CONFIG_PWM3, 925);  //右推进器
					}
					else{
						stdb_pwm.data = 0.3;
						port_pwm.data = 0.3;
						stdb_thrust_pub.publish(stdb_pwm);
						port_thrust_pub.publish(port_pwm);
						printf("same center\n");
					}
				}
				else{
						stdb_pwm.data = 0.3;
						port_pwm.data = 0.3;
						stdb_thrust_pub.publish(stdb_pwm);
						port_thrust_pub.publish(port_pwm);
						printf("same center\n");
					}
			}
			else if(rarea>30000) //停止
			{
					stdb_pwm.data = 0;
					port_pwm.data = 0;	
					stdb_thrust_pub.publish(stdb_pwm);
					port_thrust_pub.publish(port_pwm);
			}
			ros::spinOnce();
			//loop_rate.sleep();
		}
	}

	void pid_centerxrack(){
			error= rcenterX-CENTER;
			i_error= i_error+error*dt;
			delta_pwm = K_g*(K_p*error + K_i*i_error + K_d*d_error);
			//ROS_INFO("%f|%f|%f",delta_pwm,dt,ct);
			stdb_pwm.data = total_pwm/2.0 - delta_pwm/2.0;
			port_pwm.data = total_pwm - stdb_pwm.data;	
			if(stdb_pwm.data>1) stdb_pwm.data=1;
			if(stdb_pwm.data<0) stdb_pwm.data=0;
			if(port_pwm.data>1) port_pwm.data=1;
			if(port_pwm.data<0) port_pwm.data=0;
			if(circle>2000){
				stdb_pwm.data=0;
				port_pwm.data=0;
			}
			stdb_thrust_pub.publish(stdb_pwm);
			port_thrust_pub.publish(port_pwm);
}
/******PID增量生成函数*********/ 




	private:
		ros::NodeHandle nh;
		ros::Publisher stdb_thrust_pub;
		ros::Publisher port_thrust_pub;
		ros::Subscriber imu_sub;
		ros::Subscriber cv_sub;		
};


//global variables



int main(int argc, char** argv)
{
	// initialize a ros node (name used in ros context)
	ros::init(argc, argv, "Docking");
	Docking docking;
	docking.running();

  	return 0;
}

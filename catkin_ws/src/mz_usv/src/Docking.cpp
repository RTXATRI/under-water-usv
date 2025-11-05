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
#define CENTER 480  
#define AdSpeed 0.6
//#define Low 400
class Docking
{
	std_msgs::Int32MultiArray cv_data;  //heading data
	std_msgs::Float64 stdb_pwm, port_pwm;
	float K_p, K_i,K_d,K_g,total_pwm, delta_pwm,error,d_error,i_error;
	int rarea=0,barea=0,rcenterX,bcenterX,centerX,rbrate=12,th_low=200,th_middle=4000,th_high=30000,th_rb=10;
	int sleeptime1=2000000,sleeptime2=3000000,sleeptime3=3000000;
	float k_ad=1.0;
	int circle=0,count=0;
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
		nh_priv.getParam("sleeptime1",sleeptime1);
		nh_priv.getParam("sleeptime2",sleeptime2);
		nh_priv.getParam("sleeptime3",sleeptime3);
		nh_priv.getParam("th_low",th_low);
		nh_priv.getParam("th_middle",th_middle);
		nh_priv.getParam("th_high",th_high);
		nh_priv.getParam("th_rb",th_rb);
		nh_priv.getParam("Total_thrust", total_pwm);

		ROS_INFO("Node started");
		ct = ros::Time::now().toSec();
	}

	void cv_callback(const std_msgs::Int32MultiArray::ConstPtr& msg)
	{
		rarea=msg->data[0];
		barea=msg->data[1];
		//ROS_INFO("calculating rbrate");
		if(barea>10){
		rbrate=rarea/barea;
		}
		//ROS_INFO("rbrate:::%d",rbrate);
		if(rarea*rbrate>3000)
		{
			k_ad=2;
		}
		if(rarea*rbrate>=1000&&rarea*rbrate<=3000)
		{
			k_ad=1.5;
		}
		if(rarea*rbrate<1000)
		{
			k_ad=1;
		}
	       // ROS_INFO("red:%d,,,,blue:%d",rarea,barea);
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
			if((rarea>=th_low&&rarea<th_middle&&  rbrate>=th_rb)||( rarea>=th_middle &&rarea<=th_high))//追踪:红色蓝色都比较小 或者红色比较大
			{
				//printf("Docking: %d\n",rcenterX);
				ROS_INFO("tracking");
				                ROS_INFO("red:%d,,,,blue:%d",rarea,barea);

			//	ROS_INFO("count:::%d",count);
				pid_centerxrack();
				

			}
			else if(rarea<th_low)//慢速旋转
			{
					stdb_pwm.data = 0.1;
					port_pwm.data = -0.1;	
					stdb_thrust_pub.publish(stdb_pwm);
					port_thrust_pub.publish(port_pwm);
					                ROS_INFO("Looking for target");
			}
			else if(rarea>th_low&&  rbrate<th_rb &&rarea<th_middle&&barea>10){//调整前进：红色比较小，蓝色比较大
			//	count++;
			//	if (count>0){
					count=0;
					if(bcenterX-rcenterX>20)
					{
					    ROS_INFO("adjusting");
					                    ROS_INFO("red:%d,,,,blue:%d",rarea,barea);

						stdb_pwm.data = AdSpeed/k_ad;
						port_pwm.data = -AdSpeed/k_ad;
						stdb_thrust_pub.publish(stdb_pwm);
						port_thrust_pub.publish(port_pwm);
						usleep(sleeptime1);
						stdb_pwm.data = AdSpeed/k_ad;
						port_pwm.data = AdSpeed/k_ad;
						stdb_thrust_pub.publish(stdb_pwm);
						port_thrust_pub.publish(port_pwm);
						usleep(sleeptime2);
						stdb_pwm.data = -AdSpeed/k_ad;
						port_pwm.data = AdSpeed/k_ad;
						stdb_thrust_pub.publish(stdb_pwm);
						port_thrust_pub.publish(port_pwm);
						usleep(sleeptime3);
					//ioctl(fd_pwm,IOCTL_CONFIG_PWM2, 925);  //左推进器
					//ioctl(fd_pwm,IOCTL_CONFIG_PWM3, 925);  //右推进器
					}
					else if(bcenterX-rcenterX<-20)
					{
						                ROS_INFO("adjusting to to right");

		                ROS_INFO("red:%d,,,,blue:%d",rarea,barea);
						stdb_pwm.data = -AdSpeed/k_ad;
						port_pwm.data = AdSpeed/k_ad;
						stdb_thrust_pub.publish(stdb_pwm);
						port_thrust_pub.publish(port_pwm);
						usleep(sleeptime1);
						stdb_pwm.data = AdSpeed/k_ad;
						port_pwm.data = AdSpeed/k_ad;
						stdb_thrust_pub.publish(stdb_pwm);
						port_thrust_pub.publish(port_pwm);
						usleep(sleeptime2);
						stdb_pwm.data = AdSpeed/k_ad;
						port_pwm.data = -AdSpeed/k_ad;
						stdb_thrust_pub.publish(stdb_pwm);
						port_thrust_pub.publish(port_pwm);
						usleep(sleeptime3);
					//ioctl(fd_pwm,IOCTL_CONFIG_PWM2, 925);  //左推进器
					//ioctl(fd_pwm,IOCTL_CONFIG_PWM3, 925);  //右推进器
					}
					else{
						ROS_INFO("Forwarding");
						stdb_pwm.data = 0.3;
						port_pwm.data = 0.3;
						stdb_thrust_pub.publish(stdb_pwm);
						port_thrust_pub.publish(port_pwm);
						printf("same center\n");
					}
			//	}
			}
			else if(rarea>th_high) //停止
			{
				    ROS_INFO("Stoping");
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

/**This program use controls the PWM based on the Joy node
Jan-09-2020 Mingxi This node sets the PWM to two motors
								changed the ros node Subscriber

										directly subscribe to joy message
Dec-24-2020 Jianguang Modified for bidirectional sec.
*/


//include c language libraries
extern "C" {
#include "roboticscape.h"
}
#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h>  // for atoi() and exit()
#include <signal.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int16.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int8.h"
int c_port_cmd = 0, c_stdb_cmd = 0, heartbeat=0, count=0;
std_msgs::Float64 c_port_duty, c_stdb_duty;
std_msgs::Int16 c_port_us, c_stdb_us;
int running = 0;
int INIT_PULSE_WIDTH = 1500; //us
double WAKEUP_HS = 0.0;// 1000us
double WAKEUP_HT = 0.5;// 1500us half-throttle
float MAX_PW_INCREMENT =200.0;// This is the hard limit for the thruster to prevent drawing too much current

void Motor_Callback (const sensor_msgs::Joy::ConstPtr& msg)
{
	//check for button press for motor driver relay
	if (msg->buttons[9] == 1)
  	{
 	   	
        	//wait for the esc
        	ROS_INFO("waking ESC up from idle for 3 seconds");
        	for(int i=0;i<=150;i++)
        	{
            		if(running==0) return;
            		if(rc_servo_send_esc_pulse_normalized(1,WAKEUP_HS)==-1) return;
					rc_usleep(1000000/50);
					if(rc_servo_send_esc_pulse_normalized(1,WAKEUP_HT )==-1) return;
					rc_usleep(1000000/50);
					if(rc_servo_send_esc_pulse_normalized(3,WAKEUP_HS)==-1) return;
					rc_usleep(1000000/50);
					if(rc_servo_send_esc_pulse_normalized(3,WAKEUP_HT )==-1) return;
               		rc_usleep(1000000/50);
         	}
        	ROS_INFO("done with wakeup period");
  	}

	//only cmd PWM when it is postive
	if(msg->axes[1]>0.05||msg->axes[1]<-0.05)
	{
	//left stick
		c_port_duty.data = msg->axes[1];
		c_port_cmd = (int)(msg->axes[1]*MAX_PW_INCREMENT) + INIT_PULSE_WIDTH;	//set global pulse width varilable
		ROS_INFO("axes1:%d",c_port_cmd);
	}
	else 
	{
	c_port_cmd=1500;
	}
	if(msg->axes[3]>0.05||msg->axes[3]<-0.05)
	{
	//right stick
		c_stdb_duty.data = msg->axes[3];
		c_stdb_cmd = (int)( msg->axes[3]*MAX_PW_INCREMENT) + INIT_PULSE_WIDTH;
	}
	else
	{
	c_stdb_cmd=1500;
	}
}

// interrupt handler to catch ctrl-c
void MySigintHandler(int sig)
{
	//shutting down
  	//ROS_INFO("shutting down!");
	rc_servo_power_rail_en(0);
 	 rc_servo_cleanup();
	running = 0;
  	ros::shutdown();
}

void heartbeatCallback(const std_msgs::Int8::ConstPtr& msg)
{
    heartbeat=heartbeat+1;
}

int main(int argc, char** argv)
{
  	usleep(100000);  //there will be problems if there is no delay here
	//setup servo
	if(rc_servo_init()) return -1;
        if(rc_servo_set_esc_range(RC_ESC_DEFAULT_MIN_US,RC_ESC_DEFAULT_MAX_US)) return -1;
        rc_servo_power_rail_en(0);

	  //if ctrl-c we shut down things
	signal(SIGINT, MySigintHandler);
  	running = 1;

	//ROS node stuff
  	ros::init(argc, argv, "rc_pwm");

  	ros::NodeHandle nh;

	//Remote control mode
	ros::Subscriber sub1 = nh.subscribe("/joy", 2, Motor_Callback);
	ros::Subscriber sub2 = nh.subscribe("/heartbeat", 2, heartbeatCallback);
	ros::Publisher port_pwm_pub = nh.advertise<std_msgs::Int16>("/usv/port_motor/cmd", 2);
  	ros::Publisher stdb_pwm_pub = nh.advertise<std_msgs::Int16>("/usv/stdb_motor/cmd", 2);
	ros::Rate loop_rate(50);  //esc need 50 Hz
  	while (ros::ok() & running)
  	{
		count++;
		if(count>200){
		  if(heartbeat<1){
			  ros::shutdown();
		  }
		  count=0;
		  heartbeat=0;
		}
	  	// send pwm adjust command
		rc_servo_send_pulse_us(1, c_port_cmd);//port side
		rc_servo_send_pulse_us(3, c_stdb_cmd);//starboard side
		//ROS_INFO("current[%d,%d]", c_port_cmd,c_stdb_cmd);
		//ROS_INFO("voltage[%f,%f]", c_port_duty.data,c_stdb_duty.data);
		//publish the cmd to ros topics
		c_port_us.data=c_port_cmd;
		port_pwm_pub.publish(c_port_us);
		c_stdb_us.data=c_stdb_cmd;
		stdb_pwm_pub.publish(c_stdb_us);

	    	ros::spinOnce();
    		loop_rate.sleep();

  	}
  	return 0;
}

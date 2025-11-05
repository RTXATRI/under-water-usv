/**This program use controls the PWM based on the Joy node
Jan-09-2020 Mingxi This node sets the PWM to two motors
										changed the ros node Subscriber
										directly subscribe to joy message
Aug-12-2020 heartbeat added. This node will stop if there are no heatbeat signals in 4s.

May-2020 control the usv with the right joy alone. remote mode 2 if "X"is pressed.
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
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/Joy.h"


int c_port_cmd = 0,c_stdb_cmd=0,heartbeat=0,count=0;
std_msgs::Float64 c_port_duty, c_stdb_duty,now_heading;
int running = 0, pbcheading_flag=0;
int INIT_PULSE_WIDTH = 1500; //us
//int WAKEUP_EN = 1;      // wakeup period enabled by default
double WAKEUP_HS = 0.0;// 1000us
double WAKEUP_HT = 0.5;// 1500us half-throttle
double TURN_P = 0.5;
float MAX_PW_INCREMENT =200.0;// This is the hard limit for the thruster to prevent drawing too much current

int AUTO_FLAG=0;

void stdb_callback (const std_msgs::Float64::ConstPtr& msg)
{
	if(AUTO_FLAG==1)
	{
	c_stdb_duty.data = msg->data;
	c_stdb_cmd = (int)( c_stdb_duty.data*MAX_PW_INCREMENT) + INIT_PULSE_WIDTH;
	}
}

void port_callback (const std_msgs::Float64::ConstPtr& msg)
{
	if(AUTO_FLAG==1)
	{
	c_port_duty.data = msg->data;
	c_port_cmd = (int)( c_port_duty.data*MAX_PW_INCREMENT) + INIT_PULSE_WIDTH;
	}
}

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
	//Close the relay
  	if (msg->buttons[8] == 1)
  	{
    		if(rc_servo_send_esc_pulse_normalized(3,WAKEUP_HT )==-1) return;
            rc_usleep(1000000/50);
  	}
	//only cmd PWM when it is postive
	if(AUTO_FLAG==0)
	{
	//left stick
		c_port_cmd = (int)(msg->axes[1]*MAX_PW_INCREMENT) + INIT_PULSE_WIDTH;	//set global pulse width varilable
		c_stdb_cmd = (int)( msg->axes[3]*MAX_PW_INCREMENT) + INIT_PULSE_WIDTH;
		ROS_INFO("c_port_cmd%d",c_port_cmd);
	}
	// remote mode 2, turn with joystick
	if(AUTO_FLAG==2)
	{
	//left stick
		ROS_INFO("FLAG%d",AUTO_FLAG);
		c_stdb_duty.data = msg->axes[3]+TURN_P*msg->axes[2];
		c_port_duty.data = msg->axes[3]-TURN_P*msg->axes[2];
		c_stdb_cmd = (int)(c_stdb_duty.data*MAX_PW_INCREMENT) + INIT_PULSE_WIDTH;
		c_port_cmd = (int)(c_port_duty.data*MAX_PW_INCREMENT) + INIT_PULSE_WIDTH;
		
	}
	//Set auto flag if "A" is pressed
	if(msg->buttons[1]==1)
	{
		AUTO_FLAG=1;
		pbcheading_flag=0;
	}
	//disable auto flag if "B" is pressed
	if(msg->buttons[2]==1)
	{
		AUTO_FLAG=0;
		pbcheading_flag=0;
	}
	//remote mode 2 if "X"is pressed
	if(msg->buttons[0]==1)
	{
		AUTO_FLAG=2;
		pbcheading_flag=0;
	}
	
		//remote mode 2 if "Y"is pressed
	if(msg->buttons[3]==1)
	{
		AUTO_FLAG=1;
		pbcheading_flag=1;
	}
	
}

// interrupt handler to catch ctrl-c
void MySigintHandler(int sig)
{
	//shutting down
  	ROS_INFO("shutting down!");
	rc_servo_power_rail_en(0);
 	 rc_servo_cleanup();
  	//rc_gpio_cleanup(1, 25);
  	//rc_gpio_cleanup(1, 17);
	running = 0;
  	ros::shutdown();
}

void heartbeatCallback(const std_msgs::Int8::ConstPtr& msg)
{
    heartbeat=heartbeat+1;
}

void imuCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    now_heading.data=msg->z;
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
  	ros::init(argc, argv, "auto_pwm");

  	ros::NodeHandle nh;

	//Remote control mode
	ros::Subscriber sub1 = nh.subscribe("/joy", 2, Motor_Callback);
	ros::Subscriber sub2 = nh.subscribe("/thrusters/stdb_cmd",2, stdb_callback);
	ros::Subscriber sub3 = nh.subscribe("/thrusters/port_cmd",2, port_callback);
	ros::Subscriber sub4 = nh.subscribe("/heartbeat", 2, heartbeatCallback);
	ros::Subscriber sub5 = nh.subscribe("/imu/rpy", 2, imuCallback);
	ros::Publisher port_pwm_pub = nh.advertise<std_msgs::Int8>("/joy/port_cmd", 2);
  	ros::Publisher stdb_pwm_pub = nh.advertise<std_msgs::Int8>("/joy/stdb_cmd", 2);
	ros::Publisher mode_pub = nh.advertise<std_msgs::Int8>("/light",2);
	ros::Publisher c_heading_pub = nh.advertise<std_msgs::Float64>("/control/heading",2);

	ros::Rate loop_rate(50);  //esc need 50 Hz
	
	std_msgs::Int8 gnc_mode;
	std_msgs::Int8 joy_port;
	std_msgs::Int8 joy_stdb;
	//START THE LOOP
  	while (ros::ok() & running)
  	{
	  	// send pwm adjust command
		rc_servo_send_pulse_us(1, c_port_cmd);//port side
		rc_servo_send_pulse_us(3, c_stdb_cmd);//starboard side
		//joy stuff
		joy_port.data=c_port_cmd;
		joy_stdb.data=c_stdb_cmd;
		gnc_mode.data = 3-AUTO_FLAG;	
		count++;
		if(count>200){
		port_pwm_pub.publish(joy_port);
		stdb_pwm_pub.publish(joy_stdb);
		mode_pub.publish(gnc_mode);	
		if(pbcheading_flag==1){
		pbcheading_flag=0;
		c_heading_pub.publish(now_heading);
		}
		  if(heartbeat<1){
			  ros::shutdown();
		  }
		  count=0;
		  heartbeat=0;
		}
	    	ros::spinOnce();
    		loop_rate.sleep();

  	}
  	return 0;
}

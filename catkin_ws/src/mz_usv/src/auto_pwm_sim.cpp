#include <rc/servo.h>
#include <stdlib.h>  // for atoi() and exit()
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <signal.h>


int c_port_cmd = 0;
int c_stdb_cmd=0;

int INIT_PULSE_WIDTH = 1500; //us
float MAX_PW_INCREMENT =200.0;// This is the hard limit for the thruster to prevent drawing too much current

void stdb_callback (const std_msgs::Float64::ConstPtr& msg)
{
	ROS_INFO("stdb_callback:%lf",msg->data);
	c_stdb_cmd = (int)(msg->data*MAX_PW_INCREMENT) + INIT_PULSE_WIDTH;
}

void port_callback (const std_msgs::Float64::ConstPtr& msg)
{
	ROS_INFO("port_callback:%lf",msg->data);
	c_port_cmd = (int)(msg->data*MAX_PW_INCREMENT) + INIT_PULSE_WIDTH;
}

void MySigintHandler(int sig)
{
	//shutting down
  	ROS_INFO("shutting down!");
	rc_servo_power_rail_en(0);
 	rc_servo_cleanup();
  	ros::shutdown();
}
int main(int argc, char** argv)
{
	if(rc_servo_init()) 
	{
		ROS_ERROR("rc_servo_init error");
		return -1;
	}

	if(rc_servo_set_esc_range(RC_ESC_DEFAULT_MIN_US,RC_ESC_DEFAULT_MAX_US)) 
	{
		ROS_ERROR("rc_servo_set_esc_range error");
		return -1;
	}

	rc_servo_power_rail_en(0);

  	ros::init(argc, argv, "auto_pwm_sim");
  	ros::NodeHandle nh;
	ROS_INFO("running");

	ros::Subscriber sub2 = nh.subscribe("/thrusters/stdb_cmd",2, stdb_callback);
	ros::Subscriber sub3 = nh.subscribe("/thrusters/port_cmd",2, port_callback);

	ros::Rate loop_rate(50);  //esc need 50 Hz
	
  	while (ros::ok())
  	{
	signal(SIGINT, MySigintHandler);
		rc_servo_send_pulse_us(1, c_port_cmd);//port side
		rc_servo_send_pulse_us(3, c_stdb_cmd);//starboard side

		ros::spinOnce();
		loop_rate.sleep();
  	}
  	return 0;
}

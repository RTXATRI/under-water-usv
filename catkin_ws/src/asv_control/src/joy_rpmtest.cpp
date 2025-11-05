/**
 * subscribe std_msgs::Joy and control the left motor
 sleep is required to generate pwm signal. To avoid interference between two sigals, seperate nodes are used to deal
 with two motors
 */

#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
extern "C" {
#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h>  // for atoi() and exit()
#include <rc/time.h>
#include <rc/gpio.h>
#include <rc/adc.h>
#include <signal.h>
#include <rc/dsm.h>
}

float rpm = 0;
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  ROS_INFO("joyaxes: [%f]", joy->axes[1]);
  if (joy->axes[1] > 0)
  {
    rpm = 1600 * joy->axes[1];  // set the rpm according to the stick position
  }
  ROS_INFO("rpm: [%f]", rpm); 
}

void MySigintHandler(int sig)
{
  ROS_INFO("shutting down!");
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_rpm");
  // ros::init(argc, argv, "rightmotor");
  ros::NodeHandle joyrpm;
  std_msgs::Float64 rpm_msg;
  signal(SIGINT, MySigintHandler);
  ros::Rate loop_rate(100);
  ros::Publisher pub = joyrpm.advertise<std_msgs::Float64>("d_rightrpm", 100);
  ros::Publisher pub1 = joyrpm.advertise<std_msgs::Float64>("d_leftrpm", 100);
  ros::Subscriber sub = joyrpm.subscribe<sensor_msgs::Joy>("/joy", 10, joyCallback);
  // subscribe joy joy topic and start callback function
  while (ros::ok())
  {
    rpm_msg.data = rpm;
	  pub.publish(rpm_msg);
    pub1.publish(rpm_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

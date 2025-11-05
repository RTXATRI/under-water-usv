/**
 * 该例程将订阅chatter话题，消息类型String
 */

#include "ros/ros.h"
#include "std_msgs/Int8.h"
extern "C" {
#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h>  // for atoi() and exit()
#include <rc/time.h>
#include <rc/adc.h>
#include <rc/dsm.h>
#include <rc/servo.h>
}
// 接收到订阅的消息后，会进入消息回调函数
void leftmotorCallback(const std_msgs::Int8::ConstPtr& width_us)
{
  // 将接收到的消息打印出来

  //		int width_us = atof(optarg);
  int k = width_us->data * 100;
  rc_servo_send_pulse_us(1, width_us->data * 100);
  ROS_INFO("I heard: [%d]", width_us->data);
}

int main(int argc, char** argv)
{
  //			int width_us=1500;
  //			int ch=1;
  rc_servo_init();

  ros::init(argc, argv, "motor_left");
  ros::NodeHandle leftmotor;

  ////  ros::Publisher imu_pub = dmp.advertise<std_msgs::String>("command_to_imu", 1000);

  //  ros::Rate loop_rate(5);

  ros::Subscriber sub = leftmotor.subscribe("motor_left", 8, leftmotorCallback);

  //收到命令后调用相应的函数
  // ros::Subscriber odom_sub = nh.subscribe("odometry/filtered",1, &Controller::odom_callback, &heron_control);

  // ros::Timer control_output = nh.createTimer(ros::Duration(1/50.0), &Controller::control_update,&heron_control);
  // ros::Timer console_update = nh.createTimer(ros::Duration(1), &Controller::console_update, &heron_control);

  ros::spin();

  return 0;
}

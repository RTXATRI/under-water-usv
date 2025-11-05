/**
 * 该例程将发布chatter话题，消息类型String
 */

#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
extern "C" {
#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h>  // for atoi() and exit()
#include <rc/time.h>
#include <rc/gpio.h>
#include <rc/adc.h>
#include <rc/dsm.h>
#include <rc/servo.h>
}

int main(int argc, char** argv)
{
  // ROS节点初始化
  ros::init(argc, argv, "motortest");

  // 创建节点句柄
  ros::NodeHandle n;

  // 创建一个Publisher，发布名为chatter的topic，消息类型为std_msgs::String
  ros::Publisher chatter_pub = n.advertise<sensor_msgs::Joy>("joy", 1000);

  // 设置循环的频率

  while (ros::ok())
  {
    // 初始化std_msgs::String类型的消息
    sensor_msgs::Joy joy;
    joy.axes.resize(5);
    joy.axes[1] = 0.99;
    joy.axes[3] = -0.6;
    // 发布消息
    ROS_INFO("%f", joy.axes[1]);
    chatter_pub.publish(joy);
    // 循环等待回调函数
    ros::spinOnce();
    // 按照循环频率延时
  }

  return 0;
}

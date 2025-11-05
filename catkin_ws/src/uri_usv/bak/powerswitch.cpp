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
#include <rc/gpio.h>
#include <rc/time.h>
}
// 接收到订阅的消息后，会进入消息回调函数
void powerswitchCallback(const std_msgs::Int8::ConstPtr& command)
{
  // 将接收到的消息打印出来

  switch (command->data)
  {
    case 1:
      // rc_gpio_set_value(3, 1, 1);
      break;
    case 0:
      // rc_gpio_set_value(3, 1, 0);
      break;
  }
  ROS_INFO("I heard: [%d]", command->data);
}

int main(int argc, char** argv)
{
  // if(rc_gpio_init(3, 1, GPIOHANDLE_REQUEST_OUTPUT)==-1){
  // printf("rc_gpio_init failed\n");
  // return -1;
  //}
  ros::init(argc, argv, "reedswitch");
  ros::NodeHandle reedswitch;

  ////  ros::Publisher imu_pub = dmp.advertise<std_msgs::String>("command_to_imu", 1000);

  //  ros::Rate loop_rate(5);

  ros::Subscriber sub = reedswitch.subscribe("powerswitch", 8, powerswitchCallback);

  //收到命令后调用相应的函数
  // ros::Subscriber odom_sub = nh.subscribe("odometry/filtered",1, &Controller::odom_callback, &heron_control);

  // ros::Timer control_output = nh.createTimer(ros::Duration(1/50.0), &Controller::control_update,&heron_control);
  // ros::Timer console_update = nh.createTimer(ros::Duration(1), &Controller::console_update, &heron_control);

  ros::spin();

  return 0;
}

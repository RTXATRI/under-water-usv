/**
 * this node functions to switch on and off the reed switch by outputting 1 or 0 , controlled by joy.
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
#include <signal.h>
#include <rc/dsm.h>
}

float rpm = 0;
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  //ROS_INFO("start: [%d]", joy->buttons[9]);
  //ROS_INFO("back: [%d]", joy->buttons[8]);
  if (joy->buttons[9] == 1)
  {
    rc_gpio_set_value(3, 1, 1);  // turn on the relay
  }
    if (joy->buttons[8] == 1)
  {
    rc_gpio_set_value(3, 1, 0);  // turn off the relay
  }
}

void MySigintHandler(int sig)
{
  ROS_INFO("shutting down!");
  rc_gpio_set_value(3, 1, 0);
  ros::shutdown();
}

int main(int argc, char** argv)
{
  if (rc_gpio_init(3, 1, GPIOHANDLE_REQUEST_OUTPUT) == -1)
  {
    printf("rc_gpio_init failed\n");
    return -1;
  }
  ros::init(argc, argv, "switch_io");
  ros::NodeHandle nh;
  signal(SIGINT, MySigintHandler);
  // subscribe joy joy topic and start callback function
  ros::Subscriber sub = nh.subscribe<sensor_msgs::Joy>("/joy", 2, joyCallback);
  ros::spin();
  return 0;
}

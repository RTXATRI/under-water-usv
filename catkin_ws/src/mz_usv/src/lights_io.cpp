/**
 * subscribe light topic and flash the lights, only for the red light.
 jianguang 01/25/2020
 */

#include "ros/ros.h"
#include "std_msgs/Int8.h"
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
void lightCallback(const std_msgs::Int8::ConstPtr& light)
{
  switch (light->data)
  {
    case 1:
	  usleep(1000000);
      rc_gpio_set_value(3, 2, 1);
	  usleep(1000000);
      rc_gpio_set_value(3, 2, 0);
      break;
    case 0:
	  usleep(200000);
      rc_gpio_set_value(3, 2, 1);
	  usleep(200000);
      rc_gpio_set_value(3, 2, 0);
      break;
    case 2:
      rc_gpio_set_value(3, 2, 1);
      break;
    case 3:
      rc_gpio_set_value(3, 2, 0);
      break;
    case 11:
	  usleep(1000000);
      rc_gpio_set_value(3, 1, 1);
	  usleep(1000000);
      rc_gpio_set_value(3, 1, 0);
      break;
    case 10:
	  usleep(200000);
      rc_gpio_set_value(3, 1, 1);
	  usleep(200000);
      rc_gpio_set_value(3, 1, 0);
      break;
    case 12:
      rc_gpio_set_value(3, 1, 1);
      break;
    case 13:
      rc_gpio_set_value(3, 1, 0);
      break;
  }
  //ROS_INFO("I heard: [%d]", light->data);
}

void MySigintHandler(int sig)
{
  ROS_INFO("shutting down!");
  rc_gpio_set_value(3, 2, 0);
  rc_gpio_set_value(3, 1, 0);
  ros::shutdown();
}

int main(int argc, char** argv)
{
  if (rc_gpio_init(3, 2, GPIOHANDLE_REQUEST_OUTPUT) == -1)
  {
    printf("rc_gpio_init failed\n");
    return -1;
  }
   if (rc_gpio_init(3, 1, GPIOHANDLE_REQUEST_OUTPUT) == -1)
  {
    printf("rc_gpio_init failed\n");
    return -1;
  }
  ros::init(argc, argv, "light_io");
  // ros::init(argc, argv, "rightmotor");
  ros::NodeHandle nh;
  signal(SIGINT, MySigintHandler);
  // subscribe joy joy topic and start callback function
  ros::Subscriber sub = nh.subscribe<std_msgs::Int8>("light", 2, lightCallback);
  ros::spin();
  return 0;
}

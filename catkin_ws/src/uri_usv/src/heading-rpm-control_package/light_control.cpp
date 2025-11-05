/**
 * subscribe std_msgs::Joy and control the left motor
 sleep is required to generate pwm signal. To avoid interference between two sigals, seperate nodes are used to deal
 with two motors
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
      rc_gpio_set_value(3, 20, 1);
	  usleep(1000000);
      rc_gpio_set_value(3, 20, 0);
      break;
    case 0:
	  usleep(200000);
      rc_gpio_set_value(3, 20, 1);
	  usleep(200000);
      rc_gpio_set_value(3, 20, 0);
      break;
    case 2:
      rc_gpio_set_value(3, 20, 1);
      break;
    case 3:
      rc_gpio_set_value(3, 20, 0);
      break;
  }
  ROS_INFO("I heard: [%d]", light->data);
}

void MySigintHandler(int sig)
{
  ROS_INFO("shutting down!");
  rc_gpio_set_value(3, 20, 0);
  ros::shutdown();
}

int main(int argc, char** argv)
{
  if (rc_gpio_init(3, 20, GPIOHANDLE_REQUEST_OUTPUT) == -1)
  {
    printf("rc_gpio_init failed\n");
    return -1;
  }
  ros::init(argc, argv, "light_control");
  // ros::init(argc, argv, "rightmotor");
  ros::NodeHandle light_control;
  signal(SIGINT, MySigintHandler);
  // subscribe joy joy topic and start callback function
  ros::Subscriber sub = light_control.subscribe<std_msgs::Int8>("light", 10, lightCallback);
  ros::spin();
  return 0;
}

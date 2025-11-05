/**
 * this node functions to switch on and off the reed switch by outputting 1 or 0
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
void powerswitchCallback(const std_msgs::Int8::ConstPtr& command)
{
  switch (command->data)
  {
    case 1:
      rc_gpio_set_value(3, 1, 1);
      break;
    case 0:
      rc_gpio_set_value(3, 1, 0);
      break;
  }
  ROS_INFO("I heard: [%d]", command->data);
}

int main(int argc, char** argv)
{
  if (rc_gpio_init(3, 1, GPIOHANDLE_REQUEST_OUTPUT) == -1)
  {
    printf("rc_gpio_init failed\n");
    return -1;
  }
  ros::init(argc, argv, "reedswitch");
  ros::NodeHandle reedswitch;
  ros::Subscriber sub = reedswitch.subscribe("powerswitch", 2, powerswitchCallback);
  ros::spin();
  return 0;
}

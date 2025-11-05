/**
 * subscribe std_msgs::Joy and control the right motor
 sleep is required to generate pwm signal. To avoid interference between two sigals, seperate nodes are used to deal
 with two motors
 */

#include "ros/ros.h"
//#include <std_msgs/Float64MultiArray.h>
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
#include <rc/servo.h>
}
int frequency = 200;  // set the frequency of the pwm
int period = 1000000 / frequency;
float rightduty = 0.5;
// get in to the callback when message is received
void rpmCallback(const std_msgs::Float64::ConstPtr& control_effort)
{
	ROS_INFO("control_effort->data: [%f]", control_effort->data);
	rightduty = rightduty-(control_effort->data)/100;
  if (rightduty>=1)
  {
	  rightduty=1;
  }
  if (rightduty<=0)
  {
	  rightduty=0;
  }
  ROS_INFO("rightduty: [%f]", rightduty);
  rc_gpio_set_value(1, 17, 0);
  // send pwm adjust command
  rc_servo_send_pulse_us(2, 0.7 * period);
  ROS_INFO("sendrightduty: [%f]", rightduty);
  rc_usleep(period);
}

void MySigintHandler(int sig)
{
  //这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
  ROS_INFO("shutting down!");
  rc_servo_cleanup();
  rc_gpio_cleanup(1, 17);
  ros::shutdown();
}

int main(int argc, char** argv)
{
  // initiate pwm;
  rc_servo_init();
  // initiate 4 gpios
  if (rc_gpio_init(1, 17, GPIOHANDLE_REQUEST_OUTPUT) == -1)
  {
    printf("rc_gpio_init failed\n");
    return -1;
  }
  /*          if(rc_gpio_init(3, 20, GPIOHANDLE_REQUEST_OUTPUT)==-1){
     printf("rc_gpio_init failed\n");
     return -1;
     }
              if(rc_gpio_init(3, 17, GPIOHANDLE_REQUEST_OUTPUT)==-1){
     printf("rc_gpio_init failed\n");
     return -1;
     } */
  ros::init(argc, argv, "set_right_pwm");
  // ros::init(argc, argv, "rightmotor");
  ros::NodeHandle rightrpm;
  ros::Subscriber sub = rightrpm.subscribe("rightpwm", 10, rpmCallback);
  signal(SIGINT, MySigintHandler);
  // subscribe joy joy topic and start callback function
  ros::spin();
  return 0;
    //count = ++;
}

/**
 *
*directly turn rpm to pwm by deviding 1600 and send the pwm signal.
*01/22 2020 add the function: control the asv by joystick when it is stopped.
 */

#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
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
#include <rc/servo.h>
}
int frequency,heartbeat=0,count=0;  // flags
int period;
float portduty = 0,stdbduty=0;
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	if (joy->buttons[9] == 1)
  {
    rc_gpio_set_value(3, 1, 1);  // turn on the relay
    //ROS_INFO("turn on");
  }
    if (joy->buttons[8] == 1)
  {
    rc_gpio_set_value(3, 1, 0);  // turn off the relay
    //ROS_INFO("turn off");
  }
  if (joy->axes[1] > 0)
  {
    // set the direction of the motor to be forward
    rc_gpio_set_value(1, 25, 0);;
    // set the duty
    portduty = joy->axes[1];
    if(portduty<0.3){
    	portduty=0.0;
    }
  }
  if (joy->axes[1] < 0)
  {
    // set the direction of the motor to be backward
    rc_gpio_set_value(1, 25, 1);
    // set the duty
    portduty = -joy->axes[1];
    if(portduty<0.3){
    	portduty=0.0;
    }
  }
  if (joy->axes[3] > 0)
  {
    rc_gpio_set_value(1, 17, 0);
    stdbduty = joy->axes[3];
    if(stdbduty<0.3){
    	stdbduty=0.0;
    }
  }
  if (joy->axes[3] < 0)
  {
    rc_gpio_set_value(1, 17, 1);
    stdbduty = -joy->axes[3];
    if(stdbduty<0.3){
    	stdbduty=0.0;
    }
  }
}


void heartbeatCallback(const std_msgs::Int8::ConstPtr& msg)
{
    heartbeat=heartbeat+1;
}


void MySigintHandler(int sig)
{
  //ROS_INFO("shutting down!");
  rc_servo_cleanup();
  rc_gpio_cleanup(1, 25);
  rc_gpio_cleanup(1, 17);
  ros::shutdown();
}

int main(int argc, char** argv)
{
   rc_servo_init();
  // initiate pwm;
  usleep(100000);  //there will be problems if there is no delay here
  // initiate 2 gpios
  if (rc_gpio_init(1, 25, GPIOHANDLE_REQUEST_OUTPUT) == -1)
  {
    printf("rc_gpio_init failed\n");
    return -1;
  }
  if (rc_gpio_init(1, 17, GPIOHANDLE_REQUEST_OUTPUT) == -1)
  {
    printf("rc_gpio_init failed\n");
    return -1;
  }
  if (rc_gpio_init(3, 1, GPIOHANDLE_REQUEST_OUTPUT) == -1)
  {
    printf("rc_gpio_init failed\n");
    return -1;
  }
  ros::init(argc, argv, "pwm_output");
  ros::NodeHandle node_priv("~");
  //ROS_INFO("f111111111: [%d]", frequency);
  node_priv.param("frequency", frequency,50);
  ros::NodeHandle nh;
  ros::Rate loop_rate(frequency);
  ros::Subscriber sub6 = nh.subscribe("/joy", 2, joyCallback);;
  ros::Subscriber sub5 = nh.subscribe("/heartbeat", 2, heartbeatCallback);
  //ros::AsyncSpinner spinner(6);
  period = 1000000 / frequency;
  signal(SIGINT, MySigintHandler);
  while (ros::ok())
  {
	  count++;
	  if(count>2500){
		  if(heartbeat<1){
			  ros::shutdown();
		  }
		  count=0;
		  heartbeat=0;
	  }
	rc_servo_send_pulse_us(2, (int)(stdbduty * period));
	rc_servo_send_pulse_us(1, (int)(portduty * period));
	//ROS_INFO("count:%d",count);
	//ROS_INFO("stdbduty3====%f",stdbduty);
	//ROS_INFO("portduty3====%f",portduty);
    loop_rate.sleep();
    ros::spinOnce();
	//spinner.stop();
  }
  return 0;
}

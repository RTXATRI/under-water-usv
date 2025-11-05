/**
 * subscribe std_msgs::Joy and control the left motor
 sleep is required to generate pwm signal. To avoid interference between two sigals, seperate nodes are used to deal
 with two motors
 01/07/2020 zero duty of pwm added.
 */

#include "ros/ros.h"
//#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include "std_msgs/Int8.h"
#include <boost/thread.hpp>
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
int frequency = 50;  // set the frequency of the pwm
int period,l_reverse=0,r_reverse=0;
float leftduty = 0,rightduty=0,dt,ct;
// multiThread to handle subscribers


void lioCallback(const std_msgs::Int8::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%d]", msg->data);
	l_reverse = msg->data;
}


void lpwmCallback(const std_msgs::Float64::ConstPtr& msg)
{
  //ROS_INFO("control_effort->data: [%f]", msg->data);
  leftduty = leftduty+(msg->data)/1000;
  if (leftduty>=1)
  {
	  leftduty=1;
  }
  if (leftduty<=0)
  {
	  leftduty=0;
  }
  ROS_INFO("leftduty: [%f]", leftduty);
  // send pwm adjust command
}

void rioCallback(const std_msgs::Int8::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%d]", msg->data);
    r_reverse = msg->data;

}


void rpwmCallback(const std_msgs::Float64::ConstPtr& msg)
{
  	rightduty = rightduty+(msg->data)/1000;
  if (rightduty>=1)
  {
	  rightduty=1;
  }
  if (rightduty<=0)
  {
	  rightduty=0;
  }
  ROS_INFO("controleffort: [%f]", msg->data);
}


void MySigintHandler(int sig)
{
  //这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
  ROS_INFO("shutting down!");
  rc_servo_cleanup();
  rc_gpio_cleanup(1, 25);
  ros::shutdown();
}

int main(int argc, char** argv)
{
  // initiate pwm;
  if(rc_servo_init()) return -1;
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
  /*          if(rc_gpio_init(3, 20, GPIOHANDLE_REQUEST_OUTPUT)==-1){
     printf("rc_gpio_init failed\n");
     return -1;
     }
              if(rc_gpio_init(3, 17, GPIOHANDLE_REQUEST_OUTPUT)==-1){
     printf("rc_gpio_init failed\n");
     return -1;
     } */
  ros::init(argc, argv, "pwm");
  ros::NodeHandle node_priv("~");
  //ROS_INFO("f111111111: [%d]", frequency);
  node_priv.getParam("frequency", frequency);
  ros::NodeHandle nh;
  ros::Rate loop_rate(frequency);
  ros::Subscriber sub1 = nh.subscribe("/left_motor/l_reverse", 2, lioCallback);
  ros::Subscriber sub2 = nh.subscribe("/left_motor/leftpwm", 2, lpwmCallback);
  ros::Subscriber sub3 = nh.subscribe("/right_motor/r_reverse", 2, rioCallback);
  ros::Subscriber sub4 = nh.subscribe("/right_motor/rightpwm", 2, rpwmCallback);
  //ROS_INFO("f222222222: [%d]", frequency);
  //sleep(5);
  period = 1000000 / frequency;
  // ros::init(argc, argv, "leftmotor");
  signal(SIGINT, MySigintHandler);
  while (ros::ok())
  {
	  //ROS_INFO("bbbbbbbbbbb");
	  dt = ((float)rc_nanos_since_boot() - (float)ct) / 1000000.00;
	  //ROS_INFO("dtttttttt111111111111=%f",dt);
	  ct = rc_nanos_since_boot();
		if(l_reverse==1){   //reverse
			  rc_gpio_set_value(1, 25, 1);
				//ROS_INFO("lllllreverse");
		}
		if(l_reverse==0){
			  rc_gpio_set_value(1, 25, 0);
		}
		if(r_reverse==1){   //reverse
			  rc_gpio_set_value(1, 17, 1);
				//ROS_INFO("rrrrrrreverse");
		}
		if(r_reverse==0){
			  rc_gpio_set_value(1, 17, 0);
		}
		if(r_reverse==3||l_reverse==3){
			rightduty=leftduty=0;
			//ROS_INFO("zero_outputtttt");
		}
	  // send pwm adjust command
	rc_servo_send_pulse_us(2, (int)(rightduty * period));
	rc_servo_send_pulse_us(1, (int)(leftduty * period));
	//predt=dt;
	//dt = ((float)rc_nanos_since_boot() - (float)ct) / 1000000000.00;
	//ROS_INFO("dtttttttt111111111111=%f",dt);
	//if(dt<0.0005){
	//	dt=predt;
	//}
	//ROS_INFO("dtttttttt=%f",dt);
    ros::spinOnce();
    //ct = rc_nanos_since_boot();
    loop_rate.sleep();
    //count = ++;
  }
  // subscribe joy joy topic and start callback function
  return 0;
    //count = ++;
}

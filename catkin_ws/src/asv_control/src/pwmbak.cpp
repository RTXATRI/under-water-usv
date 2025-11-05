/**
 * subscribe std_msgs::Joy and control the left motor
 sleep is required to generate pwm signal. To avoid interference between two sigals, seperate nodes are used to deal
 with two motors
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
int period;
float leftduty = 0,rightduty=0;
// multiThread to handle subscribers
class multiThreadListener
{
public:
	multiThreadListener()
	{	
		sub1 = n.subscribe("/left_motor/l_reverse", 2, &multiThreadListener::lioCallback,this);
		sub2 = n.subscribe("/left_motor/leftpwm", 2, &multiThreadListener::lpwmCallback,this);
		sub3 = n.subscribe("/right_motor/r_reverse", 2, &multiThreadListener::rioCallback,this);
		sub4 = n.subscribe("/right_motor/rightpwm", 2, &multiThreadListener::rpwmCallback,this);
	}
	void lioCallback(const std_msgs::Int8::ConstPtr& msg);
	void lpwmCallback(const std_msgs::Float64::ConstPtr& msg);
	void rioCallback(const std_msgs::Int8::ConstPtr& msg);
	void rpwmCallback(const std_msgs::Float64::ConstPtr& msg);

private:
	ros::NodeHandle n;
	ros::Subscriber sub1;
	ros::Subscriber sub2;
	ros::Subscriber sub3;
	ros::Subscriber sub4;  
};


void multiThreadListener::lioCallback(const std_msgs::Int8::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%d]", msg->data);
	if(msg->data==1){   //reverse
		  rc_gpio_set_value(1, 25, 1);
			ROS_INFO("reverse");
	}
	if(msg->data==0){
		  rc_gpio_set_value(1, 25, 0);
	}
}


void multiThreadListener::lpwmCallback(const std_msgs::Float64::ConstPtr& msg)
{
  //ROS_INFO("control_effort->data: [%f]", msg->data);
  leftduty = leftduty+(msg->data)/1000.00;
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
  rc_servo_send_pulse_us(1, (int)(leftduty * period));
  //ROS_INFO("sendleftduty: [%f], [%d]", leftduty,(int)(leftduty * period));
  rc_usleep(period);
}

void multiThreadListener::rioCallback(const std_msgs::Int8::ConstPtr& msg)
{
  ROS_INFO("I heard: [%d]", msg->data);
	if(msg->data==1){   //reverse
		  rc_gpio_set_value(1, 17, 1);
			ROS_INFO("rrrrrrrrrrreverse");
	}
	if(msg->data==0){
		  rc_gpio_set_value(1, 17, 0);
	}
}


void multiThreadListener::rpwmCallback(const std_msgs::Float64::ConstPtr& msg)
{
  	rightduty = rightduty+(msg->data)/1000.00;
  if (rightduty>=1)
  {
	  rightduty=1;
  }
  if (rightduty<=0)
  {
	  rightduty=0;
  }
  ROS_INFO("rightduty: [%f]", rightduty);
  //rc_gpio_set_value(1, 17, 0);
  // send pwm adjust command
  //rc_servo_init();
  rc_servo_send_pulse_us(2, (int)(rightduty * period));
  //ROS_INFO("sendrightduty: [%f], [%d]", rightduty,(int)(rightduty * period));
  rc_usleep(period);
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
  ROS_INFO("f111111111: [%d]", frequency);
  node_priv.getParam("frequency", frequency);
  ROS_INFO("f222222222: [%d]", frequency);
  sleep(5);
  period = 1000000 / frequency;
  // ros::init(argc, argv, "leftmotor");
  multiThreadListener listener_obj;
  ros::MultiThreadedSpinner s(4);
  ros::spin(s);
  signal(SIGINT, MySigintHandler);
  // subscribe joy joy topic and start callback function
  return 0;
    //count = ++;
}

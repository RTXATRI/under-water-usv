/**
 *
directly turn rpm to pwm
01/22 2020 add the function: control the asv by joystick when it is stopped.
 */

#include "ros/ros.h"
//#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include "std_msgs/Int8.h"
//#include <boost/thread.hpp>
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
int frequency = 50,heartbeat=0,count=0,joyFlag=1,stopcount=0;  // set the frequency of the pwm
int period,l_reverse=0,r_reverse=0;
float leftduty = 0,rightduty=0,dt,ct;
// multiThread to handle subscribers
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
if(joyFlag==1){
  //pwmFlag=1;
  //ROS_INFO("joyyyyyyyyyyyyyy");
  if (joy->axes[1] > 0)
  {
    // set the direction of the motor to be forward
    rc_gpio_set_value(1, 25, 0);
    // rc_gpio_set_value(1, 17, 0);
    // set the duty
    leftduty = joy->axes[1];
    if(leftduty<0.3){
    	leftduty=0.0;
    }
  }
  if (joy->axes[1] < 0)
  {
    // set the direction of the motor to be backward
    rc_gpio_set_value(1, 25, 1);
    // rc_gpio_set_value(1, 17, 1);
    // set the duty
    leftduty = -joy->axes[1];
    if(leftduty<0.3){
    	leftduty=0.0;
    }
  }
  // send pwm adjust command
  if (joy->axes[3] > 0)
  {
    // set the direction of the motor to be forward
    // rc_gpio_set_value(1, 25, 0);
    rc_gpio_set_value(1, 17, 0);
    // set the duty
    rightduty = joy->axes[3];
    if(rightduty<0.3){
    	rightduty=0.0;
    }
  }
  if (joy->axes[3] < 0)
  {
    // set the direction of the motor to be backward
    // rc_gpio_set_value(1, 25, 1);
    rc_gpio_set_value(1, 17, 1);
    // set the duty
    rightduty = -joy->axes[3];
    if(rightduty<0.3){
    	rightduty=0.0;
    }
  }
  // print the received data
}
}

void lioCallback(const std_msgs::Int8::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%d]", msg->data);
	l_reverse = msg->data;
}


void lpwmCallback(const std_msgs::Float64::ConstPtr& msg)
{
  //only act when there is rpm, to avoid interface with joy

	if(msg->data>1){
		//pwmFlag=1;
		  leftduty = (msg->data)/1600;
		  ROS_INFO("leftduty: [%f]", leftduty);
	}

  // send pwm adjust command
}

void rioCallback(const std_msgs::Int8::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%d]", msg->data);
    r_reverse = msg->data;

}

void heartbeatCallback(const std_msgs::Int8::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%d]", msg->data);
    heartbeat=heartbeat+1;

}
void rpwmCallback(const std_msgs::Float64::ConstPtr& msg)
{
	if(msg->data>1){
		//pwmFlag=1;
		rightduty = (msg->data)/1600;
		  ROS_INFO("rightduty: [%f]", msg->data);
	}
}


void MySigintHandler(int sig)
{
  //这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
  ROS_INFO("shutting down!");
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
  /*          if(rc_gpio_init(3, 20, GPIOHANDLE_REQUEST_OUTPUT)==-1){
     printf("rc_gpio_init failed\n");
     return -1;
     }
              if(rc_gpio_init(3, 17, GPIOHANDLE_REQUEST_OUTPUT)==-1){
     printf("rc_gpio_init failed\n");
     return -1;
     } */
  ros::init(argc, argv, "di_pwm");
  ros::NodeHandle node_priv("~");
  //ROS_INFO("f111111111: [%d]", frequency);
  node_priv.getParam("frequency", frequency);
  ros::NodeHandle nh;
  ros::Rate loop_rate(frequency);
  ros::Subscriber sub1 = nh.subscribe("/left_motor/l_reverse", 2, lioCallback);
  ros::Subscriber sub2 = nh.subscribe("/left_motor/d_leftrpm", 2, lpwmCallback);
  ros::Subscriber sub3 = nh.subscribe("/right_motor/r_reverse", 2, rioCallback);
  ros::Subscriber sub4 = nh.subscribe("/right_motor/d_rightrpm", 2, rpwmCallback);
  ros::Subscriber sub6 = nh.subscribe("/heading/joy", 2, joyCallback);;
  ros::Subscriber sub5 = nh.subscribe("/heartbeat", 2, heartbeatCallback);
  //ros::AsyncSpinner spinner(6);
  //ROS_INFO("f222222222: [%d]", frequency);
  //sleep(5);
  period = 1000000 / frequency;
  // ros::init(argc, argv, "leftmotor");
  signal(SIGINT, MySigintHandler);
  while (ros::ok())
  {
	  if(joyFlag==1&&stopcount<3){
		  rightduty=leftduty=0;
	  }
	  count++;
	  if(count>200){
		  if(heartbeat<1){
			  ros::shutdown();
		  }
		  count=0;
		  heartbeat=0;
	  }
	  dt = ((float)rc_nanos_since_boot() - (float)ct) / 1000000.00;
	  //ROS_INFO("dtttttttt111111111111=%f",dt);
	  ct = rc_nanos_since_boot();
	  //spinner.start();
		if(l_reverse==1){   //reverse
			  joyFlag=0;
			  stopcount=0;
			  rc_gpio_set_value(1, 25, 1);
		}
		if(l_reverse==0){
			  joyFlag=0;
			  stopcount=0;
			  rc_gpio_set_value(1, 25, 0);
		}
		if(r_reverse==1){   //reverse
			  joyFlag=0;
			  rc_gpio_set_value(1, 17, 1);
		}
		if(r_reverse==0){
			  joyFlag=0;
			  rc_gpio_set_value(1, 17, 0);
		}
		if(r_reverse==3||l_reverse==3){
			//rightduty=leftduty=0;
			joyFlag=1;
			//pwmFlag=0;
			stopcount++;
			//ROS_INFO("zero_outputtttt");
		}
	  // send pwm adjust command
		//ROS_INFO("sssssssssstate:%d",pwmState);
	rc_servo_send_pulse_us(2, (int)(rightduty * period));
	rc_servo_send_pulse_us(1, (int)(leftduty * period));
	//predt=dt;
	//dt = ((float)rc_nanos_since_boot() - (float)ct) / 1000000000.00;
	ROS_INFO("rightduty3====%f",rightduty);
	ROS_INFO("leftduty3====%f",leftduty);
	//if(dt<0.0005){
	//	dt=predt;
	//}
	//ROS_INFO("dtttttttt=%f",dt);
    //ct = rc_nanos_since_boot();
    loop_rate.sleep();
    ros::spinOnce();
	//spinner.stop();
    //count = ++;
  }
  // subscribe joy joy topic and start callback function
  return 0;
    //count = ++;
}

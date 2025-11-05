/**
 * this node is the core controller. 
 1  It set the commanded heading angle at the start instant or from the set heading angle topic. 
 2  It set the c_heading to pid controller to get diff_rpm to adjust the rpm of the motors to track the c_heading.
 3  It send out c_rpm to the pwm output node to activate to motors.
 4  It has turn function, in which the rpm of each motor is set at a signed velue.
 5  It receives command from the joy. If the command is stop, it sends "0" c_rpm and "3" reverse signal. 
 6  When the node runs, the red light will flash.
 Jianguang 01/25/2020
 */

#include "ros/ros.h"
//#include <std_msgs/Float64MultiArray.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include <sensor_msgs/Joy.h>
//#include "sensor_msgs/Imu.h"
extern "C" {
#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h>  // for atoi() and exit()
#include <rc/time.h>
#include <rc/adc.h>
#include <signal.h>
}
float rpm1,rpm2,rpm3,rpm4,leftturn_portrpm,leftturn_stdbrpm,rightturn_stdbrpm,rightturn_portrpm;
int leftturn_flag=0,rightturn_flag=0,start_flag=0,port_reverse=0,stdb_reverse=0,stop_flag=0;
int balance=50;//for compensation of imbalance of the two motors
float portrpm = 0.0;
float portrpm_total = 0.0;//sum of the base part and control part
//float test=100.0;
float stdbrpm = 0.0;
float stdbrpm_total = 0.0;
float set_heading_angle = 0.0;
int forward_gear = 0;
int pre_forward_gear=0;
float baserpm=0;
int pre_b[4];
int count=0;
// get in to the callback when message is received
//Totally 4 gears. Y:add speed, A: reduce speed, X: left turn, B: right turn
void rpmCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if(count>10&&start_flag==1){   //keep the value 1 of start_flag for a few loops
	  start_flag=0;
  }
  pre_forward_gear=forward_gear;
  ///velocity adjust
  if (pre_b[3]==0 && joy->buttons[3]==1) //activate at the instance when the button is pushed.
  {
	stop_flag=0;
	rightturn_flag=0;
    leftturn_flag=0;
    forward_gear ++;
    if(forward_gear>4)
    {
      forward_gear=4;
    }
  }
  if (pre_b[1]==0 && joy->buttons[1]==1)
  {
    rightturn_flag=0;
    leftturn_flag=0;
    forward_gear --;
    if (forward_gear<0)
    {
      forward_gear=0;
    }
  }
  if (pre_b[0]==0 &&joy->buttons[0]==1)
  {
    leftturn_flag=1;
    stop_flag=0;
    forward_gear=0;
    rightturn_flag=0;
  }
  if (pre_b[2]==0 &&joy->buttons[2]==1)
  {
    rightturn_flag=1;
    stop_flag=0;
    forward_gear=0;
    leftturn_flag=0;
  }
  ///velocity adjust
  //ROS_INFO("forward:%d,leftturn_flag:%d,rightturn_flag:d%",forward_gear,leftturn_flag,rightturn_flag);
  switch (forward_gear) {  //set the basic rpm
    case 0:
	if(leftturn_flag==0&&rightturn_flag==0){
	stop_flag=1;  //in this case, pwms are set to 0;
	}
	break;
    case 1:
    baserpm=rpm1;
    if(pre_forward_gear==0)
    {
      start_flag=1;//start signal
      count=0;
    }
    break;
    case 2:
    baserpm=rpm2;
    break;
    case 3:
    baserpm=rpm3;
    break;
    case 4:
    baserpm=rpm4;
    break;
  }
  pre_b[0] = joy->buttons[0];
  pre_b[1] = joy->buttons[1];
  pre_b[2] = joy->buttons[2];
  pre_b[3] = joy->buttons[3];
  count++;
  //ROS_INFO("count: [%d]", count);
  //ROS_INFO("forward_gear: [%d]", forward_gear);
  //ROS_INFO("leftturn_flag: [%d]", leftturn_flag);
  //ROS_INFO("rightturn_flag: [%d]", rightturn_flag);
}

void controlCallback(const std_msgs::Float64::ConstPtr& control_msg_)
{
  if (forward_gear==0 && rightturn_flag==0 && leftturn_flag==0)
  {
	  portrpm_total=stdbrpm_total=portrpm=stdbrpm=0;
  }
  if (forward_gear>0)
  {
    //portrpm = portrpm - control_msg_->data;
    //stdbrpm = stdbrpm + control_msg_->data;
    portrpm = portrpm - control_msg_->data;
    stdbrpm = stdbrpm + control_msg_->data;
    if(portrpm<-300.0)
    {
  	  portrpm=-300.0;
    }
    if(portrpm>300.0)
    {
  	  portrpm=300.0;
    }
    if(stdbrpm<-300.0)
    {
  	  stdbrpm=-300.0;
    }
    if(stdbrpm>300.0)
    {
  	  stdbrpm=300.0;
    }
    portrpm_total=portrpm+baserpm+balance;
    stdbrpm_total=stdbrpm+baserpm;
    //set the limit of rpm
    if(portrpm_total<0.0)
    {
  	  portrpm_total=0.0;
    }
    if(portrpm_total>1600.0)
    {
  	  portrpm_total=1600.0;
    }
    if(stdbrpm_total<0.0)
    {
  	  stdbrpm_total=0.0;
    }
    if(stdbrpm_total>1600.0)
    {
  	  stdbrpm_total=1600.0;
    }
  }
  if (leftturn_flag==1)
  {
    portrpm_total=leftturn_portrpm;
    stdbrpm_total=leftturn_stdbrpm;
  }
  if (rightturn_flag==1)
  {
    portrpm_total=rightturn_portrpm;
    stdbrpm_total=rightturn_stdbrpm;
  }
  //test=test+control_msg_->data;
  //ROS_INFO("test: [%f]", test);
  //ROS_INFO("left_rpm: [%f]", portrpm_total);
  //ROS_INFO("right_rpm: [%f]", stdbrpm_total);
}

void headingCallback(const std_msgs::Float64::ConstPtr& heading_angle)
{
	//set the desire heading angle to the heading angle at the start point.
  if (start_flag == 1)
  {
    set_heading_angle = heading_angle->data;
  }
}

void setheadingCallback(const std_msgs::Float64::ConstPtr& set_heading)
{
    //set the desire heading angle from topic information.
    set_heading_angle = set_heading->data;

}

void MySigintHandler(int sig)
{
  ROS_INFO("shutting down!");
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "heading");
  std_msgs::Float64 c_portrpm_msg, c_stdbrpm_msg, c_heading_msg;
  std_msgs::Int8 stdb_reverse_msg, port_reverse_msg, light_msg;
  ros::NodeHandle heading;
  ros::NodeHandle node_priv("~");
  node_priv.getParam("rpm1", rpm1);
  node_priv.getParam("rpm2", rpm2);
  node_priv.getParam("rpm3", rpm3);
  node_priv.getParam("rpm4", rpm4);
  node_priv.getParam("leftturn_portrpm", leftturn_portrpm);
  node_priv.getParam("leftturn_stdbrpm", leftturn_stdbrpm);
  node_priv.getParam("rightturn_portrpm", rightturn_portrpm);
  node_priv.getParam("rightturn_stdbrpm", rightturn_stdbrpm);
  ros::Publisher pub1 = heading.advertise<std_msgs::Float64>("c_portrpm", 2);
  ros::Publisher pub2 = heading.advertise<std_msgs::Float64>("c_stdbrpm", 2);
  ros::Publisher pub3 = heading.advertise<std_msgs::Float64>("c_heading", 2);
  ros::Publisher pub4 = heading.advertise<std_msgs::Int8>("port_reverse", 2);
  ros::Publisher pub5 = heading.advertise<std_msgs::Int8>("stdb_reverse", 2);
  ros::Publisher pub6 = heading.advertise<std_msgs::Int8>("/light", 2);
  ros::Rate loop_rate(10);
  signal(SIGINT, MySigintHandler);
  ros::Subscriber sub3 = heading.subscribe("m_heading", 2, headingCallback);   // from imu
  ros::Subscriber sub1 = heading.subscribe("diff_rpm", 2, controlCallback);  // from heading controller
  ros::Subscriber sub2 = heading.subscribe("/joy", 2, rpmCallback);       // from joymessage node
  ros::Subscriber sub4 = heading.subscribe("set_heading", 2, setheadingCallback);  // from path planner
  while (ros::ok())
  {
    port_reverse=0;
    stdb_reverse=0;
	  if(portrpm_total<0){
      port_reverse=1;
      portrpm_total=-portrpm_total;
    }
    if(stdbrpm_total<0){
      stdb_reverse=1;
      stdbrpm_total=-stdbrpm_total;
    }
    if(stop_flag==1){
    	port_reverse=stdb_reverse=3;
    	portrpm_total=stdbrpm_total=0;
    }
    c_portrpm_msg.data = portrpm_total;
    c_stdbrpm_msg.data = stdbrpm_total;
    c_heading_msg.data = set_heading_angle;
    stdb_reverse_msg.data = stdb_reverse;
    port_reverse_msg.data = port_reverse;
	light_msg.data = 1;
    //ROS_INFO("stdb_reverse: [%d]", stdb_reverse);
    pub1.publish(c_portrpm_msg);   // to left controller
    pub2.publish(c_stdbrpm_msg);  // to right controller
    pub4.publish(port_reverse_msg);
    pub5.publish(stdb_reverse_msg);
	pub6.publish(light_msg);
    if(set_heading_angle!=0.0)
    {
    pub3.publish(c_heading_msg);   // to heading controller,only when the angle is set.
    }
    ros::spinOnce();
    loop_rate.sleep();
    //count = ++;
  }
  return 0;
}

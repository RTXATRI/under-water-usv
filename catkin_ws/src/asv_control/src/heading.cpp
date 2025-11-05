/**
 * subscribe std_msgs::Joy and control the left motor
 sleep is required to generate pwm signal. To avoid interference between two sigals, seperate nodes are used to deal
 with two motors
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
float rpm1,rpm2,rpm3,rpm4,l_leftrpm,l_rightrpm,r_rightrpm,r_leftrpm;
int leftTurn=0,rightTurn=0,Start=0,l_reverse=0,r_reverse=0,stopFlag=0;
int balance=50;//for compensation of imbalance of the two motors
float leftrpm = 0.0;
float leftrpm_total = 0.0;//sum of the base part and control part
//float test=100.0;
float rightrpm = 0.0;
float rightrpm_total = 0.0;
float set_heading_angle = 0.0;
int Forward = 0;
int pre_Forward=0;
float baserpm=0;
int pre_b[4];
int count=0;
// get in to the callback when message is received
//Totally 4 gears. Y:add speed, A: reduce speed, X: left turn, B: right turn
void rpmCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if(count>10&&Start==1){   //keep the value 1 of Start for a few loops
	  Start=0;
  }
  pre_Forward=Forward;
  ///velocity adjust
  if (pre_b[3]==0 && joy->buttons[3]==1) //activate at the instance when the button is pushed.
  {
	stopFlag=0;
	rightTurn=0;
    leftTurn=0;
    Forward ++;
    if(Forward>4)
    {
      Forward=4;
    }
  }
  if (pre_b[1]==0 && joy->buttons[1]==1)
  {
    rightTurn=0;
    leftTurn=0;
    Forward --;
    if (Forward<0)
    {
      Forward=0;
    }
  }
  if (pre_b[0]==0 &&joy->buttons[0]==1)
  {
    leftTurn=1;
    stopFlag=0;
    Forward=0;
    rightTurn=0;
  }
  if (pre_b[2]==0 &&joy->buttons[2]==1)
  {
    rightTurn=1;
    stopFlag=0;
    Forward=0;
    leftTurn=0;
  }
  ///velocity adjust
  //ROS_INFO("forward:%d,leftTurn:%d,rightTurn:d%",Forward,leftTurn,rightTurn);
  switch (Forward) {  //set the basic rpm
    case 0:
    //ROS_INFO("bbbbbbbbbbbbbbbbbbb");
	if(leftTurn==0&&rightTurn==0){
	stopFlag=1;  //in this case, pwms are set to 0;
	//ROS_INFO("aaaaaaaaaaaaaaaaaaaaaaaa");
	}
	break;
    case 1:
    baserpm=rpm1;
    if(pre_Forward==0)
    {
      Start=1;//start signal
      count=0;
      //ROS_INFO("aaaaaa");
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
  //turn

  pre_b[0] = joy->buttons[0];
  pre_b[1] = joy->buttons[1];
  pre_b[2] = joy->buttons[2];
  pre_b[3] = joy->buttons[3];
  count++;
  ROS_INFO("count: [%d]", count);
  ROS_INFO("Forward: [%d]", Forward);
  ROS_INFO("leftTurn: [%d]", leftTurn);
  ROS_INFO("rightTurn: [%d]", rightTurn);
}

void controlCallback(const std_msgs::Float64::ConstPtr& control_msg_)
{
  if (Forward==0 && rightTurn==0 && leftTurn==0)
  {
	  leftrpm_total=rightrpm_total=leftrpm=rightrpm=0;
  }
  if (Forward>0)
  {
	//ROS_INFO("aaaaaa");
    //leftrpm = leftrpm - control_msg_->data;
    //rightrpm = rightrpm + control_msg_->data;
    leftrpm = leftrpm - control_msg_->data;
    rightrpm = rightrpm + control_msg_->data;
    if(leftrpm<-300.0)
    {
  	  leftrpm=-300.0;
    }
    if(leftrpm>300.0)
    {
  	  leftrpm=300.0;
    }
    if(rightrpm<-300.0)
    {
  	  rightrpm=-300.0;
    }
    if(rightrpm>300.0)
    {
  	  rightrpm=300.0;
    }
    leftrpm_total=leftrpm+baserpm+balance;
    rightrpm_total=rightrpm+baserpm;
    //set the limit of rpm
    if(leftrpm_total<0.0)
    {
  	  leftrpm_total=0.0;
    }
    if(leftrpm_total>1600.0)
    {
  	  leftrpm_total=1600.0;
    }
    if(rightrpm_total<0.0)
    {
  	  rightrpm_total=0.0;
    }
    if(rightrpm_total>1600.0)
    {
  	  rightrpm_total=1600.0;
    }
  }
  if (leftTurn==1)
  {
    leftrpm_total=l_leftrpm;
    rightrpm_total=l_rightrpm;
  }
  if (rightTurn==1)
  {
    leftrpm_total=r_leftrpm;
    rightrpm_total=r_rightrpm;
  }
  //test=test+control_msg_->data;
  //ROS_INFO("test: [%f]", test);
  //ROS_INFO("left_rpm: [%f]", leftrpm_total);
  //ROS_INFO("right_rpm: [%f]", rightrpm_total);
}

void headingCallback(const std_msgs::Float64::ConstPtr& heading_angle)
{
  if (Start == 1)
  {
    set_heading_angle = heading_angle->data;
  }
  //ROS_INFO("Start: [%d]", Start);
  //ROS_INFO("d_heading: [%f]", set_heading_angle);
}

void setheadingCallback(const std_msgs::Float64::ConstPtr& set_heading)
{

    set_heading_angle = set_heading->data;

  //ROS_INFO("Start: [%d]", Start);
  //ROS_INFO("d_heading: [%f]", set_heading_angle);
}

void MySigintHandler(int sig)
{
  //这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
  ROS_INFO("shutting down!");
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "headingcontrol");
  std_msgs::Float64 d_leftrpm_msg, d_rightrpm_msg, d_heading_msg;
  std_msgs::Int8 r_reverse_msg, l_reverse_msg, light_msg;
  // ros::init(argc, argv, "rightmotor");
  ros::NodeHandle heading;
  ros::NodeHandle node_priv("~");
 // float rpm1,rpm2,rpm3,rpm4,l_leftrpm,l_rightrpm,r_rightrpm,r_leftrpm;
  node_priv.getParam("rpm1", rpm1);
  node_priv.getParam("rpm2", rpm2);
  node_priv.getParam("rpm3", rpm3);
  node_priv.getParam("rpm4", rpm4);
  node_priv.getParam("l_leftrpm", l_leftrpm);
  node_priv.getParam("l_rightrpm", l_rightrpm);
  node_priv.getParam("r_leftrpm", r_leftrpm);
  node_priv.getParam("r_rightrpm", r_rightrpm);
  ros::Publisher pub1 = heading.advertise<std_msgs::Float64>("d_leftrpm", 2);
  ros::Publisher pub2 = heading.advertise<std_msgs::Float64>("d_rightrpm", 2);
  ros::Publisher pub3 = heading.advertise<std_msgs::Float64>("d_heading", 2);
  ros::Publisher pub4 = heading.advertise<std_msgs::Int8>("l_reverse", 2);
  ros::Publisher pub5 = heading.advertise<std_msgs::Int8>("r_reverse", 2);
  ros::Publisher pub6 = heading.advertise<std_msgs::Int8>("light", 2);
  ros::Rate loop_rate(10);
  signal(SIGINT, MySigintHandler);
  ros::Subscriber sub3 = heading.subscribe("heading_angle", 2, headingCallback);   // from imu
  ros::Subscriber sub1 = heading.subscribe("diff_rpm", 2, controlCallback);  // from heading controller
  ros::Subscriber sub2 = heading.subscribe("joy", 2, rpmCallback);       // from joymessage node
  ros::Subscriber sub4 = heading.subscribe("set_heading", 2, setheadingCallback);  // from path planner
  //int count = 1;
  while (ros::ok())
  {
    // subscribe joy joy topic and start callback function  此处名空间要处理清楚
    l_reverse=0;
    r_reverse=0;
	  if(leftrpm_total<0){
      l_reverse=1;
      leftrpm_total=-leftrpm_total;
    }
    if(rightrpm_total<0){
      r_reverse=1;
      rightrpm_total=-rightrpm_total;
    }
    if(stopFlag==1){
    	l_reverse=r_reverse=3;
    	leftrpm_total=rightrpm_total=0;
    }
    d_leftrpm_msg.data = leftrpm_total;
    d_rightrpm_msg.data = rightrpm_total;
    d_heading_msg.data = set_heading_angle;
    r_reverse_msg.data = r_reverse;
    l_reverse_msg.data = l_reverse;
	light_msg.data = 1;
    //ROS_INFO("r_reverse: [%d]", r_reverse);
    pub1.publish(d_leftrpm_msg);   // to left controller
    pub2.publish(d_rightrpm_msg);  // to right controller
    pub4.publish(l_reverse_msg);
    pub5.publish(r_reverse_msg);
	pub6.publish(light_msg);
    if(set_heading_angle!=0.0)
    {
    pub3.publish(d_heading_msg);   // to heading controller,only when the angle is set.
    }
    ros::spinOnce();
    loop_rate.sleep();
    //count = ++;
  }
  return 0;
}

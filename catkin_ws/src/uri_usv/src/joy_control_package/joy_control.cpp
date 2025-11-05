/**This program use controls the PWM based on the Joy node
Jan-09-2020 Mingxi This node sets the PWM to two motors
										changed the ros node Subscriber
										directly subscribe to joy message
*/
//include c language libraries
extern "C" {
#include "rc_usefulincludes.h"
}
extern "C" {
#include "roboticscape.h"
}
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
#include <boost/thread.hpp>

//include ros
#include "ros/ros.h"
//include ros msgs tyeps
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/Joy.h"

float c_port_duty = 0,c_stdb_duty=0;
int running = 0;

void Motor_Callback (const sensor_msgs::Joy::ConstPtr& msg)
{
	//check for button press for motor driver relay
	if (msg->buttons[9] == 1)
  {
    rc_gpio_set_value(3, 1, 1);  // turn on the relay
  }
  if (msg->buttons[8] == 1)
  {
    rc_gpio_set_value(3, 1, 0);  // turn off the relay
  }
	//left stick
	if(msg->axes[1]>0){
		//set direction
		 rc_gpio_set_value(1, 25, 0); //forward
	}
	if(msg->axes[1]<0){
		rc_gpio_set_vale(1, 25, 1); //reverse
	}
	//right stick
	if(msg->axes[3]>0){
		//set direction
		 rc_gpio_set_value(1, 17, 0); //forward
	}
	if(msg->axes[3]<0){
		rc_gpio_set_vale(1, 17, 1); //reverse
	}
	c_port_duty = fabs( msg->axes[1]);	//set global duty cycle varilables
	c_stdb_duty = fabs( msg->axes[3]);
}

// interrupt handler to catch ctrl-c
void MySigintHandler(int sig)
{
	//shutting down
  ROS_INFO("shutting down!");
	rc_servo_power_rail_en(0);
  rc_servo_cleanup();
  rc_gpio_cleanup(1, 25);
  rc_gpio_cleanup(1, 17);
	rc_gpio_cleanup(3, 17);
	running = 0;
  ros::shutdown();
}

int main(int argc, char** argv)
{
  // initiate pwm;
  if(rc_servo_init()) return -1;
  usleep(100000);  //there will be problems if there is no delay here
  // initiate 2 gpios
	//enable the gpio controls the direction of port motor
  if (rc_gpio_init(1, 25, GPIOHANDLE_REQUEST_OUTPUT) == -1)
  {
    printf("rc_gpio_init failed\n");
    return -1;
  }
	//enable the gpio controls the starboard motor
  if (rc_gpio_init(1, 17, GPIOHANDLE_REQUEST_OUTPUT) == -1)
  {
    printf("rc_gpio_init failed\n");
    return -1;
  }
	//enable the IO which controls the motor driver relay
	if (rc_gpio_init(3, 1, GPIOHANDLE_REQUEST_OUTPUT) == -1)
	{
		printf("rc_gpio_init failed\n");
		return -1;
	}
  //if ctrl-c we shut down things
	signal(SIGINT, MySigintHandler);
  running = 1;

	//ROS node stuff
  ros::init(argc, argv, "pwm");
	//create a private node handle to get parameters
  ros::NodeHandle np("~");
	int frequency;
	np.param("frequency",frequency, 100);
	int period = 1000000 / frequency; // use the frequency to set the PWM period
  //node_priv.getParam("frequency", frequency);
  ros::NodeHandle nh;
  ros::Rate loop_rate(frequency);

	//Remote control mode
	ros::Subscriber sub1 = nh.subscribe("/joy", 2, Motor_Callback);
	ros::Publisher port_pwm_pub = nh.advertise<std_msgs::Float64>("/port_motor/c_pwm", 2);
  ros::Publisher stdb_pwm_pub = nh.advertise<std_msgs::Float64>("/stdb_motor/c_pwm", 2);
	std_msgs::Float64 c_pwm_port, c_pwm_stdb;

  // ros::init(argc, argv, "leftmotor");
  while (ros::ok() & running)
  {
	  // send pwm adjust command
		rc_servo_send_pulse_us(1, (int)(c_port_duty * period));
		rc_servo_send_pulse_us(2, (int)(c_stdb_duty * period));
		//publish the duty cycle to ros topics
		c_pwm_port.data = c_port_duty;
		c_pwm_stdb.data = c_stdb_duty;
		port_pwm_pub.publish(c_pwm_port);
		port_pwm_pub.publish(c_pwm_stdb);

    ros::spinOnce();
    loop_rate.sleep();
    //count = ++;
  }
  return 0;
}

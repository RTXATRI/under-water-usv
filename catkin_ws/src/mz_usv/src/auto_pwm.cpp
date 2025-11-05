/**This program use controls the PWM based on the Joy node
	Jan-09-2020 Mingxi 	This node sets the PWM to two motors
						changed the ros node Subscriber
						directly subscribe to joy message

	Aug-12-2020 Mingxi	heartbeat added. This node will stop if there are no heatbeat signals in 4s.

	May-2020 			control the usv with the right joy alone. remote mode 2 if "X"is pressed.

	May-2022 			publish heading if "Y"is pressed.

	Jul-2025			Removed all unused code, rewrited the driver code, only retain the basic remote control motor output function
*/

//include c language libraries
extern "C" {
#include "roboticscape.h" 
}
#include <cstdlib>
#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"  
#include "std_msgs/Int8.h" 
#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
#include "sensor_msgs/Joy.h"

typedef enum {NONE_MODE, MANUAL_MODE, STABILIZE_MODE, ALTITUDE_MODE, POSITION_MODE, DOCK_MODE} USV_STATE;
typedef enum {NORMAL_MODE, AMERICAN_MODE} JOY_STATE;

#define INIT_PULSE_WIDTH  	1500
#define MAX_PW_INCREMENT  	500
#define MAX_PW_INCREMENT_XY 400
#define MAX_PW_INCREMENT_Z  300
#define DOCK_PW				100
#define TURN_P  			0.5

int c_port_cmd = 0, c_stdb_cmd = 0, c_up_cmd1 = 0, c_up_cmd2 = 0, c_dowm_cmd1 = 0, c_dowm_cmd2 = 0;
int heartbeat = 0, count = 0, running = 0;
int c_pid_x = 0, c_pid_y = 0, c_pid_z = 0, c_pid_d = 0;
int button_buf[6] = {0};

std_msgs::Float64 c_port_duty, c_stdb_duty, c_updowm_duty;

std_msgs::Float64 set_roll, set_pitch;

std_msgs::Int8 usv_state_pub, joy_mode_pub;

USV_STATE STATE = NONE_MODE, STATE_tmp = NONE_MODE;
JOY_STATE joy_mode = NORMAL_MODE, joy_mode_tmp = AMERICAN_MODE;

void esc_wakeup(void);
int limit_pwm_PID (int pwm);
void MySigintHandler(int sig);

void Joy_Callback (const sensor_msgs::Joy::ConstPtr& msg);
void heartbeatCallback(const std_msgs::Int8::ConstPtr& msg);
void Motor_X_Callback(const std_msgs::Float64::ConstPtr& msg);
void Motor_Y_Callback(const std_msgs::Float64::ConstPtr& msg);
void Motor_Z_Callback(const std_msgs::Float64::ConstPtr& msg);
void Motor_D_Callback(const std_msgs::Float64::ConstPtr& msg);

int main(int argc, char** argv)
{
  	usleep(100000);

	if(rc_servo_init()) 
		return -1;
    if(rc_servo_set_esc_range(RC_ESC_DEFAULT_MIN_US,RC_ESC_DEFAULT_MAX_US)) 
		return -1;
    rc_servo_power_rail_en(0);

	signal(SIGINT, MySigintHandler);
  	running = 1;
	usv_state_pub.data = 0;

	//ROS node stuff
  	ros::init(argc, argv, "auto_pwm");

  	ros::NodeHandle nh;

	//Remote control mode
	ros::Subscriber sub1 = nh.subscribe("/joy", 2, Joy_Callback);
 	ros::Subscriber sub2 = nh.subscribe("/PID/X", 2, Motor_X_Callback);
 	ros::Subscriber sub3 = nh.subscribe("/PID/Y", 2, Motor_Y_Callback);
	ros::Subscriber sub4 = nh.subscribe("/PID/Z", 2, Motor_Z_Callback); 
 	ros::Subscriber sub5 = nh.subscribe("/PID/depth", 2, Motor_D_Callback); 
	ros::Subscriber sub6 = nh.subscribe("/heartbeat", 2, heartbeatCallback);

	ros::Publisher ptich_set_pub = nh.advertise<std_msgs::Float64>("/under_usv/joy/ptich_set", 2);
  	ros::Publisher roll_set_pub = nh.advertise<std_msgs::Float64>("/under_usv/joy/roll_set", 2);
	ros::Publisher mode_pub = nh.advertise<std_msgs::Int8>("/under_usv/usv_mode",2);
	ros::Publisher joy_pub = nh.advertise<std_msgs::Int8>("/under_usv/joy_mode",2);
	ros::Publisher c_heading_pub = nh.advertise<std_msgs::Float64>("/control/heading",2); 

	ros::Rate loop_rate(50);

	esc_wakeup();
	
  	while (ros::ok() && running) 
  	{
		usv_state_pub.data = STATE;
		joy_mode_pub.data = joy_mode;

		mode_pub.publish(usv_state_pub);
		joy_pub.publish(joy_mode_pub);
		ptich_set_pub.publish(set_pitch);
		roll_set_pub.publish(set_roll);



		switch (STATE) 
		{
    		case NONE_MODE:
				// XY axis thruster
				c_stdb_cmd  = INIT_PULSE_WIDTH;
      			c_port_cmd  = INIT_PULSE_WIDTH;
				// Z  axis thruster
				c_up_cmd1   = INIT_PULSE_WIDTH;
				c_up_cmd2   = INIT_PULSE_WIDTH;
				c_dowm_cmd1 = INIT_PULSE_WIDTH;
				c_dowm_cmd2 = INIT_PULSE_WIDTH;
        		break;
    		case MANUAL_MODE:
				// XY axis thruster
				c_stdb_cmd  = INIT_PULSE_WIDTH + (int)(c_stdb_duty.data * MAX_PW_INCREMENT);
	  			c_port_cmd  = INIT_PULSE_WIDTH + (int)(c_port_duty.data * MAX_PW_INCREMENT);
				// Z  axis thruster
				c_up_cmd1	= INIT_PULSE_WIDTH + (int)(c_updowm_duty.data * MAX_PW_INCREMENT); // Motor 2
				c_up_cmd2	= INIT_PULSE_WIDTH + (int)(c_updowm_duty.data * MAX_PW_INCREMENT); // Motor 5
		 		c_dowm_cmd1	= INIT_PULSE_WIDTH + (int)(c_updowm_duty.data * MAX_PW_INCREMENT); // Motor 4
		 		c_dowm_cmd2	= INIT_PULSE_WIDTH + (int)(c_updowm_duty.data * MAX_PW_INCREMENT); // Motor 6
        		break;
    		case STABILIZE_MODE:
				// XY axis thruster
				c_stdb_cmd  = INIT_PULSE_WIDTH + (int)(c_stdb_duty.data * MAX_PW_INCREMENT);
	  			c_port_cmd  = INIT_PULSE_WIDTH + (int)(c_port_duty.data * MAX_PW_INCREMENT);
				// Z  axis thruster
				c_up_cmd1	= INIT_PULSE_WIDTH + (int)(c_updowm_duty.data * MAX_PW_INCREMENT_Z) - c_pid_x + c_pid_y; // Motor 2
				c_up_cmd2	= INIT_PULSE_WIDTH + (int)(c_updowm_duty.data * MAX_PW_INCREMENT_Z) + c_pid_x - c_pid_y; // Motor 5
		 		c_dowm_cmd1	= INIT_PULSE_WIDTH + (int)(c_updowm_duty.data * MAX_PW_INCREMENT_Z) + c_pid_x + c_pid_y; // Motor 4
		 		c_dowm_cmd2	= INIT_PULSE_WIDTH + (int)(c_updowm_duty.data * MAX_PW_INCREMENT_Z) - c_pid_x - c_pid_y; // Motor 6
        		break;
			case ALTITUDE_MODE:
				// XY axis thruster
				c_stdb_cmd  = INIT_PULSE_WIDTH + (int)(c_stdb_duty.data * MAX_PW_INCREMENT_XY);
	  			c_port_cmd  = INIT_PULSE_WIDTH + (int)(c_port_duty.data * MAX_PW_INCREMENT_XY);
				// Z  axis thruster
				c_up_cmd1	= INIT_PULSE_WIDTH + (int)(c_updowm_duty.data * MAX_PW_INCREMENT_Z) - c_pid_x + c_pid_y + c_pid_d; // Motor 2
				c_up_cmd2	= INIT_PULSE_WIDTH + (int)(c_updowm_duty.data * MAX_PW_INCREMENT_Z) + c_pid_x - c_pid_y + c_pid_d; // Motor 5
		 		c_dowm_cmd1	= INIT_PULSE_WIDTH + (int)(c_updowm_duty.data * MAX_PW_INCREMENT_Z) + c_pid_x + c_pid_y + c_pid_d; // Motor 4
		 		c_dowm_cmd2	= INIT_PULSE_WIDTH + (int)(c_updowm_duty.data * MAX_PW_INCREMENT_Z) - c_pid_x - c_pid_y + c_pid_d; // Motor 6
        		break;
			case POSITION_MODE:
				switch (joy_mode) 
				{
					case NORMAL_MODE:
						// XY axis thruster
						c_stdb_cmd  = INIT_PULSE_WIDTH + (int)(c_stdb_duty.data * MAX_PW_INCREMENT_XY) + c_pid_z;
	  					c_port_cmd  = INIT_PULSE_WIDTH + (int)(c_port_duty.data * MAX_PW_INCREMENT_XY) - c_pid_z;
						// Z  axis thruster
						c_up_cmd1	= INIT_PULSE_WIDTH + (int)(c_updowm_duty.data * MAX_PW_INCREMENT_Z) - c_pid_x + c_pid_y + c_pid_d; // Motor 2
						c_up_cmd2	= INIT_PULSE_WIDTH + (int)(c_updowm_duty.data * MAX_PW_INCREMENT_Z) + c_pid_x - c_pid_y + c_pid_d; // Motor 5
		 				c_dowm_cmd1	= INIT_PULSE_WIDTH + (int)(c_updowm_duty.data * MAX_PW_INCREMENT_Z) + c_pid_x + c_pid_y + c_pid_d; // Motor 4
		 				c_dowm_cmd2	= INIT_PULSE_WIDTH + (int)(c_updowm_duty.data * MAX_PW_INCREMENT_Z) - c_pid_x - c_pid_y + c_pid_d; // Motor 6		
        				break;
					case AMERICAN_MODE:
						c_stdb_cmd  = INIT_PULSE_WIDTH + (int)(c_stdb_duty.data * MAX_PW_INCREMENT_XY) + c_pid_z;
	  					c_port_cmd  = INIT_PULSE_WIDTH + (int)(c_port_duty.data * MAX_PW_INCREMENT_XY) - c_pid_z;
						// Z  axis thruster
						c_up_cmd1	= INIT_PULSE_WIDTH + (int)(c_updowm_duty.data * ( 2 * MAX_PW_INCREMENT_Z / 3 )) - c_pid_x + c_pid_y + c_pid_d; // Motor 2
						c_up_cmd2	= INIT_PULSE_WIDTH + (int)(c_updowm_duty.data * ( 2 * MAX_PW_INCREMENT_Z / 3 )) + c_pid_x - c_pid_y + c_pid_d; // Motor 5
		 				c_dowm_cmd1	= INIT_PULSE_WIDTH + (int)(c_updowm_duty.data * ( 2 * MAX_PW_INCREMENT_Z / 3 )) + c_pid_x + c_pid_y + c_pid_d; // Motor 4
		 				c_dowm_cmd2	= INIT_PULSE_WIDTH + (int)(c_updowm_duty.data * ( 2 * MAX_PW_INCREMENT_Z / 3 )) - c_pid_x - c_pid_y + c_pid_d; // Motor 6
        				break;
    				default:
						ROS_INFO("[Error]: Unkown joy mode!");
        				break;
				}
        		break;
			case DOCK_MODE:
				// XY axis thruster
				// c_stdb_cmd = INIT_PULSE_WIDTH + (int)(c_stdb_duty.data * MAX_PW_INCREMENT_XY);
	  			// c_port_cmd = INIT_PULSE_WIDTH + (int)(c_port_duty.data * MAX_PW_INCREMENT_XY);
				c_stdb_cmd  = INIT_PULSE_WIDTH + (int)(c_stdb_duty.data * MAX_PW_INCREMENT_XY) + c_pid_z;
	  			c_port_cmd  = INIT_PULSE_WIDTH + (int)(c_port_duty.data * MAX_PW_INCREMENT_XY) - c_pid_z;
				// Z  axis thruster
				c_up_cmd1	= INIT_PULSE_WIDTH + DOCK_PW - c_pid_x + c_pid_y ; // Motor 2
				c_up_cmd2	= INIT_PULSE_WIDTH + DOCK_PW + c_pid_x - c_pid_y ; // Motor 5
		 		c_dowm_cmd1	= INIT_PULSE_WIDTH + DOCK_PW + c_pid_x + c_pid_y ; // Motor 4
		 		c_dowm_cmd2	= INIT_PULSE_WIDTH + DOCK_PW - c_pid_x - c_pid_y ; // Motor 6
        		break;	
    		default:
				// XY axis thruster
				c_stdb_cmd  = INIT_PULSE_WIDTH;
      			c_port_cmd  = INIT_PULSE_WIDTH;
				// Z  axis thruster
				c_up_cmd1   = INIT_PULSE_WIDTH;
				c_up_cmd2   = INIT_PULSE_WIDTH;
				c_dowm_cmd1 = INIT_PULSE_WIDTH;
				c_dowm_cmd2 = INIT_PULSE_WIDTH;
        		break;
		}
		c_stdb_cmd  = limit_pwm_PID(c_stdb_cmd);
      	c_port_cmd  = limit_pwm_PID(c_port_cmd); 
		c_up_cmd1   = limit_pwm_PID(c_up_cmd1);
		c_up_cmd2   = limit_pwm_PID(c_up_cmd2);
		c_dowm_cmd1 = limit_pwm_PID(c_dowm_cmd1);
		c_dowm_cmd2 = limit_pwm_PID(c_dowm_cmd2);

		// XY axis thruster
		rc_servo_send_pulse_us(1, c_port_cmd);	// port side
		rc_servo_send_pulse_us(3, c_stdb_cmd);	// starboard side
		// Z  axis thruster
   		rc_servo_send_pulse_us(2, c_up_cmd1);	// c_up_cmd1
		rc_servo_send_pulse_us(4, c_dowm_cmd1); // c_dowm_cmd1
	 	rc_servo_send_pulse_us(5, c_up_cmd2);	// c_up_cmd2
	 	rc_servo_send_pulse_us(6, c_dowm_cmd2);	// c_dowm_cmd2

		// The following code is used to solve the problem of normal program shutdown,
		// Such as the motor need be stopping when the program exits
		count++;
		if(count>200)
		{
		  	if(heartbeat<1)
			{
			  ros::shutdown();
		  	}
		  	count=0;
		  	heartbeat=0;
		}
	    ros::spinOnce();
    	loop_rate.sleep();
  	}
  	return 0;
}

void Joy_Callback (const sensor_msgs::Joy::ConstPtr& msg)
{
	// button[0]
	if((msg->buttons[0] == 1) && (button_buf[0] != msg->buttons[0])) 
	{
		switch (STATE) 
		{
    		case NONE_MODE:
				STATE = MANUAL_MODE;
				// ROS_INFO("MODE: MANUAL_MODE(%d)", STATE);
        		break;
    		case MANUAL_MODE:
 				STATE = STABILIZE_MODE;
				// ROS_INFO("MODE: STABILIZE_MODE(%d)", STATE);
        		break;
    		case STABILIZE_MODE:
				STATE = ALTITUDE_MODE;
				// ROS_INFO("MODE: POSITION_MODE(%d)", STATE);
        		break;
			case ALTITUDE_MODE:
				STATE = POSITION_MODE;
				// ROS_INFO("MODE: POSITION_MODE(%d)", STATE);
        		break;
			case POSITION_MODE:
				STATE = DOCK_MODE;
				// ROS_INFO("MODE: DOCK_MODE(%d)", STATE);
        		break;
			case DOCK_MODE:
				STATE = MANUAL_MODE;
				// ROS_INFO("MODE: MANUAL_MODE(%d)", STATE);
        		break;
    		default:
				STATE = NONE_MODE;
				// ROS_INFO("MODE: NONE_MODE(%d)", STATE);
				break;
		}
		joy_mode = NORMAL_MODE;
		button_buf[0] = msg->buttons[0];
	}
	else if((msg->buttons[0] == 0) && (button_buf[0] != msg->buttons[0]))
	{
		button_buf[0] = msg->buttons[0] ;
	} 

	// button[1]
	if((msg->buttons[1] == 1) && (button_buf[1] != msg->buttons[1]))
	{

	}
	else if((msg->buttons[1] == 0) && (button_buf[1] != msg->buttons[1]))
	{
		button_buf[1] = msg->buttons[1] ;
	} 

	// button[2]
	if((msg->buttons[2] == 1) && (button_buf[2] != msg->buttons[2]))
	{
		button_buf[2] = msg->buttons[2];
		switch (STATE) 
		{
			case POSITION_MODE:
				if (joy_mode == NORMAL_MODE)
				{
					joy_mode = AMERICAN_MODE;
				}
				else if(joy_mode == AMERICAN_MODE)
				{
					joy_mode = NORMAL_MODE;
				}
        		break;
			case DOCK_MODE:
				if (joy_mode == NORMAL_MODE)
				{
					joy_mode = AMERICAN_MODE;
				}
				else if(joy_mode == AMERICAN_MODE)
				{
					joy_mode = NORMAL_MODE;
				}
        		break;
    		default:
				ROS_INFO("[Error]: Changing joy mode only be useful in POSITION_MODE or DOCK_MODE !");
				// button_buf[2] = 0;
        		break;
		}
	}
	else if((msg->buttons[2] == 0) && (button_buf[2] != msg->buttons[2]))
	{
		button_buf[2] = msg->buttons[2] ;
	} 

	// button[3]
	if((msg->buttons[3] == 1) && (button_buf[3] != msg->buttons[3]))
	{

	}
	else if((msg->buttons[3] == 0) && (button_buf[3] != msg->buttons[3]))
	{
		button_buf[3] = msg->buttons[3] ;
	} 

	if(msg->axes[0] > 0)
	{

	}

	if(msg->axes[1] > 0)
	{
		
	}

	if(msg->axes[2] > 0)
	{
		
	}

	if(msg->axes[3] > 0)
	{
		
	}

	switch (STATE) 
	{
    	case NONE_MODE:
			c_stdb_duty.data = 0.0;
			c_port_duty.data = 0.0;
			c_updowm_duty.data = 0.0;
        	break;
    	case MANUAL_MODE:
			c_stdb_duty.data = msg->axes[3]+TURN_P*msg->axes[2];
			c_port_duty.data = msg->axes[3]-TURN_P*msg->axes[2];

			c_updowm_duty.data = msg->axes[1];
        	break;
    	case STABILIZE_MODE:
			c_stdb_duty.data = msg->axes[3]+TURN_P*msg->axes[2];
			c_port_duty.data = msg->axes[3]-TURN_P*msg->axes[2];

			c_updowm_duty.data = msg->axes[1];
        	break;
		case ALTITUDE_MODE:
			c_stdb_duty.data = msg->axes[3]+TURN_P*msg->axes[2];
			c_port_duty.data = msg->axes[3]-TURN_P*msg->axes[2];

			c_updowm_duty.data = msg->axes[1];
        	break;
		case POSITION_MODE:
			switch (joy_mode) 
			{
				case NORMAL_MODE:
					c_stdb_duty.data = msg->axes[3] + TURN_P * msg->axes[2];
					c_port_duty.data = msg->axes[3] - TURN_P * msg->axes[2];

					c_updowm_duty.data = msg->axes[1];
        			break;
				case AMERICAN_MODE:
					c_stdb_duty.data =  TURN_P * msg->axes[0];
					c_port_duty.data = -TURN_P * msg->axes[0];
					set_roll.data  =  - msg->axes[2] * 15.0f;
					set_pitch.data =  - msg->axes[3] * 15.0f;
					// ROS_INFO("vx: %.2f, vy: %.2f", -joy_to_pitch.data, -joy_to_roll.data);
					c_updowm_duty.data = msg->axes[1];
					break;
    			default:
					ROS_INFO("[Error]: Unkown joy mode!");
        			break;
			}

        	break;	
    	default:
			c_stdb_duty.data = 0.0;
			c_port_duty.data = 0.0;
			c_updowm_duty.data = 0.0;
        	break;
	}
}

void esc_wakeup() 
{
    ROS_INFO("Starting ESC wakeup ...");

    ros::Rate wakeup_rate(50);  				// 50Hz������20ms

    int pulses[4] = {1000, 1500, 1000, 1500};  	// ���廽�ѵ���������

    for (int stage = 0; stage < 4; ++stage) 
	{  											// �����ĸ��׶�
        ROS_INFO("Stage %d: Sending %dus pulses for 2 seconds...", stage + 1, pulses[stage]);
        for (int i = 0; i < 100; ++i) 
		{  										// ÿ�׶η���100���źţ�Լ2��
            for (int channel = 1; channel <= 6; ++channel) 
			{  									// �����?6��ͨ��
                if (rc_servo_send_pulse_us(channel, pulses[stage]) != 0) 
				{
                    ROS_ERROR("Failed to send pulse to channel %d", channel);
                }
            }
            wakeup_rate.sleep();  				// ÿ��ѭ���ȴ�20ms������50Hz
        }
    }
    ROS_INFO("ESC wakeup sequence completed.");
} 

int limit_pwm_PID (int pwm)
{
  	if(pwm > 2000) 
    	return 2000;
  	else if(pwm < 1000)
	  	return 1000;
	else
		return pwm;
}


void MySigintHandler(int sig)
{
	//shutting down
  	ROS_INFO("shutting down!");
	rc_servo_power_rail_en(0);
 	rc_servo_cleanup();
	running = 0;
  	ros::shutdown();
}

void heartbeatCallback(const std_msgs::Int8::ConstPtr& msg)
{
    heartbeat=heartbeat+1;
}

void Motor_X_Callback(const std_msgs::Float64::ConstPtr& msg)
{
   c_pid_x=msg->data;
}

void Motor_Y_Callback(const std_msgs::Float64::ConstPtr& msg)
{
   c_pid_y=msg->data;
}

void Motor_Z_Callback(const std_msgs::Float64::ConstPtr& msg)
{
   c_pid_z=msg->data;
}

void Motor_D_Callback(const std_msgs::Float64::ConstPtr& msg)
{
   c_pid_d=msg->data;
}

/**This program use controls the PWM based on the Joy node
Jan-09-2020 Mingxi This node sets the PWM to two motors
										changed the ros node Subscriber
										directly subscribe to joy message
Aug-12-2020 heartbeat added. This node will stop if there are no heatbeat signals in 4s.

May-2020 control the usv with the right joy alone. remote mode 2 if "X"is pressed.

May-2022 publish heading if "Y"is pressed.
*/

//include c language libraries
extern "C" {
#include "roboticscape.h" 
}
#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h>  // for atoi() and exit()
#include <signal.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"  
#include "std_msgs/Int8.h" 
#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
#include "sensor_msgs/Joy.h"

int c_port_cmd = 0,c_stdb_cmd=0,heartbeat=0,count=0,joycount=0,c_up_cmd1=0,c_up_cmd2=0,c_dowm_cmd1=0,c_dowm_cmd2=0,c_pid_x=0,c_pid_y=0,c_pid_z=0,c_pid_d=0,mode=0,dowm=0;
std_msgs::Float64 c_port_duty, c_stdb_duty,c_updowm_duty,now_heading;
int delay=0;
int running = 0, pbcheading_flag=0;
int INIT_PULSE_WIDTH = 1500; //us 
//int WAKEUP_EN = 1;      // wakeup period enabled by default
double WAKEUP_HS = 0.0;// 1000us 
double WAKEUP_HT = 0.5;// 1500us half-throttle
double TURN_P = 0.5;
float MAX_PW_INCREMENT = 300;
float MAX_PW_INCREMENT1 = 400;

int AUTO_FLAG=0; 
bool lock_state = false;  // ����״̬
bool last_button_state = false;  // �ϴΰ�ť״̬ 
ros::Publisher lock_pub;

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

void stdb_callback (const std_msgs::Float64::ConstPtr& msg)
{ 
//	if(AUTO_FLAG==1) 
//	{
//		ROS_INFO("stdb_callback:%lf",msg->data);
//	c_stdb_duty.data = msg->data;
//	c_stdb_cmd = (int)( c_stdb_duty.data*MAX_PW_INCREMENT) + INIT_PULSE_WIDTH;
//	}
} 


void port_callback (const std_msgs::Float64::ConstPtr& msg)
{
//	if(AUTO_FLAG==1)
//	{
//		ROS_INFO("port_callback:%lf",msg->data);
//	  c_port_duty.data = msg->data;
//	  c_port_cmd = (int)( c_port_duty.data*MAX_PW_INCREMENT) + INIT_PULSE_WIDTH;
//	}
} 


void limit_pwm(int pwm)
{
  	if(pwm>1600) 
    	pwm=1600;
  	else if(pwm<1400)
    	pwm=1400;  
}

void limit_pwm1(int pwm)
{
  	if(pwm>2000) 
    	pwm=2000;
  	else if(pwm<1000)   
    	pwm=1000;  
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

void Motor_Callback (const sensor_msgs::Joy::ConstPtr& msg)
{
	bool button_pressed = msg->buttons[1];
    // ÿ�μ�⵽��ť״�?�����ֱ���л�����״�?
  	if (button_pressed && !last_button_state) 
  	{
      	lock_state = !lock_state;
      	std_msgs::Bool lock_msg;
      	lock_msg.data = lock_state;
      	lock_pub.publish(lock_msg);  // ��������״̬
  		// ROS_INFO("lock state: %s", lock_msg.data ? "ON" : "OFF");
  	}
  
  	last_button_state = button_pressed;
                                                                     
	//check for button press for motor driver relay
	if (msg->buttons[3] == 1)
  	{   	 
  		//wait for the esc
        ROS_INFO("waking ESC up from idle for 3 seconds");
        for(int i=0;i<=150;i++)
        {
            if(running==0) return ; 
            if(rc_servo_send_esc_pulse_normalized(1,WAKEUP_HS)==-1) return ;
			rc_usleep(1000000/50);
			if(rc_servo_send_esc_pulse_normalized(1,WAKEUP_HT )==-1) return ;
			rc_usleep(1000000/50);
			if(rc_servo_send_esc_pulse_normalized(2,WAKEUP_HS)==-1) return ;
			rc_usleep(1000000/50);
			if(rc_servo_send_esc_pulse_normalized(2,WAKEUP_HT )==-1) return ;
			rc_usleep(1000000/50);
			if(rc_servo_send_esc_pulse_normalized(3,WAKEUP_HS)==-1) return ;
			rc_usleep(1000000/50);
			if(rc_servo_send_esc_pulse_normalized(3,WAKEUP_HT )==-1) return ;
            rc_usleep(1000000/50);
			if(rc_servo_send_esc_pulse_normalized(4,WAKEUP_HS)==-1) return ;
			rc_usleep(1000000/50);
			if(rc_servo_send_esc_pulse_normalized(4,WAKEUP_HT )==-1) return ;
			rc_usleep(1000000/50);
            if(rc_servo_send_esc_pulse_normalized(5,WAKEUP_HS)==-1) return ; 
			rc_usleep(1000000/50);
			if(rc_servo_send_esc_pulse_normalized(5,WAKEUP_HT )==-1) return ;
            rc_usleep(1000000/50);
			if(rc_servo_send_esc_pulse_normalized(6,WAKEUP_HS)==-1) return ;
			rc_usleep(1000000/50); 
			if(rc_servo_send_esc_pulse_normalized(6,WAKEUP_HT )==-1) return ;
            rc_usleep(1000000/50);
        }
        ROS_INFO("done with wakeup period");
  	} 
	//Close the relay
  	if (msg->buttons[4] == 1)
  	{
    	if(rc_servo_send_esc_pulse_normalized(3,WAKEUP_HT )==-1) return;
        rc_usleep(1000000/50);
        //if(rc_servo_send_esc_pulse_normalized(1,WAKEUP_HT )==-1) return;
        //rc_usleep(1000000/50);
        //if(rc_servo_send_esc_pulse_normalized(5,WAKEUP_HT )==-1) return;
        //rc_usleep(1000000/50);

  	}
	//only cmd PWM when it is postive
	if(AUTO_FLAG==0)
	{
	//left stick
    	mode=1;
		joycount=0;
		c_port_cmd = (int)(msg->axes[1]*MAX_PW_INCREMENT1) + INIT_PULSE_WIDTH;	//set global pulse width varilable
		c_stdb_cmd = (int)( msg->axes[3]*MAX_PW_INCREMENT1) + INIT_PULSE_WIDTH;
//		ROS_INFO("c_port_cmd%d",c_port_cmd);
	}
	// remote mode 2, turn with joystick
	if(AUTO_FLAG==2)  // 主要的工作模式！！！！！！！！！！！！！！！！！！！
	{   
	//left stick
    	mode=2; 
		joycount=0;
		// ROS_INFO("FLAG%d",AUTO_FLAG);
		c_stdb_duty.data = msg->axes[3]+TURN_P*msg->axes[2];
		c_port_duty.data = msg->axes[3]-TURN_P*msg->axes[2];
		c_updowm_duty.data = msg->axes[1];
   		if(msg->buttons[0]==1)
   		{
    		if(dowm==0) 
    	 	{
				dowm=1;
			}
    	 	else 
			{
				dowm=0;
			}
   		}
	}
	//Set auto flag if "A" is pressed
	// if(msg->buttons[1]==1)
	// {
    // 	AUTO_FLAG=1;
	// 	pbcheading_flag=0;
	// }
	//disable auto flag if "B" is pressed
	if(msg->buttons[2]==1)
	{
		AUTO_FLAG=0;
		pbcheading_flag=0;
	}
	//remote mode 2 if "X"is pressed
	if(msg->buttons[0]==1) 
	{
		AUTO_FLAG=2;
		pbcheading_flag=0; 
	}
	
		//remote mode 2 if "Y"is pressed
	if(msg->buttons[5]==1)
	{
		AUTO_FLAG=1;
		pbcheading_flag=1;
	}	
}

// interrupt handler to catch ctrl-c
void MySigintHandler(int sig)
{
	//shutting down
  	ROS_INFO("shutting down!");
	rc_servo_power_rail_en(0);
 	rc_servo_cleanup();
  	//rc_gpio_cleanup(1, 25);
  	//rc_gpio_cleanup(1, 17);
	running = 0;
  	ros::shutdown();
}

void inetMotor_Callback(const std_msgs::Int8::ConstPtr& msg)
{
	if(msg->data==0)
 	{
   		delay=1;
	}
 	if(msg->data==1)
 	{
   		delay=2;
 	}
 	if(msg->data==2)
 	{
   		delay=3;
 	}
 	if(msg->data==3)
 	{
  	 	delay=0;
 	}
}
void heartbeatCallback(const std_msgs::Int8::ConstPtr& msg)
{
    heartbeat=heartbeat+1;
}

void imuCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    now_heading.data=msg->z;
}

void Motor_X(const std_msgs::Float64::ConstPtr& msg)
{
   c_pid_x=msg->data;
}

void Motor_Y(const std_msgs::Float64::ConstPtr& msg)
{
   c_pid_y=msg->data;
}

void Motor_Z(const std_msgs::Float64::ConstPtr& msg)
{
   c_pid_z=msg->data;
}

void Motor_D(const std_msgs::Float64::ConstPtr& msg)
{
   c_pid_d=msg->data;
}

int main(int argc, char** argv)
{
  	usleep(100000);  //there will be problems if there is no delay here
	//setup servo
	if(rc_servo_init()) 
		return -1;
    if(rc_servo_set_esc_range(RC_ESC_DEFAULT_MIN_US,RC_ESC_DEFAULT_MAX_US)) 
		return -1;
    rc_servo_power_rail_en(0);

	//if ctrl-c we shut down things
	signal(SIGINT, MySigintHandler);
  	running = 1;

	//ROS node stuff
  	ros::init(argc, argv, "auto_pwm");

  	ros::NodeHandle nh;

	//Remote control mode
	ros::Subscriber sub6 = nh.subscribe("/joy", 2, Motor_Callback);
 	ros::Subscriber sub8 = nh.subscribe("/PID/X", 2, Motor_X);
 	ros::Subscriber sub7 = nh.subscribe("/PID/Y", 2, Motor_Y);
 	ros::Subscriber sub9 = nh.subscribe("/PID/depth", 2, Motor_D); 
  	ros::Subscriber sub10 = nh.subscribe("/PID/Z", 2, Motor_Z); 
 	ros::Subscriber sub1 = nh.subscribe("/inet", 2, inetMotor_Callback);
	ros::Subscriber sub2 = nh.subscribe("/thrusters/stdb_cmd",2, stdb_callback);
	ros::Subscriber sub3 = nh.subscribe("/thrusters/port_cmd",2, port_callback);
	ros::Subscriber sub4 = nh.subscribe("/heartbeat", 2, heartbeatCallback);
	ros::Subscriber sub5 = nh.subscribe("/imu/rpy", 2, imuCallback);
	ros::Publisher port_pwm_pub = nh.advertise<std_msgs::Int8>("/joy/port_cmd", 2);
  	ros::Publisher stdb_pwm_pub = nh.advertise<std_msgs::Int8>("/joy/stdb_cmd", 2);
	ros::Publisher mode_pub = nh.advertise<std_msgs::Int8>("/light",2);
	ros::Publisher c_heading_pub = nh.advertise<std_msgs::Float64>("/control/heading",2); 
 	lock_pub = nh.advertise<std_msgs::Bool>("/lock",2);

	ros::Rate loop_rate(50);  //esc need 50 Hz
	
	std_msgs::Int8 gnc_mode;
	std_msgs::Int8 joy_port; 
	std_msgs::Int8 joy_stdb; 

	esc_wakeup();
	//START THE LOOP
  	while (ros::ok() && running) 
  	{		//rc_servo_send_pulse_us(2, 2000);

	  	// send pwm adjust command
    	if(AUTO_FLAG==2)
    	{
  	  		c_stdb_cmd = INIT_PULSE_WIDTH + (int)(c_stdb_duty.data * MAX_PW_INCREMENT1) - c_pid_z;//-c_pid_z;
	  		c_port_cmd = INIT_PULSE_WIDTH + (int)(c_port_duty.data * MAX_PW_INCREMENT1) + c_pid_z;//-c_pid_z;
      		limit_pwm1(c_stdb_cmd);
      		limit_pwm1(c_port_cmd); 
    	}
		if(delay == 0 && mode != 0)
		{
			c_up_cmd1	= INIT_PULSE_WIDTH + (int)(c_updowm_duty.data * MAX_PW_INCREMENT) - c_pid_x + c_pid_y + c_pid_d; // ���2
			c_up_cmd2	= INIT_PULSE_WIDTH + (int)(c_updowm_duty.data * MAX_PW_INCREMENT) + c_pid_x - c_pid_y + c_pid_d; // ���5
		 	c_dowm_cmd1	= INIT_PULSE_WIDTH + (int)(c_updowm_duty.data * MAX_PW_INCREMENT) + c_pid_x + c_pid_y + c_pid_d; // ���4
		 	c_dowm_cmd2	= INIT_PULSE_WIDTH + (int)(c_updowm_duty.data * MAX_PW_INCREMENT) - c_pid_x - c_pid_y + c_pid_d; // ���6
			
			c_up_cmd1 = limit_pwm_PID (c_up_cmd1);
			c_up_cmd2 = limit_pwm_PID (c_up_cmd2);
			c_dowm_cmd1 = limit_pwm_PID (c_dowm_cmd1);
			c_dowm_cmd2 = limit_pwm_PID (c_dowm_cmd2);

			rc_servo_send_pulse_us(1, c_stdb_cmd);//port side
			rc_servo_send_pulse_us(3, c_port_cmd);//starboard side

   			if(dowm==1)
   			{
   				rc_servo_send_pulse_us(2, c_up_cmd1);	//c_up_cmd1����
				rc_servo_send_pulse_us(4, c_dowm_cmd1 ); //c_dowm_cmd1����                                                              
	 			rc_servo_send_pulse_us(5, c_up_cmd2 );	// c_up_cmd2ǰ��
	 			rc_servo_send_pulse_us(6, c_dowm_cmd2);	//c_dowm_cmd2ǰ�� 

		 		//ROS_INFO("qqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqq:%d",c_dowm_cmd2);
   			}
   			else
   			{
    			rc_servo_send_pulse_us(2,1500);//c_up_cmd1����
				rc_servo_send_pulse_us(4,1500 ); //c_dowm_cmd1����                                                              
				rc_servo_send_pulse_us(5,1500 );// c_up_cmd2ǰ��
				rc_servo_send_pulse_us(6,1500);//c_dowm_cmd2ǰ��
   			}
		
		}
   		if (delay==1) 
		{
			rc_servo_send_pulse_us(1, c_port_cmd);//port side
			rc_servo_send_pulse_us(2, 1200);
			rc_servo_send_pulse_us(3, c_stdb_cmd);//starboard side
			rc_servo_send_pulse_us(4, 1800);
			rc_servo_send_pulse_us(5, 1200);
			rc_servo_send_pulse_us(6, 1800);
		
		}
   		if (delay==2)
		{
			rc_servo_send_pulse_us(1, c_port_cmd);//port side
			rc_servo_send_pulse_us(2, 1800);
			rc_servo_send_pulse_us(3, c_stdb_cmd);//starboard side
			rc_servo_send_pulse_us(4, 1200);
			rc_servo_send_pulse_us(5, 1800);
			rc_servo_send_pulse_us(6, 1200);
		}
   		if (delay==3)
		{
			rc_servo_send_pulse_us(1, c_port_cmd);//port side 
			rc_servo_send_pulse_us(2, 1500);
			rc_servo_send_pulse_us(3, c_stdb_cmd);//starboard side
			rc_servo_send_pulse_us(4, 1500);
			rc_servo_send_pulse_us(5, 1500);
			rc_servo_send_pulse_us(6, 1500);
		}
		//joy stuff
		joy_port.data=c_port_cmd;
		joy_stdb.data=c_stdb_cmd;
		gnc_mode.data = 3-AUTO_FLAG;	
		count++;
		joycount++;
		if(count>200)
		{
			port_pwm_pub.publish(joy_port);
			stdb_pwm_pub.publish(joy_stdb);
			mode_pub.publish(gnc_mode);	
			if(pbcheading_flag==1)
			{
				pbcheading_flag=0;
				c_heading_pub.publish(now_heading);
			}
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
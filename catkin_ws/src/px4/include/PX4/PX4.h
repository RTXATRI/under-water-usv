/*
 * 使用 ROS 和 mavros 控制 PX4 的一些数据封装以及基础处理函数
 *
 * Created by ATRI on 8 Dec, 2024
 */
#ifndef _PX4_H
#define _PX4_H

/* Includes ------------------------------------------------------------------*/
#include <ros/ros.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/FluidPressure.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/BatteryStatus.h>


/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

#define PX4_FRE 20

/* USER CODE END Private defines */

/* USER CODE BEGIN PTD */

enum PX4_STATE
{
    PX4_ERROR,
    PX4_OK,
    PX4_RUNNING,
    PX4_NULL
};

/* USER CODE END PTD */

/* USER CODE BEGIN Variables */

ros::Subscriber px4_state_sub; // 用于接受无人机当前状态

mavros_msgs::State px4_current_state; // 无人机当前状态

ros::Subscriber px4_local_pos_sub; // 订阅无人机当前坐标查询话题px4_local_pos_sub，将无人机当前坐标位置存储在px4_local_pos中

geometry_msgs::PoseStamped px4_local_pos; // 无人机当前坐标位置

ros::Publisher px4_local_pos_pub; // 订阅发布无人机目标位置话题px4_local_pos_pub，用于发布无人机目标位置px4_set_pose，用法:px4_loacl_pos_pub.publish(px4_set_pose)

geometry_msgs::PoseStamped px4_set_pose; // 无人机目标位置

ros::Publisher px4_vec_pub; // 订阅无人机目标速度发布话题px4_vec_pub，用于发布无人机目标速度，用法:px4_vec_pub.publish(px4_set_vec)

geometry_msgs::Twist px4_set_vec; // 无人机目标速度

ros::ServiceClient px4_arming_client; // 订阅无人机解锁话题px4_arming_client，用于发布无人机解锁指令，用法:px4_arming_client.call(px4_arm_cmd)

mavros_msgs::CommandBool px4_arm_cmd; // 无人机的解锁状态

ros::ServiceClient px4_set_mode_client; // 订阅无人机飞行模式话题px4_set_mode_client，用于发布无人机模式切换指令，用法:px4_set_mode_client.call(px4_set_mode)

mavros_msgs::SetMode px4_set_mode; // 无人机的设定工作模式

ros::Subscriber px4_imu_sub; // 无人机的IMU数据

ros::Subscriber px4_vel_sub; // 订阅无人机当前速度

geometry_msgs::TwistStamped px4_vel;

ros::Subscriber px4_battery_sub;

mavros_msgs::BatteryStatus px4_battery;

ros::Publisher px4_imu_pub;

double px4_pitch, px4_roll, px4_yaw;

// double px4_battery_vlotage, px4_battery_current, px4_battery_percentage;

// int px4_battery_health_status;
// ros::Subscriber px4_distance_sub; // 无人机

// sensor_msgs::FluidPressure px4_sensor_range;

//
double use_time;

/* USER CODE END Variables */

/* USER CODE BEGIN Prototypes */

void PX4_Node_Init();
PX4_STATE PX4_FCU_Connect(ros::Rate rate, double outtime);
PX4_STATE PX4_Mode_Set(ros::Rate rate, const std::string &name, double outtime);
PX4_STATE PX4_Arm_Set(ros::Rate rate, uint8_t arm_cfg, double outtime);

void PX4_STATE_SUB_CALLBACK(const mavros_msgs::State::ConstPtr &msg);
void PX4_LOCAL_POS_SUB_CALLBACK(const geometry_msgs::PoseStamped::ConstPtr &msg);
void PX4_IMU_SUB_CALLBACK(const sensor_msgs::Imu::ConstPtr &msg);
void PX4_VEL_SUB_CALLBACK(const geometry_msgs::TwistStamped::ConstPtr &msg);
void PX4_BATTERY_CALLBACK(const mavros_msgs::BatteryStatus::ConstPtr& msg);
// void PX4_SENSOR_RANGE_SUB_CALLBACK(const sensor_msgs::FluidPressure::ConstPtr &msg);
/* USER CODE END Prototypes */

/* USER CODE BEGIN Private defines Prototypes*/

/* USER CODE END Private defines Prototypes*/

#endif
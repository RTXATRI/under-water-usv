/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 **/

/* Includes ------------------------------------------------------------------*/
// #include <ros/ros.h>
#include <PX4/PX4.h>
#include <typeinfo>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* USER CODE BEGIN Private defines Prototypes*/

#define Ros_Init(argc, argv, name, fre) \
    ros::init(argc, argv, name);        \
    PX4_Node_Init();                    \
    ros::Rate rate(fre)

/* USER CODE END Private defines Prototypes*/

int main(int argc, char **argv)
{
    Ros_Init(argc, argv, "px4_node", 20);

    if (PX4_FCU_Connect(rate, 20.0) == PX4_OK)
    {
        ROS_INFO("PX4 has Connected! Use time: %.2fs", use_time);
    }

    while (ros::ok())
    {

        // ROS_INFO("Roll: %.2f, Pitch: %.2f, Yaw: %.2f", px4_roll, px4_pitch, px4_yaw);
        // ROS_INFO("Vx: %0.2fm/s, Vy: %.2fm/s, Vz: %.2fm/s", px4_vel.twist.linear.x, px4_vel.twist.linear.y, px4_vel.twist.linear.z);
        // ROS_INFO("Voltage: %.2f V, Current: %.2f A (%s), Percent: %.2f %%", px4_battery.voltage, std::abs(px4_battery.current), ((px4_battery.current < 0.0) ? "working" : "charging"), std::abs(((px4_battery.voltage - 21.60) / 3.60) * 100));
        
        ROS_INFO("Voltage: %.2f V, Percent: %.2f %%", px4_battery.voltage, std::abs(((px4_battery.voltage - 21.60) / 3.60) * 100));
        
        // 处理所有挂起的回调函数
        ros::spinOnce();

        // 影响消息发布与更新的周期
        rate.sleep();
    }

    return 0;
}

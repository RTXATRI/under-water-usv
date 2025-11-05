/**
 * 该例程将发布chatter话题，消息类型String
 */

#include <sstream>
#include "ros/ros.h"
//#include "/usr/include/roboticscape.h"
extern "C" {
#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h>  // for atoi() and exit()
#include <rc/mpu.h>
#include <rc/time.h>
}
#include "std_msgs/String.h"
#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN 21
rc_mpu_data_t data;

int main(int argc, char** argv)
{
  // ROS节点初始化
  ros::init(argc, argv, "talker");

  // 创建节点句柄
  ros::NodeHandle dmp;

  // 创建一个Publisher，发布名为chatter的topic，消息类型为std_msgs::String
  ros::Publisher chatter_pub = dmp.advertise<std_msgs::String>("chatter", 1000);

  // 设置循环的频率
  ros::Rate loop_rate(5);
  // rc_imu_config_t conf = rc_default_imu_config();
  rc_mpu_config_t conf = rc_mpu_default_config();
  conf.i2c_bus = I2C_BUS;
  conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
  conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
  conf.dmp_sample_rate = 5;
  conf.enable_magnetometer = 1;
  //	if(rc_initialize_imu_dmp(&data, conf)){
  if (rc_mpu_initialize_dmp(&data, conf))
  {
    printf("rc_initialize_imu_failed\n");
    return -1;
  }

  int count = 0;
  while (ros::ok())
  {
    // 初始化std_msgs::String类型的消息
    std_msgs::String heading;
    std::stringstream ss;
    ss << data.dmp_TaitBryan[TB_YAW_Z] * RAD_TO_DEG << count;
    heading.data = ss.str();
    ROS_INFO("raw data: %5.2f", data.fused_TaitBryan[TB_YAW_Z] * RAD_TO_DEG);
    // 发布消息
    ROS_INFO("%s", heading.data.c_str());
    ROS_INFO("heading: %6.2f", data.compass_heading * RAD_TO_DEG);
    chatter_pub.publish(heading);

    // 循环等待回调函数
    ros::spinOnce();

    // 按照循环频率延时
    loop_rate.sleep();
    ++count;
  }

  return 0;
}

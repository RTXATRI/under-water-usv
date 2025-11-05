/*
 * 使用 ROS 和 mavros 控制 PX4 的一些数据封装以及基础处理函数
 *
 * Created by ATRI on 8 Dec, 2024
 */

/* Includes ------------------------------------------------------------------*/
#include <PX4/PX4.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* USER CODE BEGIN Application */

/************************************************************
 * 函数名称: PX4_Node_Init
 * 功能描述: 订阅初始化
 * 参数说明:
 *     par1: argc
 *     par2: argv
 *     par3: name - 程序命名
 * 返回值: 无
 ************************************************************/
// void PX4_Node_Init(int argc, char **argv, const std::string &name)
void PX4_Node_Init()
{
    ros::NodeHandle nh;
    px4_state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, PX4_STATE_SUB_CALLBACK);
    px4_local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, PX4_LOCAL_POS_SUB_CALLBACK);
    px4_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    px4_vec_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    px4_arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    px4_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    px4_imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 20, PX4_IMU_SUB_CALLBACK);
    px4_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 20, PX4_VEL_SUB_CALLBACK);
    px4_battery_sub = nh.subscribe<mavros_msgs::BatteryStatus>("mavros/battery", 20, PX4_BATTERY_CALLBACK);

    px4_imu_pub = nh.advertise<sensor_msgs::Imu>("/px4/imu", 20);
    // px4_distance_sub = nh.subscribe("/mavros/barometric_pressure/height", 10, PX4_SENSOR_RANGE_SUB_CALLBACK);

    px4_set_pose.pose.position.z = px4_local_pos.pose.position.z;
    px4_set_pose.pose.position.x = px4_local_pos.pose.position.x;
    px4_set_pose.pose.position.y = px4_local_pos.pose.position.y;

    px4_set_vec.linear.x = 0.0;
    px4_set_vec.linear.y = 0.0;
    px4_set_vec.linear.z = 0.0;
}

/************************************************************
 * 函数名称: PX4_FCU_Connect
 * 功能描述: 连接PX4飞控
 * 参数说明:
 *     par1: rate - 控制循环频率
 *     par3: outtime - 超时时间
 * 返回值:
 *     PX4_ERROT: 连接失败
 *     PX4_OK：连接成功
 ************************************************************/
PX4_STATE PX4_FCU_Connect(ros::Rate rate, double outtime)
{
    ROS_INFO("Start connceting PX4(V1.15.2 base on pixhawk 6C),please wait......");
    ros::Time start_time = ros::Time::now();
    // MAVROS会自动连接，收到心跳包后会自动设置标志位
    while (ros::ok() && !px4_current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
        // ROS_INFO("State: %d ", px4_current_state.connected);
        if (ros::Time::now() - start_time > ros::Duration(outtime))
        {
            ROS_INFO("Connected Fail, Please check Link!");
            return PX4_ERROR;
        }
    }
    use_time = ros::Time::now().toSec() - start_time.toSec();
    return PX4_OK;
}

/************************************************************
 * 函数名称: PX4_Mode_Set
 * 功能描述: 设定飞机飞行模式
 * 参数说明:
 *     par1: rate - 控制循环频率
 *     par2: name - 设定的模式
 *     par3: outtime - 超时时间
 * 返回值:
 *     PX4_ERROT: 设定飞行模式失败
 *     PX4_OK：设定飞行模式成功
 *     PX4_NULL: 未知错误
 * 备注：PX4的Offboard切换需要满足两个条件：1.命令间隔不能超时。2.有控制信号。
 *      即offboard切换命令之间设置了500毫秒的超时检查。一但发生超时，飞控组件中的commander模块会立即切换回进入Offboard模式之前的飞行模式。
 *      因此发布频率必须大于2Hz的原因。
 *      除此之外，板载模式需要满足存在控制信号，所以需要加入一行伪速度信号控制发布才能实现模式的切换
 *      此外，建议从Position模式进入Offboard模式，因为在这种情况下如果无人机退出Offboard模式，它将会悬停于当前位置。
 ************************************************************/
PX4_STATE PX4_Mode_Set(ros::Rate rate, const std::string &name, double outtime)
{
    geometry_msgs::Twist px4_set_vec_fake;
    px4_set_vec_fake.linear.x = 0;
    px4_set_vec_fake.linear.y = 0;
    px4_set_vec_fake.linear.z = 0;

    px4_set_mode.request.custom_mode = name;

    ros::Time last_request = ros::Time::now();
    ros::Time start_request = ros::Time::now();

    ROS_INFO("Start setting mode as: %s,please wait......", px4_set_mode.request.custom_mode.c_str());

    if (px4_current_state.mode == px4_set_mode.request.custom_mode)
    {
        use_time = ros::Time::now().toSec() - start_request.toSec();
        return PX4_OK;
    }
    else
    {
        while (ros::ok() && (ros::Time::now() - start_request < ros::Duration(outtime)))
        {
            if (px4_set_mode.request.custom_mode == "OFFBOARD")
            {
                px4_vec_pub.publish(px4_set_vec_fake); // 板载模式需要满足存在控制信号，所以需要加入一行伪速度信号控制发布才能实现模式的切换
            }
            ros::spinOnce();
            rate.sleep();
            if (ros::Time::now() - last_request > ros::Duration(1.0))
            {
                last_request = ros::Time::now();
                if (px4_current_state.mode == px4_set_mode.request.custom_mode)
                {
                    use_time = ros::Time::now().toSec() - start_request.toSec();
                    return PX4_OK;
                }
                else
                {
                    if (px4_set_mode_client.call(px4_set_mode) && px4_set_mode.response.mode_sent)
                    {
                        continue;
                    }
                }
            }
        }
        ROS_INFO("Try to set mode error: Time out");
        return PX4_ERROR;
    }
    ROS_INFO("Try to set mode error: Unknown error");
    return PX4_NULL;
}

/************************************************************
 * 函数名称: PX4_Arm_Set
 * 功能描述: 设定飞机解锁
 * 参数说明:
 *     par1: rate - 控制循环频率
 *     par2: arm_cfg - 是否解锁：false为未解锁，ture为解锁
 *     par3: outtime - 超时时间
 * 返回值:
 *     PX4_ERROT: 设定解锁状态失败
 *     PX4_OK：设定解锁状态成功
 *     PX4_NULL: 未知错误
 ************************************************************/
PX4_STATE PX4_Arm_Set(ros::Rate rate, uint8_t arm_cfg, double outtime)
{
    px4_arm_cmd.request.value = arm_cfg;
    ros::Time last_request = ros::Time::now();
    ros::Time start_request = ros::Time::now();

    if (px4_arm_cmd.request.value == true)
    {
        if (px4_current_state.mode != "OFFBOARD") // 警告当前处于非ofboard模式下用板载计算机解锁飞机
        {
            ROS_INFO("Warning! The mode is %s, Please confirm if you really want to unlock the aircraft in non-OFFBOARD mode!", px4_current_state.mode.c_str());
        }
        ROS_INFO("Start arming plane,please wait......");
    }
    else if (px4_arm_cmd.request.value == false)
    {
        ROS_INFO("Start disarming plane,please wait......");
    }

    if (px4_current_state.armed == px4_arm_cmd.request.value)
    {
        use_time = ros::Time::now().toSec() - start_request.toSec();
        return PX4_OK;
    }
    else
    {
        while (ros::ok() && (ros::Time::now() - start_request < ros::Duration(outtime)))
        {
            ros::spinOnce();
            rate.sleep();
            if (ros::Time::now() - last_request > ros::Duration(3.0))
            {
                last_request = ros::Time::now();
                if (px4_current_state.armed == px4_arm_cmd.request.value)
                {
                    use_time = ros::Time::now().toSec() - start_request.toSec();
                    return PX4_OK;
                }
                else
                {
                    if (px4_arming_client.call(px4_arm_cmd) && px4_arm_cmd.response.success)
                    {
                        continue;
                    }
                }
            }
        }
        ROS_INFO("Try to set arm error: Time out");
        return PX4_ERROR;
    }
    ROS_INFO("Try to set arm error: Unknown error");
    return PX4_NULL;
}

/************************************************************
 * 函数名称: PX4_STATE_SUB_CALLBACK
 * 功能描述: 无人机当前状态更新回调函数
 * 参数说明:
 *     par1: msg - 消息值
 * 返回值: 无
 ************************************************************/
void PX4_STATE_SUB_CALLBACK(const mavros_msgs::State::ConstPtr &msg)
{
    px4_current_state = *msg;
}

/************************************************************
 * 函数名称: PX4_LOCAL_POS_SUB_CALLBACK
 * 功能描述: 无人机当前位置更新回调函数
 * 参数说明:
 *     par1: msg - 消息值
 * 返回值: 无
 ************************************************************/
void PX4_LOCAL_POS_SUB_CALLBACK(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    px4_local_pos = *msg;
    // tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    // tf::Matrix3x3 m(q);
    // m.getRPY(px4_roll, px4_pitch, px4_yaw);
}

/************************************************************
 * 函数名称: PX4_IMU_SUB_CALLBACK
 * 功能描述: 无人机IMU回调函数
 * 参数说明:
 *     par1: msg - 消息值
 * 返回值: 无
 ************************************************************/
void PX4_IMU_SUB_CALLBACK(const sensor_msgs::Imu::ConstPtr &msg)
{

    px4_imu_pub.publish(*msg);
    // 提取时间戳
    ros::Time timestamp = msg->header.stamp;

    // 提取姿态数据（四元数）
    geometry_msgs::Quaternion orientation = msg->orientation;

    // 提取角速度（弧度/秒）
    geometry_msgs::Vector3 angular_velocity = msg->angular_velocity;

    // 提取线加速度（米/秒²）
    geometry_msgs::Vector3 linear_acceleration = msg->linear_acceleration;

    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3 m(q);
    
    m.getRPY(px4_roll, px4_pitch, px4_yaw);

    // // 打印数据（示例）
    // ROS_INFO("Timestamp: %f", timestamp.toSec());
    // ROS_INFO("Orientation: [%f, %f, %f, %f]",
    //          orientation.x, orientation.y, orientation.z, orientation.w);
    // ROS_INFO("Angular Velocity: [%f, %f, %f]",
    //          angular_velocity.x, angular_velocity.y, angular_velocity.z);
    // ROS_INFO("Linear Acceleration: [%f, %f, %f]",
    //          linear_acceleration.x, linear_acceleration.y, linear_acceleration.z);
}

/************************************************************
 * 函数名称: PX4_VEL_SUB_CALLBACK
 * 功能描述: 无人机当速度
 * 参数说明:
 *     par1: msg - 消息值
 * 返回值: 无
 ************************************************************/
void PX4_VEL_SUB_CALLBACK(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    px4_vel = *msg;
}

/************************************************************
 * 函数名称: PX4_BATTERY_CALLBACK
 * 功能描述: 无人机当前电池状态
 * 参数说明:
 *     par1: msg - 消息值
 * 返回值: 无
 ************************************************************/
void PX4_BATTERY_CALLBACK(const mavros_msgs::BatteryStatus::ConstPtr& msg)
{
    px4_battery = *msg;
}



/************************************************************
 * 函数名称: PX4_SENSOR_RANGE_SUB_CALLBACK
 * 功能描述: 无人机当前光流传感器回调函数
 * 参数说明:
 *     par1: msg - 消息值
 * 返回值: 无
 * 目前看来没必要且不现实，local返回的坐标高度也够用
 ************************************************************/
// void PX4_SENSOR_RANGE_SUB_CALLBACK(const sensor_msgs::FluidPressure::ConstPtr &msg)
// {
//     px4_sensor_range = *msg;
// }

/* USER CODE END Application */
/***************************************************************************************************************************
 * Description: set arm + set mode + flight control for px4 flight stack
 ***************************************************************************************************************************/

#include "flight_control.h"

FlightControl::FlightControl() {}

FlightControl::FlightControl(string mavros_ns) : mavros_ns_(mavros_ns)
{
    nh_ = ros::NodeHandle("");
    setpoint_position_local_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    setpoint_raw_local_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
}

FlightControl::~FlightControl()
{
    // Destructor
}

void FlightControl::set_arm(bool is_arm)
{
    CommandBool arm_msg;
    arm_msg.request.value = is_arm;

    ros::ServiceClient set_arm_client = nh_.serviceClient<CommandBool>(mavros_ns_ + "/mavros/cmd/arming");
    if (!set_arm_client.call(arm_msg))
    {
        ROS_ERROR("Failed to call service arming");
    }
}

void FlightControl::set_mode(string mode)
{
    mavros_msgs::SetMode mode_msg;
    mode_msg.request.custom_mode = mode;

    ros::ServiceClient set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>(mavros_ns_ + "/mavros/set_mode");
    if (!set_mode_client.call(mode_msg))
    {
        ROS_ERROR("Failed to call service set_mode: %s", mode.c_str());
    }
}

// void PX4RosNav::set_mode(string mode)
// {
//     mavros_msgs::SetMode set_mode_msg;
//     set_mode_msg.request.custom_mode = mode;

//     ros::ServiceClient set_mode_client = nh_private_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
//     while (!set_mode_client.call(set_mode_msg))
//     {
//     }
//     if (set_mode_msg.response.mode_sent)
//     {
//         ROS_WARN("Set Mode: %s Finished", modes.c_str());
//     }
// }

void FlightControl::setpoint_position_local(const geometry_msgs::Point pos)
{
    geometry_msgs::PoseStamped poseTarget;
    poseTarget.pose.position = pos;
    setpoint_position_local_pub_.publish(poseTarget);
}

void FlightControl::setpoint_raw_local_pos(const geometry_msgs::Point pos, float yaw, int frame)
{
    // ROS_WARN_ONCE("[setpoint_raw_local_pos] yaw_rate = %f,frame = %d.", yaw_rate, frame);
    mavros_msgs::PositionTarget positionTarget;
    positionTarget.header.stamp = ros::Time::now();
    positionTarget.type_mask =
        PositionTarget::IGNORE_VX + PositionTarget::IGNORE_VY + PositionTarget::IGNORE_VZ +
        PositionTarget::IGNORE_AFX + PositionTarget::IGNORE_AFY + PositionTarget::IGNORE_AFZ +
        PositionTarget::FORCE +
        PositionTarget::IGNORE_YAW_RATE;

    positionTarget.coordinate_frame = frame; // PositionTarget::FRAME_LOCAL_NED or PositionTarget::FRAME_BODY_NED

    positionTarget.position = pos;
    positionTarget.yaw = yaw;

    setpoint_raw_local_pub_.publish(positionTarget);
}

void FlightControl::setpoint_raw_local_vel(const geometry_msgs::Point vel, float yaw_rate, int frame)
{
    mavros_msgs::PositionTarget positionTarget;
    positionTarget.type_mask =
        PositionTarget::IGNORE_PX + PositionTarget::IGNORE_PY + PositionTarget::IGNORE_PZ +
        PositionTarget::IGNORE_AFX + PositionTarget::IGNORE_AFY + PositionTarget::IGNORE_AFZ +
        PositionTarget::FORCE +
        PositionTarget::IGNORE_YAW;

    positionTarget.coordinate_frame = frame; // PositionTarget::FRAME_LOCAL_NED or PositionTarget::FRAME_BODY_NED

    positionTarget.velocity.x = vel.x;
    positionTarget.velocity.y = vel.y;
    positionTarget.velocity.z = vel.z;
    positionTarget.yaw_rate = yaw_rate;
    setpoint_raw_local_pub_.publish(positionTarget);
}

void FlightControl::setpoint_raw_local_pos_vel(const geometry_msgs::Point pos, const geometry_msgs::Point vel, float yaw, int frame)
{
    mavros_msgs::PositionTarget positionTarget;
    positionTarget.type_mask =
        PositionTarget::IGNORE_AFX + PositionTarget::IGNORE_AFY + PositionTarget::IGNORE_AFZ +
        PositionTarget::FORCE +
        PositionTarget::IGNORE_YAW_RATE;

    positionTarget.coordinate_frame = frame; // PositionTarget::FRAME_LOCAL_NED(default) or PositionTarget::FRAME_BODY_NED

    positionTarget.position = pos;

    positionTarget.velocity.x = vel.x;
    positionTarget.velocity.y = vel.y;
    positionTarget.velocity.z = vel.z;

    positionTarget.yaw = yaw;
    setpoint_raw_local_pub_.publish(positionTarget);
}

void FlightControl::setpoint_raw_local_velxy_posz(const geometry_msgs::Point vel_xy_pos_z, float yaw, int frame)
{
    mavros_msgs::PositionTarget positionTarget;
    positionTarget.type_mask =
        PositionTarget::IGNORE_PX + PositionTarget::IGNORE_PY +
        PositionTarget::IGNORE_AFX + PositionTarget::IGNORE_AFY + PositionTarget::IGNORE_AFZ +
        PositionTarget::FORCE +
        PositionTarget::IGNORE_YAW_RATE;

    positionTarget.coordinate_frame = frame; // PositionTarget::FRAME_LOCAL_NED or PositionTarget::FRAME_BODY_NED
    positionTarget.position.z = vel_xy_pos_z.z;

    positionTarget.velocity.x = vel_xy_pos_z.x;
    positionTarget.velocity.y = vel_xy_pos_z.y;

    positionTarget.yaw = yaw;
    setpoint_raw_local_pub_.publish(positionTarget);
}

/**
 * @brief 控制X轴线速度、偏航角速度和Z轴高度位置
 * @param vx X轴线速度（前向速度，m/s），正值为前进，负值为后退
 * @param yaw_rate 偏航角速度（rad/s），正值为顺时针旋转
 * @param target_z 目标高度位置（m），相对于起飞点
 * @param frame 坐标系：FRAME_LOCAL_NED 或 FRAME_BODY_NED
 * 
 * 这种控制模式适合：
 * 1. 定高前进侦察
 * 2. 螺旋搜索（前进+旋转）
 * 3. 航向调整的定高飞行
 */
void FlightControl::setpoint_raw_local_vx_yawrate_posz(const geometry_msgs::Point vx_yawrate_posz, int frame)
{
    mavros_msgs::PositionTarget positionTarget;
    positionTarget.header.stamp = ros::Time::now();
    
    // 设置type_mask掩码：
    // - 忽略XY位置（PX, PY）：因为我们控制X速度，不控制XY位置
    // - 忽略Y轴速度（VY）：只控制X轴速度，Y轴设为0或忽略
    // - 忽略Z轴速度（VZ）：我们用位置控制高度，所以忽略速度控制
    // - 忽略所有加速度（AFX, AFY, AFZ）
    // - 忽略偏航角（YAW）：使用偏航率控制，而不是绝对偏航角
    positionTarget.type_mask =
        PositionTarget::IGNORE_PX + PositionTarget::IGNORE_PY +  // 忽略XY位置
        PositionTarget::IGNORE_VY + PositionTarget::IGNORE_VZ +  // 忽略Y和Z速度
        PositionTarget::IGNORE_AFX + PositionTarget::IGNORE_AFY + PositionTarget::IGNORE_AFZ +  // 忽略加速度
        PositionTarget::FORCE +  // 使用力控制（通常是默认的）
        PositionTarget::IGNORE_YAW;  // 忽略偏航角，使用偏航率
    
    positionTarget.coordinate_frame = frame;  // 坐标系
    
    // 设置控制量
    positionTarget.position.z = vx_yawrate_posz.z;     // Z轴目标高度
    
    positionTarget.velocity.x = vx_yawrate_posz.x;           // X轴线速度
    positionTarget.yaw_rate = vx_yawrate_posz.y;       // 偏航角速度
    
    // 发布控制命令
    setpoint_raw_local_pub_.publish(positionTarget);
}

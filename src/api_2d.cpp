#include "api_2d.h"

inline float normalize_angle(float angle) {
    while (angle > M_PI)  angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

ASNAV::ASNAV(ros::NodeHandle& nh) : nh_(nh)
{
    // 从参数服务器获取参数
    ros::NodeHandle nh_private("~");
    nh_private.param<bool>("is_auto_land", is_auto_land, true);
    nh_private.param<std::string>("target_class_name", target_class_name, "red_ballon");
    nh_private.param<float>("fly_height", fly_height, 0.5f);
    nh_private.param<float>("descend_z", descend_z, 0.3f);

    // 初始化订阅和发布
    mavros_state_sub_ = nh_.subscribe("/mavros/state", 10, &ASNAV::mavros_state_cb, this);
    mavros_local_position_pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &ASNAV::mavros_local_position_pose_cb, this);
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_cmd_command_client_ = nh_.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    planner_cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &ASNAV::planner_cmd_vel_cb, this);
    mavros_setpoint_raw_local_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    yolo_info_sub_ = nh_.subscribe("/yolov8/BoundingBoxes", 10, &ASNAV::yolo_info_cb, this);
    ac_ = new MoveBaseClient("move_base", true);
    ROS_INFO("等待 move_base 服务启动...");
    ac_->waitForServer(); 
    ROS_INFO("move_base 已连接");

    is_offboard = false;
    current_position = geometry_msgs::Point();
    target_position = mavros_msgs::PositionTarget();
    planner_velxy_posz = geometry_msgs::Point();
    start_planning_time = 0;
    finish_planning_time = 0;
    goal_sent_ = false;
}

ASNAV::~ASNAV()
{
}
// 起飞接口
bool ASNAV::takeoff(float height)
{   
    ros::Rate rate(20);
    while (!current_state.connected && ros::ok())
    {
        ROS_INFO_THROTTLE(1.0, "等待飞控连接中.....");
        ros::spinOnce();
        rate.sleep();    
    }
    ROS_INFO("飞控连接成功，准备进入OFFBOARD模式");

    for(int i = 0; i < 100 && ros::ok() ; ++i)
    {
        ros::spinOnce();
        rate.sleep();
    }

    position(0.0f, 0.0f, height, 0.0f, 0.2f);

    for(int i = 0; i < 100 && ros::ok() ; ++i)
    {
        setpointPublish();
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok()) 
    {
    setpointPublish();
    start_planning_time = ros::Time::now().toSec();
    if(!is_offboard)
     {
        if(current_state.armed && current_state.mode == "OFFBOARD")
        {
            is_offboard = true;
            ROS_INFO("已切换到OFFBOARD模式");
            ROS_INFO("正在上升到目标高度: %.2f m", height);
        }
        else
        {
            is_offboard = false;
        }
     }
    if (std::fabs(current_position.z - height) < 0.1f)
    {
        ROS_INFO("已达到目标高度: %.2f m", height);
        return true;
    }
    ros::spinOnce();
    rate.sleep();
    }
    return false;
}
// 位置控制接口
bool ASNAV::position(float x, float y, float z, float yaw, float tol)
{
    // if (!is_position_inited)
    // {
    //     return false;
    // }
    // float cos_yaw = cos(initial_yaw);
    // float sin_yaw = sin(initial_yaw);
    // float rotated_x = cos_yaw * x - sin_yaw * y;
    // float rotated_y = sin_yaw * x + cos_yaw * y;
    // float target_x = initial_position.x + rotated_x;
    // float target_y = initial_position.y + rotated_y;
    // float target_z = initial_position.z + z;
    // float target_yaw = initial_yaw + yaw;
    
    target_position.header.stamp = ros::Time::now();
    target_position.coordinate_frame =
    mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                                mavros_msgs::PositionTarget::IGNORE_VY |
                                mavros_msgs::PositionTarget::IGNORE_VZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::FORCE |
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    target_position.position.x = x;
    target_position.position.y = y;
    target_position.position.z = z;
    target_position.yaw = current_yaw;
    return tolerance(x, y, z) < tol;
}
//位置平滑接口
bool ASNAV::positionSmooth(float target_x, float target_y, float target_z, float tol, float hover_sec)
{
    // 1. 静态变量初始化
    static float last_target_x = -999.0f;
    static float last_target_y = -999.0f;
    static float last_target_z = -999.0f;
    static float locked_yaw = 0.0f;      // 新增：用于锁死起始时刻的角度
    static geometry_msgs::Point virtual_sp;
    
    static bool is_hovering = false;
    static ros::Time hover_start_time;
    static bool first_run = true;

    // 2. 检测新任务
    bool is_new_task = (std::abs(target_x - last_target_x) > 0.05f || 
                        std::abs(target_y - last_target_y) > 0.05f || 
                        std::abs(target_z - last_target_z) > 0.05f);

    if (is_new_task || first_run) 
    {
        // 关键点：记录起始位置和当前的 Yaw
        virtual_sp = current_position;
        locked_yaw = current_yaw; // 在这一瞬间锁死角度，不再变化
        
        last_target_x = target_x;
        last_target_y = target_y;
        last_target_z = target_z;
        is_hovering = false;
        first_run = false;
        ROS_INFO("平滑任务启动 [保持航向]: (%.2f, %.2f) -> (%.2f, %.2f) Yaw: %.2f", 
                 virtual_sp.x, virtual_sp.y, target_x, target_y, locked_yaw);
    }

    float dist_to_final = std::sqrt(std::pow(target_x - current_position.x, 2) + 
                                    std::pow(target_y - current_position.y, 2));

    // 3. 构建 MAVROS 消息
    target_position = mavros_msgs::PositionTarget(); 
    target_position.header.stamp = ros::Time::now();
    target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    if (!is_hovering) {
        // 掩码设置：我们依然控制位置和绝对偏航(Yaw)，但发送的角度始终不变
        target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                                mavros_msgs::PositionTarget::IGNORE_VY |
                                mavros_msgs::PositionTarget::IGNORE_VZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::FORCE |
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        float dx = target_x - virtual_sp.x;
        float dy = target_y - virtual_sp.y;
        float dz = target_z - virtual_sp.z;
        float dist_sp = std::sqrt(dx * dx + dy * dy);

        // 步长限制
        float base_step_xy = 0.035f; // 正常速度 0.5m/s
        if (dist_to_final < 0.5f) {
            base_step_xy = 0.01f;    // 接近目标时减速到 0.2m/s，防止冲过头
        }

        if (dist_sp > base_step_xy) {
            virtual_sp.x += (dx / dist_sp) * base_step_xy;
            virtual_sp.y += (dy / dist_sp) * base_step_xy;
        } else {
            virtual_sp.x = target_x;
            virtual_sp.y = target_y;
        }
        
        // 高度处理略... (保持之前的逻辑)
        virtual_sp.z = (std::abs(dz) > 0.015f) ? (virtual_sp.z + (dz > 0 ? 0.015f : -0.015f)) : target_z;

        target_position.position.x = virtual_sp.x;
        target_position.position.y = virtual_sp.y;
        target_position.position.z = virtual_sp.z;
    } else {
        // 悬停制动阶段，发死最终目标点
        target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                                    mavros_msgs::PositionTarget::IGNORE_AFY |
                                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                                    mavros_msgs::PositionTarget::FORCE |
                                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        target_position.position.x = target_x;
        target_position.position.y = target_y;
        target_position.position.z = target_z;

        target_position.velocity.x = 0.0f;
        target_position.velocity.y = 0.0f;
        target_position.velocity.z = 0.0f;
    }

    // 重点：无论飞到哪，Yaw 始终发任务开始时锁定的那个值
    target_position.yaw = locked_yaw; 

    // 4. 到达与悬停判断
    if (dist_to_final < tol && std::abs(target_z - current_position.z) < tol) {
        if (!is_hovering) {
            is_hovering = true;
            hover_start_time = ros::Time::now();
            ROS_INFO("进入刹车区，强制零速闭环...");
        } else if ((ros::Time::now() - hover_start_time).toSec() >= hover_sec) {
            last_target_x = -999.0f;
            is_hovering = false;
            return true;
        }
    }
    return false;
}
// 导航接口
bool ASNAV::navigation(float x, float y, float z, float yaw, float tol)
{
    // 1. 如果这是新航点（或者第一次进入），发送 Goal 给 ActionServer
    if (!goal_sent_) {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.header.frame_id = "map"; // 或者是你的 odom/world
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.position.z = z;
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

        ac_->sendGoal(goal);
        goal_sent_ = true;
        ROS_INFO("Action 发送目标: (%.2f, %.2f)", x, y);
    }

    // 2. 执行原本的速度控制逻辑（维持 Offboard 飞行）
    // 只要 move_base 还在算速度 (is_as_received)，我们就把速度喂给飞控
    if (is_as_received)
    {
        target_position.header.stamp = ros::Time::now();
        target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        // 掩码：控制速度和高度
        target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                    mavros_msgs::PositionTarget::IGNORE_PY |
                                    mavros_msgs::PositionTarget::IGNORE_VZ |
                                    mavros_msgs::PositionTarget::IGNORE_AFX |
                                    mavros_msgs::PositionTarget::IGNORE_AFY |
                                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                                    mavros_msgs::PositionTarget::FORCE |
                                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        target_position.velocity.x = planner_velxy_posz.x;
        target_position.velocity.y = planner_velxy_posz.y;
        target_position.position.z = z;
        target_position.yaw = current_yaw; // 或者 yaw 参数
    }

    // 3. 关键：判断是否到达
    // 我们检查 Actionlib 的状态。SUCCEEDED 表示 move_base 觉得自己到了。
    auto state = ac_->getState();
    
    // 如果 move_base 说到了，或者我们的数学计算也觉得到了
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) //|| tolerance(x, y, z) < tol)
    {
        ROS_INFO("导航任务完成！状态: %s, 剩余距离: %.2f", state.toString().c_str(), tolerance(x,y,z));
        goal_sent_ = false;  // 重置标志，给下一个 case 使用
        is_as_received = false; // 停止当前速度指令执行
        return true;         // 返回 true，主状态机 mission_num 才会 ++
    }

    return false; // 还没到，继续循环
}
// 控制Yaw角接口
bool ASNAV::controlYaw(float x, float y, float z, float target_yaw, float wait_sec)
{
    float error = normalize_angle(target_yaw - current_yaw);

    target_position.header.stamp = ros::Time::now();
    target_position.coordinate_frame =
    mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |                    
                                mavros_msgs::PositionTarget::IGNORE_VY |
                                mavros_msgs::PositionTarget::IGNORE_VZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::FORCE |
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    target_position.position.x = x; 
    target_position.position.y = y;
    target_position.position.z = z;
    target_position.yaw = current_yaw; // 实时更新的 Yaw

    if (std::fabs(error) > 0.1f) {
        // 使用定点悬停模式，配合极小的旋转步长 (约 20-30度/秒)
        target_position.position.x = x; // 传入固定的目标点 X
        target_position.position.y = y; // 传入固定的目标点 Y
        target_position.position.z = z; // 传入固定的目标点 Z
        
        // 限制旋转步长 (假设循环 20Hz, 0.05s)
        float max_step = 0.21f; 
        float step = std::min(std::fabs(error), max_step);
        target_position.yaw = current_yaw + (error > 0 ? step : -step);

        return false; // 旋转进行中
    }
    
    if (!is_yaw_finished) 
    {
        yaw_finish_time = ros::Time::now();
        is_yaw_finished = true;
        ROS_INFO("偏航到位，开始悬停稳定...");
    }

    // 3. 检查悬停时间是否达到要求
    if ((ros::Time::now() - yaw_finish_time).toSec() < 2.0f) 
    {
        // 旋转到位后，持续发布该点，强制维持悬停
        target_position.position.x = x;
        target_position.position.y = y;
        target_position.yaw = target_yaw;
        return false; // 悬停未结束
    }

    // 4. 全部完成
    is_yaw_finished = false;
    return true; // 可以进入下一阶段
}
// 飞行下降接口
bool ASNAV::flyDown(float descend_z)
{
   position(current_position.x, current_position.y, descend_z, current_yaw);
    if (std::fabs(current_position.z - descend_z) < 0.1f)
    {
     ROS_INFO("已达到目标高度: %.2f m", descend_z);
     return true;
    }
    return false;
}
// 飞行上升接口
bool ASNAV::flyUp(float height)
{
   position(current_position.x, current_position.y, height, current_yaw);
    if (std::fabs(current_position.z - height) < 0.1f)
    {
     ROS_INFO("已达到目标高度: %.2f m", height);
     return true;
    }
    return false;
}
// 自动降落接口
bool ASNAV::autoLand()
{
    if (!is_auto_land) 
    {
        ROS_WARN("自动降落功能未启用");
        return false;
    }
    else
    {
    set_mode("POSCTL");
    ros::Duration(0.5).sleep();
    set_mode("AUTO.LAND");
    ROS_INFO("已切换到AUTO.LAND模式，正在降落...");
    return true;
    }
    return false;
}
// 下视视觉跟随接口
bool ASNAV::trackYoloDown(float max_distance, int tol)
{
    float err_x = yolo_box_info.cameraXCenter - 320.0f;
    float err_y = yolo_box_info.cameraYCenter - 240.0f;

    // 1. PID 控制器计算 (增加 D 项用于刹车)
    float Kp = 0.003f;
    float Ki = 0.0002f;
    float Kd = 0.005f; // 新增：微分系数，用于抑制震荡

    integral_error_x += err_x;
    integral_error_y += err_y;
    // 缩小抗积分饱和的上限，防止累积误差过大导致冲过头
    integral_error_x = std::clamp(integral_error_x, -500.0f, 500.0f);
    integral_error_y = std::clamp(integral_error_y, -500.0f, 500.0f);

    float diff_err_x = err_x - last_err_x;
    float diff_err_y = err_y - last_err_y;

    // 计算基础控制量
    float u_x = Kp * err_x + Ki * integral_error_x + Kd * diff_err_x;
    float u_y = Kp * err_y + Ki * integral_error_y + Kd * diff_err_y;

    // 更新上一次误差
    last_err_x = err_x;
    last_err_y = err_y;

    // 2. 将像素误差映射到机身坐标系 (Body Frame)
    // 假设相机镜头向下：
    // 目标在图像下方 (err_y > 0) -> 无人机需要向后飞 -> body_x 为负
    // 目标在图像右方 (err_x > 0) -> 无人机需要向右飞 -> body_y 为正
    float body_offset_x = -u_y; 
    float body_offset_y = u_x;  // 注：你原代码这里是负的，可能会导致反向，请检查相机安装方向

    // 3. 将机身坐标系旋转到 LOCAL_NED 坐标系
    // 根据无人机当前的偏航角 (current_yaw) 进行 2D 旋转
    float offset_x = body_offset_x * std::cos(current_yaw) - body_offset_y * std::sin(current_yaw);
    float offset_y = body_offset_x * std::sin(current_yaw) + body_offset_y * std::cos(current_yaw);

    // 4. 步长限幅
    float step_mag = std::sqrt(offset_x * offset_x + offset_y * offset_y);
    if (step_mag > max_distance) 
    {
        float scale = max_distance / step_mag;
        offset_x *= scale;
        offset_y *= scale;
    }

    // 5. 发布控制指令
    target_position.header.stamp = ros::Time::now();
    target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                                mavros_msgs::PositionTarget::IGNORE_VY |
                                mavros_msgs::PositionTarget::IGNORE_VZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::FORCE |
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    target_position.position.x = current_position.x + offset_x;
    target_position.position.y = current_position.y + offset_y;
    target_position.position.z = current_position.z; // 保持当前高度
    target_position.yaw = current_yaw;               // 保持当前航向

    // 6. 成功对齐判定
    if (std::abs(err_x) < tol && std::abs(err_y) < tol)
    {
        ROS_INFO("图像目标已对准");
        // 对准后清除积分，防止下次追踪时带入旧的历史误差
        integral_error_x = 0.0f;
        integral_error_y = 0.0f;
        return true;
    }
    return false;   
}
// 正向视觉跟随接口
bool ASNAV::trackYoloForward(float Kp_x, float Kp_y, float Kp_z, float target_box_height, int tol_xy, int tol_size)
{
    static bool is_traversing = false;
    static float start_x, start_y, start_z, start_yaw;
    static float current_step_dist = 0.0f;
    // 1. 安全机制：检查目标是否丢失超过 2 秒 (复现 .py 脚本逻辑)
    if(!is_traversing)
    {
    if ((ros::Time::now() - last_yolo_time_).toSec() > 2.0)
    {
        ROS_WARN_THROTTLE(1.0, "目标丢失超过2秒，进入悬停模式 (HOVER)");
        target_position.header.stamp = ros::Time::now();
        target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED; // 机体坐标系
        target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                    mavros_msgs::PositionTarget::IGNORE_PY |
                                    mavros_msgs::PositionTarget::IGNORE_PZ |
                                    mavros_msgs::PositionTarget::IGNORE_AFX |
                                    mavros_msgs::PositionTarget::IGNORE_AFY |
                                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                                    mavros_msgs::PositionTarget::IGNORE_YAW;
        // 速度清零，保持航向
        target_position.velocity.x = 0.0f;
        target_position.velocity.y = 0.0f;
        target_position.velocity.z = 0.0f;
        target_position.yaw_rate = 0.0f; 
        return false;
    }
    }

    if(!is_traversing)
    {
    // 2. 误差计算 (假设相机分辨率为 640x480，请根据实际情况修改 320 和 240)
    float img_center_x = 331.0f;
    float img_center_y = 239.0f;

    // 图像坐标系：右正左负，下正上负
    float err_x = yolo_box_info.cameraXCenter - img_center_x; 
    float err_y = yolo_box_info.cameraYCenter - img_center_y; 

    float err_size = target_box_height - yolo_box_info.boxHeight;

    // 3. P控制计算速度
    float vx = Kp_x * err_size;  // X轴：前后跟随
    // 目标在右(err_x > 0) -> 飞机向右侧飞 (Vy > 0)
    float vy = -(Kp_y * err_x);
    // 目标在下(err_y > 0) -> 飞机下降 (NED坐标系中，Z轴正方向为下，Vz > 0)
    float vz = -(Kp_z * err_y);

    // 速度限幅，防止剧烈晃动 (限制在 ±1.0 m/s)
    vy = std::clamp(vy, -1.0f, 1.0f);
    vz = std::clamp(vz, -1.0f, 1.0f);
    vx = std::clamp(vx, -1.0f, 1.0f);

    // 4. 构建 MAVROS 速度控制指令
    target_position.header.stamp = ros::Time::now();
    target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED; // 注意这里换成了 BODY_NED
    target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                mavros_msgs::PositionTarget::IGNORE_PY |
                                mavros_msgs::PositionTarget::IGNORE_PZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::IGNORE_YAW; // 忽略偏航角，但控制偏航角速度

    target_position.position.x = 0;
    target_position.position.y = 0;
    target_position.position.z = 0;
    
    target_position.velocity.x = 0.0f;    // 前后速度
    target_position.velocity.y = vy;    // 左右侧飞跟随
    target_position.velocity.z = vz;    // 上下高度跟随
    target_position.yaw_rate = 0.0f;    // 保持机头航向不变
    
        // 1. 检查是否对准
    if (std::abs(err_x) < tol_xy && std::abs(err_y) < tol_xy) 
    {
        ROS_INFO("对准成功，开始平滑穿框...");
        is_traversing = true;
        start_x = current_position.x;
        start_y = current_position.y;
        start_z = current_position.z;
        start_yaw = current_yaw;
        current_step_dist = 0.0f; 
        return false;
    }
    return false;
    }
    else
    {
        // 2. 步进逻辑：假设控制频率为 30Hz，希望速度为 0.5m/s
        // 每一帧前进的距离 = 0.5 / 30 = 0.016m
        float step_size = 0.04f; 
        float max_dist = 1.0f;

        if (current_step_dist < max_dist) 
        {
            current_step_dist += step_size;
        }

        // 3. 根据起始位置和当前航向，计算出“胡萝卜点”的地图坐标 (ENU)
        float target_x = start_x + current_step_dist * std::cos(start_yaw);
        float target_y = start_y + current_step_dist * std::sin(start_yaw);

        // 4. 调用位置控制函数，注意这里的容差可以设小一点
        bool reached = position(target_x, target_y, start_z, start_yaw, 0.1f);

        // 5. 判定最终终点
        if (current_step_dist >= max_dist && reached) 
        {
            is_traversing = false; // 任务完成，重置状态
            current_step_dist = 0.0f;
            return true; 
        }
        return false; 
    }
    return false;
}
// 视觉追踪接口
bool ASNAV::trackYoloing(float Kp_x, float Kp_y, float Kp_z, float target_box_height, int tol_xy, int tol_size)
{
   if ((ros::Time::now() - last_yolo_time_).toSec() > 2.0)
    {
        ROS_WARN_THROTTLE(1.0, "目标丢失超过2秒，进入悬停模式 (HOVER)");
        target_position.header.stamp = ros::Time::now();
        target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED; // 机体坐标系
        target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                    mavros_msgs::PositionTarget::IGNORE_PY |
                                    mavros_msgs::PositionTarget::IGNORE_PZ |
                                    mavros_msgs::PositionTarget::IGNORE_AFX |
                                    mavros_msgs::PositionTarget::IGNORE_AFY |
                                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                                    mavros_msgs::PositionTarget::IGNORE_YAW;
        // 速度清零，保持航向
        target_position.velocity.x = 0.0f;
        target_position.velocity.y = 0.0f;
        target_position.velocity.z = 0.0f;
        target_position.yaw_rate = 0.0f; 
        return false;
    } 
    // 误差计算 (假设相机分辨率为 640x480，请根据实际情况修改 320 和 240)
    float img_center_x = 320.0f;
    float img_center_y = 240.0f;

    // 图像坐标系：右正左负，下正上负
    float err_x = yolo_box_info.cameraXCenter - img_center_x; 
    float err_y = yolo_box_info.cameraYCenter - img_center_y; 

    float err_size = target_box_height - yolo_box_info.boxHeight;

    // 3. P控制计算速度
    float vx = Kp_x * err_size;  // X轴：前后跟随
    // 调整偏航
    float yaw_rate_val = -(Kp_y * err_x);
    // 目标在右(err_x > 0) -> 飞机向右侧飞 (Vy > 0)
    // float vy = -(Kp_y * err_x);
    // 目标在下(err_y > 0) -> 飞机下降 (NED坐标系中，Z轴正方向为下，Vz > 0)
    float vz = -(Kp_z * err_y);

    // 速度限幅，防止剧烈晃动 (限制在 ±1.0 m/s)
    yaw_rate_val = std::clamp(yaw_rate_val, -1.0f, 1.0f);
    // vy = std::clamp(vy, -1.0f, 1.0f);
    vz = std::clamp(vz, -1.0f, 1.0f);
    vx = std::clamp(vx, -1.0f, 1.0f);

    // 4. 构建 MAVROS 速度控制指令
    target_position.header.stamp = ros::Time::now();
    target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED; // 注意这里换成了 BODY_NED
    target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                mavros_msgs::PositionTarget::IGNORE_PY |
                                mavros_msgs::PositionTarget::IGNORE_PZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::IGNORE_YAW; // 忽略偏航角，但控制偏航角速度

    target_position.velocity.x = vx;    // 前后速度
    target_position.velocity.y = 0.0f;    // 左右侧飞跟随
    target_position.velocity.z = vz;    // 上下高度跟随
    target_position.yaw_rate = yaw_rate_val;    // 保持机头航向不变
    return false;
}

void ASNAV::reset_target()
{
    // 彻底重新构造对象，清空所有 position 和 velocity 残留
    target_position = mavros_msgs::PositionTarget();
    
    // 设置一个“全部忽略”的掩码，确保安全
    target_position.type_mask = 0x7FF; 
    
    // 默认回到局部坐标系
    target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    
    ROS_INFO("指令对象已重置，消除残留数据");
}

bool ASNAV::interceptBalloon(float charge_speed, float Kp_y, float Kp_z, float pop_box_height)
{
    static float last_seen_box_height = 0.0f;

    // 1. 刺破判定 (因为相机离气球还有近2米时气球就破了，所以判定框的高度要根据实测调整)
    if ((ros::Time::now() - last_yolo_time_).toSec() > 0.5)
    {
        // pop_box_height 是你需要实测的：当两米长的枪尖顶到气球时，气球在画面里的高度是多少？
        if (last_seen_box_height > pop_box_height * 0.8) // 稍微放宽一点判定条件
        {
            ROS_INFO("检测到气球爆炸！刺破成功！开始紧急倒车！");
            last_seen_box_height = 0.0f; 
            target_position.type_mask = 0x7BF;
            target_position.velocity.x = 0; target_position.velocity.y = 0; target_position.velocity.z = 0;
            return true; // 返回 true 触发战术撤退
        }
        else
        {
            ROS_WARN_THROTTLE(1.0, "目标跟丢，悬停等待...");
            target_position.type_mask = 0x7BF;
            target_position.velocity.x = 0; target_position.velocity.y = 0; target_position.velocity.z = 0;
            return false;
        }
    }

    float err_x = yolo_box_info.cameraXCenter - 320.0f;
    float err_y = yolo_box_info.cameraYCenter - 270.0f;
    last_seen_box_height = yolo_box_info.boxHeight;

    // 2. 长矛冲锋分级控制策略
    float vx = 0.0f;
    float vy = 0.0f;
    float vz = 0.0f;

    // 这个值你需要根据你的相机焦距和实际情况去操场上推一推飞机来测定
    float danger_box_height = 100.0f; 

    if (yolo_box_info.boxHeight < danger_box_height)
    {
        // ==========================================
        // 远距离阶段：常规追踪（允许微调 Yaw 来大范围跟目标）
        // ==========================================
        vx = charge_speed * 0.5f; // 远距离慢慢飞，保证对准
        vy = std::clamp(-(Kp_y * err_x), -1.0f, 1.0f);
        vz = std::clamp(-(Kp_z * err_y), -1.0f, 1.0f);
    }
    else
    {
        // ==========================================
        // 近距离“长矛冲锋”阶段：绝对锁死 Yaw，全速平移追击！
        // ==========================================
        vx = charge_speed; // 全速冲锋！
        
        // 因为不转机头了，所以必须加大左右和上下的 P 控制力度，纯靠侧飞来追踪转动的气球
        vy = std::clamp(-(Kp_y * err_x), -1.0f, 1.0f); 
        vz = std::clamp(-(Kp_z * err_y), -1.0f, 1.0f);
        
        ROS_INFO_THROTTLE(0.5, "进入冲锋范围，已锁死机头，平移追击！");
    }

    // 3. 发布 MAVROS 速度指令
    target_position.header.stamp = ros::Time::now();
    target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                mavros_msgs::PositionTarget::IGNORE_PY |
                                mavros_msgs::PositionTarget::IGNORE_PZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::IGNORE_YAW;
                                
    target_position.velocity.x = vx;
    target_position.velocity.y = vy;
    target_position.velocity.z = vz;
    target_position.yaw_rate = 0.0f;
    

    return false;
}

bool ASNAV::escapeBackward(float distance, float tol)
{
    static bool is_escaping = false;
    static float escape_target_x, escape_target_y, escape_target_z;

    if (!is_escaping)
    {
        // 计算正后方的绝对坐标
        escape_target_x = current_position.x - distance * std::cos(current_yaw);
        escape_target_y = current_position.y - distance * std::sin(current_yaw);
        escape_target_z = current_position.z;
        is_escaping = true;
        ROS_WARN("启动战术撤退，向后退避 %.1f 米！", distance);
    }

    bool reached = position(escape_target_x, escape_target_y, escape_target_z, current_yaw, tol);
    
    if (reached)
    {
        is_escaping = false;
        return true;
    }
    return false;
}

bool ASNAV::attackBalloon(float charge_speed)
{
    // 内部战术状态：
    // 0 = 索敌与冲锋阶段 (ATTACK)
    // 1 = 战术后退与判定阶段 (RETREAT & ASSESS)
    static int combat_state = 0; 
    static ros::Time retreat_start_time; // 记录开始后退的时间

    // --- 比赛核心参数（必须线下实测） ---
    const float LANCE_OVERSHOOT_HEIGHT = 193.0f; // 杆长设定值：框高于此值说明已越过气球刺空了
    const double POP_CONFIRM_TIME = 2.5;         // 判定戳爆所需的时间：几秒内没看到气球就算爆了
    const float KP_Y = 0.005f;
    const float KP_Z = 0.005f;

    // 当前帧是否能看到气球 (0.3秒内有更新就算看到，过滤掉单帧闪烁)
    bool is_visible = (ros::Time::now() - last_yolo_time_).toSec() < 0.3;

    // ====================================================
    // 状态 0：索敌与冲锋
    // ====================================================
    if (combat_state == 0) 
    {
        if (is_visible)
        {
            // 1. 判定是否刺空（越界）
            if (yolo_box_info.boxHeight > LANCE_OVERSHOOT_HEIGHT)
            {
                ROS_WARN("框高度(%.1f) > 设定杆长，刺偏越界！启动后退重试！", yolo_box_info.boxHeight);
                combat_state = 1;
                retreat_start_time = ros::Time::now(); // 开始计时
                return false;
            }

            // 2. 正常长矛冲锋逻辑（锁死机头，平移追击）
            float err_x = yolo_box_info.cameraXCenter - 320.0f;
            float err_y = yolo_box_info.cameraYCenter - 270.0f;

            float vy = std::clamp(-KP_Y * err_x, -1.0f, 1.0f);
            float vz = std::clamp(-KP_Z * err_y, -1.0f, 1.0f);
            
            target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
            target_position.type_mask = 1471; // 忽略位置，使用速度
            target_position.velocity.x = charge_speed; // 向前冲锋
            target_position.velocity.y = vy;
            target_position.velocity.z = vz;
            target_position.yaw_rate = 0.0f; // 绝对锁死机头！
        }
        else
        {
            // 3. 冲锋途中气球突然消失（可能爆了，也可能被机械臂转走了）
            ROS_INFO("目标突然消失，启动后退进行扎破确认...");
            combat_state = 1;
            retreat_start_time = ros::Time::now();
        }
    }
    // ====================================================
    // 状态 1：后退与判定（容错与确认机制）
    // ====================================================
    else if (combat_state == 1) 
    {
        // 1. 持续发送后退指令
        target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
        target_position.type_mask = 1471;
        target_position.velocity.x = -0.3f; // 以 0.8m/s 的速度往后退
        target_position.velocity.y = 0.0f;
        target_position.velocity.z = 0.0f;
        target_position.yaw_rate = 0.0f;

        // 2. 判定 A：捕捉到气球身影 -> 继续追踪继续戳
        // 注意：加了 * 0.8f 是个专业技巧（滞回区间），防止它在边界值疯狂左右横跳
        if (is_visible && yolo_box_info.boxHeight < (LANCE_OVERSHOOT_HEIGHT * 0.8f)) 
        {
            ROS_INFO("后退拉开距离后重新捕捉到气球，继续发起冲锋！");
            combat_state = 0; // 切回冲锋状态
            return false;
        }

        // 3. 判定 B：几秒内都没有气球 -> 判定戳爆
        if (!is_visible && (ros::Time::now() - retreat_start_time).toSec() > POP_CONFIRM_TIME)
        {
            ROS_INFO("后退验证完成，%.1f 秒内未见目标，判定气球已扎爆！", POP_CONFIRM_TIME);
            combat_state = 0; // 重置内部状态，为打下一个气球做准备
            
            // 发送一次刹车指令清空速度
            target_position.velocity.x = 0.0f; 
            return true; // 告诉主循环：这个目标搞定了！
        }
    }

    return false;
}

// 三通道 PWM 舵机控制接口（复刻 lib_pwm_control，M5/M6/M7，参数 0~100 占空比）
// 2026-10-02: 不再调用预编译库 liblib.so，自实现 CommandLong(187, MAV_CMD_DO_SET_ACTUATOR)
bool ASNAV::pwmControl(int pwm_channel_5, int pwm_channel_6, int pwm_channel_7)
{
    ROS_INFO("PWM 指令发送：M5=%d, M6=%d, M7=%d", pwm_channel_5, pwm_channel_6, pwm_channel_7);

    ros::Rate rate(20); // 20Hz 发送频率，每次 0.05 秒
    for (int i = 0; i < 6; ++i)
    {
        mavros_msgs::CommandLong ctrl_pwm;
        ctrl_pwm.request.command = 187;   // MAV_CMD_DO_SET_ACTUATOR
        ctrl_pwm.request.param1 = (float)((double)pwm_channel_5 / 50.0 - 1.0);  // M5: 0~100 -> -1.0~+1.0
        ctrl_pwm.request.param2 = (float)((double)pwm_channel_6 / 50.0 - 1.0);  // M6
        ctrl_pwm.request.param3 = (float)((double)pwm_channel_7 / 50.0 - 1.0);  // M7
        if (!mavros_cmd_command_client_.call(ctrl_pwm)) {
            ROS_ERROR_ONCE("pwmControl: 调用 /mavros/cmd/command 失败（mavros 未连接飞控？）");
        }

        setpointPublish();   // 维持 OFFBOARD 模式的心跳指令
        ros::spinOnce();
        rate.sleep();
    }
    return true;
}


// MAVROS状态回调
void ASNAV::mavros_state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}
// MAVROS位置回调
void ASNAV::mavros_local_position_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_position.x = msg->pose.position.x;
    current_position.y = msg->pose.position.y;
    current_position.z = msg->pose.position.z;

    //提取并转换姿态（四元数转Yaw）
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat); // 注意这里比 pose.pose 少了一层 pose
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    current_yaw = static_cast<float>(yaw);
    
}
// move_base速度回调
void ASNAV::planner_cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    planner_velxy_posz.x = msg->linear.x;
    planner_velxy_posz.y = msg->linear.y;
    is_as_received = true;
} 
// 判断容差函数
float ASNAV::tolerance(float x, float y, float z) const
{
    return std::sqrt(std::pow(current_position.x - x, 2) +
                     std::pow(current_position.y - y, 2) +
                     std::pow(current_position.z - z, 2));
    ROS_INFO_THROTTLE(1, "可以接受");
}
// 发布目标位置函数
void ASNAV::setpointPublish()
{
        mavros_setpoint_raw_local_pub_.publish(target_position);
}
// 设置飞行模式函数
void ASNAV::set_mode(std::string mode)
{
    mavros_msgs::SetMode mode_msg;
    mode_msg.request.custom_mode = mode;
    if (set_mode_client_.call(mode_msg) && mode_msg.response.mode_sent)
    {
        ROS_INFO("已切换到模式: %s", mode.c_str());
    }
    else
    {
        ROS_ERROR("切换模式失败: %s", mode.c_str());
    }
}
// yolo回调
void ASNAV::yolo_info_cb(const yolov8_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
    if (msg->bounding_boxes.empty())
    {
    ROS_WARN_THROTTLE(1, "YOLO识别结果为空");
    return;
    }
    // 遍历所有识别到的框，寻找标签为 "A" 的目标
    bool found_target = false;
    for (const auto& box : msg->bounding_boxes)
    {
        if (box.Class == target_class_name)  // 这里修改为你想要的标签名称
        {
            yolo_box_info.Class = box.Class;
            yolo_box_info.cameraXCenter = (box.xmax + box.xmin) * 0.5f;
            yolo_box_info.cameraYCenter = (box.ymax + box.ymin) * 0.5f;
            yolo_box_info.boxHeight = box.ymax - box.ymin;

            last_yolo_time_ = ros::Time::now();
            
            ROS_INFO_THROTTLE(1, "成功锁定目标 : (%.2f, %.2f)", 
                              yolo_box_info.cameraXCenter, 
                              yolo_box_info.cameraYCenter);
            
            found_target = true;
            break; 
        }
    }

    if (!found_target)
    {
        ROS_WARN_THROTTLE(1, "未在当前帧中找到目标 ");
    }
}
            
  

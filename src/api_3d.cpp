#include "api_3d.h"

inline float normalize_angle(float angle) {
    while (angle > M_PI)  angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

ASNAV::ASNAV(ros::NodeHandle& nh) : nh_(nh)
{
    // 从参数服务器获取参数
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("target_class_name", target_class_name, "red_ballon");
    nh_private.param<float>("fly_height", fly_height, 0.5f);
    nh_private.param<float>("descend_z", descend_z, 0.3f);


    // navigationEgo 参数加载
    nh_private.param<float>("ego_kp_outer", ego_kp_outer_, 2.5f);
    nh_private.param<float>("ego_kv_outer", ego_kv_outer_, 0.8f);
    nh_private.param<float>("ego_ki_outer", ego_ki_outer_, 0.3f);
    nh_private.param<float>("ego_ff_gain", ego_ff_gain_, 0.5f);
    nh_private.param<float>("ego_ff_gain_z", ego_ff_gain_z_, 0.0f);
    nh_private.param<float>("ego_max_vel", ego_max_vel_, 2.0f);
    nh_private.param<float>("ego_max_vel_z", ego_max_vel_z_, 1.0f);
    nh_private.param<float>("ego_max_integral", ego_max_integral_, 0.5f);
    nh_private.param<float>("ego_traj_timeout", ego_traj_timeout_, 0.5f);

    // 导航轴策略全局默认（nav_mode < 0 时使用；见 NavMode）
    nh_private.param<int>("nav_default_mode", nav_default_mode_, 0);

    // positionSmooth 步长控制参数
    nh_private.param<double>("smooth_step_xy", smooth_step_xy_, 0.18f);
    nh_private.param<double>("smooth_slow_step_xy", smooth_slow_step_xy_, 0.01f);
    nh_private.param<double>("smooth_slow_dist", smooth_slow_dist_, 0.5f);

    // navigationSuper 参数加载（2026-08 自 sim_work 移植）
    nh_private.param<float>("super_kp_outer", super_kp_outer_, 2.5f);
    nh_private.param<float>("super_kv_outer", super_kv_outer_, 0.8f);
    nh_private.param<float>("super_ki_outer", super_ki_outer_, 0.3f);
    nh_private.param<float>("super_ff_gain", super_ff_gain_, 0.5f);
    nh_private.param<float>("super_ff_gain_z", super_ff_gain_z_, 0.0f);
    nh_private.param<float>("super_max_vel", super_max_vel_, 2.0f);
    nh_private.param<float>("super_max_vel_z", super_max_vel_z_, 1.0f);
    nh_private.param<float>("super_max_integral", super_max_integral_, 0.5f);
    nh_private.param<float>("super_traj_timeout", super_traj_timeout_, 0.5f);

    
    // 初始化订阅和发布
    mavros_state_sub_ = nh_.subscribe("/mavros/state", 10, &ASNAV::mavros_state_cb, this);
    mavros_local_position_pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &ASNAV::mavros_local_position_pose_cb, this);
    mavros_local_velocity_sub_ = nh_.subscribe("/mavros/local_position/velocity_local", 10, &ASNAV::mavros_local_velocity_cb, this);
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_cmd_command_client_ = nh_.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    ego_planner_pos_cmd_sub_ = nh_.subscribe("/drone_0_planning/pos_cmd", 10, &ASNAV::ego_planner_pos_cmd_cb, this);
    super_planner_pos_cmd_sub_ = nh_.subscribe("/planning/pos_cmd", 10, &ASNAV::super_planner_pos_cmd_cb, this);
    mavros_setpoint_raw_local_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    super_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    yolo_info_sub_ = nh_.subscribe("/yolov8/BoundingBoxes", 10, &ASNAV::yolo_info_cb, this);
    yolo_d435i_info_sub_ = nh_.subscribe("/yolov8/d435i/BoundingBoxes", 10, &ASNAV::yolo_d435i_info_cb, this);
    ar_pose_sub_ = nh_.subscribe("/ar_pose_marker", 10, &ASNAV::ar_pose_cb, this);

    is_offboard = false;
    goal_sent_ = false;
    super_goal_sent_ = false;
    super_cmd_received_ = false;
    super_rviz_mode_ = false;
    current_position = geometry_msgs::Point();
    target_position = mavros_msgs::PositionTarget();
    start_planning_time = 0;
    finish_planning_time = 0;

    ar_marker_found_ = false;
    ar_position_detec_x_ = 0;
    ar_position_detec_y_ = 0;
    ar_position_detec_z_ = 0;

    integral_egox_ = 0.0;
    integral_egoy_ = 0.0;
    integral_egoz_ = 0.0;
    last_ego_msg_time_ = ros::Time(0);
    ego_rviz_mode_ = false;
    ego_hold_px_ = 0.0;
    ego_hold_py_ = 0.0;
    ego_hold_active_ = false;

    ego_tol_timing_ = false;
    ego_tol_entry_time_ = ros::Time(0);

    integral_spx_ = 0.0;
    integral_spy_ = 0.0;
    integral_spz_ = 0.0;
    last_super_msg_time_ = ros::Time(0);
    super_tol_timing_ = false;
    super_tol_entry_time_ = ros::Time(0);
    super_yaw_finishing_ = false;
    super_yaw_timing_ = false;
    super_yaw_entry_time_ = ros::Time(0);
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

    position(0.0f, 0.0f, height, 0.0f, 0.15f);

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
    if (std::fabs(current_position.z - height) < 0.25f)
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
    target_position.yaw = 0.0f;
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
        float base_step_xy = smooth_step_xy_;
        if (dist_to_final < smooth_slow_dist_) {
            base_step_xy = smooth_slow_step_xy_;
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
        // 悬停制动阶段，发死最终目标点（纯位置保持，与 navigationSuper 锁存一致：
        // 只发位置、忽略速度字段，避免"位置+零速度"双约束与 PX4 位置环打架导致收敛慢/晃动）
        target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                                    mavros_msgs::PositionTarget::IGNORE_VY |
                                    mavros_msgs::PositionTarget::IGNORE_VZ |
                                    mavros_msgs::PositionTarget::IGNORE_AFX |
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

    // 4. 到达与悬停判断（累计计时，不要求连续在tol内）&& std::abs(target_z - current_position.z) < tol
    if (dist_to_final < tol ) {
        if (!is_hovering) {
            is_hovering = true;
            hover_start_time = ros::Time::now();
            ROS_INFO("进入刹车区，强制零速闭环...");
        }
    }
    if (is_hovering && (ros::Time::now() - hover_start_time).toSec() >= hover_sec) {
        last_target_x = -999.0f;
        is_hovering = false;
        return true;
    }
    return false;
}
// 导航接口
bool ASNAV::navigation(float x, float y, float z, float yaw, float tol, float hover_sec)
{
    if (!goal_sent_) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = z;
        goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        goal_pub_.publish(goal);

        goal_sent_ = true;
        ROS_INFO_THROTTLE(1, "[导航] 目标: x=%.2f y=%.2f z=%.2f yaw=%.2f", x, y, z, yaw);
    }

    // 2. 执行原本的速度控制逻辑（维持 Offboard 飞行）
    // 只要 move_base 还在算速度 (is_as_received)，我们就把速度喂给飞控
    if (ego_cmd_received_)
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
        target_position.velocity.x = ego_cmd_.velocity.x;
        target_position.velocity.y = ego_cmd_.velocity.y;
        // target_position.velocity.z = ego_cmd_.velocity.z;
        target_position.position.z = z;
        target_position.yaw = current_yaw; //ego_cmd_.yaw; //current_yaw
    }

    bool pos_reached = (tolerance(x, y, z) < tol);

    // float yaw_error = normalize_angle(yaw - current_yaw);

    // float yaw_tol = 0.1f; 
    // bool yaw_reached = (std::abs(yaw_error) < yaw_tol);

    if (pos_reached) 
    {
        goal_sent_ = false;
        // ROS_WARN("开始精定位！");
        // bool arrived = positionSmooth(x, y, z, tol * 0.5f, hover_sec);
        // if (arrived)
        // {
        //     ROS_INFO_THROTTLE(1, "已精确到达目标点");
        // }
        // else 
        // {
        //     ROS_INFO_THROTTLE(1, "[导航→点位控制] 正在精确靠近目标...");
        // }
        // return arrived;
        return true;
    }
    return false;
}
// 导航+精定位接口
bool ASNAV::navigationWithPosition(float x, float y, float z, float yaw, float tol, float hover_sec)
{
    static bool fine_locked = false;

    if (!fine_locked)
    {
        if (navigationSuper(x, y, z, yaw, tol, false))
        {
            fine_locked = true;
            ROS_WARN("开始精定位！");
        }
        return false;
    }

    if (positionSmooth(x, y, z, 0.1f, hover_sec))
    {
        ROS_INFO("已精确到达目标点");
        fine_locked = false;
        return true;
    }
    return false;
}
// 导航接口（Ego)
bool ASNAV::navigationEgo(float x, float y, float z, float yaw, float tol, bool stop_at_goal, int nav_mode)
{
    // ====== 轴策略解析（2026-10-05）======
    // nav_mode < 0 → 用 launch 全局默认；navigationEgoRviz 会传入 fly_height / 0.0f 作为锁定值
    const int m = (nav_mode < 0) ? nav_default_mode_ : nav_mode;
    const bool use_z   = (m == NAV_FULL || m == NAV_Z_ONLY);
    const bool use_yaw = (m == NAV_FULL || m == NAV_YAW_ONLY);
    const float yaw_cmd = std::isnan(yaw) ? 0.0f : yaw;   // 形参保护：NAN → 0

    ros::Time now = ros::Time::now();
    const float kDebounce = 0.15f;    // 到达防抖
    const double kStopVel = 0.3;      // 末点锁存收敛速度

    // ====== (A) 新目标检测：发布 goal 到 EGO 规划器 ======
    // rviz 测试模式（ego_rviz_mode_）下不发 goal
    if (!goal_sent_ && !ego_rviz_mode_)
    {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = now;
        goal.header.frame_id = "map";
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = z;
        goal.pose.orientation = tf::createQuaternionMsgFromYaw(0.0f);
        goal_pub_.publish(goal);

        goal_sent_ = true;
        last_ego_msg_time_ = now;   // 刷新超时：换点空窗不进超时刹车
        integral_egox_ = 0.0;
        integral_egoy_ = 0.0;
        integral_egoz_ = 0.0;
        ego_tol_timing_ = false;
        ego_hold_active_ = false;   // 新目标退出锁存
        ROS_INFO("[Ego] 新目标 (%.2f, %.2f, %.2f) → 已发布到 /move_base_simple/goal", x, y, z);
    }

    // ====== (B) 轨迹有效判定：无轨迹 / 断联超时 / 轨迹完成 → 位置锁存刹车 ======
    double dt_since_ego =
        last_ego_msg_time_.isZero() ? 0.0 : (now - last_ego_msg_time_).toSec();
    bool traj_timeout = (ego_cmd_received_ && dt_since_ego > ego_traj_timeout_);
    bool traj_completed = ego_cmd_received_ &&
        (ego_cmd_.trajectory_flag == quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED);
    if (!ego_cmd_received_ || traj_timeout || traj_completed)
    {
        if (traj_timeout)
            ROS_WARN_THROTTLE(1.0,
                "[Ego] Traj timeout! dt=%.2fs > %.2fs → HOLD(pos-hold)",
                dt_since_ego, ego_traj_timeout_);

        if (!ego_hold_active_)
        {
            ego_hold_active_ = true;
            ego_hold_px_ = current_position.x;
            ego_hold_py_ = current_position.y;
        }
        target_position.header.stamp = now;
        target_position.coordinate_frame =
            mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        target_position.type_mask =
            mavros_msgs::PositionTarget::IGNORE_VX |
            mavros_msgs::PositionTarget::IGNORE_VY |
            mavros_msgs::PositionTarget::IGNORE_VZ |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::FORCE |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        target_position.position.x = ego_hold_px_;
        target_position.position.y = ego_hold_py_;
        // Z 跟规划器时：锁存最后一条轨迹的高度（从未收到则保持当前高度）；
        // Z 锁定时：用调用方传入的 z。本分支 type_mask 走位置控制，z 必须给位置量。
        target_position.position.z = use_z
            ? (ego_cmd_received_ ? ego_cmd_.position.z : current_position.z)
            : z;
        target_position.velocity.x = 0.0;
        target_position.velocity.y = 0.0;
        target_position.yaw = use_yaw ? 0.0f : yaw_cmd;
        return false;
    }

    // 有新鲜轨迹 → 复位锁存
    ego_hold_active_ = false;

    // ====== (C) PD + 参考速度前馈（XYZ 统一为速度控制量） ======
    double ex = ego_cmd_.position.x - current_position.x;
    double ey = ego_cmd_.position.y - current_position.y;
    double dvx = ego_cmd_.velocity.x - current_velocity.x;
    double dvy = ego_cmd_.velocity.y - current_velocity.y;

    // Z 目标：跟规划器时用轨迹高度，锁定时用调用方传入的 z（与输出侧 use_z 统一）
    double z_ref = use_z ? ego_cmd_.position.z : z;
    double ez = z_ref - current_position.z;
    double dvz = ego_cmd_.velocity.z - current_velocity.z;

    // 无条件积分（固定步长 0.02）
    integral_egox_ += ex * 0.02;
    integral_egoy_ += ey * 0.02;
    integral_egoz_ += ez * 0.02;
    integral_egox_ = std::max(-(double)ego_max_integral_, std::min(integral_egox_, (double)ego_max_integral_));
    integral_egoy_ = std::max(-(double)ego_max_integral_, std::min(integral_egoy_, (double)ego_max_integral_));
    integral_egoz_ = std::max(-(double)ego_max_integral_, std::min(integral_egoz_, (double)ego_max_integral_));

    double vx_cmd = ego_kp_outer_ * ex + ego_kv_outer_ * dvx + ego_ki_outer_ * integral_egox_ + ego_ff_gain_ * ego_cmd_.velocity.x;
    double vy_cmd = ego_kp_outer_ * ey + ego_kv_outer_ * dvy + ego_ki_outer_ * integral_egoy_ + ego_ff_gain_ * ego_cmd_.velocity.y;
    double vz_cmd = ego_kp_outer_ * ez + ego_kv_outer_ * dvz + ego_ki_outer_ * integral_egoz_ + ego_ff_gain_z_ * ego_cmd_.velocity.z;

    // XY 速度幅值限幅
    double speed = std::hypot(vx_cmd, vy_cmd);
    if (speed > ego_max_vel_)
    {
        double scale = ego_max_vel_ / speed;
        vx_cmd *= scale;
        vy_cmd *= scale;
    }
    // Z 速度独立限幅
    vz_cmd = std::max(-(double)ego_max_vel_z_, std::min(vz_cmd, (double)ego_max_vel_z_));

    // 构建消息：XY 速度 + Z（速度/位置由 nav_mode 决定）+ yaw
    target_position.header.stamp = now;
    target_position.coordinate_frame =
        mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target_position.type_mask =
        mavros_msgs::PositionTarget::IGNORE_PX |
        mavros_msgs::PositionTarget::IGNORE_PY |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::FORCE |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    if (use_z)
        target_position.type_mask |= mavros_msgs::PositionTarget::IGNORE_PZ;

    target_position.velocity.x = vx_cmd;
    target_position.velocity.y = vy_cmd;
    if (use_z)
        target_position.velocity.z = vz_cmd;   // Z 跟规划器：速度控制
    else
        target_position.position.z = z;        // Z 锁定：PX4 位置控制，用形参高度
    target_position.yaw = use_yaw ? ego_cmd_.yaw : yaw_cmd;

    // rviz 调参模式：永不到达
    if (ego_rviz_mode_)
        return false;

    // ====== (D) 到达判定：2D 距离 + debounce（无速度闸） ======
    float dist = std::hypot(x - current_position.x, y - current_position.y);
    if (dist < tol)
    {
        if (!ego_tol_timing_)
        {
            ego_tol_timing_ = true;
            ego_tol_entry_time_ = now;
            ROS_INFO("[Ego] 进入容差 dist=%.2f<%.2f, debounce %.2fs...", dist, tol, kDebounce);
        }
        else if ((now - ego_tol_entry_time_).toSec() > kDebounce)
        {
            if (!stop_at_goal)
            {
                // 中间点：到达即完成，下一 case 立刻发下一目标（动着换点）
                ROS_INFO("[Ego] 到达（中间点）");
                ego_tol_timing_ = false;
                goal_sent_ = false;
                integral_egox_ = 0.0;
                integral_egoy_ = 0.0;
                integral_egoz_ = 0.0;
                return true;
            }
            // 末点：位置锁存收敛（HOVER_HIGH），v<0.3 才完成
            if (!ego_hold_active_)
            {
                ego_hold_active_ = true;
                ego_hold_px_ = current_position.x;
                ego_hold_py_ = current_position.y;
            }
            target_position.header.stamp = now;
            target_position.coordinate_frame =
                mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            target_position.type_mask =
                mavros_msgs::PositionTarget::IGNORE_VX |
                mavros_msgs::PositionTarget::IGNORE_VY |
                mavros_msgs::PositionTarget::IGNORE_VZ |
                mavros_msgs::PositionTarget::IGNORE_AFX |
                mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ |
                mavros_msgs::PositionTarget::FORCE |
                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            target_position.position.x = ego_hold_px_;
            target_position.position.y = ego_hold_py_;
            target_position.position.z = use_z
                ? (ego_cmd_received_ ? ego_cmd_.position.z : current_position.z)
                : z;
            target_position.velocity.x = 0.0;
            target_position.velocity.y = 0.0;
            target_position.yaw = use_yaw ? 0.0f : yaw_cmd;
            if (std::hypot(current_velocity.x, current_velocity.y) < kStopVel)
            {
                ROS_INFO("[Ego] 到达（末点，位置锁存收敛）");
                ego_tol_timing_ = false;
                goal_sent_ = false;
                ego_hold_active_ = false;
                integral_egox_ = 0.0;
                integral_egoy_ = 0.0;
                integral_egoz_ = 0.0;
                return true;
            }
        }
    }
    else
    {
        ego_tol_timing_ = false;
    }
    return false;
}
// 纯跟随 ego 轨迹（点击飞行模式）：不发自己的目标，目标来自 RViz 2D Nav Goal
bool ASNAV::navigationEgoRviz(int nav_mode)
{
    ROS_INFO("[EgoRviz] 只接收 rviz 打点，跟踪 EGO 轨迹（navigationEgo 控制律）...");
    ros::Rate rate(50.0);

    ego_rviz_mode_ = true;
    ego_cmd_received_ = false;   // 重新等一个 rviz 点触发的轨迹
    integral_egox_ = 0.0;
    integral_egoy_ = 0.0;
    integral_egoz_ = 0.0;
    ego_hold_active_ = false;

    while (ros::ok())
    {
        // rviz 模式下 x/y 参数被忽略（不发 goal、不判到达）；
        // z/yaw 作为「锁定值」传入：nav_mode 开放对应轴时它们不被使用
        navigationEgo(0.0f, 0.0f, fly_height, 0.0f, 0.2f, false, nav_mode);
        setpointPublish();
        ros::spinOnce();
        rate.sleep();
    }

    ego_rviz_mode_ = false;
    return false;   // 永不返回 true
}
// ====== navigationSuper: SUPER 规划器接口
bool ASNAV::navigationSuper(float x, float y, float z, float yaw, float tol, bool stop_at_goal, int nav_mode)
{
    // ====== 轴策略解析（2026-10-05）======
    // nav_mode < 0 → 用 launch 全局默认；navigationSuperRviz 会传入 fly_height / 0.0f 作为锁定值
    const int m = (nav_mode < 0) ? nav_default_mode_ : nav_mode;
    const bool use_z   = (m == NAV_FULL || m == NAV_Z_ONLY);
    const bool use_yaw = (m == NAV_FULL || m == NAV_YAW_ONLY);
    const float yaw_cmd = std::isnan(yaw) ? 0.0f : yaw;   // 形参保护：NAN → 0

    ros::Time now = ros::Time::now();
    const float kDebounce = 0.15f;    // 到达防抖
    const double kStopVel = 0.3;      // 末点锁存收敛速度

    // ====== (A) 新目标检测：发布 goal 到 SUPER ======
    // rviz 测试模式（super_rviz_mode_）下不发 goal
    if (!super_goal_sent_ && !super_rviz_mode_)
    {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = now;
        goal.header.frame_id = "map";
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = z;
        goal.pose.orientation = tf::createQuaternionMsgFromYaw(0.0f);
        super_goal_pub_.publish(goal);

        super_goal_sent_ = true;
        last_super_msg_time_ = now;   // 刷新超时：换点空窗不进超时刹车
        integral_spx_ = 0.0;
        integral_spy_ = 0.0;
        integral_spz_ = 0.0;
        super_tol_timing_ = false;
        super_hold_active_ = false;   // 新目标退出锁存
        super_yaw_finishing_ = false; // 新目标退出 yaw 收尾
        ROS_INFO("[Super] 新目标 (%.2f, %.2f, %.2f) → 已发布到 /move_base_simple/goal", x, y, z);
    }

    // ====== (A2) yaw 收尾（2026-10-05 恢复）======
    // 仅在 use_yaw（nav_mode 开放机头朝向）时启用：位置容差满足后本接口本可直接返回
    // true，但 SUPER 自身还在把机头转向目标 yaw；若立刻进入下一任务，yaw 指令会被打断。
    // 故先发零速 + 目标 yaw，等 yaw 转入容差（防抖）后再返回 true。
    if (super_yaw_finishing_)
    {
        float yaw_err = std::fabs(normalize_angle(yaw_cmd - current_yaw));

        target_position.header.stamp = now;
        target_position.coordinate_frame =
            mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        target_position.type_mask =
            mavros_msgs::PositionTarget::IGNORE_PX |
            mavros_msgs::PositionTarget::IGNORE_PY |
            mavros_msgs::PositionTarget::IGNORE_VZ |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::FORCE |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;   // yaw 有效
        target_position.velocity.x = 0;
        target_position.velocity.y = 0;
        target_position.position.z = use_z
            ? (super_cmd_received_ ? super_cmd_.position.z : current_position.z)
            : z;
        target_position.velocity.z = 0;
        target_position.yaw = yaw_cmd;

        const float kYawTol = 0.15f;      // yaw 到位阈值 (rad ≈ 8.6°)
        const float kYawDebounce = 0.3f;  // yaw 到位防抖
        if (yaw_err < kYawTol)
        {
            if (!super_yaw_timing_)
            {
                super_yaw_timing_ = true;
                super_yaw_entry_time_ = now;
            }
            else if ((now - super_yaw_entry_time_).toSec() > kYawDebounce)
            {
                ROS_INFO("[Super] yaw 收尾完成 err=%.3f rad → 到达!", yaw_err);
                super_yaw_finishing_ = false;
                super_yaw_timing_ = false;
                super_goal_sent_ = false;
                integral_spx_ = 0.0;
                integral_spy_ = 0.0;
                integral_spz_ = 0.0;
                return true;
            }
        }
        else
        {
            super_yaw_timing_ = false;
        }
        return false;
    }

    // ====== (B) 轨迹有效判定：无轨迹 / 断联超时 / 轨迹完成 → 位置锁存刹车 ======
    // 无轨迹时本可零速悬停；这里保留 holdfix 位置锁存（PX4 位置环），
    // 防零速不锁位的漂移（run30 实测 v_cmd=0 空窗漂 0.4m+）。
    double dt_since_super =
        last_super_msg_time_.isZero() ? 0.0 : (now - last_super_msg_time_).toSec();
    bool traj_timeout = (super_cmd_received_ && dt_since_super > super_traj_timeout_);
    bool traj_completed = super_cmd_received_ &&
        (super_cmd_.trajectory_flag == quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED);
    if (!super_cmd_received_ || traj_timeout || traj_completed)
    {
        if (traj_timeout)
            ROS_WARN_THROTTLE(1.0,
                "[Super] Traj timeout! dt=%.2fs > %.2fs → HOLD(pos-hold)",
                dt_since_super, super_traj_timeout_);

        if (!super_hold_active_)
        {
            super_hold_active_ = true;
            super_hold_px_ = current_position.x;
            super_hold_py_ = current_position.y;
        }
        target_position.header.stamp = now;
        target_position.coordinate_frame =
            mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        target_position.type_mask =
            mavros_msgs::PositionTarget::IGNORE_VX |
            mavros_msgs::PositionTarget::IGNORE_VY |
            mavros_msgs::PositionTarget::IGNORE_VZ |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::FORCE |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        target_position.position.x = super_hold_px_;
        target_position.position.y = super_hold_py_;
        // Z 跟规划器时：锁存最后一条轨迹的高度（从未收到则保持当前高度，避免被压向原点）；
        // Z 锁定时：用调用方传入的 z。本分支 type_mask 走位置控制，z 必须给位置量。
        target_position.position.z = use_z
            ? (super_cmd_received_ ? super_cmd_.position.z : current_position.z)
            : z;
        target_position.velocity.x = 0.0;
        target_position.velocity.y = 0.0;
        target_position.yaw = use_yaw ? 0.0f : yaw_cmd;
        return false;
    }

    // 有新鲜轨迹 → 复位锁存
    super_hold_active_ = false;

    // ====== (C) PD + 参考速度前馈（XYZ 统一为速度控制量） ======
    // 2026-10-02: Z 由「直发 position.z 交给 PX4 位置环」改为与本接口 XY
    // 同构的速度控制（对齐 fuel_nav）。仿真下 z 估计噪声大，位置环会全通透
    // 放每一个 z 抖动，是高度震荡的主因之一。
    double ex = super_cmd_.position.x - current_position.x;
    double ey = super_cmd_.position.y - current_position.y;
    double dvx = super_cmd_.velocity.x - current_velocity.x;
    double dvy = super_cmd_.velocity.y - current_velocity.y;

    // Z 目标：跟规划器时用轨迹高度，锁定时用调用方传入的 z（与输出侧 use_z 统一）
    double z_ref = use_z ? super_cmd_.position.z : z;
    double ez = z_ref - current_position.z;
    double dvz = super_cmd_.velocity.z - current_velocity.z;

    // 无条件积分（固定步长 0.02）
    integral_spx_ += ex * 0.02;
    integral_spy_ += ey * 0.02;
    integral_spz_ += ez * 0.02;
    integral_spx_ = std::max(-(double)super_max_integral_, std::min(integral_spx_, (double)super_max_integral_));
    integral_spy_ = std::max(-(double)super_max_integral_, std::min(integral_spy_, (double)super_max_integral_));
    integral_spz_ = std::max(-(double)super_max_integral_, std::min(integral_spz_, (double)super_max_integral_));

    double vx = super_kp_outer_ * ex + super_kv_outer_ * dvx + super_ki_outer_ * integral_spx_ + super_ff_gain_ * super_cmd_.velocity.x;
    double vy = super_kp_outer_ * ey + super_kv_outer_ * dvy + super_ki_outer_ * integral_spy_ + super_ff_gain_ * super_cmd_.velocity.y;
    double vz = super_kp_outer_ * ez + super_kv_outer_ * dvz + super_ki_outer_ * integral_spz_ + super_ff_gain_z_ * super_cmd_.velocity.z;

    // XY 速度幅值限幅
    double speed = std::hypot(vx, vy);
    if (speed > super_max_vel_)
    {
        double scale = super_max_vel_ / speed;
        vx *= scale;
        vy *= scale;
    }
    // Z 速度独立限幅（super_max_vel_z_ 此前只加载未使用，这里启用）
    vz = std::max(-(double)super_max_vel_z_, std::min(vz, (double)super_max_vel_z_));

    // 构建消息：XY 速度 + Z（速度/位置由 nav_mode 决定）+ yaw
    target_position.header.stamp = now;
    target_position.coordinate_frame =
        mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target_position.type_mask =
        mavros_msgs::PositionTarget::IGNORE_PX |
        mavros_msgs::PositionTarget::IGNORE_PY |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::FORCE |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    if (use_z)
        target_position.type_mask |= mavros_msgs::PositionTarget::IGNORE_PZ;

    target_position.velocity.x = vx;
    target_position.velocity.y = vy;
    if (use_z)
        target_position.velocity.z = vz;   // Z 跟规划器：速度控制
    else
        target_position.position.z = z;    // Z 锁定：PX4 位置控制，用形参高度
    target_position.yaw = use_yaw ? super_cmd_.yaw : yaw_cmd;

    // rviz 调参模式：永不到达
    if (super_rviz_mode_)
        return false;

    // ====== (D) 到达判定：2D 距离 + debounce（无速度闸） ======
    float dist = std::hypot(x - current_position.x, y - current_position.y);
    if (dist < tol)
    {
        if (!super_tol_timing_)
        {
            super_tol_timing_ = true;
            super_tol_entry_time_ = now;
            ROS_INFO("[Super] 进入容差 dist=%.2f<%.2f, debounce %.2fs...", dist, tol, kDebounce);
        }
        else if ((now - super_tol_entry_time_).toSec() > kDebounce)
        {
            // ====== yaw 收尾入口（2026-10-05 恢复）======
            // 启用 yaw 时：位置已到位但机头还没转到目标 yaw → 先进入收尾阶段，等 yaw 收敛
            // 再返回 true。参考轨迹仍在移动时不进入（避免高速清零 XY 造成惯性过冲）。
            if (use_yaw)
            {
                double ref_spd = std::hypot(super_cmd_.velocity.x, super_cmd_.velocity.y);
                float yaw_err = std::fabs(normalize_angle(yaw_cmd - current_yaw));
                if (ref_spd < 0.15 && yaw_err > 0.15f)
                {
                    super_yaw_finishing_ = true;
                    super_yaw_timing_ = false;
                    ROS_INFO("[Super] 位置到位且参考停止, yaw err=%.3f rad > 0.15 → 进入 yaw 收尾", yaw_err);
                    return false;
                }
            }
            if (!stop_at_goal)
            {
                // 中间点：到达即完成，下一 case 立刻发下一目标（动着换点）
                ROS_INFO("[Super] 到达（中间点）");
                super_tol_timing_ = false;
                super_goal_sent_ = false;
                integral_spx_ = 0.0;
                integral_spy_ = 0.0;
                integral_spz_ = 0.0;
                return true;
            }
            // 末点：位置锁存收敛（HOVER_HIGH），v<0.3 才完成
            if (!super_hold_active_)
            {
                super_hold_active_ = true;
                super_hold_px_ = current_position.x;
                super_hold_py_ = current_position.y;
            }
            target_position.header.stamp = now;
            target_position.coordinate_frame =
                mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            target_position.type_mask =
                mavros_msgs::PositionTarget::IGNORE_VX |
                mavros_msgs::PositionTarget::IGNORE_VY |
                mavros_msgs::PositionTarget::IGNORE_VZ |
                mavros_msgs::PositionTarget::IGNORE_AFX |
                mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ |
                mavros_msgs::PositionTarget::FORCE |
                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            target_position.position.x = super_hold_px_;
            target_position.position.y = super_hold_py_;
            target_position.position.z = use_z
                ? (super_cmd_received_ ? super_cmd_.position.z : current_position.z)
                : z;
            target_position.velocity.x = 0.0;
            target_position.velocity.y = 0.0;
            target_position.yaw = use_yaw ? 0.0f : yaw_cmd;
            if (std::hypot(current_velocity.x, current_velocity.y) < kStopVel)
            {
                ROS_INFO("[Super] 到达（末点，位置锁存收敛）");
                super_tol_timing_ = false;
                super_goal_sent_ = false;
                super_hold_active_ = false;
                integral_spx_ = 0.0;
                integral_spy_ = 0.0;
                integral_spz_ = 0.0;
                return true;
            }
        }
    }
    else
    {
        super_tol_timing_ = false;
    }
    return false;
}
// ====== navigationSuperRviz: 只接收 rviz 打点的 SUPER 测试接口（复用 navigationSuper 控制律） ======
bool ASNAV::navigationSuperRviz(int nav_mode)
{
    ROS_INFO("[SuperRviz] 只接收 rviz 打点，跟踪 SUPER 轨迹（navigationSuper 控制律）...");
    ros::Rate rate(50.0);

    super_rviz_mode_ = true;
    super_cmd_received_ = false;   // 重新等一个 rviz 点触发的轨迹
    integral_spx_ = 0.0;
    integral_spy_ = 0.0;
    integral_spz_ = 0.0;
    super_hold_active_ = false;

    while (ros::ok())
    {
        // rviz 模式下 x/y 参数被忽略（不发 goal、不判到达）；
        // z/yaw 作为「锁定值」传入：nav_mode 开放对应轴时它们不被使用
        navigationSuper(0.0f, 0.0f, fly_height, 0.0f, 0.2f, false, nav_mode);
        setpointPublish();
        ros::spinOnce();
        rate.sleep();
    }

    super_rviz_mode_ = false;
    return false;   // 永不返回 true
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

    if (std::fabs(error) > 0.02f) {
        // 使用定点悬停模式，配合极小的旋转步长 (约 20-30度/秒)
        target_position.position.x = x; // 传入固定的目标点 X
        target_position.position.y = y; // 传入固定的目标点 Y
        target_position.position.z = z; // 传入固定的目标点 Z
        
        // 限制旋转步长 (假设循环 20Hz, 0.05s)
        float max_step = 0.66f; 
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
    if ((ros::Time::now() - yaw_finish_time).toSec() < wait_sec) 
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
    set_mode("POSCTL");
    ros::Duration(0.5).sleep();
    set_mode("AUTO.LAND");
    ROS_INFO("已切换到AUTO.LAND模式，正在降落...");
    return true;
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
    static bool was_approaching = false;  // 记录是否曾进入逼近状态（用于判断”目标消失=扎破”）

    double data_age = (ros::Time::now() - last_yolo_d435i_time_).toSec();

    // 1. 安全机制：检查目标是否丢失超过 2 秒
    if (data_age > 1.0)
    {
        if (was_approaching)
        {
            // 逼近中目标消失 → 气球已扎破，立即刹车返回
            ROS_INFO("逼近中目标消失(%.1fs)，判定气球已扎破! 刹车!", data_age);
            was_approaching = false;
            target_position.header.stamp = ros::Time::now();
            target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
            target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                        mavros_msgs::PositionTarget::IGNORE_PY |
                                        mavros_msgs::PositionTarget::IGNORE_PZ |
                                        mavros_msgs::PositionTarget::IGNORE_AFX |
                                        mavros_msgs::PositionTarget::IGNORE_AFY |
                                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                                        mavros_msgs::PositionTarget::IGNORE_YAW;
            target_position.velocity.x = 0.0f;
            target_position.velocity.y = 0.0f;
            target_position.velocity.z = 0.0f;
            target_position.yaw_rate = 0.0f;
            return true;
        }

        // 未逼近但数据过期 → 悬停等待，不用旧数据
        ROS_WARN_THROTTLE(1.0, "D435i数据过期(%.1fs)，悬停等待...\"", data_age);
        target_position.header.stamp = ros::Time::now();
        target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
        target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                    mavros_msgs::PositionTarget::IGNORE_PY |
                                    mavros_msgs::PositionTarget::IGNORE_PZ |
                                    mavros_msgs::PositionTarget::IGNORE_AFX |
                                    mavros_msgs::PositionTarget::IGNORE_AFY |
                                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                                    mavros_msgs::PositionTarget::IGNORE_YAW;
        target_position.velocity.x = 0.0f;
        target_position.velocity.y = 0.0f;
        target_position.velocity.z = 0.0f;
        target_position.yaw_rate = 0.0f;
        return false;
    }

    // 2. 误差计算 (假设相机分辨率为 640x480，请根据实际情况修改 320 和 240)
    float img_center_x = 320.0f;
    float img_center_y = 245.0f;

    // 图像坐标系：右正左负，下正上负
    float err_x = yolo_d435i_box_info_.cameraXCenter - img_center_x;
    float err_y = yolo_d435i_box_info_.cameraYCenter - img_center_y;

    // err_size > 0 表示目标太小（距离太远），err_size < 0 表示目标太大（距离太近）
    float err_size = target_box_height - yolo_d435i_box_info_.boxHeight;

    // 3. P控制计算速度
    float vx = Kp_x * err_size;   // 前后：目标太小→前进，目标太大→后退
    float vy = -(Kp_y * err_x);   // 左右：目标偏右→右侧飞
    float vz = -(Kp_z * err_y);   // 上下：目标偏下→下降 (NED坐标系Z正方向为下)

    // 速度限幅
    vx = std::clamp(vx, -1.0f, 1.0f);
    vy = std::clamp(vy, -1.0f, 1.0f);
    vz = std::clamp(vz, -1.0f, 1.0f);

    // ★ 核心：判断XY是否已对准
    bool is_aligned = (std::abs(err_x) < tol_xy && std::abs(err_y) < tol_xy);

    const char* phase_str = was_approaching ? "逼近中→" : (is_aligned ? "已对准✓" : "对齐中");
    ROS_INFO_THROTTLE(1.0, "跟踪中 | dt=%.2fs | err_x=%+.0f err_y=%+.0f err_size=%+.0f | vx=%.2f vy=%.2f vz=%.2f | %s",
                      data_age, err_x, err_y, err_size,
                      vx, vy, vz, phase_str);

    if (!was_approaching)
    {
        // ═══ 阶段1：初始对齐 ═══
        if (!is_aligned)
        {
            vx = 0.0f;  // 没对准时暂停前进，三轴同时对齐
        }
        else
        {
            was_approaching = true;  // 首次对准 → 锁定，进入逼近阶段
        }
    }
    else
    {
        // ═══ 阶段2：逼近穿刺 ═══
        // 一旦进入就不再回退！不因微小的视角波动重新停下来对齐
        vz = 0.0f;  // 锁定Z轴高度
        // Vx保持计算值（持续向前），Vy保持计算值（持续左右跟踪）
    }

    // 4. 构建 MAVROS 速度控制指令 (BODY_NED)
    target_position.header.stamp = ros::Time::now();
    target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                mavros_msgs::PositionTarget::IGNORE_PY |
                                mavros_msgs::PositionTarget::IGNORE_PZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::IGNORE_YAW;

    target_position.position.x = 0;
    target_position.position.y = 0;
    target_position.position.z = 0;

    target_position.velocity.x = vx;    // 前后速度（对齐后启用，动态逼近）
    target_position.velocity.y = vy;    // 左右侧飞跟随（持续对齐）
    target_position.velocity.z = vz;    // 上下高度跟随（持续对齐）
    target_position.yaw_rate = 0.0f;

    // 5. 判定到达：XY已对准 且 距离足够近（box高度接近目标值）
    if (is_aligned && std::abs(err_size) < tol_size)
    {
        target_position.velocity.x = 0.0f;
        target_position.velocity.y = 0.0f;
        target_position.velocity.z = 0.0f;
        ROS_INFO("目标已到达，气球扎破!");
        was_approaching = false;
        return true;
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
// 重置指令对象接口
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

// 三通道 PWM 舵机控制接口（复刻 lib_pwm_control，支持 M5/M6/M7）
// 循环发送 + setpointPublish() 维持 OFFBOARD
bool ASNAV::pwmControl(int pwm_channel_5, int pwm_channel_6, int pwm_channel_7)
{
    // 每次调用打印一次实际下发的通道值（不用 ONCE：投放/激光分次调用参数不同，ONCE 会吞掉后续日志）
    ROS_INFO("PWM 指令发送：M5=%d, M6=%d, M7=%d", pwm_channel_5, pwm_channel_6, pwm_channel_7);

    ros::Rate rate(20); // 20Hz 发送频率，每次 0.05 秒
    for (int i = 0; i < 6; ++i)
    {
        // 1. 填充并发送舵机控制指令（复刻 lib_pwm_control：command=187, param=ch/50.0-1.0）
        mavros_msgs::CommandLong ctrl_pwm;
        ctrl_pwm.request.command = 187;   // MAV_CMD_DO_SET_ACTUATOR
        ctrl_pwm.request.param1 = (float)((double)pwm_channel_5 / 50.0 - 1.0);  // M5: 0~100 -> -1.0~+1.0
        ctrl_pwm.request.param2 = (float)((double)pwm_channel_6 / 50.0 - 1.0);  // M6
        ctrl_pwm.request.param3 = (float)((double)pwm_channel_7 / 50.0 - 1.0);  // M7
        // target_system/component 默认 0 由 mavros 自动填；param7=0 是 DO_SET_ACTUATOR 索引位，必须为 0
        if (!mavros_cmd_command_client_.call(ctrl_pwm)) {
            ROS_ERROR_ONCE("pwmControl: 调用 /mavros/cmd/command 失败（mavros 未连接飞控？）");
        }

        // 2. 维持 OFFBOARD 模式的心跳指令 (极为关键)
        setpointPublish();

        // 3. 延时，给舵机响应时间
        ros::spinOnce();
        rate.sleep();
    }
    return true; // 循环结束后再返回
}
// 投放打靶接口
bool ASNAV::putShoot(float x, float y, float z, float yaw, float tol)
{
    static int shoot_phase = 0;
    static std::string target_letter;     // 悬停时保存的字母 A 或 B
    static ros::Time arrive_time;         // 到达靶点的时间戳
    static bool letter_captured = false;  // 是否已在悬停阶段捕捉到字母

    // ====================================================
    // 阶段 0：导航到目标点 + 悬停3s，期间 YOLO 捕捉 A/B
    // ====================================================
    if (shoot_phase == 0)
    {
        bool nav_done = navigationWithPosition(x, y, z, yaw, tol, 3.0f);
        if (nav_done)
        {
            // ★ 导航完成后（已在投放点上方悬停2秒）才捕捉字母
            // 避免飞行途中经过其他字母导致误捕获
            double yolo_age = (ros::Time::now() - last_yolo_time_).toSec();
            bool yolo_fresh = (yolo_age < 1.0f);

            if (yolo_fresh && !yolo_box_info.Class.empty())
            {
                target_letter = yolo_box_info.Class;
                letter_captured = true;
                ROS_INFO("[投放打靶] 悬停完成，捕捉到字母: %s (YOLO数据%.1fs前)",
                         target_letter.c_str(), yolo_age);
            }
            else
            {
                ROS_WARN("[投放打靶] 悬停完成但未获得新鲜字母 (YOLO年龄=%.1fs, Class=%s)",
                         yolo_age,
                         yolo_box_info.Class.empty() ? "(空)" : yolo_box_info.Class.c_str());
            }

            ROS_INFO("[投放打靶] 导航悬停完成，目标字母: %s，开始投放",
                     letter_captured ? target_letter.c_str() : "未识别");
            shoot_phase = 1;
        }
        return false;
    }

    // ====================================================
    // 阶段 1：投放 pwmControl(100, 0)
    // ====================================================
    if (shoot_phase == 1)
    {
        pwmControl(100, 0);
        ROS_INFO("[投放打靶] 已投放，飞向第一个靶点 (1.2, 1.0, 1)");
        shoot_phase = 2;
        return false;
    }

    // ====================================================
    // 阶段 2：飞到第一个靶点 (0, -2, fly_height)
    // ====================================================
    if (shoot_phase == 2)
    {
        bool arrived = positionSmooth(x, -1.8f, descend_z, tol, 0.0f);
        if (arrived)
        {
            arrive_time = ros::Time::now();
            shoot_phase = 3;
            ROS_INFO("[投放打靶] 到达第一个靶点，等待 YOLO 识别...");
        }
        return false;
    }

    // ====================================================
    // 阶段 3：悬停于第一靶点，等待 D435i 识别并判断
    // ====================================================
    if (shoot_phase == 3)
    {
        position(x, -1.8f, descend_z, 0.0f, tol);

        if (last_yolo_d435i_time_ > arrive_time && !yolo_d435i_box_info_.Class.empty())
        {
            std::string detected = yolo_d435i_box_info_.Class;
            ROS_INFO("[投放打靶] 第一个靶点(D435i)识别: %s (目标: %s)",
                     detected.c_str(), target_letter.c_str());

            if (detected == target_letter)
            {
                ROS_INFO("[投放打靶] 第一个靶点匹配！下降到%.2fm打靶...", descend_z);
                shoot_phase = 51;
                return false;
            }
            else
            {
                ROS_INFO("[投放打靶] 不匹配，飞向第二个靶点 (0, -3, %.2f)", fly_height);
                shoot_phase = 4;
                return false;
            }
        }

        if ((ros::Time::now() - arrive_time).toSec() > 3.0f)
        {
            ROS_WARN("[投放打靶] 第一个靶点D435i识别超时，飞向第二个靶点");
            shoot_phase = 4;
        }
        return false;
    }

    // ====================================================
    // 阶段 4：飞到第二个靶点 (0, -3, fly_height)
    // ====================================================
    if (shoot_phase == 4)
    {
        bool arrived = positionSmooth(x, -2.7f, descend_z, tol, 0.0f);
        if (arrived)
        {
            ROS_INFO("[投放打靶] 到达第二个靶点，下降到%.2fm打靶", descend_z);
            shoot_phase = 52;
        }
        return false;
    }

    // ====================================================
    // 阶段 51：第一靶点降高 (0, -2, descend_z)
    // ====================================================
    if (shoot_phase == 51)
    {
        bool descended = flyDown(fly_height);
        if (descended)
        {
            ROS_INFO("[投放打靶] 已降至%.2fm，激光打靶", descend_z);
            shoot_phase = 6;
        }
        return false;
    }

    // ====================================================
    // 阶段 52：第二靶点降高 (0, -3, descend_z)
    // ====================================================
    if (shoot_phase == 52)
    {
        bool descended = flyDown(fly_height);
        if (descended)
        {
            ROS_INFO("[投放打靶] 已降至%.2fm，激光打靶", descend_z);
            shoot_phase = 6;
        }
        return false;
    }

    // ====================================================
    // 阶段 6：开火打靶 pwmControl(100, 100)
    // ====================================================
    if (shoot_phase == 6)
    {
        pwmControl(100, 100);
        ROS_INFO("[投放打靶] 打靶完成，关闭激光");
        shoot_phase = 7;
        return false;
    }

    // ====================================================
    // 阶段 7：关闭激光 pwmControl(100, 0)
    // ====================================================
    if (shoot_phase == 7)
    {
        pwmControl(100, 0);
        ROS_INFO("[投放打靶] 激光已关闭，任务结束 ✓");
        shoot_phase = 0;
        letter_captured = false;
        return true;
    }

    return false;
}
// 投放优化接口
// 悬停二段投放 + 激光打靶（2026-09-02 改）
//   接线：M5=1号舵机(投放第1段, 使能=100)；M6=2号舵机(投放第2段, 使能=0, 设计如此)；M7=激光笔(开=100/关=0)
bool ASNAV::putShootSimple(float x, float y, float z, float yaw, float tol)
{
    static int shoot_phase = 0;
    static std::string target_letter;     // 悬停时保存的字母 A 或 B
    static ros::Time arrive_time;         // 到达投放点/靶点的时间戳
    static bool letter_captured = false;  // 是否已在悬停阶段捕捉到字母
    static bool drop1_done = false;       // 悬停第1s：1号舵机(M5=100)是否已投放
    static bool drop2_done = false;       // 悬停第2s：2号舵机(M6=0)是否已投放

    // ====================================================
    // 阶段 0：仅导航到投放点（不悬停）；悬停+二段投放放到阶段1
    // ====================================================
    if (shoot_phase == 0)
    {
        bool nav_done = navigationSuper(x, y, z, yaw, tol);
        if (nav_done)
        {
            arrive_time = ros::Time::now();   // 悬停投放计时起点
            drop1_done = false;
            drop2_done = false;
            shoot_phase = 1;
            ROS_INFO("[单靶打靶] 已到达投放点 (%.2f, %.2f, %.2f)，开始悬停二段投放(共2.5s)", x, y, z);
        }
        return false;
    }

    // ====================================================
    // 阶段 1：投放点悬停 2.5s，期间两舵机分时投放
    //   悬停第1s：M5=100 → 1号舵机投放第1段载荷
    //   悬停第2s：M6=0   → 2号舵机投放第2段载荷（设计如此：M6 传 0 即使能）
    //   悬停末段捕捉 A/B（沿用旧逻辑：到位悬停后才读 YOLO，避免中途误捕获）
    // ====================================================
    if (shoot_phase == 1)
    {
        // 维持投放点位置悬停（沿用 phase3 的保持写法）
        position(x, y, z, yaw, 0.1f);

        double hover_t = (ros::Time::now() - arrive_time).toSec();

        // 悬停第1s：1号舵机投放。M6 保持 100（2号未投仍夹住），M7=0（激光关）
        if (!drop1_done && hover_t >= 1.5)
        {
            pwmControl(100, 100, 0);
            drop1_done = true;
            ROS_INFO("[单靶打靶] 悬停第1s：1号舵机(M5=100)投放第1段");
        }
        // 悬停第2s：2号舵机投放（0 即使能）。M5 保持 100（1号已投），M7=0（激光关）
        if (!drop2_done && hover_t >= 2.7)
        {
            pwmControl(100, 0, 0);
            drop2_done = true;
            ROS_INFO("[单靶打靶] 悬停第2s：2号舵机(M6=0)投放第2段");
        }

        // 悬停收尾：第2s投放动作约0.3s，留到 2.5s 再走，保证动作完成
        if (hover_t >= 3.0)
        {
            double yolo_age = (ros::Time::now() - last_yolo_time_).toSec();
            bool yolo_fresh = (yolo_age < 1.0f);

            if (yolo_fresh && !yolo_box_info.Class.empty())
            {
                target_letter = yolo_box_info.Class;
                letter_captured = true;
                ROS_INFO("[单靶打靶] 悬停投放完成，捕捉到字母: %s (YOLO数据%.1fs前)",
                         target_letter.c_str(), yolo_age);
            }
            else
            {
                ROS_WARN("[单靶打靶] 悬停投放完成但未获得新鲜字母 (YOLO年龄=%.1fs, Class=%s)",
                         yolo_age,
                         yolo_box_info.Class.empty() ? "(空)" : yolo_box_info.Class.c_str());
            }

            ROS_INFO("[单靶打靶] 二段投放完成，目标字母: %s，飞向靶点 (%.2f, %.2f, %.2f)",
                     letter_captured ? target_letter.c_str() : "未识别", x, y, descend_z);
            shoot_phase = 2;
        }
        return false;
    }

    // ====================================================
    // 阶段 2：飞到唯一靶点
    // ====================================================
    if (shoot_phase == 2)
    {
        bool arrived = position(-0.4, -1.7, descend_z, 0.0f, tol);
        if (arrived)
        {
            arrive_time = ros::Time::now();
            shoot_phase = 3;
            ROS_INFO("[单靶打靶] 到达靶点，等待 D435i 识别...");
        }
        return false;
    }

    // ====================================================
    // 阶段 3：悬停于靶点，等待 D435i 识别到即下降
    // ====================================================
    if (shoot_phase == 3)
    {
        position(-0.4, -1.7, descend_z, 0.0f, tol);

        if (last_yolo_d435i_time_ > arrive_time && !yolo_d435i_box_info_.Class.empty())
        {
            std::string detected = yolo_d435i_box_info_.Class;
            ROS_INFO("[单靶打靶] 靶点(D435i)识别: %s (目标: %s)",
                     detected.c_str(), target_letter.c_str());

            if (detected == target_letter)
            {
                ROS_INFO("[单靶打靶] 靶点匹配！下降到%.2fm打靶...", descend_z);
            }
            else
            {
                ROS_WARN("[单靶打靶] 字母不匹配(%s vs %s)，仍下降打靶",
                         detected.c_str(), target_letter.c_str());
            }
            shoot_phase = 4;
            return false;
        }

        if ((ros::Time::now() - arrive_time).toSec() > 0.5f)
        {
            ROS_WARN("[单靶打靶] D435i识别超时，直接下降打靶");
            shoot_phase = 4;
        }
        return false;
    }

    // ====================================================
    // 阶段 4：下降至打靶高度
    // ====================================================
    if (shoot_phase == 4)
    {
        bool descended = flyDown(fly_height);
        if (descended)
        {
            ROS_INFO("[单靶打靶] 已降至%.2fm，激光打靶", descend_z);
            shoot_phase = 5;
        }
        return false;
    }

    // ====================================================
    // 阶段 5：激光开火打靶（激光笔=M7，开=100）
    // ====================================================
    if (shoot_phase == 5)
    {
        // 激光开：M7=100；M5/M6 保持投放后状态(100/0)，不再动舵机
        pwmControl(100, 0, 100);
        ROS_INFO("[单靶打靶] 激光开火完成，关闭激光");
        shoot_phase = 6;
        return false;
    }

    // ====================================================
    // 阶段 6：关闭激光（M7=0）
    // ====================================================
    if (shoot_phase == 6)
    {
        pwmControl(100, 0, 0);
        ROS_INFO("[单靶打靶] 激光已关闭，任务结束 ✓");
        shoot_phase = 0;
        letter_captured = false;
        drop1_done = false;
        drop2_done = false;
        return true;
    }

    return false;
}
// 投放优化接口 + A/B 动态靶点（2026-09-15 改）
// 流程与接口调用方式完全对齐 putShootSimple（navigationSuper 导航 + 悬停二段投放 + M7 激光打靶）；
// 唯一区别：打靶点位按阶段1悬停时识别的字母动态选择 —— A -> y=-1.7, B -> y=-2.7
bool ASNAV::putShootPlus(float x, float y, float z, float yaw, float tol)
{
    static int shoot_phase = 0;
    static std::string target_letter;     // 悬停时保存的字母 A 或 B
    static ros::Time arrive_time;         // 到达投放点/靶点的时间戳
    static bool letter_captured = false;  // 是否已在悬停阶段捕捉到字母
    static bool drop1_done = false;       // 悬停第1.5s：1号舵机(M5=100)是否已投放
    static bool drop2_done = false;       // 悬停第2.7s：2号舵机(M6=0)是否已投放

    // ====================================================
    // 阶段 0：仅导航到投放点（不悬停）；悬停+二段投放放到阶段1
    // ====================================================
    if (shoot_phase == 0)
    {
        bool nav_done = navigationSuper(x, y, z, yaw, tol);
        if (nav_done)
        {
            arrive_time = ros::Time::now();   // 悬停投放计时起点
            drop1_done = false;
            drop2_done = false;
            shoot_phase = 1;
            ROS_INFO("[单靶打靶] 已到达投放点 (%.2f, %.2f, %.2f)，开始悬停二段投放(共2.5s)", x, y, z);
        }
        return false;
    }

    // ====================================================
    // 阶段 1：投放点悬停 2.5s，期间两舵机分时投放
    //   悬停第1s：M5=100 → 1号舵机投放第1段载荷
    //   悬停第2s：M6=0   → 2号舵机投放第2段载荷（设计如此：M6 传 0 即使能）
    //   悬停末段捕捉 A/B（到位悬停后才读 YOLO，避免中途误捕获）→ 决定打靶点位
    // ====================================================
    if (shoot_phase == 1)
    {
        // 维持投放点位置悬停（沿用 phase3 的保持写法）
        position(x, y, z, yaw, 0.1f);

        double hover_t = (ros::Time::now() - arrive_time).toSec();

        // 悬停第1s：1号舵机投放。M6 保持 100（2号未投仍夹住），M7=0（激光关）
        if (!drop1_done && hover_t >= 1.5)
        {
            pwmControl(100, 100, 0);
            drop1_done = true;
            ROS_INFO("[单靶打靶] 悬停第1s：1号舵机(M5=100)投放第1段");
        }
        // 悬停第2s：2号舵机投放（0 即使能）。M5 保持 100（1号已投），M7=0（激光关）
        if (!drop2_done && hover_t >= 2.7)
        {
            pwmControl(100, 0, 0);
            drop2_done = true;
            ROS_INFO("[单靶打靶] 悬停第2s：2号舵机(M6=0)投放第2段");
        }

        // 悬停收尾：第2s投放动作约0.3s，留到 2.5s 再走，保证动作完成
        if (hover_t >= 3.0)
        {
            double yolo_age = (ros::Time::now() - last_yolo_time_).toSec();
            bool yolo_fresh = (yolo_age < 1.0f);

            if (yolo_fresh && !yolo_box_info.Class.empty())
            {
                target_letter = yolo_box_info.Class;
                letter_captured = true;
                ROS_INFO("[单靶打靶] 悬停投放完成，捕捉到字母: %s (YOLO数据%.1fs前)",
                         target_letter.c_str(), yolo_age);
            }
            else
            {
                ROS_WARN("[单靶打靶] 悬停投放完成但未获得新鲜字母 (YOLO年龄=%.1fs, Class=%s)",
                         yolo_age,
                         yolo_box_info.Class.empty() ? "(空)" : yolo_box_info.Class.c_str());
                target_letter.clear();   // 未识别到则清空, 防止上次 A/B 残留导致打错靶
                letter_captured = false;
            }

            // 打靶点位由识别结果决定：A -> y=-1.7, B -> y=-2.7
            float target_y = (target_letter == "B") ? -2.7f : -1.7f;
            ROS_INFO("[单靶打靶] 二段投放完成，目标字母: %s，飞向靶点 (%.2f, %.2f, %.2f)",
                     letter_captured ? target_letter.c_str() : "未识别", x, target_y, descend_z);
            shoot_phase = 2;
        }
        return false;
    }

    // ====================================================
    // 阶段 2：飞到唯一靶点
    // ====================================================
    if (shoot_phase == 2)
    {
        // A/B 靶点动态选择（阶段1识别的字母）: A->y=-1.7, B->y=-2.7
        float target_y = (target_letter == "B") ? -2.7f : -1.7f;
        bool arrived = position(x, target_y, descend_z, 0.0f, tol);
        if (arrived)
        {
            arrive_time = ros::Time::now();
            shoot_phase = 3;
            ROS_INFO("[单靶打靶] 到达靶点 (%.2f, %.2f)，等待 D435i 识别...", x, target_y);
        }
        return false;
    }

    // ====================================================
    // 阶段 3：悬停于靶点，等待 D435i 识别到即下降
    // ====================================================
    if (shoot_phase == 3)
    {
        float target_y = (target_letter == "B") ? -2.7f : -1.7f;
        position(x, target_y, descend_z, 0.0f, tol);

        if (last_yolo_d435i_time_ > arrive_time && !yolo_d435i_box_info_.Class.empty())
        {
            std::string detected = yolo_d435i_box_info_.Class;
            ROS_INFO("[单靶打靶] 靶点(D435i)识别: %s (目标: %s)",
                     detected.c_str(), target_letter.c_str());

            if (detected == target_letter)
            {
                ROS_INFO("[单靶打靶] 靶点匹配！下降到%.2fm打靶...", descend_z);
            }
            else
            {
                ROS_WARN("[单靶打靶] 字母不匹配(%s vs %s)，仍下降打靶",
                         detected.c_str(), target_letter.c_str());
            }
            shoot_phase = 4;
            return false;
        }

        if ((ros::Time::now() - arrive_time).toSec() > 2.0f)
        {
            ROS_WARN("[单靶打靶] D435i识别超时，直接下降打靶");
            shoot_phase = 4;
        }
        return false;
    }

    // ====================================================
    // 阶段 4：下降至打靶高度
    // ====================================================
    if (shoot_phase == 4)
    {
        bool descended = flyDown(fly_height);
        if (descended)
        {
            ROS_INFO("[单靶打靶] 已降至%.2fm，激光打靶", descend_z);
            shoot_phase = 5;
        }
        return false;
    }

    // ====================================================
    // 阶段 5：激光开火打靶（2026-09-02 改：激光笔=M7，开=100，用法同 putShootSimple）
    // ====================================================
    if (shoot_phase == 5)
    {
        // 激光开：M7=100；M5/M6 保持投放后状态(100/0)，不再动舵机
        pwmControl(100, 0, 100);
        ROS_INFO("[单靶打靶] 激光开火完成，关闭激光");
        shoot_phase = 6;
        return false;
    }

    // ====================================================
    // 阶段 6：关闭激光（M7=0）
    // ====================================================
    if (shoot_phase == 6)
    {
        pwmControl(100, 0, 0);
        ROS_INFO("[单靶打靶] 激光已关闭，任务结束 ✓");
        shoot_phase = 0;
        letter_captured = false;
        target_letter.clear();
        drop1_done = false;
        drop2_done = false;
        return true;
    }

    return false;
}

// AR标签跟踪降落接口
bool ASNAV::arTrackLanding(float ground_z, float altitude, float max_error, float vel_set, float camera_offset_x, float camera_offset_y)
{
    static bool flag_init_position = false;
    static float init_position_z_take_off = 0;
    static float last_position_x = 0;
    static float last_position_y = 0;
    static bool flag_arrive = false;
    static float current_altitude = 1.0;

    // 初始化：使用用户传入的地面Z坐标，而非当前高度（避免飞到ar码上空后记录错误基准）
    if (!flag_init_position && current_position.z != 0)
    {
        init_position_z_take_off = ground_z;
        last_position_x = current_position.x;
        last_position_y = current_position.y;
        current_altitude = altitude;
        flag_init_position = true;
        flag_arrive = false;
        ROS_INFO("[AR降落] 初始化完成，地面Z: %.2f, 跟踪高度: %.2f(相对地面)，检测到任意AR码即跟踪",
                 init_position_z_take_off, altitude);
    }

    if (!flag_init_position) return false;

    // 设置坐标系与航向
    target_position.header.stamp = ros::Time::now();
    target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target_position.yaw = current_yaw;

    if (ar_marker_found_ && !flag_arrive)
    {
        // 相机安装位置偏移修正：摄像头在机头前方0.12m，朝下安装
        // 相机坐标系：相机X→无人机Y(左右)，相机Y→无人机X(前后)
        // offset补偿后，使无人机重心（而非摄像头）对准AR码
        float err_x = ar_position_detec_x_ - camera_offset_x;
        float err_y = ar_position_detec_y_ - camera_offset_y;

        // 识别到ar码且对准（误差在阈值内）
        if (fabs(err_x) < max_error && fabs(err_y) < max_error)
        {
            // 记录当前水平位置，防止风或GPS误差导致飘移
            last_position_x = current_position.x;
            last_position_y = current_position.y;

            // 高度判断：小于0.6m则触发降落
            if (current_altitude < 0.6)
            {
                flag_arrive = true;
                ROS_INFO("[AR降落] 已对准AR码，高度<0.6m，触发最终降落");
            }
            else
            {
                // 递减降低高度
                current_altitude -= 0.01;
                ROS_INFO_THROTTLE(1.0, "[AR降落] 已对准AR码(含相机偏移修正)，当前高度: %.2f m，持续下降中...", current_altitude);
            }

            // 位置控制：保持水平位置，调整高度
            target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                                        mavros_msgs::PositionTarget::IGNORE_VY |
                                        mavros_msgs::PositionTarget::IGNORE_VZ |
                                        mavros_msgs::PositionTarget::IGNORE_AFX |
                                        mavros_msgs::PositionTarget::IGNORE_AFY |
                                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                                        mavros_msgs::PositionTarget::FORCE |
                                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            target_position.position.x = last_position_x;
            target_position.position.y = last_position_y;
            target_position.position.z = init_position_z_take_off + current_altitude;
        }
        else
        {
            // 未对准，速度控制移动到ar码上方
            // 摄像头朝下安装：摄像头X对应无人机Y(左右)，摄像头Y对应无人机X(前后)
            if (err_x >= max_error)
                target_position.velocity.y = -vel_set;
            else if (err_x <= -max_error)
                target_position.velocity.y = vel_set;
            else
                target_position.velocity.y = 0;

            if (err_y >= max_error)
                target_position.velocity.x = -vel_set;
            else if (err_y <= -max_error)
                target_position.velocity.x = vel_set;
            else
                target_position.velocity.x = 0;

            ROS_INFO_THROTTLE(1.0, "[AR降落] 检测到AR码，未对准 | err_x: %.3f err_y: %.3f (已含offset修正) | 速度 vx: %.2f vy: %.2f",
                              err_x, err_y,
                              target_position.velocity.x, target_position.velocity.y);

            target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                        mavros_msgs::PositionTarget::IGNORE_PY |
                                        mavros_msgs::PositionTarget::IGNORE_AFX |
                                        mavros_msgs::PositionTarget::IGNORE_AFY |
                                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                                        mavros_msgs::PositionTarget::FORCE |
                                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            target_position.position.z = init_position_z_take_off + current_altitude;

            // 更新水平位置，确保丢失目标时能保持位置
            last_position_x = current_position.x;
            last_position_y = current_position.y;
        }
    }
    else if (!flag_arrive)
    {
        // 未识别到ar码，保持当前位置悬停
        ROS_INFO_THROTTLE(2.0, "[AR降落] 未检测到AR码，保持悬停 | 当前Z: %.2f 目标Z: %.2f",
                          current_position.z,
                          init_position_z_take_off + current_altitude);

        target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                                    mavros_msgs::PositionTarget::IGNORE_VY |
                                    mavros_msgs::PositionTarget::IGNORE_VZ |
                                    mavros_msgs::PositionTarget::IGNORE_AFX |
                                    mavros_msgs::PositionTarget::IGNORE_AFY |
                                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                                    mavros_msgs::PositionTarget::FORCE |
                                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        target_position.velocity.x = 0;
        target_position.velocity.y = 0;
        target_position.position.x = last_position_x;
        target_position.position.y = last_position_y;
        target_position.position.z = init_position_z_take_off + current_altitude;
    }

    // 最终降落：已对准AR码且高度足够低，直接调用autoLand切换降落模式
    if (flag_arrive)
    {
        ROS_INFO("[AR降落] 已对准，调用autoLand切换AUTO.LAND模式降落");
        autoLand();
        flag_init_position = false;
        flag_arrive = false;
        return true;
    }

    return false;
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
// MAVROS速度回调 (用于 Ego 的 Kv 速度误差阻尼项)
void ASNAV::mavros_local_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_velocity.x = msg->twist.linear.x;
    current_velocity.y = msg->twist.linear.y;
    current_velocity.z = msg->twist.linear.z;
}
// ego速度回调
void ASNAV::ego_planner_pos_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    ego_cmd_ = *msg;
    ego_cmd_received_ = true;
    last_ego_msg_time_ = ros::Time::now();
}
// SUPER 规划器位置指令回调
void ASNAV::super_planner_pos_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    super_cmd_ = *msg;
    super_cmd_received_ = true;
    last_super_msg_time_ = ros::Time::now();
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
    // ROS_WARN_THROTTLE(1, "YOLO识别结果为空");
    return;
    }
    // 遍历识别框，只取字母 A/B，忽略干扰物
    bool found_target = false;
    for (const auto& box : msg->bounding_boxes)
    {
        if (box.Class == "A" || box.Class == "B")
        {
            yolo_box_info.Class = box.Class;
            yolo_box_info.cameraXCenter = (box.xmax + box.xmin) * 0.5f;
            yolo_box_info.cameraYCenter = (box.ymax + box.ymin) * 0.5f;
            yolo_box_info.boxHeight = box.ymax - box.ymin;

            last_yolo_time_ = ros::Time::now();

            // ROS_INFO_THROTTLE(1, "成功锁定目标 : %s (%.2f, %.2f)",
            //                   yolo_box_info.Class.c_str(),
            //                   yolo_box_info.cameraXCenter,
            //                   yolo_box_info.cameraYCenter);

            found_target = true;
            break;
        }
    }

    if (!found_target)
    {
        ROS_WARN_THROTTLE(1, "未在当前帧中找到目标A/B");
    }
}
// D435i前视YOLO回调（与单目回调逻辑一致，写入独立的d435i存储区）
void ASNAV::yolo_d435i_info_cb(const yolov8_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
    if (msg->bounding_boxes.empty())
    {
        // ROS_WARN_THROTTLE(1, "D435i YOLO识别结果为空");
        return;
    }
    bool found_target = false;
    for (const auto& box : msg->bounding_boxes)
    {
        //睿抗
        if (box.Class == "A" || box.Class == "B")
        {
            yolo_d435i_box_info_.Class = box.Class;
            yolo_d435i_box_info_.cameraXCenter = (box.xmax + box.xmin) * 0.5f;
            yolo_d435i_box_info_.cameraYCenter = (box.ymax + box.ymin) * 0.5f;
            yolo_d435i_box_info_.boxHeight = box.ymax - box.ymin;

            last_yolo_d435i_time_ = ros::Time::now();

            // ROS_INFO_THROTTLE(1, "D435i 锁定目标: %s (%.2f, %.2f)",
            //                   yolo_d435i_box_info_.Class.c_str(),
            //                   yolo_d435i_box_info_.cameraXCenter,
            //                   yolo_d435i_box_info_.cameraYCenter);

            found_target = true;
            break;
        }

        // if (box.Class == target_class_name)
        // {
        //     yolo_d435i_box_info_.Class = box.Class;
        //     yolo_d435i_box_info_.cameraXCenter = (box.xmax + box.xmin) * 0.5f;
        //     yolo_d435i_box_info_.cameraYCenter = (box.ymax + box.ymin) * 0.5f;
        //     yolo_d435i_box_info_.boxHeight = box.ymax - box.ymin;

        //     last_yolo_d435i_time_ = ros::Time::now();

        //     // ROS_INFO_THROTTLE(1, "D435i 锁定目标: %s (%.2f, %.2f)",
        //     //                   yolo_d435i_box_info_.Class.c_str(),
        //     //                   yolo_d435i_box_info_.cameraXCenter,
        //     //                   yolo_d435i_box_info_.cameraYCenter);

        //     found_target = true;
        //     break;
        // }
    }
    if (!found_target)
    {
        ROS_WARN_THROTTLE(1, "D435i 未在当前帧中找到目标%s", target_class_name.c_str());
    }
}
// ar标签回调
void ASNAV::ar_pose_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    int count = msg->markers.size();
    if (count != 0)
    {
        // 不匹配特定ID，取第一个检测到的AR码直接使用
        ar_track_alvar_msgs::AlvarMarker marker = msg->markers[0];
        ar_marker_found_ = true;
        // 根据摄像头安装方向进行静态坐标转换
        // 摄像头朝下安装：摄像头X对应无人机Y(左右)，摄像头Y对应无人机X(前后)，摄像头Z对应上下
        ar_position_detec_x_ = marker.pose.pose.position.x;
        ar_position_detec_y_ = marker.pose.pose.position.y;
        ar_position_detec_z_ = marker.pose.pose.position.z;
        ROS_INFO_THROTTLE(1.0, "[AR回调] 检测到AR码 id=%d(忽略ID匹配) | pos=(%.3f, %.3f, %.3f)",
                          marker.id, ar_position_detec_x_, ar_position_detec_y_, ar_position_detec_z_);

        // // ---- 以下为按ID匹配的原始逻辑，如需恢复取消注释 ----
        // bool found = false;
        // for (int i = 0; i < count; i++)
        // {
        //     ar_track_alvar_msgs::AlvarMarker marker = msg->markers[i];
        //     if (marker.id == ar_target_id_)
        //     {
        //         ar_marker_found_ = true;
        //         found = true;
        //         ar_position_detec_x_ = marker.pose.pose.position.x;
        //         ar_position_detec_y_ = marker.pose.pose.position.y;
        //         ar_position_detec_z_ = marker.pose.pose.position.z;
        //         ROS_INFO_THROTTLE(1.0, "[AR回调] 锁定目标AR码 id=%d | pos=(%.3f, %.3f, %.3f)",
        //                           marker.id, ar_position_detec_x_, ar_position_detec_y_, ar_position_detec_z_);
        //         return;
        //     }
        // }
        // ar_marker_found_ = false;
        // if (!found)
        // {
        //     ROS_WARN_THROTTLE(2.0, "[AR回调] 收到%d个AR码，但未找到目标id=%d", count, ar_target_id_);
        // }
        // // ---- 原始逻辑结束 ----
    }
    else
    {
        ar_marker_found_ = false;
        ROS_WARN_THROTTLE(3.0, "[AR回调] /ar_pose_marker 话题为空，未检测到任何AR码");
    }
}

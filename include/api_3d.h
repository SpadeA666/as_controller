#ifndef ASNAV_API_H
#define ASNAV_API_H

#include "mavros_msgs/CommandBool.h"
#include <mavros_msgs/CommandLong.h>
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "nav_msgs/Odometry.h"
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <yolov8_ros_msgs/BoundingBoxes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <string>

#include <actionlib/client/simple_action_client.h> 


class ASNAV
{
    public:
    explicit ASNAV(ros::NodeHandle& nh_);
    ~ASNAV();
    bool takeoff(float height);
    bool position(float x, float y, float z, float yaw, float tol = 0.2f);
    bool positionSmooth(float target_x, float target_y, float target_z, float tol, float hover_sec = 0.0f);
    bool navigation(float x, float y, float z, float yaw, float tol = 0.2f, float hover_sec = 1.0f);
    bool navigationWithPosition(float x, float y, float z, float yaw, float tol, float hover_sec);
    bool navigationEgo(float x, float y, float z, float yaw, float tol = 0.2f);
    // SUPER 规划器接口（对齐 ruikang 控制器 2026-08-25；navigationSuperRviz 内部复用）
    // stop_at_goal=true: 到达后位置锁存收敛（= ruikang HOVER_HIGH），v<0.3 才返回 true（末点用）
    bool navigationSuper(float x, float y, float z, float yaw, float tol = 0.2f, bool stop_at_goal = false);
    // rviz 打点测试接口（移植自 sim_work followEgo）：内部阻塞循环，只认 rviz 点触发的 EGO 规划器轨迹，永不返回 true
    bool navigationEgoRviz();
    // 只接收 rviz 打点的 SUPER 跟踪接口（移植自 sim_work navigationSuperRvizPID）：复用 navigationSuper 控制律，不传坐标、不自己发 goal，永不返回 true
    bool navigationSuperRviz();
    bool controlYaw(float x, float y, float z, float target_yaw, float wait_sec);
    bool flyDown(float descend_z);
    bool flyUp(float height);
    void setpointPublish();
    void set_mode(std::string mode);
    bool autoLand();
    bool trackYoloDown(float max_distance = 0.35f, int tol = 30);
    bool trackYoloForward(float Kp_x, float Kp_y, float Kp_z, float target_box_height, int tol_xy, int tol_size);
    bool trackYoloing(float Kp_x, float Kp_y, float Kp_z, float target_box_height, int tol_xy, int tol_size);
    void reset_target();
    // 三通道 PWM 舵机控制接口（复刻 lib_pwm_control，M5/M6/M7，参数 0~100 占空比）
    // 循环发送 + setpointPublish() 维持 OFFBOARD
    // pwm_channel_7 默认 50（中位）：只传两个参数时 M7 输出中位，行为与原 lib_pwm_control 一致
    bool pwmControl(int pwm_channel_5, int pwm_channel_6, int pwm_channel_7 = 50);
    bool putShoot(float x, float y, float z, float yaw, float tol);
    bool putShootSimple(float x, float y, float z, float yaw, float tol);
    // 2026-09-15: 流程与接口调用方式完全同 putShootSimple, 仅打靶点位按阶段1识别的字母动态选择: A->y=-1.7, B->y=-2.7
    bool putShootPlus(float x, float y, float z, float yaw, float tol);
    bool arTrackLanding(float ground_z = 0.0f, float altitude = 1.0f, float max_error = 0.20f, float vel_set = 0.15f, float camera_offset_x = 0.0f, float camera_offset_y = 0.0f);

    struct yoloBox
    {
        std::string Class;
        float cameraXCenter, cameraYCenter;
        float boxHeight;
    };

    float fly_height;            
    float descend_z;
    
    private:
    //容差函数
    float tolerance(float x, float y, float z) const;
    //mavros回调
    void mavros_state_cb(const mavros_msgs::State::ConstPtr& msg);
    void mavros_local_position_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void mavros_local_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    //ego回调
    void ego_planner_pos_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
    //yolo回调
    void yolo_info_cb(const yolov8_ros_msgs::BoundingBoxes::ConstPtr& msg);
    void yolo_d435i_info_cb(const yolov8_ros_msgs::BoundingBoxes::ConstPtr& msg);  // D435i前视YOLO回调
    //ar标签回调
    void ar_pose_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
    ros::NodeHandle nh_;
    //mavros相关组件
    ros::Subscriber mavros_state_sub_, mavros_local_position_pose_sub_, mavros_local_velocity_sub_;
    ros::Publisher mavros_setpoint_raw_local_pub_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient mavros_cmd_command_client_;
    //ego相关组件
    ros::Subscriber ego_planner_pos_cmd_sub_;  
    ros::Publisher goal_pub_;
    //yolo相关组件
    ros::Subscriber yolo_info_sub_;
    ros::Subscriber yolo_d435i_info_sub_;  // D435i前视相机YOLO订阅
    //ar标签相关组件
    ros::Subscriber ar_pose_sub_;
    //mavros变量
    bool is_offboard, is_auto_land, is_yaw_finished = false;
    geometry_msgs::Point current_position;
    geometry_msgs::Vector3 current_velocity;
    mavros_msgs::PositionTarget target_position;
    mavros_msgs::State current_state;
    float current_yaw;
    ros::Time yaw_finish_time;

    // mavros_msgs::CommandLong lib_ctrl_pwm;

    // EGO 规划器指令缓存
    quadrotor_msgs::PositionCommand ego_cmd_;
    bool ego_cmd_received_ = false;
    ros::Time ego_stale_time_;   // ego指令最后活跃时间（用于检测ego停摆）
    bool goal_sent_;
    uint32_t zplus_last_traj_id_; // ego trajectory_id 跟踪（新轨迹检测→清积分，navigationEgoRviz 使用）
    bool ego_pos_hold_ = false;   // EGO 到点位置保持（navigationEgoRviz 使用）
    int ego_pos_hold_frames_ = 0; // 到点判定防抖帧计数（navigationEgoRviz 使用）

    // SUPER 规划器指令缓存（navigationSuper / navigationSuperRviz 使用）
    quadrotor_msgs::PositionCommand super_cmd_;
    bool super_cmd_received_ = false;
    ros::Time last_super_msg_time_;
    bool super_goal_sent_;
    bool super_rviz_mode_;        // navigationSuper 的 rviz 测试模式：不发 goal、不判到达
    ros::Publisher super_goal_pub_;
    ros::Subscriber super_planner_pos_cmd_sub_;
    void super_planner_pos_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg);

    // navigationSuper 参数（从launch加载）
    float super_kp_outer_;       // Kp: 位置误差 → 速度修正
    float super_kv_outer_;       // Kv: 速度误差阻尼 (v_ref→v_actual)
    float super_ki_outer_;       // Ki: 位置误差积分
    float super_max_vel_;        // 速度指令限幅 (m/s) XY
    float super_max_vel_z_;      // Z 速度指令限幅 (m/s)
    float super_max_integral_;   // 积分抗饱和上限
    float super_traj_timeout_;   // 轨迹超时时间 (s)
    bool super_rotate_180_;      // 坐标系是否需要旋转180°(LIO→PX4)
    float super_max_accel_;      // 速度指令帧间加速度限幅 (m/s^2)，0=关闭
    float super_acc_ff_;          // 加速度前馈前瞻时间 (s), v_cmd += a_ref*super_acc_ff_, 0=关闭
    // zfix-accff 2026-08-23: 消除高速转弯跟随滞后(外甩 0.2m)导致 backup 提前截断避障线的问题

    // navigationSuper 运行时状态
    double integral_spx_;         // X 位置误差积分
    double integral_spy_;         // Y 位置误差积分
    double integral_spz_;         // Z 位置误差积分
    ros::Time last_super_call_time_; // 上一帧调用时间 (dt计算)
    float super_traj_elapsed_;    // 新轨迹软启动计时器
    ros::Time super_goal_time_;       // 最近一次发布 SUPER 目标的时刻（刷新超时计时 + 新鲜轨迹判定）
    double last_super_vx_;            // 上一帧速度指令（斜率限制用）
    double last_super_vy_;
    double last_super_vz_;
    float super_slew_timer_ = 0.0f;   // >0 时启用 slewLimit（新目标/换点瞬间，正常跟踪旁路）
    bool super_pos_hold_ = false;     // SUPER 到点位置保持（仿 ruikang HOVER：收敛交给 PX4）
    double super_hold_px_ = 0.0;      // 位置保持锁存点(参考跳动时锁定, 不跟跳动参考)
    double super_hold_py_ = 0.0;
    double super_hold_pz_ = 0.0;
    int super_pos_hold_frames_ = 0;   // 到点判定防抖帧计数
    bool super_hold_active_ = false;  // holdfix2: HOLD 位置锁存已初始化(进入HOLD时锁存一次)
    ros::Time super_tol_entry_time_;  // 进入容差时刻
    bool super_tol_timing_;           // 是否正在 debounce 计时
    bool super_yaw_finishing_ = false; // 位置到位后 yaw 收尾等待中（yawfix）
    bool super_yaw_timing_ = false;    // yaw 到位防抖计时中（yawfix）
    ros::Time super_yaw_entry_time_;   // yaw 到位防抖计时起点（yawfix）

    // 速度指令帧间斜率限制（消除换航点/新轨迹切入时的速度阶跃）
    void slewLimitVel(double& vx, double& vy, double& vz,
                      double& lx, double& ly, double& lz, double dt, float max_acc);

    //其他变量
    double start_planning_time, finish_planning_time;

    yoloBox yolo_box_info;            // 单目(下视)YOLO结果
    yoloBox yolo_d435i_box_info_;     // D435i(前视)YOLO结果
    std::string target_class_name;
    float integral_error_x, integral_error_y, last_err_x, last_err_y;
    ros::Time last_yolo_time_;
    ros::Time last_yolo_d435i_time_;  // D435i最后识别时间
    //ar标签跟踪变量
    bool ar_marker_found_;
    int ar_target_id_;
    float ar_position_detec_x_, ar_position_detec_y_, ar_position_detec_z_;

    // navigationEgo 参数（从launch加载）
    float zplus_kp_outer_;       // Kp: 位置误差 → 速度修正
    float zplus_kv_outer_;       // Kv: 速度误差阻尼 (v_ref→v_actual)
    float zplus_ki_outer_;       // Ki: 位置误差积分
    float zplus_max_vel_;        // 速度指令限幅 (m/s)
    float zplus_max_integral_;   // 积分抗饱和上限
    float zplus_traj_timeout_;   // 轨迹超时时间 (s)
    float zplus_max_accel_;      // 速度指令帧间加速度限幅 (m/s^2)，0=关闭（navigationEgo / navigationEgoRviz 共用）

    // positionSmooth 步长控制参数（从launch加载）
    double smooth_step_xy_;        // 正常步长 (m/cycle)，默认 0.18
    double smooth_slow_step_xy_;   // 接近目标时的减速步长 (m/cycle)，默认 0.01
    double smooth_slow_dist_;      // 触发减速的距离阈值 (m)，默认 0.5

    // navigationEgo 运行时状态
    double integral_zpx_;         // X 位置误差积分
    double integral_zpy_;         // Y 位置误差积分
    ros::Time last_zplus_call_time_; // 上一帧调用时间 (dt计算)
    float zplus_traj_elapsed_;    // 新轨迹软启动计时器
    ros::Time last_ego_msg_time_; // 最近一次收到ego消息的时间戳
    double last_zplus_vx_;            // 上一帧速度指令（斜率限制用，navigationEgo）
    double last_zplus_vy_;
    float zplus_slew_timer_ = 0.0f;   // >0 时启用 slewLimit（新目标/换点瞬间，正常跟踪旁路）
    ros::Time last_ego_rviz_call_time_; // navigationEgoRviz 帧间调用时间 (dt计算)
    double last_ego_vx_;              // 上一帧速度指令（斜率限制用，navigationEgoRviz）
    double last_ego_vy_;
    float ego_slew_timer_ = 0.0f;     // >0 时启用 slewLimit（新轨迹切入瞬间，正常跟踪旁路）

    // navigationEgo Debounce 防穿透 + 位置保持
    ros::Time zplus_tol_entry_time_;  // 进入容差时刻
    bool zplus_tol_timing_;           // 是否正在 debounce 计时
    bool zplus_holding_;              // debounce 完成后是否在位置保持阶段
    ros::Time zplus_hold_start_time_; // 位置保持开始时刻
    float zplus_hold_x_ = 0.0f;
    float zplus_hold_y_ = 0.0f;
};
#endif